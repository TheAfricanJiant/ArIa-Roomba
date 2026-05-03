"""
ARIA — Phase 3 Entry Point
EKF Localization + Occupancy Grid

Run with:
    python aria_main.py                    # simulator mode (no hardware)
    python aria_main.py --port /dev/ttyUSB0  # real hardware (Phase 2+)

The loop runs at BRIDGE_UPLINK_HZ (20 Hz default).
Every iteration:
  1. Get sensor packet from Bridge (or sim)
  2. Compute encoder deltas since last packet
  3. EKF predict from encoders
  4. EKF correct from IMU gyro
  5. Check for wall snap (side ultrasonics)
  6. Update occupancy grid
  7. Print terminal map every 0.5 s
  8. Save grid to disk every 60 s
"""

import sys
import os
import time
import argparse
import signal
import logging

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s [%(levelname)s] %(message)s')
log = logging.getLogger('ARIA')

from aria import ARIALocalization, OccupancyGrid, BridgeStub
from aria.config import (
    BRIDGE_UPLINK_HZ, GRID_VISUALIZE_HZ,
    US_WALL_SNAP_CM, CELL_SIZE_CM,
)

SAVE_PATH = os.path.expanduser('~/.aria/aria_grid.npy')
SAVE_INTERVAL_S = 60.0
VIZ_INTERVAL_S  = 1.0 / GRID_VISUALIZE_HZ


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description='ARIA Phase 3 — EKF + Occupancy Grid')
    p.add_argument('--port',    default=None,
                   help='Serial port for real hardware (e.g. /dev/ttyUSB0). '
                        'If not given, runs in simulator mode.')
    p.add_argument('--baud',    default=115200, type=int)
    p.add_argument('--no-save', action='store_true',
                   help='Do not save grid to disk')
    p.add_argument('--load',    default=None,
                   help='Load a previously saved grid .npy file')
    return p.parse_args()


def get_bridge(args: argparse.Namespace):
    """Return the appropriate Bridge (real or simulated)."""
    if args.port:
        try:
            from aria.bridge_hw import BridgeHW   # type: ignore
            log.info(f"Connecting to hardware on {args.port} @ {args.baud} baud")
            return BridgeHW(args.port, args.baud)
        except ImportError:
            log.warning("bridge_hw not found — falling back to simulator")
    log.info("Running in SIMULATOR mode (no hardware required)")
    return BridgeStub()


def main() -> None:
    args = parse_args()

    # ── Initialise ───────────────────────────────────────────────────────────
    bridge = get_bridge(args)

    ekf = ARIALocalization(start_x=0.0, start_y=0.0, start_theta=0.0)

    grid = OccupancyGrid.load(args.load) if args.load else OccupancyGrid()

    # ── Graceful shutdown ────────────────────────────────────────────────────
    running = True
    def _shutdown(sig, frame):
        nonlocal running
        log.info("Shutting down…")
        running = False
    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── Timing ───────────────────────────────────────────────────────────────
    dt          = 1.0 / BRIDGE_UPLINK_HZ
    next_tick   = time.monotonic()
    last_viz    = 0.0
    last_save   = time.monotonic()

    prev_enc_l  = 0
    prev_enc_r  = 0
    first_packet = True

    log.info("ARIA Phase 3 running — Ctrl-C to stop")

    # ── Main loop ─────────────────────────────────────────────────────────────
    while running:
        now = time.monotonic()
        sleep_for = next_tick - now
        if sleep_for > 0:
            time.sleep(sleep_for)
        next_tick += dt

        # 1. Get sensors
        pkt = bridge.get_sensors()

        # 2. Encoder deltas (handle first packet)
        if first_packet:
            prev_enc_l = pkt.enc_left
            prev_enc_r = pkt.enc_right
            first_packet = False
            continue

        delta_l = pkt.enc_left  - prev_enc_l
        delta_r = pkt.enc_right - prev_enc_r
        prev_enc_l = pkt.enc_left
        prev_enc_r = pkt.enc_right

        # 3. EKF predict (encoder dead-reckoning)
        ekf.predict(delta_l, delta_r)

        # 4. EKF correct (IMU heading)
        ekf.correct_imu(pkt.gyro_z, dt)

        # 5. Wall snap — use side ultrasonics
        x, y, theta = ekf.pose
        us = pkt.ultrasonics

        if us.get('left', 999) < US_WALL_SNAP_CM:
            # Left wall is very close — we know our X
            wall_x = x - us['left']
            ekf.wall_snap('left', wall_x + CELL_SIZE_CM / 2)
            log.debug(f"Wall snap LEFT  x={wall_x:.1f} cm")

        if us.get('right', 999) < US_WALL_SNAP_CM:
            wall_x = x + us['right']
            ekf.wall_snap('right', wall_x - CELL_SIZE_CM / 2)
            log.debug(f"Wall snap RIGHT x={wall_x:.1f} cm")

        # 6. Update occupancy grid
        x, y, theta = ekf.pose   # re-read after any snap
        grid.mark_cleaned(x, y)
        grid.update_from_ultrasonics(x, y, theta, us)

        # 7. Terminal visualisation
        if now - last_viz >= VIZ_INTERVAL_S:
            grid.print_terminal(robot_x=x, robot_y=y)
            log.info(f"Pose: x={x:+7.1f} y={y:+7.1f} θ={theta:+.2f} rad | "
                     f"coverage={grid.coverage_percent():.1f}%")
            last_viz = now

        # 8. Periodic save
        if not args.no_save and (now - last_save >= SAVE_INTERVAL_S):
            grid.save(SAVE_PATH)
            log.info(f"Grid saved → {SAVE_PATH}")
            last_save = now

    # ── Shutdown ─────────────────────────────────────────────────────────────
    if not args.no_save:
        grid.save(SAVE_PATH)
        log.info(f"Final grid saved → {SAVE_PATH}")

    log.info(f"Done. Coverage: {grid.coverage_percent():.1f}%  "
             f"Cleaned: {grid.total_cleaned()} cells")


if __name__ == '__main__':
    main()
