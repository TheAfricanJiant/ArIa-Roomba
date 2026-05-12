"""
ARIA — Telemetry
Reads XRW serial (encoders + IMU), fuses with EKF, polls UNO Q ultrasonics via Bridge RPC.

Ultrasonic readings are fetched from the UNO Q sketch.ino every 200ms using:
    Bridge.call("get_us_front") / "get_us_right" / "get_us_left"
These return float cm (999 = out of range).

Stationary detection: if |accel_xy| < 0.3 m/s² AND |gyro_z| < 0.04 rad/s
AND delta_enc == 0 for 2+ consecutive cycles, pose update is skipped.
This prevents the EKF from drifting when the robot is at rest.
"""

import time
import math
import logging
import threading
import psutil
import datetime
import serial_bridge

log = logging.getLogger(__name__)

_system_metrics_db = None

def set_system_metrics_store(db):
    global _system_metrics_db
    _system_metrics_db = db

# ── Constants ─────────────────────────────────────────────────────────────────
STATIONARY_ACCEL_THRESH = 0.3   # m/s²  (XY plane)
STATIONARY_GYRO_THRESH  = 0.04  # rad/s
US_POLL_INTERVAL_S      = 0.2   # poll UNO Q ultrasonics every 200ms
US_MAX_VALID_CM         = 380.0
MAP_PUSH_INTERVAL_S     = 1.0

# XRW reports raw encoder counts as T,encL,encR,...
# On this robot the left wheel is wired to the XRP Motor 4 port, and a
# hand-forward turn makes its raw count decrease.  ARIA's odometry expects
# forward wheel travel to be positive for both sides, so correct the sign here
# before EKF/dead-reckoning sees the counts.
ENCODER_L_SIGN          = -1
ENCODER_R_SIGN          = 1


# ── Simple dead-reckoning fallback (no dependencies) ─────────────────────────
class SimpleDeadReckoning:
    """Minimal encoder+gyro pose tracker. No filterpy required."""
    CM_PER_TICK = (math.pi * 6.0) / 360   # XRP: 6 cm wheel, 360 ticks/rev
    WHEEL_BASE  = 15.5                     # XRP track width cm

    def __init__(self):
        self.x = 0.0; self.y = 0.0; self.theta = 0.0

    def update(self, delta_l: int, delta_r: int, gyro_z: float, dt: float):
        d_l = delta_l * self.CM_PER_TICK
        d_r = delta_r * self.CM_PER_TICK
        d_c = (d_l + d_r) * 0.5
        # Fix #5 (2026-05): fuse encoder-derived rotation with gyro 50/50.
        # Previously only gyro was used, so any gyro bias caused rapid drift.
        d_theta_enc = (d_r - d_l) / self.WHEEL_BASE
        d_theta_imu = gyro_z * dt
        d_theta = 0.5 * d_theta_enc + 0.5 * d_theta_imu
        half = d_theta * 0.5
        self.theta = _wrap(self.theta + half)
        self.x += d_c * math.cos(self.theta)
        self.y += d_c * math.sin(self.theta)
        self.theta = _wrap(self.theta + half)

    def reset(self):
        self.x = 0.0; self.y = 0.0; self.theta = 0.0

    @property
    def pose(self):
        return self.x, self.y, self.theta


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ── Simple grid tracker (no numpy) ───────────────────────────────────────────
class SimpleGrid:
    CELL_CM = 15; SIZE = 67; ORIGIN = 33
    UNKNOWN = 127; CLEANED = 0

    def __init__(self):
        self._data = [[self.UNKNOWN] * self.SIZE for _ in range(self.SIZE)]
        self._cleaned = 0

    def mark_cleaned(self, x_cm, y_cm):
        col = self.ORIGIN + int(x_cm / self.CELL_CM)
        row = self.ORIGIN - int(y_cm / self.CELL_CM)
        if 0 <= col < self.SIZE and 0 <= row < self.SIZE:
            if self._data[row][col] == self.UNKNOWN:
                self._cleaned += 1
            self._data[row][col] = self.CLEANED

    def mark_obstacle(self, x_cm, y_cm):
        col = self.ORIGIN + int(x_cm / self.CELL_CM)
        row = self.ORIGIN - int(y_cm / self.CELL_CM)
        if 0 <= col < self.SIZE and 0 <= row < self.SIZE:
            self._data[row][col] = 127  # wall/obstacle

    def coverage_percent(self):
        return round(100.0 * self._cleaned / (self.SIZE * self.SIZE), 1)

    def snapshot(self):
        return {
            "cols": self.SIZE, "rows": self.SIZE,
            "cell_cm": self.CELL_CM,
            "origin_col": self.ORIGIN, "origin_row": self.ORIGIN,
            "data": self._data,
            "coverage": self.coverage_percent(),
        }


# ── Obstacle grid for nav map ─────────────────────────────────────────────────
class ObstacleGrid:
    """Separate grid updated by ultrasonic readings for the Nav map tab."""
    CELL_CM = 15; SIZE = 67; ORIGIN = 33

    def __init__(self):
        # 0 = unknown, 1-100 = obstacle probability %
        self._data = [[0] * self.SIZE for _ in range(self.SIZE)]

    def update(self, robot_x, robot_y, robot_theta, ultrasonics: dict):
        """Project each ultrasonic reading onto the grid as an obstacle."""
        sensor_angles = {
            "front": 0.0,
            "right": -math.pi / 2,
            "left":   math.pi / 2,
        }
        for name, dist in ultrasonics.items():
            if dist >= US_MAX_VALID_CM or dist <= 0:
                continue
            angle = sensor_angles.get(name, 0.0)
            abs_angle = robot_theta + angle
            obs_x = robot_x + dist * math.cos(abs_angle)
            obs_y = robot_y + dist * math.sin(abs_angle)
            col = self.ORIGIN + int(obs_x / self.CELL_CM)
            row = self.ORIGIN - int(obs_y / self.CELL_CM)
            if 0 <= col < self.SIZE and 0 <= row < self.SIZE:
                # Increment confidence (cap at 100)
                self._data[row][col] = min(100, self._data[row][col] + 20)

    def snapshot(self):
        return {
            "cols": self.SIZE, "rows": self.SIZE,
            "cell_cm": self.CELL_CM,
            "origin_col": self.ORIGIN, "origin_row": self.ORIGIN,
            "data": self._data,
        }


# ── Load full EKF + OccupancyGrid (requires filterpy + numpy) ────────────────
_EKF_AVAILABLE = False
ekf = None
grid = None
CELL_SIZE_CM = 15; GRID_COLS = 67; GRID_ROWS = 67
GRID_ORIGIN_ROW = 33; GRID_ORIGIN_COL = 33

_dr     = SimpleDeadReckoning()
_sgrid  = SimpleGrid()
_obs_grid = ObstacleGrid()

try:
    from aria import ARIALocalization, OccupancyGrid
    from aria.config import (CELL_SIZE_CM, GRID_COLS, GRID_ROWS,
                             GRID_ORIGIN_ROW, GRID_ORIGIN_COL)
    ekf  = ARIALocalization(0.0, 0.0, 0.0)
    grid = OccupancyGrid()
    _EKF_AVAILABLE = True
    log.info("ARIA EKF + OccupancyGrid loaded — full accuracy mode.")
except Exception as e:
    log.warning(f"ARIA unavailable ({e}). Using simple dead-reckoning.")


# ── Raw sensor state ──────────────────────────────────────────────────────────
telemetry = {
    "enc_l": 0, "enc_r": 0,
    "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
    "gyro_x":  0.0, "gyro_y": 0.0,  "gyro_z":  0.0,
    "us_front": 999.0, "us_right": 999.0, "us_left": 999.0,
}

_last_enc_l:    int   = 0
_last_enc_r:    int   = 0
_last_ts:       float = time.time()
_last_map_push: float = 0.0
_stationary_count: int = 0   # consecutive ticks with no motion


# ── Public API ────────────────────────────────────────────────────────────────
def get_pose() -> dict:
    if _EKF_AVAILABLE and ekf:
        x, y, theta = ekf.pose
    else:
        x, y, theta = _dr.pose
    return {"x_cm": round(x, 2), "y_cm": round(y, 2), "theta_rad": round(theta, 4)}


def get_grid_snapshot() -> dict:
    if _EKF_AVAILABLE and grid:
        data = grid._grid.tolist()
        return {
            "cols": GRID_COLS, "rows": GRID_ROWS,
            "cell_cm": CELL_SIZE_CM,
            "origin_col": GRID_ORIGIN_COL,
            "origin_row": GRID_ORIGIN_ROW,
            "data": data,
            "coverage": round(grid.coverage_percent(), 1),
        }
    return _sgrid.snapshot()


def get_obstacle_snapshot() -> dict:
    """Return the obstacle grid plus current robot pose and active goal."""
    snap = _obs_grid.snapshot()
    snap["robot"] = get_pose()   # Fix #3 (2026-05): nav canvas needs robot coords
    # Expose the active goal if the navigator module is available
    try:
        import navigator as _nav_mod
        # navigator module exposes a module-level nav object via main.py,
        # but we avoid a circular import by checking the cached attribute.
        _nav = getattr(_nav_mod, '_cached_nav', None)
        if _nav and _nav.goal:
            snap["goal"] = {"x_cm": _nav.goal[0], "y_cm": _nav.goal[1]}
        else:
            snap["goal"] = None
    except Exception:
        snap["goal"] = None
    return snap


def get_ultrasonics() -> dict:
    return {
        "front": telemetry["us_front"],
        "right": telemetry["us_right"],
        "left":  telemetry["us_left"],
    }


def reset_encoders():
    """Zero encoder counts in XRW firmware and reset local pose estimate."""
    global _last_enc_l, _last_enc_r
    serial_bridge.send("R\n")
    _last_enc_l = 0
    _last_enc_r = 0
    telemetry["enc_l"] = 0
    telemetry["enc_r"] = 0
    _dr.reset()
    if ekf:
        ekf.reset(0.0, 0.0, ekf.theta_rad)  # keep heading, reset position
    _sgrid.__init__()
    _obs_grid.__init__()
    log.info("Encoders and pose reset to origin.")


# ── Parse XRW telemetry line ──────────────────────────────────────────────────
def _parse_line(line: str) -> bool:
    if not line.startswith('T,'):
        return False
    parts = line.split(',')
    if len(parts) < 9:
        return False
    try:
        telemetry["enc_l"]   = ENCODER_L_SIGN * int(parts[1])
        telemetry["enc_r"]   = ENCODER_R_SIGN * int(parts[2])
        telemetry["accel_x"] = float(parts[3])
        telemetry["accel_y"] = float(parts[4])
        telemetry["accel_z"] = float(parts[5])
        telemetry["gyro_x"]  = float(parts[6])
        telemetry["gyro_y"]  = float(parts[7])
        telemetry["gyro_z"]  = float(parts[8])
        return True
    except (ValueError, IndexError) as e:
        log.warning(f"Parse error '{line}': {e}")
        return False


# ── Stationary detection ──────────────────────────────────────────────────────
def _is_stationary(delta_l, delta_r) -> bool:
    """True if IMU + encoders agree the robot is not moving."""
    global _stationary_count
    accel_mag = math.hypot(telemetry["accel_x"], telemetry["accel_y"])
    gyro_mag  = abs(telemetry["gyro_z"])
    enc_zero  = (delta_l == 0 and delta_r == 0)

    if enc_zero and accel_mag < STATIONARY_ACCEL_THRESH and gyro_mag < STATIONARY_GYRO_THRESH:
        _stationary_count += 1
    else:
        _stationary_count = 0

    return _stationary_count >= 2   # must be still for 2+ consecutive ticks


# ── Pose update step ──────────────────────────────────────────────────────────
def _run_pose_step():
    global _last_enc_l, _last_enc_r, _last_ts

    now = time.time()
    dt  = max(now - _last_ts, 1e-6)
    _last_ts = now

    delta_l = telemetry["enc_l"] - _last_enc_l
    delta_r = telemetry["enc_r"] - _last_enc_r
    _last_enc_l = telemetry["enc_l"]
    _last_enc_r = telemetry["enc_r"]

    # Skip update if robot is stationary (prevents EKF drift)
    if _is_stationary(delta_l, delta_r):
        return

    try:
        if _EKF_AVAILABLE and ekf:
            if delta_l != 0 or delta_r != 0:
                ekf.predict(delta_l, delta_r)
            if dt > 0:
                ekf.correct_imu(telemetry["gyro_z"], dt)
            x, y, _ = ekf.pose
            if grid:
                grid.mark_cleaned(x, y)

            # Fix #6 (2026-05): wall-snap — eliminate lateral drift when robot
            # is within US_WALL_SNAP_CM of a side wall.  We don't know the room
            # dimensions exactly, so we snap the *offset* (not an absolute coord).
            # The snap moves the EKF estimate to account for the measured wall gap.
            us = get_ultrasonics()
            snap_thresh = 8.0  # cm — must match US_WALL_SNAP_CM in config.py
            if us["left"] <= snap_thresh and us["left"] > 0:
                # Robot is flush-left: current x minus sensor gap = left boundary x
                ekf.wall_snap("left", x - us["left"])
            if us["right"] <= snap_thresh and us["right"] > 0:
                ekf.wall_snap("right", x + us["right"])
        else:
            _dr.update(delta_l, delta_r, telemetry["gyro_z"], dt)
            x, y, _ = _dr.pose
            _sgrid.mark_cleaned(x, y)

        # Update obstacle grid with latest ultrasonic readings
        pose = get_pose()
        _obs_grid.update(pose["x_cm"], pose["y_cm"], pose["theta_rad"], get_ultrasonics())

    except Exception as e:
        log.error(f"Pose step error: {e}")


# ── System Health polling thread ──────────────────────────────────────────────
def _system_health_loop(ui):
    """Background thread: poll CPU and RAM using psutil."""
    log.info("System Health poll thread started.")
    while True:
        try:
            ts = int(time.time() * 1000)
            cpu_percent = psutil.cpu_percent(interval=1)
            mem_percent = psutil.virtual_memory().percent
            if _system_metrics_db is not None:
                _system_metrics_db.write_sample('cpu', cpu_percent, ts)
                _system_metrics_db.write_sample('mem', mem_percent, ts)
            ui.send_message('cpu_usage', {"value": cpu_percent, "ts": ts})
            ui.send_message('memory_usage', {"value": mem_percent, "ts": ts})
        except Exception as e:
            log.debug(f"System Health poll error: {e}")
            time.sleep(5)
        time.sleep(4)  # interval=1 + 4 = 5 seconds total

# ── Ultrasonic polling thread ─────────────────────────────────────────────────
def _us_poll_loop(ui=None):
    """Background thread: poll UNO Q ultrasonics via Bridge RPC every 200ms."""
    try:
        from arduino.app_utils import Bridge
    except ImportError:
        log.warning("arduino.app_utils not available — no ultrasonic readings.")
        return

    log.info("Ultrasonic poll thread started.")
    while True:
        try:
            f = Bridge.call("get_us_front")
            r = Bridge.call("get_us_right")
            l = Bridge.call("get_us_left")
            telemetry["us_front"] = float(f) if f is not None else 999.0
            telemetry["us_right"] = float(r) if r is not None else 999.0
            telemetry["us_left"]  = float(l) if l is not None else 999.0
            if ui:
                ui.send_message('us_update', get_ultrasonics())
        except Exception as e:
            log.debug(f"US poll error: {e}")
        time.sleep(US_POLL_INTERVAL_S)


# ── Telemetry main loop ───────────────────────────────────────────────────────
def telemetry_loop(ui) -> None:
    """Background thread: read XRW serial, run EKF, push to WebUI."""
    global _last_map_push
    # Ultrasonic polling is disabled for now: the board-side stream is not
    # consistently available in the web UI, and stale readings made waypoint
    # following unsafe. Re-enable _us_poll_loop only after that stream is fixed.
    threading.Thread(target=_system_health_loop, args=(ui,), daemon=True).start()
    log.info("Telemetry loop started.")

    while True:
        try:
            if serial_bridge.is_connected():
                while True:
                    line = serial_bridge.readline()
                    if not line:
                        break
                    if _parse_line(line):
                        _run_pose_step()
                        ui.send_message('telemetry_update', telemetry)
                        ui.send_message('ekf_update', get_pose())

                now = time.time()
                if now - _last_map_push >= MAP_PUSH_INTERVAL_S:
                    _last_map_push = now
                    snap = get_grid_snapshot()
                    if snap:
                        ui.send_message('map_update', snap)
                    ui.send_message('obstacle_map_update', get_obstacle_snapshot())
                    log.debug(f"Pose → {get_pose()}")
        except Exception as e:
            log.error(f"telemetry_loop error: {e}")
        time.sleep(0.05)
