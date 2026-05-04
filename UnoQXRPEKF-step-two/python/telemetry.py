import time
import logging
import serial_bridge
from aria import ARIALocalization, OccupancyGrid
from aria.config import CELL_SIZE_CM, GRID_COLS, GRID_ROWS

log = logging.getLogger(__name__)

# ── Raw sensor state ──────────────────────────────────────────────────────────
telemetry = {
    "enc_l": 0, "enc_r": 0,
    "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
    "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0
}

# ── EKF + Grid ────────────────────────────────────────────────────────────────
ekf = ARIALocalization(start_x=0.0, start_y=0.0, start_theta=0.0)
grid = OccupancyGrid()

_last_enc_l: int = 0
_last_enc_r: int = 0
_last_ts: float = time.time()
_last_map_push: float = 0.0
MAP_PUSH_INTERVAL_S = 1.0  # push grid to UI every 1 s


def get_ekf_pose() -> dict:
    x, y, theta = ekf.pose
    return {"x_cm": round(x, 2), "y_cm": round(y, 2), "theta_rad": round(theta, 4)}


def get_grid_snapshot() -> dict:
    """Return a compact grid for the canvas renderer."""
    data = grid._grid.tolist()   # List[List[int]] — uint8 values
    return {
        "cols": GRID_COLS,
        "rows": GRID_ROWS,
        "cell_cm": CELL_SIZE_CM,
        "origin_col": grid._origin_col,
        "origin_row": grid._origin_row,
        "data": data,
        "coverage": round(grid.coverage_percent(), 1),
    }


def _parse_line(line: str) -> bool:
    """Parse a T,… telemetry line. Returns True on success."""
    if not line.startswith('T,'):
        return False
    parts = line.split(',')
    if len(parts) < 9:
        return False
    try:
        telemetry["enc_l"]   = int(parts[1])
        telemetry["enc_r"]   = int(parts[2])
        telemetry["accel_x"] = float(parts[3])
        telemetry["accel_y"] = float(parts[4])
        telemetry["accel_z"] = float(parts[5])
        telemetry["gyro_x"]  = float(parts[6])
        telemetry["gyro_y"]  = float(parts[7])
        telemetry["gyro_z"]  = float(parts[8])
        return True
    except (ValueError, IndexError) as e:
        log.warning(f"Parse error on '{line}': {e}")
        return False


def _run_ekf_step() -> None:
    """Calculate encoder deltas, run EKF predict + correct, update grid."""
    global _last_enc_l, _last_enc_r, _last_ts

    now = time.time()
    dt = now - _last_ts
    _last_ts = now

    delta_l = telemetry["enc_l"] - _last_enc_l
    delta_r = telemetry["enc_r"] - _last_enc_r
    _last_enc_l = telemetry["enc_l"]
    _last_enc_r = telemetry["enc_r"]

    # EKF predict from encoder deltas
    if delta_l != 0 or delta_r != 0:
        ekf.predict(delta_l, delta_r)

    # EKF correct with IMU gyro Z
    if dt > 0:
        ekf.correct_imu(telemetry["gyro_z"], dt)

    # Mark robot position on occupancy grid
    x, y, _ = ekf.pose
    grid.mark_cleaned(x, y)


def telemetry_loop(ui) -> None:
    """Background thread: read serial, run EKF, push updates to WebUI."""
    global _last_map_push

    while True:
        if serial_bridge.is_connected():
            # Drain all available lines from the buffer
            while True:
                line = serial_bridge.readline()
                if not line:
                    break
                if _parse_line(line):
                    _run_ekf_step()
                    ui.send_message('telemetry_update', telemetry)
                    ui.send_message('ekf_update', get_ekf_pose())

            # Push full grid snapshot every MAP_PUSH_INTERVAL_S
            now = time.time()
            if now - _last_map_push >= MAP_PUSH_INTERVAL_S:
                _last_map_push = now
                ui.send_message('map_update', get_grid_snapshot())

        time.sleep(0.05)