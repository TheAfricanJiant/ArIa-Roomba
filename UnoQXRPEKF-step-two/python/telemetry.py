import time
import math
import logging
import serial_bridge

log = logging.getLogger(__name__)

# ── Simple dead-reckoning fallback (no dependencies) ─────────────────────────
class SimpleDeadReckoning:
    """Minimal encoder-only pose tracker. No filterpy required."""
    CM_PER_TICK  = (math.pi * 6.0) / 360   # XRP wheel diameter = 6.0 cm
    WHEEL_BASE   = 15.5                    # XRP track width = 15.5 cm

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, delta_l: int, delta_r: int, gyro_z: float, dt: float):
        d_l = delta_l * self.CM_PER_TICK
        d_r = delta_r * self.CM_PER_TICK
        d_c = (d_l + d_r) * 0.5
        
        # 1. Use GYRO for rotation, not wheels! (Odometry-IMU fusion)
        # Discard the inaccurate (d_r - d_l) / WHEEL_BASE
        d_theta = gyro_z * dt
        
        # 2. RTR Model: Rotate half, Translate, Rotate half
        half_theta = d_theta * 0.5
        
        # Rotate first half
        self.theta = (self.theta + half_theta + math.pi) % (2 * math.pi) - math.pi
        
        # Translate
        self.x += d_c * math.cos(self.theta)
        self.y += d_c * math.sin(self.theta)
        
        # Rotate second half
        self.theta = (self.theta + half_theta + math.pi) % (2 * math.pi) - math.pi

    @property
    def pose(self):
        return self.x, self.y, self.theta

# ── Simple grid tracker (no numpy) ───────────────────────────────────────────
class SimpleGrid:
    CELL_CM = 30
    SIZE    = 33
    ORIGIN  = 16
    UNKNOWN = 127
    CLEANED = 0

    def __init__(self):
        self._data = [[self.UNKNOWN] * self.SIZE for _ in range(self.SIZE)]
        self._cleaned = 0

    def mark_cleaned(self, x_cm: float, y_cm: float):
        col = self.ORIGIN + int(x_cm / self.CELL_CM)
        row = self.ORIGIN - int(y_cm / self.CELL_CM)
        if 0 <= col < self.SIZE and 0 <= row < self.SIZE:
            if self._data[row][col] == self.UNKNOWN:
                self._cleaned += 1
            self._data[row][col] = self.CLEANED

    def coverage_percent(self) -> float:
        total = self.SIZE * self.SIZE
        return round(100.0 * self._cleaned / total, 1)

    def snapshot(self) -> dict:
        return {
            "cols": self.SIZE, "rows": self.SIZE,
            "cell_cm": self.CELL_CM,
            "origin_col": self.ORIGIN, "origin_row": self.ORIGIN,
            "data": self._data,
            "coverage": self.coverage_percent(),
        }

# ── Try to load full EKF/Grid (requires filterpy + numpy) ────────────────────
_EKF_AVAILABLE = False
ekf = None
grid = None
CELL_SIZE_CM = 30
GRID_COLS = 33
GRID_ROWS = 33

# Always create fallback objects (work without any deps)
_dr  = SimpleDeadReckoning()
_sgrid = SimpleGrid()

try:
    from aria import ARIALocalization, OccupancyGrid
    from aria.config import CELL_SIZE_CM, GRID_COLS, GRID_ROWS, GRID_ORIGIN_ROW, GRID_ORIGIN_COL
    ekf  = ARIALocalization(start_x=0.0, start_y=0.0, start_theta=0.0)
    grid = OccupancyGrid()
    _EKF_AVAILABLE = True
    log.info("ARIA EKF + OccupancyGrid loaded — full accuracy mode.")
except Exception as e:
    log.warning(f"ARIA unavailable ({e}). Using simple dead-reckoning fallback.")
    log.warning("Upgrade: pip install -r python/requirements.txt")

# ── Raw sensor state ──────────────────────────────────────────────────────────
telemetry = {
    "enc_l": 0, "enc_r": 0,
    "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
    "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0
}

_last_enc_l: int = 0
_last_enc_r: int = 0
_last_ts: float = time.time()
_last_map_push: float = 0.0
MAP_PUSH_INTERVAL_S = 1.0


def get_pose() -> dict:
    """Return pose from EKF if available, otherwise simple dead-reckoning."""
    if _EKF_AVAILABLE and ekf is not None:
        x, y, theta = ekf.pose
    else:
        x, y, theta = _dr.pose
    return {"x_cm": round(x, 2), "y_cm": round(y, 2), "theta_rad": round(theta, 4)}


def get_grid_snapshot() -> dict:
    """Return grid snapshot from full grid or simple fallback."""
    if _EKF_AVAILABLE and grid is not None:
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


def _run_pose_step() -> None:
    """Update pose estimate and grid. Always runs (EKF or fallback)."""
    global _last_enc_l, _last_enc_r, _last_ts

    now = time.time()
    dt = now - _last_ts
    _last_ts = now

    delta_l = telemetry["enc_l"] - _last_enc_l
    delta_r = telemetry["enc_r"] - _last_enc_r
    _last_enc_l = telemetry["enc_l"]
    _last_enc_r = telemetry["enc_r"]

    try:
        if _EKF_AVAILABLE and ekf is not None:
            if delta_l != 0 or delta_r != 0:
                ekf.predict(delta_l, delta_r)
            if dt > 0:
                ekf.correct_imu(telemetry["gyro_z"], dt)
            x, y, _ = ekf.pose
            if grid is not None:
                grid.mark_cleaned(x, y)   # always mark current cell
        else:
            # Fallback: RTR model with Odometry-IMU fusion
            _dr.update(delta_l, delta_r, telemetry["gyro_z"], dt)
            x, y, _ = _dr.pose
            _sgrid.mark_cleaned(x, y)     # always mark current cell
    except Exception as e:
        log.error(f"Pose step error: {e}")


def telemetry_loop(ui) -> None:
    """Background thread: read serial, run EKF, push updates to WebUI."""
    global _last_map_push
    print("TELEMETRY THREAD STARTED! 🚀", flush=True)

    while True:
        try:
            if serial_bridge.is_connected():
                # Drain all available lines
                while True:
                    line = serial_bridge.readline()
                    if not line:
                        break
                    if _parse_line(line):
                        _run_pose_step()
                        ui.send_message('telemetry_update', telemetry)
                        ui.send_message('ekf_update', get_pose())

                # Push grid snapshot every MAP_PUSH_INTERVAL_S
                now = time.time()
                if now - _last_map_push >= MAP_PUSH_INTERVAL_S:
                    _last_map_push = now
                    snap = get_grid_snapshot()
                    if snap:
                        ui.send_message('map_update', snap)
                        # Hard print to bypass logging configuration issues
                        print(f"BROADCASTING MAP & POSE -> {get_pose()}", flush=True)
        except Exception as e:
            print(f"CRITICAL THREAD ERROR: {e}")
            log.error(f"telemetry_loop error: {e}")

        time.sleep(0.05)