"""
ARIA — Robot Configuration
All physical constants and tuning parameters.
Edit these values to match your actual hardware before running.
"""
import math

# ── Robot geometry ──────────────────────────────────────────────────────────
WHEEL_BASE_CM       = 15.5      # Track width of XRP robot (cm)
WHEEL_DIAMETER_CM   = 6.0       # Standard XRP wheel diameter (cm)
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER_CM
TICKS_PER_REV       = 360       # Encoder pulses per full wheel revolution
CM_PER_TICK         = WHEEL_CIRCUMFERENCE / TICKS_PER_REV

# ── Occupancy grid ───────────────────────────────────────────────────────────
CELL_SIZE_CM        = 15        # Each grid cell = 15 cm x 15 cm
GRID_WIDTH_CM       = 1000      # Max room width  (10 m)
GRID_HEIGHT_CM      = 1000      # Max room height (10 m)
GRID_COLS           = math.ceil(GRID_WIDTH_CM / CELL_SIZE_CM)
GRID_ROWS           = math.ceil(GRID_HEIGHT_CM / CELL_SIZE_CM)
# Robot starts at grid centre.
GRID_ORIGIN_COL     = GRID_COLS // 2
GRID_ORIGIN_ROW     = GRID_ROWS // 2

# ── Ultrasonic sensor angles (radians, relative to robot heading) ─────────────
# Sensor layout: front=0°, front-left=45°, front-right=-45°,
#                left=90°, right=-90°, rear=180°
US_ANGLES = {
    "front":       0.0,
    "front_left":  math.pi / 4,
    "front_right": -math.pi / 4,
    "left":        math.pi / 2,
    "right":       -math.pi / 2,
    "rear":        math.pi,
}

# ── EKF tuning ───────────────────────────────────────────────────────────────
EKF_PROCESS_NOISE_XY    = 0.5   # Position noise (cm²) per update
EKF_PROCESS_NOISE_THETA = 0.01  # Heading noise (rad²) per update
EKF_OBS_NOISE_IMU       = 0.05  # IMU gyro observation noise (rad²)

# ── Sensor thresholds ────────────────────────────────────────────────────────
US_MAX_VALID_CM         = 380   # Ignore readings above this (bad echo)
US_WALL_SNAP_CM         = 8     # Side US < this → snap EKF to wall boundary
US_OBSTACLE_STOP_CM     = 15    # Front US < this → trigger AVOID state

# ── Timing ───────────────────────────────────────────────────────────────────
BRIDGE_UPLINK_HZ        = 20    # Sensor packets per second from STM32
EKF_UPDATE_HZ           = 20    # EKF prediction rate (matches bridge)
GRID_VISUALIZE_HZ       = 2     # Terminal map refresh rate
