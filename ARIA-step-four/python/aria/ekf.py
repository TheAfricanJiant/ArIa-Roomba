"""
ARIA — EKF Localization
Extended Kalman Filter fusing wheel encoders + IMU for stable [x, y, θ] tracking.

State vector:  x = [x_cm, y_cm, theta_rad]
Measurements:  encoder deltas (predict) + IMU gyro Z (correct)
Wall snap:     resets lateral drift when side ultrasonic detects a boundary

Performance: ~0.05 ms per update on UNO Q Linux (Qualcomm MPU).
"""

import math
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

from .config import (
    CM_PER_TICK, WHEEL_BASE_CM,
    EKF_PROCESS_NOISE_XY, EKF_PROCESS_NOISE_THETA,
    EKF_OBS_NOISE_IMU, US_WALL_SNAP_CM,
)


class ARIALocalization:
    """
    EKF-based localization for a differential-drive robot.

    Usage:
        ekf = ARIALocalization(start_x=0.0, start_y=0.0, start_theta=0.0)
        ekf.predict(enc_left_delta, enc_right_delta)   # every 50 ms
        ekf.correct_imu(gyro_z_rad_per_s, dt)          # every 50 ms
        ekf.wall_snap(side='left', wall_x_cm=30.0)     # when US < 8 cm
        x, y, theta = ekf.pose
    """

    def __init__(self, start_x: float = 0.0, start_y: float = 0.0,
                 start_theta: float = 0.0) -> None:
        self._ekf = ExtendedKalmanFilter(dim_x=3, dim_z=1)

        # Initial state [x, y, θ]
        self._ekf.x = np.array([[start_x], [start_y], [start_theta]],
                                dtype=np.float64)

        # Initial covariance — moderate uncertainty at start
        self._ekf.P = np.diag([10.0, 10.0, 0.1])

        # Process noise Q
        self._ekf.Q = np.diag([
            EKF_PROCESS_NOISE_XY,
            EKF_PROCESS_NOISE_XY,
            EKF_PROCESS_NOISE_THETA,
        ])

        # Observation noise R (IMU gyro Z)
        self._ekf.R = np.array([[EKF_OBS_NOISE_IMU]])

        # Observation matrix H: we observe theta_dot ≈ d_theta/dt
        # Updated each step from the non-linear prediction
        self._ekf.H = np.array([[0.0, 0.0, 1.0]])

        # Cache
        self._last_enc_l: int = 0
        self._last_enc_r: int = 0

    # ── Public interface ─────────────────────────────────────────────────────

    @property
    def pose(self) -> tuple[float, float, float]:
        """Return (x_cm, y_cm, theta_rad)."""
        x = self._ekf.x
        return float(x[0, 0]), float(x[1, 0]), float(x[2, 0])

    @property
    def x_cm(self) -> float:
        return float(self._ekf.x[0, 0])

    @property
    def y_cm(self) -> float:
        return float(self._ekf.x[1, 0])

    @property
    def theta_rad(self) -> float:
        return float(self._ekf.x[2, 0])

    def predict(self, enc_left_delta: int, enc_right_delta: int) -> None:
        """
        Dead-reckoning prediction from encoder tick deltas.
        Call once per Bridge uplink packet (every 50 ms).

        Args:
            enc_left_delta:  Ticks since last call (left wheel)
            enc_right_delta: Ticks since last call (right wheel)
        """
        d_l = enc_left_delta  * CM_PER_TICK
        d_r = enc_right_delta * CM_PER_TICK
        d_c = (d_l + d_r) * 0.5
        d_theta = (d_r - d_l) / WHEEL_BASE_CM

        theta = self.theta_rad

        # Non-linear motion model
        cos_t = math.cos(theta + d_theta * 0.5)  # mid-point integration
        sin_t = math.sin(theta + d_theta * 0.5)

        # Jacobian F (linearised motion model)
        F = np.array([
            [1.0, 0.0, -d_c * sin_t],
            [0.0, 1.0,  d_c * cos_t],
            [0.0, 0.0,  1.0],
        ])

        # Apply motion
        self._ekf.x[0, 0] += d_c * cos_t
        self._ekf.x[1, 0] += d_c * sin_t
        self._ekf.x[2, 0] += d_theta
        self._ekf.x[2, 0] = self._wrap_angle(self._ekf.x[2, 0])

        # Propagate covariance
        self._ekf.P = F @ self._ekf.P @ F.T + self._ekf.Q

    def correct_imu(self, gyro_z: float, dt: float) -> None:
        """
        EKF update step using IMU gyroscope Z-axis (yaw rate).
        Call after each predict().

        Args:
            gyro_z: Angular velocity in rad/s from MPU6050
            dt:     Time step in seconds (typically 0.05 s)
        """
        # Expected angular change from gyro
        z = np.array([[gyro_z * dt]])

        # Innovation: difference between measured and predicted heading change
        # (predicted change is already in x[2] after predict())
        # We observe delta_theta directly from the gyro
        self._ekf.H = np.array([[0.0, 0.0, 1.0]])
        z_pred = np.array([[self._ekf.x[2, 0]]])  # predicted θ
        # Map gyro to an absolute angle for the update
        # Simple: use gyro-integrated angle as observation of theta
        z_obs = np.array([[self._wrap_angle(z_pred[0, 0] + gyro_z * dt)]])

        # Kalman gain
        S = self._ekf.H @ self._ekf.P @ self._ekf.H.T + self._ekf.R
        K = self._ekf.P @ self._ekf.H.T @ np.linalg.inv(S)

        # Update state
        innovation = z_obs - self._ekf.H @ self._ekf.x
        self._ekf.x += K @ innovation
        self._ekf.x[2, 0] = self._wrap_angle(self._ekf.x[2, 0])

        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(3) - K @ self._ekf.H
        self._ekf.P = I_KH @ self._ekf.P @ I_KH.T + K @ self._ekf.R @ K.T

    def wall_snap(self, side: str, wall_coord_cm: float) -> None:
        """
        Hard-reset lateral position when robot touches a known wall.
        Eliminates accumulated drift at room boundaries.

        Args:
            side:          'left' | 'right' resets x, 'front' | 'rear' resets y
            wall_coord_cm: The known coordinate of the wall (cm, robot frame)
        """
        if side in ('left', 'right'):
            self._ekf.x[0, 0] = wall_coord_cm
            self._ekf.P[0, 0] = 1.0  # high confidence in x after snap
        elif side in ('front', 'rear'):
            self._ekf.x[1, 0] = wall_coord_cm
            self._ekf.P[1, 1] = 1.0

    def reset(self, x: float = 0.0, y: float = 0.0,
              theta: float = 0.0) -> None:
        """Full state reset (e.g. when returning to dock)."""
        self._ekf.x = np.array([[x], [y], [theta]], dtype=np.float64)
        self._ekf.P = np.diag([10.0, 10.0, 0.1])

    # ── Internal helpers ─────────────────────────────────────────────────────

    @staticmethod
    def _wrap_angle(angle) -> float:
        """Wrap angle to [-π, π]."""
        val = float(angle)
        return (val + math.pi) % (2 * math.pi) - math.pi
