"""
ARIA — Bridge Stub / Simulator
Hardware-free simulation of the App Lab Bridge (STM32 ↔ Linux serial link).

When you have real hardware, swap the import in aria_main.py:
    from aria.bridge import BridgeStub as Bridge       # simulator
    from aria.bridge_hw import BridgeHW as Bridge      # real hardware

The simulator drives the robot in a clockwise lawnmower pattern
so you can validate the EKF and occupancy grid offline.
"""

from __future__ import annotations
import math
import time
import random
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class SensorPacket:
    """One uplink packet from STM32 (every 50 ms)."""
    us_front:       float = 200.0   # cm
    us_front_left:  float = 200.0
    us_front_right: float = 200.0
    us_left:        float = 200.0
    us_right:       float = 200.0
    us_rear:        float = 200.0
    enc_left:       int   = 0       # cumulative ticks
    enc_right:      int   = 0
    gyro_z:         float = 0.0     # rad/s
    timestamp_ms:   int   = 0

    @property
    def ultrasonics(self) -> dict[str, float]:
        return {
            'front':       self.us_front,
            'front_left':  self.us_front_left,
            'front_right': self.us_front_right,
            'left':        self.us_left,
            'right':       self.us_right,
            'rear':        self.us_rear,
        }


class BridgeStub:
    """
    Simulates a 4 m × 4 m room with a centre obstacle.
    Uses proper differential drive kinematics (v, w) so the EKF 
    can correctly reconstruct the path.
    """
    ROOM_HALF = 200.0   # cm — room is ±200 cm from origin
    WALL_THICKNESS = 5.0

    def __init__(self) -> None:
        # Simulated robot true state
        self._x: float   = -150.0
        self._y: float   = -150.0    # start near bottom-left
        self._theta: float = 0.0     # facing right (+X)

        # Boustrophedon waypoints
        self._waypoints = []
        x_dir = 150.0
        for y in range(-150, 160, 30):
            self._waypoints.append((x_dir, float(y)))
            self._waypoints.append((-x_dir, float(y)))
            x_dir = -x_dir
            
        self._wp_idx = 0

        self._last_enc_l: int = 0
        self._last_enc_r: int = 0
        self._t0 = time.monotonic()

        # Obstacle box in centre
        self._obs = (-40.0, -40.0, 40.0, 40.0)

        # Kinematic state
        self._v = 0.0
        self._w = 0.0

    # ── Public interface mirrors real Bridge ─────────────────────────────────

    def get_sensors(self) -> SensorPacket:
        """Return simulated sensor packet. Non-blocking."""
        dt = 0.05   # simulate 50 ms tick
        self._sim_step(dt)
        return self._build_packet(dt)

    def set_motors(self, left_pwm: int, right_pwm: int) -> None:
        pass

    def set_vacuum(self, pwm: int) -> None:
        pass

    def set_brush(self, pwm: int) -> None:
        pass

    # ── Simulation internals ─────────────────────────────────────────────────

    def _sim_step(self, dt: float) -> None:
        """Advance simulated robot using waypoint following."""
        if self._wp_idx >= len(self._waypoints):
            self._v = 0.0
            self._w = 0.0
            return

        tx, ty = self._waypoints[self._wp_idx]
        dx = tx - self._x
        dy = ty - self._y
        dist = math.hypot(dx, dy)

        if dist < 5.0:
            self._wp_idx += 1
            return

        target_heading = math.atan2(dy, dx)
        diff = (target_heading - self._theta + math.pi) % (2 * math.pi) - math.pi

        # Simple P-controller steering
        if abs(diff) > 0.1:
            self._v = 0.0
            self._w = math.copysign(2.0, diff)  # 2 rad/s turn
            if abs(diff) < abs(self._w * dt):
                self._theta = target_heading
                self._w = 0.0
        else:
            self._v = 20.0  # 20 cm/s straight
            self._w = 0.0

        # Update true pose
        self._x += self._v * math.cos(self._theta) * dt
        self._y += self._v * math.sin(self._theta) * dt
        self._theta += self._w * dt
        self._theta = (self._theta + math.pi) % (2 * math.pi) - math.pi

    def _ray_to_wall(self, angle_abs: float) -> float:
        """Cast ray from robot position and return distance to nearest wall/obstacle."""
        cos_a = math.cos(angle_abs)
        sin_a = math.sin(angle_abs)
        min_d = 400.0

        # Room walls
        for wall_x in (-self.ROOM_HALF, self.ROOM_HALF):
            if abs(cos_a) > 1e-6:
                t = (wall_x - self._x) / cos_a
                if t > 0:
                    hit_y = self._y + sin_a * t
                    if -self.ROOM_HALF <= hit_y <= self.ROOM_HALF:
                        min_d = min(min_d, t)

        for wall_y in (-self.ROOM_HALF, self.ROOM_HALF):
            if abs(sin_a) > 1e-6:
                t = (wall_y - self._y) / sin_a
                if t > 0:
                    hit_x = self._x + cos_a * t
                    if -self.ROOM_HALF <= hit_x <= self.ROOM_HALF:
                        min_d = min(min_d, t)

        # Centre obstacle
        xmin, ymin, xmax, ymax = self._obs
        for ox in (xmin, xmax):
            if abs(cos_a) > 1e-6:
                t = (ox - self._x) / cos_a
                if t > 0:
                    hy = self._y + sin_a * t
                    if ymin <= hy <= ymax:
                        min_d = min(min_d, t)
        for oy in (ymin, ymax):
            if abs(sin_a) > 1e-6:
                t = (oy - self._y) / sin_a
                if t > 0:
                    hx = self._x + cos_a * t
                    if xmin <= hx <= xmax:
                        min_d = min(min_d, t)

        return max(0.0, min_d + random.gauss(0, 0.5))

    def _build_packet(self, dt: float) -> SensorPacket:
        from .config import US_ANGLES, CM_PER_TICK, WHEEL_BASE_CM
        
        # Differential drive inverse kinematics
        vr = self._v + self._w * WHEEL_BASE_CM / 2.0
        vl = self._v - self._w * WHEEL_BASE_CM / 2.0
        
        enc_ticks_r = int((vr * dt) / CM_PER_TICK)
        enc_ticks_l = int((vl * dt) / CM_PER_TICK)
        
        enc_l = self._last_enc_l + enc_ticks_l + random.randint(-1, 1)
        enc_r = self._last_enc_r + enc_ticks_r + random.randint(-1, 1)
        self._last_enc_l = enc_l
        self._last_enc_r = enc_r

        # Gyro correctly reflects angular velocity
        gyro_z = self._w + random.gauss(0, 0.01)

        pkt = SensorPacket(
            enc_left  = enc_l,
            enc_right = enc_r,
            gyro_z    = gyro_z,
            timestamp_ms = int((time.monotonic() - self._t0) * 1000),
        )
        for name, rel_angle in US_ANGLES.items():
            dist = self._ray_to_wall(self._theta + rel_angle)
            setattr(pkt, f'us_{name}', dist)

        return pkt
