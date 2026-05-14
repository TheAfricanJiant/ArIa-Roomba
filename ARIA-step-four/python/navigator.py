"""
ARIA — Ultra-Simple Proportional Navigator
No state machine, just drive toward goal with proportional steering.
"""

import math
import time
import logging

log = logging.getLogger(__name__)

def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

class Navigator:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.goal = None
        self.waypoints = []
        
        self.base_speed = 180  # Increased from 120
        self.arrival_cm = 8.0
        self.min_pwm = 120     # Increased from 80
        
        self._waypoint_start_time = 0
        self._debug = {}

        global _cached_nav
        _cached_nav = self

    def set_speed(self, speed: int):
        self.base_speed = int(_clamp(speed, 120, 255))

    def set_goal(self, x: float, y: float, speed: int):
        self.set_path([(x, y)], speed)

    def set_path(self, points: list, speed: int):
        pts = []
        for p in points:
            try:
                pts.append((float(p["x"]), float(p["y"])) if isinstance(p, dict) else (float(p[0]), float(p[1])))
            except Exception:
                continue
        if not pts: return
        self.set_speed(speed)
        self.goal = pts[0]
        self.waypoints = pts[1:]
        self._waypoint_start_time = time.time()
        log.info(f"Navigator: goal set to {self.goal}")

    def clear_goal(self):
        self.goal = None
        self.waypoints = []
        self._waypoint_start_time = 0

    def sync_pose(self, x, y, theta, enc_l=None, enc_r=None):
        # ALWAYS update position and heading from EKF
        self.x = x
        self.y = y
        self.theta = theta

    def reset_pose(self, enc_l=0, enc_r=0):
        self.x = self.y = self.theta = 0.0

    def step(self):
        if not self.goal:
            return 0, 0, False

        # Check arrival
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        dist = math.hypot(dx, dy)

        if dist < self.arrival_cm:
            if self.waypoints:
                self.goal = self.waypoints.pop(0)
                self._waypoint_start_time = time.time()
                log.info(f"Waypoint reached, next: {self.goal}")
                dx = self.goal[0] - self.x
                dy = self.goal[1] - self.y
                dist = math.hypot(dx, dy)
            else:
                self.clear_goal()
                return 0, 0, True

        # Timeout check
        if time.time() - self._waypoint_start_time > 20.0:
            log.warning("Waypoint timeout!")
            if self.waypoints:
                self.goal = self.waypoints.pop(0)
                self._waypoint_start_time = time.time()
            else:
                self.clear_goal()
                return 0, 0, True

        # Calculate heading error
        desired_hdg = math.atan2(dy, dx)
        err = _wrap(desired_hdg - self.theta)
        err_deg = math.degrees(err)

        # Simple proportional controller
        # Forward speed scales with distance
        speed_scale = min(1.0, dist / 50.0)
        forward = int(self.base_speed * max(0.5, speed_scale))
        
        # Turn amount proportional to heading error
        turn_gain = 2.5
        turn = int(err_deg * turn_gain)
        turn = _clamp(turn, -forward, forward)

        # Differential drive
        left = forward - turn
        right = forward + turn

        # Apply minimum PWM
        if left > 0 and left < self.min_pwm: left = self.min_pwm
        if left < 0 and left > -self.min_pwm: left = -self.min_pwm
        if right > 0 and right < self.min_pwm: right = self.min_pwm
        if right < 0 and right > -self.min_pwm: right = -self.min_pwm

        # Clamp to valid range
        left = int(_clamp(left, -255, 255))
        right = int(_clamp(right, -255, 255))

        self._debug = {
            "state": "driving",
            "distance_cm": round(dist, 1),
            "heading_error_deg": round(err_deg, 1),
            "left_pwm": left,
            "right_pwm": right,
            "queued": len(self.waypoints),
            "pose_x": round(self.x, 1),
            "pose_y": round(self.y, 1),
            "pose_theta_deg": round(math.degrees(self.theta), 1)
        }

        return left, right, False

    def debug_status(self):
        return dict(self._debug)
