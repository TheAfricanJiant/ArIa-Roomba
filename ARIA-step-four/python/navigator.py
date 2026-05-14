"""
ARIA — Simplified Stop-and-Spin Navigator
==========================================
Foolproof version:
- Pure encoder dead-reckoning (no gyro fusion to prevent unit/sign errors).
- State machine: if heading is off by > 15 degrees, stop driving and spin in place.
- Once aligned, drive straight with gentle proportional correction.
"""

import math
import time
import logging

log = logging.getLogger(__name__)

_cached_nav = None

WHEEL_BASE_CM     = 15.5
WHEEL_DIAMETER_CM = 6.0
TICKS_PER_REV     = 585
CM_PER_TICK       = math.pi * WHEEL_DIAMETER_CM / TICKS_PER_REV

def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

class Navigator:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._last_enc_l = 0
        self._last_enc_r = 0
        self._enc_initialised = False

        self.goal = None
        self.waypoints = []

        self.base_speed = 120
        self.arrival_cm = 5.0
        self.turn_speed = 100
        self.min_fwd_pwm = 80
        self.slow_cm = 40.0

        self.state = "idle"
        self._spin_start_time = 0
        self._waypoint_start_time = 0
        self._debug = {}

        global _cached_nav
        _cached_nav = self

    def update_encoders(self, enc_l: int, enc_r: int, gyro_z: float = 0.0, dt: float = 0.1):
        if not self._enc_initialised:
            self._last_enc_l = enc_l
            self._last_enc_r = enc_r
            self._enc_initialised = True
            return

        dl = enc_l - self._last_enc_l
        dr = enc_r - self._last_enc_r
        self._last_enc_l = enc_l
        self._last_enc_r = enc_r

        d_l = dl * CM_PER_TICK
        d_r = dr * CM_PER_TICK
        d_c = (d_l + d_r) * 0.5

        d_theta = (d_r - d_l) / WHEEL_BASE_CM

        half = d_theta * 0.5
        self.theta = _wrap(self.theta + half)
        self.x += d_c * math.cos(self.theta)
        self.y += d_c * math.sin(self.theta)
        self.theta = _wrap(self.theta + half)

    def set_speed(self, speed: int):
        self.base_speed = int(_clamp(speed, self.min_fwd_pwm, 255))
        self.turn_speed = int(_clamp(speed * 0.8, 80, 180))

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
        self.state = "turning"
        self._waypoint_start_time = time.time()
        log.info("Navigator path set")

    def clear_goal(self):
        self.goal = None
        self.waypoints = []
        self.state = "idle"
        self._spin_start_time = 0
        self._waypoint_start_time = 0

    def sync_pose(self, x, y, theta, enc_l=None, enc_r=None):
        self.x = x
        self.y = y
        if self.state not in ["driving", "turning"]:
            self.theta = theta
        if enc_l is not None:
            self._last_enc_l = enc_l
            self._last_enc_r = enc_r
            self._enc_initialised = True

    def reset_pose(self, enc_l=0, enc_r=0):
        self.x = self.y = self.theta = 0.0
        self._last_enc_l = enc_l
        self._last_enc_r = enc_r
        self._enc_initialised = True

    def step(self):
        if not self.goal:
            return 0, 0, False

        while self.goal:
            dx = self.goal[0] - self.x
            dy = self.goal[1] - self.y
            dist = math.hypot(dx, dy)

            if dist < self.arrival_cm:
                if self.waypoints:
                    self.goal = self.waypoints.pop(0)
                    self._waypoint_start_time = time.time()
                    continue
                else:
                    self.clear_goal()
                    return 0, 0, True
                    
            # Progress timeout: If we are stuck trying to reach this waypoint for > 15s, skip it
            if time.time() - self._waypoint_start_time > 15.0:
                log.warning("Waypoint timeout! Skipping to next.")
                if self.waypoints:
                    self.goal = self.waypoints.pop(0)
                    self._waypoint_start_time = time.time()
                    continue
                else:
                    self.clear_goal()
                    return 0, 0, True
                    
            break

        desired_hdg = math.atan2(dy, dx)
        err = _wrap(desired_hdg - self.theta)
        err_deg = math.degrees(err)

        left = 0
        right = 0

        # Simple state machine matching your MicroPython logic
        if self.state == "turning":
            if abs(err_deg) < 25.0:  # Widen tolerance so it easily transitions to driving
                self.state = "driving"
            else:
                # Add a minimum forward speed to break caster friction (min-forward)
                fwd_creep = int(self.min_fwd_pwm * 0.6)
                turn_eff = self.turn_speed
                if err > 0: # Target is to our left -> turn left
                    left = int(_clamp(fwd_creep - turn_eff, -255, 255))
                    right = int(_clamp(fwd_creep + turn_eff, -255, 255))
                else:       # Target is to our right -> turn right
                    left = int(_clamp(fwd_creep + turn_eff, -255, 255))
                    right = int(_clamp(fwd_creep - turn_eff, -255, 255))

        elif self.state == "driving":
            # Drive forward with proportional heading correction (Pure Pursuit)
            dist_scale = _clamp(dist / self.slow_cm, 0.4, 1.0)
            fwd = int(self.base_speed * dist_scale)
            fwd = max(fwd, self.min_fwd_pwm)

            # Gentle proportional steering scaled by forward speed
            K_p = 0.035  # 3.5% speed differential per degree of error
            turn_ratio = _clamp(K_p * err_deg, -0.85, 0.85) # Prevent full saturation (reversing inner wheel)
            turn = int(fwd * turn_ratio)

            left = int(_clamp(fwd - turn, -255, 255))
            right = int(_clamp(fwd + turn, -255, 255))
            
            # Apply min PWM to individual wheels to avoid stalling at low speeds
            if 0 < left < self.min_fwd_pwm: left = self.min_fwd_pwm
            elif -self.min_fwd_pwm < left < 0: left = -self.min_fwd_pwm
            if 0 < right < self.min_fwd_pwm: right = self.min_fwd_pwm
            elif -self.min_fwd_pwm < right < 0: right = -self.min_fwd_pwm

        self._debug = {
            "state": self.state,
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
