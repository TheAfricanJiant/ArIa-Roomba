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

        self.base_speed = 80
        self.arrival_cm = 15.0
        self.turn_speed = 60
        self.min_fwd_pwm = 42
        self.slow_cm = 40.0

        self.state = "idle"
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

        # Pure encoders. No gyro fusion to guarantee no sign/unit conflicts.
        d_theta = (d_r - d_l) / WHEEL_BASE_CM

        half = d_theta * 0.5
        self.theta = _wrap(self.theta + half)
        self.x += d_c * math.cos(self.theta)
        self.y += d_c * math.sin(self.theta)
        self.theta = _wrap(self.theta + half)

    def set_speed(self, speed: int):
        self.base_speed = int(_clamp(speed, self.min_fwd_pwm, 255))
        self.turn_speed = int(_clamp(speed * 0.7, 45, 120))

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
        log.info("Navigator path set")

    def clear_goal(self):
        self.goal = None
        self.waypoints = []
        self.state = "idle"

    def sync_pose(self, x, y, theta, enc_l=None, enc_r=None):
        self.x = x
        self.y = y
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

        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        dist = math.hypot(dx, dy)

        if dist < self.arrival_cm:
            if self.waypoints:
                self.goal = self.waypoints.pop(0)
                self.state = "turning"
                return self.step()
            self.clear_goal()
            return 0, 0, True

        desired_hdg = math.atan2(dy, dx)
        err = _wrap(desired_hdg - self.theta)
        err_deg = math.degrees(err)

        left = 0
        right = 0

        # Simple state machine matching your MicroPython logic
        if self.state == "turning":
            if abs(err_deg) < 10.0:
                self.state = "driving"
            else:
                # Spin in place
                turn_eff = self.turn_speed if abs(err_deg) > 20 else self.min_fwd_pwm
                if err > 0: # Target is to our left -> spin left (CCW)
                    left = -turn_eff
                    right = turn_eff
                else:       # Target is to our right -> spin right (CW)
                    left = turn_eff
                    right = -turn_eff

        if self.state == "driving":
            if abs(err_deg) > 25.0:
                # We drifted too far off heading, stop and spin
                self.state = "turning"
            else:
                # Drive forward with proportional heading correction
                dist_scale = _clamp(dist / self.slow_cm, 0.4, 1.0)
                fwd = int(self.base_speed * dist_scale)
                fwd = max(fwd, self.min_fwd_pwm)

                # Gentle proportional steering (P-controller)
                K = 1.5 
                turn = int(K * err_deg)
                turn = _clamp(turn, -fwd, fwd)

                left = int(_clamp(fwd - turn, -255, 255))
                right = int(_clamp(fwd + turn, -255, 255))

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
