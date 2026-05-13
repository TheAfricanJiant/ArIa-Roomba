"""
ARIA — Encoder-Only Waypoint Follower
======================================
Strategy:
  - Pure encoder dead-reckoning for pose (no gyro — gz≈0 on this platform).
  - Raw M,left,right commands (bypasses XRW PI velocity loop entirely).
  - Proportional heading correction: one wheel slows, the other speeds up.
  - NEVER stops between waypoints. Continuous motion from start to finish.
  - No odom validation gates. If encoders are ticking, we navigate.

Call update_encoders(enc_l, enc_r) each time new telemetry arrives.
Call step() at ~50 Hz to get (left_pwm, right_pwm, arrived).
"""

import math
import time
import logging

log = logging.getLogger(__name__)

# Module-level cache used by telemetry.get_obstacle_snapshot()
_cached_nav = None

# ── Robot geometry (must match telemetry.py) ──────────────────────────────────
WHEEL_BASE_CM     = 15.5
WHEEL_DIAMETER_CM = 6.0
TICKS_PER_REV     = 585
CM_PER_TICK       = math.pi * WHEEL_DIAMETER_CM / TICKS_PER_REV


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Navigator:
    """
    Encoder-only proportional waypoint follower.

    Pose is tracked internally from encoder deltas — independent of the EKF.
    Motor commands are raw M, PWM (fire-and-forget, no firmware timeout issues).
    """

    def __init__(self):
        # ── Internal pose ─────────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self._last_enc_l      = 0
        self._last_enc_r      = 0
        self._enc_initialised = False

        # ── Path state ────────────────────────────────────────────────────────
        self.goal       = None   # (x_cm, y_cm) current target
        self.waypoints  = []     # remaining waypoints

        # ── Tuning ────────────────────────────────────────────────────────────
        self.base_speed  = 80    # base forward PWM
        self.arrival_cm  = 18.0  # declare arrived within this distance
        self.accept_cm   = 28.0  # forced-accept when stalled this close
        self.slow_cm     = 80.0  # start slowing down this far from goal
        self.min_fwd_pwm = 38    # motor deadband floor
        self.K_turn      = 0.75  # heading gain  (0 = no correction, 1 = max)
        self._stall_sec  = 6.0   # force-accept stall timeout

        # ── Stall tracking ────────────────────────────────────────────────────
        self._best_dist   = float("inf")
        self._stall_since = time.monotonic()

        # ── Debug snapshot ────────────────────────────────────────────────────
        self._debug = self._make_debug("idle", 0, 0, 0.0, 0.0)

        global _cached_nav
        _cached_nav = self

    # ── Encoder integration ───────────────────────────────────────────────────
    def update_encoders(self, enc_l: int, enc_r: int):
        """Integrate encoder ticks into (x, y, theta). Call on every telemetry tick."""
        if not self._enc_initialised:
            self._last_enc_l = enc_l
            self._last_enc_r = enc_r
            self._enc_initialised = True
            return

        dl = enc_l - self._last_enc_l
        dr = enc_r - self._last_enc_r
        self._last_enc_l = enc_l
        self._last_enc_r = enc_r

        d_l     = dl * CM_PER_TICK
        d_r     = dr * CM_PER_TICK
        d_c     = (d_l + d_r) * 0.5
        # Standard differential: left motor on left side.
        # Left turn (CCW) → enc_r increases, enc_l decreases → d_r > d_l → d_theta > 0
        d_theta = (d_r - d_l) / WHEEL_BASE_CM

        half        = d_theta * 0.5
        self.theta  = _wrap(self.theta + half)
        self.x     += d_c * math.cos(self.theta)
        self.y     += d_c * math.sin(self.theta)
        self.theta  = _wrap(self.theta + half)

    # ── Path management ───────────────────────────────────────────────────────
    def set_speed(self, speed: int):
        self.base_speed = int(_clamp(speed, self.min_fwd_pwm, 255))

    def set_goal(self, x: float, y: float, speed: int):
        self.set_path([(x, y)], speed)

    def set_path(self, points: list, speed: int):
        pts = []
        for p in points:
            try:
                pts.append((float(p["x"]), float(p["y"])) if isinstance(p, dict)
                           else (float(p[0]), float(p[1])))
            except Exception:
                continue
        if not pts:
            return
        self.set_speed(speed)
        self.goal      = pts[0]
        self.waypoints = pts[1:]
        self._reset_stall()
        log.info("Navigator: path set — %d point(s), speed=%d", len(pts), self.base_speed)

    def clear_goal(self):
        self.goal      = None
        self.waypoints = []
        self._reset_stall()
        self._debug = self._make_debug("idle", 0, 0, 0.0, 0.0)

    def sync_pose(self, x: float, y: float, theta: float,
                  enc_l: int = None, enc_r: int = None):
        """Align navigator pose with the EKF estimate at goal-set time.
        Call this just before set_goal/set_path so dead-reckoning starts
        from the correct known position instead of accumulated (0,0,0)."""
        self.x     = x
        self.y     = y
        self.theta = theta
        if enc_l is not None:
            self._last_enc_l      = enc_l
            self._last_enc_r      = enc_r
            self._enc_initialised = True
        log.info("Navigator pose synced: x=%.1f y=%.1f θ=%.1f°",
                 x, y, math.degrees(theta))

    def reset_pose(self, enc_l: int = 0, enc_r: int = 0):
        """Zero navigator pose and resync encoder baseline (call on home-reset)."""
        self.x = self.y = self.theta = 0.0
        self._last_enc_l      = enc_l
        self._last_enc_r      = enc_r
        self._enc_initialised = True
        log.info("Navigator pose reset to origin.")

    # ── Control step (50 Hz) ──────────────────────────────────────────────────
    def step(self, _x=None, _y=None, _theta=None) -> tuple:
        """
        Returns (left_pwm, right_pwm, arrived).

        _x/_y/_theta are accepted but ignored — pose comes from update_encoders().
        Motors run continuously; the only time (0, 0, False) is returned is when
        there is no active goal.
        """
        if not self.goal:
            return 0, 0, False

        now = time.monotonic()
        dx   = self.goal[0] - self.x
        dy   = self.goal[1] - self.y
        dist = math.hypot(dx, dy)

        # ── Arrival ──────────────────────────────────────────────────────────
        if dist < self.arrival_cm:
            log.info("Navigator: waypoint reached (%.1f cm). Queued: %d",
                     dist, len(self.waypoints))
            if self.waypoints:
                self.goal = self.waypoints.pop(0)
                self._reset_stall()
                return self.step()          # tail-recurse to next waypoint
            self.clear_goal()
            return 0, 0, True               # final destination reached

        # ── Stall / force-accept ──────────────────────────────────────────────
        if dist < self._best_dist:
            self._best_dist   = dist
            self._stall_since = now
        elif now - self._stall_since > self._stall_sec and dist < self.accept_cm:
            log.warning("Navigator: stall-accept at %.1f cm", dist)
            if self.waypoints:
                self.goal = self.waypoints.pop(0)
                self._reset_stall()
                return self.step()
            self.clear_goal()
            return 0, 0, True

        # ── Heading error ─────────────────────────────────────────────────────
        err = _wrap(math.atan2(dy, dx) - self.theta)   # + = need to turn left

        # ── Forward speed: ramp down near goal AND when far off-heading ────────
        heading_scale = _clamp(1.0 - abs(err) / math.pi, 0.0, 1.0)
        dist_scale    = _clamp(dist / self.slow_cm, 0.35, 1.0)
        fwd = int(self.base_speed * dist_scale * heading_scale)
        fwd = max(fwd, self.min_fwd_pwm) if heading_scale > 0.2 else 0

        # ── Differential turn (standard convention) ────────────────────────────
        # err > 0 (goal to the left) → turn > 0 → right > left → robot turns left
        turn = int(self.K_turn * self.base_speed * err / math.pi)
        turn = int(_clamp(turn, -self.base_speed, self.base_speed))

        left  = int(_clamp(fwd - turn, -255, 255))   # left slower → curves left
        right = int(_clamp(fwd + turn, -255, 255))   # right faster → curves left


        self._debug = self._make_debug("driving", left, right, dist, err)
        return left, right, False

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _reset_stall(self):
        self._best_dist   = float("inf")
        self._stall_since = time.monotonic()

    def debug_status(self) -> dict:
        return dict(self._debug)

    def _make_debug(self, state, left, right, dist, err) -> dict:
        return {
            "state":             state,
            "goal":              {"x": round(self.goal[0], 1), "y": round(self.goal[1], 1)} if self.goal else None,
            "target":            {"x": round(self.goal[0], 1), "y": round(self.goal[1], 1)} if self.goal else None,
            "distance_cm":       round(dist, 1),
            "heading_error_deg": round(math.degrees(err), 1),
            "left_pwm":          left,
            "right_pwm":         right,
            "forward_pwm":       left,   # approx
            "turn_pwm":          0,
            "turn_polarity":     1,
            "queued":            len(self.waypoints),
            "mode":              "encoder-only proportional M,cmd",
            "pose_x":            round(self.x, 1),
            "pose_y":            round(self.y, 1),
            "pose_theta_deg":    round(math.degrees(self.theta), 1),
        }
