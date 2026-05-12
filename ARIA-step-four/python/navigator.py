import math
import time
import logging

log = logging.getLogger(__name__)

# Module-level reference used by telemetry.get_obstacle_snapshot() to read the
# active goal without a circular import.
_cached_nav = None


class Navigator:
    """EKF-feedback waypoint follower for a differential-drive robot."""

    def __init__(self):
        self.goal = None
        self.waypoints = []
        self.base_speed = 140
        self.arrival_dist_cm = 18.0
        self.lookahead_cm = 32.0
        self.slow_radius_cm = 90.0
        self.min_drive_pwm = 48
        self.max_drive_pwm = 190
        self.min_turn_pwm = 45
        self.max_turn_pwm = 135
        self.align_error_rad = math.radians(55)
        self.close_radius_cm = 38.0
        self.stall_timeout_s = 5.0
        self._stall_since = 0.0
        self._last_distance = float("inf")
        self._last_progress_ts = time.monotonic()
        self._debug = self._empty_debug("idle")
        global _cached_nav
        _cached_nav = self

    def set_goal(self, x: float, y: float, speed: int):
        self.set_path([(x, y)], speed)

    def set_path(self, points: list, speed: int):
        self.waypoints = self._prepare_path(points)
        self.base_speed = max(70, min(self.max_drive_pwm, int(speed)))
        self._reset_progress()
        self._pop_next_goal()
        self._debug = self._empty_debug("planning")
        log.info(
            "Navigator path set with %d points.",
            len(self.waypoints) + (1 if self.goal else 0),
        )

    def _prepare_path(self, points: list) -> list[tuple[float, float]]:
        normalized: list[tuple[float, float]] = []
        for p in points:
            try:
                if isinstance(p, dict):
                    x, y = float(p["x"]), float(p["y"])
                else:
                    x, y = float(p[0]), float(p[1])
            except Exception:
                continue
            if not normalized or math.hypot(x - normalized[-1][0], y - normalized[-1][1]) > 2.0:
                normalized.append((x, y))

        if len(normalized) < 2:
            return normalized

        dense = [normalized[0]]
        max_segment_cm = 25.0
        for x, y in normalized[1:]:
            px, py = dense[-1]
            dist = math.hypot(x - px, y - py)
            steps = max(1, int(math.ceil(dist / max_segment_cm)))
            for i in range(1, steps + 1):
                t = i / steps
                dense.append((px + (x - px) * t, py + (y - py) * t))
        return dense

    def _reset_progress(self):
        self._stall_since = 0.0
        self._last_distance = float("inf")
        self._last_progress_ts = time.monotonic()

    def _pop_next_goal(self):
        if self.waypoints:
            self.goal = self.waypoints.pop(0)
            self._reset_progress()
            log.info("Navigator heading to: %s", self.goal)
        else:
            self.goal = None

    def clear_goal(self):
        self.goal = None
        self.waypoints = []
        self._reset_progress()
        self._debug = self._empty_debug("idle")

    def step(self, current_x: float, current_y: float, current_theta: float) -> tuple[int, int, bool]:
        if not self.goal:
            self._debug = self._empty_debug("idle")
            return 0, 0, False

        now = time.monotonic()
        final_distance = self._distance_to_goal(current_x, current_y, self.goal)

        while self.goal and final_distance < self.arrival_dist_cm:
            if not self.waypoints:
                log.info("Navigator arrived at final destination.")
                self.clear_goal()
                self._debug = self._empty_debug("arrived")
                return 0, 0, True
            log.info("Navigator reached intermediate waypoint.")
            self._pop_next_goal()
            final_distance = self._distance_to_goal(current_x, current_y, self.goal)

        if final_distance < self._last_distance - 0.6:
            self._last_progress_ts = now
            self._stall_since = 0.0
        elif final_distance < self.close_radius_cm:
            if self._stall_since == 0.0:
                self._stall_since = now
            elif now - self._stall_since > self.stall_timeout_s:
                log.warning("Navigator: close-range stall at %.1f cm; accepting waypoint.", final_distance)
                self.clear_goal()
                self._debug = self._empty_debug("stalled-accepted")
                return 0, 0, True
        self._last_distance = final_distance

        target = self._lookahead_target(current_x, current_y)
        dx = target[0] - current_x
        dy = target[1] - current_y
        heading_error = _wrap_angle(math.atan2(dy, dx) - current_theta)

        abs_error = abs(heading_error)
        max_turn = min(self.max_turn_pwm, max(self.min_turn_pwm, int(self.base_speed * 0.8)))
        turn = _clamp(heading_error * 95.0, -max_turn, max_turn)
        if 0 < abs(turn) < self.min_turn_pwm:
            turn = math.copysign(self.min_turn_pwm, turn)

        if abs_error > self.align_error_rad and final_distance > self.close_radius_cm:
            forward = 0
            left = -turn
            right = turn
            mode = "aligning"
        else:
            speed_scale = max(0.30, min(1.0, final_distance / self.slow_radius_cm))
            heading_scale = max(0.35, math.cos(min(abs_error, math.radians(70))))
            forward = int(self.base_speed * speed_scale * heading_scale)
            forward = max(self.min_drive_pwm, min(self.base_speed, forward))

            if final_distance < self.close_radius_cm:
                forward = min(forward, 70)
                turn = _clamp(turn, -70, 70)

            left = forward - turn
            right = forward + turn
            mode = "tracking"

        left = _clamp(left, -self.max_drive_pwm, self.max_drive_pwm)
        right = _clamp(right, -self.max_drive_pwm, self.max_drive_pwm)

        self._debug = {
            "state": mode,
            "goal": {"x": round(self.goal[0], 1), "y": round(self.goal[1], 1)},
            "target": {"x": round(target[0], 1), "y": round(target[1], 1)},
            "distance_cm": round(final_distance, 1),
            "heading_error_deg": round(math.degrees(heading_error), 1),
            "left_pwm": int(left),
            "right_pwm": int(right),
            "queued": len(self.waypoints),
            "lookahead_cm": self.lookahead_cm,
            "mode": "ekf go-to-goal",
        }
        return int(left), int(right), False

    def _lookahead_target(self, current_x: float, current_y: float) -> tuple[float, float]:
        candidates = [self.goal] + self.waypoints
        target = self.goal
        for point in candidates:
            target = point
            if self._distance_to_goal(current_x, current_y, point) >= self.lookahead_cm:
                break
        return target

    def _distance_to_goal(self, current_x: float, current_y: float, goal: tuple[float, float]) -> float:
        return math.hypot(goal[0] - current_x, goal[1] - current_y)

    def debug_status(self) -> dict:
        return dict(self._debug)

    def _empty_debug(self, state: str) -> dict:
        return {
            "state": state,
            "goal": None,
            "target": None,
            "distance_cm": 0.0,
            "heading_error_deg": 0.0,
            "left_pwm": 0,
            "right_pwm": 0,
            "queued": 0,
            "lookahead_cm": self.lookahead_cm,
            "mode": "ekf go-to-goal",
        }


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
