import math
import time
import logging

log = logging.getLogger(__name__)

# Module-level reference used by telemetry.get_obstacle_snapshot() to read the
# active goal without a circular import.
_cached_nav = None


class Navigator:
    """Forward-only pure-pursuit waypoint follower.

    The old follower pivoted in place for large heading errors. On the real
    chassis that could become a fast 360-degree spin when heading feedback was
    noisy or delayed. This controller never commands opposite wheel directions
    during waypoint following; it always crawls forward and bends toward a
    lookahead point.
    """

    def __init__(self):
        self.goal = None
        self.waypoints = []
        self.base_speed = 140
        self.arrival_dist_cm = 12.0
        self.lookahead_cm = 45.0
        self.slow_radius_cm = 85.0
        self.min_drive_pwm = 55
        self.max_drive_pwm = 190
        self.min_inner_pwm = 18
        self.stall_timeout_s = 8.0
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

        if final_distance < self._last_distance - 1.0:
            self._last_progress_ts = now
            self._stall_since = 0.0
        elif final_distance < self.arrival_dist_cm * 2.5:
            if self._stall_since == 0.0:
                self._stall_since = now
            elif now - self._stall_since > self.stall_timeout_s:
                log.warning("Navigator: close-range stall at %.1f cm; declaring arrived.", final_distance)
                self.clear_goal()
                self._debug = self._empty_debug("stalled-arrived")
                return 0, 0, True
        self._last_distance = final_distance

        target = self._lookahead_target(current_x, current_y)
        dx = target[0] - current_x
        dy = target[1] - current_y
        target_distance = max(1.0, math.hypot(dx, dy))
        heading_error = _wrap_angle(math.atan2(dy, dx) - current_theta)

        speed_scale = max(0.38, min(1.0, final_distance / self.slow_radius_cm))
        forward = max(self.min_drive_pwm, min(self.base_speed, int(self.base_speed * speed_scale)))

        # Pure-pursuit curvature proxy. Clamp hard so both wheels keep moving
        # forward instead of pivoting in opposite directions.
        turn_ratio = max(-0.82, min(0.82, 1.55 * math.sin(heading_error)))
        if abs(heading_error) > math.radians(115):
            forward = max(self.min_drive_pwm, int(forward * 0.55))
            turn_ratio = 0.82 if heading_error > 0 else -0.82

        left = forward * (1.0 - turn_ratio)
        right = forward * (1.0 + turn_ratio)
        left = max(self.min_inner_pwm, min(self.max_drive_pwm, left))
        right = max(self.min_inner_pwm, min(self.max_drive_pwm, right))

        self._debug = {
            "state": "tracking",
            "goal": {"x": round(self.goal[0], 1), "y": round(self.goal[1], 1)},
            "target": {"x": round(target[0], 1), "y": round(target[1], 1)},
            "distance_cm": round(final_distance, 1),
            "heading_error_deg": round(math.degrees(heading_error), 1),
            "left_pwm": int(left),
            "right_pwm": int(right),
            "queued": len(self.waypoints),
            "lookahead_cm": self.lookahead_cm,
            "mode": "forward-only pure pursuit",
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
            "mode": "forward-only pure pursuit",
        }


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi
