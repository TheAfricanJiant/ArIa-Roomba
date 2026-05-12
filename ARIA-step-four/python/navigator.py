import math
import time
import logging

log = logging.getLogger(__name__)

# Module-level reference used by telemetry.get_obstacle_snapshot() to read the
# active goal without a circular import.
_cached_nav = None


class Navigator:
    def __init__(self):
        self.goal = None
        self.waypoints = []
        self.base_speed = 150
        self.k_p = 85.0
        self.arrival_dist_cm = 10.0
        self.slow_radius_cm = 70.0
        self.min_drive_pwm = 70
        self.max_turn_ratio = 0.65
        self.pivot_error_rad = math.radians(135)
        self.stall_timeout_s = 6.0
        self.spin_timeout_s = 4.0
        self._stall_since = 0.0
        self._last_abs_error = float("inf")
        self._spin_since = 0.0
        self._spin_escape_until = 0.0
        global _cached_nav
        _cached_nav = self

    def set_goal(self, x: float, y: float, speed: int):
        """Convenience method for a single waypoint."""
        self.set_path([(x, y)], speed)

    def set_path(self, points: list, speed: int):
        """Set a sequence of waypoints, adding intermediate points on long legs."""
        self.waypoints = self._prepare_path(points)
        self.base_speed = max(70, min(220, int(speed)))
        self._reset_progress()
        self._pop_next_goal()
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
        max_segment_cm = 35.0
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
        self._last_abs_error = float("inf")
        self._spin_since = 0.0
        self._spin_escape_until = 0.0

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

    def step(self, current_x: float, current_y: float, current_theta: float) -> tuple[int, int, bool]:
        if not self.goal:
            return 0, 0, False

        now = time.monotonic()
        dx, dy, distance = self._goal_error(current_x, current_y)

        if distance < self.arrival_dist_cm:
            if self.waypoints:
                log.info("Navigator reached intermediate waypoint.")
                self._pop_next_goal()
                dx, dy, distance = self._goal_error(current_x, current_y)
            else:
                log.info("Navigator arrived at final destination.")
                self.clear_goal()
                return 0, 0, True

        if distance < self.arrival_dist_cm * 2.2:
            if self._stall_since == 0.0:
                self._stall_since = now
            elif now - self._stall_since > self.stall_timeout_s:
                log.warning(
                    "Navigator: close-range stall timeout at %.1f cm; declaring arrived.",
                    distance,
                )
                self.clear_goal()
                return 0, 0, True
        else:
            self._stall_since = 0.0

        desired_theta = math.atan2(dy, dx)
        error = _wrap_angle(desired_theta - current_theta)
        abs_error = abs(error)

        if abs_error > math.radians(80):
            if self._spin_since == 0.0 or abs_error < self._last_abs_error - 0.03:
                self._spin_since = now
            elif now - self._spin_since > self.spin_timeout_s:
                self._spin_escape_until = now + 1.2
                self._spin_since = now
                log.warning("Navigator: heading not converging; using escape arc.")
        else:
            self._spin_since = 0.0
        self._last_abs_error = abs_error

        speed_scale = max(0.35, min(1.0, distance / self.slow_radius_cm))
        cruise = max(self.min_drive_pwm, int(self.base_speed * speed_scale))
        turn = int(error * self.k_p)
        max_turn = max(45, int(cruise * self.max_turn_ratio))
        turn = max(-max_turn, min(max_turn, turn))

        if now < self._spin_escape_until:
            forward = max(45, int(cruise * 0.35))
            left_speed = forward - turn
            right_speed = forward + turn
        elif abs_error > self.pivot_error_rad:
            pivot = max(65, min(120, int(self.base_speed * 0.55)))
            direction = 1 if error > 0 else -1
            left_speed = -direction * pivot
            right_speed = direction * pivot
        else:
            heading_scale = max(0.25, 1.0 - abs_error / self.pivot_error_rad)
            forward = max(45, int(cruise * heading_scale))
            left_speed = forward - turn
            right_speed = forward + turn

        return _clamp_pwm(left_speed), _clamp_pwm(right_speed), False

    def _goal_error(self, current_x: float, current_y: float) -> tuple[float, float, float]:
        gx, gy = self.goal
        dx = gx - current_x
        dy = gy - current_y
        return dx, dy, math.hypot(dx, dy)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clamp_pwm(value: float) -> int:
    return int(max(-255, min(255, value)))
