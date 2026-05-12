import math
import time
import logging

log = logging.getLogger(__name__)

# Module-level reference used by telemetry.get_obstacle_snapshot() to read the
# active goal without a circular import.
_cached_nav = None


class Navigator:
    def __init__(self):
        self.goal = None          # Current target (x_cm, y_cm)
        self.waypoints = []       # Queue of future targets
        self.base_speed  = 160
        self.k_p         = 100.0  # proportional gain for heading error
        self.arrival_dist_cm  = 15.0
        # Fix #5 (2026-05): stall timeout — declare arrival when the robot
        # is within 2× arrival distance but can't advance for stall_timeout_s.
        self.stall_timeout_s  = 8.0
        self._stall_since: float = 0.0
        self._last_dist: float   = float('inf')
        global _cached_nav
        _cached_nav = self

    def set_goal(self, x: float, y: float, speed: int):
        """Convenience method for a single waypoint."""
        self.set_path([(x, y)], speed)

    def set_path(self, points: list, speed: int):
        """Sets a sequence of waypoints."""
        self.waypoints     = list(points)
        self.base_speed    = speed
        self._stall_since  = 0.0
        self._last_dist    = float('inf')
        self._pop_next_goal()
        log.info(f"Navigator path set with "
                 f"{len(self.waypoints) + (1 if self.goal else 0)} points.")

    def _pop_next_goal(self):
        if self.waypoints:
            self.goal = self.waypoints.pop(0)
            self._stall_since = 0.0
            self._last_dist   = float('inf')
            log.info(f"Navigator heading to: {self.goal}")
        else:
            self.goal = None

    def clear_goal(self):
        self.goal          = None
        self.waypoints     = []
        self._stall_since  = 0.0

    def step(self, current_x: float, current_y: float,
             current_theta: float) -> tuple[int, int, bool]:
        """
        Calculates motor speeds to drive to the goal.
        Returns: (left_speed, right_speed, arrived_boolean)

        Fix #5 (2026-05): if the robot is within 2× arrival_dist_cm but cannot
        close the gap for stall_timeout_s seconds, it is declared arrived rather
        than looping forever (e.g. blocked just outside the safety stop zone).
        """
        if not self.goal:
            return 0, 0, False

        gx, gy = self.goal

        # ── Distance to goal ─────────────────────────────────────────────────
        dx = gx - current_x
        dy = gy - current_y
        distance = math.hypot(dx, dy)

        # ── Arrival check ────────────────────────────────────────────────────
        if distance < self.arrival_dist_cm:
            if self.waypoints:
                log.info("Navigator reached intermediate waypoint.")
                self._pop_next_goal()
                # Continue driving — fall through to steering below
            else:
                log.info("Navigator arrived at final destination.")
                self.clear_goal()
                return 0, 0, True

        # ── Stall detection (Fix #5) ─────────────────────────────────────────
        # Only watch for stalls when we're close enough that the obstacle safety
        # override (15 cm front US) might be blocking us.
        if distance < self.arrival_dist_cm * 2:
            now = time.monotonic()
            if self._stall_since == 0.0:
                self._stall_since = now
            elif now - self._stall_since > self.stall_timeout_s:
                log.warning(
                    f"Navigator: stall timeout ({self.stall_timeout_s:.0f}s) "
                    f"at {distance:.1f} cm from goal — declaring arrived."
                )
                self.clear_goal()
                return 0, 0, True
        else:
            # Moving freely — reset stall timer
            self._stall_since = 0.0

        # ── Heading error ─────────────────────────────────────────────────────
        desired_theta = math.atan2(dy, dx)
        error = desired_theta - current_theta
        error = (error + math.pi) % (2 * math.pi) - math.pi

        # ── Proportional steering controller ──────────────────────────────────
        turn     = int(error * self.k_p)
        max_turn = self.base_speed // 2
        turn     = max(-max_turn, min(max_turn, turn))

        # If facing away (>90° error), turn in place
        if abs(error) > math.pi / 2:
            left_speed  = -turn
            right_speed =  turn
        else:
            # Smooth arc: reduce forward speed as error increases
            forward     = int(self.base_speed * (1.0 - abs(error) / (math.pi / 2)))
            left_speed  = forward - turn
            right_speed = forward + turn

        # Clamp to valid 8-bit PWM range
        left_speed  = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))

        return int(left_speed), int(right_speed), False
