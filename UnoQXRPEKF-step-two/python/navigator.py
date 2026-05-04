import math
import logging

log = logging.getLogger(__name__)

class Navigator:
    def __init__(self):
        self.goal = None  # Current target (x_cm, y_cm)
        self.waypoints = [] # Queue of future targets
        self.base_speed = 160
        self.k_p = 100.0  # proportional gain for heading error
        self.arrival_dist_cm = 15.0

    def set_goal(self, x: float, y: float, speed: int):
        """Convenience method for a single waypoint"""
        self.set_path([(x, y)], speed)

    def set_path(self, points: list, speed: int):
        """Sets a sequence of waypoints"""
        self.waypoints = points
        self.base_speed = speed
        self._pop_next_goal()
        log.info(f"Navigator path set with {len(self.waypoints)+1 if self.goal else 0} points.")

    def _pop_next_goal(self):
        if self.waypoints:
            self.goal = self.waypoints.pop(0)
            log.info(f"Navigator heading to: {self.goal}")
        else:
            self.goal = None

    def clear_goal(self):
        self.goal = None
        self.waypoints = []

    def step(self, current_x: float, current_y: float, current_theta: float) -> tuple[int, int, bool]:
        """
        Calculates motor speeds to drive to the goal.
        Returns: (left_speed, right_speed, arrived_boolean)
        """
        if not self.goal:
            return 0, 0, False

        gx, gy = self.goal
        
        # Calculate distance
        dx = gx - current_x
        dy = gy - current_y
        distance = math.hypot(dx, dy)

        if distance < self.arrival_dist_cm:
            if self.waypoints:
                log.info("Navigator reached intermediate waypoint.")
                self._pop_next_goal()
                # Continue driving
            else:
                log.info("Navigator arrived at final destination.")
                self.clear_goal()
                return 0, 0, True

        # Calculate desired heading
        desired_theta = math.atan2(dy, dx)
        
        # Calculate heading error (shortest angular distance)
        error = desired_theta - current_theta
        error = (error + math.pi) % (2 * math.pi) - math.pi

        # Proportional steering controller
        turn = int(error * self.k_p)
        max_turn = self.base_speed // 2
        turn = max(-max_turn, min(max_turn, turn))

        # If facing away (>90 deg error), turn in place
        if abs(error) > math.pi / 2:
            left_speed = -turn
            right_speed = turn
        else:
            # Smooth arc: reduce forward speed slightly as error increases
            forward = int(self.base_speed * (1.0 - abs(error)/(math.pi/2)))
            left_speed = forward - turn
            right_speed = forward + turn

        # Clamp speeds to valid 8-bit PWM range
        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))

        return int(left_speed), int(right_speed), False
