"""
ARIA — Phase 4: Navigation
Boustrophedon coverage planner + potential field steering + 6-state machine.

Classes:
    BoustrophedonPlanner   — lawnmower stripe planner
    PotentialFieldSteering — real-time obstacle-aware motor commands
    CleaningStateMachine   — 6-state autonomous cleaning logic

All classes are thread-safe (called from _navigation_loop under _state_lock).
"""

from __future__ import annotations
import math
import time
import logging
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .config import (
    CELL_SIZE_CM, GRID_WIDTH_CM, GRID_HEIGHT_CM,
    US_ANGLES, US_OBSTACLE_STOP_CM,
)
from .occupancy_grid import OccupancyGrid, WALL, OBSTACLE, FREE, CLEANED, UNKNOWN
from .astar import plan_path   # Phase 5 — A* path planner

log = logging.getLogger('ARIA.nav')


# ════════════════════════════════════════════════════════════════════════════
# HELPERS
# ════════════════════════════════════════════════════════════════════════════

def _wrap(angle: float) -> float:
    """Wrap angle to [-π, π]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clamp(v: float, lo: float, hi: float) -> int:
    return int(max(lo, min(hi, v)))


# ════════════════════════════════════════════════════════════════════════════
# 1. BOUSTROPHEDON PLANNER
# ════════════════════════════════════════════════════════════════════════════

class BoustrophedonPlanner:
    """
    Lawnmower-style coverage planner.

    Divides the room into parallel stripes (each one robot-width wide).
    Alternates direction each stripe to form a continuous path.
    Skips cells blocked by WALL or OBSTACLE in the occupancy grid.

    Usage:
        planner = BoustrophedonPlanner(grid)
        planner.start(robot_x, robot_y)          # call once to init

        while not planner.is_complete():
            wx, wy = planner.next_waypoint(x, y)  # call every step
            # drive toward (wx, wy) …
    """

    WAYPOINT_REACHED_CM = 20.0   # distance threshold to consider waypoint reached

    def __init__(self, grid: OccupancyGrid,
                 robot_width_cm: float = 30.0,
                 coverage_target: float = 95.0) -> None:
        self._grid            = grid
        self._stripe_width    = robot_width_cm
        self._coverage_target = coverage_target

        # Stripe state (set on start())
        self._stripe_index: int   = 0
        self._direction:    int   = 1      # +1 = +X, -1 = -X
        self._origin_x:     float = 0.0   # starting x (left wall estimate)
        self._origin_y:     float = 0.0   # starting y (bottom of first stripe)
        self._current_wp:   Optional[tuple[float, float]] = None
        self._started:      bool  = False

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self, robot_x: float, robot_y: float) -> None:
        """Initialise planner from current robot position."""
        self._origin_x     = robot_x
        self._origin_y     = robot_y
        self._stripe_index = 0
        self._direction    = 1
        self._current_wp   = None
        self._started      = True
        log.info(f"Boustrophedon planner started at ({robot_x:.0f}, {robot_y:.0f}) cm")

    def next_waypoint(self, robot_x: float, robot_y: float) -> tuple[float, float]:
        """
        Return the current waypoint (x_cm, y_cm).
        Advances to the next waypoint when the robot is close enough.
        """
        if not self._started:
            self.start(robot_x, robot_y)

        # If we haven't generated a waypoint yet, or the robot has reached it
        if self._current_wp is None or self._reached(robot_x, robot_y):
            self._current_wp = self._generate_next(robot_x, robot_y)

        return self._current_wp

    def is_complete(self) -> bool:
        """True when coverage exceeds the target threshold."""
        return self._grid.coverage_percent() >= self._coverage_target

    def reset(self) -> None:
        self._started = False
        self._current_wp = None

    # ── Internals ──────────────────────────────────────────────────────────────

    def _reached(self, x: float, y: float) -> bool:
        if self._current_wp is None:
            return True
        wx, wy = self._current_wp
        return math.hypot(x - wx, y - wy) < self.WAYPOINT_REACHED_CM

    def _generate_next(self, robot_x: float, robot_y: float) -> tuple[float, float]:
        """
        Compute the next stripe waypoint.
        Priority:
          1. Continue along current stripe to its far wall
          2. When stripe is done, shift one robot-width and reverse
          3. Skip stripes where all cells are blocked
        """
        stripe_y = self._origin_y + self._stripe_index * self._stripe_width

        # Check if current stripe target is reachable
        target_x = self._find_stripe_end(stripe_y, self._direction)

        # If we're already past the end of this stripe, advance
        along_stripe = (robot_x - self._origin_x) * self._direction
        end_along    = (target_x - self._origin_x) * self._direction

        if along_stripe >= end_along - self.WAYPOINT_REACHED_CM:
            # Stripe done — shift to next stripe
            self._stripe_index += 1
            self._direction    *= -1
            stripe_y = self._origin_y + self._stripe_index * self._stripe_width

            # Lateral shift waypoint (keep same X, move Y)
            lateral_wp = (robot_x, stripe_y)
            log.debug(f"Stripe {self._stripe_index} start → ({robot_x:.0f}, {stripe_y:.0f})")
            return lateral_wp

        # Drive to end of current stripe
        return (target_x, stripe_y)

    def _find_stripe_end(self, stripe_y: float, direction: int) -> float:
        """
        Scan along the stripe in `direction` until a WALL/OBSTACLE is found.
        Returns the x coordinate of the last FREE/CLEANED cell.
        """
        last_free_x = self._origin_x
        step        = CELL_SIZE_CM * direction
        max_steps   = int(GRID_WIDTH_CM / CELL_SIZE_CM)

        x = self._origin_x
        for _ in range(max_steps):
            x += step
            state = self._grid.cell_state(x, stripe_y)
            if state in (int(WALL), int(OBSTACLE)):
                break
            last_free_x = x

        return last_free_x


# ════════════════════════════════════════════════════════════════════════════
# 2. POTENTIAL FIELD STEERING
# ════════════════════════════════════════════════════════════════════════════

class PotentialFieldSteering:
    """
    Computes (left_pwm, right_pwm) to drive toward a waypoint while
    smoothly avoiding nearby ultrasonic obstacles.

    Attractive force:  pulls robot toward waypoint (proportional to distance)
    Repulsive forces:  push away from each obstacle closer than threshold

    Output is differential-drive PWM: both channels in [-max_pwm, +max_pwm].
    """

    def __init__(self,
                 max_pwm:             int   = 200,
                 base_speed:          int   = 160,
                 repulse_thresh_cm:   float = 50.0,
                 k_repulse:           float = 3000.0,
                 waypoint_close_cm:   float = 15.0) -> None:
        self._max_pwm       = max_pwm
        self._base_speed    = base_speed
        self._rep_thresh    = repulse_thresh_cm
        self._k_rep         = k_repulse
        self._wp_close      = waypoint_close_cm

    def compute(self,
                robot_x:     float,
                robot_y:     float,
                robot_theta: float,
                target_x:    float,
                target_y:    float,
                ultrasonics: dict[str, float]) -> tuple[int, int]:
        """
        Returns (left_pwm, right_pwm).
        Returns (0, 0) when within waypoint_close_cm of target.
        """
        dx   = target_x - robot_x
        dy   = target_y - robot_y
        dist = math.hypot(dx, dy)

        if dist < self._wp_close:
            return 0, 0   # reached waypoint

        # ── Attractive force (unit vector toward target) ──────────────────
        attr_x = dx / dist
        attr_y = dy / dist

        # ── Repulsive forces from ultrasonics ─────────────────────────────
        rep_x, rep_y = 0.0, 0.0
        for name, d in ultrasonics.items():
            if 0 < d < self._rep_thresh:
                abs_angle = robot_theta + US_ANGLES.get(name, 0.0)
                # Repulsion strength grows as 1/d² near the threshold
                strength  = self._k_rep * (1.0 / d - 1.0 / self._rep_thresh) / (d * d)
                rep_x    -= strength * math.cos(abs_angle)
                rep_y    -= strength * math.sin(abs_angle)

        # ── Combine forces ────────────────────────────────────────────────
        fx = attr_x + rep_x
        fy = attr_y + rep_y

        target_angle = math.atan2(fy, fx)
        angle_error  = _wrap(target_angle - robot_theta)

        # ── Differential drive conversion ─────────────────────────────────
        # Speed proportional to distance (slow down near waypoint)
        speed_scale = min(1.0, dist / 80.0)
        base        = self._base_speed * speed_scale
        turn        = (angle_error / math.pi) * self._max_pwm

        left_pwm  = _clamp(base - turn, -self._max_pwm, self._max_pwm)
        right_pwm = _clamp(base + turn, -self._max_pwm, self._max_pwm)

        return left_pwm, right_pwm


# ════════════════════════════════════════════════════════════════════════════
# 3. 6-STATE CLEANING STATE MACHINE
# ════════════════════════════════════════════════════════════════════════════

class CleanState(Enum):
    IDLE    = auto()   # waiting for start command
    NAV     = auto()   # navigating to next waypoint (boustrophedon)
    CLEAN   = auto()   # actively cleaning a stripe (vacuum + brush on)
    AVOID   = auto()   # obstacle too close — potential field reroute or stop
    INSPECT = auto()   # coverage done — searching for missed zones
    DOCK    = auto()   # returning to dock (Phase 5 A* — stub for now)


@dataclass
class MotorCommand:
    left:   int = 0
    right:  int = 0
    vacuum: int = 0
    brush:  int = 0


# Vacuum and brush PWM per state
_ACTUATORS: dict[CleanState, tuple[int, int]] = {
    CleanState.IDLE:    (0,   0),
    CleanState.NAV:     (100, 80),
    CleanState.CLEAN:   (200, 160),
    CleanState.AVOID:   (80,  80),
    CleanState.INSPECT: (150, 120),
    CleanState.DOCK:    (0,   0),
}

# Front obstacle stop threshold
_OBSTACLE_STOP_CM = US_OBSTACLE_STOP_CM   # default 15 cm
_AVOID_TIMEOUT_S  = 8.0     # seconds to spend in AVOID before giving up


class CleaningStateMachine:
    """
    6-state autonomous cleaning logic.

    Call `step()` every navigation tick.  Returns a MotorCommand.
    Use `start()` / `stop()` from WebUI handlers.

    IDLE ──start()──▶ NAV ──waypoint reached──▶ CLEAN ──stripe done──▶ NAV
                       │                                                  │
                       ◀──obstacle clear──── AVOID ◀──obstacle detected───┘
                                               │
                    INSPECT ◀──coverage≥95%───┘
                       │
                    DOCK ◀──battery low──
    """

    def __init__(self,
                 grid:     OccupancyGrid,
                 planner:  BoustrophedonPlanner,
                 steering: PotentialFieldSteering,
                 dock_x:   float = 0.0,
                 dock_y:   float = 0.0) -> None:
        self._grid     = grid
        self._planner  = planner
        self._steering = steering
        self._dock_x   = dock_x
        self._dock_y   = dock_y

        self._state:        CleanState              = CleanState.IDLE
        self._avoid_since:  float                   = 0.0
        self._low_battery:  bool                    = False

        # A* path following (Phase 5)
        self._path:         list[tuple[float,float]] = []
        self._path_idx:     int                      = 0
        self._path_goal:    Optional[tuple[float,float]] = None
        self._inspect_goal: Optional[tuple[float,float]] = None

    # ── Public controls ───────────────────────────────────────────────────────

    @property
    def state(self) -> CleanState:
        return self._state

    @property
    def state_name(self) -> str:
        return self._state.name

    def start(self, robot_x: float, robot_y: float) -> None:
        """Begin a cleaning run from current position."""
        if self._state == CleanState.IDLE:
            self._planner.start(robot_x, robot_y)
            self._transition(CleanState.NAV)
            log.info("Cleaning run STARTED")

    def stop(self) -> None:
        """Emergency stop — return to IDLE immediately."""
        self._transition(CleanState.IDLE)
        log.info("Cleaning run STOPPED")

    def notify_low_battery(self) -> None:
        """Called when battery ADC reports < 15%. Triggers DOCK state."""
        self._low_battery = True

    def set_dock_position(self, x: float, y: float) -> None:
        """Update dock location (call once you know where the dock is)."""
        self._dock_x, self._dock_y = x, y

    # ── Main step ─────────────────────────────────────────────────────────────

    def step(self,
             robot_x:     float,
             robot_y:     float,
             robot_theta: float,
             ultrasonics: dict[str, float],
             battery_pct: float = 100.0) -> MotorCommand:
        """
        Advance the state machine one tick.
        Returns the MotorCommand to send to the Bridge this tick.
        """
        now = time.monotonic()

        # Low battery overrides everything
        if battery_pct < 15.0 and self._state not in (CleanState.DOCK, CleanState.IDLE):
            self._transition(CleanState.DOCK)

        # ── State logic ───────────────────────────────────────────────────
        if self._state == CleanState.IDLE:
            return MotorCommand()   # everything off

        elif self._state in (CleanState.NAV, CleanState.CLEAN):
            front_dist = ultrasonics.get('front', 999.0)

            # Obstacle check → AVOID
            if front_dist < _OBSTACLE_STOP_CM:
                self._avoid_since = now
                self._transition(CleanState.AVOID)
                return MotorCommand(*self._actuators())  # stop motors

            # Get next boustrophedon waypoint
            if self._planner.is_complete():
                # Check for missed zones
                missed = self._grid.nearest_uncleaned(robot_x, robot_y)
                if missed:
                    self._transition(CleanState.INSPECT)
                else:
                    log.info("Room 100% complete — returning to IDLE")
                    self._transition(CleanState.DOCK)
                return MotorCommand()

            wx, wy = self._planner.next_waypoint(robot_x, robot_y)
            left, right = self._steering.compute(
                robot_x, robot_y, robot_theta, wx, wy, ultrasonics
            )

            # If motors are zero we've reached the waypoint — advance state
            if left == 0 and right == 0:
                self._transition(CleanState.CLEAN)

            vac, brush = self._actuators()
            return MotorCommand(left=left, right=right, vacuum=vac, brush=brush)

        elif self._state == CleanState.AVOID:
            front_dist = ultrasonics.get('front', 999.0)

            # Obstacle gone or timeout
            if front_dist >= _OBSTACLE_STOP_CM * 1.5:
                self._transition(CleanState.NAV)
                return MotorCommand()

            if now - self._avoid_since > _AVOID_TIMEOUT_S:
                # Still blocked — try rotating right
                log.warning("AVOID timeout — rotating to escape")
                self._avoid_since = now   # reset timer
                return MotorCommand(left=100, right=-100,
                                    vacuum=80, brush=80)

            # Stop and wait (potential field handles the reroute if obstacle moves)
            return MotorCommand(vacuum=80, brush=80)

        elif self._state == CleanState.INSPECT:
            # Only replan when we have no active path
            if not self._path or self._path_idx >= len(self._path):
                missed = self._grid.nearest_uncleaned(robot_x, robot_y)
                if missed is None:
                    log.info("INSPECT: all zones cleaned — docking")
                    self._transition(CleanState.DOCK)
                    return MotorCommand()
                self._inspect_goal = missed
                self._path = plan_path(
                    self._grid, robot_x, robot_y,
                    missed[0], missed[1]
                ) or []
                self._path_idx = 0
                log.info(f"INSPECT: A* path to missed zone "
                         f"({missed[0]:.0f}, {missed[1]:.0f}) — "
                         f"{len(self._path)} waypoints")

            cmd, arrived = self._follow_path(
                robot_x, robot_y, robot_theta, ultrasonics
            )
            if arrived:
                self._clear_path()   # reached zone — replan for next on next tick
            return cmd

        elif self._state == CleanState.DOCK:
            # Plan A* path to dock on first tick in DOCK state
            if not self._path or self._path_idx >= len(self._path):
                new_path = plan_path(
                    self._grid,
                    robot_x, robot_y,
                    self._dock_x, self._dock_y,
                )
                if new_path is None:
                    log.warning("DOCK: no A* path to dock — driving direct")
                    # Fallback: drive directly (potential field only)
                    new_path = [(self._dock_x, self._dock_y)]
                self._path     = new_path
                self._path_idx = 0
                log.info(f"DOCK: A* path planned — {len(self._path)} waypoints")

            cmd, arrived = self._follow_path(
                robot_x, robot_y, robot_theta, ultrasonics
            )
            if arrived:
                log.info("DOCK: reached dock — switching to IDLE")
                self._transition(CleanState.IDLE)
            return MotorCommand(left=cmd.left, right=cmd.right)  # no vacuum at dock

        return MotorCommand()   # fallback

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _transition(self, new_state: CleanState) -> None:
        if new_state != self._state:
            log.info(f"State: {self._state.name} → {new_state.name}")
            self._state = new_state
            self._clear_path()   # always clear A* path on state change

    def _clear_path(self) -> None:
        self._path      = []
        self._path_idx  = 0
        self._path_goal = None

    def _follow_path(
        self,
        robot_x:     float,
        robot_y:     float,
        robot_theta: float,
        ultrasonics: dict[str, float],
    ) -> tuple[MotorCommand, bool]:
        """
        Follow the current A* path waypoint-by-waypoint using PotentialFieldSteering.

        Returns:
            (MotorCommand, arrived: bool)
            arrived = True when the path is fully completed.
        """
        if not self._path or self._path_idx >= len(self._path):
            return MotorCommand(), True   # path exhausted — arrived

        wx, wy = self._path[self._path_idx]
        left, right = self._steering.compute(
            robot_x, robot_y, robot_theta, wx, wy, ultrasonics
        )

        # Advance to next waypoint when steering says we're close enough
        if left == 0 and right == 0:
            self._path_idx += 1
            if self._path_idx >= len(self._path):
                return MotorCommand(), True   # final waypoint reached

        vac, brush = self._actuators()
        return MotorCommand(left=left, right=right, vacuum=vac, brush=brush), False

    def _actuators(self) -> tuple[int, int]:
        """Return (vacuum_pwm, brush_pwm) for current state."""
        return _ACTUATORS.get(self._state, (0, 0))
