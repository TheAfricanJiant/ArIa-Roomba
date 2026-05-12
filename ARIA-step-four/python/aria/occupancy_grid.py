"""
ARIA — Occupancy Grid
Efficient NumPy-based room map. Each cell is 30 cm × 30 cm.

Cell states (uint8):
  0 = UNKNOWN   — never seen
  1 = FREE       — open space, not yet cleaned
  2 = CLEANED    — robot passed through
  3 = WALL       — ultrasonic detected boundary
  4 = OBSTACLE   — dynamic obstacle (AI detected)

Performance: all grid ops are vectorised NumPy — microsecond-level latency.
"""

from __future__ import annotations
import math
import time
from typing import Optional

import numpy as np

from .config import (
    GRID_ROWS, GRID_COLS, CELL_SIZE_CM,
    GRID_ORIGIN_ROW, GRID_ORIGIN_COL,
    US_MAX_VALID_CM, US_ANGLES,
)

# Cell state constants
UNKNOWN  = np.uint8(0)
FREE     = np.uint8(1)
CLEANED  = np.uint8(2)
WALL     = np.uint8(3)
OBSTACLE = np.uint8(4)

# Terminal visualisation characters
_CELL_CHARS = {0: '·', 1: '░', 2: '✓', 3: '█', 4: '▓'}


class OccupancyGrid:
    """
    NumPy-backed occupancy grid for room mapping.

    Coordinates: robot starts at (0, 0) cm = grid origin.
    Positive X = right, positive Y = forward (robot's initial heading).

    Usage:
        grid = OccupancyGrid()
        grid.mark_cleaned(x_cm, y_cm)
        grid.update_from_ultrasonics(x_cm, y_cm, theta_rad, distances)
        print(f"Coverage: {grid.coverage_percent():.1f}%")
        grid.print_terminal(robot_x=x_cm, robot_y=y_cm)
    """

    __slots__ = ('_grid', '_dirty', '_run_timestamps')

    def __init__(self) -> None:
        # uint8 array — 1 byte per cell, cache-friendly
        self._grid = np.full((GRID_ROWS, GRID_COLS), UNKNOWN, dtype=np.uint8)
        self._dirty = False

    # ── Coordinate helpers ───────────────────────────────────────────────────

    def _to_cell(self, x_cm: float, y_cm: float) -> tuple[int, int]:
        """Convert world cm coords to (row, col). Returns (-1,-1) if out of bounds."""
        col = int(x_cm / CELL_SIZE_CM) + GRID_ORIGIN_COL
        row = GRID_ORIGIN_ROW - int(y_cm / CELL_SIZE_CM)
        if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
            return row, col
        return -1, -1

    def _to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert grid (row, col) to world cm coords (cell centre)."""
        x = (col - GRID_ORIGIN_COL) * CELL_SIZE_CM + CELL_SIZE_CM / 2
        y = (GRID_ORIGIN_ROW - row) * CELL_SIZE_CM - CELL_SIZE_CM / 2
        return x, y

    # ── Core updates ─────────────────────────────────────────────────────────

    def mark_cleaned(self, x_cm: float, y_cm: float) -> None:
        """Mark the cell at (x_cm, y_cm) as CLEANED. Call every EKF update."""
        row, col = self._to_cell(x_cm, y_cm)
        if row >= 0 and self._grid[row, col] not in (WALL, OBSTACLE):
            self._grid[row, col] = CLEANED
            self._dirty = True

    def mark_free(self, x_cm: float, y_cm: float) -> None:
        """Mark cell as FREE (open space, not yet cleaned)."""
        row, col = self._to_cell(x_cm, y_cm)
        if row >= 0 and self._grid[row, col] == UNKNOWN:
            self._grid[row, col] = FREE
            self._dirty = True

    def mark_obstacle(self, x_cm: float, y_cm: float,
                      temporary: bool = True) -> None:
        """Mark cell as dynamic OBSTACLE (AI-detected pet, person, etc.)."""
        row, col = self._to_cell(x_cm, y_cm)
        if row >= 0:
            self._grid[row, col] = OBSTACLE
            self._dirty = True

    def clear_obstacle(self, x_cm: float, y_cm: float) -> None:
        """Revert a temporary OBSTACLE cell back to FREE."""
        row, col = self._to_cell(x_cm, y_cm)
        if row >= 0 and self._grid[row, col] == OBSTACLE:
            self._grid[row, col] = FREE
            self._dirty = True

    def update_from_ultrasonics(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        distances: dict[str, float],
    ) -> None:
        """
        Update grid from a full ultrasonic reading packet.

        Args:
            robot_x, robot_y: EKF position (cm)
            robot_theta:      EKF heading (rad)
            distances:        dict of sensor_name → distance_cm
                              e.g. {'front': 24.0, 'left': 8.2, ...}

        Cells between robot and detected wall are marked FREE.
        The wall cell itself is marked WALL.
        """
        cos_h = math.cos(robot_theta)
        sin_h = math.sin(robot_theta)

        for name, dist in distances.items():
            if dist <= 0 or dist > US_MAX_VALID_CM:
                continue

            sensor_angle = US_ANGLES.get(name, 0.0)
            abs_angle    = robot_theta + sensor_angle
            cos_a = math.cos(abs_angle)
            sin_a = math.sin(abs_angle)

            # Ray march: mark FREE cells along the beam
            n_free = max(1, int(dist / CELL_SIZE_CM))
            for i in range(n_free):
                fx = robot_x + cos_a * i * CELL_SIZE_CM
                fy = robot_y + sin_a * i * CELL_SIZE_CM
                self.mark_free(fx, fy)

            # Wall cell at beam endpoint
            wall_x = robot_x + cos_a * dist
            wall_y = robot_y + sin_a * dist
            row, col = self._to_cell(wall_x, wall_y)
            if row >= 0:
                self._grid[row, col] = WALL
                self._dirty = True

    # ── Statistics ───────────────────────────────────────────────────────────

    def coverage_percent(self) -> float:
        """Fraction of known-free space that has been cleaned (0–100)."""
        cleanable = int(np.count_nonzero(self._grid >= FREE))
        cleaned   = int(np.count_nonzero(self._grid == CLEANED))
        return (cleaned / max(cleanable, 1)) * 100.0

    def total_cells_known(self) -> int:
        return int(np.count_nonzero(self._grid > UNKNOWN))

    def total_cleaned(self) -> int:
        return int(np.count_nonzero(self._grid == CLEANED))

    def nearest_uncleaned(
        self, robot_x: float, robot_y: float
    ) -> Optional[tuple[float, float]]:
        """
        Find the FREE (uncleaned) cell nearest to the robot.
        Used by INSPECT state to mop up missed zones.
        Returns (x_cm, y_cm) of cell centre, or None if room is complete.
        """
        rows, cols = np.where(self._grid == FREE)
        if rows.size == 0:
            return None

        # Vectorised distance to all FREE cells
        robot_row, robot_col = self._to_cell(robot_x, robot_y)
        dr = rows - robot_row
        dc = cols - robot_col
        dists = dr * dr + dc * dc  # squared — no sqrt needed for argmin
        idx   = int(np.argmin(dists))
        return self._to_world(int(rows[idx]), int(cols[idx]))

    # ── A* pathfinding support ───────────────────────────────────────────────

    @property
    def nav_array(self) -> np.ndarray:
        """
        Return boolean passable array (True = navigable).
        Used directly by A* planner (Phase 5).
        """
        return self._grid < WALL  # UNKNOWN, FREE, CLEANED are passable

    def cell_state(self, x_cm: float, y_cm: float) -> int:
        row, col = self._to_cell(x_cm, y_cm)
        if row < 0:
            return int(WALL)
        return int(self._grid[row, col])

    # ── Persistence ──────────────────────────────────────────────────────────

    def save(self, path: str) -> None:
        """Save grid to disk atomically to prevent corruption on power loss."""
        import os
        import tempfile
        
        dir_path = os.path.dirname(os.path.abspath(path))
        os.makedirs(dir_path, exist_ok=True)
        
        # Save to temp file in same directory, then rename atomically
        fd, tmp_path = tempfile.mkstemp(dir=dir_path, suffix='.npy')
        os.close(fd)
        try:
            np.save(tmp_path, self._grid)
            os.replace(tmp_path, path)
        except Exception:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
            raise

    @classmethod
    def load(cls, path: str) -> "OccupancyGrid":
        """Load a previously saved grid. Fallback to empty if corrupted."""
        import os
        grid = cls()
        try:
            loaded = np.load(path)
            if loaded.shape == (GRID_ROWS, GRID_COLS):
                grid._grid = loaded
            else:
                print(f"[ARIA] Warning: Map shape mismatch. Starting fresh.")
        except Exception as e:
            print(f"[ARIA] Warning: Failed to load {path} ({e}). Starting fresh.")
        return grid

    # ── Debug visualisation ──────────────────────────────────────────────────

    def print_terminal(
        self, robot_x: Optional[float] = None, robot_y: Optional[float] = None
    ) -> None:
        """
        Print a compact ASCII map to the terminal for debugging.
        Only renders rows/cols that contain any non-UNKNOWN cells
        (plus 1 border), keeping the output compact.
        """
        known_rows, known_cols = np.where(self._grid > UNKNOWN)
        if known_rows.size == 0:
            print("[ARIA Grid] — no data yet")
            return

        r_min = max(0, int(known_rows.min()) - 1)
        r_max = min(GRID_ROWS - 1, int(known_rows.max()) + 1)
        c_min = max(0, int(known_cols.min()) - 1)
        c_max = min(GRID_COLS - 1, int(known_cols.max()) + 1)

        robot_row, robot_col = -1, -1
        if robot_x is not None and robot_y is not None:
            robot_row, robot_col = self._to_cell(robot_x, robot_y)

        lines = []
        for r in range(r_max, r_min - 1, -1):   # top row = positive Y
            row_chars = []
            for c in range(c_min, c_max + 1):
                if r == robot_row and c == robot_col:
                    row_chars.append('R')
                else:
                    row_chars.append(_CELL_CHARS.get(int(self._grid[r, c]), '?'))
            lines.append(' '.join(row_chars))

        cov = self.coverage_percent()
        print(f"\n[ARIA Grid] coverage={cov:.1f}%  "
              f"cleaned={self.total_cleaned()}  known={self.total_cells_known()}")
        print('\n'.join(lines))
        print()
