"""
ARIA — Phase 5: A* Path Planning
Grid-based A* search with 8-directional movement and path simplification.

Public API:
    plan_path(grid, sx, sy, gx, gy)  → list[(x_cm, y_cm)] | None
    astar_cells(passable, start, goal) → list[(row, col)] | None

Performance: ~1–5 ms for a 33×33 grid on UNO Q Linux (Qualcomm MPU).
"""

from __future__ import annotations
import heapq
import math
from typing import Optional

import numpy as np

from .config import CELL_SIZE_CM, GRID_ORIGIN_ROW, GRID_ORIGIN_COL
from .occupancy_grid import OccupancyGrid


# ════════════════════════════════════════════════════════════════════════════
# COORDINATE HELPERS
# ════════════════════════════════════════════════════════════════════════════

def _world_to_cell(x_cm: float, y_cm: float,
                   rows: int, cols: int) -> tuple[int, int]:
    col = int(x_cm / CELL_SIZE_CM) + GRID_ORIGIN_COL
    row = int(y_cm / CELL_SIZE_CM) + GRID_ORIGIN_ROW
    return (max(0, min(rows - 1, row)),
            max(0, min(cols - 1, col)))


def _cell_to_world(row: int, col: int) -> tuple[float, float]:
    x = (col - GRID_ORIGIN_COL) * CELL_SIZE_CM + CELL_SIZE_CM / 2
    y = (row - GRID_ORIGIN_ROW) * CELL_SIZE_CM + CELL_SIZE_CM / 2
    return x, y


# ════════════════════════════════════════════════════════════════════════════
# CORE A* ALGORITHM
# ════════════════════════════════════════════════════════════════════════════

# 8-directional neighbours: (dr, dc, cost)
_NEIGHBOURS = [
    (-1,  0, 1.000), ( 1,  0, 1.000),
    ( 0, -1, 1.000), ( 0,  1, 1.000),
    (-1, -1, 1.414), (-1,  1, 1.414),
    ( 1, -1, 1.414), ( 1,  1, 1.414),
]


def astar_cells(
    passable: np.ndarray,
    start:    tuple[int, int],
    goal:     tuple[int, int],
) -> Optional[list[tuple[int, int]]]:
    """
    A* on a boolean passable grid (True = navigable).
    8-directional movement, Euclidean heuristic.

    If the goal cell is blocked, the nearest passable neighbour within
    3 cells is used as the goal instead.

    Returns an ordered list of (row, col) cells from start to goal,
    or None if no path exists.
    """
    rows, cols = passable.shape

    # Clamp & validate start
    sr, sc = start
    sr = max(0, min(rows - 1, sr))
    sc = max(0, min(cols - 1, sc))
    start = (sr, sc)

    # If goal is blocked, snap to nearest passable cell within 3-cell radius
    gr, gc = goal
    gr = max(0, min(rows - 1, gr))
    gc = max(0, min(cols - 1, gc))
    if not passable[gr, gc]:
        best_d, best = float('inf'), None
        for dr in range(-3, 4):
            for dc in range(-3, 4):
                nr, nc = gr + dr, gc + dc
                if 0 <= nr < rows and 0 <= nc < cols and passable[nr, nc]:
                    d = dr * dr + dc * dc
                    if d < best_d:
                        best_d, best = d, (nr, nc)
        if best is None:
            return None   # completely surrounded — no path possible
        gr, gc = best
    goal = (gr, gc)

    if start == goal:
        return [start]

    # ── A* search ────────────────────────────────────────────────────────────
    def h(r: int, c: int) -> float:
        return math.sqrt((r - gr) ** 2 + (c - gc) ** 2)

    open_heap: list[tuple[float, tuple[int, int]]] = [(h(sr, sc), start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score:   dict[tuple[int, int], float]           = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cr, cc = current
        cur_g  = g_score[current]

        for dr, dc, move_cost in _NEIGHBOURS:
            nr, nc = cr + dr, cc + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if not passable[nr, nc]:
                continue

            ng = cur_g + move_cost
            neighbour = (nr, nc)
            if ng < g_score.get(neighbour, float('inf')):
                came_from[neighbour] = current
                g_score[neighbour]   = ng
                heapq.heappush(open_heap, (ng + h(nr, nc), neighbour))

    return None   # no path found


# ════════════════════════════════════════════════════════════════════════════
# PATH SIMPLIFICATION
# ════════════════════════════════════════════════════════════════════════════

def _simplify_path(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """
    Remove collinear intermediate points.
    Keeps only cells where the direction changes (corners + start + end).
    Reduces waypoint count by ~60–80% on straight corridors.
    """
    if len(path) <= 2:
        return path

    result = [path[0]]
    for i in range(1, len(path) - 1):
        pr, pc = path[i - 1]
        cr, cc = path[i]
        nr, nc = path[i + 1]
        dr1, dc1 = cr - pr, cc - pc
        dr2, dc2 = nr - cr, nc - cc
        if (dr1, dc1) != (dr2, dc2):   # direction changed — keep this corner
            result.append(path[i])

    result.append(path[-1])
    return result


# ════════════════════════════════════════════════════════════════════════════
# WORLD-COORDINATE WRAPPER
# ════════════════════════════════════════════════════════════════════════════

def plan_path(
    grid:    OccupancyGrid,
    start_x: float,
    start_y: float,
    goal_x:  float,
    goal_y:  float,
) -> Optional[list[tuple[float, float]]]:
    """
    Plan an A* path in world coordinates (cm).

    Returns an ordered list of (x_cm, y_cm) waypoints from
    (start_x, start_y) to (goal_x, goal_y), or None if unreachable.

    Usage:
        path = plan_path(grid, robot_x, robot_y, dock_x, dock_y)
        if path:
            for wx, wy in path:
                # drive toward (wx, wy) using PotentialFieldSteering
    """
    nav = grid.nav_array           # bool ndarray — True = passable
    rows, cols = nav.shape

    start_cell = _world_to_cell(start_x, start_y, rows, cols)
    goal_cell  = _world_to_cell(goal_x,  goal_y,  rows, cols)

    cell_path = astar_cells(nav, start_cell, goal_cell)
    if cell_path is None:
        return None

    simplified = _simplify_path(cell_path)
    return [_cell_to_world(r, c) for r, c in simplified]
