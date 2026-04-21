#!/usr/bin/env python3
"""
A* path planning from scratch (no path-planning libraries).
Grid graph N0..N15 with coordinates and edge costs as specified for PA3.
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

# Node coordinates (x, y) — indices match node id
COORDINATES: Dict[int, Tuple[float, float]] = {
    0:  (0.0, 0.0),
    1:  (1.0, 0.0),
    2:  (2.0, 0.0),
    3:  (3.0, 0.0),
    4:  (0.0, 1.0),
    5:  (1.0, 1.0),
    6:  (2.0, 1.0),
    7:  (3.0, 1.0),
    8:  (0.0, 2.0),
    9:  (1.0, 2.0),
    10: (2.0, 2.0),
    11: (3.0, 2.0),
    12: (0.0, 3.0),
    13: (1.0, 3.0),
    14: (2.0, 3.0),
    15: (3.0, 3.0),
}

# Undirected graph: neighbor -> edge cost (exactly as in assignment)
GRAPH: Dict[int, List[Tuple[int, float]]] = {
    0:  [(1, 1.5), (4, 2.0)],
    1:  [(0, 1.5), (2, 1.0), (5, 2.0)],
    2:  [(1, 1.0), (3, 1.0), (6, 1.5)],
    3:  [(2, 1.0), (7, 1.0)],
    4:  [(0, 2.0), (5, 2.0), (8, 1.5)],
    5:  [(1, 2.0), (4, 2.0), (6, 1.0), (9, 2.0)],
    6:  [(2, 1.5), (5, 1.0), (7, 0.5), (10, 4.0)],
    7:  [(3, 1.0), (6, 0.5), (11, 1.5)],
    8:  [(4, 1.5), (9, 1.5), (12, 2.0)],
    9:  [(5, 2.0), (8, 1.5), (10, 2.0), (13, 1.5)],
    10: [(6, 4.0), (9, 2.0), (11, 1.0), (14, 1.5)],
    11: [(7, 1.5), (10, 1.0), (15, 1.0)],
    12: [(8, 2.0), (13, 1.5)],
    13: [(9, 1.5), (12, 1.5), (14, 2.0)],
    14: [(10, 1.5), (13, 2.0), (15, 1.0)],
    15: [(11, 1.0), (14, 1.0)],
}


def euclidean_heuristic(node: int, goal: int) -> float:
    """h(n): Euclidean distance from node to goal using grid coordinates."""
    x1, y1 = COORDINATES[node]
    x2, y2 = COORDINATES[goal]
    return math.hypot(x2 - x1, y2 - y1)


def astar_search(
    start: int, goal: int, verbose: bool = False
) -> Tuple[Optional[List[int]], float]:
    """
    A* search with:
      - OPEN  : min-heap ordered by (f, h) — equal f → lower h first
      - CLOSED: set of expanded nodes
      - Tracks g(n), h(n), f(n) and parent for path reconstruction
    """
    if start not in COORDINATES or goal not in COORDINATES:
        return None, float("nan")

    def h(n: int) -> float:
        return euclidean_heuristic(n, goal)

    # OPEN: (f, h, g, node)
    OPEN: List[Tuple[float, float, float, int]] = []
    CLOSED: Set[int] = set()
    g_score: Dict[int, float] = {start: 0.0}
    parent: Dict[int, Optional[int]] = {start: None}

    h0 = h(start)
    f0 = h0  # g=0 at start
    heapq.heappush(OPEN, (f0, h0, 0.0, start))

    if verbose:
        print("\n--- A* Expansion Log ---")
        print(f"{'Step':>4}  {'Node':>6}  {'g(n)':>8}  {'h(n)':>8}  {'f(n)':>8}")
        print("-" * 44)

    step = 0

    while OPEN:
        _f, _h, g, current = heapq.heappop(OPEN)

        if current in CLOSED:
            continue
        if g > g_score[current] + 1e-9:
            continue

        CLOSED.add(current)
        step += 1

        hn = h(current)
        fn = g + hn

        if verbose:
            print(f"{step:>4}  N{current:<5}  {g:>8.4f}  {hn:>8.4f}  {fn:>8.4f}")

        if current == goal:
            # Reconstruct path via parent tracking
            path: List[int] = []
            cur: Optional[int] = goal
            while cur is not None:
                path.append(cur)
                cur = parent.get(cur)
            path.reverse()
            if not path or path[0] != start or path[-1] != goal:
                return None, float("nan")
            return path, g_score[goal]

        for nbr, step_cost in GRAPH[current]:
            if nbr in CLOSED:
                continue
            tentative_g = g + step_cost
            if tentative_g < g_score.get(nbr, float("inf")) - 1e-9:
                g_score[nbr] = tentative_g
                parent[nbr] = current
                hn_nbr = h(nbr)
                fn_nbr = tentative_g + hn_nbr
                heapq.heappush(OPEN, (fn_nbr, hn_nbr, tentative_g, nbr))

    return None, float("nan")


def format_path(path: List[int]) -> str:
    return " -> ".join("N%d" % n for n in path)


def _report(start: int, goal: int) -> None:
    path, cost = astar_search(start, goal, verbose=True)
    print()
    if path is None:
        print("No path found from N%d to N%d." % (start, goal))
        return
    print("Path sequence : %s" % format_path(path))
    print("Total cost    : %.4f" % cost)


if __name__ == "__main__":
    _report(0, 15)