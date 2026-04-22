from __future__ import annotations

from typing import List

import astar


def compute_search_turn_sign(path: List[int], leg_idx: int) -> float:
    # When we don't see the tag yet, we still want a reasonable spin direction.
    # We look at the grid edge we're trying to traverse: incoming segment x outgoing segment.
    # The 2D cross product tells us if the path bends left (+) or right (-) at the current node.
    if leg_idx <= 0 or leg_idx >= len(path):
        return 0.0

    current_node = path[leg_idx - 1]
    target_node = path[leg_idx]

    nx = astar.COORDINATES[target_node][0] - astar.COORDINATES[current_node][0]
    ny = astar.COORDINATES[target_node][1] - astar.COORDINATES[current_node][1]

    # First leg is a special case on our map; small positive = bias spin direction we liked in testing.
    if leg_idx == 1:
        return 0.3

    prev_node = path[leg_idx - 2]
    px = astar.COORDINATES[current_node][0] - astar.COORDINATES[prev_node][0]
    py = astar.COORDINATES[current_node][1] - astar.COORDINATES[prev_node][1]

    cross = px * ny - py * nx
    if cross > 0.1:
        return 1.0
    if cross < -0.1:
        return -1.0
    return 0.3
