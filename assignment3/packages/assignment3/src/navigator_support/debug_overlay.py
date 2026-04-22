from __future__ import annotations

import cv2
import numpy as np


def draw_debug_overlay(frame: np.ndarray, target_node: int, state: str) -> np.ndarray:
    # Draw target node + state on a copy of the frame (use if you publish a separate debug stream).
    debug = frame.copy()
    cv2.putText(
        debug,
        "TARGET: N%d" % target_node,
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2,
    )
    cv2.putText(
        debug,
        "STATE: %s" % state,
        (10, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 0),
        2,
    )
    return debug
