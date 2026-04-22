from __future__ import annotations

import math


def norm3(x: float, y: float, z: float) -> float:
    # Distance from camera to the tag center using the translation vector from solvePnP.
    return math.sqrt(x * x + y * y + z * z)


def bearing_to_tag(tx: float, ty: float, tz: float) -> float:
    # In the camera frame, tx is left-right and tz is forward. atan2(-tx, tz) is the yaw error we care about
    # for steering (how far the tag is left/right in the image plane, converted to an angle).
    return math.atan2(-tx, tz)
