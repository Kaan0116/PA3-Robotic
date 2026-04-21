#!/usr/bin/env python3
"""
ArUco detections viewer.

Opens a local OpenCV window on your computer that shows the Duckiebot's camera
stream with ArUco detections drawn as squares (polylines), plus the detected
ID, distance estimate (if calibration is available) and bearing.

Two modes are supported:

1) RAW mode (default): Subscribe to the robot's raw camera topic and run the
   ArUco detection locally in this process. This works even if the navigator
   node is not running.

2) DEBUG mode (--debug-image): Subscribe to the navigator's already-annotated
   debug image topic (/<robot>/debug_image/compressed). Useful to see what
   the navigator "sees" at runtime.

Typical usage (from your laptop):

    # Make sure ROS can reach the robot master
    export ROS_MASTER_URI=http://bear.local:11311
    export ROS_HOSTNAME=$(hostname).local

    rosrun assignment3 aruco_viewer.py _robot_name:=bear

    # OR simply (if executable):
    ./aruco_viewer.py --robot bear
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Optional

import cv2
import numpy as np

try:
    import rospy
    from sensor_msgs.msg import CompressedImage
    _HAS_ROS = True
except Exception:  # pragma: no cover - allow --help without ROS
    _HAS_ROS = False


# Default calibration values copied from bear.yaml. They are only used for
# distance estimation in the viewer overlay; do not rely on them for control.
CAMERA_FX_DEFAULT = 270.4563591302591
CAMERA_FY_DEFAULT = 269.2951665378049
CAMERA_CX_DEFAULT = 314.1813567017415
CAMERA_CY_DEFAULT = 218.88618596346137
DIST_COEFFS_DEFAULT = np.array(
    [
        [-0.19162991260105328],
        [0.026384790215657535],
        [0.005682129590129115],
        [0.0006647376545041703],
        [0.0],
    ],
    dtype=np.float32,
)

ARUCO_TAG_SIZE_METERS_DEFAULT = 0.065
ARUCO_DICTIONARY_DEFAULT = "DICT_5X5_50"


def _build_aruco_dictionary(name: str):
    attr = getattr(cv2.aruco, name, None)
    if attr is None:
        raise ValueError("Unsupported ArUco dictionary '%s'" % name)
    return cv2.aruco.getPredefinedDictionary(attr)


def _build_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def _detect_markers(image, dictionary, parameters):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers(image)
    return cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


class ArucoViewer:
    """Subscribes to a compressed image topic and shows ArUco detections."""

    def __init__(
        self,
        robot_name: str,
        dictionary_name: str,
        tag_size: float,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        use_debug_image: bool,
        window_name: str = "ArUco Viewer",
    ) -> None:
        self.robot_name = robot_name
        self.window_name = window_name
        self.tag_size = tag_size
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.use_debug_image = use_debug_image

        self.dictionary = _build_aruco_dictionary(dictionary_name)
        self.parameters = _build_detector_parameters()

        self._last_frame_time: Optional[float] = None
        self._fps = 0.0
        self._frame_count = 0

        if use_debug_image:
            topic = "/%s/debug_image/compressed" % robot_name
        else:
            topic = "/%s/camera_node/image/compressed" % robot_name

        self.topic = topic
        self._sub = rospy.Subscriber(
            topic,
            CompressedImage,
            self._on_image,
            queue_size=1,
            buff_size=2 ** 24,
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)

        rospy.loginfo("ArUco viewer subscribed to: %s", topic)
        rospy.loginfo(
            "Dictionary=%s  tag_size=%.3fm  mode=%s",
            dictionary_name,
            tag_size,
            "debug_image" if use_debug_image else "raw_camera",
        )

    def _on_image(self, msg: CompressedImage) -> None:
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "Failed to decode frame: %s", str(exc))
            return

        frame = cv2.resize(frame, (640, 480))

        now = time.time()
        if self._last_frame_time is not None:
            dt = now - self._last_frame_time
            if dt > 0:
                # Smooth FPS estimate
                inst = 1.0 / dt
                self._fps = 0.9 * self._fps + 0.1 * inst if self._fps > 0 else inst
        self._last_frame_time = now
        self._frame_count += 1

        if self.use_debug_image:
            # Navigator already drew detections; just annotate FPS.
            self._draw_header(frame, [])
            self._safe_show(frame)
            return

        corners, ids, _rejected = _detect_markers(
            frame, self.dictionary, self.parameters
        )

        visible: list = []
        tvecs = None

        if ids is not None and len(ids) > 0:
            try:
                _rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.tag_size,
                    self.camera_matrix,
                    self.dist_coeffs,
                )
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "Pose estimation failed: %s", str(exc))
                tvecs = None

            flat_ids = ids.reshape(-1)
            for i, tag_id in enumerate(flat_ids):
                pts = corners[i][0].astype(int)
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))

                dist: Optional[float] = None
                yaw: Optional[float] = None
                if tvecs is not None:
                    tx, ty, tz = [float(v) for v in tvecs[i][0]]
                    dist = _norm3(tx, ty, tz)
                    # Bearing in robot-base convention: +y = robot's left,
                    # so positive yaw => tag is on the robot's left.
                    yaw = math.atan2(-tx, tz)

                color = (0, 255, 0)
                cv2.polylines(frame, [pts], True, color, 2)
                # Draw ID at the first corner
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

                parts = [f"ID:{int(tag_id)}"]
                if dist is not None:
                    parts.append(f"d={dist:.2f}m")
                if yaw is not None:
                    parts.append(f"y={math.degrees(yaw):+.1f}°")
                label = "  ".join(parts)
                cv2.putText(
                    frame,
                    label,
                    (cx - 60, max(20, cy - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )
                visible.append(int(tag_id))

        self._draw_header(frame, sorted(visible))
        self._safe_show(frame)

    def _draw_header(self, frame: np.ndarray, visible_ids) -> None:
        text = f"Robot: {self.robot_name}   FPS: {self._fps:4.1f}"
        cv2.putText(
            frame,
            text,
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        if not self.use_debug_image:
            cv2.putText(
                frame,
                f"Visible tags: {visible_ids if visible_ids else 'none'}",
                (10, 46),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )

    def _safe_show(self, frame: np.ndarray) -> None:
        try:
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                rospy.signal_shutdown("user quit")
        except cv2.error as exc:
            rospy.logerr_throttle(
                5.0,
                "cv2.imshow failed (no display?). Error: %s",
                str(exc),
            )

    def spin(self) -> None:
        try:
            rospy.spin()
        finally:
            cv2.destroyAllWindows()


def _parse_args(argv):
    parser = argparse.ArgumentParser(description="ArUco live viewer for Duckiebot")
    parser.add_argument(
        "--robot",
        "-r",
        default=os.environ.get("VEHICLE_NAME", "bear"),
        help="Robot name (default: $VEHICLE_NAME or 'bear')",
    )
    parser.add_argument(
        "--dictionary",
        default=ARUCO_DICTIONARY_DEFAULT,
        help="ArUco dictionary name (default: %s)" % ARUCO_DICTIONARY_DEFAULT,
    )
    parser.add_argument(
        "--tag-size",
        type=float,
        default=ARUCO_TAG_SIZE_METERS_DEFAULT,
        help="Physical tag size in meters (default: %.3f)"
        % ARUCO_TAG_SIZE_METERS_DEFAULT,
    )
    parser.add_argument(
        "--debug-image",
        action="store_true",
        help="Subscribe to navigator's /<robot>/debug_image/compressed instead "
        "of detecting locally.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)

    if not _HAS_ROS:
        print("ERROR: rospy is not available in this environment.", file=sys.stderr)
        return 2

    rospy.init_node("aruco_viewer", anonymous=True)

    # Allow ROS params to override CLI (so this works with roslaunch too).
    robot_name = str(rospy.get_param("~robot_name", args.robot))
    dictionary_name = str(rospy.get_param("~aruco_dictionary", args.dictionary))
    tag_size = float(rospy.get_param("~aruco_tag_size_meters", args.tag_size))
    use_debug_image = bool(rospy.get_param("~use_debug_image", args.debug_image))

    camera_matrix = np.array(
        [
            [
                float(rospy.get_param("~camera_fx", CAMERA_FX_DEFAULT)),
                0.0,
                float(rospy.get_param("~camera_cx", CAMERA_CX_DEFAULT)),
            ],
            [
                0.0,
                float(rospy.get_param("~camera_fy", CAMERA_FY_DEFAULT)),
                float(rospy.get_param("~camera_cy", CAMERA_CY_DEFAULT)),
            ],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )

    viewer = ArucoViewer(
        robot_name=robot_name,
        dictionary_name=dictionary_name,
        tag_size=tag_size,
        camera_matrix=camera_matrix,
        dist_coeffs=DIST_COEFFS_DEFAULT,
        use_debug_image=use_debug_image,
    )
    viewer.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
