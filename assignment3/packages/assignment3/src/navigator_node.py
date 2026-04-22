#!/usr/bin/env python3
# The robot never gets global pose; it only sees tags in the camera and drives toward the next node ID on the path.
#
# States (one "leg" = go from previous node to current target node on the path):
#   SEARCH       - can't trust the target tag yet -> spin to find it (direction guessed from the grid polyline).
#   ALIGN        - tag visible but we're pointed wrong -> turn in place (P control on yaw error).
#   APPROACH     - tag visible and roughly aligned -> drive forward while correcting omega.
#   PASS_THROUGH - we're close enough and lined up -> drive straight a bit so we physically cross the node, then next leg.
from __future__ import annotations

import math
import os
import sys
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage

# ROS installs scripts in a folder with astar.py and navigator_support/ next to this file; make imports work the same in devel and install.
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

import astar

from navigator_support.aruco_cv import build_aruco_dictionary, build_detector_parameters, detect_markers
from navigator_support.defaults import (
    ALIGN_ANGLE_MAX_DEFAULT,
    ALIGN_LINEAR_SPEED_DEFAULT,
    ANGULAR_GAIN_DEFAULT,
    ARUCO_DICTIONARY_DEFAULT,
    ARUCO_TAG_SIZE_METERS_DEFAULT,
    CAMERA_CX_DEFAULT,
    CAMERA_CY_DEFAULT,
    CAMERA_FX_DEFAULT,
    CAMERA_FY_DEFAULT,
    CMD_LOG_PERIOD_SEC_DEFAULT,
    CONFIRM_FRAMES,
    DETECTION_STALE_SEC_DEFAULT,
    DIST_K1_DEFAULT,
    DIST_K2_DEFAULT,
    DIST_K3_DEFAULT,
    DIST_P1_DEFAULT,
    DIST_P2_DEFAULT,
    LINEAR_SPEED_DEFAULT,
    LOST_COAST_SEC_DEFAULT,
    MAX_ANGULAR_SPEED_DEFAULT,
    MIN_TURN_OMEGA_DEFAULT,
    PASS_THROUGH_ALIGN_THRESHOLD_DEFAULT,
    PASS_THROUGH_SPEED_DEFAULT,
    PASS_THROUGH_TIME_DEFAULT,
    PROXIMITY_THRESHOLD_DEFAULT,
    REACH_CONFIRM_FRAMES,
    RIGHT_TURN_OMEGA_SCALE_DEFAULT,
    ROBOT_NAME_DEFAULT,
    SEARCH_ANGULAR_SPEED_DEFAULT,
    SEARCH_LINEAR_SPEED_DEFAULT,
    SEARCH_RIGHT_SCALE_DEFAULT,
    WAIT_LOG_PERIOD_SEC_DEFAULT,
    YAW_BIAS_DEFAULT,
)
from navigator_support.geometry import bearing_to_tag, norm3
from navigator_support.path_geometry import compute_search_turn_sign


class Assignment3Navigator:
    STATE_SEARCH = "SEARCH"
    STATE_ALIGN = "ALIGN"
    STATE_APPROACH = "APPROACH"
    STATE_PASS_THROUGH = "PASS_THROUGH"

    def __init__(self) -> None:
        # All tunables come from the private namespace (~param) so launch files can override without editing code.
        self.robot_name = rospy.get_param("~robot_name", ROBOT_NAME_DEFAULT)
        self.linear_speed = float(rospy.get_param("~linear_speed", LINEAR_SPEED_DEFAULT))
        self.angular_gain = float(rospy.get_param("~angular_gain", ANGULAR_GAIN_DEFAULT))
        self.proximity_threshold = float(rospy.get_param("~proximity_threshold", PROXIMITY_THRESHOLD_DEFAULT))
        self.align_angle_max = float(rospy.get_param("~align_angle_max", ALIGN_ANGLE_MAX_DEFAULT))
        self.search_omega = float(rospy.get_param("~search_angular_speed", SEARCH_ANGULAR_SPEED_DEFAULT))
        self.search_linear_speed = float(rospy.get_param("~search_linear_speed", SEARCH_LINEAR_SPEED_DEFAULT))
        self.align_linear_speed = float(rospy.get_param("~align_linear_speed", ALIGN_LINEAR_SPEED_DEFAULT))
        self.max_omega = float(rospy.get_param("~max_angular_speed", MAX_ANGULAR_SPEED_DEFAULT))
        self.detection_stale_sec = float(rospy.get_param("~detection_stale_sec", DETECTION_STALE_SEC_DEFAULT))
        self.wait_log_period_sec = float(rospy.get_param("~wait_log_period_sec", WAIT_LOG_PERIOD_SEC_DEFAULT))
        self.cmd_log_period_sec = float(rospy.get_param("~cmd_log_period_sec", CMD_LOG_PERIOD_SEC_DEFAULT))
        self.lost_coast_sec = float(rospy.get_param("~lost_coast_sec", LOST_COAST_SEC_DEFAULT))
        self.min_turn_omega = float(rospy.get_param("~min_turn_omega", MIN_TURN_OMEGA_DEFAULT))
        self.right_turn_scale = float(rospy.get_param("~right_turn_omega_scale", RIGHT_TURN_OMEGA_SCALE_DEFAULT))
        self.search_right_scale = float(rospy.get_param("~search_right_scale", SEARCH_RIGHT_SCALE_DEFAULT))
        self.yaw_bias = float(rospy.get_param("~yaw_bias", YAW_BIAS_DEFAULT))
        self.pass_through_time = float(rospy.get_param("~pass_through_time", PASS_THROUGH_TIME_DEFAULT))
        self.pass_through_speed = float(rospy.get_param("~pass_through_speed", PASS_THROUGH_SPEED_DEFAULT))
        self.pass_through_align_threshold = float(
            rospy.get_param("~pass_through_align_threshold", PASS_THROUGH_ALIGN_THRESHOLD_DEFAULT)
        )
        self.aruco_tag_size = float(rospy.get_param("~aruco_tag_size_meters", ARUCO_TAG_SIZE_METERS_DEFAULT))
        self.aruco_dictionary_name = str(rospy.get_param("~aruco_dictionary", ARUCO_DICTIONARY_DEFAULT))

        # Pinhole model + distortion: needed so estimatePoseSingleMarkers gives tvec in meters in the camera frame.
        self.camera_matrix = np.array(
            [
                [float(rospy.get_param("~camera_fx", CAMERA_FX_DEFAULT)), 0.0, float(rospy.get_param("~camera_cx", CAMERA_CX_DEFAULT))],
                [0.0, float(rospy.get_param("~camera_fy", CAMERA_FY_DEFAULT)), float(rospy.get_param("~camera_cy", CAMERA_CY_DEFAULT))],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

        self.dist_coeffs = np.array(
            [
                [float(rospy.get_param("~dist_k1", DIST_K1_DEFAULT))],
                [float(rospy.get_param("~dist_k2", DIST_K2_DEFAULT))],
                [float(rospy.get_param("~dist_p1", DIST_P1_DEFAULT))],
                [float(rospy.get_param("~dist_p2", DIST_P2_DEFAULT))],
                [float(rospy.get_param("~dist_k3", DIST_K3_DEFAULT))],
            ],
            dtype=np.float32,
        )

        self.aruco_dictionary = build_aruco_dictionary(self.aruco_dictionary_name)
        self.aruco_parameters = build_detector_parameters()

        # plan once from N0 to N15 on the given graph.
        path, cost = astar.astar_search(0, 15, verbose=True)
        if path is None:
            rospy.logfatal("A* found no path from N0 to N15.")
            raise RuntimeError("A* found no path from N0 to N15.")
        self.path: List[int] = path
        self._path_cost: float = cost

        rospy.loginfo("A* path: %s", astar.format_path(self.path))
        # Leg 0 would be "at start"; we begin by driving toward path[1] while standing near N0.
        self._leg = 1
        self._goal_done = False
        self._state = self.STATE_SEARCH

        # Per-tag: latest (distance, yaw_error, stamp). Buffers debounce noise so one bad frame doesn't switch states.
        self._tag_metrics: Dict[int, Tuple[float, float, rospy.Time]] = {}
        self._last_camera_msg_time: Optional[rospy.Time] = None
        self._detection_buffer: Dict[int, int] = {}
        self._reach_buffer: Dict[int, int] = {}
        self._search_sign = compute_search_turn_sign(self.path, self._leg)
        self._last_yaw_err: Optional[float] = None
        self._pass_through_start_time: Optional[rospy.Time] = None

        image_topic = rospy.get_param("~camera_image_topic", "/%s/camera_node/image/compressed" % self.robot_name)
        self._sub = rospy.Subscriber(image_topic, CompressedImage, self._on_camera_image, queue_size=1, buff_size=2 ** 24)

        cmd_topic = rospy.get_param("~cmd_topic", "/%s/car_cmd_switch_node/cmd" % self.robot_name)
        self._pub = rospy.Publisher(cmd_topic, Twist2DStamped, queue_size=1)
        self.debug_pub = rospy.Publisher("/%s/debug_image/compressed" % self.robot_name, CompressedImage, queue_size=1)

        # Control loop timing; camera callback runs async and only updates measurements.
        self._rate = rospy.Rate(20.0)
        rospy.on_shutdown(self._on_shutdown)
        rospy.sleep(0.3)

    def _on_camera_image(self, msg: CompressedImage) -> None:
        # Decode JPEG from Duckietown compressed image; stamp is used to drop stale ArUco measurements in the controller.
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        self._last_camera_msg_time = stamp
        np_image = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_image, cv2.IMREAD_COLOR)
        if frame is None:
            return

        frame = cv2.resize(frame, (640, 480))
        corners, ids, _rejected = detect_markers(frame, self.aruco_dictionary, self.aruco_parameters)

        metrics: Dict[int, Tuple[float, float, rospy.Time]] = {}
        if ids is not None and len(ids) > 0:
            _rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.aruco_tag_size, self.camera_matrix, self.dist_coeffs
            )
            flat_ids = ids.reshape(-1)
            for idx, tag_id in enumerate(flat_ids):
                tx, ty, tz = [float(v) for v in tvecs[idx][0]]
                dist = norm3(tx, ty, tz)
                yaw_off = bearing_to_tag(tx, ty, tz)
                metrics[int(tag_id)] = (dist, yaw_off, stamp)

        target_node = self.path[self._leg] if self._leg < len(self.path) else -1
        for tag_id, (dist, _yaw, _st) in metrics.items():
            self._detection_buffer[tag_id] = self._detection_buffer.get(tag_id, 0) + 1
            if tag_id == target_node and dist < self.proximity_threshold:
                self._reach_buffer[tag_id] = self._reach_buffer.get(tag_id, 0) + 1
            elif tag_id == target_node:
                self._reach_buffer[tag_id] = 0

        for tag_id, data in metrics.items():
            self._tag_metrics[tag_id] = data

        # Still publishing the raw compressed image (overlay helper exists separately if we want to draw on it later).
        self.debug_pub.publish(msg)

    def _compensate_right_turn(self, omega: float, *, search: bool = False) -> float:
        # Our duckie turned weaker to the right in testing, so negative omega gets scaled up (more magnitude).
        if omega >= 0:
            return omega
        scale = self.search_right_scale if search else self.right_turn_scale
        return omega * float(scale)

    def _publish_cmd(self, v: float, omega: float) -> None:
        omega = max(-self.max_omega, min(self.max_omega, omega))
        m = Twist2DStamped()
        m.header.stamp = rospy.Time.now()
        m.v = float(v)
        m.omega = float(omega)
        self._pub.publish(m)

    def _on_shutdown(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _tick_pass_through(self, target_node: int, now: rospy.Time) -> bool:
        # After we declare the node "reached", we keep driving straight briefly so the wheels actually leave the intersection.
        if self._pass_through_start_time is None:
            self._pass_through_start_time = now
        if (now - self._pass_through_start_time).to_sec() >= self.pass_through_time:
            self._advance_leg(target_node)
            return True
        self._publish_cmd(self.pass_through_speed, 0.0)
        self._rate.sleep()
        return True

    def _tick_vision_control(self, target_node: int, now: rospy.Time) -> bool:
        info = self._tag_metrics.get(target_node)
        # "Fresh" means: we saw this tag recently, and we got enough consecutive frames to trust it (CONFIRM_FRAMES).
        fresh = (
            info
            and (now - info[2]).to_sec() < self.detection_stale_sec
            and self._detection_buffer.get(target_node, 0) >= CONFIRM_FRAMES
        )
        if not fresh:
            self._state = self.STATE_SEARCH
            magnitude = max(abs(self.search_omega), self.min_turn_omega)
            # If we had a yaw error before losing the tag, keep spinning that way; else use the grid-based guess.
            # yaw_err == 0.0 counts as "no remembered direction" so we fall back to the grid hint.
            if self._last_yaw_err:
                omega = math.copysign(magnitude, self._last_yaw_err)
            else:
                omega = math.copysign(magnitude, self._search_sign)
            self._publish_cmd(self.search_linear_speed, self._compensate_right_turn(omega, search=True))
            return False

        dist, yaw_err = info[0], info[1]
        self._last_yaw_err = yaw_err
        yaw_cmd = yaw_err + self.yaw_bias

        # "Reached" the node: close, facing well enough, and reach counter stable (REACH_CONFIRM_FRAMES).
        if (
            dist < self.proximity_threshold
            and self._reach_buffer.get(target_node, 0) >= REACH_CONFIRM_FRAMES
            and abs(yaw_cmd) < self.pass_through_align_threshold
        ):
            self._state = self.STATE_PASS_THROUGH
            self._pass_through_start_time = None
            # Skip one Rate.sleep() here so we enter PASS_THROUGH right away instead of waiting a whole cycle.
            return True

        if abs(yaw_cmd) > self.align_angle_max:
            self._state = self.STATE_ALIGN
            omega = self.angular_gain * yaw_cmd
            if abs(omega) < self.min_turn_omega:
                omega = math.copysign(self.min_turn_omega, yaw_cmd)
            self._publish_cmd(self.align_linear_speed, self._compensate_right_turn(omega))
        else:
            self._state = self.STATE_APPROACH
            self._publish_cmd(self.linear_speed, self._compensate_right_turn(self.angular_gain * yaw_cmd))
        return False

    def run(self) -> None:
        while not rospy.is_shutdown() and not self._goal_done:
            if self._leg >= len(self.path):
                self._finish_goal()
                break

            target_node = self.path[self._leg]
            now = rospy.Time.now()

            if self._state == self.STATE_PASS_THROUGH:
                if self._tick_pass_through(target_node, now):
                    continue
            elif self._tick_vision_control(target_node, now):
                continue

            self._rate.sleep()

    def _advance_leg(self, reached_node: int) -> None:
        rospy.loginfo("Node N%d passed. Next leg.", reached_node)
        self._tag_metrics.pop(reached_node, None)
        self._detection_buffer, self._reach_buffer = {}, {}
        self._last_yaw_err = None
        self._leg += 1
        self._state = self.STATE_SEARCH
        if self._leg < len(self.path):
            self._search_sign = compute_search_turn_sign(self.path, self._leg)
        self._publish_cmd(0, 0)
        rospy.sleep(0.1)

    def _finish_goal(self) -> None:
        self._goal_done = True
        self._publish_cmd(0, 0)
        rospy.loginfo("Goal Reached")
        rospy.signal_shutdown("goal_reached")


def main() -> None:
    rospy.init_node("assignment3_navigator", anonymous=False)
    nav = Assignment3Navigator()
    nav.run()


if __name__ == "__main__":
    main()
