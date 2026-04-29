#!/usr/bin/env python3
"""
Navigator node (ROS): ArUco tags + OpenCV cv2 + A* path + Duckietown car_cmd.

Discrete step-based controller — every iteration of run() executes exactly ONE
atomic, open-loop, blocking action, then stops the motors and waits for a clean
camera frame. There is no continuous P-controller anywhere, so no single turn
can ever exceed `turn_step_deg` (default 30°).

Per-iteration decision (latest confirmed ArUco detection of path[_leg]):
    no detection                          -> TURN one step in A* search direction
    |yaw| > align_threshold               -> TURN one step toward the tag
    dist < proximity_threshold + aligned  -> DRIVE (dist - buffer) then advance leg
    aligned but far                       -> DRIVE forward_step_m

Between every action: stop motors and pause `step_pause_sec` so the next ArUco
frame is sharp + fresh. Camera callback keeps running during the pause.
"""
from __future__ import annotations

import math
import os
import sys
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage

# --- defaults (every value is overridable via ~param) ---
ROBOT_NAME_DEFAULT = "bear"
LINEAR_SPEED_DEFAULT = 0.18              # m/s for non-final forward steps
TURN_OMEGA_DEFAULT = 1.5                 # rad/s magnitude during a TURN step
MIN_TURN_OMEGA_DEFAULT = 0.75            # motor stiction floor
RIGHT_TURN_SCALE_DEFAULT = 2.2           # right-turn stiction compensation
MAX_ANGULAR_SPEED_DEFAULT = 3.0          # publisher clamp
TURN_STEP_DEG_DEFAULT = 30.0             # HARD CAP per TURN step
FORWARD_STEP_M_DEFAULT = 0.15            # meters per non-final forward step (small so drift is bounded per step)
STEP_PAUSE_SEC_DEFAULT = 1.0             # max halt time waiting for a fresh confirmed detection
PROXIMITY_THRESHOLD_DEFAULT = 0.45       # dist below which we PASS the tag
ALIGN_THRESHOLD_DEFAULT = 0.12           # |yaw| below which we consider aligned (tight = re-aligns often)
PASS_THROUGH_SPEED_DEFAULT = 0.18        # m/s for the final pass-through drive
PASS_THROUGH_DISTANCE_BUFFER_DEFAULT = 0.0  # meters: subtract from dist to compensate motor calibration
DETECTION_STALE_SEC_DEFAULT = 2.0        # drop ArUco readings older than this
YAW_BIAS_DEFAULT = 0.0                   # camera mounting yaw offset (rad)
ARUCO_TAG_SIZE_METERS_DEFAULT = 0.065
ARUCO_DICTIONARY_DEFAULT = "DICT_5X5_50"

# OpenCV pinhole camera intrinsics (calibration)
CAMERA_FX_DEFAULT = 270.4563591302591
CAMERA_FY_DEFAULT = 269.2951665378049
CAMERA_CX_DEFAULT = 314.1813567017415
CAMERA_CY_DEFAULT = 218.88618596346137
DIST_K1_DEFAULT = -0.19162991260105328
DIST_K2_DEFAULT = 0.026384790215657535
DIST_P1_DEFAULT = 0.005682129590129115
DIST_P2_DEFAULT = 0.0006647376545041703
DIST_K3_DEFAULT = 0.0

# Freshness is now enforced by _last_action_time (see _fresh_target).
# One detection whose timestamp is strictly after _last_action_time is enough to act on.

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

import astar


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _bearing_to_tag(tx: float, ty: float, tz: float) -> float:
    return math.atan2(-tx, tz)


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


class Assignment3Navigator:
    def __init__(self) -> None:
        self.robot_name = rospy.get_param("~robot_name", ROBOT_NAME_DEFAULT)
        self.linear_speed = float(rospy.get_param("~linear_speed", LINEAR_SPEED_DEFAULT))
        # accept both new (~turn_omega) and legacy (~search_angular_speed) param names
        self.turn_omega = float(rospy.get_param("~turn_omega",
                                rospy.get_param("~search_angular_speed", TURN_OMEGA_DEFAULT)))
        self.min_turn_omega = float(rospy.get_param("~min_turn_omega", MIN_TURN_OMEGA_DEFAULT))
        self.right_turn_scale = float(rospy.get_param("~right_turn_omega_scale", RIGHT_TURN_SCALE_DEFAULT))
        self.max_omega = float(rospy.get_param("~max_angular_speed", MAX_ANGULAR_SPEED_DEFAULT))
        self.turn_step_rad = math.radians(float(rospy.get_param("~turn_step_deg", TURN_STEP_DEG_DEFAULT)))
        self.forward_step_m = float(rospy.get_param("~forward_step_m", FORWARD_STEP_M_DEFAULT))
        self.step_pause_sec = float(rospy.get_param("~step_pause_sec", STEP_PAUSE_SEC_DEFAULT))
        self.proximity_threshold = float(rospy.get_param("~proximity_threshold", PROXIMITY_THRESHOLD_DEFAULT))
        # accept both ~align_threshold and legacy ~align_angle_max
        self.align_threshold = float(rospy.get_param("~align_threshold",
                                     rospy.get_param("~align_angle_max", ALIGN_THRESHOLD_DEFAULT)))
        self.pass_through_speed = float(rospy.get_param("~pass_through_speed", PASS_THROUGH_SPEED_DEFAULT))
        self.pass_through_distance_buffer = float(
            rospy.get_param("~pass_through_distance_buffer", PASS_THROUGH_DISTANCE_BUFFER_DEFAULT))
        self.detection_stale_sec = float(rospy.get_param("~detection_stale_sec", DETECTION_STALE_SEC_DEFAULT))
        self.yaw_bias = float(rospy.get_param("~yaw_bias", YAW_BIAS_DEFAULT))
        self.aruco_tag_size = float(rospy.get_param("~aruco_tag_size_meters", ARUCO_TAG_SIZE_METERS_DEFAULT))
        self.aruco_dictionary_name = str(rospy.get_param("~aruco_dictionary", ARUCO_DICTIONARY_DEFAULT))

        self.camera_matrix = np.array([
            [float(rospy.get_param("~camera_fx", CAMERA_FX_DEFAULT)), 0.0,
             float(rospy.get_param("~camera_cx", CAMERA_CX_DEFAULT))],
            [0.0, float(rospy.get_param("~camera_fy", CAMERA_FY_DEFAULT)),
             float(rospy.get_param("~camera_cy", CAMERA_CY_DEFAULT))],
            [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.array([
            [float(rospy.get_param("~dist_k1", DIST_K1_DEFAULT))],
            [float(rospy.get_param("~dist_k2", DIST_K2_DEFAULT))],
            [float(rospy.get_param("~dist_p1", DIST_P1_DEFAULT))],
            [float(rospy.get_param("~dist_p2", DIST_P2_DEFAULT))],
            [float(rospy.get_param("~dist_k3", DIST_K3_DEFAULT))]], dtype=np.float32)

        self.aruco_dictionary = _build_aruco_dictionary(self.aruco_dictionary_name)
        self.aruco_parameters = _build_detector_parameters()

        path, _cost = astar.astar_search(0, 15, verbose=True)
        if path is None:
            raise RuntimeError("A* found no path from N0 to N15.")
        self.path = path
        rospy.loginfo("A* path: %s", astar.format_path(self.path))
        self._leg = 1
        self._goal_done = False
        self._search_sign = self._compute_search_sign(self._leg)

        self._tag_metrics: Dict[int, Tuple[float, float, rospy.Time]] = {}
        # Timestamp of the most recent completed action. _fresh_target only
        # accepts detections whose stamp is strictly after this value, so we
        # never act on a reading that was captured before/during the last move.
        self._last_action_time: rospy.Time = rospy.Time(0)

        image_topic = rospy.get_param("~camera_image_topic",
                                      "/%s/camera_node/image/compressed" % self.robot_name)
        cmd_topic = rospy.get_param("~cmd_topic",
                                    "/%s/car_cmd_switch_node/cmd" % self.robot_name)
        self._sub = rospy.Subscriber(image_topic, CompressedImage, self._on_camera_image,
                                     queue_size=1, buff_size=2 ** 24)
        self._pub = rospy.Publisher(cmd_topic, Twist2DStamped, queue_size=1)
        self.debug_pub = rospy.Publisher("/%s/debug_image/compressed" % self.robot_name,
                                         CompressedImage, queue_size=1)

        self._rate = rospy.Rate(20.0)
        rospy.on_shutdown(self._on_shutdown)

        # don't enter run() until the camera node is actually publishing — otherwise the
        # first ~10–20s look like dead time as we wait for any image at all.
        rospy.loginfo("Waiting for first camera frame on %s ...", image_topic)
        try:
            rospy.wait_for_message(image_topic, CompressedImage, timeout=15.0)
            rospy.loginfo("Camera is alive.")
        except rospy.ROSException:
            rospy.logwarn("No camera frame after 15s — proceeding anyway.")

    # ------------------------------------------------------------------ A*
    def _compute_search_sign(self, leg_idx: int) -> float:
        """Left (+1) / right (-1) spin hint from the A* grid bend at path[leg_idx]."""
        if leg_idx <= 0 or leg_idx >= len(self.path):
            return 0.0
        cur = self.path[leg_idx - 1]
        nxt = self.path[leg_idx]
        nx = astar.COORDINATES[nxt][0] - astar.COORDINATES[cur][0]
        ny = astar.COORDINATES[nxt][1] - astar.COORDINATES[cur][1]
        if leg_idx == 1:
            return 0.3
        prev = self.path[leg_idx - 2]
        px = astar.COORDINATES[cur][0] - astar.COORDINATES[prev][0]
        py = astar.COORDINATES[cur][1] - astar.COORDINATES[prev][1]
        cross = px * ny - py * nx
        if cross > 0.1:
            return 1.0
        if cross < -0.1:
            return -1.0
        return 0.3

    # ------------------------------------------------------------- ArUco cb
    def _on_camera_image(self, msg: CompressedImage) -> None:
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        np_image = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_image, cv2.IMREAD_COLOR)
        if frame is None:
            return
        frame = cv2.resize(frame, (640, 480))
        corners, ids, _rejected = _detect_markers(frame, self.aruco_dictionary, self.aruco_parameters)
        if ids is not None and len(ids) > 0:
            _rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.aruco_tag_size, self.camera_matrix, self.dist_coeffs)
            for idx, tag_id in enumerate(ids.reshape(-1)):
                tag_id = int(tag_id)
                tx, ty, tz = [float(v) for v in tvecs[idx][0]]
                self._tag_metrics[tag_id] = (_norm3(tx, ty, tz), _bearing_to_tag(tx, ty, tz), stamp)
        self.debug_pub.publish(msg)

    # ----------------------------------------------------------- low-level
    def _compensate_right_turn(self, omega: float) -> float:
        # motor stiction: right-turn commands need extra magnitude (clamped later)
        return omega * self.right_turn_scale if omega < 0 else omega

    def _publish_cmd(self, v: float, omega: float) -> None:
        omega = max(-self.max_omega, min(self.max_omega, omega))
        m = Twist2DStamped()
        m.header.stamp = rospy.Time.now()
        m.v = float(v)
        m.omega = float(omega)
        self._pub.publish(m)

    def _on_shutdown(self) -> None:
        self._publish_cmd(0.0, 0.0)

    # ----------------------------------------------------------- atomic steps
    def _timed_publish(self, v: float, omega: float, duration_sec: float) -> None:
        end = rospy.Time.now() + rospy.Duration.from_sec(duration_sec)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            self._publish_cmd(v, omega)
            self._rate.sleep()

    def _turn_step(self, sign: float) -> None:
        """Open-loop rotate at most `turn_step_rad`. sign>0 left, sign<0 right.

        Real rotation rate <= clamped published omega (motor stiction can only slow
        it down), so duration = turn_step_rad / |published_omega| guarantees the
        active-rotation arc is <= turn_step_rad. (Coast after we send 0 may add a
        few extra degrees; lower turn_step_deg if you see overshoot.)
        """
        magnitude = max(abs(self.turn_omega), self.min_turn_omega)
        omega = math.copysign(magnitude, sign)
        omega_cmd = self._compensate_right_turn(omega)
        published_mag = min(abs(omega_cmd), self.max_omega)
        duration = self.turn_step_rad / max(published_mag, 1e-3)
        self._timed_publish(0.0, omega_cmd, duration)
        self._publish_cmd(0.0, 0.0)

    def _drive_step(self, meters: float, speed: float) -> None:
        """Open-loop straight drive of `meters` at `speed`."""
        if meters <= 0.0:
            return
        speed = max(abs(speed), 1e-3)
        self._timed_publish(speed, 0.0, meters / speed)
        self._publish_cmd(0.0, 0.0)

    # ------------------------------------------------------------- main loop
    def run(self) -> None:
        while not rospy.is_shutdown() and not self._goal_done:
            if self._leg >= len(self.path):
                self._finish_goal()
                return

            target = self.path[self._leg]

            # 1. Stop motors and wait for a fresh detection. (Don't update
            #    _last_action_time yet — we want to accept any detections that
            #    arrived during the previous action, which may have brought the
            #    next target into view.)
            self._publish_cmd(0.0, 0.0)

            # 2. Active wait: poll for up to step_pause_sec; return as soon as
            #    any fresh post-action detection is available.
            info = self._wait_for_target(target, self.step_pause_sec)
            if rospy.is_shutdown():
                return

            # 3. Pick & execute exactly ONE atomic action. Set _last_action_time
            #    RIGHT BEFORE the action so we exclude readings captured DURING
            #    the move, but accept readings from before we started moving.
            if info is None:
                sign = 1.0 if self._search_sign >= 0 else -1.0
                rospy.loginfo("[N%d] SEARCH turn %+.0f deg (no detection in %.1fs)",
                              target, math.degrees(self.turn_step_rad) * sign, self.step_pause_sec)
                self._last_action_time = rospy.Time.now()
                self._turn_step(sign)
                continue

            dist, yaw = info
            if abs(yaw) > self.align_threshold:
                sign = 1.0 if yaw > 0 else -1.0
                rospy.loginfo("[N%d] ALIGN turn %+.0f deg (yaw=%.2frad dist=%.2fm)",
                              target, math.degrees(self.turn_step_rad) * sign, yaw, dist)
                self._last_action_time = rospy.Time.now()
                self._turn_step(sign)
                continue

            if dist < self.proximity_threshold:
                meters = max(0.0, dist - self.pass_through_distance_buffer)
                rospy.loginfo("[N%d] PASS %.2fm (raw dist=%.2fm @ %.2fm/s)",
                              target, meters, dist, self.pass_through_speed)
                self._last_action_time = rospy.Time.now()
                self._drive_step(meters, self.pass_through_speed)
                self._advance_leg(target)
                continue

            rospy.loginfo("[N%d] APPROACH drive %.2fm (yaw=%.2frad dist=%.2fm)",
                          target, self.forward_step_m, yaw, dist)
            self._last_action_time = rospy.Time.now()
            self._drive_step(self.forward_step_m, self.linear_speed)

    def _wait_for_target(self, target_id: int, max_wait_sec: float) -> Optional[Tuple[float, float]]:
        """Hold the bot stopped and poll for a fresh confirmed detection of `target_id`.

        Returns (dist, yaw_with_bias) the moment _fresh_target succeeds, or None
        after `max_wait_sec` of no detection.
        """
        poll = rospy.Rate(20.0)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            info = self._fresh_target(target_id)
            if info is not None:
                return info
            if (rospy.Time.now() - start).to_sec() >= max_wait_sec:
                return None
            self._publish_cmd(0.0, 0.0)
            poll.sleep()
        return None

    # -------------------------------------------------------- helpers
    def _fresh_target(self, target_id: int) -> Optional[Tuple[float, float]]:
        """Latest (dist, yaw_with_bias) for `target_id` captured AFTER the last action started.

        A detection is accepted when its camera timestamp is strictly after
        _last_action_time (set right before each motion command) and within
        detection_stale_sec of now. This excludes readings captured during
        robot motion (blurry/unreliable) while accepting readings from before
        the action started or after it finished.
        """
        info = self._tag_metrics.get(target_id)
        if info is None:
            return None
        dist, yaw, stamp = info
        # Reject readings captured before or during the last action.
        if stamp <= self._last_action_time:
            return None
        if (rospy.Time.now() - stamp).to_sec() > self.detection_stale_sec:
            return None
        return dist, yaw + self.yaw_bias

    def _advance_leg(self, reached: int) -> None:
        rospy.loginfo("Node N%d passed. Next leg.", reached)
        self._tag_metrics.pop(reached, None)
        self._leg += 1
        if self._leg < len(self.path):
            self._search_sign = self._compute_search_sign(self._leg)
        self._publish_cmd(0.0, 0.0)

    def _finish_goal(self) -> None:
        self._goal_done = True
        self._publish_cmd(0.0, 0.0)
        rospy.loginfo("Goal Reached")
        rospy.signal_shutdown("goal_reached")


def main() -> None:
    rospy.init_node("assignment3_navigator", anonymous=False)
    nav = Assignment3Navigator()
    nav.run()


if __name__ == "__main__":
    main()
