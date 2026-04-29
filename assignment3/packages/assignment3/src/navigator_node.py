#!/usr/bin/env python3
"""
Navigator node (ROS): ArUco tags + OpenCV cv2 + A* path + Duckietown car_cmd.

Keywords for finding stuff in this file:
  - A* / astar: planned path from N0 to N15 (see import astar, astar_search, COORDINATES).
  - ArUco / cv2.aruco: marker dictionary, detectMarkers, estimatePoseSingleMarkers.
  - OpenCV / cv2: imdecode, resize, camera matrix + dist coeffs for pose.
  - ROS: rospy.get_param (~foo), Subscriber/Publisher, CompressedImage, Twist2DStamped.
  - SEARCH / ALIGN / APPROACH / PASS_THROUGH: state machine in run().

Per-leg state machine (one leg = drive toward path[_leg] tag ID):
    SEARCH       - ArUco target not reliable: spin (direction from A* grid geometry).
    ALIGN        - tag visible, big yaw error: turn in place (small v allowed).
    APPROACH     - tag visible, small yaw: drive forward + P yaw correction.
    PASS_THROUGH - node considered reached: straight v, omega=0, then next leg.
"""
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

ROBOT_NAME_DEFAULT = "bear"

LINEAR_SPEED_DEFAULT = 0.18
ANGULAR_GAIN_DEFAULT = 0.9          # control: P gain on yaw error (ALIGN/APPROACH)
PROXIMITY_THRESHOLD_DEFAULT = 0.45   # meters: "close" to tag for reach / pass-through logic
ALIGN_ANGLE_MAX_DEFAULT = 0.20      # rad (~11°): above this |yaw| we use ALIGN not APPROACH
SEARCH_ANGULAR_SPEED_DEFAULT = 1.3  # SEARCH state: spin rate magnitude
SEARCH_LINEAR_SPEED_DEFAULT = 0.0   # SEARCH: 0 = pure spin; nonzero = crawl while searching
SEARCH_TURN_STEP_DEG_DEFAULT = 30.0 # SEARCH: degrees per discrete turn before pausing
SEARCH_WAIT_SEC_DEFAULT = 1.0       # SEARCH: pause duration between discrete turns (still detects)
ALIGN_LINEAR_SPEED_DEFAULT = 0.0    # ALIGN: forward creep while turning (often 0)
MAX_ANGULAR_SPEED_DEFAULT = 3.0     # clamp |omega| on every cmd
DETECTION_STALE_SEC_DEFAULT = 1.0   # ArUco: drop measurements older than this
WAIT_LOG_PERIOD_SEC_DEFAULT = 2.0
YAW_BIAS_DEFAULT = 0.0              # fixed rad offset if camera frame is biased
CMD_LOG_PERIOD_SEC_DEFAULT = 1.0
LOST_COAST_SEC_DEFAULT = 0.35
PASS_THROUGH_TIME_DEFAULT = 1.5     # seconds of straight drive after declaring node reached
PASS_THROUGH_SPEED_DEFAULT = 0.18
PASS_THROUGH_ALIGN_THRESHOLD_DEFAULT = 0.10  # rad: must be this straight to enter PASS_THROUGH

# motors: minimum |omega| so the duckie actually turns (dead zone / stall)
MIN_TURN_OMEGA_DEFAULT = 0.75

# hardware tweak: if right turn is weak, scale negative omega (right turn) up
RIGHT_TURN_OMEGA_SCALE_DEFAULT = 1.6
SEARCH_RIGHT_SCALE_DEFAULT = 2.2

# ArUco: real edge length of printed tag (meters) + which cv2.aruco dictionary name
ARUCO_TAG_SIZE_METERS_DEFAULT = 0.065
ARUCO_DICTIONARY_DEFAULT = "DICT_5X5_50"

# OpenCV pinhole camera intrinsics (calibration) — used by estimatePoseSingleMarkers
CAMERA_FX_DEFAULT = 270.4563591302591
CAMERA_FY_DEFAULT = 269.2951665378049
CAMERA_CX_DEFAULT = 314.1813567017415
CAMERA_CY_DEFAULT = 218.88618596346137

DIST_K1_DEFAULT = -0.19162991260105328
DIST_K2_DEFAULT =  0.026384790215657535
DIST_P1_DEFAULT =  0.005682129590129115
DIST_P2_DEFAULT =  0.0006647376545041703
DIST_K3_DEFAULT =  0.0

# ArUco debounce: consecutive frames before we trust a tag / "reached" counts
CONFIRM_FRAMES = 2
REACH_CONFIRM_FRAMES = 3

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

import astar  # A* graph: GRAPH, COORDINATES, astar_search — see astar.py

def _norm3(x: float, y: float, z: float) -> float:
    # ArUco pose: distance to tag from tvec (camera frame)
    return math.sqrt(x * x + y * y + z * z)

def _bearing_to_tag(tx: float, ty: float, tz: float) -> float:
    """ArUco / geometry: yaw error toward tag in camera frame (atan2(-tx, tz))."""
    return math.atan2(-tx, tz)

def _build_aruco_dictionary(name: str):
    # cv2.aruco: map string e.g. DICT_5X5_50 to OpenCV dictionary object
    attr = getattr(cv2.aruco, name, None)
    if attr is None:
        raise ValueError("Unsupported ArUco dictionary '%s'" % name)
    return cv2.aruco.getPredefinedDictionary(attr)

def _build_detector_parameters():
    # cv2.aruco: OpenCV 4.x vs older API for detector settings
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()

def _detect_markers(image, dictionary, parameters):
    # cv2.aruco: ArucoDetector (new) vs detectMarkers (old)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers(image)
    return cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)

class Assignment3Navigator:
    STATE_SEARCH = "SEARCH"
    STATE_ALIGN = "ALIGN"
    STATE_APPROACH = "APPROACH"
    STATE_PASS_THROUGH = "PASS_THROUGH"

    def __init__(self) -> None:
        # ROS params: rospy.get_param("~name", default) — launch file can override
        self.robot_name = rospy.get_param("~robot_name", ROBOT_NAME_DEFAULT)
        self.linear_speed = float(rospy.get_param("~linear_speed", LINEAR_SPEED_DEFAULT))
        self.angular_gain = float(rospy.get_param("~angular_gain", ANGULAR_GAIN_DEFAULT))
        self.proximity_threshold = float(rospy.get_param("~proximity_threshold", PROXIMITY_THRESHOLD_DEFAULT))
        self.align_angle_max = float(rospy.get_param("~align_angle_max", ALIGN_ANGLE_MAX_DEFAULT))
        self.search_omega = float(rospy.get_param("~search_angular_speed", SEARCH_ANGULAR_SPEED_DEFAULT))
        self.search_linear_speed = float(rospy.get_param("~search_linear_speed", SEARCH_LINEAR_SPEED_DEFAULT))
        self.search_turn_step_rad = math.radians(float(rospy.get_param("~search_turn_step_deg", SEARCH_TURN_STEP_DEG_DEFAULT)))
        self.search_wait_sec = float(rospy.get_param("~search_wait_sec", SEARCH_WAIT_SEC_DEFAULT))
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
        self.pass_through_align_threshold = float(rospy.get_param("~pass_through_align_threshold", PASS_THROUGH_ALIGN_THRESHOLD_DEFAULT))
        self.aruco_tag_size = float(rospy.get_param("~aruco_tag_size_meters", ARUCO_TAG_SIZE_METERS_DEFAULT))
        self.aruco_dictionary_name = str(rospy.get_param("~aruco_dictionary", ARUCO_DICTIONARY_DEFAULT))

        # OpenCV 3x3 camera matrix K (fx, fy, cx, cy) from ~camera_fx etc.
        self.camera_matrix = np.array([
            [float(rospy.get_param("~camera_fx", CAMERA_FX_DEFAULT)), 0.0, float(rospy.get_param("~camera_cx", CAMERA_CX_DEFAULT))],
            [0.0, float(rospy.get_param("~camera_fy", CAMERA_FY_DEFAULT)), float(rospy.get_param("~camera_cy", CAMERA_CY_DEFAULT))],
            [0.0, 0.0, 1.0]], dtype=np.float32)

        # OpenCV distortion coeffs for estimatePoseSingleMarkers (~dist_k1, ...)
        self.dist_coeffs = np.array([
            [float(rospy.get_param("~dist_k1", DIST_K1_DEFAULT))],
            [float(rospy.get_param("~dist_k2", DIST_K2_DEFAULT))],
            [float(rospy.get_param("~dist_p1", DIST_P1_DEFAULT))],
            [float(rospy.get_param("~dist_p2", DIST_P2_DEFAULT))],
            [float(rospy.get_param("~dist_k3", DIST_K3_DEFAULT))]], dtype=np.float32)

        # cv2.aruco: dictionary + detector params (built once)
        self.aruco_dictionary = _build_aruco_dictionary(self.aruco_dictionary_name)
        self.aruco_parameters = _build_detector_parameters()

        # A* path planning: astar.astar_search(start, goal) — nodes N0..N15, see astar.GRAPH / COORDINATES
        path, cost = astar.astar_search(0, 15, verbose=True)
        if path is None:
            rospy.logfatal("A* found no path from N0 to N15.")
            raise RuntimeError("A* found no path from N0 to N15.")
        self.path: List[int] = path
        self._path_cost: float = cost

        rospy.loginfo("A* path: %s", astar.format_path(self.path))
        self._leg = 1  # index into A* path: current target node ID = path[_leg]
        self._goal_done = False
        self._state = self.STATE_SEARCH

        # ArUco bookkeeping: latest (dist, yaw, stamp) per tag id; buffers = debounce counters
        self._tag_metrics: Dict[int, Tuple[float, float, rospy.Time]] = {}
        self._last_camera_msg_time: Optional[rospy.Time] = None
        self._detection_buffer: Dict[int, int] = {}
        self._reach_buffer: Dict[int, int] = {}
        # SEARCH helper: +1 / -1 spin hint from A* polyline on grid (astar.COORDINATES)
        self._search_sign = self._compute_search_sign(self._leg)
        self._last_yaw_err: Optional[float] = None
        self._pass_through_start_time: Optional[rospy.Time] = None
        # PASS_THROUGH: distance-based drive duration = last_dist / pass_through_speed
        self._pass_through_duration: Optional[float] = None
        # SEARCH: discrete-turn sub-state ("TURN" -> spin 30°, "WAIT" -> hold 1s & detect)
        self._search_phase: str = "TURN"
        self._search_phase_start: Optional[rospy.Time] = None
        
        # ROS subscriber: CompressedImage from duckie camera (JPEG bytes -> cv2)
        image_topic = rospy.get_param("~camera_image_topic", "/%s/camera_node/image/compressed" % self.robot_name)
        self._sub = rospy.Subscriber(image_topic, CompressedImage, self._on_camera_image, queue_size=1, buff_size=2 ** 24)

        # ROS publishers: car_cmd (Twist2DStamped v, omega) + optional debug image topic
        cmd_topic = rospy.get_param("~cmd_topic", "/%s/car_cmd_switch_node/cmd" % self.robot_name)
        self._pub = rospy.Publisher(cmd_topic, Twist2DStamped, queue_size=1)
        self.debug_pub = rospy.Publisher("/%s/debug_image/compressed" % self.robot_name, CompressedImage, queue_size=1)

        # Control loop rate (callback only updates ArUco; run() sends cmds at this rate)
        self._rate = rospy.Rate(20.0)
        rospy.on_shutdown(self._on_shutdown)
        rospy.sleep(0.3)

    def _compute_search_sign(self, leg_idx: int) -> float:
        """SEARCH state / A* geometry: left (+1) vs right (-1) spin from grid turn at path[leg_idx]."""
        if leg_idx <= 0 or leg_idx >= len(self.path): return 0.0
        current_node = self.path[leg_idx - 1]
        target_node = self.path[leg_idx]
        
        nx = astar.COORDINATES[target_node][0] - astar.COORDINATES[current_node][0]
        ny = astar.COORDINATES[target_node][1] - astar.COORDINATES[current_node][1]
        
        if leg_idx == 1: return 0.3  # first edge tweak: mild bias we liked at start

        prev_node = self.path[leg_idx - 2]
        px = astar.COORDINATES[current_node][0] - astar.COORDINATES[prev_node][0]
        py = astar.COORDINATES[current_node][1] - astar.COORDINATES[prev_node][1]
        
        cross = px * ny - py * nx  # 2D cross: sign = bend direction on A* map
        if cross > 0.1: return 1.0   # grid says bend left -> positive omega sign convention
        if cross < -0.1: return -1.0  # bend right
        return 0.3

    def _draw_debug_overlay(self, frame, corners, ids, tvecs) -> np.ndarray:
        # OpenCV: optional HUD (not used in main publish path unless you switch to imencode)
        debug = frame.copy()
        target_node = self.path[self._leg] if self._leg < len(self.path) else -1
        cv2.putText(debug, f"TARGET: N{target_node}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(debug, f"STATE: {self._state}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        return debug

    def _on_camera_image(self, msg: CompressedImage) -> None:
        # ROS + OpenCV: JPEG from msg.data -> BGR image for cv2.aruco
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        self._last_camera_msg_time = stamp
        np_image = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_image, cv2.IMREAD_COLOR)
        if frame is None: return

        frame = cv2.resize(frame, (640, 480))
        # ArUco detect: corners, ids per frame
        corners, ids, _rejected = _detect_markers(frame, self.aruco_dictionary, self.aruco_parameters)

        metrics: Dict[int, Tuple[float, float, rospy.Time]] = {}
        if ids is not None and len(ids) > 0:
            # ArUco pose: tvec in camera frame (needs tag size + K + dist)
            _rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_tag_size, self.camera_matrix, self.dist_coeffs)
            flat_ids = ids.reshape(-1)
            for idx, tag_id in enumerate(flat_ids):
                tx, ty, tz = [float(v) for v in tvecs[idx][0]]
                dist = _norm3(tx, ty, tz)
                yaw_off = _bearing_to_tag(tx, ty, tz)
                metrics[int(tag_id)] = (dist, yaw_off, stamp)
        
        # ArUco debounce: CONFIRM_FRAMES / REACH_CONFIRM_FRAMES (per-tag counters)
        target_node = self.path[self._leg] if self._leg < len(self.path) else -1
        for tag_id, (dist, _yaw, _st) in metrics.items():
            self._detection_buffer[tag_id] = self._detection_buffer.get(tag_id, 0) + 1
            if tag_id == target_node and dist < self.proximity_threshold:
                self._reach_buffer[tag_id] = self._reach_buffer.get(tag_id, 0) + 1
            elif tag_id == target_node:
                self._reach_buffer[tag_id] = 0

        for tag_id, data in metrics.items():
            self._tag_metrics[tag_id] = data

        # Debug: still forwarding original compressed msg (could swap to overlay + cv2.imencode)
        self.debug_pub.publish(msg)

    def _compensate_right_turn(self, omega: float, *, search: bool = False) -> float:
        # motor asymmetry: strengthen negative omega (right turn) in ALIGN/APPROACH or SEARCH
        if omega >= 0: return omega
        scale = self.search_right_scale if search else self.right_turn_scale
        return omega * float(scale)

    def _publish_cmd(self, v: float, omega: float) -> None:
        # ROS: Duckietown Twist2DStamped on car_cmd topic
        omega = max(-self.max_omega, min(self.max_omega, omega))
        m = Twist2DStamped()
        m.header.stamp = rospy.Time.now()
        m.v = float(v)
        m.omega = float(omega)
        self._pub.publish(m)

    def _on_shutdown(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def run(self) -> None:
        # Main loop: state machine SEARCH / ALIGN / APPROACH / PASS_THROUGH (see module docstring)
        while not rospy.is_shutdown() and not self._goal_done:
            if self._leg >= len(self.path):
                self._finish_goal()
                break

            target_node = self.path[self._leg]
            now = rospy.Time.now()
            
            # PASS_THROUGH: distance-based straight segment (omega=0), then A* leg advance
            # duration = measured_distance_to_tag / pass_through_speed (captured on transition)
            if self._state == self.STATE_PASS_THROUGH:
                if self._pass_through_start_time is None: self._pass_through_start_time = now
                duration = self._pass_through_duration if self._pass_through_duration is not None else self.pass_through_time
                if (now - self._pass_through_start_time).to_sec() >= duration:
                    self._advance_leg(target_node)
                    continue
                else:
                    self._publish_cmd(self.pass_through_speed, 0.0)
                    self._rate.sleep()
                    continue
            
            # Vision ok: fresh ArUco on target_node + CONFIRM_FRAMES met
            info = self._tag_metrics.get(target_node)
            if info and (now - info[2]).to_sec() < self.detection_stale_sec and self._detection_buffer.get(target_node, 0) >= CONFIRM_FRAMES:
                dist, yaw_err = info[0], info[1]
                self._last_yaw_err = yaw_err
                yaw_cmd = yaw_err + self.yaw_bias

                # transition to PASS_THROUGH when close + REACH_CONFIRM_FRAMES + aligned
                if dist < self.proximity_threshold and self._reach_buffer.get(target_node, 0) >= REACH_CONFIRM_FRAMES and abs(yaw_cmd) < self.pass_through_align_threshold:
                    self._state = self.STATE_PASS_THROUGH
                    self._pass_through_start_time = None
                    # latest measured tag distance / forward speed -> seconds of straight drive
                    speed = max(abs(self.pass_through_speed), 1e-3)
                    self._pass_through_duration = float(dist) / speed
                    rospy.loginfo("PASS_THROUGH N%d: dist=%.2fm v=%.2fm/s -> %.2fs",
                                  target_node, dist, self.pass_through_speed, self._pass_through_duration)
                    continue

                # ALIGN: large |yaw_cmd| -> mostly rotate
                if abs(yaw_cmd) > self.align_angle_max:
                    self._state = self.STATE_ALIGN
                    omega = self.angular_gain * yaw_cmd
                    if abs(omega) < self.min_turn_omega: omega = math.copysign(self.min_turn_omega, yaw_cmd)
                    self._publish_cmd(self.align_linear_speed, self._compensate_right_turn(omega))
                else:
                    # APPROACH: drive forward + P yaw (nonzero omega unless perfectly centered)
                    self._state = self.STATE_APPROACH
                    self._publish_cmd(self.linear_speed, self._compensate_right_turn(self.angular_gain * yaw_cmd))
            else:
                # SEARCH: no good ArUco on target — discrete 30° turn, then 1s wait (still detecting)
                if self._state != self.STATE_SEARCH:
                    self._search_phase = "TURN"
                    self._search_phase_start = None
                self._state = self.STATE_SEARCH

                magnitude = max(abs(self.search_omega), self.min_turn_omega)
                if self._last_yaw_err: omega = math.copysign(magnitude, self._last_yaw_err)
                else: omega = math.copysign(magnitude, self._search_sign)
                omega_cmd = self._compensate_right_turn(omega, search=True)

                if self._search_phase_start is None:
                    self._search_phase_start = now
                elapsed = (now - self._search_phase_start).to_sec()

                if self._search_phase == "TURN":
                    # duration sized so the integrated rotation ≈ search_turn_step_rad
                    turn_duration = self.search_turn_step_rad / max(abs(omega_cmd), 1e-3)
                    if elapsed >= turn_duration:
                        self._search_phase = "WAIT"
                        self._search_phase_start = now
                        self._publish_cmd(0.0, 0.0)
                    else:
                        self._publish_cmd(self.search_linear_speed, omega_cmd)
                else:  # WAIT: hold still for ~search_wait_sec; ArUco callback keeps scanning
                    if elapsed >= self.search_wait_sec:
                        self._search_phase = "TURN"
                        self._search_phase_start = now
                        self._publish_cmd(self.search_linear_speed, omega_cmd)
                    else:
                        self._publish_cmd(0.0, 0.0)

            self._rate.sleep()

    def _advance_leg(self, reached_node: int) -> None:
        # A* path: finished leg to reached_node — clear ArUco buffers, next target path[_leg]
        rospy.loginfo(f"Node N{reached_node} passed. Next leg.")
        self._tag_metrics.pop(reached_node, None)
        self._detection_buffer, self._reach_buffer = {}, {}
        self._last_yaw_err = None
        self._leg += 1
        self._state = self.STATE_SEARCH
        self._search_phase = "TURN"
        self._search_phase_start = None
        self._pass_through_duration = None
        if self._leg < len(self.path): self._search_sign = self._compute_search_sign(self._leg)
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
