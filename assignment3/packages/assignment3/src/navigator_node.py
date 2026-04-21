#!/usr/bin/env python3
"""
ROS navigator: ArUco-only localization, A* path following, car_cmd_switch control
with debug image publishing.

Control logic is a small state machine per A* leg:

    SEARCH       - target tag not visible: continuously rotate in place in
                   the direction predicted from the A* grid geometry
                   (left/right turn).
    ALIGN        - target tag visible but yaw error is large: turn in place
                   (with a small forward creep) until bearing is good.
    APPROACH     - target tag visible and bearing small: drive forward with
                   proportional yaw correction.
    PASS_THROUGH - target reached: drive straight forward through the node
                   for a fixed duration before advancing to next leg.
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

# -----------------------------------------------------------------------------
# Defaults (overridden by ROS params under ~)
# -----------------------------------------------------------------------------
ROBOT_NAME_DEFAULT = "bear"

LINEAR_SPEED_DEFAULT = 0.18
ANGULAR_GAIN_DEFAULT = 0.9          # slightly higher to actually align at nodes
PROXIMITY_THRESHOLD_DEFAULT = 0.45
ALIGN_ANGLE_MAX_DEFAULT = 0.20      # rad (~11°)
SEARCH_ANGULAR_SPEED_DEFAULT = 0.9  # rad/s during SEARCH (in-place rotation)
SEARCH_LINEAR_SPEED_DEFAULT = 0.0   # in-place search, no forward drift
ALIGN_LINEAR_SPEED_DEFAULT = 0.0    # pure in-place rotation during ALIGN (safer for weak motors)
MAX_ANGULAR_SPEED_DEFAULT = 1.6
DETECTION_STALE_SEC_DEFAULT = 1.0
WAIT_LOG_PERIOD_SEC_DEFAULT = 2.0
YAW_BIAS_DEFAULT = 0.0              # calibrated from the robot's camera
CMD_LOG_PERIOD_SEC_DEFAULT = 1.0
LOST_COAST_SEC_DEFAULT = 0.35       # keep last command briefly on momentary loss
PASS_THROUGH_TIME_DEFAULT = 1.5     # seconds to drive through node after reaching it
PASS_THROUGH_SPEED_DEFAULT = 0.18   # forward speed during pass-through
PASS_THROUGH_ALIGN_THRESHOLD_DEFAULT = 0.10  # max yaw error (rad) to enter PASS_THROUGH

# Minimum angular velocity magnitude when ALIGN/SEARCH is rotating in place.
# Below this the Duckiebot motors often stall (PWM below deadband), which
# manifests as "robot turns left OK but never completes a right turn".
MIN_TURN_OMEGA_DEFAULT = 0.75

# Asymmetric compensation for robots whose right-side motor is weaker than the
# left-side motor (very common with default trim). Values > 1.0 amplify the
# magnitude of negative omega (right turns). Does not affect left turns.
RIGHT_TURN_OMEGA_SCALE_DEFAULT = 1.6
# Same idea but only for the SEARCH state (pure rotation, so the effect is
# more sensitive to motor deadband).
SEARCH_RIGHT_SCALE_DEFAULT = 1.6

ARUCO_TAG_SIZE_METERS_DEFAULT = 0.065
ARUCO_DICTIONARY_DEFAULT = "DICT_5X5_50"

# Calibration defaults (bear.yaml)
CAMERA_FX_DEFAULT = 270.4563591302591
CAMERA_FY_DEFAULT = 269.2951665378049
CAMERA_CX_DEFAULT = 314.1813567017415
CAMERA_CY_DEFAULT = 218.88618596346137

DIST_K1_DEFAULT = -0.19162991260105328
DIST_K2_DEFAULT =  0.026384790215657535
DIST_P1_DEFAULT =  0.005682129590129115
DIST_P2_DEFAULT =  0.0006647376545041703
DIST_K3_DEFAULT =  0.0

# Number of frames the target tag must be present before we trust it.
CONFIRM_FRAMES = 2
# Frames the target tag must stay within proximity_threshold to accept "reached".
REACH_CONFIRM_FRAMES = 3

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

import astar  # noqa: E402


def _norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _bearing_to_tag(tx: float, ty: float, tz: float) -> float:
    """Heading error from robot to tag, expressed in robot-base convention.

    Camera optical frame: +x right, +y down, +z forward.
    Robot base frame (REP-103): +x forward, +y left, +z up, omega>0 -> CCW/left.

    A tag to the robot's right has camera tx > 0, which corresponds to base
    y < 0. So bearing = atan2(y_base, x_base) = atan2(-tx, tz).

    With this definition a *positive* yaw error means "target is on my left",
    which pairs directly with the standard proportional controller
    ``omega = k * yaw_err`` (omega > 0 -> turn left -> tag ends up centered).
    """
    return math.atan2(-tx, tz)


# Back-compat alias
_bearing_xz = _bearing_to_tag


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
    STATE_SEARCH = "SEARCH"
    STATE_ALIGN = "ALIGN"
    STATE_APPROACH = "APPROACH"
    STATE_PASS_THROUGH = "PASS_THROUGH"

    def __init__(self) -> None:
        self.robot_name = rospy.get_param("~robot_name", ROBOT_NAME_DEFAULT)
        self.linear_speed = float(rospy.get_param("~linear_speed", LINEAR_SPEED_DEFAULT))
        self.angular_gain = float(rospy.get_param("~angular_gain", ANGULAR_GAIN_DEFAULT))
        self.proximity_threshold = float(
            rospy.get_param("~proximity_threshold", PROXIMITY_THRESHOLD_DEFAULT)
        )
        self.align_angle_max = float(
            rospy.get_param("~align_angle_max", ALIGN_ANGLE_MAX_DEFAULT)
        )
        self.search_omega = float(
            rospy.get_param("~search_angular_speed", SEARCH_ANGULAR_SPEED_DEFAULT)
        )
        self.search_linear_speed = float(
            rospy.get_param("~search_linear_speed", SEARCH_LINEAR_SPEED_DEFAULT)
        )
        self.align_linear_speed = float(
            rospy.get_param("~align_linear_speed", ALIGN_LINEAR_SPEED_DEFAULT)
        )
        self.max_omega = float(
            rospy.get_param("~max_angular_speed", MAX_ANGULAR_SPEED_DEFAULT)
        )
        self.detection_stale_sec = float(
            rospy.get_param("~detection_stale_sec", DETECTION_STALE_SEC_DEFAULT)
        )
        self.wait_log_period_sec = float(
            rospy.get_param("~wait_log_period_sec", WAIT_LOG_PERIOD_SEC_DEFAULT)
        )
        self.cmd_log_period_sec = float(
            rospy.get_param("~cmd_log_period_sec", CMD_LOG_PERIOD_SEC_DEFAULT)
        )
        self.lost_coast_sec = float(
            rospy.get_param("~lost_coast_sec", LOST_COAST_SEC_DEFAULT)
        )
        self.min_turn_omega = float(
            rospy.get_param("~min_turn_omega", MIN_TURN_OMEGA_DEFAULT)
        )
        self.right_turn_scale = float(
            rospy.get_param("~right_turn_omega_scale", RIGHT_TURN_OMEGA_SCALE_DEFAULT)
        )
        self.search_right_scale = float(
            rospy.get_param("~search_right_scale", SEARCH_RIGHT_SCALE_DEFAULT)
        )
        self.yaw_bias = float(rospy.get_param("~yaw_bias", YAW_BIAS_DEFAULT))
        self.pass_through_time = float(
            rospy.get_param("~pass_through_time", PASS_THROUGH_TIME_DEFAULT)
        )
        self.pass_through_speed = float(
            rospy.get_param("~pass_through_speed", PASS_THROUGH_SPEED_DEFAULT)
        )
        self.pass_through_align_threshold = float(
            rospy.get_param("~pass_through_align_threshold", PASS_THROUGH_ALIGN_THRESHOLD_DEFAULT)
        )
        self.aruco_tag_size = float(
            rospy.get_param("~aruco_tag_size_meters", ARUCO_TAG_SIZE_METERS_DEFAULT)
        )
        self.aruco_dictionary_name = str(
            rospy.get_param("~aruco_dictionary", ARUCO_DICTIONARY_DEFAULT)
        )

        self.camera_matrix = np.array(
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

        self.aruco_dictionary = _build_aruco_dictionary(self.aruco_dictionary_name)
        self.aruco_parameters = _build_detector_parameters()

        # Run A* with verbose expansion log (prints g, h, f for each expanded node)
        path, cost = astar.astar_search(0, 15, verbose=True)
        if path is None:
            rospy.logfatal("A* found no path from N0 to N15.")
            raise RuntimeError("A* found no path from N0 to N15.")
        self.path: List[int] = path
        self._path_cost: float = cost

        rospy.loginfo("A* path: %s", astar.format_path(self.path))
        rospy.loginfo("A* total cost: %.4f", cost)
        print("\nPath sequence : %s" % astar.format_path(self.path))
        print("Total cost    : %.4f\n" % cost)

        self._leg = 1
        self._goal_done = False
        self._state = self.STATE_SEARCH

        # Latest per-tag metrics: tag_id -> (distance, yaw_off, stamp)
        self._tag_metrics: Dict[int, Tuple[float, float, rospy.Time]] = {}
        self._last_camera_msg_time: Optional[rospy.Time] = None

        # Per-frame confirmation counters
        self._detection_buffer: Dict[int, int] = {}
        self._reach_buffer: Dict[int, int] = {}

        # Search/approach state
        self._search_sign = self._compute_search_sign(self._leg)
        self._last_yaw_err: Optional[float] = None
        self._last_info_time: Optional[rospy.Time] = None
        
        # Pass-through state
        self._pass_through_start_time: Optional[rospy.Time] = None
        
        image_topic = rospy.get_param(
            "~camera_image_topic",
            "/%s/camera_node/image/compressed" % self.robot_name,
        )
        self._sub = rospy.Subscriber(
            image_topic,
            CompressedImage,
            self._on_camera_image,
            queue_size=1,
            buff_size=2 ** 24,
        )

        cmd_topic = rospy.get_param(
            "~cmd_topic",
            "/%s/car_cmd_switch_node/cmd" % self.robot_name,
        )
        self.cmd_topic = cmd_topic
        self._pub = rospy.Publisher(cmd_topic, Twist2DStamped, queue_size=1)

        self.debug_pub = rospy.Publisher(
            "/%s/debug_image/compressed" % self.robot_name,
            CompressedImage,
            queue_size=1,
        )

        self._rate = rospy.Rate(20.0)
        rospy.on_shutdown(self._on_shutdown)
        rospy.sleep(0.3)

        rospy.loginfo("Subscribing: %s", image_topic)
        rospy.loginfo(
            "Using ArUco dictionary %s, tag size %.3f m",
            self.aruco_dictionary_name,
            self.aruco_tag_size,
        )
        rospy.loginfo("Publishing: %s (Twist2DStamped: v, omega)", cmd_topic)
        rospy.loginfo(
            "Publishing debug image: /%s/debug_image/compressed", self.robot_name
        )
        rospy.loginfo(
            "Camera matrix: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            self.camera_matrix[0, 0],
            self.camera_matrix[1, 1],
            self.camera_matrix[0, 2],
            self.camera_matrix[1, 2],
        )

    # ------------------------------------------------------------------
    # Search direction from path geometry
    # ------------------------------------------------------------------
    def _compute_search_sign(self, leg_idx: int) -> float:
        """Return the expected turn direction at the start of this leg.

        We use the cross product between the previous edge vector and the
        next edge vector in grid coordinates:
            prev x next > 0  -> left turn  -> omega > 0 (CCW)
            prev x next < 0  -> right turn -> omega < 0 (CW)
            prev x next == 0 -> straight ahead
        For the very first leg we have no prior edge; we bias slightly right,
        because the robot is placed at N0 looking toward N1 in our map.
        """
        if leg_idx <= 0 or leg_idx >= len(self.path):
            return 0.0
        current_node = self.path[leg_idx - 1]
        target_node = self.path[leg_idx]
        nx = (
            astar.COORDINATES[target_node][0]
            - astar.COORDINATES[current_node][0]
        )
        ny = (
            astar.COORDINATES[target_node][1]
            - astar.COORDINATES[current_node][1]
        )
        if leg_idx == 1:
            # Robot is assumed to start facing N1: tag is straight ahead, so
            # no strong search bias. Use a small positive value for drift.
            return 0.3
        prev_node = self.path[leg_idx - 2]
        px = (
            astar.COORDINATES[current_node][0]
            - astar.COORDINATES[prev_node][0]
        )
        py = (
            astar.COORDINATES[current_node][1]
            - astar.COORDINATES[prev_node][1]
        )
        cross = px * ny - py * nx
        if cross > 0.1:
            return 1.0   # left turn
        if cross < -0.1:
            return -1.0  # right turn
        return 0.3       # straight; gentle default

    # ------------------------------------------------------------------
    # Debug overlay
    # ------------------------------------------------------------------
    def _draw_debug_overlay(self, frame, corners, ids, tvecs) -> np.ndarray:
        debug = frame.copy()
        target_node = self.path[self._leg] if self._leg < len(self.path) else -1

        cv2.putText(
            debug,
            f"TARGET: N{target_node}" if target_node >= 0 else "TARGET: DONE",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 0),
            2,
        )
        cv2.putText(
            debug,
            f"LEG: {self._leg}/{len(self.path)-1}  STATE: {self._state}",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
        )

        if ids is None or len(ids) == 0:
            return debug

        flat_ids = ids.reshape(-1)
        for i, tag_id in enumerate(flat_ids):
            pts = corners[i][0].astype(int)
            center_x = int(np.mean(pts[:, 0]))
            center_y = int(np.mean(pts[:, 1]))

            tx, ty, tz = [float(v) for v in tvecs[i][0]]
            dist = _norm3(tx, ty, tz)
            yaw = _bearing_to_tag(tx, ty, tz)

            color = (0, 255, 0)
            if int(tag_id) == target_node:
                color = (0, 0, 255)

            cv2.polylines(debug, [pts], True, color, 2)

            conf = self._detection_buffer.get(int(tag_id), 0)
            label = f"ID:{int(tag_id)} d:{dist:.2f} y:{yaw:.2f} c:{conf}"
            cv2.putText(
                debug,
                label,
                (center_x - 60, max(20, center_y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                2,
            )

            arrow_len = 40
            # yaw>0 => target to robot's left => arrow points left on the image
            end_x = int(center_x - arrow_len * math.sin(yaw))
            end_y = center_y
            cv2.arrowedLine(
                debug,
                (center_x, center_y),
                (end_x, end_y),
                (255, 255, 255),
                2,
                tipLength=0.25,
            )

        return debug

    def _publish_debug_image(self, frame: np.ndarray) -> None:
        try:
            ok, encoded = cv2.imencode(".jpg", frame)
            if not ok:
                return
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            self.debug_pub.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(
                self.wait_log_period_sec,
                "Failed to publish debug image: %s",
                str(e),
            )

    # ------------------------------------------------------------------
    # Camera callback
    # ------------------------------------------------------------------
    def _on_camera_image(self, msg: CompressedImage) -> None:
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        self._last_camera_msg_time = stamp

        np_image = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_image, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn_throttle(
                self.wait_log_period_sec,
                "Failed to decode compressed camera image for robot '%s'.",
                self.robot_name,
            )
            return

        frame = cv2.resize(frame, (640, 480))

        corners, ids, _rejected = _detect_markers(
            frame, self.aruco_dictionary, self.aruco_parameters
        )

        if ids is not None and len(ids) > 0:
            rospy.loginfo_throttle(1.0, "Detected IDs: %s", ids.reshape(-1).tolist())

        metrics: Dict[int, Tuple[float, float, rospy.Time]] = {}
        tvecs = None

        if ids is not None and len(ids) > 0:
            try:
                _rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.aruco_tag_size,
                    self.camera_matrix,
                    self.dist_coeffs,
                )

                flat_ids = ids.reshape(-1)
                for idx, tag_id in enumerate(flat_ids):
                    tx, ty, tz = [float(v) for v in tvecs[idx][0]]
                    dist = _norm3(tx, ty, tz)
                    yaw_off = _bearing_to_tag(tx, ty, tz)
                    if 0.01 < dist < 1.5:
                        metrics[int(tag_id)] = (dist, yaw_off, stamp)
            except Exception as e:
                rospy.logwarn_throttle(
                    self.wait_log_period_sec,
                    "Pose estimation failed: %s",
                    str(e),
                )

        # Update per-tag confirmation counters
        target_node = (
            self.path[self._leg] if self._leg < len(self.path) else -1
        )
        for tag_id, (dist, _yaw, _st) in metrics.items():
            self._detection_buffer[tag_id] = self._detection_buffer.get(tag_id, 0) + 1
            if tag_id == target_node and dist < self.proximity_threshold:
                self._reach_buffer[tag_id] = self._reach_buffer.get(tag_id, 0) + 1
            elif tag_id == target_node:
                self._reach_buffer[tag_id] = 0

        # Tags not in this frame -> decay counters.
        for tag_id in list(self._detection_buffer.keys()):
            if tag_id not in metrics:
                self._detection_buffer[tag_id] = 0
        for tag_id in list(self._reach_buffer.keys()):
            if tag_id not in metrics:
                self._reach_buffer[tag_id] = 0

        for tag_id, data in metrics.items():
            self._tag_metrics[tag_id] = data

        if tvecs is not None and ids is not None and len(ids) > 0:
            debug_frame = self._draw_debug_overlay(frame, corners, ids, tvecs)
        else:
            debug_frame = frame.copy()
            cv2.putText(
                debug_frame,
                f"TARGET: N{target_node}" if target_node >= 0 else "TARGET: DONE",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 0),
                2,
            )
            cv2.putText(
                debug_frame,
                f"LEG: {self._leg}/{len(self.path)-1}  STATE: {self._state}",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 0),
                2,
            )

        self._publish_debug_image(debug_frame)

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------
    def _compensate_right_turn(self, omega: float, *, search: bool = False) -> float:
        """Apply right-turn motor compensation.

        Many Duckiebots have a weaker right-side motor (or trim/k miscalibration)
        that makes omega < 0 (right/CW rotation) stall at small magnitudes.
        Multiply negative omega by a configurable scale so the commanded value
        actually produces motion. Left turns are unchanged.
        """
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
        rospy.loginfo_throttle(
            self.cmd_log_period_sec,
            "[%s leg=%d target=N%d] v=%.3f omega=%.3f",
            self._state,
            self._leg,
            self.path[self._leg] if self._leg < len(self.path) else -1,
            m.v,
            m.omega,
        )

    def _stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _hard_stop(self) -> None:
        for _ in range(5):
            self._publish_cmd(0.0, 0.0)
            rospy.sleep(0.05)

    def _on_shutdown(self) -> None:
        try:
            self._hard_stop()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Target info helpers
    # ------------------------------------------------------------------
    def _get_fresh_target_info(
        self, target_node: int, now: rospy.Time
    ) -> Optional[Tuple[float, float]]:
        """Return (distance, yaw_err) if the target tag has a fresh, confirmed
        detection; else None."""
        info = self._tag_metrics.get(target_node)
        if info is None:
            return None
        dist, yaw_off, stamp = info
        age = (now - stamp).to_sec()
        if age > self.detection_stale_sec:
            return None
        if self._detection_buffer.get(target_node, 0) < CONFIRM_FRAMES:
            return None
        return dist, yaw_off

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def run(self) -> None:
        while not rospy.is_shutdown() and not self._goal_done:
            if self._leg >= len(self.path):
                self._finish_goal()
                break

            target_node = self.path[self._leg]
            now = rospy.Time.now()
            
            # Handle PASS_THROUGH state: drive straight through the node
            if self._state == self.STATE_PASS_THROUGH:
                if self._pass_through_start_time is None:
                    self._pass_through_start_time = now
                
                elapsed = (now - self._pass_through_start_time).to_sec()
                if elapsed >= self.pass_through_time:
                    # Finished passing through, advance to next leg
                    self._advance_leg_after_pass_through(target_node)
                    continue
                else:
                    # Keep driving straight through
                    self._publish_cmd(self.pass_through_speed, 0.0)
                    self._rate.sleep()
                    continue
            
            info = self._get_fresh_target_info(target_node, now)

            if info is not None:
                dist, yaw_err = info
                self._last_yaw_err = yaw_err
                self._last_info_time = now
                yaw_cmd = yaw_err + self.yaw_bias

                reach_count = self._reach_buffer.get(target_node, 0)
                # Check if node is reached AND robot is well-aligned before passing through
                if (dist < self.proximity_threshold and 
                    reach_count >= REACH_CONFIRM_FRAMES and
                    abs(yaw_cmd) < self.pass_through_align_threshold):
                    # Node reached and aligned: enter PASS_THROUGH state
                    rospy.loginfo(
                        "Node N%d reached (dist %.3f m, yaw %.3f rad) and aligned. Entering PASS_THROUGH state for %.1f seconds.",
                        target_node,
                        dist,
                        yaw_cmd,
                        self.pass_through_time,
                    )
                    self._state = self.STATE_PASS_THROUGH
                    self._pass_through_start_time = None  # Will be set on next iteration
                    continue

                if abs(yaw_cmd) > self.align_angle_max:
                    self._state = self.STATE_ALIGN
                    omega = self.angular_gain * yaw_cmd
                    # Floor: never command an omega magnitude below the motor
                    # deadband while the robot is supposed to be turning.
                    if abs(omega) < self.min_turn_omega:
                        omega = math.copysign(self.min_turn_omega, yaw_cmd)
                    # Compensate weaker right-side motor for CW turns.
                    omega = self._compensate_right_turn(omega)
                    self._publish_cmd(self.align_linear_speed, omega)
                else:
                    self._state = self.STATE_APPROACH
                    omega = self.angular_gain * yaw_cmd
                    # Small right-turn compensation also during APPROACH so
                    # the yaw loop can hold heading while driving forward.
                    omega = self._compensate_right_turn(omega)
                    self._publish_cmd(self.linear_speed, omega)
            else:
                # No fresh info for the target: rotate continuously in SEARCH
                self._state = self.STATE_SEARCH
                self._continuous_search()
                self._log_wait_reason(target_node)

            self._rate.sleep()

    def _continuous_search(self) -> None:
        """Rotate continuously in place while searching for the target tag."""
        omega = self._search_omega_now()
        self._publish_cmd(self.search_linear_speed, omega)

    def _search_omega_now(self) -> float:
        """Pick angular velocity for SEARCH.

        Prefer the direction the target was last seen in (if it was ever
        seen on this leg). Otherwise fall back to the grid-geometry sign.
        Right turns get an extra magnitude boost to overcome weaker right-side
        motor / deadband.
        """
        magnitude = max(abs(self.search_omega), self.min_turn_omega)
        if self._last_yaw_err is not None and abs(self._last_yaw_err) > 1e-3:
            omega = math.copysign(magnitude, self._last_yaw_err)
        elif self._search_sign != 0.0:
            omega = math.copysign(magnitude, self._search_sign)
        else:
            omega = magnitude  # small default (CCW / left)
        return self._compensate_right_turn(omega, search=True)

    def _advance_leg_after_pass_through(self, reached_node: int) -> None:
        """Advance to the next leg after passing through a node."""
        rospy.loginfo(
            "Passed through node N%d. Advancing leg %d -> %d.",
            reached_node,
            self._leg,
            self._leg + 1,
        )
        # Discard the tag we just reached so the next iteration can't use it
        self._tag_metrics.pop(reached_node, None)
        self._detection_buffer = {}
        self._reach_buffer = {}
        self._last_yaw_err = None
        self._last_info_time = None
        self._pass_through_start_time = None
        self._leg += 1
        self._state = self.STATE_SEARCH
        if self._leg < len(self.path):
            self._search_sign = self._compute_search_sign(self._leg)
            rospy.loginfo(
                "Next target: N%d  (search_sign=%+.1f)",
                self.path[self._leg],
                self._search_sign,
            )
        # Stop briefly so the next state starts cleanly
        self._hard_stop()
        if self._leg >= len(self.path):
            self._finish_goal()

    def _advance_leg(self, reached_node: int, dist: float) -> None:
        rospy.loginfo(
            "Reached node N%d (tag distance %.3f m). Advancing leg %d -> %d.",
            reached_node,
            dist,
            self._leg,
            self._leg + 1,
        )
        # Discard the tag we just reached so the next iteration can't use it
        self._tag_metrics.pop(reached_node, None)
        self._detection_buffer = {}
        self._reach_buffer = {}
        self._last_yaw_err = None
        self._last_info_time = None
        self._leg += 1
        self._state = self.STATE_SEARCH
        if self._leg < len(self.path):
            self._search_sign = self._compute_search_sign(self._leg)
            rospy.loginfo(
                "Next target: N%d  (search_sign=%+.1f)",
                self.path[self._leg],
                self._search_sign,
            )
        # Stop briefly so the next state starts cleanly
        self._hard_stop()
        if self._leg >= len(self.path):
            self._finish_goal()

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------
    def _log_wait_reason(self, target_node: int) -> None:
        if self._last_camera_msg_time is None:
            rospy.logwarn_throttle(
                self.wait_log_period_sec,
                "No camera images received yet on robot '%s'. Check camera topic.",
                self.robot_name,
            )
            return

        age = (rospy.Time.now() - self._last_camera_msg_time).to_sec()
        if age > self.detection_stale_sec:
            rospy.logwarn_throttle(
                self.wait_log_period_sec,
                "Camera images are stale (last frame %.2f s ago). Expected tag: N%d.",
                age,
                target_node,
            )
            return

        fresh_visible_tags = []
        now = rospy.Time.now()
        for tag_id, (_dist, _yaw, stamp) in self._tag_metrics.items():
            if (now - stamp).to_sec() <= self.detection_stale_sec:
                fresh_visible_tags.append(tag_id)
        fresh_visible_tags.sort()
        rospy.loginfo_throttle(
            self.wait_log_period_sec,
            "SEARCH: target tag N%d not visible. Visible tags: %s",
            target_node,
            fresh_visible_tags if fresh_visible_tags else "none",
        )

    def _finish_goal(self) -> None:
        if self._goal_done:
            return
        self._goal_done = True
        self._hard_stop()

        now = rospy.Time.now()
        n15_info = self._tag_metrics.get(15)
        if n15_info is None:
            rospy.logwarn("Stopping: N15 ARTag not in detection cache.")
        else:
            age = (now - n15_info[2]).to_sec()
            if age > self.detection_stale_sec:
                rospy.logwarn(
                    "Stopping: last N15 detection is %.2f s old (stale).", age
                )

        print("\nPath sequence : %s" % astar.format_path(self.path))
        print("Total cost    : %.4f" % self._path_cost)
        print("Goal Reached")
        rospy.loginfo("Goal Reached")
        rospy.signal_shutdown("goal_reached")


def main() -> None:
    rospy.init_node("assignment3_navigator", anonymous=False)
    nav = Assignment3Navigator()
    nav.run()


if __name__ == "__main__":
    main()
