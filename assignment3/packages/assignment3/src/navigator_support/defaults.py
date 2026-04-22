# Default private params (~name) for the navigator. Launch file values override these.

ROBOT_NAME_DEFAULT = "bear"

# Driving / control
LINEAR_SPEED_DEFAULT = 0.18
ANGULAR_GAIN_DEFAULT = 0.9  # P gain on yaw error when aligning or approaching
PROXIMITY_THRESHOLD_DEFAULT = 0.45  # meters: "close enough" to the tag along the ray
ALIGN_ANGLE_MAX_DEFAULT = 0.20  # rad: if |yaw_err| is bigger, we ALIGN (turn) instead of APPROACH
SEARCH_ANGULAR_SPEED_DEFAULT = 1.3
SEARCH_LINEAR_SPEED_DEFAULT = 0.0  # usually 0 so SEARCH is a pure spin
ALIGN_LINEAR_SPEED_DEFAULT = 0.0
MAX_ANGULAR_SPEED_DEFAULT = 3.0
YAW_BIAS_DEFAULT = 0.0  # small fixed offset if the camera frame is systematically skewed

# Measurements: ignore ArUco if the last observation is older than this
DETECTION_STALE_SEC_DEFAULT = 1.0

# Extra params we read for future logging / tuning (not all are used in the loop right now)
WAIT_LOG_PERIOD_SEC_DEFAULT = 2.0
CMD_LOG_PERIOD_SEC_DEFAULT = 1.0
LOST_COAST_SEC_DEFAULT = 0.35

# After declaring a node reached, drive straight for this long before targeting the next ID
PASS_THROUGH_TIME_DEFAULT = 1.5
PASS_THROUGH_SPEED_DEFAULT = 0.18
PASS_THROUGH_ALIGN_THRESHOLD_DEFAULT = 0.10  # rad: how straight we need to be to enter PASS_THROUGH

# Hardware-ish fixes: Duckie motors often won't move below a certain |omega|
MIN_TURN_OMEGA_DEFAULT = 0.75

# If right turns are weak, multiply negative omega by these (search uses a stronger scale)
RIGHT_TURN_OMEGA_SCALE_DEFAULT = 1.6
SEARCH_RIGHT_SCALE_DEFAULT = 2.2

# ArUco physical size on the map + which dictionary the tags use
ARUCO_TAG_SIZE_METERS_DEFAULT = 0.065
ARUCO_DICTIONARY_DEFAULT = "DICT_5X5_50"

# Camera intrinsics from calibration (fx, fy, cx, cy) and distortion (k1,k2,p1,p2,k3)
CAMERA_FX_DEFAULT = 270.4563591302591
CAMERA_FY_DEFAULT = 269.2951665378049
CAMERA_CX_DEFAULT = 314.1813567017415
CAMERA_CY_DEFAULT = 218.88618596346137

DIST_K1_DEFAULT = -0.19162991260105328
DIST_K2_DEFAULT = 0.026384790215657535
DIST_P1_DEFAULT = 0.005682129590129115
DIST_P2_DEFAULT = 0.0006647376545041703
DIST_K3_DEFAULT = 0.0

# Debouncing: need this many consecutive frames with a tag before we trust ALIGN/APPROACH
CONFIRM_FRAMES = 2
# And this many "close" frames before we allow PASS_THROUGH
REACH_CONFIRM_FRAMES = 3
