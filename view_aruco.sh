#!/usr/bin/env bash
# Launch the ArUco viewer on *your computer* (not on the robot).
#
# It opens a local OpenCV window that shows the live camera stream of the
# Duckiebot with ArUco markers drawn as colored squares and their IDs.
#
# Usage:
#   ./view_aruco.sh               # defaults to robot "bear"
#   ./view_aruco.sh autobot01     # use a different robot name
#   DEBUG=1 ./view_aruco.sh bear  # show the navigator's debug image instead
#
# Requirements on your computer:
#   - ROS (noetic/melodic) sourced in your shell
#   - The duckietown-msgs / sensor_msgs packages available
#   - Python 3 with: opencv-contrib-python, numpy
#
# Make sure your computer can reach the robot's ROS master, e.g.:
#   export ROS_MASTER_URI=http://bear.local:11311
#   export ROS_HOSTNAME=$(hostname).local

set -e

ROBOT_NAME="${1:-${VEHICLE_NAME:-bear}}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIEWER="${SCRIPT_DIR}/assignment3/packages/assignment3/src/aruco_viewer.py"

if [ ! -x "${VIEWER}" ]; then
  chmod +x "${VIEWER}" || true
fi

ARGS=(--robot "${ROBOT_NAME}")
if [ "${DEBUG:-0}" = "1" ]; then
  ARGS+=(--debug-image)
fi

# Auto-fill ROS_MASTER_URI if missing
if [ -z "${ROS_MASTER_URI:-}" ]; then
  export ROS_MASTER_URI="http://${ROBOT_NAME}.local:11311"
  echo "[info] ROS_MASTER_URI not set, using ${ROS_MASTER_URI}"
fi

echo "[info] Starting ArUco viewer for robot '${ROBOT_NAME}'"
echo "[info] Press 'q' or ESC inside the window to quit."

exec python3 "${VIEWER}" "${ARGS[@]}"
