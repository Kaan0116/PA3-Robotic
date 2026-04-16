#!/bin/bash

source /environment.sh

dt-launchfile-init

# ----------------------------------------------------------------------------
# VEHICLE_NAME: On a Duckiebot, /environment.sh (from Duckietown base images) and/or
# dts typically set this to the robot hostname when you use `dts devel run -H <robot>`.
# Official docs also note it may be unset in some setups (fallback default is used).
# This launcher passes veh:=${VEHICLE_NAME} into roslaunch; set VEHICLE_NAME manually
# before `dts devel run` only if your container does not define it and defaults are wrong.

VEH="${VEHICLE_NAME:-bear}"
dt-exec roslaunch assignment3 assignment3.launch veh:="${VEH}"

# ----------------------------------------------------------------------------

dt-launchfile-join
