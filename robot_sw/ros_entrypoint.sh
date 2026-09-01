#!/usr/bin/env bash
# robot_sw entrypoint — source ROS 2 Jazzy + the go2_bringup overlay, then exec.
#
# The default CMD brings up the headless patrol app (use_sim_time:=true). The command
# may be overridden at `docker run`, and any command must still be able to reach the ROS
# graph — so the environment is sourced here in ENTRYPOINT (survives any CMD override).
set -e

# Base ROS 2 distro (nav2 + nav2_bringup installed via apt).
source /opt/ros/jazzy/setup.bash
# Overlay: our bringup package (launch + Nav2 params).
source /opt/go2_ws/install/setup.bash

exec "$@"
