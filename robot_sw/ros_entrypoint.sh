#!/usr/bin/env bash
# robot_sw entrypoint — source ROS 2 Jazzy + the go2_bringup overlay, then exec.
#
# cv-infra spawns this SUT container as a blackbox (REQ-EXEC-005). The default CMD
# brings up the headless Nav2 stack (use_sim_time:=true). cv-infra may override the
# command at `docker run`, but must be able to reach the ROS graph regardless — so we
# source the environment here in ENTRYPOINT (survives any CMD override).
set -e

# Base ROS 2 distro (nav2 + nav2_bringup installed via apt).
source /opt/ros/jazzy/setup.bash
# Overlay: our bringup package (launch + Nav2 params).
source /opt/go2_ws/install/setup.bash

exec "$@"
