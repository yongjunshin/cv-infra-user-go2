#!/usr/bin/env bash
# web/entrypoint.sh — bring up the three back-ends the browser page talks to.
#
#   9090  rosbridge_server   websocket ↔ ROS 2 (topics + the v2 action ops)
#   8080  web_video_server   /camera/image_raw -> MJPEG over HTTP
#   8000  python http.server the static page itself
#
# All three are foregrounded children of this shell so that ONE SIGTERM from
# `docker stop` takes the whole container down cleanly, and so that a crash in any of
# them stops the container instead of leaving a half-dead controller listening.
#
# NOT `set -e`: the shutdown path below deliberately reads non-zero exits.
# NOT `set -u` either: /opt/ros/jazzy/setup.bash reads AMENT_TRACE_SETUP_FILES unguarded
# and dies instantly under nounset (measured — exit 1 before a single service started).
set -o pipefail

# Base ROS 2 distro (rosbridge, web_video_server, vision_msgs installed via apt).
source /opt/ros/jazzy/setup.bash
# Overlay: go2_msgs, so rosbridge can resolve go2_msgs/action/Patrol.
source /opt/web_ws/install/setup.bash

pids=()

shutdown() {
  trap - TERM INT
  for pid in "${pids[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait
}
trap shutdown TERM INT

# rosbridge. The stock launch file already defaults `send_action_goals_in_new_thread` to
# true (verified in rosbridge_websocket_launch.xml, 2.7.0) — which this page depends on:
# with it false, a running /patrol goal would block the protocol thread and the teleop
# publishes on the same socket would queue behind it until the mission ended.
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
pids+=($!)

# MJPEG camera. Binds :8080 at startup and holds it even with an empty ROS graph.
ros2 run web_video_server web_video_server &
pids+=($!)

# The page. Plain stdlib server — localhost only, no framework to pin.
python3 -m http.server 8000 --directory /opt/web/static &
pids+=($!)

# Wake on the FIRST child to exit, then take the other two down with it.
wait -n
echo "[go2-web] a back-end exited — shutting the others down" >&2
shutdown
