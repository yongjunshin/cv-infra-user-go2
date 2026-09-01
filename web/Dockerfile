# web/Dockerfile — localhost web controller ("로봇 조종실") for the go2 patrol app.
#
# Role: a HUMAN front-end for the robot app. It adds
# NOTHING to the robot app — it only speaks to the running ROS 2 graph from a browser:
#
#     browser ──ws:9090── rosbridge_server ──┐
#             ──http:8080── web_video_server ├── the go2 patrol app's ROS 2 graph
#             ──http:8000── static page      ┘   (/patrol, /cmd_vel_teleop, /detections,
#                                                 /scan, /tf, /camera/image_raw)
#
# BUILD CONTEXT IS THE REPO ROOT (compose: `context: .`, `dockerfile: web/Dockerfile`),
# because this image needs three things from outside web/:
#   - robot_sw/src/go2_msgs                          (rosbridge must resolve
#                                                     go2_msgs/action/Patrol at run time)
#   - robot_sw/src/go2_bringup/maps/*.png            (the 지도 tab renders the SAME static
#                                                     occupancy grid Nav2 localises against)
#   - web/static                                     (the page itself)
#
# Reproducibility (decision 2026-06-24-env-reproducibility-pinning §3; CLAUDE.md §5,§2-7):
#   - Base image = the SAME immutable digest robot_sw/Dockerfile pins, so the controller
#     and the app it drives share one ROS 2 Jazzy / RMW build. Never a floating tag.
#   - apt versions pinned via ARG, resolved LIVE against packages.ros.org on 2026-09-01
#     with `apt-cache madison` on this exact base digest. CAVEAT (same as robot_sw):
#     packages.ros.org rotates the datestamp suffix; if a later rebuild fails on a
#     superseded version, refresh the ARG defaults — do not unpin them.
FROM ros:jazzy-ros-base-noble@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f

# Non-interactive apt for reproducible CI builds.
ARG DEBIAN_FRONTEND=noninteractive

# --- Pinned apt versions (resolved 2026-09-01 via `apt-cache madison` on this base) ---
ARG ROSBRIDGE_SUITE_VERSION=2.7.0-1noble.20260615.164124
ARG WEB_VIDEO_SERVER_VERSION=3.1.0-1noble.20260615.150732
ARG VISION_MSGS_VERSION=4.1.1-3noble.20260615.113052

# Split into one package per RUN layer — the robot_sw rule ("never one fat layer"): a
# large blob has no intra-layer resume, so a slow pull stalls mid-blob.

# Layer 1/3: rosbridge_suite — the websocket ↔ ROS 2 bridge (topics, and the v2 action
#   ops `send_action_goal` / `cancel_action_goal` this page drives /patrol with).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rosbridge-suite="${ROSBRIDGE_SUITE_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer 2/3: web_video_server — turns /camera/image_raw into an MJPEG HTTP stream the
#   browser can put in a plain <img>. Do-not-reinvent: no custom transcoder here.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-web-video-server="${WEB_VIDEO_SERVER_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer 3/3: vision_msgs — NOT in ros-base. Without it rosbridge cannot resolve
#   vision_msgs/msg/Detection2DArray and the "탐지 표시" overlay silently gets nothing.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-vision-msgs="${VISION_MSGS_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# --- the app's own interface package -------------------------------------------------
# rosbridge resolves an action type by importing the generated Python module, so
# `go2_msgs` has to exist in THIS container too — the type name on the wire is not
# enough. Only go2_msgs is copied (it is a leaf: ament_cmake + rosidl on geometry_msgs /
# action_msgs, both already in ros-base); none of the app's nodes are built here.
COPY robot_sw/src/go2_msgs /opt/web_ws/src/go2_msgs
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --install-base /opt/web_ws/install \
         --base-paths /opt/web_ws/src \
    && rm -rf /opt/web_ws/build /opt/web_ws/log

# --- the page ------------------------------------------------------------------------
# Kept last so editing HTML/JS/CSS does not invalidate the apt or colcon layers.
COPY web/static/ /opt/web/static/
# The 지도 tab draws this PNG directly and georeferences it with the constants from the
# sibling .yaml (res 0.05, origin -11.975/-17.975, 480x776 px) hardcoded in app.js.
# There is no /map subscription: the map is static for the whole mission.
COPY robot_sw/src/go2_bringup/maps/carter_warehouse_navigation.png /opt/web/static/map.png

COPY web/entrypoint.sh /opt/web/entrypoint.sh
RUN chmod +x /opt/web/entrypoint.sh

#   8000 = the static page   ·   8080 = MJPEG camera   ·   9090 = rosbridge websocket
EXPOSE 8000 8080 9090

ENTRYPOINT ["/opt/web/entrypoint.sh"]
