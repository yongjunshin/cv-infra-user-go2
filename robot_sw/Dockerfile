# robot_sw/Dockerfile — Unitree Go2 patrol SUT (System Under Test) for cv-infra.
#
# Role: a robot-SW developer's app image. cv-infra consumes this container as a
# BLACKBOX — it never modifies SUT internals (REQ-EXEC-004/005) and wires it into
# its `ros2` adapter over a shared docker network (DDS). This repo's job is only to
# make the SUT "behave so it can be verified" (SUT contract, see ../README.md).
#
# STATUS: **stage-0** (U0 bootstrap) — navigation only: Nav2 + AMCL/map_server, no
# perception. The detector (torch-cpu + ultralytics + yolo11n) lands in U2 at the
# marked placeholder below; the target tracker / patrol manager land in U3. There is
# no map and no scenario yet (platform cycle C1 produces the map artifact, C3 the
# topic inventory), so this image currently brings up the nav stack with
# localization DISABLED — see launch/go2_nav.launch.py.
#
# The Go2's SUT is NOT this image alone: SUT = {image_ref, locomotion_policy}
# (decision 2026-08-31 D2). The locomotion policy runs in-process on the runner
# (D3 mirror rule) and is therefore NOT baked here.
#
# Reproducibility (decision 2026-06-24-env-reproducibility-pinning §3; CLAUDE.md §5,§2-7):
#   - Base image = the SAME immutable digest the carter consumer fixture is pinned to
#     (cv-infra-user/robot_sw/Dockerfile) — one ROS 2 Jazzy base across both consumers.
#     Digest re-confirmed live 2026-09-01 (`docker buildx imagetools inspect` -> OCI
#     index, linux/amd64 child sha256:567b81bc…, created 2026-07-02).
#   - `jazzy-ros-base-noble` = ROS 2 Jazzy, ros-base variant (no desktop/GUI), Ubuntu
#     Noble. headless by construction: this SUT never starts rviz (M8 §3.9 D-O).
#   - apt versions pinned via ARG. CAVEAT (inherited from carter): packages.ros.org
#     rotates the datestamp suffix; both pins below were re-verified to still resolve
#     on 2026-09-01 by an actual pinned install (build log in the U0 report). If a
#     rebuild later fails on a superseded version, refresh the ARG defaults.
FROM ros:jazzy-ros-base-noble@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f

# Non-interactive apt for reproducible CI builds.
ARG DEBIAN_FRONTEND=noninteractive

# --- Pinned apt versions (verified installable 2026-09-01; see CAVEAT above) ---
ARG NAVIGATION2_VERSION=1.3.12-1noble.20260615.181551
ARG NAV2_BRINGUP_VERSION=1.3.12-1noble.20260616.082701

# --- stage-0: assemble the reusable navigation stack (do-not-reinvent) ---
# Inherited verbatim from the carter fixture (measured there, p5 T3): install Nav2 in
# SEPARATE pinned RUN layers — never one fat layer, because a large blob has no
# intra-layer resume and a slow pull stalls mid-blob on the GPU runner.
#
# Differences from the carter fixture (deliberate, stage-0):
#   - NO pointcloud_to_laserscan: the platform publishes a flat `/scan` for go2
#     directly (D-2 sensor module). If the RTX-lidar flat scan turns out unavailable
#     (assumption A6 -> C0/C3 probe), U1 adds `depthimage_to_laserscan` here instead.
#   - map_server/amcl need no extra package: both ship inside `navigation2`.

# Layer 1/2: runtime Nav2 stack (navigation2 metapackage). Its Depends carry NO Gazebo
#   (measured on the carter fixture) — pure runtime: bt_navigator/controller/planner/
#   smoother/behaviors/collision-monitor/route/docking/costmap-2d/map-server/amcl/
#   lifecycle-manager. `--no-install-recommends` avoids unpinned Recommends (G-11).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2="${NAVIGATION2_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer 2/2: nav2_bringup LAUNCH package only (bringup/navigation/localization launch +
#   the stock params/BT XML our launch composes — do-not-reinvent). Installed WITHOUT
#   its Gazebo/sim hard-Depends (nav2-minimal-tb3/tb4-sim, ros-gz-sim/bridge,
#   slam-toolbox, xacro, ros2_control controllers = ~0.72 GB the headless SUT never
#   launches). The unmet-dep marks are cosmetic for a frozen leaf image; every node
#   bringup_launch.py actually starts lives in `navigation2`.
RUN apt-get update \
    && apt-get download ros-jazzy-nav2-bringup="${NAV2_BRINGUP_VERSION}" \
    && dpkg --install --force-depends ./ros-jazzy-nav2-bringup_*.deb \
    && rm -f ./ros-jazzy-nav2-bringup_*.deb \
    && rm -rf /var/lib/apt/lists/*

# --- U2 PLACEHOLDER (do not add yet) -----------------------------------------------
# Perception layer, added in U2 as its OWN split layers (same no-fat-layer rule):
#   (a) pip layer: torch (CPU wheel index) + ultralytics, exact versions pinned.
#   (b) model layer: yolo11n weights fetched by URL + verified against a pinned
#       sha256, then SEALED into the image — decision 2026-08-31 D1-A (build-time
#       sealing) so a model change shows up as an image-digest change on the
#       regression axis. CPU inference only; the SUT container never gets a GPU (D1-P2).
# Nothing is installed here in stage-0 — leaving it empty keeps the stage-0 image
# pullable and the U2 diff honest.
# -----------------------------------------------------------------------------------

WORKDIR /opt/robot_sw

# --- go2_bringup: our own thin bringup package (install-only ament package) ---
#   Kept last so an edit to launch/params does not invalidate the heavy apt layers.
COPY src/ /opt/go2_ws/src/
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --install-base /opt/go2_ws/install \
         --base-paths /opt/go2_ws/src \
    && rm -rf /opt/go2_ws/build /opt/go2_ws/log

COPY ros_entrypoint.sh /opt/robot_sw/ros_entrypoint.sh
RUN chmod +x /opt/robot_sw/ros_entrypoint.sh

# --- SUT contract the blackbox satisfies so cv-infra can verify it (see ../README.md) ---
#   - accept use_sim_time:=true, driven by external /clock                (REQ-EXEC-003)
#   - expose goal interface nav2_msgs/action/NavigateToPose
#       @ /navigate_to_pose  (stage-0: nav2 bt_navigator; U3: patrol_manager
#       re-exposes it and remaps nav2's own action inward — D4)            (REQ-EXEC-007)
#   - publish /cmd_vel + subscribe sensor topics                          (REQ-EXEC-006)
#   cv-infra injects use_sim_time at launch; this image stays unmodified by it (REQ-EXEC-005).
ENTRYPOINT ["/opt/robot_sw/ros_entrypoint.sh"]
CMD ["ros2", "launch", "go2_bringup", "go2_nav.launch.py", "use_sim_time:=true"]
