# robot_sw/Dockerfile — the Unitree Go2 patrol app, as one runnable image.
#
# Role: a robot-SW developer's app image. Everything the app needs to patrol is baked in
# and nothing is fetched at run time, so the image a developer builds is the image that
# runs. Point it at any simulator or robot publishing the sensor topics below.
#
# What is inside: Nav2 + AMCL/map_server with the warehouse occupancy grid vendored into
# go2_bringup, `go2_detector` (yolo11n, CPU) /camera/image_raw -> /detections,
# `go2_target_tracker` (C++) /detections + depth + TF -> /targets, `go2_patrol_manager`
# (C++) which serves this app's own go2_msgs/action/Patrol at /patrol and drives nav2
# through nav2's standard /navigate_to_pose, and twist_mux arbitrating /cmd_vel between
# the autonomy stack and a human.
#
# The locomotion policy is deliberately NOT baked here: it is loaded and executed by
# whatever moves the robot's legs (the simulator, or the robot itself), not by this app.
#
# Reproducibility (CLAUDE.md §5,§2-7):
#   - Base image pinned by immutable digest. Digest re-confirmed live 2026-09-01
#     (`docker buildx imagetools inspect` -> OCI index, linux/amd64 child
#     sha256:567b81bc…, created 2026-07-02).
#   - `jazzy-ros-base-noble` = ROS 2 Jazzy, ros-base variant (no desktop/GUI), Ubuntu
#     Noble. Headless by construction: this app never starts rviz.
#   - apt versions pinned via ARG. CAVEAT: packages.ros.org rotates the datestamp
#     suffix; every pin below was re-verified to still resolve on 2026-09-01 by an
#     actual pinned install. If a rebuild later fails on a superseded version,
#     re-resolve with `apt-cache madison <pkg>` in the pinned base image and refresh
#     the ARG default.
FROM ros:jazzy-ros-base-noble@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f

# Non-interactive apt for reproducible CI builds.
ARG DEBIAN_FRONTEND=noninteractive

# --- Pinned apt versions (verified installable 2026-09-01; see CAVEAT above) ---
ARG NAVIGATION2_VERSION=1.3.12-1noble.20260615.181551
ARG NAV2_BRINGUP_VERSION=1.3.12-1noble.20260616.082701

# --- the reusable navigation stack (do-not-reinvent) ---
# MEASURED build rule: install Nav2 in SEPARATE pinned RUN layers — never one fat layer,
# because a large blob has no intra-layer resume and a slow pull stalls mid-blob.
#
# Deliberate omissions:
#   - NO pointcloud_to_laserscan: this app consumes a flat `/scan` directly. If the
#     sensor only offers a depth image, add `depthimage_to_laserscan` here instead.
#   - map_server/amcl need no extra package: both ship inside `navigation2`.

# Layer 1/2: runtime Nav2 stack (navigation2 metapackage). Its Depends carry NO Gazebo
#   (measured on the carter fixture) — pure runtime: bt_navigator/controller/planner/
#   smoother/behaviors/collision-monitor/route/docking/costmap-2d/map-server/amcl/
#   lifecycle-manager. `--no-install-recommends` avoids unpinned Recommends (G-11).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2="${NAVIGATION2_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer 2/3: the ROS message package the detector publishes + pip itself.
#   `vision_msgs` is NOT in ros-base (without it `go2_detector` cannot even import) and
#   ros-base ships no pip at all (measured 2026-09-01: `command -v pip3` is empty).
#   ⚠ THIS MUST STAY ABOVE the nav2_bringup layer: `dpkg --force-depends` below leaves
#   the dpkg database with unmet dependencies, and from that point on **every**
#   `apt-get install` refuses to run (`E: Unmet dependencies`, exit 100 — measured
#   2026-09-01 when this layer sat after it). Only `apt-get download` + `dpkg
#   --force-depends` still work afterwards. So: anything that needs apt's resolver goes
#   here; anything installed with force-depends goes below.
ARG PYTHON3_PIP_VERSION=24.0+dfsg-1ubuntu1.3
ARG VISION_MSGS_VERSION=4.1.1-3noble.20260615.113052
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-vision-msgs="${VISION_MSGS_VERSION}" \
      python3-pip="${PYTHON3_PIP_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer: twist_mux — /cmd_vel arbitration between the autonomy stack and a human
#   (see go2_bringup/params/twist_mux.yaml for the priorities and the ⚠ use_stamped
#   trap). Installed with apt's RESOLVER because its Depends (diagnostic-updater,
#   twist-mux-msgs, visualization-msgs) are not in ros-base — so this layer MUST stay
#   above the `dpkg --force-depends` layer below, for the reason spelled out just above.
ARG TWIST_MUX_VERSION=4.5.0-1noble.20260615.140302
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-twist-mux="${TWIST_MUX_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer 3/3: nav2_bringup LAUNCH package only (bringup/navigation/localization launch +
#   the stock params/BT XML our launch composes — do-not-reinvent). Installed WITHOUT
#   its Gazebo/sim hard-Depends (nav2-minimal-tb3/tb4-sim, ros-gz-sim/bridge,
#   slam-toolbox, xacro, ros2_control controllers = ~0.72 GB this headless app never
#   launches). The unmet-dep marks are cosmetic for a frozen leaf image; every node
#   bringup_launch.py actually starts lives in `navigation2`.
RUN apt-get update \
    && apt-get download ros-jazzy-nav2-bringup="${NAV2_BRINGUP_VERSION}" \
    && dpkg --install --force-depends ./ros-jazzy-nav2-bringup_*.deb \
    && rm -f ./ros-jazzy-nav2-bringup_*.deb \
    && rm -rf /var/lib/apt/lists/*

# --- perception: yolo11n on CPU, in split layers -----------------------------------
# (the apt half is layer 2/3 above — it has to run while apt's resolver still works)
# Layer sizes measured 2026-09-01 (`docker history`): vision_msgs+pip 20.7 MB ·
# libgl 4.82 MB · torch 843 MB · ultralytics 494 MB · yolo11n 5.61 MB. Image total
# 4.34 GB (nav stack alone was 2.97 GB). The no-fat-layer rule above still holds: torch
# and ultralytics are never merged into one blob, so the two big pulls resume
# independently over a slow network.
ARG LIBGLVND_VERSION=1.7.0-1build1
ARG TORCH_VERSION=2.8.0+cpu
ARG TORCHVISION_VERSION=0.23.0+cpu
ARG ULTRALYTICS_VERSION=8.4.136
# yolo11n weights: pointer + digest in git, bytes sealed into the image at build time.
# The model is therefore part of the image digest — swapping it produces a different
# image instead of quietly changing what an unchanged tag does. Nothing is ever fetched
# at run time.
ARG YOLO11N_URL=https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
ARG YOLO11N_SHA256=0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1

# Layer (a): libGL.so.1, which `import cv2` dlopen()s (opencv-python is a hard
#   ultralytics dependency). Installed with the stage-0 download + `--force-depends`
#   idiom because plain `apt-get install libgl1` drags in the mesa software-GL driver
#   and libllvm20 = **194 MB measured** for a headless container that never creates a
#   GL context; the three libglvnd packages alone are 3 MB and cv2 imports fine
#   (verified in the build assertion below, not assumed).
RUN apt-get update \
    && apt-get download \
         libglvnd0="${LIBGLVND_VERSION}" \
         libglx0="${LIBGLVND_VERSION}" \
         libgl1="${LIBGLVND_VERSION}" \
    && dpkg --install --force-depends ./libglvnd0_*.deb ./libglx0_*.deb ./libgl1_*.deb \
    && rm -f ./*.deb \
    && rm -rf /var/lib/apt/lists/*

# Layer (b): torch + torchvision, CPU wheels, from the CPU-only index.
COPY constraints-detector.txt /opt/robot_sw/constraints-detector.txt
RUN pip3 install --no-cache-dir --break-system-packages \
      --index-url https://download.pytorch.org/whl/cpu \
      -c /opt/robot_sw/constraints-detector.txt \
      "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}"

# Layer (c): ultralytics, CONSTRAINED. Unconstrained, this line replaces the CPU torch
#   above with `torch 2.13.0` + triton + 16 nvidia-* CUDA wheels (measured with
#   `pip install --dry-run --report`). The assertion at the end is a
#   build-time GATE, not a log line: a CUDA torch, a shadowed distro numpy or a
#   non-importable cv2 fails the build.
RUN pip3 install --no-cache-dir --break-system-packages \
      -c /opt/robot_sw/constraints-detector.txt \
      "ultralytics==${ULTRALYTICS_VERSION}" \
    && pip3 freeze | grep -E '^(torch|torchvision|ultralytics|opencv-python)==' \
    && python3 -c "\
import cv2, numpy, torch, ultralytics; \
assert torch.version.cuda is None, 'CUDA torch leaked in: %s' % torch.version.cuda; \
assert torch.__version__.endswith('+cpu'), torch.__version__; \
assert numpy.__file__.startswith('/usr/lib/'), 'pip shadowed the distro numpy: %s' % numpy.__file__; \
print('CPU-ONLY OK torch', torch.__version__, 'cv2', cv2.__version__, 'numpy', numpy.__version__, 'ultralytics', ultralytics.__version__)"

# Layer (d): the weights. Fetch by URL, verify the pinned digest, seal. A wrong or
#   moved asset FAILS THE BUILD here rather than silently changing what the app sees.
RUN mkdir -p /opt/models \
    && curl -fsSL -o /opt/models/yolo11n.pt "${YOLO11N_URL}" \
    && echo "${YOLO11N_SHA256}  /opt/models/yolo11n.pt" | sha256sum -c -

# ultralytics writes its settings next to the model unless told otherwise, and it will
# pip-install missing deps mid-run if allowed. A sealed app image does neither: config
# goes to a writable tmp dir, and a missing dependency becomes a loud error instead of a
# network call in the middle of a mission.
ENV YOLO_CONFIG_DIR=/tmp/ultralytics \
    YOLO_AUTOINSTALL=false
# -----------------------------------------------------------------------------------

WORKDIR /opt/robot_sw

# --- our own packages: go2_msgs (the Patrol action) + go2_bringup (install-only,
#   ament_cmake) + go2_detector (ament_python) + go2_target_tracker / go2_patrol_manager
#   (ament_cmake, C++) ---
#   Kept last so an edit to launch/params/node code does not invalidate the heavy layers.
#   They compile against ros-base's own rclcpp/tf2/nav2_msgs/vision_msgs and its rosidl
#   generators; they add NO apt layer, which is what keeps them below the
#   `dpkg --force-depends` line above (after it, `apt-get install` refuses to run —
#   measured, see layer 2/3).
COPY src/ /opt/go2_ws/src/
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --install-base /opt/go2_ws/install \
         --base-paths /opt/go2_ws/src \
    && rm -rf /opt/go2_ws/build /opt/go2_ws/log

COPY ros_entrypoint.sh /opt/robot_sw/ros_entrypoint.sh
RUN chmod +x /opt/robot_sw/ros_entrypoint.sh

# --- what this image offers the outside world (see ../README.md) ---
#   - accepts use_sim_time:=true, driven by an external /clock
#   - serves go2_msgs/action/Patrol @ /patrol — the app's own mission interface
#   - nav2 serves nav2_msgs/action/NavigateToPose @ /navigate_to_pose, under its own
#     standard name, for plain point-to-point navigation
#   - subscribes the sensor topics, publishes /cmd_vel (through twist_mux)
ENTRYPOINT ["/opt/robot_sw/ros_entrypoint.sh"]
# One image, one CMD: the patrol launch. Without a camera the mission ABORTS after the
# perception probe instead of driving blind. The plain nav launch remains for hand runs:
#   ros2 launch go2_bringup go2_nav.launch.py use_sim_time:=true
CMD ["ros2", "launch", "go2_bringup", "go2_patrol.launch.py", "use_sim_time:=true"]
