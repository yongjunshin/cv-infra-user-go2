# verify/Dockerfile — the CONSUMER's derived verification image.
#
# The platform's rule (cv-infra CLAUDE.md §3): the stock Isaac Sim image IS the runtime;
# anything extra is the consumer's derived image. cv-infra starts ONE container per PICT
# case from `sim_image` and runs `verify/sim --<axis>=<value> ...` inside it — no docker
# socket, no compose, no second container. So everything the case needs at run time has
# to be in ONE image: the simulator (the stock base) AND the ROS 2 patrol app.
#
# This file is therefore robot_sw/Dockerfile's layer stack, replayed on top of Isaac
# instead of on top of `ros:jazzy-ros-base-noble`. Same packages, same pins, same layer
# discipline, same build-time assertions — the divergence is the base and the first
# layer (we have to install ros-base ourselves; the ros image came with it).
#
# WHAT IS NOT BAKED, on purpose:
#   - sim/patrol_world.py, robot_sw/models/locomotion/policy.pt and the world file: they
#     are read from the checkout at run time, exactly as compose.yaml mounts them today.
#     Editing the world script or retraining the policy must NOT require a rebuild.
#   - ENTRYPOINT / CMD: cv-infra overrides the entrypoint with `/bin/sh -lc 'exec "$0"
#     "$@"'`, so the stock image's runheadless.sh is irrelevant either way. Left alone.
#
# ⚠ ROS_DISTRO IS DELIBERATELY NEVER SET AS AN IMAGE ENV. /isaac-sim/setup_ros_env.sh
#   wires the image's OWN bundled Jazzy libraries only when ROS_DISTRO is unset (the same
#   trap compose.yaml documents). verify/sim sources /opt/ros/jazzy/setup.bash inside
#   SUBSHELLS for the app phases only, so the sim phase still sees a clean environment.
#
# REPRODUCIBILITY (CLAUDE.md §2-3): every version is an ARG, and every ARG is an exact
# pin MEASURED on this base on 2026-09-23 (`apt-cache madison` inside the pinned Isaac
# image with the ROS 2 apt source installed). Two kinds:
#   *_VERSION  the ROS packages robot_sw/Dockerfile also pins. ⚠ NOT the same strings:
#              packages.ros.org rotates the datestamp suffix (robot_sw's 2026-09-01 pins
#              no longer resolve — the first build here died on exactly that), and nav2
#              itself moved 1.3.12 -> 1.3.13 in between. So this image runs nav2 1.3.13
#              while the compose rig built on 2026-09-01 runs 1.3.12; re-pin both together
#              when robot_sw/Dockerfile is next rebuilt.
#   *_PIN      the packages only THIS image installs (ros-base, the colcon toolchain,
#              the two distro guards). Written as `=<version>` suffixes.
# If a pin stops resolving, re-measure with `apt-cache madison <pkg>` on the base and
# change the ARG — never drop the `=`.
#
# Build context is the REPOSITORY ROOT (COPY paths start with robot_sw/):
#   docker build -f verify/Dockerfile -t ghcr.io/yongjunshin/cv-infra-user-go2/go2-verify:<tag> .
# See ../README.md §cv-infra 검증 for the push + digest-pin loop.

FROM nvcr.io/nvidia/isaac-sim:5.1.0@sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54

# The stock image's default user is uid 1234; apt and colcon need root. The last two
# lines of this file put both back.
USER root

ARG DEBIAN_FRONTEND=noninteractive

# --- bootstrap: apt over TLS ---------------------------------------------------------
# The base carries the stock Ubuntu 24.04 "noble" apt sources and reaches
# archive.ubuntu.com (measured 2026-09-23: the first build's apt-get update succeeded
# with nothing added). It ships ca-certificates + curl already; this layer only lifts
# them to the pinned noble-updates builds.
ARG CA_CERTIFICATES_PIN==20260601~24.04.1
ARG CURL_PIN==8.5.0-2ubuntu10.13
RUN apt-get update && apt-get install -y --no-install-recommends \
      "ca-certificates${CA_CERTIFICATES_PIN}" \
      "curl${CURL_PIN}" \
    && rm -rf /var/lib/apt/lists/*

# --- the ROS 2 apt source ------------------------------------------------------------
# The official `ros2-apt-source` .deb (github.com/ros-infrastructure/ros-apt-source
# releases) carries the signing key AND the sources file, so no `apt-key`, no curl'd
# keyring, no `gnupg` layer. Version-pinned; the digest is the second half of that pin.
#
# The digest was read off the release asset on 2026-09-23 (`sha256sum` of the download in
# the first build); a moved or re-cut asset FAILS the build here, like YOLO11N_SHA256 below.
ARG ROS_APT_SOURCE_VERSION=1.1.0
ARG ROS_APT_SOURCE_SHA256=35441f3092fd05773a3c397fab38661bec466584c7a1f1c05366579997cb5fe7
RUN curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    && echo "${ROS_APT_SOURCE_SHA256}  /tmp/ros2-apt-source.deb" | sha256sum -c - \
    && dpkg --install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

# --- ROS 2 Jazzy (ros-base) + the colcon toolchain -----------------------------------
# `ros-base`, never `desktop`: this app never starts rviz, and the base image is already
# ~17 GB. The toolchain is only needed for the colcon build at the bottom of this file;
# it is kept in this layer anyway so that the whole apt-resolver half of the image is
# above the `--force-depends` line below (see the ⚠ on the vision_msgs layer).
#
# python3-numpy / python3-setuptools are named EXPLICITLY, not left to a transitive
# Depends: they are the two "distro guards" constraints-detector.txt pins, and the
# CPU-only assertion further down fails the build if pip ever shadows them.
#
# The base ships NO system python3 at all (dpkg-query: no such package, measured
# 2026-09-23) — ros-base is what brings the interpreter every apt ROS node runs on.
ARG ROS_BASE_PIN==0.11.0-1noble.20260903.093452
ARG COLCON_PIN==0.3.0-100
ARG BUILD_ESSENTIAL_PIN==12.10ubuntu1
ARG CMAKE_PIN==3.28.3-1build7
ARG GIT_PIN==1:2.43.0-1ubuntu7.3
ARG PYTHON3_NUMPY_PIN==1:1.26.4+ds-6ubuntu1
ARG PYTHON3_SETUPTOOLS_PIN==68.1.2-2ubuntu1.2
RUN apt-get update && apt-get install -y --no-install-recommends \
      "ros-jazzy-ros-base${ROS_BASE_PIN}" \
      "python3-colcon-common-extensions${COLCON_PIN}" \
      "build-essential${BUILD_ESSENTIAL_PIN}" \
      "cmake${CMAKE_PIN}" \
      "git${GIT_PIN}" \
      "python3-numpy${PYTHON3_NUMPY_PIN}" \
      "python3-setuptools${PYTHON3_SETUPTOOLS_PIN}" \
    && rm -rf /var/lib/apt/lists/*

# ===================================================================================== #
# Everything below is robot_sw/Dockerfile, layer for layer, pin for pin. Read that file
# for the measurements behind each one; the comments here are kept short and point at it.
# ===================================================================================== #

# --- Pinned apt versions (re-resolved on this base 2026-09-23 — see the ⚠ up top) ---
ARG NAVIGATION2_VERSION=1.3.13-1noble.20260905.072714
ARG NAV2_BRINGUP_VERSION=1.3.13-1noble.20260907.023821

# Layer 1/3: runtime Nav2 stack. SEPARATE pinned layers — never one fat layer, because a
# large blob has no intra-layer resume and a slow pull stalls mid-blob.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2="${NAVIGATION2_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer 2/3: vision_msgs (the detector cannot import without it) + pip itself.
#   ⚠ THIS MUST STAY ABOVE the nav2_bringup layer: `dpkg --force-depends` below leaves
#   the dpkg database with unmet dependencies, after which EVERY `apt-get install`
#   refuses to run (`E: Unmet dependencies`, exit 100 — measured 2026-09-01). Anything
#   that needs apt's resolver goes here; anything force-installed goes below.
ARG PYTHON3_PIP_VERSION=24.0+dfsg-1ubuntu1.3
ARG VISION_MSGS_VERSION=4.1.1-3noble.20260902.034028
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-vision-msgs="${VISION_MSGS_VERSION}" \
      python3-pip="${PYTHON3_PIP_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer: twist_mux — /cmd_vel arbitration. Needs apt's resolver (diagnostic-updater,
#   twist-mux-msgs, visualization-msgs are not in ros-base), so it also stays above the
#   force-depends line.
ARG TWIST_MUX_VERSION=4.5.0-1noble.20260903.071116
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-twist-mux="${TWIST_MUX_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer 3/3: nav2_bringup LAUNCH package only, WITHOUT its Gazebo/sim hard-Depends
#   (~0.72 GB this headless app never launches). The unmet-dep marks are cosmetic for a
#   frozen leaf image; every node bringup_launch.py starts lives in `navigation2`.
RUN apt-get update \
    && apt-get download ros-jazzy-nav2-bringup="${NAV2_BRINGUP_VERSION}" \
    && dpkg --install --force-depends ./ros-jazzy-nav2-bringup_*.deb \
    && rm -f ./ros-jazzy-nav2-bringup_*.deb \
    && rm -rf /var/lib/apt/lists/*

# --- perception: yolo11n on CPU, in split layers -----------------------------------
ARG TORCH_VERSION=2.8.0+cpu
ARG TORCHVISION_VERSION=0.23.0+cpu
ARG ULTRALYTICS_VERSION=8.4.136
ARG YOLO11N_URL=https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
ARG YOLO11N_SHA256=0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1

# robot_sw/Dockerfile's layer (a) — libGL.so.1 for `import cv2` — has no counterpart here:
#   the Isaac base is a graphics application and already ships libglvnd0 / libglx0 /
#   libgl1 1.7.0-1build1, the very versions robot_sw pins (dpkg-query on the base,
#   2026-09-23). The CPU-only assertion in layer (c) still imports cv2, so a base that
#   lost them would fail the build there, not at run time.

# Layer (b): torch + torchvision, CPU wheels, from the CPU-only index.
COPY robot_sw/constraints-detector.txt /opt/robot_sw/constraints-detector.txt
RUN pip3 install --no-cache-dir --break-system-packages \
      --index-url https://download.pytorch.org/whl/cpu \
      -c /opt/robot_sw/constraints-detector.txt \
      "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}"

# Layer (c): ultralytics, CONSTRAINED. Unconstrained it replaces the CPU torch above with
#   a CUDA torch + 16 nvidia-* wheels. The assertion is a build-time GATE, not a log line:
#   a CUDA torch, a shadowed distro numpy or a non-importable cv2 fails the build. On THIS
#   base it also proves the distro-guard layer above actually landed.
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

# Layer (d): the weights. Fetch by URL, verify the pinned digest, seal. A moved asset
#   FAILS THE BUILD here rather than silently changing what the app sees.
RUN mkdir -p /opt/models \
    && curl -fsSL -o /opt/models/yolo11n.pt "${YOLO11N_URL}" \
    && echo "${YOLO11N_SHA256}  /opt/models/yolo11n.pt" | sha256sum -c -

# ultralytics writes its settings next to the model and pip-installs missing deps mid-run
# unless told otherwise. A sealed image does neither.
ENV YOLO_CONFIG_DIR=/tmp/ultralytics \
    YOLO_AUTOINSTALL=false

# --- our own packages: go2_msgs / go2_bringup / go2_detector / go2_target_tracker /
#   go2_patrol_manager. Kept last so an edit to node code does not invalidate the heavy
#   layers above. They add NO apt layer, which is what keeps them legal below the
#   force-depends line.
COPY robot_sw/src/ /opt/go2_ws/src/
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --install-base /opt/go2_ws/install \
         --base-paths /opt/go2_ws/src \
    && rm -rf /opt/go2_ws/build /opt/go2_ws/log \
    && test -f /opt/go2_ws/install/setup.bash

# Back to the stock image's defaults. cv-infra pre-creates its cache mounts owned by uid
# 1234 and the case container runs as the image's default user, so an image that ended as
# root would make every cache and every `ros2` log write land wrong.
USER 1234
WORKDIR /isaac-sim
