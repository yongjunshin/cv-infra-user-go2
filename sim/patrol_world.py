#!/usr/bin/env python3
"""patrol_world.py — the patrol app's own Isaac Sim world, in one file.

Run it inside the Isaac Sim 5.1.0 container and it stands up the world this app
was written against and then just keeps stepping:

    ACCEPT_EULA=Y PRIVACY_CONSENT=Y ./python.sh /workspace/sim/patrol_world.py

    # then, from the app's own container / shell:
    ros2 launch go2_bringup go2_patrol.launch.py
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"

What it is: a warehouse, a Unitree Go2 standing in it, the trained locomotion
policy driving that robot's twelve joints in-process, and a ROS 2 node publishing
the streams the app consumes (``/clock``, ``/odom`` + TF, camera RGB-D + info,
``/scan``) while consuming the one stream the app produces (``/cmd_vel``). No
mission is driven here, nothing is judged and nothing is recorded — the app is
the thing under test, and this is the world it talks to. An app that only works
inside somebody else's harness is not a deliverable, so this file imports nothing
from any harness: stdlib, PyYAML, the Isaac Sim bundle and the ROS 2 messages
that ship with it.

TWO KINDS OF CONSTANT LIVE IN TWO DIFFERENT PLACES, on purpose:

* **hardware-twin** values are HERE — the USD paths, the spawn height, the joint
  order, the DC-motor limits, the sensor mounts and optics, the publish rates.
  They describe the robot and its instruments, so they change only when the
  ROBOT changes;
* **policy-contract** values are in ``policy_meta.yaml``, read from NEXT TO the
  policy weights — action scale, PD gains, decimation, the default stance, and
  the name of the observation layout. They are training artefacts: they change
  every time the policy is retrained, so they travel with the weights instead of
  being baked into this script. This file implements exactly one observation
  layout and REFUSES a policy that declares another one (it would silently walk
  the robot with a scrambled observation otherwise).

Every non-obvious number below is annotated with the measurement it came from.
They are not preferences: most of them were found by watching the wrong value
fail quietly (a lidar that returns an empty array forever, a camera with a 10°
field of view, a PD controller nobody asked for fighting the policy).
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path

import yaml

# The FIRST Isaac import of the process, and it must stay first: SimulationApp
# boots Kit, and any ``omni.*`` / ``isaacsim.*`` / numpy / torch import that
# happens BEFORE the app object exists binds against a runtime that is not up
# yet. Everything vendor-specific below is therefore imported inside a function,
# after ``SimulationApp(...)`` has returned.
from isaacsim import SimulationApp  # noqa: E402  (ordering is the point)

LOG = "[patrol-world]"

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_CONFIG = 2

# --------------------------------------------------------------------------- #
# Stage — what gets composed, and where the robot is dropped.
# --------------------------------------------------------------------------- #
#: The warehouse itself. Opened with ``open_stage`` (not referenced): it IS the
#: stage, and the occupancy map the app's AMCL localises against was drawn from
#: this scene.
SCENE_USD = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
#: The shelving/clutter layer, referenced at IDENTITY. Identity is not a detail:
#: the vendored occupancy map (robot_sw/.../maps/) was captured with these extras
#: at identity, so any transform here would silently invalidate the app's map.
EXTRAS_USD = "/Isaac/Environments/Simple_Warehouse/Stage/warehouse_extras.usd"
EXTRAS_PRIM = "/World/warehouse_extras"
#: The IsaacLab flavour of the Go2, NOT ``/Isaac/Robots/...``: this is the asset
#: the policy was trained on — same 12 dof in the same order, 19 bodies. A robot
#: whose joints are ordered differently would take the policy's action vector and
#: apply it to the wrong legs, with no error anywhere.
ROBOT_USD = "/Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd"
ROBOT_PRIM = "/World/Go2"
#: The chassis body. Ground-truth pose source for /odom, and the parent of both
#: sensor prims (so the sensors follow the robot with no per-step bookkeeping).
CHASSIS_PRIM = f"{ROBOT_PRIM}/base"
#: Everything world.yaml spawns lives under one scope, so "what did this file
#: add to the warehouse?" is a single subtree.
PROPS_ROOT = "/World/props"

#: Drop height of the robot's base at spawn, metres. MEASURED: from z = 0.40 the
#: settle slides the base 0.117 m before the policy has any say; from z = 0.32 it
#: slides 0.0197 m and pitches +0.013 rad. The settled standing height is
#: 0.279–0.288 m, so 0.32 is a 3–4 cm drop — enough to guarantee floor contact
#: (a robot spawned exactly at stance height can start interpenetrating the
#: floor) without a launch. The stance itself is NOT written at boot: the robot
#: falls from here in its USD pose and the policy's first joint targets stand it
#: up, which is also what the ~0.97 m forward lunge at policy activation is.
SPAWN_Z = 0.32

#: 200 Hz physics / 50 Hz render. The physics step is a property of the trained
#: plant (the policy was trained at 0.005 s with 4x decimation), so it belongs to
#: the robot, not to a preference. Rendering at every 4th physics step is what
#: the camera and the RTX lidar were measured to need to hold 10 Hz.
PHYSICS_DT = 0.005
RENDERING_DT = 0.02

#: Kit launch config. Headless (there is no display in the container) plus an
#: explicit texture-streaming budget: by default Kit reserves 60 % of TOTAL GPU
#: memory for texture streaming, which is fine for one instance and the first
#: thing to blow up when a second one starts. Pinning it makes the footprint
#: predictable (~4.8–5.0 GiB with the camera on; warm boot ≈ 25 s).
LAUNCH_CONFIG = {
    "headless": True,
    "extra_args": ["--/rtx-transient/resourcemanager/texturestreaming/memoryBudget=0.6"],
}

# --------------------------------------------------------------------------- #
# Prop registry — the assets world.yaml may name (hardware twin of the "set").
# --------------------------------------------------------------------------- #
BOX_ASSET = "box"
#: The built-in box: width (x) x depth (y) x height (z), metres. Low and wide —
#: it is the generic obstacle, sized to meet the chassis rather than to be
#: stepped over.
BOX_SIZE_M = (1.2, 0.4, 0.15)


@dataclass(frozen=True)
class PropAsset:
    """One referenceable prop: the USD, and how far its origin sits off the floor.

    ``z_offset`` is MEASURED per asset — it is the correction that puts the
    prop's lowest geometry exactly on z = 0. A guessed offset either floats the
    prop or sinks it through the floor, and both read as "the obstacle did
    nothing"; that is why an asset without a measured offset does not belong in
    this table, and why ``z`` is never a world.yaml input.
    """

    usd_path: str
    z_offset: float = 0.0


PROP_ASSETS: dict[str, PropAsset] = {
    # bbox 0.600 x 0.599 x 0.877 m; its own origin is on its footprint.
    "chair": PropAsset("/Isaac/Environments/Office/Props/SM_Chair.usd"),
    # bbox 0.800 x 2.800 x 0.750 m — nearly 3 m long, so it blocks an aisle.
    "desk": PropAsset("/Isaac/Environments/Office/Props/SM_SecretaryDeskA.usd"),
    # bbox 1.214 x 3.495 x 2.155 m; the only measured non-zero offset of the set.
    "forklift": PropAsset("/Isaac/Props/Forklift/forklift.usd", z_offset=0.001907),
    # bbox 1.765 x 0.441 x 1.732 m, min z -0.1248 -> +0.1248 puts the feet on the
    # floor. A static mannequin, and it spawns in BIND POSE (arms straight out),
    # which is why it is 1.76 m WIDE and not shoulder-width — see world.yaml.
    "person": PropAsset(
        "/Isaac/People/Characters/F_Business_02/F_Business_02.usd", z_offset=0.1248
    ),
}

# --------------------------------------------------------------------------- #
# Locomotion hardware — the actuator, and the order its joints answer in.
# --------------------------------------------------------------------------- #
#: This IS the articulation's own dof order (hip-major, not leg-major). The
#: observation and action vectors are positional, so a mismatch here scrambles
#: legs silently; the loop therefore remaps by NAME and refuses an articulation
#: whose joint SET differs. (URDF / Unitree-SDK vectors are leg-major — anything
#: copied from those needs reordering.)
JOINT_ORDER = (
    "FL_hip_joint",
    "FR_hip_joint",
    "RL_hip_joint",
    "RR_hip_joint",
    "FL_thigh_joint",
    "FR_thigh_joint",
    "RL_thigh_joint",
    "RR_thigh_joint",
    "FL_calf_joint",
    "FR_calf_joint",
    "RL_calf_joint",
    "RR_calf_joint",
)
ACTION_DIM = 12
OBS_DIM = 48

#: The ONE observation layout this file assembles. It is a NAME, and the policy
#: declares the same name in policy_meta.yaml — a retrain that changes the
#: observation (adds a height scan, reorders a term) must change this string too,
#: and then this harness refuses the policy instead of feeding it 48 numbers in
#: the wrong order. Layout (no normalization, no noise, no clipping anywhere):
#:   [ 0: 3] base linear velocity, in the body frame
#:   [ 3: 6] base angular velocity, in the body frame
#:   [ 6: 9] gravity direction, in the body frame (unit vector)
#:   [ 9:12] velocity command (vx, vy, wz) — /cmd_vel, verbatim
#:   [12:24] joint positions in JOINT_ORDER, minus the default stance
#:   [24:36] joint velocities in JOINT_ORDER
#:   [36:48] the PREVIOUS step's raw policy output (pre-scale); zeros at reset
OBS_LAYOUT_ID = "go2_flat_obs48_v1"

#: Gravity direction used by the observation — a unit vector, not 9.81: the term
#: is dimensionless.
GRAVITY_DIRECTION_W = (0.0, 0.0, -1.0)

#: DC-motor limits of the real actuator, N·m and rad/s. The joints are driven by
#: an EXPLICIT motor model computed here and applied as joint efforts, with the
#: simulator's own PD drive forced to zero — see PolicyLoop.bind.
EFFORT_LIMIT = 23.5  # continuous torque
SATURATION_EFFORT = 23.5  # stall torque
VELOCITY_LIMIT = 30.0  # no-load speed
#: The speed used for the saturation curve is clipped to this before it is used.
#: That clip is what keeps the torque window ORDERED (tau_min <= tau_max) at any
#: joint speed: at +60 rad/s the motor can only pull -23.5 N·m, at -60 only
#: +23.5, and unclipped speeds would invert the window.
VEL_AT_EFFORT_LIM = 60.0
#: The simulator's drive gains, forced to zero after reset (see bind()).
SIM_DRIVE_STIFFNESS = 0.0
SIM_DRIVE_DAMPING = 0.0

#: Name of the physics callback that runs the policy. Distinct name = distinct
#: callback; registering two under one name silently replaces the first.
POLICY_CALLBACK_NAME = "go2_policy"

# --------------------------------------------------------------------------- #
# Sensors — mounts, optics, and the frames they are stamped with.
# --------------------------------------------------------------------------- #
FRAME_ODOM = "odom"
FRAME_BASE = "base_link"
FRAME_CAMERA = "go2_camera"
FRAME_LIDAR = "go2_lidar"

#: Camera mount, base_link -> camera, metres. MEASURED: the base subtree spans
#: x [-0.128, +0.332] around the base origin, so 0.28 m forward sits at the head
#: and renders with zero self-occlusion, while keeping 5 cm of margin to the head
#: tip (a future asset revision then does not put the lens outside the robot).
CAMERA_MOUNT_XYZ = (0.28, 0.0, 0.12)
#: base_link -> camera OPTICAL frame (x right, y down, z forward), w-first. The
#: images are stamped with THIS frame, so a pixel unprojected with camera_info
#: lands in it directly and tf2 alone carries a detection to map — no second
#: "camera_link" frame, and no optical-rotation trap in the app.
CAMERA_OPTICAL_QUAT_WXYZ = (0.5, -0.5, 0.5, -0.5)
CAMERA_RESOLUTION = (640, 480)
#: ``set_focal_length`` takes STAGE units = the USD attribute / 10. MEASURED
#: trap: 12.0 writes focalLength 120 and yields a 9.98° horizontal FOV — a frame
#: with nothing in it. 1.2 writes the intended 12 against the stock 20.955
#: aperture: 82.25° x 66.44°, fx = fy = 366.50 px at 640x480.
CAMERA_FOCAL_LENGTH_STAGE_UNITS = 1.2
#: MEASURED: the stock near clip is 1.0 m, which blacked out the bottom 23 % of
#: every frame and floored the depth image at 1.000 m. At 0.05 m the same view is
#: 95 % finite and reads 0.607 m at the nearest floor pixel.
CAMERA_CLIPPING_RANGE_M = (0.05, 100.0)
#: A pinhole render has no lens distortion; "all zeros" is spelled plumb_bob,
#: and consumers switch on that string.
CAMERA_DISTORTION_MODEL = "plumb_bob"
CAMERA_DISTORTION_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0)
RGB_ENCODING = "rgb8"
DEPTH_ENCODING = "32FC1"

#: Lidar mount, base_link -> lidar, metres. MEASURED with 3200-beam scans in this
#: warehouse: at z = 0.00 the trunk occludes the sensor completely (0 valid
#: returns of 3200), at z = +0.15 it clears the trunk by 6 cm and returns 2723,
#: and the scan plane then sits ~0.43 m above the floor while standing — low
#: enough that a 0.877 m chair and a 1.73 m person are both in it.
LIDAR_MOUNT_XYZ = (0.0, 0.0, 0.15)
#: 3200 beams, 360°, 0.1125° resolution, 10 Hz, range [0.05, 30] m. Chosen by
#: measuring six stock configs at the same mount: the others fail on minimum
#: range (one has a 1.0 m blind ring around a 0.7 m robot) or on field of view.
LIDAR_CONFIG = "RPLIDAR_S2E"
#: MEASURED: a ray with no return comes back as -1.0 — not 0.0, not NaN. ROS says
#: "out of range" is +inf and nav2 reads it that way, so the mapping happens once
#: here instead of in every consumer.
LIDAR_NO_RETURN = -1.0

#: Publish rates, Hz, all gated on SIM time (the app's clock is /clock, and this
#: process is its only source). /clock itself is published on EVERY step: it is
#: not a sensor, it is the time base.
ODOM_RATE_HZ = 30.0
CAMERA_RATE_HZ = 10.0
SCAN_RATE_HZ = 10.0  # = the lidar's own rotation rate

TOPIC_CLOCK = "/clock"
TOPIC_ODOM = "/odom"
TOPIC_TF = "/tf"
TOPIC_TF_STATIC = "/tf_static"
TOPIC_IMAGE = "/camera/image_raw"
TOPIC_DEPTH = "/camera/depth/image_raw"
TOPIC_CAMERA_INFO = "/camera/camera_info"
TOPIC_SCAN = "/scan"
TOPIC_CMD_VEL = "/cmd_vel"

#: Everything is published RELIABLE / KEEP_LAST(5): a reliable publisher
#: satisfies both reliable and best-effort subscribers, and nav2 mixes the two
#: (sensor QoS on scans, system defaults on odom). Depth 5 bounds the buffer, so
#: a slow subscriber drops old frames instead of stalling the sim.
QOS_DEPTH = 5
#: /tf_static MUST be transient-local: it is published once and every node that
#: starts later still has to receive it. That is the tf2 contract.
TF_STATIC_QOS_DEPTH = 1

#: rclpy callbacks drained per sim step. One spin_once executes at most ONE
#: callback, so this is a budget, not a timeout: 16 is what the measured message
#: mix (one /cmd_vel plus service/parameter traffic) needs to stay drained
#: without letting a chatty app starve the physics loop.
SPIN_PER_STEP = 16

NODE_NAME = "go2_patrol_world"

#: Where policy_meta.yaml is looked for: next to the weights, always.
POLICY_META_FILENAME = "policy_meta.yaml"
REQUIRED_META_KEYS = ("obs_layout", "decimation", "action_scale", "kp", "kd", "default_joint_pos")


class ConfigError(Exception):
    """Bad input or missing consent — reported plainly and exits 2, never a stack."""


# --------------------------------------------------------------------------- #
# Small pure helpers.
# --------------------------------------------------------------------------- #
def quat_apply_inverse(quat_wxyz, vec) -> tuple[float, float, float]:
    """Rotate ``vec`` from the world frame into the body frame of ``quat_wxyz``.

    The quaternion is W-FIRST (w, x, y, z) — the order Isaac's ``get_world_pose``
    returns, and the opposite of the x-y-z-w order every ROS message field uses.
    Passing an xyzw quaternion here does not raise, it just silently rotates the
    observation, which is why the convention is in the name.

        t = -2 * (xyz x v);   result = v + w*t - (xyz x t)
    """
    w, x, y, z = (float(v) for v in quat_wxyz)
    vx, vy, vz = (float(v) for v in vec)
    tx = -2.0 * (y * vz - z * vy)
    ty = -2.0 * (z * vx - x * vz)
    tz = -2.0 * (x * vy - y * vx)
    return (
        vx + w * tx - (y * tz - z * ty),
        vy + w * ty - (z * tx - x * tz),
        vz + w * tz - (x * ty - y * tx),
    )


def yaw_to_quat_wxyz(yaw: float) -> tuple[float, float, float, float]:
    """Planar +Z rotation as a w-first quaternion (the order Isaac wants)."""
    half = float(yaw) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def quat_wxyz_to_xyzw(quat_wxyz) -> tuple[float, float, float, float]:
    """Isaac's scalar-FIRST quaternion -> the ROS scalar-LAST wire order.

    ONE reorder site in this file. A swap here is silent: the robot simply faces
    the wrong way in rviz and the app's AMCL diverges.
    """
    w, x, y, z = (float(v) for v in quat_wxyz)
    return (x, y, z, w)


def sim_time_stamp(sim_time_s: float) -> tuple[int, int]:
    """Sim seconds -> a ROS ``(sec, nanosec)`` stamp.

    Floor, not round, on the seconds: a stamp must never name a moment the sim
    has not reached. The nanosecond rounding can carry, and is carried explicitly
    because the field is unsigned.
    """
    sec = int(math.floor(sim_time_s))
    nanosec = int(round((sim_time_s - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return sec, nanosec


def reorder(values, dof_index: tuple[int, ...]) -> tuple[float, ...]:
    """Articulation dof order -> JOINT_ORDER."""
    return tuple(float(values[i]) for i in dof_index)


def scatter(values, dof_index: tuple[int, ...]) -> list[float]:
    """JOINT_ORDER -> articulation dof order (the order the sim is written in)."""
    out = [0.0] * len(dof_index)
    for slot, index in enumerate(dof_index):
        out[index] = float(values[slot])
    return out


def dc_motor_torque(q_target, q, qdot, kp: float, kd: float) -> tuple[float, ...]:
    """Explicit DC-motor torque, per joint::

        tau     = kp*(q_target - q) - kd*qdot
        v       = clip(qdot, -60, +60)
        tau_max = min(23.5 * ( 1 - v/30),  23.5)
        tau_min = max(23.5 * (-1 - v/30), -23.5)
        applied = clip(tau, tau_min, tau_max)

    This speed-dependent saturation is the whole reason the torque is computed
    here instead of being handed to the simulator as a position target with PD
    gains. The two look identical on paper and are different plants: with a PhysX
    drive the saturation simply would not exist, and the only symptom would be
    "the robot walks badly". The clamp order is min(max(...)), so even a window
    that inverted (only reachable if the limits are re-measured unequal) resolves
    to tau_max rather than to whichever bound happened to be applied first.
    """
    applied = []
    for target, pos, vel in zip(q_target, q, qdot, strict=True):
        tau = kp * (target - pos) - kd * vel
        v = min(max(vel, -VEL_AT_EFFORT_LIM), VEL_AT_EFFORT_LIM)
        tau_max = min(SATURATION_EFFORT * (1.0 - v / VELOCITY_LIMIT), EFFORT_LIMIT)
        tau_min = max(SATURATION_EFFORT * (-1.0 - v / VELOCITY_LIMIT), -EFFORT_LIMIT)
        applied.append(min(max(tau, tau_min), tau_max))
    return tuple(applied)


def pinhole_intrinsics(width, height, focal, h_aperture, v_aperture):
    """USD camera attributes -> ``(fx, fy, cx, cy)`` in pixels.

    Computed from attributes READ BACK off the prim rather than from the
    constants we meant to write: what goes on the wire has to describe the camera
    that actually rendered the frame. The ratio is unit-free, so stage units and
    raw USD tenths both work — as long as both arguments use the same one.
    """
    fx = width * focal / h_aperture
    fy = height * focal / v_aperture
    return fx, fy, width * 0.5, height * 0.5


class RateGate:
    """ "Is this stream due at sim-time t?" — keyed on sim time, not on steps.

    Gating on the clock rather than on a step count keeps the published rates
    right regardless of physics dt or render decimation. The deadline advances by
    exactly one period, so a period that is a whole number of steps drifts by
    nothing; after a stall longer than one period it is re-based on now instead
    of firing a burst of catch-up messages, because the sim is the only clock —
    a "late" sensor message is not late, it is about a moment that has passed.
    """

    def __init__(self, rate_hz: float) -> None:
        self.period_s = 1.0 / float(rate_hz)
        self._next: float | None = None

    def due(self, sim_time_s: float) -> bool:
        if self._next is not None and sim_time_s + 1e-9 < self._next:
            return False
        base = sim_time_s if self._next is None else self._next
        self._next = base + self.period_s
        if self._next <= sim_time_s:
            self._next = sim_time_s + self.period_s
        return True


class FirstDataGate:
    """Says once when a stream starts producing — and once when it never does.

    The failure this exists for is silence, not an exception: a mis-mounted or
    uninitialised RTX lidar returns an EMPTY array forever. The topic exists, the
    rate looks right, and every range is missing. The same shape of silence was
    measured from a sensor buried inside the robot's own trunk (0 of 3200 beams).
    """

    def __init__(self, name: str, patience_s: float = 2.0) -> None:
        self.name = name
        self.patience_s = patience_s
        self._seen = False
        self._warned = False
        self._start: float | None = None

    def observe(self, count: int, sim_time_s: float) -> None:
        if self._start is None:
            self._start = sim_time_s
        if count > 0:
            if not self._seen:
                self._seen = True
                print(f"{LOG} {self.name}: first data at t={sim_time_s:.3f}s ({count} samples)")
            return
        if not self._warned and sim_time_s - self._start >= self.patience_s:
            self._warned = True
            print(
                f"{LOG} WARNING: {self.name} has produced EMPTY frames for "
                f"{sim_time_s - self._start:.1f}s of sim time — the sensor is attached but "
                "returns nothing (this is how an RTX lidar fails: silently, not by raising)"
            )


# --------------------------------------------------------------------------- #
# world.yaml — the app's own world description.
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class PropSpec:
    """One prop the world file asks for. ``z`` is never here: the floor owns it."""

    asset: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class WorldSpec:
    spawn_x: float
    spawn_y: float
    spawn_yaw: float
    props: tuple[PropSpec, ...]


def _require_mapping(value, what: str) -> dict:
    if not isinstance(value, dict):
        raise ConfigError(f"{what} must be a mapping, got {type(value).__name__}")
    return value


def _number(mapping: dict, key: str, what: str, default: float | None = None) -> float:
    if key not in mapping:
        if default is None:
            raise ConfigError(f"{what} is missing required key {key!r}")
        return default
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{what}.{key} must be a number, got {value!r}")
    return float(value)


def _reject_unknown_keys(mapping: dict, allowed: tuple[str, ...], what: str) -> None:
    """A typo'd key is bad input, not a shrug.

    ``theta: 1.57`` next to a defaulted ``yaw`` would place the prop at yaw 0 and
    say nothing, and the only symptom would be a world that does not look like
    the file that describes it.
    """
    unknown = sorted(set(mapping) - set(allowed))
    if unknown:
        raise ConfigError(f"{what} has unknown key(s) {unknown} — known keys: {list(allowed)}")


def load_world_file(path: Path) -> WorldSpec:
    """Parse and validate world.yaml. Read ONCE, before the stage is opened.

    Failing here costs no GPU seconds; failing after the boot costs the boot. So
    every name is resolved against the prop registry now, rather than at the
    moment a reference fails to compose.
    """
    if not path.is_file():
        raise ConfigError(f"world file not found: {path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"{path} is not valid YAML: {exc}") from exc
    document = _require_mapping(document, str(path))
    _reject_unknown_keys(document, ("spawn", "props"), str(path))

    spawn = _require_mapping(document.get("spawn", {}), f"{path}: spawn")
    _reject_unknown_keys(spawn, ("x", "y", "yaw"), f"{path}: spawn")
    raw_props = document.get("props") or []
    if not isinstance(raw_props, list):
        raise ConfigError(f"{path}: props must be a list, got {type(raw_props).__name__}")

    props = []
    for index, entry in enumerate(raw_props):
        what = f"{path}: props[{index}]"
        entry = _require_mapping(entry, what)
        _reject_unknown_keys(entry, ("asset", "x", "y", "yaw"), what)
        asset = entry.get("asset")
        if asset not in PROP_ASSETS and asset != BOX_ASSET:
            raise ConfigError(
                f"{what} names unknown asset {asset!r} — available: "
                f"{sorted([*PROP_ASSETS, BOX_ASSET])}"
            )
        props.append(
            PropSpec(
                asset=str(asset),
                x=_number(entry, "x", what),
                y=_number(entry, "y", what),
                # yaw is optional because a box is square-ish and most props are
                # placed by position first; 0 is a real answer, not a fallback.
                yaw=_number(entry, "yaw", what, default=0.0),
            )
        )
    return WorldSpec(
        spawn_x=_number(spawn, "x", f"{path}: spawn"),
        spawn_y=_number(spawn, "y", f"{path}: spawn"),
        spawn_yaw=_number(spawn, "yaw", f"{path}: spawn", default=0.0),
        props=tuple(props),
    )


# --------------------------------------------------------------------------- #
# policy_meta.yaml — the policy's own contract, read from next to the weights.
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class PolicyMeta:
    """What the trained policy expects of the plant driving it.

    Every field here is a TRAINING artefact. It is read from the file next to the
    weights so that retraining ships one coherent pair (weights + contract), and
    so that this harness can refuse a policy it cannot honour instead of walking
    the robot with the wrong numbers.
    """

    obs_layout: str
    decimation: int
    action_scale: float
    kp: float
    kd: float
    default_joint_pos: tuple[float, ...]  # in JOINT_ORDER


def load_policy_meta(policy_path: Path) -> PolicyMeta:
    """Load ``policy_meta.yaml`` from the policy's own directory and validate it."""
    meta_path = policy_path.parent / POLICY_META_FILENAME
    if not meta_path.is_file():
        raise ConfigError(
            f"{meta_path} not found — a locomotion policy is shipped as a PAIR: the weights "
            f"({policy_path.name}) and the contract they were trained under "
            f"({POLICY_META_FILENAME}, in the same directory)"
        )
    try:
        document = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"{meta_path} is not valid YAML: {exc}") from exc
    document = _require_mapping(document, str(meta_path))

    missing = [key for key in REQUIRED_META_KEYS if key not in document]
    if missing:
        raise ConfigError(
            f"{meta_path} is missing required key(s) {missing} — required: "
            f"{list(REQUIRED_META_KEYS)}"
        )

    layout = str(document["obs_layout"])
    if layout != OBS_LAYOUT_ID:
        raise ConfigError(
            f"{meta_path} declares obs_layout {layout!r}, but this world harness assembles "
            f"exactly one observation layout: {OBS_LAYOUT_ID!r}. The two must match — a "
            "policy fed 48 numbers in another order does not fail, it just walks wrong. "
            "Either point --policy at a policy trained on this layout, or teach "
            f"patrol_world.py the {layout!r} layout and rename OBS_LAYOUT_ID."
        )

    # The stance is a MAPPING joint name -> radians, not a list: the ordering of
    # the action vector is this harness's constant (JOINT_ORDER), and a bare list
    # in a yaml file is exactly the thing that gets edited into the wrong order
    # by someone who cannot see that constant.
    stance = _require_mapping(document["default_joint_pos"], f"{meta_path}: default_joint_pos")
    unexpected = sorted(set(stance) - set(JOINT_ORDER))
    absent = sorted(set(JOINT_ORDER) - set(stance))
    if unexpected or absent:
        raise ConfigError(
            f"{meta_path}: default_joint_pos must name exactly the 12 Go2 joints — "
            f"missing {absent}, unexpected {unexpected}"
        )
    default_joint_pos = tuple(
        _number(stance, joint, f"{meta_path}: default_joint_pos") for joint in JOINT_ORDER
    )

    decimation = document["decimation"]
    if not isinstance(decimation, int) or isinstance(decimation, bool) or decimation < 1:
        raise ConfigError(f"{meta_path}: decimation must be an integer >= 1, got {decimation!r}")
    return PolicyMeta(
        obs_layout=layout,
        decimation=decimation,
        action_scale=_number(document, "action_scale", str(meta_path)),
        kp=_number(document, "kp", str(meta_path)),
        kd=_number(document, "kd", str(meta_path)),
        default_joint_pos=default_joint_pos,
    )


# --------------------------------------------------------------------------- #
# The control loop: 50 Hz policy, 200 Hz torque.
# --------------------------------------------------------------------------- #
class PolicyLoop:
    """Drives the robot's twelve joints from the trained policy, in-process.

    On the real Go2 this loop runs ON the robot; the mirror of that placement is
    this process, not the app. The app never sees a joint — it publishes
    ``/cmd_vel``, exactly as it would to a real base.

    Lifecycle, and the ORDER matters:

    * ``load()`` — any time after the simulator app exists (it imports torch);
    * ``bind(articulation)`` — only AFTER ``world.reset()``. Before reset the
      articulation view has no dof names, ``initialize()`` raises, and — worst of
      the three — ``set_gains`` returns having done NOTHING. A silent gain write
      would leave the simulator's own PD controller fighting the policy, which is
      precisely the failure this ordering exists to prevent;
    * ``on_physics_step()`` — from a physics callback, EVERY step.
    """

    def __init__(self, policy_path: Path, meta: PolicyMeta) -> None:
        self.policy_path = policy_path
        self.meta = meta
        self._torch = None
        self._policy = None
        self._articulation = None
        self._dof_index: tuple[int, ...] = ()
        self._command = (0.0, 0.0, 0.0)
        self._last_action = (0.0,) * ACTION_DIM
        self._joint_target = meta.default_joint_pos
        self._step = 0

    def load(self) -> None:
        """TorchScript-load the policy onto the CPU, single-threaded.

        A 3x128 MLP at 50 Hz costs nothing on a CPU core, and both alternatives
        cost determinism: multi-threaded reductions can reassociate, and a GPU
        forward adds a host<->device sync inside the physics callback.
        """
        if not self.policy_path.is_file():
            raise ConfigError(f"locomotion policy not found: {self.policy_path}")
        import torch  # noqa: PLC0415 (legal only after SimulationApp exists)

        torch.set_num_threads(1)
        self._torch = torch
        self._policy = torch.jit.load(str(self.policy_path), map_location="cpu")
        self._policy.eval()
        print(f"{LOG} policy loaded: {self.policy_path} (layout {self.meta.obs_layout})")

    def bind(self, articulation) -> None:
        """Map dof names to JOINT_ORDER and zero the simulator's drive gains."""
        names = tuple(str(n) for n in articulation.dof_names)
        if sorted(names) != sorted(JOINT_ORDER):
            raise RuntimeError(
                "the articulation's joints are not the trained Go2 joint set — missing "
                f"{sorted(set(JOINT_ORDER) - set(names))}, unexpected "
                f"{sorted(set(names) - set(JOINT_ORDER))}. Observations and actions are "
                "positional, so a different asset would scramble legs silently."
            )
        self._dof_index = tuple(names.index(joint) for joint in JOINT_ORDER)
        if names != JOINT_ORDER:
            # Remapping by name still works; the asset having a different order
            # is NEWS, because it means the robot changed under a policy trained
            # on the old one.
            print(f"{LOG} note: dof order differs from the trained order — remapping by name")
        # All torque comes from dc_motor_torque, so the simulator's PD drive must
        # contribute nothing. Uniform zeros, hence dof-order independent.
        articulation.get_articulation_controller().set_gains(
            kps=[SIM_DRIVE_STIFFNESS] * len(names), kds=[SIM_DRIVE_DAMPING] * len(names)
        )
        self._articulation = articulation
        print(f"{LOG} policy bound to {len(names)} joints; sim drive gains zeroed")

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        """Latch the base velocity command (/cmd_vel -> observation [9:12]).

        Latched, not queued: a velocity command is a latest-value signal, and
        replaying old ones would drive the robot with the app's past. Passed
        through verbatim — no deadzone, no scaling, no clipping — because what
        velocities the app is allowed to ask for is the app's own configuration
        (nav2's velocity_smoother is the place that caps them).
        """
        self._command = (float(vx), float(vy), float(wz))

    def on_physics_step(self) -> None:
        """One physics step: the policy every Nth, the torque EVERY time.

        The split is the trained semantics: an action is computed once per
        control period, then the motor model runs on the CURRENT joint state for
        each physics sub-step. Recomputing the torque every step is what makes
        the motor's velocity term a real 200 Hz damping term instead of a 50 Hz
        staircase — and it is why ``set_joint_efforts`` is called unconditionally
        below, outside the decimation branch.
        """
        art = self._articulation
        joint_pos = reorder(art.get_joint_positions(), self._dof_index)
        joint_vel = reorder(art.get_joint_velocities(), self._dof_index)
        if self._step % self.meta.decimation == 0:
            obs = self._observe(joint_pos, joint_vel)  # reads the PREVIOUS action
            self._last_action = self._forward(obs)
            # target = stance + scale * raw. No output clipping: an exploding
            # network is meant to be visible as an exploding robot, not squeezed
            # into a plausible pose.
            self._joint_target = tuple(
                default + self.meta.action_scale * raw
                for default, raw in zip(self.meta.default_joint_pos, self._last_action, strict=True)
            )
        self._step += 1
        torque = dc_motor_torque(
            self._joint_target, joint_pos, joint_vel, self.meta.kp, self.meta.kd
        )
        art.set_joint_efforts(scatter(torque, self._dof_index))

    def _observe(self, joint_pos, joint_vel) -> list[float]:
        """Assemble the 48-D observation described by OBS_LAYOUT_ID."""
        art = self._articulation
        _, quat = art.get_world_pose()  # ground-truth pose, w-first quaternion
        obs: list[float] = []
        obs.extend(quat_apply_inverse(quat, art.get_linear_velocity()))
        obs.extend(quat_apply_inverse(quat, art.get_angular_velocity()))
        obs.extend(quat_apply_inverse(quat, GRAVITY_DIRECTION_W))
        obs.extend(self._command)
        obs.extend(
            q - d for q, d in zip(joint_pos, self.meta.default_joint_pos, strict=True)
        )
        obs.extend(joint_vel)  # the trained default joint velocity is zero
        obs.extend(self._last_action)  # RAW output of the previous step, pre-scale
        if len(obs) != OBS_DIM:
            raise RuntimeError(f"assembled {len(obs)} observations, expected {OBS_DIM}")
        return obs

    def _forward(self, obs) -> tuple[float, ...]:
        torch = self._torch
        with torch.inference_mode():
            out = self._policy(torch.tensor([list(obs)], dtype=torch.float32))
        raw = tuple(float(v) for v in out[0])
        if len(raw) != ACTION_DIM:
            raise RuntimeError(
                f"the policy returned {len(raw)} values, expected {ACTION_DIM} — "
                f"{self.policy_path} is not an obs{OBS_DIM}/act{ACTION_DIM} policy"
            )
        return raw


# --------------------------------------------------------------------------- #
# ROS message types (importable only once the bridge extension is enabled).
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class RosTypes:
    """The bundled-Jazzy message classes plus two QoS factories."""

    Clock: type
    TFMessage: type
    TransformStamped: type
    Odometry: type
    Image: type
    CameraInfo: type
    LaserScan: type
    Twist: type
    TwistStamped: type
    qos: object
    best_effort_qos: object


def import_ros_types() -> RosTypes:
    """Import ROS 2 message types from the simulator's bundled Jazzy.

    Deferred on purpose: those packages only land on ``sys.path`` once the
    ``isaacsim.ros2.bridge`` extension has been enabled, so importing them at
    module scope would fail before the harness has even printed why.
    """
    from geometry_msgs.msg import TransformStamped, Twist, TwistStamped  # noqa: PLC0415
    from nav_msgs.msg import Odometry  # noqa: PLC0415
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: PLC0415
    from rosgraph_msgs.msg import Clock  # noqa: PLC0415
    from sensor_msgs.msg import CameraInfo, Image, LaserScan  # noqa: PLC0415
    from tf2_msgs.msg import TFMessage  # noqa: PLC0415

    def qos(depth: int = QOS_DEPTH, transient_local: bool = False):
        return QoSProfile(
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL if transient_local else DurabilityPolicy.VOLATILE
            ),
        )

    def best_effort_qos(depth: int = 1):
        return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT)

    return RosTypes(
        Clock=Clock,
        TFMessage=TFMessage,
        TransformStamped=TransformStamped,
        Odometry=Odometry,
        Image=Image,
        CameraInfo=CameraInfo,
        LaserScan=LaserScan,
        Twist=Twist,
        TwistStamped=TwistStamped,
        qos=qos,
        best_effort_qos=best_effort_qos,
    )


# --------------------------------------------------------------------------- #
# The sensor rig: the prims, and everything published from them.
# --------------------------------------------------------------------------- #
class SensorRig:
    """Camera + RTX lidar + ground-truth odometry, and their ROS publishers.

    Three phases, in this order, because each one is only legal in its window:

    * ``author_prims()`` — BEFORE ``world.reset()``: creates the camera and lidar
      prims (they must exist when physics/render products are built) and the
      chassis view used for ground truth (a rigid-body view created after reset
      is already invalidated);
    * ``initialize()`` — AFTER reset: without this the lidar's annotator returns
      an empty array forever, and the camera's focal length / clipping range are
      only settable once its render product exists;
    * ``attach(node, types)`` then ``publish(t)`` every step.
    """

    def __init__(self) -> None:
        self._body = None
        self._camera = None
        self._lidar = None
        self._types: RosTypes | None = None
        self._pubs: dict = {}
        self._camera_info = None
        self._odom_gate = RateGate(ODOM_RATE_HZ)
        self._camera_gate = RateGate(CAMERA_RATE_HZ)
        self._scan_gate = RateGate(SCAN_RATE_HZ)
        self._data = {
            "rgb": FirstDataGate("camera rgb"),
            "depth": FirstDataGate("camera depth"),
            "scan": FirstDataGate("lidar scan"),
        }

    # --- stage side -------------------------------------------------------- #
    def author_prims(self) -> None:
        """Create the sensor prims under the chassis (pre-reset)."""
        from isaacsim.core.prims import SingleRigidPrim  # noqa: PLC0415
        from isaacsim.sensors.camera import Camera  # noqa: PLC0415
        from isaacsim.sensors.rtx import LidarRtx  # noqa: PLC0415

        self._body = SingleRigidPrim(CHASSIS_PRIM)
        # Both prims are CHILDREN of the chassis, so the mounts are local offsets
        # and the sensors follow the robot with no per-step transform writing.
        # The camera's orientation is IDENTITY on purpose: this wrapper reads its
        # pose in world axes (+X forward, +Z up), so identity already looks down
        # the robot's nose — MEASURED, after passing the USD-frame quaternion
        # produced a 90°-rotated image.
        self._camera = Camera(
            prim_path=f"{CHASSIS_PRIM}/{FRAME_CAMERA}",
            name=FRAME_CAMERA,
            resolution=CAMERA_RESOLUTION,
            translation=CAMERA_MOUNT_XYZ,
            orientation=(1.0, 0.0, 0.0, 0.0),
        )
        self._lidar = LidarRtx(
            prim_path=f"{CHASSIS_PRIM}/{FRAME_LIDAR}",
            name=FRAME_LIDAR,
            translation=LIDAR_MOUNT_XYZ,
            config_file_name=LIDAR_CONFIG,
        )
        # The flat-scan annotator is what turns the 3D RTX sensor into the single
        # ring of ranges a LaserScan carries; nav2 wants the ring.
        self._lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")

    def initialize(self) -> None:
        """Post-reset initialisation and the optics that need a render product."""
        self._camera.initialize()
        self._camera.set_focal_length(CAMERA_FOCAL_LENGTH_STAGE_UNITS)
        self._camera.set_clipping_range(*CAMERA_CLIPPING_RANGE_M)
        # Depth is an OPT-IN annotator: without this the frame carries rgb only.
        self._camera.add_distance_to_image_plane_to_frame()
        # Without this the annotator's acquisition callback is never registered
        # and every scan comes back empty — forever, and without an error.
        self._lidar.initialize()

    # --- ROS side ---------------------------------------------------------- #
    def attach(self, node, types: RosTypes) -> None:
        """Create the publishers and latch the static transforms."""
        self._types = types
        self._pubs = {
            "clock": node.create_publisher(types.Clock, TOPIC_CLOCK, types.qos()),
            "tf": node.create_publisher(types.TFMessage, TOPIC_TF, types.qos()),
            "tf_static": node.create_publisher(
                types.TFMessage,
                TOPIC_TF_STATIC,
                types.qos(depth=TF_STATIC_QOS_DEPTH, transient_local=True),
            ),
            "odom": node.create_publisher(types.Odometry, TOPIC_ODOM, types.qos()),
            "rgb": node.create_publisher(types.Image, TOPIC_IMAGE, types.qos()),
            "depth": node.create_publisher(types.Image, TOPIC_DEPTH, types.qos()),
            "camera_info": node.create_publisher(types.CameraInfo, TOPIC_CAMERA_INFO, types.qos()),
            "scan": node.create_publisher(types.LaserScan, TOPIC_SCAN, types.qos()),
        }
        # Intrinsics are computed from the attributes read BACK off the prim, so
        # camera_info describes the camera that actually rendered the frames.
        width, height = self._camera.get_resolution()
        self._camera_info = pinhole_intrinsics(
            int(width),
            int(height),
            float(self._camera.get_focal_length()),
            float(self._camera.get_horizontal_aperture()),
            float(self._camera.get_vertical_aperture()),
        )
        self._publish_static_transforms()

    def publish(self, sim_time_s: float) -> None:
        """Publish /clock, plus every stream due at this sim time."""
        stamp = sim_time_stamp(sim_time_s)
        clock = self._types.Clock()
        clock.clock.sec, clock.clock.nanosec = stamp
        self._pubs["clock"].publish(clock)
        if self._odom_gate.due(sim_time_s):
            self._publish_odometry(stamp)
        if self._camera_gate.due(sim_time_s):
            # rgb, depth and camera_info go out together with ONE stamp: a
            # consumer unprojecting a depth pixel pairs the three by time, and a
            # depth frame paired with another frame's intrinsics lands the
            # detection somewhere else entirely.
            self._publish_images(stamp, sim_time_s)
            self._publish_camera_info(stamp)
        if self._scan_gate.due(sim_time_s):
            self._publish_scan(stamp, sim_time_s)

    def _publish_odometry(self, stamp) -> None:
        """Ground-truth odometry: pose in ``odom``, twist in ``base_link``.

        This odometry does not drift — it is the simulator's own pose, so the
        ``odom`` frame coincides with the world and the app's occupancy map is
        valid in it. ``map -> odom`` is the app's AMCL to publish; this harness
        never does.

        Odometry splits its frames: the pose is in ``header.frame_id`` and the
        TWIST is in ``child_frame_id``, so the world-frame velocities have to be
        rotated into the body. A world-frame twist reads to a controller as a
        robot driving sideways whenever it is not facing +X.
        """
        types, pubs = self._types, self._pubs
        position, quat = self._body.get_world_pose()
        position = tuple(float(v) for v in position)
        quat = tuple(float(v) for v in quat)
        rotation = quat_wxyz_to_xyzw(quat)
        linear = quat_apply_inverse(quat, self._body.get_linear_velocity())
        angular = quat_apply_inverse(quat, self._body.get_angular_velocity())

        tf_msg = types.TFMessage()
        tf_msg.transforms = [self._transform(stamp, FRAME_ODOM, FRAME_BASE, position, rotation)]
        pubs["tf"].publish(tf_msg)

        msg = types.Odometry()
        self._stamp_header(msg, stamp, FRAME_ODOM)
        msg.child_frame_id = FRAME_BASE
        (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) = position
        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ) = rotation
        (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z) = linear
        (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z) = angular
        pubs["odom"].publish(msg)

    def _publish_images(self, stamp, sim_time_s: float) -> None:
        """Publish the rgb and depth frames, when the renderer has produced them.

        Both readers answer None (or an empty buffer) until the first frame is
        rendered, which is normal for the first steps after boot. Skipping is the
        right answer there: an all-zero image is a lie a consumer cannot tell
        apart from a dark room.
        """
        import numpy as np  # noqa: PLC0415 (legal only after SimulationApp)

        rgba = self._camera.get_rgba()
        rgb = None
        if rgba is not None and getattr(rgba, "size", 0):
            frame = np.asarray(rgba)
            if frame.ndim == 3 and frame.shape[2] >= 3:
                # Drop alpha; ROS rgb8 is 3 bytes per pixel and the row step says so.
                rgb = np.ascontiguousarray(frame[:, :, :3]).astype(np.uint8)
        self._data["rgb"].observe(0 if rgb is None else int(rgb.size), sim_time_s)
        if rgb is not None:
            self._publish_image("rgb", stamp, rgb, RGB_ENCODING, 3)

        raw_depth = self._camera.get_depth()
        depth = None
        if raw_depth is not None and getattr(raw_depth, "size", 0):
            depth = np.ascontiguousarray(np.asarray(raw_depth)).astype(np.float32)
        self._data["depth"].observe(0 if depth is None else int(depth.size), sim_time_s)
        if depth is not None:
            self._publish_image("depth", stamp, depth, DEPTH_ENCODING, 4)

    def _publish_image(self, key: str, stamp, frame, encoding: str, bytes_per_px: int) -> None:
        msg = self._types.Image()
        self._stamp_header(msg, stamp, FRAME_CAMERA)
        msg.height, msg.width = int(frame.shape[0]), int(frame.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(frame.shape[1]) * bytes_per_px
        msg.data = frame.tobytes()
        self._pubs[key].publish(msg)

    def _publish_camera_info(self, stamp) -> None:
        fx, fy, cx, cy = self._camera_info
        width, height = self._camera.get_resolution()
        msg = self._types.CameraInfo()
        self._stamp_header(msg, stamp, FRAME_CAMERA)
        msg.height, msg.width = int(height), int(width)
        msg.distortion_model = CAMERA_DISTORTION_MODEL
        msg.d = list(CAMERA_DISTORTION_COEFFS)
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        # A monocular camera is its own rectified frame: R is identity and P is K
        # with a zero translation column.
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._pubs["camera_info"].publish(msg)

    def _publish_scan(self, stamp, sim_time_s: float) -> None:
        """One flat-scan frame -> LaserScan.

        Every geometric field comes from the annotator's OWN metadata rather than
        from the config we asked for: the sensor is the authority on what it just
        produced, and a hard-coded beam count is how a config change becomes a
        silent mismatch between ``ranges`` and the angles that describe it.
        """
        frame = self._lidar.get_current_frame().get("IsaacComputeRTXLidarFlatScan") or None
        depths = [] if frame is None else list(frame.get("linearDepthData", []))
        self._data["scan"].observe(len(depths), sim_time_s)
        if not depths:
            return
        increment = math.radians(float(frame["horizontalResolution"]))
        angle_min = math.radians(float(frame["azimuthRange"][0]))
        depth_range = [float(v) for v in frame["depthRange"]]
        scan_time = 1.0 / SCAN_RATE_HZ

        msg = self._types.LaserScan()
        # Azimuth 0 is the sensor's +X (the robot's forward) with angles going
        # counter-clockwise — MEASURED with a post 1.5 m off the robot's right,
        # which came back at -49°. That is the LaserScan convention exactly,
        # which is why the static transform below is a pure translation.
        self._stamp_header(msg, stamp, FRAME_LIDAR)
        msg.angle_min = angle_min
        msg.angle_increment = increment
        # Derived rather than copied from the reported azimuth end: consumers
        # compute the bearing of ray i as angle_min + i*increment and range-check
        # it against angle_max, so the derived value cannot contradict the array.
        msg.angle_max = angle_min + increment * (len(depths) - 1)
        msg.scan_time = scan_time
        msg.time_increment = scan_time / len(depths)
        msg.range_min, msg.range_max = depth_range[0], depth_range[1]
        msg.ranges = [math.inf if d <= LIDAR_NO_RETURN else float(d) for d in depths]
        self._pubs["scan"].publish(msg)

    def _publish_static_transforms(self) -> None:
        """Latch base_link -> the two sensor frames, once."""
        msg = self._types.TFMessage()
        # Stamped at sim time 0: /tf_static is latched, a late subscriber gets it
        # on subscription, and tf2 treats a static transform as valid at every
        # time — the stamp is bookkeeping, not a lookup key.
        msg.transforms = [
            self._transform(
                (0, 0),
                FRAME_BASE,
                FRAME_CAMERA,
                CAMERA_MOUNT_XYZ,
                quat_wxyz_to_xyzw(CAMERA_OPTICAL_QUAT_WXYZ),
            ),
            self._transform((0, 0), FRAME_BASE, FRAME_LIDAR, LIDAR_MOUNT_XYZ, (0.0, 0.0, 0.0, 1.0)),
        ]
        self._pubs["tf_static"].publish(msg)

    def _transform(self, stamp, parent: str, child: str, translation, rotation_xyzw):
        msg = self._types.TransformStamped()
        self._stamp_header(msg, stamp, parent)
        msg.child_frame_id = child
        (
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ) = (float(v) for v in translation)
        (
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ) = (float(v) for v in rotation_xyzw)
        return msg

    @staticmethod
    def _stamp_header(msg, stamp, frame_id: str) -> None:
        msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
        msg.header.frame_id = frame_id


class CmdVelBridge:
    """``/cmd_vel`` -> the policy's latched command, in either spelling.

    nav2 publishes ``Twist`` or ``TwistStamped`` depending on how it is built and
    configured, and a subscription of the wrong type matches no publisher at all:
    the robot then stands perfectly still with no error anywhere, which is the
    single most expensive way for this harness to fail. So the subscription
    starts as ``Twist`` (what the app's pinned nav2 publishes today) and, until
    the first command arrives, the topic's advertised type is checked
    occasionally — a ``TwistStamped`` publisher swaps the subscription and says
    so out loud. One subscription exists at a time on purpose: two of different
    types on one topic name is what a DDS implementation is entitled to reject.
    """

    #: Sim seconds between type checks while no command has arrived. Cheap, but
    #: not free (it queries the graph), and nothing needs it to be fast.
    PROBE_PERIOD_S = 1.0

    def __init__(self, node, types: RosTypes, on_command) -> None:
        self._node = node
        self._types = types
        self._on_command = on_command
        self._received = False
        self._stamped = False
        self._next_probe = self.PROBE_PERIOD_S
        self._subscription = self._subscribe(types.Twist)

    def _subscribe(self, msg_type):
        return self._node.create_subscription(
            msg_type, TOPIC_CMD_VEL, self._on_message, self._types.best_effort_qos(1)
        )

    def _on_message(self, message) -> None:
        # TwistStamped carries the payload under `.twist`; Twist IS the payload.
        twist = getattr(message, "twist", message)
        self._received = True
        self._on_command(twist.linear.x, twist.linear.y, twist.angular.z)

    def poll(self, sim_time_s: float) -> None:
        if self._received or self._stamped or sim_time_s < self._next_probe:
            return
        self._next_probe = sim_time_s + self.PROBE_PERIOD_S
        try:
            infos = self._node.get_publishers_info_by_topic(TOPIC_CMD_VEL)
        except Exception:  # graph queries are best-effort; never fatal
            return
        if not any(getattr(i, "topic_type", "").endswith("TwistStamped") for i in infos):
            return
        self._node.destroy_subscription(self._subscription)
        self._subscription = self._subscribe(self._types.TwistStamped)
        self._stamped = True
        print(f"{LOG} {TOPIC_CMD_VEL} is TwistStamped — subscription switched")


# --------------------------------------------------------------------------- #
# World composition.
# --------------------------------------------------------------------------- #
def asset_url(usd_path: str) -> str:
    """Resolve a path under the Isaac assets root into a URL."""
    from isaacsim.storage.native import get_assets_root_path  # noqa: PLC0415

    root = get_assets_root_path()
    if root is None:
        raise RuntimeError(
            "the Isaac assets root is unreachable (no local cache and no network) — "
            f"cannot resolve {usd_path}"
        )
    return root + usd_path


def open_warehouse(simulation_app) -> None:
    """Open the warehouse as THE stage, then pump the app once."""
    import omni.usd  # noqa: PLC0415

    url = asset_url(SCENE_USD)
    # open_stage, not add_reference: this scene is the world, and referencing it
    # under a prim would put every coordinate in the app's map off by a transform.
    if not omni.usd.get_context().open_stage(url):
        raise RuntimeError(f"open_stage failed for {url!r}")
    simulation_app.update()
    print(f"{LOG} stage opened: {url}")


def compose_robot_and_extras(simulation_app) -> None:
    """Reference the extras layer and the robot onto the open stage.

    The update pump after each reference is what makes it actually COMPOSE; a
    traversal that runs before the pump finds an empty prim and quietly does
    nothing (that is how a prop ends up with no collider).
    """
    from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: PLC0415

    add_reference_to_stage(usd_path=asset_url(EXTRAS_USD), prim_path=EXTRAS_PRIM)
    simulation_app.update()
    add_reference_to_stage(usd_path=asset_url(ROBOT_USD), prim_path=ROBOT_PRIM)
    simulation_app.update()
    print(f"{LOG} composed: {EXTRAS_PRIM} (identity) + {ROBOT_PRIM}")


def place_robot(x: float, y: float, yaw: float) -> None:
    """Put the robot at the world.yaml spawn — before reset, still a plain xform."""
    import numpy as np  # noqa: PLC0415
    from isaacsim.core.prims import SingleXFormPrim  # noqa: PLC0415

    SingleXFormPrim(ROBOT_PRIM).set_world_pose(
        position=np.array((x, y, SPAWN_Z)),
        orientation=np.array(yaw_to_quat_wxyz(yaw)),
    )
    print(f"{LOG} robot spawn: x={x} y={y} yaw={yaw} z={SPAWN_Z}")


def make_static_collider(prim_path: str) -> None:
    """Give a referenced prop colliders, and refuse one that brings its own physics.

    Walk the subtree: a collider on every geometric prim, plus a convex-hull
    approximation on every mesh. The one-line vendor helper that looks like this
    was MEASURED not to convexify the children — it leaves them as triangle
    meshes, and every contact then emits a PhysX material warning (thousands per
    run). Contact counts did not separate the two recipes; the warning class did.

    A prop carrying its own rigid body or articulation is refused rather than
    neutralised. Measured on such an asset: with colliders added its contacts
    drop to ZERO (a ghost obstacle the robot drives through), and left dynamic it
    was shoved 297 m out of the scene.
    """
    import omni.usd  # noqa: PLC0415
    from pxr import Usd, UsdGeom, UsdPhysics  # noqa: PLC0415

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(prim_path)
    if not root.IsValid():
        raise RuntimeError(
            f"{prim_path!r} is not on the stage after referencing its asset — the reference "
            "did not compose (bad usd path, or an unreachable assets root)"
        )
    dynamic = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(root)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if dynamic:
        raise RuntimeError(
            f"the asset at {prim_path!r} ships its own dynamics on {len(dynamic)} prim(s) "
            f"{dynamic[:5]} — props in this world are STATIC scenery"
        )
    gprims = meshes = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("convexHull")
            meshes += 1
        gprims += 1
    if not gprims:
        raise RuntimeError(
            f"the asset at {prim_path!r} exposed NO geometry to give a collider to — a prop "
            "with no collider is scenery the robot walks straight through"
        )
    print(f"{LOG}   collider: {prim_path} ({gprims} gprim / {meshes} mesh, convexHull)")


def spawn_props(simulation_app, props) -> None:
    """Spawn every prop world.yaml asked for, under one scope.

    ``z`` comes from the registry (or from half the box height), never from the
    file: floor contact is a property of the asset, and a prop floating 5 cm off
    the ground is an obstacle that reads as present and behaves as absent.
    """
    import numpy as np  # noqa: PLC0415
    from isaacsim.core.api.objects import FixedCuboid  # noqa: PLC0415
    from isaacsim.core.prims import SingleXFormPrim  # noqa: PLC0415
    from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: PLC0415

    for index, spec in enumerate(props):
        prim_path = f"{PROPS_ROOT}/{spec.asset}_{index}"
        if spec.asset == BOX_ASSET:
            width, depth, height = BOX_SIZE_M
            FixedCuboid(
                prim_path=prim_path,
                name=prim_path.rsplit("/", 1)[-1],
                # A cuboid is centred on its origin, so half its height puts its
                # underside on the floor.
                position=np.array((spec.x, spec.y, height / 2.0)),
                scale=np.array([width, depth, height]),
            )
            print(f"{LOG}   prop: {prim_path} at ({spec.x}, {spec.y}) {width}x{depth}x{height} m")
            continue
        asset = PROP_ASSETS[spec.asset]
        add_reference_to_stage(usd_path=asset_url(asset.usd_path), prim_path=prim_path)
        simulation_app.update()  # compose it BEFORE the collider walk traverses it
        make_static_collider(prim_path)
        SingleXFormPrim(prim_path).set_world_pose(
            position=np.array((spec.x, spec.y, asset.z_offset)),
            orientation=np.array(yaw_to_quat_wxyz(spec.yaw)),
        )
        print(f"{LOG}   prop: {prim_path} at ({spec.x}, {spec.y}) yaw={spec.yaw}")


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Boot the patrol app's Isaac Sim world and keep it running.",
        epilog="Stop it with Ctrl-C (SIGINT) or SIGTERM.",
    )
    parser.add_argument(
        "--world",
        default="/workspace/sim/world.yaml",
        help="world description: robot spawn + props (default: %(default)s)",
    )
    parser.add_argument(
        "--policy",
        default="/robot_models/locomotion/policy.pt",
        help=(
            "TorchScript locomotion policy; its contract is read from "
            f"{POLICY_META_FILENAME} in the same directory (default: %(default)s)"
        ),
    )
    return parser.parse_args(argv)


def require_eula() -> None:
    """Refuse to boot without the operator's own EULA acceptance."""
    if not os.environ.get("ACCEPT_EULA"):
        raise ConfigError(
            "ACCEPT_EULA is not set — refusing to boot Isaac Sim. The NVIDIA Isaac Sim EULA "
            "has to be accepted by the person running this world, so the flag comes from "
            "your own environment (ACCEPT_EULA=Y, plus PRIVACY_CONSENT=Y for the container) "
            "and is never baked into a committed file or an image layer."
        )


def print_banner(world_path: str, policy_path: str) -> None:
    print(f"{LOG} ready — the world is running. No mission, no scoring, no recording.")
    print(f"{LOG}   world={world_path} policy={policy_path}")
    print(
        f"{LOG}   publishes: {TOPIC_CLOCK} (every step) {TOPIC_ODOM} + {TOPIC_TF} "
        f"({ODOM_RATE_HZ:g} Hz) {TOPIC_IMAGE} {TOPIC_DEPTH} {TOPIC_CAMERA_INFO} "
        f"({CAMERA_RATE_HZ:g} Hz) {TOPIC_SCAN} ({SCAN_RATE_HZ:g} Hz) {TOPIC_TF_STATIC} (latched)"
    )
    print(f"{LOG}   subscribes: {TOPIC_CMD_VEL}")
    print(
        f"{LOG}   drive it: ros2 topic pub -r 10 {TOPIC_CMD_VEL} geometry_msgs/msg/Twist "
        '"{linear: {x: 0.4}}"'
    )
    print(f"{LOG}   stop it: Ctrl-C")


def run(args: argparse.Namespace) -> None:
    """Boot the world, wire everything up, and step until a stop signal."""
    # Consent first, then the two input files: all three are decided before the
    # simulator exists, so a bad world file costs no boot.
    require_eula()
    world_spec = load_world_file(Path(args.world))
    meta = load_policy_meta(Path(args.policy))

    # ---- boot ------------------------------------------------------------- #
    simulation_app = SimulationApp(LAUNCH_CONFIG)

    # Only now are omni/isaacsim/numpy/torch legal.
    from isaacsim.core.api import World  # noqa: PLC0415
    from isaacsim.core.prims import SingleArticulation  # noqa: PLC0415
    from isaacsim.core.utils.extensions import enable_extension  # noqa: PLC0415

    # The ROS 2 bridge carries the bundled Jazzy runtime this process publishes
    # with; nothing ROS-shaped is importable before it is on.
    if not enable_extension("isaacsim.ros2.bridge"):
        raise RuntimeError(
            "could not enable the isaacsim.ros2.bridge extension — without it this process "
            "has no ROS 2 runtime at all"
        )
    simulation_app.update()

    policy = PolicyLoop(Path(args.policy), meta)
    policy.load()  # fails before the expensive stage load if the weights are wrong

    # ---- compose ---------------------------------------------------------- #
    open_warehouse(simulation_app)
    # The World is constructed AFTER the stage is open: it binds to the stage
    # that exists when it is created.
    world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT, stage_units_in_meters=1.0)
    compose_robot_and_extras(simulation_app)
    place_robot(world_spec.spawn_x, world_spec.spawn_y, world_spec.spawn_yaw)
    spawn_props(simulation_app, world_spec.props)

    rig = SensorRig()
    rig.author_prims()  # prims must exist before reset builds the render products

    world.reset()

    # ---- post-reset wiring ------------------------------------------------ #
    articulation = SingleArticulation(ROBOT_PRIM)
    articulation.initialize()
    policy.bind(articulation)
    # The callback signature carries the step size; the loop counts STEPS (the
    # physics dt is fixed and belongs to the trained plant), so it is dropped here.
    world.add_physics_callback(POLICY_CALLBACK_NAME, lambda _step_size: policy.on_physics_step())
    rig.initialize()

    # ---- ROS -------------------------------------------------------------- #
    import rclpy  # noqa: PLC0415 (bundled with the bridge extension)

    rclpy.init()
    # ONE node for the whole process: a second one would be a second participant
    # on the app's DDS domain for no gain.
    node = rclpy.create_node(NODE_NAME)
    types = import_ros_types()
    rig.attach(node, types)
    cmd_vel = CmdVelBridge(node, types, policy.set_command)

    stop = {"requested": False}

    def request_stop(_signum, _frame) -> None:
        stop["requested"] = True
        print(f"{LOG} stop requested — leaving the world")

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print_banner(args.world, args.policy)

    # ---- the loop --------------------------------------------------------- #
    steps = 0
    try:
        while not stop["requested"]:
            # render=True is not optional here: the camera and the RTX lidar are
            # rendered sensors, and with rendering off they produce nothing. It
            # is also the reason this world runs at about 0.75x real time
            # (measured, camera on) — anything in the app that times out should
            # be counting /clock, not the wall clock.
            world.step(render=True)
            sim_time = float(world.current_time)
            rig.publish(sim_time)
            cmd_vel.poll(sim_time)
            # One spin_once executes at most one callback, so this is a bounded
            # drain rather than a wait — timeout_sec=0.0 never blocks the sim.
            for _ in range(SPIN_PER_STEP):
                rclpy.spin_once(node, timeout_sec=0.0)
            steps += 1
    finally:
        print(f"{LOG} stepped {steps} time(s); shutting down")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        run(args)
    except ConfigError as exc:
        print(f"{LOG} {exc}", file=sys.stderr)
        return EXIT_CONFIG
    except Exception:  # noqa: BLE001 — a developer tool: print the trace, then exit
        traceback.print_exc()
        return EXIT_ERROR
    return EXIT_OK


if __name__ == "__main__":
    code = main()
    # os._exit, never sys.exit and never simulation_app.close(): closing the app
    # TERMINATES the process itself with status 0, and interpreter finalisation
    # (atexit hooks, __del__, non-daemon Kit threads) can hang or exit on its own
    # afterwards. os._exit is the only delivery no vendor teardown can preempt —
    # which is also why the Python streams are flushed by hand first.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
