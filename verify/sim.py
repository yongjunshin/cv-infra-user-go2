#!/isaac-sim/python.sh
"""verify/sim.py — one verification case: does this app find what it is asked to find,
and only that?

A STANDARD Isaac Sim standalone script, and ONLY a test harness. cv-infra never imports
this file and knows nothing about what it means; per PICT case it runs, inside the image
built from verify/Dockerfile (Isaac Sim 5.1.0 + this repository's robot_sw/ baked in),

    verify/sim.py --start=S2 --target=chair --hide=H3 --decoy=present --slot_a=desk --slot_b=empty

through a `/bin/sh -lc 'exec "$0" "$@"'` wrapper, with this repository checked out
read-only at the working directory, `verify/out/` overlaid read-write, and `CV_SEED` in
the environment.

WHAT ONE CASE IS: the app's own world (sim/patrol_world.py, imported as it is — the
warehouse, the Go2, its locomotion policy and its sensor rig) with the case's objects in
it: the TARGET of the requested class hidden at one of four spots or not placed at all,
optionally a DECOY of the other class, and a desk (or nothing) in each of two slots. Then
the app (robot_sw/, the image's own build, launched exactly as its default command does)
is told where it stands — the way an operator sets "2D Pose Estimate" — and is sent ONE
`/patrol` goal naming a class. From there the harness only WATCHES: it records where the
robot really walked and when the mission answered. It never says where the target is.

`verify/oracle.py` judges the evidence: a present target has to be found, at the right
place, within a time allowance that grows with its distance; an absent one must not be
reported, however long the robot searches.

PICTURES, ALSO THE HARNESS'S JOB: a camera 60 m above the warehouse (roof hidden)
renders the case's set-up -> `verify/out/topview_initial.png`; at the end the harness
draws the start, the true target, the decoy, the desks, the walked path and the reported
target position on it -> `verify/out/topview_result.png`. The oracle cannot draw: the
platform mounts `verify/out/` read-only for it.

EXIT CODE IS NOT A VERDICT (platform contract): 0 = the evidence was written, whatever
the robot did; 1 = the case ERRORed (the world or the app never came up); 2 = bad argv.
The process ends with `os._exit`, like sim/patrol_world.py, because
`SimulationApp.close()` ends it with status 0 no matter what.

ORDERING (hard rule): nothing Isaac-shaped is imported before `SimulationApp(...)`
exists — hence stdlib-only up here. And the bundled ROS 2 runtime of the simulator only
loads when its library path is in the environment AT PROCESS START (that is all
/isaac-sim/setup_ros_env.sh does), so the script re-executes itself once with those
three variables set before anything else happens.

TWO ROS 2 ENVIRONMENTS, NEVER MIXED: this process talks DDS through the simulator's
bundled Jazzy; the app runs on the image's /opt/ros/jazzy with its own Python. The app is
therefore started from a WHITELISTED environment (APP_ENV_KEEP), not this one — this one
carries Kit's LD_PRELOAD and PYTHONPATH, which would break it.
"""

# stdlib only up here — see ORDERING above.
import argparse
import csv
import importlib.util
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

# --------------------------------------------------------------------------------------
# The layout. Map frame (= world frame), metres. The app's own map (robot_sw/.../maps/)
# shows >= 3.3 m of clearance at every hiding spot, the decoy spot and both desk slots,
# and >= 1.27 m at every start (COMPUTED from the map, 2026-10-01). The building is the
# same warehouse the carter example uses, so the top-view camera below is the same rig.
# --------------------------------------------------------------------------------------
START_YAW_RAD = math.pi / 2  # every start faces up the floor
STARTS = {
    "S1": (-6.0, -1.0),  # the app's own default spawn (sim/world.yaml)
    # Not (0, 0): the world's sensor rig leaves a small render-only shape at the origin
    # (no collider; seen in every render of this world, 2026-10-01) — not under a robot.
    "S2": (0.0, 1.5),
    "S3": (6.0, 0.0),
}
# Where the requested target may hide. `none` = it is not in the warehouse at all.
HIDES = {
    "H1": (-3.0, 4.0),
    "H2": (4.0, 6.0),
    "H3": (-5.0, 10.0),
    "H4": (3.0, 13.0),
}
NONE = "none"
# The decoy is the OTHER class, always at the same spot: asked for a chair, a person
# may be standing there, and the right answer is still "no chair".
DECOY_AT = (0.0, 8.5)
OTHER_CLASS = {"chair": "person", "person": "chair"}
# A chair only reads as `chair` from the front (sim/world.yaml), so it faces the start
# row; a person reads from every side measured.
FACING = {"chair": -math.pi / 2, "person": math.pi}
# Two obstacle slots, each a desk (0.80 x 2.80 x 0.75 m, laid across the floor) or nothing.
SLOTS = {
    "a": (-6.0, 3.0),  # across S1's way up the floor
    "b": (2.0, 3.0),  # between S2 and S3's way up
}
DESK = "desk"
DESK_YAW_RAD = math.pi / 2
DESK_SIZE_M = (2.80, 0.80)  # footprint at DESK_YAW_RAD, x by y

# --------------------------------------------------------------------------------------
# The episode — the test's rules, not the robot's.
# --------------------------------------------------------------------------------------
WARMUP_S = 2.0  # the robot settles into its stance before anything is measured
APP_READY_TIMEOUT_S = 240.0  # wall seconds: nav2 + YOLO + manager, no downloads
POSE_SET_TIMEOUT_S = 90.0  # wall seconds for AMCL to take the operator's pose
ACCEPT_TIMEOUT_S = 60.0  # wall seconds for the goal to be accepted
# The longest the harness watches a mission, in sim seconds from acceptance. The app
# gives up searching after its own budget; this only has to outlast that plus an
# approach that started late. A mission still running here is recorded as such.
WATCH_MAX_S = 240.0
TEARDOWN_WAIT_S = 30.0
TRAJ_PERIOD_S = 0.1
POLL_PERIOD_S = 0.5  # sim seconds between looks at the app's log / the mission output
INITIAL_POSE_PERIOD_S = 1.0
INITIAL_POSE_COV = (0.05, 0.05, 0.02)  # x, y, yaw variances: "I know where it is"

ROS_DOMAIN_ID = "77"  # stable, not unique: nothing else shares this container
APP_CMD = (
    "source /opt/ros/jazzy/setup.bash && source /opt/go2_ws/install/setup.bash && "
    "exec ros2 launch go2_bringup go2_patrol.launch.py use_sim_time:=true"
)
GOAL_CMD = (
    "source /opt/ros/jazzy/setup.bash && source /opt/go2_ws/install/setup.bash && "
    'exec ros2 action send_goal /patrol go2_msgs/action/Patrol "{target_class: $0}" --feedback'
)
# The app gets THESE from this process's environment and nothing else (see the module
# docstring). The image's own ENV (verify/Dockerfile) is in here on purpose.
APP_ENV_KEEP = (
    "HOME", "LANG", "LC_ALL", "TERM", "NVIDIA_VISIBLE_DEVICES", "NVIDIA_DRIVER_CAPABILITIES",
    "YOLO_CONFIG_DIR", "YOLO_AUTOINSTALL", "CV_SEED",
)  # fmt: skip
APP_PATH = "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# What the app says, read off its own log / the CLI's output.
APP_UP = "go2_patrol_manager up: serving /patrol"
AMCL_POSE_TAKEN = "initialPoseReceived"  # nav2 AMCL, once at activation (its own prior) + once per /initialpose
GOAL_ACCEPTED = "Goal accepted"
# nav2's two lifecycle managers each print this once their nodes are up; the goal waits for
# BOTH. MEASURED (CI run 36816876742): a goal sent while navigation was still activating
# was aborted 16 ms after acceptance as "no perception after 5 s" — the manager's sim clock
# had not settled. An operator sends a mission to an app that is up, so the harness does.
NAV_ACTIVE = "Managed nodes are active"
NAV_MANAGERS = 2
SETTLE_S = 2.0  # sim seconds after the app is fully up, before the goal

# The top-view camera — the same rig as the carter example (same building), MEASURED
# again in this world 2026-10-01: markers at (+-7, 1), (+-7, 11), (0, 6) land on the
# pixels to_pixel() predicts.
TOPCAM_PATH = "/World/cv_topcam"
TOPCAM_SIZE_PX = (1200, 1800)
TOPCAM_CENTRE = (-0.45, 2.9)
TOPCAM_HEIGHT_M = 60.0
TOPCAM_WIDTH_M = 22.0
TOPCAM_APERTURE_MM = 20.955
ROOF_WORDS = ("ceiling", "roof", "lamp", "beam")

# Checkout-relative: the platform mounts the case's output directory over this path.
OUT_DIR = os.path.join("verify", "out")
OUT_TRAJECTORY = os.path.join(OUT_DIR, "trajectory.csv")
OUT_RUN = os.path.join(OUT_DIR, "run.json")
OUT_APP_LOG = os.path.join(OUT_DIR, "app.log")
OUT_MISSION = os.path.join(OUT_DIR, "mission.txt")
OUT_TOPVIEW = os.path.join(OUT_DIR, "topview_initial.png")
OUT_RESULT = os.path.join(OUT_DIR, "topview_result.png")

WORLD_PY = os.path.join("sim", "patrol_world.py")
POLICY = os.path.join("robot_sw", "models", "locomotion", "policy.pt")

EXIT_ERROR = 1
EXIT_USAGE = 2
EXIT_NO_CONSENT = 3


def log(msg: str) -> None:
    print(f"[cv-go2] {msg}", flush=True)


def ensure_bundled_ros_env() -> None:
    """Re-exec once with the simulator's bundled ROS 2 on the library path.

    Exactly what /isaac-sim/setup_ros_env.sh exports (ROS_DISTRO, the bridge's lib dir
    on LD_LIBRARY_PATH, RMW_IMPLEMENTATION) — but the dynamic loader only reads
    LD_LIBRARY_PATH at process start, so setting it from inside is not enough.
    """
    if os.environ.get("ROS_DISTRO") and os.environ.get("ROS_DOMAIN_ID") == ROS_DOMAIN_ID:
        return
    lib = os.path.join(os.environ.get("ISAAC_PATH", "/isaac-sim"), "exts", "isaacsim.ros2.bridge", "jazzy", "lib")
    env = dict(os.environ)
    env["ROS_DISTRO"] = "jazzy"
    env["LD_LIBRARY_PATH"] = ":".join(p for p in (env.get("LD_LIBRARY_PATH"), lib) if p)
    env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    env["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID  # this process and the app: one domain (see app_env)
    os.execve(sys.executable, [sys.executable, os.path.abspath(__file__), *sys.argv[1:]], env)


def parse_args() -> argparse.Namespace:
    """The axes of verify/space.pict, one flag each — strict on purpose."""
    p = argparse.ArgumentParser(description="go2 search case for cv-infra")
    p.add_argument("--start", required=True, choices=sorted(STARTS))
    p.add_argument("--target", required=True, choices=sorted(OTHER_CLASS))
    p.add_argument("--hide", required=True, choices=[*sorted(HIDES), NONE])
    p.add_argument("--decoy", required=True, choices=[NONE, "present"])
    for slot in SLOTS:
        p.add_argument(f"--slot_{slot}", required=True, choices=["empty", DESK])
    try:
        return p.parse_args()
    except SystemExit:
        sys.exit(EXIT_USAGE)


def consent_guard() -> None:
    if not os.environ.get("ACCEPT_EULA"):
        print("ERROR: ACCEPT_EULA is empty — the operator has not accepted the Isaac Sim EULA. Boot refused.",
              file=sys.stderr, flush=True)  # fmt: skip
        sys.exit(EXIT_NO_CONSENT)


# --------------------------------------------------------------------------------------
# The case, as plain data. Nothing here touches Isaac.
# --------------------------------------------------------------------------------------


def case_objects(args: argparse.Namespace) -> list[dict]:
    """Every object this case adds to the world: target, decoy, desks — in spawn order."""
    objects = []
    if args.hide != NONE:
        x, y = HIDES[args.hide]
        objects.append({"role": "target", "asset": args.target, "x": x, "y": y, "yaw": FACING[args.target]})
    if args.decoy != NONE:
        other = OTHER_CLASS[args.target]
        objects.append({"role": "decoy", "asset": other, "x": DECOY_AT[0], "y": DECOY_AT[1], "yaw": FACING[other]})
    for slot, (x, y) in SLOTS.items():
        if getattr(args, f"slot_{slot}") == DESK:
            objects.append({"role": f"slot_{slot}", "asset": DESK, "x": x, "y": y, "yaw": DESK_YAW_RAD})
    return objects


def load_world_module():
    """sim/patrol_world.py, imported as it is (registered first: it defines dataclasses)."""
    spec = importlib.util.spec_from_file_location("patrol_world", WORLD_PY)
    module = importlib.util.module_from_spec(spec)
    sys.modules["patrol_world"] = module
    spec.loader.exec_module(module)
    return module


def app_env() -> dict:
    env = {k: os.environ[k] for k in APP_ENV_KEEP if k in os.environ}
    env.update(PATH=APP_PATH, ROS_DOMAIN_ID=ROS_DOMAIN_ID, ROS_HOME="/tmp/.ros", PYTHONUNBUFFERED="1")
    return env


def start_process(cmd: str, out_path: str, *argv) -> subprocess.Popen:
    """bash -c CMD in its OWN session (teardown signals the whole tree), output to a file."""
    handle = open(out_path, "w")  # noqa: SIM115 — closed with the process, see stop_process
    proc = subprocess.Popen(["bash", "-c", cmd, *argv], stdout=handle, stderr=subprocess.STDOUT,
                            env=app_env(), start_new_session=True)  # fmt: skip
    proc.out_handle = handle
    return proc


def stop_process(proc, sig) -> None:
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, sig)
            proc.wait(timeout=TEARDOWN_WAIT_S)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()
    proc.out_handle.close()


def read_text(path: str) -> str:
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            return handle.read()
    except OSError:
        return ""


def parse_result(mission: str) -> dict:
    """The /patrol result as the CLI printed it: found, reported x/y, message, status."""
    found = re.search(r"^\s*found:\s*(true|false)", mission, re.MULTILINE | re.IGNORECASE)
    status = re.search(r"Goal finished with status:\s*(\w+)", mission)
    message = re.search(r"^\s*message:\s*'?(.*?)'?\s*$", mission, re.MULTILINE)
    result = mission.split("Result:", 1)[1] if "Result:" in mission else ""
    pos = re.search(r"position:\s*x:\s*(-?[\d.e+-]+)\s*y:\s*(-?[\d.e+-]+)", result)
    return {
        "found": None if not found else found.group(1).lower() == "true",
        "status": status.group(1) if status else None,
        "message": message.group(1) if message else None,
        # Meaningful only when found (the action says so): otherwise it is a zero pose.
        "reported_xy": [float(pos.group(1)), float(pos.group(2))] if pos and found and found.group(1).lower() == "true" else None,
    }


# --------------------------------------------------------------------------------------
# Pictures: the top-view camera model and the overlay.
# --------------------------------------------------------------------------------------


def topcam_focal_mm() -> float:
    return TOPCAM_HEIGHT_M * TOPCAM_APERTURE_MM / TOPCAM_WIDTH_M


def to_pixel(x: float, y: float) -> tuple[float, float]:
    """World floor point -> top-view pixel (pinhole straight down, z = 0)."""
    width, height = TOPCAM_SIZE_PX
    px_per_m = topcam_focal_mm() / TOPCAM_APERTURE_MM * width / TOPCAM_HEIGHT_M
    return width / 2 + (x - TOPCAM_CENTRE[0]) * px_per_m, height / 2 - (y - TOPCAM_CENTRE[1]) * px_per_m


def capture_top_view(world, np) -> None:
    """Render one frame of the top-view camera to OUT_TOPVIEW, then let the camera go."""
    import omni.replicator.core as rep  # noqa: PLC0415
    from PIL import Image  # noqa: PLC0415

    product = rep.create.render_product(TOPCAM_PATH, TOPCAM_SIZE_PX)
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach([product])
    frame = None
    for attempt in range(40):
        world.step(render=True)
        frame = annotator.get_data()
        if attempt >= 8 and frame is not None and getattr(frame, "size", 0) and float(frame.mean()) > 1.0:
            break
    if frame is None or not getattr(frame, "size", 0):
        raise RuntimeError("top-view camera delivered no frame")
    Image.fromarray(np.asarray(frame)[:, :, :3].astype(np.uint8)).save(OUT_TOPVIEW)
    annotator.detach()
    product.destroy()
    log(f"top view captured -> {OUT_TOPVIEW}")


def draw_result(args, objects: list[dict], samples: list[tuple], outcome: dict) -> None:
    """Draw the case onto the initial top view — facts only; the verdict is the oracle's."""
    from PIL import Image, ImageDraw, ImageFont  # noqa: PLC0415

    image = Image.open(OUT_TOPVIEW).convert("RGB")
    draw = ImageDraw.Draw(image, "RGBA")
    width, _height = image.size
    m = to_pixel(1.0, 0.0)[0] - to_pixel(0.0, 0.0)[0]  # pixels per metre
    small, big = ImageFont.load_default(size=22), ImageFont.load_default(size=30)

    def label(x, y, dy_m, text, font, fill):
        u, v = to_pixel(x, y)
        draw.text((u, v + dy_m * m), text, font=font, anchor="mm", fill=fill, stroke_width=2, stroke_fill=(0, 0, 0, 255))

    def ring(x, y, r_m, colour, w):
        u, v = to_pixel(x, y)
        draw.ellipse((u - r_m * m, v - r_m * m, u + r_m * m, v + r_m * m), outline=colour, width=w)

    for name, (x, y) in HIDES.items():  # every hiding spot, faint
        ring(x, y, 0.5, (255, 255, 255, 80), 2)
        label(x, y, 0.95, name, small, (255, 255, 255, 170))
    for name, (x, y) in STARTS.items():
        label(x, y, 0.95, name, big, (255, 255, 255, 255 if name == args.start else 140))
    for ob in objects:
        if ob["asset"] == DESK:
            (u0, v0), (u1, v1) = (to_pixel(ob["x"] - DESK_SIZE_M[0] / 2, ob["y"] + DESK_SIZE_M[1] / 2),
                                  to_pixel(ob["x"] + DESK_SIZE_M[0] / 2, ob["y"] - DESK_SIZE_M[1] / 2))  # fmt: skip
            draw.rectangle((u0 - 2, v0 - 2, u1 + 2, v1 + 2), outline=(255, 214, 0, 255), width=3)
        elif ob["role"] == "target":
            ring(ob["x"], ob["y"], 0.9, (60, 220, 90, 255), 5)
            label(ob["x"], ob["y"], -1.4, f"TARGET {ob['asset']}", small, (120, 255, 140, 255))
        else:
            ring(ob["x"], ob["y"], 0.9, (255, 150, 40, 255), 4)
            label(ob["x"], ob["y"], -1.4, f"decoy {ob['asset']}", small, (255, 190, 110, 255))
    sx, sy = STARTS[args.start]
    ring(sx, sy, 0.5, (0, 230, 255, 255), 5)
    path = [to_pixel(r[1], r[2]) for r in samples]
    if len(path) > 1:
        draw.line(path, fill=(0, 230, 255, 255), width=4, joint="curve")
    reported = outcome.get("reported_xy")
    if reported:
        u, v = to_pixel(*reported)
        r = 0.4 * m
        draw.line((u - r, v - r, u + r, v + r), fill=(255, 60, 220, 255), width=6)
        draw.line((u - r, v + r, u + r, v - r), fill=(255, 60, 220, 255), width=6)
        label(reported[0], reported[1], 0.9, "reported", small, (255, 120, 230, 255))
    hidden = f"hidden at {args.hide}" if args.hide != NONE else "NOT in the warehouse"
    lines = [f"{args.start}: find {args.target} ({hidden}, decoy {args.decoy})"]
    if outcome.get("found"):
        lines.append(f"answer: FOUND after {outcome['mission_s']:.1f} s"
                     + (f", reported {outcome['report_error_m']:.2f} m from the target" if outcome.get("report_error_m") is not None else ""))  # fmt: skip
        colour = (60, 200, 90)
    else:
        watched = outcome.get("mission_s")
        lines.append(f"answer: not found ({outcome.get('end')}" + (f", after {watched:.1f} s)" if watched is not None else ")"))
        colour = (130, 140, 160)
    bar = 16 + 34 * len(lines)
    draw.rectangle((0, 0, width, bar), fill=(0, 0, 0, 190))
    draw.rectangle((0, 0, 18, bar), fill=colour + (255,))
    for i, text in enumerate(lines):
        draw.text((30, 25 + 34 * i), text, font=big if i == 0 else small, anchor="lm", fill=(255, 255, 255, 255))
    image.save(OUT_RESULT)


# --------------------------------------------------------------------------------------
# The case.
# --------------------------------------------------------------------------------------


def run(pw, simulation_app, args: argparse.Namespace) -> None:
    """Build the case's world, bring the app up, send one /patrol goal, watch, record."""
    import numpy as np  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415
    from isaacsim.core.api import World  # noqa: PLC0415
    from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: PLC0415
    from isaacsim.core.utils.extensions import enable_extension  # noqa: PLC0415
    from pxr import Gf, UsdGeom  # noqa: PLC0415

    objects = case_objects(args)
    start_x, start_y = STARTS[args.start]
    log(f"{args.start} ({start_x}, {start_y}); find {args.target}; hide={args.hide} decoy={args.decoy}")
    for ob in objects:
        log(f"  {ob['role']}: {ob['asset']} at ({ob['x']}, {ob['y']})")

    # ---- the app's own world, built the way sim/patrol_world.py builds it -------------
    if not enable_extension("isaacsim.ros2.bridge"):
        raise RuntimeError("could not enable isaacsim.ros2.bridge")
    simulation_app.update()
    policy = pw.PolicyLoop(pw.Path(POLICY), pw.load_policy_meta(pw.Path(POLICY)))
    policy.load()
    pw.open_warehouse(simulation_app)
    world = World(physics_dt=pw.PHYSICS_DT, rendering_dt=pw.RENDERING_DT, stage_units_in_meters=1.0)
    pw.compose_robot_and_extras(simulation_app)
    pw.place_robot(start_x, start_y, START_YAW_RAD)
    pw.spawn_props(simulation_app, [pw.PropSpec(ob["asset"], ob["x"], ob["y"], ob["yaw"]) for ob in objects])
    rig = pw.SensorRig()
    rig.author_prims()

    # ---- the test rig: the top-view camera, the roof out of its way (rendering only) --
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if any(word in prim.GetName().lower() for word in ROOF_WORDS) and prim.IsA(UsdGeom.Imageable):
            UsdGeom.Imageable(prim).MakeInvisible()
    camera = UsdGeom.Camera.Define(stage, TOPCAM_PATH)
    UsdGeom.Xformable(camera).AddTranslateOp().Set(Gf.Vec3d(TOPCAM_CENTRE[0], TOPCAM_CENTRE[1], TOPCAM_HEIGHT_M))
    camera.CreateFocalLengthAttr(topcam_focal_mm())
    camera.CreateHorizontalApertureAttr(TOPCAM_APERTURE_MM)
    camera.CreateVerticalApertureAttr(TOPCAM_APERTURE_MM * TOPCAM_SIZE_PX[1] / TOPCAM_SIZE_PX[0])
    camera.CreateClippingRangeAttr(Gf.Vec2f(1.0, 200.0))

    world.reset()
    articulation = SingleArticulation(pw.ROBOT_PRIM)
    articulation.initialize()
    policy.bind(articulation)
    world.add_physics_callback(pw.POLICY_CALLBACK_NAME, lambda _dt: policy.on_physics_step())
    rig.initialize()

    import rclpy  # noqa: PLC0415 (bundled with the bridge extension)
    from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: PLC0415

    rclpy.init()
    node = rclpy.create_node("cv_go2_case")
    types = pw.import_ros_types()
    rig.attach(node, types)
    cmd_vel = pw.CmdVelBridge(node, types, policy.set_command)
    initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", types.qos())
    chassis = SingleXFormPrim(pw.CHASSIS_PRIM)

    def step() -> float:
        world.step(render=True)
        now = float(world.current_time)
        rig.publish(now)
        cmd_vel.poll(now)
        for _ in range(pw.SPIN_PER_STEP):
            rclpy.spin_once(node, timeout_sec=0.0)
        return now

    def pose():
        position, orientation = chassis.get_world_pose()
        w, x, y, z = (float(v) for v in orientation)
        return float(position[0]), float(position[1]), math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    os.makedirs(OUT_DIR, exist_ok=True)
    t = step()
    while t < WARMUP_S:
        t = step()
    try:  # a picture is evidence, not the verdict: a render hiccup must not ERROR the case
        capture_top_view(world, np)
    except Exception as exc:
        log(f"WARN top view not captured: {exc!r}")

    record = {"axes": vars(args), "objects": objects, "start": {"name": args.start, "x": start_x, "y": start_y,
              "yaw": START_YAW_RAD}, "seed": os.environ.get("CV_SEED"), "t": {}}  # fmt: skip
    samples: list[tuple] = []
    app = goal = None
    end = "error"
    try:
        # ---- the app, exactly its image's default launch, in its own environment -------
        app = start_process(APP_CMD, OUT_APP_LOG)
        log(f"app started (pid {app.pid}) -> {OUT_APP_LOG}")
        # Up = serving /patrol AND AMCL active (it logs taking its own prior on activation,
        # and a pose sent before that would be overwritten by it).
        wall0, next_poll = time.monotonic(), t
        while True:
            if t >= next_poll:
                next_poll = t + POLL_PERIOD_S
                text = read_text(OUT_APP_LOG)
                if APP_UP in text and AMCL_POSE_TAKEN in text and text.count(NAV_ACTIVE) >= NAV_MANAGERS:
                    break
            if app.poll() is not None:
                raise RuntimeError(f"the app exited (rc {app.returncode}) before serving /patrol — see {OUT_APP_LOG}")
            if time.monotonic() - wall0 > APP_READY_TIMEOUT_S:
                raise RuntimeError(f"the app did not come up within {APP_READY_TIMEOUT_S:.0f} s wall")
            t = step()
        record["t"]["app_ready"] = round(t, 3)
        log(f"app up at sim {t:.1f} s ({time.monotonic() - wall0:.0f} s wall)")

        # ---- tell it where it stands: the operator's "2D Pose Estimate" -----------------
        taken0 = read_text(OUT_APP_LOG).count(AMCL_POSE_TAKEN)
        wall0, next_pub = time.monotonic(), t
        while True:
            if t >= next_pub and read_text(OUT_APP_LOG).count(AMCL_POSE_TAKEN) > taken0:
                break
            if time.monotonic() - wall0 > POSE_SET_TIMEOUT_S:
                raise RuntimeError("AMCL never took the initial pose")
            if t >= next_pub:
                x, y, yaw = pose()
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "map"
                msg.header.stamp.sec, msg.header.stamp.nanosec = pw.sim_time_stamp(t)
                msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35] = INITIAL_POSE_COV
                initial_pose_pub.publish(msg)
                record["initial_pose"] = {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)}
                next_pub = t + INITIAL_POSE_PERIOD_S
            t = step()
        log(f"initial pose taken at sim {t:.1f} s: {record['initial_pose']}")
        settle_until = t + SETTLE_S
        while t < settle_until:
            t = step()

        # ---- the mission: ONE goal, naming a class, never a place -----------------------
        goal = start_process(GOAL_CMD, OUT_MISSION, args.target)
        record["t"]["goal_sent"] = round(t, 3)
        wall0, accepted_t, next_poll, next_sample = time.monotonic(), None, t, t
        end = "watch_max"
        while True:
            if t >= next_sample:
                x, y, yaw = pose()
                samples.append((round(t, 3), round(x, 4), round(y, 4), round(yaw, 4)))
                next_sample = t + TRAJ_PERIOD_S
            # Acceptance is checked BEFORE the exit: a mission answered within one poll
            # period must still get its acceptance time (MEASURED: it did not, once).
            finished = goal.poll() is not None
            if accepted_t is None and (finished or t >= next_poll):
                next_poll = t + POLL_PERIOD_S
                if GOAL_ACCEPTED in read_text(OUT_MISSION):
                    accepted_t = t
                    log(f"goal accepted at sim {t:.1f} s")
                elif not finished and time.monotonic() - wall0 > ACCEPT_TIMEOUT_S:
                    end = "not_accepted"
                    break
            if finished:
                end = "answered"
                break
            if accepted_t is not None and t - accepted_t >= WATCH_MAX_S:
                break
            if app.poll() is not None:
                end = "app_died"
                break
            t = step()
        record["t"]["accepted"] = None if accepted_t is None else round(accepted_t, 3)
        record["t"]["end"] = round(t, 3)
    finally:
        stop_process(goal, signal.SIGINT)  # a client stopped mid-mission cancels its goal
        stop_process(app, signal.SIGINT)  # ros2 launch forwards SIGINT to every node
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ---- the evidence ----------------------------------------------------------------
    outcome = parse_result(read_text(OUT_MISSION))
    accepted = record["t"].get("accepted")
    outcome["end"] = end
    outcome["mission_s"] = None if accepted is None else round(record["t"]["end"] - accepted, 3)
    target = next((ob for ob in objects if ob["role"] == "target"), None)
    outcome["report_error_m"] = (
        round(math.hypot(outcome["reported_xy"][0] - target["x"], outcome["reported_xy"][1] - target["y"]), 4)
        if outcome.get("reported_xy") and target else None
    )
    record["outcome"] = outcome
    record["pictures"] = {"initial": OUT_TOPVIEW, "result": OUT_RESULT}
    with open(OUT_TRAJECTORY, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("t", "x", "y", "yaw"))
        writer.writerows(samples)
    with open(OUT_RUN, "w") as handle:
        json.dump(record, handle, indent=1, sort_keys=True)
        handle.write("\n")
    if os.path.exists(OUT_TOPVIEW):
        try:
            draw_result(args, objects, samples, outcome)
        except Exception as exc:
            log(f"WARN result picture not drawn: {exc!r}")
    log(f"wrote {OUT_DIR}/: end={end}, found={outcome['found']}, mission {outcome['mission_s']} sim-s, "
        f"{len(samples)} samples")  # fmt: skip


def main() -> int:
    ensure_bundled_ros_env()
    args = parse_args()
    consent_guard()
    log(f"CV_SEED={os.environ.get('CV_SEED')} (recorded; the app's search walk seeds itself)")
    pw = load_world_module()  # also imports SimulationApp — and nothing else Isaac-shaped
    simulation_app = pw.SimulationApp(pw.LAUNCH_CONFIG)
    try:
        run(pw, simulation_app, args)
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR case failed: {exc!r}", file=sys.stderr, flush=True)
        return EXIT_ERROR
    return 0


if __name__ == "__main__":
    code = main()
    # os._exit, as in sim/patrol_world.py: close() would end the process with status 0
    # and interpreter finalisation can hang on Kit's threads.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
