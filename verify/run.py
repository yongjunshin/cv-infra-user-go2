#!/usr/bin/env python3
"""Run one Go2 patrol case through the user's real ROS 2 application.

cv-infra only supplies CASE, OUT and SEED. Everything below is deliberately consumer
owned: world construction, compose lifecycle, the /patrol action call and evidence.
"""

from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ISAAC_IMAGE = (
    "nvcr.io/nvidia/isaac-sim:5.1.0@sha256:"
    "f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54"
)
PROJECT = "go2-verify"
ROS_DOMAIN_ID = "77"


def interrupt_handler(_signum, _frame):
    raise KeyboardInterrupt


def run(
    command: list[str], *, check: bool = True, timeout: float | None = None, env: dict | None = None
) -> str:
    proc = subprocess.run(command, text=True, capture_output=True, timeout=timeout, env=env)
    text = proc.stdout + proc.stderr
    if check and proc.returncode:
        raise RuntimeError(f"command failed rc={proc.returncode}: {' '.join(command)}\n{text}")
    return text


def load_case() -> dict:
    with open(os.environ["CASE"], encoding="utf-8") as handle:
        return json.load(handle)


def world_for(case: dict, path: Path) -> None:
    inputs = case["inputs"]
    target = inputs["target"]
    props = [
        {"asset": target, "x": -6.0, "y": 5.2, "yaw": -1.5708 if target == "chair" else 3.1416}
    ]
    for index in range(int(inputs["box_count"])):
        props.append({"asset": "box", "x": -5.0 + index * 0.8, "y": 1.5, "yaw": 0.0})
    for index in range(int(inputs["desk_count"])):
        props.append({"asset": "desk", "x": -4.0, "y": 3.0 + index * 3.0, "yaw": 0.0})
    lines = [
        "spawn:",
        f"  x: {float(inputs['start_x'])}",
        f"  y: {float(inputs['start_y'])}",
        f"  yaw: {float(inputs['start_yaw'])}",
        "props:",
    ]
    for prop in props:
        lines.extend(
            [
                f"  - asset: {prop['asset']}",
                f"    x: {prop['x']}",
                f"    y: {prop['y']}",
                f"    yaw: {prop['yaw']}",
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def compose_for(checkout: Path, world: Path, compose: Path, *, runtime_checkout: Path | None = None) -> None:
    runtime_checkout = checkout if runtime_checkout is None else runtime_checkout
    robot = runtime_checkout / "robot_sw"
    compose.write_text(
        f"""services:
  sim:
    image: {ISAAC_IMAGE}
    network_mode: host
    ipc: host
    user: root
    shm_size: 8gb
    environment:
      ROS_DOMAIN_ID: {ROS_DOMAIN_ID}
      ACCEPT_EULA: ${{ACCEPT_EULA}}
      PRIVACY_CONSENT: ${{PRIVACY_CONSENT}}
      NVIDIA_DRIVER_CAPABILITIES: all
      PYTHONUNBUFFERED: '1'
    deploy:
      resources:
        reservations:
          devices: [{{driver: nvidia, count: all, capabilities: [gpu]}}]
    volumes:
      - {checkout / 'sim'}:/workspace/sim:ro
      - {world}:/workspace/sim/world.yaml:ro
      - {checkout / 'robot_sw/models'}:/robot_models:ro
    entrypoint: []
    command: >
      bash -c "source setup_ros_env.sh && ./python.sh /workspace/sim/patrol_world.py
      --world /workspace/sim/world.yaml --policy /robot_models/locomotion/policy.pt"
  robot:
    build:
      context: {robot}
    network_mode: host
    ipc: host
    environment:
      ROS_DOMAIN_ID: {ROS_DOMAIN_ID}
""",
        encoding="utf-8",
    )


def main() -> int:
    checkout = Path.cwd()
    host_checkout = Path(os.environ["CV_CHECKOUT_HOST"])
    host_out = Path(os.environ["CV_OUT_HOST"])
    out = Path(os.environ["OUT"])
    case = load_case()
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="go2-case-") as temp:
        temp_path = Path(temp)
        runtime_world = out / "world.yaml"
        host_world = host_out / "world.yaml"
        compose = temp_path / "compose.yaml"
        world_for(case, runtime_world)
        compose_for(host_checkout, host_world, compose, runtime_checkout=checkout)
        suffix = re.sub(r"[^a-z0-9_-]", "", case["case_id"].lower())[-12:] or "case"
        env = {**os.environ, "COMPOSE_PROJECT_NAME": f"{PROJECT}-{suffix}"}
        up_output = run(
            ["docker", "compose", "-f", str(compose), "up", "-d", "--build"],
            timeout=900,
            env=env,
        )
        (out / "compose-up.log").write_text(up_output, encoding="utf-8")
        started = time.monotonic()
        signal.signal(signal.SIGTERM, interrupt_handler)
        signal.signal(signal.SIGINT, interrupt_handler)
        try:
            deadline = started + 60
            while time.monotonic() < deadline:
                probe = subprocess.run(
                    [
                        "docker",
                        "compose",
                        "-f",
                        str(compose),
                        "exec",
                        "-T",
                        "robot",
                        "bash",
                        "-lc",
                        "source /opt/ros/jazzy/setup.bash && "
                        "source /opt/go2_ws/install/setup.bash && ros2 action list",
                    ],
                    text=True,
                    capture_output=True,
                    env=env,
                )
                if "/patrol" in probe.stdout:
                    break
                time.sleep(2)
            else:
                raise RuntimeError("/patrol action did not become available")
            target = case["inputs"]["target"]
            mission = run(
                [
                    "docker",
                    "compose",
                    "-f",
                    str(compose),
                    "exec",
                    "-T",
                    "robot",
                    "bash",
                    "-lc",
                    "source /opt/ros/jazzy/setup.bash && "
                    "source /opt/go2_ws/install/setup.bash && "
                    f"ros2 action send_goal /patrol go2_msgs/action/Patrol "
                    f"'{{target_class: {target}}}' --feedback",
                ],
                timeout=240, env=env,
            )
            (out / "mission.txt").write_text(mission, encoding="utf-8")
            (out / "case.json").write_text(json.dumps(case, indent=2) + "\n", encoding="utf-8")
            (out / "run.json").write_text(json.dumps({"wall_s": time.monotonic() - started}) + "\n", encoding="utf-8")
        finally:
            logs = subprocess.run(
                ["docker", "compose", "-f", str(compose), "logs", "--no-color"],
                text=True,
                capture_output=True,
                env=env,
            )
            (out / "compose.log").write_text(
                logs.stdout + logs.stderr, encoding="utf-8"
            )
            cleanup = subprocess.run(["docker", "compose", "-f", str(compose), "down", "--volumes"], text=True, capture_output=True, env=env)
            (out / "compose-cleanup.log").write_text(cleanup.stdout + cleanup.stderr, encoding="utf-8")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"go2 verify run failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
