#!/usr/bin/env python3
"""verify/oracle.py — turn one case's evidence into a verdict.

cv-infra runs this right after `verify/sim`, in the SAME image, with the SAME argv, no
GPU, and both the checkout and verify/out/ mounted read-only. The last line of stdout
that parses as a flat JSON object is the case's verdict, and the platform reads TYPES,
not names:

    bool   a check   — the case passes when every bool is true
    number a metric  — compared against the baseline across commits, never gates
    null   unknown   — not a failure, just excluded from the ratio
    str    a note

Each bool below has exactly ONE evidence file, so a false answer names the thing that
did not happen:

    sim_booted       verify/out/sim.log      the world printed its ready banner
    app_ready        verify/out/app.log      go2_patrol_manager started serving /patrol
    action_succeeded verify/out/mission.txt  the /patrol goal finished SUCCEEDED
    target_found     verify/out/mission.txt  the result says the target was found

A MISSING FILE IS A false, NOT A CRASH. `verify/sim` exits 0 whenever the goal was sent,
so a case that reaches this script with no mission.txt is a case the sim never got to —
which is exactly a failing check, and a traceback would only turn it into an ERROR
(no verdict at all). Hence: never raise, always print one line, exit 0.

stdlib only, so this also runs on a plain laptop python3.
"""

import argparse
import json
import os
import re
import sys

OUT = os.path.join("verify", "out")
SIM_LOG = os.path.join(OUT, "sim.log")
APP_LOG = os.path.join(OUT, "app.log")
MISSION_TXT = os.path.join(OUT, "mission.txt")
RUN_JSON = os.path.join(OUT, "run.json")

# The world's own ready banner (sim/patrol_world.py, print_banner): "[patrol-world] ready
# — the world is running." Matched on the ASCII half only, so the em dash cannot turn a
# green case red over an encoding difference.
SIM_READY = "the world is running"
# go2_patrol_manager's constructor line: "go2_patrol_manager up: serving /patrol, ...".
APP_READY = re.compile(r"go2_patrol_manager up: serving\s+/patrol")
# `ros2 action send_goal --feedback` prints this verbatim when the server accepted the
# goal and the goal terminated successfully.
GOAL_SUCCEEDED = "Goal finished with status: SUCCEEDED"
# The Patrol result's own `found` field, as the CLI renders it.
TARGET_FOUND = re.compile(r"found:\s*true", re.IGNORECASE)


def parse_args() -> argparse.Namespace:
    """The same six axes as verify/sim — the platform replays the whole argv.

    Only `--target` is read (it goes in the note); the rest exist so that an axis is
    never silently dropped on one side of the pair.
    """
    p = argparse.ArgumentParser(description="verdict for one go2 patrol case")
    p.add_argument("--target", required=True)
    p.add_argument("--start_x", required=True)
    p.add_argument("--start_y", required=True)
    p.add_argument("--start_yaw", required=True)
    p.add_argument("--box_count", required=True)
    p.add_argument("--desk_count", required=True)
    args, _unknown = p.parse_known_args()
    return args


def read(path: str, missing: list) -> str:
    """File contents, or "" with the path recorded — no exception ever escapes."""
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            return handle.read()
    except OSError:
        missing.append(path)
        return ""


def mission_wall_s(missing: list):
    """`phases.mission_s` out of run.json, or None when it is not a number.

    None is the platform's "unknown", not a failure: a case that never sent a goal has
    no mission duration to report, and reporting 0 would poison the metric's baseline.
    """
    text = read(RUN_JSON, missing)
    if not text:
        return None
    try:
        value = json.loads(text)["phases"]["mission_s"]
    except (ValueError, KeyError, TypeError):
        return None
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def main() -> int:
    args = parse_args()
    missing: list = []

    sim_log = read(SIM_LOG, missing)
    app_log = read(APP_LOG, missing)
    mission = read(MISSION_TXT, missing)
    wall_s = mission_wall_s(missing)

    note = f"target={args.target}"
    if missing:
        note += "; missing evidence: " + ", ".join(missing)
    elif mission:
        # The tail is what a developer looks at first: the result block the CLI printed.
        note += "; mission tail: " + " ".join(mission.strip().splitlines()[-3:])[:300]

    verdict = {
        "sim_booted": SIM_READY in sim_log,
        "app_ready": bool(APP_READY.search(app_log)),
        "action_succeeded": GOAL_SUCCEEDED in mission,
        "target_found": bool(TARGET_FOUND.search(mission)),
        "mission_wall_s": wall_s,
        "note": note,
    }
    print(json.dumps(verdict))
    return 0


if __name__ == "__main__":
    sys.exit(main())
