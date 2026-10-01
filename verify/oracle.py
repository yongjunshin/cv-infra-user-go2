#!/usr/bin/env python3
"""verify/oracle.py — turn one search case's evidence into a verdict.

cv-infra runs this right after `verify/sim.py`, in the SAME image, with the SAME argv, no
GPU, and both the checkout and verify/out/ mounted read-only. The last line of stdout
that parses as a flat JSON object is the case's verdict, and the platform reads TYPES,
not names:

    bool   a check   — the case passes when every bool is true
    number a metric  — compared against the baseline across commits, never gates
    null   unknown   — not a failure, just excluded from the ratio
    str    a note

THE QUESTION: the app was asked for ONE class. Did it answer right, and in time?

  target PRESENT (--hide=H1..H4):
    report_correct  it said found, and the position it reported is within REPORT_TOL_M of
                    where the target really stands (a decoy or a hallucination elsewhere
                    is not a correct report)
    in_time         and it said so within the allowance for that distance:
                    ALLOW_BASE_S + ALLOW_PER_M_S * (straight-line start -> target, m)
  target ABSENT (--hide=none; a decoy of the OTHER class may be standing there):
    report_correct  it never said found — "there is a chair" when there is none is the
                    one answer that must never come
    in_time         and it kept looking for at least ABSENT_WATCH_S before giving up (a
                    mission that aborts after 5 s has not searched for anything)

The allowance and the watch time are THE TEST'S REQUIREMENT — what this repository asks
of the app — not a measurement of it. Times are sim seconds from goal acceptance to the
answer, read from run.json (the harness stamps them off the simulator's own clock).

A missing run.json means the harness never got as far as a mission: that is the ERROR
lane (exit 1, no verdict), not a failing robot.

stdlib only, so this also runs on a plain laptop python3.
"""

import argparse
import csv
import json
import math
import os
import sys

OUT = os.path.join("verify", "out")
RUN_JSON = os.path.join(OUT, "run.json")
TRAJECTORY = os.path.join(OUT, "trajectory.csv")

ALLOW_BASE_S = 45.0
ALLOW_PER_M_S = 8.0  # ~3x the time the robot needs to walk the straight line at 0.4 m/s
ABSENT_WATCH_S = 150.0
REPORT_TOL_M = 1.0


def parse_args() -> argparse.Namespace:
    """The same axes as verify/sim.py — the platform replays the whole argv."""
    p = argparse.ArgumentParser(description="verdict for one go2 search case")
    for axis in ("start", "target", "hide", "decoy", "slot_a", "slot_b"):
        p.add_argument(f"--{axis}", required=True)
    args, _unknown = p.parse_known_args()
    return args


def path_length(path: str):
    try:
        with open(path, newline="") as handle:
            rows = list(csv.DictReader(handle))
    except OSError:
        return None
    return sum(
        math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
        for a, b in zip(rows, rows[1:], strict=False)
    )


def main() -> int:
    args = parse_args()
    try:
        with open(RUN_JSON) as handle:
            run = json.load(handle)
    except (OSError, ValueError) as exc:
        print(f"ERROR cannot read the case output: {exc}", file=sys.stderr, flush=True)
        return 1

    outcome = run.get("outcome") or {}
    found = outcome.get("found") is True
    mission_s = outcome.get("mission_s")
    target = next((ob for ob in run.get("objects") or [] if ob.get("role") == "target"), None)
    start = run.get("start") or {}

    if target is not None:
        distance = math.hypot(target["x"] - start["x"], target["y"] - start["y"])
        allowed = ALLOW_BASE_S + ALLOW_PER_M_S * distance
        error = outcome.get("report_error_m")
        report_correct = found and error is not None and error <= REPORT_TOL_M
        in_time = found and mission_s is not None and mission_s <= allowed
        expect = f"present at {args.hide}, {distance:.1f} m away, allowed {allowed:.0f} s"
    else:
        distance = allowed = error = None
        report_correct = not found
        in_time = not found and mission_s is not None and mission_s >= ABSENT_WATCH_S
        expect = f"absent, must not be reported for {ABSENT_WATCH_S:.0f} s"

    answer = "FOUND" if found else "not found"
    note = (
        f"{args.start}: find {args.target} ({expect}, decoy {args.decoy}) -> {answer}"
        + (f" after {mission_s:.1f} s" if mission_s is not None else "")
        + (f", reported {error:.2f} m off" if error is not None else "")
        + f"; ended by {outcome.get('end')}; app said: {outcome.get('message')}"
    )
    verdict = {
        "report_correct": bool(report_correct),
        "in_time": bool(in_time),
        "mission_s": mission_s,
        "allowed_s": None if allowed is None else round(allowed, 1),
        "target_dist_m": None if distance is None else round(distance, 3),
        "report_error_m": error,
        "path_len_m": None if (length := path_length(TRAJECTORY)) is None else round(length, 3),
        "note": note,
    }
    print(json.dumps(verdict))
    return 0


if __name__ == "__main__":
    sys.exit(main())
