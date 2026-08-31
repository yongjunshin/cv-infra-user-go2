"""Custom oracle: hold_near_goal — "it was STILL there at the end", not "it touched it once".

Why this exists (decision 2026-08-31 D-6): the built-in ``reached_goal`` is FIRST-REACH —
a robot that clips the goal radius at 0.4 m/s and keeps walking passes it. The go2 patrol
mission is the opposite claim: the robot found the target, stopped in front of it and
STAYED there while it confirmed what it was looking at. That is a statement about the LAST
K seconds of the run, so it needs its own oracle.

Plugin rules this file obeys (platform user-guide §4, same as the carter fixture's
``max_time_to_goal.py``):

* module scope imports ONLY ``cv_infra.*`` + stdlib — never ``omni.*`` / ``isaacsim.*``
  (the runner composes the evaluation engine BEFORE the simulator boots);
* deterministic pure Python: the merged criteria view + the GT telemetry record, nothing
  else (no clock, no randomness, no network);
* defensive ``validate_params``: a bad param is rejected with a message, never silently
  defaulted — the check runs pre-boot and a raise maps to exit 2.

⚠ PLANAR distance on purpose. ``goal_position`` is planar (z = 0, the platform writes it
that way) while a walking Go2's base rides ~0.28 m above the floor (measured: settled
0.2818 m, C1 §3-3). A 3D distance would therefore spend 0.28 m of the radius on the
robot's own height before the robot has moved at all. The built-in ``reached_goal`` uses
the 3D distance and absorbs that inside a larger tolerance; this oracle judges where the
robot STOOD, so it compares x/y and says so out loud in every message it emits.
"""

from __future__ import annotations

import math

from cv_infra.oracles.base import OracleBase
from cv_infra.runner.evaluate import OracleOutcome, read_field

RADIUS_PARAM = "hold_radius_m"
DURATION_PARAM = "hold_duration_s"


def _positive_number(criteria: object, name: str) -> float:
    """Read a required, strictly positive numeric param or raise with a usable message."""
    value = read_field(criteria, name)
    if value is None:
        raise ValueError(f"hold_near_goal criteria require {name} (> 0)")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ValueError(f"{name} must be a positive number, got {value!r}")
    return float(value)


class HoldNearGoalOracle(OracleBase):
    """Passes iff EVERY GT sample in the last ``hold_duration_s`` of sim time lies within
    ``hold_radius_m`` (x/y) of the goal.

    Failure modes it separates, because they mean different things to the developer:
    ``no_telemetry`` (nothing was recorded), ``record_too_short`` (the run ended before a
    full hold window could exist — a mission that timed out cannot be said to have held),
    and the real verdict (the worst sample in the window, with its distance and sim time).
    """

    name = "hold_near_goal"
    version = "0.1.0"

    def validate_params(self, criteria: object) -> None:
        if read_field(criteria, "goal_position") is None:
            raise ValueError("hold_near_goal criteria require a goal_position [x, y, z]")
        _positive_number(criteria, RADIUS_PARAM)
        _positive_number(criteria, DURATION_PARAM)

    def evaluate(self, telemetry: object, criteria: object) -> OracleOutcome:
        try:
            self.validate_params(criteria)
        except ValueError as exc:
            return OracleOutcome(self.name, passed=False, reason="bad_criteria", detail=str(exc))

        radius_m = _positive_number(criteria, RADIUS_PARAM)
        duration_s = _positive_number(criteria, DURATION_PARAM)
        goal = read_field(criteria, "goal_position")
        goal_xy = (float(goal[0]), float(goal[1]))

        samples = telemetry.gt_pose_samples
        if not samples:
            return OracleOutcome(
                self.name, passed=False, reason="no_telemetry", detail="no GT pose samples"
            )

        end_s = samples[-1].sim_time_s
        span_s = end_s - samples[0].sim_time_s
        if span_s < duration_s:
            return OracleOutcome(
                self.name,
                passed=False,
                reason="record_too_short",
                detail=(
                    f"the run is {span_s:.2f}s of sim time, shorter than the "
                    f"{duration_s:.2f}s hold window — nothing was held"
                ),
            )

        window_start_s = end_s - duration_s
        worst_distance_m = -1.0
        worst_time_s = end_s
        counted = 0
        for sample in samples:
            if sample.sim_time_s < window_start_s:
                continue
            counted += 1
            distance_m = math.dist((sample.position[0], sample.position[1]), goal_xy)
            if distance_m > worst_distance_m:
                worst_distance_m = distance_m
                worst_time_s = sample.sim_time_s

        if counted == 0:  # pragma: no cover - a span >= duration always yields the last sample
            return OracleOutcome(
                self.name,
                passed=False,
                reason="no_telemetry",
                detail="no GT samples inside the hold window",
            )

        detail = (
            f"{counted} sample(s) in the last {duration_s:.2f}s; worst xy distance "
            f"{worst_distance_m:.3f} m at sim_time={worst_time_s:.2f}s (radius {radius_m:.3f} m)"
        )
        if worst_distance_m > radius_m:
            return OracleOutcome(self.name, passed=False, reason="left_goal", detail=detail)
        return OracleOutcome(self.name, passed=True, detail=detail)
