"""Fixture tests for the two custom oracles (`python3 -m pytest scenarios/test -q`).

Every fixture below is a hand-built GT telemetry record shaped like the platform's
(``gt_pose_samples`` of ``sim_time_s`` / ``position`` / ``orientation_wxyz``), with the
numbers this repo actually runs on: goal (-6.0, 4.4, 0), target 0.8 m beyond it, the app
standing 1.2 m from the target, and a base that rides ~0.28 m above the floor.

The pass/fail pairs are the point: for each oracle there is a run that must pass and a run
that must fail, and for `hold_near_goal` the failing one is a run that the BUILT-IN
`reached_goal` would happily pass (it clips the goal radius and walks on) — which is the
entire reason this oracle exists.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import pytest

from hold_near_goal import HoldNearGoalOracle
from upright import UprightOracle, roll_pitch_from_quat_wxyz

GOAL = [-6.0, 4.4, 0.0]  # the TB scenarios' verdict anchor (0.8 m in front of the target)
BASE_Z = 0.28  # measured settled base height (platform C1 §3-3)
LEVEL = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Sample:
    sim_time_s: float
    position: tuple
    orientation_wxyz: tuple = LEVEL


@dataclass
class Telemetry:
    gt_pose_samples: list = field(default_factory=list)


def walk_then_hold(hold_s: float = 6.0, hold_offset_m: float = 0.4, dt: float = 0.1):
    """Robot walks up the aisle and then stands `hold_offset_m` short of the goal.

    That offset is the mission's own geometry: the app stops 1.2 m from the target while
    the goal sits 0.8 m from it.
    """
    samples = []
    t = 0.0
    for y in [0.0, 1.0, 2.0, 3.0]:  # approach, 1 sample per metre is enough for the predicate
        samples.append(Sample(t, (-6.0, y, BASE_Z)))
        t += 1.0
    y_hold = GOAL[1] - hold_offset_m
    steps = int(hold_s / dt)
    for _ in range(steps + 1):
        samples.append(Sample(t, (-6.0, y_hold, BASE_Z)))
        t += dt
    return Telemetry(samples)


def walk_through(dt: float = 0.1):
    """Robot walks THROUGH the goal and keeps going — `reached_goal` passes, holding fails."""
    samples = []
    t = 0.0
    y = 0.0
    while y < 9.0:
        samples.append(Sample(t, (-6.0, y, BASE_Z)))
        y += 0.05
        t += dt
    return Telemetry(samples)


def quat_roll(angle_rad: float):
    return (math.cos(angle_rad / 2.0), math.sin(angle_rad / 2.0), 0.0, 0.0)


def quat_pitch(angle_rad: float):
    return (math.cos(angle_rad / 2.0), 0.0, math.sin(angle_rad / 2.0), 0.0)


def quat_yaw(angle_rad: float):
    return (math.cos(angle_rad / 2.0), 0.0, 0.0, math.sin(angle_rad / 2.0))


# --------------------------------------------------------------------------- #
# hold_near_goal
# --------------------------------------------------------------------------- #
HOLD_CRITERIA = {"goal_position": GOAL, "hold_radius_m": 1.0, "hold_duration_s": 4.0}


def test_hold_passes_when_the_robot_stands_in_front_of_the_target():
    outcome = HoldNearGoalOracle().evaluate(walk_then_hold(), HOLD_CRITERIA)
    assert outcome.passed, outcome.detail
    assert "worst xy distance 0.400" in outcome.detail


def test_hold_fails_the_drive_by_that_reached_goal_would_pass():
    telemetry = walk_through()
    # The built-in oracle's semantics, reproduced here in one line: SOME sample came within
    # 1.0 m of the goal. That is true...
    assert any(
        math.dist(s.position[:2], (GOAL[0], GOAL[1])) <= 1.0 for s in telemetry.gt_pose_samples
    )
    # ...and this oracle still fails it, because the robot did not stay.
    outcome = HoldNearGoalOracle().evaluate(telemetry, HOLD_CRITERIA)
    assert not outcome.passed
    assert outcome.reason == "left_goal"


def test_hold_fails_when_the_run_is_shorter_than_the_window():
    telemetry = Telemetry([Sample(0.0, (-6.0, 4.4, BASE_Z)), Sample(1.0, (-6.0, 4.4, BASE_Z))])
    outcome = HoldNearGoalOracle().evaluate(telemetry, HOLD_CRITERIA)
    assert not outcome.passed
    assert outcome.reason == "record_too_short"
    assert "nothing was held" in outcome.detail


def test_hold_judges_xy_not_the_robots_own_height():
    # Standing exactly on the goal with the base 0.28 m up: a 3D distance would be 0.28 m
    # and would fail a tight radius. This oracle judges where the robot STOOD.
    telemetry = Telemetry([Sample(t / 10.0, (GOAL[0], GOAL[1], BASE_Z)) for t in range(101)])
    outcome = HoldNearGoalOracle().evaluate(
        telemetry, {**HOLD_CRITERIA, "hold_radius_m": 0.1, "hold_duration_s": 5.0}
    )
    assert outcome.passed, outcome.detail


def test_hold_reports_no_telemetry_rather_than_crashing():
    outcome = HoldNearGoalOracle().evaluate(Telemetry([]), HOLD_CRITERIA)
    assert not outcome.passed
    assert outcome.reason == "no_telemetry"


@pytest.mark.parametrize(
    "criteria, missing",
    [
        ({"hold_radius_m": 1.0, "hold_duration_s": 4.0}, "goal_position"),
        ({"goal_position": GOAL, "hold_duration_s": 4.0}, "hold_radius_m"),
        ({"goal_position": GOAL, "hold_radius_m": 1.0}, "hold_duration_s"),
    ],
)
def test_hold_validate_params_rejects_missing_params(criteria, missing):
    with pytest.raises(ValueError, match=missing):
        HoldNearGoalOracle().validate_params(criteria)


@pytest.mark.parametrize("bad", [0, -1.0, True, "1.0", None])
def test_hold_validate_params_rejects_nonsense_radius(bad):
    with pytest.raises(ValueError):
        HoldNearGoalOracle().validate_params({**HOLD_CRITERIA, "hold_radius_m": bad})


def test_hold_evaluate_maps_bad_params_to_an_outcome_not_an_exception():
    # validate_params raising is the PRE-BOOT path (exit 2). At evaluation time the same
    # criteria must produce a readable failure, not a traceback in the middle of a job.
    outcome = HoldNearGoalOracle().evaluate(walk_then_hold(), {"goal_position": GOAL})
    assert not outcome.passed
    assert outcome.reason == "bad_criteria"
    assert "hold_radius_m" in outcome.detail


# --------------------------------------------------------------------------- #
# upright
# --------------------------------------------------------------------------- #
# Limits from the measured attitude envelope: walking roll/pitch is ~0.011/0.035 rad
# (platform C3 §4-2 tf2_echo) and a Go2 that has sat down measures roll -0.216 rad
# (C1 §3, the control run). 0.5 rad is ~14x the walking value and well under a fall.
UPRIGHT_CRITERIA = {"max_roll_rad": 0.5, "max_pitch_rad": 0.5}


def test_upright_passes_a_normal_walking_run():
    telemetry = Telemetry(
        [
            Sample(0.0, (-6.0, 0.0, BASE_Z), quat_roll(0.011)),
            Sample(1.0, (-6.0, 1.0, BASE_Z), quat_pitch(0.035)),
            Sample(2.0, (-6.0, 2.0, BASE_Z), quat_yaw(1.5708)),  # heading is not an attitude fault
        ]
    )
    outcome = UprightOracle().evaluate(telemetry, UPRIGHT_CRITERIA)
    assert outcome.passed, outcome.detail
    assert "worst |roll| 0.011" in outcome.detail


def test_upright_fails_a_run_that_topples():
    telemetry = Telemetry(
        [
            Sample(0.0, (-6.0, 0.0, BASE_Z), LEVEL),
            Sample(3.2, (-6.0, 1.0, 0.08), quat_roll(1.2)),  # on its side
            Sample(4.0, (-6.0, 1.0, 0.08), quat_roll(1.2)),
        ]
    )
    outcome = UprightOracle().evaluate(telemetry, UPRIGHT_CRITERIA)
    assert not outcome.passed
    assert outcome.reason == "toppled"
    assert "sim_time=3.20s" in outcome.detail


def test_upright_fails_on_pitch_alone():
    telemetry = Telemetry([Sample(1.0, (-6.0, 0.0, 0.1), quat_pitch(0.9))])
    outcome = UprightOracle().evaluate(telemetry, UPRIGHT_CRITERIA)
    assert not outcome.passed
    assert outcome.reason == "toppled"


def test_upright_fails_even_if_the_robot_gets_back_up():
    telemetry = Telemetry(
        [
            Sample(0.0, (-6.0, 0.0, BASE_Z), LEVEL),
            Sample(1.0, (-6.0, 0.5, 0.08), quat_roll(1.0)),
            Sample(2.0, (-6.0, 1.0, BASE_Z), LEVEL),
        ]
    )
    outcome = UprightOracle().evaluate(telemetry, UPRIGHT_CRITERIA)
    assert not outcome.passed, "a robot that fell and recovered still fell"


def test_upright_reports_no_telemetry_rather_than_crashing():
    outcome = UprightOracle().evaluate(Telemetry([]), UPRIGHT_CRITERIA)
    assert not outcome.passed
    assert outcome.reason == "no_telemetry"


@pytest.mark.parametrize("bad", [0, -0.1, True, "0.5", None])
def test_upright_validate_params_rejects_nonsense_limits(bad):
    with pytest.raises(ValueError):
        UprightOracle().validate_params({"max_roll_rad": bad, "max_pitch_rad": 0.5})
    with pytest.raises(ValueError):
        UprightOracle().validate_params({"max_roll_rad": 0.5, "max_pitch_rad": bad})


def test_upright_evaluate_maps_bad_params_to_an_outcome_not_an_exception():
    outcome = UprightOracle().evaluate(Telemetry([Sample(0.0, (0.0, 0.0, 0.0))]), {})
    assert not outcome.passed
    assert outcome.reason == "bad_criteria"


@pytest.mark.parametrize(
    "quat, expected_roll, expected_pitch",
    [
        (LEVEL, 0.0, 0.0),
        (quat_roll(0.4), 0.4, 0.0),
        (quat_pitch(-0.3), 0.0, -0.3),
        (quat_yaw(2.0), 0.0, 0.0),
    ],
)
def test_roll_pitch_extraction(quat, expected_roll, expected_pitch):
    roll, pitch = roll_pitch_from_quat_wxyz(quat)
    assert roll == pytest.approx(expected_roll, abs=1e-9)
    assert pitch == pytest.approx(expected_pitch, abs=1e-9)


def test_roll_pitch_clamps_a_slightly_denormalised_quaternion():
    # 2*(wy - zx) can exceed 1.0 by float noise on a near-gimbal quaternion; asin must be
    # judged, not raised.
    roll, pitch = roll_pitch_from_quat_wxyz((0.7071068, 0.0, 0.7071069, 0.0))
    assert pitch == pytest.approx(math.pi / 2, abs=1e-6)
    assert math.isfinite(roll)


# --------------------------------------------------------------------------- #
# plugin contract
# --------------------------------------------------------------------------- #
def test_both_oracles_satisfy_the_plugin_contract():
    from cv_infra.oracles.base import OracleBase

    for oracle in (HoldNearGoalOracle(), UprightOracle()):
        assert isinstance(oracle, OracleBase)
        assert oracle.name and oracle.version
