"""Custom oracle: upright — the quadruped never fell over (decision 2026-08-31 D-6).

A wheeled robot cannot fail this way, so the platform has no built-in for it: the MVP
oracles judge WHERE the robot went (`reached_goal`) and WHAT it hit (`no_collision`), and
a Go2 can satisfy both while ending the run on its side. Attitude is the missing axis, and
it is available in the GT telemetry the platform already records
(``orientation_wxyz`` per pose sample), so this is a pure-Python plugin — no new platform
capability, no contract change.

Plugin rules (platform user-guide §4): module scope = ``cv_infra.*`` + stdlib only,
deterministic pure Python, defensive ``validate_params`` (raise -> exit 2, pre-boot).

Thresholds are the SCENARIO's to declare; both are required here on purpose. There is no
"sensible default" for how far a robot may lean — it depends on the robot, the gait and
the terrain, and a silently defaulted attitude limit is exactly the kind of number that
turns a fall into a pass.
"""

from __future__ import annotations

import math

from cv_infra.oracles.base import OracleBase
from cv_infra.runner.evaluate import OracleOutcome, read_field

ROLL_PARAM = "max_roll_rad"
PITCH_PARAM = "max_pitch_rad"


def _positive_number(criteria: object, name: str) -> float:
    value = read_field(criteria, name)
    if value is None:
        raise ValueError(f"upright criteria require {name} (radians, > 0)")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
        raise ValueError(f"{name} must be a positive number of radians, got {value!r}")
    return float(value)


def roll_pitch_from_quat_wxyz(q: tuple[float, float, float, float]) -> tuple[float, float]:
    """(roll, pitch) in radians from a (w, x, y, z) quaternion — stdlib only.

    Standard ZYX (yaw-pitch-roll) extraction; yaw is deliberately not returned, because a
    patrol robot's heading is not an attitude fault. The pitch argument is clamped before
    ``asin`` so a quaternion that is a hair off unit length cannot raise instead of judging.
    """
    w, x, y, z = (float(v) for v in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


class UprightOracle(OracleBase):
    """Passes iff |roll| <= ``max_roll_rad`` AND |pitch| <= ``max_pitch_rad`` for EVERY GT
    sample of the run (not just at the end: a robot that fell and got back up did fall)."""

    name = "upright"
    version = "0.1.0"

    def validate_params(self, criteria: object) -> None:
        _positive_number(criteria, ROLL_PARAM)
        _positive_number(criteria, PITCH_PARAM)

    def evaluate(self, telemetry: object, criteria: object) -> OracleOutcome:
        try:
            self.validate_params(criteria)
        except ValueError as exc:
            return OracleOutcome(self.name, passed=False, reason="bad_criteria", detail=str(exc))

        max_roll_rad = _positive_number(criteria, ROLL_PARAM)
        max_pitch_rad = _positive_number(criteria, PITCH_PARAM)

        samples = telemetry.gt_pose_samples
        if not samples:
            return OracleOutcome(
                self.name, passed=False, reason="no_telemetry", detail="no GT pose samples"
            )

        worst_roll_rad = 0.0
        worst_pitch_rad = 0.0
        violation = None
        for sample in samples:
            roll, pitch = roll_pitch_from_quat_wxyz(sample.orientation_wxyz)
            worst_roll_rad = max(worst_roll_rad, abs(roll))
            worst_pitch_rad = max(worst_pitch_rad, abs(pitch))
            if violation is None and (abs(roll) > max_roll_rad or abs(pitch) > max_pitch_rad):
                violation = (sample.sim_time_s, roll, pitch)

        if violation is not None:
            time_s, roll, pitch = violation
            return OracleOutcome(
                self.name,
                passed=False,
                reason="toppled",
                detail=(
                    f"attitude limit exceeded at sim_time={time_s:.2f}s: roll {roll:+.3f} rad "
                    f"(limit {max_roll_rad:.3f}), pitch {pitch:+.3f} rad "
                    f"(limit {max_pitch_rad:.3f})"
                ),
            )
        return OracleOutcome(
            self.name,
            passed=True,
            detail=(
                f"stayed upright over {len(samples)} sample(s): worst |roll| "
                f"{worst_roll_rad:.3f} rad, worst |pitch| {worst_pitch_rad:.3f} rad "
                f"(limits {max_roll_rad:.3f} / {max_pitch_rad:.3f})"
            ),
        )
