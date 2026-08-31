"""Test wiring for the two custom oracles — hermetic by default, real when available.

The oracles import three things from the platform: ``OracleBase``, ``OracleOutcome`` and
``read_field``. A consumer developer editing an oracle should not have to install cv-infra
to run these tests, so when ``cv_infra`` is not importable this file installs a MINIMAL
stand-in for exactly those three names. That stand-in doubles as documentation of the whole
platform surface these plugins touch — if it ever needs to grow, the plugin has started
depending on more of the platform than the user guide promises.

When the real ``cv_infra`` IS importable (e.g. running under the platform's venv), it is
used as-is, and the banner below says which of the two ran. Both modes were exercised for
the U3 report; the CI-relevant one is the hermetic default.
"""

from __future__ import annotations

import pathlib
import sys
import types

# The oracles live one directory up, exactly as the platform mounts them (scenario-adjacent
# plugin dir on sys.path).
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:  # pragma: no cover - which branch runs depends on the environment, both are fine
    import cv_infra.oracles.base  # noqa: F401
    import cv_infra.runner.evaluate  # noqa: F401

    ORACLE_API = "real cv_infra"
except Exception:  # pragma: no cover - see above
    from abc import ABC, abstractmethod
    from collections.abc import Mapping
    from dataclasses import dataclass, field

    class OracleBase(ABC):
        """Stand-in for ``cv_infra.oracles.base.OracleBase`` (name/version + 2 methods)."""

        name: str
        version: str

        @abstractmethod
        def validate_params(self, criteria: object) -> None: ...

        @abstractmethod
        def evaluate(self, telemetry: object, criteria: object) -> object: ...

    @dataclass(frozen=True)
    class OracleOutcome:
        """Stand-in for ``cv_infra.runner.evaluate.OracleOutcome``."""

        name: str
        passed: bool
        reason: str = ""
        detail: str = ""
        metrics: dict = field(default_factory=dict)

    def read_field(criteria: object, name: str, default: object = None) -> object:
        """Stand-in for ``cv_infra.runner.evaluate.read_field``."""
        if isinstance(criteria, Mapping):
            return criteria.get(name, default)
        return getattr(criteria, name, default)

    cv_infra = types.ModuleType("cv_infra")
    oracles = types.ModuleType("cv_infra.oracles")
    base = types.ModuleType("cv_infra.oracles.base")
    runner = types.ModuleType("cv_infra.runner")
    evaluate = types.ModuleType("cv_infra.runner.evaluate")
    base.OracleBase = OracleBase
    evaluate.OracleOutcome = OracleOutcome
    evaluate.read_field = read_field
    oracles.base = base
    runner.evaluate = evaluate
    cv_infra.oracles = oracles
    cv_infra.runner = runner
    sys.modules.update(
        {
            "cv_infra": cv_infra,
            "cv_infra.oracles": oracles,
            "cv_infra.oracles.base": base,
            "cv_infra.runner": runner,
            "cv_infra.runner.evaluate": evaluate,
        }
    )
    ORACLE_API = "stub cv_infra (platform not installed)"


def pytest_report_header(config):  # noqa: ARG001 - pytest hook signature
    return f"oracle API under test: {ORACLE_API}"
