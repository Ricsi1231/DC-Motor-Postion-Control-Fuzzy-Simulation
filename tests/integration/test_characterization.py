"""Golden-trace tests locking in pre-refactor simulation behaviour.

The traces in ``tests/data`` were captured from the original flat-layout
scripts before the package restructure, with encoder noise disabled so the runs
are deterministic. They exist to prove the refactor did not change the physics,
the controllers, or the loop. A failure here means behaviour drifted — fix the
code, do not regenerate the goldens.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from dc_motor_sim import EncoderParams, FuzzyMotorController, PIDMotorController, run_simulation

DATA_DIR = Path(__file__).parent.parent / "data"
GOLDEN_FILES = sorted(DATA_DIR.glob("golden_*.json"))

SERIES = ("actual_positions", "measured_positions", "targets", "errors", "control_signals")
FIELD_FOR = {
    "actual_positions": "actual_position",
    "measured_positions": "measured_position",
    "targets": "target",
    "errors": "error",
    "control_signals": "control",
    "time_steps": "time",
}


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _controller(kind: str):
    return FuzzyMotorController() if kind == "fuzzy" else PIDMotorController()


def test_golden_files_exist() -> None:
    """The characterization corpus must not silently disappear."""
    assert GOLDEN_FILES, f"no golden traces found in {DATA_DIR}"


@pytest.mark.parametrize("path", GOLDEN_FILES, ids=lambda p: p.stem)
def test_matches_pre_refactor_trace(path: Path) -> None:
    """The refactored runner reproduces the original trace exactly."""
    golden = _load(path)

    result = run_simulation(
        _controller(golden["controller"]),
        golden["start_deg"],
        golden["target_deg"],
        # Noise disabled: this is what makes the original run reproducible.
        encoder_params=EncoderParams(noise_std=0.0),
    )

    assert result.steps == golden["steps"]

    for key in ("time_steps", *SERIES):
        expected = np.asarray(golden[key], dtype=float)
        actual = getattr(result, FIELD_FOR[key])
        np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=0.0, err_msg=key)
