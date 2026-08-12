"""Shared pytest fixtures.

The matplotlib backend is forced to ``Agg`` before anything imports pyplot, so
the suite never tries to open a window.
"""

from __future__ import annotations

import os

os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np
import pytest

from dc_motor_sim import (
    EncoderParams,
    FuzzyMotorController,
    MotorParams,
    PIDMotorController,
    PIDParams,
    RotaryEncoder,
    SimParams,
)

SEED = 1234


@pytest.fixture
def rng() -> np.random.Generator:
    """A seeded generator, so any test using encoder noise is reproducible."""
    return np.random.default_rng(SEED)


@pytest.fixture
def motor_params() -> MotorParams:
    return MotorParams()


@pytest.fixture
def sim_params() -> SimParams:
    return SimParams()


@pytest.fixture
def noiseless_encoder_params() -> EncoderParams:
    return EncoderParams(noise_std=0.0)


@pytest.fixture
def noiseless_encoder(noiseless_encoder_params: EncoderParams) -> RotaryEncoder:
    return RotaryEncoder(params=noiseless_encoder_params)


@pytest.fixture(scope="session")
def fuzzy_controller() -> FuzzyMotorController:
    """Session-scoped: building the scikit-fuzzy ControlSystem is expensive.

    Tests that mutate controller state must call ``reset()`` themselves; the
    ``fresh_fuzzy_controller`` fixture below does that automatically.
    """
    return FuzzyMotorController()


@pytest.fixture
def fresh_fuzzy_controller(fuzzy_controller: FuzzyMotorController) -> FuzzyMotorController:
    """The shared fuzzy controller, reset before and after each test."""
    fuzzy_controller.reset()
    yield fuzzy_controller
    fuzzy_controller.reset()


@pytest.fixture
def pid_controller() -> PIDMotorController:
    return PIDMotorController()


@pytest.fixture
def pid_params() -> PIDParams:
    return PIDParams()
