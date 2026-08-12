"""DC motor position control simulation: fuzzy logic versus PID.

A closed-loop simulation of an armature-controlled DC motor with realistic
encoder feedback (quantization plus Gaussian noise), driven by either a fuzzy
logic controller or a classical PID controller.

Typical use::

    from dc_motor_sim import FuzzyMotorController, run_simulation

    result = run_simulation(FuzzyMotorController(), start_deg=-90, target_deg=45)
    print(result.summary())
"""

from __future__ import annotations

from dc_motor_sim.config import (
    DEFAULT_ENCODER,
    DEFAULT_FUZZY,
    DEFAULT_MOTOR,
    DEFAULT_PID,
    DEFAULT_SIM,
    EncoderParams,
    FuzzyParams,
    MotorParams,
    PIDParams,
    SimParams,
)
from dc_motor_sim.control.base import PositionController
from dc_motor_sim.control.fuzzy import FuzzyMotorController
from dc_motor_sim.control.pid import PIDMotorController
from dc_motor_sim.model.dc_motor import DCMotorModel
from dc_motor_sim.sensors.encoder import RotaryEncoder
from dc_motor_sim.simulation.result import SimulationResult
from dc_motor_sim.simulation.runner import run_simulation

try:  # pragma: no cover - trivial packaging fallback
    from dc_motor_sim._version import __version__
except ImportError:  # pragma: no cover - running from a source tree without a build
    __version__ = "0.0.0+unknown"

__all__ = [
    "DEFAULT_ENCODER",
    "DEFAULT_FUZZY",
    "DEFAULT_MOTOR",
    "DEFAULT_PID",
    "DEFAULT_SIM",
    "DCMotorModel",
    "EncoderParams",
    "FuzzyMotorController",
    "FuzzyParams",
    "MotorParams",
    "PIDMotorController",
    "PIDParams",
    "PositionController",
    "RotaryEncoder",
    "SimParams",
    "SimulationResult",
    "__version__",
    "run_simulation",
]
