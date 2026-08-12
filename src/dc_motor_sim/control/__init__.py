"""Position controllers sharing a common interface."""

from dc_motor_sim.control.base import PositionController
from dc_motor_sim.control.fuzzy import FuzzyMotorController
from dc_motor_sim.control.pid import PIDMotorController

__all__ = ["FuzzyMotorController", "PIDMotorController", "PositionController"]
