"""Closed-loop simulation driver and its result container."""

from dc_motor_sim.simulation.result import SimulationResult
from dc_motor_sim.simulation.runner import run_simulation

__all__ = ["SimulationResult", "run_simulation"]
