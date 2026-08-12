"""Container for the time series produced by a simulation run."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["SimulationResult"]

_SERIES = (
    "time",
    "actual_position",
    "measured_position",
    "target",
    "error",
    "control",
    "voltage",
    "current",
    "velocity",
)


@dataclass(frozen=True, slots=True)
class SimulationResult:
    """The full trace of one closed-loop run.

    Every series has ``steps + 1`` samples: index 0 holds the initial
    condition, and index ``k`` holds the state after control step ``k``.
    """

    time: np.ndarray
    """Elapsed simulation time (s)."""
    actual_position: np.ndarray
    """True shaft position (degrees)."""
    measured_position: np.ndarray
    """Encoder reading — quantized and noisy (degrees)."""
    target: np.ndarray
    """Setpoint (degrees)."""
    error: np.ndarray
    """``target - measured_position`` at the start of each step (degrees)."""
    control: np.ndarray
    """Controller output, before voltage scaling."""
    voltage: np.ndarray
    """Voltage applied to the armature (V)."""
    current: np.ndarray
    """Armature current (A)."""
    velocity: np.ndarray
    """Angular velocity (degrees/s)."""
    steps: int
    """Number of control steps executed."""
    converged: bool
    """Whether the run met the convergence criteria before ``max_steps``."""
    controller_name: str
    """Name of the controller that produced this trace."""
    start_deg: float
    """Initial position the run started from (degrees)."""
    target_deg: float
    """Setpoint the run aimed for (degrees)."""

    def __post_init__(self) -> None:
        expected = self.steps + 1
        for name in _SERIES:
            series = getattr(self, name)
            if series.shape != (expected,):
                raise ValueError(
                    f"{name} has shape {series.shape}, expected ({expected},) "
                    f"for a {self.steps}-step run"
                )

    @property
    def final_position(self) -> float:
        """True shaft position at the end of the run (degrees)."""
        return float(self.actual_position[-1])

    @property
    def final_error(self) -> float:
        """Setpoint minus the true final position (degrees)."""
        return self.target_deg - self.final_position

    @property
    def duration(self) -> float:
        """Total simulated time (s)."""
        return float(self.time[-1])

    @property
    def overshoot(self) -> float:
        """Peak excursion beyond the setpoint, as a fraction of the step size.

        Returns ``0.0`` when the run never crosses the setpoint or when the
        start and target coincide.
        """
        span = self.target_deg - self.start_deg
        if span == 0:
            return 0.0
        beyond = (self.actual_position - self.target_deg) / span
        return float(max(beyond.max(), 0.0))

    def summary(self) -> dict[str, float | int | bool | str]:
        """Return the scalar metrics of the run, for logging or reporting."""
        return {
            "controller": self.controller_name,
            "start_deg": self.start_deg,
            "target_deg": self.target_deg,
            "steps": self.steps,
            "converged": self.converged,
            "duration_s": self.duration,
            "final_position_deg": self.final_position,
            "final_error_deg": self.final_error,
            "overshoot": self.overshoot,
        }

    def __repr__(self) -> str:
        return (
            f"SimulationResult(controller={self.controller_name!r}, "
            f"start={self.start_deg}, target={self.target_deg}, "
            f"steps={self.steps}, converged={self.converged}, "
            f"final={self.final_position:.3f})"
        )
