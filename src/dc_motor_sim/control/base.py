"""Common interface shared by every position controller.

The fuzzy and PID controllers were originally driven by two near-identical
simulation loops because their ``compute_control`` signatures disagreed: the
fuzzy one wanted ``(error, delta_error, dt)`` while the PID one wanted
``(measured_position, dt)``. This protocol removes that asymmetry — each
controller owns its own setpoint and derives whatever error terms it needs from
the measurement — which lets a single loop drive both.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

__all__ = ["PositionController"]


@runtime_checkable
class PositionController(Protocol):
    """A closed-loop position controller driven by encoder measurements."""

    name: str
    """Human-readable controller name, used in logs and plot titles."""

    def set_target(self, target_deg: float) -> None:
        """Set the desired shaft position in degrees."""
        ...

    def compute(self, measured_deg: float, dt: float) -> float:
        """Return the control signal for the current measurement.

        Args:
            measured_deg: Encoder reading in degrees.
            dt: Time since the previous control update (s).

        Returns:
            A control signal, later scaled into a motor voltage by
            :attr:`~dc_motor_sim.config.SimParams.voltage_scale`.
        """
        ...

    def reset(self) -> None:
        """Clear accumulated internal state (integral terms, error history)."""
        ...
