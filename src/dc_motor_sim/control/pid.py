"""PID position controller, wrapping the :mod:`simple_pid` library."""

from __future__ import annotations

from simple_pid import PID

from dc_motor_sim.config import DEFAULT_PID, PIDParams

__all__ = ["PIDMotorController"]


class PIDMotorController:
    """PID controller.

    Implements :class:`~dc_motor_sim.control.base.PositionController`.

    Args:
        params: Gains and output saturation limits. Defaults to
            :data:`~dc_motor_sim.config.DEFAULT_PID`.
    """

    name = "PID"

    def __init__(self, params: PIDParams = DEFAULT_PID) -> None:
        self.params = params
        self.pid = PID(params.kp, params.ki, params.kd, setpoint=0.0)
        self.pid.output_limits = params.output_limits
        # Let the caller own the timing: the simulation supplies dt explicitly.
        self.pid.sample_time = None

    def set_target(self, target_deg: float) -> None:
        """Set the PID setpoint in degrees."""
        self.pid.setpoint = target_deg

    def compute(self, measured_deg: float, dt: float) -> float:
        """Return the control signal for the given encoder reading.

        Returns ``0.0`` if the underlying PID has been switched to manual
        (``auto_mode = False``), in which case it produces no output.
        """
        output = self.pid(measured_deg, dt=dt)
        return 0.0 if output is None else float(output)

    def reset(self) -> None:
        """Clear the integral term and derivative history."""
        self.pid.reset()

    def get_components(self) -> tuple[float, float, float]:
        """Return the ``(proportional, integral, derivative)`` contributions."""
        p, i, d = self.pid.components
        return float(p), float(i), float(d)

    def set_tunings(self, kp: float, ki: float, kd: float) -> None:
        """Update the PID gains in place."""
        self.pid.tunings = (kp, ki, kd)

    @property
    def tunings(self) -> tuple[float, float, float]:
        """Return the current ``(kp, ki, kd)`` gains."""
        kp, ki, kd = self.pid.tunings
        return float(kp), float(ki), float(kd)
