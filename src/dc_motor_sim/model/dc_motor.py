"""DC motor physical model implementing realistic motor dynamics.

The model integrates three coupled states with explicit Euler:

- Electrical dynamics: ``L·di/dt + R·i = V - K_b·ω``
- Mechanical dynamics: ``J·dω/dt = K_m·i - K_f·ω``
- Kinematics:          ``dθ/dt = ω``

where ``V`` is the applied voltage, ``i`` the armature current, ``ω`` the
angular velocity, and ``θ`` the angular position.
"""

from __future__ import annotations

import math

from dc_motor_sim.config import DEFAULT_MOTOR, MotorParams

__all__ = ["DCMotorModel"]


class DCMotorModel:
    """Simulates an armature-controlled DC motor.

    Args:
        initial_position_deg: Starting shaft angle in degrees.
        params: Physical constants. Defaults to :data:`~dc_motor_sim.config.DEFAULT_MOTOR`.
    """

    def __init__(
        self,
        initial_position_deg: float = 0.0,
        params: MotorParams = DEFAULT_MOTOR,
    ) -> None:
        self.params = params
        self._initial_position_deg = initial_position_deg
        self.position_rad = math.radians(initial_position_deg)
        self.omega = 0.0
        self.current = 0.0

    def step(self, voltage: float, dt: float) -> float:
        """Advance the motor state by ``dt`` seconds under ``voltage``.

        Explicit Euler is only stable while ``dt`` stays below the electrical
        time constant ``L/R``; see :attr:`~dc_motor_sim.config.SimParams.substeps`.

        Args:
            voltage: Applied armature voltage (V).
            dt: Integration step (s). Must be positive.

        Returns:
            The updated shaft position in degrees.

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        if dt <= 0:
            raise ValueError(f"dt must be positive, got {dt!r}")

        p = self.params

        di_dt = (voltage - p.R * self.current - p.K_b * self.omega) / p.L
        self.current += di_dt * dt

        torque = p.K_m * self.current
        friction = p.K_f * self.omega
        dw_dt = (torque - friction) / p.J
        self.omega += dw_dt * dt

        self.position_rad += self.omega * dt

        return self.get_position_deg()

    def get_position_deg(self) -> float:
        """Return the shaft position in degrees."""
        return math.degrees(self.position_rad)

    def get_velocity_deg_per_sec(self) -> float:
        """Return the angular velocity in degrees per second."""
        return math.degrees(self.omega)

    def get_current(self) -> float:
        """Return the armature current in amperes."""
        return self.current

    def reset(self, position_deg: float | None = None) -> None:
        """Return the motor to rest.

        Args:
            position_deg: Position to reset to. Defaults to the position the
                model was constructed with.
        """
        target = self._initial_position_deg if position_deg is None else position_deg
        self.position_rad = math.radians(target)
        self.omega = 0.0
        self.current = 0.0
