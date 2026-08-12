"""Validated configuration for the DC motor simulation.

Every tunable value lives here as a frozen dataclass field. Instances are
immutable and validate themselves on construction, so an invalid configuration
fails fast at the boundary instead of producing silently wrong physics.

The module-level ``DEFAULT_*`` singletons are the values the simulation used
before this configuration layer existed; they remain the defaults everywhere.
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = [
    "DEFAULT_ENCODER",
    "DEFAULT_FUZZY",
    "DEFAULT_MOTOR",
    "DEFAULT_PID",
    "DEFAULT_SIM",
    "EncoderParams",
    "FuzzyParams",
    "MotorParams",
    "PIDParams",
    "SimParams",
]

Trapezoid = tuple[float, float, float, float]
Triangle = tuple[float, float, float]
Limits = tuple[float, float]


def _require_positive(name: str, value: float) -> None:
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {value!r}")


def _require_ordered(name: str, limits: Limits) -> None:
    low, high = limits
    if low >= high:
        raise ValueError(f"{name} must be (low, high) with low < high, got {limits!r}")


@dataclass(frozen=True, slots=True)
class MotorParams:
    """Physical constants of the armature-controlled DC motor.

    Electrical:  ``L·di/dt + R·i = V - K_b·ω``
    Mechanical:  ``J·dω/dt = K_m·i - K_f·ω``
    Kinematic:   ``dθ/dt = ω``
    """

    J: float = 3.2e-6
    """Moment of inertia (kg·m²)."""
    K_f: float = 3.5e-6
    """Viscous friction coefficient (N·m·s/rad)."""
    K_m: float = 0.03
    """Torque constant (N·m/A)."""
    K_b: float = 0.03
    """Back-EMF constant (V·s/rad). Numerically equal to ``K_m`` in SI units."""
    R: float = 4.0
    """Armature resistance (Ω)."""
    L: float = 0.001
    """Armature inductance (H)."""

    def __post_init__(self) -> None:
        for name in ("J", "K_m", "R", "L"):
            _require_positive(name, getattr(self, name))
        if self.K_f < 0:
            raise ValueError(f"K_f must be non-negative, got {self.K_f!r}")
        if self.K_b < 0:
            raise ValueError(f"K_b must be non-negative, got {self.K_b!r}")

    @property
    def electrical_time_constant(self) -> float:
        """``L/R`` (s). An explicit-Euler step must stay well below this."""
        return self.L / self.R

    @property
    def steady_state_speed_per_volt(self) -> float:
        """Steady-state ``ω/V`` (rad/s per volt): ``K_m / (K_f·R + K_m·K_b)``."""
        return self.K_m / (self.K_f * self.R + self.K_m * self.K_b)


@dataclass(frozen=True, slots=True)
class EncoderParams:
    """Rotary encoder resolution and noise characteristics."""

    ppr: int = 1000
    """Pulses per revolution."""
    noise_std: float = 0.1
    """Standard deviation of additive Gaussian measurement noise (degrees)."""

    def __post_init__(self) -> None:
        if self.ppr <= 0:
            raise ValueError(f"ppr must be positive, got {self.ppr!r}")
        if self.noise_std < 0:
            raise ValueError(f"noise_std must be non-negative, got {self.noise_std!r}")

    @property
    def degrees_per_count(self) -> float:
        """Angular resolution of one encoder count (degrees)."""
        return 360.0 / self.ppr


@dataclass(frozen=True, slots=True)
class SimParams:
    """Control-loop timing, integration, and termination settings."""

    dt: float = 0.001
    """Control period (s) — the loop runs at 1 kHz."""
    substeps: int = 10
    """Motor integration substeps per control period.

    Explicit Euler is only stable while the integration step stays below the
    electrical time constant ``L/R`` (250 µs here). ``dt/substeps`` = 100 µs
    satisfies that; ``dt`` alone (1 ms) does not and diverges.
    """
    max_steps: int = 300
    """Maximum control steps before the run gives up."""
    voltage_scale: float = 0.082
    """Control signal to applied-voltage conversion (V per control unit)."""
    convergence_position: float = 0.5
    """Position error below which the run is considered converged (degrees)."""
    convergence_delta: float = 0.5
    """Error change-rate below which the run is considered converged (degrees)."""
    delta_error_limits: Limits = (-50.0, 50.0)
    """Clamp applied to the error derivative before fuzzy inference (degrees)."""
    position_limits: Limits = (-180.0, 180.0)
    """Valid range for start and target positions (degrees)."""
    display_interval: int = 20
    """Emit a progress log line every N steps."""

    def __post_init__(self) -> None:
        _require_positive("dt", self.dt)
        _require_positive("voltage_scale", self.voltage_scale)
        _require_positive("convergence_position", self.convergence_position)
        _require_positive("convergence_delta", self.convergence_delta)
        for name in ("substeps", "max_steps", "display_interval"):
            value = getattr(self, name)
            if value <= 0:
                raise ValueError(f"{name} must be positive, got {value!r}")
        _require_ordered("delta_error_limits", self.delta_error_limits)
        _require_ordered("position_limits", self.position_limits)

    @property
    def substep_dt(self) -> float:
        """Integration step used inside the motor model (s)."""
        return self.dt / self.substeps

    def is_position_valid(self, position_deg: float) -> bool:
        """Return whether ``position_deg`` lies within :attr:`position_limits`."""
        low, high = self.position_limits
        return low <= position_deg <= high


@dataclass(frozen=True, slots=True)
class FuzzyParams:
    """Universes, membership-function breakpoints, and integral gain.

    Universe maxima are exclusive because the universes are built with
    :func:`numpy.arange`, hence the ``+1`` upper bounds.
    """

    ki: float = 0.5
    """Integral gain added on top of the fuzzy inference output."""
    integral_limits: Limits = (-300.0, 300.0)
    """Anti-windup clamp on the accumulated error."""

    error_range: Limits = (-180.0, 181.0)
    delta_error_range: Limits = (-50.0, 51.0)
    control_range: Limits = (-100.0, 101.0)

    error_negative: Trapezoid = (-180.0, -180.0, -30.0, -5.0)
    error_zero: Triangle = (-8.0, 0.0, 8.0)
    error_positive: Trapezoid = (5.0, 30.0, 180.0, 180.0)

    delta_negative: Trapezoid = (-50.0, -50.0, -6.0, -1.0)
    delta_zero: Triangle = (-2.0, 0.0, 2.0)
    delta_positive: Trapezoid = (1.0, 6.0, 50.0, 50.0)

    control_negative: Trapezoid = (-100.0, -100.0, -35.0, -10.0)
    control_zero: Triangle = (-15.0, 0.0, 15.0)
    control_positive: Trapezoid = (10.0, 35.0, 100.0, 100.0)

    def __post_init__(self) -> None:
        for name in ("error_range", "delta_error_range", "control_range", "integral_limits"):
            _require_ordered(name, getattr(self, name))


@dataclass(frozen=True, slots=True)
class PIDParams:
    """Gains and saturation limits for the PID controller."""

    kp: float = 2.0
    ki: float = 0.5
    kd: float = 0.1
    output_limits: Limits = (-100.0, 100.0)

    def __post_init__(self) -> None:
        _require_ordered("output_limits", self.output_limits)


DEFAULT_MOTOR = MotorParams()
DEFAULT_ENCODER = EncoderParams()
DEFAULT_SIM = SimParams()
DEFAULT_FUZZY = FuzzyParams()
DEFAULT_PID = PIDParams()
