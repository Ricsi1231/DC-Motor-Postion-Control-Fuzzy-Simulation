"""Fuzzy logic position controller with an added integral term.

A Mamdani controller over a 3x3 rule base. Error and its derivative are each
partitioned into Negative / Zero / Positive sets; defuzzification yields a
control signal in ``[-100, 100]``, to which a clamped integral term is added to
remove the steady-state offset that pure fuzzy inference leaves behind.
"""

from __future__ import annotations

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

from dc_motor_sim.config import DEFAULT_FUZZY, DEFAULT_SIM, FuzzyParams, Limits, SimParams

__all__ = ["FuzzyMotorController"]


def _clamp(value: float, limits: Limits) -> float:
    low, high = limits
    return min(max(value, low), high)


class FuzzyMotorController:
    """Fuzzy position controller.

    Implements :class:`~dc_motor_sim.control.base.PositionController`.
    Constructing this class builds a :class:`skfuzzy.control.ControlSystem`,
    which is comparatively expensive — reuse one instance where possible.

    Args:
        params: Membership functions, universes, and integral settings.
        sim_params: Supplies the derivative clamp applied before inference.
    """

    name = "Fuzzy"

    def __init__(
        self,
        params: FuzzyParams = DEFAULT_FUZZY,
        sim_params: SimParams = DEFAULT_SIM,
    ) -> None:
        self.params = params
        self.sim_params = sim_params

        self.integral = 0.0
        self._target_deg = 0.0
        self._previous_error: float | None = None

        self.error = ctrl.Antecedent(self._universe(params.error_range), "error")
        self.delta_error = ctrl.Antecedent(self._universe(params.delta_error_range), "delta_error")
        self.control = ctrl.Consequent(self._universe(params.control_range), "control")

        self.error["N"] = fuzz.trapmf(self.error.universe, list(params.error_negative))
        self.error["Z"] = fuzz.trimf(self.error.universe, list(params.error_zero))
        self.error["P"] = fuzz.trapmf(self.error.universe, list(params.error_positive))

        self.delta_error["N"] = fuzz.trapmf(self.delta_error.universe, list(params.delta_negative))
        self.delta_error["Z"] = fuzz.trimf(self.delta_error.universe, list(params.delta_zero))
        self.delta_error["P"] = fuzz.trapmf(self.delta_error.universe, list(params.delta_positive))

        self.control["N"] = fuzz.trapmf(self.control.universe, list(params.control_negative))
        self.control["Z"] = fuzz.trimf(self.control.universe, list(params.control_zero))
        self.control["P"] = fuzz.trapmf(self.control.universe, list(params.control_positive))

        self.control_system = ctrl.ControlSystem(self._build_rules())
        self.simulation = ctrl.ControlSystemSimulation(self.control_system)

    @staticmethod
    def _universe(bounds: Limits) -> np.ndarray:
        """Build a unit-spaced universe. The upper bound is exclusive."""
        low, high = bounds
        return np.arange(low, high, 1)

    def _build_rules(self) -> list[ctrl.Rule]:
        """The 3x3 rule base, in error-major order."""
        table = {
            ("N", "N"): "N",
            ("N", "Z"): "N",
            ("N", "P"): "Z",
            ("Z", "N"): "Z",
            ("Z", "Z"): "Z",
            ("Z", "P"): "Z",
            ("P", "N"): "Z",
            ("P", "Z"): "P",
            ("P", "P"): "P",
        }
        return [
            ctrl.Rule(self.error[e] & self.delta_error[d], self.control[out])
            for (e, d), out in table.items()
        ]

    def set_target(self, target_deg: float) -> None:
        """Set the desired shaft position in degrees."""
        self._target_deg = target_deg

    def compute(self, measured_deg: float, dt: float) -> float:
        """Return the control signal for the given encoder reading.

        Derives the error and its clamped derivative from the measurement,
        runs fuzzy inference, and adds the clamped integral term.
        """
        error = self._target_deg - measured_deg
        if self._previous_error is None:
            delta_error = 0.0
        else:
            delta_error = _clamp(error - self._previous_error, self.sim_params.delta_error_limits)
        self._previous_error = error

        self.integral = _clamp(self.integral + error * dt, self.params.integral_limits)

        return self.infer(error, delta_error) + self.params.ki * self.integral

    def infer(self, error: float, delta_error: float) -> float:
        """Run fuzzy inference alone, without the integral term.

        Exposed separately so the control surface can be plotted and so tests
        can assert the inference output stays within the control universe.
        """
        self.simulation.input["error"] = error
        self.simulation.input["delta_error"] = delta_error
        self.simulation.compute()
        return float(self.simulation.output["control"])

    def reset(self) -> None:
        """Clear the integral accumulator and error history."""
        self.integral = 0.0
        self._previous_error = None

    def get_membership_functions(
        self,
    ) -> tuple[ctrl.Antecedent, ctrl.Antecedent, ctrl.Consequent]:
        """Return the ``(error, delta_error, control)`` fuzzy variables."""
        return self.error, self.delta_error, self.control
