"""Unit tests for the DC motor physics model."""

from __future__ import annotations

import math

import pytest

from dc_motor_sim import DCMotorModel, MotorParams, SimParams


def _run(motor: DCMotorModel, voltage: float, dt: float, steps: int) -> None:
    for _ in range(steps):
        motor.step(voltage, dt)


class TestInitialState:
    def test_starts_at_rest(self) -> None:
        motor = DCMotorModel()

        assert motor.get_position_deg() == 0.0
        assert motor.get_velocity_deg_per_sec() == 0.0
        assert motor.get_current() == 0.0

    @pytest.mark.parametrize("position", [-180.0, -45.5, 0.0, 90.0, 180.0])
    def test_initial_position_round_trips(self, position: float) -> None:
        motor = DCMotorModel(initial_position_deg=position)

        assert motor.get_position_deg() == pytest.approx(position)

    def test_zero_voltage_keeps_motor_at_rest(self) -> None:
        motor = DCMotorModel(initial_position_deg=30.0)

        _run(motor, voltage=0.0, dt=1e-4, steps=1000)

        assert motor.get_position_deg() == pytest.approx(30.0)
        assert motor.get_velocity_deg_per_sec() == pytest.approx(0.0)
        assert motor.get_current() == pytest.approx(0.0)


class TestDynamics:
    @pytest.mark.parametrize("voltage", [0.5, 1.0, 2.0])
    def test_reaches_analytic_steady_state_speed(self, voltage: float) -> None:
        """Constant voltage drives omega to K_m*V / (K_f*R + K_m*K_b)."""
        params = MotorParams()
        motor = DCMotorModel(params=params)

        _run(motor, voltage, dt=1e-5, steps=200_000)

        expected = params.steady_state_speed_per_volt * voltage
        assert motor.omega == pytest.approx(expected, rel=1e-3)

    def test_steady_state_current_balances_friction(self) -> None:
        params = MotorParams()
        motor = DCMotorModel(params=params)

        _run(motor, voltage=1.0, dt=1e-5, steps=200_000)

        # At equilibrium the motor torque exactly cancels viscous friction.
        assert params.K_m * motor.current == pytest.approx(params.K_f * motor.omega, rel=1e-3)

    def test_voltage_sign_determines_direction(self) -> None:
        forward, reverse = DCMotorModel(), DCMotorModel()

        _run(forward, voltage=1.0, dt=1e-4, steps=100)
        _run(reverse, voltage=-1.0, dt=1e-4, steps=100)

        assert forward.get_velocity_deg_per_sec() > 0
        assert reverse.get_velocity_deg_per_sec() < 0
        assert forward.get_position_deg() == pytest.approx(-reverse.get_position_deg())

    def test_position_is_the_integral_of_velocity(self) -> None:
        motor = DCMotorModel()
        dt = 1e-4

        motor.step(1.0, dt)
        expected = math.degrees(motor.omega) * dt

        assert motor.get_position_deg() == pytest.approx(expected)


class TestEulerStability:
    """Explicit Euler is only stable below the electrical time constant L/R.

    This is exactly why the simulation splits each 1 ms control period into
    ``SimParams.substeps`` sub-intervals. If someone "simplifies" that away,
    these tests fail.
    """

    def test_substep_is_below_the_electrical_time_constant(self) -> None:
        params, sim = MotorParams(), SimParams()

        assert sim.substep_dt < params.electrical_time_constant
        assert sim.dt > params.electrical_time_constant

    def test_stable_at_the_substep_size(self) -> None:
        motor = DCMotorModel()

        _run(motor, voltage=1.0, dt=SimParams().substep_dt, steps=1000)

        assert math.isfinite(motor.get_position_deg())
        assert abs(motor.get_current()) < 10.0

    def test_diverges_at_the_full_control_period(self) -> None:
        motor = DCMotorModel()

        _run(motor, voltage=1.0, dt=SimParams().dt, steps=200)

        assert not math.isfinite(motor.get_current()) or abs(motor.get_current()) > 1e6


class TestValidation:
    @pytest.mark.parametrize("dt", [0.0, -1e-4])
    def test_rejects_non_positive_dt(self, dt: float) -> None:
        motor = DCMotorModel()

        with pytest.raises(ValueError, match="dt must be positive"):
            motor.step(1.0, dt)


class TestReset:
    def test_reset_returns_to_construction_position(self) -> None:
        motor = DCMotorModel(initial_position_deg=25.0)
        _run(motor, voltage=1.0, dt=1e-4, steps=100)

        motor.reset()

        assert motor.get_position_deg() == pytest.approx(25.0)
        assert motor.get_velocity_deg_per_sec() == 0.0
        assert motor.get_current() == 0.0

    def test_reset_accepts_an_explicit_position(self) -> None:
        motor = DCMotorModel(initial_position_deg=25.0)

        motor.reset(position_deg=-60.0)

        assert motor.get_position_deg() == pytest.approx(-60.0)
