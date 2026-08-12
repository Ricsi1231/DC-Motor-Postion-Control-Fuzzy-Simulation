"""Unit tests for the PID position controller."""

from __future__ import annotations

import pytest

from dc_motor_sim import PIDMotorController, PIDParams, PositionController


class TestProtocol:
    def test_satisfies_the_position_controller_protocol(
        self, pid_controller: PIDMotorController
    ) -> None:
        assert isinstance(pid_controller, PositionController)
        assert pid_controller.name == "PID"


class TestGains:
    def test_proportional_only_output_is_exactly_kp_times_error(self) -> None:
        controller = PIDMotorController(PIDParams(kp=2.0, ki=0.0, kd=0.0))
        controller.set_target(50.0)

        assert controller.compute(20.0, dt=0.001) == pytest.approx(2.0 * 30.0)

    def test_integral_accumulates_over_repeated_steps(self) -> None:
        controller = PIDMotorController(PIDParams(kp=0.0, ki=1.0, kd=0.0))
        controller.set_target(10.0)

        first = controller.compute(0.0, dt=0.1)
        second = controller.compute(0.0, dt=0.1)

        assert first == pytest.approx(1.0)
        assert second == pytest.approx(2.0)

    def test_derivative_responds_to_a_change_in_measurement(self) -> None:
        controller = PIDMotorController(PIDParams(kp=0.0, ki=0.0, kd=1.0))
        controller.set_target(0.0)

        controller.compute(0.0, dt=0.1)
        moving = controller.compute(5.0, dt=0.1)

        assert moving < 0  # measurement rising toward the setpoint opposes the output

    def test_set_tunings_updates_the_gains(self, pid_controller: PIDMotorController) -> None:
        pid_controller.set_tunings(1.0, 2.0, 3.0)

        assert pid_controller.tunings == (1.0, 2.0, 3.0)

    def test_default_gains_are_the_documented_ones(self) -> None:
        assert PIDMotorController().tunings == (2.0, 0.5, 0.1)


class TestSaturation:
    @pytest.mark.parametrize(("measured", "target"), [(-180.0, 180.0), (180.0, -180.0)])
    def test_output_respects_the_configured_limits(
        self, pid_controller: PIDMotorController, measured: float, target: float
    ) -> None:
        low, high = pid_controller.params.output_limits
        pid_controller.set_target(target)

        for _ in range(20):
            output = pid_controller.compute(measured, dt=0.01)
            assert low <= output <= high

    def test_extreme_error_saturates_the_output(self, pid_controller: PIDMotorController) -> None:
        _, high = pid_controller.params.output_limits
        pid_controller.set_target(180.0)

        assert pid_controller.compute(-180.0, dt=0.001) == pytest.approx(high)


class TestComponents:
    def test_components_sum_to_the_output(self) -> None:
        controller = PIDMotorController(PIDParams(kp=1.0, ki=1.0, kd=0.0))
        controller.set_target(10.0)

        output = controller.compute(4.0, dt=0.1)
        p, i, d = controller.get_components()

        assert p + i + d == pytest.approx(output)

    def test_components_start_at_zero(self, pid_controller: PIDMotorController) -> None:
        assert pid_controller.get_components() == (0.0, 0.0, 0.0)


class TestReset:
    def test_reset_clears_the_integral(self) -> None:
        controller = PIDMotorController(PIDParams(kp=0.0, ki=1.0, kd=0.0))
        controller.set_target(10.0)
        controller.compute(0.0, dt=0.1)

        controller.reset()

        assert controller.compute(0.0, dt=0.1) == pytest.approx(1.0)


class TestTargeting:
    def test_set_target_moves_the_setpoint(self, pid_controller: PIDMotorController) -> None:
        pid_controller.set_target(33.0)

        assert pid_controller.pid.setpoint == pytest.approx(33.0)

    def test_caller_owns_the_timing(self, pid_controller: PIDMotorController) -> None:
        """dt is supplied by the simulation, not measured from the wall clock."""
        assert pid_controller.pid.sample_time is None
