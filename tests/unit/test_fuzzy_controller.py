"""Unit tests for the fuzzy logic position controller."""

from __future__ import annotations

import numpy as np
import pytest

from dc_motor_sim import FuzzyMotorController, FuzzyParams, PositionController


class TestProtocol:
    def test_satisfies_the_position_controller_protocol(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        assert isinstance(fresh_fuzzy_controller, PositionController)
        assert fresh_fuzzy_controller.name == "Fuzzy"


class TestInference:
    def test_output_stays_within_the_control_universe(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        low, high = fresh_fuzzy_controller.params.control_range

        for error in np.linspace(-180, 180, 13):
            for delta in np.linspace(-50, 50, 7):
                output = fresh_fuzzy_controller.infer(float(error), float(delta))
                assert low <= output <= high

    @pytest.mark.parametrize("error", [30.0, 90.0, 180.0])
    def test_positive_error_drives_positive_control(
        self, fresh_fuzzy_controller: FuzzyMotorController, error: float
    ) -> None:
        assert fresh_fuzzy_controller.infer(error, 0.0) > 0

    @pytest.mark.parametrize("error", [-30.0, -90.0, -180.0])
    def test_negative_error_drives_negative_control(
        self, fresh_fuzzy_controller: FuzzyMotorController, error: float
    ) -> None:
        assert fresh_fuzzy_controller.infer(error, 0.0) < 0

    def test_zero_error_gives_near_zero_control(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        assert fresh_fuzzy_controller.infer(0.0, 0.0) == pytest.approx(0.0, abs=1e-6)

    def test_inference_does_not_touch_the_integral(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        fresh_fuzzy_controller.infer(50.0, 0.0)

        assert fresh_fuzzy_controller.integral == 0.0


class TestComputeLoop:
    def test_first_step_sees_zero_derivative(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        """With no prior error there is nothing to difference against."""
        fresh_fuzzy_controller.set_target(45.0)

        control = fresh_fuzzy_controller.compute(measured_deg=-90.0, dt=0.001)
        expected = fresh_fuzzy_controller.infer(135.0, 0.0)
        expected += fresh_fuzzy_controller.params.ki * fresh_fuzzy_controller.integral

        assert control == pytest.approx(expected)

    def test_integral_accumulates_error_over_time(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        fresh_fuzzy_controller.set_target(10.0)

        fresh_fuzzy_controller.compute(0.0, dt=0.001)
        after_one = fresh_fuzzy_controller.integral
        fresh_fuzzy_controller.compute(0.0, dt=0.001)

        assert after_one == pytest.approx(0.01)
        assert fresh_fuzzy_controller.integral == pytest.approx(0.02)

    def test_integral_saturates_at_the_configured_limit(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        _, upper = fresh_fuzzy_controller.params.integral_limits
        fresh_fuzzy_controller.set_target(180.0)

        for _ in range(3000):
            fresh_fuzzy_controller.compute(-180.0, dt=1.0)

        assert fresh_fuzzy_controller.integral == pytest.approx(upper)

    def test_integral_saturates_at_the_lower_limit(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        lower, _ = fresh_fuzzy_controller.params.integral_limits
        fresh_fuzzy_controller.set_target(-180.0)

        for _ in range(3000):
            fresh_fuzzy_controller.compute(180.0, dt=1.0)

        assert fresh_fuzzy_controller.integral == pytest.approx(lower)

    def test_derivative_is_clamped_before_inference(
        self, fresh_fuzzy_controller: FuzzyMotorController, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """A large error jump must not push delta_error outside its universe."""
        seen: list[tuple[float, float]] = []
        monkeypatch.setattr(
            fresh_fuzzy_controller,
            "infer",
            lambda error, delta: seen.append((error, delta)) or 0.0,
        )
        lower, upper = fresh_fuzzy_controller.sim_params.delta_error_limits

        fresh_fuzzy_controller.set_target(0.0)
        fresh_fuzzy_controller.compute(0.0, dt=0.001)
        # Unclamped this step would give delta_error = -170, far outside the
        # [-50, 50] universe the fuzzy sets are defined over.
        fresh_fuzzy_controller.compute(170.0, dt=0.001)

        assert seen[-1] == (-170.0, lower)
        assert lower <= seen[-1][1] <= upper

    def test_derivative_is_not_clamped_within_range(
        self, fresh_fuzzy_controller: FuzzyMotorController, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        seen: list[tuple[float, float]] = []
        monkeypatch.setattr(
            fresh_fuzzy_controller,
            "infer",
            lambda error, delta: seen.append((error, delta)) or 0.0,
        )

        fresh_fuzzy_controller.set_target(0.0)
        fresh_fuzzy_controller.compute(0.0, dt=0.001)
        fresh_fuzzy_controller.compute(-3.0, dt=0.001)

        assert seen[-1] == (3.0, 3.0)

    def test_reset_clears_integral_and_history(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        fresh_fuzzy_controller.set_target(90.0)
        fresh_fuzzy_controller.compute(0.0, dt=0.01)

        fresh_fuzzy_controller.reset()

        assert fresh_fuzzy_controller.integral == 0.0
        assert fresh_fuzzy_controller._previous_error is None


class TestStructure:
    def test_rule_base_has_nine_rules(self, fresh_fuzzy_controller: FuzzyMotorController) -> None:
        assert len(list(fresh_fuzzy_controller.control_system.rules)) == 9

    def test_each_variable_has_three_terms(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        for variable in fresh_fuzzy_controller.get_membership_functions():
            assert set(variable.terms) == {"N", "Z", "P"}

    def test_universes_match_the_configured_ranges(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        params = fresh_fuzzy_controller.params
        error, delta, control = fresh_fuzzy_controller.get_membership_functions()

        for variable, (low, high) in (
            (error, params.error_range),
            (delta, params.delta_error_range),
            (control, params.control_range),
        ):
            assert variable.universe.min() == pytest.approx(low)
            assert variable.universe.max() == pytest.approx(high - 1)

    def test_membership_breakpoints_match_the_configuration(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        params = fresh_fuzzy_controller.params
        error, delta, control = fresh_fuzzy_controller.get_membership_functions()

        # The peak of each triangular Zero set sits at its configured centre.
        for variable, triangle in (
            (error, params.error_zero),
            (delta, params.delta_zero),
            (control, params.control_zero),
        ):
            peak_index = int(np.argmax(variable["Z"].mf))
            assert variable.universe[peak_index] == pytest.approx(triangle[1])

    def test_default_parameters_are_the_documented_ones(self) -> None:
        params = FuzzyParams()

        assert params.ki == 0.5
        assert params.integral_limits == (-300.0, 300.0)
        assert params.error_negative == (-180.0, -180.0, -30.0, -5.0)
        assert params.control_positive == (10.0, 35.0, 100.0, 100.0)
