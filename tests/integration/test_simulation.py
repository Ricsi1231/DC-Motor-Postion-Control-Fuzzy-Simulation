"""Integration tests for the closed-loop simulation runner."""

from __future__ import annotations

import dataclasses
import logging

import numpy as np
import pytest

from dc_motor_sim import (
    DEFAULT_SIM,
    EncoderParams,
    FuzzyMotorController,
    PIDMotorController,
    SimParams,
    SimulationResult,
    run_simulation,
)

NOISELESS = EncoderParams(noise_std=0.0)


def make_controller(kind: str):
    return FuzzyMotorController() if kind == "fuzzy" else PIDMotorController()


@pytest.fixture(params=["fuzzy", "pid"])
def controller_kind(request: pytest.FixtureRequest) -> str:
    return request.param


class TestConvergence:
    def test_reaches_the_target(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, encoder_params=NOISELESS
        )

        assert result.converged
        assert result.steps < DEFAULT_SIM.max_steps
        assert abs(result.final_error) < 1.0

    @pytest.mark.parametrize(("start", "target"), [(0.0, 90.0), (45.0, -45.0), (-180.0, 0.0)])
    def test_converges_across_a_range_of_setpoints(
        self, controller_kind: str, start: float, target: float
    ) -> None:
        result = run_simulation(
            make_controller(controller_kind), start, target, encoder_params=NOISELESS
        )

        assert result.converged
        assert abs(result.final_error) < 2.0

    def test_already_at_target_converges_immediately(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), 30.0, 30.0, encoder_params=NOISELESS
        )

        assert result.converged
        assert result.steps == 1

    def test_reports_failure_when_out_of_steps(self, controller_kind: str) -> None:
        params = dataclasses.replace(DEFAULT_SIM, max_steps=3)

        result = run_simulation(
            make_controller(controller_kind),
            -180.0,
            180.0,
            sim_params=params,
            encoder_params=NOISELESS,
        )

        assert not result.converged
        assert result.steps == 3


class TestResultShape:
    def test_every_series_has_one_sample_per_step_plus_the_initial_state(
        self, controller_kind: str
    ) -> None:
        result = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, encoder_params=NOISELESS
        )
        expected = result.steps + 1

        for name in (
            "time",
            "actual_position",
            "measured_position",
            "target",
            "error",
            "control",
            "voltage",
            "current",
            "velocity",
        ):
            assert getattr(result, name).shape == (expected,), name

    def test_time_advances_by_one_control_period_per_step(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, encoder_params=NOISELESS
        )

        np.testing.assert_allclose(np.diff(result.time), DEFAULT_SIM.dt, rtol=1e-9)

    def test_target_series_is_constant(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, encoder_params=NOISELESS
        )

        assert np.all(result.target == 45.0)

    def test_voltage_is_the_scaled_control_signal(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, encoder_params=NOISELESS
        )

        np.testing.assert_allclose(
            result.voltage, result.control * DEFAULT_SIM.voltage_scale, rtol=1e-12
        )

    def test_rejects_inconsistent_series_lengths(self) -> None:
        with pytest.raises(ValueError, match="expected"):
            SimulationResult(
                time=np.zeros(3),
                actual_position=np.zeros(3),
                measured_position=np.zeros(3),
                target=np.zeros(3),
                error=np.zeros(3),
                control=np.zeros(3),
                voltage=np.zeros(3),
                current=np.zeros(3),
                velocity=np.zeros(2),  # one short
                steps=2,
                converged=True,
                controller_name="Fuzzy",
                start_deg=0.0,
                target_deg=1.0,
            )


class TestDeterminism:
    def test_same_seed_reproduces_the_run(self, controller_kind: str) -> None:
        kwargs = {"encoder_params": EncoderParams(noise_std=0.1)}

        first = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, rng=np.random.default_rng(99), **kwargs
        )
        second = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, rng=np.random.default_rng(99), **kwargs
        )

        assert first.steps == second.steps
        np.testing.assert_array_equal(first.actual_position, second.actual_position)
        np.testing.assert_array_equal(first.measured_position, second.measured_position)

    def test_different_seeds_diverge(self, controller_kind: str) -> None:
        kwargs = {"encoder_params": EncoderParams(noise_std=0.5)}

        first = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, rng=np.random.default_rng(1), **kwargs
        )
        second = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, rng=np.random.default_rng(2), **kwargs
        )

        assert not np.array_equal(
            first.measured_position[: min(first.steps, second.steps)],
            second.measured_position[: min(first.steps, second.steps)],
        )


class TestControllerHandling:
    def test_runner_resets_a_reused_controller(self) -> None:
        controller = FuzzyMotorController()

        first = run_simulation(controller, -90.0, 45.0, encoder_params=NOISELESS)
        second = run_simulation(controller, -90.0, 45.0, encoder_params=NOISELESS)

        assert first.steps == second.steps
        np.testing.assert_allclose(first.actual_position, second.actual_position, rtol=1e-12)

    def test_result_records_the_controller_name(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), 0.0, 10.0, encoder_params=NOISELESS
        )

        assert result.controller_name == ("Fuzzy" if controller_kind == "fuzzy" else "PID")


class TestMetrics:
    def test_summary_reports_the_run(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, encoder_params=NOISELESS
        )
        summary = result.summary()

        assert summary["start_deg"] == -90.0
        assert summary["target_deg"] == 45.0
        assert summary["steps"] == result.steps
        assert summary["converged"] is result.converged

    def test_duration_matches_steps_times_period(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, encoder_params=NOISELESS
        )

        assert result.duration == pytest.approx(result.steps * DEFAULT_SIM.dt)

    def test_overshoot_is_zero_for_a_null_move(self) -> None:
        result = run_simulation(PIDMotorController(), 30.0, 30.0, encoder_params=NOISELESS)

        assert result.overshoot == 0.0

    def test_overshoot_is_non_negative(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, encoder_params=NOISELESS
        )

        assert result.overshoot >= 0.0

    def test_repr_is_informative(self, controller_kind: str) -> None:
        result = run_simulation(
            make_controller(controller_kind), -90.0, 45.0, encoder_params=NOISELESS
        )

        assert "SimulationResult" in repr(result)
        assert result.controller_name in repr(result)


class TestLogging:
    def test_progress_is_logged_not_printed(
        self, caplog: pytest.LogCaptureFixture, capsys: pytest.CaptureFixture[str]
    ) -> None:
        with caplog.at_level(logging.INFO, logger="dc_motor_sim.simulation.runner"):
            run_simulation(PIDMotorController(), -90.0, 45.0, encoder_params=NOISELESS)

        assert any("Starting closed-loop simulation" in r.message for r in caplog.records)
        assert capsys.readouterr().out == ""

    def test_failure_to_converge_is_logged_as_a_warning(
        self, caplog: pytest.LogCaptureFixture
    ) -> None:
        params = SimParams(max_steps=2)

        with caplog.at_level(logging.WARNING, logger="dc_motor_sim.simulation.runner"):
            run_simulation(
                PIDMotorController(),
                -180.0,
                180.0,
                sim_params=params,
                encoder_params=NOISELESS,
            )

        assert any(r.levelno == logging.WARNING for r in caplog.records)
