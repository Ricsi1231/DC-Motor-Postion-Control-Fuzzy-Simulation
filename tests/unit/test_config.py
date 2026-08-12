"""Unit tests for the configuration dataclasses."""

from __future__ import annotations

import dataclasses

import pytest

from dc_motor_sim import (
    DEFAULT_ENCODER,
    DEFAULT_MOTOR,
    DEFAULT_PID,
    DEFAULT_SIM,
    EncoderParams,
    FuzzyParams,
    MotorParams,
    PIDParams,
    SimParams,
)

ALL_PARAMS = (MotorParams, EncoderParams, SimParams, FuzzyParams, PIDParams)


class TestImmutability:
    @pytest.mark.parametrize("cls", ALL_PARAMS, ids=lambda c: c.__name__)
    def test_instances_are_frozen(self, cls: type) -> None:
        instance = cls()
        field = dataclasses.fields(instance)[0].name

        with pytest.raises(dataclasses.FrozenInstanceError):
            setattr(instance, field, 1.0)


class TestDocumentedDefaults:
    def test_motor_constants(self) -> None:
        assert DEFAULT_MOTOR.J == pytest.approx(3.2e-6)
        assert DEFAULT_MOTOR.K_f == pytest.approx(3.5e-6)
        assert DEFAULT_MOTOR.K_m == pytest.approx(0.03)
        assert DEFAULT_MOTOR.K_b == pytest.approx(0.03)
        assert DEFAULT_MOTOR.R == pytest.approx(4.0)
        assert DEFAULT_MOTOR.L == pytest.approx(0.001)

    def test_encoder_settings(self) -> None:
        assert DEFAULT_ENCODER.ppr == 1000
        assert DEFAULT_ENCODER.noise_std == pytest.approx(0.1)
        assert DEFAULT_ENCODER.degrees_per_count == pytest.approx(0.36)

    def test_simulation_settings(self) -> None:
        assert DEFAULT_SIM.dt == pytest.approx(0.001)
        assert DEFAULT_SIM.substeps == 10
        assert DEFAULT_SIM.max_steps == 300
        assert DEFAULT_SIM.voltage_scale == pytest.approx(0.082)
        assert DEFAULT_SIM.substep_dt == pytest.approx(0.0001)

    def test_pid_gains(self) -> None:
        assert (DEFAULT_PID.kp, DEFAULT_PID.ki, DEFAULT_PID.kd) == (2.0, 0.5, 0.1)
        assert DEFAULT_PID.output_limits == (-100.0, 100.0)


class TestDerivedProperties:
    def test_electrical_time_constant(self) -> None:
        assert DEFAULT_MOTOR.electrical_time_constant == pytest.approx(0.00025)

    def test_steady_state_speed_per_volt(self) -> None:
        expected = 0.03 / (3.5e-6 * 4.0 + 0.03 * 0.03)
        assert DEFAULT_MOTOR.steady_state_speed_per_volt == pytest.approx(expected)

    @pytest.mark.parametrize(
        ("position", "valid"),
        [(-180.0, True), (0.0, True), (180.0, True), (-180.1, False), (181.0, False)],
    )
    def test_position_bounds(self, position: float, valid: bool) -> None:
        assert DEFAULT_SIM.is_position_valid(position) is valid


class TestValidation:
    @pytest.mark.parametrize("field", ["J", "K_m", "R", "L"])
    def test_motor_rejects_non_positive_constants(self, field: str) -> None:
        with pytest.raises(ValueError, match=f"{field} must be positive"):
            MotorParams(**{field: 0.0})

    @pytest.mark.parametrize("field", ["K_f", "K_b"])
    def test_motor_rejects_negative_coefficients(self, field: str) -> None:
        with pytest.raises(ValueError, match=f"{field} must be non-negative"):
            MotorParams(**{field: -1.0})

    def test_encoder_rejects_non_positive_ppr(self) -> None:
        with pytest.raises(ValueError, match="ppr must be positive"):
            EncoderParams(ppr=0)

    def test_encoder_rejects_negative_noise(self) -> None:
        with pytest.raises(ValueError, match="noise_std must be non-negative"):
            EncoderParams(noise_std=-0.1)

    @pytest.mark.parametrize("field", ["dt", "voltage_scale", "convergence_position"])
    def test_sim_rejects_non_positive_floats(self, field: str) -> None:
        with pytest.raises(ValueError, match=f"{field} must be positive"):
            SimParams(**{field: 0.0})

    @pytest.mark.parametrize("field", ["substeps", "max_steps", "display_interval"])
    def test_sim_rejects_non_positive_counts(self, field: str) -> None:
        with pytest.raises(ValueError, match=f"{field} must be positive"):
            SimParams(**{field: 0})

    @pytest.mark.parametrize("field", ["delta_error_limits", "position_limits"])
    def test_sim_rejects_inverted_limits(self, field: str) -> None:
        with pytest.raises(ValueError, match=f"{field} must be"):
            SimParams(**{field: (10.0, -10.0)})

    def test_fuzzy_rejects_inverted_ranges(self) -> None:
        with pytest.raises(ValueError, match="error_range must be"):
            FuzzyParams(error_range=(100.0, -100.0))

    def test_pid_rejects_inverted_output_limits(self) -> None:
        with pytest.raises(ValueError, match="output_limits must be"):
            PIDParams(output_limits=(100.0, -100.0))


class TestReplacement:
    def test_replace_produces_a_validated_copy(self) -> None:
        tightened = dataclasses.replace(DEFAULT_SIM, max_steps=50)

        assert tightened.max_steps == 50
        assert DEFAULT_SIM.max_steps == 300

    def test_replace_still_validates(self) -> None:
        with pytest.raises(ValueError, match="max_steps must be positive"):
            dataclasses.replace(DEFAULT_SIM, max_steps=-1)
