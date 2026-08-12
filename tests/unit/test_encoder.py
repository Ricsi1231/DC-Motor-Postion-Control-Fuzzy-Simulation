"""Unit tests for the rotary encoder sensor model."""

from __future__ import annotations

import numpy as np
import pytest

from dc_motor_sim import EncoderParams, RotaryEncoder


class TestQuantization:
    def test_reading_lands_on_the_encoder_grid(self, noiseless_encoder: RotaryEncoder) -> None:
        resolution = noiseless_encoder.get_resolution()

        for actual in np.linspace(-180.0, 180.0, 501):
            measured = noiseless_encoder.read_position(float(actual))
            assert measured / resolution == pytest.approx(round(measured / resolution))

    def test_quantization_error_is_at_most_half_a_count(
        self, noiseless_encoder: RotaryEncoder
    ) -> None:
        half_count = noiseless_encoder.get_resolution() / 2

        for actual in np.linspace(-180.0, 180.0, 1001):
            measured = noiseless_encoder.read_position(float(actual))
            assert abs(measured - actual) <= half_count + 1e-12

    def test_resolution_matches_pulses_per_revolution(self) -> None:
        assert RotaryEncoder().get_resolution() == pytest.approx(0.36)
        assert RotaryEncoder(EncoderParams(ppr=360)).get_resolution() == pytest.approx(1.0)

    @pytest.mark.parametrize("actual", [-90.0, -0.5, 0.0, 12.34, 179.9])
    def test_count_matches_the_rounded_position(
        self, noiseless_encoder: RotaryEncoder, actual: float
    ) -> None:
        noiseless_encoder.read_position(actual)

        assert noiseless_encoder.get_count() == round(actual / noiseless_encoder.get_resolution())


class TestNoise:
    def test_noise_is_reproducible_from_a_seed(self) -> None:
        first = RotaryEncoder(rng=np.random.default_rng(7))
        second = RotaryEncoder(rng=np.random.default_rng(7))

        readings_a = [first.read_position(x) for x in range(50)]
        readings_b = [second.read_position(x) for x in range(50)]

        assert readings_a == readings_b

    def test_different_seeds_give_different_noise(self) -> None:
        first = RotaryEncoder(rng=np.random.default_rng(1))
        second = RotaryEncoder(rng=np.random.default_rng(2))

        assert [first.read_position(x) for x in range(50)] != [
            second.read_position(x) for x in range(50)
        ]

    def test_noise_has_the_configured_distribution(self, rng: np.random.Generator) -> None:
        params = EncoderParams(noise_std=0.1)
        encoder = RotaryEncoder(params=params, rng=rng)

        # Sample at an exact grid point so quantization contributes nothing.
        exact = 10 * encoder.get_resolution()
        residuals = np.array([encoder.read_position(exact) - exact for _ in range(10_000)])

        assert residuals.mean() == pytest.approx(0.0, abs=0.01)
        assert residuals.std() == pytest.approx(params.noise_std, rel=0.05)

    def test_zero_noise_is_deterministic(self, noiseless_encoder: RotaryEncoder) -> None:
        readings = {noiseless_encoder.read_position(33.3) for _ in range(20)}

        assert len(readings) == 1


class TestVelocity:
    def test_returns_zero_before_two_readings(self, noiseless_encoder: RotaryEncoder) -> None:
        assert noiseless_encoder.get_velocity(dt=0.001) == 0.0

        noiseless_encoder.read_position(0.0)
        assert noiseless_encoder.get_velocity(dt=0.001) == 0.0

    def test_differences_the_last_two_readings(self, noiseless_encoder: RotaryEncoder) -> None:
        """Regression: velocity used to always be zero after a read.

        ``read_position`` overwrote the stored previous reading with the
        current one, so the difference was structurally always zero.
        """
        dt = 0.001
        first = noiseless_encoder.read_position(0.0)
        second = noiseless_encoder.read_position(3.6)

        assert noiseless_encoder.get_velocity(dt) == pytest.approx((second - first) / dt)
        assert noiseless_encoder.get_velocity(dt) != 0.0

    @pytest.mark.parametrize("dt", [0.0, -0.001])
    def test_rejects_non_positive_dt(self, noiseless_encoder: RotaryEncoder, dt: float) -> None:
        with pytest.raises(ValueError, match="dt must be positive"):
            noiseless_encoder.get_velocity(dt)


class TestReset:
    def test_reset_clears_count_and_history(self, noiseless_encoder: RotaryEncoder) -> None:
        noiseless_encoder.read_position(90.0)
        noiseless_encoder.read_position(120.0)

        noiseless_encoder.reset()

        assert noiseless_encoder.get_count() == 0
        assert noiseless_encoder.last_reading_deg == 0.0
        assert noiseless_encoder.get_velocity(dt=0.001) == 0.0


class TestConfiguration:
    def test_exposes_its_parameters(self) -> None:
        encoder = RotaryEncoder(EncoderParams(ppr=2048, noise_std=0.25))

        assert encoder.ppr == 2048
        assert encoder.noise_std == 0.25

    def test_defaults_to_an_unseeded_generator(self) -> None:
        assert isinstance(RotaryEncoder()._rng, np.random.Generator)
