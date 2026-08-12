"""Rotary encoder model for DC motor position feedback.

Simulates a quadrature-style incremental encoder:

1. the true shaft angle is quantized to the nearest discrete count,
2. optional Gaussian measurement noise is added,
3. the result is reported in degrees.

The noise source is an injectable :class:`numpy.random.Generator`, so any
simulation using this sensor is exactly reproducible from a seed.
"""

from __future__ import annotations

import numpy as np

from dc_motor_sim.config import DEFAULT_ENCODER, EncoderParams

__all__ = ["RotaryEncoder"]


class RotaryEncoder:
    """Incremental rotary encoder with quantization and measurement noise.

    Args:
        params: Resolution and noise settings. Defaults to
            :data:`~dc_motor_sim.config.DEFAULT_ENCODER`.
        rng: Random generator used for the noise term. Defaults to a fresh
            unseeded :func:`numpy.random.default_rng`. Pass a seeded generator
            for reproducible runs.
    """

    def __init__(
        self,
        params: EncoderParams = DEFAULT_ENCODER,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.params = params
        self._rng = np.random.default_rng() if rng is None else rng
        self.count = 0
        self.last_reading_deg = 0.0
        # Tracked separately from last_reading_deg, which starts at a 0.0
        # sentinel that must never be mistaken for a real sample.
        self._has_reading = False
        self._previous_reading_deg: float | None = None

    @property
    def ppr(self) -> int:
        """Encoder resolution in pulses per revolution."""
        return self.params.ppr

    @property
    def noise_std(self) -> float:
        """Standard deviation of the additive measurement noise (degrees)."""
        return self.params.noise_std

    def read_position(self, actual_position_deg: float) -> float:
        """Sample the encoder at the given true shaft angle.

        Args:
            actual_position_deg: True shaft angle in degrees.

        Returns:
            The measured angle in degrees, quantized to the encoder grid and
            perturbed by noise when :attr:`noise_std` is greater than zero.
        """
        resolution = self.params.degrees_per_count
        counts = round(actual_position_deg / resolution)
        measured = counts * resolution

        if self.params.noise_std > 0:
            measured += float(self._rng.normal(0.0, self.params.noise_std))

        self.count = counts
        if self._has_reading:
            self._previous_reading_deg = self.last_reading_deg
        self.last_reading_deg = measured
        self._has_reading = True

        return measured

    def get_count(self) -> int:
        """Return the encoder count from the most recent reading."""
        return self.count

    def get_resolution(self) -> float:
        """Return the angular resolution of one count (degrees)."""
        return self.params.degrees_per_count

    def get_velocity(self, dt: float) -> float:
        """Estimate angular velocity by differencing the last two readings.

        Args:
            dt: Time elapsed between the two readings (s).

        Returns:
            Estimated velocity in degrees per second, or ``0.0`` when fewer
            than two readings have been taken.

        Raises:
            ValueError: If ``dt`` is not positive.
        """
        if dt <= 0:
            raise ValueError(f"dt must be positive, got {dt!r}")
        if self._previous_reading_deg is None:
            return 0.0
        return (self.last_reading_deg - self._previous_reading_deg) / dt

    def reset(self) -> None:
        """Clear the count and reading history."""
        self.count = 0
        self.last_reading_deg = 0.0
        self._has_reading = False
        self._previous_reading_deg = None
