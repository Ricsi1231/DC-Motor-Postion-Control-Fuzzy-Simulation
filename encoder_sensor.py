"""
Encoder sensor model for DC motor position feedback.

Simulates a realistic rotary encoder with:
- Discrete resolution (pulses per revolution - PPR)
- Quantization error due to discrete counts
- Optional measurement noise
- Conversion between encoder counts and angular position
"""

import numpy as np


class RotaryEncoder:
    def __init__(self, pulses_per_revolution=1000, noise_std=0.0):
        """
        Initialize the rotary encoder sensor.

        Args:
            pulses_per_revolution: Encoder resolution (PPR)
            noise_std: Standard deviation of measurement noise in degrees
        """
        self.ppr = pulses_per_revolution
        self.noise_std = noise_std
        self.degrees_per_count = 360.0 / pulses_per_revolution
        self.count = 0
        self.previous_position_deg = 0.0

    def read_position(self, actual_position_deg):
        """
        Read encoder position from actual motor position.

        Simulates:
        1. Converting actual position to encoder counts (quantization)
        2. Adding measurement noise
        3. Converting back to degrees

        Args:
            actual_position_deg: Actual motor position in degrees

        Returns:
            Measured position in degrees (with quantization and noise)
        """
        counts = round(actual_position_deg / self.degrees_per_count)
        quantized_position = counts * self.degrees_per_count

        if self.noise_std > 0:
            noise = np.random.normal(0, self.noise_std)
            measured_position = quantized_position + noise
        else:
            measured_position = quantized_position

        self.count = counts
        self.previous_position_deg = measured_position

        return measured_position

    def get_count(self):
        """Get current encoder count."""
        return self.count

    def get_resolution(self):
        """Get encoder resolution in degrees per count."""
        return self.degrees_per_count

    def get_velocity(self, current_position_deg, dt):
        """
        Estimate angular velocity from position difference.

        Args:
            current_position_deg: Current measured position
            dt: Time step

        Returns:
            Estimated velocity in degrees per second
        """
        velocity = (current_position_deg - self.previous_position_deg) / dt
        return velocity

    def reset(self):
        """Reset encoder to zero position."""
        self.count = 0
        self.previous_position_deg = 0.0
