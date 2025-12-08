from simple_pid import PID
import numpy as np


class PIDMotorController:
    """
    PID controller for DC motor position control.

    Uses the simple-pid library for standard PID control.
    """

    def __init__(self, kp=2.0, ki=0.5, kd=0.1):
        """
        Initialize PID controller with tuning parameters.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.pid = PID(kp, ki, kd, setpoint=0)
        self.pid.output_limits = (-100, 100)
        self.pid.sample_time = None

    def set_target(self, target_position):
        """
        Set the target position (setpoint) for the PID controller.

        Args:
            target_position: Desired position in degrees
        """
        self.pid.setpoint = target_position

    def compute_control(self, measured_position, dt=None):
        """
        Compute control signal using PID algorithm.

        Args:
            measured_position: Current measured position in degrees
            dt: Time step (optional, for compatibility)

        Returns:
            Control signal value (-100 to 100)
        """
        control_signal = self.pid(measured_position, dt=dt)
        return control_signal

    def reset(self):
        """Reset the PID controller's integral term."""
        self.pid.reset()

    def get_components(self):
        """
        Get the individual PID components.

        Returns:
            Tuple of (proportional, integral, derivative) terms
        """
        return self.pid.components

    def set_tunings(self, kp, ki, kd):
        """
        Update PID tuning parameters.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.pid.tunings = (kp, ki, kd)
