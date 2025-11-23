"""
DC Motor physical model implementing realistic motor dynamics.

The model includes:
- Electrical dynamics: L*di/dt + R*i = v_app - K_b*ω
- Mechanical dynamics: J*dω/dt = K_m*i - K_f*ω
- Kinematic: dθ/dt = ω

Where:
- v_app: Applied voltage (control input)
- i: Armature current
- ω: Angular velocity
- θ: Angular position
"""

import numpy as np
from motor_parameters import J, K_f, K_m, K_b, R, L, DT

class DCMotorModel:
    def __init__(self, initial_position_deg=0.0):
        """
        Initialize the DC motor model.

        Args:
            initial_position_deg: Initial position in degrees
        """
        self.position_rad = np.deg2rad(initial_position_deg)
        self.omega = 0.0  
        self.current = 0.0  
        self.dt = DT

    def step(self, voltage, dt=None):
        """
        Perform one simulation step with the given applied voltage.

        Args:
            voltage: Applied voltage to the motor (V)
            dt: Time step for this step (if None, uses self.dt)

        Returns:
            Current position in degrees
        """
        if dt is None:
            dt = self.dt

        
        di_dt = (voltage - R * self.current - K_b * self.omega) / L
        self.current += di_dt * dt
        
        torque = K_m * self.current
        friction = K_f * self.omega
        dw_dt = (torque - friction) / J
        self.omega += dw_dt * dt

        
        self.position_rad += self.omega * dt

        return np.rad2deg(self.position_rad)

    def get_position_deg(self):
        """Get current position in degrees."""
        return np.rad2deg(self.position_rad)

    def get_velocity_deg_per_sec(self):
        """Get current angular velocity in degrees per second."""
        return np.rad2deg(self.omega)

    def get_current(self):
        """Get current armature current in Amperes."""
        return self.current

    def reset(self, position_deg=0.0):
        """Reset the motor to initial conditions."""
        self.position_rad = np.deg2rad(position_deg)
        self.omega = 0.0
        self.current = 0.0
