import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from motor_parameters import (ERROR_RANGE_MIN, ERROR_RANGE_MAX,
                              DELTA_ERROR_RANGE_MIN, DELTA_ERROR_RANGE_MAX,
                              CONTROL_RANGE_MIN, CONTROL_RANGE_MAX)


class FuzzyMotorController:
    """
    Fuzzy logic controller for DC motor position control.

    Uses a 3x3 fuzzy rule base with error and delta error as inputs,
    and control signal as output.
    """

    def __init__(self):
        self.integral = 0.0
        self.ki = 0.1   

        self.error = ctrl.Antecedent(np.arange(ERROR_RANGE_MIN, ERROR_RANGE_MAX, 1), 'error')

        self.delta_error = ctrl.Antecedent(np.arange(DELTA_ERROR_RANGE_MIN, DELTA_ERROR_RANGE_MAX, 1), 'delta_error')

        self.control = ctrl.Consequent(np.arange(CONTROL_RANGE_MIN, CONTROL_RANGE_MAX, 1), 'control')

        self.error['N'] = fuzz.trapmf(self.error.universe, [-180, -180, -30, -5])
        self.error['Z'] = fuzz.trimf(self.error.universe, [-8, 0, 8])
        self.error['P'] = fuzz.trapmf(self.error.universe, [5, 30, 180, 180])

        self.delta_error['N'] = fuzz.trapmf(self.delta_error.universe, [-50, -50, -6, -1])
        self.delta_error['Z'] = fuzz.trimf(self.delta_error.universe, [-2, 0, 2])
        self.delta_error['P'] = fuzz.trapmf(self.delta_error.universe, [1, 6, 50, 50])

        self.control['N'] = fuzz.trapmf(self.control.universe, [-100, -100, -35, -10])
        self.control['Z'] = fuzz.trimf(self.control.universe, [-15, 0, 15])
        self.control['P'] = fuzz.trapmf(self.control.universe, [10, 35, 100, 100])

        rule1 = ctrl.Rule(self.error['N'] & self.delta_error['N'], self.control['N'])
        rule2 = ctrl.Rule(self.error['N'] & self.delta_error['Z'], self.control['N'])
        rule3 = ctrl.Rule(self.error['N'] & self.delta_error['P'], self.control['Z'])
        rule4 = ctrl.Rule(self.error['Z'] & self.delta_error['N'], self.control['Z'])
        rule5 = ctrl.Rule(self.error['Z'] & self.delta_error['Z'], self.control['Z'])
        rule6 = ctrl.Rule(self.error['Z'] & self.delta_error['P'], self.control['Z'])
        rule7 = ctrl.Rule(self.error['P'] & self.delta_error['N'], self.control['Z'])
        rule8 = ctrl.Rule(self.error['P'] & self.delta_error['Z'], self.control['P'])
        rule9 = ctrl.Rule(self.error['P'] & self.delta_error['P'], self.control['P'])

        self.control_system = ctrl.ControlSystem([
            rule1, rule2, rule3,
            rule4, rule5, rule6,
            rule7, rule8, rule9,
        ])

        self.simulation = ctrl.ControlSystemSimulation(self.control_system)

    def compute_control(self, error_val, delta_error_val, dt):
        """
        Compute control signal using fuzzy inference.

        Args:
            error_val: Position error in degrees
            delta_error_val: Change in error

        Returns:
            Control signal value
        """
        self.integral += error_val * dt
        self.integral = np.clip(self.integral, -300, 300)

        self.simulation.input['error'] = error_val
        self.simulation.input['delta_error'] = delta_error_val
        self.simulation.compute()

        fuzzy_output = self.simulation.output['control']

        return fuzzy_output + self.ki * self.integral

    def get_membership_functions(self):
        """
        Get membership functions for visualization.

        Returns:
            Tuple of (error, delta_error, control) membership functions
        """
        return self.error, self.delta_error, self.control
