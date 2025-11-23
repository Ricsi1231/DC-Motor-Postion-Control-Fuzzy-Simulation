MOTOR_RESPONSE_GAIN = 0.05  # Response gain multiplier

J = 3.2e-6  # Moment of inertia (kg·m²)
K_f = 3.5e-6  # Friction coefficient (N·m·s/rad)
K_m = 0.03  # Motor torque constant (N·m/A)
K_b = K_m  # Back-EMF constant (V·s/rad)
R = 4.0  # Armature resistance (Ω)
L = 0.001  # Armature inductance (H)

DT = 0.001  # Simulation time step (s)

VOLTAGE_SCALE = 0.082  # Control signal to voltage conversion

MAX_SIMULATION_STEPS = 300  # Maximum simulation steps
CONVERGENCE_THRESHOLD_POSITION = 0.5  # Position error threshold (degrees)
CONVERGENCE_THRESHOLD_DELTA = 0.5  # Error change threshold (degrees)

DELTA_ERROR_MIN = -50  # Minimum delta error (degrees)
DELTA_ERROR_MAX = 50  # Maximum delta error (degrees)

ERROR_RANGE_MIN = -180  # Minimum error for fuzzy set (degrees)
ERROR_RANGE_MAX = 181  # Maximum error for fuzzy set (degrees)
DELTA_ERROR_RANGE_MIN = -50  # Minimum delta error for fuzzy set (degrees)
DELTA_ERROR_RANGE_MAX = 51  # Maximum delta error for fuzzy set (degrees)
CONTROL_RANGE_MIN = -100  # Minimum control signal
CONTROL_RANGE_MAX = 101  # Maximum control signal

POSITION_MIN = -180  # Minimum position limit (degrees)
POSITION_MAX = 180  # Maximum position limit (degrees)

DISPLAY_INTERVAL = 20  # Console output interval (steps)

ENCODER_PPR = 1000  # Pulses per revolution
ENCODER_NOISE_STD = 0.1  # Measurement noise (degrees)
