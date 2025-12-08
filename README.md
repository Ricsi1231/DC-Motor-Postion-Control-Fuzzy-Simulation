# DC Motor Position Control - Fuzzy & PID Simulation

A comprehensive simulation system for DC motor position control comparing **Fuzzy Logic** and **PID** control strategies. Features realistic motor physics, encoder feedback with quantization and noise, and extensive visualization capabilities.

## Features

- **Dual Control Strategies**: Fuzzy Logic Controller (9 rules with integral term) and traditional PID Controller
- **Realistic Motor Physics**: First-order electrical and mechanical dynamics with back-EMF modeling
- **Encoder Simulation**: Discrete quantization (1000 PPR) with Gaussian measurement noise
- **Visualization**: Membership functions, 3D control surfaces, simulation results, and convergence plots
- **Modular Architecture**: Clean separation of motor model, sensors, controllers, and visualization

## Requirements

- Python 3.x
- numpy
- scipy
- scikit-fuzzy
- matplotlib
- simple-pid
- networkx

## Installation

### Using setup script (recommended)

```bash
./setup.sh
```

This creates a virtual environment and installs all dependencies.

### Manual installation

```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Usage

### Using the run script

```bash
# Fuzzy controller (default)
./run_simulation.sh start_position=-90 end_position=45

# PID controller
./run_simulation.sh start_position=-90 end_position=45 controller=pid
```

### Direct Python execution

```bash
python main.py <start_position> <target_position> [fuzzy|pid]
```

**Arguments:**
- `start_position`: Initial motor position in degrees (-180 to 180)
- `target_position`: Target motor position in degrees (-180 to 180)
- `controller`: Optional - `fuzzy` (default) or `pid`

**Examples:**
```bash
python main.py 0 90 fuzzy    # Start at 0°, target 90° with Fuzzy
python main.py -45 45 pid    # Start at -45°, target 45° with PID
python main.py 0 180         # Default fuzzy controller
```

## Project Structure

```
├── main.py                 # Entry point and simulation orchestration
├── fuzzy_controller.py     # Fuzzy logic controller (9 rules + integral)
├── pid_controller.py       # PID controller wrapper
├── dc_motor_model.py       # DC motor physics simulation
├── encoder_sensor.py       # Encoder with quantization & noise
├── visualization.py        # Matplotlib plotting functions
├── motor_parameters.py     # Configuration constants
├── run_simulation.sh       # Bash wrapper script
├── setup.sh                # Environment setup
└── requirements.txt        # Python dependencies
```

## Python Modules Documentation

### main.py

The entry point that orchestrates the entire simulation.

**Key Functions:**

| Function | Description |
|----------|-------------|
| `simulate_motor_control_fuzzy()` | Runs closed-loop simulation with fuzzy controller |
| `simulate_motor_control_pid()` | Runs closed-loop simulation with PID controller |
| `main()` | CLI interface, argument parsing, and execution flow |

**Simulation Loop Logic:**
```python
for step in range(MAX_SIMULATION_STEPS):
    # 1. Read encoder (quantized + noisy)
    measured_position = encoder.read_position(actual_position)

    # 2. Calculate errors
    error = target - measured_position
    delta_error = error - previous_error

    # 3. Compute control signal
    control = controller.compute_control(error, delta_error, dt)

    # 4. Apply voltage to motor (10 substeps for accuracy)
    voltage = control * VOLTAGE_SCALE
    for _ in range(10):
        motor.step(voltage, dt/10)

    # 5. Check convergence
    if abs(error) < 0.5 and abs(delta_error) < 0.5:
        break
```

### dc_motor_model.py

Simulates realistic DC motor physics using first-order coupled differential equations.

**Class: `DCMotorModel`**

```python
class DCMotorModel:
    def __init__(self, initial_position_deg=0.0)
    def step(self, voltage, dt) -> None
    def get_position_deg() -> float
    def get_velocity_deg_per_sec() -> float
    def get_current() -> float
    def reset() -> None
```

**Physics Equations:**

*Electrical Circuit (Armature):*
```
L·(di/dt) + R·i = V_applied - K_b·ω
```

*Mechanical System:*
```
J·(dω/dt) = K_m·i - K_f·ω
dθ/dt = ω
```

**Motor Parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| J | 3.2e-6 kg·m² | Moment of inertia |
| K_f | 3.5e-6 N·m·s/rad | Friction coefficient |
| K_m | 0.03 N·m/A | Torque constant |
| K_b | 0.03 V·s/rad | Back-EMF constant |
| R | 4.0 Ω | Armature resistance |
| L | 0.001 H | Armature inductance |

**State Variables:**
- `position_rad`: Angular position (radians)
- `omega`: Angular velocity (rad/s)
- `current`: Armature current (A)

**Integration Method:** Explicit Euler with configurable time step

### encoder_sensor.py

Simulates a rotary encoder with realistic quantization and measurement noise.

**Class: `RotaryEncoder`**

```python
class RotaryEncoder:
    def __init__(self, pulses_per_revolution=1000, noise_std=0.1)
    def read_position(actual_position_deg) -> float
    def get_count() -> int
    def get_resolution() -> float
    def get_velocity(dt) -> float
    def reset() -> None
```

**Encoder Characteristics:**
- **Resolution**: 1000 PPR (0.36° per count)
- **Noise**: Gaussian with σ = 0.1°
- **Quantization**: Rounds to nearest encoder count

**Processing Pipeline:**
```
Actual Position → Quantize to Counts → Add Gaussian Noise → Output
```

### fuzzy_controller.py

Implements a fuzzy logic controller using scikit-fuzzy.

**Class: `FuzzyMotorController`**

```python
class FuzzyMotorController:
    def __init__()
    def compute_control(error, delta_error, dt) -> float
    def get_membership_functions() -> tuple
```

**Fuzzy System Design:**

*Input Variables:*
| Variable | Range | Membership Sets |
|----------|-------|-----------------|
| Error | -180° to 180° | Negative (N), Zero (Z), Positive (P) |
| Delta Error | -50 to 50 | Negative (N), Zero (Z), Positive (P) |

*Output Variable:*
| Variable | Range | Membership Sets |
|----------|-------|-----------------|
| Control | -100 to 100 | Negative (N), Zero (Z), Positive (P) |

**Membership Functions:**
- Error: Trapezoidal - N(-180,-180,-30,-5), Triangular - Z(-8,0,8), Trapezoidal - P(5,30,180,180)
- Delta Error: Trapezoidal - N(-50,-50,-6,-1), Triangular - Z(-2,0,2), Trapezoidal - P(1,6,50,50)
- Control: Trapezoidal - N(-100,-100,-35,-10), Triangular - Z(-15,0,15), Trapezoidal - P(10,35,100,100)

**Rule Base (9 rules):**

| Error \ Delta | Negative | Zero | Positive |
|---------------|----------|------|----------|
| **Negative**  | N        | N    | Z        |
| **Zero**      | Z        | Z    | Z        |
| **Positive**  | Z        | P    | P        |

**Integral Term:** Ki = 0.5 (cumulative error, clipped to [-300, 300])

### pid_controller.py

Wraps the `simple-pid` library for PID control.

**Class: `PIDMotorController`**

```python
class PIDMotorController:
    def __init__(self, kp=2.0, ki=0.5, kd=0.1)
    def set_target(target) -> None
    def compute_control(measured_position, dt) -> float
    def reset() -> None
    def set_tunings(kp, ki, kd) -> None
    def get_components() -> tuple  # Returns (P, I, D) terms
```

**PID Parameters:**
| Parameter | Value | Purpose |
|-----------|-------|---------|
| Kp | 2.0 | Proportional gain - main response |
| Ki | 0.5 | Integral gain - eliminates steady-state error |
| Kd | 0.1 | Derivative gain - reduces overshoot |
| Output limits | [-100, 100] | Control signal bounds |

### motor_parameters.py

Centralized configuration for all simulation parameters.

**Motor Physics:**
```python
J = 3.2e-6          # Moment of inertia (kg·m²)
K_f = 3.5e-6        # Friction coefficient (N·m·s/rad)
K_m = 0.03          # Motor torque constant (N·m/A)
K_b = 0.03          # Back-EMF constant (V·s/rad)
R = 4.0             # Armature resistance (Ω)
L = 0.001           # Armature inductance (H)
```

**Control Parameters:**
```python
DT = 0.001                          # Time step (1 ms)
VOLTAGE_SCALE = 0.082               # Control to voltage scaling
MAX_SIMULATION_STEPS = 300          # Max steps (0.3 seconds)
CONVERGENCE_THRESHOLD_POSITION = 0.5  # Position threshold (°)
CONVERGENCE_THRESHOLD_DELTA = 0.5     # Error rate threshold (°)
```

**Encoder Settings:**
```python
ENCODER_PPR = 1000      # Pulses per revolution
ENCODER_NOISE_STD = 0.1 # Measurement noise (°)
```

### visualization.py

Matplotlib-based visualization functions.

**Functions:**

| Function | Description |
|----------|-------------|
| `plot_membership_functions(controller)` | 3-panel plot of fuzzy membership functions |
| `plot_control_surface(controller)` | 3D surface plot of fuzzy output |
| `plot_simulation_results(...)` | 2x2 plot: position, error, control, phase |
| `plot_pid_results(...)` | Same as above for PID controller |
| `plot_final_summary(...)` | Bar chart comparing initial/target/final |

**Visualization Features:**
- Color-coded membership functions (Red/Green/Blue for N/Z/P)
- Interactive 3D control surface with 30×30 grid
- Phase plane plot (control signal vs error)
- Encoder readings shown as dashed lines

## Simulation Details

### Control Loop Timing
- **Control frequency**: 1000 Hz (1 ms per step)
- **Motor substeps**: 10 per control step (0.1 ms integration)
- **Maximum duration**: 300 ms (300 steps)

### Convergence Criteria
Simulation terminates when both conditions are met:
1. Position error < 0.5°
2. Error change rate < 0.5° per step

### Data Flow Per Step

1. **Encoder reads actual position**
   - Applies quantization (1000 PPR)
   - Adds Gaussian noise (σ=0.1°)
   - Returns measured position

2. **Error calculation**
   - error = target - measured
   - delta_error = error - previous_error

3. **Controller computes output**
   - Fuzzy: Fuzzification → Rule evaluation → Defuzzification + Integral
   - PID: P×error + I×∫error + D×d(error)/dt

4. **Motor receives voltage**
   - voltage = control × 0.082
   - 10 integration substeps (Euler method)
   - Updates position, velocity, current

5. **Logging and convergence check**

## Example Output

```
=== DC Motor Position Control with Fuzzy Logic ===
Controller: Fuzzy Logic with Integral
Encoder: 1000 PPR (0.36°/count), noise σ=0.10°
Initial Position: -90.00°, Target: 45.00°
Time step: 0.001s

Step  20 (0.020s): Actual=-67.23°, Measured=-67.32°, Err=112.32°, V=8.20V
Step  40 (0.040s): Actual=-21.45°, Measured=-21.60°, Err=66.60°, V=8.20V
Step  60 (0.060s): Actual=15.67°, Measured=15.48°, Err=29.52°, V=8.20V
Step  80 (0.080s): Actual=35.89°, Measured=35.64°, Err=9.36°, V=6.89V
Step 100 (0.100s): Actual=42.34°, Measured=42.12°, Err=2.88°, V=3.45V
...
Converged at step 156 (0.156s)

=== Final State ===
Actual Position: 44.89°
Measured Position: 44.82°
Error: 0.18°
```

## Comparing Controllers

| Aspect | Fuzzy Logic | PID |
|--------|-------------|-----|
| Tuning | Rule-based, intuitive | Gain parameters (Kp, Ki, Kd) |
| Non-linearity | Inherently handles | Linear response |
| Steady-state | Integral term added | Integral term built-in |
| Overshoot | Generally lower | Depends on tuning |
| Complexity | Higher (9 rules) | Lower (3 gains) |

## Extending the Simulation

### Adding a New Controller

1. Create `new_controller.py`:
```python
class NewController:
    def __init__(self):
        # Initialize controller
        pass

    def compute_control(self, error, delta_error, dt):
        # Return control signal in range [-100, 100]
        return control_signal
```

2. Import in `main.py` and add simulation function
3. Update CLI argument parsing

### Modifying Motor Parameters

Edit `motor_parameters.py` to change:
- Motor physical constants (J, K_m, K_b, R, L, K_f)
- Control parameters (DT, VOLTAGE_SCALE)
- Encoder settings (PPR, noise level)
- Convergence thresholds

## License

© 2025 Nagy Richárd.
This project was created as part of the **Intelligent Control Systems** course.

It is intended for **educational and prototyping purposes only**.
Commercial use, distribution, or modification requires **prior written permission** from the author.
