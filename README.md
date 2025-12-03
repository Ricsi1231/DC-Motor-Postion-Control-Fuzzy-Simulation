# DC Motor Position Control - Fuzzy & PID Simulation

A simulation system for DC motor position control comparing **Fuzzy Logic** and **PID** control strategies. Features realistic motor physics, encoder feedback with quantization and noise, and comprehensive visualization.

## Features

- **Dual Control Strategies**: Fuzzy Logic Controller (9 rules) and traditional PID Controller
- **Realistic Motor Physics**: First-order electrical and mechanical dynamics with back-EMF
- **Encoder Simulation**: Discrete quantization (1000 PPR) with Gaussian measurement noise
- **Visualization**: Membership functions, control surfaces, simulation results, and convergence plots

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

## How It Works

### Motor Model

The DC motor simulation uses realistic first-order dynamics:

**Electrical:**
```
L·(di/dt) + R·i = V_applied - K_b·ω
```

**Mechanical:**
```
J·(dω/dt) = K_m·i - K_f·ω
dθ/dt = ω
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| J | 3.2e-6 kg·m² | Moment of inertia |
| K_f | 3.5e-6 N·m·s/rad | Friction coefficient |
| K_m | 0.03 N·m/A | Torque constant |
| K_b | 0.03 V·s/rad | Back-EMF constant |
| R | 4.0 Ω | Armature resistance |
| L | 0.001 H | Armature inductance |

### Encoder Sensor

- **Resolution**: 1000 pulses per revolution (0.36° per count)
- **Noise**: Gaussian with 0.1° standard deviation
- Simulates real-world quantization effects

### Fuzzy Logic Controller

Uses scikit-fuzzy with 2 inputs and 1 output:

**Inputs:**
- **Error**: Position error (Negative, Zero, Positive)
- **Delta Error**: Rate of error change (Negative, Zero, Positive)

**Output:**
- **Control Signal**: Motor drive signal (Negative, Zero, Positive)

**Rule Base (9 rules):**

| Error \ Delta | Negative | Zero | Positive |
|---------------|----------|------|----------|
| **Negative**  | N        | N    | Z        |
| **Zero**      | Z        | Z    | Z        |
| **Positive**  | Z        | P    | P        |

An integral term (Ki = 0.1) is added to eliminate steady-state error.

### PID Controller

Standard PID with tuned parameters:
- **Kp** = 2.0 (Proportional)
- **Ki** = 0.5 (Integral)
- **Kd** = 0.1 (Derivative)
- Output limits: [-100, 100]

### Simulation Loop

1. Read encoder feedback (quantized + noisy)
2. Calculate error and error derivative
3. Compute control signal (Fuzzy or PID)
4. Scale to voltage (×0.082)
5. Integrate motor dynamics (10 substeps per control step)
6. Check convergence (error < 0.5° AND |dError| < 0.5°)

**Time step**: 1 ms | **Max steps**: 300 (0.3 seconds)

## Visualization

The fuzzy simulation displays:

1. **Membership Functions**: Error, Delta Error, and Control membership functions
2. **Control Surface**: 3D plot showing fuzzy output vs error and delta error
3. **Simulation Results**: Position tracking, error, control signal, and phase plot
4. **Final Summary**: Bar chart comparing initial, target, and final positions

## Example Output

```
=== DC Motor Position Control with Fuzzy Logic ===
Controller: Fuzzy Logic with Integral
Encoder: 1000 PPR (0.36°/count), noise σ=0.10°
Initial Position: -90.00°, Target: 45.00°
Time step: 0.001s

Step  20 (0.020s): Actual=-67.23°, Measured=-67.32°, Err=112.32°, V=8.20V
Step  40 (0.040s): Actual=-21.45°, Measured=-21.60°, Err=66.60°, V=8.20V
...
Converged at step 156 (0.156s)

=== Final State ===
Actual Position: 44.89°
Measured Position: 44.82°
Error: 0.18°
```