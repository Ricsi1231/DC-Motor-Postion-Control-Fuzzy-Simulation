# DC Motor Position Control — Fuzzy & PID Simulation

[![CI](https://github.com/Ricsi1231/DC-Motor-Postion-Control-Fuzzy-Simulation/actions/workflows/ci.yml/badge.svg)](https://github.com/Ricsi1231/DC-Motor-Postion-Control-Fuzzy-Simulation/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/dc-motor-fuzzy-sim.svg)](https://pypi.org/project/dc-motor-fuzzy-sim/)
[![Python versions](https://img.shields.io/pypi/pyversions/dc-motor-fuzzy-sim.svg)](https://pypi.org/project/dc-motor-fuzzy-sim/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A closed-loop simulation of DC motor position control comparing **fuzzy logic** and **PID**
strategies. It models the motor's electrical and mechanical dynamics, feeds back through a
realistic encoder (discrete quantization plus Gaussian noise), and plots the result.

## Features

- **Two control strategies** — a 9-rule fuzzy logic controller with an integral term, and a
  classical PID controller, both behind one common interface.
- **Realistic motor physics** — coupled first-order electrical and mechanical dynamics with
  back-EMF, integrated with explicit Euler at 10 substeps per control period.
- **Encoder simulation** — 1000 PPR quantization with Gaussian measurement noise from an
  injectable random generator, so every run is reproducible from a seed.
- **Visualization** — membership functions, a 3D fuzzy control surface, time-series results, and
  a summary chart. Works headless: plots can be written straight to PNG.
- **Typed and tested** — a fully type-annotated package with 200+ tests and ~98% coverage.

## Installation

```bash
pip install dc-motor-fuzzy-sim
```

Requires Python 3.10 or newer.

<details>
<summary>Development install</summary>

```bash
git clone https://github.com/Ricsi1231/DC-Motor-Postion-Control-Fuzzy-Simulation.git
cd DC-Motor-Postion-Control-Fuzzy-Simulation
./setup.sh                 # creates .venv, installs -e '.[dev]', sets up pre-commit
source .venv/bin/activate
```

</details>

## Usage

### Command line

```bash
dc-motor-sim <start> <target> [options]
```

```bash
dc-motor-sim -90 45                       # fuzzy controller (default)
dc-motor-sim -90 45 --controller pid      # PID controller
dc-motor-sim 0 90 --seed 42               # reproducible encoder noise
dc-motor-sim 0 90 --no-show --output-dir plots   # headless: write PNGs
dc-motor-sim 0 90 --no-plot               # numbers only
```

| Option | Description | Default |
|---|---|---|
| `start`, `target` | Positions in degrees, within ±180 | required |
| `-c`, `--controller` | `fuzzy` or `pid` | `fuzzy` |
| `--output-dir DIR` | Write plots as PNG files into `DIR` | none |
| `--no-show` | Do not open plot windows (implied when there is no display) | off |
| `--no-plot` | Skip plotting entirely | off |
| `--seed N` | Seed the encoder noise for a reproducible run | random |
| `--max-steps N` | Maximum control steps | 300 |
| `--noise-std DEG` | Encoder noise standard deviation | 0.1 |
| `--ppr N` | Encoder pulses per revolution | 1000 |
| `-v`, `--verbose` | Log every progress line | off |

Invalid input exits with status `2`. `python -m dc_motor_sim` is equivalent to the console script.

### Python API

```python
from dc_motor_sim import FuzzyMotorController, PIDMotorController, run_simulation
import numpy as np

result = run_simulation(
    FuzzyMotorController(),
    start_deg=-90.0,
    target_deg=45.0,
    rng=np.random.default_rng(42),   # reproducible encoder noise
)

print(result.summary())
print(result.final_error, result.overshoot, result.converged)

# Every series is a numpy array of length steps + 1
result.time, result.actual_position, result.control, result.current
```

Plotting is opt-in and never blocks unless you ask it to:

```python
from dc_motor_sim.viz import plot_simulation_results

fig = plot_simulation_results(result, save_path="results.png", show=False)
```

### Configuration

All tunables live in frozen, self-validating dataclasses in `dc_motor_sim.config`:

```python
import dataclasses
from dc_motor_sim import EncoderParams, MotorParams, SimParams, run_simulation, PIDMotorController

result = run_simulation(
    PIDMotorController(),
    0.0, 90.0,
    sim_params=dataclasses.replace(SimParams(), max_steps=500),
    motor_params=MotorParams(R=2.0),
    encoder_params=EncoderParams(ppr=2048, noise_std=0.05),
)
```

## Project structure

```
version.txt                # single source of truth for the version
src/dc_motor_sim/
├── config.py              # frozen, validated parameter dataclasses
├── cli.py                 # argparse command line interface
├── model/dc_motor.py      # DCMotorModel — physics
├── sensors/encoder.py     # RotaryEncoder — quantization + noise
├── control/
│   ├── base.py            # PositionController protocol
│   ├── fuzzy.py           # FuzzyMotorController
│   └── pid.py             # PIDMotorController
├── simulation/
│   ├── runner.py          # run_simulation — the closed loop
│   └── result.py          # SimulationResult
└── viz/                   # backend selection + plotting
scripts/bump_version.py    # semver bump used by the release pipeline
tests/
├── unit/                  # per-module tests
└── integration/           # closed-loop, CLI, and golden-trace tests
docs/paper/                # seminar paper (Hungarian)
```

## How it works

Each control step, at 1 kHz:

1. **Encoder reads** the true shaft angle — quantized to 1000 PPR (0.36°/count), plus Gaussian
   noise (σ = 0.1° by default).
2. **Error terms**: `error = target − measured`, and its step-to-step change.
3. **Controller computes** a signal in roughly ±100.
   Fuzzy: fuzzification → 9-rule inference → defuzzification, plus a clamped integral term.
   PID: `Kp·e + Ki·∫e + Kd·de/dt`, saturated to ±100.
4. **Motor integrates** `voltage = control × 0.082` over 10 substeps of 100 µs each.
5. **Convergence check**: stop when `|error| < 0.5°` and `|Δerror| < 0.5°`.

### Why 10 substeps

Explicit Euler is stable only while the integration step stays below the electrical time constant
`L/R = 250 µs`. The 1 ms control period exceeds that and diverges; `1 ms / 10 = 100 µs` does not.
`tests/unit/test_dc_motor.py` asserts both halves of this, so the substepping cannot be
"simplified" away by accident.

## Physics

*Electrical:* `L·di/dt + R·i = V − K_b·ω`
*Mechanical:* `J·dω/dt = K_m·i − K_f·ω`
*Kinematic:* `dθ/dt = ω`

| Parameter | Value | Description |
|---|---|---|
| `J` | 3.2e-6 kg·m² | Moment of inertia |
| `K_f` | 3.5e-6 N·m·s/rad | Viscous friction coefficient |
| `K_m` | 0.03 N·m/A | Torque constant |
| `K_b` | 0.03 V·s/rad | Back-EMF constant |
| `R` | 4.0 Ω | Armature resistance |
| `L` | 0.001 H | Armature inductance |

Steady state: `ω/V = K_m / (K_f·R + K_m·K_b)` ≈ 32.8 rad/s per volt.

## Fuzzy controller design

Error, delta-error, and control are each partitioned into Negative / Zero / Positive sets.

| Error \ Δ Error | Negative | Zero | Positive |
|---|---|---|---|
| **Negative** | N | N | Z |
| **Zero** | Z | Z | Z |
| **Positive** | Z | P | P |

Membership functions (trapezoidal for N and P, triangular for Z):

- **Error**: N(−180, −180, −30, −5), Z(−8, 0, 8), P(5, 30, 180, 180)
- **Δ Error**: N(−50, −50, −6, −1), Z(−2, 0, 2), P(1, 6, 50, 50)
- **Control**: N(−100, −100, −35, −10), Z(−15, 0, 15), P(10, 35, 100, 100)

An integral term (`Ki = 0.5`, clamped to ±300) is added to the defuzzified output to remove the
steady-state offset that pure inference leaves behind.

## Controller comparison

| Aspect | Fuzzy logic | PID |
|---|---|---|
| Tuning | Rule-based, intuitive | Three gains (Kp, Ki, Kd) |
| Non-linearity | Handled inherently | Linear response |
| Steady state | Integral term added on | Integral term built in |
| Overshoot | Generally lower | Depends on tuning |
| Complexity | Higher (9 rules) | Lower (3 gains) |

## Extending

Add a controller by implementing the `PositionController` protocol — no changes to the runner
are needed:

```python
class MyController:
    name = "MyController"

    def set_target(self, target_deg: float) -> None: ...
    def compute(self, measured_deg: float, dt: float) -> float: ...
    def reset(self) -> None: ...

run_simulation(MyController(), -90.0, 45.0)
```

## Development

```bash
pytest                        # full suite with an 80% coverage gate
pytest -m "not slow"          # skip the full-resolution control surface
ruff check . && ruff format --check .
mypy src
```

### Versioning

`version.txt` is the single source of truth, and it starts at `1.0.0`. Every
merge to `main` bumps it automatically from the
[Conventional Commits](https://www.conventionalcommits.org/) in that merge, then
tags and publishes the result:

| Commit | Bump |
|---|---|
| `feat!: ...` or a `BREAKING CHANGE:` footer | major |
| `feat: ...` | minor |
| anything else (including a merge with no conventional commits) | patch |

Preview what a merge would produce without changing anything:

```bash
python scripts/bump_version.py --since v1.0.0
```

See [CONTRIBUTING.md](CONTRIBUTING.md) for the full workflow.

## License

MIT — see [LICENSE](LICENSE).

© 2025 Nagy Richárd. Created as part of the **Intelligent Control Systems** course.
