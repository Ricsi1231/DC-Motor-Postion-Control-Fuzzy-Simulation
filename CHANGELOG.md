# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

Restructured the project from a set of loose scripts into an installable, tested
Python package. Simulation behaviour is unchanged: golden traces captured from
the pre-restructure code are asserted to reproduce exactly, to a relative
tolerance of 1e-12 (`tests/integration/test_characterization.py`).

### Added

- `version.txt` at the repository root as the single source of truth for the
  version, starting at `1.0.0`. The build backend reads it, so there is no
  version string in the source and no dependency on git metadata — an sdist can
  now be rebuilt without a git checkout.
- `scripts/bump_version.py`, which derives the next semantic version from
  Conventional Commits (`feat!`/`BREAKING CHANGE:` → major, `feat` → minor,
  anything else → patch, minimum patch).
- Automatic release on every merge to `main`: CI bumps `version.txt`, commits it
  back with `[skip ci]`, tags `vX.Y.Z`, then builds and publishes.
- `src/` layout package `dc_motor_sim`, published to PyPI as `dc-motor-fuzzy-sim`.
- `dc-motor-sim` console script and `python -m dc_motor_sim` entry point, with a
  proper `argparse` interface: `--controller`, `--output-dir`, `--no-show`,
  `--no-plot`, `--seed`, `--max-steps`, `--noise-std`, `--ppr`, `--verbose`,
  `--version`.
- `PositionController` protocol; both controllers now implement one interface.
- `SimulationResult` dataclass exposing every series as a numpy array, plus
  `converged`, `duration`, `final_error`, `overshoot`, and `summary()`. The
  voltage, current, and velocity series were previously collected and discarded.
- Validated, frozen configuration dataclasses in `dc_motor_sim.config`:
  `MotorParams`, `EncoderParams`, `SimParams`, `FuzzyParams`, `PIDParams`.
- Injectable `numpy.random.Generator` on `RotaryEncoder`, making every run
  reproducible from a seed.
- `save_path` and `show` parameters on every plotting function; each now returns
  its `Figure`. `plot_control_surface` gained a `grid_size` parameter.
- Test suite: 205 tests across unit, integration, CLI, and golden-trace levels,
  at ~98% coverage with an 80% gate.
- CI (lint, mypy, test matrix over Python 3.10–3.13 plus macOS/Windows, build)
  and a tag-driven release pipeline publishing to PyPI via Trusted Publishing
  with build provenance attestation.
- `pyproject.toml`, `.pre-commit-config.yaml`, `CONTRIBUTING.md`, type hints
  throughout, and a `py.typed` marker.

### Changed

- **Breaking:** modules moved. `dc_motor_model` → `dc_motor_sim.model.dc_motor`,
  `encoder_sensor` → `dc_motor_sim.sensors.encoder`, `fuzzy_controller` →
  `dc_motor_sim.control.fuzzy`, `pid_controller` → `dc_motor_sim.control.pid`,
  `motor_parameters` → `dc_motor_sim.config`, `visualization` →
  `dc_motor_sim.viz`, `main` → `dc_motor_sim.simulation.runner` + `cli`.
- **Breaking:** `simulate_motor_control_fuzzy` and `simulate_motor_control_pid`
  are replaced by a single `run_simulation()` returning a `SimulationResult`
  instead of a seven-element tuple.
- **Breaking:** controller entry point renamed from `compute_control` to
  `compute(measured_deg, dt)`. Each controller now owns its setpoint and derives
  its own error terms, which is what allowed the two duplicated simulation loops
  (~95 near-identical lines each) to collapse into one.
- **Breaking:** invalid CLI input now exits with status `2` instead of `1`.
- **Breaking:** licence changed from Apache-2.0 to MIT. The previous `LICENSE`
  file (Apache-2.0) contradicted the README, which claimed educational use only.
- `plot_simulation_results` and `plot_pid_results` merged into one function; they
  differed only in two title strings.
- `DCMotorModel.step` now requires an explicit `dt`, and `reset()` defaults to
  the position the model was constructed with rather than zero.
- Simulation progress uses `logging` instead of `print`.
- Seminar paper moved to `docs/paper/` and excluded from the distributions,
  taking the sdist from roughly 9 MB to 38 KB.
- `setup.sh` and `run_simulation.sh` now use `.venv` and the console script.

### Fixed

- `RotaryEncoder.get_velocity` always returned `0.0`. `read_position` overwrote
  the stored previous reading with the current one, so the difference was
  structurally always zero. The signature is now `get_velocity(dt)`, matching
  what the README always documented. It now correctly returns `0.0` until two
  samples exist, rather than differencing against an initial `0.0` sentinel.
- `--verbose` was ignored on any call to `cli.main()` after the first in the
  same process, because `logging.basicConfig` is a no-op once the root logger
  has handlers. The level is now set explicitly.
- `SimulationResult.overshoot` could return `-0.0` for a negative-going move
  that landed exactly on target, rendering as `-0.0000` in the CLI summary.
- `viz.backend.ensure_backend` did not latch its "select once" flag on the
  `MPLBACKEND` path, so a later call could force a backend switch underneath
  figures that were already open.
- Importing the plotting code no longer forces the `TkAgg` backend at import
  time, and plots no longer unconditionally block on `plt.show()`. The package
  is now importable and usable on a headless machine.
- Removed the unused `MOTOR_RESPONSE_GAIN` constant and an unused `Axes3D`
  import.
- The magic `substeps = 10`, duplicated in both simulation loops, is now a
  documented `SimParams` field with tests asserting why it is required.

### Deprecated

- `main.py` at the repository root. It still works, translating the old
  `python main.py 0 90 fuzzy` form, but prints a deprecation notice and will be
  removed in a future release. Use `dc-motor-sim` instead.

## [1.0.0] — 2025

### Added

- Fuzzy logic controller with a 9-rule base and an integral term.
- PID controller built on `simple-pid`.
- DC motor physics model with electrical and mechanical dynamics.
- Rotary encoder model with quantization and Gaussian noise.
- Matplotlib visualizations: membership functions, 3D control surface,
  simulation results, and a summary chart.
- Bash wrappers `setup.sh` and `run_simulation.sh`.

[Unreleased]: https://github.com/Ricsi1231/DC-Motor-Postion-Control-Fuzzy-Simulation/compare/1.0.0...HEAD
[1.0.0]: https://github.com/Ricsi1231/DC-Motor-Postion-Control-Fuzzy-Simulation/releases/tag/1.0.0
