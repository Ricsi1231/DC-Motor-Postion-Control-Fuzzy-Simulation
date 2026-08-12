# Contributing

Thanks for your interest in the project. This document covers the development
setup, the quality bar, and how releases are cut.

## Development setup

```bash
git clone https://github.com/Ricsi1231/DC-Motor-Postion-Control-Fuzzy-Simulation.git
cd DC-Motor-Postion-Control-Fuzzy-Simulation
./setup.sh
source .venv/bin/activate
```

`setup.sh` creates `.venv`, installs the package with `-e '.[dev]'`, and installs
the pre-commit hooks. To do it by hand:

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e '.[dev]'
pre-commit install
```

## Quality checks

Everything CI runs, you can run locally:

```bash
pytest                        # full suite; fails under 80% coverage
pytest -m "not slow"          # skip the full-resolution control surface
ruff check .                  # lint
ruff format --check .         # formatting
mypy src                      # strict type checking
```

Plotting tests run under the `Agg` backend, which `tests/conftest.py` sets
automatically. Nothing in the suite should ever open a window.

## Testing expectations

- New behaviour needs tests. Coverage must stay at or above 80%; it currently
  sits near 98%.
- Tests follow Arrange–Act–Assert with descriptive names
  (`test_reset_clears_the_integral`, not `test_reset_2`).
- Anything touching encoder noise must inject a seeded
  `numpy.random.Generator` so the test is deterministic.
- Building `FuzzyMotorController` is expensive. Use the session-scoped
  `fuzzy_controller` fixture, or `fresh_fuzzy_controller` when the test mutates
  controller state.
- Slow tests get `@pytest.mark.slow`.

### Do not regenerate the golden traces

`tests/data/golden_*.json` were captured from the pre-restructure code and are
asserted to reproduce exactly by
`tests/integration/test_characterization.py`. They exist to prove that
refactoring does not silently change the physics or the control loop.

If one fails, the change altered simulation behaviour. Fix the code. Only
regenerate a golden when the behaviour change is deliberate, and say so
explicitly in the commit message and the changelog.

## Code style

- Immutable configuration: parameters live in frozen dataclasses in
  `dc_motor_sim/config.py` that validate themselves. No bare module constants.
- No magic numbers in the simulation loop — put them in `config.py` with a
  docstring explaining the value.
- Full type annotations; `mypy --strict` must pass on `src/`.
- Functions stay small and files stay focused. Physics-style names (`J`, `R`,
  `K_m`) are allowed and are exempted in the ruff config.
- Errors are raised with a message that names the offending value.

## Adding a controller

Implement the `PositionController` protocol and the runner will drive it
unchanged:

```python
class MyController:
    name = "MyController"

    def set_target(self, target_deg: float) -> None: ...
    def compute(self, measured_deg: float, dt: float) -> float: ...
    def reset(self) -> None: ...
```

Add a unit test module mirroring `tests/unit/test_pid_controller.py`, and add
the controller to the parameterized integration tests.

## Commits and pull requests

Commits follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <description>
```

Types: `feat`, `fix`, `refactor`, `docs`, `test`, `chore`, `perf`, `ci`.
Examples: `feat(control): add sliding mode controller`,
`fix(encoder): correct velocity estimation`.

Before opening a pull request:

1. All checks above pass locally.
2. Coverage has not regressed.
3. `CHANGELOG.md` has an entry under `## [Unreleased]`.
4. The branch is up to date with `main` and has no merge conflicts.

## Releasing

Versions come from git tags via `hatch-vcs` — there is no version string to bump
in the source.

1. Move the `## [Unreleased]` entries in `CHANGELOG.md` under the new version
   heading, and commit.
2. Rehearse with a pre-release tag first. It publishes to TestPyPI only:
   ```bash
   git tag v1.1.0rc1 && git push origin v1.1.0rc1
   ```
3. When that pipeline is green, tag the real release:
   ```bash
   git tag v1.1.0 && git push origin v1.1.0
   ```

The release workflow then runs the full CI suite, builds the sdist and wheel,
verifies the tag matches the built version, publishes to PyPI through Trusted
Publishing (OIDC — no API token is stored), attests build provenance, and
creates a GitHub Release with the artifacts attached.

Note that the historical tag is bare `1.0.0`; all tags from here on use the `v`
prefix.

### One-time publishing setup

These are configured outside the repository:

- On PyPI, under **Publishing**, add a pending trusted publisher: owner
  `Ricsi1231`, repository `DC-Motor-Postion-Control-Fuzzy-Simulation`, workflow
  `release.yml`, environment `pypi`.
- Repeat on TestPyPI with environment `testpypi`.
- In GitHub under **Settings → Environments**, create `pypi` and `testpypi`.
  Adding a required reviewer to `pypi` is recommended.
