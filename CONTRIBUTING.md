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

## Versioning and releasing

`version.txt` at the repository root is the single source of truth. The build
backend reads it, so there is no version string anywhere in the source.

**Every merge to `main` cuts a release automatically.** You do not edit
`version.txt` by hand — `.github/workflows/release.yml` does it:

1. It classifies every commit since the last tag using Conventional Commits.
2. It rewrites `version.txt`, commits it back to `main` as
   `chore(release): vX.Y.Z [skip ci]`, and tags `vX.Y.Z`.
3. It runs the full CI suite, builds, publishes to PyPI via Trusted Publishing,
   attests build provenance, and creates a GitHub Release.

### What your commit message decides

| Commit | Bump | 1.4.2 becomes |
|---|---|---|
| `feat!: ...`, or `BREAKING CHANGE:` in the footer | major | `2.0.0` |
| `feat: ...` | minor | `1.5.0` |
| `fix:`, `docs:`, `chore:`, `perf:`, anything else | patch | `1.4.3` |

The strongest level in the merge wins. A merge with no conventional commits at
all still bumps the patch, so every merge to `main` ships something.

**This means your commit message directly determines the published version.**
Use `feat!:` or a `BREAKING CHANGE:` footer deliberately — it burns a major
version.

### Previewing a bump locally

```bash
python scripts/bump_version.py --show                     # current version
python scripts/bump_version.py --since v1.4.2             # what would be next
python scripts/bump_version.py --level minor              # force a level
python scripts/bump_version.py --set 2.0.0 --write        # override entirely
```

Without `--write` nothing is modified; the next version is just printed.

### Cutting a release by hand

Pushing a tag yourself skips the bump step and publishes that tag as-is. Use a
pre-release tag to rehearse the pipeline against TestPyPI without burning a real
version:

```bash
git tag v1.5.0rc1 && git push origin v1.5.0rc1
```

The build job refuses to publish if the tag, `version.txt`, and the built wheel
disagree.

### Notes

- The historical tag is bare `1.0.0`; every tag from here on uses the `v` prefix.
- If `main` is protected, the release bot needs permission to push to it.
  Allow `github-actions[bot]` to bypass the restriction, or the bump commit
  will be rejected.

### One-time publishing setup

These are configured outside the repository:

- On PyPI, under **Publishing**, add a pending trusted publisher: owner
  `Ricsi1231`, repository `DC-Motor-Postion-Control-Fuzzy-Simulation`, workflow
  `release.yml`, environment `pypi`.
- Repeat on TestPyPI with environment `testpypi`.
- In GitHub under **Settings → Environments**, create `pypi` and `testpypi`.
  Adding a required reviewer to `pypi` is recommended.
