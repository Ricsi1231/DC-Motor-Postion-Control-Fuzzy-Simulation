# Article draft and its figures

An English write-up of the project for publication, kept with the figures it uses. Unrelated to the
seminar paper under [`../paper/`](../paper/), which is Hungarian.

- [`medium-article.md`](medium-article.md) — the draft.
- `figures/` — the four images it embeds.

## Regenerating the figures

Two come from the package's own plotting helpers:

```bash
python - <<'PY'
import matplotlib; matplotlib.use("Agg")
from dc_motor_sim import FuzzyMotorController
from dc_motor_sim.viz import plot_control_surface, plot_membership_functions

d = "docs/article/figures"
plot_membership_functions(FuzzyMotorController(), save_path=f"{d}/membership_functions.png", show=False)
plot_control_surface(FuzzyMotorController(), save_path=f"{d}/control_surface.png", show=False)
PY
```

The other two compare the controllers against each other and come from
[`../../scripts/article_figures.py`](../../scripts/article_figures.py):

```bash
python scripts/article_figures.py --output-dir docs/article/figures --seeds 15
```

That script runs `5 moves x 15 seeds x 2 controllers` closed-loop simulations and takes a few
minutes; lower `--seeds` for a quicker, noisier answer.

## The measurement in the article

`convergence_rate.png` reports how often each controller reaches the convergence band
(`convergence_position` / `convergence_delta`, both 0.5°) within `max_steps` (300), over the five
moves and fifteen encoder seeds fixed in `article_figures.py`. As committed, that sweep gives
**81% for the fuzzy controller and 100% for PID**, with the fuzzy misses overshooting and stalling
roughly 5° past the setpoint.

The article attributes this to the fuzzy sets being defined in absolute degrees rather than
relative to the commanded move — the Zero set spans about ±8° whatever the move size, so a small
step starts outside it and draws near-full effort. Scaling the sets with the move is the obvious
follow-up; it would change these numbers, and the figures should be regenerated if it lands.
