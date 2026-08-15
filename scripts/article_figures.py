#!/usr/bin/env python3
"""Render the two comparison figures used by the article write-up.

The package already ships plots for the fuzzy design itself (membership
functions, control surface). This script adds the two figures that compare the
controllers against each other, both produced from real simulation runs:

- ``fuzzy_vs_pid_trace.png`` — shaft position over time for one move, both
  controllers, same encoder seed.
- ``convergence_rate.png`` — how often each controller reaches the convergence
  band within ``max_steps``, over a sweep of moves and encoder seeds.

The second one takes a few minutes: it runs ``moves x seeds x 2`` closed-loop
simulations. Lower ``--seeds`` for a quicker, noisier answer.

Usage::

    python scripts/article_figures.py --output-dir docs/article/figures
    python scripts/article_figures.py --output-dir /tmp/figs --seeds 5
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
import numpy as np

from dc_motor_sim import FuzzyMotorController, PIDMotorController, run_simulation

SERIES_FUZZY = "#2a78d6"
SERIES_PID = "#eb6834"
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#8a8880"
SURFACE = "#fcfcfb"

TRACE_MOVE = (-90.0, 45.0)
TRACE_SEED = 42
SWEEP_MOVES = ((-90.0, 45.0), (0.0, 90.0), (0.0, 10.0), (120.0, -120.0), (0.0, 175.0))


def _style_axes(ax: plt.Axes) -> None:
    """Apply the shared recessive grid and spine treatment."""
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=INK_MUTED, alpha=0.22, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK_MUTED)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=INK_SECONDARY, labelsize=9)


def plot_trace(output: Path) -> None:
    """Draw both controllers' position traces for one move and encoder seed."""
    start, target = TRACE_MOVE
    runs = {}
    for label, controller in (("Fuzzy", FuzzyMotorController), ("PID", PIDMotorController)):
        runs[label] = run_simulation(
            controller(), start, target, rng=np.random.default_rng(TRACE_SEED)
        )

    fig, ax = plt.subplots(figsize=(10, 5.2))
    fig.patch.set_facecolor(SURFACE)
    _style_axes(ax)

    ax.axhline(target, color=INK_MUTED, linewidth=1.2, linestyle="--", zorder=1)
    ax.annotate(
        f"setpoint {target:g}°",
        xy=(0.012, target),
        xycoords=("axes fraction", "data"),
        ha="left",
        va="bottom",
        fontsize=9,
        color=INK_SECONDARY,
    )

    for label, color in (("Fuzzy", SERIES_FUZZY), ("PID", SERIES_PID)):
        result = runs[label]
        ax.plot(
            result.time * 1000.0,
            result.actual_position,
            color=color,
            linewidth=2.0,
            label=label,
            zorder=3,
        )
        ax.annotate(
            f"{label} — {result.steps} steps",
            xy=(result.time[-1] * 1000.0, result.actual_position[-1]),
            xytext=(8, -4 if label == "PID" else 8),
            textcoords="offset points",
            fontsize=10,
            color=INK_PRIMARY,
            va="center",
        )
        ax.plot(
            [result.time[-1] * 1000.0],
            [result.actual_position[-1]],
            marker="o",
            markersize=8,
            color=color,
            markeredgecolor=SURFACE,
            markeredgewidth=2.0,
            zorder=4,
        )

    ax.set_xlabel("time (ms)", fontsize=10, color=INK_SECONDARY)
    ax.set_ylabel("shaft position (degrees)", fontsize=10, color=INK_SECONDARY)
    ax.set_title(
        f"Position response, {start:g}° → {target:g}°, identical encoder noise",
        fontsize=13,
        color=INK_PRIMARY,
        pad=12,
        loc="left",
    )
    ax.set_xlim(left=0)
    ax.margins(x=0.14)
    legend = ax.legend(frameon=False, fontsize=10, loc="lower right")
    for text in legend.get_texts():
        text.set_color(INK_PRIMARY)

    fig.tight_layout()
    fig.savefig(output, dpi=200, facecolor=SURFACE)
    plt.close(fig)
    print(f"OK    {output}")


def plot_convergence(output: Path, seeds: int) -> None:
    """Draw how often each controller converges, per move, over ``seeds`` runs."""
    labels = []
    rates: dict[str, list[float]] = {"Fuzzy": [], "PID": []}
    for start, target in SWEEP_MOVES:
        labels.append(f"{start:g}° → {target:g}°")
        for name, controller in (("Fuzzy", FuzzyMotorController), ("PID", PIDMotorController)):
            converged = sum(
                run_simulation(
                    controller(), start, target, rng=np.random.default_rng(seed)
                ).converged
                for seed in range(seeds)
            )
            rates[name].append(100.0 * converged / seeds)

    positions = np.arange(len(labels))
    width = 0.34

    fig, ax = plt.subplots(figsize=(10, 5.2))
    fig.patch.set_facecolor(SURFACE)
    _style_axes(ax)
    ax.grid(axis="x", visible=False)

    for offset, (name, color) in zip(
        (-width / 2 - 0.02, width / 2 + 0.02),
        (("Fuzzy", SERIES_FUZZY), ("PID", SERIES_PID)),
        strict=True,
    ):
        bars = ax.bar(
            positions + offset,
            rates[name],
            width,
            label=name,
            color=color,
            zorder=3,
        )
        for bar, value in zip(bars, rates[name], strict=True):
            ax.annotate(
                f"{value:.0f}%",
                xy=(bar.get_x() + bar.get_width() / 2, value),
                xytext=(0, 4),
                textcoords="offset points",
                ha="center",
                fontsize=9,
                color=INK_PRIMARY,
            )

    ax.set_xticks(positions)
    ax.set_xticklabels(labels, fontsize=10, color=INK_SECONDARY)
    ax.set_ylabel("runs that converged (%)", fontsize=10, color=INK_SECONDARY)
    ax.set_ylim(0, 112)
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_title(
        f"Convergence rate over {seeds} encoder seeds per move",
        fontsize=13,
        color=INK_PRIMARY,
        pad=28,
        loc="left",
    )
    legend = ax.legend(
        frameon=False,
        fontsize=10,
        loc="lower left",
        bbox_to_anchor=(0.0, 1.005),
        ncol=2,
    )
    for text in legend.get_texts():
        text.set_color(INK_PRIMARY)

    fig.tight_layout()
    fig.savefig(output, dpi=200, facecolor=SURFACE)
    plt.close(fig)
    print(f"OK    {output}")


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse the output directory and sweep size."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("docs/article/figures"),
        help="directory the PNGs are written into",
    )
    parser.add_argument(
        "--seeds",
        type=int,
        default=15,
        help="encoder seeds per move for the convergence sweep",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Render both figures into the output directory."""
    args = _parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plot_trace(args.output_dir / "fuzzy_vs_pid_trace.png")
    plot_convergence(args.output_dir / "convergence_rate.png", args.seeds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
