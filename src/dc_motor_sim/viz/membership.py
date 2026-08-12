"""Membership-function plots for the fuzzy controller."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from matplotlib.axes import Axes
from matplotlib.figure import Figure

from dc_motor_sim.viz.backend import ensure_backend, finish_figure

if TYPE_CHECKING:
    from dc_motor_sim.control.fuzzy import FuzzyMotorController

__all__ = ["plot_membership_functions"]

TERM_COLORS = {"N": "#E74C3C", "Z": "#27AE60", "P": "#3498DB"}
TERM_LABELS = {"N": "Negative (N)", "Z": "Zero (Z)", "P": "Positive (P)"}


def _draw_variable(ax: Axes, variable: object, xlabel: str, title: str) -> None:
    """Draw every term of one fuzzy variable onto a single axis."""
    universe = variable.universe  # type: ignore[attr-defined]

    for term in ("N", "Z", "P"):
        if term not in variable.terms:  # type: ignore[attr-defined]
            continue
        mf = variable[term].mf  # type: ignore[index]
        ax.fill_between(universe, mf, alpha=0.3, color=TERM_COLORS[term])
        ax.plot(universe, mf, linewidth=2.5, color=TERM_COLORS[term], label=TERM_LABELS[term])

    ax.set_xlabel(xlabel, fontsize=11)
    ax.set_ylabel("Membership Degree", fontsize=11)
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.legend(loc="upper right", fontsize=10)
    ax.set_ylim(-0.05, 1.1)
    ax.set_xlim(universe.min(), universe.max())
    ax.axhline(y=0, color="black", linewidth=0.5)
    ax.axhline(y=1, color="gray", linewidth=0.5, linestyle="--", alpha=0.5)
    ax.grid(True, alpha=0.3)


def plot_membership_functions(
    controller: FuzzyMotorController,
    *,
    save_path: Path | str | None = None,
    show: bool = True,
) -> Figure:
    """Plot the error, delta-error, and control membership functions.

    Args:
        controller: The fuzzy controller whose variables should be drawn.
        save_path: Optional path to write the figure to.
        show: Whether to open an interactive window.

    Returns:
        The resulting figure.
    """
    ensure_backend()
    from matplotlib import pyplot as plt

    error, delta_error, control = controller.get_membership_functions()

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    _draw_variable(axes[0], error, "Error (degrees)", "Error Membership Functions")
    _draw_variable(
        axes[1],
        delta_error,
        "Delta Error (degrees/step)",
        "Delta Error Membership Functions",
    )
    _draw_variable(axes[2], control, "Control Signal", "Control Signal Membership Functions")

    return finish_figure(fig, save_path, show)
