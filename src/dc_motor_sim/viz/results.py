"""Time-series and summary plots for a completed simulation run."""

from __future__ import annotations

from pathlib import Path

from matplotlib.figure import Figure

from dc_motor_sim.simulation.result import SimulationResult
from dc_motor_sim.viz.backend import ensure_backend, finish_figure

__all__ = ["plot_final_summary", "plot_simulation_results"]


def plot_simulation_results(
    result: SimulationResult,
    *,
    save_path: Path | str | None = None,
    show: bool = True,
) -> Figure:
    """Plot position tracking, error, control signal, and the phase plane.

    Replaces the former ``plot_simulation_results`` / ``plot_pid_results``
    pair, which differed only in their titles; the controller name now comes
    from the result itself.

    Args:
        result: The completed simulation trace.
        save_path: Optional path to write the figure to.
        show: Whether to open an interactive window.

    Returns:
        The resulting figure.
    """
    ensure_backend()
    from matplotlib import pyplot as plt

    name = result.controller_name
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    axes[0, 0].plot(result.time, result.actual_position, "b-", linewidth=2, label="Actual Position")
    axes[0, 0].plot(
        result.time,
        result.measured_position,
        "c--",
        linewidth=1.5,
        alpha=0.7,
        label="Encoder Reading",
    )
    axes[0, 0].plot(result.time, result.target, "r--", linewidth=2, label="Target Position")
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Position (degrees)")
    axes[0, 0].set_title(f"Motor Position vs Target ({name} Control, Encoder Feedback)")
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    axes[0, 1].plot(result.time, result.error, "g-", linewidth=2)
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Error (degrees)")
    axes[0, 1].set_title("Position Error Over Time")
    axes[0, 1].grid(True)

    axes[1, 0].plot(result.time, result.control, "m-", linewidth=2)
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("Control Signal")
    axes[1, 0].set_title(f"Control Signal Over Time ({name} Output)")
    axes[1, 0].grid(True)

    axes[1, 1].plot(result.error, result.control, "c-", linewidth=1.5)
    axes[1, 1].set_xlabel("Error (degrees)")
    axes[1, 1].set_ylabel("Control Signal")
    axes[1, 1].set_title("Control Signal vs Error")
    axes[1, 1].grid(True)

    return finish_figure(fig, save_path, show)


def plot_final_summary(
    result: SimulationResult,
    *,
    save_path: Path | str | None = None,
    show: bool = True,
) -> Figure:
    """Plot a bar chart comparing the initial, target, and final positions.

    Args:
        result: The completed simulation trace.
        save_path: Optional path to write the figure to.
        show: Whether to open an interactive window.

    Returns:
        The resulting figure.
    """
    ensure_backend()
    from matplotlib import pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 6))

    positions = [result.start_deg, result.target_deg, result.final_position]
    labels = ["Initial\nPosition", "Target\nPosition", "Final\nPosition"]
    colors = ["blue", "red", "green"]

    bars = ax.bar(labels, positions, color=colors, alpha=0.7, edgecolor="black", linewidth=2)

    for bar, position in zip(bars, positions, strict=True):
        height = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            height,
            f"{position:.1f}\N{DEGREE SIGN}",
            ha="center",
            va="bottom" if position >= 0 else "top",
            fontsize=12,
            fontweight="bold",
        )

    outcome = "Converged" if result.converged else "Stopped"
    ax.set_ylabel("Position (degrees)", fontsize=12)
    ax.set_title(
        f"Motor Position Control Summary ({result.controller_name})\n"
        f"({outcome} in {result.steps} steps)",
        fontsize=14,
    )
    ax.grid(True, axis="y", alpha=0.3)

    low, high = min(positions), max(positions)
    margin = (high - low) * 0.2 if high != low else 10
    ax.set_ylim((low - margin, high + margin))

    return finish_figure(fig, save_path, show)
