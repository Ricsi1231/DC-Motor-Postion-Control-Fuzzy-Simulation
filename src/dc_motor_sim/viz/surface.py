"""3D control-surface plot for the fuzzy controller."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from matplotlib.figure import Figure
from skfuzzy import control as ctrl

from dc_motor_sim.viz.backend import ensure_backend, finish_figure

if TYPE_CHECKING:
    from dc_motor_sim.control.fuzzy import FuzzyMotorController

__all__ = ["compute_control_surface", "plot_control_surface"]

DEFAULT_GRID_SIZE = 30


def compute_control_surface(
    controller: FuzzyMotorController,
    grid_size: int = DEFAULT_GRID_SIZE,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate the fuzzy inference output over an error x delta-error grid.

    Cost grows as ``grid_size²`` inference calls, so keep the grid small in
    tests. Uses a dedicated simulation object to avoid disturbing the
    controller's own inference state.

    Args:
        controller: The fuzzy controller to evaluate.
        grid_size: Number of samples per axis.

    Returns:
        The ``(x, y, z)`` meshgrid arrays, each of shape ``(grid_size, grid_size)``.

    Raises:
        ValueError: If ``grid_size`` is less than 2.
    """
    if grid_size < 2:
        raise ValueError(f"grid_size must be at least 2, got {grid_size!r}")

    params = controller.params
    error_low, error_high = params.error_range
    delta_low, delta_high = params.delta_error_range

    # The universes are numpy.arange-style, so their upper bound is exclusive.
    x, y = np.meshgrid(
        np.linspace(error_low, error_high - 1, grid_size),
        np.linspace(delta_low, delta_high - 1, grid_size),
    )
    z = np.zeros_like(x, dtype=float)

    simulation = ctrl.ControlSystemSimulation(controller.control_system)
    for i in range(x.shape[0]):
        for j in range(x.shape[1]):
            simulation.input["error"] = x[i, j]
            simulation.input["delta_error"] = y[i, j]
            simulation.compute()
            z[i, j] = simulation.output["control"]

    return x, y, z


def plot_control_surface(
    controller: FuzzyMotorController,
    *,
    grid_size: int = DEFAULT_GRID_SIZE,
    save_path: Path | str | None = None,
    show: bool = True,
) -> Figure:
    """Plot the fuzzy control surface as a 3D mesh.

    Args:
        controller: The fuzzy controller to evaluate.
        grid_size: Samples per axis; the default 30 means 900 inference calls.
        save_path: Optional path to write the figure to.
        show: Whether to open an interactive window.

    Returns:
        The resulting figure.
    """
    ensure_backend()
    from matplotlib import pyplot as plt

    x, y, z = compute_control_surface(controller, grid_size)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(
        x, y, z, rstride=1, cstride=1, cmap="viridis", linewidth=0.4, antialiased=True
    )

    ax.set_xlabel("Error (degrees)")
    ax.set_ylabel("Delta Error (degrees/step)")
    ax.set_zlabel("Control Signal")
    ax.set_title("Fuzzy Control Surface")
    ax.view_init(30, 200)
    fig.colorbar(surf)

    return finish_figure(fig, save_path, show)
