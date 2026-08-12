"""Matplotlib visualisations.

Importing this package does not select a matplotlib backend; the backend is
resolved lazily when a plotting function is first called.
"""

from dc_motor_sim.viz.backend import ensure_backend, has_display
from dc_motor_sim.viz.membership import plot_membership_functions
from dc_motor_sim.viz.results import plot_final_summary, plot_simulation_results
from dc_motor_sim.viz.surface import compute_control_surface, plot_control_surface

__all__ = [
    "compute_control_surface",
    "ensure_backend",
    "has_display",
    "plot_control_surface",
    "plot_final_summary",
    "plot_membership_functions",
    "plot_simulation_results",
]
