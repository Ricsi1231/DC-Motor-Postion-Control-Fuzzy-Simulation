"""Unit tests for the plotting layer.

These run under the ``Agg`` backend (forced in ``conftest.py``). The point is
to prove the package can plot on a machine with no display — the original code
called ``matplotlib.use('TkAgg')`` at import time and always blocked on
``plt.show()``, which made it unusable in CI.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from matplotlib import pyplot as plt
from matplotlib.figure import Figure

from dc_motor_sim import EncoderParams, FuzzyMotorController, PIDMotorController, run_simulation
from dc_motor_sim.viz import backend as backend_module
from dc_motor_sim.viz import (
    compute_control_surface,
    ensure_backend,
    has_display,
    plot_control_surface,
    plot_final_summary,
    plot_membership_functions,
    plot_simulation_results,
)


@pytest.fixture(scope="module")
def result():
    """A short deterministic run to plot."""
    return run_simulation(
        PIDMotorController(),
        start_deg=-90.0,
        target_deg=45.0,
        encoder_params=EncoderParams(noise_std=0.0),
    )


@pytest.fixture(autouse=True)
def close_figures():
    yield
    plt.close("all")


@pytest.fixture
def no_show(monkeypatch: pytest.MonkeyPatch) -> list[int]:
    """Record calls to ``plt.show`` instead of opening a window."""
    calls: list[int] = []

    def record(*_args: object, **_kwargs: object) -> None:
        calls.append(1)

    monkeypatch.setattr(plt, "show", record)
    return calls


class TestBackend:
    def test_ensure_backend_returns_a_backend_name(self) -> None:
        assert isinstance(ensure_backend(), str)

    def test_explicit_mplbackend_wins(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setenv("MPLBACKEND", "Agg")
        monkeypatch.setattr(backend_module, "_backend_ready", False)

        assert ensure_backend().lower() == "agg"

    def test_has_display_reads_the_environment(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(backend_module.sys, "platform", "linux")
        monkeypatch.delenv("DISPLAY", raising=False)
        monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
        assert has_display() is False

        monkeypatch.setenv("DISPLAY", ":0")
        assert has_display() is True

    @pytest.mark.parametrize("platform", ["win32", "darwin"])
    def test_display_is_assumed_on_windows_and_macos(
        self, monkeypatch: pytest.MonkeyPatch, platform: str
    ) -> None:
        monkeypatch.setattr(backend_module.sys, "platform", platform)
        monkeypatch.delenv("DISPLAY", raising=False)

        assert has_display() is True

    def test_selection_latches_after_the_first_call(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """Selecting twice could force a switch under already-open figures."""
        monkeypatch.setenv("MPLBACKEND", "Agg")
        monkeypatch.setattr(backend_module, "_backend_ready", False)
        ensure_backend()

        assert backend_module._backend_ready is True

    @pytest.mark.skipif(
        sys.platform in ("win32", "darwin"),
        reason="a display is always assumed on Windows and macOS",
    )
    def test_falls_back_to_agg_with_no_display(self) -> None:
        """The branch that matters in CI, exercised in a truly headless process.

        It cannot be tested in-process: the backend is global, latched, and
        already resolved by the time this test runs.
        """
        script = (
            "from dc_motor_sim.viz.backend import ensure_backend, has_display;"
            "print(has_display(), ensure_backend())"
        )
        env = {
            k: v
            for k, v in os.environ.items()
            if k not in ("DISPLAY", "WAYLAND_DISPLAY", "MPLBACKEND")
        }

        result = subprocess.run(
            [sys.executable, "-c", script],
            capture_output=True,
            text=True,
            env=env,
            check=True,
        )

        display, chosen = result.stdout.split()
        assert display == "False"
        assert chosen.lower() == "agg"


class TestPlotsReturnFigures:
    def test_membership_functions(
        self, fresh_fuzzy_controller: FuzzyMotorController, no_show: list[int]
    ) -> None:
        fig = plot_membership_functions(fresh_fuzzy_controller, show=False)

        assert isinstance(fig, Figure)
        assert len(fig.axes) == 3
        assert no_show == []

    def test_simulation_results(self, result, no_show: list[int]) -> None:
        fig = plot_simulation_results(result, show=False)

        assert isinstance(fig, Figure)
        assert len(fig.axes) == 4
        assert no_show == []

    def test_final_summary(self, result, no_show: list[int]) -> None:
        fig = plot_final_summary(result, show=False)

        assert isinstance(fig, Figure)
        assert no_show == []

    def test_control_surface(
        self, fresh_fuzzy_controller: FuzzyMotorController, no_show: list[int]
    ) -> None:
        fig = plot_control_surface(fresh_fuzzy_controller, grid_size=5, show=False)

        assert isinstance(fig, Figure)
        assert no_show == []


class TestSaving:
    @pytest.mark.parametrize("name", ["results", "summary"])
    def test_writes_a_non_empty_png(self, result, tmp_path: Path, name: str) -> None:
        plotter = {"results": plot_simulation_results, "summary": plot_final_summary}[name]
        target = tmp_path / "nested" / f"{name}.png"

        plotter(result, save_path=target, show=False)

        assert target.exists()
        assert target.stat().st_size > 0

    def test_creates_missing_parent_directories(self, result, tmp_path: Path) -> None:
        target = tmp_path / "a" / "b" / "c" / "plot.png"

        plot_final_summary(result, save_path=target, show=False)

        assert target.exists()

    def test_show_opens_a_window_when_requested(self, result, no_show: list[int]) -> None:
        plot_final_summary(result, show=True)

        assert no_show == [1]


class TestControlSurface:
    def test_grid_shape_matches_the_requested_size(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        x, y, z = compute_control_surface(fresh_fuzzy_controller, grid_size=6)

        assert x.shape == y.shape == z.shape == (6, 6)

    def test_surface_stays_within_the_control_universe(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        low, high = fresh_fuzzy_controller.params.control_range
        _, _, z = compute_control_surface(fresh_fuzzy_controller, grid_size=5)

        assert np.all(z >= low)
        assert np.all(z <= high)

    def test_surface_increases_with_error(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        x, _, z = compute_control_surface(fresh_fuzzy_controller, grid_size=5)

        # Along a row, error increases left to right, so control should too.
        assert z[0, 0] < z[0, -1]
        assert x[0, 0] < x[0, -1]

    @pytest.mark.parametrize("grid_size", [0, 1, -3])
    def test_rejects_a_degenerate_grid(
        self, fresh_fuzzy_controller: FuzzyMotorController, grid_size: int
    ) -> None:
        with pytest.raises(ValueError, match="grid_size must be at least 2"):
            compute_control_surface(fresh_fuzzy_controller, grid_size=grid_size)

    def test_evaluation_does_not_disturb_controller_state(
        self, fresh_fuzzy_controller: FuzzyMotorController
    ) -> None:
        fresh_fuzzy_controller.set_target(45.0)
        fresh_fuzzy_controller.compute(0.0, dt=0.001)
        integral_before = fresh_fuzzy_controller.integral

        compute_control_surface(fresh_fuzzy_controller, grid_size=3)

        assert fresh_fuzzy_controller.integral == integral_before

    @pytest.mark.slow
    def test_full_resolution_surface(self, fresh_fuzzy_controller: FuzzyMotorController) -> None:
        _, _, z = compute_control_surface(fresh_fuzzy_controller)

        assert z.shape == (30, 30)
