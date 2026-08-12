"""Integration tests for the command line interface."""

from __future__ import annotations

import logging
import os
import subprocess
import sys
from pathlib import Path

import pytest

from dc_motor_sim import __version__
from dc_motor_sim.cli import build_parser, main

# Inherit the real environment: a hand-built one omits variables the interpreter
# needs on Windows (SYSTEMROOT) and macOS, which would break those CI legs.
SUBPROCESS_ENV = {**os.environ, "MPLBACKEND": "Agg"}


def run_cli(*args: str) -> subprocess.CompletedProcess[str]:
    """Invoke the CLI in a subprocess, as a user would."""
    return subprocess.run(
        [sys.executable, "-m", "dc_motor_sim", *args],
        capture_output=True,
        text=True,
        env=SUBPROCESS_ENV,
        check=False,
    )


def _parse_summary(stdout: str) -> dict[str, str]:
    """Turn the printed ``key    value`` summary block into a dict."""
    rows = {}
    for line in stdout.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("=") or " " not in stripped:
            continue
        key, _, value = stripped.partition(" ")
        rows[key] = value.strip()
    return rows


class TestArgumentParsing:
    def test_defaults_to_the_fuzzy_controller(self) -> None:
        args = build_parser().parse_args(["-90", "45"])

        assert args.controller == "fuzzy"
        assert args.start == -90.0
        assert args.target == 45.0

    @pytest.mark.parametrize("flag", ["-c", "--controller"])
    @pytest.mark.parametrize("kind", ["fuzzy", "pid"])
    def test_controller_can_be_selected(self, flag: str, kind: str) -> None:
        assert build_parser().parse_args(["0", "90", flag, kind]).controller == kind

    def test_output_dir_is_a_path(self) -> None:
        args = build_parser().parse_args(["0", "90", "--output-dir", "plots"])

        assert args.output_dir == Path("plots")


class TestSuccessfulRuns:
    @pytest.mark.parametrize("kind", ["fuzzy", "pid"])
    def test_runs_and_prints_a_summary(self, capsys: pytest.CaptureFixture[str], kind: str) -> None:
        exit_code = main(["-90", "45", "-c", kind, "--no-plot", "--seed", "42"])

        out = capsys.readouterr().out
        assert exit_code == 0
        assert "Simulation summary" in out
        assert "final_position_deg" in out

    def test_seeded_runs_are_reproducible(self, capsys: pytest.CaptureFixture[str]) -> None:
        main(["-90", "45", "--no-plot", "--seed", "7"])
        first = capsys.readouterr().out
        main(["-90", "45", "--no-plot", "--seed", "7"])
        second = capsys.readouterr().out

        assert first == second

    def test_max_steps_is_honoured(self, capsys: pytest.CaptureFixture[str]) -> None:
        main(["-180", "180", "--no-plot", "--max-steps", "4", "--seed", "1"])

        summary = _parse_summary(capsys.readouterr().out)
        assert summary["steps"] == "4"
        assert summary["converged"] == "False"

    def test_verbose_emits_progress_logs(self, capsys: pytest.CaptureFixture[str]) -> None:
        main(["-90", "45", "--no-plot", "--seed", "3", "--verbose"])

        assert logging.getLogger("dc_motor_sim").level == logging.INFO
        assert "Simulation summary" in capsys.readouterr().out

    def test_verbose_still_applies_on_a_later_call(self) -> None:
        """Regression: basicConfig is a one-shot, so the level must be set directly.

        Calling main() without --verbose first used to leave the logger stuck
        at WARNING for every subsequent call.
        """
        main(["0", "5", "--no-plot", "--seed", "1"])
        assert logging.getLogger("dc_motor_sim").level == logging.WARNING

        main(["0", "5", "--no-plot", "--seed", "1", "--verbose"])
        assert logging.getLogger("dc_motor_sim").level == logging.INFO


class TestPlotting:
    def test_writes_plots_to_the_output_directory(
        self, tmp_path: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        exit_code = main(
            ["-90", "45", "-c", "pid", "--no-show", "--seed", "5", "--output-dir", str(tmp_path)]
        )

        written = sorted(p.name for p in tmp_path.glob("*.png"))
        assert exit_code == 0
        assert written == ["pid_results.png", "pid_summary.png"]
        assert all((tmp_path / name).stat().st_size > 0 for name in written)
        assert "Plots written to" in capsys.readouterr().out

    def test_fuzzy_run_also_writes_the_fuzzy_specific_plots(self, tmp_path: Path) -> None:
        main(
            [
                "-90",
                "45",
                "-c",
                "fuzzy",
                "--no-show",
                "--seed",
                "5",
                "--max-steps",
                "5",
                "--output-dir",
                str(tmp_path),
            ]
        )

        written = sorted(p.name for p in tmp_path.glob("*.png"))
        assert written == [
            "fuzzy_control_surface.png",
            "fuzzy_membership.png",
            "fuzzy_results.png",
            "fuzzy_summary.png",
        ]

    def test_no_plot_skips_rendering(self, tmp_path: Path) -> None:
        main(["-90", "45", "--no-plot", "--seed", "5", "--output-dir", str(tmp_path)])

        assert list(tmp_path.glob("*.png")) == []


class TestInvalidInputInProcess:
    """The same failures as below, exercised in-process so coverage sees them."""

    @pytest.mark.parametrize(
        ("argv", "message"),
        [
            (["0", "999", "--no-plot"], "target position 999.0 is outside the range"),
            (["-181", "0", "--no-plot"], "start position -181.0 is outside the range"),
            (["0", "90", "--ppr", "0", "--no-plot"], "ppr must be positive"),
            (["0", "90", "--noise-std", "-1", "--no-plot"], "noise_std must be non-negative"),
            (["0", "90", "--max-steps", "0", "--no-plot"], "max_steps must be positive"),
        ],
    )
    def test_exits_with_two_and_explains(
        self, capsys: pytest.CaptureFixture[str], argv: list[str], message: str
    ) -> None:
        with pytest.raises(SystemExit) as exc:
            main(argv)

        assert exc.value.code == 2
        assert message in capsys.readouterr().err


class TestInvalidInput:
    @pytest.mark.parametrize("position", ["999", "-181"])
    def test_out_of_range_position_exits_with_two(self, position: str) -> None:
        result = run_cli("0", position, "--no-plot")

        assert result.returncode == 2
        assert "outside the range" in result.stderr

    def test_invalid_controller_exits_with_two(self) -> None:
        result = run_cli("0", "90", "-c", "bogus", "--no-plot")

        assert result.returncode == 2
        assert "invalid choice" in result.stderr

    def test_non_numeric_position_exits_with_two(self) -> None:
        result = run_cli("abc", "90", "--no-plot")

        assert result.returncode == 2
        assert "invalid float value" in result.stderr

    def test_missing_arguments_exits_with_two(self) -> None:
        result = run_cli("0")

        assert result.returncode == 2
        assert "required" in result.stderr

    def test_invalid_encoder_setting_exits_with_two(self) -> None:
        result = run_cli("0", "90", "--ppr", "0", "--no-plot")

        assert result.returncode == 2
        assert "ppr must be positive" in result.stderr


class TestMetaFlags:
    def test_help_exits_cleanly(self) -> None:
        result = run_cli("--help")

        assert result.returncode == 0
        assert "dc-motor-sim" in result.stdout

    def test_version_reports_the_package_version(self) -> None:
        result = run_cli("--version")

        assert result.returncode == 0
        assert __version__ in result.stdout

    def test_module_entry_point_works(self) -> None:
        result = run_cli("-90", "45", "--no-plot", "--seed", "1")

        assert result.returncode == 0
        assert "Simulation summary" in result.stdout


class TestLegacyShim:
    def test_old_positional_form_still_runs(self) -> None:
        repo_root = Path(__file__).resolve().parents[2]
        if not (repo_root / "main.py").exists():
            pytest.skip("deprecated shim is only present in a source checkout")

        result = subprocess.run(
            [sys.executable, str(repo_root / "main.py"), "0", "90", "fuzzy", "--no-plot"],
            capture_output=True,
            text=True,
            env=SUBPROCESS_ENV,
            check=False,
        )

        assert result.returncode == 0
        assert "Simulation summary" in result.stdout
        assert "deprecated" in result.stderr
