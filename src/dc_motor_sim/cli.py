"""Command line interface for the DC motor position control simulation."""

from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

from dc_motor_sim import __version__
from dc_motor_sim.config import (
    DEFAULT_ENCODER,
    DEFAULT_SIM,
    SimParams,
)
from dc_motor_sim.control.base import PositionController
from dc_motor_sim.control.fuzzy import FuzzyMotorController
from dc_motor_sim.control.pid import PIDMotorController
from dc_motor_sim.simulation.result import SimulationResult
from dc_motor_sim.simulation.runner import run_simulation

__all__ = ["build_parser", "main"]

logger = logging.getLogger("dc_motor_sim")

CONTROLLERS = ("fuzzy", "pid")


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser for the ``dc-motor-sim`` command."""
    parser = argparse.ArgumentParser(
        prog="dc-motor-sim",
        description=(
            "Simulate closed-loop DC motor position control with encoder feedback, "
            "using either a fuzzy logic or a PID controller."
        ),
        epilog="Example: dc-motor-sim -90 45 --controller pid --no-show --output-dir plots",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("start", type=float, help="initial motor position in degrees")
    parser.add_argument("target", type=float, help="target motor position in degrees")
    parser.add_argument(
        "-c",
        "--controller",
        choices=CONTROLLERS,
        default="fuzzy",
        help="control strategy to simulate",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        metavar="DIR",
        help="write plots as PNG files into this directory",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="do not open interactive plot windows (implied when there is no display)",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="skip plotting entirely and print only the numeric summary",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        metavar="N",
        help="seed for encoder noise, making the run reproducible",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=DEFAULT_SIM.max_steps,
        metavar="N",
        help="maximum number of control steps",
    )
    parser.add_argument(
        "--noise-std",
        type=float,
        default=DEFAULT_ENCODER.noise_std,
        metavar="DEG",
        help="encoder measurement noise standard deviation in degrees",
    )
    parser.add_argument(
        "--ppr",
        type=int,
        default=DEFAULT_ENCODER.ppr,
        metavar="N",
        help="encoder resolution in pulses per revolution",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="log every simulation progress line",
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")

    return parser


def _make_controller(kind: str, sim_params: SimParams) -> PositionController:
    if kind == "fuzzy":
        return FuzzyMotorController(sim_params=sim_params)
    return PIDMotorController()


def _configure_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.INFO if verbose else logging.WARNING,
        format="%(message)s",
        stream=sys.stderr,
    )


def main(argv: list[str] | None = None) -> int:
    """Run the simulation from command line arguments.

    Args:
        argv: Argument list, defaulting to :data:`sys.argv`.

    Returns:
        ``0`` on success. Invalid arguments exit with status ``2`` via argparse.
    """
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        sim_params = replace(DEFAULT_SIM, max_steps=args.max_steps)
        encoder_params = replace(DEFAULT_ENCODER, ppr=args.ppr, noise_std=args.noise_std)
    except ValueError as exc:
        parser.error(str(exc))

    for name, value in (("start", args.start), ("target", args.target)):
        if not sim_params.is_position_valid(value):
            low, high = sim_params.position_limits
            parser.error(f"{name} position {value} is outside the range [{low}, {high}] degrees")

    _configure_logging(args.verbose)

    controller = _make_controller(args.controller, sim_params)
    rng = np.random.default_rng(args.seed)

    result = run_simulation(
        controller,
        args.start,
        args.target,
        sim_params=sim_params,
        encoder_params=encoder_params,
        rng=rng,
    )

    _print_summary(result)

    if not args.no_plot:
        _render_plots(result, controller, args)

    return 0


def _print_summary(result: SimulationResult) -> None:
    """Print the run's scalar metrics to stdout."""
    summary = result.summary()
    width = max(len(key) for key in summary)
    print(f"\n{'=' * 60}\nSimulation summary\n{'=' * 60}")
    for key, value in summary.items():
        rendered = f"{value:.4f}" if isinstance(value, float) else value
        print(f"  {key:<{width}}  {rendered}")
    print("=" * 60)


def _render_plots(
    result: SimulationResult,
    controller: PositionController,
    args: argparse.Namespace,
) -> None:
    """Draw the plots for a finished run, saving them when asked to."""
    from dc_motor_sim.viz import (
        has_display,
        plot_control_surface,
        plot_final_summary,
        plot_membership_functions,
        plot_simulation_results,
    )

    show = not args.no_show and has_display()
    out: Path | None = args.output_dir

    def target(name: str) -> Path | None:
        return None if out is None else out / f"{args.controller}_{name}.png"

    if isinstance(controller, FuzzyMotorController):
        plot_membership_functions(controller, save_path=target("membership"), show=show)
        plot_control_surface(controller, save_path=target("control_surface"), show=show)

    plot_simulation_results(result, save_path=target("results"), show=show)
    plot_final_summary(result, save_path=target("summary"), show=show)

    if out is not None:
        print(f"\nPlots written to {out.resolve()}")


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
