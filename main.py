#!/usr/bin/env python3
"""Deprecated entry point kept for backwards compatibility.

The simulation now ships as the installable ``dc_motor_sim`` package. Use the
``dc-motor-sim`` console script (or ``python -m dc_motor_sim``) instead::

    dc-motor-sim <start> <target> [--controller {fuzzy,pid}]

This shim translates the old positional form ``python main.py 0 90 fuzzy``
into the new flag-based interface and will be removed in a future release.
"""

from __future__ import annotations

import sys

from dc_motor_sim.cli import CONTROLLERS, main


def _translate(argv: list[str]) -> list[str]:
    """Rewrite a trailing bare controller name into ``--controller <name>``."""
    if len(argv) >= 3 and argv[2].lower() in CONTROLLERS:
        return [*argv[:2], "--controller", argv[2].lower(), *argv[3:]]
    return argv


if __name__ == "__main__":
    print(
        "main.py is deprecated; use the 'dc-motor-sim' command or "
        "'python -m dc_motor_sim' instead.",
        file=sys.stderr,
    )
    sys.exit(main(_translate(sys.argv[1:])))
