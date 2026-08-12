"""Entry point for ``python -m dc_motor_sim``."""

from __future__ import annotations

import sys

from dc_motor_sim.cli import main

if __name__ == "__main__":
    sys.exit(main())
