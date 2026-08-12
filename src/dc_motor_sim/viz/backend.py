"""Matplotlib backend selection and figure finalisation.

The backend is chosen lazily, inside the plotting calls, never at import time.
Importing a plotting module must stay side-effect free so the package can be
imported on a headless machine (CI, a container, an SSH session). Note that
importing :mod:`matplotlib.pyplot` is itself what resolves the backend, so it
is deliberately imported inside functions here.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from matplotlib.figure import Figure

__all__ = ["ensure_backend", "finish_figure", "has_display"]

_backend_ready = False


def has_display() -> bool:
    """Return whether an interactive display is likely available."""
    if sys.platform in ("win32", "darwin"):
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def ensure_backend() -> str:
    """Select a matplotlib backend once, and return its name.

    An explicit ``MPLBACKEND`` environment variable always wins. Otherwise an
    interactive backend is kept when a display is present, and ``Agg`` is
    forced when one is not.
    """
    global _backend_ready

    import matplotlib

    if _backend_ready or os.environ.get("MPLBACKEND"):
        return str(matplotlib.get_backend())

    if not has_display():
        matplotlib.use("Agg", force=True)

    _backend_ready = True
    return str(matplotlib.get_backend())


def finish_figure(fig: Figure, save_path: Path | str | None, show: bool) -> Figure:
    """Lay out, optionally save, and optionally display a figure.

    Args:
        fig: The figure to finalise.
        save_path: Where to write the figure. Parent directories are created.
            ``None`` skips saving.
        show: Whether to open an interactive window.

    Returns:
        The same figure, so callers can inspect or further modify it.
    """
    from matplotlib import pyplot as plt

    fig.tight_layout()

    if save_path is not None:
        path = Path(save_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, bbox_inches="tight")

    if show:
        plt.show()

    return fig
