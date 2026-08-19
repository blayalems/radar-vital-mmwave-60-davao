"""Resolve data files in a source checkout, frozen sidecar, or installed wheel."""

from __future__ import annotations

import sys
import sysconfig
from pathlib import Path


RUNTIME_SHARE_DIRECTORY = Path("share") / "rvt-trainer"


def runtime_root() -> Path:
    """Return the root containing dashboard, assets, quality, and firmware.

    Source checkouts and PyInstaller sidecars already place ``assets`` beside
    the package.  A wheel installs the same runtime tree below the interpreter's
    data directory at ``share/rvt-trainer``.
    """

    package_parent = Path(__file__).resolve().parent.parent
    candidates = [package_parent]
    frozen_root = getattr(sys, "_MEIPASS", None)
    if frozen_root:
        candidates.append(Path(frozen_root).resolve())
    candidates.append(
        Path(sysconfig.get_path("data")).resolve() / RUNTIME_SHARE_DIRECTORY
    )
    for candidate in candidates:
        if (candidate / "assets").is_dir():
            return candidate
    return package_parent


__all__ = ["RUNTIME_SHARE_DIRECTORY", "runtime_root"]
