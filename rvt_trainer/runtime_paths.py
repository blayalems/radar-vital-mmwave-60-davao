"""Resolve data files in a source checkout, frozen sidecar, or installed wheel."""

from __future__ import annotations

import sys
from pathlib import Path


RUNTIME_PACKAGE_DIRECTORY = Path("assets") / "runtime"
# Kept for external callers referencing the legacy constant name.
RUNTIME_SHARE_DIRECTORY = RUNTIME_PACKAGE_DIRECTORY


def runtime_root() -> Path:
    """Return the root containing dashboard, assets, config, and quality.

    Resolution order:
    1. Source checkout: ``package_parent`` when both ``assets/`` and
       ``config/protocol_contract.json`` exist alongside ``rvt_trainer/``.
    2. PyInstaller sidecar: ``sys._MEIPASS`` when bundled and ``assets/`` exists.
    3. Installed wheel: ``rvt_trainer/assets/runtime/`` staged via package data.
    """

    package_dir = Path(__file__).resolve().parent
    package_parent = package_dir.parent
    if (package_parent / "assets").is_dir() and (
        package_parent / "config" / "protocol_contract.json"
    ).is_file():
        return package_parent

    frozen_root = getattr(sys, "_MEIPASS", None)
    if frozen_root:
        frozen_path = Path(frozen_root).resolve()
        if (frozen_path / "assets").is_dir():
            return frozen_path

    packaged_runtime = package_dir / RUNTIME_PACKAGE_DIRECTORY
    if (packaged_runtime / "assets").is_dir():
        return packaged_runtime

    return package_parent


__all__ = ["RUNTIME_PACKAGE_DIRECTORY", "RUNTIME_SHARE_DIRECTORY", "runtime_root"]
