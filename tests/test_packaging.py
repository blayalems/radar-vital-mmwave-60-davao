"""Packaging smoke tests.

The declaration tests parse pyproject.toml directly so they pass in CI,
where the package is exercised from the working tree without `pip install`.
The installed-metadata test only runs when rvt-trainer is actually
installed (e.g. after `pip install -e .`).
"""

from __future__ import annotations

import importlib.metadata
import tomllib
from pathlib import Path

import pytest

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"


def _project() -> dict:
    with PYPROJECT.open("rb") as handle:
        return tomllib.load(handle)["project"]


def test_import_rvt_trainer() -> None:
    import rvt_trainer  # noqa: F401 — import is the test


def test_console_script_declared() -> None:
    scripts = _project().get("scripts", {})
    assert scripts.get("rvt-trainer") == "rvt_trainer.cli:main", (
        f"Unexpected console-script declaration: {scripts!r}"
    )


def test_declared_version_is_semantic() -> None:
    version = _project().get("version", "")
    assert version and version[0].isdigit(), f"Unexpected version: {version!r}"


def test_ble_extra_declares_bleak() -> None:
    extras = _project().get("optional-dependencies", {})
    assert any(dep.startswith("bleak") for dep in extras.get("ble", [])), (
        "transport/ble.py uses bleak; the [ble] extra must declare it"
    )


def test_installed_entry_point_matches_declaration() -> None:
    try:
        importlib.metadata.version("rvt-trainer")
    except importlib.metadata.PackageNotFoundError:
        pytest.skip("rvt-trainer is not pip-installed in this environment")
    eps = importlib.metadata.entry_points(group="console_scripts")
    ep = next((e for e in eps if e.name == "rvt-trainer"), None)
    assert ep is not None, "rvt-trainer console script missing from installed metadata"
    assert ep.value == "rvt_trainer.cli:main"
