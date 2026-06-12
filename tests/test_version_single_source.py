"""Tests that the project version is consistent across all artefacts.

Single-source-of-truth: pyproject.toml ``[project].version``.
Consumer artefacts: rvt_trainer.monolith.VERSION, root package.json.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent


# ── helpers ──────────────────────────────────────────────────────────────

def _read_pyproject_version() -> str:
    """Return the version string from pyproject.toml."""
    pp = REPO_ROOT / "pyproject.toml"
    if not pp.exists():
        pytest.skip("pyproject.toml not found at repo root")
    if sys.version_info >= (3, 11):
        import tomllib
    else:
        try:
            import tomllib  # type: ignore[import-not-found]
        except ModuleNotFoundError:
            import tomli as tomllib  # type: ignore[no-redef]
    with open(pp, "rb") as f:
        data = tomllib.load(f)
    return data["project"]["version"]


def _read_package_json_version() -> str:
    """Return the version string from the root package.json."""
    pj = REPO_ROOT / "package.json"
    if not pj.exists():
        pytest.skip("package.json not found at repo root")
    return json.loads(pj.read_text(encoding="utf-8"))["version"]


# ── tests ────────────────────────────────────────────────────────────────

class TestVersionSingleSource:
    """Ensure all version declarations agree with pyproject.toml."""

    def test_monolith_version_matches_pyproject(self) -> None:
        """rvt_trainer.monolith.VERSION must equal pyproject.toml version."""
        from rvt_trainer.monolith import VERSION

        expected = _read_pyproject_version()
        assert VERSION == expected, (
            f"monolith.VERSION={VERSION!r} != pyproject.toml version={expected!r}"
        )

    def test_package_json_version_matches_pyproject(self) -> None:
        """Root package.json version must equal pyproject.toml version."""
        expected = _read_pyproject_version()
        actual = _read_package_json_version()
        assert actual == expected, (
            f"package.json version={actual!r} != pyproject.toml version={expected!r}"
        )

    def test_monolith_version_is_semver_like(self) -> None:
        """VERSION should look like X.Y.Z (three integer segments)."""
        from rvt_trainer.monolith import VERSION

        parts = VERSION.split(".")
        assert len(parts) == 3, f"Expected 3-part semver, got {VERSION!r}"
        for part in parts:
            assert part.isdigit(), f"Non-numeric segment {part!r} in {VERSION!r}"
