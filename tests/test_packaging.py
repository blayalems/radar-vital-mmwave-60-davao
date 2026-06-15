"""Packaging smoke tests.

The declaration tests parse pyproject.toml directly so they pass in CI,
where the package is exercised from the working tree without `pip install`.
The installed-metadata test only runs when rvt-trainer is actually
installed (e.g. after `pip install -e .`).
"""

from __future__ import annotations

import importlib.metadata
import json
import re
import subprocess
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
PYPROJECT = ROOT / "pyproject.toml"
TAURI_CONF = ROOT / "src-tauri" / "tauri.conf.json"
TAURI_ICON = ROOT / "src-tauri" / "icons" / "icon.ico"
TERMS = ROOT / "TERMS.md"
WINDOWS_EULA = ROOT / "packaging" / "windows" / "eula.rtf"
RELEASE_WORKFLOW = ROOT / ".github" / "workflows" / "release-artifacts.yml"
MANIFEST_GENERATOR = ROOT / "scripts" / "generate-rvt-latest.mjs"
VERIFY_RELEASE_SCRIPT = ROOT / "scripts" / "verify-release-artifacts.ps1"
EXPECTED_PUBLISHER = "Lemuel Blaya, Angelo Diaz, Blessie Mugat \u2014 University of Mindanao"
EXPECTED_COPYRIGHT = (
    "Copyright (c) 2026 Lemuel Blaya, Angelo Diaz, Blessie Mugat. All rights reserved."
)


def _project() -> dict:
    with PYPROJECT.open("rb") as handle:
        return tomllib.load(handle)["project"]


def _tauri_conf() -> dict:
    return json.loads(TAURI_CONF.read_text(encoding="utf-8"))


def _rtf_text(path: Path) -> str:
    raw = path.read_text(encoding="utf-8")
    text = re.sub(r"\\'[0-9a-fA-F]{2}", " ", raw)
    text = re.sub(r"\\par[d]?", "\n", text)
    text = re.sub(r"\\[a-zA-Z]+-?\d* ?", "", text)
    text = text.replace("{", " ").replace("}", " ")
    return re.sub(r"\s+", " ", text).strip()


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


def test_tauri_windows_installer_identity_is_configured() -> None:
    bundle = _tauri_conf()["bundle"]
    assert bundle["publisher"] == EXPECTED_PUBLISHER
    assert bundle["copyright"] == EXPECTED_COPYRIGHT

    license_file = bundle["licenseFile"]
    assert license_file == "../packaging/windows/eula.rtf"
    assert (TAURI_CONF.parent / license_file).resolve() == WINDOWS_EULA.resolve()
    assert WINDOWS_EULA.exists(), "NSIS EULA file referenced by Tauri config is missing"

    installer_icon = bundle["windows"]["nsis"]["installerIcon"]
    assert installer_icon == "icons/icon.ico"
    assert (TAURI_CONF.parent / installer_icon).resolve() == TAURI_ICON.resolve()
    assert TAURI_ICON.exists(), "NSIS installer icon is missing"


def test_windows_eula_tracks_terms_anchors() -> None:
    terms = re.sub(r"\s+", " ", TERMS.read_text(encoding="utf-8").lower())
    eula = _rtf_text(WINDOWS_EULA).lower()
    for anchor in (
        "draft 2026-06-12.1",
        "lemuel blaya",
        "angelo diaz",
        "blessie mugat",
        "university of mindanao",
        "republic act no. 10173",
        "not a medical device",
        "academic evaluation",
        "as is",
    ):
        assert anchor in terms, f"TERMS.md missing expected anchor: {anchor}"
        assert anchor in eula, f"Windows EULA missing expected Terms anchor: {anchor}"


def test_release_workflow_builds_signed_aab_and_keeps_windows_fallbacks() -> None:
    workflow = RELEASE_WORKFLOW.read_text(encoding="utf-8")
    assert "./gradlew assembleRelease bundleRelease" in workflow
    assert "radar-vital-release.aab" in workflow
    assert "dist/*.aab" in workflow
    # The AAB zip is verified via hashtable splatting into verify-release-artifacts.ps1
    # (array splatting passed names positionally and broke the binding).
    assert "$verifyArgs['AabZip'] = $aabZip" in workflow

    azure_sign = workflow.index("Sign EXE with Azure Trusted Signing")
    pfx_sign = workflow.index("Sign EXE with PFX fallback")
    updater_resign = workflow.index("Re-sign Tauri updater artifact after final EXE bytes")
    assert azure_sign < pfx_sign < updater_resign
    assert "azure/artifact-signing-action@v2" in workflow
    assert "AZURE_TRUSTED_SIGNING_ENDPOINT" in workflow
    assert "Azure Trusted Signing is partially configured" in workflow
    assert "WINDOWS_CERTIFICATE_BASE64 not configured; leaving EXE unsigned." in workflow


def test_release_manifest_generator_self_test_covers_aab() -> None:
    script = MANIFEST_GENERATOR.read_text(encoding="utf-8")
    assert "radar-vital-release.aab" in script
    assert "kind: 'aab'" in script

    result = subprocess.run(
        ["node", str(MANIFEST_GENERATOR), "--self-test"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "Self-test validation passed successfully!" in result.stdout


def test_release_verifier_accepts_optional_aab_zip() -> None:
    script = VERIFY_RELEASE_SCRIPT.read_text(encoding="utf-8")
    assert "[string]$AabZip" in script
    assert "Extension 'aab'" in script
