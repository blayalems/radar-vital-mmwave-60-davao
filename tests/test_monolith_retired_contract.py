"""Structural regression tests proving rvt_trainer/monolith.py is retired."""

from __future__ import annotations

import ast
import importlib
import json
import pathlib
import subprocess
import sys


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PKG_ROOT = REPO_ROOT / "rvt_trainer"
MONOLITH_PATH = PKG_ROOT / "monolith.py"

EXTRACTED_MODULES = [
    "rvt_trainer",
    "rvt_trainer.constants",
    "rvt_trainer.protocol_contract",
    "rvt_trainer.features",
    "rvt_trainer.transport.serial",
    "rvt_trainer.transport.ble",
    "rvt_trainer.audit.verdict",
    "rvt_trainer.audit.truthfulness",
    "rvt_trainer.audit.runner",
    "rvt_trainer.model_artifacts",
    "rvt_trainer.modeling",
    "rvt_trainer.session.manifest",
    "rvt_trainer.session._legacy_dashboard_template",
    "rvt_trainer.session.live_dashboard",
    "rvt_trainer.session.storage",
    "rvt_trainer.session.analysis_jobs",
    "rvt_trainer.session.commands",
    "rvt_trainer.session.supervisor",
    "rvt_trainer.api.server_info",
    "rvt_trainer.api.server",
    "rvt_trainer.api.sse",
    "rvt_trainer.cli",
    "rvt_trainer.sidecar_entry",
]


def test_monolith_is_pure_reexport_facade() -> None:
    text = MONOLITH_PATH.read_text(encoding="utf-8")
    lines = text.splitlines()
    assert len(lines) < 350, f"Expected monolith facade <350 lines, got {len(lines)}"

    tree = ast.parse(text)
    defs = [
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
    ]
    assert defs == [], f"Expected zero defs/classes in retired monolith.py, found {[d.name for d in defs]}"

    monolith = importlib.import_module("rvt_trainer.monolith")
    assert hasattr(monolith, "json")
    exported = getattr(monolith, "__all__", [])
    assert len(exported) > 200
    for symbol in exported:
        assert hasattr(monolith, symbol), f"monolith.__all__ references missing symbol {symbol!r}"


def test_extracted_submodules_import_without_monolith() -> None:
    code = (
        "import importlib, sys\n"
        f"mods = {EXTRACTED_MODULES!r}\n"
        "for mod in mods:\n"
        "    importlib.import_module(mod)\n"
        "assert 'rvt_trainer.monolith' not in sys.modules, 'submodule loaded monolith at import time'\n"
    )
    proc = subprocess.run(
        [sys.executable, "-c", code],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )
    assert proc.returncode == 0, f"Submodule clean import failed:\nSTDOUT:{proc.stdout}\nSTDERR:{proc.stderr}"


def test_no_package_submodule_imports_monolith() -> None:
    offenders: list[str] = []
    for path in sorted(PKG_ROOT.rglob("*.py")):
        if path.name in {"monolith.py", "_compat.py"}:
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and (node.module or "").endswith("monolith"):
                offenders.append(f"{path.relative_to(REPO_ROOT)}:{node.lineno}")
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    if alias.name == "rvt_trainer.monolith" or alias.name == "monolith":
                        offenders.append(f"{path.relative_to(REPO_ROOT)}:{node.lineno}")
    assert offenders == [], f"Unexpected monolith back-imports remain: {offenders}"


def test_sidecar_self_test_resolves_package_root_when_entry_script_is_top_level(
    tmp_path: pathlib.Path,
    monkeypatch,
    capsys,
) -> None:
    from rvt_trainer import sidecar_entry

    # Simulate PyInstaller running sidecar_entry.py at top-level _MEIPASS/sidecar_entry.py
    # while bundled runtime assets sit under rvt_trainer/assets/.
    monkeypatch.setattr(sidecar_entry, "__file__", str(tmp_path / "sidecar_entry.py"))
    assert sidecar_entry._sidecar_self_test() == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["ok"] is True
    assert pathlib.Path(payload["schema_path"]).is_file()

