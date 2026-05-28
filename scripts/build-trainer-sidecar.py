#!/usr/bin/env python3
"""Build the Radar Vital Python trainer as a Tauri sidecar binary.

Tauri v2 expects external binaries to be named with the Rust target triple:

    src-tauri/binaries/rvt-trainer-x86_64-pc-windows-msvc.exe

This script keeps that convention out of CI YAML and local release notes. It is
Windows-first because the standalone operator console target is the Tauri/NSIS
EXE; non-Windows hosts may still run it for dry-run diagnostics, but only a
Windows build produces the supported sidecar artifact.
"""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ENTRYPOINT = ROOT / "radar_vital_trainer_v12_for_v16_0.py"
BIN_DIR = ROOT / "src-tauri" / "binaries"
WORK_DIR = ROOT / ".artifact-check" / "pyinstaller-work"
SPEC_DIR = ROOT / ".artifact-check" / "pyinstaller-spec"

ADD_DATA = [
    ROOT / "assets",
    ROOT / "web",
    ROOT / "www",
    ROOT / "radar_vital_v16_0_0.ino",
    ROOT / "requirements-v12.txt",
]

COLLECT_ALL = [
    "matplotlib",
    "numpy",
    "pandas",
    "sklearn",
]

HIDDEN_IMPORTS = [
    "serial",
    "serial.tools.list_ports",
    "sklearn.ensemble._gb",
    "sklearn.tree._tree",
]


def run(cmd: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(str(part) for part in cmd), flush=True)
    return subprocess.run(cmd, cwd=ROOT, text=True, check=check)


def rust_target_triple() -> str:
    env_target = os.environ.get("RVT_TAURI_TARGET_TRIPLE") or os.environ.get("TARGET")
    if env_target:
        return env_target.strip()
    try:
        proc = subprocess.run(
            ["rustc", "-vV"],
            cwd=ROOT,
            text=True,
            check=True,
            capture_output=True,
        )
        for line in proc.stdout.splitlines():
            if line.startswith("host: "):
                return line.split(":", 1)[1].strip()
    except Exception as exc:
        print(f"[WARN] Could not query rustc target triple: {exc}", file=sys.stderr)
    if platform.system().lower() == "windows":
        return "x86_64-pc-windows-msvc"
    if platform.system().lower() == "darwin":
        return "aarch64-apple-darwin" if platform.machine().lower() in {"arm64", "aarch64"} else "x86_64-apple-darwin"
    return "x86_64-unknown-linux-gnu"


def target_filename(triple: str) -> str:
    suffix = ".exe" if "windows" in triple else ""
    return f"rvt-trainer-{triple}{suffix}"


def pyinstaller_command(target_name: str, onefile: bool) -> list[str]:
    cmd = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--clean",
        "--console",
        "--name",
        target_name,
        "--distpath",
        str(BIN_DIR),
        "--workpath",
        str(WORK_DIR),
        "--specpath",
        str(SPEC_DIR),
    ]
    if onefile:
        cmd.append("--onefile")
    for package in COLLECT_ALL:
        cmd.extend(["--collect-all", package])
    for item in ADD_DATA:
        if item.exists():
            # PyInstaller uses ';' as the Windows separator and ':' elsewhere.
            separator = ";" if platform.system().lower() == "windows" else ":"
            destination = item.name
            cmd.extend(["--add-data", f"{item}{separator}{destination}"])
    for hidden in HIDDEN_IMPORTS:
        cmd.extend(["--hidden-import", hidden])
    cmd.append(str(ENTRYPOINT))
    return cmd


def main() -> int:
    parser = argparse.ArgumentParser(description="Build the Python trainer Tauri sidecar")
    parser.add_argument("--onefile", action="store_true", default=True, help="build one-file sidecar executable (default)")
    parser.add_argument("--onedir", dest="onefile", action="store_false", help="build one-dir sidecar for debugging")
    parser.add_argument("--skip-install", action="store_true", help="assume pyinstaller is already installed")
    parser.add_argument("--self-test", action="store_true", help="run the bundled trainer --test after build")
    args = parser.parse_args()

    if not ENTRYPOINT.exists():
        raise SystemExit(f"Trainer entrypoint not found: {ENTRYPOINT}")

    BIN_DIR.mkdir(parents=True, exist_ok=True)
    WORK_DIR.mkdir(parents=True, exist_ok=True)
    SPEC_DIR.mkdir(parents=True, exist_ok=True)

    if not args.skip_install:
        run([sys.executable, "-m", "pip", "install", "-r", "requirements-v12.txt", "pyinstaller>=6.0,<7.0"])

    triple = rust_target_triple()
    name = target_filename(triple)
    out_path = BIN_DIR / name
    if out_path.exists():
        out_path.unlink()
    if not args.onefile:
        shutil.rmtree(out_path.with_suffix(""), ignore_errors=True)

    run(pyinstaller_command(name, args.onefile))

    if not out_path.exists():
        # PyInstaller keeps the exact --name. On Windows, ensure the expected exe
        # extension exists even if a caller supplied a non-standard target name.
        candidates = sorted(BIN_DIR.glob(f"{name}*"))
        raise SystemExit(f"Expected sidecar not produced at {out_path}; candidates={candidates}")

    print(f"[OK] Tauri sidecar built: {out_path}")
    print(f"[OK] Size: {out_path.stat().st_size / (1024 * 1024):.1f} MiB")

    if args.self_test:
        run([str(out_path), "--test"])

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
