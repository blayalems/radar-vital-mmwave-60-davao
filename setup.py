"""Setuptools build hook staging operational runtime resources into rvt_trainer.assets."""

from __future__ import annotations

import shutil
from pathlib import Path
from setuptools import setup
from setuptools.command.build_py import build_py

ROOT = Path(__file__).resolve().parent
RUNTIME_FILES = (
    "radar_vital_live_dashboard.html",
    "config/protocol_contract.json",
    "quality/statistical-analysis-plan.json",
)
RUNTIME_DIRS = ("assets",)


class BuildPyWithRuntimeAssets(build_py):
    """Stage operational runtime files inside rvt_trainer/assets/runtime/ for wheels."""

    def run(self) -> None:
        super().run()
        if self.dry_run:
            return
        target_root = Path(self.build_lib) / "rvt_trainer" / "assets" / "runtime"
        if target_root.exists():
            shutil.rmtree(target_root)
        target_root.mkdir(parents=True, exist_ok=True)
        for rel in RUNTIME_FILES:
            src = ROOT / rel
            if not src.is_file():
                raise FileNotFoundError(f"Missing required runtime asset: {src}")
            dest = target_root / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
        for rel_dir in RUNTIME_DIRS:
            src_dir = ROOT / rel_dir
            if not src_dir.is_dir():
                raise FileNotFoundError(f"Missing required runtime asset dir: {src_dir}")
            shutil.copytree(src_dir, target_root / rel_dir, dirs_exist_ok=True)

    def get_outputs(self, include_bytecode: bool = True) -> list[str]:
        outputs = list(super().get_outputs(include_bytecode=include_bytecode))
        runtime_dir = Path(self.build_lib) / "rvt_trainer" / "assets" / "runtime"
        if runtime_dir.is_dir():
            for path in sorted(runtime_dir.rglob("*")):
                if path.is_file():
                    outputs.append(str(path))
        return outputs


setup(cmdclass={"build_py": BuildPyWithRuntimeAssets})
