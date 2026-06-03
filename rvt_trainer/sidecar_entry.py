"""Frozen sidecar entrypoint for the Windows Tauri operator console.

The unfrozen trainer starts child session captures with this argv shape:

    python.exe radar_vital_trainer_v12_for_v16_0.py session ...

Inside a PyInstaller executable, ``sys.executable`` is the frozen sidecar itself.
If the original code launches ``sys.executable`` plus the script path, the frozen
process receives the script path as argv[1] and argparse would treat it as the
subcommand. This wrapper makes that shape safe by stripping the legacy script
argument before dispatching to the real trainer main.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from rvt_trainer import monolith

LEGACY_ENTRYPOINT = "radar_vital_trainer_v12_for_v16_0.py"


def _strip_legacy_script_arg() -> None:
    if len(sys.argv) < 2:
        return
    first = Path(str(sys.argv[1])).name.lower()
    if first == LEGACY_ENTRYPOINT:
        sys.argv.pop(1)


def _sidecar_self_test() -> int:
    schema_path = Path(monolith.__file__).resolve().parent / "assets" / "help_schema.json"
    payload = {
        "ok": schema_path.exists(),
        "version": monolith.VERSION,
        "dashboard_version": monolith.DASHBOARD_VERSION,
        "schema_path": str(schema_path),
    }
    print(json.dumps(payload, sort_keys=True))
    return 0 if payload["ok"] else 1


def main() -> None:
    _strip_legacy_script_arg()
    if len(sys.argv) == 2 and sys.argv[1] == "--test":
        raise SystemExit(_sidecar_self_test())
    monolith.main()


if __name__ == "__main__":
    main()
