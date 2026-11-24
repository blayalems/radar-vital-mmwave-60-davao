"""Shared serial wire protocol contract loaded from ``config/protocol_contract.json``."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from rvt_trainer.runtime_paths import RUNTIME_PACKAGE_DIRECTORY, runtime_root

PROTOCOL_CONTRACT_RELPATH = Path("config") / "protocol_contract.json"


def protocol_contract_candidates() -> List[Path]:
    """Return ordered candidate locations for ``config/protocol_contract.json``."""

    package_dir = Path(__file__).resolve().parent
    raw_candidates = [
        runtime_root() / PROTOCOL_CONTRACT_RELPATH,
        package_dir / RUNTIME_PACKAGE_DIRECTORY / PROTOCOL_CONTRACT_RELPATH,
        package_dir.parent / PROTOCOL_CONTRACT_RELPATH,
    ]
    out: List[Path] = []
    seen: set[str] = set()
    for candidate in raw_candidates:
        resolved = candidate.resolve()
        key = str(resolved).lower()
        if key not in seen:
            seen.add(key)
            out.append(resolved)
    return out


def protocol_contract_path() -> Path:
    """Resolve the active ``config/protocol_contract.json`` path."""

    candidates = protocol_contract_candidates()
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    searched = ", ".join(str(p) for p in candidates)
    raise FileNotFoundError(
        f"Could not locate {PROTOCOL_CONTRACT_RELPATH.as_posix()} (searched: {searched})"
    )


def load_protocol_contract(path: Optional[Path] = None) -> Dict[str, Any]:
    """Load and structurally validate ``config/protocol_contract.json``."""

    resolved = path if path is not None else protocol_contract_path()
    data = json.loads(resolved.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"Protocol contract must be a JSON object: {resolved}")
    columns = data.get("columns")
    expected_count = int(data["expected_column_count"])
    expected_tail = tuple(str(col) for col in data.get("expected_tail") or ())
    if not isinstance(columns, list) or not all(isinstance(col, str) for col in columns):
        raise RuntimeError(f"Protocol contract 'columns' must be a list of strings: {resolved}")
    if len(columns) != expected_count:
        raise RuntimeError(
            f"Protocol contract column count mismatch in {resolved}: "
            f"expected {expected_count}, got {len(columns)}"
        )
    if expected_tail and tuple(columns[-len(expected_tail) :]) != expected_tail:
        raise RuntimeError(
            f"Protocol contract tail mismatch in {resolved}: "
            f"expected {expected_tail}, got {tuple(columns[-len(expected_tail) :])}"
        )
    return data


PROTOCOL_CONTRACT_PATH: Path = protocol_contract_path()
PROTOCOL_CONTRACT: Dict[str, Any] = load_protocol_contract(PROTOCOL_CONTRACT_PATH)

WIRE_PROTOCOL_VERSION: str = str(PROTOCOL_CONTRACT["wire_protocol_version"])
SERIAL_BAUD_RATE: int = int(PROTOCOL_CONTRACT["baud_rate"])
RADAR_LOG_COLUMNS: List[str] = [str(col) for col in PROTOCOL_CONTRACT["columns"]]
EXPECTED_RADAR_LOG_COLUMN_COUNT: int = int(PROTOCOL_CONTRACT["expected_column_count"])

_legacy_counts: Dict[str, int] = {
    str(k): int(v) for k, v in dict(PROTOCOL_CONTRACT["legacy_column_counts"]).items()
}
LEGACY_V14_COLUMN_COUNT: int = _legacy_counts["v14.1"]
LEGACY_V15_COLUMN_COUNT: int = _legacy_counts["v15.0"]
LEGACY_V15_1_COLUMN_COUNT: int = _legacy_counts["v15.1"]

SCHEMA_VERSION_MAP: Dict[int, str] = {
    **{count: version for version, count in _legacy_counts.items()},
    EXPECTED_RADAR_LOG_COLUMN_COUNT: WIRE_PROTOCOL_VERSION,
}

V15_NEW_COLUMN_DEFAULTS: Dict[str, float | int] = {
    str(key): (float("nan") if val is None else int(val))
    for key, val in dict(PROTOCOL_CONTRACT["v15_new_column_defaults"]).items()
}

EXPECTED_RADAR_LOG_TAIL: Tuple[str, ...] = tuple(
    str(col) for col in PROTOCOL_CONTRACT["expected_tail"]
)
LEGACY_V14_RADAR_LOG_TAIL: Tuple[str, ...] = tuple(
    str(col) for col in PROTOCOL_CONTRACT["legacy_v14_tail"]
)

__all__ = [
    "EXPECTED_RADAR_LOG_COLUMN_COUNT",
    "EXPECTED_RADAR_LOG_TAIL",
    "LEGACY_V14_COLUMN_COUNT",
    "LEGACY_V14_RADAR_LOG_TAIL",
    "LEGACY_V15_1_COLUMN_COUNT",
    "LEGACY_V15_COLUMN_COUNT",
    "PROTOCOL_CONTRACT",
    "PROTOCOL_CONTRACT_PATH",
    "PROTOCOL_CONTRACT_RELPATH",
    "RADAR_LOG_COLUMNS",
    "SCHEMA_VERSION_MAP",
    "SERIAL_BAUD_RATE",
    "V15_NEW_COLUMN_DEFAULTS",
    "WIRE_PROTOCOL_VERSION",
    "load_protocol_contract",
    "protocol_contract_candidates",
    "protocol_contract_path",
]
