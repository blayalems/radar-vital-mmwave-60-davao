"""Shared response, persistence, and timeout helpers for trainer services.

This module deliberately has no dependency on :mod:`rvt_trainer.monolith`.
Service modules can therefore use the same JSON and process-lifecycle
contracts without reintroducing circular imports. The monolith imports these
functions under its historical names for compatibility.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np


def nan_safe(obj: Any) -> Any:
    """Return the trainer's historical JSON-safe representation of ``obj``."""

    if isinstance(obj, dict):
        return {key: nan_safe(value) for key, value in obj.items()}
    if isinstance(obj, list):
        return [nan_safe(value) for value in obj]
    if isinstance(obj, tuple):
        return [nan_safe(value) for value in obj]
    if isinstance(obj, (np.floating, float)):
        return float(obj) if np.isfinite(obj) else None
    if isinstance(obj, (np.integer, int)):
        return int(obj)
    return obj


def atomic_write_json(obj: Dict[str, Any], path: str) -> None:
    """Durably replace ``path`` with JSON without exposing a partial file."""

    # Preserve the historical ``save_json`` contract: normalize relative
    # segments, but do not resolve the final path through a symlink. Replacing
    # the named path keeps an existing symlink target untouched.
    target = Path(os.path.abspath(path))
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(
        prefix=".codex-json-",
        suffix=".tmp",
        dir=str(target.parent),
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(nan_safe(obj), handle, indent=2, allow_nan=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_path, target)
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


def read_json_if_exists(path: str) -> Optional[Any]:
    """Read JSON or return ``None`` for missing, malformed, or unreadable data."""

    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError, TypeError):
        return None


def json_safe_response(obj: Any) -> bytes:
    """Encode strict JSON, coercing non-finite numeric values to ``null``."""

    def clean(value: Any) -> Any:
        if isinstance(value, bool) or value is None or isinstance(value, str):
            return value
        if isinstance(value, dict):
            return {key: clean(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [clean(item) for item in value]
        if isinstance(value, (np.floating, float)):
            return float(value) if np.isfinite(value) else None
        if isinstance(value, (np.integer, int)):
            return int(value)
        return value

    return json.dumps(
        clean(obj),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def api_error(code: str, message: str, **details: Any) -> Dict[str, Any]:
    """Build the stable control-API error envelope."""

    error: Dict[str, Any] = {
        "code": str(code),
        "message": str(message),
    }
    error.update(details)
    return {"ok": False, "error": error}


def wait_for_process_exit(proc: Any, timeout_s: float) -> bool:
    """Wait at most ``timeout_s`` for a child, returning whether it was reaped."""

    try:
        proc.wait(timeout=max(0.0, float(timeout_s)))
        return True
    except subprocess.TimeoutExpired:
        return False
    except Exception:
        try:
            return proc.poll() is not None
        except Exception:
            return False


__all__ = [
    "api_error",
    "atomic_write_json",
    "json_safe_response",
    "nan_safe",
    "read_json_if_exists",
    "wait_for_process_exit",
]
