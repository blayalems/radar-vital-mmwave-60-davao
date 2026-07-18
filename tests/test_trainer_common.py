"""Characterization tests for shared trainer service helpers."""

from __future__ import annotations

import json
import subprocess
from unittest.mock import MagicMock

import numpy as np
import pytest

from rvt_trainer import monolith
from rvt_trainer.api.common import (
    api_error,
    atomic_write_json,
    json_safe_response,
    nan_safe,
    read_json_if_exists,
    wait_for_process_exit,
)


def test_monolith_keeps_historical_helper_imports():
    assert monolith.save_json is atomic_write_json
    assert monolith._read_json_if_exists is read_json_if_exists
    assert monolith._json_safe_response is json_safe_response
    assert monolith.nan_safe is nan_safe
    assert monolith._api_error is api_error
    assert monolith._wait_for_process_exit is wait_for_process_exit


def test_json_response_schema_and_non_finite_coercion_are_unchanged():
    payload = {
        "ok": True,
        "nan": float("nan"),
        "positive_infinity": np.float64(float("inf")),
        "count": np.int64(3),
    }

    decoded = json.loads(json_safe_response(payload).decode("utf-8"))

    assert decoded == {
        "ok": True,
        "nan": None,
        "positive_infinity": None,
        "count": 3,
    }


def test_atomic_json_round_trip_replaces_target_without_temp_leak(tmp_path):
    target = tmp_path / "state.json"
    target.write_text('{"old": 1}', encoding="utf-8")

    atomic_write_json({"new": np.float64(2.5), "valid": True}, str(target))

    # ``nan_safe`` historically converts bool through the integer branch for
    # persisted files. Preserve that on-disk contract during extraction.
    assert read_json_if_exists(str(target)) == {"new": 2.5, "valid": 1}
    assert list(tmp_path.glob(".codex-json-*.tmp")) == []


def test_atomic_json_replaces_symlink_name_without_overwriting_target(tmp_path):
    target = tmp_path / "target.json"
    target.write_text('{"original": 1}', encoding="utf-8")
    alias = tmp_path / "alias.json"
    try:
        alias.symlink_to(target)
    except (NotImplementedError, OSError) as exc:
        pytest.skip(f"symlinks unavailable: {exc}")

    atomic_write_json({"replacement": 2}, str(alias))

    assert not alias.is_symlink()
    assert read_json_if_exists(str(alias)) == {"replacement": 2}
    assert read_json_if_exists(str(target)) == {"original": 1}


def test_read_json_if_exists_is_fail_closed(tmp_path):
    malformed = tmp_path / "bad.json"
    malformed.write_text("{bad", encoding="utf-8")

    assert read_json_if_exists(str(tmp_path / "missing.json")) is None
    assert read_json_if_exists(str(malformed)) is None


def test_api_error_keeps_stable_envelope_and_details():
    assert api_error(
        "VALIDATION_FAILED",
        "invalid field",
        field="duration_s",
    ) == {
        "ok": False,
        "error": {
            "code": "VALIDATION_FAILED",
            "message": "invalid field",
            "field": "duration_s",
        },
    }


def test_bounded_process_wait_reports_reap_timeout_and_poll_fallback():
    reaped = MagicMock()
    reaped.wait.return_value = 0
    assert wait_for_process_exit(reaped, 0.5) is True
    reaped.wait.assert_called_once_with(timeout=0.5)

    timed_out = MagicMock()
    timed_out.wait.side_effect = subprocess.TimeoutExpired("child", 0.5)
    assert wait_for_process_exit(timed_out, 0.5) is False

    already_gone = MagicMock()
    already_gone.wait.side_effect = OSError("already reaped")
    already_gone.poll.return_value = 0
    assert wait_for_process_exit(already_gone, 0.5) is True
