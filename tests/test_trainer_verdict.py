"""
test_trainer_verdict.py
-----------------------
Unit tests for _build_ml_readiness_verdict.

Key constants (from monolith.py):
  EXPECTED_RADAR_LOG_COLUMN_COUNT = 207
  ML_READY_MIN_HR_COVERAGE_PCT   = 50.0
  ML_READY_MIN_HR_MAE_BPM        = 8.0
  ML_READY_MIN_HR_BIAS_BPM       = 5.0
  ML_READY_MIN_RR_COVERAGE_PCT   = 70.0
  ML_READY_MIN_RR_MAE_BPM        = 4.0
  ML_READY_MIN_BLE_COVERAGE_PCT  = 70.0
  BLE_PI_QUALITY_THRESHOLD       = 1.0

The function signature is:
  _build_ml_readiness_verdict(analyse_summary: dict, **kwargs) -> dict
"""
from __future__ import annotations

import pytest

from rvt_trainer.monolith import (
    _build_ml_readiness_verdict,
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    ML_READY_MIN_HR_COVERAGE_PCT,
    ML_READY_MIN_HR_MAE_BPM,
    ML_READY_MIN_HR_BIAS_BPM,
    ML_READY_MIN_RR_COVERAGE_PCT,
    ML_READY_MIN_RR_MAE_BPM,
    ML_READY_MIN_BLE_COVERAGE_PCT,
)


# ---------------------------------------------------------------------------
# Helpers to build minimal analyse_summary dicts
# ---------------------------------------------------------------------------

def _fw_ok() -> dict:
    """Minimal fw_truthfulness that passes schema checks."""
    return {
        "contract_length": EXPECTED_RADAR_LOG_COLUMN_COUNT,
        "critical_columns_ok": True,
        "module_version_valid": True,
        "version": "v16.3.0",
    }


def _ble_ok() -> dict:
    """BLE quality that passes the coverage and PI thresholds."""
    return {
        "distilled_rows_pct_of_raw": ML_READY_MIN_BLE_COVERAGE_PCT + 5.0,  # 75 %
        "coverage_pct": ML_READY_MIN_BLE_COVERAGE_PCT + 5.0,
        "pi_median": 2.0,  # > BLE_PI_QUALITY_THRESHOLD (1.0)
    }


def _gate_passed() -> dict:
    """ml_gate that reports a passing primary gate."""
    return {
        "passed": True,
        "status": "pass",
        "secondary_gate": {"kind": "ok", "basis": "sufficient_data"},
    }


def _hr_baseline_ok() -> dict:
    """hr_baseline with coverage and MAE within thresholds."""
    return {
        "coverage_pct": ML_READY_MIN_HR_COVERAGE_PCT + 10.0,   # 60 %
        "valid_only": {
            "mae": ML_READY_MIN_HR_MAE_BPM - 2.0,              # 6.0 bpm
            "bias": ML_READY_MIN_HR_BIAS_BPM - 2.0,            # 3.0 bpm
            "rmse": 5.0,
            "r": 0.92,
        },
    }


def _rr_baseline_ok() -> dict:
    """rr_baseline with coverage and MAE within thresholds."""
    return {
        "coverage_pct": ML_READY_MIN_RR_COVERAGE_PCT + 5.0,    # 75 %
        "valid_only": {
            "mae": ML_READY_MIN_RR_MAE_BPM - 1.0,              # 3.0 bpm
            "bias": 1.0,
            "rmse": 2.5,
            "r": 0.88,
        },
    }


def _ready_summary() -> dict:
    """A complete analyse_summary that should yield verdict='ready'."""
    return {
        "fw_truthfulness": _fw_ok(),
        "ble_ref_quality": _ble_ok(),
        "ml_gate": _gate_passed(),
        "hr_baseline": _hr_baseline_ok(),
        "rr_baseline": _rr_baseline_ok(),
        "pqi_lock_pct": 75.0,
        "feature_schema_hash": None,   # None → schema_hash_ok is True
    }


# ---------------------------------------------------------------------------
# Case 1: fully-passing input → verdict 'ready'
# ---------------------------------------------------------------------------

class TestVerdictReady:
    def test_ready_verdict_value(self):
        result = _build_ml_readiness_verdict(_ready_summary())
        assert result["verdict"] == "ready"

    def test_ready_has_required_keys(self):
        result = _build_ml_readiness_verdict(_ready_summary())
        for key in ("verdict", "readiness_kind", "headline", "schema_verdict",
                    "signal_verdict", "reference_verdict", "training_verdict",
                    "categories", "passed", "failed", "limitation_kind", "next_action"):
            assert key in result, f"missing key: {key}"

    def test_ready_schema_verdict_ok(self):
        result = _build_ml_readiness_verdict(_ready_summary())
        assert result["schema_verdict"] in {"ok", "provenance_warning"}

    def test_ready_training_verdict(self):
        result = _build_ml_readiness_verdict(_ready_summary())
        assert result["training_verdict"] == "ready"

    def test_ready_categories_nonempty(self):
        result = _build_ml_readiness_verdict(_ready_summary())
        assert len(result["categories"]) > 0


# ---------------------------------------------------------------------------
# Case 2: too-short session → 'deferred' or 'conditional'
# ---------------------------------------------------------------------------

class TestVerdictTooShort:
    def _short_summary(self) -> dict:
        """Session where the secondary gate signals 'too short to judge'."""
        summary = _ready_summary()
        # Override ml_gate so that secondary_gate signals deferred
        summary["ml_gate"] = {
            "passed": False,
            "status": "deferred",
            "secondary_gate": {"kind": "deferred", "basis": "too_short_to_judge"},
        }
        return summary

    def test_short_verdict_is_not_ready_or_conditional(self):
        result = _build_ml_readiness_verdict(self._short_summary())
        # The verdict top-level maps readiness_kind:
        #   'conditional' / 'deferred' → 'conditional'
        #   anything else               → 'not_ready'
        assert result["verdict"] in {"not_ready", "conditional"}

    def test_short_readiness_kind_is_deferred_or_conditional(self):
        result = _build_ml_readiness_verdict(self._short_summary())
        assert result["readiness_kind"] in {"deferred", "conditional"}

    def test_short_training_verdict_is_deferred_or_conditional(self):
        result = _build_ml_readiness_verdict(self._short_summary())
        assert result["training_verdict"] in {"deferred", "conditional"}

    def test_short_limitation_kind_is_duration_or_data(self):
        result = _build_ml_readiness_verdict(self._short_summary())
        assert result["limitation_kind"] in {"duration", "data", None}


# ---------------------------------------------------------------------------
# Case 3: firmware contract violation → 'firmware_rejected'
# ---------------------------------------------------------------------------

class TestVerdictFirmwareRejected:
    def _bad_contract_summary(self) -> dict:
        summary = _ready_summary()
        # Override fw_truthfulness with a wrong contract_length
        summary["fw_truthfulness"] = {
            "contract_length": 100,   # not 207 → triggers firmware_rejected
            "critical_columns_ok": False,
            "module_version_valid": False,
            "version": "v14.0.0",
        }
        return summary

    def test_firmware_rejected_schema_verdict(self):
        result = _build_ml_readiness_verdict(self._bad_contract_summary())
        assert result["schema_verdict"] == "firmware_rejected"

    def test_firmware_rejected_readiness_kind(self):
        result = _build_ml_readiness_verdict(self._bad_contract_summary())
        assert result["readiness_kind"] == "firmware_rejected"

    def test_firmware_rejected_verdict_is_not_ready(self):
        result = _build_ml_readiness_verdict(self._bad_contract_summary())
        # Top-level 'verdict' maps 'firmware_rejected' readiness_kind to 'not_ready'
        assert result["verdict"] == "not_ready"

    def test_firmware_rejected_has_firmware_category(self):
        result = _build_ml_readiness_verdict(self._bad_contract_summary())
        cat_ids = [c["id"] for c in result["categories"]]
        assert "firmware" in cat_ids


# ---------------------------------------------------------------------------
# Case 4: non-dict input → deferred / unavailable
# ---------------------------------------------------------------------------

class TestVerdictNonDictInput:
    def test_none_input_returns_not_ready(self):
        result = _build_ml_readiness_verdict(None)
        assert result["verdict"] == "not_ready"

    def test_none_input_schema_verdict_firmware_rejected(self):
        # Per the code, missing dict → schema_verdict='firmware_rejected'
        result = _build_ml_readiness_verdict(None)
        assert result["schema_verdict"] == "firmware_rejected"

    def test_string_input_returns_not_ready(self):
        result = _build_ml_readiness_verdict("bad_input")
        assert result["verdict"] == "not_ready"
