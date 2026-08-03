"""Regression tests for the PR72 session-data audit (sessions s01-s14).

Each test pins one defect found by auditing the recorded sessions in
``sessions/`` (s08-s11 analyse output plus the codex probe captures):

1. ``contract_length`` was measured on the loader-augmented dataframe
   (222 on-disk columns + timestamp_s + hr_trust_fresh + session_id = 225),
   so every v16.4.0 session was wrongly marked ``firmware_rejected``.
2. ``module_fw_major/sub/mod`` are renamed to ``fw_major/fw_sub/fw_mod`` by
   ``canonicalize_with_synonyms`` before truthfulness ran, so a valid module
   version could never be detected server-side.
3. The adaptive-correction shadow ran on the 1 Hz feature frame where radar
   columns carry ``_mean`` suffixes, so it always skipped with
   "missing one of (...)" despite the data being present.
4. The v15 PQI shadow claimed "columns not present (firmware <v15.0)" because
   the v15 columns are never aggregated into the 1 Hz frame.
5. ``ble_ref_quality`` used the raw-packet ratio as coverage (~33% by protocol
   design) and mislabelled the gap as ``decode_error_pct`` (~66%), and never
   emitted the ``pi_median`` the readiness verdict reads.
"""

from __future__ import annotations

import math

import numpy as np
import pandas as pd

import rvt_trainer.monolith as monolith


# ---------------------------------------------------------------------------
# 1 + 2: firmware truthfulness
# ---------------------------------------------------------------------------

def test_on_disk_contract_length_reads_csv_header(tmp_path):
    csv_path = tmp_path / "radar.csv"
    csv_path.write_text("a,b,c,d,e\n1,2,3,4,5\n", encoding="utf-8")
    assert monolith._radar_csv_on_disk_contract_length([str(csv_path)]) == 5


def test_on_disk_contract_length_handles_missing_files(tmp_path):
    assert monolith._radar_csv_on_disk_contract_length([str(tmp_path / "nope.csv")]) is None
    assert monolith._radar_csv_on_disk_contract_length([]) is None


def test_truthfulness_prefers_explicit_contract_length_over_augmented_frame():
    # Simulate the loader-augmented frame: on-disk contract is 222, but the
    # dataframe carries loader-derived columns on top.
    df = pd.DataFrame({
        "timestamp_ms": [1000, 2000],
        "sketch_major": [16, 16],
        "sketch_sub": [5, 5],
            "sketch_mod": [8, 8],
        "module_fw_valid": [0, 0],
        # loader-derived columns that inflated len(df.columns):
        "timestamp_s": [1.0, 2.0],
        "hr_trust_fresh": [0, 0],
        "session_id": ["session_1", "session_1"],
    })
    out = monolith._truthfulness_from_radar(df, contract_length=222)
    assert out["contract_length"] == 222
    assert out["version"] == "v16.5.8"
    # Without the explicit width the old (buggy) behavior falls back to the
    # augmented frame width — keep that as the documented fallback.
    assert monolith._truthfulness_from_radar(df)["contract_length"] == len(df.columns)


def test_supported_contract_widths_pin_the_bug():
    # 222 is the v15.2 contract; 225 is the loader-augmented width that must
    # never be treated as a firmware contract.
    assert monolith._is_supported_radar_contract_length(222)
    assert not monolith._is_supported_radar_contract_length(225)


def test_truthfulness_reads_module_version_after_loader_rename():
    # canonicalize_with_synonyms renames module_fw_* -> fw_* (copy + drop), so
    # truthfulness must accept either spelling.
    df = pd.DataFrame({
        "timestamp_ms": [1000],
        "sketch_major": [16],
        "sketch_sub": [5],
        "sketch_mod": [0],
        "fw_major": [2],
        "fw_sub": [1],
        "fw_mod": [0],
        "module_fw_valid": [1],
    })
    out = monolith._truthfulness_from_radar(df, contract_length=222)
    assert out["module_version"] == "v2.1.0"
    assert out["module_version_detected"] is True
    assert out["module_version_valid"] is True


# ---------------------------------------------------------------------------
# 3: adaptive-correction shadow on the 1 Hz feature frame
# ---------------------------------------------------------------------------

def test_adaptive_shadow_accepts_suffixed_1hz_columns():
    n = 120
    ref = np.linspace(62.0, 96.0, n)
    rng = np.random.default_rng(7)
    df = pd.DataFrame({
        "raw_hr_uncorrected_mean": ref + 9.0 + rng.normal(0.0, 1.0, n),
        "raw_hr_corrected_mean": ref + 2.0 + rng.normal(0.0, 0.8, n),
        "ref_hr": ref,
    })
    out = monolith._build_adaptive_correction_shadow(df)
    assert out["n_paired_points"] == n
    assert out["firmware_default_rmse_bpm"] is not None
    assert "missing" not in str(out.get("recommendation", "")).lower()


def test_adaptive_shadow_reports_truly_missing_columns():
    df = pd.DataFrame({"ref_hr": [80.0, 81.0]})
    out = monolith._build_adaptive_correction_shadow(df)
    rec = str(out.get("recommendation", ""))
    assert "missing" in rec.lower()
    assert "raw_hr_uncorrected" in rec


# ---------------------------------------------------------------------------
# 4: v15 PQI shadow sentinel handling
# ---------------------------------------------------------------------------

def test_pqi_v15_shadow_sentinel_rows_report_inactive_not_absent():
    n = 40
    df = pd.DataFrame({
        "pqi_heart": np.linspace(0.1, 0.4, n),
        "pqi_breath": np.linspace(0.1, 0.4, n),
        "pqi_heart_v15": [-1.0] * n,
        "pqi_breath_v15": [-1.0] * n,
        "phase_buffer_valid_pct": [-1.0] * n,
        "pqi_v15_pair_coverage_min": [-1.0] * n,
    })
    out = monolith._build_pqi_v15_shadow(df)
    assert out["enabled"] == 0 or out["enabled"] is False
    assert out["n_v15_active"] == 0
    assert "sentinel" in str(out.get("recommendation", "")).lower()


# ---------------------------------------------------------------------------
# 5: BLE reference quality metrics
# ---------------------------------------------------------------------------

def _write_ble_fixture(tmp_path, with_summary: bool):
    ref = tmp_path / "ref.csv"
    raw = tmp_path / "ref_ble_raw.csv"
    ref_rows = ["timestamp_ms,source_uuid,ref_hr,ref_rr,ref_spo2,ref_pi"]
    for sec in range(10):
        ref_rows.append(f"{sec * 1000 + 100},u,80,15,98,2.0")
    ref.write_text("\n".join(ref_rows) + "\n", encoding="utf-8")
    raw_rows = ["timestamp_ms,source_uuid,sender,packet_len,raw_hex"]
    for i in range(34):
        raw_rows.append(f"{i * 300},u,u (Handle: 54),20,a7002100")
    raw.write_text("\n".join(raw_rows) + "\n", encoding="utf-8")
    if with_summary:
        (tmp_path / "ref_ble_summary.json").write_text(
            '{"stats_by_source": {"u": {"packet_count": 34, "parsed_count": 34}}}',
            encoding="utf-8",
        )
    return str(ref)


def test_ble_ref_quality_time_coverage_and_pi_median(tmp_path):
    ref_path = _write_ble_fixture(tmp_path, with_summary=False)
    out = monolith._compute_ble_ref_quality([ref_path])
    # 10 distinct covered seconds against a ~9.9 s raw span -> ~100%, nothing
    # like the old 33% packet ratio.
    assert out["coverage_pct"] > 90.0
    assert out["pi_median"] == 2.0
    # Without the logger summary there is no way to measure decode errors —
    # the old fallback fabricated ~66% out of protocol-normal redundancy.
    assert math.isnan(out["decode_error_pct"])
    assert out["status"] == "OK"


def test_ble_ref_quality_decode_errors_from_summary_stats(tmp_path):
    ref_path = _write_ble_fixture(tmp_path, with_summary=True)
    out = monolith._compute_ble_ref_quality([ref_path])
    assert out["decode_error_pct"] == 0.0
    assert out["status"] == "OK"
