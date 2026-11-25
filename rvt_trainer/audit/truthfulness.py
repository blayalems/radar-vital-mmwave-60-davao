"""Truthfulness, coverage-loss ledgers, gate audits, and golden comparisons."""

from __future__ import annotations

import json
import os
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

from rvt_trainer.audit.verdict import (
    baseline_summary,
    compute_metrics,
    finite_pair_mask,
)
from rvt_trainer.constants import (
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    HR_PUBLISH_SOURCE_NAMES_INVERSE,
    HR_RESCUE_PLAUSIBLE_MAX_BPM,
    HR_RESCUE_PLAUSIBLE_MIN_BPM,
    _clip01,
    _row_num,
    _row_text,
    _safe_int,
    _to_num,
    safe_series,
)

def _derive_raw_rescue_reject_count(raw_radar_df) -> int:
    """Count frames where the F1.1 rescue-path plausibility gate rejected publish.

    The firmware does not emit a dedicated reason code for the range check;
    we infer it from existing telemetry: ``hr_publish_reason ==
    RAW_PHASE_STALE_NO_COMPARISON`` (15) while the rescue source was NOT
    selected and ``raw_hr_corrected`` is implausible (non-finite or outside
    [HR_RESCUE_PLAUSIBLE_MIN_BPM, HR_RESCUE_PLAUSIBLE_MAX_BPM]).

    Returns 0 when required columns are unavailable (legacy v13.x captures
    or tests with trimmed schemas).
    """
    try:
        needed = {"raw_hr_corrected", "hr_publish_source", "hr_publish_reason"}
        if not needed.issubset(set(raw_radar_df.columns)):
            return 0
        corrected = pd.to_numeric(raw_radar_df["raw_hr_corrected"], errors="coerce")
        implausible = ~corrected.between(
            HR_RESCUE_PLAUSIBLE_MIN_BPM, HR_RESCUE_PLAUSIBLE_MAX_BPM, inclusive="both"
        ) | corrected.isna()
        phase_stale = pd.to_numeric(raw_radar_df["hr_publish_reason"], errors="coerce") == 15
        rescue_code = HR_PUBLISH_SOURCE_NAMES_INVERSE.get("corrected_raw_rescue", 8)
        not_rescued = pd.to_numeric(raw_radar_df["hr_publish_source"], errors="coerce") != rescue_code
        return int((implausible & phase_stale & not_rescued).sum())
    except Exception:
        return 0


def _fmt_quick_metrics(label: str, block: Optional[Dict]) -> List[str]:
    if not isinstance(block, dict):
        return [f"{label}: unavailable"]
    vo = block.get("valid_only", {})
    if not isinstance(vo, dict):
        vo = {}
    try:
        n = int(float(vo.get("n", 0) or 0))
    except Exception:
        n = 0
    rmse = _to_num(vo.get("rmse"), float("nan"))
    mae = _to_num(vo.get("mae"), float("nan"))
    bias = _to_num(vo.get("bias"), float("nan"))
    r = _to_num(vo.get("r"), float("nan"))
    cov = _to_num(block.get("coverage_pct"), float("nan"))
    return [
        f"{label} valid-only: n={n} | RMSE={rmse:.2f} | "
        f"MAE={mae:.2f} | Bias={bias:+.2f} | "
        f"r={r:.3f} | coverage={cov:.1f}%"
    ]


def _ml_gate_block_from_baseline(block: Optional[Dict]) -> Dict[str, object]:
    block = block if isinstance(block, dict) else {}
    vo = block.get("valid_only", {}) if isinstance(block.get("valid_only"), dict) else {}
    r = _to_num(vo.get("r"), float("nan"))
    rmse = _to_num(vo.get("rmse"), float("nan"))
    n = _safe_int(vo.get("n"), 0)
    return {
        "passed": bool(np.isfinite(r) and r > 0.4 and np.isfinite(rmse) and rmse < 8.0),
        "r": float(r),
        "rmse": float(rmse),
        "n": int(n),
    }


def _phase_subset_summary(feat_df: pd.DataFrame, phase_col: str) -> Tuple[Dict[str, Dict[str, float]], Dict[str, object]]:
    if phase_col not in feat_df.columns:
        empty = baseline_summary(feat_df.iloc[0:0].copy(), "hr")
        return empty, _ml_gate_block_from_baseline(empty)
    sub = feat_df.loc[safe_series(feat_df, phase_col, default=0.0) >= 0.5].copy()
    if len(sub) == 0:
        empty = baseline_summary(sub, "hr")
        return empty, _ml_gate_block_from_baseline(empty)
    block = baseline_summary(sub, "hr")
    return block, _ml_gate_block_from_baseline(block)


def _format_sketch_version(major, sub, mod) -> Optional[str]:
    m = _safe_int(major, -1)
    s = _safe_int(sub, -1)
    x = _safe_int(mod, -1)
    if min(m, s, x) < 0:
        return None
    if x == 0:
        return f"v{m}.{s}.0"
    # Preserve legacy branch-style decoding only for the known 13.9a-d family.
    if m == 13 and s == 9 and 1 <= x <= 4:
        return f"v{m}.{s}{chr(ord('a') + x - 1)}"
    return f"v{m}.{s}.{x}"


def _radar_csv_on_disk_contract_length(paths: Sequence[str]) -> Optional[int]:
    """Widest CSV header width across the given radar log files, read from disk.

    PR72 session-data audit (s08-s11): the loaded dataframe gains loader-derived
    columns (timestamp_s, the hr_trust_fresh alias, session_id from
    _combine_radar_frames), so ``len(df.columns)`` overstates the firmware
    contract width (222 -> 225) and every session was wrongly marked
    ``firmware_rejected``. The firmware contract must be measured against the
    on-disk header, not the augmented frame.
    """
    widths: List[int] = []
    for path in paths or []:
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                header = fh.readline().rstrip("\r\n")
        except OSError:
            continue
        if header:
            widths.append(header.count(",") + 1)
    return max(widths) if widths else None


def _first_populated_series(raw_df: pd.DataFrame, *names: str) -> pd.Series:
    """Return the first named column that exists with any non-NaN values.

    load_radar's canonicalize_with_synonyms renames module_fw_major/sub/mod to
    fw_major/fw_sub/fw_mod (copy + drop), so truthfulness must accept either
    spelling depending on whether it receives a raw or a loader-canonical frame.
    """
    for name in names:
        if name in raw_df.columns:
            series = safe_series(raw_df, name, default=np.nan).dropna()
            if len(series):
                return series
    return pd.Series(dtype=float)


def _truthfulness_from_radar(raw_df: pd.DataFrame, contract_length: Optional[int] = None) -> Dict[str, object]:
    fallback_len = int(len(raw_df.columns)) if isinstance(raw_df, pd.DataFrame) else EXPECTED_RADAR_LOG_COLUMN_COUNT
    out: Dict[str, object] = {
        "version": "unknown",
        "module_version": None,
        "module_version_detected": False,
        "module_version_valid": False,
        "contract_length": int(contract_length) if contract_length else fallback_len,
        "critical_columns_ok": False,
        "notes": "Derived from sketch/module telemetry in radar.csv",
    }
    if not isinstance(raw_df, pd.DataFrame) or len(raw_df) == 0:
        return out
    sketch_major = safe_series(raw_df, "sketch_major", default=np.nan).dropna()
    sketch_sub = safe_series(raw_df, "sketch_sub", default=np.nan).dropna()
    sketch_mod = safe_series(raw_df, "sketch_mod", default=np.nan).dropna()
    if len(sketch_major) and len(sketch_sub) and len(sketch_mod):
        out["version"] = _format_sketch_version(sketch_major.iloc[-1], sketch_sub.iloc[-1], sketch_mod.iloc[-1]) or "unknown"
    mod_major = _first_populated_series(raw_df, "module_fw_major", "fw_major")
    mod_sub = _first_populated_series(raw_df, "module_fw_sub", "fw_sub")
    mod_mod = _first_populated_series(raw_df, "module_fw_mod", "fw_mod")
    critical_cols = {"timestamp_ms", "reported_hr", "reported_rr", "candidate_hr", "candidate_rr", "pqi_heart", "pqi_breath"}
    out["critical_columns_ok"] = critical_cols.issubset(set(raw_df.columns))
    if "module_fw_valid" in raw_df.columns:
        out["module_version_valid"] = bool((safe_series(raw_df, "module_fw_valid", default=0.0).fillna(0.0) >= 0.5).iloc[-1])
    if len(mod_major) and len(mod_sub) and len(mod_mod):
        module_version = _format_sketch_version(mod_major.iloc[-1], mod_sub.iloc[-1], mod_mod.iloc[-1])
        out["module_version"] = module_version
        out["module_version_detected"] = module_version is not None
    return out


def _coverage_stage_label(row: pd.Series, target: str, has_reference: bool) -> str:
    publish_reason = _row_text(row, f"{target}_publish_reason_name", f"{target}_publish_reason")
    gate_reason = _row_text(row, f"{target}_gate_reason_name", f"{target}_gate_reason")
    block_stage = _safe_int(row.get(f"{target}_publish_block_stage"), -1)
    if target == "hr":
        disagree = _safe_int(row.get("hr_raw_disagree_subreason"), 0)
        if _row_num(row, "logged_hr_valid", default=0.0) >= 0.5 or _safe_int(row.get("hr_publish_reason"), -1) == 0:
            return "OK"
        if _row_num(row, "human_detected", default=1.0) < 0.5 or "NO_HUMAN" in publish_reason:
            return "NO_HUMAN"
        if _row_num(row, "phase_valid_this_frame", default=1.0) < 0.5 or _row_num(row, "phase_fresh", default=1.0) < 0.5:
            return "NO_PHASE"
        if disagree == 2 or "PHASE_STALE" in publish_reason or "PHASE_STALE" in gate_reason:
            return "RAW_PHASE_STALE_NO_COMPARISON"
        if disagree == 3 or "ANCHOR_DRIFT" in publish_reason or "ANCHOR_DRIFT" in gate_reason:
            return "RAW_ANCHOR_DRIFT"
        if disagree == 1 or "RAW_DISAGREE" in publish_reason:
            return "RAW_DISAGREE_TRUE"
        if disagree == 6 or "HARMONIC" in publish_reason:
            return "RAW_HARMONIC_SUSPECT"
    else:
        if _row_num(row, "logged_rr_valid", default=0.0) >= 0.5 or _safe_int(row.get("rr_publish_reason"), -1) == 0:
            return "OK"
        if _row_num(row, "human_detected", default=1.0) < 0.5 or "NO_HUMAN" in publish_reason:
            return "NO_HUMAN"
        if _row_num(row, "rr_phase_backed_publish_ready", default=1.0) < 0.5:
            return "NO_PHASE"
        if "PQI" in publish_reason or "PQI" in gate_reason:
            return "LOW_PQI"
        if "CONF" in publish_reason or "CONF" in gate_reason:
            return "LOW_CONF"
        if "STALE" in publish_reason or "STALE" in gate_reason:
            return "STALE_TRACK"
        if "HARMONIC" in publish_reason or "HARMONIC" in gate_reason:
            return "HARMONIC"
    if block_stage == 1:
        return "NO_HUMAN"
    if block_stage == 5:
        return "NO_PHASE"
    if block_stage == 7:
        return "LOW_PQI"
    if block_stage == 8:
        return "LOW_CONF"
    if block_stage == 9:
        return "HARMONIC"
    if block_stage == 10:
        return "RAW_DISAGREE_TRUE"
    if not has_reference:
        return "NO_REFERENCE_PAIR"
    return "PUBLISH_POLICY"


def _coverage_loss_ledger(raw_df: pd.DataFrame, has_reference: bool = True) -> Dict[str, object]:
    ledger: Dict[str, object] = {
        "hr": {"histogram": {}, "salvageable_pct": float("nan")},
        "rr": {"histogram": {}, "salvageable_pct": float("nan")},
    }
    if not isinstance(raw_df, pd.DataFrame) or len(raw_df) == 0:
        return ledger
    hr_hist: Dict[str, int] = {}
    rr_hist: Dict[str, int] = {}
    hr_salvage = 0
    rr_salvage = 0
    for _, row in raw_df.iterrows():
        hr_stage = _coverage_stage_label(row, "hr", bool(has_reference))
        rr_stage = _coverage_stage_label(row, "rr", bool(has_reference))
        hr_hist[hr_stage] = hr_hist.get(hr_stage, 0) + 1
        rr_hist[rr_stage] = rr_hist.get(rr_stage, 0) + 1
        hr_candidate_ok = any(
            np.isfinite(_row_num(row, col, default=float("nan"))) and _row_num(row, col, default=float("nan")) > 0
            for col in ("candidate_hr", "raw_hr_corrected", "raw_hr", "hr_spec_bpm", "hr_zc_bpm")
        )
        rr_candidate_ok = any(
            np.isfinite(_row_num(row, col, default=float("nan"))) and _row_num(row, col, default=float("nan")) > 0
            for col in ("candidate_rr", "raw_rr_effective", "raw_rr", "rr_spec_bpm", "rr_zc_bpm")
        )
        if hr_stage in {"RAW_PHASE_STALE_NO_COMPARISON", "PUBLISH_POLICY", "LOW_CONF", "LOW_PQI", "STALE_TRACK"} and hr_candidate_ok:
            hr_salvage += 1
        if rr_stage in {"PUBLISH_POLICY", "LOW_CONF", "LOW_PQI", "STALE_TRACK", "NO_PHASE"} and rr_candidate_ok:
            rr_salvage += 1
    total = float(len(raw_df))
    ledger["hr"] = {
        "histogram": dict(sorted(hr_hist.items(), key=lambda kv: (-kv[1], kv[0]))),
        "salvageable_pct": float(100.0 * hr_salvage / total),
    }
    ledger["rr"] = {
        "histogram": dict(sorted(rr_hist.items(), key=lambda kv: (-kv[1], kv[0]))),
        "salvageable_pct": float(100.0 * rr_salvage / total),
    }
    return ledger


def _oracle_candidate_audit(feat_df: pd.DataFrame) -> Dict[str, object]:
    def _audit_target(target: str, candidates: Sequence[Tuple[str, str]]) -> Dict[str, object]:
        if not isinstance(feat_df, pd.DataFrame) or len(feat_df) == 0:
            return {
                "best_candidate_source_histogram": {},
                "oracle_candidate_mae_bpm": float("nan"),
                "oracle_candidate_coverage_pct": float("nan"),
                "oracle_candidate_rmse_bpm": float("nan"),
            }
        ref = pd.to_numeric(feat_df.get(f"ref_{target}", pd.Series(dtype=float)), errors="coerce")
        chosen_true = []
        chosen_pred = []
        hist: Dict[str, int] = {}
        total_ref = int(ref.notna().sum())
        for idx in range(len(feat_df)):
            ref_value = ref.iloc[idx] if idx < len(ref) else np.nan
            if not np.isfinite(ref_value):
                continue
            options = []
            for source_name, col_name in candidates:
                if col_name not in feat_df.columns:
                    continue
                pred_value = _to_num(feat_df.iloc[idx].get(col_name), default=float("nan"))
                if np.isfinite(pred_value) and pred_value > 0:
                    options.append((abs(pred_value - ref_value), source_name, pred_value))
            if not options:
                continue
            _, source_name, pred_value = min(options, key=lambda item: (item[0], item[1]))
            hist[source_name] = hist.get(source_name, 0) + 1
            chosen_true.append(float(ref_value))
            chosen_pred.append(float(pred_value))
        metrics = compute_metrics(np.asarray(chosen_true, dtype=float), np.asarray(chosen_pred, dtype=float), target)
        return {
            "best_candidate_source_histogram": dict(sorted(hist.items(), key=lambda kv: (-kv[1], kv[0]))),
            "oracle_candidate_mae_bpm": float(metrics.get("mae", float("nan"))),
            "oracle_candidate_rmse_bpm": float(metrics.get("rmse", float("nan"))),
            "oracle_candidate_coverage_pct": float(100.0 * len(chosen_true) / max(total_ref, 1)),
        }

    return {
        "hr": _audit_target(
            "hr",
            (
                ("reported_hr", "reported_hr_mean"),
                ("candidate_hr", "candidate_hr_mean"),
                ("raw_hr_corrected", "raw_hr_corrected_mean"),
                ("hr_spec", "hr_spec_bpm_mean"),
                ("hr_zc", "hr_zc_bpm_mean"),
            ),
        ),
        "rr": _audit_target(
            "rr",
            (
                ("reported_rr", "reported_rr_mean"),
                ("candidate_rr", "candidate_rr_mean"),
                ("raw_rr_effective", "raw_rr_effective_mean"),
                ("rr_spec", "rr_spec_bpm_mean"),
                ("rr_zc", "rr_zc_bpm_mean"),
            ),
        ),
    }


def _build_gate_audit(raw_df: pd.DataFrame, feat_df: pd.DataFrame) -> Dict[str, object]:
    audit: Dict[str, object] = {
        "primary_basis": "combined",
        "funnel": {},
        "valid_pairs": {},
    }
    if not isinstance(raw_df, pd.DataFrame) or len(raw_df) == 0:
        return audit
    funnel = {
        "radar_frames_total": int(len(raw_df)),
        "human_frames": int((safe_series(raw_df, "human_detected", default=0.0) >= 0.5).sum()),
        "dsp_frames": int((safe_series(raw_df, "dsp_ran_this_frame", default=0.0) >= 0.5).sum()),
        "phase_valid_frames": int((safe_series(raw_df, "phase_valid_this_frame", default=0.0) >= 0.5).sum()),
        "logged_hr_valid_frames": int((safe_series(raw_df, "logged_hr_valid", default=0.0) >= 0.5).sum()),
        "logged_rr_valid_frames": int((safe_series(raw_df, "logged_rr_valid", default=0.0) >= 0.5).sum()),
        "one_hz_bins": int(len(feat_df)),
        "hr_eval_bins": int(safe_series(feat_df, "hr_valid_for_eval", default=0.0).fillna(0).astype(bool).sum()),
        "rr_eval_bins": int(safe_series(feat_df, "rr_valid_for_eval", default=0.0).fillna(0).astype(bool).sum()),
    }
    audit["funnel"] = funnel
    audit["valid_pairs"] = {
        "combined_hr_pairs": int(np.sum(finite_pair_mask(feat_df, "ref_hr", "reported_hr_mean") & safe_series(feat_df, "hr_valid_for_eval", default=0.0).to_numpy(dtype=bool))),
        "combined_rr_pairs": int(np.sum(finite_pair_mask(feat_df, "ref_rr", "reported_rr_mean") & safe_series(feat_df, "rr_valid_for_eval", default=0.0).to_numpy(dtype=bool))),
    }
    return audit


def _build_agc_anomaly_flags(raw_df: pd.DataFrame) -> Dict[str, object]:
    flags: Dict[str, object] = {
        "gain_floor_pct": 0.0,
        "near_field_pct": 0.0,
        "skipdsp_pct": 0.0,
        "gain_floor_suspected": False,
        "near_field_suspected": False,
        "skipdsp_dead_zone_suspected": False,
    }
    if not isinstance(raw_df, pd.DataFrame) or len(raw_df) == 0:
        return flags
    if "agc_floor_suspect" in raw_df.columns:
        gain_floor = safe_series(raw_df, "agc_floor_suspect", default=0.0).fillna(0.0) >= 0.5
    else:
        gain_floor = safe_series(raw_df, "radar_gain", default=np.nan).fillna(np.nan) <= 0.21
    if "near_field_reflector_suspect" in raw_df.columns:
        near_field = safe_series(raw_df, "near_field_reflector_suspect", default=0.0).fillna(0.0) >= 0.5
    else:
        near_field = safe_series(raw_df, "reflector_distance_cm", default=np.nan).fillna(np.nan) < 10.0
    skipdsp = safe_series(raw_df, "skipdsp_misses", default=0.0).fillna(0.0) >= 3.0
    gain_floor_pct = float(gain_floor.mean() * 100.0)
    near_field_pct = float(near_field.mean() * 100.0)
    skipdsp_pct = float(skipdsp.mean() * 100.0)
    flags["gain_floor_pct"] = gain_floor_pct
    flags["near_field_pct"] = near_field_pct
    flags["skipdsp_pct"] = skipdsp_pct
    flags["gain_floor_suspected"] = bool(gain_floor_pct >= 10.0)
    flags["near_field_suspected"] = bool(near_field_pct >= 10.0)
    flags["skipdsp_dead_zone_suspected"] = bool(skipdsp_pct >= 10.0)
    return flags


def _experimental_hr_gate_metrics(df: pd.DataFrame, mask: np.ndarray) -> Dict[str, object]:
    if not isinstance(df, pd.DataFrame) or len(df) == 0:
        return {"n": 0, "r": float("nan"), "rmse": float("nan"), "spearman_r": float("nan")}
    base_mask = (
        safe_series(df, "hr_valid_for_eval", default=0.0).to_numpy(dtype=bool) &
        finite_pair_mask(df, "ref_hr", "reported_hr_mean")
    )
    final_mask = base_mask & mask
    sub = df.loc[final_mask].copy()
    n = int(len(sub))
    if n < 2:
        return {"n": n, "r": float("nan"), "rmse": float("nan"), "spearman_r": float("nan")}
    ref = pd.to_numeric(sub["ref_hr"], errors="coerce").to_numpy(dtype=float)
    pred = pd.to_numeric(sub["reported_hr_mean"], errors="coerce").to_numpy(dtype=float)
    r = float(np.corrcoef(ref, pred)[0, 1]) if n >= 2 else float("nan")
    rmse = float(np.sqrt(np.mean((pred - ref) ** 2)))
    try:
        spearman_r = float(pd.Series(ref).rank().corr(pd.Series(pred).rank(), method="pearson"))
    except Exception:
        spearman_r = float("nan")
    return {"n": n, "r": r, "rmse": rmse, "spearman_r": spearman_r}


def _compute_internal_consistency_score(summary: Optional[Dict]) -> float:
    if not isinstance(summary, dict):
        return float("nan")
    ml = summary.get("ml_gate", {}) if isinstance(summary.get("ml_gate"), dict) else {}
    hr = summary.get("hr_baseline", {}) if isinstance(summary.get("hr_baseline"), dict) else {}
    vo = hr.get("valid_only", {}) if isinstance(hr.get("valid_only"), dict) else {}
    r = float(vo.get("r", float("nan")))
    rmse = float(vo.get("rmse", float("nan")))
    cov = float(hr.get("coverage_pct", float("nan")))
    pqi_lock = float(summary.get("pqi_lock_pct", float("nan")))

    score = 0.0
    score += 30.0 if ml.get("passed") else 0.0
    if np.isfinite(r):
        score += 20.0 * _clip01(r / 0.4)
    if np.isfinite(rmse):
        score += 20.0 * _clip01((8.0 - rmse) / 8.0)
    if np.isfinite(pqi_lock):
        score += 20.0 * _clip01(pqi_lock / 100.0)
    if np.isfinite(cov):
        score += 10.0 * _clip01(cov / 100.0)
    return round(score, 1)


def _compute_session_quality_score(summary: Optional[Dict], feat_df: Optional[pd.DataFrame] = None) -> float:
    if not isinstance(summary, dict):
        return float("nan")
    internal = _compute_internal_consistency_score(summary)
    hr = summary.get("hr_baseline", {}) if isinstance(summary.get("hr_baseline"), dict) else {}
    rr = summary.get("rr_baseline", {}) if isinstance(summary.get("rr_baseline"), dict) else {}
    hr_vo = hr.get("valid_only", {}) if isinstance(hr.get("valid_only"), dict) else {}
    rr_vo = rr.get("valid_only", {}) if isinstance(rr.get("valid_only"), dict) else {}
    has_ref = any(float((blk.get("all_frames", {}) if isinstance(blk.get("all_frames"), dict) else {}).get("n", 0)) > 0 for blk in (hr, rr))

    if has_ref:
        hr_rmse = float(hr_vo.get("rmse", float("nan")))
        rr_rmse = float(rr_vo.get("rmse", float("nan")))
        hr_r = float(hr_vo.get("r", float("nan")))
        rr_r = float(rr_vo.get("r", float("nan")))
        hr_bias = abs(float(hr_vo.get("bias", float("nan"))))
        rr_bias = abs(float(rr_vo.get("bias", float("nan"))))
        hr_cov = float(hr.get("coverage_pct", float("nan")))
        rr_cov = float(rr.get("coverage_pct", float("nan")))
        score = 0.0
        if np.isfinite(hr_rmse):
            score += 18.0 * _clip01((20.0 - hr_rmse) / 20.0)
        if np.isfinite(rr_rmse):
            score += 18.0 * _clip01((10.0 - rr_rmse) / 10.0)
        if np.isfinite(hr_r):
            score += 14.0 * _clip01((hr_r + 0.2) / 0.8)
        if np.isfinite(rr_r):
            score += 14.0 * _clip01((rr_r + 0.2) / 0.8)
        if np.isfinite(hr_bias):
            score += 10.0 * _clip01((10.0 - hr_bias) / 10.0)
        if np.isfinite(rr_bias):
            score += 10.0 * _clip01((5.0 - rr_bias) / 5.0)
        if np.isfinite(hr_cov):
            score += 8.0 * _clip01(hr_cov / 40.0)
        if np.isfinite(rr_cov):
            score += 8.0 * _clip01(rr_cov / 60.0)
        raw_hr_bias_est = float(((summary.get("raw_hr_bias_estimate") or {}).get("bias_bpm", float("nan"))))
        if np.isfinite(raw_hr_bias_est):
            # v8.3: Temporarily neutralised penalty until v13.7 data validates the new arithmetic
            score -= 0.0 * _clip01((abs(raw_hr_bias_est) - 8.0) / 12.0)
        if bool((summary.get("ml_gate") or {}).get("passed")):
            score += 10.0
        return round(max(0.0, min(100.0, score)), 1)

    score = internal
    if isinstance(feat_df, pd.DataFrame) and len(feat_df):
        human = safe_series(feat_df, "human_detected_mean")
        cand_hr = safe_series(feat_df, "candidate_hr_mean")
        cand_rr = safe_series(feat_df, "candidate_rr_mean")
        if human.notna().any() and human.fillna(0).mean() > 0.5 and ((cand_hr.fillna(0) > 0).any() or (cand_rr.fillna(0) > 0).any()):
            score = 0.0
    return round(max(0.0, min(100.0, score)), 1)


def _extract_golden_metrics(summary: Dict) -> Dict[str, float]:
    out = {}
    if not isinstance(summary, dict):
        return out
    out["quality_score"] = float(summary.get("session_quality_score", float("nan")))
    hr = summary.get("hr_baseline", {}) if isinstance(summary.get("hr_baseline"), dict) else {}
    rr = summary.get("rr_baseline", {}) if isinstance(summary.get("rr_baseline"), dict) else {}
    for prefix, block in (("hr", hr), ("rr", rr)):
        vo = block.get("valid_only", {}) if isinstance(block.get("valid_only"), dict) else {}
        out[f"{prefix}_rmse"] = float(vo.get("rmse", float("nan")))
        out[f"{prefix}_r"] = float(vo.get("r", float("nan")))
        out[f"{prefix}_coverage_pct"] = float(block.get("coverage_pct", float("nan")))
    return out


def _compare_to_golden(summary: Dict, golden_path: str, rmse_tol: float, r_tol: float) -> Dict[str, object]:
    with open(golden_path, "r", encoding="utf-8") as f:
        golden = json.load(f)
    gold = _extract_golden_metrics(golden)
    cur = _extract_golden_metrics(summary)
    checks = []
    status = True
    for target in ("hr", "rr"):
        g_rmse = gold.get(f"{target}_rmse", float("nan"))
        c_rmse = cur.get(f"{target}_rmse", float("nan"))
        if np.isfinite(g_rmse) and np.isfinite(c_rmse):
            ok = c_rmse <= (g_rmse + rmse_tol)
            status = status and ok
            checks.append({"metric": f"{target}_rmse", "golden": g_rmse, "current": c_rmse, "ok": ok})
        g_r = gold.get(f"{target}_r", float("nan"))
        c_r = cur.get(f"{target}_r", float("nan"))
        if np.isfinite(g_r) and np.isfinite(c_r):
            ok = c_r >= (g_r - r_tol)
            status = status and ok
            checks.append({"metric": f"{target}_r", "golden": g_r, "current": c_r, "ok": ok})
    g_q = gold.get("quality_score", float("nan"))
    c_q = cur.get("quality_score", float("nan"))
    if np.isfinite(g_q) and np.isfinite(c_q):
        ok = c_q >= (g_q - 3.0)
        status = status and ok
        checks.append({"metric": "quality_score", "golden": g_q, "current": c_q, "ok": ok})
    return {"passed": bool(status), "golden_path": os.path.abspath(golden_path), "checks": checks}
