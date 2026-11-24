"""Metrics, validity masks, adaptive/PQI shadow models, and ML-readiness verdict."""

from __future__ import annotations

from typing import Any, Dict, List, Mapping, Optional, Sequence

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.metrics import mean_absolute_error, mean_squared_error

from rvt_trainer.constants import (
    BLE_PI_QUALITY_THRESHOLD,
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    HR_VALID_MAX_AGE_MS,
    HR_VALID_PUBLISH_OK,
    MIN_PHASE_VALID_RUN_LEN,
    ML_READY_MIN_BLE_COVERAGE_PCT,
    ML_READY_MIN_HR_BIAS_BPM,
    ML_READY_MIN_HR_COVERAGE_PCT,
    ML_READY_MIN_HR_MAE_BPM,
    ML_READY_MIN_RR_COVERAGE_PCT,
    ML_READY_MIN_RR_MAE_BPM,
    RR_VALID_MAX_AGE_MS,
    RR_VALID_PUBLISH_OK,
    SECONDARY_GATE_MIN_DURATION_S,
    SECONDARY_GATE_MIN_PAIRS,
    _safe_int,
    _session_duration_s,
    _to_num,
    safe_series,
)
from rvt_trainer.features import feature_schema_hash
from rvt_trainer.transport.serial import _is_supported_radar_contract_length


def safe_corr(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    if len(y_true) < 2:
        return float("nan")
    if np.std(y_true) < 1e-9 or np.std(y_pred) < 1e-9:
        return float("nan")
    return float(np.corrcoef(y_true, y_pred)[0, 1])


def lins_ccc(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    if len(y_true) < 2:
        return float("nan")
    mean_t, mean_p = np.mean(y_true), np.mean(y_pred)
    var_t,  var_p  = np.var(y_true, ddof=1), np.var(y_pred, ddof=1)
    covar  = np.sum((y_true - mean_t) * (y_pred - mean_p)) / max(1, (len(y_true) - 1))
    denom  = var_t + var_p + (mean_t - mean_p) ** 2
    if denom < 1e-12:
        return 1.0 if np.allclose(y_true, y_pred, equal_nan=False) else float("nan")
    return float(2.0 * covar / denom)


def longest_false_run(mask: np.ndarray) -> int:
    longest, current = 0, 0
    for flag in mask.astype(bool):
        if flag:
            current = 0
        else:
            current += 1
            longest = max(longest, current)
    return int(longest)


def compute_metrics(y_true: np.ndarray, y_pred: np.ndarray, target: str) -> Dict[str, Any]:
    if len(y_true) == 0:
        return {
            "n": 0, "rmse": float("nan"), "mae": float("nan"), "bias": float("nan"),
            "r": float("nan"), "ccc": float("nan"), "p95_abs_err": float("nan"),
            "max_abs_err": float("nan"), "within_tol_pct": float("nan"),
            "loa_lower": float("nan"), "loa_upper": float("nan"),
        }
    err     = y_pred - y_true
    abs_err = np.abs(err)
    tol     = 5.0 if target == "hr" else 3.0
    bias    = float(np.mean(err))
    sd_err  = float(np.std(err, ddof=1)) if len(err) > 1 else float("nan")
    return {
        "n":              int(len(y_true)),
        "rmse":           float(np.sqrt(mean_squared_error(y_true, y_pred))),
        "mae":            float(mean_absolute_error(y_true, y_pred)),
        "bias":           bias,
        "r":              safe_corr(y_true, y_pred),
        "ccc":            lins_ccc(y_true, y_pred),
        "p95_abs_err":    float(np.percentile(abs_err, 95)),
        "max_abs_err":    float(np.max(abs_err)),
        "within_tol_pct": float(100.0 * np.mean(abs_err <= tol)),
        "loa_lower":      float(bias - 1.96 * sd_err) if np.isfinite(sd_err) else float("nan"),
        "loa_upper":      float(bias + 1.96 * sd_err) if np.isfinite(sd_err) else float("nan"),
    }


def _dynamic_range_gate(y_pred: np.ndarray, *, min_std: float, min_unique: int = 3) -> Dict[str, Any]:
    if len(y_pred) == 0:
        return {"passed": False, "std": float("nan"), "unique_count": 0}
    arr = np.asarray(y_pred, dtype=float)
    arr = arr[np.isfinite(arr)]
    if len(arr) == 0:
        return {"passed": False, "std": float("nan"), "unique_count": 0}
    uniq = np.unique(np.round(arr, 3))
    std = float(np.std(arr, ddof=0)) if len(arr) else float("nan")
    passed = bool(np.isfinite(std) and std >= min_std and len(uniq) >= min_unique)
    return {"passed": passed, "std": std, "unique_count": int(len(uniq))}


def _confidently_wrong_curve(y_true: np.ndarray, y_pred: np.ndarray, target: str) -> Dict[str, float]:
    thresholds = (5.0, 10.0, 15.0) if target == "hr" else (2.0, 4.0, 6.0)
    if len(y_true) == 0:
        return {f"abs_err_gt_{thr:g}": float("nan") for thr in thresholds}
    abs_err = np.abs(np.asarray(y_pred, dtype=float) - np.asarray(y_true, dtype=float))
    return {f"abs_err_gt_{thr:g}": float(100.0 * np.mean(abs_err > thr)) for thr in thresholds}


def _evaluability_block(y_true: np.ndarray, y_pred: np.ndarray, target: str, dyn_gate: Dict[str, Any]) -> Dict[str, object]:
    min_n = 15 if target == "hr" else 30
    preferred_n = 30
    ref = np.asarray(y_true, dtype=float)
    ref = ref[np.isfinite(ref)]
    ref_std = float(np.std(ref, ddof=0)) if len(ref) else float("nan")
    ref_range = float(np.max(ref) - np.min(ref)) if len(ref) else float("nan")
    pred_std = float(dyn_gate.get("std", float("nan")))
    unique_count = int(dyn_gate.get("unique_count", 0))
    min_ref_std = 2.0 if target == "hr" else 0.5
    min_ref_range = 5.0 if target == "hr" else 2.0
    passed = bool(
        len(ref) >= min_n and
        unique_count >= 3 and
        np.isfinite(pred_std) and pred_std >= (2.0 if target == "hr" else 0.5) and
        np.isfinite(ref_std) and ref_std >= min_ref_std and
        np.isfinite(ref_range) and ref_range >= min_ref_range
    )
    return {
        "basis": "trainer_valid_1hz",
        "physiologically_evaluable": passed,
        "status": "evaluable" if passed else "not physiologically evaluable",
        "min_n": min_n,
        "preferred_n": preferred_n,
        "ref_std": ref_std,
        "ref_range": ref_range,
    }


def validity_masks(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()

    def _pick(col_last: str, col_mean: str, default=np.nan) -> pd.Series:
        if col_last in df.columns:
            return safe_series(df, col_last, default=default)
        if col_mean in df.columns:
            return safe_series(df, col_mean, default=default)
        return pd.Series(default, index=df.index, dtype=float)

    warm_mask = (_pick("phase_warmup_complete_last", "phase_warmup_complete_mean", 1.0) >= 0.5)
    phase_run_mask = (_pick("phase_valid_run_len_last", "phase_valid_run_len_mean", 0.0) >= MIN_PHASE_VALID_RUN_LEN)
    agc_mask = (_pick("agc_floor_suspect_last", "agc_floor_suspect_mean", 0.0) < 0.5)
    post_motion_mask = ~(_pick("session_phase_last", "session_phase_mean", -1.0).round().astype(int) == 4)

    trusted_hr_mask = (_pick("trusted_hr_fresh_last", "trusted_hr_fresh_mean", 1.0) >= 0.5)
    trusted_rr_mask = (_pick("trusted_rr_fresh_last", "trusted_rr_fresh_mean", 1.0) >= 0.5)
    rr_source_latched_ok_mask = (_pick("rr_source_latched_ok_last", "rr_source_latched_ok_mean", 0.0) >= 0.5)
    if "rr_source_current_ok_last" in df.columns or "rr_source_current_ok_mean" in df.columns:
        rr_source_current_ok_mask = (_pick("rr_source_current_ok_last", "rr_source_current_ok_mean", 0.0) >= 0.5)
    else:
        # v13.9.7 did not emit rr_source_current_ok. Reconstruct it from
        # rr_gate_reason_last, which is emitted by newer firmware contracts.
        rr_source_current_ok_mask = (_pick("rr_gate_reason_last", "rr_gate_reason_mean", -1.0).round().astype(int) == 0)
    rr_phase_ready_mask = (_pick("rr_phase_backed_publish_ready_last", "rr_phase_backed_publish_ready_mean", 1.0) >= 0.5)

    hr_anchor_source_last = _pick("hr_anchor_source_last", "hr_anchor_source_mean", 0.0).round().astype(int)
    df["raw_anchor_is_derived"] = hr_anchor_source_last.isin([2, 3, 4, 5])

    phase_live_mask = phase_run_mask & warm_mask
    agc_skip_mask = agc_mask
    df["phase_live_mask"] = phase_live_mask
    df["agc_skip_mask"] = agc_skip_mask
    df["post_motion_mask"] = post_motion_mask
    df["rr_source_latched_ok_mask"] = rr_source_latched_ok_mask
    df["rr_source_current_ok_mask"] = rr_source_current_ok_mask
    df["rr_phase_ready_mask"] = rr_phase_ready_mask

    if {"raw_hr_mean", "ref_hr", "ref_rr"}.issubset(df.columns):
        raw_sumfreq = (safe_series(df, "raw_hr_mean") - (safe_series(df, "ref_hr") + safe_series(df, "ref_rr"))).abs() <= 6.0
        df["raw_hr_sumfreq_suspect_mask"] = raw_sumfreq.fillna(False)
    else:
        df["raw_hr_sumfreq_suspect_mask"] = False

    if "logged_hr_valid_mean" in df.columns:
        base_hr = safe_series(df, "logged_hr_valid_mean", default=0.0) >= 0.5
    else:
        base_hr = (
            (safe_series(df, "hr_valid_frac", default=0.0) >= 0.5) &
            (safe_series(df, "reported_hr_mean") > 10)
        )

    if "hr_publish_reason_last" in df.columns:
        pub_hr = safe_series(df, "hr_publish_reason_last", default=-1.0).round().astype(int)
        base_hr = base_hr & pub_hr.isin(HR_VALID_PUBLISH_OK)
    elif "hr_publish_reason_mean" in df.columns:
        base_hr = base_hr & (safe_series(df, "hr_publish_reason_mean", default=99.0) < 0.4)

    if "hr_age_ms_last" in df.columns:
        base_hr = base_hr & (safe_series(df, "hr_age_ms_last", default=0.0) <= HR_VALID_MAX_AGE_MS)
    elif "hr_age_ms_mean" in df.columns:
        base_hr = base_hr & (safe_series(df, "hr_age_ms_mean", default=0.0) <= HR_VALID_MAX_AGE_MS)

    if "hr_updated_this_cycle_last" in df.columns:
        base_hr = base_hr & (safe_series(df, "hr_updated_this_cycle_last", default=1.0) >= 0.5)

    base_hr = base_hr & trusted_hr_mask & phase_run_mask & warm_mask & agc_mask & post_motion_mask

    if "logged_rr_valid_mean" in df.columns:
        base_rr = safe_series(df, "logged_rr_valid_mean", default=0.0) >= 0.5
    else:
        base_rr = (
            (safe_series(df, "rr_valid_frac", default=0.0) >= 0.5) &
            (safe_series(df, "reported_rr_mean") > 0)
        )

    if "rr_publish_reason_last" in df.columns:
        pub_rr = safe_series(df, "rr_publish_reason_last", default=-1.0).round().astype(int)
        base_rr = base_rr & pub_rr.isin(RR_VALID_PUBLISH_OK)
    elif "rr_publish_reason_mean" in df.columns:
        base_rr = base_rr & (safe_series(df, "rr_publish_reason_mean", default=99.0) < 0.4)

    if "candidate_rr_age_ms_last" in df.columns:
        base_rr = base_rr & (safe_series(df, "candidate_rr_age_ms_last", default=0.0) <= RR_VALID_MAX_AGE_MS)
    elif "candidate_rr_age_ms_mean" in df.columns:
        base_rr = base_rr & (safe_series(df, "candidate_rr_age_ms_mean", default=0.0) <= RR_VALID_MAX_AGE_MS)

    rr_source_any_ok_mask = rr_source_current_ok_mask | rr_source_latched_ok_mask
    df["rr_source_any_ok_mask"] = rr_source_any_ok_mask
    base_rr = base_rr & trusted_rr_mask & warm_mask & agc_mask & post_motion_mask & rr_phase_ready_mask & rr_source_any_ok_mask

    df["hr_valid_for_eval"] = base_hr
    df["rr_valid_for_eval"] = base_rr
    df["rr_valid_current_source_for_eval"] = base_rr & rr_source_current_ok_mask
    df["rr_valid_latched_source_for_eval"] = base_rr & (~rr_source_current_ok_mask) & rr_source_latched_ok_mask
    return df


def finite_pair_mask(df: pd.DataFrame, true_col: str, pred_col: str) -> np.ndarray:
    return (safe_series(df, true_col).notna().to_numpy(dtype=bool) &
            safe_series(df, pred_col).notna().to_numpy(dtype=bool))


def baseline_summary(df: pd.DataFrame, target: str) -> Dict[str, Any]:
    pred_col_valid = f"reported_{target}_mean"
    pred_col_all = f"reported_{target}_all_mean" if f"reported_{target}_all_mean" in df.columns else pred_col_valid
    ref_col = f"ref_{target}"
    valid_col = f"{target}_valid_for_eval"
    pair_mask_all = finite_pair_mask(df, ref_col, pred_col_all)
    pair_mask_valid = finite_pair_mask(df, ref_col, pred_col_valid)
    valid_claim_mask = df[valid_col].fillna(False).to_numpy(dtype=bool) if valid_col in df.columns else np.zeros(len(df), dtype=bool)
    valid_mask = pair_mask_valid & valid_claim_mask
    y_true = safe_series(df, ref_col).to_numpy(dtype=float)
    y_pred_valid = safe_series(df, pred_col_valid).to_numpy(dtype=float)
    y_pred_all = safe_series(df, pred_col_all).to_numpy(dtype=float)
    y_pred_zero_invalid = np.where(valid_claim_mask, y_pred_all, 0.0)
    valid_only = compute_metrics(y_true[valid_mask], y_pred_valid[valid_mask], target)
    dyn_gate = _dynamic_range_gate(y_pred_valid[valid_mask], min_std=(2.0 if target == "hr" else 0.5))
    valid_only["dynamic_range_gate_passed"] = bool(dyn_gate["passed"])
    valid_only["pred_std"] = float(dyn_gate["std"])
    valid_only["pred_unique_count"] = int(dyn_gate["unique_count"])
    valid_only.update(_confidently_wrong_curve(y_true[valid_mask], y_pred_valid[valid_mask], target))
    valid_only.update(_evaluability_block(y_true[valid_mask], y_pred_valid[valid_mask], target, dyn_gate))
    all_frames = compute_metrics(y_true[pair_mask_all], y_pred_all[pair_mask_all], target)
    all_frames["basis"] = "aligned_1hz"
    all_frames.update(_confidently_wrong_curve(y_true[pair_mask_all], y_pred_all[pair_mask_all], target))
    all_frames_true = compute_metrics(y_true[pair_mask_all], y_pred_zero_invalid[pair_mask_all], target)
    all_frames_true["basis"] = "aligned_1hz_zero_invalid"
    all_frames_true.update(_confidently_wrong_curve(y_true[pair_mask_all], y_pred_zero_invalid[pair_mask_all], target))
    source_blocks: Dict[str, Dict[str, Any]] = {}
    if target == "rr":
        for label, mask_col in (
            ("valid_current_source", "rr_valid_current_source_for_eval"),
            ("valid_latched_source", "rr_valid_latched_source_for_eval"),
        ):
            source_mask = valid_mask & (
                df[mask_col].fillna(False).to_numpy(dtype=bool)
                if mask_col in df.columns else np.zeros(len(df), dtype=bool)
            )
            block = compute_metrics(y_true[source_mask], y_pred_valid[source_mask], target)
            src_dyn = _dynamic_range_gate(y_pred_valid[source_mask], min_std=0.5)
            block["basis"] = "trainer_valid_1hz"
            block["dynamic_range_gate_passed"] = bool(src_dyn["passed"])
            block["pred_std"] = float(src_dyn["std"])
            block["pred_unique_count"] = int(src_dyn["unique_count"])
            block.update(_confidently_wrong_curve(y_true[source_mask], y_pred_valid[source_mask], target))
            block["coverage_pct"] = float(100.0 * source_mask.mean()) if len(source_mask) else float("nan")
            source_blocks[label] = block
    coverage_frac = float(valid_mask.mean()) if len(valid_mask) else float("nan")
    pipeline_coverage_frac = float(valid_claim_mask.mean()) if len(valid_claim_mask) else float("nan")
    cov_pen_rmse = float(valid_only.get("rmse", float("nan"))) / max(np.sqrt(coverage_frac), 0.1) if np.isfinite(coverage_frac) else float("nan")
    out = {
        "all_frames": all_frames,
        "all_frames_true": all_frames_true,
        "valid_only": valid_only,
        "coverage_pct": float(100.0 * coverage_frac) if np.isfinite(coverage_frac) else float("nan"),
        "pipeline_coverage_pct": float(100.0 * pipeline_coverage_frac) if np.isfinite(pipeline_coverage_frac) else float("nan"),
        "ref_aligned_coverage_pct": float(100.0 * coverage_frac) if np.isfinite(coverage_frac) else float("nan"),
        "coverage_penalized_rmse": cov_pen_rmse,
        "longest_invalid_run_s": longest_false_run(valid_mask),
    }
    out.update(source_blocks)
    return out


def compute_hr_bias_estimate(df: pd.DataFrame, raw_candidates: Optional[Sequence[str]] = None, series_label: str = "raw_hr") -> Dict[str, object]:
    raw_candidates = list(raw_candidates or ["raw_hr_nonzero_mean", "raw_hr_mean"])
    raw_col = next((c for c in raw_candidates if c in df.columns), None)
    raw = safe_series(df, raw_col) if raw_col else pd.Series(dtype=float)
    ref = safe_series(df, "ref_hr") if "ref_hr" in df.columns else pd.Series(dtype=float)
    mask = raw.notna() & ref.notna() & (raw > 0) & (ref > 0)
    raw_key = f"{series_label}_mean"
    if not mask.any():
        return {
            "series_label": series_label,
            "source_column": raw_col,
            "n": 0,
            "bias_bpm": float("nan"),
            raw_key: float("nan"),
            "ref_hr_mean": float("nan"),
            "rate_buckets": [],
        }
    raw_v = raw[mask].astype(float)
    ref_v = ref[mask].astype(float)
    buckets = []
    bucket_edges = [30, 60, 80, 100, 120, 220]
    bucket_labels = ["30-60", "60-80", "80-100", "100-120", "120-220"]
    for lo, hi, label in zip(bucket_edges[:-1], bucket_edges[1:], bucket_labels):
        bmask = (ref_v >= lo) & ((ref_v < hi) if hi < 220 else (ref_v <= hi))
        if not bmask.any():
            buckets.append({
                "bucket": label,
                "ref_hr_min": float(lo),
                "ref_hr_max": float(hi),
                "n": 0,
                "bias_bpm": float("nan"),
                raw_key: float("nan"),
                "ref_hr_mean": float("nan"),
            })
            continue
        braw = raw_v[bmask]
        bref = ref_v[bmask]
        buckets.append({
            "bucket": label,
            "ref_hr_min": float(lo),
            "ref_hr_max": float(hi),
            "n": int(bmask.sum()),
            "bias_bpm": float((braw - bref).mean()),
            raw_key: float(braw.mean()),
            "ref_hr_mean": float(bref.mean()),
        })
    return {
        "series_label": series_label,
        "source_column": raw_col,
        "n": int(mask.sum()),
        "bias_bpm": float((raw_v - ref_v).mean()),
        raw_key: float(raw_v.mean()),
        "ref_hr_mean": float(ref_v.mean()),
        "rate_buckets": buckets,
    }


def compute_raw_hr_bias_estimate(df: pd.DataFrame) -> Dict[str, object]:
    return compute_hr_bias_estimate(df, ["raw_hr_nonzero_mean", "raw_hr_mean"], series_label="raw_hr")


def compute_raw_hr_sumfreq_error_estimate(df: pd.DataFrame) -> Dict[str, float]:
    raw_col = "raw_hr_mean" if "raw_hr_mean" in df.columns else None
    if "raw_hr_nonzero_mean" in df.columns:
        raw_col = "raw_hr_nonzero_mean"
    raw = safe_series(df, raw_col) if raw_col else pd.Series(dtype=float)
    ref_hr = safe_series(df, "ref_hr") if "ref_hr" in df.columns else pd.Series(dtype=float)
    ref_rr = safe_series(df, "ref_rr") if "ref_rr" in df.columns else pd.Series(dtype=float)
    expected = ref_hr + ref_rr
    mask = raw.notna() & ref_hr.notna() & ref_rr.notna() & (raw > 0) & (ref_hr > 0) & (ref_rr > 0)
    if not mask.any():
        return {"n": 0, "raw_hr_mean": float("nan"), "expected_sumfreq_bpm_mean": float("nan"), "mean_abs_error_bpm": float("nan"), "bias_bpm": float("nan")}
    raw_v = raw[mask].astype(float)
    exp_v = expected[mask].astype(float)
    diff = raw_v - exp_v
    return {
        "n": int(mask.sum()),
        "raw_hr_mean": float(raw_v.mean()),
        "expected_sumfreq_bpm_mean": float(exp_v.mean()),
        "mean_abs_error_bpm": float(diff.abs().mean()),
        "bias_bpm": float(diff.mean()),
    }


def model_summary(df: pd.DataFrame, target: str) -> Dict[str, Any]:
    pred_col  = f"pred_{target}"
    ref_col   = f"ref_{target}"
    valid_col = f"{target}_valid_for_eval"
    pair_mask = finite_pair_mask(df, ref_col, pred_col)
    valid_claim_mask = df[valid_col].fillna(False).to_numpy(dtype=bool) if valid_col in df.columns \
                       else np.zeros(len(df), dtype=bool)
    valid_mask = pair_mask & valid_claim_mask
    y_true = safe_series(df, ref_col).to_numpy(dtype=float)
    y_pred = safe_series(df, pred_col).to_numpy(dtype=float)
    return {
        "all_frames":          compute_metrics(y_true[pair_mask],  y_pred[pair_mask],  target),
        "valid_only":          compute_metrics(y_true[valid_mask], y_pred[valid_mask], target),
        "coverage_pct":        float(100.0 * valid_mask.mean()) if len(valid_mask) else float("nan"),
        "longest_invalid_run_s": longest_false_run(valid_mask),
    }


def print_metric_block(label: str, metrics: Mapping[str, Any]):
    print(f"\n[{label}]")
    af = metrics["all_frames"]
    aft = metrics.get("all_frames_true", {})
    vo = metrics["valid_only"]
    print(f"  all-frames   | n={af['n']:4d} | RMSE={af['rmse']:.2f} | "
          f"Bias={af['bias']:+.2f} | MAE={af['mae']:.2f} | r={af['r']:.3f} | CCC={af['ccc']:.3f}")
    if aft:
        print(f"  all-true     | n={aft['n']:4d} | RMSE={aft['rmse']:.2f} | "
              f"Bias={aft['bias']:+.2f} | MAE={aft['mae']:.2f} | r={aft['r']:.3f} | CCC={aft['ccc']:.3f}")
    print(f"  valid-only   | n={vo['n']:4d} | RMSE={vo['rmse']:.2f} | "
          f"Bias={vo['bias']:+.2f} | MAE={vo['mae']:.2f} | r={vo['r']:.3f} | CCC={vo['ccc']:.3f}")
    print(f"  p95 | max    | {vo['p95_abs_err']:.2f} | {vo['max_abs_err']:.2f}")
    print(f"  within tol   | {vo['within_tol_pct']:.1f}%")
    print(f"  coverage     | aligned-valid={metrics.get('coverage_pct', float('nan')):.1f}% | pipeline={metrics.get('pipeline_coverage_pct', float('nan')):.1f}%")
    cpr = float(metrics.get('coverage_penalized_rmse', float('nan')))
    print(f"  cov-pen RMSE | {cpr:.2f}" if np.isfinite(cpr) else "  cov-pen RMSE | nan")
    print(f"  longest drop | {metrics['longest_invalid_run_s']} s")


def maybe_plot_timeseries(df, target, out_path, pred_col=None, enabled=True):
    if not enabled:
        return
    ref_col  = f"ref_{target}"
    base_col = f"reported_{target}_mean"
    plt.figure(figsize=(10, 4.5))
    plt.plot(df["timestamp_s"], df[ref_col],  label=f"ref_{target}")
    plt.plot(df["timestamp_s"], df[base_col], label=f"reported_{target}")
    if pred_col and pred_col in df.columns:
        plt.plot(df["timestamp_s"], df[pred_col], label=pred_col)
    plt.xlabel("timestamp_s")
    plt.ylabel("BPM" if target == "hr" else "br/min")
    plt.title(f"{target.upper()} overlay")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def maybe_plot_phase_signals(df, out_path, enabled=True):
    if not enabled or "timestamp_s" not in df.columns:
        return
    phase_cols = [c for c in ["heart_phase", "breath_phase", "heart_phase_stabilized", "breath_phase_stabilized"] if c in df.columns]
    if not phase_cols:
        return
    plt.figure(figsize=(10, 4.8))
    for c in phase_cols:
        vals = pd.to_numeric(df[c], errors="coerce")
        if vals.notna().any():
            plt.plot(df["timestamp_s"], vals, label=c)
    plt.xlabel("timestamp_s")
    plt.ylabel("phase")
    plt.title("Stabilized phase signals over time")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def maybe_plot_bland_altman(y_true, y_pred, title, out_path, enabled=True, annotate=False):
    if not enabled or len(y_true) == 0:
        return
    mean = (y_true + y_pred) / 2.0
    diff = y_pred - y_true
    bias = np.mean(diff)
    sd   = np.std(diff, ddof=1) if len(diff) > 1 else np.nan
    loa_u = bias + 1.96 * sd if np.isfinite(sd) else np.nan
    loa_l = bias - 1.96 * sd if np.isfinite(sd) else np.nan
    plt.figure(figsize=(6.5, 5.0))
    plt.scatter(mean, diff, s=12, alpha=0.7)
    plt.axhline(bias, linestyle="--")
    if np.isfinite(loa_u):
        plt.axhline(loa_u, linestyle=":")
    if np.isfinite(loa_l):
        plt.axhline(loa_l, linestyle=":")
    plt.xlabel("Mean of reference and estimate")
    plt.ylabel("Estimate - reference")
    plt.title(title)
    if annotate:
        note = (f"n={len(diff)}\nBias={bias:+.2f}\nLoA=[{loa_l:+.2f}, {loa_u:+.2f}]"
                if np.isfinite(loa_u) else f"n={len(diff)}\nBias={bias:+.2f}")
        plt.gca().text(0.02, 0.98, note, transform=plt.gca().transAxes, va="top", ha="left",
                       fontsize=8, bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def save_feature_importance(model, feature_names, out_path):
    imp = pd.DataFrame({"feature": list(feature_names),
                        "importance": model.feature_importances_}
                       ).sort_values("importance", ascending=False)
    imp.to_csv(out_path, index=False)


def get_valid_pair_arrays(df, target, pred_col, valid_only=True):
    ref_col   = f"ref_{target}"
    pair_mask = finite_pair_mask(df, ref_col, pred_col)
    if valid_only:
        valid_col  = f"{target}_valid_for_eval"
        claim_mask = df[valid_col].fillna(False).to_numpy(dtype=bool) if valid_col in df.columns \
                     else np.zeros(len(df), dtype=bool)
        pair_mask  = pair_mask & claim_mask
    y_true = safe_series(df, ref_col).to_numpy(dtype=float)[pair_mask]
    y_pred = safe_series(df, pred_col).to_numpy(dtype=float)[pair_mask]
    return y_true, y_pred


def _fit_piecewise3(x_arr, y_arr, breakpoints=(58.0, 80.0)) -> Optional[Dict[str, object]]:
    """v15 Stage E: 3-segment piecewise-linear fit for adaptive correction.

    Fits residual ≈ offset + slope_low*max(bp1-x,0) + slope_mid*x_mid + slope_high*max(x-bp2,0)
    via OLS. Returns coefficients dict or None if insufficient data.
    """
    try:
        import numpy as _np
    except Exception:
        return None
    bp1, bp2 = float(breakpoints[0]), float(breakpoints[1])
    x = _np.asarray(x_arr, dtype=float)
    y = _np.asarray(y_arr, dtype=float)
    finite = _np.isfinite(x) & _np.isfinite(y)
    x = x[finite]
    y = y[finite]
    if x.size < 30:
        return None
    f_low = _np.maximum(bp1 - x, 0.0)
    f_mid = _np.clip(x, bp1, bp2)
    f_high = _np.maximum(x - bp2, 0.0)
    X = _np.column_stack([_np.ones_like(x), f_low, f_mid, f_high])
    try:
        beta, *_ = _np.linalg.lstsq(X, y, rcond=None)
    except _np.linalg.LinAlgError:
        return None
    pred = X @ beta
    resid = y - pred
    return {
        "breakpoints": [bp1, bp2],
        "offset": float(beta[0]),
        "slope_low": float(-beta[1]),
        "slope_mid": float(beta[2]),
        "slope_high": float(beta[3]),
        "train_rmse": float(_np.sqrt(_np.mean(resid * resid))),
        "n_points": int(x.size),
    }


def _apply_piecewise3(x_arr, fit):
    """Apply a piecewise3 fit to an x array. Returns predicted residuals as numpy array."""
    if fit is None:
        return None
    try:
        import numpy as _np
    except Exception:
        return None
    x = _np.asarray(x_arr, dtype=float)
    bp1, bp2 = fit["breakpoints"]
    f_low = _np.maximum(bp1 - x, 0.0)
    f_mid = _np.clip(x, bp1, bp2)
    f_high = _np.maximum(x - bp2, 0.0)
    return (fit["offset"]
            + (-fit["slope_low"]) * f_low
            + fit["slope_mid"] * f_mid
            + fit["slope_high"] * f_high)


def _build_adaptive_correction_shadow(df) -> Dict[str, object]:
    """v15.0 Stage E: dual-domain piecewise-linear correction shadow analysis.

    Computes BOTH a reference-domain fit (residual vs ref_hr — analysis quality
    metric only, NOT runnable on firmware) AND a deployment-domain fit
    (residual vs raw_hr_uncorrected — the only fit runnable on firmware
    without BLE leakage).

    Cross-domain RMSE compares the two fits' predictions; low cross-RMSE means
    the deployment-domain fit can stand in for the reference-domain fit, which
    is the prerequisite for v12 live correction push.

    Validation RMSE: fit on first half, evaluate on second half. Surfaces
    within-session generalization.

    push_eligible is ALWAYS False in v11 — Stage E is analysis-only.
    """
    out: Dict[str, Any] = {
        "schema_version": "v15.0",
        "enabled": False,
        "n_paired_points": 0,
        "firmware_default_rmse_bpm": None,
        "reference_domain_fit": None,
        "deployment_domain_fit": None,
        "cross_domain_rmse_bpm": None,
        "validation_rmse_bpm": None,
        "improvement_bpm": None,
        "push_eligible": False,
        "push_reason": "Shadow mode only in v11 — live push deferred to v12 after 5-session validation",
        "recommendation": None,
    }
    try:
        import numpy as _np
        import pandas as _pd
    except Exception:
        return out
    if df is None or not hasattr(df, "columns") or len(df) == 0:
        return out

    # PR72 session-data audit: this shadow runs on the 1 Hz feature frame,
    # where radar columns carry aggregation suffixes (raw_hr_uncorrected_mean),
    # so the unsuffixed names never matched and the shadow was skipped on every
    # session ("missing one of ...") even though the data was present.
    def _resolve_column(*candidates: str) -> Optional[str]:
        for cand in candidates:
            if cand in df.columns:
                return cand
        return None

    unc_col = _resolve_column("raw_hr_uncorrected", "raw_hr_uncorrected_mean")
    cor_col = _resolve_column("raw_hr_corrected", "raw_hr_corrected_mean")
    ref_col = _resolve_column("ref_hr", "ref_hr_mean")
    missing = [label for label, col in (
        ("raw_hr_uncorrected", unc_col),
        ("raw_hr_corrected", cor_col),
        ("ref_hr", ref_col),
    ) if col is None]
    if missing:
        out["recommendation"] = (
            f"Adaptive correction shadow skipped: missing {tuple(missing)}."
        )
        return out

    motion_col = None
    for c in ("in_motion", "is_motion", "motion_active"):
        if c in df.columns:
            motion_col = c
            break

    raw_unc = _pd.to_numeric(df[unc_col], errors="coerce")
    raw_cor = _pd.to_numeric(df[cor_col], errors="coerce")
    ref_hr  = _pd.to_numeric(df[ref_col], errors="coerce")
    valid_mask = (raw_unc.notna() & raw_cor.notna() & ref_hr.notna()
                  & (raw_unc > 30.0) & (raw_cor > 30.0) & (ref_hr > 30.0)
                  & (raw_unc < 200.0) & (raw_cor < 200.0) & (ref_hr < 200.0))
    if motion_col is not None:
        motion_mask = _pd.to_numeric(df[motion_col], errors="coerce").fillna(0).astype(int) == 0
        valid_mask = valid_mask & motion_mask

    paired = df[valid_mask]
    n_paired = int(len(paired))
    out["n_paired_points"] = n_paired
    if n_paired < 60:
        out["recommendation"] = (
            f"Insufficient paired non-motion points ({n_paired} < 60) for adaptive correction fit."
        )
        return out

    raw_unc_p = _pd.to_numeric(paired[unc_col], errors="coerce").to_numpy(dtype=float)
    raw_cor_p = _pd.to_numeric(paired[cor_col], errors="coerce").to_numpy(dtype=float)
    ref_hr_p  = _pd.to_numeric(paired[ref_col], errors="coerce").to_numpy(dtype=float)

    residual = raw_cor_p - ref_hr_p
    out["firmware_default_rmse_bpm"] = float(_np.sqrt(_np.mean(residual * residual)))

    ref_fit = _fit_piecewise3(ref_hr_p, residual, breakpoints=(58.0, 80.0))
    dep_fit = _fit_piecewise3(raw_unc_p, residual, breakpoints=(58.0, 80.0))
    out["reference_domain_fit"] = ref_fit
    out["deployment_domain_fit"] = dep_fit
    if ref_fit is None or dep_fit is None:
        out["recommendation"] = "Piecewise fit failed (degenerate input)."
        return out

    out["enabled"] = True
    out["improvement_bpm"] = float(out["firmware_default_rmse_bpm"] - dep_fit["train_rmse"])

    dep_pred = _apply_piecewise3(raw_unc_p, dep_fit)
    ref_pred = _apply_piecewise3(ref_hr_p, ref_fit)
    if dep_pred is not None and ref_pred is not None:
        out["cross_domain_rmse_bpm"] = float(_np.sqrt(_np.mean((dep_pred - ref_pred) ** 2)))

    mid = n_paired // 2
    if mid >= 30:
        dep_fit_h1 = _fit_piecewise3(raw_unc_p[:mid], residual[:mid], breakpoints=(58.0, 80.0))
        if dep_fit_h1 is not None:
            dep_pred_h2 = _apply_piecewise3(raw_unc_p[mid:], dep_fit_h1)
            if dep_pred_h2 is not None:
                val_resid = residual[mid:] - dep_pred_h2
                out["validation_rmse_bpm"] = float(_np.sqrt(_np.mean(val_resid * val_resid)))

    improvement = out["improvement_bpm"]
    cross_rmse = out["cross_domain_rmse_bpm"]
    val_rmse = out["validation_rmse_bpm"]
    train_rmse = dep_fit["train_rmse"]
    if (improvement is not None and improvement >= 1.5
        and cross_rmse is not None and cross_rmse < 2.0
        and val_rmse is not None and abs(val_rmse - train_rmse) < 1.5):
        out["recommendation"] = (
            f"PASS Stage E shadow: improvement {improvement:.2f} BPM, "
            f"cross-domain RMSE {cross_rmse:.2f}, validation within "
            f"{abs(val_rmse - train_rmse):.2f} of train. Eligible for v12 live push after 5-session aggregation."
        )
    elif improvement is not None and improvement < 1.5:
        out["recommendation"] = (
            f"INSUFFICIENT GAIN: improvement {improvement:.2f} BPM < 1.5 BPM threshold."
        )
    elif cross_rmse is not None and cross_rmse >= 2.0:
        out["recommendation"] = (
            f"DOMAIN MISMATCH: cross-domain RMSE {cross_rmse:.2f} BPM >= 2.0. "
            f"Deployment-domain fit does not match reference-domain fit — "
            f"firmware-only correction would not reproduce reference-domain quality."
        )
    elif val_rmse is not None and abs(val_rmse - train_rmse) >= 1.5:
        out["recommendation"] = (
            f"OVERFIT: validation {val_rmse:.2f} differs from train {train_rmse:.2f} "
            f"by >= 1.5 BPM — fit does not generalize within session."
        )
    else:
        out["recommendation"] = "Insufficient data for Stage E recommendation."

    return out


def _build_pqi_v15_shadow(df) -> Dict[str, object]:
    """v15.0 Stage C: Compare gap-aware (v15) PQI against legacy (v14) PQI.

    Returns a structured shadow-analysis dict written into analyse_summary.json
    under "pqi_v15_shadow". This is diagnostic-only — no firmware gate or
    candidate path consumes these values. The motion_frame_inflation_check is
    the go/no-go criterion for Stage D activation: if pqi_v15>0.30 occurs on
    motion frames more than 10% of the time, v15 PQI is rewarding motion
    artifacts and Stage D must NOT be activated.
    """
    out: Dict[str, Any] = {
        "schema_version": "v15.0",
        "enabled": False,
        "n_rows": 0,
        "n_v15_active": 0,
        "hr_pqi_v14_mean": None,
        "hr_pqi_v15_mean": None,
        "hr_pqi_v14_above_030_pct": None,
        "hr_pqi_v15_above_030_pct": None,
        "hr_gate_unlock_pp": None,
        "rr_pqi_v14_mean": None,
        "rr_pqi_v15_mean": None,
        "phase_buffer_valid_pct_mean": None,
        "pqi_v15_pair_coverage_min_mean": None,
        "motion_frame_inflation_check": {
            "n_motion_rows": 0,
            "v15_above_030_on_motion_pct": None,
            "threshold_pct": 10.0,
            "passes": None,
            "note": (
                "If v15_above_030_on_motion_pct exceeds threshold_pct, the gap-aware "
                "PQI is rewarding motion artifacts. Stage D activation MUST be blocked "
                "until the lag-energy weighting is hardened."
            ),
        },
        "recommendation": None,
    }
    try:
        import numpy as _np
        import pandas as _pd
    except Exception:
        return out
    if df is None or not hasattr(df, "columns") or len(df) == 0:
        return out

    # The v15 firmware emits these columns; v14.1 firmware (or padded rows) emits
    # safe defaults (-1.0 for floats, 0 for ints). Detect activation by counting
    # rows where pqi_heart_v15 is finite and >= 0.
    v15_cols = ("pqi_heart_v15", "pqi_breath_v15",
                "phase_buffer_valid_pct", "pqi_v15_pair_coverage_min")
    if not all(c in df.columns for c in v15_cols):
        out["recommendation"] = "v15 PQI columns not present in this frame (firmware <v15.0, or frame lacks the raw radar columns). No shadow data."
        out["n_rows"] = int(len(df))
        return out

    n_rows = int(len(df))
    out["n_rows"] = n_rows

    # A row is "v15-active" when pqi_heart_v15 is finite and >= 0
    # (the firmware default sentinel is -1.0).
    hr_v15 = _pd.to_numeric(df["pqi_heart_v15"], errors="coerce")
    rr_v15 = _pd.to_numeric(df["pqi_breath_v15"], errors="coerce")
    valid_pct = _pd.to_numeric(df["phase_buffer_valid_pct"], errors="coerce")
    coverage_min = _pd.to_numeric(df["pqi_v15_pair_coverage_min"], errors="coerce")
    active_mask = hr_v15.notna() & (hr_v15 >= 0.0)
    n_active = int(active_mask.sum())
    out["n_v15_active"] = n_active

    if n_active == 0:
        out["recommendation"] = "v15 PQI sentinel (-1.0) on all rows — Stage C compute path never ran."
        return out

    out["enabled"] = True

    # v14 PQI columns already exist in v14.1+ schemas
    hr_v14 = _pd.to_numeric(df.get("pqi_heart"), errors="coerce") \
             if "pqi_heart" in df.columns else _pd.Series([_np.nan] * n_rows)
    rr_v14 = _pd.to_numeric(df.get("pqi_breath"), errors="coerce") \
             if "pqi_breath" in df.columns else _pd.Series([_np.nan] * n_rows)

    # Restrict comparisons to v15-active rows
    hr_v14a = hr_v14[active_mask]
    hr_v15a = hr_v15[active_mask]
    rr_v14a = rr_v14[active_mask]
    rr_v15a = rr_v15[active_mask]

    def _safe_mean(s):
        s2 = s.dropna() if hasattr(s, "dropna") else s
        if len(s2) == 0:
            return None
        return float(s2.mean())

    def _safe_above(s, thr):
        s2 = s.dropna() if hasattr(s, "dropna") else s
        if len(s2) == 0:
            return None
        return float(100.0 * (s2 >= thr).mean())

    out["hr_pqi_v14_mean"] = _safe_mean(hr_v14a)
    out["hr_pqi_v15_mean"] = _safe_mean(hr_v15a)
    out["hr_pqi_v14_above_030_pct"] = _safe_above(hr_v14a, 0.30)
    out["hr_pqi_v15_above_030_pct"] = _safe_above(hr_v15a, 0.30)
    out["rr_pqi_v14_mean"] = _safe_mean(rr_v14a)
    out["rr_pqi_v15_mean"] = _safe_mean(rr_v15a)
    out["phase_buffer_valid_pct_mean"] = _safe_mean(valid_pct[active_mask])
    out["pqi_v15_pair_coverage_min_mean"] = _safe_mean(coverage_min[active_mask])

    # Gate unlock estimate (HR): if pqi_v15>=0.30 on a row where pqi_v14<0.30,
    # that row would gain candidate eligibility under Stage D activation.
    if out["hr_pqi_v15_above_030_pct"] is not None and out["hr_pqi_v14_above_030_pct"] is not None:
        out["hr_gate_unlock_pp"] = float(
            out["hr_pqi_v15_above_030_pct"] - out["hr_pqi_v14_above_030_pct"]
        )

    # Motion-frame inflation check (Stage D go/no-go)
    motion_col = None
    for c in ("in_motion", "is_motion", "motion_active"):
        if c in df.columns:
            motion_col = c
            break
    mfic: Dict[str, Any] = out["motion_frame_inflation_check"]
    if motion_col is not None:
        motion_vals = _pd.to_numeric(df[motion_col], errors="coerce").fillna(0).astype(int)
        motion_active_mask = active_mask & (motion_vals > 0)
        n_motion = int(motion_active_mask.sum())
        mfic["n_motion_rows"] = n_motion
        if n_motion >= 10:
            v15_motion = hr_v15[motion_active_mask].dropna()
            if len(v15_motion) > 0:
                pct = float(100.0 * (v15_motion >= 0.30).mean())
                mfic["v15_above_030_on_motion_pct"] = pct
                mfic["passes"] = bool(pct <= mfic["threshold_pct"])
        else:
            mfic["passes"] = None  # insufficient motion frames to judge

    # Recommendation string
    unlock = out["hr_gate_unlock_pp"]
    inflation_passes = mfic.get("passes")
    n_motion = mfic.get("n_motion_rows", 0) or 0
    # If no motion frames in session, inflation check is vacuously satisfied
    inflation_clear = (inflation_passes is True) or (n_motion < 10)
    if unlock is not None and unlock >= 15.0 and inflation_clear:
        if n_motion < 10:
            note = f" (motion check skipped: only {n_motion} motion frames in session)"
        else:
            note = ""
        out["recommendation"] = (
            f"PASS Stage D candidate: v15 PQI would unlock {unlock:.1f}pp more HR-eligible "
            f"frames AND motion inflation check passes{note}. Validate on 2 more sessions before EXP_USE_PQI_V15=true."
        )
    elif inflation_passes is False:
        out["recommendation"] = (
            f"BLOCK Stage D: motion-frame inflation pct ({mfic.get('v15_above_030_on_motion_pct')}) "
            f"exceeds threshold ({mfic['threshold_pct']}). Gap-aware PQI is rewarding motion artifacts."
        )
    elif unlock is not None and unlock < 15.0:
        out["recommendation"] = (
            f"INSUFFICIENT GAIN: v15 PQI unlocks only {unlock:.1f}pp HR-eligible frames "
            f"(<15pp threshold). Stage D not justified by current evidence."
        )
    else:
        out["recommendation"] = "Insufficient data for Stage D recommendation."

    return out


def _build_ml_readiness_verdict(analyse_summary, **kwargs) -> Dict[str, object]:
    if not isinstance(analyse_summary, dict):
        return {
            "verdict": "not_ready",
            "readiness_kind": "deferred",
            "headline": "Analysis summary is unavailable",
            "schema_verdict": "firmware_rejected",
            "signal_verdict": "deferred",
            "reference_verdict": "deferred",
            "training_verdict": "deferred",
            "categories": [{"id": "analysis", "label": "Analysis", "status": "fail", "detail": "Missing analysis summary", "remediation": "Rerun analysis."}],
            "passed": [],
            "failed": ["analysis summary missing"],
            "limitation_kind": "unknown",
            "next_action": "rerun analysis",
        }
    raw_gate = analyse_summary.get("ml_gate")
    gate = raw_gate if isinstance(raw_gate, dict) else {}
    raw_secondary_gate = gate.get("secondary_gate")
    secondary_gate = raw_secondary_gate if isinstance(raw_secondary_gate, dict) else {}
    raw_ble = analyse_summary.get("ble_ref_quality")
    ble = raw_ble if isinstance(raw_ble, dict) else {}
    raw_fw = analyse_summary.get("fw_truthfulness")
    fw = raw_fw if isinstance(raw_fw, dict) else {}
    pqi = _to_num(analyse_summary.get("pqi_lock_pct"), float("nan"))
    # PR72 session-data audit: prefer true time coverage over the packet ratio
    # (distilled_rows_pct_of_raw), which protocol-normally sits near 33% and
    # made the reference gate fail every session.
    coverage = _to_num(ble.get("coverage_pct"), float("nan"))
    if not np.isfinite(coverage):
        coverage = _to_num(ble.get("distilled_rows_pct_of_raw"), float("nan"))
    pi = _to_num(ble.get("pi_median"), float("nan"))
    categories: List[Dict[str, str]] = []
    passed: List[str] = []
    failed: List[str] = []

    def add_cat(cid: str, label: str, status: str, detail: str, remediation: str = ""):
        categories.append({"id": cid, "label": label, "status": status, "detail": detail, "remediation": remediation})
        if status == "pass":
            passed.append(label)
        elif status in {"fail", "warn"}:
            failed.append(label)

    contract_len = _safe_int(fw.get("contract_length", EXPECTED_RADAR_LOG_COLUMN_COUNT), EXPECTED_RADAR_LOG_COLUMN_COUNT)
    critical_cols_ok = bool(fw.get("critical_columns_ok", False))
    module_valid = bool(fw.get("module_version_valid", False))
    schema_hash = analyse_summary.get("feature_schema_hash")
    schema_hash_ok = not bool(schema_hash) or schema_hash == feature_schema_hash()
    secondary_kind = str(secondary_gate.get("kind", "")).lower()
    secondary_basis = str(secondary_gate.get("basis", ""))
    raw_hr_base = analyse_summary.get("hr_baseline")
    hr_base = raw_hr_base if isinstance(raw_hr_base, dict) else {}
    raw_rr_base = analyse_summary.get("rr_baseline")
    rr_base = raw_rr_base if isinstance(raw_rr_base, dict) else {}
    raw_hr_valid = hr_base.get("valid_only")
    hr_valid = raw_hr_valid if isinstance(raw_hr_valid, dict) else {}
    raw_rr_valid = rr_base.get("valid_only")
    rr_valid = raw_rr_valid if isinstance(raw_rr_valid, dict) else {}
    hr_cov = _to_num(hr_base.get("coverage_pct"), float("nan"))
    rr_cov = _to_num(rr_base.get("coverage_pct"), float("nan"))
    hr_mae = _to_num(hr_valid.get("mae"), float("nan"))
    hr_bias = _to_num(hr_valid.get("bias"), float("nan"))
    rr_mae = _to_num(rr_valid.get("mae"), float("nan"))
    raw_publish_hist = analyse_summary.get("publish_reason_histogram")
    publish_hist = raw_publish_hist if isinstance(raw_publish_hist, dict) else {}
    raw_publish_hr = publish_hist.get("hr")
    publish_hr = raw_publish_hr if isinstance(raw_publish_hr, dict) else publish_hist
    raw_publish_rr = publish_hist.get("rr")
    publish_rr = raw_publish_rr if isinstance(raw_publish_rr, dict) else publish_hist
    total_publish = sum(_safe_int(v, 0) for v in list(publish_hr.values()) + list(publish_rr.values())) if isinstance(raw_publish_hist, dict) else 0
    suppress = sum(_safe_int(publish_hr.get(k), 0) for k in ("TRUST_STALE", "NO_EVIDENCE", "STATE"))
    suppress += sum(_safe_int(publish_rr.get(k), 0) for k in ("TRUST_STALE", "NO_EVIDENCE", "STALE"))
    suppress_frac = suppress / max(total_publish, 1)
    gate_status = str(gate.get("status", "")).lower()

    schema_verdict = "ok"
    signal_verdict = "ok"
    reference_verdict = "ok"
    training_verdict = "not_ready"
    readiness_kind = "not_ready"
    limitation_kind = None
    next_action = "ship"

    if not fw:
        schema_verdict = "unknown"
        readiness_kind = "not_ready"
        limitation_kind = "unknown"
        next_action = "rerun analysis with firmware truthfulness enabled"
        add_cat("unknown", "Firmware truthfulness", "fail", "Firmware truthfulness metadata is missing", "Rerun analysis with the current trainer.")
    elif not _is_supported_radar_contract_length(contract_len):
        schema_verdict = "firmware_rejected"
        readiness_kind = "firmware_rejected"
        limitation_kind = "unknown"
        next_action = "repair firmware schema or session export"
        add_cat("firmware", "Firmware schema", "fail", f"Observed contract length {contract_len}; expected {EXPECTED_RADAR_LOG_COLUMN_COUNT} or supported legacy width", "Use a session collected with the expected radar schema.")
    elif not critical_cols_ok or contract_len <= 0:
        schema_verdict = "firmware_rejected"
        readiness_kind = "firmware_rejected"
        limitation_kind = "schema"
        next_action = "repair firmware schema or session export"
        add_cat("firmware", "Firmware schema", "fail", "Critical radar columns are missing or the contract is invalid", "Use a session collected with the expected radar schema.")
    elif not schema_hash_ok:
        schema_verdict = "schema_warning"
        readiness_kind = "schema_warning"
        limitation_kind = "schema"
        next_action = "rerun analysis with the current trainer"
        add_cat("schema", "Feature schema", "warn", "Feature schema hash differs from the current trainer", "Rerun analysis with the current trainer or keep this lineage as a separate model branch.")

    if schema_verdict == "ok" and fw:
        if not module_valid and critical_cols_ok and schema_hash_ok:
            schema_verdict = "provenance_warning"
            readiness_kind = "provenance_warning"
            limitation_kind = "provenance"
            next_action = "inspect module identity lookup or treat this session as provenance-warned"
            add_cat("firmware", "Firmware provenance", "warn", "Module firmware identity is missing, but the sketch and critical columns are present", "Treat this as a provenance warning and continue analysis.")
        elif not module_valid:
            schema_verdict = "schema_warning"
            add_cat("firmware", "Firmware provenance", "warn", "Module firmware identity is missing", "Continue analysis, but keep the session separate from fully trusted firmware.")

    if "conditional_pass_n_too_small" in gate_status or ("conditional" in gate_status and "too_small" in gate_status):
        training_verdict = "conditional"
        readiness_kind = "conditional"
        limitation_kind = limitation_kind or "data"
        next_action = "collect a longer session before ML training"
        add_cat("data", "Training sample count", "warn", "Primary gate is conditionally passing, but n is too small for ML.", "Collect a longer session before training.")
    elif str(secondary_kind).startswith("deferred") or secondary_basis == "too_short_to_judge":
        training_verdict = "deferred"
        readiness_kind = "deferred"
        limitation_kind = limitation_kind or "duration"
        next_action = "run a longer session"
        add_cat("duration", "Session duration", "warn", "Secondary gate deferred the session because it is too short to judge", "Collect a longer capture before training.")
    elif bool(gate.get("passed")) and (not np.isfinite(coverage) or coverage >= ML_READY_MIN_BLE_COVERAGE_PCT):
        pass
    elif bool(gate.get("passed")):
        signal_verdict = "degraded_signal"
        training_verdict = "degraded_signal"

    if np.isfinite(pqi) and pqi < 50.0:
        signal_verdict = "degraded_signal"
        training_verdict = "degraded_signal"
        readiness_kind = "degraded_signal" if readiness_kind == "not_ready" else readiness_kind
        limitation_kind = limitation_kind or "hygiene"
        next_action = "still subject, closer range, check radar placement"
        add_cat("hygiene", "Radar signal hygiene", "fail", f"PQI lock {pqi:.1f}% is low", "Keep subject still, check radar placement and range.")

    if (np.isfinite(coverage) and coverage < ML_READY_MIN_BLE_COVERAGE_PCT) or (np.isfinite(pi) and pi < BLE_PI_QUALITY_THRESHOLD):
        reference_verdict = "degraded_signal"
        training_verdict = "degraded_signal" if training_verdict not in {"deferred", "firmware_rejected"} else training_verdict
        if readiness_kind == "not_ready":
            readiness_kind = "degraded_signal"
        limitation_kind = limitation_kind or "reference"
        next_action = "check oximeter placement / contact"
        add_cat("reference", "BLE reference quality", "fail", f"coverage={coverage:.1f}% pi={pi:.2f}", "Check oximeter placement, finger contact, and BLE connection.")

    if total_publish > 0 and suppress_frac >= 0.5:
        signal_verdict = "degraded_signal"
        training_verdict = "degraded_signal" if training_verdict not in {"deferred", "firmware_rejected"} else training_verdict
        if readiness_kind == "not_ready":
            readiness_kind = "degraded_signal"
        limitation_kind = limitation_kind or "policy"
        next_action = "extend session or adjust publish policy"
        add_cat("policy", "Publish policy", "fail", "Publish suppression dominates", "Extend session or review warmup/publish policy.")

    raw_bias_audit = analyse_summary.get("raw_hr_bias_audit")
    raw = raw_bias_audit if isinstance(raw_bias_audit, dict) else {}
    if _to_num(raw.get("rr_harmonic_k_frac"), 0.0) >= 0.5 or _to_num(raw.get("hr_raw_disagree_rr_harmonic_k"), 0.0) > 0:
        signal_verdict = "degraded_signal"
        training_verdict = "degraded_signal" if training_verdict not in {"deferred", "firmware_rejected"} else training_verdict
        if readiness_kind == "not_ready":
            readiness_kind = "degraded_signal"
        limitation_kind = limitation_kind or "physiology"
        next_action = "record with RR spec-friendly setup"
        add_cat("physiology", "Harmonic contamination", "fail", "RR harmonic contamination dominates", "Record with a more RR-friendly setup.")

    if readiness_kind not in {"conditional", "deferred"} and bool(gate.get("passed")) and (not np.isfinite(coverage) or coverage >= ML_READY_MIN_BLE_COVERAGE_PCT) and schema_verdict in {"ok", "provenance_warning"}:
        if np.isfinite(hr_cov) and hr_cov >= ML_READY_MIN_HR_COVERAGE_PCT and np.isfinite(hr_mae) and hr_mae < ML_READY_MIN_HR_MAE_BPM and np.isfinite(hr_bias) and abs(hr_bias) < ML_READY_MIN_HR_BIAS_BPM and np.isfinite(rr_cov) and rr_cov >= ML_READY_MIN_RR_COVERAGE_PCT and np.isfinite(rr_mae) and rr_mae < ML_READY_MIN_RR_MAE_BPM and schema_verdict != "firmware_rejected":
            training_verdict = "ready"
            limitation_kind = None
            next_action = "ship"
            add_cat("primary_gate", "Primary gate", "pass", "Primary HR gate passed", "")
            add_cat("quality", "Signal quality", "pass", f"HR cov={hr_cov:.1f}% HR MAE={hr_mae:.2f} bpm RR cov={rr_cov:.1f}% RR MAE={rr_mae:.2f} bpm", "")
            return {
                "verdict": "ready",
                "readiness_kind": readiness_kind,
                "headline": "Session is ready for ML training" if readiness_kind == "ready" else "Session is analyzable with provenance warning",
                "schema_verdict": schema_verdict,
                "signal_verdict": signal_verdict,
                "reference_verdict": reference_verdict,
                "training_verdict": training_verdict,
                "categories": categories,
                "passed": passed or ["Primary HR gate", "BLE coverage >= 70%", "Schema hash consistent"],
                "failed": failed,
                "limitation_kind": limitation_kind,
                "next_action": next_action,
            }

    if not np.isfinite(hr_cov) or hr_cov < ML_READY_MIN_HR_COVERAGE_PCT or not np.isfinite(hr_mae) or hr_mae >= ML_READY_MIN_HR_MAE_BPM or not np.isfinite(rr_cov) or rr_cov < ML_READY_MIN_RR_COVERAGE_PCT:
        signal_verdict = "degraded_signal"
        training_verdict = "degraded_signal" if training_verdict not in {"deferred", "firmware_rejected"} else training_verdict
        if readiness_kind == "not_ready":
            readiness_kind = "degraded_signal"
        limitation_kind = limitation_kind or "signal"
        next_action = "improve coverage or lower estimator bias"
        add_cat("signal", "Signal quality", "fail", f"HR cov={hr_cov:.1f}% HR MAE={hr_mae:.2f} bpm RR cov={rr_cov:.1f}%", "Improve DSP coverage and reduce bias before training.")

    if not passed and not categories:
        add_cat("unknown", "ML gate", "fail", "ML gate did not pass", "Inspect analysis and rerun if needed.")
        if training_verdict == "not_ready":
            training_verdict = "degraded_signal" if readiness_kind == "not_ready" else readiness_kind

    headline_map = {
        "ready": "Session is ready for ML training",
        "provenance_warning": "Session is analyzable with provenance warning",
        "schema_warning": "Session has a schema warning",
        "conditional": "Session is conditionally usable, but too small for ML training",
        "degraded_signal": "Session has degraded signal quality",
        "deferred": "Session is too short to judge for ML training",
        "firmware_rejected": "Firmware truthfulness mismatch",
        "not_ready": "Session is not ready for ML training",
    }
    headline = headline_map.get(readiness_kind, "Session is not ready for ML training")
    verdict = "ready" if readiness_kind == "ready" else ("conditional" if readiness_kind in {"conditional", "deferred"} else "not_ready")
    return {
        "verdict": verdict,
        "readiness_kind": readiness_kind,
        "headline": headline,
        "schema_verdict": schema_verdict,
        "signal_verdict": signal_verdict,
        "reference_verdict": reference_verdict,
        "training_verdict": training_verdict,
        "categories": categories,
        "passed": passed,
        "failed": failed,
        "limitation_kind": limitation_kind,
        "next_action": next_action,
    }


def _coverage_penalized_hr_gate(
    feat_df: pd.DataFrame,
    window_s: float = 600.0,
    min_duration_s: float = SECONDARY_GATE_MIN_DURATION_S,
    min_pairs: int = SECONDARY_GATE_MIN_PAIRS,
) -> Dict[str, object]:
    empty = {
        "passed": False,
        "status": "NO_VALID_BURST",
        "kind": "failed",
        "basis": "coverage_penalized_10min",
        "threshold_bpm": 8.0,
        "window_s": float(window_s),
        "coverage_penalized_rmse": float("nan"),
        "rmse": float("nan"),
        "r": float("nan"),
        "n": 0,
        "coverage_pct": float("nan"),
        "window_start_s": float("nan"),
        "window_end_s": float("nan"),
        "duration_s": float("nan"),
    }
    if not isinstance(feat_df, pd.DataFrame) or len(feat_df) == 0:
        empty["kind"] = "deferred"
        empty["status"] = "DEFERRED"
        empty["basis"] = "too_short_to_judge"
        return empty
    needed = {"timestamp_s", "ref_hr", "reported_hr_mean", "hr_valid_for_eval"}
    if not needed.issubset(feat_df.columns):
        empty["kind"] = "deferred"
        empty["status"] = "DEFERRED"
        empty["basis"] = "too_short_to_judge"
        return empty
    work = feat_df.sort_values("timestamp_s").reset_index(drop=True).copy()
    duration_s = _session_duration_s(work)
    empty["duration_s"] = duration_s
    ts = safe_series(work, "timestamp_s").to_numpy(dtype=float)
    finite_ts = np.isfinite(ts)
    if not finite_ts.any():
        empty["kind"] = "deferred"
        empty["status"] = "DEFERRED"
        empty["basis"] = "too_short_to_judge"
        return empty
    pair_mask = (
        finite_pair_mask(work, "ref_hr", "reported_hr_mean") &
        work["hr_valid_for_eval"].fillna(False).to_numpy(dtype=bool) &
        finite_ts
    )
    valid_pair_count = int(pair_mask.sum())
    empty["n"] = valid_pair_count
    if valid_pair_count == 0:
        empty["kind"] = "deferred"
        empty["status"] = "DEFERRED"
        empty["basis"] = "too_short_to_judge"
        empty["operator_message"] = "No valid HR pairs available for secondary gate evaluation."
        return empty
    if (np.isfinite(duration_s) and duration_s < min_duration_s) or valid_pair_count < min_pairs:
        empty["kind"] = "deferred"
        empty["status"] = "DEFERRED"
        empty["basis"] = "too_short_to_judge"
        empty["operator_message"] = (
            f"Session duration {duration_s:.1f}s or valid pair count {valid_pair_count} is below the secondary gate minimum."
        )
        return empty

    candidates = np.unique(ts[pair_mask])
    best: Dict[str, Any] = dict(empty)
    best_score = float("inf")
    for anchor in candidates:
        start = float(anchor)
        end = start + float(window_s)
        window_mask = finite_ts & (ts >= start) & (ts < end)
        if not window_mask.any():
            continue
        valid_mask = window_mask & pair_mask
        n = int(valid_mask.sum())
        if n < min_pairs:
            continue
        ref = safe_series(work, "ref_hr").to_numpy(dtype=float)[valid_mask]
        pred = safe_series(work, "reported_hr_mean").to_numpy(dtype=float)[valid_mask]
        metrics = compute_metrics(ref, pred, "hr")
        rmse = float(metrics.get("rmse", float("nan")))
        coverage_frac = float(valid_mask.sum() / max(1, int(window_mask.sum())))
        cov_pen = rmse / max(np.sqrt(coverage_frac), 0.1) if np.isfinite(rmse) else float("nan")
        if np.isfinite(cov_pen) and cov_pen < best_score:
            best_score = cov_pen
            best.update({
                "coverage_penalized_rmse": float(cov_pen),
                "rmse": rmse,
                "r": float(metrics.get("r", float("nan"))),
                "n": n,
                "coverage_pct": float(100.0 * coverage_frac),
                "window_start_s": start,
                "window_end_s": end,
            })
    passed = bool(np.isfinite(best.get("coverage_penalized_rmse", float("nan"))) and best["coverage_penalized_rmse"] < 8.0)
    best["passed"] = passed
    best["kind"] = "passed" if passed else "failed"
    best["status"] = "CONDITIONAL_PASS_N_TOO_SMALL_FOR_ML" if passed else "FAIL"
    best["basis"] = "coverage_penalized_10min"
    best["operator_message"] = (
        "Conditional pass, n too small for ML; short HR coverage burst met coverage-penalized RMSE."
        if passed else
        "Coverage-penalized HR burst did not meet the operator conditional-pass threshold."
    )
    return best
