"""Temporal feature engineering and ML feature-schema leakage firewall."""

from __future__ import annotations

import hashlib
import json
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
import pandas as pd

from rvt_trainer.constants import FEATURE_SCHEMA_VERSION, safe_series

TEMPORAL_BASE_COLS_CORE = [
    "reported_hr_mean", "reported_rr_mean", "raw_hr_mean", "raw_rr_mean", "raw_rr_effective_mean",
    "candidate_hr_mean", "candidate_rr_mean", "candidate_hr_conf_mean", "candidate_rr_conf_mean",
    "candidate_hr_minus_reported_hr_mean", "candidate_rr_minus_reported_hr_mean",
    "candidate_hr_minus_raw_hr_mean", "candidate_rr_minus_raw_rr_mean",
    "candidate_hr_minus_raw_hr_corrected_mean", "candidate_rr_minus_raw_rr_effective_mean",
    "pqi_heart_mean", "pqi_breath_mean", "hr_confidence_mean",
    "hr_valid_frac", "rr_valid_frac", "hr_triple_agree_mean", "rr_triple_agree_mean",
]

TEMPORAL_BASE_COLS_FULL = TEMPORAL_BASE_COLS_CORE + [
    "reported_distance_cm_mean", "reflector_distance_cm_mean", "dist_sd_cm_mean",
    "heart_phase_mean", "breath_phase_mean",
    "heart_phase_excursion", "breath_phase_excursion",
    "harmonic_mode_max", "session_phase_mean", "session_phase_last", "phase_valid_this_frame_mean",
    "current_clutter_alpha_mean", "current_clutter_alpha_last",
    "num_targets_mean", "max_dop_abs_mean", "max_dop_speed_cms_mean", "primary_dop_mean", "primary_dop_speed_cms_mean",
    "doppler_motion_mean", "cluster_anomaly_mean", "use_fast_path_mean",
]

BASE_FEATURES_CORE = [
    "heart_phase_dom_freq_hz", "heart_phase_band_contrast",
    "breath_phase_dom_freq_hz", "breath_phase_band_contrast",
    "reported_hr_mean", "reported_hr_std", "reported_hr_median",
    "reported_rr_mean", "reported_rr_std", "reported_rr_median",
    "candidate_hr_mean", "candidate_hr_std", "candidate_hr_median",
    "candidate_rr_mean", "candidate_rr_std", "candidate_rr_median",
    "candidate_hr_conf_mean", "candidate_hr_conf_std", "candidate_hr_conf_max",
    "candidate_rr_conf_mean", "candidate_rr_conf_std", "candidate_rr_conf_max",
    "candidate_hr_minus_reported_hr_mean", "candidate_hr_minus_reported_hr_std",
    "candidate_rr_minus_reported_rr_mean", "candidate_rr_minus_reported_rr_std",
    "candidate_hr_minus_raw_hr_mean", "candidate_hr_minus_raw_hr_std",
    "candidate_rr_minus_raw_rr_mean", "candidate_rr_minus_raw_rr_std",
    "candidate_hr_minus_raw_hr_corrected_mean", "candidate_hr_minus_raw_hr_corrected_std",
    "candidate_rr_minus_raw_rr_effective_mean", "candidate_rr_minus_raw_rr_effective_std",
    "raw_hr_mean", "raw_hr_std", "raw_rr_mean", "raw_rr_std", "raw_rr_effective_mean", "raw_rr_effective_std",
    "hr_zc_bpm_mean", "hr_zc_bpm_std", "hr_zc_conf_mean",
    "hr_spec_bpm_mean", "hr_spec_bpm_std", "hr_spec_mag_mean",
    "rr_zc_bpm_mean", "rr_zc_bpm_std", "rr_zc_conf_mean",
    "rr_spec_bpm_mean", "rr_spec_bpm_std", "rr_spec_conf_mean",
    "pqi_heart_mean", "pqi_heart_std", "pqi_heart_max",
    "pqi_breath_mean", "pqi_breath_std", "pqi_breath_max",
    "hr_confidence_mean", "hr_confidence_std", "hr_confidence_max",
    "reported_distance_cm_mean", "reflector_distance_cm_mean", "dist_sd_cm_mean",
    "hr_state_last", "ghost_suspect_mean", "human_detected_mean", "radar_is_present_mean",
    "hr_valid_frac", "rr_valid_frac", "hr_valid_count", "rr_valid_count",
    "samples_per_sec",
    "phase_fresh_mean", "trusted_vital_fresh_mean",
    "harmonic_mode_mean", "harmonic_mode_max", "session_phase_mean", "session_phase_last",
    "phase_valid_this_frame_mean", "current_clutter_alpha_mean", "current_clutter_alpha_last",
    "hr_triple_agree_mean", "rr_triple_agree_mean",
    "harm_rr_raw_harmonic", "harm_hr_arbiter_corrected", "harm_hr_half_rate", "harm_hr_ambiguous",
    "harm_rr_subharmonic", "harm_rr_harmonic",
    "phase_absent", "phase_warmup", "phase_settling", "phase_locked", "phase_post_motion", "phase_leaving",
    "hr_rr_ratio", "pqi_product", "raw_reported_hr_diff", "raw_reported_rr_diff",
    "hr_raw_candidate_delta", "spec_zc_delta", "rr_source_latched_ok_frac", "buffer_zero_injected_mean",
]

BASE_FEATURES_FULL = BASE_FEATURES_CORE + [
    "heart_phase_peak_power", "heart_phase_bandpower",
    "heart_phase_spectral_entropy", "heart_phase_peak_ratio",
    "breath_phase_peak_power", "breath_phase_bandpower",
    "breath_phase_spectral_entropy", "breath_phase_peak_ratio",
    "reported_hr_min", "reported_hr_max", "reported_rr_min", "reported_rr_max",
    "reported_distance_cm_std", "reflector_distance_cm_std", "dist_sd_cm_std",
    "heart_phase_mean", "heart_phase_std", "heart_phase_last", "heart_phase_excursion",
    "breath_phase_mean", "breath_phase_std", "breath_phase_last", "breath_phase_excursion",
    "present_votes_mean", "absent_votes_mean",
    "logged_hr_valid_mean", "logged_rr_valid_mean",
    "point_cloud_ok_mean", "target_info_ok_mean",
    "num_targets_mean", "num_targets_max", "num_targets_last",
    "spatial_source_last", "spatial_age_ms_mean", "spatial_age_ms_last", "position_radius_cm_mean", "position_radius_cm_std", "position_radius_cm_last",
    "max_dop_abs_mean", "max_dop_abs_max", "max_dop_abs_last",
    "max_dop_speed_cms_mean", "max_dop_speed_cms_max", "max_dop_speed_cms_last",
    "doppler_motion_mean", "cluster_anomaly_mean", "multi_target_mean",
    "primary_x_mean", "primary_x_last", "primary_y_mean", "primary_y_last",
    "primary_dop_mean", "primary_dop_last", "primary_dop_speed_cms_mean", "primary_dop_speed_cms_max", "primary_dop_speed_cms_last",
    "primary_cluster_last", "harmonic_mode_max", "harmonic_mode_last",
    "session_phase_mean", "session_phase_last", "phase_valid_this_frame_mean",
    "current_clutter_alpha_mean", "current_clutter_alpha_last", "use_fast_path_mean", "hr_path_source_last",
    "fw_major_last", "fw_sub_last", "fw_mod_last",
]

TEMPORAL_SUFFIXES_CORE = ["lag1", "lag2", "lag5", "d1",
                           "roll3_mean", "roll3_std", "roll5_mean", "roll5_std"]
TEMPORAL_SUFFIXES_FULL = TEMPORAL_SUFFIXES_CORE + ["roll10_mean", "roll10_std"]

TRAINING_EXCLUDE_COLS = {"rr_fundamental_recovery_triggered"}
LABEL_FEATURES = {
    "ref_hr", "ref_rr", "ref_spo2", "ref_pi", "ref_timestamp_s", "ref_dt_s",
}
POLICY_FEATURE_PREFIXES = (
    "reported_hr", "reported_rr", "logged_hr_valid", "logged_rr_valid",
    "allow_logged_hr_vitals", "allow_logged_rr_vitals",
    "trusted_hr_fresh", "trusted_rr_fresh",
    "hr_publish_reason", "rr_publish_reason",
    "hr_gate_reason", "rr_gate_reason",
    "hr_publish_block_stage", "rr_publish_block_stage",
    "hr_publish_source_class", "rr_publish_source_class",
    "rr_source_", "logged_rr_valid_current", "logged_rr_valid_latched",
    "hr_freeze_", "hr_raw_disagree_",
)
POLICY_FEATURE_EXACT = {
    "hr_valid_frac", "rr_valid_frac", "hr_valid_count", "rr_valid_count",
    "phase_backed_publish_ready_mean", "phase_backed_publish_ready_last", "phase_backed_publish_ready_max",
    "logged_hr_quality_gate_mean", "logged_hr_quality_gate_last",
    "logged_rr_quality_gate_mean", "logged_rr_quality_gate_last",
}


def _strip_missing_flag(col: str) -> str:
    text = str(col)
    return text[:-9] if text.endswith("__missing") else text


def _temporal_derivatives(cols: Iterable[str], suffixes: Iterable[str]) -> List[str]:
    out: List[str] = []
    for col in cols:
        for suffix in suffixes:
            out.append(f"{col}_{suffix}")
    return out


def _build_feature_category() -> Dict[str, str]:
    categories: Dict[str, str] = {}
    for col in LABEL_FEATURES:
        categories[col] = "label"
    physio = set(BASE_FEATURES_CORE) | set(BASE_FEATURES_FULL)
    physio |= set(TEMPORAL_BASE_COLS_CORE) | set(TEMPORAL_BASE_COLS_FULL)
    physio |= set(_temporal_derivatives(TEMPORAL_BASE_COLS_CORE, TEMPORAL_SUFFIXES_CORE))
    physio |= set(_temporal_derivatives(TEMPORAL_BASE_COLS_FULL, TEMPORAL_SUFFIXES_FULL))
    for col in sorted(physio):
        categories.setdefault(col, "physio")
    for col in POLICY_FEATURE_EXACT:
        categories[col] = "policy"
    for prefix in POLICY_FEATURE_PREFIXES:
        categories[prefix] = "policy"
        for suffix in ("mean", "std", "min", "max", "median", "last", "sum"):
            categories[f"{prefix}_{suffix}"] = "policy"
    return categories


FEATURE_CATEGORY: Dict[str, str] = _build_feature_category()


def feature_schema_hash() -> str:
    payload = {
        "schema_version": FEATURE_SCHEMA_VERSION,
        "feature_category": FEATURE_CATEGORY,
        "policy_prefixes": POLICY_FEATURE_PREFIXES,
        "policy_exact": sorted(POLICY_FEATURE_EXACT),
        "training_exclude": sorted(TRAINING_EXCLUDE_COLS),
    }
    blob = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


def feature_category(col: str) -> str:
    base = _strip_missing_flag(col)
    if base in FEATURE_CATEGORY:
        return FEATURE_CATEGORY[base]
    if base in LABEL_FEATURES or base.startswith("ref_"):
        return "label"
    if base in POLICY_FEATURE_EXACT:
        return "policy"
    if any(base == pref or base.startswith(pref + "_") for pref in POLICY_FEATURE_PREFIXES):
        return "policy"
    return "physio"


def policy_feature_columns(cols: Sequence[str]) -> List[str]:
    return [str(c) for c in cols if feature_category(str(c)) == "policy"]


def label_feature_columns(cols: Sequence[str]) -> List[str]:
    return [str(c) for c in cols if feature_category(str(c)) == "label"]


def assert_ml_feature_schema(cols: Sequence[str], *, allow_policy_features: bool = False) -> None:
    labels = label_feature_columns(cols)
    if labels:
        raise ValueError(
            "Label columns are not allowed in the ML feature matrix: "
            + ", ".join(labels[:20])
        )
    policies = policy_feature_columns(cols)
    if policies and not allow_policy_features:
        raise ValueError(
            "Policy/gate columns are blocked by the leakage firewall. "
            "Pass --allow-policy-features only for an explicit ablation. "
            "Blocked: " + ", ".join(policies[:30])
        )


def engineer_temporal_features(df: pd.DataFrame, feature_mode: str = "full") -> pd.DataFrame:
    df = df.copy()
    lag_cols = TEMPORAL_BASE_COLS_FULL if feature_mode == "full" else TEMPORAL_BASE_COLS_CORE

    def add_group_features(g: pd.DataFrame) -> pd.DataFrame:
        g = g.sort_values("timestamp_s").copy()
        rr = safe_series(g, "reported_rr_mean")
        rr_safe = rr.where(rr > 0, np.nan)
        g["hr_rr_ratio"]         = safe_series(g, "reported_hr_mean") / rr_safe
        g["pqi_product"]         = safe_series(g, "pqi_heart_mean") * safe_series(g, "pqi_breath_mean")
        g["raw_reported_hr_diff"] = safe_series(g, "raw_hr_mean") - safe_series(g, "reported_hr_mean")
        g["raw_reported_rr_diff"] = safe_series(g, "raw_rr_mean") - safe_series(g, "reported_rr_mean")
        g["hr_raw_candidate_delta"] = (safe_series(g, "raw_hr_mean") - safe_series(g, "candidate_hr_mean")).abs()
        g["spec_zc_delta"] = (safe_series(g, "hr_spec_bpm_mean") - safe_series(g, "hr_zc_bpm_mean")).abs()
        g["candidate_hr_minus_reported_hr"] = safe_series(g, "candidate_hr_mean") - safe_series(g, "reported_hr_mean")
        g["candidate_rr_minus_reported_rr"] = safe_series(g, "candidate_rr_mean") - safe_series(g, "reported_rr_mean")
        g["candidate_hr_minus_raw_hr"] = safe_series(g, "candidate_hr_mean") - safe_series(g, "raw_hr_mean")
        g["candidate_rr_minus_raw_rr"] = safe_series(g, "candidate_rr_mean") - safe_series(g, "raw_rr_mean")
        g["candidate_hr_minus_raw_hr_corrected"] = safe_series(g, "candidate_hr_mean") - safe_series(g, "raw_hr_corrected_mean")
        g["candidate_rr_minus_raw_rr_effective"] = safe_series(g, "candidate_rr_mean") - safe_series(g, "raw_rr_effective_mean")
        g["candidate_hr_minus_reported_hr_mean"] = g["candidate_hr_minus_reported_hr"]
        g["candidate_rr_minus_reported_rr_mean"] = g["candidate_rr_minus_reported_rr"]
        g["candidate_hr_minus_raw_hr_mean"] = g["candidate_hr_minus_raw_hr"]
        g["candidate_rr_minus_raw_rr_mean"] = g["candidate_rr_minus_raw_rr"]
        g["candidate_hr_minus_raw_hr_corrected_mean"] = g["candidate_hr_minus_raw_hr_corrected"]
        g["candidate_rr_minus_raw_rr_effective_mean"] = g["candidate_rr_minus_raw_rr_effective"]
        for delta_col in (
            "candidate_hr_minus_reported_hr",
            "candidate_rr_minus_reported_rr",
            "candidate_hr_minus_raw_hr",
            "candidate_rr_minus_raw_rr",
            "candidate_hr_minus_raw_hr_corrected",
            "candidate_rr_minus_raw_rr_effective",
        ):
            g[f"{delta_col}_std"] = 0.0
        g["rr_source_latched_ok_frac"] = safe_series(g, "rr_source_latched_ok_mean", default=0.0)
        g["buffer_zero_injected_mean"] = safe_series(g, "buffer_zero_injected_mean", default=0.0)
        harmonic = safe_series(g, "harmonic_mode_max", default=0.0).fillna(0.0).round().astype(int)
        g["harm_rr_raw_harmonic"] = ((harmonic & 0x01) > 0).astype(float)
        g["harm_hr_arbiter_corrected"] = ((harmonic & 0x02) > 0).astype(float)
        g["harm_hr_half_rate"] = ((harmonic & 0x04) > 0).astype(float)
        g["harm_hr_ambiguous"] = ((harmonic & 0x08) > 0).astype(float)
        g["harm_rr_subharmonic"] = ((harmonic & 0x10) > 0).astype(float)
        g["harm_rr_harmonic"] = ((harmonic & 0x20) > 0).astype(float)
        phase = safe_series(g, "session_phase_last", default=0.0).fillna(0.0).round().astype(int)
        g["phase_absent"] = (phase == 0).astype(float)
        g["phase_warmup"] = (phase == 1).astype(float)
        g["phase_settling"] = (phase == 2).astype(float)
        g["phase_locked"] = (phase == 3).astype(float)
        g["phase_post_motion"] = (phase == 4).astype(float)
        g["phase_leaving"] = (phase == 5).astype(float)
        for col in [c for c in lag_cols if c in g.columns]:
            s = safe_series(g, col)
            for lag in (1, 2, 5):
                g[f"{col}_lag{lag}"]     = s.shift(lag)
            g[f"{col}_d1"]           = s.diff()
            g[f"{col}_roll3_mean"]   = s.rolling(window=3,  min_periods=1).mean()
            g[f"{col}_roll3_std"]    = s.rolling(window=3,  min_periods=2).std(ddof=0)
            g[f"{col}_roll5_mean"]   = s.rolling(window=5,  min_periods=1).mean()
            g[f"{col}_roll5_std"]    = s.rolling(window=5,  min_periods=2).std(ddof=0)
            if feature_mode == "full":
                g[f"{col}_roll10_mean"] = s.rolling(window=10, min_periods=1).mean()
                g[f"{col}_roll10_std"]  = s.rolling(window=10, min_periods=2).std(ddof=0)
        return g

    groups = [add_group_features(g) for _, g in df.groupby("session_id", sort=False)]
    return pd.concat(groups, ignore_index=True)


def pick_feature_columns(
    df: pd.DataFrame, feature_mode: str,
    max_nan_frac: float, min_variance: float,
    allow_policy_features: bool = False,
) -> List[str]:
    base     = BASE_FEATURES_FULL if feature_mode == "full" else BASE_FEATURES_CORE
    suffixes = TEMPORAL_SUFFIXES_FULL if feature_mode == "full" else TEMPORAL_SUFFIXES_CORE
    temporal = [c for c in df.columns if any(c.endswith("_" + s) for s in suffixes)]
    cols = [c for c in base if c in df.columns] + sorted(temporal)
    cols = [c for c in cols if c not in TRAINING_EXCLUDE_COLS]
    if not allow_policy_features:
        cols = [c for c in cols if feature_category(c) != "policy"]
    cols = [c for c in cols if c in df.columns and pd.api.types.is_numeric_dtype(df[c])]
    kept = []
    for c in cols:
        s = pd.to_numeric(df[c], errors="coerce")
        if s.isna().all():
            continue
        if float(s.isna().mean()) > max_nan_frac:
            continue
        var = float(s.var(skipna=True))
        if (not np.isfinite(var)) or var <= min_variance:
            continue
        kept.append(c)
    return kept


def prepare_feature_matrix(
    train_df: pd.DataFrame, apply_df: pd.DataFrame, feature_cols: Sequence[str],
) -> Tuple[pd.DataFrame, pd.DataFrame, Dict[str, float], List[str]]:
    X_train = train_df.reindex(columns=feature_cols).apply(pd.to_numeric, errors="coerce")
    X_apply = apply_df.reindex(columns=feature_cols).apply(pd.to_numeric, errors="coerce")
    mask_train = X_train.isna().copy()
    mask_apply = X_apply.isna().copy()
    impute_values = X_train.median(axis=0, skipna=True).fillna(0.0).to_dict()
    missing_cols  = [c for c in feature_cols if mask_train[c].any() or mask_apply[c].any()]
    for c in feature_cols:
        X_train[c] = X_train[c].fillna(impute_values[c])
        X_apply[c] = X_apply[c].fillna(impute_values[c])
    for c in missing_cols:
        X_train[f"{c}__missing"] = mask_train[c].astype(float)
        X_apply[f"{c}__missing"] = mask_apply[c].astype(float)
    return X_train.astype(np.float32), X_apply.astype(np.float32), impute_values, missing_cols


def transform_feature_matrix(
    apply_df: pd.DataFrame, feature_cols: Sequence[str],
    impute_values: Dict[str, float], missing_flag_cols: Sequence[str],
) -> pd.DataFrame:
    X_apply   = apply_df.reindex(columns=feature_cols).apply(pd.to_numeric, errors="coerce")
    mask_apply = X_apply.isna().copy()
    for c in feature_cols:
        X_apply[c] = X_apply[c].fillna(impute_values.get(c, 0.0))
    for c in missing_flag_cols:
        X_apply[f"{c}__missing"] = mask_apply[c].astype(float) if c in mask_apply.columns else 0.0
    return X_apply.astype(np.float32)
