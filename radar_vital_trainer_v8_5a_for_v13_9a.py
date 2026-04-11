"""radar_vital_trainer_v8_4_1_for_v13_8_0.py — v8.4.3
=====================================================
Leakage-aware offline trainer + full session pipeline for the
XIAO ESP32-C6 + MR60BHA2 system.

v8.4.0 keeps the full session pipeline and adds the v13.8.0 telemetry-contract release: sketch/module firmware identity, stabilized phase labels, DSP freshness flags, and confidence-source truthfulness for thesis-ready analysis.
Material 3 Expressive live dashboard stack for thesis use:
- session orchestration remains the primary collection workflow
- live dashboarding, regression QA, and stronger analysis outputs remain intact
- stale v6 command references are removed from help text and quickstart
- version strings and guidance are aligned with the current file

Change highlights
────────────────
  • v13.7.1 telemetry contract: raw_rr_effective/raw_rr_likely_harmonic, session_phase, harmonic_mode, and HR/RR zero-crossing + spectral agreement fields
  • fixes the silent v8.1.8a header-shift bug by aligning RADAR_LOG_COLUMNS with the widened firmware CSV contract
  • write_text_report now forces UTF-8 so Windows auto-analyse runs do not crash on Unicode arrows in the text report
  • training keeps explicit pre-Kalman candidate HR/RR + confidence features in the model input set
  • dashboard loader prefers an external v8.2 HTML template when present

Primary workflow
────────────────
  1.  python radar_vital_trainer_v8_4_3_for_v13_8_0.py doctor
  2.  python radar_vital_trainer_v8_4_3_for_v13_8_0.py quickstart

  Per session (single command):
      python radar_vital_trainer_v8_4_3_for_v13_8_0.py session \
        --port COM10 \
        --address 10:22:33:9E:8F:63 \
        --duration-s 480 \
        --open-dashboard

  After session:
      python radar_vital_trainer_v8_4_3_for_v13_8_0.py compare --sessions-dir sessions/ --out report.html

  Optional manual workflow:
      log / ble_reflog / align / analyse / train / sweep
"""

import argparse
import importlib.util
import json
import os
import pickle
import re
import shutil
import subprocess
import sys
import tempfile
import time
import threading
import csv
from collections import deque
from functools import partial
from html import escape as html_escape
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.metrics import mean_absolute_error, mean_squared_error

import warnings
warnings.filterwarnings("ignore")


VERSION = "8.5a"
DEFAULT_RADAR_PORT = "COM10"
DEFAULT_BLE_ADDRESS = "10:22:33:9E:8F:63"
HR_RANGE = (30.0, 220.0)
RR_RANGE = (4.0, 60.0)

HR_GATE_REASON_NAMES = {0:"OK",1:"NO_AUTO",2:"PQI",3:"HARMONIC",4:"RAW_DISAGREE",5:"SPEC_REJECT",6:"MOTION",7:"DISPLAY_ONLY",8:"RAW_FALLBACK",9:"UPWARD_CONFIRM"}
RR_GATE_REASON_NAMES = {0:"OK",1:"NO_AUTO",2:"PQI",3:"CONF",4:"RAW_DISAGREE",5:"SUBHARMONIC",6:"HARMONIC",7:"MOTION",8:"HOLDOFF",9:"DISPLAY_ONLY"}
HR_PUBLISH_REASON_NAMES = {0:"OK",1:"NO_HUMAN",2:"TRUST_STALE",3:"NO_EVIDENCE",4:"STATE",5:"AGE",6:"STALE",7:"PQI",8:"CONF",9:"HARMONIC",10:"RAW_DISAGREE",11:"OTHER",12:"GRACE_BLOCKED"}
RR_PUBLISH_REASON_NAMES = {0:"OK",1:"NO_HUMAN",2:"TRUST_STALE",3:"NO_EVIDENCE",4:"STALE",5:"PQI",6:"HOLDOFF",7:"SOURCE",8:"OTHER"}

SESSION_PHASE_NAMES = {0:"ABSENT",1:"WARMUP",2:"SETTLING",3:"LOCKED",4:"POST_MOTION",5:"LEAVING"}
HR_ANCHOR_SOURCE_NAMES = {
    0: "none",
    1: "trusted_phase",
    2: "tracking_raw",
    3: "latched_raw",
    4: "arbiter_anchor",
    5: "rejectphase_anchor",
}

HR_CONFIDENCE_SOURCE_NAMES = {
    0: "none",
    1: "auto_phase",
    2: "auto_phase_raw_blend",
    3: "spectral_only",
    4: "spectral_raw_blend",
    5: "raw_fallback",
    6: "raw_only",
    7: "direct_raw_fast",
}

RADAR_LOG_COLUMNS = [
    "timestamp_ms",
    "heart_phase_stabilized",
    "breath_phase_stabilized",
    "raw_hr",
    "raw_rr",
    "raw_rr_effective",
    "raw_rr_likely_harmonic",
    "reported_hr",
    "reported_rr",
    "candidate_hr",
    "candidate_rr",
    "candidate_hr_conf",
    "candidate_rr_conf",
    "pqi_heart",
    "pqi_breath",
    "hr_confidence",
    "hr_gate_reason",
    "rr_gate_reason",
    "hr_gate_pqi_used",
    "rr_gate_pqi_used",
    "hr_zc_bpm",
    "hr_zc_conf",
    "hr_spec_bpm",
    "hr_spec_mag",
    "hr_triple_agree",
    "rr_zc_bpm",
    "rr_zc_conf",
    "rr_spec_bpm",
    "rr_spec_conf",
    "rr_triple_agree",
    "reflector_distance_cm",
    "reported_distance_cm",
    "in_motion",
    "human_detected",
    "radar_is_present",
    "hr_state",
    "ghost_suspect",
    "dist_sd_cm",
    "disp_state",
    "dsp_task",
    "present_votes",
    "absent_votes",
    "phase_fresh",
    "trusted_vital_fresh",
    "logged_hr_valid",
    "logged_rr_valid",
    "hr_publish_reason",
    "rr_publish_reason",
    "skipdsp_misses",
    "hr_band_min",
    "hr_band_max",
    "radar_gain",
    "hr_arbiter_corrected",
    "hr_rejectphase_rejected",
    "hr_coherence_rejected",
    "hr_raw_source",
    "hr_raw_agree",
    "hr_agree_err_bpm",
    "hr_bypass_pqi_ok",
    "hr_bypass_conf_ok",
    "hr_bypass_gate_ok",
    "hr_bypass_active",
    "hr_grace_eligible",
    "hr_grace_active",
    "hr_trust_fresh",
    "rr_anchor_fresh",
    "rr_outlier_persist",
    "rr_raw_agree_ok",
    "rr_pre_acceptphase",
    "rr_post_acceptphase",
    "rr_post_blend",
    "rr_post_bias_correction",
    "rr_post_kalman",
    "rr_final_publish_candidate",
    "rr_anchor_value",
    "rr_anchor_age_ms",
    "rr_anchor_source",
    "rr_anchor_confidence",
    "rr_fundamental_recovery_count",
    "rr_fundamental_recovery_triggered",
    "rr_raw_seed_consistent_count",
    "rr_midsession_raw_reanchor_allowed",
    "rr_midsession_raw_reanchor_blocked",
    "rr_midsession_raw_reanchor_reason",
    "rr_raw_anchor_err_bpm",
    "hr_pre_rejectphase",
    "hr_post_rejectphase",
    "hr_post_blend",
    "hr_post_coherence",
    "hr_final_publish_candidate",
    "hr_arbiter_anchor_used",
    "hr_arbiter_anchor_value",
    "hr_rejectphase_anchor_used",
    "hr_rejectphase_anchor_value",
    "hr_raw_looks_like_half_rate",
    "hr_trusted_anchor_value",
    "hr_age_ms",
    "candidate_hr_age_ms",
    "candidate_rr_age_ms",
    "hr_updated_this_cycle",
    "hr_update_source",
    "latched_raw_hr",
    "trusted_rr_fresh",
    "radar_is_present_raw",
    "harmonic_mode",
    "session_phase",
    "point_cloud_ok",
    "target_info_ok",
    "num_targets",
    "max_dop_abs",
    "max_dop_speed_cms",
    "doppler_motion",
    "cluster_anomaly",
    "multi_target",
    "primary_x",
    "primary_y",
    "primary_dop",
    "primary_dop_speed_cms",
    "primary_cluster",
    "spatial_source",
    "spatial_age_ms",
    "position_radius_cm",
    "phase_warmup_complete",
    "clutter_warmup_count",
    "current_clutter_alpha",
    "use_fast_path",
    "phase_valid_this_frame",
    "dsp_ran_this_frame",
    "hr_confidence_source",
    "hr_path_source",
    "module_fw_major",
    "module_fw_sub",
    "module_fw_mod",
    "sketch_major",
    "sketch_sub",
    "sketch_mod",
    "hr_trusted_phase_anchor",
    "hr_anchor_source",
    "hr_anchor_err_bpm",
    "hr_raw_high_bias_suspect",
    "rr_seed_from_raw_used",
    "trusted_hr_fresh",
    "allow_logged_hr_vitals",
    "allow_logged_rr_vitals",
    "logged_hr_quality_gate",
    "logged_rr_quality_gate",
    "phase_valid_run_len",
    "phase_invalid_run_len",
    "hr_phase_backed_update_count",
    "rr_phase_backed_update_count",
    "near_field_reflector_suspect",
    "agc_floor_suspect",
    "phase_backed_publish_ready",
    "hr_anchor_drift_suspect",
    "phase_gap_fill_count",
    "clutter_rewarm_count",
    "experimental_profile_enabled",
]

HR_PATH_SOURCE_NAMES = {
    0: "None",
    1: "Auto publish",
    2: "Spectral path",
    3: "Raw blend path",
    4: "Raw fallback",
    5: "Raw only (no phase)",
    6: "Direct raw fast path",
}

HARMONIC_MODE_BIT_LABELS = {
    0x01: "RR raw harmonic corrected",
    0x02: "HR arbiter corrected",
    0x04: "HR raw looked half-rate",
    0x08: "HR harmonic ambiguous",
    0x10: "RR subharmonic rejected",
    0x20: "RR harmonic rejected",
}

HEART_PQI_GATE_NEAR = 0.15
HEART_PQI_GATE_MID = 0.20
HEART_PQI_GATE_FAR = 0.35
EXPECTED_RADAR_LOG_COLUMN_COUNT = 157
EXPECTED_RADAR_LOG_TAIL = (
    "phase_backed_publish_ready",
    "hr_anchor_drift_suspect",
    "phase_gap_fill_count",
    "clutter_rewarm_count",
    "experimental_profile_enabled",
)
LEGACY_RADAR_LOG_COLUMN_COUNT = 131
LEGACY_RADAR_LOG_TAIL = ("hr_path_source", "fw_major", "fw_sub", "fw_mod")
_FIRMWARE_CONTRACT_CACHE: Optional[Tuple[str, Tuple[str, ...]]] = None


# ─────────────────────────────────────────────────────────────────────────────
# Terminal colour helpers (auto-disabled when not a TTY)
# ─────────────────────────────────────────────────────────────────────────────

_USE_COLOR = sys.stdout.isatty()

def _c(code: str, text: str) -> str:
    if not _USE_COLOR:
        return text
    return f"\033[{code}m{text}\033[0m"

def _green(t):  return _c("32", t)
def _red(t):    return _c("31", t)
def _yellow(t): return _c("33", t)
def _bold(t):   return _c("1",  t)
def _dim(t):    return _c("2",  t)
def _cyan(t):   return _c("36", t)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers (unchanged from v5.0)
# ─────────────────────────────────────────────────────────────────────────────

def safe_series(df: pd.DataFrame, col: str, default=np.nan) -> pd.Series:
    if col in df.columns:
        s = pd.to_numeric(df[col], errors="coerce").astype(float)
        if pd.isna(default):
            return s
        return s.fillna(float(default))
    fill = np.nan if pd.isna(default) else float(default)
    return pd.Series(fill, index=df.index, dtype=float)


def tensorflow_is_available() -> bool:
    return importlib.util.find_spec("tensorflow") is not None


def bytes_to_c_array(blob: bytes, var_name: str) -> str:
    lines = [f"const unsigned char {var_name}[] = {{"]
    for i in range(0, len(blob), 12):
        chunk = ", ".join(f"0x{b:02x}" for b in blob[i:i+12])
        lines.append(f"  {chunk},")
    lines.append("};")
    lines.append(f"const unsigned int {var_name}_len = {len(blob)};")
    return "\n".join(lines) + "\n"


def nan_safe(obj):
    if isinstance(obj, dict):
        return {k: nan_safe(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [nan_safe(v) for v in obj]
    if isinstance(obj, tuple):
        return [nan_safe(v) for v in obj]
    if isinstance(obj, (np.floating, float)):
        return None if np.isnan(obj) else float(obj)
    if isinstance(obj, (np.integer, int)):
        return int(obj)
    return obj


def save_json(obj: Dict, path: str):
    abs_path = os.path.abspath(path)
    out_dir = os.path.dirname(abs_path) or os.getcwd()
    os.makedirs(out_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".codex-json-", suffix=".tmp", dir=out_dir)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(nan_safe(obj), f, indent=2, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, abs_path)
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


def write_text_report(path: str, info: Dict):
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"RADAR VITAL TRAINER REPORT v{VERSION}\n")
        f.write("=" * 72 + "\n")
        f.write(f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        for section, payload in info.items():
            f.write(f"[{section}]\n")
            if isinstance(payload, dict):
                f.write(json.dumps(nan_safe(payload), indent=2, allow_nan=False))
            else:
                f.write(str(payload))
            f.write("\n\n")


def warn(msg: str):
    print(f"[WARN] {msg}")


def _stream_subprocess_output(pipe, prefix: str, event_cb=None, show_output: bool = True):
    try:
        for line in iter(pipe.readline, ''):
            line = line.rstrip("\r\n")
            if not line:
                continue
            if event_cb is not None:
                try:
                    event_cb(prefix, line)
                except Exception:
                    pass
            if show_output:
                print(f"{prefix} {line}")
    finally:
        try:
            pipe.close()
        except Exception:
            pass


def _terminate_subprocess(proc: subprocess.Popen, label: str, timeout_s: float = 5.0):
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=timeout_s)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def _read_json_if_exists(path: str) -> Optional[Dict]:
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def _dashboard_analysis_payload(session_dir: str) -> Optional[Dict[str, object]]:
    analyse_dir = os.path.join(session_dir, "analysis")
    analyse_summary = _read_json_if_exists(os.path.join(analyse_dir, "analyse_summary.json"))
    if not isinstance(analyse_summary, dict):
        return None
    hr = analyse_summary.get("hr_baseline", {}) if isinstance(analyse_summary.get("hr_baseline"), dict) else {}
    rr = analyse_summary.get("rr_baseline", {}) if isinstance(analyse_summary.get("rr_baseline"), dict) else {}
    hr_vo = hr.get("valid_only", {}) if isinstance(hr.get("valid_only"), dict) else {}
    rr_vo = rr.get("valid_only", {}) if isinstance(rr.get("valid_only"), dict) else {}
    return {
        "analysis_dir": os.path.abspath(analyse_dir),
        "session_quality_score": analyse_summary.get("session_quality_score"),
        "pqi_lock_pct": analyse_summary.get("pqi_lock_pct"),
        "ml_gate": analyse_summary.get("ml_gate", {}),
        "alignment_dt_s": analyse_summary.get("alignment_dt_s", {}),
        "hr_valid_only": hr_vo,
        "rr_valid_only": rr_vo,
        "hr_coverage_pct": hr.get("coverage_pct"),
        "rr_coverage_pct": rr.get("coverage_pct"),
        "ml_gate_combined": analyse_summary.get("ml_gate_combined", analyse_summary.get("ml_gate")),
        "ml_gate_locked": analyse_summary.get("ml_gate_locked"),
        "ml_gate_settling": analyse_summary.get("ml_gate_settling"),
        "coverage_locked": analyse_summary.get("coverage_locked"),
        "coverage_settling": analyse_summary.get("coverage_settling"),
        "gate_audit": analyse_summary.get("gate_audit"),
        "publish_reason_histogram": analyse_summary.get("publish_reason_histogram"),
        "hr_gate_reason_histogram": analyse_summary.get("hr_gate_reason_histogram"),
        "rr_gate_reason_histogram": analyse_summary.get("rr_gate_reason_histogram"),
        "agc_anomaly_flags": analyse_summary.get("agc_anomaly_flags"),
        "fw_truthfulness": analyse_summary.get("fw_truthfulness"),
        "golden_check": analyse_summary.get("golden_check"),
        "report_html": os.path.abspath(os.path.join(analyse_dir, "analyse_report.html")),
        "report_txt": os.path.abspath(os.path.join(analyse_dir, "analyse_report.txt")),
    }


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


def _safe_int(value, default: int = 0) -> int:
    try:
        n = int(round(float(value)))
        return n
    except Exception:
        return int(default)


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
    if s == 9 and 1 <= x <= 26:
        return f"v{m}.{s}{chr(ord('a') + x - 1)}"
    return f"v{m}.{s}.{x}"


def _truthfulness_from_radar(raw_df: pd.DataFrame) -> Dict[str, object]:
    out = {
        "version": "unknown",
        "module_version": None,
        "module_version_valid": False,
        "notes": "Derived from sketch/module telemetry in radar.csv",
    }
    if not isinstance(raw_df, pd.DataFrame) or len(raw_df) == 0:
        return out
    sketch_major = safe_series(raw_df, "sketch_major", default=np.nan).dropna()
    sketch_sub = safe_series(raw_df, "sketch_sub", default=np.nan).dropna()
    sketch_mod = safe_series(raw_df, "sketch_mod", default=np.nan).dropna()
    if len(sketch_major) and len(sketch_sub) and len(sketch_mod):
        out["version"] = _format_sketch_version(sketch_major.iloc[-1], sketch_sub.iloc[-1], sketch_mod.iloc[-1]) or "unknown"
    mod_major = safe_series(raw_df, "module_fw_major", default=np.nan).dropna()
    mod_sub = safe_series(raw_df, "module_fw_sub", default=np.nan).dropna()
    mod_mod = safe_series(raw_df, "module_fw_mod", default=np.nan).dropna()
    if len(mod_major) and len(mod_sub) and len(mod_mod):
        module_version = _format_sketch_version(mod_major.iloc[-1], mod_sub.iloc[-1], mod_mod.iloc[-1])
        out["module_version"] = module_version
        out["module_version_valid"] = module_version is not None
    return out


def _histogram_from_series(series: pd.Series, names: Dict[int, str]) -> Dict[str, int]:
    if not isinstance(series, pd.Series) or len(series) == 0:
        return {}
    vals = pd.to_numeric(series, errors="coerce").dropna().round().astype(int)
    if len(vals) == 0:
        return {}
    counts = vals.value_counts().sort_index()
    out: Dict[str, int] = {}
    for key, count in counts.items():
        label = names.get(int(key), str(int(key)))
        out[str(label)] = int(count)
    return out


def _top_histogram_items(hist: Optional[Dict[str, int]], limit: int = 3) -> List[str]:
    if not isinstance(hist, dict):
        return []
    items = sorted(hist.items(), key=lambda kv: (-int(kv[1]), kv[0]))
    return [f"{k}={v}" for k, v in items[:limit]]


def _combine_radar_frames(paths: Sequence[str]) -> pd.DataFrame:
    frames = []
    for idx, path in enumerate(paths):
        df = load_radar(path)
        if "session_id" not in df.columns:
            df["session_id"] = f"session_{idx+1}"
        frames.append(df)
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


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
    flags["gain_floor_pct"] = float(gain_floor.mean() * 100.0)
    flags["near_field_pct"] = float(near_field.mean() * 100.0)
    flags["skipdsp_pct"] = float(skipdsp.mean() * 100.0)
    flags["gain_floor_suspected"] = bool(flags["gain_floor_pct"] >= 10.0)
    flags["near_field_suspected"] = bool(flags["near_field_pct"] >= 10.0)
    flags["skipdsp_dead_zone_suspected"] = bool(flags["skipdsp_pct"] >= 10.0)
    return flags



def _clip01(x: float) -> float:
    return max(0.0, min(1.0, float(x)))


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
        if np.isfinite(hr_rmse): score += 18.0 * _clip01((20.0 - hr_rmse) / 20.0)
        if np.isfinite(rr_rmse): score += 18.0 * _clip01((10.0 - rr_rmse) / 10.0)
        if np.isfinite(hr_r): score += 14.0 * _clip01((hr_r + 0.2) / 0.8)
        if np.isfinite(rr_r): score += 14.0 * _clip01((rr_r + 0.2) / 0.8)
        if np.isfinite(hr_bias): score += 10.0 * _clip01((10.0 - hr_bias) / 10.0)
        if np.isfinite(rr_bias): score += 10.0 * _clip01((5.0 - rr_bias) / 5.0)
        if np.isfinite(hr_cov): score += 8.0 * _clip01(hr_cov / 40.0)
        if np.isfinite(rr_cov): score += 8.0 * _clip01(rr_cov / 60.0)
        raw_hr_bias_est = float(((summary.get("raw_hr_bias_estimate") or {}).get("bias_bpm", float("nan"))))
        if np.isfinite(raw_hr_bias_est):
            # v8.3: Temporarily neutralised penalty until v13.7 data validates the new arithmetic
            score -= 0.0 * _clip01((abs(raw_hr_bias_est) - 8.0) / 12.0)
        if bool((summary.get("ml_gate") or {}).get("passed")): score += 10.0
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


def _plot_phase_signal(df: pd.DataFrame, col: str, title: str, out_path: str, enabled: bool = True):
    if not enabled or col not in df.columns:
        return
    y = safe_series(df, col)
    if not y.notna().any():
        return
    x = safe_series(df, "timestamp_s") if "timestamp_s" in df.columns else pd.Series(np.arange(len(df)), index=df.index, dtype=float)
    plt.figure(figsize=(10, 3.6))
    plt.plot(x, y, linewidth=1.0)
    plt.xlabel("Time (s)")
    plt.ylabel(col)
    plt.title(title)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def save_feature_importance_plot(model, feature_names, out_path, top_n: int = 20):
    if not hasattr(model, "feature_importances_"):
        return
    imp = pd.DataFrame({"feature": list(feature_names), "importance": model.feature_importances_})
    imp = imp.sort_values("importance", ascending=False).head(int(top_n))
    if imp.empty:
        return
    plt.figure(figsize=(10, max(4, 0.28 * len(imp))))
    plt.barh(imp["feature"].iloc[::-1], imp["importance"].iloc[::-1])
    plt.xlabel("Importance")
    plt.title(f"Top {len(imp)} Feature Importances")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def _write_simple_html_report(title: str, sections: List[Tuple[str, str]], image_paths: List[Tuple[str, str]], out_path: str):
    parts = [
        "<html><head><meta charset='utf-8'><title>%s</title>" % html_escape(title),
        "<style>body{font-family:Arial,sans-serif;margin:24px;line-height:1.45;}"
        "h1,h2{margin-bottom:0.35em;}pre{background:#f6f8fa;padding:12px;border-radius:8px;overflow:auto;}"
        "img{max-width:100%%;height:auto;border:1px solid #ddd;border-radius:8px;margin:12px 0;}"
        ".muted{color:#666}</style></head><body>",
        f"<h1>{html_escape(title)}</h1>",
        f"<p class='muted'>Generated: {html_escape(time.strftime('%Y-%m-%d %H:%M:%S'))}</p>",
    ]
    for heading, body in sections:
        parts.append(f"<h2>{html_escape(heading)}</h2><pre>{html_escape(body)}</pre>")
    for caption, img_path in image_paths:
        if os.path.exists(img_path):
            rel = os.path.basename(img_path)
            parts.append(f"<h2>{html_escape(caption)}</h2><img src='{html_escape(rel)}' alt='{html_escape(caption)}'>")
    parts.append("</body></html>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))



class _CSVAppendTracker:
    def __init__(self, path: str, keep_rows: int = 240):
        self.path = path
        self.position = 0
        self.header = None
        self.partial = ""
        self.rows = 0
        self.last_row = None
        self._last_size = 0
        self.recent_rows: Deque[Dict[str, str]] = deque(maxlen=int(keep_rows))
        self.last_poll_wall_s: Optional[float] = None

    def poll(self):
        if not os.path.exists(self.path):
            return
        size = os.path.getsize(self.path)
        if size < self._last_size:
            self.position = 0
            self.header = None
            self.partial = ""
            self.rows = 0
            self.last_row = None
            self.recent_rows.clear()
        self._last_size = size
        with open(self.path, "r", encoding="utf-8", errors="ignore") as f:
            f.seek(self.position)
            chunk = f.read()
            self.position = f.tell()
        if not chunk:
            return
        text = self.partial + chunk
        if text.endswith("\n"):
            lines = text.splitlines()
            self.partial = ""
        else:
            lines = text.splitlines()
            self.partial = lines.pop() if lines else text
        got_row = False
        for line in lines:
            if not line.strip():
                continue
            if self.header is None:
                self.header = next(csv.reader([line]))
                continue
            vals = next(csv.reader([line]))
            if vals == self.header:
                continue
            if len(vals) != len(self.header):
                continue
            row = {k: vals[i] if i < len(vals) else "" for i, k in enumerate(self.header)}
            self.rows += 1
            self.last_row = row
            self.recent_rows.append(row)
            got_row = True
        if got_row:
            self.last_poll_wall_s = time.time()


def _to_num(v, default=float("nan")):
    try:
        if v is None or v == "":
            return default
        return float(v)
    except Exception:
        return default


def _dashboard_html_path(session_dir: str) -> str:
    return os.path.join(session_dir, "live_dashboard.html")


def _dashboard_json_path(session_dir: str) -> str:
    return os.path.join(session_dir, "live_dashboard.json")


_DASHBOARD_TEMPLATE_EMBEDDED = r'''<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Radar Vital Trainer Live Dashboard — v13.8.0 / v8.4.3 Full Telemetry + Position/Waveforms + Agreement Labels</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@400;500;700;900&family=Roboto+Mono:wght@400;500;700&display=swap" rel="stylesheet">
  <link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Material+Symbols+Rounded:opsz,wght,FILL,GRAD@24,500,1,0" />
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    :root {
      color-scheme: light;
      --bg: #f7f9ff;
      --surface: #ffffff;
      --surface-2: #f3f4fb;
      --surface-3: #eceef5;
      --text: #171c24;
      --text-muted: #5b6170;
      --outline: #c8ccd6;
      --primary: #2f6df6;
      --primary-container: #dde4ff;
      --secondary-container: #e5e9f7;
      --success: #146c37;
      --success-container: #d6f4db;
      --warning: #8f4f00;
      --warning-container: #ffe1bf;
      --error: #b3261e;
      --error-container: #ffd9d4;
      --rr: #1b9c67;
      --rr-container: #d5f5e7;
      --radius-xl: 28px;
      --radius-lg: 22px;
      --radius-md: 18px;
      --shadow-1: 0 1px 2px rgba(16,24,40,.06), 0 2px 8px rgba(16,24,40,.05);
    }
    * { box-sizing: border-box; }
    html, body { margin: 0; padding: 0; }
    body {
      min-height: 100vh;
      font-family: Roboto, system-ui, sans-serif;
      background:
        radial-gradient(circle at top left, rgba(47,109,246,.10), transparent 24%),
        radial-gradient(circle at top right, rgba(27,156,103,.08), transparent 18%),
        var(--bg);
      color: var(--text);
      padding: 20px 16px 28px;
    }
    .page { max-width: 1520px; margin: 0 auto; display: grid; gap: 18px; }
    .topbar, .card {
      background: color-mix(in srgb, var(--surface) 95%, white);
      border: 1px solid var(--outline);
      box-shadow: var(--shadow-1);
    }
    .topbar {
      border-radius: 34px;
      padding: 22px;
      display: grid;
      gap: 16px;
      position: sticky;
      top: 8px;
      z-index: 10;
      backdrop-filter: blur(12px);
    }
    .row { display: flex; align-items: center; justify-content: space-between; gap: 16px; flex-wrap: wrap; }
    .brand { display: flex; align-items: center; gap: 14px; min-width: 0; }
    .brand-icon {
      width: 56px; height: 56px; border-radius: 18px;
      display: grid; place-items: center;
      background: linear-gradient(135deg, var(--primary-container), white);
      color: var(--primary);
      flex: 0 0 auto;
    }
    .brand-icon .material-symbols-rounded { font-size: 30px; }
    .eyebrow {
      font-size: 12px; letter-spacing: .12em; text-transform: uppercase; font-weight: 800; color: var(--text-muted);
      margin-bottom: 4px;
    }
    h1 {
      margin: 0; font-size: clamp(28px, 4vw, 42px); line-height: 1.02; letter-spacing: -.5px; font-weight: 900;
    }
    .subtitle { color: var(--text-muted); font-size: 14px; margin-top: 6px; }
    .chip-row { display: flex; flex-wrap: wrap; gap: 10px; }
    .chip {
      display: inline-flex; align-items: center; gap: 8px;
      padding: 8px 14px; border-radius: 999px;
      font-size: 14px; font-weight: 700; background: var(--secondary-container); color: #22314c;
    }
    .chip.good { background: var(--success-container); color: var(--success); }
    .chip.warn { background: var(--warning-container); color: var(--warning); }
    .chip.bad { background: var(--error-container); color: var(--error); }
    .grid { display: grid; gap: 16px; }
    .cards-3 { grid-template-columns: repeat(3, minmax(0,1fr)); }
    .cards-4 { grid-template-columns: repeat(4, minmax(0,1fr)); }
    .cards-2 { grid-template-columns: repeat(2, minmax(0,1fr)); }
    .card { border-radius: var(--radius-xl); padding: 18px; min-width: 0; }
    .card-header { display: flex; align-items: flex-start; justify-content: space-between; gap: 12px; margin-bottom: 14px; }
    .title-wrap { display: flex; align-items: center; gap: 10px; }
    .title-wrap .material-symbols-rounded { color: var(--primary); font-size: 24px; }
    .title-wrap.rr .material-symbols-rounded { color: var(--rr); }
    .card-title { margin: 0; font-size: 20px; font-weight: 900; letter-spacing: -.02em; }
    .card-subtitle { margin-top: 4px; font-size: 13px; color: var(--text-muted); }
    .metric-row { display: grid; grid-template-columns: repeat(2, minmax(0,1fr)); gap: 12px; margin-bottom: 12px; }
    .metric {
      display: flex; align-items: baseline; gap: 8px;
      background: var(--surface-2); padding: 14px 16px; border-radius: 20px;
    }
    .metric.rr { background: color-mix(in srgb, var(--rr-container) 58%, white); }
    .metric-label { font-size: 18px; font-weight: 600; color: var(--text-muted); }
    .metric-value { font-size: 40px; font-weight: 900; color: var(--primary); letter-spacing: -1px; }
    .metric.rr .metric-value { color: var(--rr); }
    .metric-unit { font-size: 14px; color: var(--text-muted); font-weight: 700; }
    table { width: 100%; border-collapse: collapse; }
    td { padding: 10px 4px; border-bottom: 1px solid var(--outline); font-size: 14px; }
    tr:last-child td { border-bottom: none; }
    td:first-child { color: var(--text-muted); font-weight: 500; }
    td:last-child { color: var(--text); text-align: right; font-weight: 700; font-family: 'Roboto Mono', monospace; }
    .pillbar { display: flex; flex-wrap: wrap; gap: 8px; }
    .pill {
      display: inline-flex; align-items: center; gap: 6px; padding: 7px 10px; border-radius: 999px;
      background: var(--surface-2); color: var(--text-muted); font-size: 12px; font-weight: 700;
    }
    .pill.good { background: var(--success-container); color: var(--success); }
    .pill.warn { background: var(--warning-container); color: var(--warning); }
    .pill.bad { background: var(--error-container); color: var(--error); }
    .faults { display: grid; gap: 10px; }
    .fault {
      border-radius: var(--radius-md); padding: 14px 16px; display: grid; gap: 6px;
      border: 1px solid rgba(0,0,0,.04);
    }
    .fault.good { background: color-mix(in srgb, var(--success-container) 60%, white); }
    .fault.warn { background: color-mix(in srgb, var(--warning-container) 62%, white); }
    .fault.bad { background: color-mix(in srgb, var(--error-container) 64%, white); }
    .fault-title { display: flex; align-items: center; gap: 8px; font-weight: 900; }
    .fault-copy { font-size: 13px; color: var(--text-muted); line-height: 1.45; }
    .chart-panel {
      background: var(--surface-2); border-radius: 22px; padding: 14px 14px 8px; border: 1px solid rgba(0,0,0,.03); min-height: 300px;
    }
    .chart-title { margin: 0; font-size: 16px; font-weight: 900; }
    .chart-subtitle { margin: 4px 0 10px; font-size: 12px; color: var(--text-muted); }
    canvas { width: 100% !important; height: 230px !important; }
    pre {
      background: var(--surface-2); color: var(--text); border: 1px solid var(--outline); border-radius: 20px; padding: 14px;
      white-space: pre-wrap; font-family: 'Roboto Mono', monospace; font-size: 12px; line-height: 1.55; margin: 0;
      overflow-x: auto;
    }
    .footer { display: flex; align-items: center; gap: 8px; color: var(--text-muted); font-size: 12px; font-family: 'Roboto Mono', monospace; padding: 0 8px; }

    .radar-card-union {
      padding: 18px;
    }
    .radar-layout {
      display: grid;
      grid-template-columns: minmax(0, 1.2fr) minmax(320px, .8fr);
      gap: 16px;
      align-items: stretch;
    }
    .radar-panel, .telemetry-block, .wave-card {
      background: linear-gradient(180deg, rgba(255,255,255,.95), rgba(243,244,251,.9));
      border: 1px solid rgba(0,0,0,.05);
      border-radius: 22px;
      box-shadow: inset 0 1px 0 rgba(255,255,255,.7);
    }
    .radar-panel {
      padding: 16px;
      display: grid;
      gap: 12px;
      min-height: 100%;
    }
    .radar-panel-head {
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 12px;
    }
    .radar-panel-note {
      font-size: 13px;
      line-height: 1.45;
      color: var(--text-muted);
      max-width: 62ch;
    }
    .target-badge {
      flex: 0 0 auto;
      padding: 10px 14px;
      border-radius: 999px;
      background: var(--primary-container);
      color: var(--primary);
      font-weight: 900;
      font-size: 14px;
      box-shadow: inset 0 1px 0 rgba(255,255,255,.7);
    }
    .radar-wrap {
      position: relative;
      background: linear-gradient(180deg, #eef3ff 0%, #f8faff 100%);
      border: 1px solid rgba(47,109,246,.12);
      border-radius: 20px;
      padding: 10px;
      min-height: 360px;
      overflow: hidden;
    }
    #radarCanvas { height: 340px !important; }
    .spatial-side {
      display: grid;
      gap: 16px;
      min-width: 0;
    }
    .telemetry-block {
      padding: 16px;
    }
    .telemetry-big {
      display: grid;
      grid-template-columns: repeat(3, minmax(0,1fr));
      gap: 10px;
      margin-bottom: 14px;
    }
    .telemetry-line {
      background: var(--surface);
      border: 1px solid rgba(0,0,0,.05);
      border-radius: 18px;
      padding: 12px 14px;
      display: grid;
      gap: 4px;
      min-width: 0;
    }
    .telemetry-line .label {
      font-size: 12px;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: .08em;
      color: var(--text-muted);
    }
    .telemetry-line .value {
      font-size: clamp(24px, 3vw, 34px);
      line-height: 1;
      font-weight: 900;
      color: var(--primary);
      letter-spacing: -.04em;
      min-width: 0;
    }
    .telemetry-line .value.rr { color: var(--rr); }
    .kv-grid {
      display: grid;
      grid-template-columns: minmax(0,1fr) auto;
      gap: 10px 14px;
      align-items: center;
    }
    .kv-grid > div:nth-child(odd) {
      color: var(--text-muted);
      font-size: 13px;
      font-weight: 600;
    }
    .kv-grid > div:nth-child(even) {
      text-align: right;
      font-family: 'Roboto Mono', monospace;
      font-size: 13px;
      font-weight: 700;
      color: var(--text);
      word-break: break-word;
    }
    .wave-grid-union {
      display: grid;
      grid-template-columns: 1fr;
      gap: 14px;
    }
    .wave-card {
      padding: 14px 16px 10px;
    }
    .wave-title {
      margin: 0;
      font-size: 16px;
      font-weight: 900;
      letter-spacing: -.02em;
    }
    .wave-sub {
      margin: 4px 0 10px;
      font-size: 12px;
      color: var(--text-muted);
      line-height: 1.4;
    }
    #breathChart, #heartChart { height: 170px !important; }
    @media (max-width: 1180px) {
      .radar-layout { grid-template-columns: 1fr; }
    }
    @media (max-width: 820px) {
      .telemetry-big { grid-template-columns: 1fr; }
      .kv-grid { grid-template-columns: 1fr; }
      .kv-grid > div:nth-child(even) { text-align: left; }
      #radarCanvas { height: 280px !important; }
    }

    @media (max-width: 1180px) { .cards-3, .cards-4, .cards-2 { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="page">
    <section class="topbar">
      <div class="row">
        <div class="brand">
          <div class="brand-icon"><span class="material-symbols-rounded">monitor_heart</span></div>
          <div>
            <div class="eyebrow">Live collection</div>
            <h1>Radar Vital Trainer</h1>
            <div class="subtitle">v13.8.0 / v8.4.3 dashboard with spatial telemetry, session phases, harmonic labels, and HR anchor-bias diagnostics</div>
          </div>
        </div>
        <div class="chip-row" id="statusChips"></div>
      </div>
    </section>

    <section class="grid cards-2">
      <article class="card radar-card-union">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">explore</span><div><h2 class="card-title">Position / module view</h2><div class="card-subtitle">Semicircle position view plus live module-side spatial telemetry</div></div></div>
        </div>
        <div class="radar-layout">
          <div class="radar-panel">
            <div class="radar-panel-head">
              <div>
                <div class="eyebrow">Spatial view</div>
                <div class="radar-panel-note">Primary position uses <strong>target-info-first</strong> telemetry. The semicircle stays while all original 8.1 telemetry panels remain below.</div>
              </div>
              <div class="target-badge" id="targetCount">Target: --</div>
            </div>
            <div class="radar-wrap"><canvas id="radarCanvas"></canvas></div>
          </div>
          <div class="spatial-side">
            <div class="telemetry-block">
              <div class="telemetry-big">
                <div class="telemetry-line"><span class="label">FPS</span><span class="value" id="fpsValue">--</span></div>
                <div class="telemetry-line"><span class="label">Breath rate</span><span class="value rr" id="brValue">--</span></div>
                <div class="telemetry-line"><span class="label">Heart rate</span><span class="value" id="hrValue">--</span></div>
              </div>
              <div class="kv-grid">
                <div>Person detected</div><div id="personValue">--</div>
                <div>Distance</div><div id="distanceValue">--</div>
                <div>X position</div><div id="xValue">--</div>
                <div>Y position</div><div id="yValue">--</div>
                <div>Radius</div><div id="radiusValue">--</div>
                <div>Firmware</div><div id="fwValue">--</div>
                <div>Spatial source</div><div id="sourceValue">--</div>
                <div>Spatial age</div><div id="sourceAgeValue">--</div>
                <div>Doppler index</div><div id="dopValue">--</div>
                <div>Doppler estimate</div><div id="dopCmsValue">--</div>
              </div>
            </div>
            <div class="telemetry-block">
              <div class="card-title" style="font-size:16px; margin-bottom:10px;">Position source summary</div>
              <table><tbody id="positionTable"></tbody></table>
            </div>
          </div>
        </div>
      </article>
      <article class="card">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">monitor_heart</span><div><h2 class="card-title">Live phase waveforms</h2><div class="card-subtitle">Keep the new breath and heartbeat phase graphs without removing the original 8.1 trend panels</div></div></div>
        </div>
        <div class="wave-grid-union">
          <div class="wave-card">
            <h3 class="wave-title">Breath</h3>
            <div class="wave-sub">Live breath_phase history from 0x0A13-compatible phase stream</div>
            <canvas id="breathChart"></canvas>
          </div>
          <div class="wave-card">
            <h3 class="wave-title">Heartbeat</h3>
            <div class="wave-sub">Live heart_phase history from 0x0A13-compatible phase stream</div>
            <canvas id="heartChart"></canvas>
          </div>
        </div>
      </article>
    </section>

    <section class="grid cards-4">
      <article class="card">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">radar</span><div><h2 class="card-title">Radar</h2><div class="card-subtitle">Latest parsed radar row</div></div></div>
        </div>
        <div class="metric-row">
          <div class="metric"><span class="metric-label">HR</span><span class="metric-value" id="radarHr">--</span><span class="metric-unit">bpm</span></div>
          <div class="metric rr"><span class="metric-label">RR</span><span class="metric-value" id="radarRr">--</span><span class="metric-unit">br/min</span></div>
        </div>
        <table><tbody id="radarTable"></tbody></table>
      </article>

      <article class="card">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">bluetooth_connected</span><div><h2 class="card-title">BLE Reference</h2><div class="card-subtitle">Latest decoded oximeter packet</div></div></div>
        </div>
        <div class="metric-row">
          <div class="metric"><span class="metric-label">HR</span><span class="metric-value" id="bleHr">--</span><span class="metric-unit">bpm</span></div>
          <div class="metric rr"><span class="metric-label">RR</span><span class="metric-value" id="bleRr">--</span><span class="metric-unit">br/min</span></div>
        </div>
        <table><tbody id="bleTable"></tbody></table>
      </article>

      <article class="card">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">difference</span><div><h2 class="card-title">Current mismatch</h2><div class="card-subtitle">Operator-facing delta and fault view</div></div></div>
        </div>
        <div class="faults" id="faults"></div>
      </article>

      <article class="card">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">my_location</span><div><h2 class="card-title">Spatial / module</h2><div class="card-subtitle">Point cloud, target count, Doppler index + speed estimate, and firmware</div></div></div>
        </div>
        <div class="metric-row">
          <div class="metric"><span class="metric-label">Targets</span><span class="metric-value" id="numTargets">--</span><span class="metric-unit">count</span></div>
          <div class="metric rr"><span class="metric-label">|Dop|</span><span class="metric-value" id="maxDop">--</span><span class="metric-unit">idx</span></div>
        </div>
        <table><tbody id="spatialTable"></tbody></table>
      </article>
    </section>

    <section class="grid cards-2">
      <article class="card">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">conversion_path</span><div><h2 class="card-title">HR funnel telemetry</h2><div class="card-subtitle">Arbiter → reject-phase → blend → coherence → publish</div></div></div>
        </div>
        <div class="pillbar" id="hrFunnelPills"></div>
        <table><tbody id="hrFunnelTable"></tbody></table>
      </article>

      <article class="card">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">analytics</span><div><h2 class="card-title">HR stage values</h2><div class="card-subtitle">Numeric values carried through the HR path</div></div></div>
        </div>
        <table><tbody id="hrStageTable"></tbody></table>
      </article>
    </section>

    <section class="grid cards-2">
      <article class="card">
        <div class="card-header">
          <div class="title-wrap rr"><span class="material-symbols-rounded">air</span><div><h2 class="card-title">RR funnel telemetry</h2><div class="card-subtitle">Selector → accept-phase → blend → bias correction → Kalman → publish</div></div></div>
        </div>
        <div class="pillbar" id="rrFunnelPills"></div>
        <table><tbody id="rrFunnelTable"></tbody></table>
      </article>

      <article class="card">
        <div class="card-header">
          <div class="title-wrap rr"><span class="material-symbols-rounded">timeline</span><div><h2 class="card-title">RR stage values</h2><div class="card-subtitle">Anchor, recovery, and final RR stage values</div></div></div>
        </div>
        <table><tbody id="rrStageTable"></tbody></table>
      </article>
    </section>

    <section class="grid cards-2">
      <article class="card">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">show_chart</span><div><h2 class="card-title">Heart-rate trend</h2><div class="card-subtitle">Reported, candidate, raw, and BLE reference</div></div></div>
        </div>
        <div class="chart-panel"><canvas id="hrChart"></canvas></div>
      </article>
      <article class="card">
        <div class="card-header">
          <div class="title-wrap rr"><span class="material-symbols-rounded">show_chart</span><div><h2 class="card-title">Respiration trend</h2><div class="card-subtitle">Reported, candidate, final-publish, anchor, and BLE reference</div></div></div>
        </div>
        <div class="chart-panel"><canvas id="rrChart"></canvas></div>
      </article>
    </section>

    <section class="grid cards-2">
      <article class="card">
        <div class="card-header">
          <div class="title-wrap rr"><span class="material-symbols-rounded">monitoring</span><div><h2 class="card-title">RR recovery diagnostics</h2><div class="card-subtitle">Confidence, recovery counter, seed consistency, re-anchor state</div></div></div>
        </div>
        <div class="chart-panel"><canvas id="rrDiagChart"></canvas></div>
      </article>
      <article class="card">
        <div class="card-header">
          <div class="title-wrap"><span class="material-symbols-rounded">notification_important</span><div><h2 class="card-title">RR / stale-state warnings</h2><div class="card-subtitle">Anchor, recovery, candidate aging, and telemetry integrity</div></div></div>
        </div>
        <div class="faults" id="rrFaults"></div>
      </article>
    </section>

    <article class="card">
      <div class="card-header">
        <div class="title-wrap"><span class="material-symbols-rounded">history</span><div><h2 class="card-title">Recent events</h2><div class="card-subtitle">Live tail from the current session</div></div></div>
      </div>
      <pre id="eventsLog">Waiting for data...</pre>
    </article>

    <div class="footer" id="footerText"></div>
  </div>

  <script>
    const state = { charts: {}, dataUrl: './live_dashboard.json', refreshMs: 1000, lastPayload: null, disconnectedCount: 0, hardStaleAfter: 5, hardStaleAgeS: 5.0 };

    function num(v){ const n = Number(v); return Number.isFinite(n) ? n : null; }
    function fmt(v, d=1){ const n = num(v); return n === null ? '--' : n.toFixed(d); }
    function boolish(v){ const n = num(v); return n !== null ? (n >= 0.5) : !!v; }
    function textOrDash(v){ return (v === null || v === undefined || v === '') ? '--' : String(v); }
    function makeRow(label, value){ return `<tr><td>${label}</td><td>${value}</td></tr>`; }
    function chip(label, cls, icon){ return `<div class="chip ${cls||''}"><span class="material-symbols-rounded">${icon}</span>${label}</div>`; }
    function pill(label, value){
      let cls = '';
      if (value === 1 || value === true || value === 'yes' || value === 'triggered') cls = 'good';
      else if (value === 0 || value === false || value === 'no') cls = 'warn';
      return `<div class="pill ${cls}">${label}: ${value === null || value === undefined || value === '' ? '--' : value}</div>`;
    }
    function faultCard(severity, title, copy){
      const icon = severity === 'bad' ? 'error' : severity === 'warn' ? 'warning' : 'check_circle';
      return `<div class="fault ${severity}"><div class="fault-title"><span class="material-symbols-rounded">${icon}</span>${title}</div><div class="fault-copy">${copy}</div></div>`;
    }
    function normalize(raw){ return { meta: raw?.meta||{}, radar: raw?.radar||{}, ble: raw?.ble||{}, thresholds: raw?.thresholds||{}, faults: Array.isArray(raw?.faults)?raw.faults:[], events: Array.isArray(raw?.events)?raw.events:[], series: raw?.series||{}, analysis: raw?.analysis||null }; }
    function lastFromSeries(series, key){ const arr = Array.isArray(series?.[key]) ? series[key] : []; return arr.length ? arr[arr.length-1] : null; }
    function getRadar(payload, key){
      const radar = payload.radar || {};
      if (radar[key] !== undefined) return radar[key];
      const direct = lastFromSeries(payload.series, key);
      if (direct !== null) return direct;
      return null;
    }
    function stageOrFallback(payload, primary, fallbackKeys=[]){
      const first = getRadar(payload, primary);
      if (first !== null && first !== undefined) return first;
      for (const k of fallbackKeys){
        const val = getRadar(payload, k);
        if (val !== null && val !== undefined) return val;
      }
      return null;
    }
    function payloadIsHardStale(payload){
      const age = num(payload?.radar?.last_seen_age_s);
      return state.disconnectedCount >= state.hardStaleAfter || (age !== null && age >= state.hardStaleAgeS);
    }
    function rrStageValue(payload, primary, fallbackKeys=[]){
      const anchorFresh = boolish(getRadar(payload,'rr_anchor_fresh'));
      if (!anchorFresh && primary === 'rr_post_kalman') return null;
      return stageOrFallback(payload, primary, fallbackKeys);
    }

    function sourceName(v){ return ({0:'none',1:'target_info',2:'point_cloud',3:'frame_0x0A17'}[Math.round(num(v) ?? 0)] || textOrDash(v)); }
    function sessionPhaseName(v){ return ({0:'absent',1:'warmup',2:'settling',3:'locked',4:'post_motion',5:'leaving'}[Math.round(num(v) ?? 0)] || textOrDash(v)); }
    function harmonicSummary(v){ const n=Math.round(num(v) ?? 0); if(!n) return 'none'; const tags=[]; if(n&1) tags.push('rr_raw'); if(n&2) tags.push('hr_arb'); if(n&4) tags.push('hr_half'); if(n&8) tags.push('hr_amb'); if(n&16) tags.push('rr_sub'); if(n&32) tags.push('rr_harm'); return tags.join('|'); }
    function hrPathSourceName(v, fallback){ if (fallback) return String(fallback); const n=Math.round(num(v) ?? -999); return ({0:'none',1:'auto publish',2:'spectral path',3:'raw blend',4:'raw fallback',5:'raw only (no phase)',6:'direct raw fast path'})[n] || textOrDash(v); }
    function hrAnchorSourceName(v, fallback){ if (fallback) return String(fallback); const n=Math.round(num(v) ?? -999); return ({0:'none',1:'trusted_phase',2:'tracking_raw',3:'latched_raw',4:'arbiter_anchor',5:'rejectphase_anchor'})[n] || textOrDash(v); }
    function computeFps(payload){
      const explicit = num(getRadar(payload, 'fps_hz'));
      if (explicit !== null && explicit > 0) return explicit;
      const t = Array.isArray(payload?.series?.t) ? payload.series.t.map(num).filter(v => v !== null) : [];
      if (t.length < 3) return null;
      const diffs = [];
      for (let i=1;i<t.length;i++){ const d=t[i]-t[i-1]; if (d>0) diffs.push(d); }
      if (!diffs.length) return null;
      diffs.sort((a,b)=>a-b);
      const med = diffs[Math.floor(diffs.length/2)];
      return med > 0 ? 1/med : null;
    }
    function drawRadar(payload){
      const c = document.getElementById('radarCanvas');
      if (!c) return;
      const ctx = c.getContext('2d');
      const dpr = window.devicePixelRatio || 1;
      const rect = c.getBoundingClientRect();
      if (!rect.width || !rect.height) return;
      c.width = rect.width * dpr; c.height = rect.height * dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
      const w = rect.width, h = rect.height;
      ctx.clearRect(0,0,w,h);
      ctx.fillStyle = '#f7f9fc'; ctx.fillRect(0,0,w,h);
      const originX = w/2, originY = h - 18;
      const maxR = Math.min(w*0.46, h*0.84); const maxMeters = 6;
      ctx.strokeStyle = '#cfd7e4'; ctx.lineWidth = 2;
      for(let i=1;i<=6;i++){
        const r = maxR * (i/maxMeters);
        ctx.beginPath(); ctx.arc(originX, originY, r, Math.PI, 2*Math.PI); ctx.stroke();
        ctx.fillStyle = '#9aa6ba'; ctx.font='12px Roboto'; ctx.fillText(`${i}m`, originX + 6, originY - r - 4);
      }
      ctx.beginPath(); ctx.moveTo(originX, originY); ctx.lineTo(originX, originY - maxR); ctx.strokeStyle='#3b434f'; ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, originY); ctx.lineTo(w, originY); ctx.stroke();
      ctx.fillStyle = '#171c24'; ctx.beginPath(); ctx.arc(originX, originY, 4, 0, Math.PI*2); ctx.fill();
      const x = num(getRadar(payload,'primary_x'));
      const y = num(getRadar(payload,'primary_y'));
      if (x !== null && y !== null && y >= 0){
        const px = originX + (x / maxMeters) * maxR;
        const py = originY - (y / maxMeters) * maxR;
        ctx.fillStyle = '#2f6df6'; ctx.beginPath(); ctx.arc(px, py, 8, 0, Math.PI*2); ctx.fill();
        ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 3; ctx.stroke();
        ctx.fillStyle = '#171c24'; ctx.font = '12px Roboto Mono';
        ctx.fillText(`(${x.toFixed(2)}, ${y.toFixed(2)}) m`, Math.min(w-120, px+12), Math.max(18, py-10));
      } else {
        ctx.fillStyle = '#7b8494'; ctx.font = '14px Roboto'; ctx.fillText('No fresh spatial fix', 16, 24);
      }
    }

    function buildCharts(){
      const common = {
        responsive: true, maintainAspectRatio: false, animation: false,
        interaction: { mode: 'index', intersect: false },
        plugins: { legend: { position: 'top' } },
        scales: { x: { ticks: { color: '#5b6070' }, grid: { color: 'rgba(116,120,132,.12)' } }, y: { ticks: { color: '#5b6070' }, grid: { color: 'rgba(116,120,132,.12)' } } }
      };
      state.charts.hr = new Chart(document.getElementById('hrChart'), {
        type: 'line',
        data: { labels: [], datasets: [
          { label: 'Reported HR', data: [], borderColor: '#2f6df6', backgroundColor: '#2f6df6', borderWidth: 3, tension: .25 },
          { label: 'Candidate HR', data: [], borderColor: '#7e57c2', backgroundColor: '#7e57c2', borderWidth: 2, tension: .25, borderDash: [7,4] },
          { label: 'Raw HR', data: [], borderColor: '#8d99ae', backgroundColor: '#8d99ae', borderWidth: 2, tension: .25, borderDash: [3,4] },
          { label: 'BLE HR', data: [], borderColor: '#ef4444', backgroundColor: '#ef4444', borderWidth: 2, tension: .25 },
          { label: 'Trusted phase anchor', data: [], borderColor: '#0f766e', backgroundColor: '#0f766e', borderWidth: 2, tension: .25, borderDash: [4,4] }
        ] }, options: common
      });
      state.charts.rr = new Chart(document.getElementById('rrChart'), {
        type: 'line',
        data: { labels: [], datasets: [
          { label: 'Reported RR', data: [], borderColor: '#16a34a', backgroundColor: '#16a34a', borderWidth: 3, tension: .25 },
          { label: 'Candidate RR', data: [], borderColor: '#1b9c67', backgroundColor: '#1b9c67', borderWidth: 2, tension: .25, borderDash: [7,4] },
          { label: 'Final publish candidate RR', data: [], borderColor: '#0ea5a4', backgroundColor: '#0ea5a4', borderWidth: 2, tension: .25, borderDash: [3,4] },
          { label: 'RR anchor', data: [], borderColor: '#f59e0b', backgroundColor: '#f59e0b', borderWidth: 2, tension: .2 },
          { label: 'BLE RR', data: [], borderColor: '#ef4444', backgroundColor: '#ef4444', borderWidth: 2, tension: .25 }
        ] }, options: common
      });
      state.charts.rrDiag = new Chart(document.getElementById('rrDiagChart'), {
        type: 'line',
        data: { labels: [], datasets: [
          { label: 'RR candidate conf', data: [], borderColor: '#2f6df6', backgroundColor: '#2f6df6', borderWidth: 2, tension: .25, yAxisID: 'y1' },
          { label: 'RR recovery count', data: [], borderColor: '#7e57c2', backgroundColor: '#7e57c2', borderWidth: 2, tension: .1, yAxisID: 'y2' },
          { label: 'RR seed consistency', data: [], borderColor: '#1b9c67', backgroundColor: '#1b9c67', borderWidth: 2, tension: .1, yAxisID: 'y2' }
        ] },
        options: { ...common, scales: { ...common.scales, y1: { position: 'left', min: 0, max: 1.2, grid: { color: 'rgba(116,120,132,.12)' }, ticks: { color: '#5b6070' } }, y2: { position: 'right', min: 0, max: 6, grid: { drawOnChartArea: false }, ticks: { color: '#5b6070' } } } }
      });
      state.charts.breath = new Chart(document.getElementById('breathChart'), {
        type: 'line',
        data: { labels: [], datasets: [
          { label: 'breath_phase', data: [], borderColor: '#e05a2a', backgroundColor: '#e05a2a', borderWidth: 2, pointRadius: 0, tension: .18 }
        ] },
        options: common
      });
      state.charts.heart = new Chart(document.getElementById('heartChart'), {
        type: 'line',
        data: { labels: [], datasets: [
          { label: 'heart_phase', data: [], borderColor: '#c93232', backgroundColor: '#c93232', borderWidth: 2, pointRadius: 0, tension: .18 }
        ] },
        options: common
      });
    }
    function updateCharts(payload){
      const s = payload.series || {};
      const labels = Array.isArray(s.t) ? s.t : [];
      state.charts.hr.data.labels = labels;
      state.charts.hr.data.datasets[0].data = Array.isArray(s.reported_hr) ? s.reported_hr : (Array.isArray(s.radar_hr) ? s.radar_hr : []);
      state.charts.hr.data.datasets[1].data = Array.isArray(s.candidate_hr) ? s.candidate_hr : [];
      state.charts.hr.data.datasets[2].data = Array.isArray(s.raw_hr) ? s.raw_hr : [];
      state.charts.hr.data.datasets[3].data = Array.isArray(s.ble_hr) ? s.ble_hr : [];
      state.charts.hr.data.datasets[4].data = Array.isArray(s.hr_trusted_phase_anchor) ? s.hr_trusted_phase_anchor : [];
      state.charts.hr.update('none');

      state.charts.rr.data.labels = labels;
      state.charts.rr.data.datasets[0].data = Array.isArray(s.reported_rr) ? s.reported_rr : (Array.isArray(s.radar_rr) ? s.radar_rr : []);
      state.charts.rr.data.datasets[1].data = Array.isArray(s.candidate_rr) ? s.candidate_rr : [];
      state.charts.rr.data.datasets[2].data = Array.isArray(s.rr_final_publish_candidate) ? s.rr_final_publish_candidate : [];
      state.charts.rr.data.datasets[3].data = Array.isArray(s.rr_anchor_value) ? s.rr_anchor_value : [];
      state.charts.rr.data.datasets[4].data = Array.isArray(s.ble_rr) ? s.ble_rr : [];
      state.charts.rr.update('none');

      state.charts.rrDiag.data.labels = labels;
      state.charts.rrDiag.data.datasets[0].data = Array.isArray(s.candidate_rr_conf) ? s.candidate_rr_conf : [];
      state.charts.rrDiag.data.datasets[1].data = Array.isArray(s.rr_fundamental_recovery_count) ? s.rr_fundamental_recovery_count : [];
      state.charts.rrDiag.data.datasets[2].data = Array.isArray(s.rr_raw_seed_consistent_count) ? s.rr_raw_seed_consistent_count : [];
      state.charts.rrDiag.update('none');

      const breath = Array.isArray(s.breath_phase) ? s.breath_phase : [];
      const heart = Array.isArray(s.heart_phase) ? s.heart_phase : [];
      const breathLabels = labels.slice(-breath.length);
      const heartLabels = labels.slice(-heart.length);
      state.charts.breath.data.labels = breathLabels;
      state.charts.breath.data.datasets[0].data = breath;
      state.charts.breath.update('none');
      state.charts.heart.data.labels = heartLabels;
      state.charts.heart.data.datasets[0].data = heart;
      state.charts.heart.update('none');
    }
    function render(payload){
      const meta = payload.meta || {};
      const radar = payload.radar || {};
      const ble = payload.ble || {};
      const thresholds = payload.thresholds || {};
      const hardStale = payloadIsHardStale(payload);
      const statusChips = [
        chip(meta.status || 'unknown', meta.status === 'running' ? 'good' : (String(meta.status||'').includes('stopped') ? 'bad' : 'warn'), meta.status === 'running' ? 'play_circle' : 'pause_circle'),
        chip(`${fmt(meta.elapsed_s,1)}s elapsed`, '', 'timer'),
        chip(meta.remaining_s === null || meta.remaining_s === undefined ? 'manual stop' : `${fmt(meta.remaining_s,1)}s remaining`, Number(meta.remaining_s) <= 5 ? 'warn' : '', 'schedule')
      ];
      const moduleFwParts = [radar.module_fw_major, radar.module_fw_sub, radar.module_fw_mod].filter(v => v !== undefined && v !== null && `${v}` !== '');
      const sketchFwParts = [radar.sketch_major, radar.sketch_sub, radar.sketch_mod].filter(v => v !== undefined && v !== null && `${v}` !== '');
      if (sketchFwParts.length === 3 && sketchFwParts.every(v => Number.isFinite(Number(v)) && Number(v) >= 0)) statusChips.push(chip(`sketch ${sketchFwParts.join('.')}`, 'good', 'memory'));
      if (moduleFwParts.length === 3 && moduleFwParts.every(v => Number.isFinite(Number(v)) && Number(v) >= 0)) statusChips.push(chip(`module fw ${moduleFwParts.join('.')}`, '', 'memory'));
      if (boolish(radar.use_fast_path)) statusChips.push(chip('raw fast path', 'good', 'bolt'));
      if (num(radar.session_phase) !== null) statusChips.push(chip(`phase ${sessionPhaseName(radar.session_phase)}`, '', 'schedule'));
      if (boolish(radar.phase_valid_this_frame)) statusChips.push(chip('live phase valid', 'good', 'check_circle'));
      if (!boolish(radar.dsp_ran_this_frame) && !hardStale) statusChips.push(chip('DSP idle this loop', 'warn', 'pause_circle'));
      const lastSeenAge = num(radar.last_seen_age_s);
      if (lastSeenAge !== null && lastSeenAge > 2.5) statusChips.push(chip(`telemetry stale ${fmt(lastSeenAge,1)}s`, hardStale ? 'bad' : 'warn', hardStale ? 'error' : 'warning'));
      if (state.disconnectedCount > 0) statusChips.push(chip(`using last payload (${state.disconnectedCount})`, hardStale ? 'bad' : 'warn', 'wifi_off'));
      if (hardStale) statusChips.push(chip('live values expired', 'bad', 'timer_off'));
      
      const anchorName = textOrDash(getRadar(payload, 'hr_anchor_source_name') || hrAnchorSourceName(getRadar(payload, 'hr_anchor_source')));
      if (anchorName !== '--' && anchorName !== 'none') statusChips.push(chip(`anchor ${anchorName}`, '', 'anchor'));
      if (boolish(getRadar(payload, 'hr_raw_high_bias_suspect'))) statusChips.push(chip('high bias suspect', 'warn', 'trending_up'));
      
      document.getElementById('statusChips').innerHTML = statusChips.join('');

      const radarHrLive = getRadar(payload, 'reported_hr') ?? getRadar(payload, 'radar_hr');
      const radarRrLive = getRadar(payload, 'reported_rr') ?? getRadar(payload, 'radar_rr');
      const radarHr = hardStale ? null : radarHrLive;
      const radarRr = hardStale ? null : radarRrLive;
      const bleHr = ble.hr ?? lastFromSeries(payload.series, 'ble_hr');
      const bleRr = ble.rr ?? lastFromSeries(payload.series, 'ble_rr');
      document.getElementById('radarHr').textContent = fmt(radarHr, 1);
      document.getElementById('radarRr').textContent = fmt(radarRr, 1);
      document.getElementById('bleHr').textContent = fmt(bleHr, 1);
      document.getElementById('bleRr').textContent = fmt(bleRr, 1);

      const fps = computeFps(payload);
      const radiusCm = num(getRadar(payload,'position_radius_cm')) ?? ((num(getRadar(payload,'primary_x')) !== null && num(getRadar(payload,'primary_y')) !== null) ? Math.hypot(num(getRadar(payload,'primary_x')), num(getRadar(payload,'primary_y'))) * 100 : null);
      const sketchFwParts2 = [radar.sketch_major, radar.sketch_sub, radar.sketch_mod].map(v => num(v));
      const moduleFwParts2 = [radar.module_fw_major, radar.module_fw_sub, radar.module_fw_mod].map(v => num(v));
      const sketchFwText = textOrDash(radar.sketch_firmware_version || radar.firmware_version);
      const moduleFwText = textOrDash(radar.module_firmware_version);
      const fwText = (() => {
        const sketchText = (sketchFwText !== '--' && !String(sketchFwText).includes('-1'))
          ? sketchFwText
          : (sketchFwParts2.every(v => v !== null && v >= 0) ? `${sketchFwParts2[0]}.${sketchFwParts2[1]}.${sketchFwParts2[2]}` : '--');
        const moduleText = (moduleFwText !== '--' && !String(moduleFwText).includes('-1'))
          ? moduleFwText
          : (moduleFwParts2.every(v => v !== null && v >= 0) ? `${moduleFwParts2[0]}.${moduleFwParts2[1]}.${moduleFwParts2[2]}` : '--');
        if (sketchText !== '--' && moduleText !== '--') return `${sketchText} | module ${moduleText}`;
        return sketchText !== '--' ? sketchText : moduleText;
      })();
      document.getElementById('targetCount').textContent = `Target: ${textOrDash(getRadar(payload,'num_targets'))}`;
      document.getElementById('fpsValue').textContent = fmt(fps,1);
      document.getElementById('brValue').textContent = fmt(radarRr,1);
      document.getElementById('hrValue').textContent = fmt(radarHr,1);
      document.getElementById('personValue').textContent = boolish(getRadar(payload,'human')) ? 'Yes' : 'No';
      document.getElementById('distanceValue').textContent = fmt(getRadar(payload,'distance_cm'),1) === '--' ? '--' : `${fmt(getRadar(payload,'distance_cm'),1)} cm`;
      document.getElementById('xValue').textContent = fmt(getRadar(payload,'primary_x'),3) === '--' ? '--' : `${fmt(getRadar(payload,'primary_x'),3)} m`;
      document.getElementById('yValue').textContent = fmt(getRadar(payload,'primary_y'),3) === '--' ? '--' : `${fmt(getRadar(payload,'primary_y'),3)} m`;
      document.getElementById('radiusValue').textContent = radiusCm === null ? '--' : `${radiusCm.toFixed(1)} cm`;
      document.getElementById('fwValue').textContent = fwText;
      document.getElementById('sourceValue').textContent = sourceName(getRadar(payload,'spatial_source'));
      document.getElementById('sourceAgeValue').textContent = fmt(getRadar(payload,'spatial_age_ms'),0) === '--' ? '--' : `${fmt(getRadar(payload,'spatial_age_ms'),0)} ms`;
      document.getElementById('dopValue').textContent = textOrDash(getRadar(payload,'primary_dop'));
      document.getElementById('dopCmsValue').textContent = fmt(getRadar(payload,'primary_dop_speed_cms'),2) === '--' ? '--' : `${fmt(getRadar(payload,'primary_dop_speed_cms'),2)} cm/s`;
      document.getElementById('positionTable').innerHTML = [
        makeRow('Spatial source', sourceName(getRadar(payload,'spatial_source'))),
        makeRow('Target info live', boolish(getRadar(payload,'target_info_ok')) ? 'Yes' : 'No'),
        makeRow('Point cloud live', boolish(getRadar(payload,'point_cloud_ok')) ? 'Yes' : 'No'),
        makeRow('Multi-target', boolish(getRadar(payload,'multi_target')) ? 'Yes' : 'No'),
        makeRow('Cluster', textOrDash(getRadar(payload,'primary_cluster'))),
        makeRow('Spatial age', fmt(getRadar(payload,'spatial_age_ms'),0) === '--' ? '--' : `${fmt(getRadar(payload,'spatial_age_ms'),0)} ms`),
        makeRow('Phase warmup', boolish(getRadar(payload,'phase_warmup_complete')) ? 'Yes' : 'No'),
        makeRow('Clutter alpha', fmt(getRadar(payload,'current_clutter_alpha'),3))
      ].join('');
      drawRadar(payload);

      document.getElementById('radarTable').innerHTML = [
        makeRow('Rows', textOrDash(radar.rows)),
        makeRow('Display source', hardStale ? 'expired / stale payload' : ((num(radar.reported_hr) !== null && num(radar.reported_hr) > 0) ? 'validated publish' : ((num(radar.hr) !== null && num(radar.hr) > 0) ? 'last valid (stale)' : '--'))),
        makeRow('Session phase', sessionPhaseName(getRadar(payload,'session_phase'))),
        makeRow('Harmonic mode', (Array.isArray(radar.harmonic_mode_labels) && radar.harmonic_mode_labels.length) ? radar.harmonic_mode_labels.join(' | ') : harmonicSummary(getRadar(payload,'harmonic_mode'))),
        makeRow('HR triple agree', textOrDash(getRadar(payload,'hr_triple_agree'))),
        makeRow('RR triple agree', textOrDash(getRadar(payload,'rr_triple_agree'))),
        makeRow('Candidate HR', fmt(getRadar(payload,'candidate_hr'),1)),
        makeRow('Candidate RR', fmt(getRadar(payload,'candidate_rr'),1)),
        makeRow('Raw HR', fmt(getRadar(payload,'raw_hr'),1)),
        makeRow('Raw RR', fmt(getRadar(payload,'raw_rr'),1)),
        makeRow('PQI Heart', fmt(getRadar(payload,'pqi_heart'),2)),
        makeRow('Heart PQI gate', fmt(getRadar(payload,'heart_pqi_gate'),2)),
        makeRow('PQI Breath', fmt(getRadar(payload,'pqi_breath'),2)),
        makeRow('Phase valid this frame', boolish(getRadar(payload,'phase_valid_this_frame')) ? 'Yes' : 'No'),
        makeRow('Distance (cm)', fmt(getRadar(payload,'distance_cm') ?? radar.distance_cm,1)),
        makeRow('Motion', textOrDash(radar.motion)),
        makeRow('Human', textOrDash(radar.human)),
        makeRow('HR valid', textOrDash(radar.logged_hr_valid)),
        makeRow('RR valid', textOrDash(radar.logged_rr_valid))
      ].join('');

      document.getElementById('numTargets').textContent = textOrDash(radar.num_targets);
      document.getElementById('maxDop').textContent = fmt(radar.max_dop_abs, 1);
      const maxDopSpeedEl = document.getElementById('maxDopSpeed'); if (maxDopSpeedEl) maxDopSpeedEl.textContent = fmt(radar.max_dop_speed_cms, 2);
      document.getElementById('spatialTable').innerHTML = [
        makeRow('Point cloud ok', textOrDash(radar.point_cloud_ok)),
        makeRow('Target info ok', textOrDash(radar.target_info_ok)),
        makeRow('Doppler motion', textOrDash(radar.doppler_motion)),
        makeRow('Cluster anomaly', textOrDash(radar.cluster_anomaly)),
        makeRow('Multi-target', textOrDash(radar.multi_target)),
        makeRow('Primary X', fmt(radar.primary_x, 2)),
        makeRow('Primary Y', fmt(radar.primary_y, 2)),
        makeRow('Primary dop', textOrDash(radar.primary_dop)),
        makeRow('Primary cluster', textOrDash(radar.primary_cluster)),
        makeRow('Fast path', textOrDash(radar.use_fast_path)),
        makeRow('Phase warmup', textOrDash(radar.phase_warmup_complete)),
        makeRow('Clutter alpha', fmt(radar.current_clutter_alpha, 3)),
        makeRow('Phase valid', textOrDash(radar.phase_valid_this_frame)),
        makeRow('Speed estimate basis', '0.49 cm/s per dop index'),
        makeRow('HR path source', hrPathSourceName(radar.hr_path_source, radar.hr_path_source_name)),
        makeRow('Module fw', moduleFwParts.length === 3 ? moduleFwParts.join('.') : '--')
      ].join('');

      document.getElementById('bleTable').innerHTML = [
        makeRow('Rows', textOrDash(ble.rows)),
        makeRow('Raw packets', textOrDash(ble.raw_packets)),
        makeRow('SpO₂', ble.spo2 === undefined ? '--' : `${fmt(ble.spo2,0)}%`),
        makeRow('PI', fmt(ble.pi,1)),
        makeRow('Address', textOrDash(ble.address)),
        makeRow('Profile', textOrDash(ble.profile))
      ].join('');

      const hrDelta = (num(radarHr) !== null && num(bleHr) !== null) ? num(radarHr) - num(bleHr) : null;
      const rrDelta = (num(radarRr) !== null && num(bleRr) !== null) ? num(radarRr) - num(bleRr) : null;
      const hrWarn = num(thresholds.hr_delta_warn) ?? 3; const hrCrit = num(thresholds.hr_delta_crit) ?? 8;
      const rrWarn = num(thresholds.rr_delta_warn) ?? 2; const rrCrit = num(thresholds.rr_delta_crit) ?? 4;
      const cards = [];
      if (hrDelta !== null) cards.push(faultCard(Math.abs(hrDelta) > hrCrit ? 'bad' : (Math.abs(hrDelta) > hrWarn ? 'warn' : 'good'), 'Heart-rate mismatch', `Radar ${fmt(radarHr,1)} vs BLE ${fmt(bleHr,1)} → Δ ${fmt(hrDelta,1)} bpm.`));
      if (rrDelta !== null) cards.push(faultCard(Math.abs(rrDelta) > rrCrit ? 'bad' : (Math.abs(rrDelta) > rrWarn ? 'warn' : 'good'), 'Respiration mismatch', `Radar ${fmt(radarRr,1)} vs BLE ${fmt(bleRr,1)} → Δ ${fmt(rrDelta,1)} br/min.`));
      if (boolish(radar.multi_target)) cards.push(faultCard('bad', 'Multiple targets detected', 'More than one target is present in the point-cloud/target-info stream. Host DSP is likely contaminated.'));
      if (boolish(radar.cluster_anomaly)) cards.push(faultCard('warn', 'Cluster anomaly', 'Point-cloud clusters disagree with the single-target assumption. Review multipath / ghost behavior.'));
      if (boolish(radar.doppler_motion)) cards.push(faultCard('warn', 'Doppler motion active', 'Point-cloud Doppler indicates body motion even if phase-energy motion is subtle.'));
      if (boolish(getRadar(payload,'hr_raw_high_bias_suspect'))) cards.push(faultCard('warn', 'High HR anchor bias suspected', 'Raw/latch HR is staying above the trusted phase anchor while lower phase-backed evidence is present.'));
      const faults = Array.isArray(payload.faults) ? payload.faults : [];
      cards.push(...faults.slice(0,2).map(f => faultCard(f.severity || 'warn', f.title || 'Fault', f.copy || '')));
      document.getElementById('faults').innerHTML = cards.length ? cards.join('') : faultCard('good','No active fault','No operator-facing fault was reported in the current payload.');

      document.getElementById('hrFunnelPills').innerHTML = [
        pill('Arbiter corrected', boolish(getRadar(payload,'hr_arbiter_corrected')) ? 1 : 0),
        pill('Reject-phase rejected', boolish(getRadar(payload,'hr_rejectphase_rejected')) ? 1 : 0),
        pill('Coherence rejected', boolish(getRadar(payload,'hr_coherence_rejected')) ? 1 : 0),
        pill('Raw source', textOrDash(getRadar(payload,'hr_raw_source'))),
        pill('Raw agree', boolish(getRadar(payload,'hr_raw_agree')) ? 1 : 0),
        pill('Bypass active', boolish(getRadar(payload,'hr_bypass_active')) ? 1 : 0),
        pill('Grace active', boolish(getRadar(payload,'hr_grace_active')) ? 1 : 0),
        pill('Triple agree', boolish(getRadar(payload,'hr_triple_agree')) ? 1 : 0),
        pill('Trust fresh', boolish(getRadar(payload,'hr_trust_fresh')) ? 1 : 0),
        pill('High-bias suspect', boolish(getRadar(payload,'hr_raw_high_bias_suspect')) ? 1 : 0),
        pill('RR anchor fresh', boolish(getRadar(payload,'rr_anchor_fresh')) ? 1 : 0),
        pill('RR outlier persist', textOrDash(getRadar(payload,'rr_outlier_persist')))
      ].join('');

      document.getElementById('hrFunnelTable').innerHTML = [
        makeRow('Agreement error (bpm)', fmt(getRadar(payload,'hr_agree_err_bpm'),2)),
        makeRow('HR ZC bpm/conf', `${fmt(getRadar(payload,'hr_zc_bpm'),2)} / ${fmt(getRadar(payload,'hr_zc_conf'),3)}`),
        makeRow('HR spec bpm/mag', `${fmt(getRadar(payload,'hr_spec_bpm'),2)} / ${fmt(getRadar(payload,'hr_spec_mag'),5)}`),
        makeRow('HR triple agree', textOrDash(getRadar(payload,'hr_triple_agree'))),
        makeRow('Bypass PQI ok', textOrDash(getRadar(payload,'hr_bypass_pqi_ok'))),
        makeRow('Bypass conf ok', textOrDash(getRadar(payload,'hr_bypass_conf_ok'))),
        makeRow('Bypass gate ok', textOrDash(getRadar(payload,'hr_bypass_gate_ok'))),
        makeRow('Grace eligible', textOrDash(getRadar(payload,'hr_grace_eligible'))),
        makeRow('Grace active', textOrDash(getRadar(payload,'hr_grace_active'))),
        makeRow('Reject-phase anchor used', textOrDash(getRadar(payload,'hr_rejectphase_anchor_used'))),
        makeRow('Reject-phase anchor value', fmt(getRadar(payload,'hr_rejectphase_anchor_value'),2)),
        makeRow('Arbiter anchor used', textOrDash(getRadar(payload,'hr_arbiter_anchor_used'))),
        makeRow('Arbiter anchor value', fmt(getRadar(payload,'hr_arbiter_anchor_value'),2)),
        makeRow('Raw looks like half-rate', textOrDash(getRadar(payload,'hr_raw_looks_like_half_rate'))),
        makeRow('Trusted anchor value', fmt(getRadar(payload,'hr_trusted_anchor_value'),2)),
        makeRow('Trusted phase anchor', fmt(getRadar(payload,'hr_trusted_phase_anchor'),2)),
        makeRow('Anchor source', textOrDash(getRadar(payload,'hr_anchor_source_name') || hrAnchorSourceName(getRadar(payload,'hr_anchor_source')))),
        makeRow('Anchor error (bpm)', fmt(getRadar(payload,'hr_anchor_err_bpm'),2)),
        makeRow('Raw high-bias suspect', textOrDash(getRadar(payload,'hr_raw_high_bias_suspect'))),
        makeRow('RR raw agree ok', textOrDash(getRadar(payload,'rr_raw_agree_ok')))
      ].join('');

      document.getElementById('hrStageTable').innerHTML = [
        makeRow('Pre reject-phase', fmt(getRadar(payload,'hr_pre_rejectphase'),2)),
        makeRow('Post reject-phase', fmt(getRadar(payload,'hr_post_rejectphase'),2)),
        makeRow('Post blend', fmt(getRadar(payload,'hr_post_blend'),2)),
        makeRow('Post coherence', fmt(getRadar(payload,'hr_post_coherence'),2)),
        makeRow('Final publish candidate', fmt(getRadar(payload,'hr_final_publish_candidate'),2)),
        makeRow('Trusted phase anchor', fmt(getRadar(payload,'hr_trusted_phase_anchor'),2)),
        makeRow('Anchor source', textOrDash(getRadar(payload,'hr_anchor_source_name') || hrAnchorSourceName(getRadar(payload,'hr_anchor_source')))),
        makeRow('Anchor error', fmt(getRadar(payload,'hr_anchor_err_bpm'),2)),
        makeRow('HR ZC bpm', fmt(getRadar(payload,'hr_zc_bpm'),2)),
        makeRow('HR spec bpm', fmt(getRadar(payload,'hr_spec_bpm'),2)),
        makeRow('Logged HR valid', textOrDash(radar.logged_hr_valid)),
        makeRow('HR publish reason', textOrDash(radar.hr_publish_reason_name ? `${radar.hr_publish_reason_name} (${textOrDash(getRadar(payload,'hr_publish_reason') ?? radar.hr_publish_reason)})` : (getRadar(payload,'hr_publish_reason') ?? radar.hr_publish_reason))),
        makeRow('RR publish reason', textOrDash(radar.rr_publish_reason_name ? `${radar.rr_publish_reason_name} (${textOrDash(getRadar(payload,'rr_publish_reason') ?? radar.rr_publish_reason)})` : (getRadar(payload,'rr_publish_reason') ?? radar.rr_publish_reason)))
      ].join('');

      document.getElementById('rrFunnelPills').innerHTML = [
        pill('Anchor fresh', boolish(getRadar(payload,'rr_anchor_fresh')) ? 1 : 0),
        pill('Raw agree', boolish(getRadar(payload,'rr_raw_agree_ok')) ? 1 : 0),
        pill('Outlier persist', textOrDash(getRadar(payload,'rr_outlier_persist'))),
        pill('Recovery trigger', boolish(getRadar(payload,'rr_fundamental_recovery_triggered')) ? 'triggered' : 'no'),
        pill('Seed from raw', boolish(getRadar(payload,'rr_seed_from_raw_used')) ? 1 : 0),
        pill('Re-anchor allowed', boolish(getRadar(payload,'rr_midsession_raw_reanchor_allowed')) ? 1 : 0),
        pill('Re-anchor blocked', boolish(getRadar(payload,'rr_midsession_raw_reanchor_blocked')) ? 1 : 0),
        pill('Trust fresh', boolish(getRadar(payload,'trusted_rr_fresh')) ? 1 : 0)
      ].join('');

      document.getElementById('rrFunnelTable').innerHTML = [
        makeRow('Gate reason', textOrDash(radar.rr_gate_reason_name ? `${radar.rr_gate_reason_name} (${textOrDash(getRadar(payload,'rr_gate_reason') ?? radar.rr_gate_reason)})` : (getRadar(payload,'rr_gate_reason') ?? radar.rr_gate_reason))),
        makeRow('Publish reason', textOrDash(radar.rr_publish_reason_name ? `${radar.rr_publish_reason_name} (${textOrDash(getRadar(payload,'rr_publish_reason') ?? radar.rr_publish_reason)})` : (getRadar(payload,'rr_publish_reason') ?? radar.rr_publish_reason))),
        makeRow('RR ZC bpm/conf', `${fmt(getRadar(payload,'rr_zc_bpm'),2)} / ${fmt(getRadar(payload,'rr_zc_conf'),3)}`),
        makeRow('RR spec bpm/conf', `${fmt(getRadar(payload,'rr_spec_bpm'),2)} / ${fmt(getRadar(payload,'rr_spec_conf'),3)}`),
        makeRow('RR triple agree', textOrDash(getRadar(payload,'rr_triple_agree'))),
        makeRow('Gate PQI used', fmt(getRadar(payload,'rr_gate_pqi_used'),3)),
        makeRow('Candidate conf', fmt(getRadar(payload,'candidate_rr_conf'),3)),
        makeRow('Raw anchor err', fmt(getRadar(payload,'rr_raw_anchor_err_bpm'),2)),
        makeRow('Recovery count', textOrDash(getRadar(payload,'rr_fundamental_recovery_count'))),
        makeRow('Seed consistency', textOrDash(getRadar(payload,'rr_raw_seed_consistent_count'))),
        makeRow('Re-anchor reason', textOrDash(getRadar(payload,'rr_midsession_raw_reanchor_reason')))
      ].join('');

      document.getElementById('rrStageTable').innerHTML = [
        makeRow('Pre accept-phase', fmt(stageOrFallback(payload,'rr_pre_acceptphase',['candidate_rr']),2)),
        makeRow('Post accept-phase', fmt(stageOrFallback(payload,'rr_post_acceptphase',['candidate_rr']),2)),
        makeRow('Post blend', fmt(stageOrFallback(payload,'rr_post_blend',['reported_rr']),2)),
        makeRow('Post bias correction', fmt(stageOrFallback(payload,'rr_post_bias_correction',['rr_post_blend','reported_rr']),2)),
        makeRow('Post Kalman', rrStageValue(payload,'rr_post_kalman',['reported_rr']) === null ? '--' : fmt(rrStageValue(payload,'rr_post_kalman',['reported_rr']),2)),
        makeRow('Final publish candidate', fmt(stageOrFallback(payload,'rr_final_publish_candidate',['reported_rr']),2)),
        makeRow('Anchor value', fmt(getRadar(payload,'rr_anchor_value'),2)),
        makeRow('Anchor age (ms)', textOrDash(getRadar(payload,'rr_anchor_age_ms'))),
        makeRow('Anchor source', textOrDash(getRadar(payload,'rr_anchor_source'))),
        makeRow('Anchor confidence', fmt(getRadar(payload,'rr_anchor_confidence'),2)),
        makeRow('RR ZC bpm', fmt(getRadar(payload,'rr_zc_bpm'),2)),
        makeRow('RR spec bpm', fmt(getRadar(payload,'rr_spec_bpm'),2))
      ].join('');

      const rrFaultCards = [];
      const rrAnchorFresh = boolish(getRadar(payload,'rr_anchor_fresh'));
      const rrCandidateAge = num(getRadar(payload,'candidate_rr_age_ms'));
      const rrReanchorBlocked = boolish(getRadar(payload,'rr_midsession_raw_reanchor_blocked'));
      const rrRecoveryTriggered = boolish(getRadar(payload,'rr_fundamental_recovery_triggered'));
      const rrPublishReason = textOrDash(radar.rr_publish_reason_name || getRadar(payload,'rr_publish_reason'));
      if (!rrAnchorFresh) rrFaultCards.push(faultCard('warn','RR anchor stale','The RR anchor is not fresh. Recovery and publish behavior may now rely on stale state or fallback gating.'));
      if (rrCandidateAge !== null && rrCandidateAge > 10000) rrFaultCards.push(faultCard('warn','RR candidate aging','The RR candidate is older than 10 seconds. Treat it as historical unless fresh telemetry confirms it.'));
      if (rrReanchorBlocked) rrFaultCards.push(faultCard('warn','RR re-anchor blocked',`The current payload says mid-session RR re-anchor is blocked (${textOrDash(getRadar(payload,'rr_midsession_raw_reanchor_reason'))}).`));
      if (rrRecoveryTriggered) rrFaultCards.push(faultCard('good','RR recovery active','Fundamental recovery triggered on the RR path. Check anchor and final publish candidate to verify the recovery direction.'));
      if (rrPublishReason.includes('TRUST_STALE')) rrFaultCards.push(faultCard('bad','RR publish suppressed by stale trust','RR still has an internal candidate path, but publish is blocked by stale trust.'));
      if (!rrAnchorFresh && num(getRadar(payload,'rr_post_kalman')) !== null) rrFaultCards.push(faultCard('warn','RR stage frozen','Post-Kalman RR is hidden when the anchor is stale so the stage panel does not look live while publish is suppressed.'));
      if (hardStale) rrFaultCards.push(faultCard('bad','Dashboard payload expired','Live values are now blanked because telemetry age exceeded the stale timeout.'));
      document.getElementById('rrFaults').innerHTML = rrFaultCards.length ? rrFaultCards.join('') : faultCard('good','No RR-specific warning','No RR telemetry-specific warning was raised in the current payload.');

      document.getElementById('eventsLog').textContent = (payload.events || []).length ? payload.events.join('\n') : 'Waiting for events...';
      document.getElementById('footerText').innerHTML = `<span class="material-symbols-rounded" style="font-size:16px;">folder_open</span>Session folder: ${textOrDash(meta.session_dir)}`;
      updateCharts(payload);
    }
    async function poll(){
      try {
        const res = await fetch(`${state.dataUrl}?t=${Date.now()}`, { cache: 'no-store' });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const payload = normalize(await res.json());
        state.lastPayload = payload; state.disconnectedCount = 0;\n        try { render(payload); } catch (renderErr) { console.error('Dashboard render failed', renderErr); throw renderErr; }
      } catch (e) {
        console.error('Dashboard poll/render failed', e);
        state.disconnectedCount += 1;
        if (state.lastPayload) render(state.lastPayload);
      }
    }
    buildCharts();
    render(normalize({ meta: { status: 'waiting' }, events: ['Waiting for live_dashboard.json...'] }));
    poll();
    setInterval(poll, state.refreshMs);
    window.addEventListener('resize', ()=> state.lastPayload && drawRadar(state.lastPayload));
  </script>
</body>
</html>
'''

def _dashboard_template_candidates() -> List[Path]:
    base = Path(os.path.dirname(os.path.abspath(__file__)))
    cwd = Path(os.getcwd())
    script_name = Path(__file__).name
    dynamic_name = script_name.replace("trainer", "live_dashboard").replace(".py", ".html")
    names = [
        dynamic_name,
        "radar_vital_live_dashboard_v8_4_3_for_v13_8_0.html",
        "radar_vital_live_dashboard_v8_4_2_for_v13_8_0.html",
        "radar_vital_live_dashboard_v8_4_1_for_v13_8_0.html",
        "radar_vital_live_dashboard_v8_4_0_for_v13_8_0.html",
        "radar_vital_live_dashboard_v8_1_5_for_v13_5_2.html",
        "radar_vital_live_dashboard_v7_4_for_v12_5.html",
        "radar_vital_live_dashboard_m3e_v7_2_2_patched_for_v11_6_14.html",
        "radar_vital_live_dashboard_m3e_v7_2_2.html",
        "radar_vital_live_dashboard_m3e_v7_2_1.html",
        "radar_vital_live_dashboard_m3e.html",
    ]
    out: List[Path] = []
    seen = set()
    for root in (base, cwd):
        for name in names:
            p = (root / name).resolve()
            key = str(p).lower()
            if key in seen:
                continue
            seen.add(key)
            out.append(p)
    return out

for _dashboard_candidate in _dashboard_template_candidates():
    if _dashboard_candidate.exists():
        try:
            _DASHBOARD_TEMPLATE_EMBEDDED = _dashboard_candidate.read_text(encoding="utf-8")
            break
        except Exception:
            pass
_DASHBOARD_TEMPLATE_EMBEDDED = _DASHBOARD_TEMPLATE_EMBEDDED.replace("v13.8.0 / v8.4.0", "v13.9a / v8.5a")

def _load_dashboard_template_text() -> str:
    for path in _dashboard_template_candidates():
        if path.exists():
            try:
                return Path(path).read_text(encoding="utf-8")
            except Exception:
                pass
    return _DASHBOARD_TEMPLATE_EMBEDDED

class _SilentDashboardHandler(SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass


class _DashboardServer:
    def __init__(self, session_dir: str, port: int = 0):
        self.session_dir = os.path.abspath(session_dir)
        handler = partial(_SilentDashboardHandler, directory=self.session_dir)
        self.httpd = ThreadingHTTPServer(("127.0.0.1", int(port)), handler)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.httpd.server_port}/live_dashboard.html"

    def start(self):
        self.thread.start()

    def stop(self):
        try:
            self.httpd.shutdown()
        except Exception:
            pass
        try:
            self.httpd.server_close()
        except Exception:
            pass
        try:
            self.thread.join(timeout=1.5)
        except Exception:
            pass


def _row_get(row: Optional[Dict[str, str]], *keys: str):
    row = row or {}
    for key in keys:
        if key in row and row.get(key) not in (None, ""):
            return row.get(key)
    return None


def _firmware_contract_candidates() -> List[Path]:
    roots = [
        Path(os.path.dirname(os.path.abspath(__file__))),
        Path(os.getcwd()),
    ]
    relatives = [
        Path("radar_vital_v13_8_0.ino"),
        Path("radar_vital_v13_7_5.ino"),
        Path("radar_vital_v13_7_4.ino"),
        Path("radar_vital_v13_7_3.ino"),
    ]
    out: List[Path] = []
    seen = set()
    for root in roots:
        for rel in relatives:
            path = (root / rel).resolve()
            key = str(path).lower()
            if key in seen:
                continue
            seen.add(key)
            out.append(path)
    return out


def _extract_firmware_data_header_tokens(ino_path: Path) -> List[str]:
    text = ino_path.read_text(encoding="utf-8", errors="ignore")
    for match in re.finditer(r'Serial\.println\(\s*((?:"[^"]*"\s*)+)\);', text, flags=re.S):
        literals = re.findall(r'"([^"]*)"', match.group(1))
        header = "".join(literals)
        if header.startswith("DATA,"):
            return header.split(",")[1:]
    raise RuntimeError(f"Could not locate DATA header in firmware file: {ino_path}")


def _assert_radar_log_contract() -> Path:
    global _FIRMWARE_CONTRACT_CACHE
    expected = list(RADAR_LOG_COLUMNS)
    if len(expected) != EXPECTED_RADAR_LOG_COLUMN_COUNT:
        raise RuntimeError(
            f"RADAR_LOG_COLUMNS contract drift: expected {EXPECTED_RADAR_LOG_COLUMN_COUNT} columns, got {len(expected)}"
        )
    if tuple(expected[-len(EXPECTED_RADAR_LOG_TAIL):]) != EXPECTED_RADAR_LOG_TAIL:
        raise RuntimeError(
            f"RADAR_LOG_COLUMNS tail drift: expected {EXPECTED_RADAR_LOG_TAIL}, got {tuple(expected[-len(EXPECTED_RADAR_LOG_TAIL):])}"
        )
    if _FIRMWARE_CONTRACT_CACHE is not None and list(_FIRMWARE_CONTRACT_CACHE[1]) == expected:
        return Path(_FIRMWARE_CONTRACT_CACHE[0])

    searched = []
    mismatches = []
    for path in _firmware_contract_candidates():
        searched.append(str(path))
        if not path.exists():
            continue
        actual = _extract_firmware_data_header_tokens(path)
        if len(actual) == LEGACY_RADAR_LOG_COLUMN_COUNT and tuple(actual[-len(LEGACY_RADAR_LOG_TAIL):]) == LEGACY_RADAR_LOG_TAIL:
            continue
        if actual == expected:
            _FIRMWARE_CONTRACT_CACHE = (str(path), tuple(actual))
            return path
        mismatch_idx = next(
            (i for i, (trainer_col, fw_col) in enumerate(zip(expected, actual)) if trainer_col != fw_col),
            min(len(expected), len(actual)),
        )
        trainer_col = expected[mismatch_idx] if mismatch_idx < len(expected) else "<end>"
        fw_col = actual[mismatch_idx] if mismatch_idx < len(actual) else "<end>"
        mismatches.append((path, mismatch_idx, trainer_col, fw_col, len(actual)))

    if mismatches:
        path, mismatch_idx, trainer_col, fw_col, actual_len = mismatches[0]
        raise RuntimeError(
            "Firmware DATA header mismatch: "
            f"file={path}, index={mismatch_idx}, trainer={trainer_col!r}, firmware={fw_col!r}, "
            f"trainer_count={len(expected)}, firmware_count={actual_len}. "
            f"Searched: {', '.join(searched)}"
        )

    raise RuntimeError(
        "Could not locate firmware DATA header source for contract verification. "
        f"Tried: {', '.join(searched)}"
    )

def _heart_pqi_gate(distance_cm) -> float:
    dist = _to_num(distance_cm, default=float("nan"))
    if np.isfinite(dist) and dist > 1.0:
        if dist <= 40.0:
            return HEART_PQI_GATE_NEAR
        if dist <= 80.0:
            return HEART_PQI_GATE_MID
    return HEART_PQI_GATE_FAR


def _resolve_live_distance_cm(display_distance_cm: float, radar_row: Optional[Dict[str, str]]) -> float:
    for candidate in (
        display_distance_cm,
        _to_num(_row_get(radar_row, "reported_distance_cm"), default=float("nan")),
        _to_num(_row_get(radar_row, "reflector_distance_cm"), default=float("nan")),
    ):
        if np.isfinite(candidate) and candidate > 0.0:
            return float(candidate)
    return float("nan")


def _hr_path_source_name(value) -> str:
    numeric = _to_num(value, default=float("nan"))
    if not np.isfinite(numeric):
        return ""
    return HR_PATH_SOURCE_NAMES.get(int(round(numeric)), f"Source {int(round(numeric))}")


def _harmonic_mode_labels(value) -> List[str]:
    numeric = _to_num(value, default=float("nan"))
    if not np.isfinite(numeric):
        return []
    mode = int(round(numeric))
    return [label for bit, label in HARMONIC_MODE_BIT_LABELS.items() if mode & bit]




def _parse_radar_data_line(line: str, cols: Sequence[str]) -> Tuple[str, Optional[List[str]], str]:
    if not line.startswith("DATA,"):
        return "skip", None, ""
    payload = line.split(",")[1:]
    if payload == list(cols) or (payload and payload[0] == "timestamp_ms"):
        return "header", None, ""
    accepted_legacy_counts = (LEGACY_RADAR_LOG_COLUMN_COUNT, 136)
    if len(payload) not in ((len(cols),) + accepted_legacy_counts):
        return "reject", None, f"{len(payload)} fields, expected {accepted_legacy_counts} or {len(cols)}"
    if len(payload) in accepted_legacy_counts:
        payload = payload + [""] * (EXPECTED_RADAR_LOG_COLUMN_COUNT - len(payload))
    try:
        float(payload[0])
    except Exception:
        return "reject", None, "non-numeric timestamp_ms field"
    return "data", payload, ""

def _format_dashboard_event(prefix: str, line: str) -> str:
    line = re.sub(r"\x1b\[[0-9;]*m", "", str(line)).strip()
    if len(line) > 220:
        line = line[:217] + "..."
    return f"{prefix} {line}"


def _coerce_numeric_series(df: pd.DataFrame, *names: str) -> pd.Series:
    for name in names:
        if name in df.columns:
            return pd.to_numeric(df[name], errors="coerce")
    return pd.Series(np.nan, index=df.index, dtype=float)


def _coerce_boolish_series(df: pd.DataFrame, *names: str) -> pd.Series:
    s = _coerce_numeric_series(df, *names)
    return s.fillna(0.0).astype(float)


def _coerce_text_series(df: pd.DataFrame, *names: str) -> pd.Series:
    for name in names:
        if name in df.columns:
            return df[name].astype(str).replace({"nan": "", "None": ""}).fillna("")
    return pd.Series([""] * len(df), index=df.index, dtype=object)


def _decode_enum_series(df: pd.DataFrame, name: str, mapping: Dict[int, str]) -> pd.Series:
    if name not in df.columns:
        return pd.Series([""] * len(df), index=df.index, dtype=object)
    s = pd.to_numeric(df[name], errors="coerce")
    return s.map(lambda v: mapping.get(int(v), str(int(v))) if pd.notna(v) else "").astype(object)


def _prefer_display_series(df: pd.DataFrame, primary: str, *fallbacks: str, require_positive: bool = True) -> pd.Series:
    out = _coerce_numeric_series(df, primary).copy()
    mask = out.isna() | ((out <= 0) if require_positive else False)
    for name in fallbacks:
        alt = _coerce_numeric_series(df, name)
        if require_positive:
            repl = mask & alt.notna() & (alt > 0)
        else:
            repl = mask & alt.notna()
        out.loc[repl] = alt.loc[repl]
        mask = out.isna() | ((out <= 0) if require_positive else False)
    return out


def _last_positive_or_nan(values: pd.Series) -> float:
    s = pd.to_numeric(values, errors="coerce")
    s = s[s.notna() & (s > 0)]
    return float(s.iloc[-1]) if len(s) else float("nan")


def _tracker_dataframe(tracker: _CSVAppendTracker) -> pd.DataFrame:
    if not tracker.recent_rows:
        return pd.DataFrame()
    return pd.DataFrame(list(tracker.recent_rows))


def _series_from_trackers(radar_tracker: _CSVAppendTracker, ref_tracker: _CSVAppendTracker, window_s: float = 180.0) -> Dict[str, List[object]]:
    rad_df = _tracker_dataframe(radar_tracker)
    ref_df = _tracker_dataframe(ref_tracker)
    empty = {
        "t": [],
        "radar_hr": [], "reported_hr": [], "candidate_hr": [], "raw_hr": [], "radar_hr_valid": [],
        "radar_rr": [], "reported_rr": [], "candidate_rr": [], "raw_rr": [], "radar_rr_valid": [],
        "candidate_hr_conf": [], "candidate_rr_conf": [],
        "ble_hr": [], "ble_rr": [],
        "pqi_heart": [], "pqi_breath": [],
        "heart_phase": [], "breath_phase": [],
        "hr_zc_bpm": [], "hr_zc_conf": [], "hr_spec_bpm": [], "hr_spec_mag": [], "hr_triple_agree": [],
        "rr_zc_bpm": [], "rr_zc_conf": [], "rr_spec_bpm": [], "rr_spec_conf": [], "rr_triple_agree": [],
        "distance_cm": [], "motion": [], "human": [], "radar_is_present_raw": [],
        "hr_gate_reason": [], "rr_gate_reason": [],
        "hr_gate_pqi_used": [], "rr_gate_pqi_used": [],
        "hr_publish_reason": [], "rr_publish_reason": [], "skipdsp_misses": [],
        "hr_arbiter_corrected": [], "hr_rejectphase_rejected": [], "hr_coherence_rejected": [],
        "hr_raw_source": [], "hr_raw_agree": [], "hr_agree_err_bpm": [],
        "hr_bypass_pqi_ok": [], "hr_bypass_conf_ok": [], "hr_bypass_gate_ok": [], "hr_bypass_active": [],
        "hr_grace_eligible": [], "hr_grace_active": [], "hr_trust_fresh": [],
        "rr_anchor_fresh": [], "rr_outlier_persist": [], "rr_raw_agree_ok": [],
        "rr_pre_acceptphase": [], "rr_post_acceptphase": [], "rr_post_blend": [], "rr_post_bias_correction": [], "rr_post_kalman": [], "rr_final_publish_candidate": [],
        "rr_anchor_value": [], "rr_anchor_age_ms": [], "rr_anchor_source": [], "rr_anchor_confidence": [],
        "rr_fundamental_recovery_count": [], "rr_fundamental_recovery_triggered": [], "rr_raw_seed_consistent_count": [],
        "rr_midsession_raw_reanchor_allowed": [], "rr_midsession_raw_reanchor_blocked": [], "rr_midsession_raw_reanchor_reason": [], "rr_raw_anchor_err_bpm": [],
        "hr_pre_rejectphase": [], "hr_post_rejectphase": [], "hr_post_blend": [], "hr_post_coherence": [],
        "hr_final_publish_candidate": [],
        "hr_arbiter_anchor_used": [], "hr_arbiter_anchor_value": [],
        "hr_rejectphase_anchor_used": [], "hr_rejectphase_anchor_value": [],
        "hr_raw_looks_like_half_rate": [], "hr_trusted_anchor_value": [],
        "hr_age_ms": [], "candidate_hr_age_ms": [], "candidate_rr_age_ms": [],
        "hr_updated_this_cycle": [], "hr_update_source": [], "latched_raw_hr": [], "trusted_rr_fresh": [],
        "hr_band_min": [], "hr_band_max": [], "radar_gain": [],
        "point_cloud_ok": [], "target_info_ok": [], "num_targets": [], "max_dop_abs": [], "max_dop_speed_cms": [],
        "doppler_motion": [], "cluster_anomaly": [], "multi_target": [],
        "primary_x": [], "primary_y": [], "primary_dop": [], "primary_dop_speed_cms": [], "primary_cluster": [],
        "spatial_source": [], "spatial_age_ms": [], "position_radius_cm": [],
        "phase_warmup_complete": [], "clutter_warmup_count": [], "current_clutter_alpha": [], "phase_valid_this_frame": [], "dsp_ran_this_frame": [], "hr_confidence_source": [],
        "use_fast_path": [], "hr_path_source": [], "module_fw_major": [], "module_fw_sub": [], "module_fw_mod": [], "sketch_major": [], "sketch_sub": [], "sketch_mod": [],
        "hr_trusted_phase_anchor": [], "hr_anchor_source": [], "hr_anchor_err_bpm": [], "hr_raw_high_bias_suspect": [],
        "rr_seed_from_raw_used": [],
    }
    if rad_df.empty:
        return empty

    rad_ts_ms = _coerce_numeric_series(rad_df, "timestamp_ms")
    if rad_ts_ms.notna().any():
        rad_df = rad_df.loc[rad_ts_ms.notna()].copy()
        rad_df["_ts_s"] = rad_ts_ms.loc[rad_df.index] / 1000.0
    else:
        rad_ts_s = _coerce_numeric_series(rad_df, "timestamp_s")
        rad_df = rad_df.loc[rad_ts_s.notna()].copy()
        rad_df["_ts_s"] = rad_ts_s.loc[rad_df.index]
    if rad_df.empty:
        return empty

    max_t = float(rad_df["_ts_s"].max())
    rad_df = rad_df[rad_df["_ts_s"] >= (max_t - float(window_s))].copy()
    rad_df = rad_df.sort_values("_ts_s").reset_index(drop=True)

    if not ref_df.empty:
        ref_ts_ms = _coerce_numeric_series(ref_df, "timestamp_ms")
        if ref_ts_ms.notna().any():
            ref_df = ref_df.loc[ref_ts_ms.notna()].copy()
            ref_df["_ts_s"] = ref_ts_ms.loc[ref_df.index] / 1000.0
        else:
            ref_ts_s = _coerce_numeric_series(ref_df, "timestamp_s")
            ref_df = ref_df.loc[ref_ts_s.notna()].copy()
            ref_df["_ts_s"] = ref_ts_s.loc[ref_df.index]
        ref_df = ref_df[ref_df["_ts_s"] >= (max_t - float(window_s) - 5.0)].copy()
        ref_df = ref_df.sort_values("_ts_s").reset_index(drop=True)

    radar_hr_display = _coerce_numeric_series(rad_df, "reported_hr")
    radar_rr_display = _coerce_numeric_series(rad_df, "reported_rr")
    distance_display = _prefer_display_series(rad_df, "reported_distance_cm", "reflector_distance_cm", require_positive=True)

    out = {
        "t": [str(int(round(v))) for v in rad_df["_ts_s"].tolist()],
        "radar_hr": radar_hr_display.round(3).tolist(),
        "reported_hr": _coerce_numeric_series(rad_df, "reported_hr").round(3).tolist(),
        "candidate_hr": _coerce_numeric_series(rad_df, "candidate_hr").round(3).tolist(),
        "raw_hr": _coerce_numeric_series(rad_df, "raw_hr").round(3).tolist(),
        "latched_raw_hr": _coerce_numeric_series(rad_df, "latched_raw_hr").round(3).tolist(),
        "radar_hr_valid": _coerce_boolish_series(rad_df, "logged_hr_valid").clip(0, 1).tolist(),
        "radar_rr": radar_rr_display.round(3).tolist(),
        "reported_rr": _coerce_numeric_series(rad_df, "reported_rr").round(3).tolist(),
        "candidate_rr": _coerce_numeric_series(rad_df, "candidate_rr").round(3).tolist(),
        "raw_rr": _coerce_numeric_series(rad_df, "raw_rr").round(3).tolist(),
        "radar_rr_valid": _coerce_boolish_series(rad_df, "logged_rr_valid").clip(0, 1).tolist(),
        "candidate_hr_conf": _coerce_numeric_series(rad_df, "candidate_hr_conf").round(4).tolist(),
        "candidate_rr_conf": _coerce_numeric_series(rad_df, "candidate_rr_conf").round(4).tolist(),
        "pqi_heart": _coerce_numeric_series(rad_df, "pqi_heart").round(4).tolist(),
        "pqi_breath": _coerce_numeric_series(rad_df, "pqi_breath").round(4).tolist(),
        "heart_phase": _coerce_numeric_series(rad_df, "heart_phase_stabilized", "heart_phase").round(6).tolist(),
        "breath_phase": _coerce_numeric_series(rad_df, "breath_phase_stabilized", "breath_phase").round(6).tolist(),
        "distance_cm": distance_display.round(3).tolist(),
        "motion": _coerce_boolish_series(rad_df, "in_motion").clip(0, 1).tolist(),
        "human": _coerce_boolish_series(rad_df, "human_detected", "humanDetected").clip(0, 1).tolist(),
        "radar_is_present_raw": _coerce_boolish_series(rad_df, "radar_is_present_raw").clip(0, 1).tolist(),
        "hr_gate_reason": _decode_enum_series(rad_df, "hr_gate_reason", HR_GATE_REASON_NAMES).tolist(),
        "rr_gate_reason": _decode_enum_series(rad_df, "rr_gate_reason", RR_GATE_REASON_NAMES).tolist(),
        "hr_gate_pqi_used": _coerce_numeric_series(rad_df, "hr_gate_pqi_used").round(4).tolist(),
        "rr_gate_pqi_used": _coerce_numeric_series(rad_df, "rr_gate_pqi_used").round(4).tolist(),
        "hr_publish_reason": _coerce_numeric_series(rad_df, "hr_publish_reason").round(3).tolist(),
        "rr_publish_reason": _coerce_numeric_series(rad_df, "rr_publish_reason").round(3).tolist(),
        "hr_publish_reason_name": _decode_enum_series(rad_df, "hr_publish_reason", HR_PUBLISH_REASON_NAMES).tolist(),
        "rr_publish_reason_name": _decode_enum_series(rad_df, "rr_publish_reason", RR_PUBLISH_REASON_NAMES).tolist(),
        "skipdsp_misses": _coerce_numeric_series(rad_df, "skipdsp_misses").round(3).tolist(),
        "hr_arbiter_corrected": _coerce_boolish_series(rad_df, "hr_arbiter_corrected").clip(0, 1).tolist(),
        "hr_rejectphase_rejected": _coerce_boolish_series(rad_df, "hr_rejectphase_rejected").clip(0, 1).tolist(),
        "hr_coherence_rejected": _coerce_boolish_series(rad_df, "hr_coherence_rejected").clip(0, 1).tolist(),
        "hr_raw_source": _coerce_numeric_series(rad_df, "hr_raw_source").round(3).tolist(),
        "hr_raw_agree": _coerce_boolish_series(rad_df, "hr_raw_agree").clip(0, 1).tolist(),
        "hr_agree_err_bpm": _coerce_numeric_series(rad_df, "hr_agree_err_bpm").round(4).tolist(),
        "hr_bypass_pqi_ok": _coerce_boolish_series(rad_df, "hr_bypass_pqi_ok").clip(0, 1).tolist(),
        "hr_bypass_conf_ok": _coerce_boolish_series(rad_df, "hr_bypass_conf_ok").clip(0, 1).tolist(),
        "hr_bypass_gate_ok": _coerce_boolish_series(rad_df, "hr_bypass_gate_ok").clip(0, 1).tolist(),
        "hr_bypass_active": _coerce_boolish_series(rad_df, "hr_bypass_active").clip(0, 1).tolist(),
        "hr_grace_eligible": _coerce_boolish_series(rad_df, "hr_grace_eligible").clip(0, 1).tolist(),
        "hr_grace_active": _coerce_boolish_series(rad_df, "hr_grace_active").clip(0, 1).tolist(),
        "hr_trust_fresh": _coerce_boolish_series(rad_df, "hr_trust_fresh").clip(0, 1).tolist(),
        "rr_anchor_fresh": _coerce_boolish_series(rad_df, "rr_anchor_fresh").clip(0, 1).tolist(),
        "rr_outlier_persist": _coerce_numeric_series(rad_df, "rr_outlier_persist").round(3).tolist(),
        "rr_raw_agree_ok": _coerce_boolish_series(rad_df, "rr_raw_agree_ok").clip(0, 1).tolist(),
        "rr_pre_acceptphase": _coerce_numeric_series(rad_df, "rr_pre_acceptphase").round(4).tolist(),
        "rr_post_acceptphase": _coerce_numeric_series(rad_df, "rr_post_acceptphase").round(4).tolist(),
        "rr_post_blend": _coerce_numeric_series(rad_df, "rr_post_blend").round(4).tolist(),
        "rr_post_bias_correction": _coerce_numeric_series(rad_df, "rr_post_bias_correction").round(4).tolist(),
        "rr_post_kalman": _coerce_numeric_series(rad_df, "rr_post_kalman").round(4).tolist(),
        "rr_final_publish_candidate": _coerce_numeric_series(rad_df, "rr_final_publish_candidate").round(4).tolist(),
        "rr_anchor_value": _coerce_numeric_series(rad_df, "rr_anchor_value").round(4).tolist(),
        "rr_anchor_age_ms": _coerce_numeric_series(rad_df, "rr_anchor_age_ms").round(3).tolist(),
        "rr_anchor_source": _coerce_numeric_series(rad_df, "rr_anchor_source").round(3).tolist(),
        "rr_anchor_confidence": _coerce_numeric_series(rad_df, "rr_anchor_confidence").round(4).tolist(),
        "rr_fundamental_recovery_count": _coerce_numeric_series(rad_df, "rr_fundamental_recovery_count").round(3).tolist(),
        "rr_fundamental_recovery_triggered": _coerce_boolish_series(rad_df, "rr_fundamental_recovery_triggered").clip(0, 1).tolist(),
        "rr_raw_seed_consistent_count": _coerce_numeric_series(rad_df, "rr_raw_seed_consistent_count").round(3).tolist(),
        "rr_midsession_raw_reanchor_allowed": _coerce_boolish_series(rad_df, "rr_midsession_raw_reanchor_allowed").clip(0, 1).tolist(),
        "rr_midsession_raw_reanchor_blocked": _coerce_boolish_series(rad_df, "rr_midsession_raw_reanchor_blocked").clip(0, 1).tolist(),
        "rr_midsession_raw_reanchor_reason": _coerce_numeric_series(rad_df, "rr_midsession_raw_reanchor_reason").round(3).tolist(),
        "rr_raw_anchor_err_bpm": _coerce_numeric_series(rad_df, "rr_raw_anchor_err_bpm").round(4).tolist(),
        "hr_pre_rejectphase": _coerce_numeric_series(rad_df, "hr_pre_rejectphase").round(4).tolist(),
        "hr_post_rejectphase": _coerce_numeric_series(rad_df, "hr_post_rejectphase").round(4).tolist(),
        "hr_post_blend": _coerce_numeric_series(rad_df, "hr_post_blend").round(4).tolist(),
        "hr_post_coherence": _coerce_numeric_series(rad_df, "hr_post_coherence").round(4).tolist(),
        "hr_final_publish_candidate": _coerce_numeric_series(rad_df, "hr_final_publish_candidate").round(4).tolist(),
        "hr_arbiter_anchor_used": _coerce_boolish_series(rad_df, "hr_arbiter_anchor_used").clip(0, 1).tolist(),
        "hr_arbiter_anchor_value": _coerce_numeric_series(rad_df, "hr_arbiter_anchor_value").round(4).tolist(),
        "hr_rejectphase_anchor_used": _coerce_boolish_series(rad_df, "hr_rejectphase_anchor_used").clip(0, 1).tolist(),
        "hr_rejectphase_anchor_value": _coerce_numeric_series(rad_df, "hr_rejectphase_anchor_value").round(4).tolist(),
        "hr_raw_looks_like_half_rate": _coerce_boolish_series(rad_df, "hr_raw_looks_like_half_rate").clip(0, 1).tolist(),
        "hr_trusted_anchor_value": _coerce_numeric_series(rad_df, "hr_trusted_anchor_value").round(4).tolist(),
        "hr_age_ms": _coerce_numeric_series(rad_df, "hr_age_ms").round(3).tolist(),
        "candidate_hr_age_ms": _coerce_numeric_series(rad_df, "candidate_hr_age_ms").round(3).tolist(),
        "candidate_rr_age_ms": _coerce_numeric_series(rad_df, "candidate_rr_age_ms").round(3).tolist(),
        "hr_updated_this_cycle": _coerce_boolish_series(rad_df, "hr_updated_this_cycle").clip(0, 1).tolist(),
        "hr_update_source": _coerce_numeric_series(rad_df, "hr_update_source").round(3).tolist(),
        "trusted_rr_fresh": _coerce_boolish_series(rad_df, "trusted_rr_fresh").clip(0, 1).tolist(),
        "hr_band_min": _coerce_numeric_series(rad_df, "hr_band_min").round(4).tolist(),
        "hr_band_max": _coerce_numeric_series(rad_df, "hr_band_max").round(4).tolist(),
        "radar_gain": _coerce_numeric_series(rad_df, "radar_gain").round(4).tolist(),
        "hr_zc_bpm": _coerce_numeric_series(rad_df, "hr_zc_bpm").round(4).tolist(),
        "hr_zc_conf": _coerce_numeric_series(rad_df, "hr_zc_conf").round(4).tolist(),
        "hr_spec_bpm": _coerce_numeric_series(rad_df, "hr_spec_bpm").round(4).tolist(),
        "hr_spec_mag": _coerce_numeric_series(rad_df, "hr_spec_mag").round(6).tolist(),
        "hr_triple_agree": _coerce_boolish_series(rad_df, "hr_triple_agree").clip(0, 1).tolist(),
        "rr_zc_bpm": _coerce_numeric_series(rad_df, "rr_zc_bpm").round(4).tolist(),
        "rr_zc_conf": _coerce_numeric_series(rad_df, "rr_zc_conf").round(4).tolist(),
        "rr_spec_bpm": _coerce_numeric_series(rad_df, "rr_spec_bpm").round(4).tolist(),
        "rr_spec_conf": _coerce_numeric_series(rad_df, "rr_spec_conf").round(4).tolist(),
        "rr_triple_agree": _coerce_boolish_series(rad_df, "rr_triple_agree").clip(0, 1).tolist(),
        "point_cloud_ok": _coerce_boolish_series(rad_df, "point_cloud_ok").clip(0, 1).tolist(),
        "target_info_ok": _coerce_boolish_series(rad_df, "target_info_ok").clip(0, 1).tolist(),
        "num_targets": _coerce_numeric_series(rad_df, "num_targets").round(3).tolist(),
        "max_dop_abs": _coerce_numeric_series(rad_df, "max_dop_abs").round(4).tolist(),
        "max_dop_speed_cms": _coerce_numeric_series(rad_df, "max_dop_speed_cms").round(4).tolist(),
        "doppler_motion": _coerce_boolish_series(rad_df, "doppler_motion").clip(0, 1).tolist(),
        "cluster_anomaly": _coerce_boolish_series(rad_df, "cluster_anomaly").clip(0, 1).tolist(),
        "multi_target": _coerce_boolish_series(rad_df, "multi_target").clip(0, 1).tolist(),
        "primary_x": _coerce_numeric_series(rad_df, "primary_x").round(4).tolist(),
        "primary_y": _coerce_numeric_series(rad_df, "primary_y").round(4).tolist(),
        "primary_dop": _coerce_numeric_series(rad_df, "primary_dop").round(4).tolist(),
        "primary_dop_speed_cms": _coerce_numeric_series(rad_df, "primary_dop_speed_cms").round(4).tolist(),
        "primary_cluster": _coerce_numeric_series(rad_df, "primary_cluster").round(4).tolist(),
        "spatial_source": _coerce_numeric_series(rad_df, "spatial_source").round(3).tolist(),
        "spatial_age_ms": _coerce_numeric_series(rad_df, "spatial_age_ms").round(3).tolist(),
        "position_radius_cm": _coerce_numeric_series(rad_df, "position_radius_cm").round(3).tolist(),
        "phase_warmup_complete": _coerce_boolish_series(rad_df, "phase_warmup_complete").clip(0, 1).tolist(),
        "clutter_warmup_count": _coerce_numeric_series(rad_df, "clutter_warmup_count").round(3).tolist(),
        "current_clutter_alpha": _coerce_numeric_series(rad_df, "current_clutter_alpha").round(6).tolist(),
        "phase_valid_this_frame": _coerce_boolish_series(rad_df, "phase_valid_this_frame").clip(0, 1).tolist(),
        "use_fast_path": _coerce_boolish_series(rad_df, "use_fast_path").clip(0, 1).tolist(),
        "hr_path_source": _coerce_numeric_series(rad_df, "hr_path_source").round(4).tolist(),
        "module_fw_major": _coerce_numeric_series(rad_df, "module_fw_major", "fw_major").round(3).tolist(),
        "module_fw_sub": _coerce_numeric_series(rad_df, "module_fw_sub", "fw_sub").round(3).tolist(),
        "module_fw_mod": _coerce_numeric_series(rad_df, "module_fw_mod", "fw_mod").round(3).tolist(),
        "sketch_major": _coerce_numeric_series(rad_df, "sketch_major").round(3).tolist(),
        "sketch_sub": _coerce_numeric_series(rad_df, "sketch_sub").round(3).tolist(),
        "sketch_mod": _coerce_numeric_series(rad_df, "sketch_mod").round(3).tolist(),
        "fw_major": _coerce_numeric_series(rad_df, "module_fw_major", "fw_major").round(3).tolist(),
        "fw_sub": _coerce_numeric_series(rad_df, "module_fw_sub", "fw_sub").round(3).tolist(),
        "fw_mod": _coerce_numeric_series(rad_df, "module_fw_mod", "fw_mod").round(3).tolist(),
        "hr_trusted_phase_anchor": _coerce_numeric_series(rad_df, "hr_trusted_phase_anchor").round(3).tolist(),
        "hr_anchor_source": _coerce_numeric_series(rad_df, "hr_anchor_source").round(3).tolist(),
        "hr_anchor_err_bpm": _coerce_numeric_series(rad_df, "hr_anchor_err_bpm").round(3).tolist(),
        "hr_raw_high_bias_suspect": _coerce_boolish_series(rad_df, "hr_raw_high_bias_suspect").clip(0, 1).tolist(),
        "rr_seed_from_raw_used": _coerce_boolish_series(rad_df, "rr_seed_from_raw_used").clip(0, 1).tolist(),
        "ble_hr": [],
        "ble_rr": [],
    }

    if ref_df.empty:
        out["ble_hr"] = [None] * len(rad_df)
        out["ble_rr"] = [None] * len(rad_df)
        return out

    ref_small = pd.DataFrame({
        "_ts_s": ref_df["_ts_s"],
        "ref_hr": _coerce_numeric_series(ref_df, "ref_hr"),
        "ref_rr": _coerce_numeric_series(ref_df, "ref_rr"),
    }).sort_values("_ts_s")
    joined = pd.merge_asof(
        rad_df[["_ts_s"]].sort_values("_ts_s"),
        ref_small,
        on="_ts_s",
        direction="nearest",
        tolerance=1.25,
    )
    out["ble_hr"] = pd.to_numeric(joined["ref_hr"], errors="coerce").round(3).tolist()
    out["ble_rr"] = pd.to_numeric(joined["ref_rr"], errors="coerce").round(3).tolist()
    return out


def _age_from_tracker(tracker: _CSVAppendTracker, now_elapsed_s: float) -> Optional[float]:
    row = tracker.last_row or {}
    t_ms = _to_num(_row_get(row, "timestamp_ms"), default=float("nan"))
    if np.isfinite(t_ms):
        return max(0.0, float(now_elapsed_s) - float(t_ms) / 1000.0)
    t_s = _to_num(_row_get(row, "timestamp_s"), default=float("nan"))
    if np.isfinite(t_s):
        return max(0.0, float(now_elapsed_s) - float(t_s))
    return None


def _last_valid_age_s(tracker: _CSVAppendTracker, valid_keys: Sequence[str], ts_key: str = "timestamp_ms") -> Optional[float]:
    if not tracker.recent_rows:
        return None
    latest_ts = None
    last_valid_ts = None
    for row in tracker.recent_rows:
        t = _to_num(_row_get(row, ts_key, "timestamp_s"), default=float("nan"))
        if np.isfinite(t):
            latest_ts = t
        valid_val = _to_num(_row_get(row, *valid_keys), default=float("nan"))
        if np.isfinite(valid_val) and valid_val >= 0.5 and np.isfinite(t):
            last_valid_ts = t
    if latest_ts is None or last_valid_ts is None:
        return None
    if ts_key == "timestamp_ms" or latest_ts > 1000.0:
        return max(0.0, (latest_ts - last_valid_ts) / 1000.0)
    return max(0.0, latest_ts - last_valid_ts)


def _recent_rate_hz(tracker: _CSVAppendTracker, ts_key: str = "timestamp_ms", max_points: int = 32) -> Optional[float]:
    rows = list(tracker.recent_rows)[-max_points:] if tracker.recent_rows else []
    ts: List[float] = []
    for row in rows:
        t = _to_num(_row_get(row, ts_key, "timestamp_s"), default=float("nan"))
        if np.isfinite(t):
            ts.append(float(t))
    if len(ts) < 3:
        return None
    diffs = np.diff(np.asarray(ts, dtype=float))
    diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
    if diffs.size == 0:
        return None
    med = float(np.median(diffs))
    if med <= 0:
        return None
    return 1000.0 / med if ts_key == "timestamp_ms" or max(ts) > 1000.0 else 1.0 / med


def _build_dashboard_faults(radar: Dict[str, object], ble: Dict[str, object], thresholds: Dict[str, float]) -> List[Dict[str, str]]:
    faults: List[Dict[str, str]] = []
    hr = radar.get("hr")
    rr = radar.get("rr")
    ref_hr = ble.get("hr")
    ref_rr = ble.get("rr")
    pqi_h = radar.get("pqi_heart")
    pqi_b = radar.get("pqi_breath")
    hr_gate_reason = str(radar.get("hr_gate_reason_name") or radar.get("hr_gate_reason") or "").strip()
    rr_gate_reason = str(radar.get("rr_gate_reason_name") or radar.get("rr_gate_reason") or "").strip()
    hr_publish_reason = str(radar.get("hr_publish_reason_name") or radar.get("hr_publish_reason") or "").strip()
    rr_publish_reason = str(radar.get("rr_publish_reason_name") or radar.get("rr_publish_reason") or "").strip()

    pqi_h_val = _to_num(pqi_h)
    pqi_b_val = _to_num(pqi_b)
    if np.isfinite(pqi_h_val) and pqi_h_val < 0.20:
        faults.append({"severity": "warn", "title": "PQI heart low", "copy": "Heart PQI is below 0.20, so HR may be held or invalid even if a number is visible."})
    if np.isfinite(pqi_b_val) and pqi_b_val < 0.20:
        faults.append({"severity": "warn", "title": "PQI breath low", "copy": "Breath PQI is below 0.20. Treat RR as suspect until the signal settles."})
    if radar.get("motion") == 1:
        faults.append({"severity": "bad", "title": "Motion detected", "copy": "Macro-motion is active. Hold still before trusting HR or RR."})
    if radar.get("human") == 0:
        faults.append({"severity": "bad", "title": "No human lock", "copy": "Presence is absent right now, so vitals are either stale or unavailable."})
    if radar.get("human") == 1 and (ble.get("rows") in (0, None)) and (np.isfinite(float(radar.get("candidate_hr", float("nan")))) or np.isfinite(float(radar.get("candidate_rr", float("nan"))))):
        faults.append({"severity": "bad", "title": "Ghost target suspected", "copy": "No BLE/reference stream is present, but the radar still reports human presence and internal candidates. Review empty-room false positive behavior."})
    if radar.get("multi_target") == 1:
        faults.append({'severity': 'bad', 'title': 'Multi-target contamination', 'copy': f"Point-cloud / target-info reports {int(radar.get('num_targets') or 0)} targets. Host DSP should treat this window as contaminated."})
    if radar.get("cluster_anomaly") == 1:
        faults.append({"severity": "warn", "title": "Cluster anomaly", "copy": "Point-cloud clusters disagree with the single-target assumption. Review multipath / ghost behavior."})
    if radar.get("doppler_motion") == 1 and radar.get("motion") == 0:
        faults.append({"severity": "warn", "title": "Doppler-only motion", "copy": "Point-cloud Doppler shows motion even though phase-energy motion has not fully opened yet."})

    if radar.get("logged_hr_valid") != 1 and np.isfinite(float(radar.get("candidate_hr", float('nan')))):
        faults.append({
            "severity": "warn",
            "title": "HR not published",
            "copy": f"A candidate HR exists but it was not published. Gate reason: {hr_gate_reason or 'unknown'}; publish reason: {hr_publish_reason or 'unknown'}."
        })
    if radar.get("logged_rr_valid") != 1 and np.isfinite(float(radar.get("candidate_rr", float('nan')))):
        faults.append({
            "severity": "warn",
            "title": "RR not published",
            "copy": f"A candidate RR exists but it was not published. Gate reason: {rr_gate_reason or 'unknown'}; publish reason: {rr_publish_reason or 'unknown'}."
        })

    reported_hr = radar.get("reported_hr")
    reported_rr = radar.get("reported_rr")
    if np.isfinite(float(hr)) and not (np.isfinite(float(reported_hr)) and float(reported_hr) > 0.0):
        faults.append({
            "severity": "warn",
            "title": "Radar HR is internal only",
            "copy": "The displayed radar HR is an internal smoothed estimate, not a validated published output for this instant."
        })
    if np.isfinite(float(rr)) and not (np.isfinite(float(reported_rr)) and float(reported_rr) > 0.0):
        faults.append({
            "severity": "warn",
            "title": "Radar RR is internal only",
            "copy": "The displayed radar RR is an internal smoothed estimate, not a validated published output for this instant."
        })

    if np.isfinite(float(hr)) and np.isfinite(float(rr)) and float(hr) > 0.0 and float(rr) > 0.0:
        ratio = float(hr) / max(float(rr), 1e-6)
        if ratio < 3.0 or ratio > 8.0:
            faults.append({
                "severity": "warn",
                "title": "HR/RR ratio review",
                "copy": f"Radar HR/RR ratio is {ratio:.1f}:1, outside the usual rest-range sanity band. Review for possible harmonic lock or unstable tracking."
            })

    if np.isfinite(float(hr)) and np.isfinite(float(ref_hr)):
        d = abs(float(hr) - float(ref_hr))
        if d > float(thresholds.get("hr_delta_crit", 8.0)):
            faults.append({"severity": "bad", "title": "HR mismatch critical", "copy": f"Radar HR differs from BLE by {d:.1f} bpm."})
        elif d > float(thresholds.get("hr_delta_warn", 3.0)):
            faults.append({"severity": "warn", "title": "HR mismatch elevated", "copy": f"Radar HR differs from BLE by {d:.1f} bpm."})
    if np.isfinite(float(rr)) and np.isfinite(float(ref_rr)):
        d = abs(float(rr) - float(ref_rr))
        if d > float(thresholds.get("rr_delta_crit", 4.0)):
            faults.append({"severity": "bad", "title": "RR mismatch critical", "copy": f"Radar RR differs from BLE by {d:.1f} br/min."})
            if (np.isfinite(float(rr)) and np.isfinite(float(ref_rr)) and float(rr) > (float(ref_rr) * 1.5)):
                faults.append({"severity": "bad", "title": "RR harmonic-lock suspected", "copy": "Radar RR is much higher than BLE while still marked valid. Review anchor lock / harmonic rejection behavior."})
        elif d > float(thresholds.get("rr_delta_warn", 2.0)):
            faults.append({"severity": "warn", "title": "RR mismatch elevated", "copy": f"Radar RR differs from BLE by {d:.1f} br/min."})

    if radar.get("logged_hr_valid") == 1 and radar.get("logged_rr_valid") == 1 and not faults:
        faults.append({"severity": "good", "title": "Streams healthy", "copy": "Radar and BLE are fresh and both validity flags are currently open."})
    elif not faults:
        faults.append({"severity": "good", "title": "Collecting", "copy": "The dashboard is live and waiting for stronger valid windows."})
    return faults[:6]


def _write_live_dashboard_html(session_dir: str):
    template = _load_dashboard_template_text()
    with open(_dashboard_html_path(session_dir), "w", encoding="utf-8") as f:
        f.write(template)


def _write_live_dashboard_json(session_dir: str, status: str, elapsed: float, remain: Optional[float],
                               radar_tracker: _CSVAppendTracker, ref_tracker: _CSVAppendTracker,
                               raw_tracker: _CSVAppendTracker, recent_events: List[str],
                               analysis_summary: Optional[Dict[str, object]] = None):
    radar_row = radar_tracker.last_row or {}
    ref_row = ref_tracker.last_row or {}
    series = _series_from_trackers(radar_tracker, ref_tracker)
    rad_df = _tracker_dataframe(radar_tracker)

    display_hr = _last_positive_or_nan(pd.Series(series.get("radar_hr", []), dtype=float)) if series.get("radar_hr") else float("nan")
    display_rr = _last_positive_or_nan(pd.Series(series.get("radar_rr", []), dtype=float)) if series.get("radar_rr") else float("nan")
    display_dist = _last_positive_or_nan(pd.Series(series.get("distance_cm", []), dtype=float)) if series.get("distance_cm") else float("nan")
    harmonic_mode_live = _to_num(_row_get(radar_row, "harmonic_mode"), default=float("nan"))
    session_phase_live = _to_num(_row_get(radar_row, "session_phase"), default=float("nan"))
    session_phase_int = int(round(session_phase_live)) if np.isfinite(session_phase_live) else None
    hr_path_source_live = _to_num(_row_get(radar_row, "hr_path_source"), default=float("nan"))
    hr_anchor_source_live = _to_num(_row_get(radar_row, "hr_anchor_source"), default=float("nan"))
    distance_for_pqi_gate = _resolve_live_distance_cm(display_dist, radar_row)

    radar = {
        "rows": int(radar_tracker.rows),
        "hr": display_hr,
        "rr": display_rr,
        "reported_hr": _to_num(_row_get(radar_row, "reported_hr"), default=float("nan")),
        "reported_rr": _to_num(_row_get(radar_row, "reported_rr"), default=float("nan")),
        "candidate_hr": _to_num(_row_get(radar_row, "candidate_hr"), default=float("nan")),
        "candidate_rr": _to_num(_row_get(radar_row, "candidate_rr"), default=float("nan")),
        "raw_hr": _to_num(_row_get(radar_row, "raw_hr"), default=float("nan")),
        "latched_raw_hr": _to_num(_row_get(radar_row, "latched_raw_hr"), default=float("nan")),
        "harmonic_mode": harmonic_mode_live,
        "harmonic_mode_hex": (None if not np.isfinite(harmonic_mode_live) else f"0x{int(round(harmonic_mode_live)):02X}"),
        "harmonic_mode_labels": _harmonic_mode_labels(harmonic_mode_live),
        "session_phase": session_phase_live,
        "session_phase_name": SESSION_PHASE_NAMES.get(session_phase_int, str(session_phase_int) if session_phase_int is not None else ""),
        "hr_locked_live": int(session_phase_int == 3),
        "hr_zc_bpm": _to_num(_row_get(radar_row, "hr_zc_bpm"), default=float("nan")),
        "hr_zc_conf": _to_num(_row_get(radar_row, "hr_zc_conf"), default=float("nan")),
        "hr_spec_bpm": _to_num(_row_get(radar_row, "hr_spec_bpm"), default=float("nan")),
        "hr_spec_mag": _to_num(_row_get(radar_row, "hr_spec_mag"), default=float("nan")),
        "hr_triple_agree": int(_to_num(_row_get(radar_row, "hr_triple_agree"), default=0.0) >= 0.5),
        "rr_zc_bpm": _to_num(_row_get(radar_row, "rr_zc_bpm"), default=float("nan")),
        "rr_zc_conf": _to_num(_row_get(radar_row, "rr_zc_conf"), default=float("nan")),
        "rr_spec_bpm": _to_num(_row_get(radar_row, "rr_spec_bpm"), default=float("nan")),
        "rr_spec_conf": _to_num(_row_get(radar_row, "rr_spec_conf"), default=float("nan")),
        "rr_triple_agree": int(_to_num(_row_get(radar_row, "rr_triple_agree"), default=0.0) >= 0.5),
        "raw_rr": _to_num(_row_get(radar_row, "raw_rr"), default=float("nan")),
        "candidate_hr_conf": _to_num(_row_get(radar_row, "candidate_hr_conf"), default=float("nan")),
        "candidate_rr_conf": _to_num(_row_get(radar_row, "candidate_rr_conf"), default=float("nan")),
        "pqi_heart": _to_num(_row_get(radar_row, "pqi_heart"), default=float("nan")),
        "pqi_breath": _to_num(_row_get(radar_row, "pqi_breath"), default=float("nan")),
        "heart_phase": _to_num(_row_get(radar_row, "heart_phase_stabilized", "heart_phase"), default=float("nan")),
        "breath_phase": _to_num(_row_get(radar_row, "breath_phase_stabilized", "breath_phase"), default=float("nan")),
        "fps_hz": _recent_rate_hz(radar_tracker),
        "hr_confidence": _to_num(_row_get(radar_row, "hr_confidence"), default=float("nan")),
        "hr_gate_reason": _row_get(radar_row, "hr_gate_reason") or "",
        "rr_gate_reason": _row_get(radar_row, "rr_gate_reason") or "",
        "hr_gate_reason_name": HR_GATE_REASON_NAMES.get(int(_to_num(_row_get(radar_row, "hr_gate_reason"), default=-1)), _row_get(radar_row, "hr_gate_reason") or ""),
        "rr_gate_reason_name": RR_GATE_REASON_NAMES.get(int(_to_num(_row_get(radar_row, "rr_gate_reason"), default=-1)), _row_get(radar_row, "rr_gate_reason") or ""),
        "hr_gate_pqi_used": _to_num(_row_get(radar_row, "hr_gate_pqi_used"), default=float("nan")),
        "rr_gate_pqi_used": _to_num(_row_get(radar_row, "rr_gate_pqi_used"), default=float("nan")),
        "distance_cm": display_dist,
        "heart_pqi_gate": _heart_pqi_gate(distance_for_pqi_gate),
        "reported_distance_cm": _to_num(_row_get(radar_row, "reported_distance_cm"), default=float("nan")),
        "reflector_distance_cm": _to_num(_row_get(radar_row, "reflector_distance_cm"), default=float("nan")),
        "motion": int(_to_num(_row_get(radar_row, "in_motion"), default=0.0) >= 0.5),
        "human": int(_to_num(_row_get(radar_row, "human_detected", "humanDetected"), default=0.0) >= 0.5),
        "radar_is_present_raw": int(_to_num(_row_get(radar_row, "radar_is_present_raw"), default=0.0) >= 0.5),
        "logged_hr_valid": int(_to_num(_row_get(radar_row, "logged_hr_valid"), default=0.0) >= 0.5),
        "logged_rr_valid": int(_to_num(_row_get(radar_row, "logged_rr_valid"), default=0.0) >= 0.5),
        "hr_publish_reason": _to_num(_row_get(radar_row, "hr_publish_reason"), default=float("nan")),
        "rr_publish_reason": _to_num(_row_get(radar_row, "rr_publish_reason"), default=float("nan")),
        "hr_publish_reason_name": HR_PUBLISH_REASON_NAMES.get(int(_to_num(_row_get(radar_row, "hr_publish_reason"), default=-1)), _row_get(radar_row, "hr_publish_reason") or ""),
        "rr_publish_reason_name": RR_PUBLISH_REASON_NAMES.get(int(_to_num(_row_get(radar_row, "rr_publish_reason"), default=-1)), _row_get(radar_row, "rr_publish_reason") or ""),
        "skipdsp_misses": _to_num(_row_get(radar_row, "skipdsp_misses"), default=float("nan")),
        "hr_band_min": _to_num(_row_get(radar_row, "hr_band_min"), default=float("nan")),
        "hr_band_max": _to_num(_row_get(radar_row, "hr_band_max"), default=float("nan")),
        "radar_gain": _to_num(_row_get(radar_row, "radar_gain"), default=float("nan")),
        "point_cloud_ok": int(_to_num(_row_get(radar_row, "point_cloud_ok"), default=0.0) >= 0.5),
        "target_info_ok": int(_to_num(_row_get(radar_row, "target_info_ok"), default=0.0) >= 0.5),
        "num_targets": _to_num(_row_get(radar_row, "num_targets"), default=float("nan")),
        "max_dop_abs": _to_num(_row_get(radar_row, "max_dop_abs"), default=float("nan")),
        "max_dop_speed_cms": _to_num(_row_get(radar_row, "max_dop_speed_cms"), default=float("nan")),
        "doppler_motion": int(_to_num(_row_get(radar_row, "doppler_motion"), default=0.0) >= 0.5),
        "cluster_anomaly": int(_to_num(_row_get(radar_row, "cluster_anomaly"), default=0.0) >= 0.5),
        "multi_target": int(_to_num(_row_get(radar_row, "multi_target"), default=0.0) >= 0.5),
        "primary_x": _to_num(_row_get(radar_row, "primary_x"), default=float("nan")),
        "primary_y": _to_num(_row_get(radar_row, "primary_y"), default=float("nan")),
        "primary_dop": _to_num(_row_get(radar_row, "primary_dop"), default=float("nan")),
        "primary_dop_speed_cms": _to_num(_row_get(radar_row, "primary_dop_speed_cms"), default=float("nan")),
        "primary_cluster": _to_num(_row_get(radar_row, "primary_cluster"), default=float("nan")),
        "spatial_source": _to_num(_row_get(radar_row, "spatial_source"), default=(1.0 if _to_num(_row_get(radar_row, "target_info_ok"), default=0.0) >= 0.5 else (2.0 if _to_num(_row_get(radar_row, "point_cloud_ok"), default=0.0) >= 0.5 else 0.0))),
        "spatial_age_ms": _to_num(_row_get(radar_row, "spatial_age_ms"), default=float("nan")),
        "position_radius_cm": _to_num(_row_get(radar_row, "position_radius_cm"), default=float("nan")),
        "phase_warmup_complete": int(_to_num(_row_get(radar_row, "phase_warmup_complete"), default=0.0) >= 0.5),
        "clutter_warmup_count": _to_num(_row_get(radar_row, "clutter_warmup_count"), default=float("nan")),
        "current_clutter_alpha": _to_num(_row_get(radar_row, "current_clutter_alpha"), default=float("nan")),
        "use_fast_path": int(_to_num(_row_get(radar_row, "use_fast_path"), default=0.0) >= 0.5),
        "phase_valid_this_frame": int(_to_num(_row_get(radar_row, "phase_valid_this_frame"), default=0.0) >= 0.5),
        "dsp_ran_this_frame": int(_to_num(_row_get(radar_row, "dsp_ran_this_frame"), default=0.0) >= 0.5),
        "hr_confidence_source": _to_num(_row_get(radar_row, "hr_confidence_source"), default=float("nan")),
        "hr_confidence_source_name": HR_CONFIDENCE_SOURCE_NAMES.get(int(_to_num(_row_get(radar_row, "hr_confidence_source"), default=-1)), _row_get(radar_row, "hr_confidence_source") or ""),
        "hr_path_source": hr_path_source_live,
        "hr_path_source_name": _hr_path_source_name(hr_path_source_live),
        "module_fw_major": _to_num(_row_get(radar_row, "module_fw_major", "fw_major"), default=float("nan")),
        "module_fw_sub": _to_num(_row_get(radar_row, "module_fw_sub", "fw_sub"), default=float("nan")),
        "module_fw_mod": _to_num(_row_get(radar_row, "module_fw_mod", "fw_mod"), default=float("nan")),
        "sketch_major": _to_num(_row_get(radar_row, "sketch_major"), default=float("nan")),
        "sketch_sub": _to_num(_row_get(radar_row, "sketch_sub"), default=float("nan")),
        "sketch_mod": _to_num(_row_get(radar_row, "sketch_mod"), default=float("nan")),
        "hr_trusted_phase_anchor": _to_num(_row_get(radar_row, "hr_trusted_phase_anchor"), default=float("nan")),
        "hr_anchor_source": hr_anchor_source_live,
        "hr_anchor_source_name": (
            HR_ANCHOR_SOURCE_NAMES.get(int(round(hr_anchor_source_live)), "")
            if np.isfinite(hr_anchor_source_live) else ""
        ),
        "hr_anchor_err_bpm": _to_num(_row_get(radar_row, "hr_anchor_err_bpm"), default=float("nan")),
        "hr_raw_high_bias_suspect": int(_to_num(_row_get(radar_row, "hr_raw_high_bias_suspect"), default=0.0) >= 0.5),
        "module_firmware_version": (
            None
            if any((not np.isfinite(_to_num(_row_get(radar_row, key), default=float("nan")))) or (int(_to_num(_row_get(radar_row, key), default=-1)) < 0)
                   for key in ("module_fw_major", "module_fw_sub", "module_fw_mod"))
            else f"{int(_to_num(_row_get(radar_row, 'module_fw_major'), default=-1))}.{int(_to_num(_row_get(radar_row, 'module_fw_sub'), default=-1))}.{int(_to_num(_row_get(radar_row, 'module_fw_mod'), default=-1))}"
        ),
        "sketch_firmware_version": (
            None
            if any((not np.isfinite(_to_num(_row_get(radar_row, key), default=float("nan")))) or (int(_to_num(_row_get(radar_row, key), default=-1)) < 0)
                   for key in ("sketch_major", "sketch_sub", "sketch_mod"))
            else f"{int(_to_num(_row_get(radar_row, 'sketch_major'), default=-1))}.{int(_to_num(_row_get(radar_row, 'sketch_sub'), default=-1))}.{int(_to_num(_row_get(radar_row, 'sketch_mod'), default=-1))}"
        ),
        "firmware_version": (
            None
            if any((not np.isfinite(_to_num(_row_get(radar_row, key), default=float("nan")))) or (int(_to_num(_row_get(radar_row, key), default=-1)) < 0)
                   for key in ("sketch_major", "sketch_sub", "sketch_mod"))
            else f"{int(_to_num(_row_get(radar_row, 'sketch_major'), default=-1))}.{int(_to_num(_row_get(radar_row, 'sketch_sub'), default=-1))}.{int(_to_num(_row_get(radar_row, 'sketch_mod'), default=-1))}"
        ),
        "hr_arbiter_corrected": int(_to_num(_row_get(radar_row, "hr_arbiter_corrected"), default=0.0) >= 0.5),
        "hr_rejectphase_rejected": int(_to_num(_row_get(radar_row, "hr_rejectphase_rejected"), default=0.0) >= 0.5),
        "hr_coherence_rejected": int(_to_num(_row_get(radar_row, "hr_coherence_rejected"), default=0.0) >= 0.5),
        "hr_raw_source": _to_num(_row_get(radar_row, "hr_raw_source"), default=float("nan")),
        "hr_raw_agree": int(_to_num(_row_get(radar_row, "hr_raw_agree"), default=0.0) >= 0.5),
        "hr_agree_err_bpm": _to_num(_row_get(radar_row, "hr_agree_err_bpm"), default=float("nan")),
        "hr_bypass_pqi_ok": int(_to_num(_row_get(radar_row, "hr_bypass_pqi_ok"), default=0.0) >= 0.5),
        "hr_bypass_conf_ok": int(_to_num(_row_get(radar_row, "hr_bypass_conf_ok"), default=0.0) >= 0.5),
        "hr_bypass_gate_ok": int(_to_num(_row_get(radar_row, "hr_bypass_gate_ok"), default=0.0) >= 0.5),
        "hr_bypass_active": int(_to_num(_row_get(radar_row, "hr_bypass_active"), default=0.0) >= 0.5),
        "hr_grace_eligible": int(_to_num(_row_get(radar_row, "hr_grace_eligible"), default=0.0) >= 0.5),
        "hr_grace_active": int(_to_num(_row_get(radar_row, "hr_grace_active"), default=0.0) >= 0.5),
        "hr_trust_fresh": int(_to_num(_row_get(radar_row, "hr_trust_fresh"), default=0.0) >= 0.5),
        "rr_anchor_fresh": int(_to_num(_row_get(radar_row, "rr_anchor_fresh"), default=0.0) >= 0.5),
        "rr_outlier_persist": _to_num(_row_get(radar_row, "rr_outlier_persist"), default=float("nan")),
        "rr_raw_agree_ok": int(_to_num(_row_get(radar_row, "rr_raw_agree_ok"), default=0.0) >= 0.5),
        "rr_pre_acceptphase": _to_num(_row_get(radar_row, "rr_pre_acceptphase"), default=float("nan")),
        "rr_post_acceptphase": _to_num(_row_get(radar_row, "rr_post_acceptphase"), default=float("nan")),
        "rr_post_blend": _to_num(_row_get(radar_row, "rr_post_blend"), default=float("nan")),
        "rr_post_bias_correction": _to_num(_row_get(radar_row, "rr_post_bias_correction"), default=float("nan")),
        "rr_post_kalman": _to_num(_row_get(radar_row, "rr_post_kalman"), default=float("nan")),
        "rr_final_publish_candidate": _to_num(_row_get(radar_row, "rr_final_publish_candidate"), default=float("nan")),
        "rr_anchor_value": _to_num(_row_get(radar_row, "rr_anchor_value"), default=float("nan")),
        "rr_anchor_age_ms": _to_num(_row_get(radar_row, "rr_anchor_age_ms"), default=float("nan")),
        "rr_anchor_source": _to_num(_row_get(radar_row, "rr_anchor_source"), default=float("nan")),
        "rr_anchor_confidence": _to_num(_row_get(radar_row, "rr_anchor_confidence"), default=float("nan")),
        "rr_fundamental_recovery_count": _to_num(_row_get(radar_row, "rr_fundamental_recovery_count"), default=float("nan")),
        "rr_fundamental_recovery_triggered": int(_to_num(_row_get(radar_row, "rr_fundamental_recovery_triggered"), default=0.0) >= 0.5),
        "rr_raw_seed_consistent_count": _to_num(_row_get(radar_row, "rr_raw_seed_consistent_count"), default=float("nan")),
        "rr_midsession_raw_reanchor_allowed": int(_to_num(_row_get(radar_row, "rr_midsession_raw_reanchor_allowed"), default=0.0) >= 0.5),
        "rr_midsession_raw_reanchor_blocked": int(_to_num(_row_get(radar_row, "rr_midsession_raw_reanchor_blocked"), default=0.0) >= 0.5),
        "rr_midsession_raw_reanchor_reason": _to_num(_row_get(radar_row, "rr_midsession_raw_reanchor_reason"), default=float("nan")),
        "rr_raw_anchor_err_bpm": _to_num(_row_get(radar_row, "rr_raw_anchor_err_bpm"), default=float("nan")),
        "hr_pre_rejectphase": _to_num(_row_get(radar_row, "hr_pre_rejectphase"), default=float("nan")),
        "hr_post_rejectphase": _to_num(_row_get(radar_row, "hr_post_rejectphase"), default=float("nan")),
        "hr_post_blend": _to_num(_row_get(radar_row, "hr_post_blend"), default=float("nan")),
        "hr_post_coherence": _to_num(_row_get(radar_row, "hr_post_coherence"), default=float("nan")),
        "hr_final_publish_candidate": _to_num(_row_get(radar_row, "hr_final_publish_candidate"), default=float("nan")),
        "hr_arbiter_anchor_used": int(_to_num(_row_get(radar_row, "hr_arbiter_anchor_used"), default=0.0) >= 0.5),
        "hr_arbiter_anchor_value": _to_num(_row_get(radar_row, "hr_arbiter_anchor_value"), default=float("nan")),
        "hr_rejectphase_anchor_used": int(_to_num(_row_get(radar_row, "hr_rejectphase_anchor_used"), default=0.0) >= 0.5),
        "hr_rejectphase_anchor_value": _to_num(_row_get(radar_row, "hr_rejectphase_anchor_value"), default=float("nan")),
        "hr_raw_looks_like_half_rate": int(_to_num(_row_get(radar_row, "hr_raw_looks_like_half_rate"), default=0.0) >= 0.5),
        "hr_trusted_anchor_value": _to_num(_row_get(radar_row, "hr_trusted_anchor_value"), default=float("nan")),
        "hr_age_ms": _to_num(_row_get(radar_row, "hr_age_ms"), default=float("nan")),
        "candidate_hr_age_ms": _to_num(_row_get(radar_row, "candidate_hr_age_ms"), default=float("nan")),
        "candidate_rr_age_ms": _to_num(_row_get(radar_row, "candidate_rr_age_ms"), default=float("nan")),
        "hr_updated_this_cycle": int(_to_num(_row_get(radar_row, "hr_updated_this_cycle"), default=0.0) >= 0.5),
        "hr_update_source": _to_num(_row_get(radar_row, "hr_update_source"), default=float("nan")),
        "trusted_rr_fresh": int(_to_num(_row_get(radar_row, "trusted_rr_fresh"), default=0.0) >= 0.5),
        "rr_seed_from_raw_used": int(_to_num(_row_get(radar_row, "rr_seed_from_raw_used"), default=_to_num(_row_get(radar_row, "rr_raw_seed_consistent_count"), default=0.0)) >= 0.5),
        "trusted_hr_fresh": int(_to_num(_row_get(radar_row, "trusted_hr_fresh", "hr_trust_fresh"), default=0.0) >= 0.5),
        "allow_logged_hr_vitals": int(_to_num(_row_get(radar_row, "allow_logged_hr_vitals"), default=0.0) >= 0.5),
        "allow_logged_rr_vitals": int(_to_num(_row_get(radar_row, "allow_logged_rr_vitals"), default=0.0) >= 0.5),
        "logged_hr_quality_gate": int(_to_num(_row_get(radar_row, "logged_hr_quality_gate"), default=0.0) >= 0.5),
        "logged_rr_quality_gate": int(_to_num(_row_get(radar_row, "logged_rr_quality_gate"), default=0.0) >= 0.5),
        "phase_valid_run_len": _to_num(_row_get(radar_row, "phase_valid_run_len"), default=float("nan")),
        "phase_invalid_run_len": _to_num(_row_get(radar_row, "phase_invalid_run_len"), default=float("nan")),
        "hr_phase_backed_update_count": _to_num(_row_get(radar_row, "hr_phase_backed_update_count"), default=float("nan")),
        "rr_phase_backed_update_count": _to_num(_row_get(radar_row, "rr_phase_backed_update_count"), default=float("nan")),
        "near_field_reflector_suspect": int(_to_num(_row_get(radar_row, "near_field_reflector_suspect"), default=0.0) >= 0.5),
        "agc_floor_suspect": int(_to_num(_row_get(radar_row, "agc_floor_suspect"), default=0.0) >= 0.5),
        "phase_backed_publish_ready": int(_to_num(_row_get(radar_row, "phase_backed_publish_ready"), default=0.0) >= 0.5),
        "hr_anchor_drift_suspect": int(_to_num(_row_get(radar_row, "hr_anchor_drift_suspect"), default=0.0) >= 0.5),
        "phase_gap_fill_count": _to_num(_row_get(radar_row, "phase_gap_fill_count"), default=float("nan")),
        "clutter_rewarm_count": _to_num(_row_get(radar_row, "clutter_rewarm_count"), default=float("nan")),
        "experimental_profile_enabled": int(_to_num(_row_get(radar_row, "experimental_profile_enabled"), default=0.0) >= 0.5),
        "last_seen_age_s": _age_from_tracker(radar_tracker, elapsed),
        "last_valid_hr_age_s": _last_valid_age_s(radar_tracker, ("logged_hr_valid",)),
        "last_valid_rr_age_s": _last_valid_age_s(radar_tracker, ("logged_rr_valid",)),
    }
    ble = {
        "rows": int(ref_tracker.rows),
        "raw_packets": int(raw_tracker.rows),
        "hr": _to_num(_row_get(ref_row, "ref_hr"), default=float("nan")),
        "rr": _to_num(_row_get(ref_row, "ref_rr"), default=float("nan")),
        "spo2": _to_num(_row_get(ref_row, "ref_spo2"), default=float("nan")),
        "pi": _to_num(_row_get(ref_row, "ref_pi"), default=float("nan")),
        "last_seen_age_s": _age_from_tracker(ref_tracker, elapsed),
    }
    thresholds = {
        "hr_delta_warn": 3.0,
        "hr_delta_crit": 8.0,
        "rr_delta_warn": 2.0,
        "rr_delta_crit": 4.0,
        "freshness_warn_s": 1.5,
        "freshness_crit_s": 3.0,
    }
    payload = {
        "meta": {
            "status": status,
            "elapsed_s": round(float(elapsed), 1),
            "remaining_s": None if remain is None else round(float(remain), 1),
            "version": VERSION,
            "session_dir": os.path.abspath(session_dir),
        },
        "radar": nan_safe(radar),
        "ble": nan_safe(ble),
        "thresholds": thresholds,
        "faults": _build_dashboard_faults(radar, ble, thresholds),
        "events": list(recent_events[-12:] if recent_events else []),
        "series": nan_safe(series),
        "analysis": nan_safe(analysis_summary) if isinstance(analysis_summary, dict) else None,
    }
    save_json(payload, _dashboard_json_path(session_dir))


def _render_live_dashboard(session_dir: str, start_t: float, duration_s: Optional[float], radar_tracker: _CSVAppendTracker,
                           ref_tracker: _CSVAppendTracker, raw_tracker: _CSVAppendTracker, recent_events: Deque[str],
                           event_lock: threading.Lock, status: str,
                           analysis_summary: Optional[Dict[str, object]] = None):
    elapsed = max(0.0, time.time() - start_t)
    radar = radar_tracker.last_row or {}
    ref = ref_tracker.last_row or {}
    remain = None if duration_s is None else max(0.0, duration_s - elapsed)
    lines = []
    lines.append(_bold(f"Radar Vital Trainer v{VERSION} — Live Session Dashboard"))
    lines.append("=" * 78)
    lines.append(f"Session: {session_dir}")
    lines.append(f"Status:  {status}")
    lines.append(f"Elapsed: {elapsed:7.1f}s" + ("" if remain is None else f"   Remaining: {remain:7.1f}s"))
    lines.append("")
    lines.append(_bold("Radar"))
    lines.append(f"  rows={radar_tracker.rows:5d} | HR={_to_num(_row_get(radar, 'reported_hr'), float('nan')):6.1f} | RR={_to_num(_row_get(radar, 'reported_rr'), float('nan')):5.1f} | "
                 f"PQIh={_to_num(_row_get(radar, 'pqi_heart'), float('nan')):4.2f} | PQIb={_to_num(_row_get(radar, 'pqi_breath'), float('nan')):4.2f} | "
                 f"Dist={_to_num(_row_get(radar, 'reported_distance_cm', 'reflector_distance_cm'), float('nan')):6.1f} cm | Motion={_row_get(radar, 'in_motion') or ''} | Human={_row_get(radar, 'human_detected', 'humanDetected') or ''}")
    lines.append(_bold("BLE Reference"))
    lines.append(f"  rows={ref_tracker.rows:5d} | raw_packets={raw_tracker.rows:5d} | HR={_to_num(_row_get(ref, 'ref_hr'), float('nan')):6.1f} | RR={_to_num(_row_get(ref, 'ref_rr'), float('nan')):5.1f} | "
                 f"SpO2={_to_num(_row_get(ref, 'ref_spo2'), float('nan')):5.1f} | PI={_to_num(_row_get(ref, 'ref_pi'), float('nan')):5.1f}")
    lines.append("")
    lines.append(_bold("Recent events"))
    with event_lock:
        tail = list(recent_events)[-8:]
    if tail:
        lines.extend(f"  {ln}" for ln in tail)
    else:
        lines.append("  waiting for events...")
    screen = "\n".join(lines)
    if sys.stdout.isatty():
        sys.stdout.write("\x1b[2J\x1b[H" + screen + "\n")
        sys.stdout.flush()
    else:
        print(screen)
    try:
        _write_live_dashboard_html(session_dir)
        _write_live_dashboard_json(session_dir, status, elapsed, remain, radar_tracker, ref_tracker, raw_tracker, tail, analysis_summary=analysis_summary)
    except Exception as e:
        warn(f"Dashboard update failed: {e}")


def _start_dashboard_server(session_dir: str, port: int = 0) -> _DashboardServer:
    server = _DashboardServer(session_dir, port=port)
    server.start()
    return server


def _next_session_dir(root: str, prefix: str = "s", digits: int = 2) -> str:
    os.makedirs(root, exist_ok=True)
    pat = re.compile(rf"^{re.escape(prefix)}(\d+)$", re.IGNORECASE)
    max_n = 0
    for name in os.listdir(root):
        m = pat.match(name)
        if m:
            max_n = max(max_n, int(m.group(1)))
    return os.path.join(root, f"{prefix}{max_n + 1:0{digits}d}")
def _print_and_save_session_report(session_dir: str, analyse_dir: str, radar_out: str, ref_out: str):
    analyse_summary_path = os.path.join(analyse_dir, "analyse_summary.json")
    ble_summary_path = os.path.join(session_dir, "ref_ble_summary.json")
    analyse_summary = _read_json_if_exists(analyse_summary_path)
    ble_summary = _read_json_if_exists(ble_summary_path)

    lines = []
    lines.append(f"SESSION REPORT v{VERSION}")
    lines.append("=" * 72)
    lines.append(f"session_dir: {os.path.abspath(session_dir)}")
    lines.append(f"radar_csv:   {os.path.abspath(radar_out)}")
    lines.append(f"ref_csv:     {os.path.abspath(ref_out)}")
    lines.append(f"analyse_dir: {os.path.abspath(analyse_dir)}")
    lines.append(f"dashboard:   {os.path.abspath(_dashboard_html_path(session_dir))}")
    if isinstance(ble_summary, dict):
        lines.append("")
        lines.append("[BLE]")
        lines.append(f"parsed_rows: {ble_summary.get('parsed_rows')}")
        lines.append(f"raw_packets: {ble_summary.get('raw_packets')}")
        lines.append(f"notify_active: {ble_summary.get('notify_chars_active')}")
        lines.append(f"parse_source_uuid: {ble_summary.get('parse_source_uuid')}")
        fmap = ble_summary.get("field_map", {})
        lines.append(f"field_map: HR@{fmap.get('hr_byte_index')} x{fmap.get('hr_scale')}, "
                     f"RR@{fmap.get('rr_byte_index')} x{fmap.get('rr_scale')}, "
                     f"SpO2@{fmap.get('spo2_byte_index')} x{fmap.get('spo2_scale')}, "
                     f"PI@{fmap.get('pi_byte_index')} x{fmap.get('pi_scale')}")
    if isinstance(analyse_summary, dict):
        lines.append("")
        lines.append("[ANALYSIS]")
        ml = analyse_summary.get("ml_gate", {})
        if not isinstance(ml, dict):
            ml = {}
        ml_r = _to_num(ml.get("r"), float("nan"))
        ml_rmse = _to_num(ml.get("rmse"), float("nan"))
        lines.append(f"ml_gate: {'PASS' if ml.get('passed') else 'FAIL'} | "
                     f"r={ml_r:.3f} | rmse={ml_rmse:.2f}")
        for label, key in (("combined", "ml_gate_combined"), ("locked", "ml_gate_locked"), ("settling", "ml_gate_settling")):
            block = analyse_summary.get(key)
            if isinstance(block, dict):
                lines.append(f"ml_gate_{label}: {'PASS' if block.get('passed') else 'FAIL'} | "
                             f"r={_to_num(block.get('r'), float('nan')):.3f} | "
                             f"rmse={_to_num(block.get('rmse'), float('nan')):.2f} | "
                             f"n={_safe_int(block.get('n'), 0)}")
        q = _to_num(analyse_summary.get("session_quality_score"), float("nan"))
        iq = _to_num(analyse_summary.get("internal_consistency_score"), float("nan"))
        lines.append(f"session_quality_score: {q:.1f}" if np.isfinite(q) else "session_quality_score: nan")
        lines.append(f"internal_consistency_score: {iq:.1f}" if np.isfinite(iq) else "internal_consistency_score: nan")
        if analyse_summary.get("review_required"):
            lines.append(f"review_flags: {','.join(analyse_summary.get('review_flags', []))}")
        fw_truth = analyse_summary.get("fw_truthfulness", {})
        if isinstance(fw_truth, dict):
            lines.append(f"fw_truthfulness: sketch={fw_truth.get('version', 'unknown')} | "
                         f"module={fw_truth.get('module_version') or 'unknown'} | "
                         f"module_valid={bool(fw_truth.get('module_version_valid', False))}")
        pqi = _to_num(analyse_summary.get("pqi_lock_pct"), float("nan"))
        lines.append(f"pqi_lock_pct: {pqi:.1f}" if np.isfinite(pqi) else "pqi_lock_pct: nan")
        cov_locked = _to_num(analyse_summary.get("coverage_locked"), float("nan"))
        cov_settling = _to_num(analyse_summary.get("coverage_settling"), float("nan"))
        if np.isfinite(cov_locked) or np.isfinite(cov_settling):
            lines.append(f"coverage_split: locked={cov_locked:.1f}% | settling={cov_settling:.1f}%")
        raw_hr_bias_est = analyse_summary.get("raw_hr_bias_estimate", {}) if isinstance(analyse_summary.get("raw_hr_bias_estimate"), dict) else {}
        raw_hr_bias = _to_num(raw_hr_bias_est.get("bias_bpm"), float("nan"))
        try:
            raw_hr_bias_n = int(float(raw_hr_bias_est.get("n", 0) or 0)) if raw_hr_bias_est else 0
        except Exception:
            raw_hr_bias_n = 0
        if np.isfinite(raw_hr_bias):
            lines.append(f"raw_hr_bias_estimate: {raw_hr_bias:+.2f} BPM (n={raw_hr_bias_n})")
        sumfreq_est = analyse_summary.get("raw_hr_sumfreq_estimate", {}) if isinstance(analyse_summary.get("raw_hr_sumfreq_estimate"), dict) else {}
        sumfreq_mae = _to_num(sumfreq_est.get("mean_abs_error_bpm"), float("nan"))
        try:
            sumfreq_n = int(float(sumfreq_est.get("n", 0) or 0)) if sumfreq_est else 0
        except Exception:
            sumfreq_n = 0
        if np.isfinite(sumfreq_mae):
            lines.append(f"raw_hr_sumfreq_mae: {sumfreq_mae:.2f} BPM (n={sumfreq_n})")
        lines.extend(_fmt_quick_metrics("HR", analyse_summary.get("hr_baseline")))
        lines.extend(_fmt_quick_metrics("RR", analyse_summary.get("rr_baseline")))
        dt = analyse_summary.get("alignment_dt_s", {})
        if not isinstance(dt, dict):
            dt = {}
        dt_mean = _to_num(dt.get("mean"), float("nan"))
        dt_max_abs = _to_num(dt.get("max_abs"), float("nan"))
        lines.append(f"alignment_dt_s: mean={dt_mean:.3f} | max_abs={dt_max_abs:.3f}")
        gate_audit = analyse_summary.get("gate_audit", {})
        if isinstance(gate_audit, dict):
            funnel = gate_audit.get("funnel", {})
            if isinstance(funnel, dict) and funnel:
                lines.append("funnel: " + " | ".join([
                    f"frames={_safe_int(funnel.get('radar_frames_total'), 0)}",
                    f"human={_safe_int(funnel.get('human_frames'), 0)}",
                    f"dsp={_safe_int(funnel.get('dsp_frames'), 0)}",
                    f"phase={_safe_int(funnel.get('phase_valid_frames'), 0)}",
                    f"hr_valid={_safe_int(funnel.get('logged_hr_valid_frames'), 0)}",
                    f"rr_valid={_safe_int(funnel.get('logged_rr_valid_frames'), 0)}",
                    f"hr_bins={_safe_int(funnel.get('hr_eval_bins'), 0)}",
                    f"rr_bins={_safe_int(funnel.get('rr_eval_bins'), 0)}",
                ]))
        pub_hist = analyse_summary.get("publish_reason_histogram", {})
        if isinstance(pub_hist, dict):
            hr_pub = _top_histogram_items(pub_hist.get("hr"), limit=3)
            rr_pub = _top_histogram_items(pub_hist.get("rr"), limit=3)
            if hr_pub:
                lines.append("top_hr_publish_blocks: " + " | ".join(hr_pub))
            if rr_pub:
                lines.append("top_rr_publish_blocks: " + " | ".join(rr_pub))
        hr_gate_hist = analyse_summary.get("hr_gate_reason_histogram", {})
        rr_gate_hist = analyse_summary.get("rr_gate_reason_histogram", {})
        if isinstance(hr_gate_hist, dict) and hr_gate_hist:
            lines.append("top_hr_gate_reasons: " + " | ".join(_top_histogram_items(hr_gate_hist, limit=3)))
        if isinstance(rr_gate_hist, dict) and rr_gate_hist:
            lines.append("top_rr_gate_reasons: " + " | ".join(_top_histogram_items(rr_gate_hist, limit=3)))
        agc = analyse_summary.get("agc_anomaly_flags", {})
        if isinstance(agc, dict):
            lines.append(f"agc_anomalies: gain_floor={_to_num(agc.get('gain_floor_pct'), float('nan')):.1f}% | "
                         f"near_field={_to_num(agc.get('near_field_pct'), float('nan')):.1f}% | "
                         f"skipdsp={_to_num(agc.get('skipdsp_pct'), float('nan')):.1f}%")
        golden = analyse_summary.get("golden_check")
        if isinstance(golden, dict):
            lines.append(f"golden_check: {'PASS' if golden.get('passed') else 'FAIL'}")
    report_path = os.path.join(session_dir, "session_quick_report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print("\n" + _bold("=== SESSION QUICK REPORT ==="))
    print("\n".join(lines))
    print(_green(f"[OUT] Quick report saved to {report_path}"))

def cmd_session(args):
    if str(args.session_dir).strip().lower() == "auto":
        session_dir = _next_session_dir(os.path.abspath(args.sessions_root), args.session_prefix, args.session_digits)
    else:
        session_dir = os.path.abspath(args.session_dir)
    os.makedirs(session_dir, exist_ok=True)
    analysis_dir = os.path.join(session_dir, "analysis")
    radar_out = os.path.join(session_dir, "radar.csv")
    ref_out = os.path.join(session_dir, "ref.csv")
    script_path = os.path.abspath(__file__)

    radar_cmd = [sys.executable, "-u", script_path, "log",
                 "--port", args.port,
                 "--out", radar_out]

    ble_cmd = [sys.executable, "-u", script_path, "ble_reflog",
               "--out", ref_out]
    if getattr(args, "address", None):
        ble_cmd += ["--address", args.address]
    if getattr(args, "ble_profile", None):
        ble_cmd += ["--ble-profile", args.ble_profile]
    if getattr(args, "notify_char", None):
        ble_cmd += ["--notify-char", args.notify_char]
    if args.echo_raw:
        ble_cmd.append("--echo-raw")
    if args.probe_services:
        ble_cmd.append("--probe-services")
    if args.subscribe_all:
        ble_cmd.append("--subscribe-all")

    dashboard_enabled = bool(getattr(args, "live_dashboard", False) or getattr(args, "open_dashboard", True))
    radar_tracker = _CSVAppendTracker(radar_out, keep_rows=480)
    ref_tracker = _CSVAppendTracker(ref_out, keep_rows=480)
    raw_tracker = _CSVAppendTracker(os.path.join(session_dir, "ref_ble_raw.csv"), keep_rows=480)
    recent_events: Deque[str] = deque(["[INFO] Session initialized"], maxlen=24)
    event_lock = threading.Lock()
    dashboard_server = None

    def event_cb(prefix: str, line: str):
        msg = _format_dashboard_event(prefix, line)
        with event_lock:
            recent_events.append(msg)

    print(_bold("\n=== Combined Session Runner ==="))
    print(f"Session dir: {session_dir}")
    print(f"Radar CSV:   {radar_out}")
    print(f"Ref CSV:     {ref_out}")
    print(f"Dashboard:   {_dashboard_html_path(session_dir)}")
    if args.duration_s:
        print(f"Auto-stop:   {args.duration_s:.1f}s")
    else:
        print("Auto-stop:   manual (Ctrl-C)")
    print(f"Analyse:     {analysis_dir}\n")

    if dashboard_enabled:
        _write_live_dashboard_html(session_dir)
        _write_live_dashboard_json(session_dir, "starting", 0.0, args.duration_s, radar_tracker, ref_tracker, raw_tracker, ["[INFO] Session starting"], analysis_summary=_dashboard_analysis_payload(session_dir))
        try:
            dashboard_server = _start_dashboard_server(session_dir, port=int(getattr(args, "dashboard_port", 0) or 0))
            print(_green(f"[DASHBOARD] {dashboard_server.url}"))
            if getattr(args, "open_dashboard", True):
                try:
                    import webbrowser
                    webbrowser.open(dashboard_server.url)
                except Exception as e:
                    warn(f"Could not open dashboard in browser: {e}")
        except Exception as e:
            dashboard_enabled = False
            warn(f"Could not start dashboard server: {e}")

    radar_proc = None
    ble_proc = None
    radar_thread = None
    ble_thread = None
    stop_reason = "completed"
    start_t = time.time()

    try:
        radar_proc = subprocess.Popen(
            radar_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, universal_newlines=True
        )
        radar_thread = threading.Thread(
            target=_stream_subprocess_output,
            args=(radar_proc.stdout, "[RADAR]", event_cb, not dashboard_enabled),
            daemon=True
        )
        radar_thread.start()

        ble_proc = subprocess.Popen(
            ble_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, universal_newlines=True
        )
        ble_thread = threading.Thread(
            target=_stream_subprocess_output,
            args=(ble_proc.stdout, "[BLE]", event_cb, not dashboard_enabled),
            daemon=True
        )
        ble_thread.start()

        while True:
            time.sleep(max(0.15, float(getattr(args, "dashboard_refresh_s", 1.0))))
            elapsed = time.time() - start_t
            if dashboard_enabled:
                radar_tracker.poll(); ref_tracker.poll(); raw_tracker.poll()
                _render_live_dashboard(session_dir, start_t, args.duration_s, radar_tracker, ref_tracker, raw_tracker,
                                       recent_events, event_lock, "collecting", analysis_summary=_dashboard_analysis_payload(session_dir))

            if args.duration_s and elapsed >= args.duration_s:
                stop_reason = f"duration reached ({args.duration_s:.1f}s)"
                break

            radar_done = (radar_proc.poll() is not None)
            ble_done = (ble_proc.poll() is not None)
            if radar_done and ble_done:
                stop_reason = "both loggers exited"
                break
            if radar_done and not ble_done:
                stop_reason = "radar logger exited"
                break
            if ble_done and not radar_done:
                stop_reason = "BLE logger exited"
                break

    except KeyboardInterrupt:
        stop_reason = "user interrupt"
        print(_yellow("\n[SESSION] Stop requested by user."))
    finally:
        print(_yellow(f"[SESSION] Stopping child processes ({stop_reason})..."))
        _terminate_subprocess(radar_proc, "RADAR")
        _terminate_subprocess(ble_proc, "BLE")
        if radar_thread is not None:
            radar_thread.join(timeout=3.0)
        if ble_thread is not None:
            ble_thread.join(timeout=3.0)
        radar_tracker.poll(); ref_tracker.poll(); raw_tracker.poll()
        if dashboard_enabled:
            _render_live_dashboard(session_dir, start_t, args.duration_s, radar_tracker, ref_tracker, raw_tracker,
                                   recent_events, event_lock, f"stopped ({stop_reason})", analysis_summary=_dashboard_analysis_payload(session_dir))

    if not (os.path.exists(radar_out) and os.path.exists(ref_out)):
        warn("Expected radar/ref CSV files were not created; skipping analyse.")
        if dashboard_server is not None:
            dashboard_server.stop()
        return

    print(_bold("\n=== Auto Analyse ==="))
    analyse_args = argparse.Namespace(
        radar=[radar_out],
        ref=[ref_out],
        out=analysis_dir,
        tolerance_s=args.tolerance_s,
        merge_direction=args.merge_direction,
        ref_offset_s=None,
        auto_align_start=args.auto_align_start,
        feature_mode=args.feature_mode,
        phase_unit=args.phase_unit,
        heart_fft_window_s=args.heart_fft_window_s,
        breath_fft_window_s=args.breath_fft_window_s,
        no_plots=args.no_plots,
        annotate_bland_altman=args.annotate_bland_altman,
        golden_json=args.golden_json,
        golden_hr_rmse_tol=args.golden_hr_rmse_tol,
        golden_hr_r_tol=args.golden_hr_r_tol,
    )
    try:
        cmd_analyse(analyse_args)
    except Exception as e:
        warn(f"Auto analyse failed: {e}")
        if dashboard_server is not None:
            dashboard_server.stop()
        return

    _print_and_save_session_report(session_dir, analysis_dir, radar_out, ref_out)
    analysis_payload = _dashboard_analysis_payload(session_dir)
    with event_lock:
        recent_events.append("[ANALYSIS] Summary ready")
    if dashboard_enabled:
        radar_tracker.poll(); ref_tracker.poll(); raw_tracker.poll()
        _render_live_dashboard(session_dir, start_t, args.duration_s, radar_tracker, ref_tracker, raw_tracker,
                               recent_events, event_lock, "analysis complete", analysis_summary=analysis_payload)
        hold_s = max(0.0, float(getattr(args, "dashboard_hold_s", 6.0) or 0.0))
        if dashboard_server is not None and hold_s > 0:
            print(_dim(f"[DASHBOARD] Holding live server for {hold_s:.1f}s so the browser can pull the final analysis summary..."))
            time.sleep(hold_s)
    if dashboard_server is not None:
        print(_dim(f"[DASHBOARD] Served live during the run at {dashboard_server.url}"))
        dashboard_server.stop()


def cmd_dashboard(args):
    session_dir = os.path.abspath(args.session_dir)
    radar_tracker = _CSVAppendTracker(os.path.join(session_dir, "radar.csv"), keep_rows=480)
    ref_tracker = _CSVAppendTracker(os.path.join(session_dir, "ref.csv"), keep_rows=480)
    raw_tracker = _CSVAppendTracker(os.path.join(session_dir, "ref_ble_raw.csv"), keep_rows=480)
    recent_events: Deque[str] = deque(["[INFO] Watching existing CSV files"], maxlen=24)
    event_lock = threading.Lock()
    _write_live_dashboard_html(session_dir)
    dashboard_server = None
    try:
        dashboard_server = _start_dashboard_server(session_dir, port=int(getattr(args, "dashboard_port", 0) or 0))
        print(_green(f"[DASHBOARD] {dashboard_server.url}"))
        if getattr(args, "open_dashboard", True):
            try:
                import webbrowser
                webbrowser.open(dashboard_server.url)
            except Exception as e:
                warn(f"Could not open dashboard in browser: {e}")
    except Exception as e:
        warn(f"Could not start dashboard server: {e}")
    start_t = time.time()
    try:
        while True:
            time.sleep(max(0.15, float(args.dashboard_refresh_s)))
            radar_tracker.poll(); ref_tracker.poll(); raw_tracker.poll()
            _render_live_dashboard(session_dir, start_t, args.duration_s, radar_tracker, ref_tracker, raw_tracker,
                                   recent_events, event_lock, "watching", analysis_summary=_dashboard_analysis_payload(session_dir))
            if args.duration_s and (time.time() - start_t) >= args.duration_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if dashboard_server is not None:
            dashboard_server.stop()


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 0: DOCTOR & QUICKSTART
# ─────────────────────────────────────────────────────────────────────────────

_QUICKSTART_TEXT = """
╔══════════════════════════════════════════════════════════════════╗
║     Radar Vital Trainer v{ver} — Quick-Start Guide              ║
╚══════════════════════════════════════════════════════════════════╝

RECOMMENDED THESIS WORKFLOW (single command per session)

STEP 0 — Check your setup
  python radar_vital_trainer_v8_4_3_for_v13_8_0.py doctor

STEP 1 — Run one full session
  python radar_vital_trainer_v8_4_3_for_v13_8_0.py session \
    --port COM10 \
    --address 10:22:33:9E:8F:63 \
    --duration-s 480 \
    --open-dashboard

  What this does automatically:
    • starts radar logging
    • starts BLE oximeter reference logging
    • starts a localhost dashboard server
    • opens the Material 3 Expressive live dashboard in your browser
    • stops both collectors together
    • runs analyse immediately after capture
    • prints a quick session report right away

  Session tips:
    • Sit still and face the radar chest-on
    • Use a fixed distance (recommended default: ~60 cm)
    • Keep the pulse oximeter on the same finger each run
    • Avoid fans, talking, and large body motion
    • Start with 8-minute sessions (480 s)

STEP 2 — Review the outputs
  In the new session folder you will get:
    • radar.csv
    • ref.csv
    • ref_ble_raw.csv
    • ref_ble_summary.json
    • live_dashboard.html
    • live_dashboard.json
    • analysis/analyse_summary.json
    • analysis/analyse_report.txt
    • analysis/analyse_report.html
    • session_quick_report.txt

STEP 3 — Compare sessions after you collect several runs
  python radar_vital_trainer_v8_4_3_for_v13_8_0.py compare \
    --sessions-dir sessions/ --out report.html

STEP 4 — Train a correction model after baseline quality is acceptable
  python radar_vital_trainer_v8_4_3_for_v13_8_0.py train \
    --radar sessions/s01/radar.csv --ref sessions/s01/ref.csv \
    --feature-mode core --require-baseline-gate \
    --out model_s01/

ADVANCED / MANUAL WORKFLOW

Manual radar logging:
  python radar_vital_trainer_v8_4_3_for_v13_8_0.py log --port COM10 --out sessions/s01/radar.csv

Manual BLE reference logging:
  python radar_vital_trainer_v8_4_3_for_v13_8_0.py ble_reflog \
    --out sessions/s01/ref.csv \
    --address 10:22:33:9E:8F:63 \
    --ble-profile ailink_oximeter \
    --notify-char 0000ffe2-0000-1000-8000-00805f9b34fb

Manual analysis:
  python radar_vital_trainer_v8_4_3_for_v13_8_0.py analyse \
    --radar sessions/s01/radar.csv --ref sessions/s01/ref.csv \
    --out sessions/s01/analysis/

DSP sweep:
  python radar_vital_trainer_v8_4_3_for_v13_8_0.py sweep \
    --sweep-json params_sweep.json \
    --port COM10 --sketch path/to/radar_vital_v11.ino \
    --out-dir sweep_results/

Run  python radar_vital_trainer_v8_4_3_for_v13_8_0.py doctor  to verify your environment.
For BLE pulse oximeter logging, also install:  pip install bleak
""".format(ver=VERSION)


def cmd_quickstart(args):
    print(_QUICKSTART_TEXT)


def cmd_doctor(args):
    print(_bold(f"\n=== Radar Vital Trainer v{VERSION} — Dependency Check ===\n"))
    ok_all = True

    def check_pkg(name, import_name=None, extra=""):
        nonlocal ok_all
        mod = importlib.util.find_spec(import_name or name)
        if mod is not None:
            try:
                m = __import__(import_name or name)
                ver = getattr(m, "__version__", "?")
            except Exception:
                ver = "?"
            print(f"  {_green('[OK]')}  {name:<20} {_dim(ver)}")
            return True
        else:
            print(f"  {_red('[MISSING]')}  {name:<20} {_yellow('pip install ' + name)}{extra}")
            ok_all = False
            return False

    print(_bold("Core (required for analyse / train / predict):"))
    check_pkg("numpy")
    check_pkg("pandas")
    check_pkg("scipy")
    check_pkg("scikit-learn", "sklearn")
    check_pkg("matplotlib")

    print(_bold("\nSerial (required for log / reflog / flash / sweep):"))
    check_pkg("pyserial", "serial")

    print(_bold("\nHardware (required for flash / sweep):"))
    cli_path = shutil.which("arduino-cli")
    if cli_path:
        try:
            result = subprocess.run(["arduino-cli", "version"], capture_output=True, text=True, timeout=5)
            ver_line = result.stdout.strip().split("\n")[0] if result.stdout else "?"
        except Exception:
            ver_line = "found"
        print(f"  {_green('[OK]')}  {'arduino-cli':<20} {_dim(ver_line)}")
    else:
        print(f"  {_red('[MISSING]')}  {'arduino-cli':<20} "
              f"{_yellow('https://arduino.github.io/arduino-cli/latest/installation/')}")
        print(f"            {'':20} After installing: arduino-cli core install esp32:esp32")
        ok_all = False

    print(_bold("\nOptional (for TFLite embedded export):"))
    mod = importlib.util.find_spec("tensorflow")
    if mod:
        try:
            import tensorflow as tf
            print(f"  {_green('[OK]')}  {'tensorflow':<20} {_dim(tf.__version__)}")
        except Exception:
            print(f"  {_green('[OK]')}  {'tensorflow':<20} {_dim('(version unknown)')}")
    else:
        print(f"  {_dim('[--]')}  {'tensorflow':<20} not installed  "
              f"{_dim('(optional — pip install tensorflow)')}")

    print()
    if ok_all:
        print(_green("All required dependencies satisfied. You are ready to go!"))
        print(f"Run  {_bold('python radar_vital_trainer_v8_4_3_for_v13_8_0.py quickstart')}  for the workflow guide.\n")
    else:
        print(_red("Some required dependencies are missing. Install them and re-run doctor.\n"))


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 1: LIVE LOGGING
# ─────────────────────────────────────────────────────────────────────────────

def cmd_log(args):
    start_wall = time.time()
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not found. Run: pip install pyserial")

    cols = list(RADAR_LOG_COLUMNS)
    contract_path = _assert_radar_log_contract()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    print(f"[LOG] Firmware contract OK: {contract_path.name} ({len(cols)} columns)")
    print(f"[LOG] Opening {args.port} @ 115200 baud")
    print(f"[LOG] Writing to {args.out}")
    print("[LOG] Waiting for first DATA frame... (Ctrl-C to stop)\n")
    accepted = 0
    dropped = 0
    header_lines = 0
    first_sample = False
    legacy_warned = False
    with serial.Serial(args.port, 115200, timeout=1) as ser, \
         open(args.out, "w", encoding="utf-8") as f:
        f.write(",".join(cols) + "\n")
        try:
            while True:
                if getattr(args, "duration_s", None) and (time.time() - start_wall) >= float(args.duration_s):
                    print(f"\n[LOG] Auto-stop reached at {float(args.duration_s):.1f}s")
                    break
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("SESSION_SUMMARY") or line.startswith("SESSION,"):
                    print(f"  {_dim(line)}")
                    continue
                kind, row, detail = _parse_radar_data_line(line, cols)
                if kind == "skip":
                    continue
                if kind == "header":
                    header_lines += 1
                    continue

                if kind == "data" and row is not None:
                    pad_len = EXPECTED_RADAR_LOG_COLUMN_COUNT - LEGACY_RADAR_LOG_COLUMN_COUNT
                    if (not legacy_warned) and len(row) == len(cols) and row[-pad_len:] == [""] * pad_len:
                        print(_yellow(f"[LOG] Legacy 131-column firmware detected; padding {pad_len} anchor telemetry fields with blanks."))
                        legacy_warned = True
                    f.write(",".join(row) + "\n")
                    f.flush()
                    accepted += 1
                    if not first_sample:
                        print(_green(f"[LOG] First sample received — logging active."))
                        first_sample = True
                    if accepted % 100 == 0:
                        print(f"[LOG] {accepted} frames logged...", end="\r")
                else:
                    dropped += 1
                    if dropped <= 5:
                        print(f"[LOG] Dropped malformed DATA line ({detail})")
        except KeyboardInterrupt:
            print(f"\n[LOG] Stopped.")
            print(f"[LOG] Accepted: {_green(str(accepted))} frames")
            if header_lines:
                print(f"[LOG] Skipped firmware header lines: {header_lines}")
            if dropped:
                print(f"[LOG] Dropped:  {_yellow(str(dropped))} malformed lines")
            print(f"[LOG] Saved to: {args.out}")


def cmd_reflog(args):
    """
    Live keyboard-driven reference logger.
    Run this in a second terminal alongside `log`.

    Usage:
      Type  HR=72  then Enter  to log a heart rate reading.
      Type  RR=14  then Enter  to log a respiration rate reading.
      Press Ctrl-C to stop and save.
    """
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    print(_bold("\n=== Reference Logger ==="))
    print(f"Output: {args.out}")
    print()
    print("Type readings and press Enter:")
    print(f"  {_cyan('HR=72')}  -> log heart rate")
    print(f"  {_cyan('RR=14')}  -> log respiration rate")
    print(f"  {_cyan('Ctrl-C')} -> stop and save\n")
    print(_dim("TIP: Log HR every 30 seconds, RR every 60 seconds.\n"))

    start_ms = int(time.time() * 1000)
    rows: List[Dict] = []

    try:
        while True:
            try:
                line = input("> ").strip().upper().replace(" ", "")
            except EOFError:
                break
            t = int(time.time() * 1000) - start_ms
            elapsed = t / 1000.0

            if line.startswith("HR="):
                try:
                    val = float(line[3:])
                    if not (30 <= val <= 220):
                        print(_yellow(f"  Ignored — HR {val:.0f} out of range [30–220]"))
                        continue
                    rows.append({"timestamp_ms": t, "ref_hr": val, "ref_rr": ""})
                    print(_green(f"  t={elapsed:.1f}s  HR={val:.0f} BPM logged"))
                except ValueError:
                    print(_red(f"  Could not parse: {line}  (example: HR=72)"))

            elif line.startswith("RR="):
                try:
                    val = float(line[3:])
                    if not (4 <= val <= 60):
                        print(_yellow(f"  Ignored — RR {val:.0f} out of range [4–60]"))
                        continue
                    rows.append({"timestamp_ms": t, "ref_hr": "", "ref_rr": val})
                    print(_green(f"  t={elapsed:.1f}s  RR={val:.0f} br/min logged"))
                except ValueError:
                    print(_red(f"  Could not parse: {line}  (example: RR=14)"))

            elif line in ("Q", "QUIT", "EXIT"):
                break
            elif line == "":
                pass
            else:
                print(_dim(f"  Unknown: '{line}'  (use HR=<number> or RR=<number>)"))

    except KeyboardInterrupt:
        pass

    if not rows:
        print(_yellow("\n[REFLOG] No readings captured."))
        return

    import csv
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["timestamp_ms", "ref_hr", "ref_rr"])
        writer.writeheader()
        writer.writerows(rows)

    hr_rows = [r for r in rows if r["ref_hr"] != ""]
    rr_rows = [r for r in rows if r["ref_rr"] != ""]
    duration_s = (rows[-1]["timestamp_ms"] - rows[0]["timestamp_ms"]) / 1000 if len(rows) > 1 else 0

    print(f"\n{_green('[REFLOG] Saved')} {len(rows)} readings to {args.out}")
    print(f"  HR readings: {len(hr_rows)}")
    print(f"  RR readings: {len(rr_rows)}")
    print(f"  Duration:    {duration_s:.0f}s")
    if hr_rows:
        hr_vals = [float(r["ref_hr"]) for r in hr_rows]
        print(f"  HR range:    {min(hr_vals):.0f} – {max(hr_vals):.0f} BPM "
              f"(mean {sum(hr_vals)/len(hr_vals):.1f})")


def _normalize_ble_address(addr: Optional[str]) -> Optional[str]:
    if addr is None:
        return None
    s = str(addr).strip().upper()
    if not s:
        return None
    m = re.search(r"DEV[_-]?([0-9A-F]{12})", s)
    if m:
        s = m.group(1)
    s = re.sub(r"[^0-9A-F]", "", s)
    if len(s) == 12:
        return ":".join(s[i:i+2] for i in range(0, 12, 2))
    return addr


def _pick_notify_characteristics(services, preferred_uuid: Optional[str] = None) -> List[str]:
    preferred = preferred_uuid.lower() if preferred_uuid else None
    notify_chars: List[str] = []

    def score_uuid(uuid: str, props: set) -> Tuple[int, str]:
        u = uuid.lower()
        score = 0

        # Strongly prefer explicitly requested UUID.
        if preferred and u == preferred:
            score += 10000
        elif preferred and u.endswith(preferred.replace('-', '').lower()):
            score += 9000

        # Known common data-stream candidates.
        if u == "6e400003-b5a3-f393-e0a9-e50e24dcca9e":
            score += 8000
        if u == "0000ffe2-0000-1000-8000-00805f9b34fb":
            score += 7000
        if u == "0000ffe3-0000-1000-8000-00805f9b34fb":
            score += 6500
        if u == "49535343-1e4d-4bd9-ba61-23c647249616":
            score += 7500
        if u == "49535343-aca3-481c-91ec-d85e28a60318":
            score += 6000

        # Prefer vendor-specific / custom UUIDs over standard GATT housekeeping.
        if u.startswith("49535343-"):
            score += 4000
        if u.startswith("0000ffe"):
            score += 3500

        # Prefer notify over indicate-only.
        if "notify" in props:
            score += 500
        if "indicate" in props:
            score += 100

        # Penalize standard characteristics that are usually not the live vital stream.
        if u == "00002a05-0000-1000-8000-00805f9b34fb":  # Service Changed
            score -= 100000
        if u == "00002a19-0000-1000-8000-00805f9b34fb":  # Battery Level
            score -= 5000
        if u == "00002a07-0000-1000-8000-00805f9b34fb":  # Tx Power Level
            score -= 5000

        return score, u

    for service in services:
        for char in getattr(service, "characteristics", []):
            uuid = str(getattr(char, "uuid", ""))
            props = {str(p).lower() for p in getattr(char, "properties", [])}
            if "notify" in props or "indicate" in props:
                notify_chars.append(uuid)

    if not notify_chars:
        return []

    ranked = sorted(notify_chars, key=lambda u: score_uuid(u, set()), reverse=True)
    # Re-score with actual props preserved to break ties correctly.
    scored = []
    for service in services:
        for char in getattr(service, "characteristics", []):
            uuid = str(getattr(char, "uuid", ""))
            props = {str(p).lower() for p in getattr(char, "properties", [])}
            if uuid in notify_chars:
                scored.append((score_uuid(uuid, props), uuid))
    ordered = [uuid for _, uuid in sorted(scored, key=lambda x: x[0], reverse=True)]

    deduped: List[str] = []
    seen = set()
    for uuid in ordered:
        if uuid not in seen:
            deduped.append(uuid)
            seen.add(uuid)
    return deduped



def _parse_ble_vitals_packet(
    data: bytes,
    hr_byte_index: Optional[int] = None,
    spo2_byte_index: Optional[int] = None,
    rr_byte_index: Optional[int] = None,
    pi_byte_index: Optional[int] = None,
    hr_scale: float = 1.0,
    spo2_scale: float = 1.0,
    rr_scale: float = 1.0,
    pi_scale: float = 1.0,
    last_hr: Optional[float] = None,
    allow_heuristic: bool = False,
) -> Tuple[
    Optional[float], Optional[float], Optional[float], Optional[float],
    Optional[int], Optional[int], Optional[int], Optional[int]
]:
    hr = rr = spo2 = pi = None
    hr_idx = spo2_idx = rr_idx = pi_idx = None
    payload = list(data)

    def pick_value(index: Optional[int], scale: float, valid_min: float, valid_max: float):
        if index is None:
            return None, None
        if index < 0 or index >= len(payload):
            return None, None
        raw_val = int(payload[index])
        val = float(raw_val) * float(scale)
        if valid_min <= val <= valid_max:
            return float(val), index
        return None, index

    hr, hr_idx = pick_value(hr_byte_index, hr_scale, 30, 220)
    rr, rr_idx = pick_value(rr_byte_index, rr_scale, 4, 60)
    spo2, spo2_idx = pick_value(spo2_byte_index, spo2_scale, 70, 100)
    pi, pi_idx = pick_value(pi_byte_index, pi_scale, 0, 40)

    if allow_heuristic and hr is None:
        candidates = []
        seen = set()
        priority = []
        if len(payload) > 4:
            priority.extend([4, 5, 3])
        if len(payload) > 8:
            priority.extend([8, 9, 10])
        if len(payload) > 1:
            priority.append(1)
        if len(payload) > 0:
            priority.append(len(payload) - 1)
        for idx in priority:
            if idx in seen or idx < 0 or idx >= len(payload):
                continue
            seen.add(idx)
            val = float(int(payload[idx]))
            if 30 <= val <= 220:
                candidates.append((idx, val))
        if candidates:
            if last_hr is not None:
                hr_idx, hr = min(candidates, key=lambda p: abs(p[1] - last_hr))
            else:
                hr_idx, hr = candidates[0]

    if allow_heuristic and spo2 is None:
        for idx, val in enumerate(payload):
            if 70 <= int(val) <= 100:
                spo2 = float(val)
                spo2_idx = idx
                break

    return hr, rr, spo2, pi, hr_idx, rr_idx, spo2_idx, pi_idx


def _hex_to_bytes(raw_hex: str) -> Optional[bytes]:
    try:
        return bytes.fromhex(str(raw_hex).strip())
    except Exception:
        return None


def _prepare_ble_write_ops(write_chars: Optional[Sequence[str]], write_hexes: Optional[Sequence[str]]) -> List[Tuple[str, bytes]]:
    chars = [str(x).strip() for x in (write_chars or []) if str(x).strip()]
    hexes = [str(x).strip() for x in (write_hexes or []) if str(x).strip()]
    if not chars and not hexes:
        return []
    if len(chars) == 1 and len(hexes) > 1:
        chars = chars * len(hexes)
    if len(chars) != len(hexes):
        raise ValueError("--write-char and --write-hex must have equal counts, or provide one --write-char with multiple --write-hex values.")
    ops = []
    for char_uuid, hex_str in zip(chars, hexes):
        cleaned = re.sub(r"[^0-9A-Fa-f]", "", hex_str)
        if len(cleaned) % 2 != 0:
            raise ValueError(f"Write payload must contain whole bytes: {hex_str}")
        try:
            payload = bytes.fromhex(cleaned)
        except ValueError as e:
            raise ValueError(f"Invalid --write-hex payload '{hex_str}': {e}") from e
        ops.append((char_uuid, payload))
    return ops


def _apply_ble_profile_defaults(args):
    """
    Apply BLE profile defaults without clobbering profile-specific scales.

    v7.2.8e keeps the BLE/dashboard truthfulness fixes, corrects the remaining HR/RR gate-reason enum mappings, adds a non-gating physiological ratio review warning, and keeps nearest alignment as the default so stricter backward matching stays opt-in
    before the profile block ran, which prevented the AILink oximeter profile
    from applying its intended RR/PI scale of 0.1 unless the user overrode them
    explicitly on the command line.
    """
    profile = str(getattr(args, "ble_profile", "none") or "none").strip().lower()

    if profile == "ailink_oximeter":
        if not getattr(args, "notify_char", None):
            args.notify_char = "0000ffe2-0000-1000-8000-00805f9b34fb"
        if getattr(args, "hr_byte_index", None) is None:
            args.hr_byte_index = 7
        if getattr(args, "spo2_byte_index", None) is None:
            args.spo2_byte_index = 6
        if getattr(args, "rr_byte_index", None) is None:
            args.rr_byte_index = 11
        if getattr(args, "pi_byte_index", None) is None:
            args.pi_byte_index = 8
        if getattr(args, "hr_scale", None) is None:
            args.hr_scale = 1.0
        if getattr(args, "spo2_scale", None) is None:
            args.spo2_scale = 1.0
        if getattr(args, "rr_scale", None) is None:
            args.rr_scale = 0.1
        if getattr(args, "pi_scale", None) is None:
            args.pi_scale = 0.1

    if getattr(args, "hr_scale", None) is None:
        args.hr_scale = 1.0
    if getattr(args, "spo2_scale", None) is None:
        args.spo2_scale = 1.0
    if getattr(args, "rr_scale", None) is None:
        args.rr_scale = 1.0
    if getattr(args, "pi_scale", None) is None:
        args.pi_scale = 1.0

    return args


def _fmt_ref_value(value: Optional[float], decimals_if_needed: int = 1) -> str:
    if value is None or not np.isfinite(value):
        return ""
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{float(value):.{int(decimals_if_needed)}f}"


def _ble_sender_column(df: pd.DataFrame) -> str:
    if "source_uuid" in df.columns:
        return "source_uuid"
    if "sender" in df.columns:
        return "sender"
    raise ValueError("Raw BLE CSV must contain source_uuid or sender column")


def _score_ble_byte_candidate(row: pd.Series, target: str) -> float:
    unique_count = float(row.get("unique_count", 0.0) or 0.0)
    change_frac = float(row.get("change_frac", 0.0) or 0.0)
    dominant_frac = float(row.get("dominant_frac", 1.0) or 1.0)
    if target == "hr":
        frac = float(row.get("frac_in_hr_range", 0.0) or 0.0)
        center_bonus = float(row.get("frac_in_hr_core", 0.0) or 0.0)
        score = 100.0 * frac + 20.0 * center_bonus + 8.0 * min(unique_count, 20.0) + 30.0 * change_frac - 40.0 * dominant_frac
    elif target == "rr":
        frac = float(row.get("frac_in_rr_range", 0.0) or 0.0)
        scaled = float(row.get("frac_u16le_div10_in_rr_range", 0.0) or row.get("frac_div10_in_rr_range", 0.0) or 0.0)
        score = 100.0 * frac + 80.0 * scaled + 10.0 * min(unique_count, 12.0) + 30.0 * change_frac - 35.0 * dominant_frac
    else:
        frac = float(row.get("frac_in_spo2_range", 0.0) or 0.0)
        score = 100.0 * frac + 6.0 * min(unique_count, 8.0) + 10.0 * change_frac - 20.0 * dominant_frac
    if unique_count <= 1.0:
        score -= 50.0
    return float(score)


def _build_ble_byte_candidates(raw_df: pd.DataFrame) -> pd.DataFrame:
    sender_col = _ble_sender_column(raw_df)
    rows: List[Dict[str, object]] = []
    work = raw_df.copy()
    work["_payload"] = work["raw_hex"].map(_hex_to_bytes)
    work = work[work["_payload"].notna()].copy()
    if len(work) == 0:
        return pd.DataFrame()

    for sender, g in work.groupby(sender_col, sort=False):
        payloads = list(g["_payload"])
        max_len = max(len(p) for p in payloads)
        for idx in range(max_len):
            vals = [p[idx] for p in payloads if idx < len(p)]
            if not vals:
                continue
            s = pd.Series(vals, dtype=float)
            diffs = s.diff().fillna(0.0)
            vc = s.value_counts(dropna=False)
            dominant_val = float(vc.index[0]) if len(vc) else float("nan")
            dominant_frac = float(vc.iloc[0] / len(s)) if len(vc) else float("nan")
            row = {
                "source_uuid": sender,
                "field_type": "u8",
                "index": int(idx),
                "n": int(len(s)),
                "unique_count": int(s.nunique(dropna=True)),
                "min": float(s.min()),
                "max": float(s.max()),
                "mean": float(s.mean()),
                "std": float(s.std(ddof=0)) if len(s) > 1 else 0.0,
                "change_frac": float((diffs != 0).mean()) if len(s) > 1 else 0.0,
                "dominant_value": dominant_val,
                "dominant_frac": dominant_frac,
                "frac_in_hr_range": float(((s >= 30) & (s <= 220)).mean()),
                "frac_in_hr_core": float(((s >= 45) & (s <= 160)).mean()),
                "frac_in_rr_range": float(((s >= 4) & (s <= 60)).mean()),
                "frac_in_spo2_range": float(((s >= 70) & (s <= 100)).mean()),
            }
            row["score_hr"] = _score_ble_byte_candidate(pd.Series(row), "hr")
            row["score_rr"] = _score_ble_byte_candidate(pd.Series(row), "rr")
            row["score_spo2"] = _score_ble_byte_candidate(pd.Series(row), "spo2")
            rows.append(row)
    return pd.DataFrame(rows)


def _build_ble_word_candidates(raw_df: pd.DataFrame) -> pd.DataFrame:
    sender_col = _ble_sender_column(raw_df)
    rows: List[Dict[str, object]] = []
    work = raw_df.copy()
    work["_payload"] = work["raw_hex"].map(_hex_to_bytes)
    work = work[work["_payload"].notna()].copy()
    if len(work) == 0:
        return pd.DataFrame()

    for sender, g in work.groupby(sender_col, sort=False):
        payloads = list(g["_payload"])
        max_len = max(len(p) for p in payloads)
        for idx in range(max_len - 1):
            vals_le = [p[idx] + 256 * p[idx + 1] for p in payloads if idx + 1 < len(p)]
            vals_be = [256 * p[idx] + p[idx + 1] for p in payloads if idx + 1 < len(p)]
            for endian, vals in [("u16le", vals_le), ("u16be", vals_be)]:
                if not vals:
                    continue
                s = pd.Series(vals, dtype=float)
                diffs = s.diff().fillna(0.0)
                vc = s.value_counts(dropna=False)
                dominant_val = float(vc.index[0]) if len(vc) else float("nan")
                dominant_frac = float(vc.iloc[0] / len(s)) if len(vc) else float("nan")
                s_div10 = s / 10.0
                row = {
                    "source_uuid": sender,
                    "field_type": endian,
                    "index": int(idx),
                    "n": int(len(s)),
                    "unique_count": int(s.nunique(dropna=True)),
                    "min": float(s.min()),
                    "max": float(s.max()),
                    "mean": float(s.mean()),
                    "std": float(s.std(ddof=0)) if len(s) > 1 else 0.0,
                    "change_frac": float((diffs != 0).mean()) if len(s) > 1 else 0.0,
                    "dominant_value": dominant_val,
                    "dominant_frac": dominant_frac,
                    "frac_u16_in_hr_range": float(((s >= 30) & (s <= 220)).mean()),
                    "frac_u16_div10_in_hr_range": float(((s_div10 >= 30) & (s_div10 <= 220)).mean()),
                    "frac_u16_in_rr_range": float(((s >= 4) & (s <= 60)).mean()),
                    "frac_u16_div10_in_rr_range": float(((s_div10 >= 4) & (s_div10 <= 60)).mean()),
                    "frac_u16_in_spo2_range": float(((s >= 70) & (s <= 100)).mean()),
                }
                hr_score = 100.0 * row["frac_u16_in_hr_range"] + 80.0 * row["frac_u16_div10_in_hr_range"] + 8.0 * min(row["unique_count"], 20) + 25.0 * row["change_frac"] - 35.0 * row["dominant_frac"]
                rr_score = 100.0 * row["frac_u16_in_rr_range"] + 100.0 * row["frac_u16_div10_in_rr_range"] + 10.0 * min(row["unique_count"], 15) + 25.0 * row["change_frac"] - 35.0 * row["dominant_frac"]
                spo2_score = 100.0 * row["frac_u16_in_spo2_range"] + 5.0 * min(row["unique_count"], 8) + 10.0 * row["change_frac"] - 20.0 * row["dominant_frac"]
                row["score_hr"] = float(hr_score)
                row["score_rr"] = float(rr_score)
                row["score_spo2"] = float(spo2_score)
                rows.append(row)
    return pd.DataFrame(rows)


def cmd_ble_analyse_raw(args):
    os.makedirs(args.out, exist_ok=True)
    raw_df = pd.read_csv(args.raw)
    sender_col = _ble_sender_column(raw_df)
    if args.sender:
        raw_df = raw_df[raw_df[sender_col].astype(str) == str(args.sender)].copy()
    if len(raw_df) == 0:
        raise ValueError("No raw BLE packets matched the requested filter.")

    byte_df = _build_ble_byte_candidates(raw_df)
    word_df = _build_ble_word_candidates(raw_df)

    if len(byte_df):
        byte_df = byte_df.sort_values(["source_uuid", "score_rr", "score_hr", "score_spo2"], ascending=[True, False, False, False]).reset_index(drop=True)
        byte_df.to_csv(os.path.join(args.out, "byte_candidates.csv"), index=False)
    if len(word_df):
        word_df = word_df.sort_values(["source_uuid", "score_rr", "score_hr", "score_spo2"], ascending=[True, False, False, False]).reset_index(drop=True)
        word_df.to_csv(os.path.join(args.out, "word_candidates.csv"), index=False)

    top_summary = {"bytes": {}, "words": {}}
    for name, df, key in [("bytes", byte_df, "index"), ("words", word_df, "index")]:
        if df is None or len(df) == 0:
            continue
        for sender, g in df.groupby("source_uuid", sort=False):
            top_summary[name][sender] = {
                "hr": g.sort_values("score_hr", ascending=False).head(int(args.top_k)).to_dict(orient="records"),
                "rr": g.sort_values("score_rr", ascending=False).head(int(args.top_k)).to_dict(orient="records"),
                "spo2": g.sort_values("score_spo2", ascending=False).head(int(args.top_k)).to_dict(orient="records"),
            }

    summary = {
        "version": VERSION,
        "raw_csv": os.path.abspath(args.raw),
        "rows": int(len(raw_df)),
        "source_column": sender_col,
        "sources": [str(x) for x in pd.Series(raw_df[sender_col]).dropna().astype(str).unique().tolist()],
        "top_k": int(args.top_k),
        "top_summary": nan_safe(top_summary),
    }
    save_json(summary, os.path.join(args.out, "ble_raw_analysis_summary.json"))

    print(_bold("\n=== BLE RAW ANALYSIS ==="))
    print(f"Rows: {len(raw_df)}")
    print(f"Sources: {', '.join(summary['sources'])}")
    if len(byte_df):
        print(_bold("\nTop byte candidates by source:"))
        for sender, g in byte_df.groupby("source_uuid", sort=False):
            print(f"  {sender}")
            top_hr = g.sort_values("score_hr", ascending=False).head(int(args.top_k))
            top_rr = g.sort_values("score_rr", ascending=False).head(int(args.top_k))
            print("    HR:")
            for _, row in top_hr.iterrows():
                print(f"      idx={int(row['index'])} score={row['score_hr']:.1f} unique={int(row['unique_count'])} hr_frac={row['frac_in_hr_range']:.2f} dom={row['dominant_frac']:.2f}")
            print("    RR:")
            for _, row in top_rr.iterrows():
                print(f"      idx={int(row['index'])} score={row['score_rr']:.1f} unique={int(row['unique_count'])} rr_frac={row['frac_in_rr_range']:.2f} dom={row['dominant_frac']:.2f}")
    if len(word_df):
        print(_bold("\nTop 16-bit candidates by source:"))
        for sender, g in word_df.groupby("source_uuid", sort=False):
            print(f"  {sender}")
            top_rr = g.sort_values("score_rr", ascending=False).head(int(args.top_k))
            for _, row in top_rr.iterrows():
                print(f"    {row['field_type']} @ {int(row['index'])}: score={row['score_rr']:.1f} rr_frac={row['frac_u16_in_rr_range']:.2f} rr_div10_frac={row['frac_u16_div10_in_rr_range']:.2f} unique={int(row['unique_count'])}")
    print(f"\n[OUT] Saved raw BLE field analysis to {args.out}")


def cmd_ble_reflog(args):
    """
    Automated BLE reference logger for pulse oximeters.

    Writes the standard reference CSV expected by align/analyse/train, while also
    saving raw packets for protocol reverse-engineering. In v6.2 this command can
    subscribe to all notify-capable characteristics, issue optional initialization
    writes, and run in conservative raw-only mode to avoid fabricating HR/RR fields.
    """
    try:
        import asyncio
        from bleak import BleakScanner, BleakClient
    except ImportError:
        sys.exit("The 'bleak' library is required for BLE logging. Run: pip install bleak")

    import csv

    args = _apply_ble_profile_defaults(args)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    raw_out = args.raw_out or os.path.join(
        os.path.dirname(os.path.abspath(args.out)),
        os.path.splitext(os.path.basename(args.out))[0] + "_ble_raw.csv")
    summary_out = args.summary_out or os.path.join(
        os.path.dirname(os.path.abspath(args.out)),
        os.path.splitext(os.path.basename(args.out))[0] + "_ble_summary.json")

    start_ms = int(time.time() * 1000)
    parsed_count = 0
    raw_count = 0
    last_logged_hr: Optional[float] = None
    last_logged_rr: Optional[float] = None
    last_log_ms = -10**12
    write_ops = _prepare_ble_write_ops(args.write_char, args.write_hex)

    name_hints = [h.strip() for h in (args.name_hint or []) if str(h).strip()] or ["Oxy", "PC-60", "Wilcare", "Oximeter"]
    preferred_notify = args.notify_char or "0000ffe2-0000-1000-8000-00805f9b34fb"
    normalized_address = _normalize_ble_address(args.address)

    print(_bold("\n=== BLE Reference Logger ==="))
    print(f"Reference CSV: {args.out}")
    print(f"Raw packet log: {raw_out}")
    print(f"Summary JSON:   {summary_out}")
    if args.raw_only:
        print(_yellow("Raw-only mode enabled: the trainer will not invent HR/RR from unknown packet bytes."))
    elif not args.auto_parse and args.hr_byte_index is None and args.rr_byte_index is None and args.spo2_byte_index is None and args.pi_byte_index is None:
        print(_yellow("No explicit byte indices supplied. Parsed CSV will stay sparse unless you enable --auto-parse."))
    if str(getattr(args, "ble_profile", "none") or "none").lower() != "none":
        print(_dim(f"BLE profile: {args.ble_profile}"))
    print(_dim("Press Ctrl-C to stop. Keep the oximeter awake with a finger inserted.\n"))

    parsed_f = open(args.out, "w", newline="", encoding="utf-8")
    raw_f = open(raw_out, "w", newline="", encoding="utf-8")
    parsed_writer = csv.DictWriter(parsed_f, fieldnames=[
        "timestamp_ms", "source_uuid", "ref_hr", "ref_rr", "ref_spo2", "ref_pi",
        "hr_byte_index", "rr_byte_index", "spo2_byte_index", "pi_byte_index", "raw_hex",
    ])
    raw_writer = csv.DictWriter(raw_f, fieldnames=[
        "timestamp_ms", "source_uuid", "sender", "packet_len", "raw_hex"
    ])
    parsed_writer.writeheader()
    raw_writer.writeheader()
    parsed_f.flush()
    raw_f.flush()

    stats: Dict[str, Dict[str, object]] = {}
    parse_source_uuid: Optional[str] = None
    active_notify_chars: List[str] = []

    def record_stat(source_uuid: str, payload: bytes, parsed: bool):
        ent = stats.setdefault(source_uuid, {
            "packet_count": 0,
            "parsed_count": 0,
            "packet_lens": {},
            "first_timestamp_ms": None,
            "last_timestamp_ms": None,
        })
        ent["packet_count"] = int(ent["packet_count"]) + 1
        if parsed:
            ent["parsed_count"] = int(ent["parsed_count"]) + 1
        lens = dict(ent["packet_lens"])
        lens[str(len(payload))] = int(lens.get(str(len(payload)), 0)) + 1
        ent["packet_lens"] = lens
        now_ms = int(time.time() * 1000) - start_ms
        if ent["first_timestamp_ms"] is None:
            ent["first_timestamp_ms"] = now_ms
        ent["last_timestamp_ms"] = now_ms

    def make_notification_handler(source_uuid: str):
        def notification_handler(sender, data: bytearray):
            nonlocal parsed_count, raw_count, last_logged_hr, last_logged_rr, last_log_ms, parse_source_uuid
            t_ms = int(time.time() * 1000) - start_ms
            elapsed = t_ms / 1000.0
            payload = bytes(data)
            raw_hex = payload.hex()
            raw_writer.writerow({
                "timestamp_ms": t_ms,
                "source_uuid": source_uuid,
                "sender": str(sender),
                "packet_len": len(payload),
                "raw_hex": raw_hex,
            })
            raw_f.flush()
            raw_count += 1

            should_try_parse = (not args.raw_only) and (parse_source_uuid is None or source_uuid == parse_source_uuid)
            parsed_any = False
            hr = rr = spo2 = pi = None
            hr_idx = rr_idx = spo2_idx = pi_idx = None
            if should_try_parse:
                hr, rr, spo2, pi, hr_idx, rr_idx, spo2_idx, pi_idx = _parse_ble_vitals_packet(
                    payload,
                    hr_byte_index=args.hr_byte_index,
                    spo2_byte_index=args.spo2_byte_index,
                    rr_byte_index=args.rr_byte_index,
                    pi_byte_index=args.pi_byte_index,
                    hr_scale=float(args.hr_scale if args.hr_scale is not None else 1.0),
                    spo2_scale=float(args.spo2_scale if args.spo2_scale is not None else 1.0),
                    rr_scale=float(args.rr_scale if args.rr_scale is not None else 1.0),
                    pi_scale=float(args.pi_scale if args.pi_scale is not None else 1.0),
                    last_hr=last_logged_hr,
                    allow_heuristic=bool(args.auto_parse),
                )
                parsed_any = (hr is not None) or (rr is not None) or (spo2 is not None) or (pi is not None)

            record_stat(source_uuid, payload, parsed_any)

            should_log = parsed_any
            if should_log and args.min_log_interval_ms is not None:
                if (t_ms - last_log_ms) < int(args.min_log_interval_ms):
                    should_log = False
            if should_log and args.dedupe:
                same_hr = (hr is None and last_logged_hr is None) or (hr is not None and last_logged_hr is not None and abs(hr - last_logged_hr) < 1e-9)
                same_rr = (rr is None and last_logged_rr is None) or (rr is not None and last_logged_rr is not None and abs(rr - last_logged_rr) < 1e-9)
                if same_hr and same_rr:
                    should_log = False

            if should_log:
                if parse_source_uuid is None:
                    parse_source_uuid = source_uuid
                parsed_writer.writerow({
                    "timestamp_ms": t_ms,
                    "source_uuid": source_uuid,
                    "ref_hr": _fmt_ref_value(hr, decimals_if_needed=1),
                    "ref_rr": _fmt_ref_value(rr, decimals_if_needed=1),
                    "ref_spo2": _fmt_ref_value(spo2, decimals_if_needed=1),
                    "ref_pi": _fmt_ref_value(pi, decimals_if_needed=1),
                    "hr_byte_index": "" if hr_idx is None else hr_idx,
                    "rr_byte_index": "" if rr_idx is None else rr_idx,
                    "spo2_byte_index": "" if spo2_idx is None else spo2_idx,
                    "pi_byte_index": "" if pi_idx is None else pi_idx,
                    "raw_hex": raw_hex,
                })
                parsed_f.flush()
                parsed_count += 1
                last_log_ms = t_ms
                if hr is not None:
                    last_logged_hr = hr
                if rr is not None:
                    last_logged_rr = rr
                details = []
                if hr is not None:
                    details.append(f"HR={_fmt_ref_value(hr, decimals_if_needed=1)}")
                if rr is not None:
                    details.append(f"RR={_fmt_ref_value(rr, decimals_if_needed=1)}")
                if spo2 is not None:
                    details.append(f"SpO2={_fmt_ref_value(spo2, decimals_if_needed=1)}%")
                if pi is not None:
                    details.append(f"PI={_fmt_ref_value(pi, decimals_if_needed=1)}")
                details_s = "  ".join(details) if details else "parsed packet"
                print(_green(f"  t={elapsed:.1f}s  [{source_uuid}] {details_s}  (raw={raw_hex})"))
            elif args.echo_raw:
                print(_dim(f"  t={elapsed:.1f}s  [{source_uuid}] raw={raw_hex}"))
        return notification_handler

    async def find_device() -> str:
        if normalized_address:
            return normalized_address
        print(f"Scanning for BLE devices for up to {args.scan_timeout:.1f}s...")
        devices = await BleakScanner.discover(timeout=float(args.scan_timeout))
        for d in devices:
            name = getattr(d, "name", None) or ""
            if any(h.lower() in name.lower() for h in name_hints):
                print(_green(f"Found candidate: {name or '(unnamed)'} at {d.address}"))
                return d.address
        raise RuntimeError("BLE oximeter not found. Try --address, or adjust --name-hint.")

    async def run_ble():
        nonlocal parse_source_uuid, active_notify_chars
        target_address = await find_device()
        print(f"Connecting to {target_address}...")
        async with BleakClient(target_address) as client:
            if not client.is_connected:
                raise RuntimeError("BLE connection failed.")
            print(_green("Connected. Enumerating services..."))
            services = client.services
            if not services:
                services = await client.get_services()
            if args.probe_services:
                print(_bold("Available services / characteristics:"))
                for service in services:
                    print(f"  [SERVICE] {service.uuid}")
                    for char in getattr(service, "characteristics", []):
                        props = ",".join(str(p) for p in getattr(char, "properties", []))
                        print(f"    - {char.uuid}  ({props})")
            notify_candidates = _pick_notify_characteristics(services, preferred_uuid=preferred_notify)
            if not notify_candidates:
                raise RuntimeError("No notify-capable BLE characteristic found.")
            if args.probe_services:
                print(_dim("Notify candidates (best first):"))
                for c in notify_candidates:
                    print(_dim(f"  - {c}"))

            subscribe_targets = notify_candidates if args.subscribe_all else notify_candidates[:]
            if not args.subscribe_all:
                subscribe_targets = notify_candidates

            last_notify_error = None
            if args.subscribe_all:
                for candidate in subscribe_targets:
                    try:
                        print(f"Trying notify characteristic: {candidate}")
                        await client.start_notify(candidate, make_notification_handler(candidate))
                        active_notify_chars.append(candidate)
                        print(_green(f"Subscribed: {candidate}"))
                        if parse_source_uuid is None and (args.notify_char is None or candidate.lower() == preferred_notify.lower()):
                            parse_source_uuid = candidate
                    except Exception as e:
                        last_notify_error = e
                        print(_yellow(f"  start_notify failed on {candidate}: {e}"))
                if not active_notify_chars:
                    raise RuntimeError(
                        "Could not start notify on any candidate characteristic. "
                        f"Last error: {last_notify_error}")
                if parse_source_uuid is None:
                    parse_source_uuid = active_notify_chars[0]
                print(_green(f"Using parse source: {parse_source_uuid}"))
            else:
                for candidate in notify_candidates:
                    try:
                        print(f"Trying notify characteristic: {candidate}")
                        await client.start_notify(candidate, make_notification_handler(candidate))
                        active_notify_chars.append(candidate)
                        parse_source_uuid = candidate
                        print(_green(f"Using notify characteristic: {candidate}"))
                        break
                    except Exception as e:
                        last_notify_error = e
                        print(_yellow(f"  start_notify failed on {candidate}: {e}"))
                if not active_notify_chars:
                    raise RuntimeError(
                        "Could not start notify on any candidate characteristic. "
                        f"Last error: {last_notify_error}")

            if write_ops:
                print(_bold("Issuing BLE write operations:"))
                for char_uuid, payload in write_ops:
                    try:
                        await client.write_gatt_char(char_uuid, payload, response=bool(args.write_with_response))
                        print(_green(f"  wrote {payload.hex()} -> {char_uuid}"))
                        if args.post_write_wait_ms and args.post_write_wait_ms > 0:
                            await asyncio.sleep(float(args.post_write_wait_ms) / 1000.0)
                    except Exception as e:
                        print(_yellow(f"  write failed on {char_uuid}: {e}"))

            try:
                if args.duration_s is not None and args.duration_s > 0:
                    end_time = time.time() + float(args.duration_s)
                    while time.time() < end_time:
                        await asyncio.sleep(0.25)
                else:
                    while True:
                        await asyncio.sleep(0.25)
            finally:
                for candidate in reversed(active_notify_chars):
                    try:
                        await client.stop_notify(candidate)
                    except Exception:
                        pass

    error_text = None
    try:
        import asyncio
        asyncio.run(run_ble())
    except KeyboardInterrupt:
        print("\n[BLE REFLOG] Stopped by user.")
    except Exception as e:
        error_text = str(e)
        raise
    finally:
        parsed_f.close()
        raw_f.close()
        summary = {
            "version": VERSION,
            "reference_csv": os.path.abspath(args.out),
            "raw_csv": os.path.abspath(raw_out),
            "parsed_rows": int(parsed_count),
            "raw_packets": int(raw_count),
            "notify_char_requested": args.notify_char,
            "notify_chars_active": active_notify_chars,
            "parse_source_uuid": parse_source_uuid,
            "address": normalized_address or args.address,
            "raw_only": bool(args.raw_only),
            "auto_parse": bool(args.auto_parse),
            "ble_profile": getattr(args, "ble_profile", "none"),
            "field_map": {
                "hr_byte_index": args.hr_byte_index,
                "hr_scale": args.hr_scale,
                "rr_byte_index": args.rr_byte_index,
                "rr_scale": args.rr_scale,
                "spo2_byte_index": args.spo2_byte_index,
                "spo2_scale": args.spo2_scale,
                "pi_byte_index": args.pi_byte_index,
                "pi_scale": args.pi_scale,
            },
            "write_ops": [{"char_uuid": c, "payload_hex": p.hex()} for c, p in write_ops],
            "stats_by_source": nan_safe(stats),
            "error": error_text,
        }
        save_json(summary, summary_out)

    if parsed_count == 0:
        print(_yellow("\n[BLE REFLOG] No parsed HR/RR/SpO2 values were logged."))
        if args.raw_only:
            print(_dim("Raw-only mode intentionally disables parsed reference inference."))
        else:
            print(_yellow("Try --auto-parse for exploration, or run ble_analyse_raw on the raw packet CSV to rank candidate fields."))
        print(_yellow(f"Raw packet capture is still saved to: {raw_out}"))
        return

    duration_s = max(0.0, (int(time.time() * 1000) - start_ms) / 1000.0)
    print(f"\n{_green('[BLE REFLOG] Saved')} {parsed_count} parsed readings to {args.out}")
    print(f"  Raw packets: {raw_count}")
    print(f"  Duration:    {duration_s:.0f}s")


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 1B: FLASH
# ─────────────────────────────────────────────────────────────────────────────

def _check_arduino_cli() -> str:
    """Returns path to arduino-cli or exits with friendly instructions."""
    path = shutil.which("arduino-cli")
    if path:
        return path
    print(_red("\n[FLASH] arduino-cli not found.\n"))
    print("Install it from: https://arduino.github.io/arduino-cli/latest/installation/\n")
    print("Quick install (Linux/Mac):")
    print("  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/"
          "master/install.sh | sh")
    print("\nAfter installing, add the ESP32 core:")
    print("  arduino-cli core update-index")
    print("  arduino-cli core install esp32:esp32")
    print("\nThen re-run this command.\n")
    sys.exit(1)


def _patch_ino_constants(sketch_path: str, params: Dict[str, str], out_path: str):
    """
    Reads sketch_path, replaces constant definitions matching params keys,
    writes result to out_path.

    Matches patterns like:
      const float  RLS_LAMBDA = 0.97f;
      const int    ABSENT_VOTES = 5;
      #define      ABSENT_VOTES 5
    """
    src = open(sketch_path, "r", encoding="utf-8", errors="ignore").read()
    patched = 0
    for key, val in params.items():
        # Match:  const <type> KEY = <old_value>;
        pattern_const = rf"(const\s+\w+\s+{re.escape(key)}\s*=\s*)([^;]+)(;)"
        new_src, n = re.subn(pattern_const, lambda m: f"{m.group(1)}{val}{m.group(3)}", src)
        if n:
            src = new_src
            patched += n
            continue
        # Match:  #define KEY <old_value>
        pattern_define = rf"(#define\s+{re.escape(key)}\s+)(\S+)"
        new_src, n = re.subn(pattern_define, lambda m: f"{m.group(1)}{val}", src)
        if n:
            src = new_src
            patched += n
            continue
        warn(f"Constant '{key}' not found in sketch — skipping")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    open(out_path, "w", encoding="utf-8").write(src)
    print(f"[FLASH] Patched {patched} constant(s) in sketch")
    return patched


def cmd_flash(args):
    _check_arduino_cli()

    # Load params
    if args.params:
        if not os.path.exists(args.params):
            sys.exit(f"[FLASH] params file not found: {args.params}")
        params = json.load(open(args.params, encoding="utf-8"))
        print(f"[FLASH] Loaded {len(params)} param(s) from {args.params}")
    else:
        params = {}
        print("[FLASH] No params file — compiling sketch as-is")

    sketch_path = args.sketch
    if not os.path.exists(sketch_path):
        sys.exit(f"[FLASH] Sketch not found: {sketch_path}")

    # Determine working sketch (patched copy or original)
    if params:
        work_dir = os.path.join(args.build_dir, "patched_sketch")
        os.makedirs(work_dir, exist_ok=True)
        sketch_name = os.path.basename(sketch_path)
        patched_sketch = os.path.join(work_dir, sketch_name)
        _patch_ino_constants(sketch_path, params, patched_sketch)
        compile_sketch = work_dir
    else:
        compile_sketch = os.path.dirname(os.path.abspath(sketch_path))

    build_path = os.path.join(args.build_dir, "build")
    os.makedirs(build_path, exist_ok=True)

    fqbn = args.fqbn

    # Compile
    print(f"\n[FLASH] Compiling for {fqbn} ...")
    compile_cmd = [
        "arduino-cli", "compile",
        "--fqbn", fqbn,
        "--build-path", build_path,
        compile_sketch,
    ]
    if args.verbose:
        compile_cmd.append("--verbose")
    result = subprocess.run(compile_cmd, capture_output=not args.verbose, text=True)
    if result.returncode != 0:
        print(_red("[FLASH] Compilation FAILED"))
        if not args.verbose:
            print(result.stderr[-2000:] if result.stderr else "(no output)")
        sys.exit(1)
    print(_green("[FLASH] Compilation successful"))

    # Upload
    if args.port:
        print(f"[FLASH] Uploading to {args.port} ...")
        upload_cmd = [
            "arduino-cli", "upload",
            "--fqbn", fqbn,
            "--port", args.port,
            "--input-dir", build_path,
        ]
        if args.verbose:
            upload_cmd.append("--verbose")
        result = subprocess.run(upload_cmd, capture_output=not args.verbose, text=True)
        if result.returncode != 0:
            print(_red("[FLASH] Upload FAILED"))
            if not args.verbose:
                print(result.stderr[-2000:] if result.stderr else "(no output)")
            sys.exit(1)
        print(_green(f"[FLASH] Upload successful -> {args.port}"))
        print(f"[FLASH] Waiting {args.boot_wait}s for device to boot...")
        time.sleep(args.boot_wait)
    else:
        print(_yellow("[FLASH] No --port specified — skipping upload (build only)"))

    print(_green("\n[FLASH] Done."))


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 2: DATA LOADING & ALIGNMENT  (unchanged from v5.0)
# ─────────────────────────────────────────────────────────────────────────────

RADAR_SYNONYMS = {
    "smooth_hr": "reported_hr",
    "smooth_rr": "reported_rr",
    "humandetected": "human_detected",
    "radarispresent": "radar_is_present",
    "hrstate": "hr_state",
    "heart_phase_stabilized": "heart_phase",
    "breath_phase_stabilized": "breath_phase",
    "module_fw_major": "fw_major",
    "module_fw_sub": "fw_sub",
    "module_fw_mod": "fw_mod",
}
REF_SYNONYMS = {
    "hr": "ref_hr",
    "rr": "ref_rr",
    "heart_rate": "ref_hr",
    "resp_rate": "ref_rr",
}


def lower_columns(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df.columns = [str(c).strip().lower() for c in df.columns]
    return df


def canonicalize_with_synonyms(df: pd.DataFrame, synonyms: Dict[str, str], source_name: str) -> pd.DataFrame:
    df = df.copy()
    for src, dst in synonyms.items():
        if src not in df.columns:
            continue
        if dst not in df.columns:
            df[dst] = df[src]
        else:
            dst_num = pd.to_numeric(df[dst], errors="coerce")
            src_num = pd.to_numeric(df[src], errors="coerce")
            overlap = dst_num.notna() & src_num.notna()
            if overlap.any():
                disagreements = np.abs(dst_num[overlap] - src_num[overlap]) > 1e-9
                if disagreements.any():
                    warn(f"{source_name}: columns '{dst}' and '{src}' both exist and disagree "
                         f"on {int(disagreements.sum())} row(s); preferring '{dst}' where present.")
            df[dst] = dst_num.where(dst_num.notna(), src_num)
        if src != dst:
            df.drop(columns=[src], inplace=True, errors="ignore")
    return df


def _check_monotonic_before_sort(df: pd.DataFrame, path: str):
    ts = pd.to_numeric(df["timestamp_s"], errors="coerce")
    if (ts.diff() < 0).any():
        warn(f"Non-monotonic timestamps in {path}; rows will be sorted by timestamp_s.")


def load_radar(path: str) -> pd.DataFrame:
    df = lower_columns(pd.read_csv(path))
    df = canonicalize_with_synonyms(df, RADAR_SYNONYMS, source_name=os.path.basename(path))

    # v8.4.0: materialize widened telemetry for older logs and create stable aliases.
    for col in ("hr_trusted_phase_anchor", "hr_anchor_err_bpm"):
        if col not in df.columns:
            df[col] = np.nan
    if "hr_anchor_source" not in df.columns:
        df["hr_anchor_source"] = 0
    if "hr_raw_high_bias_suspect" not in df.columns:
        df["hr_raw_high_bias_suspect"] = 0
    if "heart_phase" not in df.columns and "heart_phase_stabilized" in df.columns:
        df["heart_phase"] = df["heart_phase_stabilized"]
    if "breath_phase" not in df.columns and "breath_phase_stabilized" in df.columns:
        df["breath_phase"] = df["breath_phase_stabilized"]
    if "fw_major" not in df.columns and "module_fw_major" in df.columns:
        df["fw_major"] = df["module_fw_major"]
    if "fw_sub" not in df.columns and "module_fw_sub" in df.columns:
        df["fw_sub"] = df["module_fw_sub"]
    if "fw_mod" not in df.columns and "module_fw_mod" in df.columns:
        df["fw_mod"] = df["module_fw_mod"]
    for col, default in (
        ("dsp_ran_this_frame", 0),
        ("hr_confidence_source", 0),
        ("sketch_major", np.nan),
        ("sketch_sub", np.nan),
        ("sketch_mod", np.nan),
        ("rr_seed_from_raw_used", 0),
        ("trusted_hr_fresh", 0),
        ("allow_logged_hr_vitals", 0),
        ("allow_logged_rr_vitals", 0),
        ("logged_hr_quality_gate", 0),
        ("logged_rr_quality_gate", 0),
        ("phase_valid_run_len", 0),
        ("phase_invalid_run_len", 0),
        ("hr_phase_backed_update_count", 0),
        ("rr_phase_backed_update_count", 0),
        ("near_field_reflector_suspect", 0),
        ("agc_floor_suspect", 0),
        ("phase_backed_publish_ready", 0),
        ("hr_anchor_drift_suspect", 0),
        ("phase_gap_fill_count", 0),
        ("clutter_rewarm_count", 0),
        ("experimental_profile_enabled", 0),
    ):
        if col not in df.columns:
            df[col] = default

    if "timestamp_s" not in df.columns:
        if "timestamp_ms" not in df.columns:
            raise ValueError(f"Radar file {path} must contain timestamp_ms or timestamp_s")
        df["timestamp_s"] = pd.to_numeric(df["timestamp_ms"], errors="coerce") / 1000.0
    else:
        df["timestamp_s"] = pd.to_numeric(df["timestamp_s"], errors="coerce")

    if "distance_m" in df.columns:
        dist_m = pd.to_numeric(df["distance_m"], errors="coerce") * 100.0
        if "reflector_distance_cm" in df.columns:
            canon = pd.to_numeric(df["reflector_distance_cm"], errors="coerce")
            overlap = canon.notna() & dist_m.notna()
            if overlap.any():
                disagreements = np.abs(canon[overlap] - dist_m[overlap]) > 1e-6
                if disagreements.any():
                    warn(f"{os.path.basename(path)}: reflector_distance_cm and distance_m disagree "
                         f"on {int(disagreements.sum())} row(s); preferring reflector_distance_cm.")
            df["reflector_distance_cm"] = canon.where(canon.notna(), dist_m)
        else:
            df["reflector_distance_cm"] = dist_m
        df.drop(columns=["distance_m"], inplace=True, errors="ignore")

    required = ["timestamp_s", "reported_hr", "reported_rr", "pqi_heart", "pqi_breath"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing radar columns in {path}: {missing}")

    numeric_candidates = [
        "timestamp_ms", "heart_phase", "breath_phase", "raw_hr", "raw_rr", "raw_rr_effective", "raw_rr_likely_harmonic",
        "hr_zc_bpm", "hr_zc_conf", "hr_spec_bpm", "hr_spec_mag", "hr_triple_agree",
        "rr_zc_bpm", "rr_zc_conf", "rr_spec_bpm", "rr_spec_conf", "rr_triple_agree",
        "reported_hr", "reported_rr", "candidate_hr", "candidate_rr",
        "candidate_hr_conf", "candidate_rr_conf",
        "pqi_heart", "pqi_breath", "hr_confidence",
        "hr_gate_pqi_used", "rr_gate_pqi_used",
        "reflector_distance_cm", "reported_distance_cm", "in_motion",
        "human_detected", "radar_is_present", "hr_state", "ghost_suspect",
        "dist_sd_cm", "disp_state", "dsp_task", "present_votes", "absent_votes",
        "phase_fresh", "trusted_vital_fresh", "logged_hr_valid", "logged_rr_valid",
        "hr_publish_reason", "rr_publish_reason", "skipdsp_misses",
        "hr_band_min", "hr_band_max", "radar_gain",
        "hr_arbiter_corrected", "hr_rejectphase_rejected", "hr_coherence_rejected",
        "hr_raw_source", "hr_raw_agree", "hr_agree_err_bpm",
        "hr_bypass_pqi_ok", "hr_bypass_conf_ok", "hr_bypass_gate_ok", "hr_bypass_active",
        "hr_grace_eligible", "hr_grace_active", "hr_trust_fresh",
        "rr_anchor_fresh", "rr_outlier_persist", "rr_raw_agree_ok",
        "rr_pre_acceptphase", "rr_post_acceptphase", "rr_post_blend", "rr_post_bias_correction", "rr_post_kalman", "rr_final_publish_candidate",
        "rr_anchor_value", "rr_anchor_age_ms", "rr_anchor_source", "rr_anchor_confidence",
        "rr_fundamental_recovery_count", "rr_fundamental_recovery_triggered", "rr_raw_seed_consistent_count",
        "rr_midsession_raw_reanchor_allowed", "rr_midsession_raw_reanchor_blocked", "rr_midsession_raw_reanchor_reason", "rr_raw_anchor_err_bpm",
        "hr_pre_rejectphase", "hr_post_rejectphase", "hr_post_blend", "hr_post_coherence",
        "hr_final_publish_candidate",
        "hr_arbiter_anchor_used", "hr_arbiter_anchor_value",
        "hr_rejectphase_anchor_used", "hr_rejectphase_anchor_value",
        "hr_raw_looks_like_half_rate", "hr_trusted_anchor_value",
        "hr_age_ms", "candidate_hr_age_ms", "candidate_rr_age_ms",
        "hr_updated_this_cycle", "hr_update_source", "latched_raw_hr", "trusted_rr_fresh", "radar_is_present_raw",
        "harmonic_mode", "session_phase",
        "point_cloud_ok", "target_info_ok", "num_targets", "max_dop_abs", "max_dop_speed_cms", "doppler_motion", "cluster_anomaly", "multi_target",
        "primary_x", "primary_y", "primary_dop", "primary_dop_speed_cms", "primary_cluster", "spatial_source", "spatial_age_ms", "position_radius_cm",
        "phase_warmup_complete", "clutter_warmup_count", "current_clutter_alpha", "use_fast_path", "phase_valid_this_frame", "hr_path_source", "fw_major", "fw_sub", "fw_mod",
        "trusted_hr_fresh", "allow_logged_hr_vitals", "allow_logged_rr_vitals", "logged_hr_quality_gate", "logged_rr_quality_gate",
        "phase_valid_run_len", "phase_invalid_run_len", "hr_phase_backed_update_count", "rr_phase_backed_update_count",
        "near_field_reflector_suspect", "agc_floor_suspect", "phase_backed_publish_ready", "hr_anchor_drift_suspect",
        "phase_gap_fill_count", "clutter_rewarm_count", "experimental_profile_enabled",
    ]
    for col in [c for c in numeric_candidates if c in df.columns]:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["timestamp_s"]).copy()
    _check_monotonic_before_sort(df, path)
    if "in_motion" in df.columns:
        df = df[df["in_motion"].fillna(0) == 0].copy()
    return df.sort_values("timestamp_s").reset_index(drop=True)


def load_reference(path: str) -> pd.DataFrame:
    df = lower_columns(pd.read_csv(path))
    df = canonicalize_with_synonyms(df, REF_SYNONYMS, source_name=os.path.basename(path))

    if "timestamp_s" not in df.columns:
        if "timestamp_ms" not in df.columns:
            raise ValueError(f"Reference file {path} must contain timestamp_ms or timestamp_s")
        df["timestamp_s"] = pd.to_numeric(df["timestamp_ms"], errors="coerce") / 1000.0
    else:
        df["timestamp_s"] = pd.to_numeric(df["timestamp_s"], errors="coerce")

    for col in ["ref_hr", "ref_rr", "ref_spo2"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    if "ref_hr" not in df.columns:
        df["ref_hr"] = np.nan
    if "ref_rr" not in df.columns:
        df["ref_rr"] = np.nan

    if not (df["ref_hr"].notna().any() or df["ref_rr"].notna().any()):
        raise ValueError(f"Reference file {path} must contain at least one of ref_hr or ref_rr")

    df = df.dropna(subset=["timestamp_s"]).copy()
    _check_monotonic_before_sort(df, path)
    return df.sort_values("timestamp_s").reset_index(drop=True)


def normalize_offsets(n: int, offsets: Optional[Sequence[float]]) -> List[float]:
    if offsets is None or len(offsets) == 0:
        return [0.0] * n
    if len(offsets) == 1 and n > 1:
        return [float(offsets[0])] * n
    if len(offsets) != n:
        raise ValueError(f"Expected 1 or {n} offsets, got {len(offsets)}")
    return [float(x) for x in offsets]


def align_reference_time(
    radar_df: pd.DataFrame,
    ref_df: pd.DataFrame,
    ref_offset_s: float = 0.0,
    auto_align_start: bool = False,
) -> pd.DataFrame:
    ref_df = ref_df.copy()
    shift = float(ref_offset_s)
    if auto_align_start:
        shift += float(radar_df["timestamp_s"].min() - ref_df["timestamp_s"].min())
    ref_df["timestamp_s"] = ref_df["timestamp_s"] + shift
    return ref_df


def circular_mean(arr_like) -> float:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").dropna().to_numpy(dtype=float)
    if len(arr) == 0:
        return np.nan
    z = np.exp(1j * arr)
    return float(np.angle(np.mean(z)))


def circular_std(arr_like) -> float:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").dropna().to_numpy(dtype=float)
    if len(arr) == 0:
        return np.nan
    R = np.abs(np.mean(np.exp(1j * arr)))
    if R <= 1e-12:
        return np.nan
    return float(np.sqrt(max(0.0, -2.0 * np.log(R))))


def last_valid(arr_like) -> float:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").dropna().to_numpy(dtype=float)
    if len(arr) == 0:
        return np.nan
    return float(arr[-1])


def unwrapped_excursion(arr_like) -> float:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").dropna().to_numpy(dtype=float)
    if len(arr) == 0:
        return np.nan
    uw = np.unwrap(arr)
    return float(np.max(uw) - np.min(uw))


def phase_to_radians(arr_like, unit: str = "rad") -> np.ndarray:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").to_numpy(dtype=float)
    if len(arr) == 0:
        return arr
    valid = np.isfinite(arr)
    out = arr.astype(float, copy=True)
    if unit == "deg":
        out[valid] = np.deg2rad(out[valid])
    return out


def finite_phase_series_with_ts(ts_like, phase_like, unit: str = "rad") -> Tuple[np.ndarray, np.ndarray]:
    ts = pd.to_numeric(pd.Series(ts_like), errors="coerce").to_numpy(dtype=float)
    phase = phase_to_radians(phase_like, unit=unit)
    mask = np.isfinite(ts) & np.isfinite(phase)
    return ts[mask], phase[mask]


def spectral_entropy(power: np.ndarray) -> float:
    power = np.asarray(power, dtype=float)
    power = power[np.isfinite(power) & (power > 0)]
    if len(power) == 0:
        return np.nan
    p = power / np.sum(power)
    ent = -np.sum(p * np.log(p + 1e-12))
    return float(ent / np.log(len(p))) if len(p) > 1 else 0.0


def phase_band_features(
    ts_like, phase_like, band: Tuple[float, float],
    phase_unit: str = "rad",
    broader_band: Optional[Tuple[float, float]] = None,
) -> Dict[str, float]:
    ts, phase = finite_phase_series_with_ts(ts_like, phase_like, unit=phase_unit)
    empty = {
        "dom_freq_hz": np.nan, "peak_power": np.nan, "bandpower": np.nan,
        "spectral_entropy": np.nan, "peak_ratio": np.nan, "band_contrast": np.nan,
    }
    if len(ts) < 8:
        return empty
    order = np.argsort(ts)
    ts = ts[order]
    phase = phase[order]
    dt = np.diff(ts)
    dt = dt[np.isfinite(dt) & (dt > 1e-6)]
    if len(dt) == 0:
        return empty
    step = float(np.median(dt))
    if (not np.isfinite(step)) or step <= 0:
        return empty
    grid = np.arange(ts[0], ts[-1] + 0.5 * step, step)
    if len(grid) < 8:
        return empty
    uw = np.unwrap(phase)
    interp = np.interp(grid, ts, uw)
    t0 = grid - grid[0]
    if len(interp) >= 2:
        coeff = np.polyfit(t0, interp, deg=1)
        detrended = interp - np.polyval(coeff, t0)
    else:
        detrended = interp - np.nanmean(interp)
    detrended = detrended - np.nanmean(detrended)
    window = np.hanning(len(detrended)) if len(detrended) >= 4 else np.ones(len(detrended))
    spec = np.abs(np.fft.rfft(detrended * window)) ** 2
    freqs = np.fft.rfftfreq(len(detrended), d=step)
    band_mask = (freqs >= float(band[0])) & (freqs <= float(band[1]))
    if not np.any(band_mask):
        return empty
    band_freqs = freqs[band_mask]
    band_power = spec[band_mask]
    if len(band_power) == 0 or np.all(~np.isfinite(band_power)):
        return empty
    peak_idx = int(np.nanargmax(band_power))
    peak_power = float(band_power[peak_idx])
    dom_freq = float(band_freqs[peak_idx])
    total_bandpower = float(np.nansum(band_power))
    finite_power = band_power[np.isfinite(band_power)]
    if len(finite_power) >= 2:
        sorted_power = np.sort(finite_power)[::-1]
        second = float(sorted_power[1])
        peak_ratio = float(peak_power / (second + 1e-12))
    elif len(finite_power) == 1:
        peak_ratio = 1.0
    else:
        peak_ratio = np.nan
    entropy = spectral_entropy(finite_power)
    if broader_band is None:
        broader_band = (0.05, max(3.0, float(band[1]) * 1.5))
    broad_mask = (freqs >= float(broader_band[0])) & (freqs <= float(broader_band[1]))
    noise_mask = broad_mask & (~band_mask)
    noise_power = spec[noise_mask]
    noise_mean = float(np.nanmean(noise_power)) if np.any(np.isfinite(noise_power)) else np.nan
    band_contrast = float(peak_power / (noise_mean + 1e-12)) if np.isfinite(noise_mean) else np.nan
    return {
        "dom_freq_hz": dom_freq, "peak_power": peak_power, "bandpower": total_bandpower,
        "spectral_entropy": entropy, "peak_ratio": peak_ratio, "band_contrast": band_contrast,
    }


def aggregate_reference_1hz(ref_df: pd.DataFrame) -> pd.DataFrame:
    ref_df = ref_df.copy()
    ref_df["sec_bin"] = np.floor(ref_df["timestamp_s"]).astype(int)
    agg_map = {"ref_timestamp_s": ("timestamp_s", "mean")}
    if "ref_hr" in ref_df.columns:
        agg_map["ref_hr"] = ("ref_hr", "mean")
    if "ref_rr" in ref_df.columns:
        agg_map["ref_rr"] = ("ref_rr", "mean")
    if "ref_spo2" in ref_df.columns:
        agg_map["ref_spo2"] = ("ref_spo2", "mean")
    out = ref_df.groupby("sec_bin", as_index=False).agg(**agg_map)
    if "ref_hr" not in out.columns:
        out["ref_hr"] = np.nan
    if "ref_rr" not in out.columns:
        out["ref_rr"] = np.nan
    return out.sort_values("ref_timestamp_s").reset_index(drop=True)


def aggregate_radar_1hz(
    radar_df: pd.DataFrame,
    phase_unit: str = "rad",
    heart_fft_window_s: float = 5.0,
    breath_fft_window_s: float = 10.0,
) -> pd.DataFrame:
    df = radar_df.copy()
    df["sec_bin"] = np.floor(df["timestamp_s"]).astype(int)

    if "logged_hr_valid" in df.columns:
        df["row_hr_valid"] = (safe_series(df, "logged_hr_valid", default=0.0) > 0).astype(float)
    else:
        df["row_hr_valid"] = (safe_series(df, "reported_hr") > 10).astype(float)

    if "logged_rr_valid" in df.columns:
        df["row_rr_valid"] = (safe_series(df, "logged_rr_valid", default=0.0) > 0).astype(float)
    else:
        df["row_rr_valid"] = (safe_series(df, "reported_rr") > 0).astype(float)

    rows = []
    phase_cache = {}
    phase_ts_cache = {}
    for phase_col in [c for c in ["heart_phase", "breath_phase"] if c in df.columns]:
        pts, pvals = finite_phase_series_with_ts(df["timestamp_s"], df[phase_col], unit=phase_unit)
        phase_ts_cache[phase_col] = pts
        phase_cache[phase_col] = pvals

    for sec_bin, g in df.groupby("sec_bin", sort=True):
        row = {
            "sec_bin": int(sec_bin),
            "timestamp_s": float(pd.to_numeric(g["timestamp_s"], errors="coerce").dropna().iloc[-1]),
            "samples_per_sec": int(len(g)),
        }

        def add_numeric(prefix: str, col: str, stats: Sequence[str], *, nonzero_only: bool = False):
            if col not in g.columns:
                return
            s = pd.to_numeric(g[col], errors="coerce")
            if nonzero_only:
                s = s.where(s > 0, np.nan)
                row[f"{prefix}_nonzero_count"] = int(s.notna().sum())
            arr = s.dropna().to_numpy(dtype=float)
            if "mean"   in stats: row[f"{prefix}_mean"]   = float(np.mean(arr))   if len(arr) else np.nan
            if "std"    in stats: row[f"{prefix}_std"]    = float(np.std(arr, ddof=0)) if len(arr) else np.nan
            if "min"    in stats: row[f"{prefix}_min"]    = float(np.min(arr))    if len(arr) else np.nan
            if "max"    in stats: row[f"{prefix}_max"]    = float(np.max(arr))    if len(arr) else np.nan
            if "median" in stats: row[f"{prefix}_median"] = float(np.median(arr)) if len(arr) else np.nan
            if "last"   in stats: row[f"{prefix}_last"]   = float(arr[-1])        if len(arr) else np.nan
            if "sum"    in stats: row[f"{prefix}_sum"]    = float(np.sum(arr))    if len(arr) else np.nan

        g = g.copy()
        g["reported_hr__valid_only"] = pd.to_numeric(g["reported_hr"], errors="coerce").where(g["row_hr_valid"] > 0.0, np.nan)
        g["reported_rr__valid_only"] = pd.to_numeric(g["reported_rr"], errors="coerce").where(g["row_rr_valid"] > 0.0, np.nan)
        g["reported_hr__all"] = pd.to_numeric(g["reported_hr"], errors="coerce")
        g["reported_rr__all"] = pd.to_numeric(g["reported_rr"], errors="coerce")
        add_numeric("reported_hr", "reported_hr__valid_only", ["mean", "std", "min", "max", "median"])
        add_numeric("reported_rr", "reported_rr__valid_only", ["mean", "std", "min", "max", "median"])
        add_numeric("reported_hr_all", "reported_hr__all", ["mean", "std", "min", "max", "median"])
        add_numeric("reported_rr_all", "reported_rr__all", ["mean", "std", "min", "max", "median"])
        add_numeric("candidate_hr", "candidate_hr", ["mean", "std", "min", "max", "median"])
        add_numeric("candidate_rr", "candidate_rr", ["mean", "std", "min", "max", "median"])
        add_numeric("candidate_hr_conf", "candidate_hr_conf", ["mean", "std", "max"])
        add_numeric("candidate_rr_conf", "candidate_rr_conf", ["mean", "std", "max"])
        add_numeric("raw_hr",      "raw_hr",      ["mean", "std"], nonzero_only=True)
        add_numeric("raw_rr",      "raw_rr",      ["mean", "std"], nonzero_only=True)
        add_numeric("raw_rr_effective", "raw_rr_effective", ["mean", "std"], nonzero_only=True)
        add_numeric("hr_zc_bpm", "hr_zc_bpm", ["mean", "std", "last"])
        add_numeric("hr_zc_conf", "hr_zc_conf", ["mean", "std", "max"])
        add_numeric("hr_spec_bpm", "hr_spec_bpm", ["mean", "std", "last"])
        add_numeric("hr_spec_mag", "hr_spec_mag", ["mean", "std", "max"])
        add_numeric("hr_triple_agree", "hr_triple_agree", ["mean"])
        add_numeric("rr_zc_bpm", "rr_zc_bpm", ["mean", "std", "last"])
        add_numeric("rr_zc_conf", "rr_zc_conf", ["mean", "std", "max"])
        add_numeric("rr_spec_bpm", "rr_spec_bpm", ["mean", "std", "last"])
        add_numeric("rr_spec_conf", "rr_spec_conf", ["mean", "std", "max"])
        add_numeric("rr_triple_agree", "rr_triple_agree", ["mean"])
        add_numeric("pqi_heart",   "pqi_heart",   ["mean", "std", "max"])
        add_numeric("pqi_breath",  "pqi_breath",  ["mean", "std", "max"])
        add_numeric("hr_confidence","hr_confidence",["mean","std","max"])
        add_numeric("hr_gate_pqi_used", "hr_gate_pqi_used", ["mean", "std", "max"])
        add_numeric("rr_gate_pqi_used", "rr_gate_pqi_used", ["mean", "std", "max"])
        add_numeric("reported_distance_cm",  "reported_distance_cm",  ["mean", "std"])
        add_numeric("reflector_distance_cm", "reflector_distance_cm", ["mean", "std"])
        add_numeric("dist_sd_cm",  "dist_sd_cm",  ["mean", "std"])
        add_numeric("hr_state",    "hr_state",    ["last"])
        add_numeric("ghost_suspect","ghost_suspect",["mean"])
        add_numeric("human_detected","human_detected",["mean"])
        add_numeric("radar_is_present","radar_is_present",["mean"])
        add_numeric("present_votes","present_votes",["mean"])
        add_numeric("absent_votes", "absent_votes", ["mean"])
        add_numeric("row_hr_valid", "row_hr_valid", ["mean", "sum"])
        add_numeric("row_rr_valid", "row_rr_valid", ["mean", "sum"])
        for col in ["phase_fresh", "trusted_vital_fresh", "logged_hr_valid", "logged_rr_valid"]:
            add_numeric(col, col, ["mean"])

        add_numeric("point_cloud_ok", "point_cloud_ok", ["mean"])
        add_numeric("target_info_ok", "target_info_ok", ["mean"])
        add_numeric("num_targets", "num_targets", ["mean", "max", "last"])
        add_numeric("max_dop_abs", "max_dop_abs", ["mean", "max", "last"])
        add_numeric("max_dop_speed_cms", "max_dop_speed_cms", ["mean", "max", "last"])
        add_numeric("doppler_motion", "doppler_motion", ["mean"])
        add_numeric("cluster_anomaly", "cluster_anomaly", ["mean"])
        add_numeric("multi_target", "multi_target", ["mean"])
        add_numeric("primary_x", "primary_x", ["mean", "last"])
        add_numeric("primary_y", "primary_y", ["mean", "last"])
        add_numeric("primary_dop", "primary_dop", ["mean", "last"])
        add_numeric("primary_dop_speed_cms", "primary_dop_speed_cms", ["mean", "max", "last"])
        add_numeric("primary_cluster", "primary_cluster", ["last"])
        add_numeric("spatial_source", "spatial_source", ["last"])
        add_numeric("spatial_age_ms", "spatial_age_ms", ["mean", "last"])
        add_numeric("position_radius_cm", "position_radius_cm", ["mean", "std", "last"])
        add_numeric("harmonic_mode", "harmonic_mode", ["mean", "max", "last"])
        add_numeric("session_phase", "session_phase", ["mean", "last"])
        add_numeric("phase_valid_this_frame", "phase_valid_this_frame", ["mean"])
        add_numeric("dsp_ran_this_frame", "dsp_ran_this_frame", ["mean"])
        add_numeric("hr_confidence_source", "hr_confidence_source", ["last"])
        add_numeric("current_clutter_alpha", "current_clutter_alpha", ["mean", "last"])
        add_numeric("use_fast_path", "use_fast_path", ["mean"])
        add_numeric("hr_path_source", "hr_path_source", ["last"])
        add_numeric("module_fw_major", "module_fw_major", ["last"])
        add_numeric("module_fw_sub", "module_fw_sub", ["last"])
        add_numeric("module_fw_mod", "module_fw_mod", ["last"])
        add_numeric("sketch_major", "sketch_major", ["last"])
        add_numeric("sketch_sub", "sketch_sub", ["last"])
        add_numeric("sketch_mod", "sketch_mod", ["last"])
        add_numeric("hr_trusted_phase_anchor", "hr_trusted_phase_anchor", ["mean", "last"])
        add_numeric("hr_anchor_source", "hr_anchor_source", ["last"])
        add_numeric("hr_anchor_err_bpm", "hr_anchor_err_bpm", ["mean", "last", "max"])
        add_numeric("hr_raw_high_bias_suspect", "hr_raw_high_bias_suspect", ["mean", "last", "max"])
        add_numeric("trusted_hr_fresh", "trusted_hr_fresh", ["mean", "last"])
        add_numeric("allow_logged_hr_vitals", "allow_logged_hr_vitals", ["mean", "last"])
        add_numeric("allow_logged_rr_vitals", "allow_logged_rr_vitals", ["mean", "last"])
        add_numeric("logged_hr_quality_gate", "logged_hr_quality_gate", ["mean", "last"])
        add_numeric("logged_rr_quality_gate", "logged_rr_quality_gate", ["mean", "last"])
        add_numeric("phase_valid_run_len", "phase_valid_run_len", ["mean", "max", "last"])
        add_numeric("phase_invalid_run_len", "phase_invalid_run_len", ["mean", "max", "last"])
        add_numeric("hr_phase_backed_update_count", "hr_phase_backed_update_count", ["mean", "max", "last"])
        add_numeric("rr_phase_backed_update_count", "rr_phase_backed_update_count", ["mean", "max", "last"])
        add_numeric("near_field_reflector_suspect", "near_field_reflector_suspect", ["mean", "last", "max"])
        add_numeric("agc_floor_suspect", "agc_floor_suspect", ["mean", "last", "max"])
        add_numeric("phase_backed_publish_ready", "phase_backed_publish_ready", ["mean", "last", "max"])
        add_numeric("hr_anchor_drift_suspect", "hr_anchor_drift_suspect", ["mean", "last", "max"])
        add_numeric("phase_gap_fill_count", "phase_gap_fill_count", ["mean", "max", "last"])
        add_numeric("clutter_rewarm_count", "clutter_rewarm_count", ["mean", "max", "last"])
        add_numeric("experimental_profile_enabled", "experimental_profile_enabled", ["mean", "last", "max"])

        for phase_col in ["heart_phase", "breath_phase"]:
            if phase_col in g.columns:
                _, phase_arr_rad = finite_phase_series_with_ts(
                    g["timestamp_s"], g[phase_col], unit=phase_unit)
                row[f"{phase_col}_mean"]      = circular_mean(phase_arr_rad)
                row[f"{phase_col}_std"]       = circular_std(phase_arr_rad)
                row[f"{phase_col}_last"]      = last_valid(phase_arr_rad)
                row[f"{phase_col}_excursion"] = unwrapped_excursion(phase_arr_rad)

        t_end = row["timestamp_s"]
        if "heart_phase" in phase_cache:
            pts   = phase_ts_cache["heart_phase"]
            pvals = phase_cache["heart_phase"]
            si = np.searchsorted(pts, t_end - float(heart_fft_window_s), side="left")
            ei = np.searchsorted(pts, t_end + 1e-9, side="right")
            feats = phase_band_features(
                pts[si:ei], pvals[si:ei], band=(0.8, 2.5),
                phase_unit="rad", broader_band=(0.1, 3.0))
            for k, v in feats.items():
                row[f"heart_phase_{k}"] = v

        if "breath_phase" in phase_cache:
            pts   = phase_ts_cache["breath_phase"]
            pvals = phase_cache["breath_phase"]
            si = np.searchsorted(pts, t_end - float(breath_fft_window_s), side="left")
            ei = np.searchsorted(pts, t_end + 1e-9, side="right")
            feats = phase_band_features(
                pts[si:ei], pvals[si:ei], band=(0.10, 0.60),
                phase_unit="rad", broader_band=(0.05, 1.0))
            for k, v in feats.items():
                row[f"breath_phase_{k}"] = v

        rows.append(row)

    out = pd.DataFrame(rows)
    out = out.rename(columns={
        "row_hr_valid_mean": "hr_valid_frac",
        "row_rr_valid_mean": "rr_valid_frac",
        "row_hr_valid_sum":  "hr_valid_count",
        "row_rr_valid_sum":  "rr_valid_count",
    })
    return out.sort_values("timestamp_s").reset_index(drop=True)


def align_one_session(
    radar_path: str, ref_path: str, session_id: str,
    tolerance_s: float, ref_offset_s: float,
    auto_align_start: bool, merge_direction: str,
    phase_unit: str, heart_fft_window_s: float, breath_fft_window_s: float,
) -> pd.DataFrame:
    radar = load_radar(radar_path)
    ref   = load_reference(ref_path)
    ref   = align_reference_time(radar, ref, ref_offset_s=ref_offset_s,
                                  auto_align_start=auto_align_start)
    radar_1hz = aggregate_radar_1hz(
        radar, phase_unit=phase_unit,
        heart_fft_window_s=heart_fft_window_s,
        breath_fft_window_s=breath_fft_window_s)
    ref_1hz   = aggregate_reference_1hz(ref)
    merged = pd.merge_asof(
        radar_1hz.sort_values("timestamp_s"),
        ref_1hz.sort_values("ref_timestamp_s"),
        left_on="timestamp_s", right_on="ref_timestamp_s",
        tolerance=tolerance_s, direction=merge_direction,
    )
    merged["ref_dt_s"] = merged["timestamp_s"] - merged["ref_timestamp_s"]
    ref_cols = [c for c in ["ref_hr", "ref_rr"] if c in merged.columns]
    if not ref_cols:
        raise ValueError("Merged dataset has no reference columns after alignment.")
    merged = merged.dropna(subset=ref_cols, how="all").copy()
    if "ref_hr" in merged.columns:
        merged = merged[(merged["ref_hr"].isna()) | ((merged["ref_hr"] >= 30) & (merged["ref_hr"] <= 220))]
    if "ref_rr" in merged.columns:
        merged = merged[(merged["ref_rr"].isna()) | ((merged["ref_rr"] >= 4) & (merged["ref_rr"] <= 60))]
    merged["session_id"] = session_id
    merged["radar_file"] = os.path.basename(radar_path)
    merged["ref_file"]   = os.path.basename(ref_path)
    return merged.reset_index(drop=True)


def load_and_align_multiple_base(
    radar_paths: Sequence[str], ref_paths: Sequence[str],
    tolerance_s: float, ref_offsets_s: Optional[Sequence[float]],
    auto_align_start: bool, merge_direction: str,
    phase_unit: str, heart_fft_window_s: float, breath_fft_window_s: float,
) -> pd.DataFrame:
    offsets = normalize_offsets(len(radar_paths), ref_offsets_s)
    frames = []
    for idx, (rp, refp, off) in enumerate(zip(radar_paths, ref_paths, offsets), start=1):
        sid = f"session_{idx:02d}"
        merged = align_one_session(
            rp, refp, sid, tolerance_s, off, auto_align_start,
            merge_direction, phase_unit, heart_fft_window_s, breath_fft_window_s)
        frames.append(merged)
        print(
            f"[ALIGN] {sid}: {len(merged)} paired 1 Hz rows | "
            f"radar={os.path.basename(rp)} | ref={os.path.basename(refp)} | "
            f"offset={off:+.3f}s | direction={merge_direction}")
    return pd.concat(frames, ignore_index=True).reset_index(drop=True)


def load_and_aggregate_radar_only(
    radar_paths: Sequence[str],
    phase_unit: str, heart_fft_window_s: float, breath_fft_window_s: float,
) -> pd.DataFrame:
    frames = []
    for idx, rp in enumerate(radar_paths, start=1):
        sid = f"session_{idx:02d}"
        radar = load_radar(rp)
        agg   = aggregate_radar_1hz(
            radar, phase_unit=phase_unit,
            heart_fft_window_s=heart_fft_window_s,
            breath_fft_window_s=breath_fft_window_s)
        agg["session_id"]  = sid
        agg["radar_file"]  = os.path.basename(rp)
        frames.append(agg)
    return pd.concat(frames, ignore_index=True).reset_index(drop=True)


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 3: FEATURES  (unchanged from v5.0)
# ─────────────────────────────────────────────────────────────────────────────

TEMPORAL_BASE_COLS_CORE = [
    "reported_hr_mean", "reported_rr_mean", "raw_hr_mean", "raw_rr_mean", "raw_rr_effective_mean",
    "candidate_hr_mean", "candidate_rr_mean", "candidate_hr_conf_mean", "candidate_rr_conf_mean",
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
) -> List[str]:
    base     = BASE_FEATURES_FULL if feature_mode == "full" else BASE_FEATURES_CORE
    suffixes = TEMPORAL_SUFFIXES_FULL if feature_mode == "full" else TEMPORAL_SUFFIXES_CORE
    temporal = [c for c in df.columns if any(c.endswith("_" + s) for s in suffixes)]
    cols = [c for c in base if c in df.columns] + sorted(temporal)
    cols = [c for c in cols if c not in TRAINING_EXCLUDE_COLS]
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


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 4: SPLITTING  (unchanged from v5.0)
# ─────────────────────────────────────────────────────────────────────────────

def chronological_split_single_session(
    df: pd.DataFrame, val_ratio: float = 0.2, purge_gap_s: float = 10.0,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    df = df.sort_values("timestamp_s").reset_index(drop=True)
    t_min = float(df["timestamp_s"].min())
    t_max = float(df["timestamp_s"].max())
    if not np.isfinite(t_min) or not np.isfinite(t_max) or t_max <= t_min:
        raise ValueError("Single-session timestamps are not valid for chronological splitting.")
    split_time = t_min + (1.0 - val_ratio) * (t_max - t_min)
    train_df = df[df["timestamp_s"] <= split_time].copy()
    val_df   = df[df["timestamp_s"] > split_time + purge_gap_s].copy()
    if len(train_df) == 0 or len(val_df) == 0:
        raise ValueError("Validation set became empty after purge gap. Reduce --purge-gap-s.")
    if len(val_df) < 10:
        raise ValueError("Validation set too small. Use longer capture or reduce --purge-gap-s.")
    print(f"[SPLIT] single-session | train={len(train_df)} | val={len(val_df)} | "
          f"split_time={split_time:.2f}s | purge_gap={purge_gap_s:.1f}s")
    return train_df.reset_index(drop=True), val_df.reset_index(drop=True)


def split_sessions(df: pd.DataFrame, val_ratio: float, purge_gap_s: float) -> Tuple[pd.DataFrame, pd.DataFrame]:
    session_ids = list(dict.fromkeys(df["session_id"].tolist()))
    if len(session_ids) >= 2:
        warn("Multi-session hold-out uses input file order. Pass files in chronological order.")
        train_ids = session_ids[:-1]
        val_id    = session_ids[-1]
        train_df  = df[df["session_id"].isin(train_ids)].copy()
        val_df    = df[df["session_id"] == val_id].copy()
        print(f"[SPLIT] hold-out last session | train={train_ids} | val={val_id} | "
              f"n_train={len(train_df)} | n_val={len(val_df)}")
        return train_df.reset_index(drop=True), val_df.reset_index(drop=True)
    return chronological_split_single_session(df, val_ratio=val_ratio, purge_gap_s=purge_gap_s)


def split_three_way(
    df: pd.DataFrame, test_ratio: float, early_stop_ratio: float, purge_gap_s: float,
) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, Dict[str, object]]:
    session_ids = list(dict.fromkeys(df["session_id"].tolist()))
    info: Dict[str, object] = {"mode": "three_way", "session_ids": session_ids}

    if len(session_ids) >= 3:
        warn("Three-way multi-session split uses input file order.")
        train_ids = session_ids[:-2]
        stop_id   = session_ids[-2]
        test_id   = session_ids[-1]
        train_df  = df[df["session_id"].isin(train_ids)].copy()
        stop_df   = df[df["session_id"] == stop_id].copy()
        test_df   = df[df["session_id"] == test_id].copy()
        info.update({"train_sessions": train_ids, "stop_session": stop_id, "test_session": test_id})
        return train_df.reset_index(drop=True), stop_df.reset_index(drop=True), test_df.reset_index(drop=True), info

    if len(session_ids) == 2:
        train_source = df[df["session_id"] == session_ids[0]].copy()
        test_df      = df[df["session_id"] == session_ids[1]].copy()
        train_df, stop_df = chronological_split_single_session(
            train_source, val_ratio=early_stop_ratio, purge_gap_s=purge_gap_s)
        info.update({"train_source_session": session_ids[0], "test_session": session_ids[1]})
        return train_df.reset_index(drop=True), stop_df.reset_index(drop=True), test_df.reset_index(drop=True), info

    train_stop_df, test_df = chronological_split_single_session(df, val_ratio=test_ratio, purge_gap_s=purge_gap_s)
    train_df, stop_df = chronological_split_single_session(train_stop_df, val_ratio=early_stop_ratio, purge_gap_s=purge_gap_s)
    info.update({"mode": "three_way_single_session"})
    return train_df.reset_index(drop=True), stop_df.reset_index(drop=True), test_df.reset_index(drop=True), info


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 5: METRICS, EVALUATION, PLOTTING  (unchanged from v5.0)
# ─────────────────────────────────────────────────────────────────────────────

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


def compute_metrics(y_true: np.ndarray, y_pred: np.ndarray, target: str) -> Dict[str, float]:
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


def validity_masks(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    if "logged_hr_valid_mean" in df.columns:
        df["hr_valid_for_eval"] = safe_series(df, "logged_hr_valid_mean", default=0.0) >= 0.5
    else:
        df["hr_valid_for_eval"] = (
            (safe_series(df, "hr_valid_frac", default=0.0) >= 0.5) &
            (safe_series(df, "reported_hr_mean") > 10)
        )
    if "logged_rr_valid_mean" in df.columns:
        df["rr_valid_for_eval"] = safe_series(df, "logged_rr_valid_mean", default=0.0) >= 0.5
    else:
        df["rr_valid_for_eval"] = (
            (safe_series(df, "rr_valid_frac", default=0.0) >= 0.5) &
            (safe_series(df, "reported_rr_mean") > 0)
        )
    if "phase_warmup_complete_mean" in df.columns:
        warm = safe_series(df, "phase_warmup_complete_mean", default=1.0) >= 0.5
        df["hr_valid_for_eval"] = df["hr_valid_for_eval"] & warm
        df["rr_valid_for_eval"] = df["rr_valid_for_eval"] & warm
    return df


def finite_pair_mask(df: pd.DataFrame, true_col: str, pred_col: str) -> np.ndarray:
    return (safe_series(df, true_col).notna().to_numpy(dtype=bool) &
            safe_series(df, pred_col).notna().to_numpy(dtype=bool))


def baseline_summary(df: pd.DataFrame, target: str) -> Dict[str, Dict[str, float]]:
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
    coverage_frac = float(valid_mask.mean()) if len(valid_mask) else float("nan")
    pipeline_coverage_frac = float(valid_claim_mask.mean()) if len(valid_claim_mask) else float("nan")
    cov_pen_rmse = float(valid_only.get("rmse", float("nan"))) / max(np.sqrt(coverage_frac), 0.1) if np.isfinite(coverage_frac) else float("nan")
    return {
        "all_frames": compute_metrics(y_true[pair_mask_all], y_pred_all[pair_mask_all], target),
        "all_frames_true": compute_metrics(y_true[pair_mask_all], y_pred_zero_invalid[pair_mask_all], target),
        "valid_only": valid_only,
        "coverage_pct": float(100.0 * coverage_frac) if np.isfinite(coverage_frac) else float("nan"),
        "pipeline_coverage_pct": float(100.0 * pipeline_coverage_frac) if np.isfinite(pipeline_coverage_frac) else float("nan"),
        "ref_aligned_coverage_pct": float(100.0 * coverage_frac) if np.isfinite(coverage_frac) else float("nan"),
        "coverage_penalized_rmse": cov_pen_rmse,
        "longest_invalid_run_s": longest_false_run(valid_mask),
    }

def compute_raw_hr_bias_estimate(df: pd.DataFrame) -> Dict[str, float]:
    raw_col = "raw_hr_mean" if "raw_hr_mean" in df.columns else None
    if "raw_hr_nonzero_mean" in df.columns:
        raw_col = "raw_hr_nonzero_mean"
    raw = safe_series(df, raw_col) if raw_col else pd.Series(dtype=float)
    ref = safe_series(df, "ref_hr") if "ref_hr" in df.columns else pd.Series(dtype=float)
    mask = raw.notna() & ref.notna() & (raw > 0) & (ref > 0)
    if not mask.any():
        return {"n": 0, "bias_bpm": float("nan"), "raw_hr_mean": float("nan"), "ref_hr_mean": float("nan")}
    raw_v = raw[mask].astype(float)
    ref_v = ref[mask].astype(float)
    return {
        "n": int(mask.sum()),
        "bias_bpm": float((raw_v - ref_v).mean()),
        "raw_hr_mean": float(raw_v.mean()),
        "ref_hr_mean": float(ref_v.mean()),
    }

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


def model_summary(df: pd.DataFrame, target: str) -> Dict[str, Dict[str, float]]:
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


def print_metric_block(label: str, metrics: Dict[str, Dict[str, float]]):
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
    if np.isfinite(loa_u): plt.axhline(loa_u, linestyle=":")
    if np.isfinite(loa_l): plt.axhline(loa_l, linestyle=":")
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


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 6: MODELS  (unchanged from v5.0)
# ─────────────────────────────────────────────────────────────────────────────

def resolve_model_params(args) -> Dict[str, float]:
    if args.light_mode:
        return {"n_estimators": 150, "early_stop_max_estimators": 250,
                "max_depth": 2, "learning_rate": 0.05, "subsample": 0.8}
    return {
        "n_estimators":             args.n_estimators,
        "early_stop_max_estimators": args.early_stop_max_estimators,
        "max_depth":                args.max_depth,
        "learning_rate":            args.learning_rate,
        "subsample":                args.subsample,
    }


def build_model(random_state, params, n_estimators):
    return GradientBoostingRegressor(
        n_estimators=int(n_estimators), max_depth=int(params["max_depth"]),
        learning_rate=float(params["learning_rate"]), subsample=float(params["subsample"]),
        random_state=random_state,
    )


def get_sample_weights(train_rows, target, mode):
    if mode == "none":
        return None
    q = safe_series(train_rows, "pqi_heart_mean" if target == "hr" else "pqi_breath_mean", default=0.5)
    return q.clip(lower=0.05, upper=1.0).to_numpy(dtype=np.float32)


def choose_best_n_estimators(model, X_val, y_val):
    scores = [float(np.sqrt(mean_squared_error(y_val, pred)))
              for pred in model.staged_predict(X_val)]
    best_idx = int(np.argmin(scores))
    return best_idx + 1, float(scores[best_idx])


def truncate_gbr_model(model, best_n):
    best_n = int(best_n)
    model.estimators_ = model.estimators_[:best_n]
    model.n_estimators = best_n
    if hasattr(model, "n_estimators_"):
        model.n_estimators_ = best_n
    for attr in ["train_score_", "oob_improvement_", "oob_scores_"]:
        if hasattr(model, attr):
            try:
                setattr(model, attr, getattr(model, attr)[:best_n])
            except Exception:
                pass
    return model


def fit_target_model(
    df_train, df_val, target, X_train_all, X_val_all, params,
    random_state=42, sample_weight_mode="none",
    use_early_stopping=True, early_stop_strategy="retrain",
):
    valid_col  = f"{target}_valid_for_eval"
    ref_col    = f"ref_{target}"
    train_mask = df_train[valid_col].fillna(False).to_numpy(dtype=bool)
    val_mask   = df_val[valid_col].fillna(False).to_numpy(dtype=bool)
    train_rows = df_train.loc[train_mask].copy()
    val_rows   = df_val.loc[val_mask].copy()
    if len(train_rows) < 20:
        raise ValueError(f"Not enough valid {target.upper()} rows to train ({len(train_rows)}).")
    X_train  = X_train_all.loc[train_mask].to_numpy(dtype=np.float32)
    y_train  = train_rows[ref_col].to_numpy(dtype=np.float32)
    X_val    = X_val_all.loc[val_mask].to_numpy(dtype=np.float32)
    y_val    = val_rows[ref_col].to_numpy(dtype=np.float32)
    weights  = get_sample_weights(train_rows, target, sample_weight_mode)
    meta = {"train_rows": int(len(train_rows)), "val_rows": int(len(val_rows)),
            "used_early_stopping": False,
            "best_n_estimators": int(params["n_estimators"]),
            "best_val_rmse": float("nan")}
    def fit(m):
        if weights is None: m.fit(X_train, y_train)
        else:               m.fit(X_train, y_train, sample_weight=weights)
        return m
    if use_early_stopping and len(val_rows) >= 10:
        max_n      = int(params["early_stop_max_estimators"])
        model_full = fit(build_model(random_state, params, max_n))
        best_n, best_val_rmse = choose_best_n_estimators(model_full, X_val, y_val)
        if early_stop_strategy == "truncate":
            final_model = truncate_gbr_model(model_full, best_n)
        else:
            final_model = fit(build_model(random_state, params, best_n))
        meta.update({"used_early_stopping": True,
                     "best_n_estimators": int(best_n),
                     "best_val_rmse": float(best_val_rmse)})
        return final_model, meta
    return fit(build_model(random_state, params, int(params["n_estimators"]))), meta


def apply_causal_slew_limit(df, col, max_delta_per_s):
    if max_delta_per_s is None or not np.isfinite(max_delta_per_s) or max_delta_per_s <= 0:
        return df
    df = df.copy()
    pieces = []
    for _, g in df.groupby("session_id", sort=False):
        g    = g.sort_values("timestamp_s").copy()
        vals = pd.to_numeric(g[col], errors="coerce").to_numpy(dtype=float)
        ts   = pd.to_numeric(g["timestamp_s"], errors="coerce").to_numpy(dtype=float)
        fi   = np.where(np.isfinite(vals) & np.isfinite(ts))[0]
        if len(fi) >= 1:
            anchor = float(np.median(vals[fi[:min(3, len(fi))]]))
            first_idx = fi[0]
            vals[first_idx] = anchor
            prev_time = ts[first_idx]
            prev_out = vals[first_idx]
            for idx in fi[1:]:
                dt        = max(1e-6, ts[idx] - prev_time)
                max_delta = float(max_delta_per_s) * dt
                vals[idx] = np.clip(vals[idx], prev_out - max_delta, prev_out + max_delta)
                prev_out  = vals[idx]; prev_time = ts[idx]
        g[col] = vals
        pieces.append(g)
    return pd.concat(pieces, ignore_index=True)


def add_predictions(df, X_all, model_hr=None, model_rr=None, hr_slew_limit=None, rr_slew_limit=None):
    df = df.copy()
    X  = X_all.to_numpy(dtype=np.float32)
    if model_hr is not None:
        df["pred_hr"] = np.clip(model_hr.predict(X), HR_RANGE[0], HR_RANGE[1])
        df = apply_causal_slew_limit(df, "pred_hr", hr_slew_limit)
    else:
        df["pred_hr"] = np.nan
    if model_rr is not None:
        df["pred_rr"] = np.clip(model_rr.predict(X), RR_RANGE[0], RR_RANGE[1])
        df = apply_causal_slew_limit(df, "pred_rr", rr_slew_limit)
    else:
        df["pred_rr"] = np.nan
    return df


def enforce_baseline_gate(hr_baseline, args):
    if not args.require_baseline_gate:
        return
    r    = hr_baseline["valid_only"]["r"]
    rmse = hr_baseline["valid_only"]["rmse"]
    if ((np.isfinite(args.gate_hr_min_r) and (not np.isfinite(r)   or r    < args.gate_hr_min_r)) or
        (np.isfinite(args.gate_hr_max_rmse) and (not np.isfinite(rmse) or rmse > args.gate_hr_max_rmse))):
        raise ValueError(
            f"Baseline gate FAILED: r={r:.3f}, RMSE={rmse:.2f}. "
            f"Required: r>={args.gate_hr_min_r:.3f}, RMSE<={args.gate_hr_max_rmse:.2f}. "
            f"Do NOT train until signal quality improves.")


def fit_linear_embedded_surrogate(X, y):
    Xn    = X.to_numpy(dtype=np.float64)
    X_aug = np.column_stack([np.ones(len(Xn)), Xn])
    coeffs, *_ = np.linalg.lstsq(X_aug, y.astype(np.float64), rcond=None)
    return coeffs[1:, :].astype(np.float32), coeffs[0, :].astype(np.float32)


def export_linear_embedded_header(feature_names, weights, bias, out_path):
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("#pragma once\n\n")
        f.write(f"// Auto-generated by radar_vital_trainer_v{VERSION}\n")
        f.write(f"#define RVT_EMBEDDED_INPUT_DIM {len(feature_names)}\n\n")
        f.write("static const float RVT_EMBEDDED_BIAS[2] = {")
        f.write(", ".join(f"{float(x):.8g}f" for x in bias.tolist()))
        f.write("};\n\n")
        f.write(f"static const char* RVT_EMBEDDED_FEATURE_NAMES[{len(feature_names)}] = {{\n")
        for name in feature_names:
            f.write(f'  "{name}",\n')
        f.write("};\n\n")
        f.write(f"static const float RVT_EMBEDDED_WEIGHTS[{len(feature_names)}][2] = {{\n")
        for row in weights:
            f.write("  {" + ", ".join(f"{float(x):.8g}f" for x in row.tolist()) + "},\n")
        f.write("};\n")


def build_tiny_surrogate(input_dim):
    import tensorflow as tf
    inp = tf.keras.Input(shape=(input_dim,), name="features")
    x   = tf.keras.layers.Dense(32, activation="relu")(inp)
    x   = tf.keras.layers.Dense(16, activation="relu")(x)
    out = tf.keras.layers.Dense(2, name="vitals")(x)
    m   = tf.keras.Model(inp, out, name="radar_vital_surrogate")
    m.compile(optimizer=tf.keras.optimizers.Adam(1e-3), loss="mse", metrics=["mae"])
    return m


def export_tflite_surrogate(X_train, Y_train, X_val, Y_val, feature_names, out_dir):
    try:
        import tensorflow as tf
    except Exception as e:
        raise RuntimeError("TensorFlow required for TFLite export. pip install tensorflow") from e
    model = build_tiny_surrogate(X_train.shape[1])
    model.fit(X_train.to_numpy(dtype=np.float32), Y_train.astype(np.float32),
              validation_data=(X_val.to_numpy(dtype=np.float32), Y_val.astype(np.float32)),
              epochs=250, batch_size=min(64, max(8, len(X_train)//8)), verbose=0,
              callbacks=[
                  tf.keras.callbacks.EarlyStopping(monitor="val_loss", patience=15, restore_best_weights=True),
                  tf.keras.callbacks.ReduceLROnPlateau(monitor="val_loss", patience=7, factor=0.5, min_lr=1e-5),
              ])
    pred_val = model.predict(X_val.to_numpy(dtype=np.float32), verbose=0)
    surrogate_val_rmse = float(np.sqrt(np.mean((pred_val - Y_val.astype(np.float32))**2)))
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.target_spec.supported_types = [tf.float16]
    tflite_model = converter.convert()
    tflite_path  = os.path.join(out_dir, "embedded_surrogate.tflite")
    with open(tflite_path, "wb") as f: f.write(tflite_model)
    header_path = os.path.join(out_dir, "embedded_surrogate_tflite.h")
    with open(header_path, "w", encoding="utf-8") as f:
        f.write("#pragma once\n\n")
        f.write(bytes_to_c_array(tflite_model, "g_radar_vital_surrogate_tflite"))
    save_json({"feature_names": list(feature_names), "input_dim": int(X_train.shape[1]),
               "tflite_bytes": int(len(tflite_model)), "surrogate_val_rmse": surrogate_val_rmse},
              os.path.join(out_dir, "embedded_surrogate_manifest.json"))
    return {"tflite_path": tflite_path, "header_path": header_path,
            "surrogate_val_rmse": surrogate_val_rmse, "tflite_bytes": int(len(tflite_model))}


def maybe_export_embedded(args, out_dir, X_train_all, X_eval_all, train_pred, eval_pred,
                           eval_target_name="validation"):
    mode = getattr(args, "embedded_export", "none")
    if mode == "none":
        return {"mode": "none"}
    teacher_train = np.column_stack([train_pred["pred_hr"].to_numpy(dtype=np.float32),
                                     train_pred["pred_rr"].to_numpy(dtype=np.float32)])
    teacher_eval  = np.column_stack([eval_pred["pred_hr"].to_numpy(dtype=np.float32),
                                     eval_pred["pred_rr"].to_numpy(dtype=np.float32)])
    info = {"mode": mode, "eval_target": eval_target_name}
    if mode in {"linear", "both"}:
        weights, bias = fit_linear_embedded_surrogate(X_train_all, teacher_train)
        linear_header = os.path.join(out_dir, "embedded_linear_surrogate.h")
        export_linear_embedded_header(list(X_train_all.columns), weights, bias, linear_header)
        linear_eval = X_eval_all.to_numpy(dtype=np.float32) @ weights + bias
        linear_teacher_rmse = float(np.sqrt(np.mean((linear_eval - teacher_eval)**2)))
        linear_eval_df = eval_pred.copy()
        linear_eval_df["pred_hr"] = np.clip(linear_eval[:, 0], HR_RANGE[0], HR_RANGE[1])
        linear_eval_df["pred_rr"] = np.clip(linear_eval[:, 1], RR_RANGE[0], RR_RANGE[1])
        info.update({"linear_header": linear_header, "linear_teacher_rmse": linear_teacher_rmse,
                     "linear_ref_hr": model_summary(linear_eval_df, "hr"),
                     "linear_ref_rr": model_summary(linear_eval_df, "rr")})
    if mode in {"tflite", "both"}:
        if not tensorflow_is_available():
            msg = "TensorFlow not installed. pip install tensorflow"
            if mode == "tflite": raise RuntimeError(msg)
            warn(msg + " Skipping TFLite export.")
            info["tflite_skipped"] = msg
        else:
            info.update(export_tflite_surrogate(
                X_train_all, teacher_train, X_eval_all, teacher_eval,
                list(X_train_all.columns), out_dir))
    save_json(info, os.path.join(out_dir, "embedded_export_summary.json"))
    return info


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 7: COMMANDS — predict / analyse / train  (unchanged from v5.0)
# ─────────────────────────────────────────────────────────────────────────────

def cmd_predict(args):
    os.makedirs(args.out, exist_ok=True)
    model_hr = None
    model_rr = None
    hr_path = os.path.join(args.model_dir, "model_hr.pkl")
    rr_path = os.path.join(args.model_dir, "model_rr.pkl")
    if os.path.exists(hr_path):
        with open(hr_path, "rb") as f:
            model_hr = pickle.load(f)
    if os.path.exists(rr_path):
        with open(rr_path, "rb") as f:
            model_rr = pickle.load(f)
    if model_hr is None and model_rr is None:
        raise ValueError("No model_hr.pkl or model_rr.pkl found in model_dir.")
    with open(os.path.join(args.model_dir, "preprocessor.pkl"), "rb") as f:
        pre = pickle.load(f)
    feature_mode          = pre.get("feature_mode", "full")
    phase_unit            = pre.get("phase_unit", "rad")
    heart_fft_window_s    = float(pre.get("heart_fft_window_s", 5.0))
    breath_fft_window_s   = float(pre.get("breath_fft_window_s", 10.0))
    missing_flag_cols     = list(pre.get("missing_flag_cols", []))
    feature_cols          = list(pre["base_feature_cols"])
    expanded_feature_cols = list(pre["expanded_feature_cols"])
    impute_values         = {str(k): float(v) for k, v in pre.get("impute_values", {}).items()}
    trained_targets       = list(pre.get("trained_targets", []))

    base_df = load_and_aggregate_radar_only(
        args.radar, phase_unit=phase_unit,
        heart_fft_window_s=heart_fft_window_s, breath_fft_window_s=breath_fft_window_s)
    feat_df = validity_masks(engineer_temporal_features(base_df, feature_mode=feature_mode))
    X_all   = transform_feature_matrix(feat_df, feature_cols, impute_values, missing_flag_cols)
    X_all   = X_all.reindex(columns=expanded_feature_cols, fill_value=0.0).astype(np.float32)
    pred_df = add_predictions(feat_df, X_all, model_hr=model_hr, model_rr=model_rr,
                              hr_slew_limit=pre.get("slew_limit_hr_per_s"),
                              rr_slew_limit=pre.get("slew_limit_rr_per_s"))
    out_csv = os.path.join(args.out, "predictions.csv")
    pred_df.to_csv(out_csv, index=False)
    if model_hr is not None:
        maybe_plot_timeseries(pred_df.assign(ref_hr=np.nan, ref_rr=np.nan), "hr",
                              os.path.join(args.out, "predict_hr_overlay.png"),
                              pred_col="pred_hr", enabled=not args.no_plots)
    if model_rr is not None:
        maybe_plot_timeseries(pred_df.assign(ref_hr=np.nan, ref_rr=np.nan), "rr",
                              os.path.join(args.out, "predict_rr_overlay.png"),
                              pred_col="pred_rr", enabled=not args.no_plots)
    summary = {
        "version": VERSION,
        "rows": int(len(pred_df)),
        "sessions": list(dict.fromkeys(pred_df["session_id"].tolist())),
        "feature_mode": feature_mode,
        "phase_unit": phase_unit,
        "heart_fft_window_s": heart_fft_window_s,
        "breath_fft_window_s": breath_fft_window_s,
        "trained_targets": trained_targets if trained_targets else [t for t, m in [("hr", model_hr), ("rr", model_rr)] if m is not None],
        "output_csv": out_csv,
    }
    save_json(summary, os.path.join(args.out, "predict_summary.json"))
    write_text_report(os.path.join(args.out, "predict_report.txt"), summary)
    print(f"[OUT] Predictions saved to {out_csv}")


def cmd_analyse(args):
    os.makedirs(args.out, exist_ok=True)
    base_df = load_and_align_multiple_base(
        args.radar, args.ref,
        tolerance_s=args.tolerance_s, ref_offsets_s=args.ref_offset_s,
        auto_align_start=args.auto_align_start, merge_direction=args.merge_direction,
        phase_unit=args.phase_unit, heart_fft_window_s=args.heart_fft_window_s,
        breath_fft_window_s=args.breath_fft_window_s)
    feat_df = validity_masks(engineer_temporal_features(base_df, feature_mode=args.feature_mode))
    hr_base = baseline_summary(feat_df, "hr")
    rr_base = baseline_summary(feat_df, "rr")
    raw_radar_df = _combine_radar_frames(args.radar)
    hr_locked_base, ml_gate_locked = _phase_subset_summary(feat_df, "phase_locked")
    hr_settling_base, ml_gate_settling = _phase_subset_summary(feat_df, "phase_settling")
    ml_gate_combined = _ml_gate_block_from_baseline(hr_base)
    coverage_locked = float(hr_locked_base.get("coverage_pct", float("nan")))
    coverage_settling = float(hr_settling_base.get("coverage_pct", float("nan")))
    gate_audit = _build_gate_audit(raw_radar_df, feat_df)
    locked_mask = safe_series(feat_df, "phase_locked", default=0.0) >= 0.5
    settling_mask = safe_series(feat_df, "phase_settling", default=0.0) >= 0.5
    hr_eval_mask = safe_series(feat_df, "hr_valid_for_eval", default=0.0).to_numpy(dtype=bool)
    gate_audit["valid_pairs"]["locked_hr_pairs"] = int(np.sum(finite_pair_mask(feat_df, "ref_hr", "reported_hr_mean") & hr_eval_mask & locked_mask.to_numpy(dtype=bool)))
    gate_audit["valid_pairs"]["settling_hr_pairs"] = int(np.sum(finite_pair_mask(feat_df, "ref_hr", "reported_hr_mean") & hr_eval_mask & settling_mask.to_numpy(dtype=bool)))
    publish_reason_histogram = {
        "hr": _histogram_from_series(raw_radar_df.get("hr_publish_reason", pd.Series(dtype=float)), HR_PUBLISH_REASON_NAMES),
        "rr": _histogram_from_series(raw_radar_df.get("rr_publish_reason", pd.Series(dtype=float)), RR_PUBLISH_REASON_NAMES),
    }
    hr_gate_reason_histogram = _histogram_from_series(raw_radar_df.get("hr_gate_reason", pd.Series(dtype=float)), HR_GATE_REASON_NAMES)
    rr_gate_reason_histogram = _histogram_from_series(raw_radar_df.get("rr_gate_reason", pd.Series(dtype=float)), RR_GATE_REASON_NAMES)
    agc_anomaly_flags = _build_agc_anomaly_flags(raw_radar_df)
    fw_truthfulness = _truthfulness_from_radar(raw_radar_df)
    print("\n== BASELINE SUMMARY ==")
    print_metric_block("HR baseline", hr_base)
    print_metric_block("RR baseline", rr_base)

    r = hr_base["valid_only"]["r"]
    rmse = hr_base["valid_only"]["rmse"]
    gate_pass = np.isfinite(r) and r > 0.4 and np.isfinite(rmse) and rmse < 8.0
    print(f"\n{'='*50}")
    print(f"ML gate  r>0.4:    {_green('PASS') if (np.isfinite(r) and r>0.4) else _red('FAIL')}  (r={r:.3f})")
    print(f"ML gate  RMSE<8:   {_green('PASS') if (np.isfinite(rmse) and rmse<8) else _red('FAIL')}  (RMSE={rmse:.2f})")
    print(f"ML gate locked audit:    r={_to_num(ml_gate_locked.get('r'), float('nan')):.3f}  RMSE={_to_num(ml_gate_locked.get('rmse'), float('nan')):.2f}  n={_safe_int(ml_gate_locked.get('n'), 0)}")
    print(f"ML gate settling audit:  r={_to_num(ml_gate_settling.get('r'), float('nan')):.3f}  RMSE={_to_num(ml_gate_settling.get('rmse'), float('nan')):.2f}  n={_safe_int(ml_gate_settling.get('n'), 0)}")
    if gate_pass:
        print(_green("-> Gate PASSED. You may proceed to `train`."))
    else:
        print(_red("-> Gate FAILED. Do NOT run `train` yet."))
        print("  Investigate pqi_heart, firmware fixes, or collect more sessions.")
    print(f"{'='*50}\n")

    out_csv = os.path.join(args.out, "aligned_1hz_features.csv")
    feat_df.to_csv(out_csv, index=False)
    hr_overlay = os.path.join(args.out, "analyse_hr_overlay.png")
    rr_overlay = os.path.join(args.out, "analyse_rr_overlay.png")
    hr_ba = os.path.join(args.out, "analyse_hr_bland_altman.png")
    rr_ba = os.path.join(args.out, "analyse_rr_bland_altman.png")
    phase_png = os.path.join(args.out, "analyse_phase_signals.png")
    maybe_plot_timeseries(feat_df, "hr", hr_overlay, enabled=not args.no_plots)
    maybe_plot_timeseries(feat_df, "rr", rr_overlay, enabled=not args.no_plots)
    maybe_plot_phase_signals(feat_df, phase_png, enabled=not args.no_plots)
    y_true_hr, y_pred_hr = get_valid_pair_arrays(feat_df, "hr", "reported_hr_mean", valid_only=True)
    y_true_rr, y_pred_rr = get_valid_pair_arrays(feat_df, "rr", "reported_rr_mean", valid_only=True)
    maybe_plot_bland_altman(y_true_hr, y_pred_hr,
                            "HR Bland-Altman (reported vs ref, valid-only)",
                            hr_ba,
                            enabled=not args.no_plots, annotate=args.annotate_bland_altman)
    maybe_plot_bland_altman(y_true_rr, y_pred_rr,
                            "RR Bland-Altman (reported vs ref, valid-only)",
                            rr_ba,
                            enabled=not args.no_plots, annotate=args.annotate_bland_altman)

    ref_dt = safe_series(feat_df, "ref_dt_s")
    pqi = safe_series(feat_df, "pqi_heart_mean")
    pqi_lock_pct = float((pqi >= 0.35).mean() * 100.0) if pqi.notna().any() else float("nan")
    raw_hr_bias_estimate = compute_raw_hr_bias_estimate(feat_df)
    raw_hr_sumfreq_estimate = compute_raw_hr_sumfreq_error_estimate(feat_df)
    summary = {
        "version": VERSION, "n_rows": int(len(feat_df)),
        "sessions": list(dict.fromkeys(feat_df["session_id"].tolist())),
        "tolerance_s": args.tolerance_s, "merge_direction": args.merge_direction,
        "auto_align_start": bool(args.auto_align_start),
        "ref_offset_s": normalize_offsets(len(args.radar), args.ref_offset_s),
        "feature_mode": args.feature_mode, "phase_unit": args.phase_unit,
        "heart_fft_window_s": args.heart_fft_window_s,
        "breath_fft_window_s": args.breath_fft_window_s,
        "alignment_dt_s": {
            "mean": float(ref_dt.mean()) if ref_dt.notna().any() else float("nan"),
            "max_abs": float(ref_dt.abs().max()) if ref_dt.notna().any() else float("nan"),
        },
        "pqi_lock_pct": pqi_lock_pct,
        "ml_gate": {"passed": bool(gate_pass), "r": float(r), "rmse": float(rmse)},
        "ml_gate_combined": ml_gate_combined,
        "ml_gate_locked": ml_gate_locked,
        "ml_gate_settling": ml_gate_settling,
        "hr_baseline": hr_base, "rr_baseline": rr_base,
        "hr_baseline_locked": hr_locked_base,
        "hr_baseline_settling": hr_settling_base,
        "coverage_locked": coverage_locked,
        "coverage_settling": coverage_settling,
        "raw_hr_bias_estimate": raw_hr_bias_estimate,
        "raw_hr_sumfreq_estimate": raw_hr_sumfreq_estimate,
        "fw_truthfulness": fw_truthfulness,
        "gate_audit": gate_audit,
        "publish_reason_histogram": publish_reason_histogram,
        "hr_gate_reason_histogram": hr_gate_reason_histogram,
        "rr_gate_reason_histogram": rr_gate_reason_histogram,
        "agc_anomaly_flags": agc_anomaly_flags,
    }
    summary["internal_consistency_score"] = _compute_internal_consistency_score(summary)
    summary["session_quality_score"] = _compute_session_quality_score(summary, feat_df)
    has_ref = any(float((blk.get("all_frames", {}) if isinstance(blk.get("all_frames"), dict) else {}).get("n", 0)) > 0 for blk in (hr_base, rr_base))
    ghost_session = False
    if not has_ref:
        human_mean = safe_series(feat_df, "human_detected_mean")
        cand_hr_mean = safe_series(feat_df, "candidate_hr_mean")
        cand_rr_mean = safe_series(feat_df, "candidate_rr_mean")
        ghost_session = human_mean.notna().any() and human_mean.fillna(0).mean() > 0.5 and (((cand_hr_mean.fillna(0) > 0).sum() > 0) or ((cand_rr_mean.fillna(0) > 0).sum() > 0))
    review_flags = []
    rr_bias = float(((rr_base.get("valid_only") or {}).get("bias", float("nan"))))
    rr_r = float(((rr_base.get("valid_only") or {}).get("r", float("nan"))))
    hr_cov = float(hr_base.get("coverage_pct", float("nan")))
    raw_hr_bias = float((raw_hr_bias_estimate or {}).get("bias_bpm", float("nan")))
    raw_hr_sumfreq_mae = float((raw_hr_sumfreq_estimate or {}).get("mean_abs_error_bpm", float("nan")))
    if np.isfinite(rr_bias) and abs(rr_bias) > 3.5:
        review_flags.append("rr_bias_high")
    if np.isfinite(rr_r) and rr_r < 0.0:
        review_flags.append("rr_negative_correlation")
    if np.isfinite(hr_cov) and hr_cov < 20.0:
        review_flags.append("hr_coverage_low")
    if np.isfinite(raw_hr_bias) and abs(raw_hr_bias) > 8.0:
        review_flags.append("raw_hr_bias_high")
    if np.isfinite(raw_hr_sumfreq_mae) and raw_hr_sumfreq_mae <= 4.0:
        review_flags.append("raw_hr_sumfreq_match")
    if ghost_session:
        review_flags.append("ghost_target_session")
    summary["has_reference"] = bool(has_ref)
    summary["ghost_target_session"] = bool(ghost_session)
    summary["review_required"] = bool(review_flags)
    summary["review_flags"] = review_flags

    golden_result = None
    if getattr(args, "golden_json", None):
        try:
            golden_result = _compare_to_golden(summary, args.golden_json,
                                               getattr(args, "golden_hr_rmse_tol", 0.25),
                                               getattr(args, "golden_hr_r_tol", 0.03))
            summary["golden_check"] = golden_result
            status = _green("PASS") if golden_result.get("passed") else _red("FAIL")
            print(_bold("== GOLDEN REGRESSION CHECK =="))
            print(f"Status: {status}")
            for chk in golden_result.get("checks", []):
                print(f"  {chk.get('metric')}: gold={chk.get('golden')} current={chk.get('current')} ok={chk.get('ok')}")
        except Exception as e:
            warn(f"Golden comparison failed: {e}")

    save_json(summary, os.path.join(args.out, "analyse_summary.json"))
    write_text_report(os.path.join(args.out, "analyse_report.txt"), summary)

    sections = [
        ("Baseline summary", json.dumps(nan_safe({
            "ml_gate": summary.get("ml_gate"),
            "session_quality_score": summary.get("session_quality_score"),
            "internal_consistency_score": summary.get("internal_consistency_score"),
            "review_required": summary.get("review_required"),
            "review_flags": summary.get("review_flags"),
            "pqi_lock_pct": summary.get("pqi_lock_pct"),
            "alignment_dt_s": summary.get("alignment_dt_s"),
            "hr_baseline": hr_base,
            "rr_baseline": rr_base,
            "golden_check": golden_result,
        }), indent=2, allow_nan=False)),
    ]
    images = []
    for label, path in [
        ("HR Overlay", hr_overlay),
        ("RR Overlay", rr_overlay),
        ("Phase Signals", phase_png),
        ("HR Bland-Altman", hr_ba),
        ("RR Bland-Altman", rr_ba),
    ]:
        if os.path.exists(path):
            images.append((label, path))
    _write_simple_html_report(f"Radar Vital Trainer Analyse — {', '.join(summary['sessions'])}",
                              sections, images,
                              os.path.join(args.out, "analyse_report.html"))
    print(f"[OUT] Analysis outputs saved to {args.out}")
    return summary

def _run_loso_evaluation(base_df: pd.DataFrame, args, feature_cols, impute_values, missing_flag_cols,
                         expanded_feature_cols, params, available_targets):
    session_ids = list(dict.fromkeys(base_df["session_id"].tolist()))
    if len(session_ids) < 3:
        return {"enabled": False, "reason": "need at least 3 sessions"}
    folds = []
    for i, holdout in enumerate(session_ids):
        train_source = base_df[base_df["session_id"] != holdout].copy().reset_index(drop=True)
        eval_base = base_df[base_df["session_id"] == holdout].copy().reset_index(drop=True)
        if train_source.empty or eval_base.empty:
            continue
        loo_train_base, loo_stop_base = split_sessions(train_source, val_ratio=args.val_ratio, purge_gap_s=args.purge_gap_s)
        loo_train_df = validity_masks(engineer_temporal_features(loo_train_base, feature_mode=args.feature_mode))
        loo_stop_df = validity_masks(engineer_temporal_features(loo_stop_base, feature_mode=args.feature_mode))
        loo_eval_df = validity_masks(engineer_temporal_features(eval_base, feature_mode=args.feature_mode))
        X_train = transform_feature_matrix(loo_train_df, feature_cols, impute_values, missing_flag_cols)
        X_train = X_train.reindex(columns=expanded_feature_cols, fill_value=0.0).astype(np.float32)
        X_stop = transform_feature_matrix(loo_stop_df, feature_cols, impute_values, missing_flag_cols)
        X_stop = X_stop.reindex(columns=expanded_feature_cols, fill_value=0.0).astype(np.float32)
        X_eval = transform_feature_matrix(loo_eval_df, feature_cols, impute_values, missing_flag_cols)
        X_eval = X_eval.reindex(columns=expanded_feature_cols, fill_value=0.0).astype(np.float32)
        model_hr = model_rr = None
        if "hr" in available_targets and loo_train_df.get("hr_valid_for_eval", pd.Series(False, index=loo_train_df.index)).fillna(False).sum() >= 20:
            model_hr, _ = fit_target_model(loo_train_df, loo_stop_df, "hr", X_train, X_stop, params=params,
                                           random_state=args.random_state + i, sample_weight_mode=args.sample_weight_mode,
                                           use_early_stopping=not args.disable_early_stopping,
                                           early_stop_strategy=args.early_stop_strategy)
        if "rr" in available_targets and loo_train_df.get("rr_valid_for_eval", pd.Series(False, index=loo_train_df.index)).fillna(False).sum() >= 20:
            model_rr, _ = fit_target_model(loo_train_df, loo_stop_df, "rr", X_train, X_stop, params=params,
                                           random_state=args.random_state + 100 + i, sample_weight_mode=args.sample_weight_mode,
                                           use_early_stopping=not args.disable_early_stopping,
                                           early_stop_strategy=args.early_stop_strategy)
        pred = add_predictions(loo_eval_df, X_eval, model_hr=model_hr, model_rr=model_rr,
                               hr_slew_limit=args.slew_limit_hr_per_s, rr_slew_limit=args.slew_limit_rr_per_s)
        fold = {"holdout_session": holdout}
        if model_hr is not None:
            fold["hr_model"] = model_summary(pred, "hr")
        if model_rr is not None:
            fold["rr_model"] = model_summary(pred, "rr")
        folds.append(fold)
    out = {"enabled": True, "folds": folds}
    if folds:
        for target in ("hr", "rr"):
            rmses = []
            rs = []
            for f in folds:
                block = f.get(f"{target}_model")
                if isinstance(block, dict):
                    vo = block.get("valid_only", {})
                    if np.isfinite(float(vo.get("rmse", float("nan")))):
                        rmses.append(float(vo.get("rmse")))
                    if np.isfinite(float(vo.get("r", float("nan")))):
                        rs.append(float(vo.get("r")))
            out[f"{target}_mean_rmse"] = float(np.mean(rmses)) if rmses else float("nan")
            out[f"{target}_mean_r"] = float(np.mean(rs)) if rs else float("nan")
    return out

def cmd_train(args):
    os.makedirs(args.out, exist_ok=True)
    base_df = load_and_align_multiple_base(
        args.radar, args.ref,
        tolerance_s=args.tolerance_s, ref_offsets_s=args.ref_offset_s,
        auto_align_start=args.auto_align_start, merge_direction=args.merge_direction,
        phase_unit=args.phase_unit, heart_fft_window_s=args.heart_fft_window_s,
        breath_fft_window_s=args.breath_fft_window_s)

    if args.three_way_split:
        train_base, stop_base, eval_base, split_info = split_three_way(
            base_df, test_ratio=args.val_ratio,
            early_stop_ratio=args.early_stop_ratio, purge_gap_s=args.purge_gap_s)
        eval_name = "test"
    else:
        train_base, eval_base = split_sessions(base_df, val_ratio=args.val_ratio,
                                               purge_gap_s=args.purge_gap_s)
        stop_base  = eval_base
        split_info = {"mode": "two_way",
                      "sessions": list(dict.fromkeys(base_df["session_id"].tolist()))}
        eval_name  = "validation"

    train_df = validity_masks(engineer_temporal_features(train_base, feature_mode=args.feature_mode))
    stop_df  = validity_masks(engineer_temporal_features(stop_base,  feature_mode=args.feature_mode))
    eval_df  = validity_masks(engineer_temporal_features(eval_base,  feature_mode=args.feature_mode))

    hr_base_train = baseline_summary(train_df, "hr")
    rr_base_train = baseline_summary(train_df, "rr")
    hr_base_stop  = baseline_summary(stop_df,  "hr")
    rr_base_stop  = baseline_summary(stop_df,  "rr")
    hr_base_eval  = baseline_summary(eval_df,  "hr")
    rr_base_eval  = baseline_summary(eval_df,  "rr")

    gate_source = hr_base_stop if args.three_way_split else hr_base_eval
    enforce_baseline_gate(gate_source, args)

    requested_targets = list(dict.fromkeys(args.targets))
    available_targets = []
    skipped_targets = []
    for target in requested_targets:
        ref_col = f"ref_{target}"
        valid_col = f"{target}_valid_for_eval"
        has_ref = ref_col in train_df.columns and train_df[ref_col].notna().any()
        has_valid = valid_col in train_df.columns and bool(train_df[valid_col].fillna(False).sum() >= 20)
        if has_ref and has_valid:
            available_targets.append(target)
        else:
            skipped_targets.append(target)

    if not available_targets:
        raise ValueError(
            "No requested targets have enough valid training rows. "
            f"Requested={requested_targets}, skipped={skipped_targets}")
    if skipped_targets:
        warn(f"Skipping targets without enough valid reference rows: {skipped_targets}")

    feature_cols = pick_feature_columns(
        train_df, feature_mode=args.feature_mode,
        max_nan_frac=args.max_nan_frac, min_variance=args.min_variance)
    if not feature_cols:
        raise ValueError("No numeric feature columns available for training.")

    X_train_all, X_stop_all, impute_values, missing_flag_cols = prepare_feature_matrix(
        train_df, stop_df, feature_cols)
    X_eval_all = transform_feature_matrix(eval_df, feature_cols, impute_values, missing_flag_cols)
    expanded_feature_cols = list(X_train_all.columns)
    X_eval_all = X_eval_all.reindex(columns=expanded_feature_cols, fill_value=0.0).astype(np.float32)

    valid_counts = {
        "hr": int(train_df["hr_valid_for_eval"].fillna(False).sum()),
        "rr": int(train_df["rr_valid_for_eval"].fillna(False).sum()),
    }
    for target in available_targets:
        if len(expanded_feature_cols) > max(1, valid_counts[target] // max(1, args.min_samples_per_feature)):
            msg = (f"Feature/sample ratio high for {target}: features={len(expanded_feature_cols)} | "
                   f"valid_rows={valid_counts[target]}")
            if args.strict_feature_ratio:
                raise ValueError(msg)
            warn(msg)

    params = resolve_model_params(args)
    use_es = not args.disable_early_stopping
    model_hr = None
    model_rr = None
    hr_fit_meta = {"skipped": True}
    rr_fit_meta = {"skipped": True}

    if "hr" in available_targets:
        model_hr, hr_fit_meta = fit_target_model(
            train_df, stop_df, "hr", X_train_all, X_stop_all, params=params,
            random_state=args.random_state, sample_weight_mode=args.sample_weight_mode,
            use_early_stopping=use_es, early_stop_strategy=args.early_stop_strategy)
    if "rr" in available_targets:
        model_rr, rr_fit_meta = fit_target_model(
            train_df, stop_df, "rr", X_train_all, X_stop_all, params=params,
            random_state=args.random_state + 1, sample_weight_mode=args.sample_weight_mode,
            use_early_stopping=use_es, early_stop_strategy=args.early_stop_strategy)

    train_pred = add_predictions(train_df, X_train_all, model_hr=model_hr, model_rr=model_rr,
                                 hr_slew_limit=args.slew_limit_hr_per_s,
                                 rr_slew_limit=args.slew_limit_rr_per_s)
    stop_pred  = add_predictions(stop_df,  X_stop_all,  model_hr=model_hr, model_rr=model_rr,
                                 hr_slew_limit=args.slew_limit_hr_per_s,
                                 rr_slew_limit=args.slew_limit_rr_per_s)
    eval_pred  = add_predictions(eval_df,  X_eval_all,  model_hr=model_hr, model_rr=model_rr,
                                 hr_slew_limit=args.slew_limit_hr_per_s,
                                 rr_slew_limit=args.slew_limit_rr_per_s)

    hr_model_train = model_summary(train_pred, "hr") if model_hr is not None else None
    rr_model_train = model_summary(train_pred, "rr") if model_rr is not None else None
    hr_model_stop  = model_summary(stop_pred,  "hr") if model_hr is not None else None
    rr_model_stop  = model_summary(stop_pred,  "rr") if model_rr is not None else None
    hr_model_eval  = model_summary(eval_pred,  "hr") if model_hr is not None else None
    rr_model_eval  = model_summary(eval_pred,  "rr") if model_rr is not None else None

    print("\n== TRAIN SUMMARY ==")
    print_metric_block("HR baseline train", hr_base_train)
    if hr_model_train is not None:
        print_metric_block("HR model train", hr_model_train)
    else:
        print("\n[HR model train]\n  skipped")
    if rr_model_train is not None:
        print_metric_block("RR model train", rr_model_train)
    else:
        print("\n[RR model train]\n  skipped")
    print(f"\n== {'STOP' if args.three_way_split else 'VALIDATION'} SUMMARY ==")
    print_metric_block("HR baseline stop", hr_base_stop)
    if hr_model_stop is not None:
        print_metric_block("HR model stop", hr_model_stop)
    else:
        print("\n[HR model stop]\n  skipped")
    print_metric_block("RR baseline stop", rr_base_stop)
    if rr_model_stop is not None:
        print_metric_block("RR model stop", rr_model_stop)
    else:
        print("\n[RR model stop]\n  skipped")

    if args.three_way_split:
        print("\n== TEST SUMMARY ==")
        print_metric_block("HR baseline test", hr_base_eval)
        if hr_model_eval is not None:
            print_metric_block("HR model test", hr_model_eval)
        else:
            print("\n[HR model test]\n  skipped")
        print_metric_block("RR baseline test", rr_base_eval)
        if rr_model_eval is not None:
            print_metric_block("RR model test", rr_model_eval)
        else:
            print("\n[RR model test]\n  skipped")

    feature_manifest = {
        "feature_mode": args.feature_mode,
        "base_feature_count": len(feature_cols),
        "expanded_feature_count": len(expanded_feature_cols),
        "base_feature_cols": feature_cols,
        "expanded_feature_cols": expanded_feature_cols,
        "missing_flag_cols": missing_flag_cols,
        "requested_targets": requested_targets,
        "trained_targets": available_targets,
        "skipped_targets": skipped_targets,
    }
    save_json(feature_manifest, os.path.join(args.out, "feature_manifest.json"))
    if model_hr is not None:
        with open(os.path.join(args.out, "model_hr.pkl"), "wb") as f:
            pickle.dump(model_hr, f)
    if model_rr is not None:
        with open(os.path.join(args.out, "model_rr.pkl"), "wb") as f:
            pickle.dump(model_rr, f)
    with open(os.path.join(args.out, "preprocessor.pkl"), "wb") as f:
        pickle.dump({
            "version": VERSION, "feature_mode": args.feature_mode,
            "base_feature_cols": feature_cols, "expanded_feature_cols": expanded_feature_cols,
            "impute_values": impute_values, "missing_flag_cols": missing_flag_cols,
            "max_nan_frac": args.max_nan_frac, "min_variance": args.min_variance,
            "phase_unit": args.phase_unit,
            "heart_fft_window_s": args.heart_fft_window_s,
            "breath_fft_window_s": args.breath_fft_window_s,
            "slew_limit_hr_per_s": args.slew_limit_hr_per_s,
            "slew_limit_rr_per_s": args.slew_limit_rr_per_s,
            "trained_targets": available_targets,
        }, f)
    if model_hr is not None:
        save_feature_importance(model_hr, expanded_feature_cols,
                                os.path.join(args.out, "feature_importance_hr.csv"))
        save_feature_importance_plot(model_hr, expanded_feature_cols,
                                     os.path.join(args.out, "feature_importance_hr.png"))
    if model_rr is not None:
        save_feature_importance(model_rr, expanded_feature_cols,
                                os.path.join(args.out, "feature_importance_rr.csv"))
        save_feature_importance_plot(model_rr, expanded_feature_cols,
                                     os.path.join(args.out, "feature_importance_rr.png"))
    train_pred.to_csv(os.path.join(args.out, "train_predictions.csv"), index=False)
    stop_pred.to_csv(os.path.join(args.out,
                                  f"{'stop' if args.three_way_split else 'validation'}_predictions.csv"),
                     index=False)
    eval_pred.to_csv(os.path.join(args.out, f"{eval_name}_predictions.csv"), index=False)
    if model_hr is not None:
        maybe_plot_timeseries(eval_pred, "hr",
                              os.path.join(args.out, f"{eval_name}_hr_overlay.png"),
                              pred_col="pred_hr", enabled=not args.no_plots)
        y_true_hr, y_pred_hr = get_valid_pair_arrays(eval_pred, "hr", "pred_hr", valid_only=True)
        maybe_plot_bland_altman(y_true_hr, y_pred_hr,
                                f"HR Bland-Altman (model vs ref, {eval_name}, valid-only)",
                                os.path.join(args.out, f"{eval_name}_hr_bland_altman.png"),
                                enabled=not args.no_plots, annotate=args.annotate_bland_altman)
    if model_rr is not None:
        maybe_plot_timeseries(eval_pred, "rr",
                              os.path.join(args.out, f"{eval_name}_rr_overlay.png"),
                              pred_col="pred_rr", enabled=not args.no_plots)
        y_true_rr, y_pred_rr = get_valid_pair_arrays(eval_pred, "rr", "pred_rr", valid_only=True)
        maybe_plot_bland_altman(y_true_rr, y_pred_rr,
                                f"RR Bland-Altman (model vs ref, {eval_name}, valid-only)",
                                os.path.join(args.out, f"{eval_name}_rr_bland_altman.png"),
                                enabled=not args.no_plots, annotate=args.annotate_bland_altman)

    if model_hr is not None and model_rr is not None:
        embedded_info = maybe_export_embedded(args, args.out, X_train_all, X_eval_all,
                                              train_pred, eval_pred, eval_target_name=eval_name)
    else:
        embedded_info = {
            "mode": "skipped",
            "reason": "embedded export currently requires both HR and RR models",
        }
        save_json(embedded_info, os.path.join(args.out, "embedded_export_summary.json"))

    loo_eval = _run_loso_evaluation(base_df, args, feature_cols, impute_values, missing_flag_cols, expanded_feature_cols, params, available_targets) \
        if getattr(args, "loo_eval", False) else {"enabled": False}

    summary = {
        "version": VERSION, "split": split_info,
        "alignment": {
            "tolerance_s": args.tolerance_s, "merge_direction": args.merge_direction,
            "auto_align_start": bool(args.auto_align_start),
            "ref_offset_s": normalize_offsets(len(args.radar), args.ref_offset_s),
            "phase_unit": args.phase_unit,
            "heart_fft_window_s": args.heart_fft_window_s,
            "breath_fft_window_s": args.breath_fft_window_s,
        },
        "model_params": params, "feature_manifest": feature_manifest,
        "train_valid_counts": {
            "hr_valid_train_rows": valid_counts["hr"],
            "rr_valid_train_rows": valid_counts["rr"],
        },
        "sample_weight_mode": args.sample_weight_mode,
        "fit_meta": {"hr": hr_fit_meta, "rr": rr_fit_meta},
        "slew_limits": {"hr_per_s": args.slew_limit_hr_per_s, "rr_per_s": args.slew_limit_rr_per_s},
        "embedded_export": embedded_info,
        "loo_evaluation": loo_eval,
        "train": {
            "hr_baseline": hr_base_train, "hr_model": hr_model_train,
            "rr_baseline": rr_base_train, "rr_model": rr_model_train,
        },
        "stop": {
            "hr_baseline": hr_base_stop, "hr_model": hr_model_stop,
            "rr_baseline": rr_base_stop, "rr_model": rr_model_stop,
        },
        eval_name: {
            "hr_baseline": hr_base_eval, "hr_model": hr_model_eval,
            "rr_baseline": rr_base_eval, "rr_model": rr_model_eval,
        },
    }
    save_json(summary, os.path.join(args.out, "train_summary.json"))
    write_text_report(os.path.join(args.out, "train_report.txt"), summary)
    print(f"\n[OUT] Training outputs saved to {args.out}")


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 7B: ALIGN — standalone sync step
# ─────────────────────────────────────────────────────────────────────────────

def cmd_align(args):
    """
    Sync a radar CSV to a reference CSV and write the merged output.

    This is the same alignment performed inside `analyse` and `train`, exposed
    as a standalone step so you can inspect the merged CSV before training.

    Outputs:
      <out>/aligned_merged.csv   — merged 1 Hz radar+ref rows
      <out>/alignment_report.txt — sync quality metrics
    """
    os.makedirs(args.out, exist_ok=True)

    if len(args.radar) != len(args.ref):
        sys.exit("[ALIGN] --radar and --ref must have the same number of files.")

    offsets = normalize_offsets(len(args.radar), args.ref_offset_s)
    frames  = []
    for idx, (rp, refp, off) in enumerate(zip(args.radar, args.ref, offsets), start=1):
        sid    = f"session_{idx:02d}"
        merged = align_one_session(
            rp, refp, sid, args.tolerance_s, off,
            args.auto_align_start, args.merge_direction,
            args.phase_unit, args.heart_fft_window_s, args.breath_fft_window_s)
        frames.append(merged)
        ref_dt = merged["ref_dt_s"].dropna()
        print(f"[ALIGN] {sid}: {len(merged)} rows | "
              f"dt mean={ref_dt.mean():.3f}s  max_abs={ref_dt.abs().max():.3f}s")

        # Warn about poor alignment
        if ref_dt.abs().max() > 2.0:
            print(_yellow(f"[ALIGN] WARNING: max alignment gap {ref_dt.abs().max():.1f}s > 2s. "
                          f"Consider --ref-offset-s or --auto-align-start."))
        if len(merged) < 30:
            print(_yellow(f"[ALIGN] WARNING: only {len(merged)} paired rows. "
                          f"Session may be too short or timing offset too large."))

    combined = pd.concat(frames, ignore_index=True)
    out_csv  = os.path.join(args.out, "aligned_merged.csv")
    combined.to_csv(out_csv, index=False)

    report = {
        "version": VERSION, "n_sessions": len(args.radar),
        "total_paired_rows": int(len(combined)),
        "tolerance_s": args.tolerance_s, "merge_direction": args.merge_direction,
        "ref_offset_s": offsets, "auto_align_start": bool(args.auto_align_start),
        "sessions": [],
    }
    for frame in frames:
        dt = frame["ref_dt_s"].dropna()
        report["sessions"].append({
            "session_id":  frame["session_id"].iloc[0],
            "n_rows":      int(len(frame)),
            "dt_mean_s":   float(dt.mean())       if len(dt) else float("nan"),
            "dt_max_abs_s": float(dt.abs().max()) if len(dt) else float("nan"),
        })

    save_json(report, os.path.join(args.out, "alignment_report.json"))
    write_text_report(os.path.join(args.out, "alignment_report.txt"), report)
    print(f"\n[OUT] Merged CSV: {out_csv}")
    print(f"[OUT] Report:     {os.path.join(args.out, 'alignment_report.txt')}")
    print(f"\nNext step:")
    print(f"  python radar_vital_trainer_v8_4_3_for_v13_8_0.py analyse --radar {args.radar[0]} "
          f"--ref {args.ref[0]} --out <analysis_dir>")


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 7C: SWEEP — automated multi-variant flash -> log -> analyse
# ─────────────────────────────────────────────────────────────────────────────

def _log_serial_for_duration(port: str, duration_s: int, out_csv: str) -> int:
    """Logs serial DATA frames for duration_s seconds. Returns number of frames."""
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not found. Run: pip install pyserial")

    cols = list(RADAR_LOG_COLUMNS)

    os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)
    accepted    = 0
    dropped     = 0
    header_lines = 0
    t_start     = time.time()
    t_end       = t_start + duration_s
    first_frame = False

    with serial.Serial(port, 115200, timeout=1) as ser, \
         open(out_csv, "w", encoding="utf-8") as f:
        f.write(",".join(cols) + "\n")
        while time.time() < t_end:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("SESSION_SUMMARY") or line.startswith("SESSION,"):
                print(f"  {_dim(line)}")
                continue
            kind, row, detail = _parse_radar_data_line(line, cols)
            if kind == "skip":
                continue
            if kind == "header":
                header_lines += 1
                continue
            if kind == "data" and row is not None:
                f.write(",".join(row) + "\n")
                f.flush()
                accepted += 1
                if not first_frame:
                    print(_green("  First frame received — logging active."))
                    first_frame = True
                elapsed = time.time() - t_start
                remaining = max(0, duration_s - elapsed)
                if accepted % 50 == 0:
                    print(f"  {accepted} frames | {remaining:.0f}s remaining...", end="\r")
    print(f"  {accepted} frames logged in {duration_s}s")
    return accepted


def cmd_sweep(args):
    """
    Iterates a params_sweep.json, flashing and logging each variant.

    params_sweep.json format:
    [
      {"name": "baseline",    "params": {"RLS_LAMBDA": "0.97f"}, "duration_s": 920},
      {"name": "shorter_mem", "params": {"RLS_LAMBDA": "0.94f"}, "duration_s": 920}
    ]

    For each entry:
      1. Patches .ino constants and flashes firmware
      2. Waits for you to get into position (press Enter)
      3. Logs serial data for duration_s seconds
      4. Optionally runs analyse if a ref CSV is provided
    """
    _check_arduino_cli()

    if not os.path.exists(args.sweep_json):
        sys.exit(f"[SWEEP] Sweep file not found: {args.sweep_json}")

    experiments = json.load(open(args.sweep_json, encoding="utf-8"))
    if not isinstance(experiments, list) or len(experiments) == 0:
        sys.exit("[SWEEP] params_sweep.json must be a non-empty JSON array.")

    os.makedirs(args.out_dir, exist_ok=True)
    fqbn  = args.fqbn
    port  = args.port
    total = len(experiments)

    print(_bold(f"\n=== Sweep: {total} experiment(s) — {args.sweep_json} ===\n"))

    results = []

    for i, exp in enumerate(experiments, start=1):
        name       = exp.get("name", f"exp_{i:02d}")
        params     = exp.get("params", {})
        duration_s = int(exp.get("duration_s", args.default_duration_s))
        exp_dir    = os.path.join(args.out_dir, name)
        os.makedirs(exp_dir, exist_ok=True)

        print(_bold(f"\n[{i}/{total}] {name}"))
        print(f"  Params:   {params}")
        print(f"  Duration: {duration_s}s ({duration_s/60:.1f} min)")
        print(f"  Output:   {exp_dir}")

        # 1. Flash
        print(f"\n  Flashing firmware...")
        build_dir = os.path.join(exp_dir, "build")
        os.makedirs(build_dir, exist_ok=True)

        if params:
            work_dir = os.path.join(build_dir, "patched_sketch")
            os.makedirs(work_dir, exist_ok=True)
            sketch_name    = os.path.basename(args.sketch)
            patched_sketch = os.path.join(work_dir, sketch_name)
            _patch_ino_constants(args.sketch, params, patched_sketch)
            compile_sketch = work_dir
        else:
            compile_sketch = os.path.dirname(os.path.abspath(args.sketch))

        compile_cmd = ["arduino-cli", "compile", "--fqbn", fqbn,
                       "--build-path", os.path.join(build_dir, "build_out"), compile_sketch]
        result = subprocess.run(compile_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(_red(f"  Compile FAILED for {name}. Skipping."))
            print(result.stderr[-1000:])
            results.append({"name": name, "status": "compile_failed"})
            continue
        print(_green("  Compile OK"))

        upload_cmd = ["arduino-cli", "upload", "--fqbn", fqbn, "--port", port,
                      "--input-dir", os.path.join(build_dir, "build_out")]
        result = subprocess.run(upload_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(_red(f"  Upload FAILED for {name}. Skipping."))
            print(result.stderr[-500:])
            results.append({"name": name, "status": "upload_failed"})
            continue
        print(_green(f"  Upload OK -> {port}"))

        print(f"\n  Device booting ({args.boot_wait}s)...")
        time.sleep(args.boot_wait)

        # 2. Wait for user
        print(f"\n  {_bold('GET INTO POSITION:')} sit {args.distance_hint}m from the radar.")
        print(f"  Session will log for {duration_s}s (~{duration_s/60:.1f} min) once you press Enter.")
        if args.ref:
            print(f"  {_yellow('REMINDER:')} run  python radar_vital_trainer_v8_4_3_for_v13_8_0.py reflog  in a second terminal!")
        try:
            input("  Press Enter when ready... ")
        except KeyboardInterrupt:
            print(_yellow("\n  Skipped by user."))
            results.append({"name": name, "status": "skipped"})
            continue

        # 3. Log
        radar_csv = os.path.join(exp_dir, "radar.csv")
        print(f"\n  Logging to {radar_csv} ...")
        n_frames  = _log_serial_for_duration(port, duration_s, radar_csv)
        print(_green(f"  Session complete. {n_frames} frames saved."))

        # 4. Analyse (if ref provided)
        exp_result = {"name": name, "status": "logged", "n_frames": n_frames,
                      "radar_csv": radar_csv, "params": params}
        if args.ref and os.path.exists(args.ref):
            print(f"\n  Analysing against {args.ref} ...")
            try:
                merged = align_one_session(
                    radar_csv, args.ref, "session_01",
                    args.tolerance_s, 0.0, args.auto_align_start,
                    "nearest", "rad", 5.0, 10.0)
                feat_df  = validity_masks(engineer_temporal_features(merged, feature_mode="core"))
                hr_base  = baseline_summary(feat_df, "hr")
                rr_base  = baseline_summary(feat_df, "rr")
                r    = hr_base["valid_only"]["r"]
                rmse = hr_base["valid_only"]["rmse"]
                bias = hr_base["valid_only"]["bias"]
                gate = "PASS" if (np.isfinite(r) and r > 0.4 and
                                  np.isfinite(rmse) and rmse < 8.0) else "FAIL"
                exp_result.update({"status": "analysed", "hr_rmse": float(rmse),
                                   "hr_bias": float(bias), "hr_r": float(r), "ml_gate": gate})
                save_json({"hr_baseline": hr_base, "rr_baseline": rr_base},
                          os.path.join(exp_dir, "analyse_summary.json"))
                gate_str = _green(gate) if gate == "PASS" else _red(gate)
                print(f"  HR: RMSE={rmse:.2f}  bias={bias:+.2f}  r={r:.3f}  gate={gate_str}")
            except Exception as e:
                warn(f"Analysis failed for {name}: {e}")
                exp_result["status"] = "analyse_failed"
        results.append(exp_result)

    # Summary table
    print(_bold(f"\n\n{'='*60}"))
    print(_bold(f"  SWEEP SUMMARY — {total} experiments"))
    print(f"{'='*60}")
    print(f"  {'Name':<20} {'Status':<15} {'RMSE':>6} {'Bias':>6} {'r':>6} {'Gate'}")
    print(f"  {'-'*56}")
    for r in results:
        rmse = r.get("hr_rmse", float("nan"))
        bias = r.get("hr_bias", float("nan"))
        rv   = r.get("hr_r",   float("nan"))
        gate = r.get("ml_gate", "—")
        gate_str = _green(gate) if gate == "PASS" else (_red(gate) if gate == "FAIL" else gate)
        rmse_str = f"{rmse:.2f}" if np.isfinite(rmse) else "—"
        bias_str = f"{bias:+.2f}" if np.isfinite(bias) else "—"
        r_str    = f"{rv:.3f}"   if np.isfinite(rv)   else "—"
        print(f"  {r.get('name','?'):<20} {r.get('status','?'):<15} "
              f"{rmse_str:>6} {bias_str:>6} {r_str:>6} {gate_str}")
    print(f"{'='*60}")

    save_json({"version": VERSION, "sweep_json": args.sweep_json,
               "results": results},
              os.path.join(args.out_dir, "sweep_results.json"))
    print(f"\n[OUT] Sweep results saved to {args.out_dir}")

    if args.ref:
        print(f"\nTo compare all sessions:")
        print(f"  python radar_vital_trainer_v8_4_3_for_v13_8_0.py compare --sessions-dir {args.out_dir} --out report.html")


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 7D: COMPARE — HTML report across all sessions
# ─────────────────────────────────────────────────────────────────────────────

def _safe_float_val(v, fmt=".2f") -> str:
    if v is None: return "—"
    try:
        f = float(v)
        return "—" if not np.isfinite(f) else format(f, fmt)
    except Exception:
        return "—"


def _generate_compare_html(rows: List[Dict], out_path: str):
    """Generates a self-contained HTML comparison report."""

    def cell_color(val, thresholds, colors, reverse=False):
        """Returns inline CSS color for a metric value."""
        if val is None: return ""
        try:
            f = float(val)
            if not np.isfinite(f): return ""
        except Exception:
            return ""
        if reverse:
            if f > thresholds[0]: return f"color:{colors[0]}"
            if f > thresholds[1]: return f"color:{colors[1]}"
            return f"color:{colors[2]}"
        else:
            if f < thresholds[0]: return f"color:{colors[0]}"
            if f < thresholds[1]: return f"color:{colors[1]}"
            return f"color:{colors[2]}"

    gate_counts = {"PASS": 0, "FAIL": 0, "—": 0}
    for r in rows:
        g = r.get("ml_gate", "—")
        gate_counts[g if g in gate_counts else "—"] += 1

    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    best_rmse = min((float(r["hr_rmse"]) for r in rows
                     if r.get("hr_rmse") and np.isfinite(float(r["hr_rmse"]))),
                    default=float("nan"))

    table_rows = ""
    for r in sorted(rows, key=lambda x: float(x.get("hr_rmse") or 999)):
        gate = r.get("ml_gate", "—")
        gate_style = ("color:#3b6d11;font-weight:500" if gate == "PASS"
                      else "color:#a32d2d;font-weight:500" if gate == "FAIL" else "color:#888")
        rmse = r.get("hr_rmse")
        r_v  = r.get("hr_r")
        bias = r.get("hr_bias")
        pqi  = r.get("pqi_mean")
        rr_rmse = r.get("rr_rmse")

        rmse_style  = cell_color(rmse, [5, 8, 99],  ["#3b6d11", "#854f0b", "#a32d2d"])
        r_style     = cell_color(r_v,  [0.4, 0.2, -1], ["#3b6d11", "#854f0b", "#a32d2d"], reverse=True)
        pqi_style   = cell_color(pqi,  [0.3, 0.15, -1], ["#3b6d11", "#854f0b", "#a32d2d"], reverse=True)
        rr_style    = cell_color(rr_rmse, [3, 5, 99], ["#3b6d11", "#854f0b", "#a32d2d"])

        table_rows += f"""
        <tr>
          <td style="font-weight:500">{r.get('name','—')}</td>
          <td>{r.get('firmware','—')}</td>
          <td>{r.get('distance','—')}</td>
          <td>{r.get('n_pairs','—')}</td>
          <td style="{rmse_style}">{_safe_float_val(rmse)}</td>
          <td>{_safe_float_val(bias, '+.2f')}</td>
          <td style="{r_style}">{_safe_float_val(r_v, '.3f')}</td>
          <td style="{rr_style}">{_safe_float_val(rr_rmse)}</td>
          <td style="{pqi_style}">{_safe_float_val(pqi, '.4f')}</td>
          <td style="{pqi_style}">{_safe_float_val(r.get('pqi_lock_pct'), '.1f')}%</td>
          <td style="{gate_style}">{gate}</td>
        </tr>"""

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Radar Vital Monitor — Session Comparison</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
          background: #f9f9f7; color: #1a1a18; margin: 0; padding: 2rem; font-size: 14px; }}
  h1   {{ font-size: 20px; font-weight: 500; margin: 0 0 4px; }}
  .meta {{ color: #73726c; font-size: 13px; margin-bottom: 24px; }}
  .cards {{ display: grid; grid-template-columns: repeat(4, minmax(0,1fr)); gap: 10px; margin-bottom: 24px; }}
  .card {{ background: #f1efe8; border-radius: 8px; padding: 12px 14px; }}
  .card-label {{ font-size: 11px; color: #73726c; margin: 0 0 3px; }}
  .card-val {{ font-size: 20px; font-weight: 500; margin: 0; }}
  .card-sub {{ font-size: 11px; color: #73726c; margin: 2px 0 0; }}
  .sec {{ font-size: 11px; font-weight: 500; color: #73726c; letter-spacing: .06em;
          text-transform: uppercase; margin: 20px 0 8px; }}
  table {{ width: 100%; border-collapse: collapse; background: #fff;
           border: 0.5px solid #d3d1c7; border-radius: 8px; overflow: hidden; }}
  th {{ background: #f1efe8; font-size: 11px; font-weight: 500; color: #73726c;
        letter-spacing: .04em; padding: 8px 10px; text-align: left; border-bottom: 0.5px solid #d3d1c7; }}
  td {{ padding: 7px 10px; border-bottom: 0.5px solid #e8e6df; font-size: 13px; }}
  tr:last-child td {{ border-bottom: none; }}
  tr:hover td {{ background: #faf9f6; }}
  .legend {{ margin-top: 16px; font-size: 12px; color: #73726c; }}
  .dot {{ display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 4px; }}
  footer {{ margin-top: 32px; font-size: 11px; color: #b4b2a9; }}
</style>
</head>
<body>

<h1>Radar Vital Monitor — Session Comparison</h1>
<p class="meta">Generated: {timestamp} &nbsp;·&nbsp; Trainer v{VERSION} &nbsp;·&nbsp;
{len(rows)} session(s)</p>

<div class="cards">
  <div class="card">
    <p class="card-label">Sessions</p>
    <p class="card-val">{len(rows)}</p>
    <p class="card-sub">total analysed</p>
  </div>
  <div class="card">
    <p class="card-label">Best HR RMSE</p>
    <p class="card-val" style="color:{'#3b6d11' if best_rmse<8 else '#a32d2d'}">{
        _safe_float_val(best_rmse)} BPM</p>
    <p class="card-sub">{'gate range' if best_rmse<8 else 'above gate'}</p>
  </div>
  <div class="card">
    <p class="card-label">ML gate PASS</p>
    <p class="card-val" style="color:{'#3b6d11' if gate_counts['PASS']>0 else '#a32d2d'}">{
        gate_counts['PASS']}</p>
    <p class="card-sub">of {len(rows)} sessions</p>
  </div>
  <div class="card">
    <p class="card-label">Gate criteria</p>
    <p class="card-val" style="font-size:14px;padding-top:3px">r &gt; 0.4 &amp; RMSE &lt; 8</p>
    <p class="card-sub">HR valid-only</p>
  </div>
</div>

<p class="sec">All sessions — sorted by HR RMSE</p>
<table>
  <thead>
    <tr>
      <th>Session</th>
      <th>Firmware</th>
      <th>Distance</th>
      <th>n pairs</th>
      <th>HR RMSE</th>
      <th>HR bias</th>
      <th>Pearson r</th>
      <th>RR RMSE</th>
      <th>pqi_heart</th>
      <th>pqi lock %</th>
      <th>ML gate</th>
    </tr>
  </thead>
  <tbody>
    {table_rows}
  </tbody>
</table>

<div class="legend">
  <span class="dot" style="background:#3b6d11"></span> AAMI target &nbsp;
  <span class="dot" style="background:#854f0b"></span> borderline &nbsp;
  <span class="dot" style="background:#a32d2d"></span> needs work &nbsp;&nbsp;
  HR RMSE targets: &lt;5 AAMI / &lt;8 ML gate &nbsp;&nbsp;
  RR RMSE target: &lt;3 br/min &nbsp;&nbsp;
  pqi lock: ≥0.35
</div>

<footer>radar_vital_trainer_v{VERSION} &nbsp;·&nbsp;
XIAO ESP32-C6 + MR60BHA2 60 GHz FMCW &nbsp;·&nbsp; {timestamp}</footer>
</body>
</html>"""

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"[OUT] HTML report: {out_path}")


def cmd_compare(args):
    """
    Scans a directory of session folders and generates an HTML comparison report.

    Each session folder should contain:
      - analyse_summary.json   (written by `analyse`)  OR
      - radar.csv + ref.csv    (will run alignment+analysis on the fly)

    The folder name is used as the session name.
    """
    sessions_dir = args.sessions_dir
    if not os.path.isdir(sessions_dir):
        sys.exit(f"[COMPARE] Directory not found: {sessions_dir}")

    print(f"[COMPARE] Scanning {sessions_dir} ...")
    rows = []

    for entry in sorted(os.scandir(sessions_dir), key=lambda e: e.name):
        if not entry.is_dir():
            continue
        name = entry.name
        d    = entry.path

        # Try reading pre-computed analyse_summary.json first
        summary_path = os.path.join(d, "analyse_summary.json")
        if os.path.exists(summary_path):
            try:
                s = json.load(open(summary_path, encoding="utf-8"))
                hr = s.get("hr_baseline", {}).get("valid_only", {})
                rr = s.get("rr_baseline", {}).get("valid_only", {})

                # Try reading pqi from aligned CSV
                pqi_mean = pqi_lock = None
                aligned_csv = os.path.join(d, "aligned_1hz_features.csv")
                if os.path.exists(aligned_csv):
                    try:
                        df = pd.read_csv(aligned_csv)
                        if "pqi_heart_mean" in df.columns:
                            still = df
                            pqi_mean = float(still["pqi_heart_mean"].mean())
                            pqi_lock = float(100 * (still["pqi_heart_mean"] >= 0.35).mean())
                    except Exception:
                        pass

                gate = s.get("ml_gate", {})
                gate_str = ("PASS" if gate.get("passed") else
                            "FAIL" if gate.get("passed") is False else "—")
                rows.append({
                    "name":        name,
                    "firmware":    s.get("sessions", ["?"])[0] if s.get("sessions") else "?",
                    "distance":    "—",
                    "n_pairs":     hr.get("n", "—"),
                    "hr_rmse":     hr.get("rmse"),
                    "hr_bias":     hr.get("bias"),
                    "hr_r":        hr.get("r"),
                    "rr_rmse":     rr.get("rmse"),
                    "pqi_mean":    pqi_mean,
                    "pqi_lock_pct": pqi_lock,
                    "ml_gate":     gate_str,
                })
                print(f"  [OK]  {name}  (from analyse_summary.json)")
                continue
            except Exception as e:
                warn(f"  Could not read {summary_path}: {e}")

        # Fall back: find radar.csv + ref.csv and run on the fly
        radar_csv = None
        ref_csv   = None
        for fname in os.listdir(d):
            fl = fname.lower()
            if fl.endswith(".csv"):
                if "ref" in fl:
                    ref_csv = os.path.join(d, fname)
                elif "radar" in fl or "session" in fl or "run" in fl:
                    radar_csv = os.path.join(d, fname)

        if radar_csv and ref_csv:
            try:
                print(f"  [RUN] {name}  (analysing {os.path.basename(radar_csv)} + "
                      f"{os.path.basename(ref_csv)}) ...")
                merged  = align_one_session(radar_csv, ref_csv, "s01",
                                            0.9, 0.0, False, "nearest", "rad", 5.0, 10.0)
                feat_df = validity_masks(engineer_temporal_features(merged, feature_mode="core"))
                hr_base = baseline_summary(feat_df, "hr")
                rr_base = baseline_summary(feat_df, "rr")
                hr      = hr_base["valid_only"]
                rr      = rr_base["valid_only"]
                pqi_mean = float(feat_df["pqi_heart_mean"].mean()) \
                           if "pqi_heart_mean" in feat_df.columns else None
                pqi_lock = float(100 * (feat_df["pqi_heart_mean"] >= 0.35).mean()) \
                           if "pqi_heart_mean" in feat_df.columns else None
                gate = ("PASS" if (np.isfinite(hr.get("r", float("nan"))) and
                                   hr.get("r", 0) > 0.4 and
                                   np.isfinite(hr.get("rmse", float("nan"))) and
                                   hr.get("rmse", 99) < 8.0) else "FAIL")
                rows.append({
                    "name": name, "firmware": "?", "distance": "—",
                    "n_pairs": hr.get("n", "—"),
                    "hr_rmse": hr.get("rmse"), "hr_bias": hr.get("bias"),
                    "hr_r": hr.get("r"), "rr_rmse": rr.get("rmse"),
                    "pqi_mean": pqi_mean, "pqi_lock_pct": pqi_lock, "ml_gate": gate,
                })
            except Exception as e:
                warn(f"  Analysis failed for {name}: {e}")
        else:
            print(f"  [SKIP] {name}  (no analyse_summary.json or radar+ref CSV pair found)")

    if not rows:
        print(_yellow("[COMPARE] No sessions found. Run `analyse` on each session first."))
        return

    print(f"\n[COMPARE] {len(rows)} session(s) found.")
    _generate_compare_html(rows, args.out)

    # Print quick text table
    print(f"\n{'Name':<22} {'RMSE':>6} {'Bias':>6} {'r':>6} {'Gate'}")
    print("-" * 52)
    for r in sorted(rows, key=lambda x: float(x.get("hr_rmse") or 999)):
        gate = r.get("ml_gate", "—")
        g    = _green(gate) if gate == "PASS" else _red(gate) if gate == "FAIL" else gate
        print(f"  {r.get('name','?'):<20} "
              f"{_safe_float_val(r.get('hr_rmse')):>6} "
              f"{_safe_float_val(r.get('hr_bias'), '+.2f'):>6} "
              f"{_safe_float_val(r.get('hr_r'), '.3f'):>6}  {g}")


# ─────────────────────────────────────────────────────────────────────────────
# SECTION 8: CLI
# ─────────────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=f"Radar Vital Trainer v{VERSION} — full session pipeline + ML training",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Quick start:\n"
            "  python radar_vital_trainer_v8_4_3_for_v13_8_0.py doctor      # check dependencies\n"
            "  python radar_vital_trainer_v8_4_3_for_v13_8_0.py quickstart  # full workflow guide\n"
        ),
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    # ── doctor ────────────────────────────────────────────────────────────────
    p_doc = sub.add_parser("doctor", help="check all dependencies and print install instructions")
    p_doc.set_defaults(func=cmd_doctor)

    # ── quickstart ────────────────────────────────────────────────────────────
    p_qs = sub.add_parser("quickstart", help="print the full new-user workflow guide")
    p_qs.set_defaults(func=cmd_quickstart)

    # ── log ───────────────────────────────────────────────────────────────────
    p_log = sub.add_parser("log",
        help="capture serial DATA lines to CSV (run in Terminal A during session)")
    p_log.add_argument("--port", default=DEFAULT_RADAR_PORT,
                       help=f"serial port (default: {DEFAULT_RADAR_PORT}), e.g. COM10 (Windows) or /dev/ttyACM0 (Linux/Mac)")
    p_log.add_argument("--out",  default="session.csv",
                       help="output CSV path (default: session.csv)")
    p_log.set_defaults(func=cmd_log)

    # ── reflog ────────────────────────────────────────────────────────────────
    p_rl = sub.add_parser("reflog",
        help="live keyboard HR/RR reference logger (run in Terminal B during session)")
    p_rl.add_argument("--out", default="ref.csv",
                      help="output reference CSV path (default: ref.csv)")
    p_rl.set_defaults(func=cmd_reflog)

    # ── ble_reflog ────────────────────────────────────────────────────────────
    p_ble = sub.add_parser("ble_reflog",
        help="capture BLE pulse oximeter data into trainer CSV + raw protocol logs")
    p_ble.add_argument("--out", default="ref.csv",
                       help="parsed reference CSV path (default: ref.csv)")
    p_ble.add_argument("--raw-out", default=None,
                       help="raw BLE packet CSV path (default: alongside --out)")
    p_ble.add_argument("--summary-out", default=None,
                       help="BLE session summary JSON path (default: alongside --out)")
    p_ble.add_argument("--address", default=DEFAULT_BLE_ADDRESS,
                       help=f"BLE MAC address / Windows DEV token (default: {DEFAULT_BLE_ADDRESS})")
    p_ble.add_argument("--name-hint", nargs="*", default=None,
                       help="scan name substrings, e.g. Oximeter Wilcare PC-60")
    p_ble.add_argument("--notify-char", default=None,
                       help="preferred notification characteristic UUID (auto-ranked if omitted)")
    p_ble.add_argument("--ble-profile", choices=["none", "ailink_oximeter"], default="none",
                       help="apply a known BLE field map preset (default: none)")
    p_ble.add_argument("--subscribe-all", action="store_true",
                       help="subscribe to all notify-capable characteristics and log them all")
    p_ble.add_argument("--scan-timeout", type=float, default=8.0,
                       help="BLE scan timeout in seconds when --address is not supplied")
    p_ble.add_argument("--duration-s", type=float, default=None,
                       help="optional fixed logging duration in seconds")
    p_ble.add_argument("--hr-byte-index", type=int, default=None,
                       help="explicit byte index containing HR")
    p_ble.add_argument("--hr-scale", type=float, default=None,
                       help="scale factor applied to the HR byte (default: 1.0, or profile preset)")
    p_ble.add_argument("--rr-byte-index", type=int, default=None,
                       help="explicit byte index containing RR if your BLE device provides it")
    p_ble.add_argument("--rr-scale", type=float, default=None,
                       help="scale factor applied to the RR byte (example: 0.1 for 17.0 stored as 170)")
    p_ble.add_argument("--spo2-byte-index", type=int, default=None,
                       help="explicit byte index containing SpO2 if available")
    p_ble.add_argument("--spo2-scale", type=float, default=None,
                       help="scale factor applied to the SpO2 byte (default: 1.0, or profile preset)")
    p_ble.add_argument("--pi-byte-index", type=int, default=None,
                       help="explicit byte index containing perfusion index (PI) if available")
    p_ble.add_argument("--pi-scale", type=float, default=None,
                       help="scale factor applied to the PI byte (example: 0.1 for PI x10)")
    p_ble.add_argument("--auto-parse", action="store_true",
                       help="allow heuristic HR/SpO2 guesses when explicit byte indices are unknown")
    p_ble.add_argument("--raw-only", action="store_true",
                       help="disable parsed HR/RR inference; capture raw BLE only")
    p_ble.add_argument("--write-char", action="append", default=None,
                       help="characteristic UUID to write to before or during capture; repeatable")
    p_ble.add_argument("--write-hex", action="append", default=None,
                       help="hex payload for the matching --write-char; repeatable")
    p_ble.add_argument("--write-with-response", action="store_true",
                       help="use GATT write-with-response for all configured writes")
    p_ble.add_argument("--post-write-wait-ms", type=int, default=250,
                       help="wait after each write operation in milliseconds (default: 250)")
    p_ble.add_argument("--min-log-interval-ms", type=int, default=250,
                       help="minimum interval between parsed log rows (default: 250 ms)")
    p_ble.add_argument("--dedupe", action="store_true",
                       help="skip rows when parsed HR/RR did not change")
    p_ble.add_argument("--echo-raw", action="store_true",
                       help="print raw packets even when no valid HR/RR is parsed")
    p_ble.add_argument("--probe-services", action="store_true",
                       help="print BLE services/characteristics before subscribing")
    p_ble.set_defaults(func=cmd_ble_reflog)

    # ── ble_analyse_raw ───────────────────────────────────────────────────────
    p_bar = sub.add_parser("ble_analyse_raw",
        help="rank likely HR/RR/SpO2 fields from a raw BLE packet CSV")
    p_bar.add_argument("--raw", required=True,
                       help="raw BLE CSV produced by ble_reflog")
    p_bar.add_argument("--out", default="ble_raw_analysis",
                       help="output directory for ranked field candidates")
    p_bar.add_argument("--sender", default=None,
                       help="optional source_uuid/sender filter")
    p_bar.add_argument("--top-k", type=int, default=5,
                       help="number of top candidates to print per source (default: 5)")
    p_bar.set_defaults(func=cmd_ble_analyse_raw)


    # ── session ───────────────────────────────────────────────────────────────
    p_ss = sub.add_parser("session",
        help="run radar logging + BLE reference logging together, then auto-analyse")
    p_ss.add_argument("--session-dir", default="auto",
                      help="session directory that will receive radar.csv, ref.csv, analysis/ (default: auto-increment)")
    p_ss.add_argument("--sessions-root", default="sessions",
                      help="root directory used when --session-dir auto (default: sessions)")
    p_ss.add_argument("--session-prefix", default="s",
                      help="prefix for auto session names (default: s)")
    p_ss.add_argument("--session-digits", type=int, default=2,
                      help="zero-padding for auto session names (default: 2)")
    p_ss.add_argument("--port", default=DEFAULT_RADAR_PORT,
                      help=f"serial port for the radar logger (default: {DEFAULT_RADAR_PORT})")
    p_ss.add_argument("--address", default=DEFAULT_BLE_ADDRESS,
                      help=f"BLE MAC address / Windows DEV token for the oximeter (default: {DEFAULT_BLE_ADDRESS})")
    p_ss.add_argument("--ble-profile", default="ailink_oximeter",
                      choices=["none", "ailink_oximeter"],
                      help="BLE parser profile (default: ailink_oximeter)")
    p_ss.add_argument("--notify-char", default="0000ffe2-0000-1000-8000-00805f9b34fb",
                      help="notify characteristic UUID (default: working AiLink/Wilcare FFE2)")
    p_ss.add_argument("--duration-s", type=float, default=None,
                      help="optional auto-stop duration in seconds; omit to stop with Ctrl-C")
    p_ss.add_argument("--echo-raw", action="store_true",
                      help="echo raw BLE packets during the session")
    p_ss.add_argument("--probe-services", action="store_true",
                      help="print GATT services before subscribing (debug)")
    p_ss.add_argument("--subscribe-all", action="store_true",
                      help="subscribe to multiple notify candidates (debug)")
    p_ss.add_argument("--tolerance-s", type=float, default=0.9,
                      help="analyse merge tolerance in seconds (default: 0.9)")
    p_ss.add_argument("--merge-direction", choices=["nearest","backward","forward"],
                      default="nearest",
                      help="analyse merge_asof direction (default: nearest)")
    p_ss.add_argument("--auto-align-start", action="store_true", default=True,
                      help="shift reference start to radar start before analyse (default: enabled)")
    p_ss.add_argument("--feature-mode", choices=["core","full"], default="full",
                      help="feature mode passed to analyse (default: full)")
    p_ss.add_argument("--phase-unit", choices=["rad","deg"], default="rad")
    p_ss.add_argument("--heart-fft-window-s", type=float, default=5.0)
    p_ss.add_argument("--breath-fft-window-s", type=float, default=10.0)
    p_ss.add_argument("--no-plots", action="store_true",
                      help="skip analyse plots for faster turnaround")
    p_ss.add_argument("--annotate-bland-altman", action="store_true")
    p_ss.add_argument("--golden-json", default=None,
                      help="optional analyse_summary.json baseline to compare against immediately after session")
    p_ss.add_argument("--golden-hr-rmse-tol", type=float, default=0.25)
    p_ss.add_argument("--golden-hr-r-tol", type=float, default=0.03)
    p_ss.add_argument("--dashboard-refresh-s", type=float, default=1.0,
                      help="live dashboard refresh period in seconds (default: 1.0)")
    p_ss.add_argument("--dashboard-port", type=int, default=0,
                      help="dashboard localhost port (default: auto)")
    p_ss.add_argument("--live-dashboard", dest="live_dashboard", action="store_true", default=True,
                      help="show live clean dashboard during session (default: enabled)")
    p_ss.add_argument("--no-dashboard", dest="live_dashboard", action="store_false",
                      help="disable live dashboard and show only final report")
    p_ss.add_argument("--open-dashboard", dest="open_dashboard", action="store_true", default=True,
                      help="open the live dashboard automatically in your default browser (default: enabled)")
    p_ss.add_argument("--no-open-dashboard", dest="open_dashboard", action="store_false",
                      help="do not auto-open the dashboard browser tab")
    p_ss.set_defaults(func=cmd_session)

    # ── dashboard ──────────────────────────────────────────────────────────────
    p_db = sub.add_parser("dashboard",
        help="live clean dashboard for an existing session directory")
    p_db.add_argument("--session-dir", required=True,
                      help="session directory containing radar.csv and/or ref.csv")
    p_db.add_argument("--duration-s", type=float, default=None,
                      help="optional auto-stop duration")
    p_db.add_argument("--dashboard-refresh-s", type=float, default=1.0)
    p_db.add_argument("--dashboard-port", type=int, default=0,
                      help="dashboard localhost port (default: auto)")
    p_db.add_argument("--open-dashboard", dest="open_dashboard", action="store_true", default=True,
                      help="open the live dashboard automatically in your default browser (default: enabled)")
    p_db.add_argument("--no-open-dashboard", dest="open_dashboard", action="store_false",
                      help="do not auto-open the dashboard browser tab")
    p_db.set_defaults(func=cmd_dashboard)

    # ── flash ─────────────────────────────────────────────────────────────────
    p_fl = sub.add_parser("flash",
        help="patch .ino DSP constants, compile, and upload via arduino-cli")
    p_fl.add_argument("--sketch",    required=True,
                      help="path to .ino sketch file")
    p_fl.add_argument("--port",      default=None,
                      help="serial port for upload (omit for build-only)")
    p_fl.add_argument("--fqbn",      default="esp32:esp32:XIAO_ESP32C6",
                      help="arduino-cli FQBN (default: esp32:esp32:XIAO_ESP32C6)")
    p_fl.add_argument("--params",    default=None,
                      help="JSON file with constant overrides, e.g. {\"RLS_LAMBDA\": \"0.94f\"}")
    p_fl.add_argument("--build-dir", default="build",
                      help="temporary build directory (default: build/)")
    p_fl.add_argument("--boot-wait", type=float, default=3.0,
                      help="seconds to wait after upload for device boot (default: 3)")
    p_fl.add_argument("--verbose",   action="store_true",
                      help="print full arduino-cli output")
    p_fl.set_defaults(func=cmd_flash)

    # ── align ─────────────────────────────────────────────────────────────────
    p_al = sub.add_parser("align",
        help="sync radar CSV to reference CSV and write merged output for inspection")
    p_al.add_argument("--radar",  nargs="+", required=True)
    p_al.add_argument("--ref",    nargs="+", required=True)
    p_al.add_argument("--out",    default="align_out",
                      help="output directory (default: align_out/)")
    p_al.add_argument("--ref-offset-s",    nargs="*", type=float, default=None,
                      help="manual timestamp offset(s) in seconds (one per session)")
    p_al.add_argument("--tolerance-s",     type=float, default=0.9)
    p_al.add_argument("--merge-direction", choices=["nearest","backward","forward"],
                      default="nearest")
    p_al.add_argument("--auto-align-start", action="store_true",
                      help="shift ref timestamps so they start at radar start time")
    p_al.add_argument("--phase-unit",  choices=["rad","deg"], default="rad")
    p_al.add_argument("--heart-fft-window-s",  type=float, default=5.0)
    p_al.add_argument("--breath-fft-window-s", type=float, default=10.0)
    p_al.set_defaults(func=cmd_align)

    # ── predict ───────────────────────────────────────────────────────────────
    p_pr = sub.add_parser("predict",
        help="apply a saved trained model to new radar-only CSV(s)")
    p_pr.add_argument("--radar",     nargs="+", required=True)
    p_pr.add_argument("--model-dir", required=True,
                      help="directory containing model_hr.pkl, model_rr.pkl, preprocessor.pkl")
    p_pr.add_argument("--out",       default="predict_out")
    p_pr.add_argument("--no-plots",  action="store_true")
    p_pr.set_defaults(func=cmd_predict)

    # ── shared args for analyse + train ───────────────────────────────────────
    def add_common(p):
        p.add_argument("--radar",  nargs="+", required=True)
        p.add_argument("--ref",    nargs="+", required=True)
        p.add_argument("--out",    default="model_out")
        p.add_argument("--tolerance-s",     type=float, default=0.9)
        p.add_argument("--merge-direction", choices=["nearest","backward","forward"],
                       default="nearest",
                       help="merge_asof direction (default: nearest; use backward only when you explicitly want stricter delayed-reference matching)")
        p.add_argument("--ref-offset-s",    nargs="*", type=float, default=None)
        p.add_argument("--auto-align-start", action="store_true")
        p.add_argument("--feature-mode",    choices=["core","full"], default="full",
                       help="core=~86 features (safe for <15min); full=~120+ (needs 15+ min)")
        p.add_argument("--phase-unit",      choices=["rad","deg"], default="rad")
        p.add_argument("--heart-fft-window-s",  type=float, default=5.0)
        p.add_argument("--breath-fft-window-s", type=float, default=10.0)
        p.add_argument("--no-plots",             action="store_true")
        p.add_argument("--annotate-bland-altman", action="store_true")

    # ── analyse ───────────────────────────────────────────────────────────────
    p_an = sub.add_parser("analyse",
        help="baseline RMSE/r/pqi analysis with ML gate check")
    add_common(p_an)
    p_an.add_argument("--golden-json", default=None,
                      help="optional analyse_summary.json baseline to compare current metrics against")
    p_an.add_argument("--golden-hr-rmse-tol", type=float, default=0.25,
                      help="allowed HR RMSE regression versus golden before FAIL (default: 0.25)")
    p_an.add_argument("--golden-hr-r-tol", type=float, default=0.03,
                      help="allowed HR correlation drop versus golden before FAIL (default: 0.03)")
    p_an.set_defaults(func=cmd_analyse)

    # ── train ─────────────────────────────────────────────────────────────────
    p_tr = sub.add_parser("train",
        help="train HR and RR GBM correction models (only after gate passes)")
    add_common(p_tr)
    p_tr.add_argument("--val-ratio",         type=float, default=0.2)
    p_tr.add_argument("--three-way-split",   action="store_true")
    p_tr.add_argument("--early-stop-ratio",  type=float, default=0.15)
    p_tr.add_argument("--purge-gap-s",       type=float, default=10.0)
    p_tr.add_argument("--random-state",      type=int,   default=42)
    p_tr.add_argument("--sample-weight-mode",choices=["none","pqi"], default="none")
    p_tr.add_argument("--disable-early-stopping", action="store_true")
    p_tr.add_argument("--early-stop-strategy", choices=["retrain","truncate"], default="retrain")
    p_tr.add_argument("--light-mode",        action="store_true",
                      help="fast/small model for quick iteration")
    p_tr.add_argument("--n-estimators",      type=int,   default=250)
    p_tr.add_argument("--early-stop-max-estimators", type=int, default=500)
    p_tr.add_argument("--max-depth",         type=int,   default=3)
    p_tr.add_argument("--learning-rate",     type=float, default=0.04)
    p_tr.add_argument("--subsample",         type=float, default=0.8)
    p_tr.add_argument("--max-nan-frac",      type=float, default=0.70)
    p_tr.add_argument("--min-variance",      type=float, default=1e-8)
    p_tr.add_argument("--min-samples-per-feature", type=int, default=10)
    p_tr.add_argument("--strict-feature-ratio", action="store_true")
    p_tr.add_argument("--require-baseline-gate", action="store_true",
                      help="abort training if r<gate_hr_min_r or RMSE>gate_hr_max_rmse")
    p_tr.add_argument("--gate-hr-min-r",    type=float, default=0.4)
    p_tr.add_argument("--gate-hr-max-rmse", type=float, default=8.0)
    p_tr.add_argument("--slew-limit-hr-per-s", type=float, default=None)
    p_tr.add_argument("--slew-limit-rr-per-s", type=float, default=None)
    p_tr.add_argument("--embedded-export",  choices=["none","linear","tflite","both"],
                      default="none")
    p_tr.add_argument("--targets", nargs="+", choices=["hr", "rr"], default=["hr", "rr"],
                      help="targets to train. Use --targets hr for BLE pulse-ox reference sessions")
    p_tr.add_argument("--loo-eval", action="store_true",
                      help="run leave-one-session-out evaluation across available sessions (recommended for paper-grade validation)")
    p_tr.set_defaults(func=cmd_train)

    # ── sweep ─────────────────────────────────────────────────────────────────
    p_sw = sub.add_parser("sweep",
        help="automated DSP parameter sweep: flash -> log -> analyse each variant")
    p_sw.add_argument("--sweep-json",  required=True,
                      help="JSON array of experiment configs (see quickstart for format)")
    p_sw.add_argument("--sketch",      required=True,
                      help="path to .ino sketch file")
    p_sw.add_argument("--port",        required=True,
                      help="serial port, e.g. COM3 or /dev/ttyACM0")
    p_sw.add_argument("--ref",         default=None,
                      help="reference CSV to use for all variants (optional but recommended)")
    p_sw.add_argument("--out-dir",     default="sweep_out",
                      help="output directory for all experiment results (default: sweep_out/)")
    p_sw.add_argument("--fqbn",        default="esp32:esp32:XIAO_ESP32C6")
    p_sw.add_argument("--default-duration-s", type=int, default=920,
                      help="fallback session duration if not set per-experiment (default: 920s)")
    p_sw.add_argument("--boot-wait",   type=float, default=3.0)
    p_sw.add_argument("--distance-hint", default="0.7–0.9",
                      help="distance reminder shown to user (default: 0.7–0.9)")
    p_sw.add_argument("--tolerance-s",  type=float, default=0.9)
    p_sw.add_argument("--auto-align-start", action="store_true")
    p_sw.set_defaults(func=cmd_sweep)

    # ── compare ───────────────────────────────────────────────────────────────
    p_cmp = sub.add_parser("compare",
        help="generate HTML comparison report across all sessions in a directory")
    p_cmp.add_argument("--sessions-dir", required=True,
                       help="directory containing session sub-folders")
    p_cmp.add_argument("--out", default="compare_report.html",
                       help="output HTML report path (default: compare_report.html)")
    p_cmp.set_defaults(func=cmd_compare)

    return parser


def main():
    parser = build_parser()
    args   = parser.parse_args()

    if hasattr(args, "radar") and hasattr(args, "ref") and args.ref is not None:
        if len(args.radar) != len(args.ref):
            parser.error("--radar and --ref must have the same number of files")

    args.func(args)


if __name__ == "__main__":
    main()
