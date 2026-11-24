"""Live session dashboard HTML/JSON rendering and local preview server."""

from __future__ import annotations

import csv
import os
import re
import sys
import threading
import time
from collections import deque
from functools import lru_cache, partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Sequence

import numpy as np
import pandas as pd

from rvt_trainer.api.common import (
    atomic_write_json as save_json,
    nan_safe,
    read_json_if_exists as _read_json_if_exists,
)
from rvt_trainer.constants import (
    HARMONIC_MODE_BIT_LABELS,
    HEART_PQI_GATE_FAR,
    HEART_PQI_GATE_MID,
    HEART_PQI_GATE_NEAR,
    HR_ANCHOR_SOURCE_NAMES,
    HR_CONFIDENCE_SOURCE_NAMES,
    HR_GATE_REASON_NAMES,
    HR_PATH_SOURCE_NAMES,
    HR_PUBLISH_REASON_NAMES,
    HR_PUBLISH_SOURCE_NAMES,
    HR_RAW_DISAGREE_SUBREASON_NAMES,
    LIVE_EVENT_SCHEMA_VERSION,
    PUBLISH_BLOCK_STAGE_NAMES,
    RR_GATE_REASON_NAMES,
    RR_PUBLISH_REASON_NAMES,
    RR_PUBLISH_SOURCE_NAMES,
    RR_SOURCE_NAMES,
    RR_SOURCE_REJECT_REASON_NAMES,
    SESSION_PHASE_NAMES,
    VERSION,
    _REPO_ROOT,
    _bold,
    _row_get,
    _to_num,
    warn,
)
from rvt_trainer.session._legacy_dashboard_template import _RAW_LEGACY_DASHBOARD_TEMPLATE

class _CSVAppendTracker:
    def __init__(self, path: str, keep_rows: int = 240):
        self.path = path
        self.position = 0
        self.header: Optional[List[str]] = None
        self.partial = ""
        self.rows = 0
        self.last_row: Optional[Dict[str, str]] = None
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


def _dashboard_html_path(session_dir: str) -> str:
    return os.path.join(session_dir, "live_dashboard.html")


def _dashboard_json_path(session_dir: str) -> str:
    return os.path.join(session_dir, "live_dashboard.json")


_DASHBOARD_TEMPLATE_EMBEDDED = _RAW_LEGACY_DASHBOARD_TEMPLATE

_DASHBOARD_HTML_NAME = "radar_vital_live_dashboard.html"
_LEGACY_DASHBOARD_HTML_NAMES = (
    "radar_vital_live_dashboard_v12_for_v16_0.html",
    "radar_vital_live_dashboard_v10_for_v14_1.html",
    "radar_vital_live_dashboard_v10_for_v14_1_UIUX_REVISED_EXTERNAL_RESTORED_ICONS_FIXED_PHASES_COMPLETE.html",
    "radar_vital_live_dashboard_v9_1_for_v14_1.html",
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
)
_DASHBOARD_HTML_FALLBACK_NAMES = (
    (_DASHBOARD_HTML_NAME,) + _LEGACY_DASHBOARD_HTML_NAMES
    if os.environ.get("RVT_ALLOW_LEGACY_DASHBOARD_FALLBACK") == "1"
    else (_DASHBOARD_HTML_NAME,)
)


@lru_cache(maxsize=1)
def _dashboard_html_template_paths() -> List[Path]:
    base = _REPO_ROOT
    cwd = Path(os.getcwd())
    out: List[Path] = []
    seen = set()
    for root in (base, cwd):
        for name in _DASHBOARD_HTML_FALLBACK_NAMES:
            p = (root / str(name)).resolve()
            key = str(p).lower()
            if key in seen:
                continue
            seen.add(key)
            out.append(p)
    return out


def _dashboard_html_template_path() -> Optional[Path]:
    for path in _dashboard_html_template_paths():
        if path.exists():
            return path
    return None


def _dashboard_template_candidates() -> List[Path]:
    return _dashboard_html_template_paths()


_dashboard_template_path = _dashboard_html_template_path()
if _dashboard_template_path is not None:
    try:
        _DASHBOARD_TEMPLATE_EMBEDDED = _dashboard_template_path.read_text(encoding="utf-8")
    except Exception:
        pass
_DASHBOARD_TEMPLATE_EMBEDDED = _DASHBOARD_TEMPLATE_EMBEDDED.replace("v13.8.0 / v8.4.0", "v15 / v11")


@lru_cache(maxsize=1)
def _load_dashboard_template_text() -> str:
    """Load and cache the dashboard HTML template."""
    for path in _dashboard_html_template_paths():
        if path.exists():
            try:
                return Path(path).read_text(encoding="utf-8")
            except Exception:
                pass
    return _DASHBOARD_TEMPLATE_EMBEDDED


class _SilentDashboardHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def log_message(self, format, *args):
        pass


class _DashboardServer:
    def __init__(self, session_dir: str, port: int = 8765):
        self.session_dir = os.path.abspath(session_dir)
        handler = partial(_SilentDashboardHandler, directory=self.session_dir)
        self.httpd = ThreadingHTTPServer(("127.0.0.1", int(port)), handler)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.httpd.server_port}/live_dashboard.html?v={int(time.time())}"

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
    empty: Dict[str, List[object]] = {
        "t": [],
        "radar_hr": [], "reported_hr": [], "candidate_hr": [], "raw_hr": [], "raw_hr_uncorrected": [], "raw_hr_corrected": [], "radar_hr_valid": [],
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
        "phase_warmup_complete": [], "clutter_warmup_count": [], "rewarm_triggered": [], "rewarm_reason": [], "current_clutter_alpha": [], "phase_valid_this_frame": [], "dsp_ran_this_frame": [], "hr_confidence_source": [],
        "use_fast_path": [], "hr_path_source": [], "module_fw_major": [], "module_fw_sub": [], "module_fw_mod": [], "sketch_major": [], "sketch_sub": [], "sketch_mod": [],
        "hr_trusted_phase_anchor": [], "hr_anchor_source": [], "hr_anchor_err_bpm": [], "hr_raw_high_bias_suspect": [],
        "rr_seed_from_raw_used": [], "rr_phase_backed_publish_ready": [], "hr_raw_age_ms": [], "hr_raw_disagree_subreason": [],
        "rr_source_current_ok": [], "logged_rr_valid_current": [], "logged_rr_valid_latched": [],
        "hr_publish_source_class": [], "rr_publish_source_class": [], "hr_freeze_suspect": [],
        "hr_value_frozen_confirmed": [], "hr_freeze_duration_ms": [], "hr_no_fresh_update_duration_ms": [],
        "hr_raw_disagree_anchor_drift_suspect": [], "hr_raw_disagree_phase_stale_suspect": [],
        "hr_raw_disagree_high_bias_suspect": [], "hr_raw_disagree_low_dynamic_range_suspect": [],
        "hr_raw_disagree_sumfreq_suspect": [], "hr_raw_disagree_rr_harmonic_k": [],
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
        "raw_hr_uncorrected": _coerce_numeric_series(rad_df, "raw_hr_uncorrected", "raw_hr").round(3).tolist(),
        "raw_hr_corrected": _coerce_numeric_series(rad_df, "raw_hr_corrected").round(3).tolist(),
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
        "hr_trust_fresh": _coerce_boolish_series(rad_df, "hr_trust_fresh", "trusted_hr_fresh").clip(0, 1).tolist(),
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
        "rr_phase_backed_publish_ready": _coerce_boolish_series(rad_df, "rr_phase_backed_publish_ready").clip(0, 1).tolist(),
        "hr_raw_age_ms": _coerce_numeric_series(rad_df, "hr_raw_age_ms").round(3).tolist(),
        "hr_raw_disagree_subreason": _coerce_numeric_series(rad_df, "hr_raw_disagree_subreason").round(3).tolist(),
        "hr_raw_disagree_subreason_name": _decode_enum_series(rad_df, "hr_raw_disagree_subreason", HR_RAW_DISAGREE_SUBREASON_NAMES).tolist(),
        "rr_source_current_ok": _coerce_boolish_series(rad_df, "rr_source_current_ok").clip(0, 1).tolist(),
        "logged_rr_valid_current": _coerce_boolish_series(rad_df, "logged_rr_valid_current").clip(0, 1).tolist(),
        "logged_rr_valid_latched": _coerce_boolish_series(rad_df, "logged_rr_valid_latched").clip(0, 1).tolist(),
        "hr_publish_source_class": _coerce_numeric_series(rad_df, "hr_publish_source_class").round(3).tolist(),
        "rr_publish_source_class": _coerce_numeric_series(rad_df, "rr_publish_source_class").round(3).tolist(),
        "hr_freeze_suspect": _coerce_boolish_series(rad_df, "hr_freeze_suspect").clip(0, 1).tolist(),
        "hr_value_frozen_confirmed": _coerce_boolish_series(rad_df, "hr_value_frozen_confirmed").clip(0, 1).tolist(),
        "hr_freeze_duration_ms": _coerce_numeric_series(rad_df, "hr_freeze_duration_ms").round(3).tolist(),
        "hr_no_fresh_update_duration_ms": _coerce_numeric_series(rad_df, "hr_no_fresh_update_duration_ms").round(3).tolist(),
        "hr_raw_disagree_anchor_drift_suspect": _coerce_boolish_series(rad_df, "hr_raw_disagree_anchor_drift_suspect").clip(0, 1).tolist(),
        "hr_raw_disagree_phase_stale_suspect": _coerce_boolish_series(rad_df, "hr_raw_disagree_phase_stale_suspect").clip(0, 1).tolist(),
        "hr_raw_disagree_high_bias_suspect": _coerce_boolish_series(rad_df, "hr_raw_disagree_high_bias_suspect").clip(0, 1).tolist(),
        "hr_raw_disagree_low_dynamic_range_suspect": _coerce_boolish_series(rad_df, "hr_raw_disagree_low_dynamic_range_suspect").clip(0, 1).tolist(),
        "hr_raw_disagree_sumfreq_suspect": _coerce_boolish_series(rad_df, "hr_raw_disagree_sumfreq_suspect").clip(0, 1).tolist(),
        "hr_raw_disagree_rr_harmonic_k": _coerce_numeric_series(rad_df, "hr_raw_disagree_rr_harmonic_k").round(3).tolist(),
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


def _build_dashboard_faults(radar: Dict[str, Any], ble: Dict[str, Any], thresholds: Dict[str, float]) -> List[Dict[str, str]]:
    faults: List[Dict[str, str]] = []
    hr: Any = radar.get("hr")
    rr: Any = radar.get("rr")
    ref_hr: Any = ble.get("hr")
    ref_rr: Any = ble.get("rr")
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
    if str(hr_publish_reason).upper().replace(" ", "_").find("STALE_FROZEN") >= 0 or radar.get("hr_freeze_suspect") == 1:
        faults.append({
            "severity": "bad",
            "title": "HR stale/frozen suppressed",
            "copy": f"HR publish is blocked by stale/frozen source evidence. Freeze duration={radar.get('hr_freeze_duration_ms')} ms; trust age={radar.get('hr_trust_age_ms')} ms."
        })
    if radar.get("logged_rr_valid") != 1 and np.isfinite(float(radar.get("candidate_rr", float('nan')))):
        faults.append({
            "severity": "warn",
            "title": "RR not published",
            "copy": f"A candidate RR exists but it was not published. Gate reason: {rr_gate_reason or 'unknown'}; publish reason: {rr_publish_reason or 'unknown'}."
        })

    reported_hr: Any = radar.get("reported_hr")
    reported_rr: Any = radar.get("reported_rr")
    if np.isfinite(float(radar.get("candidate_hr", float("nan"))) ) and not (np.isfinite(float(reported_hr)) and float(reported_hr) > 0.0):
        faults.append({
            "severity": "warn",
            "title": "Radar HR is internal only",
            "copy": "The displayed radar HR is an internal smoothed estimate, not a validated published output for this instant."
        })
    if np.isfinite(float(radar.get("candidate_rr", float("nan"))) ) and not (np.isfinite(float(reported_rr)) and float(reported_rr) > 0.0):
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
    template_path = _dashboard_html_template_path()
    target = Path(_dashboard_html_path(session_dir))
    if template_path is not None:
        text = template_path.read_text(encoding="utf-8", errors="ignore")
        text = re.sub(r"(<html\b(?![^>]*data-rvt-archive)[^>]*)>", r'\1 data-rvt-archive="1">', text, count=1)
        text = re.sub(r"\s*<link\s+rel=\"manifest\"\s+href=\"/manifest\.webmanifest\">\s*", "\n", text, count=1)
        target.write_text(text, encoding="utf-8")
        return
    text = _load_dashboard_template_text()
    text = re.sub(r"(<html\b(?![^>]*data-rvt-archive)[^>]*)>", r'\1 data-rvt-archive="1">', text, count=1)
    text = re.sub(r"\s*<link\s+rel=\"manifest\"\s+href=\"/manifest\.webmanifest\">\s*", "\n", text, count=1)
    target.write_text(text, encoding="utf-8")


def _write_live_dashboard_json(session_dir: str, status: str, elapsed: float, remain: Optional[float],
                               radar_tracker: _CSVAppendTracker, ref_tracker: _CSVAppendTracker,
                               raw_tracker: _CSVAppendTracker, recent_events: List[str],
                               analysis_summary: Optional[Dict[str, object]] = None):
    radar_row = radar_tracker.last_row or {}
    ref_row = ref_tracker.last_row or {}
    series = _series_from_trackers(radar_tracker, ref_tracker)
    ble_summary = _read_json_if_exists(os.path.join(session_dir, "ref_ble_summary.json")) or {}
    ble_quality = (
        analysis_summary.get("ble_ref_quality")
        if isinstance(analysis_summary, dict) and isinstance(analysis_summary.get("ble_ref_quality"), dict)
        else None
    )

    logged_hr_now = _to_num(_row_get(radar_row, "logged_hr_valid"), default=0.0) >= 0.5
    logged_rr_now = _to_num(_row_get(radar_row, "logged_rr_valid"), default=0.0) >= 0.5
    reported_hr_now = _to_num(_row_get(radar_row, "reported_hr"), default=float("nan"))
    reported_rr_now = _to_num(_row_get(radar_row, "reported_rr"), default=float("nan"))
    display_hr = reported_hr_now if logged_hr_now and np.isfinite(reported_hr_now) and reported_hr_now > 0.0 else float("nan")
    display_rr = reported_rr_now if logged_rr_now and np.isfinite(reported_rr_now) and reported_rr_now > 0.0 else float("nan")
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
        "reported_hr": reported_hr_now,
        "reported_rr": reported_rr_now,
        "candidate_hr": _to_num(_row_get(radar_row, "candidate_hr"), default=float("nan")),
        "candidate_rr": _to_num(_row_get(radar_row, "candidate_rr"), default=float("nan")),
        "raw_hr": _to_num(_row_get(radar_row, "raw_hr"), default=float("nan")),
        "raw_hr_uncorrected": _to_num(_row_get(radar_row, "raw_hr_uncorrected", "raw_hr"), default=float("nan")),
        "raw_hr_corrected": _to_num(_row_get(radar_row, "raw_hr_corrected"), default=float("nan")),
        "latched_raw_hr": _to_num(_row_get(radar_row, "latched_raw_hr"), default=float("nan")),
        "harmonic_mode": harmonic_mode_live,
        "harmonic_mode_hex": (None if not np.isfinite(harmonic_mode_live) else f"0x{int(round(harmonic_mode_live)):02X}"),
        "harmonic_mode_labels": _harmonic_mode_labels(harmonic_mode_live),
        "session_phase": session_phase_live,
        "session_phase_name": SESSION_PHASE_NAMES.get(session_phase_int, str(session_phase_int)) if session_phase_int is not None else "",
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
        "logged_hr_valid": int(logged_hr_now),
        "logged_rr_valid": int(logged_rr_now),
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
        "rewarm_triggered": int(_to_num(_row_get(radar_row, "rewarm_triggered"), default=0.0) >= 0.5),
        "rewarm_reason": _to_num(_row_get(radar_row, "rewarm_reason"), default=float("nan")),
        "rewarm_reason_name": {1: "agc_floor", 2: "ghost_suspect"}.get(int(_to_num(_row_get(radar_row, "rewarm_reason"), default=-1)), _row_get(radar_row, "rewarm_reason") or ""),
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
        "hr_trust_fresh": int(_to_num(_row_get(radar_row, "hr_trust_fresh", "trusted_hr_fresh"), default=0.0) >= 0.5),
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
        "fs_effective": _to_num(_row_get(radar_row, "fs_effective"), default=float("nan")),
        "fs_snap_used": _to_num(_row_get(radar_row, "fs_snap_used"), default=float("nan")),
        "hr_fs_guard_min": _to_num(_row_get(radar_row, "hr_fs_guard_min"), default=float("nan")),
        "hr_autocorr_best_conf": _to_num(_row_get(radar_row, "hr_autocorr_best_conf"), default=float("nan")),
        "phase_zero_fill_pct": _to_num(_row_get(radar_row, "phase_zero_fill_pct"), default=float("nan")),
        "fs_fallback_used": int(_to_num(_row_get(radar_row, "fs_fallback_used"), default=0.0) >= 0.5),
        "module_fw_valid": int(_to_num(_row_get(radar_row, "module_fw_valid"), default=(
            1.0 if (
                all(np.isfinite(_to_num(_row_get(radar_row, key), default=float("nan"))) and int(_to_num(_row_get(radar_row, key), default=-1)) >= 0
                    for key in ("module_fw_major", "module_fw_sub", "module_fw_mod"))
            ) else 0.0
        )) >= 0.5),
        "hr_trust_age_ms": _to_num(_row_get(radar_row, "hr_trust_age_ms"), default=float("nan")),
        "rr_trust_age_ms": _to_num(_row_get(radar_row, "rr_trust_age_ms"), default=float("nan")),
        "skipdsp_run_len": _to_num(_row_get(radar_row, "skipdsp_run_len"), default=float("nan")),
        "agc_floor_run_len": _to_num(_row_get(radar_row, "agc_floor_run_len"), default=float("nan")),
        "buffer_zero_injected": int(_to_num(_row_get(radar_row, "buffer_zero_injected"), default=0.0) >= 0.5),
        "hr_raw_minus_anchor_bpm": _to_num(_row_get(radar_row, "hr_raw_minus_anchor_bpm"), default=float("nan")),
        "hr_phase_minus_anchor_bpm": _to_num(_row_get(radar_row, "hr_phase_minus_anchor_bpm"), default=float("nan")),
        "hr_raw_minus_phase_bpm": _to_num(_row_get(radar_row, "hr_raw_minus_phase_bpm"), default=float("nan")),
        "rr_candidate_present": int(_to_num(_row_get(radar_row, "rr_candidate_present"), default=0.0) >= 0.5),
        "rr_candidate_source": _to_num(_row_get(radar_row, "rr_candidate_source"), default=float("nan")),
        "rr_candidate_source_name": RR_SOURCE_NAMES.get(int(_to_num(_row_get(radar_row, "rr_candidate_source"), default=-1)), _row_get(radar_row, "rr_candidate_source") or ""),
        "rr_source_reject_reason": _to_num(_row_get(radar_row, "rr_source_reject_reason"), default=float("nan")),
        "rr_source_reject_reason_name": RR_SOURCE_REJECT_REASON_NAMES.get(int(_to_num(_row_get(radar_row, "rr_source_reject_reason"), default=-1)), _row_get(radar_row, "rr_source_reject_reason") or ""),
        "rr_source_latched_ok": int(_to_num(_row_get(radar_row, "rr_source_latched_ok"), default=0.0) >= 0.5),
        "rr_source_current_ok": int(_to_num(_row_get(radar_row, "rr_source_current_ok"), default=0.0) >= 0.5),
        "logged_rr_valid_current": int(_to_num(_row_get(radar_row, "logged_rr_valid_current"), default=0.0) >= 0.5),
        "logged_rr_valid_latched": int(_to_num(_row_get(radar_row, "logged_rr_valid_latched"), default=0.0) >= 0.5),
        "hr_publish_source_class": _to_num(_row_get(radar_row, "hr_publish_source_class"), default=float("nan")),
        "hr_publish_source_class_name": HR_PUBLISH_SOURCE_NAMES.get(int(_to_num(_row_get(radar_row, "hr_publish_source_class"), default=-1)), _row_get(radar_row, "hr_publish_source_class") or ""),
        "rr_publish_source_class": _to_num(_row_get(radar_row, "rr_publish_source_class"), default=float("nan")),
        "rr_publish_source_class_name": RR_PUBLISH_SOURCE_NAMES.get(int(_to_num(_row_get(radar_row, "rr_publish_source_class"), default=-1)), _row_get(radar_row, "rr_publish_source_class") or ""),
        "hr_freeze_suspect": int(_to_num(_row_get(radar_row, "hr_freeze_suspect"), default=0.0) >= 0.5),
        "hr_value_frozen_confirmed": int(_to_num(_row_get(radar_row, "hr_value_frozen_confirmed"), default=0.0) >= 0.5),
        "hr_freeze_duration_ms": _to_num(_row_get(radar_row, "hr_freeze_duration_ms"), default=float("nan")),
        "hr_no_fresh_update_duration_ms": _to_num(_row_get(radar_row, "hr_no_fresh_update_duration_ms"), default=float("nan")),
        "hr_raw_disagree_anchor_drift_suspect": int(_to_num(_row_get(radar_row, "hr_raw_disagree_anchor_drift_suspect"), default=0.0) >= 0.5),
        "hr_raw_disagree_phase_stale_suspect": int(_to_num(_row_get(radar_row, "hr_raw_disagree_phase_stale_suspect"), default=0.0) >= 0.5),
        "hr_raw_disagree_high_bias_suspect": int(_to_num(_row_get(radar_row, "hr_raw_disagree_high_bias_suspect"), default=0.0) >= 0.5),
        "hr_raw_disagree_low_dynamic_range_suspect": int(_to_num(_row_get(radar_row, "hr_raw_disagree_low_dynamic_range_suspect"), default=0.0) >= 0.5),
        "hr_raw_disagree_sumfreq_suspect": int(_to_num(_row_get(radar_row, "hr_raw_disagree_sumfreq_suspect"), default=0.0) >= 0.5),
        "hr_raw_disagree_rr_harmonic_k": _to_num(_row_get(radar_row, "hr_raw_disagree_rr_harmonic_k"), default=float("nan")),
        "rr_phase_backed_publish_ready": int(_to_num(_row_get(radar_row, "rr_phase_backed_publish_ready"), default=0.0) >= 0.5),
        "hr_publish_block_stage": _to_num(_row_get(radar_row, "hr_publish_block_stage"), default=float("nan")),
        "hr_publish_block_stage_name": PUBLISH_BLOCK_STAGE_NAMES.get(int(_to_num(_row_get(radar_row, "hr_publish_block_stage"), default=-1)), _row_get(radar_row, "hr_publish_block_stage") or ""),
        "rr_publish_block_stage": _to_num(_row_get(radar_row, "rr_publish_block_stage"), default=float("nan")),
        "rr_publish_block_stage_name": PUBLISH_BLOCK_STAGE_NAMES.get(int(_to_num(_row_get(radar_row, "rr_publish_block_stage"), default=-1)), _row_get(radar_row, "rr_publish_block_stage") or ""),
        "hr_raw_age_ms": _to_num(_row_get(radar_row, "hr_raw_age_ms"), default=float("nan")),
        "hr_raw_disagree_subreason": _to_num(_row_get(radar_row, "hr_raw_disagree_subreason"), default=float("nan")),
        "hr_raw_disagree_subreason_name": HR_RAW_DISAGREE_SUBREASON_NAMES.get(int(_to_num(_row_get(radar_row, "hr_raw_disagree_subreason"), default=-1)), _row_get(radar_row, "hr_raw_disagree_subreason") or ""),
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
        "address": ble_summary.get("address") if isinstance(ble_summary, dict) else None,
        "profile": ble_summary.get("ble_profile") if isinstance(ble_summary, dict) else None,
        "quality": ble_quality,
        "packet_loss_pct": (ble_quality or {}).get("packet_loss_pct") if isinstance(ble_quality, dict) else None,
        "decode_error_pct": (ble_quality or {}).get("decode_error_pct") if isinstance(ble_quality, dict) else None,
        "pi_below_threshold_pct": (ble_quality or {}).get("pi_below_threshold_pct") if isinstance(ble_quality, dict) else None,
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
    save_json(payload, os.path.join(session_dir, "dashboard.json"))


def _render_live_dashboard(session_dir: str, start_t: float, duration_s: Optional[float], radar_tracker: _CSVAppendTracker,
                           ref_tracker: _CSVAppendTracker, raw_tracker: _CSVAppendTracker, recent_events: Deque[str],
                           event_lock: threading.Lock, status: str,
                           analysis_summary: Optional[Dict[str, object]] = None):
    elapsed = max(0.0, time.time() - start_t)
    radar = radar_tracker.last_row or {}
    ref = ref_tracker.last_row or {}
    remain = None if duration_s is None else max(0.0, duration_s - elapsed)
    lines = []
    lines.append(_bold(f"Radar Vital Trainer v{VERSION} - Live Session Dashboard"))
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


def _start_dashboard_server(session_dir: str, port: int = 8765) -> _DashboardServer:
    server = _DashboardServer(session_dir, port=port)
    server.start()
    return server


def _mock_live_payload(seq: int = 0, window_s: int = 60) -> Dict[str, object]:
    now = time.time()
    n = max(30, min(600, int(window_s)))
    ts = np.linspace(max(0.0, now - n), now, n)
    phase = ts - ts[0]
    hr = 72.0 + 5.0 * np.sin(phase / 7.5) + 1.5 * np.sin(phase / 2.7)
    rr = 15.0 + 1.2 * np.sin(phase / 9.0)
    pqi = np.clip(0.55 + 0.35 * np.sin(phase / 11.0), 0.05, 0.95)
    return {
        "schema_version": LIVE_EVENT_SCHEMA_VERSION,
        "revision": int(now * 1000),
        "session_id": "mock",
        "meta": {
            "status": "mock",
            "session_id": "mock",
            "elapsed_s": float(phase[-1]) if len(phase) else 0.0,
            "remaining_s": None,
            "note": "Integrated mock sensor mode",
        },
        "radar": {
            "reported_hr": float(hr[-1]),
            "reported_rr": float(rr[-1]),
            "pqi_heart": float(pqi[-1]),
            "pqi_breath": float(pqi[-1]),
            "distance_cm": 62.0 + 2.0 * np.sin(phase[-1] / 13.0),
            "fps": 20.0,
        },
        "series": {
            "t": [float(x - ts[0]) for x in ts],
            "reported_hr": [float(x) for x in hr],
            "reported_rr": [float(x) for x in rr],
            "pqi": [float(x) for x in pqi],
            "confidence": [float(x) for x in pqi],
        },
        "events": [f"[MOCK] virtual radar frame {seq}"],
    }


def _idle_live_payload(sessions_root: str, message: str = "No active telemetry session.") -> Dict[str, object]:
    return {
        "schema_version": LIVE_EVENT_SCHEMA_VERSION,
        "revision": int(time.time() * 1000),
        "session_id": None,
        "meta": {
            "status": "waiting",
            "session_id": None,
            "active": False,
            "elapsed_s": 0.0,
            "remaining_s": None,
            "version": VERSION,
            "session_dir": os.path.abspath(sessions_root),
            "note": message,
        },
        "radar": {"rows": 0},
        "ble": {"rows": 0, "raw_packets": 0},
        "thresholds": {},
        "faults": [],
        "events": [f"[INFO] {message}"],
        "series": {},
        "analysis": None,
    }
