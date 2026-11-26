"""Shared version, schema, threshold, color, and utility constants."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import subprocess
import sys
import time
import warnings
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Sequence, cast

import numpy as np
import pandas as pd

from rvt_trainer.api.common import nan_safe
from rvt_trainer.protocol_contract import (
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    EXPECTED_RADAR_LOG_TAIL,
    LEGACY_V14_COLUMN_COUNT,
    LEGACY_V14_RADAR_LOG_TAIL,
    LEGACY_V15_1_COLUMN_COUNT,
    LEGACY_V15_COLUMN_COUNT,
    RADAR_LOG_COLUMNS,
    SCHEMA_VERSION_MAP,
    V15_NEW_COLUMN_DEFAULTS,
)
from rvt_trainer.runtime_paths import runtime_root

warnings.filterwarnings("ignore")

MODEL_FAMILY_GRADIENT_BOOSTING = "gradient_boosting"
MODEL_FAMILY_CNN_1D = "cnn_1d"
MODEL_FAMILIES = (MODEL_FAMILY_GRADIENT_BOOSTING, MODEL_FAMILY_CNN_1D)
MODEL_BUNDLE_METADATA_FILENAME = "model_bundle_metadata.json"

_TRAINER_LOG: Deque[str] = deque(maxlen=200)

_PACKAGE_ROOT = Path(__file__).resolve().parent
_REPO_ROOT = runtime_root()
_TRAINER_ENTRYPOINT = _REPO_ROOT / "radar_vital_trainer.py"


def _trainer_command_prefix() -> List[str]:
    if getattr(sys, "frozen", False):
        return [sys.executable]
    if _TRAINER_ENTRYPOINT.is_file():
        return [sys.executable, str(_TRAINER_ENTRYPOINT)]
    return [sys.executable, "-m", "rvt_trainer"]

VERSION = "16.9.0"
DASHBOARD_VERSION = "16.9.0"
FIRMWARE_VERSION_EXPECTED = "v16.9.0"
UPDATE_MANIFEST_URL = "https://blayalems.github.io/radar-vital-mmwave-60-davao/rvt-latest.json"

MAX_JSON_BODY_BYTES = 1_048_576
DEFAULT_RADAR_PORT = "COM10"
DEFAULT_BLE_ADDRESS = "10:22:33:9E:8F:63"
FEATURE_ENGINEERING_VERSION = "v11.0-physio-2026"
FEATURE_SCHEMA_VERSION = "v11.0-feature-categories-2026"
CONTROL_API_SCHEMA_VERSION = "rvt-control-api-v12.0"
SESSION_NOTES_SCHEMA_VERSION = "rvt-session-notes-v12.0"
SESSION_SIGNOFF_SCHEMA_VERSION = "rvt-session-signoff-v12.0"
TRAINING_PROGRESS_SCHEMA_VERSION = "rvt-training-progress-v12.0"
LIVE_EVENT_SCHEMA_VERSION = "rvt-live-events-v12.0"
SESSION_MANIFEST_SCHEMA_VERSION = "rvt-session-manifest-v1"
SESSION_MANIFEST_VERSION = "v16.5.9-session-manifest-2026-08-03"
CHART_ANNOTATIONS_SCHEMA_VERSION = "rvt-chart-annotations-v12.0"
FEATURE_FLAGS = {
    "enable_sse": True,
    "enable_auto_analyse": True,
    "enable_hc_theme": True,
    "enable_mobile_gestures": True,
    "enable_compare_view": True,
}
REFERENCE_PHYSIO_HR_RANGE = (30.0, 220.0)
REFERENCE_PHYSIO_RR_RANGE = (4.0, 60.0)
FIRMWARE_HR_PUBLISH_RANGE = (40.0, 180.0)
FIRMWARE_RR_PUBLISH_RANGE = (8.0, 35.0)
HR_RANGE = REFERENCE_PHYSIO_HR_RANGE
RR_RANGE = REFERENCE_PHYSIO_RR_RANGE
BLE_PI_QUALITY_THRESHOLD = 1.0
SUBJECT_PROFILE_SCHEMA_VERSION = "rvt-subject-profiles-v12.0"
DEFAULT_SUBJECT_PROFILES = {
    "adult_default": {
        "label": "Adult Default",
        "age_group": "adult",
        "fitness_level": "typical",
        "expected_hr_range": [50, 120],
        "notes": "General adult research profile; protocol thresholds still take precedence.",
    },
    "adult_athlete": {
        "label": "Adult Athlete",
        "age_group": "adult",
        "fitness_level": "athlete",
        "expected_hr_range": [40, 140],
        "notes": "Allows wider resting HR variability for trained subjects; not a diagnostic profile.",
    },
}
SECONDARY_GATE_MIN_DURATION_S = 480.0
SECONDARY_GATE_MIN_PAIRS = 60
ML_READY_MIN_HR_VALID_PAIRS = 120
ML_READY_MIN_HR_COVERAGE_PCT = 50.0
ML_READY_MIN_HR_MAE_BPM = 8.0
ML_READY_MIN_HR_BIAS_BPM = 5.0
ML_READY_MIN_RR_COVERAGE_PCT = 70.0
ML_READY_MIN_RR_MAE_BPM = 4.0
ML_READY_MIN_BLE_COVERAGE_PCT = 70.0

SCORING_WEIGHTS = {
    "internal": {
        "ml_gate_passed": 30.0,
        "hr_r": 20.0,
        "hr_rmse": 20.0,
        "pqi_lock": 20.0,
        "hr_coverage": 10.0,
        "hr_r_gate": 0.4,
        "hr_rmse_gate": 8.0,
    },
    "session_with_reference": {
        "hr_rmse": 18.0,
        "rr_rmse": 18.0,
        "hr_r": 14.0,
        "rr_r": 14.0,
        "hr_bias": 10.0,
        "rr_bias": 10.0,
        "hr_coverage": 8.0,
        "rr_coverage": 8.0,
        "ml_gate_bonus": 10.0,
    },
}

HR_GATE_REASON_NAMES = {0:"OK",1:"NO_AUTO",2:"PQI",3:"HARMONIC",4:"RAW_DISAGREE",5:"SPEC_REJECT",6:"MOTION",7:"DISPLAY_ONLY",8:"RAW_FALLBACK",9:"UPWARD_CONFIRM"}
RR_GATE_REASON_NAMES = {0:"OK",1:"NO_AUTO",2:"PQI",3:"CONF",4:"RAW_DISAGREE",5:"SUBHARMONIC",6:"HARMONIC",7:"MOTION",8:"HOLDOFF",9:"DISPLAY_ONLY"}
HR_PUBLISH_REASON_NAMES = {0:"OK",1:"NO_HUMAN",2:"TRUST_STALE",3:"NO_EVIDENCE",4:"STATE",5:"AGE",6:"STALE",7:"PQI",8:"CONF",9:"HARMONIC",10:"RAW_DISAGREE",11:"OTHER",12:"GRACE_BLOCKED",13:"PHASE_HOLDOFF",14:"STALE_FROZEN",15:"RAW_PHASE_STALE_NO_COMPARISON"}
RR_PUBLISH_REASON_NAMES = {0:"OK",1:"NO_HUMAN",2:"TRUST_STALE",3:"NO_EVIDENCE",4:"STALE",5:"PQI",6:"HOLDOFF",7:"SOURCE",8:"OTHER"}
RR_SOURCE_NAMES = {0:"none",1:"phase",2:"raw_fallback",3:"latched_accept"}
RR_SOURCE_REJECT_REASON_NAMES = {0:"OK",1:"NO_CURRENT_OK",2:"NO_LATCH",3:"LATCH_STALE",4:"CANDIDATE_STALE",5:"POST_MOTION"}
PUBLISH_BLOCK_STAGE_NAMES = {0:"OK",1:"NO_HUMAN",2:"TRUST",3:"STATE",4:"EVIDENCE",5:"PHASE",6:"FRESHNESS",7:"PQI",8:"CONF",9:"HARMONIC",10:"RAW_OR_SOURCE",11:"OTHER"}
HR_RAW_DISAGREE_SUBREASON_NAMES = {0:"NONE",1:"HIGH_BIAS",2:"PHASE_STALE",3:"ANCHOR_DRIFT",4:"LOW_DYNAMIC_RANGE",5:"OTHER",6:"SUMFREQ_OR_RR_HARMONIC"}
HR_PUBLISH_SOURCE_NAMES = {0:"none",1:"current_phase",2:"current_raw",3:"latched_anchor",4:"blend",5:"internal_only",8:"corrected_raw_rescue"}
HR_PUBLISH_SOURCE_NAMES_INVERSE = {v: k for k, v in HR_PUBLISH_SOURCE_NAMES.items()}
RR_PUBLISH_SOURCE_NAMES = {0:"none",1:"current",2:"latched",3:"internal_only"}
HR_RESCUE_PLAUSIBLE_MIN_BPM = 40.0
HR_RESCUE_PLAUSIBLE_MAX_BPM = 200.0
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
LEGACY_RADAR_LOG_COLUMN_COUNT = 131
LEGACY_RADAR_LOG_TAIL = ("hr_path_source", "fw_major", "fw_sub", "fw_mod")

__all__ = [
    "EXPECTED_RADAR_LOG_COLUMN_COUNT",
    "EXPECTED_RADAR_LOG_TAIL",
    "LEGACY_RADAR_LOG_COLUMN_COUNT",
    "LEGACY_RADAR_LOG_TAIL",
    "LEGACY_V14_COLUMN_COUNT",
    "LEGACY_V14_RADAR_LOG_TAIL",
    "LEGACY_V15_1_COLUMN_COUNT",
    "LEGACY_V15_COLUMN_COUNT",
    "RADAR_LOG_COLUMNS",
    "SCHEMA_VERSION_MAP",
    "V15_NEW_COLUMN_DEFAULTS",
]

MODEL_MANIFEST_FILENAME = "model_manifest.json"
MODEL_MANIFEST_SCHEMA_VERSION = "rvt-model-manifest-v1"
MODEL_ARTIFACT_NAMES = (
    "model_hr.pkl",
    "model_rr.pkl",
    "preprocessor.pkl",
    MODEL_BUNDLE_METADATA_FILENAME,
)
MODEL_SIGNING_KEY_ENV = "RVT_MODEL_SIGNING_KEY"
REQUIRE_MODEL_MANIFEST_ENV = "RVT_REQUIRE_MODEL_MANIFEST"
REQUIRE_MODEL_SIGNATURE_ENV = "RVT_REQUIRE_MODEL_SIGNATURE"
CONFIRMATORY_RUN_MANIFEST_SCHEMA_VERSION = "rvt-confirmatory-run-manifest-v1"
CONFIRMATORY_ANALYSIS_PLAN_ID = "RVT-STA-PLAN-16.5.8"
ALIGNMENT_REPORT_SCHEMA_VERSION = "rvt-alignment-report-v1"
SYNC_ANCHORS_SCHEMA_VERSION = "rvt-sync-anchors-v1"

HR_VALID_PUBLISH_OK = {0}
RR_VALID_PUBLISH_OK = {0}
HR_VALID_MAX_AGE_MS = 5000
RR_VALID_MAX_AGE_MS = 5000
MIN_PHASE_VALID_RUN_LEN = 3

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


def _iso_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _report_stamp() -> str:
    """Human-readable, unambiguous UTC stamp for report headers / 'generated_at'.

    Uses time.gmtime() so the value is always UTC regardless of the host's
    local timezone, and labels it explicitly with a trailing ' UTC' so
    downstream readers cannot mistake it for naive local time.
    """
    return time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime())


def _pid_alive(pid) -> bool:
    try:
        pid_i = int(pid)
    except Exception:
        return False
    if pid_i <= 0:
        return False
    if pid_i == os.getpid():
        return True
    try:
        os.kill(pid_i, 0)
        return True
    except Exception:
        return False


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


def write_text_report(path: str, info: Dict):
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"RADAR VITAL TRAINER REPORT v{VERSION}\n")
        f.write("=" * 72 + "\n")
        f.write(f"Generated: {_report_stamp()}\n\n")
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


def _safe_int(value, default: int = 0) -> int:
    try:
        n = int(round(float(value)))
        return n
    except Exception:
        return int(default)


def _safe_float_val(v, fmt=".2f") -> str:
    if v is None:
        return "-"
    try:
        f = float(v)
        return "-" if not np.isfinite(f) else format(f, fmt)
    except Exception:
        return "-"


def _to_num(v, default=float("nan")):
    try:
        if v is None or v == "":
            return default
        return float(v)
    except Exception:
        return default


def _clip01(x: float) -> float:
    return max(0.0, min(1.0, float(x)))


def _row_get(row: Optional[Dict[str, str]], *keys: str):
    row = row or {}
    for key in keys:
        if key in row and row.get(key) not in (None, ""):
            return row.get(key)
    return None


def _row_text(row: pd.Series, *names: str) -> str:
    for name in names:
        if name not in row.index:
            continue
        value = row.get(name)
        if value is None:
            continue
        text = str(value).strip()
        if text and text.lower() not in {"nan", "none"}:
            return text
    return ""


def _row_num(row: pd.Series, *names: str, default: float = float("nan")) -> float:
    for name in names:
        if name not in row.index:
            continue
        value = _to_num(row.get(name), default=float("nan"))
        if np.isfinite(value):
            return float(value)
    return float(default)


def _json_hash(obj: object) -> str:
    blob = json.dumps(nan_safe(obj), sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


def _file_sha256(path: str) -> Optional[str]:
    if not path or not os.path.exists(path) or not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _env_flag_enabled(name: str) -> bool:
    """Truthy parse of an env flag: 1/true/yes/on, case-insensitive, whitespace-trimmed."""
    return str(os.environ.get(name, "")).strip().lower() in {"1", "true", "yes", "on"}


def _scoring_weights_hash() -> str:
    return _json_hash(SCORING_WEIGHTS)


def _sanitize_user_string(value, max_len: int = 1000) -> str:
    text = str(value or "")[:max_len]
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _schema_wrap(payload: Dict[str, object], schema_version: str = CONTROL_API_SCHEMA_VERSION) -> Dict[str, object]:
    if isinstance(payload, dict) and "schema_version" not in payload:
        payload = dict(payload)
        payload["schema_version"] = schema_version
    return payload


def _cleanup_json_temp_files(root: str):
    try:
        for path in Path(root).rglob(".codex-json-*.tmp"):
            try:
                path.unlink()
            except Exception:
                pass
    except Exception:
        pass


def _append_trainer_log(line: str):
    stamp = _report_stamp()
    _TRAINER_LOG.append(f"{stamp} {line}")


def _is_relative_to_path(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def _first_existing(paths: Sequence[str]) -> Optional[str]:
    for path in paths:
        if path and os.path.exists(path):
            return path
    return None


def _session_duration_s(df: pd.DataFrame) -> float:
    if not isinstance(df, pd.DataFrame) or len(df) == 0 or "timestamp_s" not in df.columns:
        return float("nan")
    ts = pd.to_numeric(df["timestamp_s"], errors="coerce").dropna()
    if len(ts) == 0:
        return float("nan")
    return float(ts.max() - ts.min()) if len(ts) > 1 else 0.0


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


def _iso_from_mtime(path: Path) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(path.stat().st_mtime))


def _numeric_or_none(value: object) -> Optional[float]:
    try:
        numeric = float(cast(Any, value))
    except Exception:
        return None
    return numeric if math.isfinite(numeric) else None


def _summary_value(*values):
    for v in values:
        if v not in (None, ""):
            return v
    return None


def _decimate_records(records: List[Dict[str, object]], max_points: int) -> List[Dict[str, object]]:
    if max_points <= 0 or len(records) <= max_points:
        return records
    step = max(1, int(np.ceil(len(records) / float(max_points))))
    return records[::step][:max_points]
