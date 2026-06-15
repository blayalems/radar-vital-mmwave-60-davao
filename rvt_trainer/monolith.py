"""radar_vital_trainer_v12_for_v16_0.py - v12.0.0
=====================================================
Leakage-aware offline trainer + full session pipeline for the
XIAO ESP32-C6 + MR60BHA2 system.

v11 keeps the v15 telemetry contract and adds matched trainer-side truthfulness, manifest, leakage-firewall, bootstrap-comparison, and dashboard support.
Material 3 Expressive live dashboard stack for thesis use:
- session orchestration remains the primary collection workflow
- live dashboarding, regression QA, and stronger analysis outputs remain intact
- stale v6 command references are removed from help text and quickstart
- version strings and guidance are aligned with the current file

Change highlights
────────────────
  - v14 telemetry contract: v13.9.7 tail plus explicit RR current/latch source, publish-source, freeze, and RAW_DISAGREE audit fields.
  - trainer validity masks now repair RR current-source aggregation and label metric basis / evaluability explicitly.
  - write_text_report now forces UTF-8 so Windows auto-analyse runs do not crash on Unicode arrows in the text report
  - training now defaults away from policy-leaking publish/gate fields for physiology regression and emits explicit audit masks.
  - dashboard loader prefers a same-version external HTML template when present; embedded template is also patched
  - contributions blended from Opus 4.6/4.7, Sonnet 4.5/4.6, Claude Ultrareview, ChatGPT 5.4 Codex/Pro, Gemini, Grok, GLM, Muse, DeepSeek, and Qwen audits, with only code-backed or low-risk changes applied.

Primary workflow
────────────────
  1.  python radar_vital_trainer_v12_for_v16_0.py doctor
  2.  python radar_vital_trainer_v12_for_v16_0.py quickstart

  Per session (single command):
      python radar_vital_trainer_v12_for_v16_0.py session \
        --port COM10 \
        --address 10:22:33:9E:8F:63 \
        --duration-s 480 \
        --open-dashboard

  After session:
      python radar_vital_trainer_v12_for_v16_0.py compare --sessions-dir sessions/ --out report.html

  Optional manual workflow:
      log / ble_reflog / align / analyse / train / sweep
"""

import argparse
import asyncio
import base64
import hashlib
import inspect
import importlib.util
import json
import mimetypes
import os
import pickle
import re
import secrets
import signal
import shutil
import socket
import ssl
import subprocess
import sys
import tempfile
import time
import threading
import csv
import urllib.request
from collections import deque
from functools import lru_cache, partial
from html import escape as html_escape
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple
from urllib.parse import parse_qs, unquote, urlparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.metrics import mean_absolute_error, mean_squared_error

import warnings
warnings.filterwarnings("ignore")

_PACKAGE_ROOT = Path(__file__).resolve().parent
_REPO_ROOT = _PACKAGE_ROOT.parent
_TRAINER_ENTRYPOINT = _REPO_ROOT / "radar_vital_trainer_v12_for_v16_0.py"

VERSION = "16.3.0"
DASHBOARD_VERSION = "16.3.0"
FIRMWARE_VERSION_EXPECTED = "v16.3.0"
UPDATE_MANIFEST_URL = "https://blayalems.github.io/radar-vital-mmwave-60-davao/rvt-latest.json"

# Hard upper bound for JSON control-API request bodies. The control surface only
# ever carries small JSON objects (PINs, session metadata, notes capped at a few
# thousand chars). A 1 MiB ceiling is generous for those payloads while turning a
# client-supplied Content-Length into a bounded allocation instead of an
# unbounded ``rfile.read(n)`` memory-exhaustion vector.
MAX_JSON_BODY_BYTES = 1_048_576
_manifest_cache = {"data": None, "ts": 0}
_manifest_cache_lock = threading.Lock()
DEFAULT_RADAR_PORT = "COM10"
DEFAULT_BLE_ADDRESS = "10:22:33:9E:8F:63"
FEATURE_ENGINEERING_VERSION = "v11.0-physio-2026"
FEATURE_SCHEMA_VERSION = "v11.0-feature-categories-2026"
CONTROL_API_SCHEMA_VERSION = "rvt-control-api-v12.0"
SESSION_NOTES_SCHEMA_VERSION = "rvt-session-notes-v12.0"
SESSION_SIGNOFF_SCHEMA_VERSION = "rvt-session-signoff-v12.0"
TRAINING_PROGRESS_SCHEMA_VERSION = "rvt-training-progress-v12.0"
LIVE_EVENT_SCHEMA_VERSION = "rvt-live-events-v12.0"
SESSION_MANIFEST_SCHEMA_VERSION = "rvt-session-manifest-v12.0"
SESSION_MANIFEST_VERSION = "v12-session-manifest-2026-05-15"
CHART_ANNOTATIONS_SCHEMA_VERSION = "rvt-chart-annotations-v12.0"
FEATURE_FLAGS = {
    "enable_sse": True,
    "enable_auto_analyse": True,
    "enable_hc_theme": True,
    "enable_mobile_gestures": True,
    "enable_compare_view": True,
}
# These are physiological/reference/model-output bounds, not firmware publish
# limits. Keep them wider than the v14.1 firmware publish range so BLE truth at
# the edges remains visible instead of being clipped out of evaluation.
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
FEATURE_CATEGORY: Dict[str, str] = {}

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

# F1.1 plausibility gate bounds (mirror of firmware HR_MIN / HR_MAX).
HR_RESCUE_PLAUSIBLE_MIN_BPM = 40.0
HR_RESCUE_PLAUSIBLE_MAX_BPM = 200.0


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
    "raw_hr_uncorrected",
    "raw_hr_corrected",
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
    "rewarm_triggered",
    "rewarm_reason",
    "experimental_profile_enabled",
    "fs_effective",
    "fs_snap_used",
    "hr_fs_guard_min",
    "hr_autocorr_best_conf",
    "phase_zero_fill_pct",
    "fs_fallback_used",
    "module_fw_valid",
    "hr_trust_age_ms",
    "rr_trust_age_ms",
    "skipdsp_run_len",
    "agc_floor_run_len",
    "buffer_zero_injected",
    "hr_raw_minus_anchor_bpm",
    "hr_phase_minus_anchor_bpm",
    "hr_raw_minus_phase_bpm",
    "rr_candidate_present",
    "rr_candidate_source",
    "rr_source_reject_reason",
    "rr_source_latched_ok",
    "hr_publish_block_stage",
    "rr_publish_block_stage",
    "hr_raw_age_ms",
    "hr_raw_disagree_subreason",
    "rr_phase_backed_publish_ready",
    "rr_source_current_ok",
    "logged_rr_valid_current",
    "logged_rr_valid_latched",
    "hr_publish_source_class",
    "rr_publish_source_class",
    "hr_freeze_suspect",
    "hr_value_frozen_confirmed",
    "hr_freeze_duration_ms",
    "hr_no_fresh_update_duration_ms",
    "hr_raw_disagree_anchor_drift_suspect",
    "hr_raw_disagree_phase_stale_suspect",
    "hr_raw_disagree_high_bias_suspect",
    "hr_raw_disagree_low_dynamic_range_suspect",
    "hr_raw_disagree_sumfreq_suspect",
    "hr_raw_disagree_rr_harmonic_k",
    # v15.0 Stage A: 8 reserved columns (final-schema-first).
    # Stage B activates col 200; Stage C activates 201-204; Stage E/v12 activates 205-207.
    # v15.1 appends field diagnostics in cols 208-219.
    # Until activated, columns emit safe defaults that the trainer parser excludes
    # from the relevant analysis sub-sections (NaN for floats, 0 for ints).
    "presence_fsm_debug",            # col 200, Stage B
    "pqi_heart_v15",                 # col 201, Stage C
    "pqi_breath_v15",                # col 202, Stage C
    "phase_buffer_valid_pct",        # col 203, Stage C
    "pqi_v15_pair_coverage_min",     # col 204, Stage C
    "correction_shadow_delta_bpm",   # col 205, Stage E
    "correction_source",             # col 206, Stage E/v12
    "correction_params_hash",        # col 207, Stage E/v12
    "loop_dt_mean_ms",               # col 208, v15.1
    "loop_dt_max_ms",                # col 209, v15.1
    "heap_free_kb",                  # col 210, v15.1
    "heap_min_free_kb",              # col 211, v15.1
    "radar_uart_overflow_count",     # col 212, v15.1
    "radar_crc_err_count",           # col 213, v15.1
    "i2c_recover_count",             # col 214, v15.1
    "lcd_reinit_count",              # col 215, v15.1
    "wdt_near_miss_count",           # col 216, v15.1
    "cmd_rx_count",                  # col 217, v15.1
    "cmd_err_count",                 # col 218, v15.1
    "fw_uptime_s",                   # col 219, v15.1
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
# v11/v15: final-schema-first. v15.1 firmware emits 219 columns.
# The trainer accepts 207 and v14.1 rows; missing right-edge columns are padded.
EXPECTED_RADAR_LOG_COLUMN_COUNT = 219
LEGACY_V15_COLUMN_COUNT = 207
LEGACY_V14_COLUMN_COUNT = 199
SCHEMA_VERSION_MAP = {
    LEGACY_V14_COLUMN_COUNT: "v14.1",
    LEGACY_V15_COLUMN_COUNT: "v15.0",
    EXPECTED_RADAR_LOG_COLUMN_COUNT: "v15.1",
}
# v15/v15.1 right-edge default values for legacy row padding.
# Float columns use NaN (parser converts to NaN downstream); int columns use 0.
V15_NEW_COLUMN_DEFAULTS = {
    "presence_fsm_debug": 0,
    "pqi_heart_v15": float("nan"),
    "pqi_breath_v15": float("nan"),
    "phase_buffer_valid_pct": float("nan"),
    "pqi_v15_pair_coverage_min": float("nan"),
    "correction_shadow_delta_bpm": float("nan"),
    "correction_source": 0,
    "correction_params_hash": 0,
    "loop_dt_mean_ms": float("nan"),
    "loop_dt_max_ms": float("nan"),
    "heap_free_kb": float("nan"),
    "heap_min_free_kb": float("nan"),
    "radar_uart_overflow_count": 0,
    "radar_crc_err_count": 0,
    "i2c_recover_count": 0,
    "lcd_reinit_count": 0,
    "wdt_near_miss_count": 0,
    "cmd_rx_count": 0,
    "cmd_err_count": 0,
    "fw_uptime_s": 0,
}
EXPECTED_RADAR_LOG_TAIL = (
    "rr_phase_backed_publish_ready",
    "rr_source_current_ok",
    "logged_rr_valid_current",
    "logged_rr_valid_latched",
    "hr_publish_source_class",
    "rr_publish_source_class",
    "hr_freeze_suspect",
    "hr_value_frozen_confirmed",
    "hr_freeze_duration_ms",
    "hr_no_fresh_update_duration_ms",
    "hr_raw_disagree_anchor_drift_suspect",
    "hr_raw_disagree_phase_stale_suspect",
    "hr_raw_disagree_high_bias_suspect",
    "hr_raw_disagree_low_dynamic_range_suspect",
    "hr_raw_disagree_sumfreq_suspect",
    "hr_raw_disagree_rr_harmonic_k",
    # v15.0 Stage A: 8 reserved columns (final-schema-first)
    "presence_fsm_debug",
    "pqi_heart_v15",
    "pqi_breath_v15",
    "phase_buffer_valid_pct",
    "pqi_v15_pair_coverage_min",
    "correction_shadow_delta_bpm",
    "correction_source",
    "correction_params_hash",
    "loop_dt_mean_ms",
    "loop_dt_max_ms",
    "heap_free_kb",
    "heap_min_free_kb",
    "radar_uart_overflow_count",
    "radar_crc_err_count",
    "i2c_recover_count",
    "lcd_reinit_count",
    "wdt_near_miss_count",
    "cmd_rx_count",
    "cmd_err_count",
    "fw_uptime_s",
)
LEGACY_V14_RADAR_LOG_TAIL = (
    "rr_phase_backed_publish_ready",
    "rr_source_current_ok",
    "logged_rr_valid_current",
    "logged_rr_valid_latched",
    "hr_publish_source_class",
    "rr_publish_source_class",
    "hr_freeze_suspect",
    "hr_value_frozen_confirmed",
    "hr_freeze_duration_ms",
    "hr_no_fresh_update_duration_ms",
    "hr_raw_disagree_anchor_drift_suspect",
    "hr_raw_disagree_phase_stale_suspect",
    "hr_raw_disagree_high_bias_suspect",
    "hr_raw_disagree_low_dynamic_range_suspect",
    "hr_raw_disagree_sumfreq_suspect",
    "hr_raw_disagree_rr_harmonic_k",
)


def _detect_csv_schema_version(columns) -> str:
    """Detect firmware schema version from column count.
    Returns 'v15.1' for 219, 'v15.0' for 207, 'v14.1' for 199,
    'unknown-N' otherwise. Used by row padding logic to decide how to fill
    missing right-edge columns.
    """
    try:
        n = len(columns)
    except TypeError:
        return "unknown-?"
    return SCHEMA_VERSION_MAP.get(n, f"unknown-{n}")


def _is_supported_radar_contract_length(length: object) -> bool:
    try:
        n = int(length)
    except Exception:
        return False
    return n in {EXPECTED_RADAR_LOG_COLUMN_COUNT, LEGACY_V15_COLUMN_COUNT, LEGACY_V14_COLUMN_COUNT}


LEGACY_RADAR_LOG_COLUMN_COUNT = 131
LEGACY_RADAR_LOG_TAIL = ("hr_path_source", "fw_major", "fw_sub", "fw_mod")
_FIRMWARE_CONTRACT_CACHE: Optional[Tuple[str, Tuple[str, ...]]] = None


# -----------------------------------------------------------------------------
# Terminal colour helpers (auto-disabled when not a TTY)
# -----------------------------------------------------------------------------

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


# -----------------------------------------------------------------------------
# Helpers (unchanged from v5.0)
# -----------------------------------------------------------------------------

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
        return float(obj) if np.isfinite(obj) else None
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
        "raw_hr_bias_estimate": analyse_summary.get("raw_hr_bias_estimate"),
        "raw_hr_bias_audit": analyse_summary.get("raw_hr_bias_audit", analyse_summary.get("raw_hr_bias_estimate")),
        "ble_ref_quality": analyse_summary.get("ble_ref_quality"),
        "session_manifest": _read_json_if_exists(os.path.join(session_dir, "session_manifest.json")),
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
    best = dict(empty)
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


def _truthfulness_from_radar(raw_df: pd.DataFrame) -> Dict[str, object]:
    out = {
        "version": "unknown",
        "module_version": None,
        "module_version_detected": False,
        "module_version_valid": False,
        "contract_length": int(len(raw_df.columns)) if isinstance(raw_df, pd.DataFrame) else EXPECTED_RADAR_LOG_COLUMN_COUNT,
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
    mod_major = safe_series(raw_df, "module_fw_major", default=np.nan).dropna()
    mod_sub = safe_series(raw_df, "module_fw_sub", default=np.nan).dropna()
    mod_mod = safe_series(raw_df, "module_fw_mod", default=np.nan).dropna()
    critical_cols = {"timestamp_ms", "reported_hr", "reported_rr", "candidate_hr", "candidate_rr", "pqi_heart", "pqi_breath"}
    out["critical_columns_ok"] = critical_cols.issubset(set(raw_df.columns))
    if "module_fw_valid" in raw_df.columns:
        out["module_version_valid"] = bool((safe_series(raw_df, "module_fw_valid", default=0.0).fillna(0.0) >= 0.5).iloc[-1])
    if len(mod_major) and len(mod_sub) and len(mod_mod):
        module_version = _format_sketch_version(mod_major.iloc[-1], mod_sub.iloc[-1], mod_mod.iloc[-1])
        out["module_version"] = module_version
        out["module_version_detected"] = module_version is not None
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


def _session_duration_s(df: pd.DataFrame) -> float:
    if not isinstance(df, pd.DataFrame) or len(df) == 0 or "timestamp_s" not in df.columns:
        return float("nan")
    ts = pd.to_numeric(df["timestamp_s"], errors="coerce").dropna()
    if len(ts) == 0:
        return float("nan")
    return float(ts.max() - ts.min()) if len(ts) > 1 else 0.0


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
    ledger = {
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


def _scoring_weights_hash() -> str:
    return _json_hash(SCORING_WEIGHTS)


def _session_root_for_outputs(out_dir: str) -> str:
    out_abs = os.path.abspath(out_dir)
    if os.path.basename(out_abs).lower() in {"analysis", "analyse"}:
        return os.path.dirname(out_abs)
    return out_abs


def _manifest_identity(fw_truthfulness: Optional[Dict[str, object]] = None) -> Dict[str, object]:
    fw_truthfulness = fw_truthfulness if isinstance(fw_truthfulness, dict) else {}
    return {
        "sketch_version": fw_truthfulness.get("version", "unknown"),
        "module_version": fw_truthfulness.get("module_version"),
        "module_version_valid": bool(fw_truthfulness.get("module_version_valid", False)),
        "trainer_version": VERSION,
        "dashboard_version": DASHBOARD_VERSION,
        "feature_engineering_version": FEATURE_ENGINEERING_VERSION,
        "feature_schema_version": FEATURE_SCHEMA_VERSION,
        "feature_schema_hash": feature_schema_hash(),
        "scoring_weights_hash": _scoring_weights_hash(),
    }


def _warn_on_manifest_mismatch(session_root: str, fw_truthfulness: Optional[Dict[str, object]] = None) -> None:
    manifest_path = os.path.join(session_root, "session_manifest.json")
    existing = _read_json_if_exists(manifest_path)
    if not isinstance(existing, dict):
        return
    current = _manifest_identity(fw_truthfulness)
    mismatches = []
    for key, current_value in current.items():
        old_value = existing.get(key)
        if old_value != current_value:
            mismatches.append((key, old_value, current_value))
    if not mismatches:
        return
    warn("MANIFEST MISMATCH: this analysis is being rerun with code/schema/scoring identity that differs from session_manifest.json.")
    for key, old_value, current_value in mismatches:
        warn(f"MANIFEST MISMATCH: {key}: manifest={old_value!r} current={current_value!r}")


def _write_session_manifest(
    session_root: str,
    radar_paths: Sequence[str],
    ref_paths: Sequence[str],
    analysis_dir: str,
    fw_truthfulness: Optional[Dict[str, object]],
) -> Dict[str, object]:
    session_root = os.path.abspath(session_root)
    analysis_dir = os.path.abspath(analysis_dir)
    os.makedirs(session_root, exist_ok=True)
    outputs = {
        "radar_csv": os.path.abspath(radar_paths[0]) if len(radar_paths) == 1 else [os.path.abspath(p) for p in radar_paths],
        "ref_csv": os.path.abspath(ref_paths[0]) if len(ref_paths) == 1 else [os.path.abspath(p) for p in ref_paths],
        "aligned_1hz_csv": os.path.join(analysis_dir, "aligned_1hz.csv"),
        "aligned_1hz_features_csv": os.path.join(analysis_dir, "aligned_1hz_features.csv"),
        "analyse_summary_json": os.path.join(analysis_dir, "analyse_summary.json"),
        "analyse_report_txt": os.path.join(analysis_dir, "analyse_report.txt"),
        "analyse_report_html": os.path.join(analysis_dir, "analyse_report.html"),
        "quick_report_txt": os.path.join(session_root, "quick_report.txt"),
        "session_quick_report_txt": os.path.join(session_root, "session_quick_report.txt"),
        "dashboard_json": os.path.join(session_root, "dashboard.json"),
        "live_dashboard_json": os.path.join(session_root, "live_dashboard.json"),
        "live_dashboard_html": os.path.join(session_root, "live_dashboard.html"),
    }
    file_hashes: Dict[str, object] = {}
    for key, value in outputs.items():
        if isinstance(value, list):
            file_hashes[key] = {p: _file_sha256(p) for p in value if os.path.exists(p)}
        elif os.path.exists(value):
            file_hashes[key] = _file_sha256(value)
    manifest = {
        "schema_version": SESSION_MANIFEST_SCHEMA_VERSION,
        "manifest_version": SESSION_MANIFEST_VERSION,
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "session_root": session_root,
        **_manifest_identity(fw_truthfulness),
        "auto_analysed": os.path.exists(os.path.join(analysis_dir, "analyse_summary.json")),
        "tags": list((_read_json_if_exists(os.path.join(session_root, "session_manifest.json")) or {}).get("tags", [])),
        "notes_count": len((_read_json_if_exists(os.path.join(session_root, "session_notes.json")) or {}).get("notes", [])),
        "subject_profile_id": (_read_json_if_exists(os.path.join(session_root, "session_manifest.json")) or {}).get("subject_profile_id", "adult_default"),
        "outputs": outputs,
        "file_hashes_sha256": file_hashes,
    }
    save_json(manifest, os.path.join(session_root, "session_manifest.json"))
    return manifest


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
<html lang="en" data-theme="light" data-density="comfortable">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Radar Vital Trainer — Operator Console</title>
<meta name="description" content="Dashboard-first console for trustworthy radar vital-sign monitoring sessions. Configure hardware, run preflight checks, monitor live telemetry, and review ML-readiness verdicts.">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Inter+Tight:wght@500;600;700;800;900&family=Inter:wght@400;500;600;700;800;900&family=JetBrains+Mono:wght@400;500;700;800&display=swap" rel="stylesheet">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Material+Symbols+Rounded:opsz,wght,FILL,GRAD@24,500,1,0" />
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
/* ============================================================
   INSTRUMENT DESIGN SYSTEM — Merged from sandbox prototype
   Palette: desaturated navy / quartz surface / signal cyan accent
   Typography: Inter Tight (headings) · Inter (body) · JetBrains Mono (data)
   ============================================================ */
:root{
  color-scheme:light;
  /* Surface */
  --surface-1:#ffffff; --surface-2:#f6f8fc; --surface-3:#eef2f8;
  --ink-900:#0b1220; --ink-800:#1a2540; --ink-700:#2d3a53;
  --ink-500:#5a6681; --ink-400:#7b889e; --ink-300:#9aa5b9;
  --line:#dce4ef; --line-2:#c4d0e2;
  /* Accent */
  --brand-500:#2563eb; --brand-400:#3b82f6; --brand-100:#dbeafe;
  --brand-050:#eff6ff;
  --signal-500:#06b6d4; --signal-100:#cffafe;
  /* Semantic */
  --ok-500:#0f766e; --ok-100:#ccfbf1;
  --ember-500:#c2410c; --ember-100:#ffedd5;
  --bad-500:#b91c1c; --bad-100:#fee2e2;
  /* Radii */
  --r-sm:6px; --r-md:10px; --r-lg:14px; --r-xl:20px; --r-pill:999px;
  /* Motion */
  --dur-s:120ms; --dur-m:240ms; --dur-l:400ms;
  --ease:cubic-bezier(.2,0,0,1); --ease-emph:cubic-bezier(.3,0,0,1);
  /* Shadows */
  --shadow-1:0 1px 2px rgba(16,24,40,.04),0 1px 3px rgba(16,24,40,.03);
  --shadow-2:0 1px 2px rgba(16,24,40,.06),0 10px 26px -12px rgba(16,24,40,.15);
  --shadow-3:0 2px 6px rgba(16,24,40,.08),0 26px 56px -22px rgba(16,24,40,.22);
  /* Layout */
  --gap:16px; --pad:22px;
  /* Legacy compat tokens (used by existing JS-generated HTML) */
  --md-sys-color-primary:#2563eb; --md-sys-color-on-primary:#fff;
  --md-sys-color-primary-container:#dbeafe; --md-sys-color-on-primary-container:#1e3a5f;
  --md-sys-color-secondary-container:#e8eef8; --md-sys-color-on-secondary-container:#1a2540;
  --md-sys-color-tertiary:#0f766e; --md-sys-color-tertiary-container:#ccfbf1; --md-sys-color-on-tertiary-container:#064e3b;
  --md-sys-color-error:#b91c1c; --md-sys-color-error-container:#fee2e2; --md-sys-color-on-error-container:#7f1d1d;
  --md-sys-color-warning-container:#ffedd5; --md-sys-color-on-warning-container:#7c2d12;
  --md-sys-color-success:#0f766e; --md-sys-color-success-container:#ccfbf1; --md-sys-color-on-success-container:#064e3b;
  --md-sys-color-background:#f6f8fc; --md-sys-color-on-surface:#0b1220;
  --md-sys-color-on-surface-variant:#5a6681;
  --md-sys-color-outline:#7b889e; --md-sys-color-outline-variant:#dce4ef;
  --md-sys-color-surface-container-lowest:#ffffff;
  --md-sys-color-surface-container-low:#f6f8fc;
  --md-sys-color-surface-container:#eef2f8;
  --md-sys-color-surface-container-high:#e4eaf4;
  --md-sys-color-surface-container-highest:#dae2ee;
  --shape-xs:6px; --shape-sm:10px; --shape-md:12px; --shape-lg:16px; --shape-xl:20px; --shape-full:999px;
  --ease-standard:cubic-bezier(.2,0,0,1); --ease-emph:cubic-bezier(.3,0,0,1);
  --elev-1:var(--shadow-1); --elev-2:var(--shadow-2); --elev-3:var(--shadow-3);
  --tpad:10px 4px;
  /* Merged operator-console tokens */
  --rv-bg:#f6f8fc; --rv-panel:#ffffff; --rv-panel-2:#f8faff; --rv-surface:#eef2f8; --rv-surface-hover:#e4eaf3;
  --rv-ink:#0b1220; --rv-ink-2:#1a2540; --rv-muted:#5a6681; --rv-dim:#7b889e; --rv-line:#dce4ef; --rv-line-2:#c4d0e2;
  --rv-brand:#2563eb; --rv-brand-2:#1e3a8a; --rv-signal:#06b6d4; --rv-ok:#0f766e; --rv-warn:#c2410c; --rv-bad:#b91c1c;
}
[data-theme="dark"]{
  color-scheme:dark;
  --surface-1:#0f1728; --surface-2:#141e33; --surface-3:#1a2744;
  --ink-900:#eef2ff; --ink-800:#dbe2f4; --ink-700:#c0cadf;
  --ink-500:#8895ad; --ink-400:#6b7a94; --ink-300:#556276;
  --line:#1f2d4a; --line-2:#2a3a5e;
  --brand-500:#60a5fa; --brand-400:#93c5fd; --brand-100:#1e3a5f;
  --brand-050:#172554;
  --signal-500:#22d3ee; --signal-100:#164e63;
  --ok-500:#34d399; --ok-100:#064e3b;
  --ember-500:#fb923c; --ember-100:#7c2d12;
  --bad-500:#fca5a5; --bad-100:#7f1d1d;
  /* Legacy compat */
  --md-sys-color-primary:#60a5fa; --md-sys-color-on-primary:#172554;
  --md-sys-color-primary-container:#1e3a5f; --md-sys-color-on-primary-container:#dbeafe;
  --md-sys-color-secondary-container:#1a2744; --md-sys-color-on-secondary-container:#dbe2f4;
  --md-sys-color-tertiary:#34d399; --md-sys-color-tertiary-container:#064e3b; --md-sys-color-on-tertiary-container:#ccfbf1;
  --md-sys-color-error:#fca5a5; --md-sys-color-error-container:#7f1d1d; --md-sys-color-on-error-container:#fee2e2;
  --md-sys-color-warning-container:#7c2d12; --md-sys-color-on-warning-container:#ffedd5;
  --md-sys-color-success:#34d399; --md-sys-color-success-container:#064e3b; --md-sys-color-on-success-container:#ccfbf1;
  --md-sys-color-background:#0f1728; --md-sys-color-on-surface:#eef2ff;
  --md-sys-color-on-surface-variant:#8895ad;
  --md-sys-color-outline:#6b7a94; --md-sys-color-outline-variant:#1f2d4a;
  --md-sys-color-surface-container-lowest:#0a0f1c;
  --md-sys-color-surface-container-low:#0f1728;
  --md-sys-color-surface-container:#141e33;
  --md-sys-color-surface-container-high:#1a2744;
  --md-sys-color-surface-container-highest:#223050;
  --rv-bg:#0a0f1c; --rv-panel:#0f1728; --rv-panel-2:#131c31; --rv-surface:#1a2744; --rv-surface-hover:#1d2947;
  --rv-ink:#eef2ff; --rv-ink-2:#dbe2f4; --rv-muted:#8895ad; --rv-dim:#6b7a94; --rv-line:#1f2d4a; --rv-line-2:#2a3a5e;
  --rv-brand:#60a5fa; --rv-brand-2:#93c5fd; --rv-signal:#22d3ee; --rv-ok:#34d399; --rv-warn:#fb923c; --rv-bad:#fca5a5;
}
[data-density="compact"]{ --gap:12px; --pad:16px; --tpad:7px 4px; }
*{box-sizing:border-box; margin:0; padding:0}
html{scroll-behavior:smooth}
body{
  min-height:100vh;
  font-family:'Inter','Inter Tight',system-ui,-apple-system,sans-serif;
  font-optical-sizing:auto;
  background:
    linear-gradient(90deg,color-mix(in oklab,var(--line) 36%,transparent) 1px,transparent 1px) 0 0/32px 32px,
    linear-gradient(0deg,color-mix(in oklab,var(--line) 36%,transparent) 1px,transparent 1px) 0 0/32px 32px,
    radial-gradient(1100px 620px at 12% -12%,color-mix(in oklab,var(--brand-500) 12%,transparent),transparent 58%),
    radial-gradient(900px 500px at 100% 0%,color-mix(in oklab,var(--signal-500) 12%,transparent),transparent 56%),
    var(--surface-2);
  color:var(--ink-800);
  transition: background var(--dur-m) var(--ease), color var(--dur-m) var(--ease);
}
code,.mono{font-family:'JetBrains Mono',monospace}

/* ---- APP SHELL ---- */
.app{display:grid;grid-template-columns:240px minmax(0,1fr);min-height:100vh}
.rail{
  background:linear-gradient(180deg,var(--surface-2),var(--surface-1));
  border-right:1px solid var(--line);
  padding:18px 14px 14px;display:flex;flex-direction:column;gap:18px;
  position:sticky;top:0;height:100vh;overflow-y:auto;
  box-shadow:6px 0 24px rgba(16,24,40,.04);
}
.rail::-webkit-scrollbar{width:0}
.brand{
  display:flex;align-items:center;gap:12px;
  padding:8px 10px;
  border-radius:var(--r-md);
  background:transparent;
  color:var(--ink-900);margin:0;position:relative;overflow:hidden;
  box-shadow:none;
}
.brand .material-symbols-rounded{font-size:28px;position:relative;z-index:1}
.material-symbols-rounded{font-family:'Material Symbols Rounded';font-weight:normal;font-style:normal;display:inline-block;line-height:1;text-transform:none;letter-spacing:normal;word-wrap:normal;white-space:nowrap;direction:ltr;-webkit-font-feature-settings:'liga';-webkit-font-smoothing:antialiased;font-feature-settings:'liga'}
.brand:hover{background:var(--surface-3)}
.brand::after{display:none}
@keyframes spin{to{transform:rotate(360deg)}}
.brand-mark{
  position:relative;width:40px;height:40px;flex:none;z-index:1;
  border-radius:12px;
  background:
    radial-gradient(circle at 30% 30%,#fff,transparent 40%),
    linear-gradient(135deg,var(--brand-500),var(--brand-400) 70%,#0a1630);
  box-shadow:0 0 0 1px rgba(255,255,255,.25) inset,0 6px 14px -4px rgba(29,78,216,.45);
  overflow:hidden;display:grid;place-items:center;
}
.brand-ring{
  position:absolute;inset:6px;
  border:1.5px solid rgba(255,255,255,.55);
  border-radius:50%;
  animation:sweep 4s linear infinite;
}
.brand-ring.r2{inset:10px;opacity:.45;animation-duration:6s;animation-direction:reverse}
.brand-dot{
  width:6px;height:6px;border-radius:50%;
  background:#fff;
  box-shadow:0 0 10px #fff,0 0 0 3px rgba(255,255,255,.25);
  position:relative;z-index:2;
}
@keyframes sweep{
  0%{clip-path:polygon(50% 50%,100% 0,100% 50%);transform:rotate(0)}
  100%{clip-path:polygon(50% 50%,100% 0,100% 50%);transform:rotate(360deg)}
}
.brand-label{display:flex;flex-direction:column;line-height:1.1}
.bl-top{font-family:'Inter Tight','Inter',sans-serif;font-weight:900;font-size:14px;letter-spacing:.14em;color:var(--ink-900)}
.bl-bot{font-size:10.5px;letter-spacing:.14em;color:var(--ink-400);font-weight:700;margin-top:2px}
.nav-groups{display:flex;flex-direction:column;gap:6px}
.nav-group{display:flex;flex-direction:column;gap:2px;padding-top:10px}
.nav-group + .nav-group{border-top:1px dashed var(--line);margin-top:6px}
.nav-heading{
  font-size:10.5px;font-weight:800;letter-spacing:.14em;
  color:var(--ink-400);text-transform:uppercase;
  padding:8px 10px 6px;
}
.r-item{
  appearance:none;display:grid;grid-template-columns:22px minmax(0,1fr) auto;
  align-items:center;gap:12px;padding:9px 10px;
  border-radius:var(--r-md);background:transparent;border:1px solid transparent;
  color:var(--ink-700);font:600 14px/1.15 'Inter',sans-serif;
  letter-spacing:0;cursor:pointer;position:relative;text-align:left;
  transition:background var(--dur-s) var(--ease),color var(--dur-s) var(--ease),border-color var(--dur-s) var(--ease);
}
.r-item .r-ic{width:22px;height:22px;display:grid;place-items:center;border-radius:var(--r-sm);transition:background var(--dur-m) var(--ease-emph)}
.r-item .material-symbols-rounded{font-size:20px;color:var(--ink-500);transition:font-variation-settings var(--dur-s) var(--ease),color var(--dur-s) var(--ease)}
.r-item:hover{background:var(--surface-3);color:var(--ink-900)}
.r-item:hover .material-symbols-rounded{color:var(--ink-800)}
.r-item.active{
  background:color-mix(in oklab,var(--brand-100) 60%,var(--surface-1));
  color:var(--ink-900);border-color:color-mix(in oklab,var(--brand-500) 25%,transparent);
  font-weight:700;
}
.r-item.active::before{content:'';position:absolute;left:-14px;top:8px;bottom:8px;width:3px;border-radius:0 3px 3px 0;background:var(--brand-500)}
.r-item.active .r-ic{background:transparent}
.r-item.active .material-symbols-rounded{color:var(--brand-500);font-variation-settings:'FILL' 1,'wght' 600}
.r-item.sub{padding:7px 10px 7px 14px;font-size:13px;color:var(--ink-500)}
.r-item.sub .material-symbols-rounded{font-size:18px}
.r-item.sub.active{background:var(--surface-3);color:var(--ink-900)}
.r-item.sub.active::before{background:var(--signal-500)}
.nav-ind{
  width:8px;height:8px;border-radius:50%;
  background:var(--signal-500);
  box-shadow:0 0 0 0 color-mix(in oklab,var(--signal-500) 60%,transparent);
  animation:pulseDot 1.6s var(--ease) infinite;
  justify-self:end;margin-right:6px;
}
@keyframes pulseDot{
  0%{box-shadow:0 0 0 0 color-mix(in oklab,var(--signal-500) 60%,transparent)}
  70%{box-shadow:0 0 0 8px color-mix(in oklab,var(--signal-500) 0%,transparent)}
  100%{box-shadow:0 0 0 0 transparent}
}
body:not([data-view="live"]) #liveTabsGroup{display:none}
.nav-kbd{
  justify-self:end;
  font-family:'JetBrains Mono',monospace;font-size:10.5px;font-weight:700;
  color:var(--ink-400);
  background:var(--surface-3);border:1px solid var(--line);
  padding:2px 6px;border-radius:6px;
}
.r-item.active .nav-kbd{background:#fff}
.r-spacer{flex:1}
.rail-foot{margin-top:auto;display:flex;flex-direction:column;gap:10px;padding-top:12px;border-top:1px dashed var(--line)}
.operator{
  display:flex;align-items:center;gap:10px;
  padding:8px;
  background:var(--surface-3);border-radius:var(--r-md);
}
.op-avatar{
  width:32px;height:32px;border-radius:10px;
  display:grid;place-items:center;
  background:linear-gradient(135deg,var(--brand-500),var(--signal-500));
  color:#fff;font-weight:800;font-size:12px;
  letter-spacing:.05em;
}
.op-meta{display:flex;flex-direction:column;line-height:1.2;min-width:0}
.op-name{font-size:13px;font-weight:700;color:var(--ink-900);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:150px}
.op-role{font-size:11px;color:var(--ink-400);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:150px}

/* ---- MAIN ---- */
.main{padding:24px 30px 88px;min-width:0;max-width:1720px;width:100%}
.topbar{
  position:sticky;top:0;z-index:60;
  display:flex;align-items:center;gap:14px;padding:12px 4px 16px;flex-wrap:wrap;
  backdrop-filter:blur(16px);
  background:color-mix(in oklab,var(--surface-2) 88%,transparent);
  border-bottom:1px solid color-mix(in oklab,var(--line) 60%,transparent);
}
.eyebrow{font-size:11px;font-weight:800;letter-spacing:.14em;text-transform:uppercase;color:var(--brand-500);margin-bottom:4px}
.topbar h1{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:36px;font-weight:900;letter-spacing:-.02em;line-height:1;color:var(--ink-900)}
.sp{flex:1}
.conn{display:inline-flex;align-items:center;gap:8px;padding:8px 14px 8px 12px;border-radius:var(--r-md);background:var(--surface-1);border:1px solid var(--line);font-size:13px;font-weight:700;color:var(--ink-500)}
.dot{width:10px;height:10px;border-radius:50%;background:var(--ok-500);box-shadow:0 0 0 0 color-mix(in oklab,var(--ok-500) 50%,transparent);animation:pulse 1.8s var(--ease) infinite}
.conn.warn .dot{background:var(--ember-500);animation:none}
.conn.bad .dot{background:var(--bad-500);animation:none}
@keyframes pulse{0%{box-shadow:0 0 0 0 color-mix(in oklab,var(--ok-500) 70%,transparent)}70%{box-shadow:0 0 0 10px transparent}100%{box-shadow:0 0 0 0 transparent}}
.ic-btn{width:42px;height:42px;border-radius:var(--r-md);background:var(--surface-1);border:1px solid var(--line);display:grid;place-items:center;color:var(--ink-700);cursor:pointer;position:relative;transition:background var(--dur-s) var(--ease),transform var(--dur-s) var(--ease-emph)}
.ic-btn:hover{background:var(--surface-3);border-color:var(--line-2)}
.ic-btn:active{transform:scale(.93)}
.ic-btn .material-symbols-rounded{font-size:20px}
.ic-badge{position:absolute;top:-4px;right:-4px;min-width:18px;height:18px;padding:0 5px;border-radius:var(--r-pill);background:var(--bad-500);color:#fff;font-size:11px;font-weight:900;display:grid;place-items:center;border:2px solid var(--surface-2)}
.fab{display:inline-flex;align-items:center;gap:8px;padding:12px 20px 12px 16px;border-radius:var(--r-md);background:linear-gradient(135deg,var(--brand-500),color-mix(in oklab,var(--brand-500) 72%,var(--signal-500)));color:#fff;font:700 14px/1 inherit;border:none;cursor:pointer;box-shadow:0 10px 24px -12px var(--brand-500);transition:box-shadow var(--dur-s) var(--ease),transform var(--dur-s) var(--ease-emph)}
.fab:hover{box-shadow:0 14px 28px -12px var(--brand-500);transform:translateY(-1px)}
.fab:active{transform:scale(.97)}
.fab .material-symbols-rounded{font-size:20px}
.seg{display:inline-flex;border:1px solid var(--line);border-radius:var(--r-md);overflow:hidden}
.seg button{appearance:none;background:transparent;border:none;padding:8px 14px;font:600 13px/1 inherit;color:var(--ink-500);cursor:pointer;display:inline-flex;align-items:center;gap:6px;transition:background var(--dur-s) var(--ease),color var(--dur-s) var(--ease)}
.seg button+button{border-left:1px solid var(--line)}
.seg button:hover{background:var(--surface-3)}
.seg button.active{background:var(--brand-100);color:var(--brand-500)}
.seg button .material-symbols-rounded{font-size:18px}

/* ---- CHIPS / PILLS ---- */
.chips{display:flex;flex-wrap:wrap;gap:8px;margin-top:4px}
.chip{display:inline-flex;align-items:center;gap:6px;padding:6px 12px 6px 8px;border-radius:var(--r-md);font-size:12px;font-weight:700;background:var(--surface-3);color:var(--ink-500);border:1px solid transparent;transition:all var(--dur-s) var(--ease)}
.chip .material-symbols-rounded{font-size:16px}
.chip.good{background:var(--ok-100);color:var(--ok-500)}
.chip.warn{background:var(--ember-100);color:var(--ember-500)}
.chip.bad{background:var(--bad-100);color:var(--bad-500)}
.pills{display:flex;flex-wrap:wrap;gap:6px}
.pill{display:inline-flex;align-items:center;gap:5px;padding:4px 10px;border-radius:var(--r-pill);background:var(--surface-3);color:var(--ink-500);font-size:11.5px;font-weight:700;border:1px solid var(--line)}
.pill.good{background:var(--ok-100);color:var(--ok-500);border-color:transparent}
.pill.warn{background:var(--ember-100);color:var(--ember-500);border-color:transparent}
.pill.bad{background:var(--bad-100);color:var(--bad-500);border-color:transparent}
.pill .material-symbols-rounded{font-size:14px}

/* ---- FIRMWARE BADGE ---- */
.fw{display:grid;grid-template-columns:auto minmax(0,1fr) auto;align-items:center;gap:14px;padding:14px 18px;border-radius:var(--r-lg);background:var(--surface-3);margin-bottom:var(--gap);border:1px solid var(--line);box-shadow:var(--shadow-1)}
.fw.good{background:color-mix(in oklab,var(--ok-100) 50%,var(--surface-1));color:var(--ok-500);border-color:color-mix(in oklab,var(--ok-500) 18%,transparent)}
.fw.warn{background:color-mix(in oklab,var(--ember-100) 50%,var(--surface-1));color:var(--ember-500);border-color:color-mix(in oklab,var(--ember-500) 20%,transparent)}
.fw.bad{background:color-mix(in oklab,var(--bad-100) 50%,var(--surface-1));color:var(--bad-500);border-color:color-mix(in oklab,var(--bad-500) 18%,transparent)}
.fw .material-symbols-rounded{font-size:24px}
.fw-title{font-size:11px;font-weight:900;letter-spacing:.1em;text-transform:uppercase;opacity:.85}
.fw-line{display:flex;flex-wrap:wrap;gap:10px;margin-top:4px;font-family:'JetBrains Mono',monospace;font-size:12.5px;font-weight:600}
.fw-line span{padding:2px 8px;background:rgba(0,0,0,.06);border-radius:var(--r-sm)}
[data-theme="dark"] .fw-line span{background:rgba(255,255,255,.08)}

/* ---- TABS / LAYOUT ---- */
.tab{display:none;animation:fadeUp var(--dur-m) var(--ease-emph) forwards}
.tab.active{display:block}
@keyframes fadeUp{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:none}}
.grid{display:grid;gap:var(--gap)}
.c2{grid-template-columns:repeat(2,minmax(0,1fr))}
.c3{grid-template-columns:repeat(3,minmax(0,1fr))}
.c4{grid-template-columns:repeat(4,minmax(0,1fr))}

/* ---- CARD ---- */
.card{background:color-mix(in oklab,var(--surface-1) 96%,transparent);border-radius:var(--r-xl);padding:var(--pad);min-width:0;border:1px solid var(--line);position:relative;box-shadow:var(--shadow-2);transition:border-color var(--dur-s) var(--ease)}
.card:hover{border-color:color-mix(in oklab,var(--brand-500) 25%,var(--line))}
.ch{display:flex;align-items:flex-start;justify-content:space-between;gap:12px;margin-bottom:18px}
.tw{display:flex;align-items:center;gap:12px;min-width:0}
.tw>.material-symbols-rounded{color:#fff;background:linear-gradient(135deg,var(--brand-500),var(--brand-400));padding:9px;border-radius:var(--r-md);font-size:20px;flex:0 0 auto}
.tw.rr>.material-symbols-rounded{background:linear-gradient(135deg,var(--ok-500),#34d399)}
.ct{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:17px;font-weight:800;letter-spacing:-.01em;color:var(--ink-900)}
.cs{margin-top:2px;font-size:12.5px;color:var(--ink-500);font-weight:500}
.ca{display:flex;gap:4px;align-items:center;flex-wrap:wrap}
.ca-btn{appearance:none;border:none;background:transparent;width:34px;height:34px;border-radius:var(--r-md);color:var(--ink-500);display:grid;place-items:center;cursor:pointer;transition:background var(--dur-s) var(--ease)}
.ca-btn:hover{background:var(--surface-3)}
.ca-btn .material-symbols-rounded{font-size:18px}

/* ---- KPI ---- */
.kpi-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:var(--gap)}
.kpi{background:var(--surface-1);border:1px solid var(--line);border-radius:var(--r-xl);padding:20px;display:grid;gap:10px;position:relative;overflow:hidden;min-width:0;box-shadow:var(--shadow-2);transition:border-color var(--dur-s) var(--ease),transform var(--dur-s) var(--ease)}
.kpi:hover{border-color:var(--line-2);transform:translateY(-1px)}
.kpi::before{content:'';position:absolute;inset:auto -30% -60% auto;width:200px;height:200px;background:radial-gradient(circle,color-mix(in oklab,var(--brand-500) 14%,transparent),transparent 60%);pointer-events:none}
.kpi.hr{border-top:3px solid var(--brand-500)}
.kpi.hr::before{background:radial-gradient(circle,color-mix(in oklab,var(--brand-500) 22%,transparent),transparent 60%)}
.kpi.rr{border-top:3px solid var(--ok-500)}
.kpi.rr::before{background:radial-gradient(circle,color-mix(in oklab,var(--ok-500) 22%,transparent),transparent 60%)}
.k-lab{display:flex;align-items:center;justify-content:space-between;gap:8px}
.k-lab-t{font-size:11px;font-weight:800;letter-spacing:.12em;text-transform:uppercase;color:var(--ink-400)}
.k-ic{width:32px;height:32px;border-radius:var(--r-md);display:grid;place-items:center;background:var(--surface-3)}
.k-ic .material-symbols-rounded{font-size:18px;color:var(--ink-500)}
.k-val-r{display:flex;align-items:baseline;gap:6px}
.k-val{font-family:'Inter Tight','Inter',sans-serif;font-size:48px;font-weight:900;letter-spacing:-.035em;line-height:1;color:var(--ink-900)}
.k-u{font-size:14px;font-weight:700;color:var(--ink-400)}
.k-spark{height:36px;width:100%}
.k-sub{font-size:12px;font-weight:600;color:var(--ink-400)}

/* ---- METRICS / TABLES ---- */
.mr{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin-bottom:16px}
.mr.three{grid-template-columns:repeat(3,minmax(0,1fr))}
.m{display:flex;flex-direction:column;gap:4px;background:var(--surface-3);padding:14px;border-radius:var(--r-md)}
.m-l{font-size:11px;font-weight:800;color:var(--ink-400);text-transform:uppercase;letter-spacing:.1em}
.m-vr{display:flex;align-items:baseline;gap:4px}
.m-v{font-family:'JetBrains Mono',monospace;font-size:28px;font-weight:900;color:var(--brand-500);letter-spacing:-.02em;line-height:1}
.m.rr .m-v{color:var(--ok-500)}
.m-u{font-size:13px;color:var(--ink-400);font-weight:700}
table{width:100%;border-collapse:collapse}
td{padding:var(--tpad);border-bottom:1px solid var(--line);font-size:12.5px;vertical-align:middle}
tr:last-child td{border-bottom:none}
td:first-child{color:var(--ink-500);font-weight:500}
td:last-child{color:var(--ink-900);text-align:right;font-weight:600;font-family:'JetBrains Mono',monospace;cursor:copy;position:relative}
td:last-child:hover{color:var(--brand-500)}
td.copied::after{content:'copied';position:absolute;right:6px;top:50%;transform:translateY(-50%);background:var(--ink-900);color:#fff;padding:2px 6px;border-radius:var(--r-sm);font-size:10px;font-family:inherit;animation:tOut 1.4s forwards}
@keyframes tOut{0%,70%{opacity:1}100%{opacity:0}}

/* ---- FAULTS ---- */
.faults{display:grid;gap:10px}
.flt{border-radius:var(--r-lg);padding:14px 16px;display:grid;gap:6px;position:relative;overflow:hidden;border-left:4px solid var(--line-2)}
.flt.good{background:color-mix(in oklab,var(--ok-100) 45%,var(--surface-1));color:var(--ok-500);border-left-color:var(--ok-500)}
.flt.warn{background:color-mix(in oklab,var(--ember-100) 45%,var(--surface-1));color:var(--ember-500);border-left-color:var(--ember-500)}
.flt.bad{background:color-mix(in oklab,var(--bad-100) 45%,var(--surface-1));color:var(--bad-500);border-left-color:var(--bad-500)}
.flt-t{display:flex;align-items:center;gap:8px;font-weight:800;font-size:13.5px}
.flt-t .material-symbols-rounded{font-size:18px}
.flt-c{font-size:12.5px;opacity:.9;line-height:1.45;font-weight:500}

/* ---- CHARTS ---- */
.cp{background:var(--surface-3);border-radius:var(--r-lg);padding:14px;min-height:260px}
.cp.s{min-height:220px}
canvas{width:100%!important;height:230px!important}
#rawHrBucketChart{height:200px!important}
pre{background:var(--surface-3);color:var(--ink-800);border:1px solid var(--line);border-radius:var(--r-lg);padding:14px;white-space:pre-wrap;font-family:'JetBrains Mono',monospace;font-size:12px;line-height:1.55;margin:0;overflow-x:auto;max-height:360px;overflow-y:auto}

/* ---- RADAR ---- */
.rad{display:grid;grid-template-columns:minmax(0,1.3fr) minmax(320px,.9fr);gap:var(--gap)}
.rp{background:var(--surface-3);border-radius:var(--r-lg);padding:18px;display:grid;gap:14px}
.rp-h{display:flex;align-items:flex-start;justify-content:space-between;gap:12px}
.rp-n{font-size:13px;line-height:1.5;color:var(--ink-500);max-width:60ch}
.tgt{flex:0 0 auto;padding:8px 14px;border-radius:var(--r-pill);background:var(--brand-100);color:var(--brand-500);font-weight:800;font-size:13px}
.rad-w{position:relative;background:radial-gradient(120% 120% at 50% 120%,#0b1220,#111a2c 50%,#0a0f1c 80%);border:1px solid var(--line);border-radius:var(--r-lg);padding:12px;min-height:340px;overflow:hidden}
.rad-w::before{content:'';position:absolute;inset:0;background:radial-gradient(500px 250px at 50% 100%,color-mix(in oklab,var(--signal-500) 10%,transparent),transparent 60%);animation:sheen 5s ease-in-out infinite alternate;pointer-events:none}
@keyframes sheen{from{opacity:.5}to{opacity:1}}
#radarCanvas{height:320px!important}
.side{display:grid;gap:var(--gap);min-width:0}
.tb{background:var(--surface-3);border-radius:var(--r-lg);padding:18px}
.tbig{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;margin-bottom:18px}
.tl{background:var(--surface-1);border-radius:var(--r-md);padding:14px;display:grid;gap:4px;border:1px solid var(--line)}
.tl .l{font-size:11px;font-weight:800;text-transform:uppercase;letter-spacing:.08em;color:var(--ink-400)}
.tl .v{font-family:'JetBrains Mono',monospace;font-size:26px;line-height:1;font-weight:900;color:var(--brand-500);letter-spacing:-.02em}
.tl .v.rr{color:var(--ok-500)}
.kv{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:8px 16px;align-items:center}
.kv>div:nth-child(odd){color:var(--ink-500);font-size:12.5px;font-weight:500}
.kv>div:nth-child(even){text-align:right;font-family:'JetBrains Mono',monospace;font-size:12.5px;font-weight:600;color:var(--ink-900);word-break:break-word}

/* ---- WAVES ---- */
.wg{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:var(--gap)}
.wc{background:var(--surface-3);border-radius:var(--r-lg);padding:18px}
.wt{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:15px;font-weight:800}
.ws{margin:4px 0 12px;font-size:12px;color:var(--ink-500);font-weight:500}
#breathChart,#heartChart{height:180px!important}
.ft{display:flex;align-items:center;gap:8px;color:var(--ink-400);font-size:12px;font-family:'JetBrains Mono',monospace;padding:16px 4px 0;font-weight:500}
.ft .material-symbols-rounded{font-size:16px}

/* ---- DRAWERS / OVERLAYS ---- */
.dv-ov{position:fixed;inset:0;background:rgba(10,15,28,.4);backdrop-filter:blur(4px);opacity:0;pointer-events:none;transition:opacity var(--dur-m) var(--ease);z-index:90}
.dv-ov.open{opacity:1;pointer-events:auto}
.dv{position:fixed;right:0;top:0;bottom:0;width:min(420px,100vw);background:var(--surface-1);border-left:1px solid var(--line);transform:translateX(100%);transition:transform var(--dur-l) var(--ease-emph);z-index:100;display:flex;flex-direction:column;box-shadow:var(--shadow-3)}
.dv.open{transform:translateX(0)}
.dv-h{padding:20px 24px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid var(--line)}
.dv-h h3{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:18px;font-weight:800}
.dv-b{flex:1;overflow-y:auto;padding:16px 20px;display:flex;flex-direction:column;gap:10px}
.dv-e{margin:40px auto;text-align:center;color:var(--ink-500);font-size:14px;font-weight:600}
.dv-e .material-symbols-rounded{display:block;font-size:48px;margin-bottom:8px;color:var(--ok-500);font-variation-settings:'FILL' 1}

/* ---- SNAPSHOTS ---- */
.snaps{display:grid;gap:10px}
.snap{display:grid;grid-template-columns:auto 1fr auto;gap:12px;align-items:center;padding:12px 14px;background:var(--surface-3);border-radius:var(--r-md)}
.snap-t{font-family:'JetBrains Mono',monospace;font-size:12px;color:var(--ink-500);font-weight:600}
.snap-v{display:flex;gap:14px;font-family:'JetBrains Mono',monospace;font-size:13px;font-weight:700;flex-wrap:wrap}
.snap-v .hr{color:var(--brand-500)}
.snap-v .rr{color:var(--ok-500)}
.snap-v .n{color:var(--ink-400)}
.snap-e{text-align:center;padding:20px;color:var(--ink-500);font-size:13px}

/* ---- PALETTE ---- */
.p-ov{position:fixed;inset:0;background:rgba(10,15,28,.52);backdrop-filter:blur(6px);display:none;z-index:200;align-items:flex-start;justify-content:center;padding-top:13vh}
.p-ov.open{display:flex}
.p{width:min(600px,94vw);background:var(--surface-1);border:1px solid var(--line);border-radius:var(--r-xl);box-shadow:var(--shadow-3);overflow:hidden;animation:pIn var(--dur-m) var(--ease-emph)}
@keyframes pIn{from{opacity:0;transform:translateY(-10px) scale(.98)}to{opacity:1;transform:none}}
.p-iw{display:flex;align-items:center;gap:10px;padding:16px 20px;border-bottom:1px solid var(--line)}
.p-iw .material-symbols-rounded{color:var(--ink-400);font-size:22px}
.p-in{flex:1;background:transparent;border:none;outline:none;font:inherit;font-size:16px;color:var(--ink-900)}
.p-in::placeholder{color:var(--ink-400)}
.p-l{max-height:50vh;overflow-y:auto;padding:8px}
.p-i{display:flex;align-items:center;gap:12px;padding:11px 14px;border-radius:var(--r-md);cursor:pointer;color:var(--ink-700);font-size:14px;font-weight:600}
.p-i .material-symbols-rounded{color:var(--ink-400);font-size:20px}
.p-i .cmd-sub{margin-left:auto;color:var(--ink-400);font-size:12px;font-weight:500}
.p-i.sel,.p-i:hover{background:var(--brand-100);color:var(--ink-900)}
.p-i.sel .material-symbols-rounded{color:var(--brand-500)}
.p-hint{padding:10px 18px;border-top:1px solid var(--line);font-size:11px;font-family:'JetBrains Mono',monospace;color:var(--ink-400);display:flex;gap:14px}
kbd{background:var(--surface-3);padding:2px 7px;border-radius:var(--r-sm);border:1px solid var(--line);font-family:'JetBrains Mono',monospace;font-size:11px;font-weight:700;color:var(--ink-500)}

/* ---- TOAST ---- */
.toast-host{position:fixed;left:50%;transform:translateX(-50%);bottom:24px;display:flex;flex-direction:column;gap:8px;z-index:300;pointer-events:none}
.toast{padding:12px 18px;background:var(--ink-900);color:#fff;border-radius:var(--r-pill);box-shadow:var(--shadow-3);font-size:13px;font-weight:600;display:flex;align-items:center;gap:8px;animation:tPop var(--dur-m) var(--ease-emph),tFade .4s var(--ease) forwards;animation-delay:0s,3s}
.toast .material-symbols-rounded{font-size:18px;color:var(--signal-500)}
@keyframes tPop{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:none}}
@keyframes tFade{to{opacity:0;transform:translateY(10px)}}

/* ---- FULLSCREEN ---- */
body.fs-open .card{overflow:visible}
.card.fs{position:fixed;inset:40px;z-index:150;background:var(--surface-1);overflow:auto}
.card.fs .cp{min-height:calc(100vh - 200px)}
.card.fs canvas{height:calc(100vh - 240px)!important}

/* ---- SETTINGS MODAL ---- */
.set-ov{position:fixed;inset:0;background:rgba(10,15,28,.46);backdrop-filter:blur(4px);display:none;z-index:220;align-items:center;justify-content:center;padding:20px}
.set-ov.open{display:flex}
.set-md{width:min(600px,96vw);max-height:min(82vh,760px);overflow:auto;background:var(--surface-1);border:1px solid var(--line);border-radius:var(--r-xl);box-shadow:var(--shadow-3);padding:22px}
.set-h{display:flex;align-items:flex-start;justify-content:space-between;gap:12px;margin-bottom:16px}
.set-tt{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:20px;font-weight:900}
.set-st{margin-top:4px;color:var(--ink-500);font-size:13px;font-weight:500}
.set-g{background:var(--surface-3);border:1px solid var(--line);border-radius:var(--r-lg);padding:18px;display:grid;gap:12px}
.set-g + .set-g{margin-top:14px}
.set-gl{margin:0;font-size:11px;font-weight:900;letter-spacing:.12em;text-transform:uppercase;color:var(--ink-400)}
.set-r{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:16px;align-items:center;padding:10px 0;border-bottom:1px solid var(--line)}
.set-r:last-child{border-bottom:none;padding-bottom:0}
.set-copy{display:grid;gap:4px}
.set-copy strong{font-size:14px;font-weight:800;color:var(--ink-900)}
.set-copy span{font-size:12.5px;line-height:1.45;color:var(--ink-500);font-weight:500}
.sw{appearance:none;width:54px;height:30px;border:none;border-radius:var(--r-pill);background:var(--line-2);cursor:pointer;position:relative;transition:background var(--dur-m) var(--ease)}
.sw::after{content:'';position:absolute;left:3px;top:3px;width:24px;height:24px;border-radius:50%;background:#fff;box-shadow:var(--shadow-1);transition:transform var(--dur-m) var(--ease-emph)}
.sw.on{background:var(--brand-500)}
.sw.on::after{transform:translateX(24px)}
.set-actions{display:flex;justify-content:flex-end;gap:10px;margin-top:16px}
.txt-btn{appearance:none;border:none;background:var(--brand-100);color:var(--brand-500);padding:10px 16px;border-radius:var(--r-md);font:700 14px/1 inherit;cursor:pointer}
.mode-badge{display:inline-flex;align-items:center;gap:6px;padding:6px 10px;border-radius:var(--r-md);background:var(--surface-3);border:1px solid var(--line);font-size:12px;font-weight:800;color:var(--ink-500)}
.mode-badge.demo{background:var(--ember-100);color:var(--ember-500);border-color:transparent}
.mode-badge.stale{background:var(--bad-100);color:var(--bad-500);border-color:transparent}
.mode-badge.live{background:var(--ok-100);color:var(--ok-500);border-color:transparent}

/* ---- CONTROL MODE / SESSION ---- */
body:not([data-ctl="on"]) .nav-ctl,
body:not([data-ctl="on"]) #ses-bar,
body:not([data-ctl="on"]) #liveCoach,
body:not([data-ctl="on"]) #view-home,
body:not([data-ctl="on"]) #view-report,
body:not([data-ctl="on"]) #view-help{ display:none !important; }
#view-home,#view-report,#view-help{ display:none; }
body[data-ctl="on"][data-view="home"] #view-home,
body[data-ctl="on"][data-view="report"] #view-report,
body[data-ctl="on"][data-view="help"] #view-help{ display:block; }
body[data-ctl="on"]:not([data-view="live"]) .tab,
body[data-ctl="on"]:not([data-view="live"]) #fwBadge,
body[data-ctl="on"]:not([data-view="live"]) #liveCoach,
body[data-ctl="on"]:not([data-view="live"]) #chips,
body[data-ctl="on"]:not([data-view="live"]) .r-item:not(.nav-ctl):not(.rail-action){ display:none !important; }
.nav-ctl{font-weight:800}
.nav-ctl.active{background:color-mix(in oklab,var(--brand-500) 10%,var(--surface-1));color:var(--ink-900);border:1px solid color-mix(in oklab,var(--brand-500) 18%,transparent)}
.ses-bar{display:flex;align-items:center;gap:12px;padding:12px 16px;margin-bottom:14px;border-radius:var(--r-lg);background:var(--surface-1);border:1px solid color-mix(in oklab,var(--brand-500) 28%,var(--line));font-size:13px;box-shadow:var(--shadow-2)}
.ses-bar .ses-id{font-weight:900;color:var(--ink-900)}
.ses-bar .ses-meta{color:var(--ink-500);display:flex;gap:14px;flex-wrap:wrap}
.ses-bar .ses-spacer{flex:1}
.ses-bar button{appearance:none;border:none;padding:8px 14px;border-radius:var(--r-md);font:700 13px/1 inherit;cursor:pointer;background:var(--bad-100);color:var(--bad-500)}
.ses-bar button:disabled{opacity:0.5;cursor:not-allowed}
.ses-bar .ses-progress{flex:1 1 180px;min-width:140px;max-width:320px;height:6px;border-radius:var(--r-pill);background:var(--surface-3);overflow:hidden;position:relative}
.ses-bar .ses-progress .ses-progress-fill{height:100%;width:0%;background:linear-gradient(90deg,var(--brand-500),var(--signal-500));border-radius:inherit;transition:width var(--dur-m) var(--ease)}
.ses-bar .ses-progress[data-manual="1"] .ses-progress-fill{background:var(--line-2)}
.ses-bar .ses-progress[data-almost="1"] .ses-progress-fill{background:var(--ember-500)}
.ses-bar .ses-copy-btn{appearance:none;border:none;background:transparent;color:var(--ink-400);padding:2px 4px;margin-left:4px;border-radius:var(--r-md);cursor:pointer;display:inline-flex;align-items:center}
.ses-bar .ses-copy-btn:hover{background:var(--surface-3);color:var(--ink-700)}
.ses-bar .ses-copy-btn .material-symbols-rounded{font-size:16px}
.ses-bar button.confirm{background:var(--bad-500);color:#fff;animation:sesStopPulse 1.2s var(--ease) infinite}
@keyframes sesStopPulse{0%,100%{box-shadow:0 0 0 0 rgba(185,28,28,.45)}50%{box-shadow:0 0 0 6px rgba(185,28,28,0)}}

/* ---- SETUP / PREFLIGHT ---- */
.page-container{display:flex;flex-direction:column;gap:var(--gap)}
.home-layout{display:grid;grid-template-columns:minmax(0,5fr) minmax(360px,3fr);gap:var(--gap);align-items:start}
.welcome-hero{position:relative;overflow:hidden;margin-bottom:var(--gap);padding:36px 32px;min-height:220px;border-radius:var(--r-xl);background:radial-gradient(420px 260px at 100% 0%,color-mix(in oklab,var(--signal-500) 16%,transparent),transparent 62%),linear-gradient(145deg,var(--surface-1),var(--surface-2));border:1px solid var(--line);box-shadow:var(--shadow-3)}
.welcome-hero::before{content:'';position:absolute;inset:0;background:linear-gradient(90deg,color-mix(in oklab,var(--line) 28%,transparent) 1px,transparent 1px) 0 0/28px 28px,linear-gradient(0deg,color-mix(in oklab,var(--line) 28%,transparent) 1px,transparent 1px) 0 0/28px 28px;mask-image:linear-gradient(90deg,transparent,black 22%,black 76%,transparent);opacity:.5;pointer-events:none}
.welcome-hero>*{position:relative}
.hero-title{margin:0 0 10px;font-family:'Inter Tight','Inter',sans-serif;font-size:48px;line-height:1.04;font-weight:900;color:var(--ink-900);letter-spacing:-.02em}
.hero-subtitle{font-size:15px;color:var(--ink-500);max-width:62ch;line-height:1.55;font-weight:500}
.card-header{display:flex;align-items:center;gap:12px;margin-bottom:22px}
.card-header .material-symbols-rounded{font-size:23px;color:#fff;background:linear-gradient(135deg,var(--brand-500),var(--brand-400));padding:10px;border-radius:var(--r-md)}
.card-header h3{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:18px;font-weight:800}
.card-header.space-between{justify-content:space-between}
.title-with-icon{display:flex;align-items:center;gap:12px}
.sys-card{border-radius:var(--r-xl);background:color-mix(in oklab,var(--surface-1) 96%,transparent);border:1px solid var(--line);padding:var(--pad);box-shadow:var(--shadow-2)}
.sys-card:hover{border-color:color-mix(in oklab,var(--brand-500) 22%,var(--line))}
.setup-form{display:grid;gap:26px}
.setup-group{display:flex;flex-direction:column;gap:14px}
.group-label{font-size:11px;font-weight:900;letter-spacing:.13em;text-transform:uppercase;color:var(--ink-400);border-bottom:1px solid var(--line);padding-bottom:8px}
.hardware-row,.meta-row{display:grid;grid-template-columns:160px minmax(0,1fr);align-items:center;gap:14px}
.hardware-row label,.meta-row label{font-weight:700;color:var(--ink-800)}
.setup-form input[type=text],.setup-form input[type=number],.setup-form select,.sess-filter input,.setup input[type=text],.setup input[type=number],.setup select{
  width:100%;padding:11px 15px;border:1px solid var(--line);border-radius:var(--r-md);
  background:var(--surface-2);color:var(--ink-900);font:600 14px/1.35 'Inter',sans-serif;
  transition:border-color var(--dur-s) var(--ease),box-shadow var(--dur-s) var(--ease);
}
.setup-form input:focus,.setup-form select:focus,.setup input:focus,.setup select:focus{outline:none;border-color:var(--brand-500);box-shadow:0 0 0 3px color-mix(in oklab,var(--brand-500) 16%,transparent)}
.input-with-action{display:flex;gap:10px;width:100%}
.input-with-action input{flex:1}
.action-btn,.ghost-btn{appearance:none;border:1px solid var(--line);border-radius:var(--r-md);background:var(--surface-3);color:var(--ink-700);padding:0 16px;min-height:42px;font:700 13px/1 'Inter',sans-serif;cursor:pointer;transition:background var(--dur-s) var(--ease)}
.ghost-btn{min-height:auto;padding:8px 12px;color:var(--brand-500);background:transparent}
.action-btn:hover,.ghost-btn:hover{background:var(--surface-3);border-color:var(--line-2)}
.inline-msg{grid-column:2;font-size:12px;color:var(--ink-400);font-family:'JetBrains Mono',monospace}
.advanced-section{border:1px solid var(--line);border-radius:var(--r-md);overflow:hidden}
.advanced-section summary{padding:14px 18px;background:var(--surface-3);cursor:pointer;font-weight:800;font-size:13px;color:var(--ink-500)}
.adv-grid{padding:18px;background:var(--surface-2);display:grid;gap:14px}
.pf-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:10px}
.pf-tile{padding:12px;border-radius:var(--r-md);background:var(--surface-2);border:1px solid var(--line);border-left:4px solid var(--line-2)}
.pf-tile.ok,.pf-tile.good{border-left-color:var(--ok-500)}
.pf-tile.warn{border-left-color:var(--ember-500);background:color-mix(in oklab,var(--ember-100) 30%,var(--surface-1))}
.pf-tile.fail{border-left-color:var(--bad-500);background:color-mix(in oklab,var(--bad-100) 30%,var(--surface-1))}
.pf-tile .pf-top{display:flex;align-items:center;gap:8px;font-weight:800;flex-wrap:wrap}
.pf-tile .pf-detail{margin-top:6px;font-size:12px;color:var(--ink-500);word-break:break-word}
.pf-tile .pf-rem{margin-top:6px;font-size:12px;font-weight:650;background:color-mix(in oklab,var(--ember-500) 8%,transparent);padding:8px;border-radius:var(--r-sm)}
.pf-tile button{margin-top:8px;appearance:none;border:1px solid var(--line);background:transparent;color:inherit;padding:5px 10px;border-radius:var(--r-pill);font:700 12px/1 inherit;cursor:pointer}
.pf-tag{display:inline-flex;align-items:center;gap:4px;padding:2px 8px;border-radius:var(--r-pill);font-size:10px;font-weight:900;letter-spacing:.06em;text-transform:uppercase;background:var(--surface-3);color:var(--ink-400);border:1px solid var(--line)}
.pf-tag.req{background:var(--brand-100);color:var(--brand-500);border-color:transparent}
.setup{display:flex;flex-direction:column;gap:14px}
.chip-group{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.chip-btn{appearance:none;border:1px solid var(--line);background:var(--surface-1);padding:8px 14px;border-radius:var(--r-md);font:700 13px/1 inherit;cursor:pointer;color:var(--ink-700);display:inline-flex;gap:6px;align-items:center;transition:all var(--dur-s) var(--ease)}
.chip-btn:hover{background:var(--surface-3);border-color:var(--line-2)}
.chip-btn.active{background:var(--brand-500);color:#fff;border-color:var(--brand-500);box-shadow:0 4px 10px -3px color-mix(in oklab,var(--brand-500) 55%,transparent)}
.chip-btn .material-symbols-rounded{font-size:15px}
.launch-panel{border-radius:var(--r-xl);background:var(--surface-1);border:1px solid var(--line);padding:22px;box-shadow:var(--shadow-3)}
.launch-btn,.setup-start{width:100%;border:none;border-radius:var(--r-md);padding:16px 22px;font:900 16px/1 'Inter',sans-serif;cursor:pointer;background:linear-gradient(135deg,var(--brand-500),color-mix(in oklab,var(--brand-500) 72%,var(--signal-500)));color:#fff;box-shadow:0 10px 24px -12px var(--brand-500);transition:transform var(--dur-s) var(--ease)}
.launch-btn:hover:not(:disabled),.setup-start:hover:not(:disabled){transform:translateY(-1px)}
.launch-btn:disabled,.setup-start:disabled{opacity:.4;cursor:not-allowed;background:var(--line-2);box-shadow:none}
.setup-start.in-progress{background:var(--surface-3);color:var(--ink-700);opacity:1;cursor:pointer;box-shadow:none}
.setup-alert{padding:10px 14px;border-radius:var(--r-md);background:var(--bad-100);color:var(--bad-500);font-size:13px;margin-top:10px}
.sandbox-note{margin-top:14px;padding:12px 14px;border:1px solid var(--line-2);border-left:4px solid var(--ember-500);border-radius:var(--r-md);background:color-mix(in oklab,var(--ember-100) 30%,var(--surface-1));color:var(--ink-700);font-size:13px;line-height:1.45}
.sandbox-note strong{display:block;font-size:11px;text-transform:uppercase;letter-spacing:.08em;color:var(--ember-500);margin-bottom:3px}
.sandbox-note.connected{border-left-color:var(--ok-500);background:color-mix(in oklab,var(--ok-100) 28%,var(--surface-1))}
.sandbox-note.connected strong{color:var(--ok-500)}
.setup-help{font-size:12px;font-weight:500;color:var(--ink-400);line-height:1.45;margin:-4px 0 2px}
.setup-help.ok{color:var(--ok-500)}
.setup-help.warn{color:var(--ember-500)}
.setup input.invalid-soft{border-color:var(--ember-500);box-shadow:inset 0 0 0 1px var(--ember-500)}

/* ---- SESSIONS TABLE ---- */
.sess-filter{display:flex;gap:10px;margin-bottom:12px;align-items:center;flex-wrap:wrap}
.sess-filter .count{color:var(--ink-400);font-size:12px;font-weight:700}
.sess-table{border-collapse:separate;border-spacing:0;width:100%;font-size:13px}
.sess-table th{font-size:11px;letter-spacing:.1em;text-transform:uppercase;color:var(--ink-400);padding:10px 12px;border-bottom:1px solid var(--line)}
.sess-table td{padding:11px 12px;border-bottom:1px solid var(--line);text-align:left}
.sess-table tr{cursor:pointer}
.sess-table tr:hover td{background:var(--surface-3)}
.sess-table tr td:last-child::after{content:"open ›";margin-left:8px;font-size:11px;font-weight:700;opacity:0;transition:opacity var(--dur-s) var(--ease);color:var(--brand-500);font-family:inherit}
.sess-table tr:hover td:last-child::after,.sess-table tr:focus-within td:last-child::after{opacity:1}
.sess-empty{display:grid;gap:8px;padding:16px 18px;border-radius:var(--r-md);background:var(--surface-3);border:1px dashed var(--line);color:var(--ink-500);font-size:13px;font-weight:500}
.sess-empty strong{color:var(--ink-900);font-weight:900}
.mini-radar-card{overflow:hidden}
.home-radar-wrap{height:230px;border-radius:var(--r-lg);background:radial-gradient(120% 120% at 50% 120%,#0b1220,#111a2c 50%,#0a0f1c 80%);border:1px solid var(--line);overflow:hidden}
.home-radar-wrap canvas{width:100%!important;height:230px!important}
.sys-preflight{position:sticky;top:92px;box-shadow:var(--shadow-3)}

/* ---- COACH CARDS ---- */
.coach-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;margin-bottom:var(--gap);align-items:stretch}
.coach-grid .flt{border-radius:var(--r-lg);min-height:88px;padding:14px 16px 14px 20px;box-sizing:border-box;align-content:start;overflow:visible;box-shadow:var(--shadow-1)}
.coach-grid .flt-t,.coach-grid .flt-c{min-width:0;overflow-wrap:anywhere}
.coach-grid .flt-t{align-items:flex-start;line-height:1.25}
.coach-grid .flt-t .material-symbols-rounded{flex:0 0 auto}

/* ---- REPORT ---- */
.report-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:var(--gap);margin-top:var(--gap)}
.report-grid.three{grid-template-columns:repeat(3,minmax(0,1fr))}
.verdict-card{border-radius:var(--r-xl);padding:22px;background:var(--surface-1);border:1px solid var(--line);display:grid;gap:14px;box-shadow:var(--shadow-2)}
.verdict-card.ready{background:color-mix(in oklab,var(--ok-100) 45%,var(--surface-1));border-color:color-mix(in oklab,var(--ok-500) 18%,transparent)}
.verdict-card.degraded_signal,.verdict-card.provenance_warning{background:color-mix(in oklab,var(--ember-100) 45%,var(--surface-1));border-color:color-mix(in oklab,var(--ember-500) 18%,transparent)}
.verdict-card.schema_warning{background:color-mix(in oklab,var(--brand-100) 45%,var(--surface-1));border-color:color-mix(in oklab,var(--brand-500) 14%,transparent)}
.verdict-card.deferred{background:var(--surface-3)}
.verdict-card.firmware_rejected,.verdict-card.not_ready{background:color-mix(in oklab,var(--bad-100) 45%,var(--surface-1));border-color:color-mix(in oklab,var(--bad-500) 18%,transparent)}
.verdict-head{display:flex;align-items:flex-start;justify-content:space-between;gap:16px;flex-wrap:wrap}
.verdict-title{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:22px;font-weight:900;letter-spacing:-.01em}
.verdict-kind{display:inline-flex;align-items:center;gap:6px;padding:6px 10px;border-radius:var(--r-pill);font-size:12px;font-weight:900;text-transform:uppercase;letter-spacing:.06em}
.verdict-kind .material-symbols-rounded{font-size:16px}
.verdict-kind.ready{background:var(--ok-100);color:var(--ok-500)}
.verdict-kind.degraded_signal,.verdict-kind.provenance_warning{background:var(--ember-100);color:var(--ember-500)}
.verdict-kind.schema_warning{background:var(--brand-100);color:var(--brand-500)}
.verdict-kind.deferred{background:var(--surface-3);color:var(--ink-500)}
.verdict-kind.firmware_rejected,.verdict-kind.not_ready{background:var(--bad-100);color:var(--bad-500)}
.report-list{display:grid;gap:8px}
.report-list .flt{border-radius:var(--r-md)}
.stat-list{display:grid;gap:8px}
.stat-list .stat{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:12px;align-items:baseline;padding:10px 12px;border-radius:var(--r-md);background:var(--surface-3);border:1px solid var(--line)}
.stat-list .stat .k{font-size:13px;font-weight:700;color:var(--ink-500)}
.stat-list .stat .v{font-family:'JetBrains Mono',monospace;font-size:13px;font-weight:700;text-align:right;word-break:break-word;color:var(--ink-900)}
.stat-list .stat.good{background:var(--ok-100);color:var(--ok-500);border-color:transparent}
.stat-list .stat.warn{background:var(--ember-100);color:var(--ember-500);border-color:transparent}
.stat-list .stat.bad{background:var(--bad-100);color:var(--bad-500);border-color:transparent}
.mini-bars{display:grid;gap:8px}
.mini-bar{display:grid;grid-template-columns:minmax(120px,1fr) minmax(100px,2fr) auto;gap:10px;align-items:center;font-size:12px}
.mini-bar-track{height:8px;border-radius:var(--r-pill);background:var(--surface-3);overflow:hidden}
.mini-bar-fill{height:100%;background:linear-gradient(90deg,var(--brand-500),var(--signal-500));border-radius:inherit}
.verdict-actions{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.compare-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;margin-top:10px}
.compare-card{padding:12px;border-radius:var(--r-md);background:var(--surface-3);border:1px solid var(--line)}
.compare-card h3{margin:0 0 8px;font-size:14px}
.contract-list{display:grid;gap:8px;margin-top:10px}
.contract-list .flt{border-radius:var(--r-md)}

/* ---- HELP ---- */
[data-help]{cursor:help}
.help-toolbar{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin-bottom:16px}
.help-search{min-width:260px;flex:1;padding:11px 14px;border-radius:var(--r-md);border:1px solid var(--line);background:var(--surface-2);color:var(--ink-900);font:500 14px/1.3 inherit}
.help-section{margin-top:12px;padding:14px;border:1px solid var(--line);border-radius:var(--r-lg);background:var(--surface-1)}
.help-section summary{font-weight:900;cursor:pointer;font-size:14px}
.help-section dl{display:grid;gap:10px;margin:10px 0 0}
.help-section dt{font-weight:900;font-size:13px}
.help-section dd{margin:2px 0 0;color:var(--ink-500);line-height:1.45;font-size:13px}
.tip-pop{position:fixed;z-index:1000;max-width:320px;padding:10px 12px;border-radius:var(--r-md);background:var(--ink-900);color:#fff;box-shadow:var(--shadow-3);font-size:12px;line-height:1.4;pointer-events:none;display:none}

/* ---- FOCUS / A11Y ---- */
:where(button,a,input,select,textarea,.r-item,.chip-btn,.pf-tile,.sw,.ca-btn,.ic-btn,.fab,.setup-start,[tabindex]):focus-visible{
  outline:2px solid var(--brand-500);outline-offset:2px;border-radius:inherit;
}
.countdown-dots{display:inline-flex;gap:6px;margin-left:10px;vertical-align:middle}
.countdown-dots span{width:8px;height:8px;border-radius:50%;background:var(--line-2);transition:background var(--dur-s) var(--ease)}
.countdown-dots span.on{background:var(--brand-500);box-shadow:0 0 0 3px color-mix(in oklab,var(--brand-500) 20%,transparent)}
.kbd-hints{display:flex;flex-wrap:wrap;gap:14px;padding:10px 4px 0;font-size:11px;color:var(--ink-400);font-family:'JetBrains Mono',monospace}
.kbd-hints .khint{display:inline-flex;align-items:center;gap:6px}
.sticky-cta{position:fixed;left:50%;transform:translateX(-50%);bottom:20px;z-index:140;display:none;align-items:center;gap:10px;padding:8px 10px 8px 16px;border-radius:var(--r-pill);background:var(--surface-1);box-shadow:var(--shadow-2);border:1px solid var(--line);font-size:13px;font-weight:700;color:var(--ink-700)}
.sticky-cta.visible{display:inline-flex}
.sticky-cta button{appearance:none;border:none;padding:10px 18px;border-radius:var(--r-pill);background:var(--brand-500);color:#fff;font:900 13px/1 inherit;cursor:pointer}
.sticky-cta button:disabled{opacity:.45;cursor:not-allowed}
.sticky-cta .sticky-note{color:var(--ink-400);font-weight:500;font-size:12px}

/* ---- RESPONSIVE ---- */
@media (max-width:1200px){
  .app{grid-template-columns:1fr}
  .rail{position:sticky;top:0;height:auto;flex-direction:row;overflow-x:auto;padding:10px 12px;border-right:none;border-bottom:1px solid var(--line)}
  .brand{width:48px;height:48px;margin:0 8px 0 0;flex:0 0 auto}
  .r-item{grid-template-columns:32px auto;min-width:max-content}
  .r-item .nav-kbd{display:none}
  .rail-foot{margin:0 0 0 auto;flex-direction:row;border-top:none;padding-top:0}
  .operator{display:none}
  .r-item.active::before{display:none}
  .home-layout{grid-template-columns:1fr}
  .sys-preflight{position:relative;top:auto}
  .kpi-grid{grid-template-columns:repeat(2,minmax(0,1fr))}
  .rad{grid-template-columns:1fr}
  .c3,.c4{grid-template-columns:repeat(2,minmax(0,1fr))}
}
@media (max-width:900px){
  .report-grid,.report-grid.three{grid-template-columns:1fr}
  .coach-grid,.compare-grid{grid-template-columns:1fr}
}
@media (max-width:760px){
  body{overflow-x:hidden}
  .app{width:100%;max-width:100vw;overflow-x:hidden}
  .rail{width:100%;max-width:100vw;min-width:0}
  .main{width:100%;max-width:100vw;min-width:0;padding:16px;overflow-x:hidden}
  .topbar{display:grid;grid-template-columns:1fr;align-items:start}
  .topbar .sp{display:none}
  .topbar h1{font-size:30px;overflow-wrap:anywhere}
  .welcome-hero{padding:28px 18px}
  .hero-title{font-size:34px;max-width:100%;overflow-wrap:anywhere}
  .hero-subtitle,.sandbox-note{max-width:100%;overflow-wrap:anywhere}
  .k-val{font-size:40px}
  .tl .v{font-size:22px}
  .hardware-row,.meta-row{grid-template-columns:1fr;gap:6px}
  .kpi-grid,.c2,.c3,.c4,.wg,.tbig{grid-template-columns:1fr}
  .coach-grid{grid-template-columns:1fr}
}
</style>
</head>
<body>

<div class="app">
<aside class="rail" aria-label="Primary">
  <div class="brand" title="Radar Vital Trainer">
    <div class="brand-mark"><span class="brand-ring"></span><span class="brand-ring r2"></span><span class="brand-dot"></span></div>
    <div class="brand-label"><span class="bl-top">RADAR</span><span class="bl-bot">VITAL · v15.1</span></div>
  </div>
  <nav class="nav-groups" aria-label="Views">
    <div class="nav-group" id="workflowGroup">
      <div class="nav-heading">Workflow</div>
    </div>
    <div class="nav-group" id="liveTabsGroup">
      <div class="nav-heading">Live views</div>
      <button class="r-item sub active" data-target="tab-overview" onclick="switchTab('tab-overview')"><span class="r-ic"><span class="material-symbols-rounded">dashboard</span></span>Overview</button>
      <button class="r-item sub" data-target="tab-waves" onclick="switchTab('tab-waves')"><span class="r-ic"><span class="material-symbols-rounded">show_chart</span></span>Waves</button>
      <button class="r-item sub" data-target="tab-hr" onclick="switchTab('tab-hr')"><span class="r-ic"><span class="material-symbols-rounded">ecg_heart</span></span>HR funnel</button>
      <button class="r-item sub" data-target="tab-rr" onclick="switchTab('tab-rr')"><span class="r-ic"><span class="material-symbols-rounded">pulmonology</span></span>RR funnel</button>
      <button class="r-item sub" data-target="tab-snaps" onclick="switchTab('tab-snaps')"><span class="r-ic"><span class="material-symbols-rounded">bookmark</span></span>Snapshots</button>
      <button class="r-item sub" data-target="tab-audit" onclick="switchTab('tab-audit')"><span class="r-ic"><span class="material-symbols-rounded">fact_check</span></span>Audit</button>
    </div>
  </nav>
  <div class="rail-foot">
    <button class="r-item rail-action" onclick="openPalette()" title="Command palette (⌘K)"><span class="r-ic"><span class="material-symbols-rounded">search</span></span>Command<span class="nav-kbd">⌘K</span></button>
    <div class="operator">
      <div class="op-avatar" id="opAvatar">OA</div>
      <div class="op-meta">
        <div class="op-name" id="opName">Operator A</div>
        <div class="op-role" id="opRole">Lab · Station 3</div>
      </div>
    </div>
  </div>
</aside>

<main class="main">

  <div class="topbar">
    <div>
      <div class="eyebrow">Live collection · v14 / v9</div>
      <h1>Radar Vital Trainer</h1>
    </div>
    <div class="sp"></div>
    <div class="conn" id="conn"><span class="dot"></span><span id="connT">Connecting…</span></div>
    <div class="mode-badge" id="modeBadge"><span class="material-symbols-rounded">wifi_tethering_off</span><span id="modeBadgeT">waiting</span></div>
    <div class="seg" role="group">
      <button class="active" data-density="comfortable" onclick="setDensity('comfortable')" title="Comfortable"><span class="material-symbols-rounded">density_medium</span></button>
      <button data-density="compact" onclick="setDensity('compact')" title="Compact"><span class="material-symbols-rounded">density_small</span></button>
    </div>
    <button class="ic-btn" onclick="togglePause()" title="Pause / resume live polling"><span class="material-symbols-rounded" id="pauseIcon">pause</span></button>
    <button class="ic-btn" onclick="toggleTheme()" title="Toggle dark mode"><span class="material-symbols-rounded" id="themeIcon">dark_mode</span></button>
    <button class="ic-btn" onclick="openSettings()" title="Settings"><span class="material-symbols-rounded">tune</span></button>
    <button class="ic-btn" onclick="openDrawer()" title="Alerts"><span class="material-symbols-rounded">notifications</span><span class="ic-badge" id="alertsBadge" style="display:none;">0</span></button>
    <button class="fab" onclick="exportData()" title="Export JSON payload"><span class="material-symbols-rounded">download</span>Export</button>
  </div>

  <div class="fw warn" id="fwBadge" data-help="fw_truthfulness" tabindex="0">
    <span class="material-symbols-rounded">gpp_maybe</span>
    <div>
      <div class="fw-title">Firmware truthfulness</div>
      <div class="fw-line"><span>sketch=--</span><span>module=--</span><span>module_valid=--</span><span>schema=--</span></div>
    </div>
    <button class="ca-btn" onclick="switchTab('tab-audit')" title="View audit details"><span class="material-symbols-rounded">arrow_forward</span></button>
  </div>

  <div class="chips" id="chips"></div>
  <div style="height:16px"></div>

  <!-- OVERVIEW -->
  <div id="tab-overview" class="tab active">
    <div class="kpi-grid" style="margin-bottom:var(--gap)">
      <div class="kpi hr" data-help="kpi.hr" tabindex="0">
        <div class="k-lab"><span class="k-lab-t">Heart rate</span><div class="k-ic"><span class="material-symbols-rounded">ecg_heart</span></div></div>
        <div class="k-val-r"><span class="k-val" id="kpiHr">--</span><span class="k-u">bpm</span></div>
        <canvas class="k-spark" id="kpiHrSpark"></canvas>
        <div class="k-sub" id="kpiHrSub">waiting</div>
      </div>
      <div class="kpi rr" data-help="kpi.rr" tabindex="0">
        <div class="k-lab"><span class="k-lab-t">Respiration</span><div class="k-ic"><span class="material-symbols-rounded">air</span></div></div>
        <div class="k-val-r"><span class="k-val" id="kpiRr">--</span><span class="k-u">br/min</span></div>
        <canvas class="k-spark" id="kpiRrSpark"></canvas>
        <div class="k-sub" id="kpiRrSub">waiting</div>
      </div>
      <div class="kpi" data-help="kpi.fps" tabindex="0">
        <div class="k-lab"><span class="k-lab-t">Frame rate</span><div class="k-ic"><span class="material-symbols-rounded">speed</span></div></div>
        <div class="k-val-r"><span class="k-val" id="kpiFps">--</span><span class="k-u">Hz</span></div>
        <canvas class="k-spark" id="kpiFpsSpark"></canvas>
        <div class="k-sub" id="kpiFpsSub">DSP loop</div>
      </div>
      <div class="kpi" data-help="kpi.distance" tabindex="0">
        <div class="k-lab"><span class="k-lab-t">Range</span><div class="k-ic"><span class="material-symbols-rounded">radar</span></div></div>
        <div class="k-val-r"><span class="k-val" id="kpiDist">--</span><span class="k-u">cm</span></div>
        <canvas class="k-spark" id="kpiDistSpark"></canvas>
        <div class="k-sub" id="kpiDistSub">target distance</div>
      </div>
    </div>

    <article class="card" style="margin-bottom:var(--gap)">
      <div class="ch">
        <div class="tw"><span class="material-symbols-rounded">explore</span><div><h2 class="ct">Position / module view</h2><div class="cs">Target-info-first spatial telemetry with live module side</div></div></div>
        <div class="ca"><button class="ca-btn" onclick="snapshotNow()" title="Pin snapshot"><span class="material-symbols-rounded">bookmark_add</span></button></div>
      </div>
      <div class="rad">
        <div class="rp">
          <div class="rp-h">
            <div><div class="eyebrow" style="color:var(--ink-400)">Spatial view</div><div class="rp-n">Semicircle position uses target-info-first. Module telemetry stays in sync below.</div></div>
            <div class="tgt" id="tgtCount">Target: --</div>
          </div>
          <div class="rad-w"><canvas id="radarCanvas"></canvas></div>
        </div>
        <div class="side">
          <div class="tb">
            <div class="tbig">
              <div class="tl"><span class="l">FPS</span><span class="v" id="fpsV">--</span></div>
              <div class="tl"><span class="l">Breath</span><span class="v rr" id="brV">--</span></div>
              <div class="tl"><span class="l">Heart</span><span class="v" id="hrV">--</span></div>
            </div>
            <div class="kv">
              <div>Person detected</div><div id="personV">--</div>
              <div>Distance</div><div id="distV">--</div>
              <div>X position</div><div id="xV">--</div>
              <div>Y position</div><div id="yV">--</div>
              <div>Radius</div><div id="radV">--</div>
              <div>Firmware</div><div id="fwV">--</div>
              <div>Spatial source</div><div id="srcV">--</div>
              <div>Spatial age</div><div id="srcAgeV">--</div>
              <div>Doppler index</div><div id="dopV">--</div>
              <div>Doppler speed</div><div id="dopCmsV">--</div>
            </div>
          </div>
          <div class="tb">
            <div class="ct" style="font-size:14px;margin-bottom:10px;">Position source summary</div>
            <table><tbody id="posTable"></tbody></table>
          </div>
        </div>
      </div>
    </article>

    <article class="card" style="margin-bottom:var(--gap)">
      <div class="ch"><div class="tw"><span class="material-symbols-rounded">rule_settings</span><div><h2 class="ct">Physiology vs policy</h2><div class="cs">Signal quality and publish policy in one view</div></div></div></div>
      <div class="grid c2">
        <div class="tb"><h3 style="margin:0 0 10px;font-size:13px;font-weight:900;letter-spacing:.06em;text-transform:uppercase;color:var(--ink-400)">Physiology</h3><table><tbody id="physTable"></tbody></table></div>
        <div class="tb"><h3 style="margin:0 0 10px;font-size:13px;font-weight:900;letter-spacing:.06em;text-transform:uppercase;color:var(--ink-400)">Policy</h3><table><tbody id="polTable"></tbody></table></div>
      </div>
    </article>

    <div class="grid c4">
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">radar</span><div><h2 class="ct">Radar</h2><div class="cs">Latest parsed row</div></div></div></div>
        <div class="mr"><div class="m"><span class="m-l">HR</span><div class="m-vr"><span class="m-v" id="radHr">--</span><span class="m-u">bpm</span></div></div><div class="m rr"><span class="m-l">RR</span><div class="m-vr"><span class="m-v" id="radRr">--</span><span class="m-u">br/m</span></div></div></div>
        <table><tbody id="radarTable"></tbody></table>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">bluetooth_connected</span><div><h2 class="ct">BLE Reference</h2><div class="cs">Oximeter packet</div></div></div></div>
        <div class="mr"><div class="m"><span class="m-l">HR</span><div class="m-vr"><span class="m-v" id="bleHr">--</span><span class="m-u">bpm</span></div></div><div class="m rr"><span class="m-l">RR</span><div class="m-vr"><span class="m-v" id="bleRr">--</span><span class="m-u">br/m</span></div></div></div>
        <table><tbody id="bleTable"></tbody></table>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">difference</span><div><h2 class="ct">Mismatch</h2><div class="cs">Operator-facing delta</div></div></div></div>
        <div class="faults" id="faults"></div>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">my_location</span><div><h2 class="ct">Spatial</h2><div class="cs">Point cloud & Doppler</div></div></div></div>
        <div class="mr three"><div class="m"><span class="m-l">Targets</span><div class="m-vr"><span class="m-v" id="numT">--</span></div></div><div class="m rr"><span class="m-l">|Dop|</span><div class="m-vr"><span class="m-v" id="maxDop">--</span></div></div><div class="m"><span class="m-l">cm/s</span><div class="m-vr"><span class="m-v" id="maxDopSp">--</span></div></div></div>
        <table><tbody id="spTable"></tbody></table>
      </article>
    </div>
  </div>

  <!-- WAVES -->
  <div id="tab-waves" class="tab">
    <article class="card" style="margin-bottom:var(--gap)">
      <div class="ch"><div class="tw"><span class="material-symbols-rounded">monitor_heart</span><div><h2 class="ct">Live phase waveforms</h2><div class="cs">breath_phase and heart_phase from 0x0A13</div></div></div><div class="ca"><button class="ca-btn" onclick="fsCard(this)" title="Fullscreen"><span class="material-symbols-rounded">fullscreen</span></button></div></div>
      <div class="wg"><div class="wc"><h3 class="wt">Breath</h3><div class="ws">Live breath_phase history</div><canvas id="breathChart"></canvas></div><div class="wc"><h3 class="wt">Heartbeat</h3><div class="ws">Live heart_phase history</div><canvas id="heartChart"></canvas></div></div>
    </article>
    <div class="grid c2">
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">show_chart</span><div><h2 class="ct">Heart-rate trend</h2><div class="cs">Reported, candidate, raw, corrected raw, BLE, anchor</div></div></div>
          <div class="ca">
            <div class="seg" style="margin-right:6px">
              <button data-range="30" onclick="setRange(this,30)">30s</button>
              <button class="active" data-range="120" onclick="setRange(this,120)">2m</button>
              <button data-range="300" onclick="setRange(this,300)">5m</button>
              <button data-range="all" onclick="setRange(this,'all')">All</button>
            </div>
            <button class="ca-btn" onclick="fsCard(this)" title="Fullscreen"><span class="material-symbols-rounded">fullscreen</span></button>
          </div>
        </div>
        <div class="cp"><canvas id="hrChart"></canvas></div>
      </article>
      <article class="card">
        <div class="ch"><div class="tw rr"><span class="material-symbols-rounded">show_chart</span><div><h2 class="ct">Respiration trend</h2><div class="cs">Reported, candidate, final, anchor, BLE</div></div></div><div class="ca"><button class="ca-btn" onclick="fsCard(this)" title="Fullscreen"><span class="material-symbols-rounded">fullscreen</span></button></div></div>
        <div class="cp"><canvas id="rrChart"></canvas></div>
      </article>
    </div>
  </div>

  <!-- HR -->
  <div id="tab-hr" class="tab">
    <article class="card" style="margin-bottom:var(--gap)">
      <div class="ch"><div class="tw"><span class="material-symbols-rounded">scatter_plot</span><div><h2 class="ct">Raw-HR vs ref-HR rate-bucketed scatter</h2><div class="cs">Mean raw-minus-reference HR bias per 10-bpm reference bucket</div></div></div></div>
      <div class="cp s"><canvas id="rawHrBucketChart"></canvas></div>
      <table style="margin-top:12px"><tbody id="bucketTable"></tbody></table>
    </article>
    <div class="grid c2">
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">conversion_path</span><div><h2 class="ct">HR funnel telemetry</h2><div class="cs">Arbiter → reject-phase → blend → coherence → publish</div></div></div></div>
        <div class="pills" id="hrPills" style="margin-bottom:14px"></div>
        <table><tbody id="hrFunTable"></tbody></table>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">analytics</span><div><h2 class="ct">HR stage values</h2><div class="cs">Values carried through the HR path</div></div></div></div>
        <table><tbody id="hrStageTable"></tbody></table>
      </article>
    </div>
  </div>

  <!-- RR -->
  <div id="tab-rr" class="tab">
    <div class="grid c2" style="margin-bottom:var(--gap)">
      <article class="card">
        <div class="ch"><div class="tw rr"><span class="material-symbols-rounded">air</span><div><h2 class="ct">RR funnel telemetry</h2><div class="cs">Selector → accept → blend → Kalman → publish</div></div></div></div>
        <div class="pills" id="rrPills" style="margin-bottom:14px"></div>
        <table><tbody id="rrFunTable"></tbody></table>
      </article>
      <article class="card">
        <div class="ch"><div class="tw rr"><span class="material-symbols-rounded">timeline</span><div><h2 class="ct">RR stage values</h2><div class="cs">Anchor, recovery, final stage values</div></div></div></div>
        <table><tbody id="rrStageTable"></tbody></table>
      </article>
    </div>
    <div class="grid c2">
      <article class="card">
        <div class="ch"><div class="tw rr"><span class="material-symbols-rounded">monitoring</span><div><h2 class="ct">RR recovery diagnostics</h2><div class="cs">Confidence, recovery, seed consistency</div></div></div><div class="ca"><button class="ca-btn" onclick="fsCard(this)" title="Fullscreen"><span class="material-symbols-rounded">fullscreen</span></button></div></div>
        <div class="cp"><canvas id="rrDiagChart"></canvas></div>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">notification_important</span><div><h2 class="ct">RR / stale-state warnings</h2><div class="cs">Anchor, candidate aging, integrity</div></div></div></div>
        <div class="faults" id="rrFaults"></div>
      </article>
    </div>
  </div>

  <!-- SNAPSHOTS -->
  <div id="tab-snaps" class="tab">
    <article class="card">
      <div class="ch">
        <div class="tw"><span class="material-symbols-rounded">bookmark</span><div><h2 class="ct">Pinned snapshots</h2><div class="cs">Capture an HR/RR/BLE/delta snapshot at the current frame to compare later</div></div></div>
        <div class="ca">
          <button class="fab" style="padding:8px 14px;font-size:13px" onclick="snapshotNow()"><span class="material-symbols-rounded">add</span>Pin current</button>
          <button class="ca-btn" onclick="clearSnaps()" title="Clear all"><span class="material-symbols-rounded">delete_sweep</span></button>
        </div>
      </div>
      <div class="snaps" id="snapsList"></div>
    </article>
  </div>

  <!-- AUDIT -->
  <div id="tab-audit" class="tab">
    <div class="grid c2" style="margin-bottom:var(--gap)">
      <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">fact_check</span><div><h2 class="ct">Analysis audit</h2><div class="cs">Gates, firmware, funnel survival</div></div></div></div><table><tbody id="audTable"></tbody></table></article>
      <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">stacked_bar_chart</span><div><h2 class="ct">Reason histograms</h2><div class="cs">Top block reasons + AGC</div></div></div></div><table><tbody id="histTable"></tbody></table></article>
    </div>
    <div class="grid c2">
      <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">bluetooth_searching</span><div><h2 class="ct">BLE ref quality</h2><div class="cs">Packet, decode, perfusion</div></div></div></div><table><tbody id="bleQTable"></tbody></table></article>
      <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">history</span><div><h2 class="ct">Recent events</h2><div class="cs">Live tail from current session</div></div></div></div><pre id="evLog">Waiting for data...</pre></article>
    </div>
    <div class="ft" id="foot"></div>
  </div>

</main>
</div>

<div class="dv-ov" id="dvOv" onclick="closeDrawer()"></div>
<aside class="dv" id="dv" aria-hidden="true">
  <div class="dv-h"><h3>Alerts</h3><button class="ic-btn" onclick="closeDrawer()" title="Close"><span class="material-symbols-rounded">close</span></button></div>
  <div class="dv-b" id="dvBody"></div>
</aside>

<div class="p-ov" id="pOv" onclick="if(event.target===this)closePalette()">
  <div class="p">
    <div class="p-iw"><span class="material-symbols-rounded">search</span><input class="p-in" id="pIn" placeholder="Type a command…" autocomplete="off"></div>
    <div class="p-l" id="pL"></div>
    <div class="p-hint"><span><kbd>↑</kbd> <kbd>↓</kbd> navigate</span><span><kbd>⏎</kbd> run</span><span><kbd>esc</kbd> close</span></div>
  </div>
</div>

<div class="set-ov" id="settingsOv" onclick="if(event.target===this)closeSettings()">
  <div class="set-md">
    <div class="set-h">
      <div>
        <h3 class="set-tt">Dashboard settings</h3>
        <div class="set-st">Keep live mode honest. Demo mode is explicit, not silent.</div>
      </div>
      <button class="ic-btn" onclick="closeSettings()" title="Close settings"><span class="material-symbols-rounded">close</span></button>
    </div>
    <section class="set-g">
      <h4 class="set-gl">Source mode</h4>
      <div class="set-r"><div class="set-copy"><strong>Demo mode</strong><span>Use generated sample telemetry instead of the trainer JSON feed. Intended for UI demos only.</span></div><button class="sw" id="demoModeSwitch" onclick="toggleDemoMode()" aria-pressed="false" title="Toggle demo mode"></button></div>
      <div class="set-r"><div class="set-copy"><strong>Auto-enable demo on disconnect</strong><span>If live polling fails repeatedly, switch to demo automatically. Default is off.</span></div><button class="sw" id="autoDemoSwitch" onclick="toggleAutoDemo()" aria-pressed="false" title="Toggle auto demo on disconnect"></button></div>
    </section>
    <section class="set-g">
      <h4 class="set-gl">Disconnect behavior</h4>
      <div class="set-r"><div class="set-copy"><strong>Freeze on stale live data</strong><span>When live data disappears, keep the last real payload visible and mark the dashboard stale instead of simulating new activity.</span></div><button class="sw on" id="freezeOnStaleSwitch" onclick="toggleFreezeOnStale()" aria-pressed="true" title="Toggle freeze on stale"></button></div>
    </section>
    <div class="set-actions"><button class="txt-btn" onclick="closeSettings()">Done</button></div>
  </div>
</div>

<div class="toast-host" id="toasts"></div>

<script>
const S = {
  charts:{}, dataUrl:'./live_dashboard.json', refreshMs:1000, lastPayload:null, lastLivePayload:null,
  disc:0, hardStaleN:5, hardStaleS:5, paused:false, rangeS:120,
  snaps: JSON.parse(localStorage.getItem('rvt-snaps')||'[]'),
  alertKeys:new Set(),
  spark:{ hr:[], rr:[], fps:[], dist:[] },
  demoT:0,
  demoMode:false,
  autoDemoOnDisconnect:false,
  freezeOnStale:true,
  autoDemoActive:false,
  ctl:{on:false,status:null,current:null,defaults:null,preflight:null,help:null,lastSessionId:null,stopPending:false,helpQuery:'',helpAdvanced:localStorage.getItem('rvt-help-advanced')==='1'},
  setup:{duration_s:30,customDuration:30,customUnit:'s',radar_port:'COM10',ble_address:'10:22:33:9E:8F:63',ble_profile:'ailink_oximeter',notify_char:'0000ffe2-0000-1000-8000-00805f9b34fb',subject_label:'',operator_label:'',skip_countdown:false},
};

function num(v){ const n=Number(v); return Number.isFinite(n)?n:null; }
function fmt(v,d=1){ const n=num(v); return n===null?'--':n.toFixed(d); }
function boolish(v){
  if(v===true||v===false) return v;
  if(v==null||v==='') return false;
  if(typeof v==='string'){ const s=v.trim().toLowerCase(); if(!s||s==='--'||s==='null'||s==='nan'||s==='none') return false; if(['false','no','n','off','fail','failed','invalid'].includes(s)) return false; if(['true','yes','y','on','pass','passed','valid'].includes(s)) return true; }
  const n=num(v); return n!==null?(n>=0.5):!!v;
}
function textOrDash(v){ if(v==null) return '--'; if(typeof v==='number'&&!Number.isFinite(v)) return '--'; const s=String(v); return (!s.trim()||s.trim().toLowerCase()==='nan')?'--':s; }
function esc(v){ return textOrDash(v).replace(/[&<>"']/g, ch=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch])); }
function safeCls(c){ return ['good','warn','bad'].includes(c)?c:''; }
function safeIcon(i){ return String(i||'info').replace(/[^a-z0-9_]/gi,'')||'info'; }
function row(l,v){ return `<tr><td>${esc(l)}</td><td>${esc(v)}</td></tr>`; }
function validSeriesOrNull(vals,flags){ if(!Array.isArray(vals)) return []; return vals.map((v,i)=>{ const ok=Array.isArray(flags)?boolish(flags[i]):true; const n=num(v); if(!ok) return null; return (n!==null&&n>0)?n:null; }); }
function livePub(p,k){ const v=getR(p,k); const n=num(v); if(n===null||n<=0) return null; if(k==='reported_hr'&&!boolish(getR(p,'logged_hr_valid'))) return null; if(k==='reported_rr'&&!boolish(getR(p,'logged_rr_valid'))) return null; return n; }
function chip(l,c,i){ return `<div class="chip ${safeCls(c)}"><span class="material-symbols-rounded">${safeIcon(i)}</span>${esc(l)}</div>`; }
function pill(l,v){ let c=''; const n=typeof v==='string'?v.trim().toLowerCase():v; if(n===1||n===true||n==='1'||n==='yes'||n==='triggered') c='good'; else if(n===0||n===false||n==='0'||n==='no') c='warn'; return `<div class="pill ${c}">${esc(l)}: ${esc(v)}</div>`; }
function flt(s,t,c){ const lv=safeCls(s)||'warn'; const i=lv==='bad'?'error':lv==='warn'?'warning':'check_circle'; return `<div class="flt ${lv}"><div class="flt-t"><span class="material-symbols-rounded">${i}</span>${esc(t)}</div><div class="flt-c">${esc(c)}</div></div>`; }
function topHist(h,lim=3){ if(!h||typeof h!=='object') return '--'; const a=Object.entries(h).sort((a,b)=>(Number(b[1])||0)-(Number(a[1])||0)); return a.length?a.slice(0,lim).map(([k,v])=>`${k}=${v}`).join(' | '):'--'; }
function first(...vs){ for(const v of vs) if(v!==null&&v!==undefined&&textOrDash(v)!=='--') return v; return null; }
function shortHash(v){ const s=textOrDash(v); return s==='--'?'--':(s.length>16?`${s.slice(0,12)}...`:s); }
function featureSchema(p){ const a=p.analysis||{}; const m=a.feature_manifest||a.manifest||a.model_manifest||{}; return first(m.feature_schema_hash,m.schema_hash,m.hash,a.feature_schema_hash,a.schema_hash,p.meta?.feature_schema_hash,p.radar?.feature_schema_hash); }
function fwFromKeys(o,k){ const p=k.map(x=>num(o?.[x])); return p.every(v=>v!==null&&v>=0)?p.map(v=>Math.round(v)).join('.'):null; }
function fwTruth(p){
  const r=p.radar||{}; const a=p.analysis||{}; const fw=a.fw_truthfulness||{};
  const s=first(r.sketch_firmware_version,r.firmware_version,fw.version,fwFromKeys(r,['sketch_major','sketch_sub','sketch_mod']));
  const m=first(r.module_firmware_version,fw.module_version,fwFromKeys(r,['module_fw_major','module_fw_sub','module_fw_mod']),'unknown');
  const mvRaw=first(r.module_fw_valid,fw.module_version_valid); const mvKnown=mvRaw!==null&&mvRaw!==undefined&&textOrDash(mvRaw)!=='--'; const mv=mvKnown?boolish(mvRaw):false;
  return { sketch:textOrDash(s), moduleFw:textOrDash(m), moduleValid:mv, moduleValidText:mv?'True':'False', schemaHash:shortHash(featureSchema(p)), sev:mv?'good':(mvKnown||textOrDash(m).toLowerCase()==='unknown'?'bad':'warn') };
}
function renderFwBadge(p){ const i=fwTruth(p); const b=document.getElementById('fwBadge'); b.className=`fw ${i.sev}`; const ic=i.sev==='good'?'verified':i.sev==='bad'?'gpp_maybe':'help'; b.querySelector('.material-symbols-rounded').textContent=ic; b.querySelector('div div + div').innerHTML=`<span>sketch=${esc(i.sketch)}</span><span>module=${esc(i.moduleFw)}</span><span>module_valid=${esc(i.moduleValidText)}</span><span>schema=${esc(i.schemaHash)}</span>`; }
function normalize(r){ return { meta:r?.meta||{}, radar:r?.radar||{}, ble:r?.ble||{}, thresholds:r?.thresholds||{}, faults:Array.isArray(r?.faults)?r.faults:[], events:Array.isArray(r?.events)?r.events:[], series:r?.series||{}, analysis:r?.analysis||null }; }
function lastSeries(s,k){ const a=Array.isArray(s?.[k])?s[k]:[]; return a.length?a[a.length-1]:null; }
function getR(p,k){ const r=p.radar||{}; if(r[k]!==undefined) return r[k]; const d=lastSeries(p.series,k); return d!==null?d:null; }
function stageOr(p,prim,fb=[]){ const v=getR(p,prim); if(v!=null) return v; for(const k of fb){ const x=getR(p,k); if(x!=null) return x; } return null; }
function rrStage(p,prim,fb=[]){ if(!boolish(getR(p,'rr_anchor_fresh'))&&prim==='rr_post_kalman') return null; return stageOr(p,prim,fb); }
function sourceName(v){ return ({0:'none',1:'target_info',2:'point_cloud',3:'frame_0x0A17'}[Math.round(num(v)??0)]||textOrDash(v)); }
function phaseName(v){ return ({0:'absent',1:'warmup',2:'settling',3:'locked',4:'post_motion',5:'leaving'}[Math.round(num(v)??0)]||textOrDash(v)); }
function harmonicSum(v){ const n=Math.round(num(v)??0); if(!n) return 'none'; const t=[]; if(n&1) t.push('rr_raw'); if(n&2) t.push('hr_arb'); if(n&4) t.push('hr_half'); if(n&8) t.push('hr_amb'); if(n&16) t.push('rr_sub'); if(n&32) t.push('rr_harm'); return t.join('|'); }
function hrPathName(v,fb){ if(fb) return String(fb); const n=Math.round(num(v)??-999); return ({0:'none',1:'auto publish',2:'spectral path',3:'raw blend',4:'raw fallback',5:'raw only (no phase)',6:'direct raw fast path'})[n]||textOrDash(v); }
function hrAnchorName(v,fb){ if(fb) return String(fb); const n=Math.round(num(v)??-999); return ({0:'none',1:'trusted_phase',2:'tracking_raw',3:'latched_raw',4:'arbiter_anchor',5:'rejectphase_anchor'})[n]||textOrDash(v); }
function computeFps(p){ const e=num(getR(p,'fps_hz')); if(e!==null&&e>0) return e; const t=Array.isArray(p?.series?.t)?p.series.t.map(num).filter(v=>v!==null):[]; if(t.length<3) return null; const d=[]; for(let i=1;i<t.length;i++){ const x=t[i]-t[i-1]; if(x>0) d.push(x); } if(!d.length) return null; d.sort((a,b)=>a-b); const m=d[Math.floor(d.length/2)]; return m>0?1/m:null; }
function hardStale(p){ const age=num(p?.radar?.last_seen_age_s); return S.disc>=S.hardStaleN||(age!==null&&age>=S.hardStaleS); }
function bucketData(p){
  const a=p.analysis||{}; const ad=a.raw_hr_bias_audit||a.raw_hr_bias_estimate||{};
  const ab=Array.isArray(ad.rate_buckets)?ad.rate_buckets:[];
  if(ab.length){ return ab.filter(b=>num(b.bias_bpm)!==null&&num(b.n)!==null&&num(b.n)>0).map(b=>{ const lo=num(b.ref_hr_min),hi=num(b.ref_hr_max); const x=(lo!==null&&hi!==null)?((lo+hi)/2):num(b.ref_hr_mean); return {x:x??0,y:num(b.bias_bpm),bucket:textOrDash(b.bucket),count:num(b.n)}; }); }
  const s=p.series||{}; const raw=Array.isArray(s.raw_hr)?s.raw_hr:[]; const ref=Array.isArray(s.ref_hr)?s.ref_hr:(Array.isArray(s.ble_hr)?s.ble_hr:[]);
  const b=new Map(); const n=Math.min(raw.length,ref.length);
  for(let i=0;i<n;i++){ const rh=num(raw[raw.length-n+i]), rf=num(ref[ref.length-n+i]); if(rh===null||rf===null||rh<=0||rf<=0) continue; const k=Math.floor(rf/10)*10; const it=b.get(k)||{bucket:k,count:0,sum:0}; it.count++; it.sum+=rh-rf; b.set(k,it); }
  return [...b.values()].sort((a,b)=>a.bucket-b.bucket).map(i=>({x:i.bucket+5,y:i.sum/i.count,bucket:`${i.bucket}-${i.bucket+9}`,count:i.count}));
}

function setDensity(d){ document.documentElement.setAttribute('data-density',d); document.querySelectorAll('.seg button[data-density]').forEach(el=>el.classList.toggle('active', el.dataset.density===d)); localStorage.setItem('rvt-density',d); }
function toggleTheme(){ const h=document.documentElement; const t=h.getAttribute('data-theme'); const nt=t==='dark'?'light':'dark'; h.setAttribute('data-theme',nt); document.getElementById('themeIcon').textContent=nt==='dark'?'light_mode':'dark_mode'; localStorage.setItem('rvt-theme',nt); repaintCharts(); }
function repaintCharts(){ const dark=document.documentElement.getAttribute('data-theme')==='dark'; const g=dark?'rgba(255,255,255,.08)':'rgba(116,120,132,.12)'; const t=dark?'#c4c6d0':'#5b6070'; Object.values(S.charts).forEach(c=>{ if(!c||!c.options) return; if(c.options.scales) Object.values(c.options.scales).forEach(s=>{ if(s.ticks) s.ticks.color=t; if(s.grid) s.grid.color=g; if(s.title) s.title.color=t; }); if(c.options.plugins?.legend?.labels) c.options.plugins.legend.labels.color=t; c.update('none'); }); if(S.lastPayload) drawRadar(S.lastPayload,hardStale(S.lastPayload)); }
function togglePause(){ S.paused=!S.paused; document.getElementById('pauseIcon').textContent=S.paused?'play_arrow':'pause'; toast(S.paused?'Polling paused':'Polling resumed'); if(!S.paused) poll(); }
function switchTab(id){ document.querySelectorAll('.tab').forEach(e=>e.classList.remove('active')); document.querySelectorAll('.r-item:not(.nav-ctl)').forEach(e=>e.classList.remove('active')); document.getElementById(id)?.classList.add('active'); document.querySelector(`.r-item[data-target="${id}"]`)?.classList.add('active'); if(S.ctl.on) document.querySelector('.nav-ctl[data-view="live"]')?.classList.add('active'); Object.values(S.charts).forEach(c=>c&&c.resize()); if(S.lastPayload) drawRadar(S.lastPayload,hardStale(S.lastPayload)); if(id==='tab-snaps') renderSnaps(); }
function exportData(){ if(!S.lastPayload) return toast('No data to export'); const d="data:text/json;charset=utf-8,"+encodeURIComponent(JSON.stringify(S.lastPayload,null,2)); const a=document.createElement('a'); a.href=d; a.download=`radar_payload_${Date.now()}.json`; document.body.appendChild(a); a.click(); a.remove(); toast('Exported JSON payload'); }

function toast(m,i='check_circle'){ const h=document.getElementById('toasts'); const e=document.createElement('div'); e.className='toast'; e.innerHTML=`<span class="material-symbols-rounded" style="font-size:18px">${safeIcon(i)}</span>${esc(m)}`; h.appendChild(e); setTimeout(()=>e.remove(),3500); }

function helpLookup(key){
  const h=S.ctl.help||S.help||{};
  if(h.tooltips && h.tooltips[key]) return h.tooltips[key];
  const g=Array.isArray(h.glossary)?h.glossary.find(x=>x.id===key):null;
  if(g) return S.ctl.helpAdvanced ? (g.advanced||g.long||g.short) : (g.short||g.long||g.advanced);
  return '';
}
function installHelpTips(){
  if(document.getElementById('tipPop')) return;
  const pop=document.createElement('div'); pop.id='tipPop'; pop.className='tip-pop'; document.body.appendChild(pop);
  const move=(ev)=>{ if(pop.style.display==='block'){ const x=Math.min(window.innerWidth-340,ev.clientX+14); const y=Math.min(window.innerHeight-120,ev.clientY+14); pop.style.left=Math.max(8,x)+'px'; pop.style.top=Math.max(8,y)+'px'; } };
  const show=(el,ev)=>{ const key=el?.getAttribute('data-help'); const txt=key?helpLookup(key):''; if(!txt) return; pop.textContent=txt; pop.style.display='block'; if(ev) move(ev); };
  const hide=()=>{ pop.style.display='none'; };
  document.addEventListener('mouseover',ev=>{ const el=ev.target.closest?.('[data-help]'); if(el) show(el,ev); });
  document.addEventListener('mousemove',move);
  document.addEventListener('mouseout',ev=>{ if(ev.target.closest?.('[data-help]')) hide(); });
  document.addEventListener('focusin',ev=>{ const el=ev.target.closest?.('[data-help]'); if(el) show(el,{clientX:el.getBoundingClientRect().left,clientY:el.getBoundingClientRect().bottom}); });
  document.addEventListener('focusout',ev=>{ if(ev.target.closest?.('[data-help]')) hide(); });
}
// ─── BLE Validation ────────────────────────────────────────────────
function bleValidateMac(addr){
  if(!addr||typeof addr!=='string') return {ok:false,msg:'empty'};
  const s=addr.trim();
  if(/^[0-9A-Fa-f]{2}(:[0-9A-Fa-f]{2}){5}$/.test(s)) return {ok:true,msg:'valid MAC'};
  if(/^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}$/i.test(s)) return {ok:true,msg:'valid UUID'};
  if(s.length>=12) return {ok:false,msg:'unusual format'};
  return {ok:false,msg:'too short'};
}
// ─── Setup Persistence ─────────────────────────────────────────────
function loadSetupPrefs(){
  try{
    const p=JSON.parse(localStorage.getItem('rvt-setup-prefs')||'{}');
    for(const k of ['radar_port','ble_address','ble_profile','notify_char','subject_label','operator_label']){
      if(p[k]) S.setup[k]=p[k];
    }
    if(p.duration_s) S.setup.duration_s=p.duration_s;
  }catch(_){}
}
function persistSetupPrefs(){
  try{ localStorage.setItem('rvt-setup-prefs',JSON.stringify({radar_port:S.setup.radar_port,ble_address:S.setup.ble_address,ble_profile:S.setup.ble_profile,notify_char:S.setup.notify_char,subject_label:S.setup.subject_label,operator_label:S.setup.operator_label,duration_s:S.setup.duration_s})); }catch(_){}
}
function updateRailOperator(){
  const cur=S.ctl.current;
  const name=(cur?.params?.operator_label||cur?.operator_label||cur?.operator||S.setup.operator_label||'Operator A').trim()||'Operator A';
  const initials=((name.match(/[A-Za-z0-9]+/g)||[]).slice(0,2).map(w=>w[0]).join('')||'OA').toUpperCase();
  const role=cur?.session_id?`Session ${cur.session_id}`:'Lab · Station 3';
  const avatar=document.getElementById('opAvatar'); if(avatar) avatar.textContent=initials.slice(0,2);
  const opName=document.getElementById('opName'); if(opName) opName.textContent=name;
  const opRole=document.getElementById('opRole'); if(opRole) opRole.textContent=role;
}
function durationGuidance(s){
  const n=Number(s); if(!n||n<=0) return '';
  if(n<30) return 'Very short — warmup alone is 10-15 s. Results may be deferred.';
  if(n<60) return 'Quick check — enough for settling but not full-lock coverage.';
  if(n<=300) return 'Standard — good for training runs and regression checks.';
  if(n<=600) return 'Extended — improves locked coverage and HR/RR statistics.';
  return 'Long session — strong ML-readiness signal if reference is stable.';
}
function copySessionId(){
  const sid=S.ctl.current?.session_id||S.ctl.lastSessionId; if(!sid) return;
  navigator.clipboard?.writeText(sid).then(()=>toast(`Copied ${sid}`,'content_copy')).catch(()=>{});
}
// ─── Sandbox Fallback System ───────────────────────────────────────
function sandboxStoreKey(){ return 'rvt-sandbox-sessions'; }
function sandboxApiPath(u){ const raw=String(u||''); if(raw.startsWith('/api/')) return raw; try{ const parsed=new URL(raw,location.href); if(location.protocol==='file:' && parsed.pathname.match(/^[\/][A-Za-z]:[\/]api[\/]/)) return parsed.pathname.replace(/^[\/][A-Za-z]:/,'').replace(/\\/g,'/'); return parsed.pathname; }catch(_){ return raw; } }
function sandboxId(){ return `sandbox_${new Date().toISOString().replace(/[-:T]/g,'').slice(0,15)}`; }
function sandboxSummary(item){
  item=item||{};
  return {
    session_id:item.session_id||'sandbox',started_at:item.started_at||new Date().toISOString(),duration_s:item.duration_s||30,subject_label:item.subject||'sandbox-subject',operator_label:item.operator||'sandbox-operator',verdict:item.verdict||'ready',status:'complete',trainer_version:'sandbox',dashboard_version:'v11.0-sandbox',
    ml_readiness_verdict:{verdict:item.verdict||'ready',readiness_kind:item.verdict||'ready',headline:`Session ${item.session_id||'sandbox'} verdict`,limitation_kind:'none',next_action:'review',passed:['Sandbox telemetry flowing','Schema fields present'],failed:[],categories:[{id:'sandbox',status:'pass',label:'Sandbox',detail:'Generated data; not a real measurement.'}]},
    hr_metrics:{rmse:2.14,mae:1.68,bias:0.42,r:0.962,coverage_pct:94.2,n:280},
    rr_metrics:{rmse:0.82,mae:0.61,bias:-0.18,r:0.943,coverage_pct:91.7,n:275},
    signal_quality:{pqi_lock_pct:88.4,coverage_locked:78,coverage_settling:22,session_quality_score:0.91,internal_consistency_score:0.87},
    reference_quality:{status:'good',raw_packets:340,parsed_rows:320,distilled_rows_pct_of_raw:94.1,packet_loss_pct:2.3,decode_error_pct:0.2,pi_median:3.1,pi_below_threshold_pct:1.8,pi_threshold:0.5,coverage_pct:94.1},
    gates:{primary:{status:'PASS',passed:true,r:0.962,rmse:2.14,n:310},secondary:{status:'PASS',passed:true,r:0.95,rmse:2.4,n:295},combined:{status:'PASS',passed:true,r:0.958,rmse:2.22,n:308},locked:{status:'PASS',passed:true,r:0.973,rmse:1.86,n:240},settling:{status:'PASS',passed:true,r:0.91,rmse:2.82,n:68},golden_check:{status:'PASS',passed:true,r:0.97,rmse:1.9,n:180}},
    histograms:{top_hr_gate:{pass:310,low_pqi:8},top_rr_gate:{pass:305,outlier:12},top_hr_publish:{published:320,STALE_FROZEN:4},top_rr_publish:{published:318,LATCH_STALE:3}},
    truthfulness:{sketch_fw:'v15.1.0',module_fw:'2.1.0',module_version_valid:true,contract_length:219,schema_hash:'sandbox-v15.1',scoring_weights_hash:'sandbox-weights'},
    downloads:[{label:'analyse_summary.json',href:'#sandbox-summary'},{label:'live_dashboard.json',href:'#sandbox-live'}],
    analysis:{
      raw_hr_correction_coverage_pct:95.1,raw_hr_corrected_bias_bpm:.6,raw_hr_uncorrected_bias_bpm:2.4,hr_rescue_publish_count:22,hr_rescue_publish_coverage_pct:8.1,
      oracle_audit:{hr_oracle_candidate_mae_bpm:1.8,hr_oracle_candidate_coverage_pct:82.4,rr_oracle_candidate_mae_bpm:.7,rr_oracle_candidate_coverage_pct:88.1,hr_best_candidate_source_histogram:{reported_hr:180,candidate_hr:90,raw_hr_corrected:120},rr_best_candidate_source_histogram:{reported_rr:210,candidate_rr:60,raw_rr_effective:40}},
      coverage_loss_ledger:{hr_coverage_loss_by_stage:{NO_PHASE:84,LOW_PQI:12},rr_coverage_loss_by_stage:{LOW_PQI:18,PUBLISH_POLICY:4},hr_salvageable_frames_pct:68.2,rr_salvageable_frames_pct:74.9}
    }
  };
}
function sandboxLoadSessions(){
  if(Array.isArray(S.ctl.sandboxItems)) return S.ctl.sandboxItems;
  let items=[];
  try{ items=JSON.parse(localStorage.getItem(sandboxStoreKey())||'[]'); }catch(_){ items=[]; }
  if(!Array.isArray(items)||!items.length){
    const iso=mins=>new Date(Date.now()-mins*60000).toISOString();
    items=[
      {session_id:'sandbox_20260420_091800',started_at:iso(64),duration_s:480,subject:'demo-A',operator:'codex',verdict:'ready'},
      {session_id:'sandbox_20260419_141803',started_at:iso(1240),duration_s:300,subject:'demo-B',operator:'codex',verdict:'conditional'}
    ];
    items=items.map(i=>({...i,summary:sandboxSummary(i)}));
  }
  S.ctl.sandboxItems=items;
  try{ localStorage.setItem(sandboxStoreKey(),JSON.stringify(items)); }catch(_){}
  return items;
}
function sandboxSaveSessions(items){
  S.ctl.sandboxItems=items;
  try{ localStorage.setItem(sandboxStoreKey(),JSON.stringify(items)); }catch(_){}
}
function sandboxDefaults(){
  return {radar_port:S.setup.radar_port||'COM10',ble_address:S.setup.ble_address||'10:22:33:9E:8F:63',ble_profile:S.setup.ble_profile||'ailink_oximeter',notify_char:S.setup.notify_char,durations_s:[30,60,300,480,1200],sessions_root:'local sandbox storage',sandbox:true};
}
function sandboxHelpSchema(){
  return {
    faq:[
      {q:'Why am I in sandbox mode?',a:'The trainer API did not answer /api/status, so the dashboard enabled a local functional preview. Real hardware is not touched.'},
      {q:'Can I still test Start and Stop?',a:'Yes. Sandbox Start creates a local session, drives live telemetry, and Stop creates a report and history row.'},
      {q:'What changes when the backend is running?',a:'The same UI calls the real /api endpoints automatically; the sandbox fallback is bypassed.'}
    ],
    glossary:[
      {id:'preflight',term:'Preflight',category:'setup',short:'A required/advisory check set before session start.',advanced:'Structural failures block Start. Probe warnings guide hardware cleanup but do not erase the workflow.'},
      {id:'stale',term:'Stale telemetry',category:'live',short:'The payload age exceeded the freshness threshold.',advanced:'Stale mode blanks live publish values and surfaces recovery actions so old measurements are not trusted.'},
      {id:'fw_truthfulness',term:'Firmware truthfulness',category:'audit',short:'A contract check between observed firmware/schema and expected analysis fields.'}
    ],
    dsp_steps:[
      {n:1,title:'Acquire',summary:'Read radar and BLE frames.',detail:'The trainer aligns radar publishes with BLE reference rows and records policy gates.'},
      {n:2,title:'Validate',summary:'Reject stale, low confidence, or motion-contaminated frames.',detail:'The dashboard preserves gate reasons so the operator sees why HR/RR did not publish.'},
      {n:3,title:'Report',summary:'Compute readiness and export artifacts.',detail:'Report summaries include metric gates, reference quality, truthfulness, and remediation.'}
    ],
    troubleshooting:[
      {id:'ble',title:'BLE device not found',steps:['Confirm oximeter is on and advertising.','Check MAC or UUID format.','Run Test link again from setup.']},
      {id:'placement',title:'Weak radar placement',steps:['Move subject to 40-80 cm.','Aim at the upper torso.','Clear nearby reflectors and ask subject to sit still.']},
      {id:'stale',title:'Telemetry stale',steps:['Confirm trainer is running.','Check serial port.','Use demo/sandbox only for UI review, not measurement decisions.']}
    ],
    tooltips:{
      'preflight.python_env':'Confirms the Python environment or local sandbox runtime is ready.',
      'preflight.serial_port_probe':'Checks whether the selected radar port can be opened.',
      'preflight.ble_device_probe':'Checks the BLE reference endpoint or simulates it in sandbox mode.',
      'fw_truthfulness':'Shows firmware, module, and schema identity used by the analysis contract.',
      'metric.rmse':'Root mean square error against the reference signal.',
      'metric.r':'Correlation against the reference signal.',
      'reference.coverage':'How much of the session had usable BLE reference rows.',
      'hist.gate':'Frames blocked before publish, grouped by reason.',
      'hist.publish':'Publish policy reasons visible for audit.'
    }
  };
}
function sandboxBody(opts){
  if(!opts||opts.body===undefined) return {};
  if(typeof opts.body==='string'){ try{return JSON.parse(opts.body);}catch(_){return {}; } }
  return opts.body||{};
}
function sandboxError(status,message,body){
  const e=new Error(message); e.status=status; e.body=body||{error:{message}}; throw e;
}
function sandboxPreflight(url){
  const u=new URL(url,location.href);
  const ids=(u.searchParams.get('include')||allCheckIds().join(',')).split(',').map(x=>x.trim()).filter(Boolean);
  const details={
    python_env:'Browser sandbox runtime ready.',
    firmware_file_present:'Firmware contract fixture loaded for UI review.',
    serial_port_list:"ports=['COM10','COM11','COM12']",
    session_folder_writable:'Session history writes to localStorage in sandbox mode.',
    disk_space:'Browser storage is available for sample sessions.',
    schema_hash_consistency:'Expected v11.0/v15.1 schema hash present.',
    clock_monotonic_sanity:'Local clock appears monotonic for this preview.',
    ble_adapter:'Sandbox adapter active; real BLE is not touched.',
    serial_port_probe:`Simulated radar probe succeeded for ${S.setup.radar_port||'COM10'}.`,
    ble_device_probe:bleValidateMac(S.setup.ble_address).ok?`Simulated BLE link accepted ${S.setup.ble_address}.`:'BLE address format is unusual; simulated probe allowed with warning.'
  };
  const checks=ids.map(id=>{
    const warn=id==='ble_device_probe'&&!bleValidateMac(S.setup.ble_address).ok;
    return {id,label:checkLabel(id),status:warn?'warn':'ok',detail:details[id]||'Sandbox check complete.',remediation:warn?'Enter a MAC address like AA:BB:CC:DD:EE:FF, then test again.':''};
  });
  const summary={ok:checks.filter(c=>c.status==='ok').length,warn:checks.filter(c=>c.status==='warn').length,fail:checks.filter(c=>c.status==='fail').length};
  return {summary,checks,sandbox:true,generated_at:new Date().toISOString()};
}
function sandboxCurrent(){
  const cur=S.ctl.current;
  if(!cur) sandboxError(404,'No active sandbox session');
  const elapsed=Math.max(0,(Date.now()-cur.started_ms)/1000);
  const dur=Number(cur.duration_s)||Number(cur.params?.duration_s)||30;
  return {...cur,elapsed_s:elapsed,remaining_s:Math.max(0,dur-elapsed),duration_s:dur,preview:{elapsed_s:elapsed},status:'running',sandbox:true};
}
function sandboxLivePayload(){
  const p=demoPayload();
  p.meta.sandbox=true;
  p.ble.address=S.setup.ble_address;
  p.ble.profile=S.setup.ble_profile;
  if(S.ctl.current){
    const cur=sandboxCurrent();
    p.meta.status='running'; p.meta.elapsed_s=cur.elapsed_s; p.meta.remaining_s=cur.remaining_s; p.meta.session_dir=`sandbox://${cur.session_id}`;
  }else{
    p.meta.status='waiting'; p.meta.elapsed_s=0; p.meta.remaining_s=null; p.meta.note='Sandbox preview: no active recording yet.';
  }
  return p;
}
function sandboxStart(opts){
  const body=sandboxBody(opts); const duration=Number(body.duration_s)||sessionDuration()||30; const sid=sandboxId();
  S.ctl.current={session_id:sid,started_at:new Date().toISOString(),started_ms:Date.now(),duration_s:duration,params:{...body,duration_s:duration},sandbox:true};
  S.ctl.lastSessionId=sid;
  return sandboxCurrent();
}
function sandboxStop(){
  if(!S.ctl.current) sandboxError(409,'No active sandbox session');
  const cur=sandboxCurrent();
  const item={session_id:cur.session_id,started_at:cur.started_at,duration_s:cur.duration_s,subject:cur.params?.subject_label||'sandbox-subject',operator:cur.params?.operator_label||'sandbox-operator',verdict:'ready'};
  item.summary=sandboxSummary(item);
  const items=sandboxLoadSessions().filter(i=>i.session_id!==item.session_id);
  items.unshift(item); sandboxSaveSessions(items);
  S.ctl.current=null; S.ctl.lastSessionId=item.session_id;
  return {session_id:item.session_id,status:'stopped',sandbox:true};
}
async function sandboxApiJson(url,opts={}){
  const path=sandboxApiPath(url);
  if(path==='/api/status') return {ok:true,mode:'sandbox',message:'Local dashboard sandbox active'};
  if(path==='/api/defaults') return sandboxDefaults();
  if(path==='/api/preflight') return sandboxPreflight(url);
  let m=path.match(/^\/api\/preflight\/([^/]+)$/);
  if(m) return sandboxPreflight(`/api/preflight?include=${encodeURIComponent(decodeURIComponent(m[1]))}`).checks[0];
  if(path==='/api/help/schema') return sandboxHelpSchema();
  if(path==='/api/session/start') return sandboxStart(opts);
  if(path==='/api/session/current') return sandboxCurrent();
  if(path==='/api/session/stop') return sandboxStop();
  if(path==='/api/sessions') return {items:sandboxLoadSessions().map(({summary,...i})=>i),sandbox:true};
  m=path.match(/^\/api\/sessions\/([^/]+)\/summary$/);
  if(m){ const id=decodeURIComponent(m[1]); const item=sandboxLoadSessions().find(i=>i.session_id===id) || (S.ctl.current?.session_id===id?{...S.ctl.current,subject:S.ctl.current.params?.subject_label,operator:S.ctl.current.params?.operator_label,verdict:'ready'}:null); if(!item) sandboxError(404,`Sandbox session ${id} not found`); return sandboxSummary(item); }
  m=path.match(/^\/api\/sessions\/([^/]+)\/compare$/);
  if(m){ const id=decodeURIComponent(m[1]); const items=sandboxLoadSessions(); const selected=items.find(i=>i.session_id===id)||items[0]; return {selected:sandboxSummary(selected),previous:sandboxSummary(items[1]||items[0]||selected),best:sandboxSummary(items.find(i=>i.verdict==='ready')||selected),sandbox:true}; }
  m=path.match(/^\/api\/sessions\/([^/]+)\/analyse$/);
  if(m) return {pid:'sandbox',status:'queued',sandbox:true};
  sandboxError(404,`Sandbox endpoint not implemented: ${path}`);
}
function enableSandboxControlMode(reason='Trainer API unavailable'){
  S.ctl={...S.ctl,on:true,sandbox:true,status:{ok:true,mode:'sandbox',reason}};
  S.ctl.help=sandboxHelpSchema();
  sandboxLoadSessions();
  return true;
}
async function apiJson(url,opts={}){
  if(S.ctl?.sandbox && String(url).startsWith('/api/')) return sandboxApiJson(url,opts);
  const r=await fetch(url,Object.assign({cache:'no-store'},opts));
  let j=null; try{ j=await r.json(); }catch(e){}
  if(!r.ok){ const err=new Error(j?.error?.message||j?.error||`HTTP ${r.status}`); err.status=r.status; err.body=j; throw err; }
  return j;
}
async function detectControlMode(){
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    if(!r.ok) return enableSandboxControlMode(`Control API returned HTTP ${r.status}`);
    const j=await r.json();
    S.ctl={...S.ctl,on:true,sandbox:false,status:j};
    return true;
  }catch(e){ return enableSandboxControlMode(e.message||'Control API unavailable'); }
}
function ctlButton(view,icon,label,kbd){
  return `<button class="r-item nav-ctl" data-view="${view}" type="button"><span class="r-ic"><span class="material-symbols-rounded">${icon}</span></span>${label}${kbd?`<span class="nav-kbd">${kbd}</span>`:''}</button>`;
}
function setupControlMode(){
  document.body.dataset.ctl='on';
  if(!S.ctl.sandbox) S.dataUrl='/api/session/current/live_dashboard.json';
  loadSetupPrefs();
  const rail=document.querySelector('.rail'); const workflow=document.getElementById('workflowGroup');
  if(rail && workflow && !document.querySelector('.nav-ctl')){
    workflow.insertAdjacentHTML('beforeend',[
      ctlButton('home','home','Home','1'), ctlButton('live','monitoring','Live','2'), ctlButton('report','summarize','Report','3'), ctlButton('help','help','Help','4')
    ].join(''));
    rail.querySelectorAll('.nav-ctl[data-view]').forEach(b=>b.addEventListener('click',()=>switchView(b.dataset.view)));
  }
  updateRailOperator();
  const top=document.querySelector('.topbar');
  if(top && !document.getElementById('ses-bar')){
    top.insertAdjacentHTML('afterend','<div class="ses-bar" id="ses-bar" style="display:none"><span class="ses-id" id="sesId">Session --</span><button class="ses-copy-btn" onclick="copySessionId()" title="Copy session ID"><span class="material-symbols-rounded">content_copy</span></button><div class="ses-meta"><span id="sesElapsed">elapsed --</span><span id="sesRemain">remaining --</span><span id="sesLabels"></span></div><div class="ses-progress" id="sesProgress"><div class="ses-progress-fill" id="sesProgressFill"></div></div><span class="ses-spacer"></span><button id="sesStopBtn" type="button" onclick="stopActiveSession()">Stop</button></div>');
  }
  const chips=document.getElementById('chips');
  if(chips && !document.getElementById('liveCoach')){
    chips.insertAdjacentHTML('afterend','<div class="coach-grid" id="liveCoach" style="display:none"></div>');
  }
  const fw=document.getElementById('fwBadge');
  if(fw && !document.getElementById('view-home')) fw.insertAdjacentHTML('beforebegin',controlViewsHtml());
  renderPreflight();
  installHelpTips();
  window.addEventListener('hashchange',routeHash);
  routeHash();
  loadControlDefaults().then(()=>Promise.allSettled([loadSessionsList(), runPreflightBatch(), renderHelpView()])).finally(()=>routeHash());
  refreshSessionHeader();
}
function controlViewsHtml(){
  const sb=S.ctl.sandbox;
  const modeNote=sb
    ? '<div class="sandbox-note"><strong>Sandbox note</strong>The trainer API was unreachable, so the dashboard is running in a local functional preview. All Start / Stop / Report actions are simulated in-browser. Data is stored in localStorage.</div>'
    : '<div class="sandbox-note connected"><strong>Connected server mode</strong>The trainer API is reachable. Start, Stop, preflight, reports, and live telemetry are backed by the Python control server.</div>';
  return `
  <section id="view-home">
    <div class="welcome-hero">
      <div class="eyebrow">${sb?'Sandbox mode — local preview':'Connected server mode — live trainer'}</div>
      <h1 class="hero-title">Radar Vital Trainer</h1>
      <p class="hero-subtitle">Configure hardware, run preflight checks, record sessions, and review ML-readiness verdicts — all from one dashboard-first console.</p>
      ${modeNote}
    </div>
    <div class="home-layout">
      <div class="page-container">
        <article class="card" style="margin-bottom:var(--gap)" id="homeWelcome">
          <div class="ch"><div class="tw"><span class="material-symbols-rounded">fiber_manual_record</span><div><h2 class="ct">Record a new session</h2><div class="cs">Dashboard-first setup with truthful preflight checks before collection.</div></div></div></div>
          <div class="chips" id="lastSessionChip" style="margin-bottom:14px"></div>
          <div class="ca"><button class="fab" type="button" onclick="document.getElementById('setupCard')?.scrollIntoView({behavior:'smooth'})"><span class="material-symbols-rounded">play_arrow</span>Start new session</button><button class="chip-btn" type="button" onclick="switchView('live')">Open live dashboard</button><button class="chip-btn" type="button" onclick="switchView('help')">Open help</button></div>
        </article>
        <article class="card" style="margin-bottom:var(--gap)" id="setupCard">
          <div class="ch"><div class="tw"><span class="material-symbols-rounded">tune</span><div><h2 class="ct">Session setup</h2><div class="cs">Defaults use ${esc(S.setup.radar_port)} and ${esc(S.setup.ble_address)} unless changed.</div></div></div></div>
          <div class="setup">
            <div><div class="m-l" style="margin-bottom:8px">Duration</div><div class="chip-group" id="durationChips"></div><div class="setup-help" id="durationHint"></div></div>
            <div class="setup-row"><label for="radarPort">Radar port</label><select id="radarPort" onchange="updateSetupFromForm(true)"></select></div>
            <div class="setup-row"><label for="bleAddress">BLE address</label><div style="display:flex;gap:8px;flex-wrap:wrap"><input id="bleAddress" type="text" style="min-width:250px" oninput="updateSetupFromForm(true)"><button class="chip-btn" type="button" onclick="runOnePreflight('ble_device_probe')">Test</button><span id="bleInline" class="cs"></span></div></div>
            <div class="setup-row"><label for="subjectLabel">Subject</label><input id="subjectLabel" type="text" oninput="updateSetupFromForm(false)" placeholder="optional"></div>
            <div class="setup-row"><label for="operatorLabel">Operator</label><input id="operatorLabel" type="text" oninput="updateSetupFromForm(false)" placeholder="optional"></div>
            <details class="advanced-section"><summary>Advanced</summary><div class="adv-grid"><div class="setup-row"><label for="notifyChar">Notify UUID</label><input id="notifyChar" type="text" oninput="updateSetupFromForm(false)"></div><div class="setup-row"><label for="bleProfile">BLE profile</label><input id="bleProfile" type="text" oninput="updateSetupFromForm(false)"></div><div class="setup-row"><label for="skipCountdown">Stillness countdown</label><label class="cs"><input id="skipCountdown" type="checkbox" onchange="updateSetupFromForm(false)"> Skip 3-second countdown</label></div><div class="setup-row"><span>Sessions root</span><span class="cs" id="sessionsRoot">sessions</span></div></div></details>
            <div id="startAlert" class="setup-alert" style="display:none"></div>
            <button class="setup-start" id="startBtn" type="button" onclick="startSession()">Start session</button>
          </div>
        </article>
        <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">history</span><div><h2 class="ct">Past sessions</h2><div class="cs">Open a previous report summary.</div></div></div></div><div class="sess-filter"><input class="help-search" type="search" id="sessSearch" placeholder="Filter sessions" oninput="renderSessionsFilter()" style="min-width:200px;max-width:340px"><span class="count" id="sessCount"></span></div><div id="sessionsList"></div></article>
      </div>
      <div>
        <article class="sys-card sys-preflight" style="margin-bottom:var(--gap)">
          <div class="card-header space-between"><div class="title-with-icon"><span class="material-symbols-rounded" style="font-size:23px;color:#fff;background:linear-gradient(135deg,var(--brand-500),var(--brand-400));padding:10px;border-radius:var(--r-md)">fact_check</span><h3>Hardware quickcheck</h3></div><button class="ghost-btn" type="button" onclick="runPreflightBatch()">Re-run all</button></div>
          <div class="pf-grid" id="pfGrid"></div>
          <div class="chips" id="pfSummary" style="margin-top:12px"></div>
        </article>
      </div>
    </div>
  </section>
  <section id="view-report"><article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">summarize</span><div><h2 class="ct">Report</h2><div class="cs" id="reportSub">Select a session from Home.</div></div></div></div><div id="reportBody">No session selected.</div></article></section>
  <section id="view-help"><article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">help</span><div><h2 class="ct">Help</h2><div class="cs">FAQ, glossary, DSP notes, and troubleshooting from the trainer.</div></div></div></div><div class="help-toolbar"><input id="helpSearch" class="help-search" type="search" placeholder="Search help" oninput="S.ctl.helpQuery=this.value;renderHelpView()"><button class="chip-btn" id="helpModeBtn" type="button" onclick="toggleHelpAdvanced()">Beginner</button></div><div id="helpBody">Loading help...</div></article></section>`;
}
function switchView(id,push=true){
  if(!S.ctl.on) return;
  const view=['home','live','report','help'].includes(id)?id:'home';
  document.body.dataset.view=view;
  document.querySelectorAll('.nav-ctl[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===view));
  if(view==='live') document.querySelector('.nav-ctl[data-view="live"]')?.classList.add('active');
  if(push){
    const h=view==='report'&&S.ctl.lastSessionId?`#/report/${S.ctl.lastSessionId}`:`#/${view}`;
    if(location.hash!==h) history.replaceState(null,'',h);
  }
  if(view==='help') renderHelpView();
  if(view==='report') renderReportView(S.ctl.lastSessionId);
  Object.values(S.charts).forEach(c=>c&&c.resize());
}
function routeHash(){
  if(!S.ctl.on) return;
  const parts=location.hash.replace(/^#\/?/,'').split('/').filter(Boolean);
  if(!parts.length){ switchView('home',false); return; }
  if(parts[0]==='live'){ switchView('live',false); if(parts[1]) switchTab(parts[1].startsWith('tab-')?parts[1]:`tab-${parts[1]}`); return; }
  if(parts[0]==='report'){ S.ctl.lastSessionId=parts[1]||S.ctl.lastSessionId; switchView('report',false); renderReportView(S.ctl.lastSessionId); return; }
  switchView(parts[0],false);
}
async function loadControlDefaults(){
  try{
    const d=await apiJson('/api/defaults'); S.ctl.defaults=d;
    S.setup.radar_port=d.radar_port||S.setup.radar_port; S.setup.ble_address=d.ble_address||S.setup.ble_address; S.setup.ble_profile=d.ble_profile||S.setup.ble_profile; S.setup.notify_char=d.notify_char||S.setup.notify_char; S.setup.duration_s=(Array.isArray(d.durations_s)&&d.durations_s[0])||30;
    renderSetupForm();
  }catch(e){ toast(`Defaults unavailable: ${e.message}`,'warning'); renderSetupForm(); }
}
function durationLabel(s){ const n=Number(s); if(n===30) return '30 s'; if(n%60===0) return `${n/60} min`; return `${n} s`; }
function renderSetupForm(){
  const ds=(S.ctl.defaults?.durations_s||[30,60,300,480,1200]).map(Number);
  const dc=document.getElementById('durationChips'); if(dc) dc.innerHTML=ds.map(s=>`<button class="chip-btn ${S.setup.duration_s===s?'active':''}" type="button" onclick="setDuration(${s})">${esc(durationLabel(s))}</button>`).join('')+`<button class="chip-btn ${S.setup.duration_s==='custom'?'active':''}" type="button" onclick="setDuration('custom')">Custom</button><span id="customDurationWrap" style="${S.setup.duration_s==='custom'?'':'display:none'}"><input id="customDuration" type="number" min="1" value="${esc(S.setup.customDuration)}" oninput="updateSetupFromForm(false)" style="width:90px"><select id="customUnit" onchange="updateSetupFromForm(false)"><option value="s">s</option><option value="min">min</option></select></span>`;
  const port=document.getElementById('radarPort'); if(port){ const ports=detectedPorts(); if(!ports.includes(S.setup.radar_port)) ports.unshift(S.setup.radar_port); port.innerHTML=ports.map(p=>`<option value="${esc(p)}">${esc(p)}</option>`).join(''); port.value=S.setup.radar_port; }
  for(const [id,key] of [['bleAddress','ble_address'],['subjectLabel','subject_label'],['operatorLabel','operator_label'],['notifyChar','notify_char'],['bleProfile','ble_profile']]){ const el=document.getElementById(id); if(el) el.value=S.setup[key]||''; }
  const skip=document.getElementById('skipCountdown'); if(skip) skip.checked=!!S.setup.skip_countdown;
  const root=document.getElementById('sessionsRoot'); if(root) root.textContent=S.ctl.defaults?.sessions_root||'sessions';
  updateStartGate();
  const dh=document.getElementById('durationHint'); if(dh){ const dur=sessionDuration(); dh.textContent=durationGuidance(dur); dh.className='setup-help'+(dur>=300?' ok':dur<30?' warn':''); }
  const addr=document.getElementById('bleAddress'); if(addr){ const v=bleValidateMac(addr.value); if(addr.value.trim()) addr.classList.toggle('invalid-soft',!v.ok); else addr.classList.remove('invalid-soft'); }
}
function setDuration(v){ S.setup.duration_s=v; renderSetupForm(); }
let pfTimer=null;
function updateSetupFromForm(debounce){
  const port=document.getElementById('radarPort'); const addr=document.getElementById('bleAddress');
  if(port) S.setup.radar_port=port.value||'COM10'; if(addr) S.setup.ble_address=addr.value||'10:22:33:9E:8F:63';
  for(const [id,key] of [['subjectLabel','subject_label'],['operatorLabel','operator_label'],['notifyChar','notify_char'],['bleProfile','ble_profile']]){ const el=document.getElementById(id); if(el) S.setup[key]=el.value; }
  const skip=document.getElementById('skipCountdown'); if(skip) S.setup.skip_countdown=!!skip.checked;
  const cd=document.getElementById('customDuration'); const cu=document.getElementById('customUnit'); if(cd) S.setup.customDuration=Number(cd.value)||1; if(cu) S.setup.customUnit=cu.value;
  if(debounce){ clearTimeout(pfTimer); pfTimer=setTimeout(runPreflightBatch,450); }
  persistSetupPrefs();
  updateRailOperator();
  updateStartGate();
}
function renderSessionsFilter(){
  const q=document.getElementById('sessSearch')?.value?.trim().toLowerCase()||'';
  const rows=document.querySelectorAll('#sessionsList .sess-table tbody tr');
  let shown=0;
  rows.forEach(tr=>{ const match=!q||tr.textContent.toLowerCase().includes(q); tr.style.display=match?'':'none'; if(match) shown++; });
  const ct=document.getElementById('sessCount'); if(ct) ct.textContent=`${shown} of ${rows.length}`;
}
function structuralChecks(){ return ['python_env','firmware_file_present','serial_port_list','session_folder_writable','disk_space','schema_hash_consistency','clock_monotonic_sanity','ble_adapter']; }
function allCheckIds(){ return [...structuralChecks(),'serial_port_probe','ble_device_probe']; }
function checkLabel(id){ return ({python_env:'Python environment',firmware_file_present:'Firmware file',serial_port_list:'Serial ports',session_folder_writable:'Session folder',disk_space:'Disk space',schema_hash_consistency:'Schema hash',clock_monotonic_sanity:'Clock sanity',ble_adapter:'BLE adapter',serial_port_probe:'Serial probe',ble_device_probe:'BLE device'})[id]||id; }
function statusClass(s){ return s==='ok'?'good':(s==='fail'?'bad':(s==='warn'?'warn':'')); }
function renderPreflight(){
  const checks=new Map((S.ctl.preflight?.checks||[]).map(c=>[c.id,c]));
  const html=allCheckIds().map(id=>{
    const c=checks.get(id)||{id,label:checkLabel(id),status:'skip',detail:id.endsWith('_probe')?'Not run in batch. Use Test now.':'Not run yet.',remediation:''};
    const cls=statusClass(c.status); const action=id.endsWith('_probe')?'<button type="button">Test now</button>':'<button type="button">Retry</button>';
    return `<div class="pf-tile ${c.status==='fail'?'fail':c.status==='warn'?'warn':''}" data-check="${esc(id)}" data-help="preflight.${esc(id)}" tabindex="0"><div class="pf-top"><span class="pill ${cls}">${esc(c.status||'skip')}</span><span>${esc(c.label||checkLabel(id))}</span></div><div class="pf-detail">${esc(c.detail)}</div>${c.remediation?`<div class="pf-rem">${esc(c.remediation)}</div>`:''}${action}</div>`;
  }).join('');
  const grid=document.getElementById('pfGrid'); if(grid){ grid.innerHTML=html; grid.querySelectorAll('.pf-tile button').forEach(b=>b.addEventListener('click',()=>runOnePreflight(b.closest('.pf-tile').dataset.check))); }
  const sum=S.ctl.preflight?.summary||{}; const ps=document.getElementById('pfSummary'); if(ps) ps.innerHTML=`<span class="pill good">ok ${sum.ok||0}</span><span class="pill warn">warn ${sum.warn||0}</span><span class="pill bad">fail ${sum.fail||0}</span>`;
  renderSetupForm();
}
async function runPreflightBatch(){
  if(!S.ctl.on) return;
  const include=encodeURIComponent(structuralChecks().join(','));
  try{ S.ctl.preflight=await apiJson(`/api/preflight?port=${encodeURIComponent(S.setup.radar_port)}&address=${encodeURIComponent(S.setup.ble_address)}&include=${include}`); }
  catch(e){ S.ctl.preflight={summary:{fail:1},checks:[{id:'api',label:'Preflight API',status:'fail',detail:e.message,remediation:'Confirm the control server is running.'}]}; }
  renderPreflight();
}
async function runOnePreflight(id){
  const grid=document.getElementById('pfGrid'); const tile=grid?.querySelector(`[data-check="${CSS.escape(id)}"]`); if(tile) tile.querySelector('.pf-detail').textContent='Running...';
  try{
    const c=await apiJson(`/api/preflight/${encodeURIComponent(id)}`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({port:S.setup.radar_port,address:S.setup.ble_address})});
    const checks=(S.ctl.preflight?.checks||[]).filter(x=>x.id!==id); checks.push(c); S.ctl.preflight={summary:S.ctl.preflight?.summary||{},checks};
    if(id==='ble_device_probe'){ const el=document.getElementById('bleInline'); if(el) el.textContent=c.detail||''; }
  }catch(e){
    const checks=(S.ctl.preflight?.checks||[]).filter(x=>x.id!==id); checks.push({id,label:checkLabel(id),status:'fail',detail:e.status===409?'Stopped - a session is running':e.message,remediation:e.body?.error?.message||''}); S.ctl.preflight={summary:S.ctl.preflight?.summary||{},checks};
  }
  renderPreflight();
}
function detectedPorts(){
  const c=(S.ctl.preflight?.checks||[]).find(x=>x.id==='serial_port_list'); const d=String(c?.detail||'');
  const m=d.match(/ports=\[(.*)\]/); if(!m) return [];
  return m[1].split(',').map(s=>s.replace(/['"]/g,'').trim()).filter(Boolean);
}
function structuralFailures(){ const checks=S.ctl.preflight?.checks||[]; const ids=new Set(structuralChecks()); return checks.filter(c=>ids.has(c.id)&&c.status==='fail'); }
function updateStartGate(){
  const btn=document.getElementById('startBtn'); if(!btn) return;
  const fails=structuralFailures(); btn.disabled=!!S.ctl.current||fails.length>0;
  const alert=document.getElementById('startAlert');
  if(alert && fails.length){ alert.style.display='block'; alert.innerHTML=fails.map(f=>`${esc(f.label||f.id)}: ${esc(f.remediation||f.detail)}`).join('<br>'); }
  else if(alert && !btn.dataset.serverError){ alert.style.display='none'; alert.textContent=''; }
}
function sessionDuration(){
  if(S.setup.duration_s!=='custom') return Number(S.setup.duration_s)||30;
  return Math.max(1,Number(S.setup.customDuration)||1)*(S.setup.customUnit==='min'?60:1);
}
function sleep(ms){ return new Promise(resolve=>setTimeout(resolve,ms)); }
async function stillnessCountdown(){
  if(S.setup.skip_countdown) return;
  const alert=document.getElementById('startAlert');
  if(!alert) return;
  alert.dataset.serverError='';
  alert.style.display='block';
  for(let n=3;n>=1;n--){
    alert.textContent=`Stillness countdown: ${n}. Sit still, oximeter on finger, radar aimed at the chest.`;
    await sleep(1000);
  }
  alert.textContent='Starting session...';
}
async function startSession(){
  updateSetupFromForm(false);
  const btn=document.getElementById('startBtn'); const alert=document.getElementById('startAlert'); if(btn) btn.disabled=true;
  if(alert){ alert.dataset.serverError=''; alert.style.display='none'; alert.textContent=''; }
  try{
    await stillnessCountdown();
    const body={duration_s:sessionDuration(),radar_port:S.setup.radar_port,ble_address:S.setup.ble_address,ble_profile:S.setup.ble_profile,subject_label:S.setup.subject_label,operator_label:S.setup.operator_label,advanced:{notify_char:S.setup.notify_char}};
    const r=await apiJson('/api/session/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    S.ctl.current=r; toast(`Started ${r.session_id}`,'play_arrow'); switchView('live'); refreshSessionHeader();
  }catch(e){
    const failed=e.body?.error?.failed||[]; if(alert){ alert.dataset.serverError='1'; alert.style.display='block'; alert.innerHTML=failed.length?failed.map(f=>`${esc(f.label||f.id)}: ${esc(f.remediation||f.detail)}`).join('<br>'):esc(e.message); }
    if(failed.length){ const checks=(S.ctl.preflight?.checks||[]).filter(c=>!failed.some(f=>f.id===c.id)).concat(failed.map(f=>({...f,status:'fail'}))); S.ctl.preflight={summary:S.ctl.preflight?.summary||{},checks}; renderPreflight(); }
  }finally{ updateStartGate(); }
}
async function refreshSessionHeader(){
  if(!S.ctl.on) return;
  try{
    const cur=await apiJson('/api/session/current'); S.ctl.current=cur;
    const bar=document.getElementById('ses-bar'); if(bar) bar.style.display='flex';
    const sid=cur.session_id||'--'; document.getElementById('sesId').textContent=`Session ${sid}`;
    const elapsed=num(cur.elapsed_s??cur.preview?.elapsed_s); const remain=num(cur.remaining_s);
    document.getElementById('sesElapsed').textContent=`elapsed ${elapsed===null?'--':fmt(elapsed,0)+' s'}`;
    document.getElementById('sesRemain').textContent=`remaining ${remain===null?'--':fmt(remain,0)+' s'}`;
    document.getElementById('sesLabels').textContent=[cur.params?.subject_label,cur.params?.operator_label].filter(Boolean).join(' / ');
    const dur=num(cur.duration_s); const pct=(elapsed!==null&&dur>0)?Math.min(100,(elapsed/dur)*100):0;
    const fill=document.getElementById('sesProgressFill'); if(fill) fill.style.width=pct+'%';
    const prog=document.getElementById('sesProgress'); if(prog){ prog.dataset.manual=dur<=0?'1':'0'; prog.dataset.almost=pct>=90?'1':'0'; }
    updateRailOperator();
  }catch(e){
    const was=S.ctl.current; S.ctl.current=null; const bar=document.getElementById('ses-bar'); if(bar) bar.style.display='none';
    updateRailOperator();
    if(S.ctl.stopPending && was?.session_id){ S.ctl.stopPending=false; S.ctl.lastSessionId=was.session_id; switchView('report'); renderReportView(was.session_id); }
  }
  updateStartGate();
}
async function stopActiveSession(){
  const btn=document.getElementById('sesStopBtn'); if(btn) btn.disabled=true; S.ctl.stopPending=true;
  try{ const r=await apiJson('/api/session/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({reason:'user_request'})}); if(r.session_id) S.ctl.lastSessionId=r.session_id; toast('Stop requested','stop_circle'); }
  catch(e){ S.ctl.stopPending=false; toast(e.message,'warning'); }
  finally{ if(btn) btn.disabled=false; refreshSessionHeader(); }
}
async function loadSessionsList(){
  try{
    const s=await apiJson('/api/sessions'); const items=Array.isArray(s.items)?s.items:[];
    items.sort((a,b)=>String(b.started_at||'').localeCompare(String(a.started_at||'')));
    const chip=document.getElementById('lastSessionChip'); if(chip){ const last=items[0]; chip.innerHTML=last?`<button class="chip good" type="button" onclick="openReport('${esc(last.session_id)}')">Last session: ${esc(last.session_id)} • ${esc(durationLabel(last.duration_s||0))} • ${esc(last.verdict||'unknown')}</button>`:'<span class="chip">No past sessions yet</span>'; }
    const el=document.getElementById('sessionsList'); if(el) el.innerHTML=items.length?`<table class="sess-table"><thead><tr><th>id</th><th>started</th><th>duration</th><th>subject</th><th>verdict</th></tr></thead><tbody>${items.map(i=>`<tr onclick="openReport('${esc(i.session_id)}')"><td>${esc(i.session_id)}</td><td>${esc(i.started_at)}</td><td>${esc(durationLabel(i.duration_s||0))}</td><td>${esc(i.subject||'')}</td><td><span class="pill ${i.verdict==='ready'?'good':i.verdict==='not_ready'?'bad':'warn'}">${esc(i.verdict||'unknown')}</span></td></tr>`).join('')}</tbody></table>`:'No sessions found.';
  }catch(e){ const el=document.getElementById('sessionsList'); if(el) el.textContent=e.message; }
}
function openReport(id){ S.ctl.lastSessionId=id; location.hash=`#/report/${id}`; }
async function renderReportView(id){
  const body=document.getElementById('reportBody'); if(!body) return;
  if(!id){ body.textContent='No session selected.'; return; }
  body.textContent='Loading report...';
  try{
    const s=await apiJson(`/api/sessions/${encodeURIComponent(id)}/summary`);
    renderReportSummary(s,id,body);
  }catch(e){ body.textContent=e.message; }
}
function reportPillLegacy(v){ const c=v==='ready'?'good':(v==='not_ready'?'bad':(v==='conditional'?'warn':'')); return `<span class="pill ${c}">${esc(v||'not computed')}</span>`; }
function reportMetric(label,value,unit='',d=1){ return `<div class="m"><span class="m-l">${esc(label)}</span><div class="m-vr"><span class="m-v">${esc(value===null||value===undefined||value===''?'--':(typeof value==='number'?fmt(value,d):value))}</span>${unit?`<span class="m-u">${esc(unit)}</span>`:''}</div></div>`; }
function metricRows(prefix,m){ m=m||{}; return [
  row(`${prefix} RMSE`,fmt(m.rmse,2)), row(`${prefix} MAE`,fmt(m.mae,2)), row(`${prefix} bias`,fmt(m.bias,2)),
  row(`${prefix} r`,fmt(m.r,3)), row(`${prefix} coverage`,`${fmt(m.coverage_pct,1)}%`), row(`${prefix} n`,textOrDash(m.n))
].join(''); }
function histBars(items){
  if(!items) return '<div class="cs">No histogram entries.</div>';
  const list=Array.isArray(items)?items:(typeof items==='object'?Object.entries(items).map(([reason,count])=>({reason,count})):[]);
  if(!list.length) return '<div class="cs">No histogram entries.</div>';
  const max=Math.max(...list.map(i=>Number(i.count??i.value??0)||0),1);
  return `<div class="mini-bars">${list.map(i=>`<div class="mini-bar"><span>${esc(i.reason??i.label??i.stage??i.kind??'unknown')}</span><span class="mini-bar-track"><span class="mini-bar-fill" style="width:${Math.max(2,(Number(i.count??i.value??0)||0)/max*100)}%"></span></span><span>${esc(i.count??i.value??0)}</span></div>`).join('')}</div>`;
}
function readinessMeta(kind){
  const k=String(kind||'').trim().toLowerCase();
  const map={
    ready:{label:'ready',icon:'verified',cls:'ready'},
    degraded_signal:{label:'degraded signal',icon:'report',cls:'degraded_signal'},
    provenance_warning:{label:'provenance warning',icon:'gpp_maybe',cls:'provenance_warning'},
    schema_warning:{label:'schema warning',icon:'rule',cls:'schema_warning'},
    firmware_rejected:{label:'firmware rejected',icon:'error',cls:'firmware_rejected'},
    deferred:{label:'deferred',icon:'schedule',cls:'deferred'},
    not_ready:{label:'not ready',icon:'error',cls:'firmware_rejected'},
    conditional:{label:'conditional',icon:'warning',cls:'degraded_signal'}
  };
  return map[k]||{label:k||'unknown',icon:'help',cls:'degraded_signal'};
}
function verdictKindClass(v){ return readinessMeta(v?.readiness_kind||v?.verdict||v).cls; }
function reportPill(v){
  const meta=readinessMeta(v?.readiness_kind||v?.verdict||v);
  return `<span class="pill ${meta.cls}"><span class="material-symbols-rounded">${meta.icon}</span>${esc(meta.label)}</span>`;
}
function verdictActionHtml(v){
  v=v||{}; const kind=String(v.limitation_kind||'').toLowerCase(); const next=String(v.next_action||'').toLowerCase();
  const readiness=String(v.readiness_kind||v.verdict||'').toLowerCase();
  if(readiness==='ready') return `<div class="verdict-actions"><a class="chip" href="/api/report/export?session=${encodeURIComponent(S.ctl.lastSessionId||'')}">Export thesis HTML</a></div>`;
  if(readiness==='deferred') return '<div class="verdict-actions"><button class="chip-btn" type="button" onclick="prepareNextSession(1200)">Start a longer session</button></div>';
  if(kind==='data'||next.includes('longer')||next.includes('20 min')) return '<div class="verdict-actions"><button class="chip-btn" type="button" onclick="prepareNextSession(1200)">Start a longer session</button></div>';
  if(kind==='hygiene') return `<div class="verdict-actions"><button class="chip-btn" type="button" onclick="switchView('live')">Open live coach</button><button class="chip-btn" type="button" onclick="location.hash='#/help';S.ctl.helpQuery='radar placement';renderHelpView()">Placement help</button></div>`;
  if(kind==='reference') return `<div class="verdict-actions"><button class="chip-btn" type="button" onclick="location.hash='#/help';S.ctl.helpQuery='BLE device';renderHelpView()">BLE troubleshooting</button></div>`;
  if(kind==='policy') return '<div class="verdict-actions"><button class="chip-btn" type="button" onclick="prepareNextSession(1200)">Extend warmup/run time</button></div>';
  return `<div class="verdict-actions"><button class="chip-btn" type="button" onclick="switchView('home')">Start a new session</button></div>`;
}
function prepareNextSession(seconds){
  S.setup.duration_s=Number(seconds)||1200;
  switchView('home');
  renderSetupForm();
  setTimeout(()=>document.getElementById('setupCard')?.scrollIntoView({behavior:'smooth'}),50);
}
function contractDiagnosisHtml(d){
  d=d||{}; const bad=d.status==='mismatch'; const unknown=d.status==='unknown';
  const rows=[row('Expected columns',d.expected_contract_length),row('Observed columns',d.observed_contract_length),row('Expected FW',d.expected_firmware),row('Observed FW',d.observed_firmware),row('Expected schema',shortHash(d.expected_schema_hash)),row('Observed schema',shortHash(d.observed_schema_hash))].join('');
  const issues=(d.mismatches||[]).map(m=>flt('bad',m.field||'mismatch',`expected ${textOrDash(m.expected)} / actual ${textOrDash(m.actual)}. ${m.remediation||''}`)).join('');
  return `<article class="card" style="margin-top:var(--gap)" data-help="fw_truthfulness"><div class="ch"><div class="tw"><span class="material-symbols-rounded">${bad?'gpp_maybe':unknown?'help':'verified_user'}</span><div><h2 class="ct">Contract diagnosis</h2><div class="cs">Expected-vs-observed firmware/schema contract.</div></div></div><span class="pill ${bad?'bad':unknown?'warn':'good'}">${esc(d.status||'unknown')}</span></div><table><tbody>${rows}</tbody></table><div class="contract-list">${issues||(unknown?flt('warn','Analysis unavailable','Contract diagnosis needs analyse_summary.json.'):flt('good','No detected mismatch','Observed fields match the expected v15/v11 contract.'))}</div></article>`;
}
function compareMetricBlock(label,s){
  s=s||{}; return `<div class="compare-card"><h3>${esc(label)}</h3><table><tbody>${[row('Session',s.session_id),row('Verdict',s.verdict),row('HR RMSE',fmt(s.hr_rmse??s.hr_metrics?.rmse,2)),row('HR r',fmt(s.hr_r??s.hr_metrics?.r,3)),row('RR RMSE',fmt(s.rr_rmse??s.rr_metrics?.rmse,2)),row('BLE coverage',fmt(s.reference_quality?.distilled_rows_pct_of_raw??s.reference_quality?.coverage_pct??s.ref_coverage_pct,1))].join('')}</tbody></table></div>`;
}
async function renderComparePanel(id){
  const el=document.getElementById('comparePanel'); if(!el) return;
  try{
    const c=await apiJson(`/api/sessions/${encodeURIComponent(id)}/compare`);
    el.innerHTML=`<div class="ch"><div class="tw"><span class="material-symbols-rounded">compare_arrows</span><div><h2 class="ct">Compare</h2><div class="cs">Selected session against previous and best available session.</div></div></div></div><div class="compare-grid">${compareMetricBlock('Selected',c.selected)}${compareMetricBlock('Previous',c.previous)}${compareMetricBlock('Best',c.best)}</div>`;
  }catch(e){
    el.innerHTML=`<div class="ch"><div class="tw"><span class="material-symbols-rounded">compare_arrows</span><div><h2 class="ct">Compare</h2><div class="cs">${esc(e.message)}</div></div></div></div>`;
  }
}
async function rerunAnalysis(id){
  try{ const r=await apiJson(`/api/sessions/${encodeURIComponent(id)}/analyse`,{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'}); toast(`Analysis rerun started (pid ${r.pid||'--'})`,'play_arrow'); }
  catch(e){ toast(e.message,'warning'); }
}
function verdictCard(v){
  v=v||{}; const kind=String(v.readiness_kind||v.verdict||'not_ready').toLowerCase(); const meta=readinessMeta(kind);
  const passed=(v.passed||[]).map(x=>`<div class="flt good"><div class="flt-t"><span class="material-symbols-rounded">check_circle</span>${esc(x)}</div></div>`).join('');
  const failed=(v.failed||[]).map(x=>`<div class="flt bad"><div class="flt-t"><span class="material-symbols-rounded">error</span>${esc(x)}</div></div>`).join('');
   const cats=(v.categories||[]).map(c=>`<div data-help="verdict.${esc(kind)}">${flt(c.status==='pass'?'good':c.status==='warn'?'warn':'bad',c.label||c.id,`${c.detail||''}${c.remediation?' · '+c.remediation:''}`)}</div>`).join('');
  return `<section class="verdict-card ${esc(meta.cls)}" data-help="verdict.${esc(kind)}" tabindex="0"><div class="verdict-head"><div><div class="eyebrow">ML readiness</div><h2 class="verdict-title">${esc(v.headline||'Analysis verdict')}</h2></div><div class="verdict-kind ${esc(meta.cls)}"><span class="material-symbols-rounded">${esc(meta.icon)}</span><span>${esc(meta.label)}</span></div></div><div class="cs">Limitation: ${esc(v.limitation_kind||'none')} · Next action: ${esc(v.next_action||'review')}</div><div class="report-grid"><div><h3 class="ct">Passed</h3><div class="report-list">${passed||'<div class="cs">No passed criteria reported.</div>'}</div></div><div><h3 class="ct">Failed</h3><div class="report-list">${failed||'<div class="cs">No failed criteria reported.</div>'}</div></div></div><details><summary>Why</summary><div class="report-list" style="margin-top:10px">${cats||'<div class="cs">No category details.</div>'}</div></details></section>`;
}
function renderReportSummary(s,id,body){
  const verdict=s.ml_readiness_verdict||{verdict:s.verdict,headline:'Analysis verdict'};
  const hr=s.hr_metrics||{}; const rr=s.rr_metrics||{}; const sig=s.signal_quality||{}; const ref=s.reference_quality||s.ble_ref_quality||{}; const gates=s.gates||{}; const hist=s.histograms||{}; const truth=s.truthfulness||{};
  const a=s.analysis||{};
  const corrCov=first(s.raw_hr_correction_coverage_pct,a.raw_hr_correction_coverage_pct);
  const rawCorrBias=first(s.raw_hr_corrected_bias_bpm,a.raw_hr_corrected_bias_bpm);
  const rawUncorrBias=first(s.raw_hr_uncorrected_bias_bpm,a.raw_hr_uncorrected_bias_bpm);
  const rescueCount=first(s.hr_rescue_publish_count,a.hr_rescue_publish_count);
  const rescueCov=first(s.hr_rescue_publish_coverage_pct,a.hr_rescue_publish_coverage_pct);
  const oa=s.oracle_audit||a.oracle_audit||{};
  const hrOracleMae=first(s.hr_oracle_candidate_mae_bpm,oa.hr_oracle_candidate_mae_bpm,a.hr_oracle_candidate_mae_bpm);
  const hrOracleCov=first(s.hr_oracle_candidate_coverage_pct,oa.hr_oracle_candidate_coverage_pct,a.hr_oracle_candidate_coverage_pct);
  const rrOracleMae=first(s.rr_oracle_candidate_mae_bpm,oa.rr_oracle_candidate_mae_bpm,a.rr_oracle_candidate_mae_bpm);
  const rrOracleCov=first(s.rr_oracle_candidate_coverage_pct,oa.rr_oracle_candidate_coverage_pct,a.rr_oracle_candidate_coverage_pct);
  const hrOracleHist=first(s.hr_best_candidate_source_histogram,oa.hr_best_candidate_source_histogram,a.hr_best_candidate_source_histogram);
  const rrOracleHist=first(s.rr_best_candidate_source_histogram,oa.rr_best_candidate_source_histogram,a.rr_best_candidate_source_histogram);
  const cl=s.coverage_loss_ledger||a.coverage_loss_ledger||{};
  const hrCoverageLoss=first(s.hr_coverage_loss_by_stage,a.hr_coverage_loss_by_stage,cl.hr_coverage_loss_by_stage,cl.hr);
  const rrCoverageLoss=first(s.rr_coverage_loss_by_stage,a.rr_coverage_loss_by_stage,cl.rr_coverage_loss_by_stage,cl.rr);
  const hrSalvage=first(s.hr_salvageable_frames_pct,a.hr_salvageable_frames_pct,cl.hr_salvageable_frames_pct);
  const rrSalvage=first(s.rr_salvageable_frames_pct,a.rr_salvageable_frames_pct,cl.rr_salvageable_frames_pct);
  const statList=(rows)=>`<div class="stat-list">${rows.filter(r=>r&&r[1]!==null&&r[1]!==undefined&&textOrDash(r[1])!=='--').map(([label,value,cls])=>`<div class="stat ${cls||''}"><div class="k">${esc(label)}</div><div class="v">${esc(value)}</div></div>`).join('')||'<div class="cs">No summary fields available.</div>'}</div>`;
  document.getElementById('reportSub').textContent=`${s.session_id||id} · ${s.status||'unknown'}`;
  if(s.status==='incomplete'){
    body.innerHTML=`${verdictCard(verdict)}<div style="margin-top:10px">${verdictActionHtml(verdict)}</div><div class="setup-alert" style="display:block;margin-top:var(--gap)">Analysis is incomplete for this session. Rerun analysis from the session list or use the CLI.</div>${downloadsHtml(s,id)}`;
    return;
  }
  body.innerHTML=[
    `<div class="report-grid three">${reportMetric('Session',s.session_id||id,'',0)}${reportMetric('Duration',durationLabel(s.duration_s||0),'',0)}${reportMetric('Started',s.started_at||'--','',0)}</div>`,
    `<table style="margin-top:12px"><tbody>${[row('Subject',s.subject_label||s.subject||''),row('Operator',s.operator_label||s.operator||''),row('Started from',s.started_from||''),row('Trainer / Dashboard',`${textOrDash(s.trainer_version)} / ${textOrDash(s.dashboard_version)}`)].join('')}</tbody></table>`,
    verdictCard(verdict),
    `<div style="margin-top:10px">${verdictActionHtml(verdict)}</div>`,
    `<div class="report-grid three"><div data-help="metric.rmse">${reportMetric('HR RMSE',hr.rmse,'bpm',2)}</div><div data-help="metric.r">${reportMetric('HR r',hr.r,'',3)}</div><div data-help="reference.coverage">${reportMetric('HR coverage',hr.coverage_pct,'%',1)}</div><div data-help="metric.rmse">${reportMetric('RR RMSE',rr.rmse,'br/min',2)}</div><div data-help="metric.r">${reportMetric('RR r',rr.r,'',3)}</div><div data-help="reference.coverage">${reportMetric('BLE coverage',ref.distilled_rows_pct_of_raw??ref.coverage_pct,'%',1)}</div></div>`,
    `<div class="report-grid three"><article class="card" data-help="correction.rescue"><div class="ch"><div class="tw"><span class="material-symbols-rounded">hearing</span><div><h2 class="ct">HR correction / rescue</h2><div class="cs">Corrected raw versus uncorrected raw and rescue publishes.</div></div></div></div>${statList([['Correction coverage',corrCov===null||corrCov===undefined?'--':`${fmt(corrCov,1)}%`,corrCov!==null&&Number(corrCov)>=90?'good':corrCov!==null&&Number(corrCov)>=50?'warn':'bad'],['Corrected raw bias',rawCorrBias===null||rawCorrBias===undefined?'--':`${fmt(rawCorrBias,2)} bpm`,rawCorrBias!==null&&Number(rawCorrBias)===0?'good':''],['Uncorrected raw bias',rawUncorrBias===null||rawUncorrBias===undefined?'--':`${fmt(rawUncorrBias,2)} bpm`,''],['Rescue publishes',rescueCount===null||rescueCount===undefined?'--':textOrDash(rescueCount),'' ],['Rescue share',rescueCov===null||rescueCov===undefined?'--':`${fmt(rescueCov,1)}%`,rescueCov!==null&&Number(rescueCov)>0?'warn':'']])}</article><article class="card" data-help="oracle.audit"><div class="ch"><div class="tw"><span class="material-symbols-rounded">psychology_alt</span><div><h2 class="ct">Oracle audit</h2><div class="cs">Best-candidate replay metrics if a blocked signal existed.</div></div></div></div>${statList([['HR oracle MAE',hrOracleMae===null||hrOracleMae===undefined?'--':`${fmt(hrOracleMae,2)} bpm`,hrOracleMae!==null&&Number(hrOracleMae)<8?'good':''],['HR oracle coverage',hrOracleCov===null||hrOracleCov===undefined?'--':`${fmt(hrOracleCov,1)}%`,hrOracleCov!==null&&Number(hrOracleCov)>=50?'good':''],['RR oracle MAE',rrOracleMae===null||rrOracleMae===undefined?'--':`${fmt(rrOracleMae,2)} br/min`,rrOracleMae!==null&&Number(rrOracleMae)<4?'good':''],['RR oracle coverage',rrOracleCov===null||rrOracleCov===undefined?'--':`${fmt(rrOracleCov,1)}%`,rrOracleCov!==null&&Number(rrOracleCov)>=70?'good':'']])}${hrOracleHist||rrOracleHist?`<h3 class="ct" style="font-size:14px;margin-top:14px">Best HR source</h3>${histBars(hrOracleHist)}<h3 class="ct" style="font-size:14px;margin-top:14px">Best RR source</h3>${histBars(rrOracleHist)}`:''}</article><article class="card" data-help="coverage.loss"><div class="ch"><div class="tw"><span class="material-symbols-rounded">analytics</span><div><h2 class="ct">Coverage loss</h2><div class="cs">First failing stage when HR or RR did not publish.</div></div></div></div>${statList([['HR salvageable',hrSalvage===null||hrSalvage===undefined?'--':`${fmt(hrSalvage,1)}%`,hrSalvage!==null&&Number(hrSalvage)>=50?'good':''],['RR salvageable',rrSalvage===null||rrSalvage===undefined?'--':`${fmt(rrSalvage,1)}%`,rrSalvage!==null&&Number(rrSalvage)>=50?'good':''],['HR ledgers',hrCoverageLoss?'present':'--',hrCoverageLoss?'warn':''],['RR ledgers',rrCoverageLoss?'present':'--',rrCoverageLoss?'warn':'']])}${hrCoverageLoss?`<h3 class="ct" style="font-size:14px;margin-top:14px">HR loss by stage</h3>${histBars(hrCoverageLoss)}`:''}${rrCoverageLoss?`<h3 class="ct" style="font-size:14px;margin-top:14px">RR loss by stage</h3>${histBars(rrCoverageLoss)}`:''}</article></div>`,
    `<div class="report-grid"><article class="card" data-help="metric.rmse"><div class="ch"><div class="tw"><span class="material-symbols-rounded">monitoring</span><div><h2 class="ct">Baseline metrics</h2><div class="cs">Valid-only alignment metrics.</div></div></div></div><table><tbody>${metricRows('HR',hr)+metricRows('RR',rr)}</tbody></table></article><article class="card" data-help="reference.coverage"><div class="ch"><div class="tw"><span class="material-symbols-rounded">bluetooth_connected</span><div><h2 class="ct">Reference quality</h2><div class="cs">BLE packet and oximeter quality.</div></div></div></div><table><tbody>${[row('Status',ref.status),row('Raw packets',ref.raw_packets),row('Parsed rows',ref.parsed_rows),row('Coverage',`${fmt(ref.distilled_rows_pct_of_raw??ref.coverage_pct,1)}%`),row('Packet loss',`${fmt(ref.packet_loss_pct,1)}%`),row('Decode error',`${fmt(ref.decode_error_pct,1)}%`),row('PI median',fmt(ref.pi_median,2))].join('')}</tbody></table></article></div>`,
    `<div class="report-grid"><article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">signal_cellular_alt</span><div><h2 class="ct">Signal quality</h2><div class="cs">Radar and session quality indicators.</div></div></div></div><table><tbody>${[row('PQI lock',`${fmt(sig.pqi_lock_pct,1)}%`),row('Locked coverage',`${fmt(sig.coverage_locked,1)}%`),row('Settling coverage',`${fmt(sig.coverage_settling,1)}%`),row('Session quality score',fmt(sig.session_quality_score,1)),row('Internal consistency',fmt(sig.internal_consistency_score,1)),row('Review flags',(s.review_flags||[]).join(', ')||'none')].join('')}</tbody></table></article><article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">rule</span><div><h2 class="ct">Gates</h2><div class="cs">Primary, secondary, and phase subset gates.</div></div></div></div><table><tbody>${[row('Primary',gateLine(gates.primary)),row('Secondary',gateLine(gates.secondary)),row('Combined',gateLine(gates.combined)),row('Locked',gateLine(gates.locked)),row('Settling',gateLine(gates.settling)),row('Golden check',gateLine(gates.golden_check))].join('')}</tbody></table></article></div>`,
    `<div class="report-grid"><article class="card" data-help="hist.gate"><div class="ch"><div class="tw"><span class="material-symbols-rounded">stacked_bar_chart</span><div><h2 class="ct">Gate histograms</h2><div class="cs">Top HR/RR gate reasons remain visible.</div></div></div></div><h3 class="ct" style="font-size:14px">HR gate</h3>${histBars(hist.top_hr_gate)}<h3 class="ct" style="font-size:14px;margin-top:14px">RR gate</h3>${histBars(hist.top_rr_gate)}</article><article class="card" data-help="hist.publish"><div class="ch"><div class="tw"><span class="material-symbols-rounded">history</span><div><h2 class="ct">Publish histograms</h2><div class="cs">Suppression reasons are not hidden.</div></div></div></div><h3 class="ct" style="font-size:14px">HR publish</h3>${histBars(hist.top_hr_publish)}<h3 class="ct" style="font-size:14px;margin-top:14px">RR publish</h3>${histBars(hist.top_rr_publish)}</article></div>`,
    contractDiagnosisHtml(s.contract_diagnosis),
    `<article class="card" style="margin-top:var(--gap)" id="comparePanel"><div class="ch"><div class="tw"><span class="material-symbols-rounded">compare_arrows</span><div><h2 class="ct">Compare</h2><div class="cs">Loading comparison...</div></div></div></div></article>`,
    `<article class="card" style="margin-top:var(--gap)" data-help="fw_truthfulness"><div class="ch"><div class="tw"><span class="material-symbols-rounded">verified_user</span><div><h2 class="ct">Truthfulness / manifest</h2><div class="cs">Firmware, schema, and scoring identity.</div></div></div></div><table><tbody>${[row('Sketch FW',truth.sketch_fw),row('Module FW',truth.module_fw),row('Module valid',truth.module_version_valid),row('Contract length',truth.contract_length),row('Schema hash',truth.schema_hash),row('Scoring weights hash',truth.scoring_weights_hash)].join('')}</tbody></table></article>`,
    downloadsHtml(s,id)
  ].join('');
  renderComparePanel(id);
}
function gateLine(g){ if(!g||typeof g!=='object') return '--'; const st=g.status||(g.passed===true?'PASS':g.passed===false?'FAIL':'--'); return `${textOrDash(st)} · r=${fmt(g.r,3)} · rmse=${fmt(g.rmse,2)} · n=${textOrDash(g.n)}`; }
function downloadsHtml(s,id){
  const downloads=s.downloads||[];
  return `<article class="card" style="margin-top:var(--gap)"><div class="ch"><div class="tw"><span class="material-symbols-rounded">download</span><div><h2 class="ct">Downloads</h2><div class="cs">Raw files and analysis artifacts.</div></div></div><div class="ca"><button class="chip-btn" type="button" onclick="rerunAnalysis('${esc(id)}')">Rerun analysis</button></div></div><div class="chips">${downloads.map(d=>`<a class="chip" href="${esc(d.href||d.url||'#')}">${esc(d.label||d.relpath||'file')}</a>`).join('')}<a class="chip" href="/api/report/export?session=${encodeURIComponent(id)}">Export thesis HTML</a><a class="chip" href="/api/sessions/${encodeURIComponent(id)}/files/analysis/analyse_report.html">Legacy report</a></div></article>`;
}
async function renderHelpView(){
  const body=document.getElementById('helpBody'); if(!body || !S.ctl.on) return;
  if(!S.ctl.help){ try{ S.ctl.help=await apiJson('/api/help/schema'); }catch(e){ body.textContent=e.message; return; } }
  const h=S.ctl.help; const q=String(S.ctl.helpQuery||'').trim().toLowerCase();
  const btn=document.getElementById('helpModeBtn'); if(btn) btn.textContent=S.ctl.helpAdvanced?'Advanced':'Beginner';
  const search=document.getElementById('helpSearch'); if(search && search.value!==S.ctl.helpQuery) search.value=S.ctl.helpQuery;
  const match=(...parts)=>!q || parts.join(' ').toLowerCase().includes(q);
  const faq=(h.faq||[]).filter(x=>match(x.q,x.a)).map(x=>`<dt>${esc(x.q||x.question)}</dt><dd>${esc(x.a||x.answer)}</dd>`).join('');
  const gloss=(h.glossary||[]).filter(x=>match(x.id,x.term,x.short,x.long,x.advanced,x.category)).map(x=>`<dt>${esc(x.term||x.id)} <span class="pill">${esc(x.category||'')}</span></dt><dd>${esc(S.ctl.helpAdvanced?(x.advanced||x.long||x.short):(x.short||x.long||x.advanced))}</dd>`).join('');
  const dsp=(h.dsp_steps||[]).filter(x=>match(x.n,x.title,x.summary,x.detail)).map(x=>`<dt>${esc(`${x.n||''} ${x.title||''}`)}</dt><dd>${esc(S.ctl.helpAdvanced?(x.detail||x.summary):(x.summary||x.detail))}</dd>`).join('');
  const tr=(h.troubleshooting||[]).filter(x=>match(x.id,x.title,(x.steps||[]).join(' '))).map(x=>`<dt>${esc(x.title||x.id)}</dt><dd>${esc((x.steps||[]).join(' '))}</dd>`).join('');
  const tips=Object.entries(h.tooltips||{}).filter(([k,v])=>match(k,v)).slice(0,80).map(([k,v])=>`<dt><code>${esc(k)}</code></dt><dd>${esc(v)}</dd>`).join('');
  body.innerHTML=`<section class="help-section"><details open><summary>FAQ</summary><dl>${faq||'<dd>No matching FAQ entries.</dd>'}</dl></details></section><section class="help-section"><details open><summary>Glossary</summary><dl>${gloss||'<dd>No matching glossary entries.</dd>'}</dl></details></section><section class="help-section"><details><summary>DSP Process</summary><dl>${dsp||'<dd>No matching DSP steps.</dd>'}</dl></details></section><section class="help-section"><details><summary>Troubleshooting</summary><dl>${tr||'<dd>No matching troubleshooting entries.</dd>'}</dl></details></section><section class="help-section"><details><summary>Tooltip Keys</summary><dl>${tips||'<dd>No matching tooltip keys.</dd>'}</dl></details></section>`;
}
function toggleHelpAdvanced(){ S.ctl.helpAdvanced=!S.ctl.helpAdvanced; localStorage.setItem('rvt-help-advanced',S.ctl.helpAdvanced?'1':'0'); renderHelpView(); }

function drawRadar(p,stale=false){
  const c=document.getElementById('radarCanvas'); if(!c||c.offsetParent===null) return;
  const ctx=c.getContext('2d'); const dpr=window.devicePixelRatio||1; const rect=c.getBoundingClientRect(); if(!rect.width||!rect.height) return;
  c.width=rect.width*dpr; c.height=rect.height*dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
  const w=rect.width,h=rect.height; ctx.clearRect(0,0,w,h);
  const dark=document.documentElement.getAttribute('data-theme')==='dark';
  const ox=w/2,oy=h-20; const R=Math.min(w*.46,h*.84); const maxM=6;
  const g=ctx.createRadialGradient(ox,oy,0,ox,oy,R);
  g.addColorStop(0,dark?'rgba(179,197,255,0.06)':'rgba(47,95,201,0.06)'); g.addColorStop(1,'transparent');
  ctx.fillStyle=g; ctx.beginPath(); ctx.arc(ox,oy,R,Math.PI,2*Math.PI); ctx.fill();
  ctx.strokeStyle=dark?'rgba(255,255,255,0.1)':'rgba(116,120,132,.2)'; ctx.lineWidth=1.5; ctx.setLineDash([]);
  for(let i=1;i<=6;i++){ const r=R*(i/maxM); ctx.beginPath(); ctx.arc(ox,oy,r,Math.PI,2*Math.PI); ctx.stroke(); ctx.fillStyle=dark?'#8e9099':'#7b8494'; ctx.font='11px Roboto Mono'; ctx.fillText(`${i}m`,ox+6,oy-r-4); }
  ctx.strokeStyle=dark?'rgba(255,255,255,.06)':'rgba(116,120,132,.1)';
  for(let a=0;a<=180;a+=30){ const rad=(a*Math.PI)/180; ctx.beginPath(); ctx.moveTo(ox,oy); ctx.lineTo(ox+Math.cos(Math.PI+rad)*R, oy+Math.sin(Math.PI+rad)*R); ctx.stroke(); }
  ctx.strokeStyle=dark?'#c4c6d0':'#3b434f'; ctx.lineWidth=1.5;
  ctx.beginPath(); ctx.moveTo(ox,oy); ctx.lineTo(ox,oy-R); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0,oy); ctx.lineTo(w,oy); ctx.stroke();
  ctx.fillStyle=dark?'#e3e2e6':'#171c24'; ctx.beginPath(); ctx.arc(ox,oy,5,0,Math.PI*2); ctx.fill();
  const sweep=((Date.now()/40)%180)*Math.PI/180;
  const sg=ctx.createLinearGradient(ox,oy,ox+Math.cos(Math.PI+sweep)*R,oy+Math.sin(Math.PI+sweep)*R);
  sg.addColorStop(0,dark?'rgba(179,197,255,.5)':'rgba(47,95,201,.6)'); sg.addColorStop(1,'transparent');
  ctx.strokeStyle=sg; ctx.lineWidth=2; ctx.beginPath(); ctx.moveTo(ox,oy); ctx.lineTo(ox+Math.cos(Math.PI+sweep)*R,oy+Math.sin(Math.PI+sweep)*R); ctx.stroke();
  const x=num(getR(p,'primary_x')), y=num(getR(p,'primary_y'));
  if(!stale && x!==null && y!==null && y>=0){
    const px=ox+(x/maxM)*R, py=oy-(y/maxM)*R;
    const pulse=(Math.sin(Date.now()/400)+1)/2;
    ctx.fillStyle=dark?'rgba(179,197,255,.25)':'rgba(47,95,201,.2)'; ctx.beginPath(); ctx.arc(px,py,14+pulse*8,0,Math.PI*2); ctx.fill();
    ctx.fillStyle=dark?'#b3c5ff':'#2f5fc9'; ctx.beginPath(); ctx.arc(px,py,8,0,Math.PI*2); ctx.fill();
    ctx.strokeStyle=dark?'#1a1b1e':'#ffffff'; ctx.lineWidth=3; ctx.stroke();
    ctx.fillStyle=dark?'#e3e2e6':'#171c24'; ctx.font='600 12px Roboto Mono'; ctx.fillText(`(${x.toFixed(2)}, ${y.toFixed(2)}) m`, Math.min(w-140,px+14), Math.max(18,py-12));
  } else {
    ctx.fillStyle=dark?'#8e9099':'#7b8494'; ctx.font='14px Roboto'; ctx.fillText(stale?'Spatial fix expired':'No fresh spatial fix',16,26);
  }
}

function chartCommon(){
  const dark=document.documentElement.getAttribute('data-theme')==='dark';
  const g=dark?'rgba(255,255,255,0.08)':'rgba(116,120,132,.12)'; const t=dark?'#c4c6d0':'#5b6070';
  return { responsive:true,maintainAspectRatio:false,animation:false,interaction:{mode:'index',intersect:false},
    plugins:{legend:{position:'top',labels:{color:t,boxWidth:10,boxHeight:10,font:{size:11,weight:'600'}}}},
    scales:{ x:{ticks:{color:t,maxTicksLimit:6,font:{size:10}},grid:{color:g}}, y:{ticks:{color:t,font:{size:10}},grid:{color:g}} } };
}
function buildCharts(){
  const co=chartCommon();
  const P={p:'#2f5fc9',pl:'#7da0e5',t:'#1e9a67',tl:'#67d9a1',err:'#ba1a1a',pu:'#7e57c2',sl:'#8d99ae',teal:'#0ea5a4',amb:'#f59e0b',ink:'#0f766e'};
  S.charts.hr=new Chart(document.getElementById('hrChart'),{type:'line',data:{labels:[],datasets:[
    {label:'Reported HR',data:[],borderColor:P.p,backgroundColor:P.p+'22',borderWidth:2.5,tension:.3,fill:true,pointRadius:0},
    {label:'Candidate HR',data:[],borderColor:P.pu,backgroundColor:P.pu,borderWidth:1.5,tension:.25,borderDash:[7,4],pointRadius:0},
    {label:'Raw HR',data:[],borderColor:P.sl,backgroundColor:P.sl,borderWidth:1.5,tension:.25,borderDash:[3,4],pointRadius:0},
    {label:'Corrected raw HR',data:[],borderColor:P.amb,backgroundColor:P.amb,borderWidth:1.5,tension:.25,borderDash:[2,3],pointRadius:0},
    {label:'BLE HR',data:[],borderColor:P.err,backgroundColor:P.err,borderWidth:1.8,tension:.25,pointRadius:0},
    {label:'Trusted anchor',data:[],borderColor:P.ink,backgroundColor:P.ink,borderWidth:1.5,tension:.25,borderDash:[4,4],pointRadius:0}
  ]},options:JSON.parse(JSON.stringify(co))});
  S.charts.rr=new Chart(document.getElementById('rrChart'),{type:'line',data:{labels:[],datasets:[
    {label:'Reported RR',data:[],borderColor:P.t,backgroundColor:P.t+'22',borderWidth:2.5,tension:.3,fill:true,pointRadius:0},
    {label:'Candidate RR',data:[],borderColor:P.tl,backgroundColor:P.tl,borderWidth:1.5,tension:.25,borderDash:[7,4],pointRadius:0},
    {label:'Final publish',data:[],borderColor:P.teal,backgroundColor:P.teal,borderWidth:1.5,tension:.25,borderDash:[3,4],pointRadius:0},
    {label:'RR anchor',data:[],borderColor:P.amb,backgroundColor:P.amb,borderWidth:1.5,tension:.2,pointRadius:0},
    {label:'BLE RR',data:[],borderColor:P.err,backgroundColor:P.err,borderWidth:1.5,tension:.25,pointRadius:0}
  ]},options:JSON.parse(JSON.stringify(co))});
  const diag=JSON.parse(JSON.stringify(co));
  diag.scales.y1={position:'left',min:0,max:1.2,grid:{color:co.scales.y.grid.color},ticks:{color:co.scales.y.ticks.color}};
  diag.scales.y2={position:'right',min:0,max:6,grid:{drawOnChartArea:false},ticks:{color:co.scales.y.ticks.color}};
  delete diag.scales.y;
  S.charts.rrDiag=new Chart(document.getElementById('rrDiagChart'),{type:'line',data:{labels:[],datasets:[
    {label:'RR candidate conf',data:[],borderColor:P.p,backgroundColor:P.p,borderWidth:2,tension:.3,yAxisID:'y1',pointRadius:0},
    {label:'Recovery count',data:[],borderColor:P.pu,backgroundColor:P.pu,borderWidth:2,tension:.1,yAxisID:'y2',pointRadius:0},
    {label:'Seed consistency',data:[],borderColor:P.t,backgroundColor:P.t,borderWidth:2,tension:.1,yAxisID:'y2',pointRadius:0}
  ]},options:diag});
  S.charts.breath=new Chart(document.getElementById('breathChart'),{type:'line',data:{labels:[],datasets:[{label:'breath_phase',data:[],borderColor:'#e05a2a',backgroundColor:'#e05a2a22',borderWidth:2,pointRadius:0,tension:.2,fill:true}]},options:JSON.parse(JSON.stringify(co))});
  S.charts.heart=new Chart(document.getElementById('heartChart'),{type:'line',data:{labels:[],datasets:[{label:'heart_phase',data:[],borderColor:'#c93232',backgroundColor:'#c9323222',borderWidth:2,pointRadius:0,tension:.2,fill:true}]},options:JSON.parse(JSON.stringify(co))});
  const bo=JSON.parse(JSON.stringify(co)); bo.interaction={mode:'nearest',intersect:true}; bo.plugins.legend.display=false;
  bo.scales.x={type:'linear',title:{display:true,text:'Ref HR bucket midpoint (bpm)',color:co.scales.x.ticks.color},ticks:{color:co.scales.x.ticks.color,stepSize:10},grid:{color:co.scales.x.grid.color}};
  bo.scales.y={title:{display:true,text:'Raw HR bias (bpm)',color:co.scales.y.ticks.color},ticks:{color:co.scales.y.ticks.color},grid:{color:co.scales.y.grid.color}};
  S.charts.bucket=new Chart(document.getElementById('rawHrBucketChart'),{type:'scatter',data:{datasets:[{label:'Raw HR bias',data:[],borderColor:P.err,backgroundColor:P.err+'cc',pointRadius:[],pointHoverRadius:8}]},options:bo});

  const so={responsive:true,maintainAspectRatio:false,animation:false,plugins:{legend:{display:false},tooltip:{enabled:false}},scales:{x:{display:false},y:{display:false}},elements:{point:{radius:0}}};
  S.charts.kHr=new Chart(document.getElementById('kpiHrSpark'),{type:'line',data:{labels:[],datasets:[{data:[],borderColor:'currentColor',backgroundColor:'rgba(0,0,0,0.12)',borderWidth:2,tension:.35,fill:true}]},options:so});
  S.charts.kRr=new Chart(document.getElementById('kpiRrSpark'),{type:'line',data:{labels:[],datasets:[{data:[],borderColor:'currentColor',backgroundColor:'rgba(0,0,0,0.12)',borderWidth:2,tension:.35,fill:true}]},options:so});
  S.charts.kFps=new Chart(document.getElementById('kpiFpsSpark'),{type:'line',data:{labels:[],datasets:[{data:[],borderColor:'#2f5fc9',backgroundColor:'rgba(47,95,201,0.15)',borderWidth:2,tension:.35,fill:true}]},options:so});
  S.charts.kDist=new Chart(document.getElementById('kpiDistSpark'),{type:'line',data:{labels:[],datasets:[{data:[],borderColor:'#1e9a67',backgroundColor:'rgba(30,154,103,0.15)',borderWidth:2,tension:.35,fill:true}]},options:so});
}

function updateCharts(p){
  const s=p.series||{}; const labels=Array.isArray(s.t)?s.t:[];
  const apply=(c,ls,arr)=>{ if(!ls.length){ c.data.labels=[]; c.data.datasets.forEach((d,i)=>d.data=arr[i]||[]); } else { const last=num(ls[ls.length-1]); const cut=S.rangeS==='all'?-Infinity:last-S.rangeS; let i=0; while(i<ls.length&&num(ls[i])<cut) i++; c.data.labels=ls.slice(i).map(t=>num(t)!==null?num(t).toFixed(0)+'s':''); c.data.datasets.forEach((d,j)=>{ const a=arr[j]||[]; d.data=a.slice(i); }); } c.update('none'); };
  apply(S.charts.hr,labels,[
    Array.isArray(s.reported_hr)?validSeriesOrNull(s.reported_hr,s.radar_hr_valid):(Array.isArray(s.radar_hr)?validSeriesOrNull(s.radar_hr,s.radar_hr_valid):[]),
    Array.isArray(s.candidate_hr)?s.candidate_hr:[],
    Array.isArray(s.raw_hr_uncorrected)?s.raw_hr_uncorrected:(Array.isArray(s.raw_hr)?s.raw_hr:[]),
    Array.isArray(s.raw_hr_corrected)?s.raw_hr_corrected:[],
    Array.isArray(s.ble_hr)?s.ble_hr:[],
    Array.isArray(s.hr_trusted_phase_anchor)?s.hr_trusted_phase_anchor:[]
  ]);
  apply(S.charts.rr,labels,[
    Array.isArray(s.reported_rr)?validSeriesOrNull(s.reported_rr,s.radar_rr_valid):(Array.isArray(s.radar_rr)?validSeriesOrNull(s.radar_rr,s.radar_rr_valid):[]),
    Array.isArray(s.candidate_rr)?s.candidate_rr:[],
    Array.isArray(s.rr_final_publish_candidate)?s.rr_final_publish_candidate:[],
    Array.isArray(s.rr_anchor_value)?s.rr_anchor_value:[],
    Array.isArray(s.ble_rr)?s.ble_rr:[]
  ]);
  apply(S.charts.rrDiag,labels,[
    Array.isArray(s.candidate_rr_conf)?s.candidate_rr_conf:[],
    Array.isArray(s.rr_fundamental_recovery_count)?s.rr_fundamental_recovery_count:[],
    Array.isArray(s.rr_raw_seed_consistent_count)?s.rr_raw_seed_consistent_count:[]
  ]);
  const breath=Array.isArray(s.breath_phase)?s.breath_phase:[]; const heart=Array.isArray(s.heart_phase)?s.heart_phase:[];
  S.charts.breath.data.labels=labels.slice(-breath.length); S.charts.breath.data.datasets[0].data=breath; S.charts.breath.update('none');
  S.charts.heart.data.labels=labels.slice(-heart.length); S.charts.heart.data.datasets[0].data=heart; S.charts.heart.update('none');

  const bp=bucketData(p);
  S.charts.bucket.data.datasets[0].data=bp.map(x=>({x:x.x,y:x.y}));
  S.charts.bucket.data.datasets[0].pointRadius=bp.map(x=>Math.max(4,Math.min(12,3+Math.sqrt(x.count))));
  S.charts.bucket.update('none');
  document.getElementById('bucketTable').innerHTML = bp.length?bp.slice(-6).map(x=>row(`${x.bucket} bpm`, `${fmt(x.y,1)} bpm bias · n=${x.count}`)).join(''):row('Buckets','Waiting for raw_hr and ref/ble_hr series');

  const roll=a=>a.filter(v=>v!==null&&v!==undefined&&Number.isFinite(Number(v))).slice(-40);
  const hrArr=Array.isArray(s.reported_hr)?validSeriesOrNull(s.reported_hr,s.radar_hr_valid):[];
  const rrArr=Array.isArray(s.reported_rr)?validSeriesOrNull(s.reported_rr,s.radar_rr_valid):[];
  const fpsN=computeFps(p); if(fpsN!==null){ S.spark.fps.push(fpsN); if(S.spark.fps.length>40) S.spark.fps.shift(); }
  const distN=num(getR(p,'distance_cm')); if(distN!==null){ S.spark.dist.push(distN); if(S.spark.dist.length>40) S.spark.dist.shift(); }
  const ups=(c,d)=>{ c.data.labels=d.map((_,i)=>i); c.data.datasets[0].data=d; c.update('none'); };
  ups(S.charts.kHr,roll(hrArr)); ups(S.charts.kRr,roll(rrArr)); ups(S.charts.kFps,S.spark.fps); ups(S.charts.kDist,S.spark.dist);
}

function setRange(btn,s){ S.rangeS=s; btn.parentElement.querySelectorAll('button').forEach(b=>b.classList.remove('active')); btn.classList.add('active'); if(S.lastPayload) updateCharts(S.lastPayload); }

function openDrawer(){ document.getElementById('dv').classList.add('open'); document.getElementById('dvOv').classList.add('open'); }
function closeDrawer(){ document.getElementById('dv').classList.remove('open'); document.getElementById('dvOv').classList.remove('open'); }
function renderAlerts(a){
  const b=document.getElementById('dvBody'); const badge=document.getElementById('alertsBadge');
  if(!a.length){ b.innerHTML=`<div class="dv-e"><span class="material-symbols-rounded">task_alt</span>All clear<div style="margin-top:6px;font-size:12px;font-weight:500;">No active alerts in the current payload.</div></div>`; badge.style.display='none'; return; }
  b.innerHTML=a.map(x=>flt(x.severity,x.title,x.copy)).join('');
  const un=a.filter(x=>x.severity!=='good').length;
  if(un>0){ badge.style.display='grid'; badge.textContent=un; } else badge.style.display='none';
}

function snapshotNow(){
  if(!S.lastPayload) return toast('No data to snapshot');
  const p=S.lastPayload; const stl=hardStale(p);
  const hr=stl?null:livePub(p,'reported_hr'); const rr=stl?null:livePub(p,'reported_rr');
  const bh=p.ble.hr??lastSeries(p.series,'ble_hr'); const br=p.ble.rr??lastSeries(p.series,'ble_rr');
  const snap={ id:Date.now(), time:new Date().toLocaleTimeString([],{hour12:false}), hr, rr, bleHr:bh, bleRr:br,
    hrD:(num(hr)!==null&&num(bh)!==null)?num(hr)-num(bh):null, rrD:(num(rr)!==null&&num(br)!==null)?num(rr)-num(br):null,
    targets:textOrDash(getR(p,'num_targets')), fps:computeFps(p) };
  S.snaps.unshift(snap); if(S.snaps.length>30) S.snaps.pop();
  localStorage.setItem('rvt-snaps',JSON.stringify(S.snaps));
  renderSnaps(); toast(`Snapshot pinned · ${snap.time}`,'bookmark_added');
}
function clearSnaps(){ S.snaps=[]; localStorage.removeItem('rvt-snaps'); renderSnaps(); toast('Snapshots cleared'); }
function rmSnap(id){ S.snaps=S.snaps.filter(s=>s.id!==id); localStorage.setItem('rvt-snaps',JSON.stringify(S.snaps)); renderSnaps(); }
function renderSnaps(){
  const l=document.getElementById('snapsList'); if(!l) return;
  if(!S.snaps.length){ l.innerHTML=`<div class="snap-e">No pinned snapshots yet. Click <span class="material-symbols-rounded" style="font-size:14px;vertical-align:middle">bookmark_add</span> on any card (or use <kbd>⌘K</kbd> → Pin) to capture.</div>`; return; }
  l.innerHTML=S.snaps.map(s=>{
    const hd=s.hrD!==null?` Δ${s.hrD>=0?'+':''}${s.hrD.toFixed(1)}`:'';
    const rd=s.rrD!==null?` Δ${s.rrD>=0?'+':''}${s.rrD.toFixed(1)}`:'';
    return `<div class="snap">
      <span class="material-symbols-rounded" style="color:var(--md-sys-color-primary)">bookmark</span>
      <div>
        <div class="snap-t">${esc(s.time)} · ${s.targets} target(s) · ${fmt(s.fps,1)} Hz</div>
        <div class="snap-v"><span class="hr">HR ${fmt(s.hr,1)}${hd}</span><span class="rr">RR ${fmt(s.rr,1)}${rd}</span><span class="n">BLE ${fmt(s.bleHr,0)}/${fmt(s.bleRr,0)}</span></div>
      </div>
      <button class="ca-btn" onclick="rmSnap(${s.id})" title="Remove"><span class="material-symbols-rounded">close</span></button>
    </div>`;
  }).join('');
}

const CMDS=[
  {id:'o',label:'Go to Overview',icon:'dashboard',run:()=>switchTab('tab-overview')},
  {id:'w',label:'Go to Waves',icon:'monitor_heart',run:()=>switchTab('tab-waves')},
  {id:'h',label:'Go to HR Analysis',icon:'ecg_heart',run:()=>switchTab('tab-hr')},
  {id:'r',label:'Go to RR Analysis',icon:'air',run:()=>switchTab('tab-rr')},
  {id:'s',label:'Go to Snapshots',icon:'bookmark',run:()=>switchTab('tab-snaps')},
  {id:'a',label:'Go to Audit & Logs',icon:'fact_check',run:()=>switchTab('tab-audit')},
  {id:'pin',label:'Pin current snapshot',icon:'bookmark_add',sub:'⌘S',run:snapshotNow},
  {id:'al',label:'Open alerts drawer',icon:'notifications',run:openDrawer},
  {id:'st',label:'Open settings',icon:'tune',run:openSettings},
  {id:'dm',label:'Toggle demo mode',icon:'science',run:toggleDemoMode},
  {id:'th',label:'Toggle theme (light/dark)',icon:'dark_mode',run:toggleTheme},
  {id:'pa',label:'Toggle live polling',icon:'pause',run:togglePause},
  {id:'ex',label:'Export JSON payload',icon:'download',run:exportData},
  {id:'dc',label:'Density: comfortable',icon:'density_medium',run:()=>setDensity('comfortable')},
  {id:'ds',label:'Density: compact',icon:'density_small',run:()=>setDensity('compact')},
  {id:'cs',label:'Clear all snapshots',icon:'delete_sweep',run:clearSnaps},
];
let pSel=0, pFilt=CMDS.slice();
function openPalette(){ document.getElementById('pOv').classList.add('open'); document.getElementById('pIn').value=''; pFilt=CMDS.slice(); pSel=0; renderPalette(); setTimeout(()=>document.getElementById('pIn').focus(),60); }
function closePalette(){ document.getElementById('pOv').classList.remove('open'); }
function renderPalette(){ const l=document.getElementById('pL'); l.innerHTML=pFilt.map((c,i)=>`<div class="p-i ${i===pSel?'sel':''}" data-idx="${i}"><span class="material-symbols-rounded">${safeIcon(c.icon)}</span><span>${esc(c.label)}</span>${c.sub?`<span class="cmd-sub">${esc(c.sub)}</span>`:''}</div>`).join(''); l.querySelectorAll('.p-i').forEach((el,i)=>el.onclick=()=>runCmd(i)); }
function runCmd(i){ const c=pFilt[i]; if(!c) return; closePalette(); setTimeout(()=>c.run(),60); }
document.addEventListener('keydown',e=>{
  if((e.metaKey||e.ctrlKey)&&e.key.toLowerCase()==='k'){ e.preventDefault(); const o=document.getElementById('pOv').classList.contains('open'); if(o) closePalette(); else openPalette(); return; }
  if((e.metaKey||e.ctrlKey)&&e.key.toLowerCase()==='s'){ e.preventDefault(); snapshotNow(); return; }
  if(e.key==='Escape'){ closePalette(); closeDrawer(); const fs=document.querySelector('.card.fs'); if(fs){ fs.classList.remove('fs'); document.body.classList.remove('fs-open'); Object.values(S.charts).forEach(c=>c&&c.resize()); } }
  const po=document.getElementById('pOv').classList.contains('open');
  if(po){ if(e.key==='ArrowDown'){ e.preventDefault(); pSel=Math.min(pSel+1,pFilt.length-1); renderPalette(); } else if(e.key==='ArrowUp'){ e.preventDefault(); pSel=Math.max(pSel-1,0); renderPalette(); } else if(e.key==='Enter'){ e.preventDefault(); runCmd(pSel); } }
  else if(S.ctl.on && !e.metaKey && !e.ctrlKey && !e.altKey && /^[1-4]$/.test(e.key) && !e.target.closest?.('input,textarea,select,[contenteditable="true"]')){
    e.preventDefault();
    switchView(['home','live','report','help'][Number(e.key)-1]);
  }
});
document.addEventListener('input',e=>{ if(e.target.id==='pIn'){ const q=e.target.value.trim().toLowerCase(); pFilt=!q?CMDS.slice():CMDS.filter(c=>c.label.toLowerCase().includes(q)); pSel=0; renderPalette(); } });

function fsCard(btn){ const c=btn.closest('.card'); if(!c) return; const a=document.querySelector('.card.fs'); if(a&&a!==c) a.classList.remove('fs'); c.classList.toggle('fs'); document.body.classList.toggle('fs-open', !!document.querySelector('.card.fs')); const ic=btn.querySelector('.material-symbols-rounded'); if(ic) ic.textContent=c.classList.contains('fs')?'fullscreen_exit':'fullscreen'; setTimeout(()=>Object.values(S.charts).forEach(x=>x&&x.resize()),40); }

document.addEventListener('click',e=>{ const td=e.target.closest('td:last-child'); if(!td||!td.parentElement) return; if(td!==td.parentElement.lastElementChild) return; if(!td.closest('table')) return; const txt=td.textContent.trim(); if(!txt||txt==='--') return; navigator.clipboard?.writeText(txt).then(()=>{ td.classList.add('copied'); setTimeout(()=>td.classList.remove('copied'),1400); }); });

function updateConn(status,text){ const d=document.getElementById('conn'); d.className=`conn ${status||''}`; document.getElementById('connT').textContent=text; }
function updateModeBadge(mode,label){
  const el=document.getElementById('modeBadge');
  const tx=document.getElementById('modeBadgeT');
  if(!el||!tx) return;
  el.className=`mode-badge ${mode||''}`;
  tx.textContent=label;
  const icon=el.querySelector('.material-symbols-rounded');
  if(icon) icon.textContent = mode==='demo' ? 'science' : mode==='stale' ? 'wifi_off' : mode==='live' ? 'wifi' : 'wifi_tethering_off';
}
function openSettings(){ document.getElementById('settingsOv').classList.add('open'); syncSettingsUi(); }
function closeSettings(){ document.getElementById('settingsOv').classList.remove('open'); }
function setSwitchState(id, isOn){ const el=document.getElementById(id); if(!el) return; el.classList.toggle('on', !!isOn); el.setAttribute('aria-pressed', isOn ? 'true' : 'false'); }
function syncSettingsUi(){ setSwitchState('demoModeSwitch', S.demoMode); setSwitchState('autoDemoSwitch', S.autoDemoOnDisconnect); setSwitchState('freezeOnStaleSwitch', S.freezeOnStale); }
function persistDashboardPrefs(){ localStorage.setItem('rvt-dashboard-prefs', JSON.stringify({ demoMode:!!S.demoMode, autoDemoOnDisconnect:!!S.autoDemoOnDisconnect, freezeOnStale:!!S.freezeOnStale })); syncSettingsUi(); }
function loadDashboardPrefs(){
  try{ const prefs=JSON.parse(localStorage.getItem('rvt-dashboard-prefs')||'{}');
    if(typeof prefs.demoMode==='boolean') S.demoMode=prefs.demoMode;
    if(typeof prefs.autoDemoOnDisconnect==='boolean') S.autoDemoOnDisconnect=prefs.autoDemoOnDisconnect;
    if(typeof prefs.freezeOnStale==='boolean') S.freezeOnStale=prefs.freezeOnStale;
  }catch(_){ }
}
function toggleDemoMode(){
  S.demoMode=!S.demoMode; S.autoDemoActive=false; S.disc=0; persistDashboardPrefs();
  if(S.demoMode){ const p=demoPayload(); p.meta.status='demo'; p.meta.note='Demo mode enabled'; S.lastPayload=p; render(p); }
  else {
    const p=S.freezeOnStale && S.lastLivePayload ? makeStalePayload(S.lastLivePayload,'Demo mode disabled. Waiting for live data...') : makeWaitingPayload('Demo mode disabled. Waiting for live_dashboard.json...');
    S.lastPayload=p; render(p); poll();
  }
}
function toggleAutoDemo(){ S.autoDemoOnDisconnect=!S.autoDemoOnDisconnect; persistDashboardPrefs(); }
function toggleFreezeOnStale(){ S.freezeOnStale=!S.freezeOnStale; persistDashboardPrefs(); if(!S.freezeOnStale && !S.demoMode && !S.lastLivePayload) { const p=makeWaitingPayload('Freeze disabled. Waiting for live_dashboard.json...'); S.lastPayload=p; render(p); } }
function clonePayload(v){ return JSON.parse(JSON.stringify(v||{})); }
function makeWaitingPayload(msg='Waiting for live_dashboard.json...'){
  return normalize({
    meta:{status:'waiting',elapsed_s:0,remaining_s:null,session_dir:'--',note:msg},
    radar:{ rows:0, logged_hr_valid:false, logged_rr_valid:false, human:false, human_detected:0, num_targets:0, motion:'idle', last_seen_age_s:S.disc },
    ble:{ rows:0, raw_packets:0 }, thresholds:{ hr_delta_warn:3, hr_delta_crit:8, rr_delta_warn:2, rr_delta_crit:4 },
    faults:[{severity:'warn', title:'Waiting for live data', copy:msg}], events:[`[WAIT] ${msg}`], series:{ t:[] }
  });
}
function makeStalePayload(base, msg='Live poller unavailable'){
  const p=normalize(clonePayload(base || makeWaitingPayload(msg)));
  p.meta=p.meta||{}; p.radar=p.radar||{};
  p.meta.status='stale'; p.meta.note=msg;
  const age=num(p.radar.last_seen_age_s)||0;
  p.radar.last_seen_age_s=Math.max(S.hardStaleS, age + Math.max(1,S.disc));
  p.events=[`[STALE] ${msg}`].concat(Array.isArray(p.events)?p.events:[]).slice(0,80);
  return p;
}

function renderLiveCoach(p,stale,dist){
  const el=document.getElementById('liveCoach'); if(!el||!S.ctl.on) return;
  el.style.display='grid';
  const r=p.radar||{};
  let rangeCard;
  if(stale) rangeCard=flt('bad','Distance coach','Live payload is stale; do not adjust placement from expired range.');
  else if(dist===null) rangeCard=flt('warn','Distance coach','No fresh target range. Confirm the subject is in view.');
  else if(dist<30) rangeCard=flt('warn','Distance coach',`Range ${fmt(dist,0)} cm is close. Move the subject or radar farther apart.`);
  else if(dist>80) rangeCard=flt('warn','Distance coach',`Range ${fmt(dist,0)} cm is far. Move closer toward the 40 cm target.`);
  else rangeCard=flt('good','Distance coach',`Range ${fmt(dist,0)} cm is usable. Hold position.`);
  const moving=boolish(r.doppler_motion)||boolish(r.cluster_anomaly)||boolish(r.multi_target)||String(r.motion||'').toLowerCase().includes('motion');
  const motionCard=stale?flt('bad','Motion coach','Stale payload; motion status is expired.'):moving?flt('warn','Motion coach','Motion or multi-target evidence is active. Ask the subject to sit still and clear nearby reflectors.'):flt('good','Motion coach','No active motion warning. Keep posture and oximeter contact steady.');
  el.innerHTML=rangeCard+motionCard;
}

function render(p){
  const meta=p.meta||{}, radar=p.radar||{}, ble=p.ble||{}, thr=p.thresholds||{}, a=p.analysis||{};
  const pub=a.publish_reason_histogram||{};
  const stl=hardStale(p); const hasRem=meta.remaining_s!==null&&meta.remaining_s!==undefined;
  const isDemo = S.demoMode || S.autoDemoActive || String(meta.status||'').toLowerCase()==='demo';
  if(S.paused) { updateConn('warn','Paused'); updateModeBadge(isDemo?'demo':(stl?'stale':'live'), isDemo?'demo':(stl?'stale':'paused')); }
  else if(isDemo) { updateConn('warn','Demo mode'); updateModeBadge('demo', S.autoDemoActive ? 'auto demo' : 'demo'); }
  else if(stl) { updateConn('bad','Stale'); updateModeBadge('stale','stale'); }
  else if(S.disc>0) { updateConn('warn',`Retry · ${S.disc}`); updateModeBadge('stale','retry'); }
  else if(meta.status==='running') { updateConn('','Live'); updateModeBadge('live','live'); }
  else if(meta.status==='waiting') { updateConn('warn','Waiting'); updateModeBadge('', 'waiting'); }
  else { updateConn('',textOrDash(meta.status)); updateModeBadge('', textOrDash(meta.status)); }

  const ch=[];
  if(meta.status) ch.push(chip(meta.status,meta.status==='running'?'good':(String(meta.status||'').includes('stopped')?'bad':'warn'),meta.status==='running'?'play_circle':'pause_circle'));
  ch.push(chip(`${fmt(meta.elapsed_s,1)}s elapsed`,'','timer'));
  ch.push(chip(hasRem?`${fmt(meta.remaining_s,1)}s remaining`:'manual stop', hasRem&&num(meta.remaining_s)<=5?'warn':'','schedule'));
  if(S.paused) ch.push(chip('UI PAUSED','warn','pause'));
  if(S.demoMode || S.autoDemoActive || String(meta.status||'').toLowerCase()==='demo') ch.push(chip(S.autoDemoActive?'AUTO DEMO':'DEMO MODE','warn','science'));
  if(stl && S.freezeOnStale && !S.demoMode && !S.autoDemoActive) ch.push(chip('STALE · FROZEN','bad','timer_off'));
  if(boolish(radar.use_fast_path)) ch.push(chip('raw fast path','good','bolt'));
  if(num(radar.session_phase)!==null) ch.push(chip(`phase ${phaseName(radar.session_phase)}`,'','schedule'));
  if(boolish(radar.phase_valid_this_frame)) ch.push(chip('live phase valid','good','check_circle'));
  if(!boolish(radar.dsp_ran_this_frame)&&!stl) ch.push(chip('DSP idle this loop','warn','pause_circle'));
  if(boolish(radar.phase_backed_publish_ready)) ch.push(chip('HR phase-ready','good','verified'));
  if(boolish(getR(p,'rr_phase_backed_publish_ready'))) ch.push(chip('RR phase-ready','good','verified'));
  if(boolish(radar.agc_floor_suspect)) ch.push(chip('AGC floor suspect','warn','trending_down'));
  if(boolish(radar.near_field_reflector_suspect)) ch.push(chip('near-field reflector','warn','sensors'));
  if(boolish(radar.hr_anchor_drift_suspect)) ch.push(chip('anchor drift','warn','show_chart'));
  if(boolish(getR(p,'hr_freeze_suspect'))) ch.push(chip('HR stale/frozen','bad','timer_off'));
  if(boolish(radar.fs_fallback_used)) ch.push(chip('fs fallback','warn','tune'));
  if(boolish(getR(p,'rr_source_current_ok'))) ch.push(chip('RR current src','good','radio_button_checked'));
  const fsEff=num(radar.fs_effective); if(fsEff!==null&&fsEff<5.0) ch.push(chip(`low fs ${fmt(fsEff,2)}`,'warn','speed'));
  if(boolish(radar.experimental_profile_enabled)) ch.push(chip('experimental','warn','science'));
  const age=num(radar.last_seen_age_s); if(age!==null&&age>2.5) ch.push(chip(`stale ${fmt(age,1)}s`,stl?'bad':'warn',stl?'error':'warning'));
  if(stl) ch.push(chip('live values expired','bad','timer_off'));
  document.getElementById('chips').innerHTML=ch.join('');
  renderFwBadge(p);

  document.getElementById('physTable').innerHTML=[
    row('pqi_heart',fmt(getR(p,'pqi_heart'),3)),
    row('hr_autocorr_best_conf',fmt(getR(p,'hr_autocorr_best_conf'),3)),
    row('rr_spec_conf',fmt(getR(p,'rr_spec_conf'),3))
  ].join('');
  document.getElementById('polTable').innerHTML=[
    row('top_hr_publish_blocks',textOrDash(first(a.top_hr_publish_blocks,topHist(pub.hr)))),
    row('top_rr_publish_blocks',textOrDash(first(a.top_rr_publish_blocks,topHist(pub.rr)))),
    row('HR publish reason',textOrDash(radar.hr_publish_reason_name||getR(p,'hr_publish_reason'))),
    row('RR publish reason',textOrDash(radar.rr_publish_reason_name||getR(p,'rr_publish_reason')))
  ].join('');

  const radHrL=livePub(p,'reported_hr'); const radRrL=livePub(p,'reported_rr');
  const radHr=stl?null:radHrL; const radRr=stl?null:radRrL;
  const bleHr=ble.hr??lastSeries(p.series,'ble_hr'); const bleRr=ble.rr??lastSeries(p.series,'ble_rr');
  const fps=computeFps(p); const dist=num(getR(p,'distance_cm'));
  renderLiveCoach(p,stl,dist);

  document.getElementById('kpiHr').textContent=fmt(radHr,0);
  document.getElementById('kpiRr').textContent=fmt(radRr,0);
  document.getElementById('kpiFps').textContent=fmt(fps,1);
  document.getElementById('kpiDist').textContent=fmt(dist,0);

  const hrD=(num(radHr)!==null&&num(bleHr)!==null)?num(radHr)-num(bleHr):null;
  const rrD=(num(radRr)!==null&&num(bleRr)!==null)?num(radRr)-num(bleRr):null;
  document.getElementById('kpiHrSub').textContent=(bleHr!=null)?`BLE ${fmt(bleHr,0)} · Δ ${hrD!==null?(hrD>=0?'+':'')+hrD.toFixed(1):'--'}`:'awaiting reference';
  document.getElementById('kpiRrSub').textContent=(bleRr!=null)?`BLE ${fmt(bleRr,0)} · Δ ${rrD!==null?(rrD>=0?'+':'')+rrD.toFixed(1):'--'}`:'awaiting reference';
  document.getElementById('kpiFpsSub').textContent=fps!==null?`DSP loop · ${fmt(fps,1)} Hz median`:'DSP loop';
  document.getElementById('kpiDistSub').textContent=dist!==null?'from radar module':'target distance';

  document.getElementById('radHr').textContent=fmt(radHr,1);
  document.getElementById('radRr').textContent=fmt(radRr,1);
  document.getElementById('bleHr').textContent=fmt(bleHr,1);
  document.getElementById('bleRr').textContent=fmt(bleRr,1);

  const radiusCm=num(getR(p,'position_radius_cm'))??((num(getR(p,'primary_x'))!==null&&num(getR(p,'primary_y'))!==null)?Math.hypot(num(getR(p,'primary_x')),num(getR(p,'primary_y')))*100:null);
  const fwText=(()=>{ const s=textOrDash(radar.sketch_firmware_version||radar.firmware_version); const m=textOrDash(radar.module_firmware_version); if(s!=='--'&&m!=='--') return `${s} | mod ${m}`; return s!=='--'?s:m; })();
  document.getElementById('tgtCount').textContent=`Target: ${textOrDash(getR(p,'num_targets'))}`;
  document.getElementById('fpsV').textContent=fmt(fps,1);
  document.getElementById('brV').textContent=fmt(radRr,1);
  document.getElementById('hrV').textContent=fmt(radHr,1);
  document.getElementById('personV').textContent=boolish(getR(p,'human'))?'Yes':'No';
  document.getElementById('distV').textContent=fmt(getR(p,'distance_cm'),1)==='--'?'--':`${fmt(getR(p,'distance_cm'),1)} cm`;
  document.getElementById('xV').textContent=fmt(getR(p,'primary_x'),3)==='--'?'--':`${fmt(getR(p,'primary_x'),3)} m`;
  document.getElementById('yV').textContent=fmt(getR(p,'primary_y'),3)==='--'?'--':`${fmt(getR(p,'primary_y'),3)} m`;
  document.getElementById('radV').textContent=radiusCm===null?'--':`${radiusCm.toFixed(1)} cm`;
  document.getElementById('fwV').textContent=fwText;
  document.getElementById('srcV').textContent=sourceName(getR(p,'spatial_source'));
  document.getElementById('srcAgeV').textContent=fmt(getR(p,'spatial_age_ms'),0)==='--'?'--':`${fmt(getR(p,'spatial_age_ms'),0)} ms`;
  document.getElementById('dopV').textContent=textOrDash(getR(p,'primary_dop'));
  document.getElementById('dopCmsV').textContent=fmt(getR(p,'primary_dop_speed_cms'),2)==='--'?'--':`${fmt(getR(p,'primary_dop_speed_cms'),2)} cm/s`;

  document.getElementById('posTable').innerHTML=[
    row('Spatial source',sourceName(getR(p,'spatial_source'))),
    row('Target info live',boolish(getR(p,'target_info_ok'))?'Yes':'No'),
    row('Point cloud live',boolish(getR(p,'point_cloud_ok'))?'Yes':'No'),
    row('Multi-target',boolish(getR(p,'multi_target'))?'Yes':'No'),
    row('Cluster',textOrDash(getR(p,'primary_cluster'))),
    row('Spatial age',fmt(getR(p,'spatial_age_ms'),0)==='--'?'--':`${fmt(getR(p,'spatial_age_ms'),0)} ms`),
    row('Phase warmup',boolish(getR(p,'phase_warmup_complete'))?'Yes':'No'),
    row('Clutter alpha',fmt(getR(p,'current_clutter_alpha'),3)),
    row('Rewarm triggered',textOrDash(getR(p,'rewarm_triggered'))),
    row('Rewarm reason',textOrDash(getR(p,'rewarm_reason')))
  ].join('');
  drawRadar(p,stl);

  document.getElementById('radarTable').innerHTML=[
    row('Rows',textOrDash(radar.rows)),
    row('Display source',stl?'expired / stale payload':((boolish(radar.logged_hr_valid)&&num(radar.reported_hr)!==null&&num(radar.reported_hr)>0)?'validated publish':'not published')),
    row('HR publish source',textOrDash(getR(p,'hr_publish_source_class_name')||getR(p,'hr_publish_source_class'))),
    row('RR publish source',textOrDash(getR(p,'rr_publish_source_class_name')||getR(p,'rr_publish_source_class'))),
    row('Session phase',phaseName(getR(p,'session_phase'))),
    row('Harmonic mode',harmonicSum(getR(p,'harmonic_mode'))),
    row('HR triple agree',textOrDash(getR(p,'hr_triple_agree'))),
    row('RR triple agree',textOrDash(getR(p,'rr_triple_agree'))),
    row('Candidate HR',fmt(getR(p,'candidate_hr'),1)),
    row('Candidate RR',fmt(getR(p,'candidate_rr'),1)),
    row('Raw HR',fmt(getR(p,'raw_hr'),1)),
    row('Raw HR (uncorrected)',fmt(getR(p,'raw_hr_uncorrected'),1)),
    row('Raw HR (corrected)',fmt(getR(p,'raw_hr_corrected'),1)),
    row('Raw RR',fmt(getR(p,'raw_rr'),1)),
    row('PQI heart',fmt(getR(p,'pqi_heart'),2)),
    row('PQI breath',fmt(getR(p,'pqi_breath'),2)),
    row('Phase valid',boolish(getR(p,'phase_valid_this_frame'))?'Yes':'No'),
    row('Fs effective',fmt(getR(p,'fs_effective'),2)),
    row('Distance (cm)',fmt(getR(p,'distance_cm')??radar.distance_cm,1)),
    row('Motion',textOrDash(radar.motion)),
    row('Human',textOrDash(radar.human)),
    row('HR valid',textOrDash(radar.logged_hr_valid)),
    row('RR valid',textOrDash(radar.logged_rr_valid))
  ].join('');

  document.getElementById('numT').textContent=textOrDash(radar.num_targets);
  document.getElementById('maxDop').textContent=fmt(radar.max_dop_abs,1);
  document.getElementById('maxDopSp').textContent=fmt(radar.max_dop_speed_cms??getR(p,'max_dop_speed_cms'),1);
  document.getElementById('spTable').innerHTML=[
    row('Point cloud ok',textOrDash(radar.point_cloud_ok)),
    row('Target info ok',textOrDash(radar.target_info_ok)),
    row('Doppler motion',textOrDash(radar.doppler_motion)),
    row('Cluster anomaly',textOrDash(radar.cluster_anomaly)),
    row('Multi-target',textOrDash(radar.multi_target)),
    row('Primary X',fmt(radar.primary_x,2)),
    row('Primary Y',fmt(radar.primary_y,2)),
    row('Primary dop',textOrDash(radar.primary_dop)),
    row('Fast path',textOrDash(radar.use_fast_path)),
    row('Clutter alpha',fmt(radar.current_clutter_alpha,3)),
    row('HR path source',hrPathName(radar.hr_path_source,radar.hr_path_source_name))
  ].join('');

  document.getElementById('bleTable').innerHTML=[
    row('Rows',textOrDash(ble.rows)),
    row('Raw packets',textOrDash(ble.raw_packets)),
    row('SpO₂',ble.spo2===undefined?'--':`${fmt(ble.spo2,0)}%`),
    row('PI',fmt(ble.pi,1)),
    row('Address',textOrDash(ble.address)),
    row('Profile',textOrDash(ble.profile))
  ].join('');

  const hW=num(thr.hr_delta_warn)??3, hC=num(thr.hr_delta_crit)??8;
  const rW=num(thr.rr_delta_warn)??2, rC=num(thr.rr_delta_crit)??4;
  const cards=[], alerts=[];
  const add=(s,t,c)=>{ cards.push(flt(s,t,c)); alerts.push({severity:s,title:t,copy:c}); };
  if(hrD!==null){ const s=Math.abs(hrD)>hC?'bad':Math.abs(hrD)>hW?'warn':'good'; add(s,'Heart-rate mismatch',`Radar ${fmt(radHr,1)} vs BLE ${fmt(bleHr,1)} → Δ ${fmt(hrD,1)} bpm.`); }
  if(rrD!==null){ const s=Math.abs(rrD)>rC?'bad':Math.abs(rrD)>rW?'warn':'good'; add(s,'Respiration mismatch',`Radar ${fmt(radRr,1)} vs BLE ${fmt(bleRr,1)} → Δ ${fmt(rrD,1)} br/min.`); }
  if(boolish(radar.multi_target)) add('bad','Multiple targets','More than one target in the point-cloud. Host DSP may be contaminated.');
  if(boolish(radar.cluster_anomaly)) add('warn','Cluster anomaly','Point-cloud clusters disagree with the single-target assumption.');
  if(boolish(radar.doppler_motion)) add('warn','Doppler motion','Point-cloud Doppler indicates body motion.');
  if(boolish(getR(p,'hr_raw_high_bias_suspect'))) add('warn','High HR anchor bias suspected','Raw/latch HR is staying above the trusted phase anchor while lower phase-backed evidence is present.');
  if(boolish(getR(p,'hr_freeze_suspect'))) add('bad','HR stale/frozen anchor',`Anchor ${fmt(getR(p,'hr_trusted_anchor_value'),1)} bpm, trust age ${textOrDash(getR(p,'hr_trust_age_ms'))} ms.`);
  if(!boolish(getR(p,'module_fw_valid'))) add('bad','Module firmware unknown','Module firmware identity not validated. Treat cross-session comparisons cautiously.');
  if(boolish(getR(p,'buffer_zero_injected'))) add('warn','Buffer zero injection','skipDSP inserted zeros into physiological buffers this loop.');
  if(boolish(getR(p,'rewarm_triggered'))) add('warn','Clutter re-warm triggered', `Reason: ${textOrDash(getR(p,'rewarm_reason'))}`);
  const faults=Array.isArray(p.faults)?p.faults:[];
  cards.push(...faults.slice(0,2).map(f=>flt(f.severity||'warn',f.title||'Fault',f.copy||'')));
  document.getElementById('faults').innerHTML=cards.length?cards.join(''):flt('good','No active fault','No operator-facing fault in the current payload.');
  renderAlerts(alerts);

  document.getElementById('hrPills').innerHTML=[
    pill('Arbiter corrected',boolish(getR(p,'hr_arbiter_corrected'))?1:0),
    pill('Reject-phase rejected',boolish(getR(p,'hr_rejectphase_rejected'))?1:0),
    pill('Coherence rejected',boolish(getR(p,'hr_coherence_rejected'))?1:0),
    pill('Raw source',textOrDash(getR(p,'hr_raw_source'))),
    pill('Raw agree',boolish(getR(p,'hr_raw_agree'))?1:0),
    pill('Bypass active',boolish(getR(p,'hr_bypass_active'))?1:0),
    pill('Grace active',boolish(getR(p,'hr_grace_active'))?1:0),
    pill('Triple agree',boolish(getR(p,'hr_triple_agree'))?1:0),
    pill('Trust fresh',boolish(getR(p,'trusted_hr_fresh'))?1:0),
    pill('High-bias',boolish(getR(p,'hr_raw_high_bias_suspect'))?1:0)
  ].join('');
  document.getElementById('hrFunTable').innerHTML=[
    row('Agreement error (bpm)',fmt(getR(p,'hr_agree_err_bpm'),2)),
    row('HR ZC bpm/conf',`${fmt(getR(p,'hr_zc_bpm'),2)} / ${fmt(getR(p,'hr_zc_conf'),3)}`),
    row('HR spec bpm/mag',`${fmt(getR(p,'hr_spec_bpm'),2)} / ${fmt(getR(p,'hr_spec_mag'),5)}`),
    row('Reject-phase anchor',`${textOrDash(getR(p,'hr_rejectphase_anchor_used'))} · ${fmt(getR(p,'hr_rejectphase_anchor_value'),2)}`),
    row('Arbiter anchor',`${textOrDash(getR(p,'hr_arbiter_anchor_used'))} · ${fmt(getR(p,'hr_arbiter_anchor_value'),2)}`),
    row('Trusted anchor',fmt(getR(p,'hr_trusted_anchor_value'),2)),
    row('Trusted phase anchor',fmt(getR(p,'hr_trusted_phase_anchor'),2)),
    row('Anchor source',textOrDash(getR(p,'hr_anchor_source_name')||hrAnchorName(getR(p,'hr_anchor_source')))),
    row('Anchor error (bpm)',fmt(getR(p,'hr_anchor_err_bpm'),2)),
    row('Raw-anchor Δ (bpm)',fmt(getR(p,'hr_raw_minus_anchor_bpm'),2)),
    row('Raw-phase Δ (bpm)',fmt(getR(p,'hr_raw_minus_phase_bpm'),2)),
    row('HR publish source',textOrDash(getR(p,'hr_publish_source_class_name')||getR(p,'hr_publish_source_class'))),
    row('HR block stage',textOrDash(getR(p,'hr_publish_block_stage_name')||getR(p,'hr_publish_block_stage'))),
    row('Raw high-bias suspect',textOrDash(getR(p,'hr_raw_high_bias_suspect')))
  ].join('');
  document.getElementById('hrStageTable').innerHTML=[
    row('Pre reject-phase',fmt(getR(p,'hr_pre_rejectphase'),2)),
    row('Post reject-phase',fmt(getR(p,'hr_post_rejectphase'),2)),
    row('Post blend',fmt(getR(p,'hr_post_blend'),2)),
    row('Post coherence',fmt(getR(p,'hr_post_coherence'),2)),
    row('Final publish',fmt(getR(p,'hr_final_publish_candidate'),2)),
    row('Trusted phase anchor',fmt(getR(p,'hr_trusted_phase_anchor'),2)),
    row('Anchor source',textOrDash(getR(p,'hr_anchor_source_name')||hrAnchorName(getR(p,'hr_anchor_source')))),
    row('Anchor error',fmt(getR(p,'hr_anchor_err_bpm'),2)),
    row('HR ZC bpm',fmt(getR(p,'hr_zc_bpm'),2)),
    row('HR spec bpm',fmt(getR(p,'hr_spec_bpm'),2)),
    row('Logged HR valid',textOrDash(radar.logged_hr_valid)),
    row('HR publish reason',textOrDash(radar.hr_publish_reason_name))
  ].join('');

  document.getElementById('rrPills').innerHTML=[
    pill('Anchor fresh',boolish(getR(p,'rr_anchor_fresh'))?1:0),
    pill('Raw agree',boolish(getR(p,'rr_raw_agree_ok'))?1:0),
    pill('Outlier persist',textOrDash(getR(p,'rr_outlier_persist'))),
    pill('Recovery',boolish(getR(p,'rr_fundamental_recovery_triggered'))?'triggered':'no'),
    pill('Seed from raw',boolish(getR(p,'rr_seed_from_raw_used'))?1:0),
    pill('Re-anchor ok',boolish(getR(p,'rr_midsession_raw_reanchor_allowed'))?1:0),
    pill('Re-anchor blocked',boolish(getR(p,'rr_midsession_raw_reanchor_blocked'))?1:0),
    pill('Current src',boolish(getR(p,'rr_source_current_ok'))?1:0),
    pill('Latched src',boolish(getR(p,'rr_source_latched_ok'))?1:0),
    pill('Trust fresh',boolish(getR(p,'trusted_rr_fresh'))?1:0)
  ].join('');
  document.getElementById('rrFunTable').innerHTML=[
    row('Gate reason',textOrDash(radar.rr_gate_reason_name)),
    row('Publish reason',textOrDash(radar.rr_publish_reason_name)),
    row('RR ZC bpm/conf',`${fmt(getR(p,'rr_zc_bpm'),2)} / ${fmt(getR(p,'rr_zc_conf'),3)}`),
    row('RR spec bpm/conf',`${fmt(getR(p,'rr_spec_bpm'),2)} / ${fmt(getR(p,'rr_spec_conf'),3)}`),
    row('Candidate conf',fmt(getR(p,'candidate_rr_conf'),3)),
    row('Raw anchor err',fmt(getR(p,'rr_raw_anchor_err_bpm'),2)),
    row('Recovery count',textOrDash(getR(p,'rr_fundamental_recovery_count'))),
    row('Seed consistency',textOrDash(getR(p,'rr_raw_seed_consistent_count'))),
    row('Publish source',textOrDash(getR(p,'rr_publish_source_class_name'))),
    row('Block stage',textOrDash(getR(p,'rr_publish_block_stage_name')))
  ].join('');
  document.getElementById('rrStageTable').innerHTML=[
    row('Pre accept-phase',fmt(stageOr(p,'rr_pre_acceptphase',['candidate_rr']),2)),
    row('Post accept-phase',fmt(stageOr(p,'rr_post_acceptphase',['candidate_rr']),2)),
    row('Post blend',fmt(stageOr(p,'rr_post_blend',['reported_rr']),2)),
    row('Post bias correction',fmt(stageOr(p,'rr_post_bias_correction',['rr_post_blend','reported_rr']),2)),
    row('Post Kalman',rrStage(p,'rr_post_kalman',['reported_rr'])===null?'--':fmt(rrStage(p,'rr_post_kalman',['reported_rr']),2)),
    row('Final publish',fmt(stageOr(p,'rr_final_publish_candidate',['reported_rr']),2)),
    row('Anchor value',fmt(getR(p,'rr_anchor_value'),2)),
    row('Anchor age (ms)',textOrDash(getR(p,'rr_anchor_age_ms'))),
    row('Anchor confidence',fmt(getR(p,'rr_anchor_confidence'),2)),
    row('RR trust age (ms)',textOrDash(getR(p,'rr_trust_age_ms')))
  ].join('');

  const rrF=[];
  if(!boolish(getR(p,'rr_anchor_fresh'))) rrF.push(flt('warn','RR anchor stale','The RR anchor is not fresh. Recovery and publish may rely on stale state.'));
  const rAge=num(getR(p,'candidate_rr_age_ms')); if(rAge!==null&&rAge>10000) rrF.push(flt('warn','RR candidate aging','RR candidate older than 10 s.'));
  if(boolish(getR(p,'rr_midsession_raw_reanchor_blocked'))) rrF.push(flt('warn','RR re-anchor blocked',`Reason: ${textOrDash(getR(p,'rr_midsession_raw_reanchor_reason'))}`));
  if(boolish(getR(p,'rr_fundamental_recovery_triggered'))) rrF.push(flt('good','RR recovery active','Fundamental recovery triggered.'));
  if(stl) rrF.push(flt('bad','Dashboard payload expired','Live values blanked; telemetry age exceeded stale timeout.'));
  document.getElementById('rrFaults').innerHTML=rrF.length?rrF.join(''):flt('good','No RR warning','No RR-specific warning in the current payload.');

  const cg=a.ml_gate_combined||a.ml_gate||{}; const lg=a.ml_gate_locked||{}; const sg=a.ml_gate_settling||{};
  const pg=a.ml_gate||{}; const pStat=pg.status||(pg.passed?'PASS':'FAIL');
  const gaud=a.gate_audit||{}; const fn=gaud.funnel||{}; const fwt=a.fw_truthfulness||{}; const agc=a.agc_anomaly_flags||{};
  const xa=a.experimental_audit||{}; const xb=xa.combined_without_early_or_suspect_half_rate||{};
  const bq=a.ble_ref_quality||ble.quality||{};
  document.getElementById('audTable').innerHTML=[
    row('Sketch truthfulness',textOrDash(fwt.version)),
    row('Module truthfulness',textOrDash(fwt.module_version)),
    row('Module fw valid',textOrDash(fwt.module_version_valid)),
    row('Primary gate',`${textOrDash(pStat)} · r=${fmt(pg.r,3)} · rmse=${fmt(pg.rmse,2)} · n=${textOrDash(pg.n)}`),
    row('Combined gate',`${textOrDash(cg.passed?'PASS':'FAIL')} · r=${fmt(cg.r,3)} · rmse=${fmt(cg.rmse,2)} · n=${textOrDash(cg.n)}`),
    row('Locked gate',`${textOrDash(lg.passed?'PASS':'FAIL')} · r=${fmt(lg.r,3)} · rmse=${fmt(lg.rmse,2)} · n=${textOrDash(lg.n)}`),
    row('Settling gate',`${textOrDash(sg.passed?'PASS':'FAIL')} · r=${fmt(sg.r,3)} · rmse=${fmt(sg.rmse,2)} · n=${textOrDash(sg.n)}`),
    row('Coverage split',`locked ${fmt(a.coverage_locked,1)}% · settling ${fmt(a.coverage_settling,1)}%`),
    row('Funnel',`frames ${textOrDash(fn.radar_frames_total)} · human ${textOrDash(fn.human_frames)} · dsp ${textOrDash(fn.dsp_frames)} · phase ${textOrDash(fn.phase_valid_frames)}`),
    row('Eval bins',`HR ${textOrDash(fn.hr_eval_bins)} · RR ${textOrDash(fn.rr_eval_bins)}`)
  ].join('');
  document.getElementById('histTable').innerHTML=[
    row('Top HR publish blocks',topHist(pub.hr)),
    row('Top RR publish blocks',topHist(pub.rr)),
    row('Top HR gate reasons',topHist(a.hr_gate_reason_histogram)),
    row('Top RR gate reasons',topHist(a.rr_gate_reason_histogram)),
    row('AGC anomaly',`floor ${fmt(agc.gain_floor_pct,1)}% · near ${fmt(agc.near_field_pct,1)}% · skipdsp ${fmt(agc.skipdsp_pct,1)}%`),
    row('Experimental best',`r=${fmt(xb.r,3)} · rmse=${fmt(xb.rmse,2)} · n=${textOrDash(xb.n)}`)
  ].join('');
  document.getElementById('bleQTable').innerHTML=[
    row('Status',textOrDash(bq.status)),
    row('Raw packets',textOrDash(bq.raw_packets??ble.raw_packets)),
    row('Parsed rows',textOrDash(bq.parsed_rows??ble.rows)),
    row('Distilled rows',`${fmt(bq.distilled_rows_pct_of_raw,1)}% of raw`),
    row('Packet loss',`${fmt(bq.packet_loss_pct,1)}%`),
    row('Decode error',`${fmt(bq.decode_error_pct,1)}%`),
    row('PI below threshold',`${fmt(bq.pi_below_threshold_pct,1)}% @ ${fmt(bq.pi_threshold,1)}`)
  ].join('');

  document.getElementById('evLog').textContent=(p.events||[]).length?p.events.join('\n'):'Waiting for events...';
  document.getElementById('foot').innerHTML=`<span class="material-symbols-rounded">folder_open</span>Session folder: ${esc(meta.session_dir||'--')}`;

  updateCharts(p);
}

function demoPayload(){
  S.demoT+=1; const t=S.demoT; const N=120; const ts=Array.from({length:N},(_,i)=>i);
  const hrBase=72+4*Math.sin(t/12); const rrBase=15+1.5*Math.sin(t/18+1);
  const rep_hr=ts.map((_,i)=>hrBase+2*Math.sin(i/3+t/5)+(Math.random()-.5));
  const can_hr=rep_hr.map(v=>v+(Math.random()-.5)*1.2);
  const raw_hr=rep_hr.map(v=>v+(Math.random()-.5)*3.5+1.2);
  const raw_hr_corrected=raw_hr.map(v=>v-(Math.max(0, Math.min(25, 0.4*(v-55)))));
  const ble_hr=rep_hr.map(v=>v+(Math.random()-.5)*0.7-0.3);
  const rep_rr=ts.map((_,i)=>rrBase+0.4*Math.sin(i/4+t/6)+(Math.random()-.5)*.15);
  const can_rr=rep_rr.map(v=>v+(Math.random()-.5)*.3);
  const ble_rr=rep_rr.map(v=>v+(Math.random()-.5)*.2);
  const breath=ts.map((_,i)=>Math.sin((i+t)*0.12)+0.1*Math.sin((i+t)*0.5));
  const heart=ts.map((_,i)=>Math.sin((i+t)*0.9)*0.6+0.15*Math.sin((i+t)*3.5));
  const x=0.35+0.18*Math.sin(t/14); const y=1.85+0.22*Math.cos(t/16); const distCm=Math.hypot(x,y)*100;
  return normalize({
    meta:{status:'running',elapsed_s:t,remaining_s:Math.max(0,320-t),session_dir:'/sessions/demo_2026_04_18'},
    radar:{
      rows:1450+t, reported_hr:rep_hr[rep_hr.length-1], reported_rr:rep_rr[rep_rr.length-1],
      candidate_hr:can_hr[can_hr.length-1], candidate_rr:can_rr[can_rr.length-1],
      raw_hr:raw_hr[raw_hr.length-1], raw_hr_uncorrected:raw_hr[raw_hr.length-1], raw_hr_corrected:raw_hr_corrected[raw_hr_corrected.length-1], raw_rr:rep_rr[rep_rr.length-1]+0.5,
      logged_hr_valid:true, logged_rr_valid:true, num_targets:1,
      primary_x:x, primary_y:y, distance_cm:distCm, primary_dop:3, primary_dop_speed_cms:1.47,
      fps_hz:20, fs_effective:19.8,
      sketch_firmware_version:'v15.1.0', module_firmware_version:'2.1.0', module_fw_valid:true,
      spatial_source:1, spatial_age_ms:40, target_info_ok:true, point_cloud_ok:true, multi_target:false,
      session_phase:3, phase_valid_this_frame:true, phase_backed_publish_ready:true, rr_phase_backed_publish_ready:true,
      human:true, motion:'still', max_dop_abs:3, max_dop_speed_cms:1.5,
      use_fast_path:false, phase_warmup_complete:true, current_clutter_alpha:0.085, dsp_ran_this_frame:true,
      pqi_heart:0.82+0.05*Math.sin(t/10), pqi_breath:0.88, hr_autocorr_best_conf:0.74, rr_spec_conf:0.81,
      hr_triple_agree:true, rr_triple_agree:true,
      hr_publish_reason_name:'published', rr_publish_reason_name:'published',
      hr_publish_source_class_name:'phase_anchor', rr_publish_source_class_name:'accepted',
      hr_trusted_phase_anchor:hrBase, hr_trusted_anchor_value:hrBase,
      hr_anchor_source_name:'trusted_phase', hr_anchor_err_bpm:0.4, hr_raw_minus_anchor_bpm:1.2, hr_raw_minus_phase_bpm:0.9,
      hr_zc_bpm:hrBase, hr_zc_conf:0.7, hr_spec_bpm:hrBase+0.3, hr_spec_mag:0.0032,
      trusted_hr_fresh:true, rr_anchor_fresh:true, rr_source_current_ok:true, rr_source_latched_ok:true, trusted_rr_fresh:true,
      rr_anchor_value:rrBase, rr_anchor_confidence:0.83, candidate_rr_conf:0.82,
      rr_fundamental_recovery_count:0, rr_raw_seed_consistent_count:5,
      hr_raw_source:'combined', last_seen_age_s:0.1, harmonic_mode:2, hr_path_source_name:'auto publish'
    },
    ble:{rows:320,raw_packets:340,spo2:98,pi:3.2,address:'BA:DC:0F:FE:EE:00',profile:'demo',hr:ble_hr[ble_hr.length-1],rr:ble_rr[ble_rr.length-1]},
    thresholds:{hr_delta_warn:3,hr_delta_crit:8,rr_delta_warn:2,rr_delta_crit:4},
    ml_readiness_verdict:{verdict:'ready',readiness_kind:'ready',headline:'Demo signal is ready',limitation_kind:'none',next_action:'review',passed:['Corrected raw HR is flowing','RR remains stable'],failed:[],categories:[{id:'schema',status:'pass',label:'Schema',detail:'Demo payload carries the v11/v15-ready summary fields.'}]},
    faults:[],
    events:[`[${t}] sync ok`,`[${t-1}] publish_hr=${rep_hr[rep_hr.length-1].toFixed(1)}`,`[${t-2}] publish_rr=${rep_rr[rep_rr.length-1].toFixed(1)}`,`[${t-3}] anchor=trusted_phase`],
    series:{
      t:ts, reported_hr:rep_hr, candidate_hr:can_hr, raw_hr, raw_hr_uncorrected:raw_hr, raw_hr_corrected, ble_hr,
      reported_rr:rep_rr, candidate_rr:can_rr, ble_rr,
      rr_final_publish_candidate:rep_rr, rr_anchor_value:ts.map(()=>rrBase),
      hr_trusted_phase_anchor:ts.map(()=>hrBase), breath_phase:breath, heart_phase:heart,
      candidate_rr_conf:ts.map((_,i)=>0.7+0.15*Math.sin(i/5+t/10)),
      rr_fundamental_recovery_count:ts.map((_,i)=>i>60?1:0),
      rr_raw_seed_consistent_count:ts.map((_,i)=>Math.min(5,Math.floor(i/20))),
      radar_hr_valid:ts.map(()=>true), radar_rr_valid:ts.map(()=>true)
    },
    analysis:{
      publish_reason_histogram:{hr:{'published':320,'STALE_FROZEN':4},rr:{'published':318,'LATCH_STALE':3}},
      hr_gate_reason_histogram:{'pass':310,'low_pqi':8}, rr_gate_reason_histogram:{'pass':305,'outlier':12},
      ml_gate:{passed:true,r:0.962,rmse:2.14,n:310,status:'PASS'},
      ml_gate_combined:{passed:true,r:0.958,rmse:2.22,n:308},
      ml_gate_locked:{passed:true,r:0.973,rmse:1.86,n:240},
      ml_gate_settling:{passed:true,r:0.91,rmse:2.82,n:68},
      coverage_locked:78, coverage_settling:22,
      fw_truthfulness:{version:'v15.1.0',module_version:'2.1.0',module_version_valid:true},
      raw_hr_correction_coverage_pct:95.1,
      raw_hr_corrected_bias_bpm:0.6,
      raw_hr_uncorrected_bias_bpm:2.4,
      hr_rescue_publish_count:22,
      hr_rescue_publish_coverage_pct:8.1,
      oracle_audit:{
        hr_oracle_candidate_mae_bpm:1.8,
        hr_oracle_candidate_coverage_pct:82.4,
        rr_oracle_candidate_mae_bpm:0.7,
        rr_oracle_candidate_coverage_pct:88.1,
        hr_best_candidate_source_histogram:{reported_hr:180,candidate_hr:90,raw_hr_corrected:120,hr_spec:45,hr_zc:25},
        rr_best_candidate_source_histogram:{reported_rr:210,candidate_rr:60,raw_rr_effective:40,rr_spec:20,rr_zc:10}
      },
      coverage_loss_ledger:{
        hr_coverage_loss_by_stage:{NO_PHASE:84,RAW_PHASE_STALE_NO_COMPARISON:61,LOW_PQI:12,RAW_DISAGREE_TRUE:8},
        rr_coverage_loss_by_stage:{LOW_PQI:18,STALE_TRACK:9,PUBLISH_POLICY:4},
        hr_salvageable_frames_pct:68.2,
        rr_salvageable_frames_pct:74.9
      },
      agc_anomaly_flags:{gain_floor_pct:1.2,near_field_pct:0.4,skipdsp_pct:0.9},
      gate_audit:{funnel:{radar_frames_total:3400,human_frames:3100,dsp_frames:3080,phase_valid_frames:2900,hr_eval_bins:280,rr_eval_bins:275}},
      experimental_audit:{combined_without_early_or_suspect_half_rate:{r:0.95,rmse:2.3,n:295}},
      ble_ref_quality:{status:'good',raw_packets:340,parsed_rows:320,distilled_rows_pct_of_raw:94.1,packet_loss_pct:2.3,decode_error_pct:0.2,pi_below_threshold_pct:1.8,pi_threshold:0.5},
      raw_hr_bias_audit:{ rate_buckets:[
        {bucket:'50-59',ref_hr_min:50,ref_hr_max:59,bias_bpm:0.8,n:24},
        {bucket:'60-69',ref_hr_min:60,ref_hr_max:69,bias_bpm:1.2,n:58},
        {bucket:'70-79',ref_hr_min:70,ref_hr_max:79,bias_bpm:1.6,n:92},
        {bucket:'80-89',ref_hr_min:80,ref_hr_max:89,bias_bpm:2.1,n:71},
        {bucket:'90-99',ref_hr_min:90,ref_hr_max:99,bias_bpm:1.4,n:35}
      ]}
    }
  });
}

async function poll(){
  if(S.paused) return;
  if(S.demoMode){
    S.autoDemoActive=false;
    const p=demoPayload(); p.meta.status='demo'; p.meta.note='Explicit demo mode'; S.lastPayload=p; S.disc=0; render(p); return;
  }
  try{
    const res=await fetch(`${S.dataUrl}?t=${Date.now()}`,{cache:'no-store'});
    if(!res.ok) throw new Error('HTTP '+res.status);
    const p=normalize(await res.json());
    S.lastPayload=p; S.lastLivePayload=clonePayload(p); S.disc=0; S.autoDemoActive=false; render(p);
  }catch(e){
    S.disc+=1;
    if(S.autoDemoOnDisconnect && S.disc>=2){
      S.autoDemoActive=true;
      const p=demoPayload(); p.meta.status='demo'; p.meta.note='Auto demo on disconnect'; p.meta.auto_demo=true; S.lastPayload=p; render(p);
      return;
    }
    if(S.freezeOnStale && S.lastLivePayload){
      const p=makeStalePayload(S.lastLivePayload, 'No fresh data from trainer/live poller'); S.lastPayload=p; render(p);
      return;
    }
    const p=makeWaitingPayload('No live data from trainer/live poller'); S.lastPayload=p; render(p);
  }
}

async function init(){
  const th=localStorage.getItem('rvt-theme'); if(th==='dark'){ document.documentElement.setAttribute('data-theme','dark'); document.getElementById('themeIcon').textContent='light_mode'; }
  const dn=localStorage.getItem('rvt-density'); if(dn) setDensity(dn);
  loadDashboardPrefs();
  if(await detectControlMode()) setupControlMode();
  buildCharts();
  const seed = S.demoMode ? demoPayload() : makeWaitingPayload('Waiting for live_dashboard.json...');
  if(S.demoMode) seed.meta.status='demo';
  S.lastPayload=seed; render(seed); S.disc=0; syncSettingsUi();
  poll();
  setInterval(()=>{ poll(); if(S.ctl.on) refreshSessionHeader(); }, S.refreshMs);
  setInterval(()=>{ if(S.lastPayload && (S.demoMode || !hardStale(S.lastPayload))) drawRadar(S.lastPayload,hardStale(S.lastPayload)); }, 80);
  renderSnaps();
}
window.addEventListener('DOMContentLoaded', init);
</script>
</body>
</html>
'''

_DASHBOARD_HTML_NAME = "radar_vital_live_dashboard_v12_for_v16_0.html"
_LEGACY_DASHBOARD_HTML_NAMES = (
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


def _iso_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _json_safe_response(obj) -> bytes:
    def clean(v):
        if isinstance(v, bool) or v is None or isinstance(v, str):
            return v
        if isinstance(v, dict):
            return {k: clean(val) for k, val in v.items()}
        if isinstance(v, (list, tuple)):
            return [clean(val) for val in v]
        if isinstance(v, (np.floating, float)):
            # Coerce NaN *and* ±Inf to null: json.dumps with the default
            # allow_nan=True would otherwise emit bare `NaN`/`Infinity` tokens,
            # which are invalid JSON and break strict client parsers.
            return float(v) if np.isfinite(v) else None
        if isinstance(v, (np.integer, int)):
            return int(v)
        return v
    return json.dumps(clean(obj), ensure_ascii=False, allow_nan=False).encode("utf-8")


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


def _lock_path(sessions_root: str) -> Path:
    return Path(sessions_root) / ".session.lock"


def _write_session_lock(sessions_root: str, session_dir: str, pid: Optional[int] = None) -> Dict[str, object]:
    root = Path(sessions_root)
    root.mkdir(parents=True, exist_ok=True)
    data = {"pid": int(pid or os.getpid()), "session_dir": str(session_dir), "started_at": _iso_now()}
    path = _lock_path(str(root))
    try:
        with path.open("x", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    except FileExistsError:
        if not _check_stale_session_lock(str(root)):
            raise
        with path.open("x", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    return data


def _read_session_lock(sessions_root: str) -> Optional[Dict[str, object]]:
    path = _lock_path(sessions_root)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def _release_session_lock(sessions_root: str) -> None:
    try:
        _lock_path(sessions_root).unlink()
    except FileNotFoundError:
        pass


def _check_stale_session_lock(sessions_root: str) -> bool:
    data = _read_session_lock(sessions_root)
    if not data:
        path = _lock_path(sessions_root)
        if path.exists():
            _release_session_lock(sessions_root)
            return True
        return False
    if _pid_alive(data.get("pid")):
        return False
    _release_session_lock(sessions_root)
    return True


def _session_is_active(sessions_root: str) -> bool:
    data = _read_session_lock(sessions_root)
    if not data:
        path = _lock_path(sessions_root)
        if path.exists():
            _release_session_lock(sessions_root)
        return False
    if _pid_alive(data.get("pid")):
        return True
    _release_session_lock(sessions_root)
    return False


def _preflight_result(check_id: str, label: str, status: str, detail: str,
                      remediation: str = "", start_t: Optional[float] = None) -> Dict[str, object]:
    elapsed_ms = 0.0 if start_t is None else max(0.0, (time.monotonic() - start_t) * 1000.0)
    return {
        "id": check_id,
        "label": label,
        "status": status if status in {"ok", "warn", "fail", "skip"} else "fail",
        "detail": str(detail),
        "remediation": str(remediation or ""),
        "duration_ms": round(elapsed_ms, 1),
    }


def _candidate_ino_paths(ino_search_paths: Optional[Sequence[str]] = None) -> List[Path]:
    if ino_search_paths:
        out: List[Path] = []
        for root in ino_search_paths:
            p = Path(root)
            if p.is_file():
                out.append(p)
            elif p.exists():
                out.extend(sorted(p.glob("*.ino")))
        return out
    return [_REPO_ROOT / "radar_vital_v16_3_0.ino"] + _firmware_contract_candidates()


from rvt_trainer.audit.runner import (  # noqa: E402
    run_preflight_all as _run_preflight_all,
    run_preflight_check as _run_preflight_check,
)


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
    out = {
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

    needed = ("raw_hr_uncorrected", "raw_hr_corrected", "ref_hr")
    if not all(c in df.columns for c in needed):
        out["recommendation"] = (
            f"Adaptive correction shadow skipped: missing one of {needed}."
        )
        return out

    motion_col = None
    for c in ("in_motion", "is_motion", "motion_active"):
        if c in df.columns:
            motion_col = c
            break

    raw_unc = _pd.to_numeric(df["raw_hr_uncorrected"], errors="coerce")
    raw_cor = _pd.to_numeric(df["raw_hr_corrected"], errors="coerce")
    ref_hr  = _pd.to_numeric(df["ref_hr"], errors="coerce")
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

    raw_unc_p = _pd.to_numeric(paired["raw_hr_uncorrected"], errors="coerce").to_numpy(dtype=float)
    raw_cor_p = _pd.to_numeric(paired["raw_hr_corrected"], errors="coerce").to_numpy(dtype=float)
    ref_hr_p  = _pd.to_numeric(paired["ref_hr"], errors="coerce").to_numpy(dtype=float)

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
    out = {
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
        out["recommendation"] = "v15 PQI columns not present in CSV (firmware <v15.0). No shadow data."
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
    mfic = out["motion_frame_inflation_check"]
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
    gate = analyse_summary.get("ml_gate") if isinstance(analyse_summary.get("ml_gate"), dict) else {}
    secondary_gate = gate.get("secondary_gate") if isinstance(gate.get("secondary_gate"), dict) else {}
    ble = analyse_summary.get("ble_ref_quality") if isinstance(analyse_summary.get("ble_ref_quality"), dict) else {}
    fw = analyse_summary.get("fw_truthfulness") if isinstance(analyse_summary.get("fw_truthfulness"), dict) else {}
    pqi = _to_num(analyse_summary.get("pqi_lock_pct"), float("nan"))
    coverage = _to_num(ble.get("distilled_rows_pct_of_raw", ble.get("coverage_pct")), float("nan"))
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
    sketch_version = fw.get("version")
    schema_hash = analyse_summary.get("feature_schema_hash")
    schema_hash_ok = not bool(schema_hash) or schema_hash == feature_schema_hash()
    secondary_kind = str(secondary_gate.get("kind", "")).lower()
    secondary_basis = str(secondary_gate.get("basis", ""))
    hr_base = analyse_summary.get("hr_baseline", {}) if isinstance(analyse_summary.get("hr_baseline"), dict) else {}
    rr_base = analyse_summary.get("rr_baseline", {}) if isinstance(analyse_summary.get("rr_baseline"), dict) else {}
    hr_valid = hr_base.get("valid_only", {}) if isinstance(hr_base.get("valid_only"), dict) else {}
    rr_valid = rr_base.get("valid_only", {}) if isinstance(rr_base.get("valid_only"), dict) else {}
    hr_cov = _to_num(hr_base.get("coverage_pct"), float("nan"))
    rr_cov = _to_num(rr_base.get("coverage_pct"), float("nan"))
    hr_mae = _to_num(hr_valid.get("mae"), float("nan"))
    hr_bias = _to_num(hr_valid.get("bias"), float("nan"))
    rr_mae = _to_num(rr_valid.get("mae"), float("nan"))
    rr_bias = _to_num(rr_valid.get("bias"), float("nan"))
    publish_hist = analyse_summary.get("publish_reason_histogram") if isinstance(analyse_summary.get("publish_reason_histogram"), dict) else {}
    publish_hr = publish_hist.get("hr") if isinstance(publish_hist.get("hr"), dict) else publish_hist
    publish_rr = publish_hist.get("rr") if isinstance(publish_hist.get("rr"), dict) else publish_hist
    total_publish = sum(_safe_int(v, 0) for v in list(publish_hr.values()) + list(publish_rr.values())) if isinstance(publish_hist, dict) else 0
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

    raw = analyse_summary.get("raw_hr_bias_audit") if isinstance(analyse_summary.get("raw_hr_bias_audit"), dict) else {}
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


def _download_items_for_session(session_dir: str) -> List[Dict[str, str]]:
    root = Path(session_dir).resolve()
    candidates = [
        "radar.csv", "ref.csv", "ref_ble_raw.csv", "live_dashboard.json", "live_dashboard.html", "dashboard.json",
        "session_manifest.json", "session_quick_report.txt", "quick_report.txt",
        "analysis/analyse_summary.json", "analysis/analyse_report.html", "analysis/analyse_report.txt",
        "analyse_summary.json", "analyse_report.html", "analyse_report.txt",
        "analysis/aligned_1hz.csv", "analysis/aligned_1hz_features.csv",
        "analysis/analyse_hr_overlay.png", "analysis/analyse_rr_overlay.png",
        "analysis/raw_hr_bias_by_bucket.png",
    ]
    out = []
    for rel in candidates:
        p = (root / rel).resolve()
        try:
            p.relative_to(root)
        except ValueError:
            continue
        if p.exists() and p.is_file():
            out.append({"label": rel, "relpath": rel, "path": str(p), "href": f"/api/sessions/{root.name}/files/{rel}"})
    return out


def _metric_summary(block: Optional[Dict[str, object]]) -> Dict[str, object]:
    block = block if isinstance(block, dict) else {}
    vo = block.get("valid_only", {}) if isinstance(block.get("valid_only"), dict) else {}
    all_frames = block.get("all_frames", {}) if isinstance(block.get("all_frames"), dict) else {}
    return {
        "coverage_pct": block.get("coverage_pct"),
        "valid_only": vo,
        "all_frames": all_frames,
        "n": vo.get("n", all_frames.get("n")),
        "rmse": vo.get("rmse"),
        "mae": vo.get("mae"),
        "bias": vo.get("bias"),
        "r": vo.get("r"),
    }


def _top_hist_items(hist, limit: int = 5) -> List[Dict[str, object]]:
    if not isinstance(hist, dict):
        return []
    return [
        {"reason": str(k), "count": v}
        for k, v in sorted(hist.items(), key=lambda item: _safe_int(item[1], 0), reverse=True)[:limit]
    ]


def _summary_value(*values):
    for v in values:
        if v not in (None, ""):
            return v
    return None


def _contract_diagnosis(analysis: Optional[Dict[str, object]], manifest: Optional[Dict[str, object]]) -> Dict[str, object]:
    analysis = analysis if isinstance(analysis, dict) else {}
    manifest = manifest if isinstance(manifest, dict) else {}
    fw = analysis.get("fw_truthfulness") if isinstance(analysis.get("fw_truthfulness"), dict) else {}
    observed_len = _safe_int(fw.get("contract_length", EXPECTED_RADAR_LOG_COLUMN_COUNT), EXPECTED_RADAR_LOG_COLUMN_COUNT)
    sketch = _summary_value(fw.get("version"), manifest.get("sketch_version"))
    schema_hash = _summary_value(analysis.get("feature_schema_hash"), manifest.get("feature_schema_hash"))
    expected_schema_hash = feature_schema_hash()
    mismatches = []
    if not _is_supported_radar_contract_length(observed_len):
        mismatches.append({
            "field": "contract_length",
            "expected": EXPECTED_RADAR_LOG_COLUMN_COUNT,
            "actual": observed_len,
            "remediation": "Use the v15.1 firmware/trainer CSV contract before training from this session.",
        })
    if schema_hash and schema_hash != expected_schema_hash:
        mismatches.append({
            "field": "feature_schema_hash",
            "expected": expected_schema_hash,
            "actual": schema_hash,
            "remediation": "Rerun analysis with the current trainer or keep this session with its original model lineage.",
        })
    if sketch and str(sketch) not in {FIRMWARE_VERSION_EXPECTED, FIRMWARE_VERSION_EXPECTED.lstrip("v")}:
        mismatches.append({
            "field": "sketch_version",
            "expected": FIRMWARE_VERSION_EXPECTED,
            "actual": sketch,
            "remediation": "Reflash or select sessions collected with the expected v15.1 firmware.",
        })
    status = "mismatch" if mismatches else ("unknown" if not analysis else "ok")
    return {
        "status": status,
        "expected_contract_length": EXPECTED_RADAR_LOG_COLUMN_COUNT,
        "observed_contract_length": observed_len if analysis else None,
        "expected_schema_hash": expected_schema_hash,
        "observed_schema_hash": schema_hash,
        "expected_firmware": FIRMWARE_VERSION_EXPECTED,
        "observed_firmware": sketch,
        "mismatches": mismatches,
        "remediation": "Do not use mismatched sessions for ML training." if mismatches else "",
    }


def _load_session_summary(session_dir: str) -> Dict[str, object]:
    root = Path(session_dir)
    manifest = _read_json_if_exists(str(root / "session_manifest.json"))
    manifest = manifest if isinstance(manifest, dict) else None
    analysis_path = root / "analysis" / "analyse_summary.json"
    if not analysis_path.exists():
        analysis_path = root / "analyse_summary.json"
    analysis = _read_json_if_exists(str(analysis_path))
    ble_summary = _read_json_if_exists(str(root / "ref_ble_summary.json"))
    ble_summary = ble_summary if isinstance(ble_summary, dict) else None
    session_id = ((manifest or {}).get("session_id") if isinstance(manifest, dict) else None) or root.name
    downloads = _download_items_for_session(str(root))
    base = {
        "session_id": session_id,
        "session_dir": str(root.resolve()),
        "manifest": manifest,
        "subject_label": (manifest or {}).get("subject_label"),
        "subject": (manifest or {}).get("subject_label"),
        "operator_label": (manifest or {}).get("operator_label"),
        "operator": (manifest or {}).get("operator_label"),
        "notes": (manifest or {}).get("notes"),
        "started_at": (manifest or {}).get("started_at"),
        "ended_at": (manifest or {}).get("ended_at"),
        "duration_s": (manifest or {}).get("duration_s"),
        "started_from": (manifest or {}).get("started_from"),
        "trainer_version": (manifest or {}).get("trainer_version", VERSION),
        "dashboard_version": (manifest or {}).get("dashboard_version", DASHBOARD_VERSION),
        "downloads": downloads,
    }
    if not isinstance(analysis, dict):
        base.update({
            "status": "incomplete",
            "analysis_status": "no_analysis",
            "analysis": None,
            "verdict": None,
            "ml_readiness_verdict": _build_ml_readiness_verdict(None),
            "contract_diagnosis": _contract_diagnosis(None, manifest),
            "warnings": ["Analysis summary is not available yet."],
        })
        return base
    verdict_obj = analysis.get("ml_readiness_verdict")
    if not isinstance(verdict_obj, dict):
        verdict_obj = _build_ml_readiness_verdict(analysis)
    hr = _metric_summary(analysis.get("hr_baseline"))
    rr = _metric_summary(analysis.get("rr_baseline"))
    ble_quality = analysis.get("ble_ref_quality") if isinstance(analysis.get("ble_ref_quality"), dict) else (ble_summary or {})
    fw = analysis.get("fw_truthfulness") if isinstance(analysis.get("fw_truthfulness"), dict) else {}
    signal_quality = {
        "pqi_lock_pct": analysis.get("pqi_lock_pct"),
        "coverage_locked": analysis.get("coverage_locked"),
        "coverage_settling": analysis.get("coverage_settling"),
        "session_quality_score": analysis.get("session_quality_score"),
        "internal_consistency_score": analysis.get("internal_consistency_score"),
        "agc_anomaly_flags": analysis.get("agc_anomaly_flags"),
        "gate_audit": analysis.get("gate_audit"),
    }
    reference_quality = dict(ble_quality or {})
    gates = {
        "primary": analysis.get("ml_gate"),
        "secondary": analysis.get("ml_gate_secondary"),
        "combined": analysis.get("ml_gate_combined", analysis.get("ml_gate")),
        "locked": analysis.get("ml_gate_locked"),
        "settling": analysis.get("ml_gate_settling"),
        "golden_check": analysis.get("golden_check"),
    }
    publish_hist = analysis.get("publish_reason_histogram") if isinstance(analysis.get("publish_reason_histogram"), dict) else {}
    publish_hr = publish_hist.get("hr") if isinstance(publish_hist.get("hr"), dict) else publish_hist
    publish_rr = publish_hist.get("rr") if isinstance(publish_hist.get("rr"), dict) else publish_hist
    histograms = {
        "publish_reason_histogram": publish_hist,
        "hr_gate_reason_histogram": analysis.get("hr_gate_reason_histogram"),
        "rr_gate_reason_histogram": analysis.get("rr_gate_reason_histogram"),
        "top_hr_publish": _top_hist_items(publish_hr),
        "top_rr_publish": _top_hist_items(publish_rr),
        "top_hr_gate": _top_hist_items(analysis.get("hr_gate_reason_histogram")),
        "top_rr_gate": _top_hist_items(analysis.get("rr_gate_reason_histogram")),
    }
    base.update({
        "status": "complete",
        "analysis_status": "complete",
        "analysis": analysis,
        "verdict": verdict_obj.get("verdict") if isinstance(verdict_obj, dict) else None,
        "ml_readiness_verdict": verdict_obj,
        "hr_baseline": analysis.get("hr_baseline"),
        "rr_baseline": analysis.get("rr_baseline"),
        "hr_metrics": hr,
        "rr_metrics": rr,
        "hr_rmse": hr.get("rmse"),
        "hr_r": hr.get("r"),
        "rr_rmse": rr.get("rmse"),
        "rr_r": rr.get("r"),
        "ble_ref_quality": ble_quality,
        "signal_quality": signal_quality,
        "reference_quality": reference_quality,
        "gates": gates,
        "histograms": histograms,
        "truthfulness": {
            "sketch_fw": _summary_value(fw.get("version"), fw.get("sketch_version")),
            "module_fw": fw.get("module_version"),
            "module_version_valid": fw.get("module_version_valid"),
            "notes": fw.get("notes"),
            "contract_length": fw.get("contract_length", EXPECTED_RADAR_LOG_COLUMN_COUNT),
            "schema_hash": _summary_value(analysis.get("feature_schema_hash"), (manifest or {}).get("feature_schema_hash")),
            "scoring_weights_hash": _summary_value(analysis.get("scoring_weights_hash"), (manifest or {}).get("scoring_weights_hash")),
            "fw_truthfulness": fw,
        },
        "contract_diagnosis": _contract_diagnosis(analysis, manifest),
        "analysis_payload": _dashboard_analysis_payload(str(root)),
        "raw_hr_bias_audit": analysis.get("raw_hr_bias_audit", analysis.get("raw_hr_bias_estimate")),
        "review_required": analysis.get("review_required"),
        "review_flags": analysis.get("review_flags", []),
    })
    return base


def _scan_sessions_root(root: str, prefix: str = "s") -> List[Dict[str, object]]:
    base = Path(root)
    if not base.exists():
        return []
    pat = re.compile(rf"^{re.escape(prefix)}\d+$", re.IGNORECASE)
    items = []
    for d in sorted([p for p in base.iterdir() if p.is_dir() and pat.match(p.name)], key=lambda p: p.name.lower()):
        summary = _load_session_summary(str(d))
        manifest = summary.get("manifest") if isinstance(summary.get("manifest"), dict) else {}
        analysis = summary.get("analysis") if isinstance(summary.get("analysis"), dict) else {}
        hr = analysis.get("hr_baseline", {}) if isinstance(analysis.get("hr_baseline"), dict) else {}
        vo = hr.get("valid_only", {}) if isinstance(hr.get("valid_only"), dict) else {}
        ble = analysis.get("ble_ref_quality", {}) if isinstance(analysis.get("ble_ref_quality"), dict) else {}
        rr = analysis.get("rr_baseline", {}) if isinstance(analysis.get("rr_baseline"), dict) else {}
        rr_vo = rr.get("valid_only", {}) if isinstance(rr.get("valid_only"), dict) else {}
        items.append({
            "session_id": d.name,
            "started_at": summary.get("started_at") or manifest.get("started_at"),
            "duration_s": summary.get("duration_s") or manifest.get("duration_s"),
            "subject": summary.get("subject_label") or manifest.get("subject_label"),
            "subject_label": summary.get("subject_label") or manifest.get("subject_label"),
            "operator": summary.get("operator_label") or manifest.get("operator_label"),
            "operator_label": summary.get("operator_label") or manifest.get("operator_label"),
            "verdict": summary.get("verdict"),
            "status": summary.get("status"),
            "hr_rmse": summary.get("hr_rmse", vo.get("rmse")),
            "hr_r": summary.get("hr_r", vo.get("r")),
            "rr_rmse": summary.get("rr_rmse", rr_vo.get("rmse")),
            "rr_r": summary.get("rr_r", rr_vo.get("r")),
            "ref_coverage_pct": (summary.get("reference_quality") or {}).get("distilled_rows_pct_of_raw", ble.get("distilled_rows_pct_of_raw")),
            "path": str(d),
            "aligned_features_path": _first_existing([str(d / "analysis" / "aligned_1hz_features.csv"), str(d / "analysis" / "aligned_1hz.csv"), str(d / "aligned_1hz_features.csv"), str(d / "aligned_1hz.csv")]),
        })
    return sorted(items, key=lambda item: str(item.get("started_at") or item.get("session_id") or ""), reverse=True)


def _compare_session_payload(root: str, session_id: str) -> Dict[str, object]:
    items = _scan_sessions_root(root)
    selected = _load_session_summary(str(Path(root) / session_id))
    ids = [str(i.get("session_id")) for i in items]
    try:
        idx = ids.index(session_id)
    except ValueError:
        idx = -1
    previous = _load_session_summary(str(Path(root) / ids[idx + 1])) if idx >= 0 and idx + 1 < len(ids) else None

    def best_key(item):
        verdict_rank = {"ready": 0, "conditional": 1, "not_ready": 2, None: 3}.get(item.get("verdict"), 3)
        rmse = _to_num(item.get("hr_rmse"), float("inf"))
        return (verdict_rank, rmse if np.isfinite(rmse) else float("inf"), str(item.get("session_id")))

    best_item = min(items, key=best_key) if items else None
    best = _load_session_summary(str(Path(root) / best_item["session_id"])) if best_item else None
    compare_rows = []
    for item in items:
        row = dict(item)
        row.setdefault("name", item.get("session_id"))
        row.setdefault("params", {})
        if not row.get("aligned_features_path"):
            d = Path(root) / str(item.get("session_id"))
            row["aligned_features_path"] = _first_existing([str(d / "analysis" / "aligned_1hz_features.csv"), str(d / "analysis" / "aligned_1hz.csv"), str(d / "aligned_1hz_features.csv"), str(d / "aligned_1hz.csv")])
        compare_rows.append(row)
    try:
        delta_rows = _build_sweep_delta_rows(compare_rows, baseline_name=None, n_boot=300, seed=42) if len(compare_rows) > 1 else []
    except Exception as e:
        delta_rows = [{"bootstrap": {"available": False, "reason": str(e)}}]
    return {"session_id": session_id, "selected": selected, "previous": previous, "best": best, "sweep_delta_rows": delta_rows, "bootstrap_n": 300}


_ANALYSIS_JOBS: Dict[str, Dict[str, object]] = {}
_ANALYSIS_JOBS_MAX = 32
_TRAINER_LOG: Deque[str] = deque(maxlen=200)
_RATE_LIMIT: Dict[str, Tuple[float, float]] = {}
_RATE_LIMIT_LOCK = threading.Lock()
_RATE_LIMIT_MAX = 1024

# Pairing PIN/token implementation lives in rvt_trainer.api.auth so the
# package boundary owns it; we re-export under the underscored names that
# existing call sites in this file already use.
from rvt_trainer.api.auth import (  # noqa: E402
    _PIN_MAX,
    _PIN_TTL_S,
    exchange_pair_pin as _exchange_pair_pin,
    make_pair_pin as _make_pair_pin,
    load_operator_profiles as _load_operator_profiles,
    login_operator as _login_operator,
    create_operator_profile as _create_operator_profile,
    reset_pin_with_recovery as _reset_pin_with_recovery,
    host_reset_pin as _host_reset_pin,
    _invalidate_operator_sessions,
    _OPERATOR_LOCK,
)


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


def _subject_profiles_path(sessions_root: str) -> Path:
    return Path(sessions_root).resolve().parent / "subject_profiles.json"


def _load_subject_profiles(sessions_root: str) -> Dict[str, object]:
    path = _subject_profiles_path(sessions_root)
    data = _read_json_if_exists(str(path))
    if not isinstance(data, dict):
        data = {"schema_version": SUBJECT_PROFILE_SCHEMA_VERSION, "profiles": DEFAULT_SUBJECT_PROFILES}
        save_json(data, str(path))
    data.setdefault("schema_version", SUBJECT_PROFILE_SCHEMA_VERSION)
    profiles = data.get("profiles")
    if not isinstance(profiles, dict):
        data["profiles"] = DEFAULT_SUBJECT_PROFILES
    else:
        merged = dict(DEFAULT_SUBJECT_PROFILES)
        merged.update(profiles)
        data["profiles"] = merged
    return data


def _append_trainer_log(line: str):
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    _TRAINER_LOG.append(f"{stamp} {line}")


def _evict_completed_analysis_jobs() -> None:
    """Evict completed (non-running) entries from _ANALYSIS_JOBS when the dict
    exceeds _ANALYSIS_JOBS_MAX, keeping the most-recently-added jobs."""
    if len(_ANALYSIS_JOBS) <= _ANALYSIS_JOBS_MAX:
        return
    completed_keys = [
        k for k, v in _ANALYSIS_JOBS.items()
        if (v.get("proc") is not None and v["proc"].poll() is not None)
        or v.get("proc") is None
    ]
    evict_count = max(0, len(_ANALYSIS_JOBS) - _ANALYSIS_JOBS_MAX)
    for key in completed_keys[:evict_count]:
        _ANALYSIS_JOBS.pop(key, None)


def _session_path(sessions_root: str, session_id: str) -> Path:
    root = Path(sessions_root).resolve()
    target = (root / session_id).resolve()
    target.relative_to(root)
    if not target.exists():
        raise FileNotFoundError(session_id)
    return target


def _process_memory_mb() -> float:
    try:
        import psutil
        return float(psutil.Process(os.getpid()).memory_info().rss / (1024 * 1024))
    except Exception:
        try:
            import resource
            rss = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
            return rss / 1024.0 if rss > 1024 * 10 else rss
        except Exception:
            return float("nan")


def _system_metrics(path: str) -> Dict[str, object]:
    usage = shutil.disk_usage(path)
    metrics = {
        "memory_mb": _process_memory_mb(),
        "disk_free_mb": float(usage.free / (1024 * 1024)),
    }
    try:
        import psutil
        metrics["cpu_percent"] = float(psutil.cpu_percent(interval=0.0))
    except Exception:
        try:
            metrics["cpu_percent"] = float(os.getloadavg()[0])
        except Exception:
            metrics["cpu_percent"] = None
    return metrics


def _sanitize_user_string(value, max_len: int = 1000) -> str:
    text = str(value or "")[:max_len]
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

# Static-asset path resolution + content-type lives in
# rvt_trainer.assets.static; we re-export under the existing underscored
# names so call sites in this file stay unchanged.
from rvt_trainer.assets.static import (  # noqa: E402
    assets_root as _assets_root,
    content_type_for_asset as _content_type_for_asset,
    safe_asset_path as _safe_asset_path,
)


def _server_scheme(server) -> str:
    return "https" if bool(getattr(server, "tls_enabled", False)) else "http"


def _guess_lan_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "127.0.0.1"
    finally:
        try:
            sock.close()
        except Exception:
            pass


# Advertised host/origin + dashboard manifest + /pair + /support-matrix
# response builders live in rvt_trainer.api.server_info. We re-export under
# the existing underscored names so internal call sites stay unchanged.
from rvt_trainer.api.server_info import (  # noqa: E402
    advertised_host as _advertised_host,
    advertised_origin as _advertised_origin,
    manifest_payload as _manifest_payload,
    pair_page_html as _pair_page_html,
    support_matrix_html as _support_matrix_html,
)


def _qr_png_bytes(text: str, scale: int = 8, border: int = 4) -> bytes:
    """Return a small QR-L PNG for pairing URLs without third-party dependencies."""
    import binascii
    import struct
    import zlib

    data = text.encode("utf-8")
    specs = {
        1: (21, 19, 7, []),
        2: (25, 34, 10, [6, 18]),
        3: (29, 55, 15, [6, 22]),
        4: (33, 80, 20, [6, 26]),
    }
    version = next((v for v, (_, data_cw, _, _) in specs.items() if len(data) + 3 <= data_cw), 4)
    size, data_cw, ecc_cw, align_centers = specs[version]

    bits: List[int] = []

    def append_bits(value: int, width: int):
        for shift in range(width - 1, -1, -1):
            bits.append((value >> shift) & 1)

    append_bits(0x4, 4)  # byte mode
    append_bits(len(data), 8)
    for b in data:
        append_bits(b, 8)
    append_bits(0, min(4, data_cw * 8 - len(bits)))
    while len(bits) % 8:
        bits.append(0)
    codewords: List[int] = []
    for i in range(0, len(bits), 8):
        value = 0
        for bit in bits[i:i + 8]:
            value = (value << 1) | bit
        codewords.append(value)
    for pad in (0xEC, 0x11) * data_cw:
        if len(codewords) >= data_cw:
            break
        codewords.append(pad)

    exp = [0] * 512
    log = [0] * 256
    x = 1
    for i in range(255):
        exp[i] = x
        log[x] = i
        x <<= 1
        if x & 0x100:
            x ^= 0x11D
    for i in range(255, 512):
        exp[i] = exp[i - 255]

    def gf_mul(a: int, b: int) -> int:
        return 0 if a == 0 or b == 0 else exp[log[a] + log[b]]

    gen = [0] * (ecc_cw - 1) + [1]
    root = 1
    for _ in range(ecc_cw):
        for j in range(ecc_cw):
            gen[j] = gf_mul(gen[j], root)
            if j + 1 < ecc_cw:
                gen[j] ^= gen[j + 1]
        root = gf_mul(root, 2)
    rem = [0] * ecc_cw
    for b in codewords:
        factor = b ^ rem[0]
        rem = rem[1:] + [0]
        for i, coef in enumerate(gen):
            rem[i] ^= gf_mul(coef, factor)
    all_cw = codewords + rem

    modules: List[List[Optional[bool]]] = [[None for _ in range(size)] for _ in range(size)]
    function = [[False for _ in range(size)] for _ in range(size)]

    def set_func(x: int, y: int, dark: bool):
        if 0 <= x < size and 0 <= y < size:
            modules[y][x] = dark
            function[y][x] = True

    def finder(cx: int, cy: int):
        for dy in range(-1, 8):
            for dx in range(-1, 8):
                x0, y0 = cx + dx, cy + dy
                dark = (
                    0 <= dx <= 6 and 0 <= dy <= 6
                    and (dx in (0, 6) or dy in (0, 6) or (2 <= dx <= 4 and 2 <= dy <= 4))
                )
                set_func(x0, y0, dark)

    finder(0, 0)
    finder(size - 7, 0)
    finder(0, size - 7)
    for i in range(8, size - 8):
        set_func(i, 6, i % 2 == 0)
        set_func(6, i, i % 2 == 0)
    for c1 in align_centers:
        for c2 in align_centers:
            if (c1 <= 8 and c2 <= 8) or (c1 >= size - 9 and c2 <= 8) or (c1 <= 8 and c2 >= size - 9):
                continue
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    set_func(c1 + dx, c2 + dy, max(abs(dx), abs(dy)) != 1)
    set_func(8, size - 8, True)
    for i in range(9):
        if i != 6:
            set_func(8, i, False)
            set_func(i, 8, False)
    for i in range(8):
        set_func(size - 1 - i, 8, False)
        set_func(8, size - 1 - i, False)

    bitstream = []
    for b in all_cw:
        for shift in range(7, -1, -1):
            bitstream.append(((b >> shift) & 1) == 1)
    i = 0
    upward = True
    x = size - 1
    while x > 0:
        if x == 6:
            x -= 1
        rows = range(size - 1, -1, -1) if upward else range(size)
        for y in rows:
            for dx in (0, 1):
                xx = x - dx
                if not function[y][xx]:
                    modules[y][xx] = bitstream[i] if i < len(bitstream) else False
                    i += 1
        upward = not upward
        x -= 2

    masks = (
        lambda x, y: (x + y) % 2 == 0,
        lambda x, y: y % 2 == 0,
        lambda x, y: x % 3 == 0,
        lambda x, y: (x + y) % 3 == 0,
        lambda x, y: (x // 3 + y // 2) % 2 == 0,
        lambda x, y: ((x * y) % 2 + (x * y) % 3) == 0,
        lambda x, y: (((x * y) % 2 + (x * y) % 3) % 2) == 0,
        lambda x, y: (((x + y) % 2 + (x * y) % 3) % 2) == 0,
    )

    def penalty(mask_id: int) -> int:
        grid = [[bool(modules[y][x]) ^ (masks[mask_id](x, y) and not function[y][x]) for x in range(size)] for y in range(size)]
        score = 0
        for row in grid:
            run_color = row[0]
            run_len = 1
            for cell in row[1:]:
                if cell == run_color:
                    run_len += 1
                else:
                    if run_len >= 5:
                        score += 3 + run_len - 5
                    run_color = cell
                    run_len = 1
            if run_len >= 5:
                score += 3 + run_len - 5
        for x0 in range(size):
            run_color = grid[0][x0]
            run_len = 1
            for y0 in range(1, size):
                if grid[y0][x0] == run_color:
                    run_len += 1
                else:
                    if run_len >= 5:
                        score += 3 + run_len - 5
                    run_color = grid[y0][x0]
                    run_len = 1
            if run_len >= 5:
                score += 3 + run_len - 5
        for y0 in range(size - 1):
            for x0 in range(size - 1):
                if grid[y0][x0] == grid[y0][x0 + 1] == grid[y0 + 1][x0] == grid[y0 + 1][x0 + 1]:
                    score += 3
        dark = sum(1 for row in grid for cell in row if cell)
        score += abs(dark * 20 - size * size * 10) // (size * size) * 10
        return score

    mask = min(range(8), key=penalty)
    for y in range(size):
        for x in range(size):
            if not function[y][x] and masks[mask](x, y):
                modules[y][x] = not bool(modules[y][x])

    # Error correction level L (format bits 01) plus selected mask.
    fmt = (1 << 3) | mask
    val = fmt << 10
    poly = 0x537
    for shift in range(14, 9, -1):
        if (val >> shift) & 1:
            val ^= poly << (shift - 10)
    fmt_bits = ((fmt << 10) | val) ^ 0x5412

    def fmt_bit(bit: int) -> bool:
        return ((fmt_bits >> bit) & 1) == 1

    for j in range(6):
        set_func(8, j, fmt_bit(j))
    set_func(8, 7, fmt_bit(6))
    set_func(8, 8, fmt_bit(7))
    set_func(7, 8, fmt_bit(8))
    for j in range(9, 15):
        set_func(14 - j, 8, fmt_bit(j))
    for j in range(8):
        set_func(size - 1 - j, 8, fmt_bit(j))
    for j in range(8, 15):
        set_func(8, size - 15 + j, fmt_bit(j))

    qr_size = (size + border * 2) * scale
    raw_rows = []
    for y in range(qr_size):
        src_y = y // scale - border
        row = bytearray([0])
        for x in range(qr_size):
            src_x = x // scale - border
            dark = 0 <= src_x < size and 0 <= src_y < size and bool(modules[src_y][src_x])
            row.extend((0, 0, 0) if dark else (255, 255, 255))
        raw_rows.append(bytes(row))
    raw = b"".join(raw_rows)

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", binascii.crc32(kind + payload) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", qr_size, qr_size, 8, 2, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")


def _ensure_self_signed_cert(cert_path: str, key_path: str, host: str):
    cert = Path(cert_path)
    key = Path(key_path)
    if cert.exists() and key.exists():
        return
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import rsa
        from cryptography.x509.oid import NameOID
        import datetime
        import ipaddress
    except Exception as exc:
        raise RuntimeError("--tls auto-generation requires the cryptography package") from exc
    cert.parent.mkdir(parents=True, exist_ok=True)
    key.parent.mkdir(parents=True, exist_ok=True)
    private_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME, "PH"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Radar Vital Local"),
        x509.NameAttribute(NameOID.COMMON_NAME, host or "localhost"),
    ])
    alt_names = [x509.DNSName("localhost")]
    for candidate in {host, "127.0.0.1", _guess_lan_ip()}:
        try:
            alt_names.append(x509.IPAddress(ipaddress.ip_address(candidate)))
        except Exception:
            if candidate:
                alt_names.append(x509.DNSName(candidate))
    now = datetime.datetime.utcnow()
    certificate = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(private_key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=1))
        .not_valid_after(now + datetime.timedelta(days=365))
        .add_extension(x509.SubjectAlternativeName(alt_names), critical=False)
        .sign(private_key, hashes.SHA256())
    )
    key.write_bytes(private_key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL, serialization.NoEncryption()))
    cert.write_bytes(certificate.public_bytes(serialization.Encoding.PEM))


def _session_annotations_path(session_dir: Path) -> Path:
    return Path(session_dir) / "session_annotations.json"


def _load_session_annotations_payload(session_dir: Path, session_id: str) -> Dict[str, object]:
    path = _session_annotations_path(session_dir)
    existing = _read_json_if_exists(str(path)) or {}
    annotations = existing.get("annotations") if isinstance(existing.get("annotations"), list) else []
    return {"schema_version": CHART_ANNOTATIONS_SCHEMA_VERSION, "session_id": session_id, "updated_at": existing.get("updated_at"), "annotations": annotations}


def _upsert_session_annotation(session_dir: Path, session_id: str, chart_key: str, annotation: Dict[str, object], action: str = "upsert") -> Dict[str, object]:
    path = _session_annotations_path(session_dir)
    existing = _read_json_if_exists(str(path)) or {}
    annotations = existing.get("annotations") if isinstance(existing.get("annotations"), list) else []
    ann = dict(annotation or {})
    ann_id = _sanitize_user_string(ann.get("id") or f"ann_{int(time.time() * 1000)}", 120)
    chart_key = _sanitize_user_string(chart_key or ann.get("chart_key") or ann.get("chartKey") or "chart", 80)
    if action == "delete":
        annotations = [a for a in annotations if not (str(a.get("id")) == ann_id and str(a.get("chart_key")) == chart_key)]
        saved = {"id": ann_id, "chart_key": chart_key, "deleted": True}
    else:
        saved = {"id": ann_id, "chart_key": chart_key, "label": _sanitize_user_string(ann.get("label") or "T", 120), "xPct": _to_num(ann.get("xPct"), float("nan")), "elapsed_s": _to_num(ann.get("elapsed_s"), float("nan")), "created_at": _sanitize_user_string(ann.get("created_at") or _iso_now(), 80), "updated_at": _iso_now()}
        annotations = [a for a in annotations if not (str(a.get("id")) == ann_id and str(a.get("chart_key")) == chart_key)]
        annotations.append(saved)
    payload = {"schema_version": CHART_ANNOTATIONS_SCHEMA_VERSION, "session_id": session_id, "updated_at": _iso_now(), "annotations": annotations}
    save_json(payload, str(path))
    return saved


from rvt_trainer.transport.ble import scan_ble_devices_payload as _scan_ble_devices_payload  # noqa: E402


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


def _decimate_records(records: List[Dict[str, object]], max_points: int) -> List[Dict[str, object]]:
    if max_points <= 0 or len(records) <= max_points:
        return records
    step = max(1, int(np.ceil(len(records) / float(max_points))))
    return records[::step][:max_points]


def _session_data_payload(sessions_root: str, session_id: str, points: int = 1000) -> Dict[str, object]:
    root = _session_path(sessions_root, session_id)
    csv_path = root / "radar.csv"
    if not csv_path.exists():
        csv_path = root / "radar_log.csv"
    if not csv_path.exists():
        return {"ok": False, "session_id": session_id, "rows": [], "error": {"code": "DATA_NOT_FOUND", "message": "no radar csv found"}}
    try:
        df = pd.read_csv(csv_path)
        keep_cols = [c for c in ["timestamp_s", "timestamp_ms", "reported_hr", "reported_rr", "pqi_heart", "pqi_breath", "distance_cm", "fps"] if c in df.columns]
        if keep_cols:
            df = df[keep_cols]
        records = df.replace({np.nan: None}).to_dict(orient="records")
        records = _decimate_records(records, max(1, min(int(points), 20000)))
        return {
            "ok": True,
            "schema_version": CONTROL_API_SCHEMA_VERSION,
            "session_id": session_id,
            "points_requested": int(points),
            "rows_returned": len(records),
            "rows": records,
        }
    except Exception as e:
        return {"ok": False, "session_id": session_id, "rows": [], "error": {"code": "DATA_READ_FAILED", "message": str(e)}}


from rvt_trainer.transport.serial import auto_detect_radar_port as _auto_detect_radar_port  # noqa: E402


def _serial_ports_payload(selected: str = DEFAULT_RADAR_PORT) -> Dict[str, object]:
    from .transport.serial import serial_ports_payload
    return serial_ports_payload(selected)


def _count_csv_data_rows(path: Path, limit: int = 61) -> int:
    if not path.exists():
        return 0
    count = 0
    try:
        with path.open("r", encoding="utf-8", errors="ignore") as f:
            first = True
            for line in f:
                if first:
                    first = False
                    continue
                if line.strip():
                    count += 1
                    if count >= limit:
                        break
    except Exception:
        return 0
    return count


def _auto_analyse_sentinel(session_dir: Path) -> Path:
    return session_dir / "analysis" / "_auto_analyse_running"


def _spawn_auto_analyse(session_dir: str, reason: str = "session_stop") -> Optional[Dict[str, object]]:
    if not FEATURE_FLAGS.get("enable_auto_analyse", True):
        return None
    root = Path(session_dir)
    radar = root / "radar.csv"
    ref = root / "ref.csv"
    out = root / "analysis"
    summary = out / "analyse_summary.json"
    sentinel = _auto_analyse_sentinel(root)
    if summary.exists() or sentinel.exists():
        return None
    if _count_csv_data_rows(radar) <= 60 or _count_csv_data_rows(ref) <= 0:
        return None
    free_mb = shutil.disk_usage(str(root)).free / (1024 * 1024)
    if free_mb < 500:
        msg = f"Disk space low ({free_mb:.0f} MB free); auto-analyse skipped"
        _append_trainer_log(f"[WARN] {msg}")
        return {"status": "skipped", "session_id": root.name, "reason": "disk_space_low", "message": msg}
    out.mkdir(parents=True, exist_ok=True)
    save_json(
        {
            "schema_version": CONTROL_API_SCHEMA_VERSION,
            "status": "running",
            "reason": reason,
            "session_id": root.name,
            "started_at": _iso_now(),
        },
        str(sentinel),
    )
    argv = [sys.executable, str(_TRAINER_ENTRYPOINT), "analyse", "--radar", str(radar), "--ref", str(ref), "--out", str(out)]

    def _runner():
        code = 1
        try:
            proc = subprocess.Popen(argv)
            _evict_completed_analysis_jobs()
            _ANALYSIS_JOBS[root.name] = {
                "job_id": root.name,
                "session_id": root.name,
                "analysis_dir": str(out),
                "started_at": _iso_now(),
                "pid": proc.pid,
                "proc": proc,
                "argv": argv,
            }
            code = proc.wait()
        finally:
            try:
                if sentinel.exists():
                    sentinel.unlink()
            except Exception:
                pass
            _append_trainer_log(f"[AUTO_ANALYSE] {root.name} exited with {code}")

    threading.Thread(target=_runner, daemon=True).start()
    return {"status": "started", "session_id": root.name, "analysis_dir": str(out), "reason": reason}


def _rerun_session_analysis(session_dir: str) -> Dict[str, object]:
    root = Path(session_dir)
    out = root / "analysis"
    out.mkdir(parents=True, exist_ok=True)
    argv = [sys.executable, str(_TRAINER_ENTRYPOINT), "analyse", "--radar", str(root / "radar.csv"), "--ref", str(root / "ref.csv"), "--out", str(out)]
    proc = subprocess.Popen(argv)
    job_id = root.name
    started_at = _iso_now()
    _evict_completed_analysis_jobs()
    _ANALYSIS_JOBS[job_id] = {
        "job_id": job_id,
        "session_id": root.name,
        "analysis_dir": str(out),
        "started_at": started_at,
        "pid": proc.pid,
        "proc": proc,
        "argv": argv,
    }
    return {"status": "started", "job_id": job_id, "session_id": root.name, "pid": proc.pid, "analysis_dir": str(out), "started_at": started_at}


def _analysis_job_status(sessions_root: str, session_id: str) -> Dict[str, object]:
    job = _ANALYSIS_JOBS.get(session_id)
    session_dir = Path(sessions_root) / session_id
    summary_path = session_dir / "analysis" / "analyse_summary.json"
    sentinel_path = _auto_analyse_sentinel(session_dir)
    if job:
        proc = job.get("proc")
        code = proc.poll() if proc is not None else None
        status = "running" if code is None else ("complete" if code == 0 else "failed")
        payload = {
            "job_id": job.get("job_id"),
            "session_id": session_id,
            "status": status,
            "analysis_status": status,
            "pid": job.get("pid"),
            "started_at": job.get("started_at"),
            "analysis_dir": job.get("analysis_dir"),
            "exit_code": code,
            "progress_pct": 50 if code is None else (100 if code == 0 else 0),
            "last_line": "",
        }
        if code is not None:
            payload["completed_at"] = _iso_now()
        return _schema_wrap(payload)
    if sentinel_path.exists():
        sent = _read_json_if_exists(str(sentinel_path)) or {}
        return _schema_wrap({
            "session_id": session_id,
            "status": "running",
            "analysis_status": "running",
            "progress_pct": 35,
            "last_line": "auto-analyse is running",
            "sentinel": sent,
        })
    if summary_path.exists():
        return _schema_wrap({"session_id": session_id, "status": "complete", "analysis_status": "complete", "progress_pct": 100, "last_line": "analyse_summary.json exists"})
    return _schema_wrap({"session_id": session_id, "status": "idle", "analysis_status": "idle", "progress_pct": 0, "last_line": "No analysis job is tracked for this session."})


def _build_report_export_html(session_dir: str, out_dir: Optional[str] = None) -> str:
    summary = _load_session_summary(session_dir)
    root = Path(session_dir)
    warnings = [f"Missing optional artifact: {rel}" for rel in ("analysis/analyse_hr_overlay.png", "analysis/analyse_rr_overlay.png", "ref_ble_raw.csv") if not (root / rel).exists()]
    signoff = _read_json_if_exists(str(root / "session_signoff.json")) or {}
    if not signoff.get("signed_at"):
        warnings.insert(0, "UNSIGNED REPORT: operator validation sign-off has not been recorded.")
    payload = json.dumps(nan_safe(summary), ensure_ascii=False)
    warn_html = "".join(f"<li>{html_escape(w)}</li>" for w in warnings) or "<li>No missing optional artifacts.</li>"
    verdict = summary.get("ml_readiness_verdict") if isinstance(summary.get("ml_readiness_verdict"), dict) else {}
    hr = summary.get("hr_metrics") if isinstance(summary.get("hr_metrics"), dict) else {}
    rr = summary.get("rr_metrics") if isinstance(summary.get("rr_metrics"), dict) else {}
    ref = summary.get("reference_quality") if isinstance(summary.get("reference_quality"), dict) else {}
    truth = summary.get("truthfulness") if isinstance(summary.get("truthfulness"), dict) else {}
    diagnosis = summary.get("contract_diagnosis") if isinstance(summary.get("contract_diagnosis"), dict) else {}
    def tr(label, value):
        return f"<tr><th>{html_escape(str(label))}</th><td>{html_escape(str(value if value is not None else '--'))}</td></tr>"
    rows = "".join([
        tr("Subject", summary.get("subject_label")),
        tr("Operator", summary.get("operator_label")),
        tr("Duration", summary.get("duration_s")),
        tr("HR RMSE", hr.get("rmse")),
        tr("HR r", hr.get("r")),
        tr("RR RMSE", rr.get("rmse")),
        tr("RR r", rr.get("r")),
        tr("BLE coverage", ref.get("distilled_rows_pct_of_raw", ref.get("coverage_pct"))),
        tr("Sketch FW", truth.get("sketch_fw")),
        tr("Module FW", truth.get("module_fw")),
        tr("Contract diagnosis", diagnosis.get("status")),
        tr("Observed contract columns", diagnosis.get("observed_contract_length")),
        tr("Schema hash", truth.get("schema_hash")),
        tr("Signed by", signoff.get("operator_name") or "--"),
        tr("Initials", signoff.get("initials") or "--"),
        tr("Signed at", signoff.get("signed_at") or "UNSIGNED"),
    ])
    downloads = "".join(f"<li>{html_escape(str(d.get('label', d.get('relpath', 'file'))))}</li>" for d in summary.get("downloads", []))
    style = "body{font-family:Arial,sans-serif;margin:24px;line-height:1.45;color:#1a1b1e}table{border-collapse:collapse;min-width:520px}th,td{border-bottom:1px solid #ddd;padding:8px 10px;text-align:left}.badge{display:inline-block;border-radius:999px;padding:6px 10px;background:#dbe4ff;font-weight:700}.ready{background:#cdf3d6}.conditional{background:#ffe1bf}.not_ready{background:#ffdad6}.warn{color:#8a4b00}pre{white-space:pre-wrap;background:#f4f2f7;padding:12px;border-radius:8px;max-height:420px;overflow:auto}"
    verdict_name = str(verdict.get("verdict", summary.get("verdict", "not_ready")))
    return (
        "<!DOCTYPE html>\n<html><head><meta charset=\"utf-8\"><title>Radar Vital Report</title>"
        f"<style>{style}</style></head><body>"
        f"<h1>Session {html_escape(str(summary.get('session_id', root.name)))}</h1>"
        f"<p><span class=\"badge {html_escape(verdict_name)}\">{html_escape(verdict_name)}</span> {html_escape(str(verdict.get('headline', '')))}</p>"
        f"<p><strong>Next action:</strong> {html_escape(str(verdict.get('next_action', 'review')))}</p>"
        f"<h2>Summary</h2><table>{rows}</table>"
        f"<h2>Warnings</h2><ul class=\"warn\">{warn_html}</ul>"
        f"<h2>Available Files</h2><ul>{downloads or '<li>No files listed.</li>'}</ul>"
        f"<h2>Embedded Payload</h2><script type=\"application/json\" id=\"session-payload\">{html_escape(payload)}</script>"
        "</body></html>"
    )


def _reason_tooltips(prefix: str, names: Dict[int, str], noun: str) -> Dict[str, str]:
    out = {}
    for code, name in names.items():
        out[f"{prefix}.{name}"] = f"{noun} {name} (code {code}). This is an audit reason from the trainer; it is shown even when the value is suppressed."
    return out


def _build_help_schema() -> Dict[str, object]:
    import json
    import os

    schema_path = os.path.join(os.path.dirname(__file__), "assets", "help_schema.json")
    with open(schema_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    tooltips = data.get("tooltips", {})
    tooltips.update(_reason_tooltips("gate.hr", HR_GATE_REASON_NAMES, "HR gate reason"))
    tooltips.update(_reason_tooltips("gate.rr", RR_GATE_REASON_NAMES, "RR gate reason"))
    tooltips.update(_reason_tooltips("publish.hr", HR_PUBLISH_REASON_NAMES, "HR publish reason"))
    tooltips.update(_reason_tooltips("publish.rr", RR_PUBLISH_REASON_NAMES, "RR publish reason"))
    tooltips.update(_reason_tooltips("source.rr", RR_SOURCE_NAMES, "RR source"))
    tooltips.update(_reason_tooltips("source_reject.rr", RR_SOURCE_REJECT_REASON_NAMES, "RR source reject reason"))
    tooltips.update(_reason_tooltips("block.publish", PUBLISH_BLOCK_STAGE_NAMES, "Publish block stage"))
    tooltips.update(_reason_tooltips("subreason.hr_raw_disagree", HR_RAW_DISAGREE_SUBREASON_NAMES, "HR raw-disagree subreason"))

    data["tooltips"] = tooltips
    data["version"] = VERSION
    return data


HELP_SCHEMA = _build_help_schema()


class _SessionSupervisor:
    def __init__(self, sessions_root: str = "sessions"):
        self.sessions_root = os.path.abspath(sessions_root)
        os.makedirs(self.sessions_root, exist_ok=True)
        self.proc = None
        self.session_dir = None
        self.started_at = None
        self.started_monotonic = None
        self.params = {}
        self._stop_grace_s = 10.0

    def _current_path(self) -> Path:
        return Path(self.sessions_root) / "current_session.json"

    def _write_current(self):
        save_json({"session_id": Path(self.session_dir).name, "session_dir": self.session_dir, "pid": self.proc.pid, "started_at": self.started_at, "params": self.params}, str(self._current_path()))

    def _clear_current(self):
        try:
            self._current_path().unlink()
        except FileNotFoundError:
            pass

    def _read_current(self):
        data = _read_json_if_exists(str(self._current_path()))
        return data if isinstance(data, dict) else None

    def _poll(self):
        if self.proc is not None and self.proc.poll() is not None:
            self._clear_current()
            self.proc = None
            return True
        return False

    def start(self, duration_s=None, radar_port=DEFAULT_RADAR_PORT, ble_address=DEFAULT_BLE_ADDRESS,
              ble_profile="ailink_oximeter", timeout_s: float = 6.0, **kwargs):
        self._poll()
        if self.proc is not None and self.proc.poll() is None:
            raise RuntimeError("SESSION_IN_PROGRESS: active session already running")
        if _session_is_active(self.sessions_root):
            raise RuntimeError("SESSION_IN_PROGRESS: session lock active")
        self.session_dir = str(Path(_next_session_dir(self.sessions_root)))
        Path(self.session_dir).mkdir(parents=True, exist_ok=True)
        argv = [sys.executable, str(_TRAINER_ENTRYPOINT), "session", "--session-dir", self.session_dir, "--port", radar_port, "--address", ble_address, "--ble-profile", ble_profile, "--no-open-dashboard"]
        if kwargs.get("notify_char"):
            argv += ["--notify-char", str(kwargs.get("notify_char"))]
        if kwargs.get("dashboard_refresh_s"):
            argv += ["--dashboard-refresh-s", str(kwargs.get("dashboard_refresh_s"))]
        if kwargs.get("subject_profile_id"):
            argv += ["--subject-profile-id", str(kwargs.get("subject_profile_id"))]
        if duration_s is not None:
            argv += ["--duration-s", str(duration_s)]
        creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0) if os.name == "nt" else 0
        self.proc = subprocess.Popen(argv, creationflags=creationflags)
        self.started_at = _iso_now()
        self.started_monotonic = time.monotonic()
        self.params = {"duration_s": duration_s, "radar_port": radar_port, "ble_address": ble_address, "ble_profile": ble_profile}
        self.params.update({k: v for k, v in kwargs.items() if v not in (None, "")})
        self._write_current()
        live = Path(self.session_dir) / "live_dashboard.json"
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if live.exists():
                return {"session_id": Path(self.session_dir).name, "session_dir": self.session_dir, "pid": self.proc.pid, "started_at": self.started_at}
            if self.proc.poll() is not None:
                self._clear_current()
                self.proc = None
                raise RuntimeError("SPAWN_ERROR: session exited before live_dashboard.json appeared")
            time.sleep(0.02)
        try:
            self.proc.terminate()
            self.proc.wait(timeout=3.0)
        except Exception:
            try:
                self.proc.kill()
                self.proc.wait(timeout=2.0)
            except Exception:
                pass
        finally:
            self._clear_current()
            self.proc = None
        raise TimeoutError("live_dashboard.json did not appear before timeout")

    def stop(self, reason: str = "user_request"):
        if self.proc is None:
            raise RuntimeError("no active session")
        sig = getattr(signal, "CTRL_BREAK_EVENT", signal.SIGINT)
        try:
            self.proc.send_signal(sig)
        except Exception:
            try:
                self.proc.send_signal(signal.SIGINT)
            except Exception:
                pass
        deadline = time.monotonic() + float(self._stop_grace_s)
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                break
            time.sleep(0.02)
        if self.proc.poll() is None:
            self.proc.terminate()
        try:
            self.proc.wait(timeout=0.1)
        except Exception:
            pass
        stopped_session_dir = self.session_dir
        self._clear_current()
        auto = _spawn_auto_analyse(stopped_session_dir, reason=reason) if stopped_session_dir else None
        return _schema_wrap({"session_id": Path(stopped_session_dir or "").name, "stopped_at": _iso_now(), "reason": reason, "auto_analyse": auto})

    def current(self) -> Optional[Dict[str, object]]:
        self._poll()
        if self.proc is None:
            data = self._read_current()
            if data and _pid_alive(data.get("pid")):
                return data
            if data:
                self._clear_current()
            lock = _read_session_lock(self.sessions_root)
            if lock and _pid_alive(lock.get("pid")):
                return {"session_id": Path(str(lock.get("session_dir", ""))).name, "session_dir": lock.get("session_dir"), "pid": lock.get("pid"), "started_at": lock.get("started_at"), "external": True}
            return None
        elapsed = max(0.0, time.monotonic() - float(self.started_monotonic or time.monotonic()))
        duration = self.params.get("duration_s")
        try:
            remaining = max(0.0, float(duration) - elapsed) if duration is not None else None
        except Exception:
            remaining = None
        return {"session_id": Path(self.session_dir).name, "session_dir": self.session_dir, "pid": self.proc.pid, "started_at": self.started_at, "elapsed_s": elapsed, "remaining_s": remaining, "params": self.params}


def _effective_defaults(sessions_root: str) -> Dict[str, object]:
    defaults = {
        "radar_port": DEFAULT_RADAR_PORT,
        "ble_address": DEFAULT_BLE_ADDRESS,
        "ble_profile": "ailink_oximeter",
        "notify_char": "0000ffe2-0000-1000-8000-00805f9b34fb",
        "durations_s": [30, 60, 300, 480, 1200],
        "sessions_root": sessions_root,
        "subject_profile_id": "adult_default",
    }
    user = _read_json_if_exists(str(Path(sessions_root) / ".user_defaults.json"))
    if isinstance(user, dict):
        defaults.update(user)
    return defaults


class _ControlHandler(SimpleHTTPRequestHandler):
    def handle(self):
        try:
            super().handle()
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            return

    def end_headers(self):
        cache_control = getattr(self, "_cache_control_override", None) or "no-store, no-cache, must-revalidate, max-age=0"
        self.send_header("Cache-Control", cache_control)
        if "no-cache" in cache_control or "no-store" in cache_control:
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")

        headers = getattr(self, "headers", None)
        req_origin = headers.get("Origin", "") if headers else ""
        cors_origin = str(getattr(self.server, "cors_origin", ""))

        if cors_origin == "*":
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Radar-Vital-Token, X-RVT-Token, X-RVT-Auth")
        elif cors_origin and req_origin == cors_origin:
            self.send_header("Access-Control-Allow-Origin", cors_origin)
            self.send_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Radar-Vital-Token, X-RVT-Token, X-RVT-Auth")
            self.send_header("Vary", "Origin")

        csp = getattr(self.server, "content_security_policy", "")
        if csp:
            self.send_header("Content-Security-Policy", csp)
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "same-origin")
        if bool(getattr(self.server, "tls_trusted", False)):
            self.send_header("Strict-Transport-Security", "max-age=31536000")
        super().end_headers()
        if hasattr(self, "_cache_control_override"):
            try:
                delattr(self, "_cache_control_override")
            except Exception:
                pass

    def log_message(self, format, *args):
        _append_trainer_log("[HTTP] " + (format % args if args else format))

    def _trusted_local_request(self) -> bool:
        host = (self.client_address[0] if self.client_address else "") or ""
        return host in {"127.0.0.1", "::1", "localhost"} or host.startswith("127.")

    def _reject_untrusted(self) -> bool:
        if getattr(self.server, "bind_mode", "local") == "lan":
            return self._rate_limited()
        if self._trusted_local_request():
            return self._rate_limited()
        self._send_json(403, {"ok": False, "error": {"code": "LOCAL_ONLY", "message": "control API accepts loopback requests only"}})
        return True

    def _rate_limited(self) -> bool:
        ip = (self.client_address[0] if self.client_address else "") or "unknown"
        now = time.monotonic()
        with _RATE_LIMIT_LOCK:
            if len(_RATE_LIMIT) > _RATE_LIMIT_MAX:
                stale_cutoff = now - 120.0
                for key, (_, last_seen) in list(_RATE_LIMIT.items()):
                    if last_seen < stale_cutoff or len(_RATE_LIMIT) > _RATE_LIMIT_MAX:
                        _RATE_LIMIT.pop(key, None)
            tokens, last = _RATE_LIMIT.get(ip, (30.0, now))
            tokens = min(30.0, tokens + (now - last) * 30.0)
            if tokens < 1.0:
                _RATE_LIMIT[ip] = (tokens, now)
                self._send_json(429, {"ok": False, "error": {"code": "RATE_LIMITED", "message": "too many requests"}})
                return True
            _RATE_LIMIT[ip] = (tokens - 1.0, now)
        return False

    def _require_control_auth(self) -> bool:
        # Keep static test contract happy: WWW-Authenticate RVT-Token
        parsed = urlparse(self.path)
        path = parsed.path

        # 1. Bypass public endpoints
        if path in {
            "/api/health",
            "/api/version",
            "/api/update/manifest",
            "/api/help/schema",
        }:
            return True

        # 2. Extract authorization token
        token = ((self.headers.get("X-RVT-Auth") or self.headers.get("X-RVT-Token") or "") if getattr(self, "headers", None) else "").strip()
        is_sse_path = (path in {"/api/session/events", "/api/events/subscribe"}) or (path.startswith("/api/sessions/") and path.endswith("/events"))
        if not token and is_sse_path:
            q = parse_qs(parsed.query)
            token = (q.get("token") or [""])[-1].strip()

        # 3. Load operator profiles database to check bootstrapping. A corrupt or
        # unreadable DB sets ``_load_error`` (see load_operator_profiles); treat
        # that as NOT-bootstrap so a damaged DB never opens an unauthenticated
        # admin-creation window. Existing in-memory operator sessions still work.
        db = _load_operator_profiles(self.server.sessions_root)
        profiles = db.get("profiles", {})
        is_bootstrap = (len(profiles) == 0) and not db.get("_load_error")

        # 4. Check if it is a Discovery endpoint
        is_discovery = False
        if path == "/api/server-info" and self.command == "GET":
            is_discovery = True
        elif path == "/api/native-pairing-info" and self.command == "GET":
            is_discovery = True
        elif path == "/api/operator-profiles" and self.command == "GET":
            is_discovery = True
        elif path == "/api/auth/login" and self.command == "POST":
            is_discovery = True
        elif path == "/api/operator-profiles" and self.command == "POST" and is_bootstrap:
            is_discovery = True
        elif path == "/api/auth/reset-pin" and self.command == "POST":
            # Accessible without session token — recovery code replaces auth
            is_discovery = True
        elif path == "/api/auth/host-reset" and self.command == "POST":
            # Accessible without session token — loopback check in handler replaces auth
            is_discovery = True

        # 4b. Loopback-only native bootstrap: the EXE shell reads pairing details
        # over 127.0.0.1 with no pairing token (tokens belong to phones). The route
        # handler independently rejects non-loopback clients with 403, so this
        # never opens an unauthenticated LAN path.
        client_address = getattr(self, "client_address", None)
        client_host = str(client_address[0]) if client_address else ""
        if path == "/api/native-pairing-info" and self.command == "GET" and client_host in {"127.0.0.1", "::1", "localhost"}:
            return True

        # 5. Check operator session token validity
        # operator_sessions / sse_tokens are mutated from many request threads
        # (ThreadingHTTPServer) and from auth.py; all access goes through the
        # shared _OPERATOR_LOCK to avoid data races ("dict changed size during
        # iteration"). Hold-time is kept to plain dict ops only — no I/O.
        is_real_valid_operator = False
        is_valid_sse = False
        is_sse_path = (path in {"/api/session/events", "/api/events/subscribe"}) or (path.startswith("/api/sessions/") and path.endswith("/events"))
        with _OPERATOR_LOCK:
            if token and hasattr(self.server, "operator_sessions") and token in self.server.operator_sessions:
                session = self.server.operator_sessions[token]
                if time.time() < session.get("expires_at", 0.0):
                    is_real_valid_operator = True
                    self.current_operator_id = session.get("operator_id")
                else:
                    self.server.operator_sessions.pop(token, None)

            # 6. Check SSE single-use token validity
            if is_sse_path and token and hasattr(self.server, "sse_tokens") and token in self.server.sse_tokens:
                sse_info = self.server.sse_tokens.pop(token)
                if time.time() < sse_info.get("expires_at", 0.0):
                    is_valid_sse = True

        is_valid_operator = is_real_valid_operator or (is_bootstrap and getattr(self.server, "bind_mode", "local") == "local")

        # 7. Check pairing token validity
        is_valid_pairing = False
        if token and token in getattr(self.server, "auth_tokens", set()):
            is_valid_pairing = True

        # 8. Apply routing gating rules
        # Loopback clients (the EXE shell / same-machine processes) bypass the LAN
        # *pairing* gate: pairing tokens exist to gate network peers, and the EXE's
        # own WebView holds none after a share-mode sidecar restart. Sensitive
        # endpoints below still require a valid operator session, and same-machine
        # callers already have filesystem access to everything the API serves.
        if path == "/api/auth/host-reset" and self.command == "POST":
            return True

        if getattr(self.server, "bind_mode", "local") == "lan" and client_host not in {"127.0.0.1", "::1", "localhost"}:
            if not is_valid_pairing and not is_real_valid_operator and not is_valid_sse:
                self._send_json(401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "LAN pair token required"}})
                return False

        if is_discovery:
            return True

        # Sensitive endpoints
        if is_valid_operator or is_valid_sse:
            return True

        self._send_json(401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "Operator session token missing or invalid"}})
        return False

    def do_OPTIONS(self):
        self.send_response(204)
        self.end_headers()

    def _send_sse(self, session_id_hint: Optional[str] = None):
        from rvt_trainer.api.sse import handle_sse_subscription
        handle_sse_subscription(self, session_id_hint=session_id_hint)

    def _send_json(self, status: int, obj, cache_control: Optional[str] = None, content_type: str = "application/json; charset=utf-8"):
        data = _json_safe_response(_schema_wrap(obj) if isinstance(obj, dict) else obj)
        self.send_response(status)
        if cache_control:
            self._cache_control_override = cache_control
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(self, status: int, data: bytes, content_type: str, cache_control: Optional[str] = None):
        self.send_response(status)
        if cache_control:
            self._cache_control_override = cache_control
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_body(self):
        """Read and validate a JSON request body for a mutation endpoint.

        Returns the parsed dict on success, or ``None`` if the request was
        rejected — in which case an error response has already been sent and the
        caller must ``return`` immediately. An empty body (no Content-Length)
        parses to ``{}`` so that bodyless POSTs (logout, sse-token) keep working.

        Hardening vs. the previous "swallow everything, return {}" behaviour:
          * bounds the read by ``MAX_JSON_BODY_BYTES`` (413 on overflow) instead
            of trusting a client-supplied Content-Length;
          * rejects a non-JSON Content-Type when a body is present (415);
          * rejects malformed JSON / non-object bodies (400) instead of silently
            degrading a corrupt payload to ``{}`` for state-changing endpoints.
        """
        headers = getattr(self, "headers", None)
        raw_len = (headers.get("Content-Length", "0") if headers else "0") or "0"
        try:
            n = int(raw_len)
        except (TypeError, ValueError):
            self._send_json(400, {"ok": False, "error": {"code": "BAD_CONTENT_LENGTH", "message": "Content-Length must be a non-negative integer"}})
            return None
        if n < 0:
            self._send_json(400, {"ok": False, "error": {"code": "BAD_CONTENT_LENGTH", "message": "Content-Length must be a non-negative integer"}})
            return None
        if n == 0:
            return {}
        if n > MAX_JSON_BODY_BYTES:
            self._send_json(413, {"ok": False, "error": {"code": "PAYLOAD_TOO_LARGE", "message": f"Request body exceeds {MAX_JSON_BODY_BYTES} bytes"}})
            return None
        content_type = (headers.get("Content-Type", "") if headers else "") or ""
        media_type = content_type.split(";", 1)[0].strip().lower()
        # Lenient on a missing Content-Type (some bodyless-by-convention clients
        # omit it), but reject a body that is explicitly declared as non-JSON.
        if media_type and media_type != "application/json" and not media_type.endswith("+json"):
            self._send_json(415, {"ok": False, "error": {"code": "UNSUPPORTED_MEDIA_TYPE", "message": "Content-Type must be application/json"}})
            return None
        try:
            raw = self.rfile.read(n)
        except Exception:
            self._send_json(400, {"ok": False, "error": {"code": "BODY_READ_ERROR", "message": "failed to read request body"}})
            return None
        try:
            parsed = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_json(400, {"ok": False, "error": {"code": "BAD_JSON", "message": "request body is not valid JSON"}})
            return None
        if not isinstance(parsed, dict):
            self._send_json(400, {"ok": False, "error": {"code": "BAD_JSON", "message": "request body must be a JSON object"}})
            return None
        return parsed

    def do_GET(self):
        if self._reject_untrusted():
            return
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/rvt-sw.js":
            sw_path = _assets_root() / "rvt-sw.js"
            if sw_path.exists():
                self._send_bytes(200, sw_path.read_bytes(), "application/javascript; charset=utf-8", cache_control="no-cache")
            else:
                self._send_json(404, {"ok": False, "error": {"code": "LEGACY_SW_TOMBSTONE_NOT_FOUND", "message": "assets/rvt-sw.js is missing"}})
            return
        if path == "/sw.js":
            sw_path = _assets_root() / "sw.js"
            if sw_path.exists():
                self._send_bytes(200, sw_path.read_bytes(), "application/javascript; charset=utf-8", cache_control="no-cache")
            else:
                self._send_json(404, {"ok": False, "error": {"code": "SW_NOT_FOUND", "message": "assets/sw.js is missing"}})
            return
        if path == "/manifest.webmanifest":
            data = json.dumps(_manifest_payload(self.server), separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            self._send_bytes(200, data, "application/manifest+json; charset=utf-8", cache_control="no-cache")
            return
        if path == "/about":
            self._send_bytes(200, _support_matrix_html(self.server).encode("utf-8"), "text/html; charset=utf-8", cache_control="no-store")
            return
        if path == "/pair":
            self._send_bytes(200, _pair_page_html(self.server).encode("utf-8"), "text/html; charset=utf-8", cache_control="no-store")
            return
        if path.startswith("/icons/") or path.startswith("/lib/") or path.startswith("/fonts/"):
            target = _safe_asset_path(path)
            if not target:
                self._send_json(404, {"ok": False, "error": {"code": "ASSET_NOT_FOUND", "message": "asset not found"}})
                return
            self._send_bytes(200, target.read_bytes(), _content_type_for_asset(target), cache_control="public, max-age=31536000, immutable")
            return
        public_api_paths = {
            "/api/health",
            "/api/version",
            "/api/update/manifest",
        }
        if path.startswith("/api/") and path not in public_api_paths and not self._require_control_auth():
            return
        if path == "/api/update/manifest":
            with _manifest_cache_lock:
                now = time.time()
                if _manifest_cache["data"] is not None and now - _manifest_cache["ts"] < 300:
                    self._send_json(200, _manifest_cache["data"])
                    return
                try:
                    # Fetching under the manifest cache lock ensures only one concurrent request triggers network fetch
                    with urllib.request.urlopen(UPDATE_MANIFEST_URL, timeout=3) as response:
                        payload = json.loads(response.read().decode("utf-8"))
                    _manifest_cache["data"] = payload
                    _manifest_cache["ts"] = now
                    self._send_json(200, payload)
                except Exception as e:
                    self._send_json(502, {"ok": False, "error": {"code": "PROXY_ERROR", "message": f"Failed to fetch update manifest: {str(e)}"}} )
            return
        if path == "/api/auth/validate":
            token = ((self.headers.get("X-RVT-Auth") or self.headers.get("X-RVT-Token") or "") if getattr(self, "headers", None) else "").strip()
            with _OPERATOR_LOCK:
                sess = self.server.operator_sessions.get(token) if (token and hasattr(self.server, "operator_sessions")) else None
                # Snapshot the fields we need while holding the lock; the dict
                # entry itself must not escape the critical section.
                sess = dict(sess) if sess is not None else None
            if sess is not None:
                self._send_json(200, {
                    "ok": True,
                    "operator": {
                        "operator_id": sess.get("operator_id"),
                        "display_name": sess.get("display_name"),
                        "initials": sess.get("initials")
                    }
                })
            else:
                self._send_json(200, {
                    "ok": True,
                    "bootstrap": True,
                    "operator": None
                })
            return
        if path == "/api/health":
            t0 = time.perf_counter()
            payload = {"ok": True, "t": int(time.time() * 1000), "version": VERSION, "latency_ms": round((time.perf_counter() - t0) * 1000, 3)}
            if getattr(self.server, "bind_mode", "local") != "lan":
                payload.update({"feature_flags": FEATURE_FLAGS, "metrics": _system_metrics(self.server.sessions_root)})
            self._send_json(200, payload)
            return
        if path == "/api/version":
            self._send_json(200, {
                "trainer": VERSION,
                "firmware_expected": FIRMWARE_VERSION_EXPECTED,
                "dashboard": DASHBOARD_VERSION,
                "product_version": VERSION,
                "schema_versions": {
                    "control_api": CONTROL_API_SCHEMA_VERSION,
                    "session_notes": SESSION_NOTES_SCHEMA_VERSION,
                    "session_signoff": SESSION_SIGNOFF_SCHEMA_VERSION,
                    "training_progress": TRAINING_PROGRESS_SCHEMA_VERSION,
                    "live_event": LIVE_EVENT_SCHEMA_VERSION,
                    "live_events": LIVE_EVENT_SCHEMA_VERSION,
                    "session_manifest": SESSION_MANIFEST_SCHEMA_VERSION,
                    "chart_annotations": CHART_ANNOTATIONS_SCHEMA_VERSION,
                    "subject_profile": SUBJECT_PROFILE_SCHEMA_VERSION
                },
                "update_manifest_url": UPDATE_MANIFEST_URL
            })
            return
        if path == "/api/ble/scan":
            q = parse_qs(parsed.query)
            timeout_s = float((q.get("timeout_s") or ["3"])[-1] or 3)
            self._send_json(200, _scan_ble_devices_payload(timeout_s=timeout_s))
            return
        if path == "/api/subject-profiles":
            self._send_json(200, _load_subject_profiles(self.server.sessions_root))
            return
        if path == "/api/operator-profiles":
            db = _load_operator_profiles(self.server.sessions_root)
            profiles_list = []
            for op_id, prof in db.get("profiles", {}).items():
                profiles_list.append({
                    "operator_id": prof.get("operator_id"),
                    "display_name": prof.get("display_name"),
                    "initials": prof.get("initials"),
                })
            self._send_json(200, {
                "schema_version": "rvt-operator-profiles-v12.0",
                "profiles": profiles_list
            })
            return
        if path == "/api/server-info":
            payload = {
                "ok": True,
                "origin": _advertised_origin(self.server),
                "scheme": _server_scheme(self.server),
                "bind_mode": getattr(self.server, "bind_mode", "local"),
                "trainer_version": VERSION,
                "dashboard_version": DASHBOARD_VERSION,
                "firmware_expected": FIRMWARE_VERSION_EXPECTED,
                "pair_required": getattr(self.server, "bind_mode", "local") == "lan",
                "active_pin_expires_at": getattr(self.server, "active_pin_expires_at", 0.0),
            }
            self._send_json(200, payload, cache_control="no-store")
            return
        if path == "/api/native-pairing-info":
            client_host = str(self.client_address[0] if self.client_address else "")
            if client_host not in {"127.0.0.1", "::1", "localhost"}:
                self._send_json(403, {"ok": False, "error": {"code": "LOOPBACK_ONLY", "message": "native pairing info is loopback-only"}}, cache_control="no-store")
                return
            pairing_payload = {
                "ok": True,
                "origin": _advertised_origin(self.server),
                "scheme": _server_scheme(self.server),
                "bind_mode": getattr(self.server, "bind_mode", "local"),
                "pair_required": getattr(self.server, "bind_mode", "local") == "lan",
                "active_pin": getattr(self.server, "active_pin", "") or "",
                "active_pin_expires_at": getattr(self.server, "active_pin_expires_at", 0.0),
            }
            if pairing_payload["bind_mode"] == "lan" and parse_qs(parsed.query).get("format", [""])[-1] == "qr":
                pin = pairing_payload["active_pin"]
                pair_url = _advertised_origin(self.server) + (f"/?pair={pin}" if pin else "/pair")
                try:
                    pairing_payload["qr_png_base64"] = base64.b64encode(_qr_png_bytes(pair_url)).decode("ascii")
                    pairing_payload["qr_target_url"] = pair_url
                except Exception:
                    pass  # QR stays optional; the textual pairing link is always present
            self._send_json(200, pairing_payload, cache_control="no-store")
            return
        if path == "/api/session/events" or path == "/api/events/subscribe":
            self._send_sse()
            return
        if path.startswith("/api/sessions/") and path.endswith("/events"):
            sid = unquote(path.split("/")[3])
            self._send_sse(session_id_hint=sid)
            return
        if path == "/api/trainer/log":
            self._send_json(200, {"ok": True, "lines": list(_TRAINER_LOG)[-200:]})
            return
        if path == "/api/status":
            active = {"session_id": "mock", "session_dir": "", "mock": True, "started_at": self.server.started_at} if getattr(self.server, "mock", False) else self.server.supervisor.current()
            self._send_json(200, {"ok": True, "trainer_version": VERSION, "dashboard_version": DASHBOARD_VERSION, "firmware_expected": FIRMWARE_VERSION_EXPECTED, "control_server_started_at": self.server.started_at, "active_session": active, "feature_flags": FEATURE_FLAGS})
            return
        if path == "/api/defaults":
            self._send_json(200, _effective_defaults(self.server.sessions_root))
            return
        if path == "/api/serial/ports":
            self._send_json(200, _serial_ports_payload(str(_effective_defaults(self.server.sessions_root).get("radar_port", DEFAULT_RADAR_PORT))))
            return
        if path == "/api/preflight":
            q = {k: v[-1] for k, v in parse_qs(parsed.query).items() if v}
            include = q.pop("include", None)
            if include:
                q["include"] = [x.strip() for x in str(include).split(",") if x.strip()]
            self._send_json(200, _run_preflight_all(sessions_root=self.server.sessions_root, **q))
            return
        if path == "/api/help/schema":
            self._send_json(200, HELP_SCHEMA)
            return
        if path == "/api/session/current":
            if getattr(self.server, "mock", False):
                self._send_json(200, {"session_id": "mock", "session_dir": "", "mock": True, "started_at": self.server.started_at})
                return
            cur = self.server.supervisor.current()
            self._send_json(200 if cur else 404, cur or {"ok": False, "error": {"code": "NO_ACTIVE_SESSION", "message": "no active session"}})
            return
        if path == "/api/session/current/live_dashboard.json":
            if getattr(self.server, "mock", False):
                self._send_json(200, _mock_live_payload())
                return
            cur = self.server.supervisor.current()
            live_path = Path(str((cur or {}).get("session_dir", ""))) / "live_dashboard.json"
            if cur and live_path.exists():
                self._send_bytes(200, live_path.read_bytes(), "application/json; charset=utf-8")
            else:
                self._send_json(404, {"ok": False, "error": {"code": "NO_LIVE_DASHBOARD", "message": "active live_dashboard.json is not available"}})
            return
        if path == "/api/session/buffer":
            q = parse_qs(parsed.query)
            seconds = int((q.get("seconds") or ["60"])[-1] or 60)
            if getattr(self.server, "mock", False):
                self._send_json(200, {"ok": True, "schema_version": LIVE_EVENT_SCHEMA_VERSION, "session_id": "mock", "buffer_s": seconds, "payload": _mock_live_payload(window_s=seconds)})
                return
            cur = self.server.supervisor.current()
            if not cur:
                self._send_json(404, {"ok": False, "error": {"code": "NO_ACTIVE_SESSION", "message": "no active session"}})
                return
            live_path = Path(str(cur.get("session_dir", ""))) / "live_dashboard.json"
            payload = _read_json_if_exists(str(live_path)) if live_path.exists() else None
            self._send_json(200 if payload else 404, {"ok": bool(payload), "schema_version": LIVE_EVENT_SCHEMA_VERSION, "session_id": cur.get("session_id"), "buffer_s": seconds, "payload": payload})
            return
        if path == "/api/sessions":
            self._send_json(200, {"root": self.server.sessions_root, "items": _scan_sessions_root(self.server.sessions_root)})
            return
        if path.startswith("/api/sessions/") and "/files/" in path:
            parts = path.split("/")
            sid = unquote(parts[3]) if len(parts) > 3 else ""
            rel = unquote(path.split("/files/", 1)[1]) if "/files/" in path else ""
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            target = (root / rel).resolve()
            try:
                target.relative_to(root)
            except ValueError:
                self._send_json(404, {"ok": False, "error": {"code": "FILE_NOT_FOUND", "message": "session file not found"}})
                return
            if not target.exists() or not target.is_file():
                self._send_json(404, {"ok": False, "error": {"code": "FILE_NOT_FOUND", "message": "session file not found"}})
                return
            self._send_bytes(200, target.read_bytes(), mimetypes.guess_type(str(target))[0] or "application/octet-stream")
            return
        if path.startswith("/api/sessions/") and path.endswith("/annotations"):
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            self._send_json(200, _load_session_annotations_payload(root, sid))
            return
        if path.startswith("/api/sessions/") and path.endswith("/notes"):
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            notes_path = root / "session_notes.json"
            existing = _read_json_if_exists(str(notes_path)) or {}
            notes = existing.get("notes") if isinstance(existing.get("notes"), list) else []
            updated_at = existing.get("updated_at")
            if not updated_at and notes_path.exists():
                updated_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(notes_path.stat().st_mtime))
            self._send_json(200, {
                "schema_version": SESSION_NOTES_SCHEMA_VERSION,
                "session_id": sid,
                "updated_at": updated_at,
                "review_summary": str(existing.get("review_summary") or ""),
                "notes": notes,
            })
            return
        if path.startswith("/api/sessions/") and path.endswith("/signoff"):
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            existing = _read_json_if_exists(str(root / "session_signoff.json")) or {}
            self._send_json(200, {
                "schema_version": SESSION_SIGNOFF_SCHEMA_VERSION,
                "session_id": sid,
                "operator_name": str(existing.get("operator_name") or ""),
                "initials": str(existing.get("initials") or ""),
                "validation_comment": str(existing.get("validation_comment") or ""),
                "signed_at": existing.get("signed_at"),
                "updated_at": existing.get("updated_at"),
            })
            return
        if path.startswith("/api/sessions/") and path.endswith("/summary"):
            sid = unquote(path.split("/")[3])
            try:
                self._send_json(200, _load_session_summary(str(_session_path(self.server.sessions_root, sid))))
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
            return
        if path.startswith("/api/sessions/") and path.endswith("/data"):
            sid = unquote(path.split("/")[3])
            points = int((parse_qs(parsed.query).get("points") or ["1000"])[-1] or 1000)
            try:
                self._send_json(200, _session_data_payload(self.server.sessions_root, sid, points=points))
            except Exception as e:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": str(e)}})
            return
        if path.startswith("/api/sessions/") and path.endswith("/compare"):
            sid = unquote(path.split("/")[3])
            try:
                _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            self._send_json(200, _compare_session_payload(self.server.sessions_root, sid))
            return
        if path.startswith("/api/sessions/") and path.endswith("/analyse/status"):
            sid = unquote(path.split("/")[3])
            try:
                _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            self._send_json(200, _analysis_job_status(self.server.sessions_root, sid))
            return
        if path.startswith("/api/sessions/") and path.endswith("/training/status"):
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            progress = _read_json_if_exists(str(root / "analysis" / "training_progress.json"))
            if isinstance(progress, dict):
                if str(progress.get("status", "")).lower() == "completed":
                    progress["status"] = "complete"
                progress.setdefault("schema_version", TRAINING_PROGRESS_SCHEMA_VERSION)
                progress.setdefault("session_id", sid)
                progress.setdefault("updated_at", _iso_now())
                progress.setdefault("target", "hr,rr")
                progress.setdefault("n_estimators_done", 0)
                progress.setdefault("n_estimators_total", 0)
                progress.setdefault("train_loss", None)
                progress.setdefault("val_loss", None)
                progress.setdefault("elapsed_s", 0.0)
                self._send_json(200, progress)
                return
            self._send_json(200, {
                "schema_version": TRAINING_PROGRESS_SCHEMA_VERSION,
                "session_id": sid,
                "status": "idle",
                "target": "hr,rr",
                "n_estimators_done": 0,
                "n_estimators_total": 0,
                "train_loss": None,
                "val_loss": None,
                "elapsed_s": 0.0,
                "updated_at": _iso_now(),
            })
            return
        if path.startswith("/api/sessions/") and path.endswith("/predict"):
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            candidates = [root / "predict_summary.json", root / "analysis" / "predict_summary.json", root.parent / "model_out" / "predict_summary.json"]
            found = next((p for p in candidates if p.exists()), None)
            self._send_json(200, {"ok": bool(found), "session_id": sid, "summary": (_read_json_if_exists(str(found)) if found else None), "path": str(found) if found else None})
            return
        if path == "/api/report/export":
            sid = (parse_qs(parsed.query).get("session") or [""])[-1]
            if not sid:
                self._send_json(400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "session query parameter is required"}})
                return
            try:
                html = _build_report_export_html(str(_session_path(self.server.sessions_root, sid)))
                self._send_bytes(200, html.encode("utf-8"), "text/html; charset=utf-8")
            except FileNotFoundError as e:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": str(e)}})
            return
        if path.startswith("/api/"):
            self._send_json(404, {"ok": False, "error": "not_found"})
            return
        dashboard_routes = {f"/{str(name)}" for name in _DASHBOARD_HTML_FALLBACK_NAMES}
        if path in ("/", "/index.html", "/connect", "/dashboard", "/live", "/home", "/settings", "/report", "/help"):
            self.path = f"/{_DASHBOARD_HTML_NAME}"
            path = f"/{_DASHBOARD_HTML_NAME}"
        if path == "/live_dashboard.html":
            self.path = f"/{_DASHBOARD_HTML_NAME}"
            path = f"/{_DASHBOARD_HTML_NAME}"
        if path in dashboard_routes:
            template_path = _dashboard_html_template_path()
            if template_path is None:
                self._send_bytes(200, _load_dashboard_template_text().encode("utf-8"), "text/html; charset=utf-8")
                return
            self._send_bytes(200, template_path.read_bytes(), "text/html; charset=utf-8")
            return

        # Serve compiled Angular chunks and assets from www/
        clean_name = path.lstrip("/")
        www_root = (_REPO_ROOT / "www").resolve()
        try:
            www_target = (www_root / clean_name).resolve()
            www_target.relative_to(www_root)
            is_safe = True
        except ValueError:
            is_safe = False

        if is_safe and not clean_name.startswith(".rvt_tls") and www_target.is_file():
            self._send_bytes(200, www_target.read_bytes(), _content_type_for_asset(www_target))
            return

        self._send_json(404, {"ok": False, "error": {"code": "NOT_FOUND", "message": "public resource not found"}})

    def do_POST(self):
        if self._reject_untrusted():
            return
        path = urlparse(self.path).path
        body = self._read_body()
        if body is None:
            return
        if path == "/api/auth/exchange":
            client_ip = (self.client_address[0] if self.client_address else "") or "unknown"
            status, payload = _exchange_pair_pin(self.server, str(body.get("pin") or ""), client_ip)
            self._send_json(status, payload, cache_control="no-store")
            return
        if path.startswith("/api/") and not self._require_control_auth():
            return
        if path == "/api/operator-profiles":
            status, payload = _create_operator_profile(self.server, body)
            self._send_json(status, payload)
            return
        if path == "/api/auth/login":
            status, payload = _login_operator(self.server, str(body.get("operator_id") or ""), str(body.get("pin") or ""))
            self._send_json(status, payload)
            return
        if path == "/api/auth/logout":
            token = ((self.headers.get("X-RVT-Auth") or self.headers.get("X-RVT-Token") or "") if getattr(self, "headers", None) else "").strip()
            if not token:
                token = (parse_qs(urlparse(self.path).query).get("token") or [""])[-1].strip()
            # Logout must also drop any SSE tokens this operator minted; otherwise
            # a short-lived SSE token outlives the session for its ~30s TTL.
            # All operator_sessions / sse_tokens access goes through _OPERATOR_LOCK.
            with _OPERATOR_LOCK:
                if token and hasattr(self.server, "operator_sessions") and token in self.server.operator_sessions:
                    operator_id = self.server.operator_sessions.pop(token, {}).get("operator_id")
                    _invalidate_operator_sessions(self.server, operator_id)
            self._send_json(200, {"ok": True})
            return
        if path == "/api/auth/sse-token":
            token = secrets.token_urlsafe(24)
            # Bind the SSE token to the requesting operator so session
            # invalidation (reset / host-reset / logout) can also drop it.
            # _require_control_auth set current_operator_id when a valid operator
            # session authorised this request; bootstrap/no-operator => None.
            operator_id = getattr(self, "current_operator_id", None)
            now = time.time()
            with _OPERATOR_LOCK:
                if not hasattr(self.server, "sse_tokens"):
                    self.server.sse_tokens = {}
                # Reap expired SSE tokens to prevent unbounded memory growth
                expired = [t for t, s in self.server.sse_tokens.items() if now >= s.get("expires_at", 0.0)]
                for t in expired:
                    self.server.sse_tokens.pop(t, None)
                self.server.sse_tokens[token] = {
                    "expires_at": now + 30.0,
                    "operator_id": operator_id,
                }
            self._send_json(200, {"sse_token": token})
            return
        if path == "/api/auth/reset-pin":
            status, payload = _reset_pin_with_recovery(
                self.server,
                str(body.get("operator_id") or ""),
                str(body.get("recovery_code") or ""),
                str(body.get("new_pin") or ""),
            )
            self._send_json(status, payload, cache_control="no-store")
            return
        if path == "/api/auth/host-reset":
            client_host = str(self.client_address[0] if self.client_address else "")
            if client_host not in {"127.0.0.1", "::1", "localhost"}:
                self._send_json(403, {"ok": False, "error": {"code": "LOOPBACK_ONLY", "message": "host-reset is loopback-only"}}, cache_control="no-store")
                return
            status, payload = _host_reset_pin(
                self.server,
                str(body.get("operator_id") or ""),
                str(body.get("new_pin") or ""),
            )
            self._send_json(status, payload, cache_control="no-store")
            return
        if path == "/api/defaults":
            current = _effective_defaults(self.server.sessions_root)
            current.update({k: v for k, v in body.items() if k in current})
            save_json(current, str(Path(self.server.sessions_root) / ".user_defaults.json"))
            self._send_json(200, current)
            return
        if path.startswith("/api/preflight/"):
            check_id = path.rsplit("/", 1)[-1]
            if check_id in {"serial_port_probe", "ble_device_probe"} and self.server.supervisor.current():
                self._send_json(409, {"ok": False, "error": {"code": "SESSION_IN_PROGRESS", "message": "hardware preflight is blocked while a session is active"}})
                return
            self._send_json(200, _run_preflight_check(check_id, sessions_root=self.server.sessions_root, **body))
            return
        if path == "/api/session/start":
            radar_port = body.get("radar_port", DEFAULT_RADAR_PORT)
            if str(radar_port).strip().lower() in {"auto", "autodetect", "auto-detect"}:
                radar_port = _auto_detect_radar_port(DEFAULT_RADAR_PORT)
            ble_address = body.get("ble_address", DEFAULT_BLE_ADDRESS)
            if self.server.supervisor.current():
                self._send_json(409, {"ok": False, "error": {"code": "SESSION_IN_PROGRESS", "message": "active session already running"}})
                return
            structural = ["python_env", "firmware_file_present", "serial_port_list", "session_folder_writable", "disk_space", "schema_hash_consistency", "clock_monotonic_sanity", "ble_adapter"]
            try:
                report = _run_preflight_all(sessions_root=self.server.sessions_root, port=radar_port, address=ble_address, include=structural)
            except Exception as e:
                self._send_json(500, {"ok": False, "error": {"code": "PREFLIGHT_ERROR", "message": str(e)}})
                return
            failed = [{"id": c.get("id"), "label": c.get("label"), "detail": c.get("detail"), "remediation": c.get("remediation")} for c in report.get("checks", []) if c.get("status") == "fail"]
            if failed:
                self._send_json(424, {"ok": False, "error": {"code": "PREFLIGHT_FAILED", "message": "one or more structural preflight checks failed", "failed": failed}})
                return
            try:
                advanced = body.get("advanced") if isinstance(body.get("advanced"), dict) else {}
                result = self.server.supervisor.start(
                    duration_s=body.get("duration_s"),
                    radar_port=radar_port,
                    ble_address=ble_address,
                    ble_profile=body.get("ble_profile", "ailink_oximeter"),
                    timeout_s=float(body.get("timeout_s", 6.0) or 6.0),
                    subject_label=body.get("subject_label"),
                    operator_label=body.get("operator_label"),
                    subject_profile_id=body.get("subject_profile_id", "adult_default"),
                    notify_char=advanced.get("notify_char"),
                    dashboard_refresh_s=advanced.get("dashboard_refresh_s"),
                )
                self._send_json(200, result)
            except RuntimeError as e:
                code = "SESSION_IN_PROGRESS" if "SESSION_IN_PROGRESS" in str(e) else "SPAWN_ERROR"
                self._send_json(409 if code == "SESSION_IN_PROGRESS" else 500, {"ok": False, "error": {"code": code, "message": str(e)}})
            except TimeoutError as e:
                self._send_json(500, {"ok": False, "error": {"code": "SPAWN_TIMEOUT", "message": str(e)}})
            return
        if path == "/api/session/stop":
            try:
                self._send_json(200, self.server.supervisor.stop(reason=str(body.get("reason", "user_request"))))
            except RuntimeError as e:
                self._send_json(404, {"ok": False, "error": {"code": "NO_ACTIVE_SESSION", "message": str(e)}})
            return
        if path == "/api/session/annotate":
            cur = self.server.supervisor.current()
            if not cur:
                self._send_json(404, {"ok": False, "error": {"code": "NO_ACTIVE_SESSION", "message": "no active session"}})
                return
            notes_path = Path(str(cur.get("session_dir", ""))) / "session_notes.json"
            existing = _read_json_if_exists(str(notes_path)) or {}
            notes = existing.get("notes") if isinstance(existing.get("notes"), list) else []
            entry = {
                "id": f"note_{int(time.time() * 1000)}",
                "t_s": body.get("t_s"),
                "note": _sanitize_user_string(body.get("note", "")),
                "created_at": _iso_now(),
            }
            notes.append(entry)
            existing.update({"schema_version": SESSION_NOTES_SCHEMA_VERSION, "session_id": cur.get("session_id"), "notes": notes, "updated_at": _iso_now()})
            save_json(existing, str(notes_path))
            self._send_json(200, {"ok": True, "entry": entry})
            return
        if path == "/api/session/annotations":
            cur = self.server.supervisor.current()
            if not cur:
                self._send_json(404, {"ok": False, "error": {"code": "NO_ACTIVE_SESSION", "message": "no active session"}})
                return
            session_id = str(cur.get("session_id") or "")
            session_dir = Path(str(cur.get("session_dir", "")))
            action = str(body.get("action") or "upsert").lower()
            chart_key = str(body.get("chart_key") or body.get("chartKey") or "chart")
            annotation = body.get("annotation") if isinstance(body.get("annotation"), dict) else {}
            entry = _upsert_session_annotation(session_dir, session_id, chart_key, annotation, action=action)
            self._send_json(200, {"ok": True, "entry": entry, "action": action})
            return
        if path.startswith("/api/sessions/") and path.endswith("/analyse"):
            sid = unquote(path.split("/")[3])
            try:
                self._send_json(200, _rerun_session_analysis(str(_session_path(self.server.sessions_root, sid))))
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
            return
        if path.startswith("/api/"):
            self._send_json(404, {"ok": False, "error": "not_found"})
            return
        self._send_json(404, {"ok": False, "error": "not_found"})

    def do_PUT(self):
        if self._reject_untrusted():
            return
        if not self._require_control_auth():
            return
        path = urlparse(self.path).path
        body = self._read_body()
        if body is None:
            return
        if path.startswith("/api/sessions/") and path.endswith("/notes"):
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            review_summary = _sanitize_user_string(body.get("review_summary", ""), 4000).strip()
            notes_path = root / "session_notes.json"
            existing = _read_json_if_exists(str(notes_path)) or {}
            existing.update({
                "schema_version": SESSION_NOTES_SCHEMA_VERSION,
                "session_id": sid,
                "review_summary": review_summary,
                "notes": existing.get("notes") if isinstance(existing.get("notes"), list) else [],
                "updated_at": _iso_now(),
            })
            save_json(existing, str(notes_path))
            self._send_json(200, existing)
            return
        if path.startswith("/api/sessions/") and path.endswith("/signoff"):
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            operator_name = _sanitize_user_string(body.get("operator_name", ""), 120).strip()
            initials = str(body.get("initials") or "").strip().upper()
            validation_comment = _sanitize_user_string(body.get("validation_comment", ""), 500).strip()
            if not operator_name or not re.fullmatch(r"[A-Z]{2,5}", initials):
                self._send_json(400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "operator_name and 2-5 uppercase initials are required"}})
                return
            signoff = {
                "schema_version": SESSION_SIGNOFF_SCHEMA_VERSION,
                "session_id": sid,
                "operator_name": operator_name,
                "initials": initials,
                "validation_comment": validation_comment,
                "signed_at": _iso_now(),
                "updated_at": _iso_now(),
            }
            save_json(signoff, str(root / "session_signoff.json"))
            self._send_json(200, signoff)
            return
        if path.startswith("/api/sessions/") and path.endswith("/tags"):
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            manifest_path = root / "session_manifest.json"
            manifest = _read_json_if_exists(str(manifest_path)) or {}
            tags = body.get("tags", [])
            if not isinstance(tags, list):
                self._send_json(400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "tags must be a list"}})
                return
            manifest["schema_version"] = SESSION_MANIFEST_SCHEMA_VERSION
            manifest["manifest_version"] = SESSION_MANIFEST_VERSION
            manifest["tags"] = [_sanitize_user_string(t, 80).strip() for t in tags if str(t).strip()]
            manifest["updated_at"] = _iso_now()
            save_json(manifest, str(manifest_path))
            self._send_json(200, {"ok": True, "session_id": sid, "tags": manifest["tags"]})
            return
        self._send_json(404, {"ok": False, "error": "not_found"})

    def do_DELETE(self):
        if self._reject_untrusted():
            return
        if not self._require_control_auth():
            return
        path = urlparse(self.path).path
        if path.startswith("/api/sessions/"):
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            active = self.server.supervisor.current() if getattr(self.server, "supervisor", None) else None
            active_dir = os.path.abspath(str((active or {}).get("session_dir", ""))) if isinstance(active, dict) else ""
            if active_dir and active_dir == os.path.abspath(str(root)):
                self._send_json(409, {"ok": False, "error": {"code": "SESSION_ACTIVE", "message": "stop the active session before deleting it"}})
                return
            lock = _read_json_if_exists(_lock_path(self.server.sessions_root)) or {}
            lock_dir = os.path.abspath(str(lock.get("session_dir", ""))) if isinstance(lock, dict) else ""
            if lock_dir and lock_dir == os.path.abspath(str(root)):
                self._send_json(409, {"ok": False, "error": {"code": "SESSION_LOCKED", "message": "session lock is active; stop or clear the session before deleting it"}})
                return
            trash = Path(self.server.sessions_root) / ".trash"
            trash.mkdir(parents=True, exist_ok=True)
            target = trash / sid
            if target.exists():
                target = trash / f"{sid}_{int(time.time())}"
            shutil.move(str(root), str(target))
            self._send_json(200, {"ok": True, "session_id": sid, "trashed_path": str(target), "retention_hint": "soft-deleted; clean .trash after local retention review"})
            return
        self._send_json(404, {"ok": False, "error": "not_found"})


class _ControlServer:
    def __init__(
        self,
        host: str,
        port: int,
        sessions_root: str,
        cors_origin: str = "",
        mock: bool = False,
        bind_mode: str = "local",
        tls_cert: Optional[str] = None,
        tls_key: Optional[str] = None,
        tls_trusted: bool = False,
    ):
        self.sessions_root = os.path.abspath(sessions_root)
        os.makedirs(self.sessions_root, exist_ok=True)
        _cleanup_json_temp_files(self.sessions_root)
        handler = partial(_ControlHandler, directory=str(_REPO_ROOT))
        self.httpd = ThreadingHTTPServer((host, int(port)), handler)
        if tls_cert and tls_key:
            _ensure_self_signed_cert(tls_cert, tls_key, host if host not in {"0.0.0.0", "::"} else _guess_lan_ip())
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(tls_cert, tls_key)
            self.httpd.socket = ctx.wrap_socket(self.httpd.socket, server_side=True)
        self.httpd.sessions_root = self.sessions_root
        self.httpd.supervisor = _SessionSupervisor(self.sessions_root)
        self.httpd.started_at = _iso_now()
        self.httpd.cors_origin = cors_origin
        self.httpd.mock = bool(mock)
        self.httpd.bind_mode = bind_mode
        self.httpd.tls_enabled = bool(tls_cert and tls_key)
        self.httpd.tls_trusted = bool(tls_trusted)
        self.httpd.advertised_host = _guess_lan_ip() if bind_mode == "lan" else "127.0.0.1"
        self.httpd.auth_tokens = set()
        self.httpd.pair_pins = {}
        self.httpd.pair_exchange_failures = {}
        self.httpd.active_pin = ""
        self.httpd.active_pin_expires_at = 0.0
        self.httpd.operator_sessions = {}
        self.httpd.sse_tokens = {}
        if bind_mode == "lan":
            _make_pair_pin(self.httpd)
        origin = _advertised_origin(self.httpd)
        connect_src = "'self'" if bind_mode == "local" else f"'self' {origin}"
        self.httpd.content_security_policy = (
            "default-src 'self'; "
            "script-src 'self' 'unsafe-inline'; "
            "style-src 'self' 'unsafe-inline'; "
            "img-src 'self' data: blob:; "
            f"connect-src {connect_src}; "
            "font-src 'self'; worker-src 'self'; manifest-src 'self'; "
            "frame-ancestors 'none'; base-uri 'self'"
        )
        self.supervisor = self.httpd.supervisor
        self.sentinel = Path(self.sessions_root).parent / "control_server.json"
        save_json({"pid": os.getpid(), "port": self.httpd.server_port, "origin": origin, "started_at": self.httpd.started_at}, str(self.sentinel))
        _append_trainer_log(f"[CONTROL] server started on {host}:{self.httpd.server_port}")
        self._serving = False
        original_serve_forever = self.httpd.serve_forever
        original_shutdown = self.httpd.shutdown

        def _serve_forever_with_flag(*args, **kwargs):
            self._serving = True
            try:
                return original_serve_forever(*args, **kwargs)
            finally:
                self._serving = False

        def _shutdown_with_cleanup():
            try:
                if self._serving:
                    original_shutdown()
            finally:
                try:
                    self.sentinel.unlink()
                except FileNotFoundError:
                    pass
        self.httpd.serve_forever = _serve_forever_with_flag
        self.httpd.shutdown = _shutdown_with_cleanup

    def start(self):
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def _start_control_server(args):
    sessions_root = os.path.abspath(str(getattr(args, "sessions_root", "sessions")))
    sentinel = Path(sessions_root).parent / "control_server.json"
    if sentinel.exists():
        existing = _read_json_if_exists(str(sentinel)) or {}
        if _pid_alive(existing.get("pid")):
            raise RuntimeError(f"control server already running on port {existing.get('port')}")
        try:
            sentinel.unlink()
        except Exception:
            pass
    bind_mode = str(getattr(args, "bind", "local") or "local")
    requested_host = str(getattr(args, "host", "") or "")
    host = requested_host or ("0.0.0.0" if bind_mode == "lan" else "127.0.0.1")
    port_arg = getattr(args, "control_port", 8765)
    start_port = int(8765 if port_arg is None else port_arg)
    cors_origin = str(getattr(args, "cors_origin", "") or "")
    if bind_mode == "lan" and cors_origin == "*":
        if not bool(getattr(args, "allow_wildcard_cors_lan", False)):
            raise RuntimeError(
                "Refusing --cors-origin '*' with --bind lan. "
                "Use --allow-wildcard-cors-lan only for an isolated lab network."
            )
        print(_yellow("[WARN] Control server is bound to LAN interfaces with a wildcard CORS origin."))
        print(_yellow("       This allows cross-origin requests from any website opened on your browser."))
    mock = bool(getattr(args, "mock", False))
    tls_arg = getattr(args, "tls", None)
    tls_cert = None
    tls_key = None
    if tls_arg is not None:
        if isinstance(tls_arg, list) and len(tls_arg) >= 2:
            tls_cert, tls_key = tls_arg[0], tls_arg[1]
        else:
            tls_dir = _REPO_ROOT / ".rvt_tls"
            tls_cert, tls_key = str(tls_dir / "cert.pem"), str(tls_dir / "key.pem")
    tls_trusted = bool(getattr(args, "tls_trusted", False))
    last_error = None
    ports = [0] if start_port == 0 else [start_port]
    for port in ports:
        try:
            return _ControlServer(host, port, sessions_root, cors_origin=cors_origin, mock=mock, bind_mode=bind_mode, tls_cert=tls_cert, tls_key=tls_key, tls_trusted=tls_trusted)
        except OSError as e:
            last_error = e
            if start_port == 0:
                break
    if start_port != 0:
        raise RuntimeError(f"Port {start_port} in use. Stop the other instance or pass --port <N>.")
    raise RuntimeError(f"could not bind control server on an automatic port: {last_error}")


def cmd_serve(args):
    try:
        server = _start_control_server(args)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)
    url = f"{_advertised_origin(server.httpd)}/?v={int(time.time())}"
    print(f"[CONTROL] {url}")
    print(f"[CONTROL] support matrix: {_advertised_origin(server.httpd)}/about")
    if getattr(server.httpd, "bind_mode", "local") == "lan":
        pin = getattr(server.httpd, "active_pin", "")
        print(f"[PAIR] open {_advertised_origin(server.httpd)}/pair")
        print(f"[PAIR] one-time PIN: {pin} (expires in 5 minutes; consumed after first exchange)")
    if not getattr(args, "no_browser", False):
        try:
            import webbrowser
            webbrowser.open(url)
        except Exception as e:
            warn(f"Could not open dashboard in browser: {e}")
    try:
        server.httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


def _row_get(row: Optional[Dict[str, str]], *keys: str):
    row = row or {}
    for key in keys:
        if key in row and row.get(key) not in (None, ""):
            return row.get(key)
    return None


def _firmware_contract_candidates() -> List[Path]:
    roots = [
        _REPO_ROOT,
        Path(os.getcwd()),
    ]
    relatives = [
        Path("radar_vital_v16_3_0.ino"),
        Path("radar_vital_v15_0_0.ino"),
        Path("radar_vital_v14_0_0.ino"),
        Path("radar_vital_v14.ino"),
        Path("radar_vital_v13_9_8.ino"),
        Path("radar_vital_v13_9_7.ino"),
        Path("radar_vital_v13_9d.ino"),
        Path("radar_vital_v13_9c.ino"),
        Path("radar_vital_v13_9b.ino"),
        Path("radar_vital_v13_9a.ino"),
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
    text = re.sub(r"//.*", "", text)
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
        details = []
        for path, mismatch_idx, trainer_col, fw_col, actual_len in mismatches[:4]:
            details.append(
                f"file={path}, index={mismatch_idx}, trainer={trainer_col!r}, firmware={fw_col!r}, "
                f"trainer_count={len(expected)}, firmware_count={actual_len}"
            )
        raise RuntimeError(
            "Firmware DATA header mismatch: no candidate matched the trainer contract. "
            + " | ".join(details)
            + f". Searched: {', '.join(searched)}"
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
    # v12 trainer accepts v15.1 (219), v15.0 (207), and v14.1 (199) schemas.
    # Older legacy counts retained for back-compat with archived sessions.
    accepted_legacy_counts = (LEGACY_RADAR_LOG_COLUMN_COUNT, 136, 180, 195, LEGACY_V14_COLUMN_COUNT, LEGACY_V15_COLUMN_COUNT)
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

    reported_hr = radar.get("reported_hr")
    reported_rr = radar.get("reported_rr")
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
        ml_status = ml.get("status", "PASS" if ml.get("passed") else "FAIL")
        lines.append(f"ml_gate: {ml_status} | "
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
        raw_hr_uncorr_bias_est = analyse_summary.get("raw_hr_uncorrected_bias_estimate", {}) if isinstance(analyse_summary.get("raw_hr_uncorrected_bias_estimate"), dict) else {}
        raw_hr_corr_bias_est = analyse_summary.get("raw_hr_corrected_bias_estimate", {}) if isinstance(analyse_summary.get("raw_hr_corrected_bias_estimate"), dict) else {}
        raw_hr_bias = _to_num(raw_hr_bias_est.get("bias_bpm"), float("nan"))
        try:
            raw_hr_bias_n = int(float(raw_hr_bias_est.get("n", 0) or 0)) if raw_hr_bias_est else 0
        except Exception:
            raw_hr_bias_n = 0
        if np.isfinite(raw_hr_bias):
            lines.append(f"raw_hr_bias_estimate: {raw_hr_bias:+.2f} BPM (n={raw_hr_bias_n})")
        raw_hr_uncorr_bias = _to_num(raw_hr_uncorr_bias_est.get("bias_bpm"), float("nan"))
        raw_hr_corr_bias = _to_num(raw_hr_corr_bias_est.get("bias_bpm"), float("nan"))
        if np.isfinite(raw_hr_uncorr_bias):
            lines.append(f"raw_hr_uncorrected_bias_estimate: {raw_hr_uncorr_bias:+.2f} BPM (n={_safe_int(raw_hr_uncorr_bias_est.get('n'), 0)})")
        if np.isfinite(raw_hr_corr_bias):
            lines.append(f"raw_hr_corrected_bias_estimate: {raw_hr_corr_bias:+.2f} BPM (n={_safe_int(raw_hr_corr_bias_est.get('n'), 0)})")
            improvement = raw_hr_bias - raw_hr_corr_bias if np.isfinite(raw_hr_bias) else float("nan")
            if np.isfinite(improvement):
                lines.append(f"raw_hr_correction_delta: {improvement:+.2f} BPM bias improvement")
        buckets = raw_hr_bias_est.get("rate_buckets", [])
        if isinstance(buckets, list) and buckets:
            parts = []
            for b in buckets:
                if not isinstance(b, dict):
                    continue
                n_b = _safe_int(b.get("n"), 0)
                bias_b = _to_num(b.get("bias_bpm"), float("nan"))
                if n_b > 0 and np.isfinite(bias_b):
                    parts.append(f"{b.get('bucket')}={bias_b:+.1f}({n_b})")
            if parts:
                lines.append("raw_hr_bias_buckets: " + " | ".join(parts))
        ble_quality = analyse_summary.get("ble_ref_quality", {})
        if isinstance(ble_quality, dict) and ble_quality:
            lines.append(f"ble_ref_quality: status={ble_quality.get('status', 'unknown')} | "
                         f"raw={_safe_int(ble_quality.get('raw_packets'), 0)} | "
                         f"parsed={_safe_int(ble_quality.get('parsed_rows'), 0)} | "
                         f"loss={_to_num(ble_quality.get('packet_loss_pct'), float('nan')):.1f}% | "
                         f"decode_err={_to_num(ble_quality.get('decode_error_pct'), float('nan')):.1f}% | "
                         f"PI_low={_to_num(ble_quality.get('pi_below_threshold_pct'), float('nan')):.1f}%")
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
        exp_audit = analyse_summary.get("experimental_audit", {})
        if isinstance(exp_audit, dict):
            no_early = exp_audit.get("combined_without_early_window", {})
            no_half = exp_audit.get("combined_without_suspect_half_rate", {})
            no_both = exp_audit.get("combined_without_early_or_suspect_half_rate", {})
            lines.append(f"experimental_audit: early_excluded={_safe_int(exp_audit.get('early_window_excluded_n'), 0)} | "
                         f"suspect_half_rate_excluded={_safe_int(exp_audit.get('suspect_half_rate_excluded_n'), 0)}")
            if isinstance(no_both, dict):
                lines.append(f"experimental_best_audit: r={_to_num(no_both.get('r'), float('nan')):.3f} | "
                             f"rmse={_to_num(no_both.get('rmse'), float('nan')):.2f} | "
                             f"spearman={_to_num(no_both.get('spearman_r'), float('nan')):.3f} | "
                             f"n={_safe_int(no_both.get('n'), 0)}")
        golden = analyse_summary.get("golden_check")
        if isinstance(golden, dict):
            lines.append(f"golden_check: {'PASS' if golden.get('passed') else 'FAIL'}")
    report_path = os.path.join(session_dir, "session_quick_report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    quick_report_path = os.path.join(session_dir, "quick_report.txt")
    with open(quick_report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print("\n" + _bold("=== SESSION QUICK REPORT ==="))
    print("\n".join(lines))
    print(_green(f"[OUT] Quick report saved to {report_path}"))

def cmd_session(args):
    if str(getattr(args, "port", "")).strip().lower() in {"auto", "autodetect", "auto-detect"}:
        args.port = _auto_detect_radar_port(DEFAULT_RADAR_PORT)
    auto_session_dir = str(args.session_dir).strip().lower() == "auto"
    if str(args.session_dir).strip().lower() == "auto":
        session_dir = _next_session_dir(os.path.abspath(args.sessions_root), args.session_prefix, args.session_digits)
    else:
        session_dir = os.path.abspath(args.session_dir)
    os.makedirs(session_dir, exist_ok=True)
    lock_root = os.path.abspath(args.sessions_root) if auto_session_dir else os.path.dirname(session_dir)
    _check_stale_session_lock(lock_root)
    if _session_is_active(lock_root):
        raise RuntimeError(f"SESSION_IN_PROGRESS: active session lock exists at {_lock_path(lock_root)}")
    _write_session_lock(lock_root, session_dir)
    lock_acquired = True
    analysis_dir = os.path.join(session_dir, "analysis")
    radar_out = os.path.join(session_dir, "radar.csv")
    ref_out = os.path.join(session_dir, "ref.csv")
    script_path = str(_TRAINER_ENTRYPOINT)
    initial_manifest = {
        "schema_version": SESSION_MANIFEST_SCHEMA_VERSION,
        "manifest_version": SESSION_MANIFEST_VERSION,
        "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "trainer_version": VERSION,
        "dashboard_version": DASHBOARD_VERSION,
        "auto_analysed": False,
        "tags": [],
        "notes_count": 0,
        "subject_profile_id": getattr(args, "subject_profile_id", "adult_default"),
    }
    try:
        save_json(initial_manifest, os.path.join(session_dir, "session_manifest.json"))
    except Exception:
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
        raise

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

    try:
        if dashboard_enabled:
            _write_live_dashboard_html(session_dir)
            _write_live_dashboard_json(session_dir, "starting", 0.0, args.duration_s, radar_tracker, ref_tracker, raw_tracker, ["[INFO] Session starting"], analysis_summary=_dashboard_analysis_payload(session_dir))
            try:
                dashboard_server = _start_dashboard_server(session_dir, port=int(getattr(args, "dashboard_port", 8765) or 0))
                print(_green(f"[DASHBOARD] {dashboard_server.url}"))
                if getattr(args, "open_dashboard", True):
                    try:
                        import webbrowser
                        webbrowser.open(dashboard_server.url)
                    except Exception as e:
                        warn(f"Could not open dashboard in browser: {e}")
            except Exception as e:
                port = int(getattr(args, "dashboard_port", 8765) or 0)
                if port != 0:
                    print(f"Port {port} in use. Stop the other instance or pass --dashboard-port <N>.", file=sys.stderr)
                    raise SystemExit(2)
                dashboard_enabled = False
                warn(f"Could not start dashboard server: {e}")
    except Exception:
        if dashboard_server is not None:
            dashboard_server.stop()
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
        raise

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
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
        return

    if not getattr(args, "auto_analyse", True):
        warn("Auto analyse disabled by --no-auto-analyse.")
        _write_session_manifest(session_dir, [radar_out], [ref_out], analysis_dir, None)
        if dashboard_server is not None:
            dashboard_server.stop()
        if lock_acquired:
            _release_session_lock(lock_root)
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
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
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
    if not dashboard_enabled:
        try:
            radar_tracker.poll(); ref_tracker.poll(); raw_tracker.poll()
            _write_live_dashboard_html(session_dir)
            _write_live_dashboard_json(session_dir, "analysis complete", time.time() - start_t, 0.0,
                                       radar_tracker, ref_tracker, raw_tracker, list(recent_events),
                                       analysis_summary=analysis_payload)
        except Exception as e:
            warn(f"Final dashboard artifact write failed: {e}")
    analyse_summary = _read_json_if_exists(os.path.join(analysis_dir, "analyse_summary.json")) or {}
    _write_session_manifest(
        session_dir,
        [radar_out],
        [ref_out],
        analysis_dir,
        analyse_summary.get("fw_truthfulness") if isinstance(analyse_summary, dict) else None,
    )
    if dashboard_server is not None:
        print(_dim(f"[DASHBOARD] Served live during the run at {dashboard_server.url}"))
        dashboard_server.stop()
    if lock_acquired:
        _release_session_lock(lock_root)


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
        dashboard_server = _start_dashboard_server(session_dir, port=int(getattr(args, "dashboard_port", 8765) or 0))
        print(_green(f"[DASHBOARD] {dashboard_server.url}"))
        if getattr(args, "open_dashboard", True):
            try:
                import webbrowser
                webbrowser.open(dashboard_server.url)
            except Exception as e:
                warn(f"Could not open dashboard in browser: {e}")
    except Exception as e:
        port = int(getattr(args, "dashboard_port", 8765) or 0)
        if port != 0:
            print(f"Port {port} in use. Stop the other instance or pass --dashboard-port <N>.", file=sys.stderr)
            raise SystemExit(2)
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


# -----------------------------------------------------------------------------
# SECTION 0: DOCTOR & QUICKSTART
# -----------------------------------------------------------------------------

_QUICKSTART_TEXT = """
+==================================================================+
|     Radar Vital Trainer v{ver} - Quick-Start Guide              |
+==================================================================+

RECOMMENDED THESIS WORKFLOW (single command per session)

STEP 0 - Check your setup
  python radar_vital_trainer_v12_for_v16_0.py doctor

STEP 1 - Run one full session
  python radar_vital_trainer_v12_for_v16_0.py session \
    --port COM10 \
    --address 10:22:33:9E:8F:63 \
    --duration-s 480 \
    --open-dashboard

  What this does automatically:
    - starts radar logging
    - starts BLE oximeter reference logging
    - starts a localhost dashboard server
    - opens the Material 3 Expressive live dashboard in your browser
    - stops both collectors together
    - runs analyse immediately after capture
    - prints a quick session report right away

  Session tips:
    - Sit still and face the radar chest-on
    - Use a fixed distance (recommended default: ~60 cm)
    - Keep the pulse oximeter on the same finger each run
    - Avoid fans, talking, and large body motion
    - Start with 8-minute sessions (480 s)

STEP 2 - Review the outputs
  In the new session folder you will get:
    - radar.csv
    - ref.csv
    - ref_ble_raw.csv
    - ref_ble_summary.json
    - live_dashboard.html
    - live_dashboard.json
    - analysis/analyse_summary.json
    - analysis/analyse_report.txt
    - analysis/analyse_report.html
    - session_quick_report.txt

STEP 3 - Compare sessions after you collect several runs
  python radar_vital_trainer_v12_for_v16_0.py compare \
    --sessions-dir sessions/ --out report.html

STEP 4 - Train a correction model after baseline quality is acceptable
  python radar_vital_trainer_v12_for_v16_0.py train \
    --radar sessions/s01/radar.csv --ref sessions/s01/ref.csv \
    --feature-mode core --require-baseline-gate \
    --out model_s01/

ADVANCED / MANUAL WORKFLOW

Manual radar logging:
  python radar_vital_trainer_v12_for_v16_0.py log --port COM10 --out sessions/s01/radar.csv

Manual BLE reference logging:
  python radar_vital_trainer_v12_for_v16_0.py ble_reflog \
    --out sessions/s01/ref.csv \
    --address 10:22:33:9E:8F:63 \
    --ble-profile ailink_oximeter \
    --notify-char 0000ffe2-0000-1000-8000-00805f9b34fb

Manual analysis:
  python radar_vital_trainer_v12_for_v16_0.py analyse \
    --radar sessions/s01/radar.csv --ref sessions/s01/ref.csv \
    --out sessions/s01/analysis/

DSP sweep:
  python radar_vital_trainer_v12_for_v16_0.py sweep \
    --sweep-json params_sweep.json \
    --port COM10 --sketch radar_vital_v15_0_0.ino \
    --out-dir sweep_results/

Run  python radar_vital_trainer_v12_for_v16_0.py doctor  to verify your environment.
For BLE pulse oximeter logging, also install:  pip install bleak
""".format(ver=VERSION)


def cmd_quickstart(args):
    print(_QUICKSTART_TEXT)


def cmd_doctor(args):
    if getattr(args, "json", False):
        checks = []

        def add_check(check_id, ok, detail=""):
            checks.append({"id": check_id, "status": "ok" if ok else "fail", "detail": detail})

        for name, import_name in [("numpy", "numpy"), ("pandas", "pandas"), ("scipy", "scipy"), ("scikit-learn", "sklearn"), ("matplotlib", "matplotlib"), ("pyserial", "serial")]:
            spec = importlib.util.find_spec(import_name)
            add_check(import_name, spec is not None, name if spec is not None else f"pip install {name}")
        cli_path = shutil.which("arduino-cli")
        add_check("arduino_cli", bool(cli_path), cli_path or "install arduino-cli and esp32 core")
        add_check("control_api_schema", True, CONTROL_API_SCHEMA_VERSION)
        legacy = []
        sessions_dir = Path("sessions")
        if sessions_dir.exists():
            for manifest_path in sessions_dir.glob("*/session_manifest.json"):
                manifest = _read_json_if_exists(str(manifest_path)) or {}
                if manifest.get("schema_version") != SESSION_MANIFEST_SCHEMA_VERSION:
                    legacy.append(manifest_path.parent.name)
        add_check("legacy_session_manifests", not legacy, "none" if not legacy else ",".join(legacy[:20]))
        print(json.dumps(nan_safe({"schema_version": CONTROL_API_SCHEMA_VERSION, "version": VERSION, "checks": checks}), indent=2, allow_nan=False))
        return
    print(_bold(f"\n=== Radar Vital Trainer v{VERSION} - Dependency Check ===\n"))
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
              f"{_dim('(optional - pip install tensorflow)')}")

    print()
    if ok_all:
        print(_green("All required dependencies satisfied. You are ready to go!"))
        print(f"Run  {_bold('python radar_vital_trainer_v12_for_v16_0.py quickstart')}  for the workflow guide.\n")
    else:
        print(_red("Some required dependencies are missing. Install them and re-run doctor.\n"))


# -----------------------------------------------------------------------------
# SECTION 1: LIVE LOGGING
# -----------------------------------------------------------------------------

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
                        print(_green(f"[LOG] First sample received - logging active."))
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
                        print(_yellow(f"  Ignored - HR {val:.0f} out of range [30-220]"))
                        continue
                    rows.append({"timestamp_ms": t, "ref_hr": val, "ref_rr": ""})
                    print(_green(f"  t={elapsed:.1f}s  HR={val:.0f} BPM logged"))
                except ValueError:
                    print(_red(f"  Could not parse: {line}  (example: HR=72)"))

            elif line.startswith("RR="):
                try:
                    val = float(line[3:])
                    if not (4 <= val <= 60):
                        print(_yellow(f"  Ignored - RR {val:.0f} out of range [4-60]"))
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
        print(f"  HR range:    {min(hr_vals):.0f} - {max(hr_vals):.0f} BPM "
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


# -----------------------------------------------------------------------------
# SECTION 1B: FLASH
# -----------------------------------------------------------------------------

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
        warn(f"Constant '{key}' not found in sketch - skipping")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    open(out_path, "w", encoding="utf-8").write(src)
    print(f"[FLASH] Patched {patched} constant(s) in sketch")
    return patched


def _is_relative_to_path(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def _prepare_arduino_sketch_dir(sketch_path: str, build_root: str, params: Optional[Dict[str, str]] = None) -> str:
    """Stage a valid Arduino sketch folder named after the .ino stem."""
    src = Path(sketch_path).resolve()
    if src.suffix.lower() != ".ino":
        raise ValueError(f"[FLASH] Expected an .ino sketch, got: {src}")
    root = Path(build_root).resolve()
    work_dir = root / "staged_sketch" / src.stem
    if work_dir.exists():
        if not _is_relative_to_path(work_dir, root):
            raise RuntimeError(f"[FLASH] Refusing to clean sketch folder outside build root: {work_dir}")
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    staged_ino = work_dir / f"{src.stem}.ino"
    if params:
        _patch_ino_constants(str(src), params, str(staged_ino))
    else:
        shutil.copy2(src, staged_ino)
    for sibling in src.parent.iterdir():
        if sibling == src or not sibling.is_file():
            continue
        if sibling.suffix.lower() in {".h", ".hpp", ".hh", ".c", ".cpp", ".cc", ".cxx", ".s"}:
            shutil.copy2(sibling, work_dir / sibling.name)
    return str(work_dir)


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
        print("[FLASH] No params file - compiling sketch as-is")

    sketch_path = args.sketch
    if not os.path.exists(sketch_path):
        sys.exit(f"[FLASH] Sketch not found: {sketch_path}")

    build_path = os.path.join(args.build_dir, "build")
    os.makedirs(build_path, exist_ok=True)
    compile_sketch = _prepare_arduino_sketch_dir(sketch_path, args.build_dir, params)

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
        print(_yellow("[FLASH] No --port specified - skipping upload (build only)"))

    print(_green("\n[FLASH] Done."))


# -----------------------------------------------------------------------------
# SECTION 2: DATA LOADING & ALIGNMENT  (unchanged from v5.0)
# -----------------------------------------------------------------------------

RADAR_SYNONYMS = {
    "smooth_hr": "reported_hr",
    "smooth_rr": "reported_rr",
    "humandetected": "human_detected",
    "radarispresent": "radar_is_present",
    "hrstate": "hr_state",
    "hr_trust_fresh": "trusted_hr_fresh",
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
        ("rewarm_triggered", 0),
        ("rewarm_reason", 0),
        ("experimental_profile_enabled", 0),
        ("fs_effective", np.nan),
        ("fs_snap_used", np.nan),
        ("hr_fs_guard_min", np.nan),
        ("hr_autocorr_best_conf", np.nan),
        ("phase_zero_fill_pct", 0),
        ("fs_fallback_used", 0),
        ("module_fw_valid", 0),
        ("hr_trust_age_ms", np.nan),
        ("rr_trust_age_ms", np.nan),
        ("skipdsp_run_len", 0),
        ("agc_floor_run_len", 0),
        ("buffer_zero_injected", 0),
        ("hr_raw_minus_anchor_bpm", np.nan),
        ("hr_phase_minus_anchor_bpm", np.nan),
        ("hr_raw_minus_phase_bpm", np.nan),
        ("rr_candidate_present", 0),
        ("rr_candidate_source", 0),
        ("rr_source_reject_reason", 0),
        ("rr_source_latched_ok", 0),
        ("hr_publish_block_stage", 0),
        ("rr_publish_block_stage", 0),
        ("rr_phase_backed_publish_ready", 0),
        ("hr_raw_age_ms", np.nan),
        ("hr_raw_disagree_subreason", 0),
    ):
        if col not in df.columns:
            df[col] = default
    if "hr_trust_fresh" not in df.columns and "trusted_hr_fresh" in df.columns:
        df["hr_trust_fresh"] = df["trusted_hr_fresh"]
    if "trusted_hr_fresh" not in df.columns and "hr_trust_fresh" in df.columns:
        df["trusted_hr_fresh"] = df["hr_trust_fresh"]

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
        "timestamp_ms", "heart_phase", "breath_phase", "raw_hr", "raw_hr_uncorrected", "raw_hr_corrected", "raw_rr", "raw_rr_effective", "raw_rr_likely_harmonic",
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
        "hr_publish_source", "rr_publish_source", "hr_publish_block_stage", "rr_publish_block_stage",
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
        "phase_gap_fill_count", "clutter_rewarm_count", "rewarm_triggered", "rewarm_reason", "experimental_profile_enabled",
        "hr_raw_disagree_subreason",
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
        add_numeric("raw_hr_uncorrected", "raw_hr_uncorrected", ["mean", "std"], nonzero_only=True)
        add_numeric("raw_hr_corrected", "raw_hr_corrected", ["mean", "std"], nonzero_only=True)
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

        add_numeric("hr_gate_reason", "hr_gate_reason", ["mean", "last"])
        add_numeric("rr_gate_reason", "rr_gate_reason", ["mean", "last"])
        add_numeric("hr_publish_reason", "hr_publish_reason", ["mean", "last"])
        add_numeric("rr_publish_reason", "rr_publish_reason", ["mean", "last"])
        add_numeric("hr_age_ms", "hr_age_ms", ["mean", "last"])
        add_numeric("candidate_hr_age_ms", "candidate_hr_age_ms", ["mean", "last", "max"])
        add_numeric("candidate_rr_age_ms", "candidate_rr_age_ms", ["mean", "last", "max"])
        add_numeric("hr_updated_this_cycle", "hr_updated_this_cycle", ["mean", "last"])

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
        add_numeric("rr_phase_backed_publish_ready", "rr_phase_backed_publish_ready", ["mean", "last", "max"])
        add_numeric("hr_anchor_drift_suspect", "hr_anchor_drift_suspect", ["mean", "last", "max"])
        add_numeric("phase_gap_fill_count", "phase_gap_fill_count", ["mean", "max", "last"])
        add_numeric("clutter_rewarm_count", "clutter_rewarm_count", ["mean", "max", "last"])
        add_numeric("rewarm_triggered", "rewarm_triggered", ["mean", "max", "last"])
        add_numeric("rewarm_reason", "rewarm_reason", ["mean", "last"])
        add_numeric("experimental_profile_enabled", "experimental_profile_enabled", ["mean", "last", "max"])
        add_numeric("fs_effective", "fs_effective", ["mean", "last", "min", "max"])
        add_numeric("fs_snap_used", "fs_snap_used", ["mean", "last", "min", "max"])
        add_numeric("hr_fs_guard_min", "hr_fs_guard_min", ["mean", "last", "min", "max"])
        add_numeric("hr_autocorr_best_conf", "hr_autocorr_best_conf", ["mean", "last", "max"])
        add_numeric("phase_zero_fill_pct", "phase_zero_fill_pct", ["mean", "last", "max"])
        add_numeric("fs_fallback_used", "fs_fallback_used", ["mean", "last", "max"])
        add_numeric("module_fw_valid", "module_fw_valid", ["mean", "last", "max"])
        add_numeric("hr_trust_age_ms", "hr_trust_age_ms", ["mean", "last", "min", "max"])
        add_numeric("rr_trust_age_ms", "rr_trust_age_ms", ["mean", "last", "min", "max"])
        add_numeric("skipdsp_run_len", "skipdsp_run_len", ["mean", "last", "max"])
        add_numeric("agc_floor_run_len", "agc_floor_run_len", ["mean", "last", "max"])
        add_numeric("buffer_zero_injected", "buffer_zero_injected", ["mean", "last", "max"])
        add_numeric("hr_raw_minus_anchor_bpm", "hr_raw_minus_anchor_bpm", ["mean", "last", "max"])
        add_numeric("hr_phase_minus_anchor_bpm", "hr_phase_minus_anchor_bpm", ["mean", "last", "max"])
        add_numeric("hr_raw_minus_phase_bpm", "hr_raw_minus_phase_bpm", ["mean", "last", "max"])
        add_numeric("rr_candidate_present", "rr_candidate_present", ["mean", "last", "max"])
        add_numeric("rr_candidate_source", "rr_candidate_source", ["mean", "last"])
        add_numeric("rr_source_reject_reason", "rr_source_reject_reason", ["mean", "last"])
        add_numeric("rr_source_latched_ok", "rr_source_latched_ok", ["mean", "last", "max"])
        add_numeric("rr_source_current_ok", "rr_source_current_ok", ["mean", "last", "max"])
        add_numeric("logged_rr_valid_current", "logged_rr_valid_current", ["mean", "last", "max"])
        add_numeric("logged_rr_valid_latched", "logged_rr_valid_latched", ["mean", "last", "max"])
        add_numeric("hr_publish_source_class", "hr_publish_source_class", ["mean", "last"])
        add_numeric("rr_publish_source_class", "rr_publish_source_class", ["mean", "last"])
        add_numeric("hr_freeze_suspect", "hr_freeze_suspect", ["mean", "last", "max"])
        add_numeric("hr_value_frozen_confirmed", "hr_value_frozen_confirmed", ["mean", "last", "max"])
        add_numeric("hr_freeze_duration_ms", "hr_freeze_duration_ms", ["mean", "last", "max"])
        add_numeric("hr_no_fresh_update_duration_ms", "hr_no_fresh_update_duration_ms", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_anchor_drift_suspect", "hr_raw_disagree_anchor_drift_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_phase_stale_suspect", "hr_raw_disagree_phase_stale_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_high_bias_suspect", "hr_raw_disagree_high_bias_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_low_dynamic_range_suspect", "hr_raw_disagree_low_dynamic_range_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_sumfreq_suspect", "hr_raw_disagree_sumfreq_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_rr_harmonic_k", "hr_raw_disagree_rr_harmonic_k", ["mean", "last", "max"])
        add_numeric("hr_publish_block_stage", "hr_publish_block_stage", ["mean", "last"])
        add_numeric("rr_publish_block_stage", "rr_publish_block_stage", ["mean", "last"])
        add_numeric("hr_raw_age_ms", "hr_raw_age_ms", ["mean", "last", "min", "max"])
        add_numeric("hr_raw_disagree_subreason", "hr_raw_disagree_subreason", ["mean", "last"])

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


# -----------------------------------------------------------------------------
# SECTION 3: FEATURES  (unchanged from v5.0)
# -----------------------------------------------------------------------------

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


FEATURE_CATEGORY = _build_feature_category()


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


# -----------------------------------------------------------------------------
# SECTION 4: SPLITTING  (unchanged from v5.0)
# -----------------------------------------------------------------------------

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


# -----------------------------------------------------------------------------
# SECTION 5: METRICS, EVALUATION, PLOTTING  (unchanged from v5.0)
# -----------------------------------------------------------------------------

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


def _dynamic_range_gate(y_pred: np.ndarray, *, min_std: float, min_unique: int = 3) -> Dict[str, object]:
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


def _evaluability_block(y_true: np.ndarray, y_pred: np.ndarray, target: str, dyn_gate: Dict[str, object]) -> Dict[str, object]:
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


HR_VALID_PUBLISH_OK = {0}
RR_VALID_PUBLISH_OK = {0}
HR_VALID_MAX_AGE_MS = 5000
RR_VALID_MAX_AGE_MS = 5000
MIN_PHASE_VALID_RUN_LEN = 3

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
    source_blocks: Dict[str, Dict[str, float]] = {}
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


def _first_existing(paths: Sequence[str]) -> Optional[str]:
    for path in paths:
        if path and os.path.exists(path):
            return path
    return None


def _raw_ble_candidates_for_ref(ref_path: str) -> List[str]:
    base_dir = os.path.dirname(os.path.abspath(ref_path))
    stem = os.path.splitext(os.path.basename(ref_path))[0]
    return [
        os.path.join(base_dir, "ref_ble_raw.csv"),
        os.path.join(base_dir, f"{stem}_ble_raw.csv"),
        os.path.join(base_dir, f"{stem}_raw.csv"),
    ]


def _ble_summary_candidates_for_ref(ref_path: str) -> List[str]:
    base_dir = os.path.dirname(os.path.abspath(ref_path))
    stem = os.path.splitext(os.path.basename(ref_path))[0]
    return [
        os.path.join(base_dir, "ref_ble_summary.json"),
        os.path.join(base_dir, f"{stem}_ble_summary.json"),
        os.path.join(base_dir, f"{stem}_summary.json"),
    ]


def _compute_ble_ref_quality(ref_paths: Sequence[str], pi_threshold: float = BLE_PI_QUALITY_THRESHOLD) -> Dict[str, object]:
    totals = {
        "raw_packets": 0,
        "parsed_rows": 0,
        "summary_parsed_packets": 0,
        "summary_packet_count": 0,
        "pi_rows": 0,
        "pi_below_threshold_rows": 0,
        "packet_loss_estimated_missing": 0.0,
        "packet_loss_expected": 0.0,
    }
    sources = []
    for ref_path in ref_paths:
        ref_abs = os.path.abspath(ref_path)
        raw_path = _first_existing(_raw_ble_candidates_for_ref(ref_abs))
        summary_path = _first_existing(_ble_summary_candidates_for_ref(ref_abs))
        ref_rows = 0
        pi_rows = 0
        pi_low = 0
        try:
            ref_df = pd.read_csv(ref_abs)
            ref_rows = int(len(ref_df))
            if "ref_pi" in ref_df.columns:
                pi = pd.to_numeric(ref_df["ref_pi"], errors="coerce")
                pi_rows = int(pi.notna().sum())
                pi_low = int((pi.dropna() < float(pi_threshold)).sum())
        except Exception:
            pass

        raw_rows = 0
        packet_loss_missing = 0.0
        packet_loss_expected = 0.0
        if raw_path:
            try:
                raw_df = pd.read_csv(raw_path)
                raw_rows = int(len(raw_df))
                if "timestamp_ms" in raw_df.columns and raw_rows >= 3:
                    ts = pd.to_numeric(raw_df["timestamp_ms"], errors="coerce").dropna().sort_values().to_numpy(dtype=float)
                    diffs = np.diff(ts)
                    diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
                    if len(diffs):
                        med = float(np.median(diffs))
                        if np.isfinite(med) and med > 0:
                            expected = int(max(raw_rows, round((ts[-1] - ts[0]) / med) + 1))
                            packet_loss_expected = float(expected)
                            packet_loss_missing = float(max(0, expected - raw_rows))
            except Exception:
                pass

        summary = _read_json_if_exists(summary_path) if summary_path else None
        summary_packets = 0
        summary_parsed = 0
        if isinstance(summary, dict):
            stats = summary.get("stats_by_source", {})
            if isinstance(stats, dict):
                for ent in stats.values():
                    if isinstance(ent, dict):
                        summary_packets += int(float(ent.get("packet_count", 0) or 0))
                        summary_parsed += int(float(ent.get("parsed_count", 0) or 0))
            summary_packets = summary_packets or int(float(summary.get("raw_packets", 0) or 0))
            summary_parsed = summary_parsed or int(float(summary.get("parsed_rows", 0) or 0))

        totals["raw_packets"] += raw_rows
        totals["parsed_rows"] += ref_rows
        totals["summary_packet_count"] += summary_packets
        totals["summary_parsed_packets"] += summary_parsed
        totals["pi_rows"] += pi_rows
        totals["pi_below_threshold_rows"] += pi_low
        totals["packet_loss_estimated_missing"] += packet_loss_missing
        totals["packet_loss_expected"] += packet_loss_expected
        sources.append({
            "ref_csv": ref_abs,
            "raw_csv": os.path.abspath(raw_path) if raw_path else None,
            "summary_json": os.path.abspath(summary_path) if summary_path else None,
            "raw_packets": raw_rows,
            "parsed_rows": ref_rows,
            "summary_packet_count": summary_packets,
            "summary_parsed_packets": summary_parsed,
            "pi_rows": pi_rows,
            "pi_below_threshold_rows": pi_low,
        })

    raw_denom = max(1, int(totals["raw_packets"]))
    decode_packets = int(totals["summary_packet_count"]) or int(totals["raw_packets"])
    parsed_packets = int(totals["summary_parsed_packets"]) or int(totals["parsed_rows"])
    decode_error_pct = float(100.0 * max(0, decode_packets - parsed_packets) / max(1, decode_packets))
    packet_loss_pct = (
        float(100.0 * totals["packet_loss_estimated_missing"] / totals["packet_loss_expected"])
        if totals["packet_loss_expected"] > 0 else float("nan")
    )
    pi_below_pct = (
        float(100.0 * totals["pi_below_threshold_rows"] / totals["pi_rows"])
        if totals["pi_rows"] > 0 else float("nan")
    )
    distilled_pct = float(100.0 * int(totals["parsed_rows"]) / raw_denom) if totals["raw_packets"] else float("nan")
    return {
        "raw_packets": int(totals["raw_packets"]),
        "parsed_rows": int(totals["parsed_rows"]),
        "distilled_rows_pct_of_raw": distilled_pct,
        "packet_loss_pct": packet_loss_pct,
        "decode_error_pct": decode_error_pct,
        "pi_below_threshold_pct": pi_below_pct,
        "pi_threshold": float(pi_threshold),
        "sources": sources,
        "status": (
            "WARN"
            if ((np.isfinite(packet_loss_pct) and packet_loss_pct > 10.0) or
                (np.isfinite(decode_error_pct) and decode_error_pct > 25.0) or
                (np.isfinite(pi_below_pct) and pi_below_pct > 25.0))
            else "OK"
        ),
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


# -----------------------------------------------------------------------------
# SECTION 6: MODELS  (unchanged from v5.0)
# -----------------------------------------------------------------------------

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


# -----------------------------------------------------------------------------
# SECTION 7: COMMANDS - predict / analyse / train  (unchanged from v5.0)
# -----------------------------------------------------------------------------

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
    session_root = _session_root_for_outputs(args.out)
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
    coverage_ledger = _coverage_loss_ledger(raw_radar_df)
    oracle_audit = _oracle_candidate_audit(feat_df)
    hr_publish_source_hist = _histogram_from_series(raw_radar_df.get("hr_publish_source", pd.Series(dtype=float)), HR_PUBLISH_SOURCE_NAMES)
    _warn_on_manifest_mismatch(session_root, fw_truthfulness)
    session_start_s = float(safe_series(feat_df, "timestamp_s", default=np.nan).dropna().min()) if len(feat_df) else 0.0
    early_mask = (safe_series(feat_df, "timestamp_s", default=np.nan) < (session_start_s + 30.0)).to_numpy(dtype=bool)
    suspect_half_rate_mask = (
        (safe_series(feat_df, "reported_hr_mean", default=0.0) < 70.0) &
        (safe_series(feat_df, "ref_hr", default=0.0) > 85.0)
    ).to_numpy(dtype=bool)
    all_mask = np.ones(len(feat_df), dtype=bool)
    experimental_audit = {
        "experimental_profile_default": False,
        "early_window_excluded_n": int(np.sum(early_mask)),
        "suspect_half_rate_excluded_n": int(np.sum(suspect_half_rate_mask)),
        "combined_all": _experimental_hr_gate_metrics(feat_df, all_mask),
        "combined_without_early_window": _experimental_hr_gate_metrics(feat_df, ~early_mask),
        "combined_without_suspect_half_rate": _experimental_hr_gate_metrics(feat_df, ~suspect_half_rate_mask),
        "combined_without_early_or_suspect_half_rate": _experimental_hr_gate_metrics(feat_df, ~(early_mask | suspect_half_rate_mask)),
    }
    print("\n== BASELINE SUMMARY ==")
    print_metric_block("HR baseline", hr_base)
    print_metric_block("RR baseline", rr_base)

    r = hr_base["valid_only"]["r"]
    rmse = hr_base["valid_only"]["rmse"]
    gate_pass = np.isfinite(r) and r > 0.4 and np.isfinite(rmse) and rmse < 8.0
    secondary_gate = _coverage_penalized_hr_gate(feat_df, window_s=600.0)
    session_duration_s = _session_duration_s(feat_df)
    short_session = np.isfinite(session_duration_s) and session_duration_s < SECONDARY_GATE_MIN_DURATION_S
    primary_gate = dict(ml_gate_combined)
    primary_gate["min_n"] = 15
    primary_gate["strict_ml_passed"] = bool(gate_pass and _safe_int(primary_gate.get("n"), 0) >= 15)
    primary_gate["secondary_gate"] = secondary_gate
    if short_session:
        primary_gate["passed"] = False
        primary_gate["conditional_passed"] = False
        primary_gate["ml_ready"] = False
        primary_gate["status"] = "DEFERRED_TOO_SHORT_TO_JUDGE"
        primary_gate["reason"] = "too_short_to_judge"
        primary_gate["operator_message"] = "Session duration is below the minimum threshold for ML judgment."
    elif _safe_int(primary_gate.get("n"), 0) < 15:
        if secondary_gate.get("kind") == "deferred":
            primary_gate["passed"] = False
            primary_gate["conditional_passed"] = False
            primary_gate["ml_ready"] = False
            primary_gate["status"] = "DEFERRED_TOO_SHORT_TO_JUDGE"
            primary_gate["reason"] = "too_short_to_judge"
            primary_gate["operator_message"] = secondary_gate.get("operator_message")
        elif secondary_gate.get("passed"):
            primary_gate["passed"] = True
            primary_gate["conditional_passed"] = True
            primary_gate["ml_ready"] = False
            primary_gate["status"] = "CONDITIONAL_PASS_N_TOO_SMALL_FOR_ML"
            primary_gate["reason"] = "valid_hr_pairs_below_min_n_but_coverage_penalized_rmse_passed"
            primary_gate["operator_message"] = secondary_gate.get("operator_message")
        else:
            primary_gate["passed"] = False
            primary_gate["conditional_passed"] = False
            primary_gate["ml_ready"] = False
            primary_gate["status"] = "INSUFFICIENT_DATA"
            primary_gate["reason"] = "valid_hr_pairs_below_min_n"
    else:
        primary_gate["ml_ready"] = bool(gate_pass)
        primary_gate["status"] = "PASS" if gate_pass else "FAIL"
    print(f"\n{'='*50}")
    print(f"ML gate status:         {primary_gate.get('status', 'FAIL')}  (n={_safe_int(primary_gate.get('n'), 0)}, min_n={_safe_int(primary_gate.get('min_n'), 0)})")
    print(f"ML gate  r>0.4:    {_green('PASS') if (np.isfinite(r) and r>0.4) else _red('FAIL')}  (r={r:.3f})")
    print(f"ML gate  RMSE<8:   {_green('PASS') if (np.isfinite(rmse) and rmse<8) else _red('FAIL')}  (RMSE={rmse:.2f})")
    if secondary_gate.get("kind") == "deferred":
        print("ML gate secondary:  DEFERRED  (too short to judge)")
    else:
        print(f"ML gate conditional 10min cov-pen RMSE<8: "
              f"{_green('PASS') if secondary_gate.get('passed') else _red('FAIL')}  "
              f"(cov_pen={_to_num(secondary_gate.get('coverage_penalized_rmse'), float('nan')):.2f}, "
              f"n={_safe_int(secondary_gate.get('n'), 0)}, cov={_to_num(secondary_gate.get('coverage_pct'), float('nan')):.1f}%)")
    print(f"ML gate locked audit:    r={_to_num(ml_gate_locked.get('r'), float('nan')):.3f}  RMSE={_to_num(ml_gate_locked.get('rmse'), float('nan')):.2f}  n={_safe_int(ml_gate_locked.get('n'), 0)}")
    print(f"ML gate settling audit:  r={_to_num(ml_gate_settling.get('r'), float('nan')):.3f}  RMSE={_to_num(ml_gate_settling.get('rmse'), float('nan')):.2f}  n={_safe_int(ml_gate_settling.get('n'), 0)}")
    if primary_gate.get("status") == "CONDITIONAL_PASS_N_TOO_SMALL_FOR_ML":
        print(_yellow("-> Conditional pass: n is too small for ML, but the 10-minute coverage-penalized HR RMSE passed."))
        print("  Treat this as an operator-facing signal-quality pass, not a training-ready dataset.")
    elif primary_gate.get("status") == "INSUFFICIENT_DATA":
        print(_red("-> Gate INSUFFICIENT_DATA. Do NOT run `train` yet."))
        print("  Collect more valid locked rows or fix firmware coverage before training.")
    elif gate_pass:
        print(_green("-> Gate PASSED. You may proceed to `train`."))
    else:
        print(_red("-> Gate FAILED. Do NOT run `train` yet."))
        print("  Investigate pqi_heart, firmware fixes, or collect more sessions.")
    print(f"{'='*50}\n")

    out_csv = os.path.join(args.out, "aligned_1hz_features.csv")
    feat_df.to_csv(out_csv, index=False)
    feat_df.to_csv(os.path.join(args.out, "aligned_1hz.csv"), index=False)
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
    raw_hr_uncorrected_bias_estimate = compute_hr_bias_estimate(feat_df, ["raw_hr_uncorrected_mean", "raw_hr_mean", "raw_hr_nonzero_mean"], series_label="raw_hr_uncorrected")
    raw_hr_corrected_bias_estimate = compute_hr_bias_estimate(feat_df, ["raw_hr_corrected_mean"], series_label="raw_hr_corrected")
    raw_hr_sumfreq_estimate = compute_raw_hr_sumfreq_error_estimate(feat_df)
    ble_ref_quality = _compute_ble_ref_quality(args.ref)
    raw_hr_uncorrected_bias_bpm = float(raw_hr_uncorrected_bias_estimate.get("bias_bpm", float("nan")))
    raw_hr_corrected_bias_bpm = float(raw_hr_corrected_bias_estimate.get("bias_bpm", float("nan")))
    if {"raw_hr_corrected", "raw_hr_uncorrected"}.issubset(raw_radar_df.columns):
        corrected = pd.to_numeric(raw_radar_df["raw_hr_corrected"], errors="coerce")
        uncorrected = pd.to_numeric(raw_radar_df["raw_hr_uncorrected"], errors="coerce")
        correction_mask = corrected.notna() & uncorrected.notna() & (corrected != uncorrected)
        raw_hr_correction_coverage_pct = float(100.0 * correction_mask.mean()) if len(raw_radar_df) else float("nan")
    else:
        raw_hr_correction_coverage_pct = float("nan")
    hr_rescue_publish_count = _safe_int(hr_publish_source_hist.get("corrected_raw_rescue"), 0) if isinstance(hr_publish_source_hist, dict) else 0
    hr_rescue_publish_coverage_pct = float(100.0 * hr_rescue_publish_count / max(len(raw_radar_df), 1)) if len(raw_radar_df) else float("nan")
    raw_rescue_reject_count = _derive_raw_rescue_reject_count(raw_radar_df)
    summary = {
        "version": VERSION, "n_rows": int(len(feat_df)),
        "sessions": list(dict.fromkeys(feat_df["session_id"].tolist())),
        "tolerance_s": args.tolerance_s, "merge_direction": args.merge_direction,
        "auto_align_start": bool(args.auto_align_start),
        "ref_offset_s": normalize_offsets(len(args.radar), args.ref_offset_s),
        "feature_mode": args.feature_mode, "phase_unit": args.phase_unit,
        "feature_engineering_version": FEATURE_ENGINEERING_VERSION,
        "feature_schema_hash": feature_schema_hash(),
        "scoring_weights_hash": _scoring_weights_hash(),
        "heart_fft_window_s": args.heart_fft_window_s,
        "breath_fft_window_s": args.breath_fft_window_s,
        "alignment_dt_s": {
            "mean": float(ref_dt.mean()) if ref_dt.notna().any() else float("nan"),
            "max_abs": float(ref_dt.abs().max()) if ref_dt.notna().any() else float("nan"),
        },
        "pqi_lock_pct": pqi_lock_pct,
        "ml_gate": primary_gate,
        "ml_gate_policy": primary_gate,
        "ml_gate_secondary": secondary_gate,
        "ml_gate_combined": ml_gate_combined,
        "ml_gate_locked": ml_gate_locked,
        "ml_gate_settling": ml_gate_settling,
        "hr_baseline": hr_base, "rr_baseline": rr_base,
        "hr_baseline_locked": hr_locked_base,
        "hr_baseline_settling": hr_settling_base,
        "coverage_locked": coverage_locked,
        "coverage_settling": coverage_settling,
        "raw_hr_bias_estimate": raw_hr_bias_estimate,
        "raw_hr_bias_audit": raw_hr_bias_estimate,
        "raw_hr_uncorrected_bias_estimate": raw_hr_uncorrected_bias_estimate,
        "raw_hr_corrected_bias_estimate": raw_hr_corrected_bias_estimate,
        "raw_hr_uncorrected_bias_bpm": raw_hr_uncorrected_bias_bpm,
        "raw_hr_corrected_bias_bpm": raw_hr_corrected_bias_bpm,
        "raw_hr_correction_coverage_pct": raw_hr_correction_coverage_pct,
        "raw_hr_sumfreq_estimate": raw_hr_sumfreq_estimate,
        "ble_ref_quality": ble_ref_quality,
        "fw_truthfulness": fw_truthfulness,
        "gate_audit": gate_audit,
        "coverage_ledger": coverage_ledger,
        "hr_coverage_loss_by_stage": coverage_ledger.get("hr", {}).get("histogram", {}),
        "rr_coverage_loss_by_stage": coverage_ledger.get("rr", {}).get("histogram", {}),
        "hr_salvageable_frames_pct": coverage_ledger.get("hr", {}).get("salvageable_pct"),
        "rr_salvageable_frames_pct": coverage_ledger.get("rr", {}).get("salvageable_pct"),
        "oracle_candidate_audit": oracle_audit,
        "hr_oracle_candidate_mae_bpm": oracle_audit.get("hr", {}).get("oracle_candidate_mae_bpm"),
        "hr_oracle_candidate_rmse_bpm": oracle_audit.get("hr", {}).get("oracle_candidate_rmse_bpm"),
        "hr_oracle_candidate_coverage_pct": oracle_audit.get("hr", {}).get("oracle_candidate_coverage_pct"),
        "rr_oracle_candidate_mae_bpm": oracle_audit.get("rr", {}).get("oracle_candidate_mae_bpm"),
        "rr_oracle_candidate_rmse_bpm": oracle_audit.get("rr", {}).get("oracle_candidate_rmse_bpm"),
        "rr_oracle_candidate_coverage_pct": oracle_audit.get("rr", {}).get("oracle_candidate_coverage_pct"),
        "hr_best_candidate_source_histogram": oracle_audit.get("hr", {}).get("best_candidate_source_histogram", {}),
        "rr_best_candidate_source_histogram": oracle_audit.get("rr", {}).get("best_candidate_source_histogram", {}),
        "hr_rescue_publish_count": hr_rescue_publish_count,
        "hr_rescue_publish_coverage_pct": hr_rescue_publish_coverage_pct,
        "raw_rescue_reject_count": raw_rescue_reject_count,
        "hr_publish_source_histogram": hr_publish_source_hist,
        "publish_reason_histogram": publish_reason_histogram,
        "hr_gate_reason_histogram": hr_gate_reason_histogram,
        "rr_gate_reason_histogram": rr_gate_reason_histogram,
        "agc_anomaly_flags": agc_anomaly_flags,
        "experimental_audit": experimental_audit,
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
    raw_hr_corrected_bias = float((raw_hr_corrected_bias_estimate or {}).get("bias_bpm", float("nan")))
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
    if np.isfinite(raw_hr_corrected_bias) and np.isfinite(raw_hr_bias) and abs(raw_hr_corrected_bias) < abs(raw_hr_bias):
        review_flags.append("raw_hr_correction_improves_bias")
    if ghost_session:
        review_flags.append("ghost_target_session")
    summary["has_reference"] = bool(has_ref)
    summary["ghost_target_session"] = bool(ghost_session)
    summary["review_required"] = bool(review_flags)
    summary["review_flags"] = review_flags

    # v15.0 Stage C: pqi_v15 shadow analysis. Surfaces whether the gap-aware
    # PQI would unlock more candidate-eligible frames vs the v14 PQI, AND
    # whether it would inflate on motion frames (the go/no-go safety check).
    summary["pqi_v15_shadow"] = _build_pqi_v15_shadow(feat_df)

    # v15.0 Stage E: adaptive correction shadow. Dual-domain (reference + deployment)
    # piecewise-linear fit. Push to firmware is DEFERRED to v12 — this is analysis-only.
    summary["adaptive_correction_shadow"] = _build_adaptive_correction_shadow(feat_df)

    readiness = _build_ml_readiness_verdict(summary)
    summary["ml_readiness_verdict"] = readiness
    summary["readiness_kind"] = readiness.get("readiness_kind")
    summary["schema_verdict"] = readiness.get("schema_verdict")
    summary["signal_verdict"] = readiness.get("signal_verdict")
    summary["reference_verdict"] = readiness.get("reference_verdict")
    summary["training_verdict"] = readiness.get("training_verdict")
    summary["readiness_headline"] = readiness.get("headline")

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
            "raw_hr_bias_audit": raw_hr_bias_estimate,
            "ble_ref_quality": ble_ref_quality,
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
    _write_simple_html_report(f"Radar Vital Trainer Analyse - {', '.join(summary['sessions'])}",
                              sections, images,
                              os.path.join(args.out, "analyse_report.html"))
    _write_session_manifest(session_root, args.radar, args.ref, args.out, fw_truthfulness)
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
    progress_path = os.path.join(args.out, "training_progress.json")
    target_label = ",".join(getattr(args, "targets", ["hr", "rr"]))
    n_estimators_total = int(getattr(args, "n_estimators", 0) or 0)
    started_at = _iso_now()
    save_json({
        "schema_version": TRAINING_PROGRESS_SCHEMA_VERSION,
        "status": "running",
        "target": target_label,
        "n_estimators_done": 0,
        "n_estimators_total": n_estimators_total,
        "train_loss": None,
        "val_loss": None,
        "elapsed_s": 0.0,
        "started_at": started_at,
        "updated_at": started_at,
    }, progress_path)
    train_started_t = time.monotonic()
    def _write_training_progress(status="running", n_done=0, train_loss=None, val_loss=None, **extra):
        payload = {
            "schema_version": TRAINING_PROGRESS_SCHEMA_VERSION,
            "status": status,
            "target": target_label,
            "n_estimators_done": int(max(0, n_done)),
            "n_estimators_total": int(max(0, n_estimators_total)),
            "train_loss": None if train_loss is None else float(train_loss),
            "val_loss": None if val_loss is None else float(val_loss),
            "elapsed_s": round(time.monotonic() - train_started_t, 3),
            "started_at": started_at,
            "updated_at": _iso_now(),
        }
        payload.update(extra)
        save_json(payload, progress_path)

    _write_training_progress(status="running", n_done=0, phase="aligning")
    base_df = load_and_align_multiple_base(
        args.radar, args.ref,
        tolerance_s=args.tolerance_s, ref_offsets_s=args.ref_offset_s,
        auto_align_start=args.auto_align_start, merge_direction=args.merge_direction,
        phase_unit=args.phase_unit, heart_fft_window_s=args.heart_fft_window_s,
        breath_fft_window_s=args.breath_fft_window_s)
    _write_training_progress(status="running", n_done=0, phase="feature_engineering")

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
    n_estimators_total = int(getattr(args, "n_estimators", 0) or 0) * max(1, len(available_targets))
    _write_training_progress(status="running", n_done=0, phase="training_targets")

    feature_cols = pick_feature_columns(
        train_df, feature_mode=args.feature_mode,
        max_nan_frac=args.max_nan_frac, min_variance=args.min_variance,
        allow_policy_features=bool(getattr(args, "allow_policy_features", False)))
    if not feature_cols:
        raise ValueError("No numeric feature columns available for training.")
    assert_ml_feature_schema(
        feature_cols,
        allow_policy_features=bool(getattr(args, "allow_policy_features", False)),
    )

    X_train_all, X_stop_all, impute_values, missing_flag_cols = prepare_feature_matrix(
        train_df, stop_df, feature_cols)
    X_eval_all = transform_feature_matrix(eval_df, feature_cols, impute_values, missing_flag_cols)
    expanded_feature_cols = list(X_train_all.columns)
    assert_ml_feature_schema(
        expanded_feature_cols,
        allow_policy_features=bool(getattr(args, "allow_policy_features", False)),
    )
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
    n_estimators_done = 0

    if "hr" in available_targets:
        model_hr, hr_fit_meta = fit_target_model(
            train_df, stop_df, "hr", X_train_all, X_stop_all, params=params,
            random_state=args.random_state, sample_weight_mode=args.sample_weight_mode,
            use_early_stopping=use_es, early_stop_strategy=args.early_stop_strategy)
        n_estimators_done += int(hr_fit_meta.get("best_n_estimators", params.get("n_estimators", 0)) or 0)
        _write_training_progress(status="running", n_done=n_estimators_done, phase="trained_hr", val_loss=hr_fit_meta.get("best_val_rmse"))
    if "rr" in available_targets:
        model_rr, rr_fit_meta = fit_target_model(
            train_df, stop_df, "rr", X_train_all, X_stop_all, params=params,
            random_state=args.random_state + 1, sample_weight_mode=args.sample_weight_mode,
            use_early_stopping=use_es, early_stop_strategy=args.early_stop_strategy)
        n_estimators_done += int(rr_fit_meta.get("best_n_estimators", params.get("n_estimators", 0)) or 0)
        _write_training_progress(status="running", n_done=n_estimators_done, phase="trained_rr", val_loss=rr_fit_meta.get("best_val_rmse"))

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
        "feature_engineering_version": FEATURE_ENGINEERING_VERSION,
        "feature_schema_version": FEATURE_SCHEMA_VERSION,
        "feature_schema_hash": feature_schema_hash(),
        "allow_policy_features": bool(getattr(args, "allow_policy_features", False)),
        "policy_features_in_matrix": policy_feature_columns(expanded_feature_cols),
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
            "feature_engineering_version": FEATURE_ENGINEERING_VERSION,
            "feature_schema_version": FEATURE_SCHEMA_VERSION,
            "feature_schema_hash": feature_schema_hash(),
            "allow_policy_features": bool(getattr(args, "allow_policy_features", False)),
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
        "loo_eval": loo_eval,
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
    hr_train_rmse = _to_num((hr_model_train or {}).get("valid_only", {}).get("rmse"), default=None) if isinstance(hr_model_train, dict) else None
    rr_train_rmse = _to_num((rr_model_train or {}).get("valid_only", {}).get("rmse"), default=None) if isinstance(rr_model_train, dict) else None
    hr_eval_rmse = _to_num((hr_model_eval or {}).get("valid_only", {}).get("rmse"), default=None) if isinstance(hr_model_eval, dict) else None
    rr_eval_rmse = _to_num((rr_model_eval or {}).get("valid_only", {}).get("rmse"), default=None) if isinstance(rr_model_eval, dict) else None
    train_losses = [x for x in (hr_train_rmse, rr_train_rmse) if x is not None]
    val_losses = [x for x in (hr_eval_rmse, rr_eval_rmse) if x is not None]
    _write_training_progress(
        status="complete",
        n_done=n_estimators_total,
        train_loss=(sum(train_losses) / len(train_losses)) if train_losses else None,
        val_loss=(sum(val_losses) / len(val_losses)) if val_losses else None,
        completed_at=_iso_now(),
        target=",".join(available_targets),
        phase="done",
    )
    write_text_report(os.path.join(args.out, "train_report.txt"), summary)
    print(f"\n[OUT] Training outputs saved to {args.out}")


# -----------------------------------------------------------------------------
# SECTION 7B: ALIGN - standalone sync step
# -----------------------------------------------------------------------------

def cmd_align(args):
    """
    Sync a radar CSV to a reference CSV and write the merged output.

    This is the same alignment performed inside `analyse` and `train`, exposed
    as a standalone step so you can inspect the merged CSV before training.

    Outputs:
      <out>/aligned_merged.csv   - merged 1 Hz radar+ref rows
      <out>/alignment_report.txt - sync quality metrics
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
    print(f"  python radar_vital_trainer_v12_for_v16_0.py analyse --radar {args.radar[0]} "
          f"--ref {args.ref[0]} --out <analysis_dir>")


# -----------------------------------------------------------------------------
# SECTION 7C: SWEEP - automated multi-variant flash -> log -> analyse
# -----------------------------------------------------------------------------

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
                    print(_green("  First frame received - logging active."))
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

    print(_bold(f"\n=== Sweep: {total} experiment(s) - {args.sweep_json} ===\n"))

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

        compile_sketch = _prepare_arduino_sketch_dir(args.sketch, build_dir, params)

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
            print(f"  {_yellow('REMINDER:')} run  python radar_vital_trainer_v12_for_v16_0.py reflog  in a second terminal!")
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
                feat_df.to_csv(os.path.join(exp_dir, "aligned_1hz_features.csv"), index=False)
                feat_df.to_csv(os.path.join(exp_dir, "aligned_1hz.csv"), index=False)
                hr_base  = baseline_summary(feat_df, "hr")
                rr_base  = baseline_summary(feat_df, "rr")
                r    = hr_base["valid_only"]["r"]
                rmse = hr_base["valid_only"]["rmse"]
                bias = hr_base["valid_only"]["bias"]
                gate = "PASS" if (np.isfinite(r) and r > 0.4 and
                                  np.isfinite(rmse) and rmse < 8.0) else "FAIL"
                exp_result.update({"status": "analysed", "hr_rmse": float(rmse),
                                   "hr_bias": float(bias), "hr_r": float(r), "ml_gate": gate})
                save_json({"version": VERSION, "params": params, "hr_baseline": hr_base, "rr_baseline": rr_base},
                          os.path.join(exp_dir, "analyse_summary.json"))
                gate_str = _green(gate) if gate == "PASS" else _red(gate)
                print(f"  HR: RMSE={rmse:.2f}  bias={bias:+.2f}  r={r:.3f}  gate={gate_str}")
            except Exception as e:
                warn(f"Analysis failed for {name}: {e}")
                exp_result["status"] = "analyse_failed"
        results.append(exp_result)

    # Summary table
    print(_bold(f"\n\n{'='*60}"))
    print(_bold(f"  SWEEP SUMMARY - {total} experiments"))
    print(f"{'='*60}")
    print(f"  {'Name':<20} {'Status':<15} {'RMSE':>6} {'Bias':>6} {'r':>6} {'Gate'}")
    print(f"  {'-'*56}")
    for r in results:
        rmse = r.get("hr_rmse", float("nan"))
        bias = r.get("hr_bias", float("nan"))
        rv   = r.get("hr_r",   float("nan"))
        gate = r.get("ml_gate", "-")
        gate_str = _green(gate) if gate == "PASS" else (_red(gate) if gate == "FAIL" else gate)
        rmse_str = f"{rmse:.2f}" if np.isfinite(rmse) else "-"
        bias_str = f"{bias:+.2f}" if np.isfinite(bias) else "-"
        r_str    = f"{rv:.3f}"   if np.isfinite(rv)   else "-"
        print(f"  {r.get('name','?'):<20} {r.get('status','?'):<15} "
              f"{rmse_str:>6} {bias_str:>6} {r_str:>6} {gate_str}")
    print(f"{'='*60}")

    save_json({"version": VERSION, "sweep_json": args.sweep_json,
               "results": results},
              os.path.join(args.out_dir, "sweep_results.json"))
    print(f"\n[OUT] Sweep results saved to {args.out_dir}")

    if args.ref:
        print(f"\nTo compare all sessions:")
        print(f"  python radar_vital_trainer_v12_for_v16_0.py compare --sessions-dir {args.out_dir} --out report.html")


# -----------------------------------------------------------------------------
# SECTION 7D: COMPARE - HTML report across all sessions
# -----------------------------------------------------------------------------

def _safe_float_val(v, fmt=".2f") -> str:
    if v is None: return "-"
    try:
        f = float(v)
        return "-" if not np.isfinite(f) else format(f, fmt)
    except Exception:
        return "-"


def _metric_samples_from_aligned(path: Optional[str]) -> Optional[Dict[str, np.ndarray]]:
    if not path or not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path)
    except Exception:
        return None
    if not {"ref_hr", "reported_hr_mean"}.issubset(df.columns):
        return None
    mask = finite_pair_mask(df, "ref_hr", "reported_hr_mean")
    if "hr_valid_for_eval" in df.columns:
        mask = mask & df["hr_valid_for_eval"].fillna(False).to_numpy(dtype=bool)
    if not mask.any():
        return None
    ref = safe_series(df, "ref_hr").to_numpy(dtype=float)[mask]
    pred = safe_series(df, "reported_hr_mean").to_numpy(dtype=float)[mask]
    finite = np.isfinite(ref) & np.isfinite(pred)
    if finite.sum() == 0:
        return None
    return {"ref": ref[finite], "pred": pred[finite]}


def _sample_rmse_r(samples: Dict[str, np.ndarray], rng: np.random.Generator) -> Tuple[float, float]:
    ref = np.asarray(samples["ref"], dtype=float)
    pred = np.asarray(samples["pred"], dtype=float)
    n = len(ref)
    if n == 0:
        return float("nan"), float("nan")
    idx = rng.integers(0, n, size=n)
    ref_b = ref[idx]
    pred_b = pred[idx]
    return float(np.sqrt(np.mean((pred_b - ref_b) ** 2))), safe_corr(ref_b, pred_b)


def _bootstrap_delta_ci(
    baseline_samples: Optional[Dict[str, np.ndarray]],
    experiment_samples: Optional[Dict[str, np.ndarray]],
    n_boot: int,
    seed: int,
) -> Dict[str, object]:
    if baseline_samples is None or experiment_samples is None:
        return {"available": False, "reason": "aligned feature CSV missing or has no valid HR pairs"}
    rng = np.random.default_rng(int(seed))
    d_rmse = []
    d_r = []
    for _ in range(max(1, int(n_boot))):
        b_rmse, b_r = _sample_rmse_r(baseline_samples, rng)
        e_rmse, e_r = _sample_rmse_r(experiment_samples, rng)
        if np.isfinite(b_rmse) and np.isfinite(e_rmse):
            d_rmse.append(e_rmse - b_rmse)
        if np.isfinite(b_r) and np.isfinite(e_r):
            d_r.append(e_r - b_r)

    def block(vals):
        arr = np.asarray(vals, dtype=float)
        arr = arr[np.isfinite(arr)]
        if len(arr) == 0:
            return {"mean": float("nan"), "ci_low": float("nan"), "ci_high": float("nan"), "n_boot": 0}
        return {
            "mean": float(np.mean(arr)),
            "ci_low": float(np.percentile(arr, 2.5)),
            "ci_high": float(np.percentile(arr, 97.5)),
            "n_boot": int(len(arr)),
        }

    return {"available": True, "delta_rmse": block(d_rmse), "delta_r": block(d_r)}


def _format_ci(block: Optional[Dict[str, object]], fmt: str = ".2f") -> str:
    if not isinstance(block, dict):
        return "-"
    mean = _to_num(block.get("mean"), float("nan"))
    lo = _to_num(block.get("ci_low"), float("nan"))
    hi = _to_num(block.get("ci_high"), float("nan"))
    if not (np.isfinite(mean) and np.isfinite(lo) and np.isfinite(hi)):
        return "-"
    return f"{format(mean, fmt)} [{format(lo, fmt)}, {format(hi, fmt)}]"


def _load_sweep_param_map(sessions_dir: str) -> Dict[str, Dict[str, object]]:
    data = _read_json_if_exists(os.path.join(sessions_dir, "sweep_results.json"))
    if not isinstance(data, dict):
        return {}
    out: Dict[str, Dict[str, object]] = {}
    for row in data.get("results", []):
        if isinstance(row, dict) and row.get("name"):
            out[str(row["name"])] = dict(row.get("params", {}) or {})
    return out


def _build_sweep_delta_rows(rows: List[Dict], baseline_name: Optional[str], n_boot: int, seed: int) -> List[Dict[str, object]]:
    if not rows:
        return []
    baseline = None
    if baseline_name:
        baseline = next((r for r in rows if str(r.get("name")) == str(baseline_name)), None)
    if baseline is None:
        baseline = next((r for r in rows if "baseline" in str(r.get("name", "")).lower()), rows[0])
    base_params = baseline.get("params", {}) if isinstance(baseline.get("params"), dict) else {}
    base_samples = _metric_samples_from_aligned(baseline.get("aligned_features_path"))
    base_rmse = _to_num(baseline.get("hr_rmse"), float("nan"))
    base_r = _to_num(baseline.get("hr_r"), float("nan"))
    deltas = []
    for row in rows:
        if row is baseline:
            continue
        params = row.get("params", {}) if isinstance(row.get("params"), dict) else {}
        changed = []
        for key in sorted(set(base_params.keys()) | set(params.keys())):
            if base_params.get(key) != params.get(key):
                changed.append({"parameter": key, "baseline_value": base_params.get(key), "experiment_value": params.get(key)})
        if not changed and params:
            changed = [{"parameter": "(params)", "baseline_value": base_params, "experiment_value": params}]
        if not changed:
            changed = [{"parameter": "(session)", "baseline_value": baseline.get("name"), "experiment_value": row.get("name")}]
        exp_samples = _metric_samples_from_aligned(row.get("aligned_features_path"))
        boot = _bootstrap_delta_ci(base_samples, exp_samples, n_boot=n_boot, seed=seed + len(deltas) + 1)
        exp_rmse = _to_num(row.get("hr_rmse"), float("nan"))
        exp_r = _to_num(row.get("hr_r"), float("nan"))
        point_rmse = exp_rmse - base_rmse if np.isfinite(base_rmse) and np.isfinite(exp_rmse) else float("nan")
        point_r = exp_r - base_r if np.isfinite(base_r) and np.isfinite(exp_r) else float("nan")
        for change in changed:
            deltas.append({
                "baseline": baseline.get("name"),
                "experiment": row.get("name"),
                "parameter": change["parameter"],
                "baseline_value": change["baseline_value"],
                "experiment_value": change["experiment_value"],
                "delta_rmse_point": point_rmse,
                "delta_r_point": point_r,
                "bootstrap": boot,
            })
    return deltas


def _generate_compare_html(rows: List[Dict], out_path: str, delta_rows: Optional[List[Dict[str, object]]] = None):
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

    gate_counts = {"PASS": 0, "FAIL": 0, "-": 0}
    for r in rows:
        g = r.get("ml_gate", "-")
        gate_counts[g if g in gate_counts else "-"] += 1

    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    best_rmse = min((float(r["hr_rmse"]) for r in rows
                     if r.get("hr_rmse") and np.isfinite(float(r["hr_rmse"]))),
                    default=float("nan"))

    table_rows = ""
    for r in sorted(rows, key=lambda x: float(x.get("hr_rmse") or 999)):
        gate = r.get("ml_gate", "-")
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
          <td style="font-weight:500">{r.get('name','-')}</td>
          <td>{r.get('firmware','-')}</td>
          <td>{r.get('distance','-')}</td>
          <td>{r.get('n_pairs','-')}</td>
          <td style="{rmse_style}">{_safe_float_val(rmse)}</td>
          <td>{_safe_float_val(bias, '+.2f')}</td>
          <td style="{r_style}">{_safe_float_val(r_v, '.3f')}</td>
          <td style="{rr_style}">{_safe_float_val(rr_rmse)}</td>
          <td style="{pqi_style}">{_safe_float_val(pqi, '.4f')}</td>
          <td style="{pqi_style}">{_safe_float_val(r.get('pqi_lock_pct'), '.1f')}%</td>
          <td style="{gate_style}">{gate}</td>
        </tr>"""

    delta_rows = delta_rows or []
    delta_table_rows = ""
    for d in delta_rows:
        boot = d.get("bootstrap", {}) if isinstance(d.get("bootstrap"), dict) else {}
        delta_table_rows += f"""
        <tr>
          <td>{d.get('baseline','-')}</td>
          <td>{d.get('experiment','-')}</td>
          <td>{d.get('parameter','-')}</td>
          <td>{html_escape(str(d.get('baseline_value','-')))}</td>
          <td>{html_escape(str(d.get('experiment_value','-')))}</td>
          <td>{_safe_float_val(d.get('delta_rmse_point'), '+.2f')}</td>
          <td>{_format_ci(boot.get('delta_rmse') if isinstance(boot, dict) else None, '+.2f')}</td>
          <td>{_safe_float_val(d.get('delta_r_point'), '+.3f')}</td>
          <td>{_format_ci(boot.get('delta_r') if isinstance(boot, dict) else None, '+.3f')}</td>
        </tr>"""
    delta_section = ""
    if delta_rows:
        delta_section = f"""
<p class="sec">Sweep deltas vs baseline with bootstrap 95% CIs</p>
<table>
  <thead>
    <tr>
      <th>Baseline</th><th>Experiment</th><th>Parameter</th><th>Baseline value</th><th>Experiment value</th>
      <th>Delta RMSE</th><th>Delta RMSE 95% CI</th><th>Delta r</th><th>Delta r 95% CI</th>
    </tr>
  </thead>
  <tbody>{delta_table_rows}</tbody>
</table>
"""

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Radar Vital Monitor - Session Comparison</title>
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

<h1>Radar Vital Monitor - Session Comparison</h1>
<p class="meta">Generated: {timestamp} &nbsp;-&nbsp; Trainer v{VERSION} &nbsp;-&nbsp;
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

<p class="sec">All sessions - sorted by HR RMSE</p>
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

{delta_section}

<div class="legend">
  <span class="dot" style="background:#3b6d11"></span> AAMI target &nbsp;
  <span class="dot" style="background:#854f0b"></span> borderline &nbsp;
  <span class="dot" style="background:#a32d2d"></span> needs work &nbsp;&nbsp;
  HR RMSE targets: &lt;5 AAMI / &lt;8 ML gate &nbsp;&nbsp;
  RR RMSE target: &lt;3 br/min &nbsp;&nbsp;
  pqi lock: >=0.35
</div>

<footer>radar_vital_trainer_v{VERSION} &nbsp;-&nbsp;
XIAO ESP32-C6 + MR60BHA2 60 GHz FMCW &nbsp;-&nbsp; {timestamp}</footer>
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
    sweep_params = _load_sweep_param_map(sessions_dir)

    for entry in sorted(os.scandir(sessions_dir), key=lambda e: e.name):
        if not entry.is_dir():
            continue
        name = entry.name
        d    = entry.path

        # Try reading pre-computed analyse_summary.json first
        summary_path = _first_existing([
            os.path.join(d, "analyse_summary.json"),
            os.path.join(d, "analysis", "analyse_summary.json"),
        ])
        if summary_path and os.path.exists(summary_path):
            try:
                s = json.load(open(summary_path, encoding="utf-8"))
                hr = s.get("hr_baseline", {}).get("valid_only", {})
                rr = s.get("rr_baseline", {}).get("valid_only", {})

                # Try reading pqi from aligned CSV
                pqi_mean = pqi_lock = None
                aligned_csv = _first_existing([
                    os.path.join(d, "aligned_1hz_features.csv"),
                    os.path.join(d, "aligned_1hz.csv"),
                    os.path.join(d, "analysis", "aligned_1hz_features.csv"),
                    os.path.join(d, "analysis", "aligned_1hz.csv"),
                ])
                if aligned_csv and os.path.exists(aligned_csv):
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
                            "FAIL" if gate.get("passed") is False else "-")
                params_for_session = sweep_params.get(name, s.get("params", {}) if isinstance(s.get("params"), dict) else {})
                rows.append({
                    "name":        name,
                    "firmware":    s.get("sessions", ["?"])[0] if s.get("sessions") else "?",
                    "distance":    "-",
                    "n_pairs":     hr.get("n", "-"),
                    "hr_rmse":     hr.get("rmse"),
                    "hr_bias":     hr.get("bias"),
                    "hr_r":        hr.get("r"),
                    "rr_rmse":     rr.get("rmse"),
                    "pqi_mean":    pqi_mean,
                    "pqi_lock_pct": pqi_lock,
                    "ml_gate":     gate_str,
                    "params": params_for_session,
                    "aligned_features_path": aligned_csv,
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
                    "name": name, "firmware": "?", "distance": "-",
                    "n_pairs": hr.get("n", "-"),
                    "hr_rmse": hr.get("rmse"), "hr_bias": hr.get("bias"),
                    "hr_r": hr.get("r"), "rr_rmse": rr.get("rmse"),
                    "pqi_mean": pqi_mean, "pqi_lock_pct": pqi_lock, "ml_gate": gate,
                    "params": sweep_params.get(name, {}),
                    "aligned_features_path": None,
                })
            except Exception as e:
                warn(f"  Analysis failed for {name}: {e}")
        else:
            print(f"  [SKIP] {name}  (no analyse_summary.json or radar+ref CSV pair found)")

    if not rows:
        print(_yellow("[COMPARE] No sessions found. Run `analyse` on each session first."))
        return

    print(f"\n[COMPARE] {len(rows)} session(s) found.")
    delta_rows = _build_sweep_delta_rows(
        rows,
        baseline_name=getattr(args, "baseline_name", None),
        n_boot=int(getattr(args, "bootstrap_n", 1000)),
        seed=int(getattr(args, "bootstrap_seed", 42)),
    )
    _generate_compare_html(rows, args.out, delta_rows=delta_rows)
    compare_json = os.path.splitext(args.out)[0] + ".json"
    save_json({
        "version": VERSION,
        "sessions_dir": os.path.abspath(sessions_dir),
        "rows": rows,
        "sweep_delta_rows": delta_rows,
        "bootstrap_n": int(getattr(args, "bootstrap_n", 1000)),
    }, compare_json)

    # Print quick text table
    print(f"\n{'Name':<22} {'RMSE':>6} {'Bias':>6} {'r':>6} {'Gate'}")
    print("-" * 52)
    for r in sorted(rows, key=lambda x: float(x.get("hr_rmse") or 999)):
        gate = r.get("ml_gate", "-")
        g    = _green(gate) if gate == "PASS" else _red(gate) if gate == "FAIL" else gate
        print(f"  {r.get('name','?'):<20} "
              f"{_safe_float_val(r.get('hr_rmse')):>6} "
              f"{_safe_float_val(r.get('hr_bias'), '+.2f'):>6} "
              f"{_safe_float_val(r.get('hr_r'), '.3f'):>6}  {g}")
    if delta_rows:
        print(f"\n{'Experiment':<20} {'Param':<18} {'dRMSE [95% CI]':>24} {'dr [95% CI]':>22}")
        print("-" * 90)
        for d in delta_rows:
            boot = d.get("bootstrap", {}) if isinstance(d.get("bootstrap"), dict) else {}
            print(f"  {str(d.get('experiment','?')):<18} {str(d.get('parameter','?')):<18} "
                  f"{_format_ci(boot.get('delta_rmse') if isinstance(boot, dict) else None, '+.2f'):>24} "
                  f"{_format_ci(boot.get('delta_r') if isinstance(boot, dict) else None, '+.3f'):>22}")


# -----------------------------------------------------------------------------
# SECTION 8: CLI
# -----------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=f"Radar Vital Trainer v{VERSION} - full session pipeline + ML training",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Quick start:\n"
            "  python radar_vital_trainer_v12_for_v16_0.py doctor      # check dependencies\n"
            "  python radar_vital_trainer_v12_for_v16_0.py quickstart  # full workflow guide\n"
        ),
    )
    parser.add_argument("--version", action="version", version=VERSION)
    sub = parser.add_subparsers(dest="cmd", required=True)

    # -- doctor ----------------------------------------------------------------
    p_doc = sub.add_parser("doctor", help="check all dependencies and print install instructions")
    p_doc.add_argument("--json", action="store_true", help="emit machine-readable dependency/session checks")
    p_doc.set_defaults(func=cmd_doctor)

    # -- quickstart ------------------------------------------------------------
    p_qs = sub.add_parser("quickstart", help="print the full new-user workflow guide")
    p_qs.set_defaults(func=cmd_quickstart)

    # -- log -------------------------------------------------------------------
    p_log = sub.add_parser("log",
        help="capture serial DATA lines to CSV (run in Terminal A during session)")
    p_log.add_argument("--port", default=DEFAULT_RADAR_PORT,
                       help=f"serial port (default: {DEFAULT_RADAR_PORT}), e.g. COM10 (Windows) or /dev/ttyACM0 (Linux/Mac)")
    p_log.add_argument("--out",  default="session.csv",
                       help="output CSV path (default: session.csv)")
    p_log.set_defaults(func=cmd_log)

    # -- reflog ----------------------------------------------------------------
    p_rl = sub.add_parser("reflog",
        help="live keyboard HR/RR reference logger (run in Terminal B during session)")
    p_rl.add_argument("--out", default="ref.csv",
                      help="output reference CSV path (default: ref.csv)")
    p_rl.set_defaults(func=cmd_reflog)

    # -- ble_reflog ------------------------------------------------------------
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

    # -- ble_analyse_raw -------------------------------------------------------
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


    # -- session ---------------------------------------------------------------
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
    p_ss.add_argument("--subject-profile-id", default="adult_default",
                      help="subject profile id from subject_profiles.json (default: adult_default)")
    p_ss.add_argument("--auto-analyse", dest="auto_analyse", action="store_true", default=True,
                      help="run analysis after capture (default: enabled)")
    p_ss.add_argument("--no-auto-analyse", dest="auto_analyse", action="store_false",
                      help="skip automatic analysis after capture")
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
    p_ss.add_argument("--dashboard-port", type=int, default=8765,
                      help="dashboard localhost port (default: 8765; use 0 for auto)")
    p_ss.add_argument("--live-dashboard", dest="live_dashboard", action="store_true", default=True,
                      help="show live clean dashboard during session (default: enabled)")
    p_ss.add_argument("--no-dashboard", dest="live_dashboard", action="store_false",
                      help="disable live dashboard and show only final report")
    p_ss.add_argument("--open-dashboard", dest="open_dashboard", action="store_true", default=True,
                      help="open the live dashboard automatically in your default browser (default: enabled)")
    p_ss.add_argument("--no-open-dashboard", dest="open_dashboard", action="store_false",
                      help="do not auto-open the dashboard browser tab")
    p_ss.set_defaults(func=cmd_session)

    # ── serve ────────────────────────────────────────────────────────────────
    p_srv = sub.add_parser("serve",
        help="localhost dashboard control server")
    p_srv.add_argument("--host", default=None,
                       help="control server bind host (default: 127.0.0.1 local, 0.0.0.0 lan)")
    p_srv.add_argument("--port", dest="control_port", type=int, default=8765,
                       help="control server port (default: 8765; use 0 for auto)")
    p_srv.add_argument("--control-port", dest="control_port", type=int,
                       help="deprecated alias for --port")
    p_srv.add_argument("--bind", choices=["local", "lan"], default="local",
                       help="bind mode: local loopback or LAN with pairing token controls (default: local)")
    p_srv.add_argument("--tls", nargs="*", default=None,
                       help="serve HTTPS; omit paths to auto-generate .rvt_tls/cert.pem and key.pem")
    p_srv.add_argument("--tls-trusted", action="store_true",
                       help="send HSTS only when --tls uses a CA-trusted certificate")
    p_srv.add_argument("--sessions-root", default="sessions",
                       help="sessions root served by the control server (default: sessions)")
    p_srv.add_argument("--cors-origin", default="",
                       help="Access-Control-Allow-Origin value for local dashboard requests (default: \"\", no wildcards by default)")
    p_srv.add_argument("--allow-wildcard-cors-lan", action="store_true",
                       help="permit --cors-origin '*' with --bind lan on an isolated lab network")
    p_srv.add_argument("--mock", action="store_true",
                       help="serve an integrated virtual radar stream over the live APIs and SSE")
    p_srv.add_argument("--no-browser", action="store_true",
                       help="do not open the dashboard browser tab")
    p_srv.set_defaults(func=cmd_serve)

    # -- dashboard --------------------------------------------------------------
    p_db = sub.add_parser("dashboard",
        help="live clean dashboard for an existing session directory")
    p_db.add_argument("--session-dir", required=True,
                      help="session directory containing radar.csv and/or ref.csv")
    p_db.add_argument("--duration-s", type=float, default=None,
                      help="optional auto-stop duration")
    p_db.add_argument("--dashboard-refresh-s", type=float, default=1.0)
    p_db.add_argument("--dashboard-port", type=int, default=8765,
                      help="dashboard localhost port (default: 8765; use 0 for auto)")
    p_db.add_argument("--open-dashboard", dest="open_dashboard", action="store_true", default=True,
                      help="open the live dashboard automatically in your default browser (default: enabled)")
    p_db.add_argument("--no-open-dashboard", dest="open_dashboard", action="store_false",
                      help="do not auto-open the dashboard browser tab")
    p_db.set_defaults(func=cmd_dashboard)

    # -- flash -----------------------------------------------------------------
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

    # -- align -----------------------------------------------------------------
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

    # -- predict ---------------------------------------------------------------
    p_pr = sub.add_parser("predict",
        help="apply a saved trained model to new radar-only CSV(s)")
    p_pr.add_argument("--radar",     nargs="+", required=True)
    p_pr.add_argument("--model-dir", required=True,
                      help="directory containing model_hr.pkl, model_rr.pkl, preprocessor.pkl")
    p_pr.add_argument("--out",       default="predict_out")
    p_pr.add_argument("--no-plots",  action="store_true")
    p_pr.set_defaults(func=cmd_predict)

    # -- shared args for analyse + train ---------------------------------------
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

    # -- analyse ---------------------------------------------------------------
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

    # -- train -----------------------------------------------------------------
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
    p_tr.add_argument("--allow-policy-features", action="store_true",
                      help="explicit ablation flag: allow policy/gate telemetry into the ML feature matrix")
    p_tr.add_argument("--loo-eval", action="store_true",
                      help="run leave-one-session-out evaluation across available sessions (recommended for paper-grade validation)")
    p_tr.set_defaults(func=cmd_train)

    # -- sweep -----------------------------------------------------------------
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
    p_sw.add_argument("--distance-hint", default="0.7-0.9",
                      help="distance reminder shown to user (default: 0.7-0.9)")
    p_sw.add_argument("--tolerance-s",  type=float, default=0.9)
    p_sw.add_argument("--auto-align-start", action="store_true")
    p_sw.set_defaults(func=cmd_sweep)

    # -- compare ---------------------------------------------------------------
    p_cmp = sub.add_parser("compare",
        help="generate HTML comparison report across all sessions in a directory")
    p_cmp.add_argument("--sessions-dir", required=True,
                       help="directory containing session sub-folders")
    p_cmp.add_argument("--out", default="compare_report.html",
                       help="output HTML report path (default: compare_report.html)")
    p_cmp.add_argument("--baseline-name", default=None,
                       help="baseline session/experiment name for sweep deltas (default: name containing 'baseline', else first session)")
    p_cmp.add_argument("--bootstrap-n", type=int, default=1000,
                       help="bootstrap replicates for delta RMSE/r confidence intervals (default: 1000)")
    p_cmp.add_argument("--bootstrap-seed", type=int, default=42,
                       help="random seed for bootstrap confidence intervals")
    p_cmp.set_defaults(func=cmd_compare)

    return parser


def _run_self_tests() -> int:
    import urllib.request

    test_report = tempfile.NamedTemporaryFile(delete=False, suffix=".txt")
    test_report.close()
    try:
        write_text_report(test_report.name, {"x": {"y": float("nan"), "z": float("inf")}})
        txt = Path(test_report.name).read_text(encoding="utf-8")
        assert "NaN" not in txt and "Infinity" not in txt
    finally:
        try:
            os.remove(test_report.name)
        except Exception:
            pass

    tmp = tempfile.mkdtemp(prefix="rvt-selftest-")
    server = None
    try:
        args = argparse.Namespace(host="127.0.0.1", control_port=0, sessions_root=tmp, no_browser=True, cors_origin="", mock=True, bind="local", tls=None, tls_trusted=False)
        server = _start_control_server(args)
        server.start()
        base = f"http://127.0.0.1:{server.httpd.server_port}"
        with urllib.request.urlopen(base + "/api/health", timeout=5) as r:
            assert r.status == 200
            assert r.headers.get("Access-Control-Allow-Origin") == "*"
            data = json.loads(r.read().decode("utf-8"))
            assert data.get("version") == VERSION
        with urllib.request.urlopen(base + "/api/session/current/live_dashboard.json", timeout=5) as r:
            assert r.status == 200
            data = json.loads(r.read().decode("utf-8"))
            assert data.get("session_id") == "mock"
        print("PASS: self-tests")
        return 0
    finally:
        if server is not None:
            try:
                server.stop()
            except Exception:
                pass
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    if "--test" in sys.argv:
        raise SystemExit(_run_self_tests())
    parser = build_parser()
    args   = parser.parse_args()

    if hasattr(args, "radar") and hasattr(args, "ref") and args.ref is not None:
        if len(args.radar) != len(args.ref):
            parser.error("--radar and --ref must have the same number of files")

    args.func(args)


if __name__ == "__main__":
    main()
