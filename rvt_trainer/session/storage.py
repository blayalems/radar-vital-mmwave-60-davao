"""Session directory scanning, summary loading, profiles, annotations, and exports."""

from __future__ import annotations

import csv
import json
import os
import re
import time
from html import escape as html_escape
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Tuple

import numpy as np
import pandas as pd

from rvt_trainer.api.common import (
    atomic_write_json as save_json,
    nan_safe,
    read_json_if_exists as _read_json_if_exists,
)
from rvt_trainer.audit.truthfulness import _fmt_quick_metrics
from rvt_trainer.audit.verdict import _build_ml_readiness_verdict
from rvt_trainer.constants import (
    CHART_ANNOTATIONS_SCHEMA_VERSION,
    CONTROL_API_SCHEMA_VERSION,
    DASHBOARD_VERSION,
    DEFAULT_SUBJECT_PROFILES,
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    FIRMWARE_VERSION_EXPECTED,
    SUBJECT_PROFILE_SCHEMA_VERSION,
    VERSION,
    _bold,
    _decimate_records,
    _first_existing,
    _green,
    _iso_from_mtime,
    _iso_now,
    _numeric_or_none,
    _safe_int,
    _sanitize_user_string,
    _summary_value,
    _to_num,
    _top_histogram_items,
)
from rvt_trainer.features import feature_schema_hash
from rvt_trainer.modeling import _build_sweep_delta_rows
from rvt_trainer.session.live_dashboard import _dashboard_html_path
from rvt_trainer.transport.serial import _is_supported_radar_contract_length


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
    raw_vo = block.get("valid_only")
    vo = raw_vo if isinstance(raw_vo, dict) else {}
    raw_all_frames = block.get("all_frames")
    all_frames = raw_all_frames if isinstance(raw_all_frames, dict) else {}
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


def _infer_session_timestamp(root: Path, manifest: Optional[Dict[str, object]]) -> Tuple[Optional[str], Optional[int], str]:
    explicit = _summary_value(
        (manifest or {}).get("started_at"),
        (manifest or {}).get("started"),
        (manifest or {}).get("started_utc"),
    )
    if explicit:
        parsed_ms = None
        try:
            parsed = pd.to_datetime(explicit, utc=True, errors="coerce")
            if not pd.isna(parsed):
                parsed_ms = int(parsed.timestamp() * 1000)
        except Exception:
            parsed_ms = None
        return str(explicit), parsed_ms, "manifest"

    data_candidates: List[Tuple[int, Path, str]] = []
    metadata_candidates: List[Tuple[int, Path, str]] = []
    for pattern, source, bucket in (
        ("*.csv", "session_file_mtime", data_candidates),
        ("*.txt", "session_file_mtime", data_candidates),
        ("live_dashboard.json", "dashboard_mtime", metadata_candidates),
        ("dashboard.json", "dashboard_mtime", metadata_candidates),
        ("session_manifest.json", "manifest_mtime", metadata_candidates),
    ):
        for path in root.glob(pattern):
            if path.is_file():
                try:
                    bucket.append((int(path.stat().st_mtime * 1000), path, source))
                except OSError:
                    pass
    candidates = data_candidates or metadata_candidates
    if not candidates:
        return None, None, "missing"
    mtime_ms, path, source = max(candidates, key=lambda item: item[0])
    return _iso_from_mtime(path), mtime_ms, source


def _duration_from_csv(path: Path) -> Optional[float]:
    if not path.exists() or not path.is_file():
        return None
    try:
        with path.open("r", encoding="utf-8", errors="ignore", newline="") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                return None
            timestamp_column = next((name for name in ("timestamp_ms", "timestamp_s", "t_s", "time_s", "elapsed_s") if name in reader.fieldnames), None)
            if not timestamp_column:
                return None
            first: Optional[float] = None
            last: Optional[float] = None
            for row in reader:
                numeric = _numeric_or_none(row.get(timestamp_column))
                if numeric is None:
                    continue
                if first is None:
                    first = numeric
                last = numeric
            if first is None or last is None or last < first:
                return None
            duration = last - first
            if timestamp_column == "timestamp_ms":
                duration /= 1000.0
            return max(0.0, duration)
    except Exception:
        return None


def _infer_session_duration_s(root: Path, manifest: Optional[Dict[str, object]], analysis: Optional[Dict[str, object]]) -> Optional[int]:
    for value in (
        (manifest or {}).get("duration_s"),
        (manifest or {}).get("duration"),
        (analysis or {}).get("duration_s") if isinstance(analysis, dict) else None,
        (analysis or {}).get("session_duration_s") if isinstance(analysis, dict) else None,
    ):
        numeric = _numeric_or_none(value)
        if numeric is not None and numeric > 0:
            return int(round(numeric))
    for path in (root / "radar.csv", root / "ref.csv"):
        duration = _duration_from_csv(path)
        if duration is not None and duration > 0:
            return int(round(duration))
    return None


def _contract_diagnosis(analysis: Optional[Dict[str, object]], manifest: Optional[Dict[str, object]]) -> Dict[str, object]:
    analysis = analysis if isinstance(analysis, dict) else {}
    manifest = manifest if isinstance(manifest, dict) else {}
    raw_fw = analysis.get("fw_truthfulness")
    fw = raw_fw if isinstance(raw_fw, dict) else {}
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
    started_at, started_ms, timestamp_source = _infer_session_timestamp(root, manifest)
    duration_s = _infer_session_duration_s(root, manifest, analysis if isinstance(analysis, dict) else None)
    subject_profile_id = str((manifest or {}).get("subject_profile_id") or "adult_default")
    profiles_payload = _read_json_if_exists(str(root.parent / "subject_profiles.json")) or {}
    subject_profile = (profiles_payload.get("profiles") or {}).get(subject_profile_id)
    subject_profile_label = (
        subject_profile.get("label")
        if isinstance(subject_profile, dict) and subject_profile.get("label")
        else DEFAULT_SUBJECT_PROFILES.get(subject_profile_id, {}).get("label")
    )
    subject_label = _summary_value(
        (manifest or {}).get("subject_label"),
        (manifest or {}).get("subject"),
        (manifest or {}).get("params", {}).get("subject_label") if isinstance((manifest or {}).get("params"), dict) else None,
        subject_profile_label,
        subject_profile_id,
    )
    operator_label = _summary_value(
        (manifest or {}).get("operator_label"),
        (manifest or {}).get("operator"),
        (manifest or {}).get("params", {}).get("operator_label") if isinstance((manifest or {}).get("params"), dict) else None,
    )
    base = {
        "session_id": session_id,
        "session_dir": str(root.resolve()),
        "manifest": manifest,
        "subject_label": subject_label,
        "subject": subject_label,
        "subject_profile_id": subject_profile_id,
        "subject_profile_label": subject_profile_label,
        "operator_label": operator_label,
        "operator": operator_label,
        "notes": (manifest or {}).get("notes"),
        "tags": (
            [str(t) for t in manifest.get("tags", []) if str(t).strip()]
            if isinstance(manifest, dict) and isinstance(manifest.get("tags"), list)
            else []
        ),
        "started_at": started_at,
        "started_ms": started_ms,
        "timestamp_source": timestamp_source,
        "ended_at": (manifest or {}).get("ended_at"),
        "duration_s": duration_s,
        "started_from": (manifest or {}).get("started_from"),
        "trainer_version": (manifest or {}).get("trainer_version", VERSION),
        "dashboard_version": (manifest or {}).get("dashboard_version", DASHBOARD_VERSION),
        "downloads": downloads,
    }
    if not isinstance(analysis, dict):
        status_path = root / "analysis" / "analyse_status.json"
        status_payload = _read_json_if_exists(str(status_path))
        if isinstance(status_payload, dict):
            base.update({
                "status": status_payload.get("status", "incomplete"),
                "analysis_status": status_payload.get("analysis_status", status_payload.get("status", "incomplete")),
                "analysis": None,
                "verdict": None,
                "ml_readiness_verdict": _build_ml_readiness_verdict(None),
                "contract_diagnosis": _contract_diagnosis(None, manifest),
                "warnings": [str(status_payload.get("last_line") or status_payload.get("message") or "Analysis summary is not available yet.")],
            })
            return base
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
    raw_ble_quality = analysis.get("ble_ref_quality")
    ble_quality = raw_ble_quality if isinstance(raw_ble_quality, dict) else (ble_summary or {})
    raw_fw = analysis.get("fw_truthfulness")
    fw = raw_fw if isinstance(raw_fw, dict) else {}
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
    raw_publish_hist = analysis.get("publish_reason_histogram")
    publish_hist = raw_publish_hist if isinstance(raw_publish_hist, dict) else {}
    raw_publish_hr = publish_hist.get("hr")
    publish_hr = raw_publish_hr if isinstance(raw_publish_hr, dict) else publish_hist
    raw_publish_rr = publish_hist.get("rr")
    publish_rr = raw_publish_rr if isinstance(raw_publish_rr, dict) else publish_hist
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
        raw_manifest = summary.get("manifest")
        manifest = raw_manifest if isinstance(raw_manifest, dict) else {}
        raw_analysis = summary.get("analysis")
        analysis = raw_analysis if isinstance(raw_analysis, dict) else {}
        raw_hr = analysis.get("hr_baseline")
        hr = raw_hr if isinstance(raw_hr, dict) else {}
        raw_vo = hr.get("valid_only")
        vo = raw_vo if isinstance(raw_vo, dict) else {}
        raw_ble = analysis.get("ble_ref_quality")
        ble = raw_ble if isinstance(raw_ble, dict) else {}
        raw_rr = analysis.get("rr_baseline")
        rr = raw_rr if isinstance(raw_rr, dict) else {}
        raw_rr_vo = rr.get("valid_only")
        rr_vo = raw_rr_vo if isinstance(raw_rr_vo, dict) else {}
        raw_ref_q = summary.get("reference_quality")
        ref_q = raw_ref_q if isinstance(raw_ref_q, dict) else {}
        items.append({
            "session_id": d.name,
            "started_at": summary.get("started_at") or manifest.get("started_at"),
            "started_ms": summary.get("started_ms"),
            "timestamp_source": summary.get("timestamp_source"),
            "duration_s": summary.get("duration_s") or manifest.get("duration_s"),
            "subject": summary.get("subject_label") or manifest.get("subject_label"),
            "subject_label": summary.get("subject_label") or manifest.get("subject_label"),
            "subject_profile_id": summary.get("subject_profile_id") or manifest.get("subject_profile_id"),
            "subject_profile_label": summary.get("subject_profile_label"),
            "operator": summary.get("operator_label") or manifest.get("operator_label"),
            "operator_label": summary.get("operator_label") or manifest.get("operator_label"),
            "verdict": summary.get("verdict"),
            "status": summary.get("status"),
            "hr_rmse": summary.get("hr_rmse", vo.get("rmse")),
            "hr_r": summary.get("hr_r", vo.get("r")),
            "rr_rmse": summary.get("rr_rmse", rr_vo.get("rmse")),
            "rr_r": summary.get("rr_r", rr_vo.get("r")),
            "ref_coverage_pct": ref_q.get("distilled_rows_pct_of_raw", ble.get("distilled_rows_pct_of_raw")),
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
    best = _load_session_summary(str(Path(root) / str(best_item["session_id"]))) if best_item else None
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


def _save_subject_profiles(sessions_root: str, payload: Mapping[str, object]) -> Dict[str, object]:
    """Persist the dashboard's non-identifying physiology profile settings.

    The Angular setup editor sends either the profile map directly or a
    ``{"profiles": ...}`` envelope.  Normalize both forms to the same
    schema, preserve the built-in defaults, and discard unknown fields so a
    browser cannot smuggle arbitrary metadata into the trainer's profile
    store.
    """

    incoming = payload.get("profiles", payload)
    if not isinstance(incoming, dict):
        raise ValueError("profiles must be an object")
    profiles: Dict[str, object] = dict(DEFAULT_SUBJECT_PROFILES)
    for raw_id, raw_profile in incoming.items():
        profile_id = str(raw_id or "").strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,47}", profile_id):
            raise ValueError("subject profile ids must use letters, digits, dash, or underscore")
        if not isinstance(raw_profile, dict):
            raise ValueError(f"subject profile {profile_id} must be an object")
        profile: Dict[str, object] = {}
        label = _sanitize_user_string(raw_profile.get("label", profile_id), 120).strip()
        if not label:
            raise ValueError(f"subject profile {profile_id} requires a label")
        profile["label"] = label
        for key in ("age_group", "fitness_level", "notes"):
            if raw_profile.get(key) not in (None, ""):
                profile[key] = _sanitize_user_string(raw_profile.get(key), 240).strip()
        expected = raw_profile.get("expected_hr_range")
        if isinstance(expected, (list, tuple)) and len(expected) == 2:
            try:
                low, high = float(expected[0]), float(expected[1])
            except (TypeError, ValueError):
                raise ValueError(f"subject profile {profile_id} expected_hr_range must be numeric") from None
            if not (0 <= low < high <= 260):
                raise ValueError(f"subject profile {profile_id} expected_hr_range is invalid")
            profile["expected_hr_range"] = [low, high]
        profiles[profile_id] = profile
    data: Dict[str, object] = {"schema_version": SUBJECT_PROFILE_SCHEMA_VERSION, "profiles": profiles}
    save_json(data, str(_subject_profiles_path(sessions_root)))
    return data


def _session_path(sessions_root: str, session_id: str) -> Path:
    root = Path(sessions_root).resolve()
    target = (root / session_id).resolve()
    # Session routes address one direct child of sessions_root.  Requiring a
    # strict child (rather than merely "under root") rejects "." / empty IDs
    # that otherwise alias the sessions root itself, as well as nested IDs.
    if target.parent != root or not target.is_dir():
        raise FileNotFoundError(session_id)
    return target


def _session_annotations_path(session_dir: Path) -> Path:
    return Path(session_dir) / "session_annotations.json"


def _load_session_annotations_payload(session_dir: Path, session_id: str) -> Dict[str, object]:
    path = _session_annotations_path(session_dir)
    existing = _read_json_if_exists(str(path)) or {}
    raw_annotations = existing.get("annotations")
    annotations = raw_annotations if isinstance(raw_annotations, list) else []
    return {"schema_version": CHART_ANNOTATIONS_SCHEMA_VERSION, "session_id": session_id, "updated_at": existing.get("updated_at"), "annotations": annotations}


def _upsert_session_annotation(session_dir: Path, session_id: str, chart_key: str, annotation: Dict[str, object], action: str = "upsert") -> Dict[str, object]:
    path = _session_annotations_path(session_dir)
    existing = _read_json_if_exists(str(path)) or {}
    raw_annotations = existing.get("annotations")
    annotations = raw_annotations if isinstance(raw_annotations, list) else []
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


def _latest_live_dashboard_payload(sessions_root: str) -> Optional[Dict[str, object]]:
    for item in _scan_sessions_root(sessions_root):
        session_id = str(item.get("session_id") or "").strip()
        if not session_id:
            continue
        live_path = Path(sessions_root) / session_id / "live_dashboard.json"
        payload = _read_json_if_exists(str(live_path))
        if not isinstance(payload, dict):
            continue
        meta = dict(payload.get("meta") or {})
        status = str(meta.get("status") or "").strip().lower()
        if status in {"starting", "collecting", "running", "active", "recording"}:
            meta["status"] = "waiting"
        meta["active"] = False
        meta["latest_session"] = True
        meta["note"] = meta.get("note") or "No active session; showing the latest completed live payload."
        payload = dict(payload)
        payload["meta"] = meta
        payload.setdefault("session_id", session_id)
        return payload
    return None


def _session_data_payload(sessions_root: str, session_id: str, points: int = 1000) -> Dict[str, object]:
    root = _session_path(sessions_root, session_id)
    csv_path = root / "radar.csv"
    if not csv_path.exists():
        csv_path = root / "radar_log.csv"
    if not csv_path.exists():
        return {"ok": False, "session_id": session_id, "rows": [], "error": {"code": "DATA_NOT_FOUND", "message": "no radar csv found"}}
    try:
        df = pd.read_csv(csv_path)
        ref_path = root / "ref.csv"
        if ref_path.is_file() and (
            "timestamp_s" in df.columns or "timestamp_ms" in df.columns
        ):
            try:
                ref_df = pd.read_csv(ref_path)
                if not ref_df.empty and (
                    "timestamp_s" in ref_df.columns or "timestamp_ms" in ref_df.columns
                ):
                    rad_ts = (
                        pd.to_numeric(df["timestamp_s"], errors="coerce")
                        if "timestamp_s" in df.columns
                        else pd.to_numeric(df["timestamp_ms"], errors="coerce") / 1000.0
                    )
                    ref_ts = (
                        pd.to_numeric(ref_df["timestamp_s"], errors="coerce")
                        if "timestamp_s" in ref_df.columns
                        else pd.to_numeric(ref_df["timestamp_ms"], errors="coerce") / 1000.0
                    )
                    rad_work = pd.DataFrame({"_row_idx": np.arange(len(df)), "_ts_s": rad_ts}).dropna(subset=["_ts_s"]).sort_values("_ts_s")
                    ref_small = pd.DataFrame({"_ts_s": ref_ts})
                    for src_col, alias_col in (("ref_hr", "ble_hr"), ("ref_rr", "ble_rr")):
                        if src_col in ref_df.columns:
                            ref_small[src_col] = pd.to_numeric(ref_df[src_col], errors="coerce")
                        elif alias_col in ref_df.columns:
                            ref_small[src_col] = pd.to_numeric(ref_df[alias_col], errors="coerce")
                    ref_small = ref_small.dropna(subset=["_ts_s"]).sort_values("_ts_s")
                    if not rad_work.empty and not ref_small.empty and any(c in ref_small.columns for c in ("ref_hr", "ref_rr")):
                        joined = pd.merge_asof(
                            rad_work,
                            ref_small,
                            on="_ts_s",
                            direction="nearest",
                            tolerance=1.25,
                        ).sort_values("_row_idx")
                        for src_col, alias_col in (("ref_hr", "ble_hr"), ("ref_rr", "ble_rr")):
                            if src_col in joined.columns:
                                aligned = pd.Series(np.nan, index=df.index, dtype=float)
                                aligned.iloc[joined["_row_idx"].to_numpy(dtype=int)] = joined[src_col].to_numpy(dtype=float)
                                if src_col not in df.columns:
                                    df[src_col] = aligned
                                if alias_col not in df.columns:
                                    df[alias_col] = aligned
            except Exception:
                pass
        keep_cols = [
            c
            for c in [
                "timestamp_s",
                "timestamp_ms",
                "reported_hr",
                "reported_rr",
                "pqi_heart",
                "pqi_breath",
                "distance_cm",
                "fps",
                "ref_hr",
                "ble_hr",
                "ref_rr",
                "ble_rr",
            ]
            if c in df.columns
        ]
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


def _build_report_export_html(session_dir: str, out_dir: Optional[str] = None) -> str:
    summary = _load_session_summary(session_dir)
    root = Path(session_dir)
    warnings = [f"Missing optional artifact: {rel}" for rel in ("analysis/analyse_hr_overlay.png", "analysis/analyse_rr_overlay.png", "ref_ble_raw.csv") if not (root / rel).exists()]
    signoff = _read_json_if_exists(str(root / "session_signoff.json")) or {}
    if not signoff.get("signed_at"):
        warnings.insert(0, "UNSIGNED REPORT: operator validation sign-off has not been recorded.")
    payload = json.dumps(nan_safe(summary), ensure_ascii=False)
    warn_html = "".join(f"<li>{html_escape(w)}</li>" for w in warnings) or "<li>No missing optional artifacts.</li>"
    raw_verdict = summary.get("ml_readiness_verdict")
    verdict = raw_verdict if isinstance(raw_verdict, dict) else {}
    raw_hr = summary.get("hr_metrics")
    hr = raw_hr if isinstance(raw_hr, dict) else {}
    raw_rr = summary.get("rr_metrics")
    rr = raw_rr if isinstance(raw_rr, dict) else {}
    raw_ref = summary.get("reference_quality")
    ref = raw_ref if isinstance(raw_ref, dict) else {}
    raw_truth = summary.get("truthfulness")
    truth = raw_truth if isinstance(raw_truth, dict) else {}
    raw_diagnosis = summary.get("contract_diagnosis")
    diagnosis = raw_diagnosis if isinstance(raw_diagnosis, dict) else {}
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
    raw_downloads = summary.get("downloads")
    download_items = raw_downloads if isinstance(raw_downloads, list) else []
    downloads = "".join(f"<li>{html_escape(str(d.get('label', d.get('relpath', 'file'))))}</li>" for d in download_items if isinstance(d, dict))
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
