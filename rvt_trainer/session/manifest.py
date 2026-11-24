"""Session manifest and reproducibility provenance persistence."""

from __future__ import annotations

import os
import secrets
from typing import Dict, Optional, Sequence

from rvt_trainer.api.common import (
    atomic_write_json as save_json,
    read_json_if_exists as _read_json_if_exists,
)
from rvt_trainer.constants import (
    DASHBOARD_VERSION,
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    FEATURE_ENGINEERING_VERSION,
    FEATURE_SCHEMA_VERSION,
    FIRMWARE_VERSION_EXPECTED,
    SESSION_MANIFEST_SCHEMA_VERSION,
    SESSION_MANIFEST_VERSION,
    VERSION,
    _file_sha256,
    _report_stamp,
    _scoring_weights_hash,
    warn,
)
from rvt_trainer.features import feature_schema_hash
from rvt_trainer.session.protocol_ledger import (
    append_session_attempt_event as _append_session_attempt_event,
    initialize_session_attempt as _initialize_session_attempt,
)
from rvt_trainer.session.study_contract import (
    merge_immutable_study_assignment as _merge_immutable_study_assignment,
    release_provenance as _release_provenance,
)

def _session_root_for_outputs(out_dir: str) -> str:
    out_abs = os.path.abspath(out_dir)
    if os.path.basename(out_abs).lower() in {"analysis", "analyse"}:
        return os.path.dirname(out_abs)
    return out_abs


def _manifest_identity(fw_truthfulness: Optional[Dict[str, object]] = None) -> Dict[str, object]:
    fw_truthfulness = fw_truthfulness if isinstance(fw_truthfulness, dict) else {}
    identity = {
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
    identity.update(_release_provenance(
        product_version=VERSION,
        trainer_version=VERSION,
        dashboard_version=DASHBOARD_VERSION,
        firmware_expected=FIRMWARE_VERSION_EXPECTED,
        firmware_observed=fw_truthfulness.get("version"),
        serial_width_expected=EXPECTED_RADAR_LOG_COLUMN_COUNT,
        serial_width_observed=fw_truthfulness.get("contract_length"),
    ))
    return identity


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
    existing_manifest = _read_json_if_exists(
        os.path.join(session_root, "session_manifest.json")
    ) or {}
    current_identity = _manifest_identity(fw_truthfulness)
    # Capture identity is frozen at session allocation. Re-analysis may use a
    # newer trainer, but it must never relabel the original capture as the
    # newer release. Legacy manifests are migrated once from their existing
    # top-level identity and then treated the same way.
    capture_provenance = existing_manifest.get("capture_provenance")
    if not isinstance(capture_provenance, dict):
        capture_provenance = {
            "captured_at": existing_manifest.get("generated_at") or _report_stamp(),
            "source": "legacy_manifest_migration",
            **{
                key: existing_manifest.get(key, value)
                for key, value in current_identity.items()
            },
            "participant_id": existing_manifest.get("participant_id"),
            "logical_trial_id": existing_manifest.get("logical_trial_id"),
            "attempt_id": existing_manifest.get("attempt_id"),
        }
    capture_identity = {
        key: capture_provenance.get(key, value)
        for key, value in current_identity.items()
    }
    analysis_runs = existing_manifest.get("analysis_runs")
    if not isinstance(analysis_runs, list):
        analysis_runs = []
    analysis_run = {
        "run_id": f"AR-{secrets.token_hex(10)}",
        "started_at": _report_stamp(),
        "completed_at": _report_stamp(),
        "source_commit": current_identity.get("source_commit"),
        "trainer_version": current_identity.get("trainer_version"),
        "feature_schema_hash": current_identity.get("feature_schema_hash"),
        "analysis_dir": analysis_dir,
        "input_file_hashes_sha256": {
            key: value
            for key, value in file_hashes.items()
            if key in {"radar_csv", "ref_csv"}
        },
    }
    analysis_runs = [*analysis_runs, analysis_run]
    radar_available = bool(radar_paths)
    manifest_status = str(existing_manifest.get("status") or "")
    if manifest_status not in {"failed_start", "aborted", "invalid"}:
        manifest_status = "completed" if radar_available else "no_output"
    manifest = {
        "schema_version": SESSION_MANIFEST_SCHEMA_VERSION,
        "manifest_version": SESSION_MANIFEST_VERSION,
        "generated_at": _report_stamp(),
        "session_root": session_root,
        **capture_identity,
        "capture_provenance": capture_provenance,
        "analysis_runs": analysis_runs,
        "status": manifest_status,
        "terminal": True,
        "ended_at": existing_manifest.get("ended_at") or _report_stamp(),
        "auto_analysed": os.path.exists(os.path.join(analysis_dir, "analyse_summary.json")),
        "tags": list(existing_manifest.get("tags", [])),
        "notes_count": len((_read_json_if_exists(os.path.join(session_root, "session_notes.json")) or {}).get("notes", [])),
        "subject_profile_id": existing_manifest.get("subject_profile_id", "adult_default"),
        "outputs": outputs,
        "file_hashes_sha256": file_hashes,
    }
    manifest = _merge_immutable_study_assignment(existing_manifest, manifest)
    for key in (
        "study_session_schema_version",
        "participant_id",
        "trial_id",
        "condition_id",
        "distance_m",
        "barrier_type",
        "trial_number",
        "planned_duration_s",
        "study_classification",
        "provenance_state",
        "confirmatory_eligible",
        "logical_trial_id",
        "attempt_id",
        "attempt_type",
        "attempt_ledger_schema_version",
        "model_family",
        "model_bundle",
        "client_compatibility",
        "client_handshake",
        "release_compatibility_state",
        "release_compatibility_verified",
    ):
        if key in existing_manifest:
            manifest[key] = existing_manifest[key]
    if "capture_provenance" not in existing_manifest:
        manifest["capture_provenance"] = capture_provenance
    if "analysis_runs" not in existing_manifest:
        manifest["analysis_runs"] = analysis_runs
    try:
        _initialize_session_attempt(session_root, manifest)
        final_status = "completed" if radar_available else "no_output"
        _append_session_attempt_event(
            session_root,
            final_status,
            reason="analysis finalized" if manifest.get("auto_analysed") else "capture finalized",
            details={"radar_paths": len(radar_paths), "ref_paths": len(ref_paths)},
        )
    except ValueError:
        # Legacy or externally-created directories may not have a protocol
        # ledger. Their session manifest remains readable and explicitly
        # carries the migration provenance above.
        pass
    save_json(manifest, os.path.join(session_root, "session_manifest.json"))
    return manifest
