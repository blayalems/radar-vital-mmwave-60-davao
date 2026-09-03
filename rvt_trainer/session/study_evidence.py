"""Durable study-control and reference-evidence primitives.

These helpers keep protocol configuration, randomized participant schedules,
reference observations, and analysis-job identity outside the legacy session
manifest.  The files are append-only where evidence is concerned and are
safe to replay from the trainer's configured sessions root.
"""

from __future__ import annotations

import hashlib
import random
import secrets
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

from rvt_trainer.api.common import atomic_write_json, read_json_if_exists


STUDY_PROTOCOL_SCHEMA_VERSION = "rvt-study-protocol-v2"
STUDY_PROTOCOL_ID = "RVT-THESIS-16.5.9"
STUDY_SCHEDULE_SCHEMA_VERSION = "rvt-study-schedule-v2"
REFERENCE_SCHEMA_VERSION = "rvt-reference-observations-v1"
STUDY_ANALYSIS_SCHEMA_VERSION = "rvt-study-analysis-v1"
STUDY_REPORT_SCHEMA_VERSION = "rvt-study-report-v2"
PROTOCOL_PATH = "study_protocol.json"
SCHEDULE_PATH = "study_schedules.json"
ANALYSIS_PATH = "study_analysis_jobs.json"
_EVIDENCE_LOCK = threading.RLock()
MAX_ANALYSIS_SESSIONS = 1024

_OBJECTIVE_POLICIES = {
    "objective_1_rr": {"confirmatory": True, "target": "rr"},
    "objective_2_temperature": {"confirmatory": False, "target": "temperature"},
    "objective_3_false_alarm": {"confirmatory": True, "target": "false_alarm"},
    "objective_4_hr": {"confirmatory": False, "target": "hr"},
}

CONDITIONS = (
    "d060_none",
    "d080_none",
    "d100_none",
    "d060_cardboard",
    "d080_cardboard",
    "d100_cardboard",
)


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _text(value: object, *, limit: int = 160) -> str:
    return str(value or "").replace("\r", " ").replace("\n", " ").strip()[:limit]


def _root(sessions_root: str) -> Path:
    root = Path(sessions_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    return root


def _load_map(path: Path) -> Dict[str, Any]:
    value = read_json_if_exists(str(path))
    return dict(value) if isinstance(value, dict) else {}


def _strict_optional_bool(value: object, *, field: str) -> Optional[bool]:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    raise ValueError(f"{field} must be a JSON boolean when supplied")


def _discover_analysis_sessions(root: Path, *, confirmatory: bool) -> list[str]:
    """Select a bounded, manifest-backed cohort when the client omits IDs."""

    selected: list[str] = []
    if not root.exists():
        return selected
    for manifest_path in sorted(root.glob("*/session_manifest.json")):
        session_dir = manifest_path.parent
        manifest = read_json_if_exists(str(manifest_path))
        if not isinstance(manifest, dict):
            continue
        if not (session_dir / "radar.csv").is_file() or not (session_dir / "ref.csv").is_file():
            continue
        if confirmatory and not (
            manifest.get("study_classification") == "confirmatory"
            and manifest.get("confirmatory_eligible") is True
            and str(manifest.get("participant_id") or "").strip()
        ):
            continue
        selected.append(session_dir.name)
    if len(selected) > MAX_ANALYSIS_SESSIONS:
        raise ValueError(
            f"analysis cohort exceeds the {MAX_ANALYSIS_SESSIONS}-session safety limit"
        )
    return selected


def default_protocol() -> Dict[str, Any]:
    return {
        "schema_version": STUDY_PROTOCOL_SCHEMA_VERSION,
        "protocol_id": STUDY_PROTOCOL_ID,
        "protocol_version": "2",
        "state": "draft",
        "randomization_seed": "rvt-v16.5.9-study-seed",
        "conditions": [
            {
                "condition_id": condition,
                "distance_m": int(condition[1:4]) / 100,
                "barrier_type": "cardboard" if condition.endswith("cardboard") else "none",
                "trial_count": 3,
                "planned_duration_s": 150,
                "confirmatory": True,
            }
            for condition in CONDITIONS
        ],
        "no_subject": {
            "trial_count": 72,
            "planned_duration_s": 150,
            "frozen_configuration": None,
        },
        "updated_at": _now(),
    }


def load_protocol(sessions_root: str) -> Dict[str, Any]:
    stored = read_json_if_exists(str(_root(sessions_root) / PROTOCOL_PATH))
    if not isinstance(stored, dict):
        return default_protocol()
    payload = default_protocol()
    payload.update(stored)
    payload["schema_version"] = STUDY_PROTOCOL_SCHEMA_VERSION
    return payload


def save_protocol(sessions_root: str, payload: Mapping[str, Any], *, actor: object = None) -> Dict[str, Any]:
    current = load_protocol(sessions_root)
    if str(current.get("state") or "draft") == "locked":
        raise ValueError("study protocol is locked and cannot be edited")
    merged = dict(current)
    for key in ("protocol_id", "protocol_version", "randomization_seed", "conditions", "no_subject"):
        if key in payload:
            merged[key] = payload[key]
    conditions = merged.get("conditions")
    if not isinstance(conditions, list) or {str(item.get("condition_id")) for item in conditions if isinstance(item, dict)} != set(CONDITIONS):
        raise ValueError("protocol conditions must contain the six canonical condition IDs")
    requested_state = str(payload.get("state") or current.get("state") or "draft")
    if requested_state not in {"draft", "locked"}:
        raise ValueError("protocol state must be draft or locked")
    if requested_state == "locked":
        from rvt_trainer.session.protocol_ledger import (
            NO_SUBJECT_MIN_DURATION_S,
            NO_SUBJECT_TRIAL_COUNT,
            validate_frozen_no_subject_configuration,
        )

        no_subject = merged.get("no_subject")
        if not isinstance(no_subject, dict):
            raise ValueError("protocol lock requires no_subject configuration")
        if (
            type(no_subject.get("trial_count")) is not int
            or no_subject.get("trial_count") != NO_SUBJECT_TRIAL_COUNT
        ):
            raise ValueError(
                f"protocol lock requires exactly {NO_SUBJECT_TRIAL_COUNT} no-subject trials"
            )
        planned_duration = no_subject.get("planned_duration_s")
        if (
            isinstance(planned_duration, bool)
            or not isinstance(planned_duration, (int, float))
            or float(planned_duration) != NO_SUBJECT_MIN_DURATION_S
        ):
            raise ValueError(
                f"protocol lock requires {NO_SUBJECT_MIN_DURATION_S:g}-second no-subject trials"
            )
        validate_frozen_no_subject_configuration(
            no_subject.get("frozen_configuration")
        )
    merged["state"] = requested_state
    merged["schema_version"] = STUDY_PROTOCOL_SCHEMA_VERSION
    merged["updated_at"] = _now()
    if requested_state == "locked":
        merged["locked_at"] = _now()
        merged["locked_by"] = _text(actor, limit=80) or None
    atomic_write_json(merged, str(_root(sessions_root) / PROTOCOL_PATH))
    return merged


def schedule_for_participant(sessions_root: str, participant_id: object) -> Dict[str, Any]:
    participant = _text(participant_id, limit=40).upper()
    if not participant:
        raise ValueError("participant_id is required")
    root = _root(sessions_root)
    schedules = _load_map(root / SCHEDULE_PATH)
    protocol = load_protocol(sessions_root)
    seed_text = f"{protocol.get('randomization_seed')}:{participant}"
    seed = hashlib.sha256(seed_text.encode("utf-8")).hexdigest()[:16]
    existing = schedules.get(participant)
    if isinstance(existing, dict) and isinstance(existing.get("entries"), list):
        return dict(existing)
    order = list(CONDITIONS)
    random.Random(seed).shuffle(order)
    entries = [
        {
            "participant_id": participant,
            "order": index + 1,
            "condition_id": condition,
            "trial_numbers": [1, 2, 3],
            "status": "missing",
            "seed": seed,
        }
        for index, condition in enumerate(order)
    ]
    result = {"ok": True, "schema_version": STUDY_SCHEDULE_SCHEMA_VERSION, "participant_id": participant, "seed": seed, "entries": entries}
    schedules[participant] = result
    atomic_write_json(schedules, str(root / SCHEDULE_PATH))
    return result


def _references_path(session_dir: Path) -> Path:
    return session_dir / "reference_observations.json"


def load_references(session_dir: str) -> Dict[str, Any]:
    root = Path(session_dir).resolve()
    value = read_json_if_exists(str(_references_path(root)))
    if not isinstance(value, dict):
        value = {"schema_version": REFERENCE_SCHEMA_VERSION, "references": [], "rr_adjudication": None}
    value.setdefault("schema_version", REFERENCE_SCHEMA_VERSION)
    value.setdefault("references", [])
    value.setdefault("rr_adjudication", None)
    return value


def append_reference(session_dir: str, payload: Mapping[str, Any], *, actor: object = None) -> Dict[str, Any]:
    with _EVIDENCE_LOCK:
        root = Path(session_dir).resolve()
        root.mkdir(parents=True, exist_ok=True)
        result = load_references(str(root))
        kind = _text(payload.get("kind"), limit=40).lower()
        if kind not in {"rr_observer", "temperature", "hr"}:
            raise ValueError("reference kind must be rr_observer, temperature, or hr")
        if kind == "temperature" and str(payload.get("barrier_type") or "none").lower() == "cardboard":
            raise ValueError("temperature reference capture is limited to unobstructed trials")
        observation = {
            "observation_id": f"OBS-{secrets.token_hex(10)}",
            "schema_version": REFERENCE_SCHEMA_VERSION,
            "session_id": root.name,
            "kind": kind,
            "observer_id": _text(payload.get("observer_id") or actor, limit=80) or None,
            "value": payload.get("value"),
            "unit": _text(payload.get("unit"), limit=24) or None,
            "duration_s": payload.get("duration_s"),
            "device_id": _text(payload.get("device_id"), limit=100) or None,
            "calibration_id": _text(payload.get("calibration_id"), limit=100) or None,
            "uncertainty": payload.get("uncertainty"),
            "observed_at": _text(payload.get("observed_at"), limit=40) or _now(),
            "missing_reason": _text(payload.get("missing_reason"), limit=160) or None,
            "locked": True,
        }
        references = result.get("references") if isinstance(result.get("references"), list) else []
        references.append(observation)
        result["references"] = references
        result["schema_version"] = REFERENCE_SCHEMA_VERSION
        atomic_write_json(result, str(_references_path(root)))
        return result


def adjudicate_rr(session_dir: str, payload: Mapping[str, Any], *, actor: object = None) -> Dict[str, Any]:
    with _EVIDENCE_LOCK:
        root = Path(session_dir).resolve()
        result = load_references(str(root))
        rr = [item for item in result.get("references", []) if isinstance(item, dict) and item.get("kind") == "rr_observer" and item.get("locked")]
        if len(rr) < 2:
            raise ValueError("RR adjudication requires two locked observer submissions")
        final_value = payload.get("final_value")
        if not isinstance(final_value, (int, float)):
            raise ValueError("final_value must be numeric")
        result["rr_adjudication"] = {
            "final_value": float(final_value),
            "rationale": _text(payload.get("rationale"), limit=500),
            "actor": _text(actor or payload.get("actor"), limit=80) or None,
            "locked_at": _now(),
        }
        atomic_write_json(result, str(_references_path(root)))
        return result


def create_analysis_job(sessions_root: str, payload: Mapping[str, Any]) -> Dict[str, Any]:
    with _EVIDENCE_LOCK:
        root = _root(sessions_root)
        jobs = _load_map(root / ANALYSIS_PATH)
        objective_id = _text(payload.get("objective_id"), limit=80)
        policy = _OBJECTIVE_POLICIES.get(objective_id)
        if policy is None:
            raise ValueError("objective_id must identify one of the four approved study objectives")
        requested_confirmatory = _strict_optional_bool(payload.get("confirmatory"), field="confirmatory")
        effective_confirmatory = bool(policy["confirmatory"])
        if requested_confirmatory is not None and requested_confirmatory != effective_confirmatory:
            raise ValueError(
                f"{objective_id} is {'confirmatory' if effective_confirmatory else 'exploratory'}; "
                "the server owns objective classification"
            )
        if str(policy["target"]) in {"temperature", "false_alarm"}:
            raise ValueError(
                f"{objective_id} is not a model-training objective; use the objective-specific evidence route"
            )
        requested_family = _text(payload.get("model_family") or "gradient_boosting", limit=40).lower()
        model_family = {"gbr": "gradient_boosting", "gradient_boosting": "gradient_boosting", "cnn": "cnn_1d", "cnn_1d": "cnn_1d"}.get(requested_family)
        if model_family is None:
            raise ValueError("model_family must be gradient_boosting/gbr or cnn_1d/cnn")
        job_id = f"JOB-{secrets.token_hex(10)}"
        now = _now()
        session_ids = payload.get("session_ids")
        if session_ids is None:
            session_ids = []
        if not isinstance(session_ids, list):
            raise ValueError("session_ids must be a list when supplied")
        if len(session_ids) > MAX_ANALYSIS_SESSIONS:
            raise ValueError(f"session_ids cannot contain more than {MAX_ANALYSIS_SESSIONS} entries")
        normalized_session_ids = []
        for session_id in session_ids:
            value = _text(session_id, limit=120)
            if value and value not in normalized_session_ids:
                normalized_session_ids.append(value)
        if not normalized_session_ids:
            normalized_session_ids = _discover_analysis_sessions(
                root,
                confirmatory=effective_confirmatory,
            )
        request = dict(payload)
        request["objective_id"] = objective_id
        request["confirmatory"] = effective_confirmatory
        request["target"] = policy["target"]
        request["session_ids"] = normalized_session_ids
        job = {
            "job_id": job_id,
            "schema_version": STUDY_ANALYSIS_SCHEMA_VERSION,
            "status": "queued",
            "objective_id": objective_id,
            "model_family": model_family,
            "created_at": now,
            "updated_at": now,
            "request": request,
            "progress_pct": 0,
            "last_line": "Queued; waiting for the trainer worker.",
            "cohort_selection": "server_discovered" if not payload.get("session_ids") else "client_explicit",
        }
        jobs[job_id] = job
        atomic_write_json(jobs, str(root / ANALYSIS_PATH))
        return job


def update_analysis_job(sessions_root: str, job_id: object, updates: Mapping[str, Any]) -> Dict[str, Any]:
    """Atomically merge worker state into one durable analysis job."""

    with _EVIDENCE_LOCK:
        root = _root(sessions_root)
        jobs = _load_map(root / ANALYSIS_PATH)
        key = _text(job_id, limit=100)
        current = jobs.get(key)
        if not isinstance(current, dict):
            raise KeyError(key)
        current = dict(current)
        current.update(dict(updates))
        current["updated_at"] = _now()
        jobs[key] = current
        atomic_write_json(jobs, str(root / ANALYSIS_PATH))
        return current


def request_analysis_cancel(sessions_root: str, job_id: object, *, reason: object = None) -> Optional[Dict[str, Any]]:
    """Atomically request cancellation before the server signals a child process."""

    with _EVIDENCE_LOCK:
        root = _root(sessions_root)
        jobs = _load_map(root / ANALYSIS_PATH)
        key = _text(job_id, limit=100)
        current = jobs.get(key)
        if not isinstance(current, dict):
            return None
        current = dict(current)
        status = str(current.get("status") or "")
        if status in {"completed", "failed", "cancelled"}:
            return current
        current.update({
            "status": "cancelling",
            "cancel_requested": True,
            "cancel_reason": _text(reason, limit=240) or "Cancelled by operator.",
            "updated_at": _now(),
        })
        jobs[key] = current
        atomic_write_json(jobs, str(root / ANALYSIS_PATH))
        return current


def finish_analysis_job(
    sessions_root: str,
    job_id: object,
    updates: Mapping[str, Any],
) -> Optional[Dict[str, Any]]:
    """Compare-and-set terminal worker state so cancellation cannot be overwritten."""

    with _EVIDENCE_LOCK:
        root = _root(sessions_root)
        jobs = _load_map(root / ANALYSIS_PATH)
        key = _text(job_id, limit=100)
        current = jobs.get(key)
        if not isinstance(current, dict) or str(current.get("status") or "") not in {"running", "queued"}:
            return dict(current) if isinstance(current, dict) else None
        current = dict(current)
        current.update(dict(updates))
        current["updated_at"] = _now()
        jobs[key] = current
        atomic_write_json(jobs, str(root / ANALYSIS_PATH))
        return current


def _sha256_file(path: Path) -> Optional[str]:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _contained(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def _validate_completed_job_artifacts(
    job: Mapping[str, Any],
    *,
    sessions_root: str,
    product_version: object,
) -> tuple[bool, list[dict[str, Any]], Optional[Path], Optional[dict[str, Any]], Optional[dict[str, Any]]]:
    """Validate the immutable identity and hashes before objective promotion."""

    root = _root(sessions_root)
    expected_version = _text(product_version, limit=32) or "16.6.3"
    job_id = _text(job.get("job_id"), limit=100)
    expected_output_root = (root / "study_analysis" / job_id).resolve()
    output_value = _text(job.get("output_dir"), limit=1000)
    output_dir = Path(output_value).resolve() if output_value else None
    reasons: list[dict[str, Any]] = []
    if output_dir is None or output_dir != expected_output_root or not output_dir.is_dir():
        reasons.append({"reason": "analysis_output_path_not_contained"})
        return False, reasons, output_dir, None, None
    manifest = read_json_if_exists(str(output_dir / "confirmatory_run_manifest.json"))
    summary = read_json_if_exists(str(output_dir / "train_summary.json"))
    stats = read_json_if_exists(str(output_dir / "statistical_report.json"))
    if not isinstance(manifest, dict):
        reasons.append({"reason": "confirmatory_run_manifest_missing"})
        return False, reasons, output_dir, None, summary if isinstance(summary, dict) else None
    request = job.get("request") if isinstance(job.get("request"), dict) else {}
    if manifest.get("analysis_job_id") != job_id:
        reasons.append({"reason": "analysis_job_identity_mismatch"})
    if manifest.get("run_product_version") != expected_version:
        reasons.append({"reason": "product_version_mismatch"})
    if manifest.get("model_family") != job.get("model_family"):
        reasons.append({"reason": "model_family_mismatch"})
    if manifest.get("analysis_plan_id") != "RVT-STA-PLAN-16.5.8":
        reasons.append({"reason": "analysis_plan_identity_mismatch"})
    plan_sha256 = _text(manifest.get("analysis_plan_sha256"), limit=64).lower()
    if len(plan_sha256) != 64 or any(char not in "0123456789abcdef" for char in plan_sha256):
        reasons.append({"reason": "analysis_plan_hash_invalid"})
    if manifest.get("study_protocol_id") != STUDY_PROTOCOL_ID:
        reasons.append({"reason": "study_protocol_identity_mismatch"})
    if manifest.get("study_session_schema_version") != "rvt-study-session-v16.5.9":
        reasons.append({"reason": "study_session_schema_identity_mismatch"})
    if not _text(manifest.get("source_commit"), limit=100):
        reasons.append({"reason": "source_commit_missing"})
    if not isinstance(manifest.get("folds"), dict) or manifest["folds"].get("complete") is not True:
        reasons.append({"reason": "fold_coverage_incomplete"})
    if request.get("confirmatory") is not True:
        reasons.append({"reason": "confirmatory_request_missing"})
    if str(job.get("objective_id") or "") == "objective_1_rr" and "rr" not in list(manifest.get("targets") or []):
        reasons.append({"reason": "rr_target_missing"})

    oof = manifest.get("outer_oof_predictions")
    oof_path = Path(str(oof.get("path") or "")).resolve() if isinstance(oof, dict) else None
    if oof_path is None or not _contained(oof_path, output_dir) or not oof_path.is_file():
        reasons.append({"reason": "outer_oof_path_invalid"})
    else:
        try:
            oof_rows = int(oof.get("rows") or 0)
        except (TypeError, ValueError):
            oof_rows = 0
        if _sha256_file(oof_path) != oof.get("sha256") or oof_rows < 1:
            reasons.append({"reason": "outer_oof_hash_or_row_count_invalid"})

    artifact_hashes = manifest.get("model_artifact_sha256")
    if not isinstance(artifact_hashes, dict) or not artifact_hashes:
        reasons.append({"reason": "model_artifact_hashes_missing"})
    else:
        for name, expected_hash in artifact_hashes.items():
            artifact = (output_dir / str(name)).resolve()
            if not _contained(artifact, output_dir) or _sha256_file(artifact) != expected_hash:
                reasons.append({"reason": "model_artifact_hash_invalid", "artifact": str(name)})

    input_hashes = manifest.get("input_file_sha256")
    if not isinstance(input_hashes, dict):
        reasons.append({"reason": "input_hashes_missing"})
    else:
        for group in ("radar", "reference"):
            values = input_hashes.get(group)
            if not isinstance(values, dict) or not values:
                reasons.append({"reason": "input_hashes_missing", "group": group})
                continue
            for raw_path, expected_hash in values.items():
                candidate = Path(str(raw_path)).resolve()
                if not _contained(candidate, root) or _sha256_file(candidate) != expected_hash:
                    reasons.append({"reason": "input_hash_invalid", "path": str(raw_path)})

    if not isinstance(summary, dict) or not isinstance(summary.get("confirmatory_contract"), dict) or summary["confirmatory_contract"].get("status") != "ready":
        reasons.append({"reason": "training_confirmatory_contract_missing"})
    if job.get("statistics_status") != "completed" or not isinstance(stats, dict) or stats.get("confirmatory") is not True:
        reasons.append({"reason": "statistical_report_missing_or_blocked"})
    else:
        stats_provenance = stats.get("provenance")
        identity_fields = (
            "run_product_version",
            "analysis_plan_id",
            "analysis_plan_sha256",
            "study_protocol_id",
            "study_session_schema_version",
        )
        if not isinstance(stats_provenance, dict) or any(
            stats_provenance.get(field) != manifest.get(field)
            for field in identity_fields
        ):
            reasons.append({"reason": "statistical_report_identity_mismatch"})
    return not reasons, reasons, output_dir, manifest, summary if isinstance(summary, dict) else None


def cancel_analysis_job(sessions_root: str, job_id: object, *, reason: object = None) -> Optional[Dict[str, Any]]:
    """Mark a queued/running job cancelled; the server owns process termination."""

    job = load_analysis_job(sessions_root, job_id)
    if job is None:
        return None
    if str(job.get("status") or "") in {"completed", "failed", "cancelled"}:
        return job
    return update_analysis_job(
        sessions_root,
        job_id,
        {
            "status": "cancelled",
            "progress_pct": int(job.get("progress_pct") or 0),
            "error": _text(reason, limit=240) or "Cancelled by operator.",
            "last_line": "Cancelled by operator.",
            "completed_at": _now(),
        },
    )


def load_analysis_job(sessions_root: str, job_id: object) -> Optional[Dict[str, Any]]:
    with _EVIDENCE_LOCK:
        jobs = _load_map(_root(sessions_root) / ANALYSIS_PATH)
        value = jobs.get(_text(job_id, limit=100))
        return dict(value) if isinstance(value, dict) else None


def list_analysis_jobs(sessions_root: str, *, limit: int = 20) -> list[Dict[str, Any]]:
    """Return the newest durable study-analysis jobs for dashboard recovery."""

    bounded_limit = max(1, min(100, int(limit)))
    # Serialize readers with worker updates. Windows can briefly deny a read
    # while the atomic replacement is in flight; treating that transient as an
    # empty map makes a just-created durable job disappear from the API list.
    with _EVIDENCE_LOCK:
        jobs = _load_map(_root(sessions_root) / ANALYSIS_PATH)
        rows = [dict(value) for value in jobs.values() if isinstance(value, dict)]
    rows.sort(key=lambda row: str(row.get("updated_at") or row.get("created_at") or ""), reverse=True)
    return rows[:bounded_limit]


def objective_report(
    objective_id: object,
    *,
    objectives: Mapping[str, Any],
    sessions_root: str,
    product_version: object = "16.6.3",
) -> Dict[str, Any]:
    wanted = _text(objective_id, limit=80)
    known = [item for item in objectives.get("objectives", []) if isinstance(item, dict) and str(item.get("id")) == wanted]
    if not known:
        raise KeyError(wanted)
    # False-alarm evidence is a trial-ledger problem, not a model-training job.
    # Build the approved report directly from exactly 72 independently bound,
    # protocol-qualified control captures.
    if wanted == "objective_3_false_alarm":
        from rvt_trainer.session.protocol_ledger import (
            completion_matrix,
            no_subject_capture_interval,
            qualified_no_subject_attempts,
        )

        matrix = completion_matrix(sessions_root)
        rows = qualified_no_subject_attempts(sessions_root)
        qualified = len(rows)
        expected = int(known[0].get("trial_count") or matrix.get("no_subject_expected") or 72)
        threshold = float(known[0].get("threshold") or 0.05)
        common_provenance = {
            "product_version": _text(product_version, limit=32) or "16.6.3",
            "objective": known[0],
            "sessions_root": str(Path(sessions_root).resolve()),
            "no_subject_attempt_count": int(matrix.get("no_subject_attempt_count") or 0),
            "no_subject_qualified_count": qualified,
            "no_subject_unqualified_count": int(matrix.get("no_subject_unqualified_count") or 0),
        }
        if qualified != expected:
            reason = (
                "insufficient_qualified_no_subject_evidence"
                if qualified < expected
                else "excess_qualified_no_subject_evidence"
            )
            return {
                "ok": True,
                "objective_id": wanted,
                "schema_version": STUDY_REPORT_SCHEMA_VERSION,
                "status": "inconclusive",
                "report": None,
                "exclusions": [{
                    "reason": reason,
                    "qualified": qualified,
                    "expected": expected,
                }],
                "provenance": common_provenance,
            }

        attempt_ids = [_text(row.get("attempt_id"), limit=64) for row in rows]
        session_ids = [_text(row.get("session_id"), limit=120) for row in rows]
        configuration_hashes = sorted({
            _text(row.get("frozen_configuration_hash"), limit=64).lower()
            for row in rows
        })
        if (
            len(set(attempt_ids)) != expected
            or len(set(session_ids)) != expected
            or len(configuration_hashes) != 1
        ):
            return {
                "ok": True,
                "objective_id": wanted,
                "schema_version": STUDY_REPORT_SCHEMA_VERSION,
                "status": "inconclusive",
                "report": None,
                "exclusions": [{
                    "reason": "non_independent_or_mixed_configuration_no_subject_evidence",
                    "unique_attempts": len(set(attempt_ids)),
                    "unique_sessions": len(set(session_ids)),
                    "configuration_count": len(configuration_hashes),
                    "expected": expected,
                }],
                "provenance": common_provenance,
            }

        capture_intervals = []
        for row in rows:
            interval = no_subject_capture_interval(row, sessions_root)
            if interval is None:
                return {
                    "ok": True,
                    "objective_id": wanted,
                    "schema_version": STUDY_REPORT_SCHEMA_VERSION,
                    "status": "inconclusive",
                    "report": None,
                    "exclusions": [{
                        "reason": "missing_no_subject_capture_interval_evidence",
                        "session_id": _text(row.get("session_id"), limit=120),
                    }],
                    "provenance": common_provenance,
                }
            capture_intervals.append((
                interval[0],
                interval[1],
                _text(row.get("session_id"), limit=120),
            ))
        capture_intervals.sort(key=lambda item: (item[0], item[1], item[2]))
        previous_start, previous_end, previous_session = capture_intervals[0]
        for captured_at, ended_at, session_id in capture_intervals[1:]:
            if captured_at < previous_end:
                return {
                    "ok": True,
                    "objective_id": wanted,
                    "schema_version": STUDY_REPORT_SCHEMA_VERSION,
                    "status": "inconclusive",
                    "report": None,
                    "exclusions": [{
                        "reason": "overlapping_no_subject_capture_intervals",
                        "first_session_id": previous_session,
                        "first_captured_at": previous_start.isoformat(),
                        "first_ended_at": previous_end.isoformat(),
                        "overlapping_session_id": session_id,
                        "overlapping_captured_at": captured_at.isoformat(),
                        "overlapping_ended_at": ended_at.isoformat(),
                    }],
                    "provenance": common_provenance,
                }
            previous_start, previous_end, previous_session = (
                captured_at,
                ended_at,
                session_id,
            )

        false_alarm_trials = sum(int(row["false_alarm_count"]) for row in rows)
        try:
            from rvt_trainer.statistics import exact_proportion

            exact = exact_proportion(
                false_alarm_trials,
                expected,
                confidence=0.95,
                null_proportion=threshold,
                alternative="less",
            )
        except Exception as exc:
            return {
                "ok": True,
                "objective_id": wanted,
                "schema_version": STUDY_REPORT_SCHEMA_VERSION,
                "status": "blocked",
                "report": None,
                "exclusions": [{
                    "reason": "false_alarm_statistics_unavailable",
                    "detail": _text(exc, limit=240),
                }],
                "provenance": common_provenance,
            }

        alpha = 0.05
        reject_null = float(exact["p_value"]) < alpha
        report = {
            "estimand": "trial-level no-subject false-alarm proportion",
            "planned_trial_count": expected,
            "qualified_trial_count": qualified,
            "false_alarm_trial_count": false_alarm_trials,
            "observed_false_alarm_rate": exact["proportion"],
            "confidence_interval_95": {
                "confidence": exact["confidence"],
                "low": exact["ci_low"],
                "high": exact["ci_high"],
                "method": exact["method"],
            },
            "exact_binomial_test": {
                "null_hypothesis": f"p >= {threshold:g}",
                "alternative_hypothesis": f"p < {threshold:g}",
                "null_proportion": exact["null_proportion"],
                "alternative": exact["alternative"],
                "p_value": exact["p_value"],
                "alpha": alpha,
                "reject_null": reject_null,
                "decision": (
                    "supports_false_alarm_rate_below_threshold"
                    if reject_null
                    else "does_not_support_false_alarm_rate_below_threshold"
                ),
            },
            # Retain the common statistics payload so downstream manuscript
            # exporters can use the same field contract as coverage_report().
            "false_alarm_exact": exact,
        }
        return {
            "ok": True,
            "objective_id": wanted,
            "schema_version": STUDY_REPORT_SCHEMA_VERSION,
            "status": "ready",
            "report": report,
            "exclusions": [],
            "provenance": {
                **common_provenance,
                "frozen_configuration_hash": configuration_hashes[0],
                "capture_interval_count": len(capture_intervals),
                "attempt_ids": attempt_ids,
                "session_ids": session_ids,
            },
        }

    jobs = _load_map(_root(sessions_root) / ANALYSIS_PATH)
    completed = [
        row for row in jobs.values()
        if isinstance(row, dict)
        and str(row.get("objective_id") or "") == wanted
        and str(row.get("status") or "") == "completed"
    ]
    completed.sort(key=lambda row: str(row.get("updated_at") or ""), reverse=True)
    latest = completed[0] if completed else None
    if latest is None:
        status = "inconclusive"
        report = None
        exclusions = [{"reason": "no_completed_analysis_job", "count": 1}]
    else:
        request = latest.get("request") if isinstance(latest.get("request"), dict) else {}
        if request.get("confirmatory") is not True:
            output_value = _text(latest.get("output_dir"), limit=1000)
            output_dir = Path(output_value).resolve() if output_value else None
            manifest = read_json_if_exists(str(output_dir / "confirmatory_run_manifest.json")) if output_dir else None
            summary = read_json_if_exists(str(output_dir / "train_summary.json")) if output_dir else None
            stats = None
            valid = False
            validation_reasons = [{"reason": "exploratory_analysis"}]
            status = "descriptive"
        else:
            valid, validation_reasons, output_dir, manifest, summary = _validate_completed_job_artifacts(
                latest,
                sessions_root=sessions_root,
                product_version=product_version,
            )
            stats = read_json_if_exists(str(output_dir / "statistical_report.json")) if output_dir else None
            status = "ready" if valid and isinstance(manifest, dict) and manifest.get("status") == "ready" else "inconclusive"
        report = {
            "job_id": latest.get("job_id"),
            "model_family": latest.get("model_family"),
            "output_dir": str(output_dir) if output_dir else None,
            "train_summary": summary if isinstance(summary, dict) else None,
            "confirmatory_run_manifest": manifest if isinstance(manifest, dict) else None,
            "statistical_report": stats if isinstance(stats, dict) else None,
        }
        exclusions = [] if status == "ready" else validation_reasons
    return {
        "ok": True,
        "objective_id": wanted,
        "schema_version": STUDY_REPORT_SCHEMA_VERSION,
        "status": status,
        "report": report,
        "exclusions": exclusions,
        "provenance": {
            "product_version": _text(product_version, limit=32) or "16.6.3",
            "objective": known[0],
            "latest_job_id": latest.get("job_id") if latest else None,
            "sessions_root": str(Path(sessions_root).resolve()),
        },
    }


__all__ = [
    "STUDY_PROTOCOL_SCHEMA_VERSION",
    "STUDY_PROTOCOL_ID",
    "STUDY_SCHEDULE_SCHEMA_VERSION",
    "REFERENCE_SCHEMA_VERSION",
    "STUDY_ANALYSIS_SCHEMA_VERSION",
    "STUDY_REPORT_SCHEMA_VERSION",
    "adjudicate_rr",
    "append_reference",
    "create_analysis_job",
    "cancel_analysis_job",
    "finish_analysis_job",
    "default_protocol",
    "load_analysis_job",
    "list_analysis_jobs",
    "load_protocol",
    "load_references",
    "objective_report",
    "update_analysis_job",
    "request_analysis_cancel",
    "save_protocol",
    "schedule_for_participant",
]
