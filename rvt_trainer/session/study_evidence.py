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
STUDY_SCHEDULE_SCHEMA_VERSION = "rvt-study-schedule-v2"
REFERENCE_SCHEMA_VERSION = "rvt-reference-observations-v1"
STUDY_ANALYSIS_SCHEMA_VERSION = "rvt-study-analysis-v1"
STUDY_REPORT_SCHEMA_VERSION = "rvt-study-report-v2"
PROTOCOL_PATH = "study_protocol.json"
SCHEDULE_PATH = "study_schedules.json"
ANALYSIS_PATH = "study_analysis_jobs.json"
_EVIDENCE_LOCK = threading.RLock()

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


def default_protocol() -> Dict[str, Any]:
    return {
        "schema_version": STUDY_PROTOCOL_SCHEMA_VERSION,
        "protocol_id": "RVT-THESIS-16.5.9",
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
        requested_family = _text(payload.get("model_family") or "gradient_boosting", limit=40).lower()
        model_family = {"gbr": "gradient_boosting", "gradient_boosting": "gradient_boosting", "cnn": "cnn_1d", "cnn_1d": "cnn_1d"}.get(requested_family)
        if model_family is None:
            raise ValueError("model_family must be gradient_boosting/gbr or cnn_1d/cnn")
        job_id = f"JOB-{secrets.token_hex(10)}"
        now = _now()
        job = {"job_id": job_id, "schema_version": STUDY_ANALYSIS_SCHEMA_VERSION, "status": "queued", "objective_id": _text(payload.get("objective_id"), limit=80) or None, "model_family": model_family, "created_at": now, "updated_at": now, "request": dict(payload)}
        jobs[job_id] = job
        atomic_write_json(jobs, str(root / ANALYSIS_PATH))
        return job


def load_analysis_job(sessions_root: str, job_id: object) -> Optional[Dict[str, Any]]:
    jobs = _load_map(_root(sessions_root) / ANALYSIS_PATH)
    value = jobs.get(_text(job_id, limit=100))
    return dict(value) if isinstance(value, dict) else None


def objective_report(objective_id: object, *, objectives: Mapping[str, Any], sessions_root: str) -> Dict[str, Any]:
    wanted = _text(objective_id, limit=80)
    known = [item for item in objectives.get("objectives", []) if isinstance(item, dict) and str(item.get("id")) == wanted]
    if not known:
        raise KeyError(wanted)
    return {
        "ok": True,
        "objective_id": wanted,
        "schema_version": STUDY_REPORT_SCHEMA_VERSION,
        "status": "inconclusive",
        "report": None,
        "exclusions": [{"reason": "no_completed_confirmatory_analysis", "count": 1}],
        "provenance": {"product_version": "16.5.9", "objective": known[0], "sessions_root": str(Path(sessions_root).resolve())},
    }


__all__ = [
    "STUDY_PROTOCOL_SCHEMA_VERSION",
    "STUDY_SCHEDULE_SCHEMA_VERSION",
    "REFERENCE_SCHEMA_VERSION",
    "STUDY_ANALYSIS_SCHEMA_VERSION",
    "STUDY_REPORT_SCHEMA_VERSION",
    "adjudicate_rr",
    "append_reference",
    "create_analysis_job",
    "default_protocol",
    "load_analysis_job",
    "load_protocol",
    "load_references",
    "objective_report",
    "save_protocol",
    "schedule_for_participant",
]
