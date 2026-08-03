"""Append-only protocol-attempt evidence for study-ready captures.

The session manifest identifies the capture, while this module records the
protocol state transitions and the attempts that were *actually tried*.  The
distinction is important for coverage: an aborted, invalid, or no-subject
attempt must remain in the denominator instead of disappearing with the
prediction file.
"""

from __future__ import annotations

import re
import secrets
import time
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

from rvt_trainer.api.common import atomic_write_json, read_json_if_exists

ATTEMPT_LEDGER_SCHEMA_VERSION = "rvt-protocol-attempt-ledger-v16.5.9"
PROTOCOL_ATTEMPTS_SCHEMA_VERSION = "rvt-protocol-attempts-v16.5.9"
ATTEMPT_TYPES = frozenset({"subject", "no_subject"})
ATTEMPT_STATUSES = frozenset(
    {
        "allocated",
        "starting",
        "collecting",
        "completed",
        "stopped",
        "failed_start",
        "aborted",
        "invalid",
        "no_output",
    }
)
TERMINAL_ATTEMPT_STATUSES = frozenset(
    {"completed", "stopped", "failed_start", "aborted", "invalid", "no_output"}
)
CONFIRMATORY_CONDITION_IDS = (
    "d060_none",
    "d080_none",
    "d100_none",
    "d060_cardboard",
    "d080_cardboard",
    "d100_cardboard",
)
TRIAL_NUMBERS = (1, 2, 3)
_PARTICIPANT_RE = re.compile(r"^P-[0-9]{3}$")
_CONDITION_RE = re.compile(r"^d[0-9]{3}_(?:none|cardboard)$")


def _iso_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _safe_text(value: object, *, default: str = "", limit: int = 160) -> str:
    text = str(value or "").replace("\r", " ").replace("\n", " ").strip()
    return text[:limit]


def canonical_logical_trial_id(
    participant_id: object,
    condition_id: object,
    trial_number: object,
) -> str:
    """Return the stable logical trial key shared by UI, capture, and stats."""

    participant = _safe_text(participant_id).upper()
    condition = _safe_text(condition_id).lower()
    if not _PARTICIPANT_RE.fullmatch(participant):
        raise ValueError("participant_id must use P-### form")
    if not _CONDITION_RE.fullmatch(condition):
        raise ValueError("condition_id must use dNNN_none or dNNN_cardboard form")
    try:
        trial = int(trial_number)
    except (TypeError, ValueError):
        raise ValueError("trial_number must be an integer") from None
    if trial < 1:
        raise ValueError("trial_number must be positive")
    return f"{participant}-{condition.replace('_', '-')}-t{trial}"


def allocate_attempt_id() -> str:
    """Allocate an opaque, non-PII attempt identifier."""

    return f"AT-{secrets.token_hex(10)}"


def ledger_path(session_root: str) -> Path:
    return Path(session_root).resolve() / "protocol_attempt.json"


def protocol_attempts_path(sessions_root: str) -> Path:
    return Path(sessions_root).resolve() / "protocol_attempts.json"


def _event(
    status: str,
    *,
    reason: object = None,
    actor: object = None,
    details: object = None,
    at: Optional[str] = None,
) -> Dict[str, object]:
    if status not in ATTEMPT_STATUSES:
        raise ValueError(f"unsupported protocol attempt status: {status}")
    item: Dict[str, object] = {
        "event_id": f"EV-{secrets.token_hex(10)}",
        "status": status,
        "at": at or _iso_now(),
    }
    if reason not in (None, ""):
        item["reason"] = _safe_text(reason)
    if actor not in (None, ""):
        item["actor"] = _safe_text(actor, limit=80)
    if details not in (None, "", [], {}):
        item["details"] = details
    return item


def _base_attempt_record(
    *,
    attempt_id: str,
    attempt_type: str,
    status: str,
    participant_id: object = None,
    logical_trial_id: object = None,
    trial_id: object = None,
    condition_id: object = None,
    trial_number: object = None,
    product_version: object = None,
    protocol_id: object = None,
) -> Dict[str, object]:
    if attempt_type not in ATTEMPT_TYPES:
        raise ValueError("attempt_type must be subject or no_subject")
    if status not in ATTEMPT_STATUSES:
        raise ValueError(f"unsupported protocol attempt status: {status}")
    record: Dict[str, object] = {
        "schema_version": ATTEMPT_LEDGER_SCHEMA_VERSION,
        "attempt_id": _safe_text(attempt_id, limit=64),
        "attempt_type": attempt_type,
        "participant_id": (
            _safe_text(participant_id).upper() if participant_id not in (None, "") else None
        ),
        "logical_trial_id": _safe_text(logical_trial_id, limit=120) or None,
        "trial_id": _safe_text(trial_id, limit=120) or None,
        "condition_id": _safe_text(condition_id).lower() or None,
        "trial_number": int(trial_number) if trial_number not in (None, "") else None,
        "product_version": _safe_text(product_version) or None,
        "protocol_id": _safe_text(protocol_id) or None,
        "status": status,
        "terminal": status in TERMINAL_ATTEMPT_STATUSES,
        "created_at": _iso_now(),
        "updated_at": _iso_now(),
        "events": [],
    }
    record["events"] = [_event(status)]
    return record


def initialize_session_attempt(
    session_root: str,
    manifest: Mapping[str, object],
    *,
    attempt_id: Optional[str] = None,
) -> Dict[str, object]:
    """Persist the first immutable attempt record for a session."""

    existing = read_json_if_exists(str(ledger_path(session_root)))
    if isinstance(existing, dict) and existing.get("attempt_id"):
        return existing
    participant_id = manifest.get("participant_id")
    attempt_type = str(manifest.get("attempt_type") or "subject")
    logical_trial_id = manifest.get("logical_trial_id")
    if attempt_type == "subject" and not logical_trial_id and participant_id:
        try:
            logical_trial_id = canonical_logical_trial_id(
                participant_id,
                manifest.get("condition_id"),
                manifest.get("trial_number"),
            )
        except ValueError:
            logical_trial_id = None
    record = _base_attempt_record(
        attempt_id=attempt_id or str(manifest.get("attempt_id") or allocate_attempt_id()),
        attempt_type=attempt_type,
        status="allocated",
        participant_id=participant_id,
        logical_trial_id=logical_trial_id,
        trial_id=manifest.get("trial_id"),
        condition_id=manifest.get("condition_id"),
        trial_number=manifest.get("trial_number"),
        product_version=manifest.get("product_version"),
        protocol_id=manifest.get("study_session_schema_version"),
    )
    atomic_write_json(record, str(ledger_path(session_root)))
    return record


def append_session_attempt_event(
    session_root: str,
    status: str,
    *,
    reason: object = None,
    actor: object = None,
    details: object = None,
) -> Dict[str, object]:
    """Append one state transition without rewriting prior events."""

    path = ledger_path(session_root)
    record = read_json_if_exists(str(path))
    if not isinstance(record, dict) or not record.get("attempt_id"):
        raise ValueError("protocol attempt ledger is not initialized")
    events = record.get("events")
    if not isinstance(events, list):
        events = []
    current_status = str(record.get("status") or "")
    if current_status in TERMINAL_ATTEMPT_STATUSES:
        # Re-analysis may call the finalizer again. It must be idempotent and
        # must never append a new state after the terminal capture state.
        if current_status == status:
            return record
        raise ValueError(
            f"protocol attempt {record.get('attempt_id')} is terminal ({current_status})"
        )
    event = _event(status, reason=reason, actor=actor, details=details)
    events.append(event)
    record["events"] = events
    record["status"] = status
    record["terminal"] = status in TERMINAL_ATTEMPT_STATUSES
    record["updated_at"] = event["at"]
    atomic_write_json(record, str(path))
    return record


def register_protocol_attempt(
    sessions_root: str,
    payload: Mapping[str, object],
) -> Dict[str, object]:
    """Append a standalone attempted-trial row, including no-subject rows."""

    attempt_type = _safe_text(payload.get("attempt_type") or "").lower()
    if attempt_type not in ATTEMPT_TYPES:
        raise ValueError("attempt_type must be subject or no_subject")
    condition_id = _safe_text(payload.get("condition_id")).lower()
    if condition_id not in CONFIRMATORY_CONDITION_IDS:
        raise ValueError("condition_id must be one of the six confirmatory conditions")
    participant_id = payload.get("participant_id")
    logical_trial_id = payload.get("logical_trial_id")
    if attempt_type == "subject":
        if not _PARTICIPANT_RE.fullmatch(_safe_text(participant_id).upper()):
            raise ValueError("subject attempts require participant_id")
        logical_trial_id = logical_trial_id or canonical_logical_trial_id(
            participant_id, condition_id, payload.get("trial_number")
        )
    else:
        participant_id = None
        logical_trial_id = None
    status = _safe_text(payload.get("status") or "allocated").lower()
    attempt_id = _safe_text(payload.get("attempt_id"), limit=64) or allocate_attempt_id()
    path = protocol_attempts_path(sessions_root)
    existing = read_json_if_exists(str(path))
    if not isinstance(existing, dict):
        existing = {
            "schema_version": PROTOCOL_ATTEMPTS_SCHEMA_VERSION,
            "attempts": [],
        }
    attempts = existing.get("attempts")
    if not isinstance(attempts, list):
        attempts = []
    if any(isinstance(row, dict) and row.get("attempt_id") == attempt_id for row in attempts):
        raise ValueError("attempt_id already exists")
    record = _base_attempt_record(
        attempt_id=attempt_id,
        attempt_type=attempt_type,
        status=status,
        participant_id=participant_id,
        logical_trial_id=logical_trial_id,
        trial_id=payload.get("trial_id"),
        condition_id=condition_id,
        trial_number=payload.get("trial_number"),
        product_version=payload.get("product_version"),
        protocol_id=payload.get("protocol_id"),
    )
    attempts.append(record)
    existing["schema_version"] = PROTOCOL_ATTEMPTS_SCHEMA_VERSION
    existing["attempts"] = attempts
    existing["updated_at"] = _iso_now()
    atomic_write_json(existing, str(path))
    return record


def _iter_attempt_records(sessions_root: str) -> Iterable[Dict[str, object]]:
    root = Path(sessions_root).resolve()
    aggregate = read_json_if_exists(str(protocol_attempts_path(str(root))))
    if isinstance(aggregate, dict) and isinstance(aggregate.get("attempts"), list):
        for item in aggregate["attempts"]:
            if isinstance(item, dict):
                yield item
    if not root.exists():
        return
    for path in sorted(root.glob("*/protocol_attempt.json")):
        record = read_json_if_exists(str(path))
        if isinstance(record, dict):
            yield record


def completion_matrix(sessions_root: str) -> Dict[str, object]:
    """Build participant × condition × repetition completion evidence."""

    registry = read_json_if_exists(str(Path(sessions_root).resolve().parent / "participant_profiles.json"))
    profiles = registry.get("profiles", {}) if isinstance(registry, dict) else {}
    participants = {
        str(key): value
        for key, value in profiles.items()
        if isinstance(value, dict) and _PARTICIPANT_RE.fullmatch(str(key))
    }
    rows = list(_iter_attempt_records(sessions_root))
    by_key: Dict[str, Dict[str, object]] = {}
    no_subject: list[Dict[str, object]] = []
    for row in rows:
        if row.get("attempt_type") == "no_subject":
            no_subject.append(row)
            continue
        participant = _safe_text(row.get("participant_id")).upper()
        condition = _safe_text(row.get("condition_id")).lower()
        trial = row.get("trial_number")
        if not participant or not condition or trial in (None, ""):
            continue
        key = f"{participant}|{condition}|{int(trial)}"
        previous = by_key.get(key)
        if previous is None or str(row.get("updated_at", "")) >= str(previous.get("updated_at", "")):
            by_key[key] = row
        participants.setdefault(participant, {"participant_id": participant, "status": "unknown"})
    matrix: Dict[str, object] = {}
    for participant, profile in sorted(participants.items()):
        cells: Dict[str, object] = {}
        completed = 0
        for condition in CONFIRMATORY_CONDITION_IDS:
            for trial in TRIAL_NUMBERS:
                key = f"{participant}|{condition}|{trial}"
                row = by_key.get(key)
                status = str(row.get("status") if row else "missing")
                if status == "completed":
                    completed += 1
                cells[f"{condition}:t{trial}"] = {
                    "status": status,
                    "attempt_id": row.get("attempt_id") if row else None,
                    "attempt_type": row.get("attempt_type") if row else "subject",
                }
        matrix[participant] = {
            "participant_id": participant,
            "status": profile.get("status", "unknown") if isinstance(profile, dict) else "unknown",
            "completed_trials": completed,
            "expected_trials": len(CONFIRMATORY_CONDITION_IDS) * len(TRIAL_NUMBERS),
            "protocol_complete": completed == len(CONFIRMATORY_CONDITION_IDS) * len(TRIAL_NUMBERS),
            "cells": cells,
        }
    return {
        "schema_version": PROTOCOL_ATTEMPTS_SCHEMA_VERSION,
        "conditions": list(CONFIRMATORY_CONDITION_IDS),
        "trials": list(TRIAL_NUMBERS),
        "participants": matrix,
        "participant_count": len(matrix),
        "protocol_complete_participant_count": sum(
            1 for row in matrix.values() if row.get("protocol_complete")
        ),
        "no_subject_attempt_count": len(no_subject),
        "no_subject_expected": 72,
        "attempt_count": len(rows),
    }


__all__ = [
    "ATTEMPT_LEDGER_SCHEMA_VERSION",
    "ATTEMPT_STATUSES",
    "ATTEMPT_TYPES",
    "CONFIRMATORY_CONDITION_IDS",
    "PROTOCOL_ATTEMPTS_SCHEMA_VERSION",
    "TRIAL_NUMBERS",
    "allocate_attempt_id",
    "append_session_attempt_event",
    "canonical_logical_trial_id",
    "completion_matrix",
    "initialize_session_attempt",
    "ledger_path",
    "protocol_attempts_path",
    "register_protocol_attempt",
]
