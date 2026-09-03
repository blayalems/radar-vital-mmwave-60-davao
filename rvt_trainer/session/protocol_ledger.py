"""Append-only protocol-attempt evidence for study-ready captures.

The session manifest identifies the capture, while this module records the
protocol state transitions and the attempts that were *actually tried*.  The
distinction is important for coverage: an aborted, invalid, or no-subject
attempt must remain in the denominator instead of disappearing with the
prediction file.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import secrets
import threading
import time
from datetime import datetime, timezone
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
NO_SUBJECT_MIN_DURATION_S = 150.0
NO_SUBJECT_TRIAL_COUNT = 72
_PARTICIPANT_RE = re.compile(r"^P-[0-9]{3}$")
_CONDITION_RE = re.compile(r"^d[0-9]{3}_(?:none|cardboard)$")
_FIRMWARE_VERSION_RE = re.compile(
    r"^v[0-9]+\.[0-9]+\.[0-9]+(?:[-+][A-Za-z0-9][A-Za-z0-9.-]*)?$"
)
_LEDGER_LOCK = threading.RLock()


def _iso_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _safe_text(value: object, *, default: str = "", limit: int = 160) -> str:
    text = str(value or "").replace("\r", " ").replace("\n", " ").strip()
    return text[:limit]


def frozen_configuration_hash(configuration: Mapping[str, object]) -> str:
    """Return the stable SHA-256 identity for one frozen no-subject setup."""

    if not isinstance(configuration, Mapping) or not configuration:
        raise ValueError("frozen configuration must be a non-empty JSON object")
    try:
        canonical = json.dumps(
            dict(configuration),
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except (TypeError, ValueError) as exc:
        raise ValueError("frozen configuration must contain JSON values") from exc
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def validate_frozen_no_subject_configuration(
    configuration: Mapping[str, object],
) -> Dict[str, object]:
    """Validate the minimum frozen identities required by Objective 3."""

    if not isinstance(configuration, Mapping) or not configuration:
        raise ValueError("frozen configuration must be a non-empty JSON object")
    firmware = _safe_text(configuration.get("firmware"), limit=100)
    if not _FIRMWARE_VERSION_RE.fullmatch(firmware):
        raise ValueError("frozen configuration firmware must use vMAJOR.MINOR.PATCH form")
    artifact_rules = configuration.get("artifact_rules")
    if not isinstance(artifact_rules, str) or not artifact_rules.strip():
        raise ValueError("frozen configuration artifact_rules identity is required")
    alert_threshold = _finite_number(configuration.get("alert_threshold"))
    if alert_threshold is None:
        raise ValueError("frozen configuration alert_threshold must be finite")
    # Confirm JSON canonicalization while validation errors can still be
    # attributed to the protocol lock rather than a later report request.
    frozen_configuration_hash(configuration)
    return {
        "firmware": firmware,
        "artifact_rules": artifact_rules.strip(),
        "alert_threshold": alert_threshold,
    }


def _finite_number(value: object) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _parse_timestamp(value: object) -> Optional[datetime]:
    text = _safe_text(value, limit=64)
    if not text:
        return None
    if text.endswith("Z"):
        text = f"{text[:-1]}+00:00"
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def no_subject_capture_interval(
    row: Mapping[str, object],
    sessions_root: str,
) -> Optional[tuple[datetime, datetime]]:
    """Return one safely resolved no-subject capture interval."""

    session_id = _safe_text(row.get("session_id"), limit=120)
    if not session_id or "/" in session_id or "\\" in session_id or session_id in {".", ".."}:
        return None
    root = Path(sessions_root).resolve()
    session_root = (root / session_id).resolve()
    if session_root.parent != root or session_root.name != session_id:
        return None
    manifest = read_json_if_exists(str(session_root / "session_manifest.json"))
    if not isinstance(manifest, dict):
        return None
    provenance = manifest.get("capture_provenance")
    if not isinstance(provenance, dict):
        return None
    captured_at = _parse_timestamp(provenance.get("captured_at"))
    ended_at = _parse_timestamp(manifest.get("ended_at"))
    if captured_at is None or ended_at is None or ended_at <= captured_at:
        return None
    return captured_at, ended_at


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


def _register_protocol_attempt_unlocked(
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
    if attempt_type == "no_subject":
        # Preserve the evidence fields needed to distinguish an attempted
        # click from a qualified 150-second control capture.  Missing fields
        # intentionally leave the row in the attempted denominator only.
        record["session_id"] = _safe_text(payload.get("session_id"), limit=120) or None
        record["duration_s"] = payload.get("duration_s")
        record["frozen_configuration_hash"] = _safe_text(
            payload.get("frozen_configuration_hash"), limit=128
        ) or None
        record["false_alarm_count"] = payload.get("false_alarm_count")
    attempts.append(record)
    existing["schema_version"] = PROTOCOL_ATTEMPTS_SCHEMA_VERSION
    existing["attempts"] = attempts
    existing["updated_at"] = _iso_now()
    atomic_write_json(existing, str(path))
    return record


def register_protocol_attempt(
    sessions_root: str,
    payload: Mapping[str, object],
) -> Dict[str, object]:
    """Append one standalone attempt without losing a concurrent API write."""

    with _LEDGER_LOCK:
        return _register_protocol_attempt_unlocked(sessions_root, payload)


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


def list_protocol_attempts(
    sessions_root: str,
    *,
    attempt_type: object = None,
    participant_id: object = None,
    status: object = None,
) -> list[Dict[str, object]]:
    """Return append-only attempt evidence with optional exact filters."""

    wanted_type = _safe_text(attempt_type).lower()
    wanted_participant = _safe_text(participant_id).upper()
    wanted_status = _safe_text(status).lower()
    with _LEDGER_LOCK:
        rows = [dict(row) for row in _iter_attempt_records(sessions_root)]
    if wanted_type:
        rows = [row for row in rows if str(row.get("attempt_type") or "") == wanted_type]
    if wanted_participant:
        rows = [
            row
            for row in rows
            if str(row.get("participant_id") or "").upper() == wanted_participant
        ]
    if wanted_status:
        rows = [row for row in rows if str(row.get("status") or "") == wanted_status]
    rows.sort(
        key=lambda row: (
            str(row.get("created_at") or ""),
            str(row.get("attempt_id") or ""),
        )
    )
    return rows


def _qualified_no_subject(row: Mapping[str, object], sessions_root: str) -> bool:
    # A confirmatory control trial must be a completed capture, not merely any
    # terminal click (for example, an aborted or failed-start attempt).
    if (
        row.get("attempt_type") != "no_subject"
        or row.get("status") != "completed"
        or row.get("terminal") is not True
    ):
        return False
    events = row.get("events")
    if not isinstance(events, list) or not any(
        isinstance(event, dict)
        and event.get("status") == "completed"
        and bool(_safe_text(event.get("event_id"), limit=64))
        and bool(_safe_text(event.get("at"), limit=64))
        for event in events
    ):
        return False

    duration = _finite_number(row.get("duration_s"))
    false_alarm_count = row.get("false_alarm_count")
    if (
        duration is None
        or duration < NO_SUBJECT_MIN_DURATION_S
        or type(false_alarm_count) is not int
        or false_alarm_count not in {0, 1}
    ):
        return False
    frozen_hash = str(row.get("frozen_configuration_hash") or "").strip().lower()
    if not re.fullmatch(r"[0-9a-f]{64}", frozen_hash):
        return False

    # The hash must resolve to the single configuration frozen in the locked
    # protocol.  A syntactically valid, caller-invented hash is not evidence.
    root = Path(sessions_root).resolve()
    protocol = read_json_if_exists(str(root / "study_protocol.json"))
    no_subject = protocol.get("no_subject") if isinstance(protocol, dict) else None
    configuration = (
        no_subject.get("frozen_configuration")
        if isinstance(no_subject, dict)
        else None
    )
    if not isinstance(protocol, dict) or protocol.get("state") != "locked":
        return False
    if not isinstance(no_subject, dict):
        return False
    if (
        type(no_subject.get("trial_count")) is not int
        or no_subject.get("trial_count") != NO_SUBJECT_TRIAL_COUNT
    ):
        return False
    protocol_duration = _finite_number(no_subject.get("planned_duration_s"))
    if protocol_duration != NO_SUBJECT_MIN_DURATION_S or duration != protocol_duration:
        return False
    try:
        configuration_evidence = validate_frozen_no_subject_configuration(configuration)
        if frozen_configuration_hash(configuration) != frozen_hash:
            return False
    except ValueError:
        return False

    session_id = _safe_text(row.get("session_id"), limit=120)
    if not session_id or "/" in session_id or "\\" in session_id or session_id in {".", ".."}:
        return False
    session_root = (root / session_id).resolve()
    if session_root.parent != root or session_root.name != session_id:
        return False
    manifest = read_json_if_exists(str(session_root / "session_manifest.json"))
    if not isinstance(manifest, dict):
        return False

    attempt_id = _safe_text(row.get("attempt_id"), limit=64)
    if (
        not attempt_id
        or manifest.get("attempt_id") != attempt_id
        or manifest.get("attempt_type") != "no_subject"
        or manifest.get("participant_id") not in (None, "")
        or manifest.get("status") != "completed"
        or manifest.get("terminal") is not True
    ):
        return False
    manifest_session_id = _safe_text(manifest.get("session_id"), limit=120)
    if manifest_session_id and manifest_session_id != session_id:
        return False

    # Bind the separately appended trial classification back to the immutable
    # capture evidence.  Duration, configuration, and alert activation must
    # agree in both records; zero is accepted only as an explicitly recorded
    # integer classification, never as a missing/default value.
    manifest_duration = _finite_number(
        manifest.get("duration_s", manifest.get("planned_duration_s"))
    )
    manifest_false_alarm_count = manifest.get("false_alarm_count")
    if (
        manifest_duration != duration
        or str(manifest.get("frozen_configuration_hash") or "").strip().lower()
        != frozen_hash
        or type(manifest_false_alarm_count) is not int
        or manifest_false_alarm_count != false_alarm_count
    ):
        return False

    provenance = manifest.get("capture_provenance")
    if not isinstance(provenance, dict):
        return False
    if (
        provenance.get("source") != "session_start"
        or provenance.get("attempt_id") != attempt_id
    ):
        return False
    for field in (
        "product_version",
        "trainer_version",
        "dashboard_version",
        "firmware_expected",
        "firmware_observed",
        "source_commit",
    ):
        captured_value = _safe_text(provenance.get(field), limit=100)
        if not captured_value or _safe_text(manifest.get(field), limit=100) != captured_value:
            return False
    frozen_firmware = str(configuration_evidence["firmware"])
    if (
        _safe_text(manifest.get("firmware_expected"), limit=100) != frozen_firmware
        or _safe_text(manifest.get("firmware_observed"), limit=100) != frozen_firmware
    ):
        return False
    captured_at = _parse_timestamp(provenance.get("captured_at"))
    ended_at = _parse_timestamp(manifest.get("ended_at"))
    if captured_at is None or ended_at is None:
        return False
    if (ended_at - captured_at).total_seconds() + 1.0 < duration:
        return False
    return True


def qualified_no_subject_attempts(sessions_root: str) -> list[Dict[str, object]]:
    """Return only protocol- and capture-backed no-subject trial evidence."""

    rows = list_protocol_attempts(sessions_root, attempt_type="no_subject")
    return [row for row in rows if _qualified_no_subject(row, sessions_root)]


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
    qualified_no_subject = [
        row for row in no_subject if _qualified_no_subject(row, sessions_root)
    ]
    roster_profiles = [value for value in profiles.values() if isinstance(value, dict)]
    recruited_count = len(roster_profiles)
    active_count = sum(1 for profile in roster_profiles if profile.get("status") == "active")
    completed_roster_count = sum(1 for profile in roster_profiles if profile.get("status") == "completed")
    withdrawn_count = sum(1 for profile in roster_profiles if profile.get("status") == "withdrawn")
    unresolved_withdrawal_count = sum(
        1
        for profile in roster_profiles
        if profile.get("status") == "withdrawn"
        and (
            not isinstance(profile.get("withdrawal_reconciliation"), dict)
            or profile["withdrawal_reconciliation"].get("disposition")
            == "pending_rec_legal_review"
        )
    )
    protocol_complete_count = sum(
        1 for row in matrix.values() if row.get("protocol_complete")
    )
    eligible_complete_count = sum(
        1
        for row in matrix.values()
        if row.get("protocol_complete") and row.get("status") != "withdrawn"
    )
    return {
        "schema_version": PROTOCOL_ATTEMPTS_SCHEMA_VERSION,
        "conditions": list(CONFIRMATORY_CONDITION_IDS),
        "trials": list(TRIAL_NUMBERS),
        "participants": matrix,
        "participant_count": len(matrix),
        "target_recruited_participant_count": 40,
        "minimum_protocol_complete_participant_count": 38,
        "minimum_primary_independent_estimate_count": 19,
        "recruited_participant_count": recruited_count,
        "recruitment_target_gap": max(0, 40 - recruited_count),
        "active_participant_count": active_count,
        "completed_roster_participant_count": completed_roster_count,
        "withdrawn_participant_count": withdrawn_count,
        "unresolved_withdrawal_count": unresolved_withdrawal_count,
        "protocol_complete_participant_count": protocol_complete_count,
        "eligible_protocol_complete_participant_count": eligible_complete_count,
        "protocol_complete_target_gap": max(0, 38 - eligible_complete_count),
        "primary_independent_estimate_count": None,
        "primary_independent_estimate_status": "analysis_required",
        "no_subject_attempt_count": len(no_subject),
        "no_subject_qualified_count": len(qualified_no_subject),
        "no_subject_unqualified_count": len(no_subject) - len(qualified_no_subject),
        "no_subject_expected": NO_SUBJECT_TRIAL_COUNT,
        "no_subject_remaining": max(0, NO_SUBJECT_TRIAL_COUNT - len(qualified_no_subject)),
        "attempt_count": len(rows),
    }


__all__ = [
    "ATTEMPT_LEDGER_SCHEMA_VERSION",
    "ATTEMPT_STATUSES",
    "ATTEMPT_TYPES",
    "CONFIRMATORY_CONDITION_IDS",
    "PROTOCOL_ATTEMPTS_SCHEMA_VERSION",
    "TRIAL_NUMBERS",
    "NO_SUBJECT_MIN_DURATION_S",
    "NO_SUBJECT_TRIAL_COUNT",
    "allocate_attempt_id",
    "append_session_attempt_event",
    "canonical_logical_trial_id",
    "completion_matrix",
    "frozen_configuration_hash",
    "initialize_session_attempt",
    "ledger_path",
    "list_protocol_attempts",
    "no_subject_capture_interval",
    "protocol_attempts_path",
    "qualified_no_subject_attempts",
    "register_protocol_attempt",
    "validate_frozen_no_subject_configuration",
]
