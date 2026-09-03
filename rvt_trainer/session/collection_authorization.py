"""Fail-closed separation between protocol freeze and collection authority."""

from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Mapping

from rvt_trainer.api.common import read_json_if_exists


COLLECTION_AUTHORIZATION_SCHEMA_VERSION = "rvt-collection-authorization-v1"
COLLECTION_AUTHORIZATION_PATH = "collection_authorization.json"
REQUIRED_APPROVAL_ROLES = frozenset(
    {"research_lead", "quality_manager", "REC_or_ethics_authority", "privacy_reviewer"}
)


class CollectionAuthorizationError(ValueError):
    """The requested study action is not covered by a valid authorization."""

    code = "STUDY_COLLECTION_NOT_AUTHORIZED"


def _canonical_sha256(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        dict(value), ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _controlled_plan() -> Dict[str, Any]:
    path = Path(__file__).resolve().parents[2] / "quality" / "statistical-analysis-plan.json"
    value = read_json_if_exists(str(path))
    return dict(value) if isinstance(value, dict) else {}


def _parse_utc(value: object) -> datetime | None:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        return None
    return parsed.astimezone(timezone.utc)


def collection_authorization_status(
    sessions_root: str,
    *,
    now: datetime | None = None,
) -> Dict[str, Any]:
    """Return objective blockers; absence or ambiguity always means blocked."""

    root = Path(sessions_root).resolve()
    authorization = read_json_if_exists(str(root / COLLECTION_AUTHORIZATION_PATH))
    protocol = read_json_if_exists(str(root / "study_protocol.json"))
    plan = _controlled_plan()
    blockers: list[str] = []
    if not isinstance(authorization, dict):
        authorization = {}
        blockers.append("authorization_record_missing_or_invalid")
    if not isinstance(protocol, dict) or protocol.get("state") != "locked":
        blockers.append("protocol_configuration_not_locked")
        protocol = protocol if isinstance(protocol, dict) else {}
    if plan.get("status") != "approved":
        blockers.append("statistical_analysis_plan_not_approved")
    if authorization.get("schema_version") != COLLECTION_AUTHORIZATION_SCHEMA_VERSION:
        blockers.append("authorization_schema_invalid")
    if authorization.get("status") != "authorized":
        blockers.append("authorization_status_not_authorized")

    identities = {
        "analysis_plan_id": plan.get("plan_id"),
        "analysis_plan_sha256": _canonical_sha256(plan) if plan else None,
        "study_protocol_id": protocol.get("protocol_id"),
        "study_protocol_sha256": _canonical_sha256(protocol) if protocol else None,
        "study_session_schema_version": "rvt-study-session-v16.5.9",
    }
    for key, expected in identities.items():
        if not expected or authorization.get(key) != expected:
            blockers.append(f"{key}_mismatch")

    approvals_value = authorization.get("approvals")
    approvals = approvals_value if isinstance(approvals_value, list) else []
    approved_roles = {
        str(item.get("role"))
        for item in approvals
        if isinstance(item, dict)
        and item.get("status") == "approved"
        and str(item.get("evidence_ref") or "").strip()
        and _parse_utc(item.get("approved_at")) is not None
    }
    if not REQUIRED_APPROVAL_ROLES.issubset(approved_roles):
        blockers.append("required_approvals_incomplete")

    current = (now or datetime.now(timezone.utc)).astimezone(timezone.utc)
    valid_from = _parse_utc(authorization.get("valid_from"))
    valid_until = _parse_utc(authorization.get("valid_until"))
    if valid_from is None or valid_until is None or not (valid_from <= current <= valid_until):
        blockers.append("authorization_outside_validity_window")
    if not str(authorization.get("withdrawal_policy_evidence_ref") or "").strip():
        blockers.append("withdrawal_policy_not_approved")
    latency_hash = str(authorization.get("latency_characterization_sha256") or "").lower()
    if len(latency_hash) != 64 or any(char not in "0123456789abcdef" for char in latency_hash):
        blockers.append("latency_characterization_missing_or_invalid")

    registry = read_json_if_exists(str(root.parent / "participant_profiles.json"))
    profiles_value = registry.get("profiles") if isinstance(registry, dict) else {}
    profiles = profiles_value if isinstance(profiles_value, dict) else {}
    unresolved_withdrawals = [
        participant_id
        for participant_id, profile in profiles.items()
        if isinstance(profile, dict)
        and profile.get("status") == "withdrawn"
        and (
            not isinstance(profile.get("withdrawal_reconciliation"), dict)
            or profile["withdrawal_reconciliation"].get("disposition")
            == "pending_rec_legal_review"
        )
    ]
    if unresolved_withdrawals:
        blockers.append("unresolved_participant_withdrawal")

    unique_blockers = list(dict.fromkeys(blockers))
    return {
        "ok": True,
        "schema_version": "rvt-collection-readiness-v1",
        "authorized": not unique_blockers,
        "protocol_state": protocol.get("state") or "draft",
        "plan_status": plan.get("status") or "unavailable",
        "authorization_record_present": bool(authorization),
        "blockers": unique_blockers,
        "identity": identities,
        "unresolved_withdrawal_count": len(unresolved_withdrawals),
    }


def require_collection_authorization(sessions_root: str) -> Dict[str, Any]:
    status = collection_authorization_status(sessions_root)
    if not status["authorized"]:
        raise CollectionAuthorizationError(
            "Confirmatory recruitment, capture, no-subject evidence, and analysis remain blocked "
            "until the separate collection authorization is approved and identity-matched."
        )
    return status


__all__ = [
    "COLLECTION_AUTHORIZATION_PATH",
    "COLLECTION_AUTHORIZATION_SCHEMA_VERSION",
    "CollectionAuthorizationError",
    "collection_authorization_status",
    "require_collection_authorization",
]
