"""Pseudonymous participant registry and release-bound study-session contract."""

from __future__ import annotations

import os
import re
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

from rvt_trainer.api.common import atomic_write_json, read_json_if_exists

PARTICIPANT_REGISTRY_SCHEMA_VERSION = "rvt-participant-profiles-v16.5.1"
STUDY_SESSION_SCHEMA_VERSION = "rvt-study-session-v16.5.1"
PARTICIPANT_ID_PATTERN = re.compile(r"^P-[0-9]{3}$")
PARTICIPANT_STATUSES = frozenset({"active", "completed", "withdrawn"})
STUDY_CLASSIFICATIONS = frozenset({"confirmatory", "exploratory"})
CONFIRMATORY_DISTANCES_M = (0.6, 0.8, 1.0)
STUDY_DISTANCE_RANGE_M = (0.5, 1.0)
CONFIRMATORY_BARRIERS = frozenset({"none", "cardboard"})
IMMUTABLE_STUDY_FIELDS = (
    "participant_id",
    "trial_id",
    "condition_id",
    "distance_m",
    "barrier_type",
    "trial_number",
    "planned_duration_s",
    "study_classification",
)
_FORBIDDEN_PROFILE_FIELDS = frozenset(
    {
        "name",
        "full_name",
        "display_name",
        "first_name",
        "last_name",
        "email",
        "phone",
        "address",
    }
)
_SOURCE_COMMIT: Optional[str] = None
_PARTICIPANT_REGISTRY_LOCK = threading.RLock()


class StudyContractError(ValueError):
    """A study request or participant profile violates the protocol contract."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def _iso_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def participant_registry_path(sessions_root: str) -> Path:
    return Path(sessions_root).resolve().parent / "participant_profiles.json"


def load_participant_registry(sessions_root: str) -> Dict[str, object]:
    path = participant_registry_path(sessions_root)
    data = read_json_if_exists(str(path))
    if not isinstance(data, dict):
        return {
            "schema_version": PARTICIPANT_REGISTRY_SCHEMA_VERSION,
            "profiles": {},
        }
    profiles = data.get("profiles")
    if not isinstance(profiles, dict):
        profiles = {}
    return {
        "schema_version": PARTICIPANT_REGISTRY_SCHEMA_VERSION,
        "profiles": {
            str(key): value
            for key, value in profiles.items()
            if isinstance(value, dict) and PARTICIPANT_ID_PATTERN.fullmatch(str(key))
        },
    }


def _next_participant_id(profiles: Mapping[str, object]) -> str:
    used = {
        int(match.group(1))
        for key in profiles
        if (match := re.fullmatch(r"P-([0-9]{3})", str(key)))
    }
    for number in range(1, 1000):
        if number not in used:
            return f"P-{number:03d}"
    raise StudyContractError(
        "PARTICIPANT_CAPACITY_REACHED",
        "participant code capacity P-001 through P-999 is exhausted",
    )


def create_participant_profile(
    sessions_root: str,
    payload: Mapping[str, object],
) -> Dict[str, object]:
    forbidden = sorted(_FORBIDDEN_PROFILE_FIELDS.intersection(payload))
    if forbidden:
        raise StudyContractError(
            "PII_FIELD_FORBIDDEN",
            "participant profiles are pseudonymous; forbidden field(s): "
            + ", ".join(forbidden),
        )
    with _PARTICIPANT_REGISTRY_LOCK:
        registry = load_participant_registry(sessions_root)
        profiles = dict(registry["profiles"])
        participant_id = str(payload.get("participant_id") or "").strip().upper()
        if not participant_id:
            participant_id = _next_participant_id(profiles)
        if not PARTICIPANT_ID_PATTERN.fullmatch(participant_id):
            raise StudyContractError(
                "INVALID_PARTICIPANT_ID",
                "participant_id must use the coded form P-001 through P-999",
            )
        if participant_id in profiles:
            raise StudyContractError(
                "PARTICIPANT_EXISTS",
                f"participant profile {participant_id} already exists",
            )
        status = str(payload.get("status") or "active").strip().lower()
        if status not in PARTICIPANT_STATUSES:
            raise StudyContractError(
                "INVALID_PARTICIPANT_STATUS",
                "status must be active, completed, or withdrawn",
            )
        now = _iso_now()
        profile = {
            "participant_id": participant_id,
            "display_code": participant_id,
            "profile_code": participant_id,
            "status": status,
            "created_at": now,
            "updated_at": now,
        }
        profiles[participant_id] = profile
        registry["profiles"] = profiles
        atomic_write_json(registry, str(participant_registry_path(sessions_root)))
        return profile


def update_participant_status(
    sessions_root: str,
    participant_id: str,
    status: object,
) -> Dict[str, object]:
    participant_id = str(participant_id).strip().upper()
    next_status = str(status or "").strip().lower()
    if next_status not in PARTICIPANT_STATUSES:
        raise StudyContractError(
            "INVALID_PARTICIPANT_STATUS",
            "status must be active, completed, or withdrawn",
        )
    with _PARTICIPANT_REGISTRY_LOCK:
        registry = load_participant_registry(sessions_root)
        profiles = dict(registry["profiles"])
        existing = profiles.get(participant_id)
        if not isinstance(existing, dict):
            raise StudyContractError(
                "PARTICIPANT_NOT_FOUND",
                f"participant profile {participant_id} was not found",
            )
        profile = dict(existing)
        profile["status"] = next_status
        profile["updated_at"] = _iso_now()
        profiles[participant_id] = profile
        registry["profiles"] = profiles
        atomic_write_json(registry, str(participant_registry_path(sessions_root)))
        return profile


def canonical_condition_id(distance_m: float, barrier_type: str) -> str:
    return f"d{int(round(distance_m * 100)):03d}_{barrier_type}"


def validate_study_assignment(
    payload: Mapping[str, object],
    *,
    sessions_root: Optional[str] = None,
) -> Dict[str, object]:
    """Validate a study assignment.

    Requests with no study fields remain valid operational captures for API
    compatibility, but are explicitly marked ``legacy_unassigned`` and cannot
    enter confirmatory evaluation.
    """

    requested_classification = str(
        payload.get("study_classification") or payload.get("study_mode") or ""
    ).strip().lower()
    has_study_fields = any(
        payload.get(key) not in (None, "")
        for key in IMMUTABLE_STUDY_FIELDS
        if key != "study_classification"
    )
    if not requested_classification and not has_study_fields:
        return {
            "schema_version": STUDY_SESSION_SCHEMA_VERSION,
            "study_classification": "operational",
            "provenance_state": "legacy_unassigned",
            "confirmatory_eligible": False,
        }
    if requested_classification not in STUDY_CLASSIFICATIONS:
        raise StudyContractError(
            "STUDY_CLASSIFICATION_REQUIRED",
            "study_classification must be confirmatory or exploratory",
        )

    participant_id = str(payload.get("participant_id") or "").strip().upper()
    if not PARTICIPANT_ID_PATTERN.fullmatch(participant_id):
        raise StudyContractError(
            "PARTICIPANT_REQUIRED",
            "study sessions require a coded participant_id such as P-001",
        )
    if sessions_root is not None:
        profile = load_participant_registry(sessions_root)["profiles"].get(
            participant_id
        )
        if not isinstance(profile, dict):
            raise StudyContractError(
                "PARTICIPANT_NOT_FOUND",
                f"participant profile {participant_id} was not found",
            )
        if profile.get("status") != "active":
            raise StudyContractError(
                "PARTICIPANT_NOT_ACTIVE",
                f"participant profile {participant_id} is {profile.get('status')}",
            )

    trial_id = str(payload.get("trial_id") or "").strip()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}", trial_id):
        raise StudyContractError(
            "INVALID_TRIAL_ID",
            "trial_id is required and may contain letters, digits, dot, dash, and underscore",
        )
    barrier_type = str(payload.get("barrier_type") or "").strip().lower()
    if barrier_type not in CONFIRMATORY_BARRIERS:
        raise StudyContractError(
            "INVALID_BARRIER_TYPE",
            "barrier_type must be none or cardboard",
        )
    try:
        distance_m = round(float(payload.get("distance_m")), 3)
        trial_number = int(payload.get("trial_number"))
        planned_duration_s = float(payload.get("planned_duration_s"))
    except (TypeError, ValueError):
        raise StudyContractError(
            "INVALID_STUDY_NUMERIC_FIELD",
            "distance_m, trial_number, and planned_duration_s must be numeric",
        ) from None
    if not STUDY_DISTANCE_RANGE_M[0] <= distance_m <= STUDY_DISTANCE_RANGE_M[1]:
        raise StudyContractError(
            "DISTANCE_OUT_OF_RANGE",
            "distance_m must be between 0.5 and 1.0 metres",
        )
    if trial_number < 1:
        raise StudyContractError(
            "INVALID_TRIAL_NUMBER",
            "trial_number must be at least 1",
        )
    if planned_duration_s <= 0:
        raise StudyContractError(
            "INVALID_PLANNED_DURATION",
            "planned_duration_s must be greater than zero",
        )
    if payload.get("duration_s") not in (None, ""):
        try:
            actual_duration_s = float(payload.get("duration_s"))
        except (TypeError, ValueError):
            raise StudyContractError(
                "INVALID_CAPTURE_DURATION",
                "duration_s must be numeric when supplied",
            ) from None
        if abs(actual_duration_s - planned_duration_s) > 1e-9:
            raise StudyContractError(
                "CAPTURE_DURATION_MISMATCH",
                "duration_s must match planned_duration_s for a study session",
            )

    expected_condition = canonical_condition_id(distance_m, barrier_type)
    condition_id = str(payload.get("condition_id") or "").strip().lower()
    if condition_id != expected_condition:
        raise StudyContractError(
            "CONDITION_MISMATCH",
            f"condition_id must be {expected_condition} for this distance and barrier",
        )

    if requested_classification == "confirmatory":
        if not any(abs(distance_m - allowed) < 1e-9 for allowed in CONFIRMATORY_DISTANCES_M):
            raise StudyContractError(
                "INVALID_CONFIRMATORY_DISTANCE",
                "confirmatory distance_m must be 0.6, 0.8, or 1.0 metres",
            )
        if trial_number not in {1, 2, 3}:
            raise StudyContractError(
                "INVALID_CONFIRMATORY_TRIAL",
                "confirmatory trial_number must be 1, 2, or 3",
            )
        if abs(planned_duration_s - 150.0) > 1e-9:
            raise StudyContractError(
                "INVALID_CONFIRMATORY_DURATION",
                "confirmatory planned_duration_s must be exactly 150 seconds",
            )

    return {
        "schema_version": STUDY_SESSION_SCHEMA_VERSION,
        "participant_id": participant_id,
        "trial_id": trial_id,
        "condition_id": condition_id,
        "distance_m": distance_m,
        "barrier_type": barrier_type,
        "trial_number": trial_number,
        "planned_duration_s": planned_duration_s,
        "study_classification": requested_classification,
        "provenance_state": "assigned",
        "confirmatory_eligible": requested_classification == "confirmatory",
    }


def merge_immutable_study_assignment(
    existing: Mapping[str, object],
    requested: Mapping[str, object],
) -> Dict[str, object]:
    merged = dict(requested)
    for field in IMMUTABLE_STUDY_FIELDS:
        old_value = existing.get(field)
        new_value = requested.get(field)
        if old_value not in (None, "") and new_value not in (None, "") and old_value != new_value:
            raise StudyContractError(
                "SESSION_ASSIGNMENT_IMMUTABLE",
                f"{field} cannot be reassigned after session start",
            )
        if old_value not in (None, ""):
            merged[field] = old_value
        elif new_value not in (None, ""):
            merged[field] = new_value
    for field in (
        "study_session_schema_version",
        "provenance_state",
        "confirmatory_eligible",
    ):
        if existing.get(field) not in (None, ""):
            merged[field] = existing[field]
        elif requested.get(field) not in (None, ""):
            merged[field] = requested[field]
    return merged


def source_commit() -> Optional[str]:
    global _SOURCE_COMMIT
    if _SOURCE_COMMIT is not None:
        return _SOURCE_COMMIT or None
    for key in ("RVT_SOURCE_COMMIT", "GITHUB_SHA"):
        value = str(os.environ.get(key) or "").strip()
        if value:
            _SOURCE_COMMIT = value
            return value
    # Resolve ordinary repositories and linked worktrees without launching a
    # subprocess. Session-start tests and packaged runtimes can replace or omit
    # process launchers; provenance lookup must never consume the one capture
    # process launch or make capture depend on ``git`` being installed.
    try:
        repo_root = Path(__file__).resolve().parents[2]
        git_path = repo_root / ".git"
        if git_path.is_file():
            marker = git_path.read_text(encoding="utf-8").strip()
            if marker.lower().startswith("gitdir:"):
                git_path = (repo_root / marker.split(":", 1)[1].strip()).resolve()
        head = (git_path / "HEAD").read_text(encoding="utf-8").strip()
        value = ""
        if head.startswith("ref:"):
            ref_name = head.split(":", 1)[1].strip()
            git_roots = [git_path]
            common_dir = git_path / "commondir"
            if common_dir.exists():
                git_roots.append(
                    (git_path / common_dir.read_text(encoding="utf-8").strip()).resolve()
                )
            for git_root in git_roots:
                ref_path = git_root / ref_name
                if ref_path.exists():
                    value = ref_path.read_text(encoding="utf-8").strip()
                    break
                packed = git_root / "packed-refs"
                if packed.exists():
                    for line in packed.read_text(
                        encoding="utf-8",
                        errors="ignore",
                    ).splitlines():
                        if line and not line.startswith(("#", "^")):
                            candidate, _, name = line.partition(" ")
                            if name.strip() == ref_name:
                                value = candidate.strip()
                                break
                if value:
                    break
        else:
            value = head
        if value:
            _SOURCE_COMMIT = value
            return value
    except Exception:
        pass
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(Path(__file__).resolve().parents[2]),
            capture_output=True,
            text=True,
            check=False,
            timeout=2,
        )
        value = result.stdout.strip() if result.returncode == 0 else ""
    # Provenance enrichment must never prevent capture. This also keeps the
    # session supervisor testable when its process launcher is replaced.
    except Exception:
        value = ""
    _SOURCE_COMMIT = value
    return value or None


def release_provenance(
    *,
    product_version: str,
    trainer_version: str,
    dashboard_version: str,
    firmware_expected: str,
    firmware_observed: object = None,
    serial_protocol: str = "v15.2",
    serial_width_expected: int = 222,
    serial_width_observed: object = None,
    model_family: object = None,
    model_bundle: object = None,
) -> Dict[str, object]:
    return {
        "product_version": product_version,
        "trainer_version": trainer_version,
        "dashboard_version": dashboard_version,
        "firmware_expected": firmware_expected,
        "firmware_observed": firmware_observed,
        "serial_protocol": serial_protocol,
        "serial_width_expected": int(serial_width_expected),
        "serial_width_observed": serial_width_observed,
        "source_commit": source_commit(),
        "model_family": model_family or "none",
        "model_bundle": model_bundle,
    }


__all__ = [
    "CONFIRMATORY_BARRIERS",
    "CONFIRMATORY_DISTANCES_M",
    "IMMUTABLE_STUDY_FIELDS",
    "PARTICIPANT_REGISTRY_SCHEMA_VERSION",
    "STUDY_SESSION_SCHEMA_VERSION",
    "StudyContractError",
    "canonical_condition_id",
    "create_participant_profile",
    "load_participant_registry",
    "merge_immutable_study_assignment",
    "participant_registry_path",
    "release_provenance",
    "source_commit",
    "update_participant_status",
    "validate_study_assignment",
]
