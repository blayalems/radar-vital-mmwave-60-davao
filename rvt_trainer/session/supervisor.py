"""Detached capture-session lifecycle ownership.

The legacy trainer still supplies analysis, schema, and CLI functions from
``rvt_trainer.monolith``. Imports of that module are deliberately deferred
until method execution so this service remains importable on its own and the
monolith can re-export it without a circular import.
"""

from __future__ import annotations

import json
import os
import secrets
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional

from rvt_trainer.api.common import (
    atomic_write_json,
    read_json_if_exists,
    wait_for_process_exit,
)
from rvt_trainer.session.study_contract import (
    release_provenance,
    validate_study_assignment,
)
from rvt_trainer.session.collection_authorization import require_collection_authorization
from rvt_trainer.session.protocol_ledger import (
    allocate_attempt_id,
    append_session_attempt_event,
    initialize_session_attempt,
)
from rvt_trainer.session.logical_trial_reservation import (
    STARTING_RESERVATION_STALE_S,
    LogicalTrialReservationStore,
)
from rvt_trainer.session.start_idempotency import (
    StartIdempotencyError,
    StartIdempotencyStore,
    canonical_start_request_hash,
    validate_idempotency_key,
)

DEFAULT_RADAR_PORT = "COM10"
DEFAULT_BLE_ADDRESS = "10:22:33:9E:8F:63"
CONTROL_API_SCHEMA_VERSION = "rvt-control-api-v12.0"
LIVE_EVENT_SCHEMA_VERSION = "rvt-live-events-v12.0"


class SessionStartPreflightError(RuntimeError):
    def __init__(
        self,
        code: str,
        message: str,
        *,
        details: object = None,
        http_status: int = 500,
    ):
        super().__init__(message)
        self.code = code
        self.details = details
        self.http_status = int(http_status)
        self.failed_session_id: Optional[str] = None


def _legacy():
    from rvt_trainer import monolith

    return monolith


def _iso_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _lock_path(sessions_root: str) -> Path:
    return Path(sessions_root) / ".session.lock"


def _write_session_lock(
    sessions_root: str,
    session_dir: str,
    pid: Optional[int] = None,
) -> Dict[str, object]:
    root = Path(sessions_root)
    root.mkdir(parents=True, exist_ok=True)
    data = {
        "pid": int(pid or os.getpid()),
        "session_dir": str(session_dir),
        "started_at": _iso_now(),
    }
    path = _lock_path(str(root))
    try:
        with path.open("x", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
    except FileExistsError:
        if not _check_stale_session_lock(str(root)):
            raise
        with path.open("x", encoding="utf-8") as handle:
            json.dump(data, handle, indent=2)
    return data


def _read_session_lock(sessions_root: str) -> Optional[Dict[str, object]]:
    data = read_json_if_exists(str(_lock_path(sessions_root)))
    return data if isinstance(data, dict) else None


def _release_session_lock(sessions_root: str) -> None:
    try:
        _lock_path(sessions_root).unlink()
    except FileNotFoundError:
        pass


def _same_session_dir(left: object, right: object) -> bool:
    if not left or not right:
        return False
    try:
        return os.path.normcase(os.path.abspath(str(left))) == os.path.normcase(
            os.path.abspath(str(right))
        )
    except Exception:
        return False


def _session_marker_owned_by(
    data: object,
    pid: object,
    session_dir: object,
) -> bool:
    if not isinstance(data, dict) or not _same_session_dir(
        data.get("session_dir"),
        session_dir,
    ):
        return False
    try:
        return int(data.get("pid")) == int(pid)
    except Exception:
        return False


def _release_session_lock_if_owned(
    sessions_root: str,
    pid: object,
    session_dir: object,
) -> bool:
    path = _lock_path(sessions_root)
    if not path.exists():
        return True
    data = _read_session_lock(sessions_root)
    if not _session_marker_owned_by(data, pid, session_dir):
        return False
    try:
        path.unlink()
        return True
    except FileNotFoundError:
        return True


def _supervisor_stop_path(session_dir: object) -> Path:
    return Path(str(session_dir)) / ".supervisor-stop.json"


def _write_supervisor_stop_request(
    session_dir: str,
    *,
    reason: str,
    session_pid: int,
    auto_analyse: bool,
) -> Dict[str, object]:
    request = {
        "schema_version": CONTROL_API_SCHEMA_VERSION,
        "request_id": secrets.token_hex(12),
        "requested_at": _iso_now(),
        "reason": str(reason),
        "session_dir": os.path.abspath(session_dir),
        "session_pid": int(session_pid),
        "supervisor_pid": os.getpid(),
        "suppress_inline_auto_analyse": True,
        "parent_auto_analyse": bool(auto_analyse),
    }
    atomic_write_json(request, str(_supervisor_stop_path(session_dir)))
    return request


def _consume_supervisor_stop_request(
    session_dir: str,
) -> Optional[Dict[str, object]]:
    path = _supervisor_stop_path(session_dir)
    data = read_json_if_exists(str(path))
    if path.exists():
        try:
            path.unlink()
        except FileNotFoundError:
            pass
    return data if isinstance(data, dict) else None


def _clear_supervisor_stop_request(
    session_dir: str,
    request_id: object,
) -> bool:
    path = _supervisor_stop_path(session_dir)
    if not path.exists():
        return True
    data = read_json_if_exists(str(path))
    if not isinstance(data, dict) or str(data.get("request_id") or "") != str(
        request_id or ""
    ):
        return False
    try:
        path.unlink()
        return True
    except FileNotFoundError:
        return True


def _pid_alive(pid: object) -> bool:
    # Keep the historical monolith monkeypatch seam used by packaged and test
    # callers while ownership lives in this module.
    return bool(_legacy()._pid_alive(pid))


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


class SessionSupervisor:
    """Own one detached session child and its durable lifecycle markers."""

    def __init__(self, sessions_root: str = "sessions"):
        self.sessions_root = os.path.abspath(sessions_root)
        os.makedirs(self.sessions_root, exist_ok=True)
        self._lifecycle_lock = threading.RLock()
        self._closing = False
        self.proc = None
        self.session_dir = None
        self.started_at = None
        self.started_monotonic = None
        self.params: Dict[str, Any] = {}
        self._stop_grace_s = 10.0
        self._terminate_grace_s = 3.0
        self._kill_grace_s = 2.0
        self._start_idempotency = StartIdempotencyStore(self.sessions_root)
        self._logical_trial_reservations = LogicalTrialReservationStore(
            self.sessions_root
        )

    def _current_path(self) -> Path:
        return Path(self.sessions_root) / "current_session.json"

    def _write_current(self) -> None:
        atomic_write_json(
            {
                "session_id": Path(self.session_dir).name,
                "session_dir": self.session_dir,
                "pid": self.proc.pid,
                "started_at": self.started_at,
                "params": self.params,
            },
            str(self._current_path()),
        )

    def _clear_current(
        self,
        pid: object = None,
        session_dir: object = None,
    ) -> bool:
        path = self._current_path()
        if not path.exists():
            return True
        if pid is not None or session_dir is not None:
            data = self._read_current()
            if not _session_marker_owned_by(data, pid, session_dir):
                return False
        try:
            path.unlink()
            return True
        except FileNotFoundError:
            return True

    def _read_current(self) -> Optional[Dict[str, object]]:
        data = read_json_if_exists(str(self._current_path()))
        return data if isinstance(data, dict) else None

    def _logical_trial_reservation_is_active(
        self,
        record: Mapping[str, object],
    ) -> bool:
        """Return whether a durable logical-trial reservation still has an owner."""

        state = str(record.get("state") or "")
        session_dir = record.get("session_dir")
        if session_dir not in (None, ""):
            if (
                self.proc is not None
                and self.proc.poll() is None
                and _same_session_dir(self.session_dir, session_dir)
            ):
                return True
            manifest = read_json_if_exists(
                str(Path(str(session_dir)) / "session_manifest.json")
            )
            if isinstance(manifest, dict) and bool(manifest.get("terminal")):
                return False
            ledger = read_json_if_exists(
                str(Path(str(session_dir)) / "protocol_attempt.json")
            )
            if isinstance(ledger, dict) and bool(ledger.get("terminal")):
                return False
            current = self._read_current()
            if (
                isinstance(current, dict)
                and _same_session_dir(current.get("session_dir"), session_dir)
                and _pid_alive(current.get("pid"))
            ):
                return True
            lock = _read_session_lock(self.sessions_root)
            if (
                isinstance(lock, dict)
                and _same_session_dir(lock.get("session_dir"), session_dir)
                and _pid_alive(lock.get("pid"))
            ):
                return True
            return False

        if state != "starting":
            return False
        try:
            age_s = time.time() - float(record.get("created_at_epoch_s") or 0.0)
        except (TypeError, ValueError):
            return False
        return 0.0 <= age_s <= STARTING_RESERVATION_STALE_S

    def _lookup_logical_trial_reservation_locked(
        self,
        logical_trial_id: object,
    ) -> Optional[Dict[str, object]]:
        logical_id = str(logical_trial_id or "").strip()
        if not logical_id:
            return None
        record = self._logical_trial_reservations.lookup(logical_id)
        if not isinstance(record, dict):
            return None
        if self._logical_trial_reservation_is_active(record):
            return record
        self._logical_trial_reservations.release(
            logical_id,
            reservation_id=record.get("reservation_id"),
        )
        return None

    def _raise_if_logical_trial_reserved_locked(
        self,
        logical_trial_id: object,
    ) -> None:
        record = self._lookup_logical_trial_reservation_locked(logical_trial_id)
        if not isinstance(record, dict):
            return
        raise StartIdempotencyError(
            "LOGICAL_TRIAL_RESERVED",
            "the participant, condition, and trial number are already reserved by another session start",
            details={
                "logical_trial_id": record.get("logical_trial_id"),
                "reservation_id": record.get("reservation_id"),
                "session_id": record.get("session_id"),
                "idempotency_key": record.get("idempotency_key"),
                "state": record.get("state"),
            },
        )

    def _reserve_logical_trial_locked(
        self,
        logical_trial_id: object,
        request_hash: str,
        idempotency_key: object,
    ) -> Optional[Dict[str, object]]:
        logical_id = str(logical_trial_id or "").strip()
        if not logical_id:
            return None
        for _attempt in range(2):
            self._raise_if_logical_trial_reserved_locked(logical_id)
            try:
                return self._logical_trial_reservations.reserve(
                    logical_id,
                    request_hash,
                    idempotency_key=idempotency_key,
                )
            except FileExistsError:
                # Another process won the exclusive create after our lookup.
                continue
        self._raise_if_logical_trial_reserved_locked(logical_id)
        raise StartIdempotencyError(
            "LOGICAL_TRIAL_RESERVED",
            "the logical trial reservation changed concurrently; retry after checking the current session",
            details={"logical_trial_id": logical_id},
        )

    def lookup_logical_trial_start(
        self,
        logical_trial_id: object,
    ) -> None:
        """Expose the reservation guard to the HTTP preflight path."""

        with self._lifecycle_lock:
            self._poll()
            self._raise_if_logical_trial_reserved_locked(logical_trial_id)

    def _reset_runtime_state(self) -> None:
        self.proc = None
        self.session_dir = None
        self.started_at = None
        self.started_monotonic = None
        self.params = {}

    def close_start_gate(self) -> None:
        with self._lifecycle_lock:
            self._closing = True

    def _write_starting_live_payload(self, duration_s=None) -> None:
        session_id = Path(self.session_dir).name
        remaining = None
        if duration_s is not None:
            try:
                remaining = max(0.0, float(duration_s))
            except Exception:
                remaining = None
        payload = {
            "schema_version": LIVE_EVENT_SCHEMA_VERSION,
            "revision": int(time.time() * 1000),
            "session_id": session_id,
            "_supervisor_placeholder": True,
            "meta": {
                "status": "starting",
                "session_id": session_id,
                "elapsed_s": 0.0,
                "remaining_s": remaining,
                "version": _legacy().VERSION,
                "session_dir": os.path.abspath(self.session_dir),
                "note": "Session subprocess is starting.",
            },
            "radar": {"rows": 0},
            "ble": {"rows": 0, "raw_packets": 0},
            "thresholds": {},
            "faults": [],
            "events": ["[INFO] Session subprocess starting"],
            "series": {},
            "analysis": None,
        }
        atomic_write_json(
            payload,
            str(Path(self.session_dir) / "live_dashboard.json"),
        )
        atomic_write_json(
            payload,
            str(Path(self.session_dir) / "dashboard.json"),
        )

    def _allocate_session_manifest(
        self,
        *,
        duration_s: object = None,
        **kwargs,
    ) -> Dict[str, object]:
        """Allocate a session directory and persist immutable start provenance."""

        study_payload = dict(kwargs)
        study_payload["duration_s"] = duration_s
        if str(study_payload.get("study_classification") or "") == "confirmatory":
            require_collection_authorization(self.sessions_root)
        legacy = _legacy()
        self.session_dir = str(Path(legacy._next_session_dir(self.sessions_root)))
        Path(self.session_dir).mkdir(parents=True, exist_ok=True)
        if str(study_payload.get("study_classification") or "") in {
            "confirmatory",
            "exploratory",
        }:
            study_assignment = validate_study_assignment(
                study_payload,
                sessions_root=self.sessions_root,
            )
        else:
            study_assignment = {
                "schema_version": "rvt-study-session-v16.5.9",
                "study_classification": "operational",
                "provenance_state": "legacy_unassigned",
                "confirmatory_eligible": False,
            }
        study_assignment = dict(study_assignment)
        attempt_id = str(kwargs.get("attempt_id") or allocate_attempt_id())
        attempt_type = str(kwargs.get("attempt_type") or "subject")
        client_compatibility = kwargs.get("client_compatibility")
        if not isinstance(client_compatibility, dict):
            client_compatibility = {
                "schema_version": "rvt-release-compatibility-v1",
                "decision": "unverified",
                "verified": False,
                "blocks_start": False,
                "confirmatory_eligible": False,
                "client_handshake": None,
                "client_metadata": None,
                "reasons": [
                    {
                        "code": "CLIENT_METADATA_MISSING",
                        "message": "Legacy client supplied no release compatibility metadata.",
                    }
                ],
            }
        release_compatible = client_compatibility.get("decision") == "compatible"
        initial_manifest = {
            "schema_version": legacy.SESSION_MANIFEST_SCHEMA_VERSION,
            "manifest_version": legacy.SESSION_MANIFEST_VERSION,
            "generated_at": _iso_now(),
            "status": "starting",
            "terminal": False,
            "attempt_id": attempt_id,
            "attempt_type": attempt_type,
            "attempt_ledger_schema_version": "rvt-protocol-attempt-ledger-v16.5.9",
            "study_session_schema_version": study_assignment.pop("schema_version"),
            **study_assignment,
            **release_provenance(
                product_version=legacy.VERSION,
                trainer_version=legacy.VERSION,
                dashboard_version=legacy.DASHBOARD_VERSION,
                firmware_expected=legacy.FIRMWARE_VERSION_EXPECTED,
                serial_width_expected=legacy.EXPECTED_RADAR_LOG_COLUMN_COUNT,
                model_family=kwargs.get("model_family"),
                model_bundle=kwargs.get("model_bundle"),
            ),
            "client_compatibility": client_compatibility,
            "client_handshake": client_compatibility.get("client_handshake"),
            "release_compatibility_state": client_compatibility.get(
                "decision",
                "unverified",
            ),
            "release_compatibility_verified": bool(
                client_compatibility.get("verified", False)
            ),
            "start_idempotency_key": kwargs.get("start_idempotency_key"),
            "start_request_hash": kwargs.get("start_request_hash"),
            "logical_trial_reservation_id": kwargs.get(
                "logical_trial_reservation_id"
            ),
            "confirmatory_eligible": bool(
                study_assignment.get("confirmatory_eligible", False)
                and release_compatible
            ),
            "auto_analysed": False,
            "tags": [],
            "notes_count": 0,
            "subject_profile_id": kwargs.get(
                "subject_profile_id",
                "adult_default",
            ),
        }
        capture_provenance = {
            "captured_at": initial_manifest["generated_at"],
            "source": "session_start",
            "product_version": initial_manifest.get("product_version"),
            "trainer_version": initial_manifest.get("trainer_version"),
            "dashboard_version": initial_manifest.get("dashboard_version"),
            "firmware_expected": initial_manifest.get("firmware_expected"),
            "firmware_observed": initial_manifest.get("firmware_observed"),
            "serial_protocol": initial_manifest.get("serial_protocol"),
            "serial_width_expected": initial_manifest.get("serial_width_expected"),
            "source_commit": initial_manifest.get("source_commit"),
            "model_family": initial_manifest.get("model_family"),
            "model_bundle": initial_manifest.get("model_bundle"),
            "participant_id": initial_manifest.get("participant_id"),
            "logical_trial_id": initial_manifest.get("logical_trial_id"),
            "attempt_id": attempt_id,
        }
        initial_manifest["capture_provenance"] = capture_provenance
        initial_manifest["analysis_runs"] = []
        atomic_write_json(
            initial_manifest,
            str(Path(self.session_dir) / "session_manifest.json"),
        )
        initialize_session_attempt(self.session_dir, initial_manifest)
        return initial_manifest

    def _record_start_failure(
        self,
        *,
        stage: str,
        code: str,
        reason: object,
        details: object = None,
    ) -> Dict[str, object]:
        """Make an allocated but failed start a durable terminal session record."""

        if not self.session_dir:
            raise RuntimeError("cannot persist failed start without a session directory")
        path = Path(self.session_dir) / "session_manifest.json"
        manifest = read_json_if_exists(str(path))
        manifest = dict(manifest) if isinstance(manifest, dict) else {}
        failed_at = _iso_now()
        failure = {
            "stage": str(stage),
            "code": str(code),
            "reason": str(reason),
            "failed_at": failed_at,
        }
        if details not in (None, "", [], {}):
            failure["details"] = details
        manifest.update(
            {
                "status": "failed_start",
                "terminal": True,
                "ended_at": failed_at,
                "failure": failure,
            }
        )
        try:
            append_session_attempt_event(
                self.session_dir,
                "failed_start",
                reason=reason,
                details={"stage": stage, "code": code},
            )
        except ValueError:
            pass
        atomic_write_json(manifest, str(path))
        return manifest

    def record_failed_start(
        self,
        *,
        stage: str,
        code: str,
        reason: object,
        details: object = None,
        duration_s: object = None,
        **kwargs,
    ) -> Dict[str, object]:
        """Reserve a failed-session record when a preflight blocks capture."""

        with self._lifecycle_lock:
            if self._closing:
                raise RuntimeError("SUPERVISOR_CLOSING: session starts are disabled")
            self._poll()
            if self.proc is not None and self.proc.poll() is None:
                raise RuntimeError("SESSION_IN_PROGRESS: active session already running")
            if _session_is_active(self.sessions_root):
                raise RuntimeError("SESSION_IN_PROGRESS: session lock active")
            self._allocate_session_manifest(duration_s=duration_s, **kwargs)
            manifest = self._record_start_failure(
                stage=stage,
                code=code,
                reason=reason,
                details=details,
            )
            result = {
                "session_id": Path(self.session_dir).name,
                "session_dir": self.session_dir,
                "manifest": manifest,
            }
            self._reset_runtime_state()
            return result

    def _poll(self) -> bool:
        with self._lifecycle_lock:
            if self.proc is not None and self.proc.poll() is not None:
                proc = self.proc
                session_dir = self.session_dir
                self._clear_current(pid=proc.pid, session_dir=session_dir)
                self._logical_trial_reservations.release_for_session(session_dir)
                self._reset_runtime_state()
                return True
            return False

    def start(
        self,
        duration_s=None,
        radar_port=DEFAULT_RADAR_PORT,
        ble_address=DEFAULT_BLE_ADDRESS,
        ble_profile="ailink_oximeter",
        timeout_s: float = 30.0,
        idempotency_key: object = None,
        idempotency_payload: Optional[Mapping[str, object]] = None,
        preflight: Optional[Callable[[], object]] = None,
        **kwargs,
    ):
        with self._lifecycle_lock:
            if self._closing:
                raise RuntimeError("SUPERVISOR_CLOSING: session starts are disabled")
            key = validate_idempotency_key(idempotency_key)
            request_hash = canonical_start_request_hash(idempotency_payload or {})
            failure_record = None
            reservation = None
            started_result = None
            try:
                if key is not None:
                    replay = self._lookup_idempotent_start_locked(key, request_hash)
                    if replay is not None:
                        return replay
                reservation = self._reserve_logical_trial_locked(
                    kwargs.get("logical_trial_id"),
                    request_hash,
                    key,
                )
                if key is not None:
                    self._start_idempotency.begin(key, request_hash)
                reservation_id = (
                    reservation.get("reservation_id")
                    if isinstance(reservation, dict)
                    else None
                )
                if preflight is not None:
                    try:
                        preflight()
                    except SessionStartPreflightError as exc:
                        failure_record = self.record_failed_start(
                            stage="preflight",
                            code=exc.code,
                            reason=exc,
                            details=exc.details,
                            duration_s=duration_s,
                            start_idempotency_key=key,
                            start_request_hash=request_hash,
                            logical_trial_reservation_id=reservation_id,
                            **kwargs,
                        )
                        exc.failed_session_id = failure_record["session_id"]
                        raise
                result = self._start_locked(
                    duration_s=duration_s,
                    radar_port=radar_port,
                    ble_address=ble_address,
                    ble_profile=ble_profile,
                    timeout_s=timeout_s,
                    start_idempotency_key=key,
                    start_request_hash=request_hash,
                    logical_trial_reservation_id=reservation_id,
                    **kwargs,
                )
                started_result = result
                if isinstance(kwargs.get("client_compatibility"), dict):
                    result["client_compatibility"] = kwargs["client_compatibility"]
                if isinstance(reservation, dict):
                    try:
                        self._logical_trial_reservations.mark_active(
                            str(reservation.get("logical_trial_id") or ""),
                            str(reservation.get("reservation_id") or ""),
                            result,
                        )
                    except OSError:
                        # The active session marker remains the authoritative
                        # safety net. Keep Start successful and leave the
                        # exclusive reservation file in its fail-closed state.
                        pass
                if key is not None:
                    self._start_idempotency.mark_succeeded(
                        key,
                        request_hash,
                        result,
                    )
                return result
            except Exception as exc:
                if isinstance(reservation, dict) and started_result is None:
                    self._logical_trial_reservations.release(
                        str(reservation.get("logical_trial_id") or ""),
                        reservation_id=reservation.get("reservation_id"),
                    )
                if key is not None:
                    code = getattr(exc, "code", None) or (
                        "SPAWN_TIMEOUT"
                        if isinstance(exc, TimeoutError)
                        else "SPAWN_ERROR"
                    )
                    self._start_idempotency.mark_failed(
                        key,
                        request_hash,
                        code=str(code),
                        message=str(exc),
                        session_id=(
                            (failure_record or {}).get("session_id")
                            or getattr(exc, "session_id", None)
                        ),
                        session_dir=(
                            (failure_record or {}).get("session_dir")
                            or getattr(exc, "session_dir", None)
                        ),
                        details=getattr(exc, "details", None),
                    )
                raise

    def _lookup_idempotent_start_locked(
        self,
        key: str,
        request_hash: str,
    ) -> Optional[Dict[str, object]]:
        state, record = self._start_idempotency.lookup(key, request_hash)
        if state == "missing":
            return None
        if state == "replay":
            return record
        if state == "failed":
            error = record.get("error") if isinstance(record, dict) else {}
            error = error if isinstance(error, dict) else {}
            replay_details = (
                dict(error.get("details"))
                if isinstance(error.get("details"), dict)
                else {}
            )
            if record.get("session_id") not in (None, ""):
                replay_details["failed_session_id"] = record.get("session_id")
            raise StartIdempotencyError(
                str(error.get("code") or "IDEMPOTENT_START_FAILED"),
                str(error.get("message") or "the original session-start request failed"),
                details=replay_details,
            )
        raise StartIdempotencyError(
            "IDEMPOTENCY_REQUEST_IN_PROGRESS",
            "the same session-start request is already in progress; check /api/session/current before retrying",
            details={"idempotency_key": key, "record": record},
        )

    def lookup_idempotent_start(
        self,
        idempotency_key: object,
        idempotency_payload: Mapping[str, object],
    ) -> Optional[Dict[str, object]]:
        key = validate_idempotency_key(idempotency_key)
        if key is None:
            return None
        request_hash = canonical_start_request_hash(idempotency_payload)
        with self._lifecycle_lock:
            return self._lookup_idempotent_start_locked(key, request_hash)

    def _start_locked(
        self,
        duration_s=None,
        radar_port=DEFAULT_RADAR_PORT,
        ble_address=DEFAULT_BLE_ADDRESS,
        ble_profile="ailink_oximeter",
        timeout_s: float = 30.0,
        **kwargs,
    ):
        legacy = _legacy()
        self._poll()
        if self.proc is not None and self.proc.poll() is None:
            raise RuntimeError("SESSION_IN_PROGRESS: active session already running")
        if _session_is_active(self.sessions_root):
            raise RuntimeError("SESSION_IN_PROGRESS: session lock active")
        radar_port = str(radar_port or DEFAULT_RADAR_PORT).strip() or DEFAULT_RADAR_PORT
        ble_address = (
            str(ble_address or DEFAULT_BLE_ADDRESS).strip() or DEFAULT_BLE_ADDRESS
        )
        self._allocate_session_manifest(duration_s=duration_s, **kwargs)
        argv = [
            sys.executable,
            str(legacy._TRAINER_ENTRYPOINT),
            "session",
            "--session-dir",
            self.session_dir,
            "--sessions-root",
            self.sessions_root,
            "--port",
            radar_port,
            "--address",
            ble_address,
            "--ble-profile",
            ble_profile,
            "--dashboard-port",
            "0",
            "--no-open-dashboard",
        ]
        if kwargs.get("notify_char"):
            argv += ["--notify-char", str(kwargs.get("notify_char"))]
        if kwargs.get("dashboard_refresh_s"):
            argv += [
                "--dashboard-refresh-s",
                str(kwargs.get("dashboard_refresh_s")),
            ]
        if kwargs.get("subject_profile_id"):
            argv += [
                "--subject-profile-id",
                str(kwargs.get("subject_profile_id")),
            ]
        study_flag_names = {
            "participant_id": "--participant-id",
            "trial_id": "--trial-id",
            "condition_id": "--condition-id",
            "distance_m": "--distance-m",
            "barrier_type": "--barrier-type",
            "trial_number": "--trial-number",
            "planned_duration_s": "--planned-duration-s",
            "study_classification": "--study-classification",
            "model_family": "--model-family",
            "model_bundle": "--model-bundle",
        }
        for key, flag in study_flag_names.items():
            value = kwargs.get(key)
            if value not in (None, "", "operational"):
                argv += [flag, str(value)]
        if duration_s is not None:
            argv += ["--duration-s", str(duration_s)]
        creationflags = (
            getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
            if os.name == "nt"
            else 0
        )
        try:
            self._write_starting_live_payload(duration_s=duration_s)
            self.proc = subprocess.Popen(argv, creationflags=creationflags)
        except Exception as exc:
            failed_session_dir = self.session_dir
            self._record_start_failure(
                stage="process_launch",
                code="SPAWN_ERROR",
                reason=exc,
            )
            self._reset_runtime_state()
            error = RuntimeError(f"SPAWN_ERROR: {exc}")
            error.session_id = Path(failed_session_dir).name
            error.session_dir = failed_session_dir
            raise error from exc
        self.started_at = _iso_now()
        self.started_monotonic = time.monotonic()
        self.params = {
            "duration_s": duration_s,
            "radar_port": radar_port,
            "ble_address": ble_address,
            "ble_profile": ble_profile,
        }
        self.params.update(
            {key: value for key, value in kwargs.items() if value not in (None, "")}
        )
        self._write_current()
        live = Path(self.session_dir) / "live_dashboard.json"
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                proc = self.proc
                failed_session_dir = self.session_dir
                self._record_start_failure(
                    stage="startup_exit",
                    code="SPAWN_ERROR",
                    reason="session exited before live_dashboard.json appeared",
                    details={"returncode": proc.poll()},
                )
                self._clear_current(pid=proc.pid, session_dir=self.session_dir)
                self._reset_runtime_state()
                error = RuntimeError(
                    "SPAWN_ERROR: session exited before live_dashboard.json appeared"
                )
                error.session_id = Path(failed_session_dir).name
                error.session_dir = failed_session_dir
                raise error
            live_payload = read_json_if_exists(str(live))
            if isinstance(live_payload, dict) and not live_payload.get(
                "_supervisor_placeholder"
            ):
                try:
                    append_session_attempt_event(self.session_dir, "collecting")
                except ValueError:
                    pass
                return {
                    "session_id": Path(self.session_dir).name,
                    "session_dir": self.session_dir,
                    "pid": self.proc.pid,
                    "started_at": self.started_at,
                }
            time.sleep(0.02)
        failed_session_dir = self.session_dir
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
            proc = self.proc
            self._record_start_failure(
                stage="startup_timeout",
                code="SPAWN_TIMEOUT",
                reason="live_dashboard.json did not appear before timeout",
                details={"timeout_s": float(timeout_s)},
            )
            self._clear_current(
                pid=getattr(proc, "pid", None),
                session_dir=self.session_dir,
            )
            self._reset_runtime_state()
        error = TimeoutError("live_dashboard.json did not appear before timeout")
        error.session_id = Path(failed_session_dir).name
        error.session_dir = failed_session_dir
        raise error

    _wait_for_exit = staticmethod(wait_for_process_exit)

    def stop(
        self,
        reason: str = "user_request",
        *,
        auto_analyse: bool = True,
        missing_ok: bool = False,
    ):
        with self._lifecycle_lock:
            if self.proc is None:
                if not missing_ok:
                    raise RuntimeError("no active session")
                return _legacy()._schema_wrap(
                    {
                        "session_id": "",
                        "stopped_at": _iso_now(),
                        "reason": reason,
                        "auto_analyse": None,
                        "already_stopped": True,
                    }
                )

            proc = self.proc
            stopped_session_dir = str(self.session_dir or "")
            request = _write_supervisor_stop_request(
                stopped_session_dir,
                reason=reason,
                session_pid=proc.pid,
                auto_analyse=auto_analyse,
            )
            exited = proc.poll() is not None
            if not exited:
                sig = getattr(signal, "CTRL_BREAK_EVENT", signal.SIGINT)
                try:
                    proc.send_signal(sig)
                except Exception:
                    try:
                        proc.send_signal(signal.SIGINT)
                    except Exception:
                        pass
                exited = self._wait_for_exit(proc, self._stop_grace_s)
            if not exited:
                try:
                    proc.terminate()
                except Exception:
                    pass
                exited = self._wait_for_exit(proc, self._terminate_grace_s)
            if not exited:
                try:
                    proc.kill()
                except Exception:
                    pass
                exited = self._wait_for_exit(proc, self._kill_grace_s)
            if not exited:
                raise RuntimeError(
                    f"SESSION_STOP_FAILED: child process {proc.pid} could not be "
                    "reaped; session markers were preserved"
                )

            try:
                append_session_attempt_event(
                    stopped_session_dir,
                    "stopped",
                    reason=reason,
                )
            except ValueError:
                pass

            current_clean = self._clear_current(
                pid=proc.pid,
                session_dir=stopped_session_dir,
            )
            lock_clean = _release_session_lock_if_owned(
                self.sessions_root,
                pid=proc.pid,
                session_dir=stopped_session_dir,
            )
            request_clean = _clear_supervisor_stop_request(
                stopped_session_dir,
                request.get("request_id"),
            )
            self._logical_trial_reservations.release_for_session(
                stopped_session_dir
            )
            self._reset_runtime_state()
            if not (current_clean and lock_clean and request_clean):
                raise RuntimeError(
                    "SESSION_CLEANUP_CONFLICT: stopped child was reaped, but a "
                    "marker owned by another session was preserved"
                )

            legacy = _legacy()
            auto = (
                legacy._spawn_auto_analyse(stopped_session_dir, reason=reason)
                if auto_analyse and stopped_session_dir
                else None
            )
            return legacy._schema_wrap(
                {
                    "session_id": Path(stopped_session_dir).name,
                    "stopped_at": _iso_now(),
                    "reason": reason,
                    "auto_analyse": auto,
                }
            )

    def current(self) -> Optional[Dict[str, object]]:
        with self._lifecycle_lock:
            self._poll()
            if self.proc is None:
                data = self._read_current()
                if data and _pid_alive(data.get("pid")):
                    return data
                if data:
                    self._clear_current(
                        pid=data.get("pid"),
                        session_dir=data.get("session_dir"),
                    )
                lock = _read_session_lock(self.sessions_root)
                if lock and _pid_alive(lock.get("pid")):
                    return {
                        "session_id": Path(
                            str(lock.get("session_dir", ""))
                        ).name,
                        "session_dir": lock.get("session_dir"),
                        "pid": lock.get("pid"),
                        "started_at": lock.get("started_at"),
                        "external": True,
                    }
                return None
            elapsed = max(
                0.0,
                time.monotonic()
                - float(self.started_monotonic or time.monotonic()),
            )
            duration = self.params.get("duration_s")
            try:
                remaining = (
                    max(0.0, float(duration) - elapsed)
                    if duration is not None
                    else None
                )
            except Exception:
                remaining = None
            return {
                "session_id": Path(self.session_dir).name,
                "session_dir": self.session_dir,
                "pid": self.proc.pid,
                "started_at": self.started_at,
                "elapsed_s": elapsed,
                "remaining_s": remaining,
                "params": self.params,
            }


__all__ = [
    "SessionSupervisor",
    "_check_stale_session_lock",
    "_clear_supervisor_stop_request",
    "_consume_supervisor_stop_request",
    "_lock_path",
    "_read_session_lock",
    "_release_session_lock",
    "_release_session_lock_if_owned",
    "_same_session_dir",
    "_session_is_active",
    "_session_marker_owned_by",
    "_supervisor_stop_path",
    "_write_session_lock",
    "_write_supervisor_stop_request",
]
