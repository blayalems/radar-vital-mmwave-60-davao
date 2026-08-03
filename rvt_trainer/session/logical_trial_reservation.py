"""Durable logical-trial reservations for study session starts.

An HTTP idempotency key identifies one transport request.  A logical trial
identifies the protocol slot (participant, condition, repetition) that request
is trying to start.  Keeping those identities separate prevents two clients
with different request keys from allocating the same live trial while still
allowing a deliberate retry after the prior attempt becomes terminal.
"""

from __future__ import annotations

import hashlib
import json
import os
import secrets
import time
from pathlib import Path
from typing import Dict, Mapping, Optional

from rvt_trainer.api.common import atomic_write_json, read_json_if_exists


LOGICAL_TRIAL_RESERVATION_SCHEMA_VERSION = "rvt-logical-trial-reservation-v1"
STARTING_RESERVATION_STALE_S = 5 * 60


def _reservation_filename(logical_trial_id: str) -> str:
    digest = hashlib.sha256(str(logical_trial_id).encode("utf-8")).hexdigest()
    return f"{digest}.json"


class LogicalTrialReservationStore:
    """One exclusive reservation file per logical trial.

    ``os.O_EXCL`` supplies the cross-thread/process arbitration that an atomic
    replace alone cannot provide.  Callers decide whether an existing record
    is stale because only the session supervisor knows whether its child still
    owns the live-session markers.
    """

    def __init__(self, sessions_root: str):
        self.root = Path(sessions_root).resolve() / ".logical_trial_reservations"

    def _path(self, logical_trial_id: str) -> Path:
        return self.root / _reservation_filename(logical_trial_id)

    def lookup(self, logical_trial_id: str) -> Optional[Dict[str, object]]:
        payload = read_json_if_exists(str(self._path(logical_trial_id)))
        if not isinstance(payload, dict):
            return None
        if str(payload.get("logical_trial_id") or "") != str(logical_trial_id):
            return None
        return dict(payload)

    def reserve(
        self,
        logical_trial_id: str,
        request_hash: str,
        *,
        idempotency_key: object = None,
    ) -> Dict[str, object]:
        self.root.mkdir(parents=True, exist_ok=True)
        path = self._path(logical_trial_id)
        now = time.time()
        record: Dict[str, object] = {
            "schema_version": LOGICAL_TRIAL_RESERVATION_SCHEMA_VERSION,
            "reservation_id": f"LTR-{secrets.token_hex(10)}",
            "logical_trial_id": str(logical_trial_id),
            "request_hash": str(request_hash),
            "idempotency_key": (
                str(idempotency_key) if idempotency_key not in (None, "") else None
            ),
            "state": "starting",
            "created_at_epoch_s": now,
            "updated_at_epoch_s": now,
        }
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        fd = os.open(str(path), flags, 0o644)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                json.dump(record, handle, indent=2, allow_nan=False)
                handle.flush()
                os.fsync(handle.fileno())
        except Exception:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
            raise
        return record

    def mark_active(
        self,
        logical_trial_id: str,
        reservation_id: str,
        result: Mapping[str, object],
    ) -> bool:
        path = self._path(logical_trial_id)
        record = self.lookup(logical_trial_id)
        if not isinstance(record, dict) or record.get("reservation_id") != reservation_id:
            return False
        record.update(
            {
                "state": "active",
                "updated_at_epoch_s": time.time(),
                "session_id": result.get("session_id"),
                "session_dir": result.get("session_dir"),
                "result": dict(result),
            }
        )
        atomic_write_json(record, str(path))
        return True

    def release(
        self,
        logical_trial_id: str,
        *,
        reservation_id: object = None,
    ) -> bool:
        path = self._path(logical_trial_id)
        if reservation_id not in (None, ""):
            record = self.lookup(logical_trial_id)
            if not isinstance(record, dict) or record.get("reservation_id") != reservation_id:
                return False
        try:
            path.unlink()
            return True
        except FileNotFoundError:
            return True

    def release_for_session(self, session_dir: object) -> bool:
        if session_dir in (None, "") or not self.root.exists():
            return False
        expected = os.path.normcase(os.path.abspath(str(session_dir)))
        released = False
        for path in self.root.glob("*.json"):
            record = read_json_if_exists(str(path))
            if not isinstance(record, dict):
                continue
            recorded_dir = record.get("session_dir")
            if recorded_dir in (None, ""):
                continue
            if os.path.normcase(os.path.abspath(str(recorded_dir))) != expected:
                continue
            try:
                path.unlink()
                released = True
            except FileNotFoundError:
                pass
        return released


__all__ = [
    "LOGICAL_TRIAL_RESERVATION_SCHEMA_VERSION",
    "STARTING_RESERVATION_STALE_S",
    "LogicalTrialReservationStore",
]
