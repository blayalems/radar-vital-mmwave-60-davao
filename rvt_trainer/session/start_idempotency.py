"""Durable, bounded idempotency records for capture-session starts."""

from __future__ import annotations

import hashlib
import json
import re
import time
from pathlib import Path
from typing import Callable, Dict, Mapping, Optional, Tuple

from rvt_trainer.api.common import atomic_write_json, read_json_if_exists


IDEMPOTENCY_SCHEMA_VERSION = "rvt-session-start-idempotency-v1"
IDEMPOTENCY_RETENTION_S = 24 * 60 * 60
IDEMPOTENCY_MAX_RECORDS = 256
IDEMPOTENCY_KEY_PATTERN = re.compile(r"^[A-Za-z0-9._:-]{1,128}$")


class StartIdempotencyError(RuntimeError):
    def __init__(self, code: str, message: str, *, details: object = None):
        super().__init__(message)
        self.code = code
        self.details = details


def canonical_start_request_hash(payload: Mapping[str, object]) -> str:
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
        default=str,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def validate_idempotency_key(value: object) -> Optional[str]:
    if value in (None, ""):
        return None
    key = str(value).strip()
    if not IDEMPOTENCY_KEY_PATTERN.fullmatch(key):
        raise StartIdempotencyError(
            "INVALID_IDEMPOTENCY_KEY",
            "idempotency_key must be 1-128 URL-safe letters, digits, dot, colon, dash, or underscore",
        )
    return key


class StartIdempotencyStore:
    """JSON-backed request ledger; callers provide their own lifecycle lock."""

    def __init__(
        self,
        sessions_root: str,
        *,
        retention_s: float = IDEMPOTENCY_RETENTION_S,
        max_records: int = IDEMPOTENCY_MAX_RECORDS,
        clock: Callable[[], float] = time.time,
    ):
        self.path = Path(sessions_root) / ".session_start_idempotency.json"
        self.retention_s = max(1.0, float(retention_s))
        self.max_records = max(1, int(max_records))
        self.clock = clock

    def _load(self) -> Dict[str, object]:
        payload = read_json_if_exists(str(self.path))
        if not isinstance(payload, dict):
            payload = {}
        records = payload.get("records")
        return {
            "schema_version": IDEMPOTENCY_SCHEMA_VERSION,
            "records": dict(records) if isinstance(records, dict) else {},
        }

    def _save(self, payload: Dict[str, object]) -> None:
        payload["schema_version"] = IDEMPOTENCY_SCHEMA_VERSION
        payload["updated_at_epoch_s"] = self.clock()
        atomic_write_json(payload, str(self.path))

    def _prune(self, payload: Dict[str, object]) -> bool:
        now = self.clock()
        records = payload["records"]
        changed = False
        for key, record in list(records.items()):
            if not isinstance(record, dict):
                records.pop(key, None)
                changed = True
                continue
            updated = float(record.get("updated_at_epoch_s") or 0.0)
            if record.get("state") != "starting" and now - updated > self.retention_s:
                records.pop(key, None)
                changed = True
        if len(records) > self.max_records:
            ordered = sorted(
                records.items(),
                key=lambda item: float(
                    (item[1] if isinstance(item[1], dict) else {}).get(
                        "updated_at_epoch_s"
                    )
                    or 0.0
                ),
            )
            terminal = [
                (key, record)
                for key, record in ordered
                if isinstance(record, dict) and record.get("state") != "starting"
            ]
            for key, _record in terminal:
                if len(records) <= self.max_records:
                    break
                records.pop(key, None)
                changed = True
            # At most one start can be live under the supervisor lock. If a
            # corrupt/imported store still exceeds the bound, discard the
            # oldest stale entries; the session lock independently prevents a
            # duplicate active process.
            for key, _record in ordered:
                if len(records) <= self.max_records:
                    break
                records.pop(key, None)
                changed = True
        return changed

    def lookup(
        self,
        key: str,
        request_hash: str,
    ) -> Tuple[str, Optional[Dict[str, object]]]:
        payload = self._load()
        changed = self._prune(payload)
        record = payload["records"].get(key)
        if changed:
            self._save(payload)
        if not isinstance(record, dict):
            return "missing", None
        if record.get("request_hash") != request_hash:
            raise StartIdempotencyError(
                "IDEMPOTENCY_KEY_REUSED",
                "idempotency_key was already used with a different session-start payload",
                details={
                    "idempotency_key": key,
                    "original_request_hash": record.get("request_hash"),
                    "request_hash": request_hash,
                },
            )
        state = str(record.get("state") or "")
        if state == "succeeded" and isinstance(record.get("result"), dict):
            return "replay", dict(record["result"])
        if state == "failed":
            return "failed", dict(record)
        return "starting", dict(record)

    def begin(self, key: str, request_hash: str) -> None:
        state, record = self.lookup(key, request_hash)
        if state != "missing":
            raise StartIdempotencyError(
                "IDEMPOTENCY_REQUEST_IN_PROGRESS",
                "the same session-start request is already in progress",
                details={"idempotency_key": key, "record": record},
            )
        now = self.clock()
        payload = self._load()
        self._prune(payload)
        payload["records"][key] = {
            "request_hash": request_hash,
            "state": "starting",
            "created_at_epoch_s": now,
            "updated_at_epoch_s": now,
        }
        self._save(payload)

    def mark_succeeded(
        self,
        key: str,
        request_hash: str,
        result: Mapping[str, object],
    ) -> None:
        payload = self._load()
        record = payload["records"].get(key)
        if not isinstance(record, dict) or record.get("request_hash") != request_hash:
            raise StartIdempotencyError(
                "IDEMPOTENCY_RECORD_LOST",
                "session-start idempotency reservation was lost before completion",
            )
        record.update(
            {
                "state": "succeeded",
                "updated_at_epoch_s": self.clock(),
                "result": dict(result),
                "session_id": result.get("session_id"),
                "session_dir": result.get("session_dir"),
            }
        )
        self._prune(payload)
        self._save(payload)

    def mark_failed(
        self,
        key: str,
        request_hash: str,
        *,
        code: str,
        message: str,
        session_id: object = None,
        session_dir: object = None,
        details: object = None,
    ) -> None:
        payload = self._load()
        record = payload["records"].get(key)
        if not isinstance(record, dict) or record.get("request_hash") != request_hash:
            return
        record.update(
            {
                "state": "failed",
                "updated_at_epoch_s": self.clock(),
                "error": {
                    "code": str(code),
                    "message": str(message),
                    "details": details,
                },
                "session_id": session_id,
                "session_dir": session_dir,
            }
        )
        self._prune(payload)
        self._save(payload)

    def prune(self) -> int:
        payload = self._load()
        before = len(payload["records"])
        if self._prune(payload):
            self._save(payload)
        return before - len(payload["records"])


__all__ = [
    "IDEMPOTENCY_KEY_PATTERN",
    "IDEMPOTENCY_MAX_RECORDS",
    "IDEMPOTENCY_RETENTION_S",
    "IDEMPOTENCY_SCHEMA_VERSION",
    "StartIdempotencyError",
    "StartIdempotencyStore",
    "canonical_start_request_hash",
    "validate_idempotency_key",
]
