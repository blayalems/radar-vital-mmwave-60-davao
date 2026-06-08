"""Pairing PIN and bearer-token helpers for the LAN-bind trainer.

The trainer exposes two related primitives:

* :func:`make_pair_pin` — mints a fresh 6-digit numeric PIN, tracked in a
  per-process TTL store on ``server.pair_pins``. PINs expire after
  ``_PIN_TTL_S`` seconds; the store is bounded to ``_PIN_MAX`` outstanding
  entries (oldest evicted first).
* :func:`exchange_pair_pin` — single-use redemption. Invalid per-client
  attempts are bounded before a short lockout is applied. On success the PIN
  is deleted from the store *immediately* and a 32-byte URL-safe token is
  added to ``server.auth_tokens``. Replay of a consumed or expired PIN
  returns ``410 Gone``.

The token store lives on the server instance for the lifetime of the
process; restart-on-restart re-pairing is expected and documented in
``/pair``.
"""

from __future__ import annotations

import hashlib
import hmac
import json
from pathlib import Path
import re
import secrets
import threading
import time
from typing import Dict, Tuple

_PIN_TTL_S = 300       # 5 minute PIN expiry
_PIN_MAX = 8           # max concurrent outstanding PINs
_PAIR_FAILURE_LIMIT = 5
_PAIR_FAILURE_WINDOW_S = 60.0
_PAIR_LOCKOUT_S = 60.0
_PIN_LOCK = threading.Lock()


def make_pair_pin(server) -> str:
    """Mint, store, and return a fresh 6-digit pairing PIN.

    Expired entries are reaped first, then the store is trimmed to
    ``_PIN_MAX`` by evicting the oldest entry.
    """
    with _PIN_LOCK:
        pins = getattr(server, "pair_pins", None)
        if pins is None:
            pins = {}
            server.pair_pins = pins

        pin = f"{secrets.randbelow(1000000):06d}"
        while pin in pins:
            pin = f"{secrets.randbelow(1000000):06d}"

        now = time.time()
        expired = [k for k, item in pins.items() if float(item.get("expires_at", 0.0)) <= now]
        for key in expired:
            pins.pop(key, None)
        while len(pins) >= _PIN_MAX:
            oldest = min(pins, key=lambda k: float(pins[k].get("created_at", 0.0)))
            pins.pop(oldest, None)
        pins[pin] = {"created_at": now, "expires_at": now + _PIN_TTL_S}
        server.active_pin = pin
        server.active_pin_expires_at = now + _PIN_TTL_S
        return pin


def exchange_pair_pin(server, pin: str, client_id: str = "") -> Tuple[int, Dict[str, object]]:
    """Redeem ``pin`` for an opaque bearer token.

    Returns a ``(status, body)`` tuple suitable for the HTTP handler:

    * ``200`` + ``{"ok": True, "token": ..., "token_type": "RVT-Token", "expires": "process"}``
      on success. The PIN is consumed; replay yields ``410``.
    * ``410`` + ``{"ok": False, "error": {...}}`` if the PIN is unknown or
      expired.
    * ``429`` after repeated failures from one client identity during the
      lockout interval.

    The token is added to ``server.auth_tokens`` (set) on success and stays
    valid for the trainer process lifetime.
    """
    with _PIN_LOCK:
        now = time.time()
        key = str(client_id or "").strip()
        failures = getattr(server, "pair_exchange_failures", None)
        if failures is None:
            failures = {}
            server.pair_exchange_failures = failures
        record = failures.get(key, {"attempts": [], "blocked_until": 0.0}) if key else None
        if record and float(record.get("blocked_until", 0.0)) > now:
            return 429, {
                "ok": False,
                "error": {
                    "code": "PAIRING_RATE_LIMITED",
                    "message": "too many invalid pairing PIN attempts; try again later",
                    "retry_after_s": max(1, int(float(record["blocked_until"]) - now)),
                },
            }

        def failure_response(code: str, message: str) -> Tuple[int, Dict[str, object]]:
            if key:
                recent = [
                    float(attempt)
                    for attempt in record.get("attempts", [])
                    if now - float(attempt) < _PAIR_FAILURE_WINDOW_S
                ]
                recent.append(now)
                record["attempts"] = recent
                if len(recent) >= _PAIR_FAILURE_LIMIT:
                    record["blocked_until"] = now + _PAIR_LOCKOUT_S
                    failures[key] = record
                    return 429, {
                        "ok": False,
                        "error": {
                            "code": "PAIRING_RATE_LIMITED",
                            "message": "too many invalid pairing PIN attempts; try again later",
                            "retry_after_s": int(_PAIR_LOCKOUT_S),
                        },
                    }
                failures[key] = record
            return 410, {"ok": False, "error": {"code": code, "message": message}}

        pins = getattr(server, "pair_pins", None)
        if pins is None:
            pins = {}
            server.pair_pins = pins

        item = pins.get(str(pin or ""))
        if not item:
            return failure_response("PIN_EXPIRED_OR_USED", "pairing PIN expired or was already used")
        if float(item.get("expires_at", 0.0)) <= now:
            pins.pop(str(pin), None)
            return failure_response("PIN_EXPIRED", "pairing PIN expired")
        pins.pop(str(pin), None)
        if getattr(server, "active_pin", "") == str(pin):
            server.active_pin = ""
            server.active_pin_expires_at = 0.0
        if key:
            failures.pop(key, None)
        token = secrets.token_urlsafe(24)

        tokens = getattr(server, "auth_tokens", None)
        if tokens is None:
            tokens = set()
            server.auth_tokens = tokens
        tokens.add(token)

        return 200, {
            "ok": True,
            "token": token,
            "token_type": "RVT-Token",
            "expires": "process",
        }


_OPERATOR_LOCK = threading.Lock()


def _operator_profiles_path(sessions_root: str) -> Path:
    return Path(sessions_root).resolve().parent / "operator_profiles.json"


def load_operator_profiles(sessions_root: str) -> dict:
    path = _operator_profiles_path(sessions_root)
    if not path.exists():
        return {"schema_version": "rvt-operator-profiles-v12.0", "profiles": {}}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
            if not isinstance(data, dict) or "profiles" not in data:
                return {"schema_version": "rvt-operator-profiles-v12.0", "profiles": {}}
            return data
    except Exception:
        return {"schema_version": "rvt-operator-profiles-v12.0", "profiles": {}}


def save_operator_profiles(sessions_root: str, data: dict):
    path = _operator_profiles_path(sessions_root)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = path.with_suffix(".tmp")
        with open(temp_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        temp_path.replace(path)
    except Exception:
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception:
            pass


def hash_pin(pin: str, salt: str, iterations: int = 200000) -> str:
    """Hash the PIN using PBKDF2 with HMAC-SHA256."""
    h = hashlib.pbkdf2_hmac("sha256", pin.encode("utf-8"), salt.encode("utf-8"), iterations)
    return h.hex()


def verify_pin(pin: str, salt: str, pin_hash: str, iterations: int = 200000) -> bool:
    """Verify the PIN against the salt and hash using hmac.compare_digest."""
    candidate_hash = hash_pin(pin, salt, iterations)
    return hmac.compare_digest(candidate_hash.encode("utf-8"), pin_hash.encode("utf-8"))


def login_operator(server, operator_id: str, pin: str) -> Tuple[int, dict]:
    """Authenticate an operator and return token or lockout info."""
    with _OPERATOR_LOCK:
        db = load_operator_profiles(server.sessions_root)
        profiles = db.get("profiles", {})
        if operator_id not in profiles:
            return 401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "Invalid operator ID or PIN"}}

        profile = profiles[operator_id]
        now = time.time()

        # Check lockout
        locked_until = float(profile.get("locked_until", 0.0))
        if locked_until > now:
            retry_after = int(locked_until - now)
            if retry_after <= 0:
                retry_after = 1
            return 429, {
                "ok": False,
                "error": {
                    "code": "LOCKOUT_ACTIVE",
                    "message": f"Too many failed attempts. Try again in {retry_after} seconds.",
                    "retry_after_s": retry_after
                }
            }

        salt = profile["pin_salt"]
        correct_hash = profile["pin_hash"]
        iterations = int(profile.get("pin_iterations", 200000))

        if verify_pin(pin, salt, correct_hash, iterations):
            profile["failed_attempts"] = 0
            profile["locked_until"] = 0.0
            profile["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            save_operator_profiles(server.sessions_root, db)

            token = secrets.token_urlsafe(24)
            expires_at = now + 8 * 3600  # 8 hours

            if not hasattr(server, "operator_sessions"):
                server.operator_sessions = {}
            server.operator_sessions[token] = {
                "operator_id": operator_id,
                "expires_at": expires_at,
                "display_name": profile.get("display_name"),
                "initials": profile.get("initials")
            }

            return 200, {
                "ok": True,
                "token": token,
                "expires_at": expires_at,
                "operator": {
                    "operator_id": operator_id,
                    "display_name": profile.get("display_name"),
                    "initials": profile.get("initials")
                }
            }
        else:
            attempts = int(profile.get("failed_attempts", 0)) + 1
            profile["failed_attempts"] = attempts
            profile["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

            if attempts >= 5:
                profile["locked_until"] = now + 30.0
                save_operator_profiles(server.sessions_root, db)
                return 429, {
                    "ok": False,
                    "error": {
                        "code": "LOCKOUT_ACTIVE",
                        "message": "Too many failed attempts. Try again in 30 seconds.",
                        "retry_after_s": 30
                    }
                }
            else:
                save_operator_profiles(server.sessions_root, db)
                return 401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "Invalid operator ID or PIN"}}


def create_operator_profile(server, body: dict) -> Tuple[int, dict]:
    display_name = str(body.get("display_name") or "").strip()
    initials = str(body.get("initials") or "").strip().upper()
    pin = str(body.get("pin") or "").strip()

    if not display_name:
        return 400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "display_name is required"}}
    if not re.fullmatch(r"[A-Z]{2,5}", initials):
        return 400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "initials must be 2 to 5 uppercase letters"}}
    if not re.fullmatch(r"\d{4}", pin):
        return 400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "pin must be exactly 4 digits"}}

    with _OPERATOR_LOCK:
        db = load_operator_profiles(server.sessions_root)
        profiles = db.get("profiles", {})

        operator_id = f"op_{secrets.token_hex(4)}"
        while operator_id in profiles:
            operator_id = f"op_{secrets.token_hex(4)}"

        salt = secrets.token_hex(16)
        iterations = 200000
        pin_hash_val = hash_pin(pin, salt, iterations)
        now_str = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

        profile = {
            "operator_id": operator_id,
            "display_name": display_name,
            "initials": initials,
            "created_at": now_str,
            "updated_at": now_str,
            "pin_hash": pin_hash_val,
            "pin_salt": salt,
            "pin_iterations": iterations,
            "failed_attempts": 0,
            "locked_until": 0.0
        }

        profiles[operator_id] = profile
        db["profiles"] = profiles
        save_operator_profiles(server.sessions_root, db)

        return 200, {
            "ok": True,
            "operator": {
                "operator_id": operator_id,
                "display_name": display_name,
                "initials": initials
            }
        }


__all__ = [
    "make_pair_pin",
    "exchange_pair_pin",
    "load_operator_profiles",
    "save_operator_profiles",
    "hash_pin",
    "verify_pin",
    "login_operator",
    "create_operator_profile",
    "_PIN_TTL_S",
    "_PIN_MAX",
    "_PAIR_FAILURE_LIMIT",
    "_PAIR_FAILURE_WINDOW_S",
    "_PAIR_LOCKOUT_S",
]
