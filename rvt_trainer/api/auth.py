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
import logging
from pathlib import Path
import re
import secrets
import threading
import time
import unicodedata
from typing import Dict, Tuple

logger = logging.getLogger("rvt_trainer.auth")

_PIN_TTL_S = 300       # 5 minute PIN expiry
_PIN_MAX = 8           # max concurrent outstanding PINs
_PAIR_FAILURE_LIMIT = 5
_PAIR_FAILURE_WINDOW_S = 60.0
_PAIR_LOCKOUT_S = 60.0
_PIN_LOCK = threading.Lock()

# Recovery code alphabet (no O/I/0/1 to avoid ambiguity) — Crockford-ish
_RECOVERY_ALPHABET = "ABCDEFGHJKMNPQRSTVWXYZ23456789"
_RECOVERY_FAILURE_LIMIT = 5
_RECOVERY_LOCKOUT_S = 30.0


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
    return Path(sessions_root).resolve() / "operator_profiles.json"


def load_operator_profiles(sessions_root: str) -> dict:
    path = _operator_profiles_path(sessions_root)
    if not path.exists():
        old_path = Path(sessions_root).resolve().parent / "operator_profiles.json"
        if old_path.exists():
            # Legacy migration must fail closed for the same reason the main path
            # does: a corrupt/unreadable legacy DB must NOT silently migrate into
            # an empty bootstrap state and reopen the unauthenticated admin-create
            # window. Validate before migrating, and preserve the legacy file on
            # error (do not destroy it).
            try:
                with open(old_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except (OSError, ValueError) as exc:
                logger.warning(
                    "legacy operator profiles DB is unreadable (%s); failing closed",
                    type(exc).__name__,
                )
                return {"schema_version": "rvt-operator-profiles-v12.0", "profiles": {}, "_load_error": "legacy_unreadable"}
            if not isinstance(data, dict) or "profiles" not in data or not isinstance(data.get("profiles"), dict):
                logger.warning("legacy operator profiles DB has an invalid schema; failing closed")
                return {"schema_version": "rvt-operator-profiles-v12.0", "profiles": {}, "_load_error": "legacy_bad_schema"}
            save_operator_profiles(sessions_root, data)
            try:
                old_path.unlink()
            except OSError:
                logger.warning("legacy operator profiles DB migrated but the old copy could not be removed")
            return data
        return {"schema_version": "rvt-operator-profiles-v12.0", "profiles": {}}
    # The file is present. Distinguish "legitimately empty" (handled above by the
    # not-exists branch) from "present but unreadable/corrupt". A transient read
    # error or on-disk corruption must NOT silently collapse into an empty
    # bootstrap state — that would let a LAN client create a fresh admin profile
    # (the bootstrap POST /api/operator-profiles path) during the failure window.
    # Fail closed: surface a ``_load_error`` marker so the auth layer refuses to
    # treat a damaged DB as bootstrap. The file is deliberately left in place
    # (not deleted) so a transient error cannot destroy real profiles.
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError) as exc:
        logger.warning(
            "operator profiles DB at %s is unreadable (%s); failing closed",
            path.name, type(exc).__name__,
        )
        return {"schema_version": "rvt-operator-profiles-v12.0", "profiles": {}, "_load_error": "unreadable"}
    if not isinstance(data, dict) or "profiles" not in data or not isinstance(data.get("profiles"), dict):
        logger.warning(
            "operator profiles DB at %s has an invalid schema; failing closed",
            path.name,
        )
        return {"schema_version": "rvt-operator-profiles-v12.0", "profiles": {}, "_load_error": "bad_schema"}
    return data


def save_operator_profiles(sessions_root: str, data: dict):
    path = _operator_profiles_path(sessions_root)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = path.with_suffix(".tmp")
        with open(temp_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        temp_path.replace(path)
    except Exception as e:
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception as e2:
            raise IOError(f"Failed to persist operator profiles: {e2}") from e


def hash_pin(pin: str, salt: str, iterations: int = 200000) -> str:
    """Hash the PIN using PBKDF2 with HMAC-SHA256."""
    h = hashlib.pbkdf2_hmac("sha256", pin.encode("utf-8"), salt.encode("utf-8"), iterations)
    return h.hex()


def verify_pin(pin: str, salt: str, pin_hash: str, iterations: int = 200000) -> bool:
    """Verify the PIN against the salt and hash using hmac.compare_digest."""
    candidate_hash = hash_pin(pin, salt, iterations)
    return hmac.compare_digest(candidate_hash.encode("utf-8"), pin_hash.encode("utf-8"))


def _mint_recovery_code() -> str:
    """Mint a fresh XXXX-XXXX-XXXX recovery code from the unambiguous alphabet."""
    def group() -> str:
        return "".join(secrets.choice(_RECOVERY_ALPHABET) for _ in range(4))
    return f"{group()}-{group()}-{group()}"


def login_operator(server, operator_id: str, pin: str) -> Tuple[int, dict]:
    """Authenticate an operator and return token or lockout info."""
    with _OPERATOR_LOCK:
        now = time.time()

        # Clean up expired sessions & sse tokens to prevent unbounded memory growth (P3-A)
        if hasattr(server, "operator_sessions"):
            expired_tokens = [
                t for t, s in server.operator_sessions.items()
                if now >= float(s.get("expires_at", 0.0))
            ]
            for t in expired_tokens:
                server.operator_sessions.pop(t, None)

        if hasattr(server, "sse_tokens"):
            expired_sse = [
                t for t, s in server.sse_tokens.items()
                if now >= float(s.get("expires_at", 0.0))
            ]
            for t in expired_sse:
                server.sse_tokens.pop(t, None)

        db = load_operator_profiles(server.sessions_root)
        profiles = db.get("profiles", {})
        if operator_id not in profiles:
            return 401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "Invalid operator ID or PIN"}}

        profile = profiles[operator_id]

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

            try:
                save_operator_profiles(server.sessions_root, db)
            except Exception as e:
                import sys
                print(f"[ERROR] Failed to save operator profiles database on successful login: {e}", file=sys.stderr)

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

            try:
                save_operator_profiles(server.sessions_root, db)
            except Exception as e:
                import sys
                print(f"[ERROR] Failed to save operator profiles database on invalid PIN attempt: {e}", file=sys.stderr)
                # Lockout is enforced in-memory regardless of save outcome

            if attempts >= 5:
                return 429, {
                    "ok": False,
                    "error": {
                        "code": "LOCKOUT_ACTIVE",
                        "message": "Too many failed attempts. Try again in 30 seconds.",
                        "retry_after_s": 30
                    }
                }
            else:
                return 401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "Invalid operator ID or PIN"}}


def _sanitize_display_name(raw: str) -> str:
    """Normalize and reject unsafe code points in an operator display name.

    Display names are echoed back in API responses, session metadata, and PDF/
    HTML reports. Stripping control characters, bidi overrides, and zero-width /
    invisible code points prevents log-spoofing, right-to-left display spoofing,
    and confusable/invisible-name attacks. Returns the normalized name, or ``""``
    if any disallowed code point is present (the caller rejects empty results).
    """
    normalized = unicodedata.normalize("NFC", raw).strip()
    for ch in normalized:
        # Reject control (Cc), format incl. bidi/zero-width (Cf), surrogate (Cs),
        # and private-use (Co) categories. Ordinary letters, marks, digits,
        # punctuation, and spaces are preserved.
        if unicodedata.category(ch) in {"Cc", "Cf", "Cs", "Co"}:
            return ""
    return normalized


def create_operator_profile(server, body: dict) -> Tuple[int, dict]:
    display_name = _sanitize_display_name(str(body.get("display_name") or ""))
    initials = str(body.get("initials") or "").strip().upper()
    pin = str(body.get("pin") or "").strip()

    if not display_name:
        return 400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "display_name is required (and must not contain control or invisible characters)"}}
    if len(display_name) > 64:
        return 400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "display_name must not exceed 64 characters"}}
    if not re.fullmatch(r"[A-Z]{2,5}", initials):
        return 400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "initials must be 2 to 5 uppercase letters"}}
    if not re.fullmatch(r"\d{6}", pin):
        return 400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "pin must be exactly 6 digits"}}

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

        # Mint recovery code — store hash only, never persist plaintext
        recovery_code = _mint_recovery_code()
        recovery_salt = secrets.token_hex(16)
        recovery_iterations = 200000
        recovery_hash_val = hash_pin(recovery_code, recovery_salt, recovery_iterations)

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
            "locked_until": 0.0,
            "recovery_hash": recovery_hash_val,
            "recovery_salt": recovery_salt,
            "recovery_iterations": recovery_iterations,
            "recovery_failed_attempts": 0,
            "recovery_locked_until": 0.0,
        }

        profiles[operator_id] = profile
        db["profiles"] = profiles
        try:
            save_operator_profiles(server.sessions_root, db)
        except Exception as e:
            import sys
            print(f"[ERROR] Failed to save operator profiles database on profile creation: {e}", file=sys.stderr)
            return 500, {"ok": False, "error": {"code": "INTERNAL_SERVER_ERROR", "message": "Failed to persist operator profile."}}

        return 200, {
            "ok": True,
            "operator": {
                "operator_id": operator_id,
                "display_name": display_name,
                "initials": initials
            },
            "recovery_code": recovery_code,
        }


def _invalidate_operator_sessions(server, operator_id: str) -> None:
    """Drop every operator session AND SSE token bound to *operator_id*.

    Callers must already hold :data:`_OPERATOR_LOCK` (it guards both
    ``server.operator_sessions`` and ``server.sse_tokens``); this helper performs
    no locking of its own so it can run inside the reset/host-reset critical
    sections that mutate the profile DB under that same lock.

    SSE tokens carry an ``operator_id`` (recorded at mint time in the
    ``/api/auth/sse-token`` handler). Without dropping them here, a short-lived
    SSE token minted by an operator would outlive a PIN reset / host-reset /
    logout for its ~30s TTL — defeating the purpose of session invalidation.
    A bootstrap-minted token has ``operator_id=None`` and is left untouched
    unless *operator_id* is also ``None``.
    """
    if hasattr(server, "operator_sessions"):
        to_drop = [t for t, s in server.operator_sessions.items() if s.get("operator_id") == operator_id]
        for t in to_drop:
            server.operator_sessions.pop(t, None)
    if hasattr(server, "sse_tokens"):
        sse_drop = [t for t, s in server.sse_tokens.items() if s.get("operator_id") == operator_id]
        for t in sse_drop:
            server.sse_tokens.pop(t, None)


def _invalidate_operator_sse_tokens(server, operator_id: str) -> None:
    """Drop only the SSE tokens bound to *operator_id*; leave sessions intact.

    This is the logout-scoped counterpart to
    :func:`_invalidate_operator_sessions`. Logging out one tab/device must revoke
    only the presented session token (popped by the caller) plus that operator's
    short-lived SSE tokens — it must NOT silently revoke the same operator's OTHER
    active sessions. So, unlike :func:`_invalidate_operator_sessions`, this helper
    deliberately leaves ``server.operator_sessions`` untouched.

    Callers must already hold :data:`_OPERATOR_LOCK` (it guards
    ``server.sse_tokens``); this helper performs no locking of its own, mirroring
    :func:`_invalidate_operator_sessions` so it can run inside the logout critical
    section without re-entrancy/deadlock.

    SSE tokens carry an ``operator_id`` recorded at mint time. A bootstrap-minted
    token has ``operator_id=None`` and is left untouched unless *operator_id* is
    also ``None``.
    """
    if hasattr(server, "sse_tokens"):
        sse_drop = [t for t, s in server.sse_tokens.items() if s.get("operator_id") == operator_id]
        for t in sse_drop:
            server.sse_tokens.pop(t, None)


def reset_pin_with_recovery(server, operator_id: str, recovery_code: str, new_pin: str) -> Tuple[int, dict]:
    """Reset a PIN using the operator's recovery code.

    On success: sets a new PIN, consumes the old recovery code and mints a
    fresh one, invalidates all existing operator sessions, and appends an
    audit log line.

    Returns ``(status, body)``.
    """
    with _OPERATOR_LOCK:
        now = time.time()

        if not re.fullmatch(r"\d{6}", str(new_pin or "")):
            return 400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "new_pin must be exactly 6 digits"}}

        db = load_operator_profiles(server.sessions_root)
        profiles = db.get("profiles", {})
        if operator_id not in profiles:
            return 401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "Invalid operator ID"}}

        profile = profiles[operator_id]

        # Check recovery lockout (separate from PIN lockout)
        recovery_locked_until = float(profile.get("recovery_locked_until", 0.0))
        if recovery_locked_until > now:
            retry_after = max(1, int(recovery_locked_until - now))
            return 429, {
                "ok": False,
                "error": {
                    "code": "LOCKOUT_ACTIVE",
                    "message": f"Too many failed recovery attempts. Try again in {retry_after} seconds.",
                    "retry_after_s": retry_after,
                },
            }

        # Legacy profile: no recovery hash present — direct to host-reset
        r_salt = profile.get("recovery_salt", "")
        r_hash = profile.get("recovery_hash", "")
        r_iters = int(profile.get("recovery_iterations", 200000))

        if not r_hash or not r_salt:
            import sys
            print(f"[AUTH] reset-pin attempted for legacy profile {operator_id} (no recovery code)", file=sys.stderr)
            return 400, {
                "ok": False,
                "error": {
                    "code": "NO_RECOVERY_CODE",
                    "message": (
                        "This operator profile was created before recovery codes were supported. "
                        "Use host-reset (POST /api/auth/host-reset) from the local machine to set a new PIN."
                    ),
                },
            }

        code_str = str(recovery_code or "").strip().upper()

        if not verify_pin(code_str, r_salt, r_hash, r_iters):
            attempts = int(profile.get("recovery_failed_attempts", 0)) + 1
            profile["recovery_failed_attempts"] = attempts
            profile["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            if attempts >= _RECOVERY_FAILURE_LIMIT:
                profile["recovery_locked_until"] = now + _RECOVERY_LOCKOUT_S
            try:
                save_operator_profiles(server.sessions_root, db)
            except Exception:
                pass
            import sys
            print(f"[AUTH] reset-pin failed recovery attempt {attempts}/{_RECOVERY_FAILURE_LIMIT} for operator {operator_id}", file=sys.stderr)
            if attempts >= _RECOVERY_FAILURE_LIMIT:
                return 429, {
                    "ok": False,
                    "error": {
                        "code": "LOCKOUT_ACTIVE",
                        "message": "Too many failed recovery attempts. Try again in 30 seconds.",
                        "retry_after_s": 30,
                    },
                }
            return 401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "Invalid recovery code"}}

        # Success — update PIN, reset PIN lockout, rotate recovery code (single-use)
        new_salt = secrets.token_hex(16)
        new_iterations = 200000
        profile["pin_hash"] = hash_pin(str(new_pin), new_salt, new_iterations)
        profile["pin_salt"] = new_salt
        profile["pin_iterations"] = new_iterations
        profile["failed_attempts"] = 0
        profile["locked_until"] = 0.0

        new_recovery_code = _mint_recovery_code()
        new_r_salt = secrets.token_hex(16)
        new_r_iterations = 200000
        profile["recovery_hash"] = hash_pin(new_recovery_code, new_r_salt, new_r_iterations)
        profile["recovery_salt"] = new_r_salt
        profile["recovery_iterations"] = new_r_iterations
        profile["recovery_failed_attempts"] = 0
        profile["recovery_locked_until"] = 0.0
        profile["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

        try:
            save_operator_profiles(server.sessions_root, db)
        except Exception as e:
            import sys
            print(f"[ERROR] Failed to save profiles after PIN reset: {e}", file=sys.stderr)
            return 500, {"ok": False, "error": {"code": "INTERNAL_SERVER_ERROR", "message": "Failed to persist PIN reset."}}

        _invalidate_operator_sessions(server, operator_id)

        import sys
        print(f"[AUTH] PIN reset via recovery code for operator {operator_id}", file=sys.stderr)

        return 200, {
            "ok": True,
            "recovery_code": new_recovery_code,
        }


def host_reset_pin(server, operator_id: str, new_pin: str) -> Tuple[int, dict]:
    """Reset a PIN from loopback (no recovery code required).

    Caller is responsible for ensuring the request arrived on loopback.
    On success: sets new PIN, clears PIN lockout, rotates recovery code,
    invalidates existing sessions, appends audit log.
    """
    with _OPERATOR_LOCK:
        if not re.fullmatch(r"\d{6}", str(new_pin or "")):
            return 400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "new_pin must be exactly 6 digits"}}

        db = load_operator_profiles(server.sessions_root)
        profiles = db.get("profiles", {})
        if operator_id not in profiles:
            return 401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "Invalid operator ID"}}

        profile = profiles[operator_id]

        new_salt = secrets.token_hex(16)
        new_iterations = 200000
        profile["pin_hash"] = hash_pin(str(new_pin), new_salt, new_iterations)
        profile["pin_salt"] = new_salt
        profile["pin_iterations"] = new_iterations
        profile["failed_attempts"] = 0
        profile["locked_until"] = 0.0

        new_recovery_code = _mint_recovery_code()
        new_r_salt = secrets.token_hex(16)
        new_r_iterations = 200000
        profile["recovery_hash"] = hash_pin(new_recovery_code, new_r_salt, new_r_iterations)
        profile["recovery_salt"] = new_r_salt
        profile["recovery_iterations"] = new_r_iterations
        profile["recovery_failed_attempts"] = 0
        profile["recovery_locked_until"] = 0.0
        profile["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

        try:
            save_operator_profiles(server.sessions_root, db)
        except Exception as e:
            import sys
            print(f"[ERROR] Failed to save profiles after host PIN reset: {e}", file=sys.stderr)
            return 500, {"ok": False, "error": {"code": "INTERNAL_SERVER_ERROR", "message": "Failed to persist host PIN reset."}}

        _invalidate_operator_sessions(server, operator_id)

        import sys
        print(f"[AUTH] PIN reset via loopback host-reset for operator {operator_id}", file=sys.stderr)

        return 200, {
            "ok": True,
            "recovery_code": new_recovery_code,
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
    "reset_pin_with_recovery",
    "host_reset_pin",
    "_invalidate_operator_sessions",
    "_invalidate_operator_sse_tokens",
    "_OPERATOR_LOCK",
    "_mint_recovery_code",
    "_RECOVERY_ALPHABET",
    "_RECOVERY_FAILURE_LIMIT",
    "_RECOVERY_LOCKOUT_S",
    "_PIN_TTL_S",
    "_PIN_MAX",
    "_PAIR_FAILURE_LIMIT",
    "_PAIR_FAILURE_WINDOW_S",
    "_PAIR_LOCKOUT_S",
]
