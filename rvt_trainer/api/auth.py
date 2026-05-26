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


__all__ = [
    "make_pair_pin",
    "exchange_pair_pin",
    "_PIN_TTL_S",
    "_PIN_MAX",
    "_PAIR_FAILURE_LIMIT",
    "_PAIR_FAILURE_WINDOW_S",
    "_PAIR_LOCKOUT_S",
]
