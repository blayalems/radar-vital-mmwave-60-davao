"""Pairing PIN and bearer-token helpers for the LAN-bind trainer.

The trainer exposes two related primitives:

* :func:`make_pair_pin` — mints a fresh 6-digit numeric PIN, tracked in a
  per-process TTL store on ``server.pair_pins``. PINs expire after
  ``_PIN_TTL_S`` seconds; the store is bounded to ``_PIN_MAX`` outstanding
  entries (oldest evicted first).
* :func:`exchange_pair_pin` — single-use redemption. On success the PIN is
  deleted from the store *immediately* and a 32-byte URL-safe token is added
  to ``server.auth_tokens``. Replay of a consumed or expired PIN returns
  ``410 Gone``.

The token store lives on the server instance for the lifetime of the
process; restart-on-restart re-pairing is expected and documented in
``/pair``.
"""

from __future__ import annotations

import secrets
import time
from typing import Dict, Tuple

_PIN_TTL_S = 300       # 5 minute PIN expiry
_PIN_MAX = 8           # max concurrent outstanding PINs


def make_pair_pin(server) -> str:
    """Mint, store, and return a fresh 6-digit pairing PIN.

    Expired entries are reaped first, then the store is trimmed to
    ``_PIN_MAX`` by evicting the oldest entry.
    """
    pin = f"{secrets.randbelow(1000000):06d}"
    now = time.time()
    pins = getattr(server, "pair_pins", {})
    expired = [k for k, item in pins.items() if float(item.get("expires_at", 0.0)) <= now]
    for key in expired:
        pins.pop(key, None)
    while len(pins) >= _PIN_MAX:
        oldest = min(pins, key=lambda k: float(pins[k].get("created_at", 0.0)))
        pins.pop(oldest, None)
    pins[pin] = {"created_at": now, "expires_at": now + _PIN_TTL_S}
    server.pair_pins = pins
    server.active_pin = pin
    server.active_pin_expires_at = now + _PIN_TTL_S
    return pin


def exchange_pair_pin(server, pin: str) -> Tuple[int, Dict[str, object]]:
    """Redeem ``pin`` for an opaque bearer token.

    Returns a ``(status, body)`` tuple suitable for the HTTP handler:

    * ``200`` + ``{"ok": True, "token": ..., "token_type": "RVT-Token", "expires": "process"}``
      on success. The PIN is consumed; replay yields ``410``.
    * ``410`` + ``{"ok": False, "error": {...}}`` if the PIN is unknown or
      expired.

    The token is added to ``server.auth_tokens`` (set) on success and stays
    valid for the trainer process lifetime.
    """
    now = time.time()
    pins = getattr(server, "pair_pins", {})
    item = pins.get(str(pin or ""))
    if not item:
        return 410, {
            "ok": False,
            "error": {
                "code": "PIN_EXPIRED_OR_USED",
                "message": "pairing PIN expired or was already used",
            },
        }
    if float(item.get("expires_at", 0.0)) <= now:
        pins.pop(str(pin), None)
        return 410, {
            "ok": False,
            "error": {"code": "PIN_EXPIRED", "message": "pairing PIN expired"},
        }
    pins.pop(str(pin), None)
    if getattr(server, "active_pin", "") == str(pin):
        server.active_pin = ""
        server.active_pin_expires_at = 0.0
    token = secrets.token_urlsafe(24)
    server.auth_tokens.add(token)
    return 200, {
        "ok": True,
        "token": token,
        "token_type": "RVT-Token",
        "expires": "process",
    }


__all__ = ["make_pair_pin", "exchange_pair_pin", "_PIN_TTL_S", "_PIN_MAX"]
