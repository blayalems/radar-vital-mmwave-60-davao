"""Unit tests for rvt_trainer.api.auth — PIN minting, single-use exchange,
expiry, and store-size bounds.

These exercise the helpers directly with a hand-rolled server stub. The
full HTTP path is covered by the Playwright smoke suite; this is the
fast feedback loop for the auth logic itself.
"""

from __future__ import annotations

import time

import pytest

from rvt_trainer.api.auth import (
    _PAIR_FAILURE_LIMIT,
    _PAIR_LOCKOUT_S,
    _PIN_MAX,
    _PIN_TTL_S,
    exchange_pair_pin,
    make_pair_pin,
)


class _StubServer:
    """Minimal stand-in for the trainer's HTTPServer subclass."""

    def __init__(self) -> None:
        self.pair_pins: dict = {}
        self.auth_tokens: set = set()
        self.active_pin: str = ""
        self.active_pin_expires_at: float = 0.0
        self.pair_exchange_failures: dict = {}


def test_make_pair_pin_format():
    server = _StubServer()
    pin = make_pair_pin(server)
    assert len(pin) == 6
    assert pin.isdigit()
    assert pin in server.pair_pins
    assert server.active_pin == pin


def test_exchange_returns_token_then_consumes_pin():
    server = _StubServer()
    pin = make_pair_pin(server)
    status, body = exchange_pair_pin(server, pin)
    assert status == 200
    assert body["ok"] is True
    assert body["token_type"] == "RVT-Token"
    assert body["token"] in server.auth_tokens
    assert pin not in server.pair_pins
    assert server.active_pin == ""


def test_exchange_replay_returns_410():
    server = _StubServer()
    pin = make_pair_pin(server)
    exchange_pair_pin(server, pin)
    status, body = exchange_pair_pin(server, pin)
    assert status == 410
    assert body["error"]["code"] == "PIN_EXPIRED_OR_USED"


def test_exchange_unknown_pin_returns_410():
    server = _StubServer()
    status, body = exchange_pair_pin(server, "999999")
    assert status == 410
    assert body["error"]["code"] == "PIN_EXPIRED_OR_USED"


def test_repeated_invalid_exchange_locks_out_client_without_consuming_valid_pin():
    server = _StubServer()
    pin = make_pair_pin(server)
    for _ in range(_PAIR_FAILURE_LIMIT - 1):
        status, _ = exchange_pair_pin(server, "999999", "phone-a")
        assert status == 410

    status, body = exchange_pair_pin(server, "999999", "phone-a")
    assert status == 429
    assert body["error"]["code"] == "PAIRING_RATE_LIMITED"
    assert body["error"]["retry_after_s"] == int(_PAIR_LOCKOUT_S)

    status, _ = exchange_pair_pin(server, pin, "phone-a")
    assert status == 429
    assert pin in server.pair_pins


def test_successful_exchange_clears_client_failure_history():
    server = _StubServer()
    pin = make_pair_pin(server)
    assert exchange_pair_pin(server, "999999", "phone-a")[0] == 410
    assert exchange_pair_pin(server, pin, "phone-a")[0] == 200
    assert "phone-a" not in server.pair_exchange_failures


def test_expired_pin_returns_410_and_clears_entry():
    server = _StubServer()
    pin = make_pair_pin(server)
    # Force the PIN to look expired by rewinding its expires_at.
    server.pair_pins[pin]["expires_at"] = time.time() - 1.0
    status, body = exchange_pair_pin(server, pin)
    assert status == 410
    assert body["error"]["code"] == "PIN_EXPIRED"
    assert pin not in server.pair_pins


def test_store_is_bounded_to_pin_max():
    server = _StubServer()
    # Mint _PIN_MAX + 2 fresh PINs; the oldest should be evicted to keep
    # len(pair_pins) <= _PIN_MAX.
    minted = [make_pair_pin(server) for _ in range(_PIN_MAX + 2)]
    assert len(server.pair_pins) <= _PIN_MAX
    # The most recent PINs survive; the oldest two were evicted.
    assert minted[-1] in server.pair_pins
    assert minted[0] not in server.pair_pins


def test_token_persists_until_process_exit_marker():
    server = _StubServer()
    pin = make_pair_pin(server)
    _, body = exchange_pair_pin(server, pin)
    assert body["expires"] == "process"


def test_pin_ttl_constants_sane():
    # Plan §5: PIN expiry = 5 min, max outstanding = 8.
    assert _PIN_TTL_S == 300
    assert _PIN_MAX == 8


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
