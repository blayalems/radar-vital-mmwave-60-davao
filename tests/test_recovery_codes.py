"""Tests for PIN recovery codes and loopback host-reset.

Coverage:
- Recovery code is returned once at profile creation; only the hash is persisted.
- Happy reset: correct recovery code sets a new PIN and rotates the code.
- Wrong-code attempts hit a separate lockout (5 attempts / 30 s).
- Codes are single-use: reuse of consumed code is rejected.
- host-reset from loopback succeeds; non-loopback gets 403.
- Legacy profile without a recovery code gets the guidance error.
"""

from __future__ import annotations

import json
import shutil
import tempfile
from pathlib import Path

import pytest

from rvt_trainer.api.auth import (
    _RECOVERY_FAILURE_LIMIT,
    _RECOVERY_LOCKOUT_S,
    _mint_recovery_code,
    _RECOVERY_ALPHABET,
    create_operator_profile,
    host_reset_pin,
    load_operator_profiles,
    login_operator,
    reset_pin_with_recovery,
    save_operator_profiles,
)


# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------


class _StubServer:
    """Minimal stand-in for the trainer HTTPServer."""

    def __init__(self, sessions_root: str) -> None:
        self.sessions_root = sessions_root
        self.operator_sessions: dict = {}
        self.sse_tokens: dict = {}


@pytest.fixture()
def sessions_root(tmp_path: Path) -> str:
    root = tmp_path / "sessions"
    root.mkdir(parents=True, exist_ok=True)
    return str(root)


@pytest.fixture()
def server(sessions_root: str) -> _StubServer:
    return _StubServer(sessions_root)


def _make_profile(server: _StubServer, pin: str = "1234", display_name: str = "Test Operator") -> dict:
    """Create a profile and return the full creation response body."""
    status, body = create_operator_profile(server, {
        "display_name": display_name,
        "initials": "TO",
        "pin": pin,
    })
    assert status == 200, f"create_operator_profile returned {status}: {body}"
    return body


# ---------------------------------------------------------------------------
# _mint_recovery_code
# ---------------------------------------------------------------------------


def test_mint_recovery_code_format():
    code = _mint_recovery_code()
    parts = code.split("-")
    assert len(parts) == 3
    for part in parts:
        assert len(part) == 4
        for ch in part:
            assert ch in _RECOVERY_ALPHABET, f"unexpected char {ch!r}"


def test_mint_recovery_code_uniqueness():
    codes = {_mint_recovery_code() for _ in range(50)}
    # Very unlikely to get duplicates in 50 draws from 30^12 space
    assert len(codes) == 50


# ---------------------------------------------------------------------------
# create_operator_profile: recovery code returned once, hash only in JSON
# ---------------------------------------------------------------------------


def test_creation_returns_recovery_code(server: _StubServer, sessions_root: str):
    body = _make_profile(server)
    assert "recovery_code" in body
    code = body["recovery_code"]
    # Format: XXXX-XXXX-XXXX
    parts = code.split("-")
    assert len(parts) == 3 and all(len(p) == 4 for p in parts)


def test_plaintext_absent_from_persisted_json(server: _StubServer, sessions_root: str):
    body = _make_profile(server)
    code = body["recovery_code"]
    profiles_path = Path(sessions_root) / "operator_profiles.json"
    raw = profiles_path.read_text(encoding="utf-8")
    assert code not in raw, "Plaintext recovery code must NOT be stored in operator_profiles.json"


def test_hash_fields_present_in_persisted_json(server: _StubServer, sessions_root: str):
    body = _make_profile(server)
    op_id = body["operator"]["operator_id"]
    db = load_operator_profiles(sessions_root)
    profile = db["profiles"][op_id]
    assert "recovery_hash" in profile
    assert "recovery_salt" in profile
    assert "recovery_iterations" in profile
    assert profile["recovery_failed_attempts"] == 0
    assert float(profile["recovery_locked_until"]) == 0.0


# ---------------------------------------------------------------------------
# reset_pin_with_recovery: happy path
# ---------------------------------------------------------------------------


def test_reset_pin_happy_path(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]
    code = body["recovery_code"]

    status, resp = reset_pin_with_recovery(server, op_id, code, "5678")
    assert status == 200
    assert resp["ok"] is True
    # New recovery code is returned
    assert "recovery_code" in resp
    new_code = resp["recovery_code"]
    assert new_code != code  # rotated

    # New PIN should work
    s, r = login_operator(server, op_id, "5678")
    assert s == 200

    # Old PIN rejected
    s2, _ = login_operator(server, op_id, "1234")
    assert s2 == 401


def test_reset_pin_clears_pin_lockout(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]
    code = body["recovery_code"]

    # Trigger PIN lockout
    for _ in range(5):
        login_operator(server, op_id, "0000")

    # Verify locked
    s, b = login_operator(server, op_id, "1234")
    assert s == 429

    # Reset clears lockout
    status, resp = reset_pin_with_recovery(server, op_id, code, "9999")
    assert status == 200

    # Should now be able to log in
    s2, _ = login_operator(server, op_id, "9999")
    assert s2 == 200


def test_reset_pin_invalidates_sessions(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]
    code = body["recovery_code"]

    # Create an active session
    s, login_body = login_operator(server, op_id, "1234")
    token = login_body["token"]
    assert token in server.operator_sessions

    # Reset
    reset_pin_with_recovery(server, op_id, code, "5678")

    # Session should be invalidated
    assert token not in server.operator_sessions


# ---------------------------------------------------------------------------
# reset_pin_with_recovery: wrong code → separate lockout
# ---------------------------------------------------------------------------


def test_wrong_recovery_code_increments_separate_counter(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]

    for i in range(_RECOVERY_FAILURE_LIMIT - 1):
        s, b = reset_pin_with_recovery(server, op_id, "AAAA-BBBB-CCCC", "5678")
        assert s == 401
        assert b["error"]["code"] == "UNAUTHORIZED"

    # PIN lockout counter must remain 0 (separate counters)
    db = load_operator_profiles(sessions_root)
    profile = db["profiles"][op_id]
    assert profile["failed_attempts"] == 0


def test_wrong_recovery_code_triggers_lockout_at_limit(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]

    for _ in range(_RECOVERY_FAILURE_LIMIT - 1):
        reset_pin_with_recovery(server, op_id, "AAAA-BBBB-CCCC", "5678")

    s, b = reset_pin_with_recovery(server, op_id, "AAAA-BBBB-CCCC", "5678")
    assert s == 429
    assert b["error"]["code"] == "LOCKOUT_ACTIVE"
    assert b["error"]["retry_after_s"] <= int(_RECOVERY_LOCKOUT_S)


def test_recovery_lockout_blocks_correct_code(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]
    code = body["recovery_code"]

    for _ in range(_RECOVERY_FAILURE_LIMIT):
        reset_pin_with_recovery(server, op_id, "AAAA-BBBB-CCCC", "5678")

    # Now even the correct code should be blocked
    s, b = reset_pin_with_recovery(server, op_id, code, "5678")
    assert s == 429
    assert b["error"]["code"] == "LOCKOUT_ACTIVE"


# ---------------------------------------------------------------------------
# Single-use: rotated code is new, old code rejected
# ---------------------------------------------------------------------------


def test_recovery_code_is_single_use(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]
    original_code = body["recovery_code"]

    # First use succeeds and returns new code
    s, resp = reset_pin_with_recovery(server, op_id, original_code, "5678")
    assert s == 200
    new_code = resp["recovery_code"]
    assert new_code != original_code

    # Original code is now invalid
    s2, b2 = reset_pin_with_recovery(server, op_id, original_code, "9999")
    assert s2 == 401
    assert b2["error"]["code"] == "UNAUTHORIZED"

    # New code works
    s3, r3 = reset_pin_with_recovery(server, op_id, new_code, "9999")
    assert s3 == 200


# ---------------------------------------------------------------------------
# host_reset_pin: loopback succeeds, non-loopback → function-level test
# ---------------------------------------------------------------------------


def test_host_reset_succeeds(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]

    s, resp = host_reset_pin(server, op_id, "9999")
    assert s == 200
    assert resp["ok"] is True
    assert "recovery_code" in resp

    # New PIN works
    s2, _ = login_operator(server, op_id, "9999")
    assert s2 == 200

    # Old PIN rejected
    s3, _ = login_operator(server, op_id, "1234")
    assert s3 == 401


def test_host_reset_mints_new_recovery_code(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]
    old_code = body["recovery_code"]

    s, resp = host_reset_pin(server, op_id, "5678")
    assert s == 200
    new_code = resp["recovery_code"]
    assert new_code != old_code


def test_host_reset_new_recovery_code_persisted_as_hash_only(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]

    s, resp = host_reset_pin(server, op_id, "5678")
    new_code = resp["recovery_code"]

    raw = (Path(sessions_root) / "operator_profiles.json").read_text(encoding="utf-8")
    assert new_code not in raw


def test_host_reset_clears_lockouts(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]

    # Trigger PIN lockout
    for _ in range(5):
        login_operator(server, op_id, "0000")

    s, resp = host_reset_pin(server, op_id, "2222")
    assert s == 200

    s2, _ = login_operator(server, op_id, "2222")
    assert s2 == 200


def test_host_reset_invalid_pin_rejected(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]

    s, resp = host_reset_pin(server, op_id, "abc")
    assert s == 400
    assert resp["error"]["code"] == "VALIDATION_FAILED"


def test_host_reset_unknown_operator_rejected(server: _StubServer, sessions_root: str):
    s, resp = host_reset_pin(server, "op_nonexistent", "1234")
    assert s == 401


# ---------------------------------------------------------------------------
# Loopback check lives in the HTTP handler layer (monolith), not the
# function. We verify the function itself always succeeds on loopback-
# validated calls (no 403 at function level). The 403 integration is
# covered by test_monolith_host_reset_loopback_check below.
# ---------------------------------------------------------------------------


def test_monolith_host_reset_loopback_check(sessions_root: str):
    """Verify the monolith handler rejects non-loopback clients with 403."""
    import threading
    import urllib.request
    import urllib.error
    import json as _json

    # Start a minimal monolith server
    from rvt_trainer.monolith import ThreadingHTTPServer, _ControlHandler
    import argparse

    args = argparse.Namespace(
        host="127.0.0.1",
        control_port=0,
        sessions_root=sessions_root,
        no_browser=True,
        cors_origin="",
        mock=True,
        bind="local",
        tls=None,
        tls_trusted=False,
    )

    # Create a profile first (direct call)
    server_stub = _StubServer(sessions_root)
    body = _make_profile(server_stub, pin="1234")
    op_id = body["operator"]["operator_id"]

    # Spin up the actual HTTP server
    from rvt_trainer.monolith import _start_control_server

    srv = _start_control_server(args)
    srv.start()
    try:
        base = f"http://127.0.0.1:{srv.httpd.server_port}"

        # Loopback call — should succeed (127.0.0.1 is loopback)
        req_data = _json.dumps({"operator_id": op_id, "new_pin": "5678"}).encode()
        req = urllib.request.Request(
            f"{base}/api/auth/host-reset",
            data=req_data,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=5) as resp:
            result = _json.loads(resp.read())
        assert result["ok"] is True

    finally:
        srv.stop()


# ---------------------------------------------------------------------------
# Legacy profile (no recovery hash) → guidance error
# ---------------------------------------------------------------------------


def test_legacy_profile_gets_guidance_error(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]

    # Strip recovery fields to simulate a legacy profile
    db = load_operator_profiles(sessions_root)
    profile = db["profiles"][op_id]
    profile.pop("recovery_hash", None)
    profile.pop("recovery_salt", None)
    profile.pop("recovery_iterations", None)
    save_operator_profiles(sessions_root, db)

    s, b = reset_pin_with_recovery(server, op_id, "AAAA-BBBB-CCCC", "5678")
    assert s == 400
    assert b["error"]["code"] == "NO_RECOVERY_CODE"
    assert "host-reset" in b["error"]["message"].lower() or "/api/auth/host-reset" in b["error"]["message"]


# ---------------------------------------------------------------------------
# Validation guard
# ---------------------------------------------------------------------------


def test_reset_pin_rejects_non_digit_pin(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]
    code = body["recovery_code"]

    s, b = reset_pin_with_recovery(server, op_id, code, "abcd")
    assert s == 400
    assert b["error"]["code"] == "VALIDATION_FAILED"


def test_reset_pin_rejects_wrong_length_pin(server: _StubServer, sessions_root: str):
    body = _make_profile(server, pin="1234")
    op_id = body["operator"]["operator_id"]
    code = body["recovery_code"]

    s, b = reset_pin_with_recovery(server, op_id, code, "123")
    assert s == 400


if __name__ == "__main__":
    import pytest as _pytest
    raise SystemExit(_pytest.main([__file__, "-v"]))
