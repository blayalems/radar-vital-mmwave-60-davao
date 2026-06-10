"""Unit tests for operator profile persistence, authentication, rate limiting and gating logic."""

import pytest
import time
import tempfile
import shutil
import json
from pathlib import Path
from rvt_trainer.api.auth import (
    hash_pin,
    verify_pin,
    create_operator_profile,
    login_operator,
    load_operator_profiles,
    save_operator_profiles,
)
from rvt_trainer.monolith import _ControlHandler

class MockServer:
    def __init__(self, sessions_root: str, bind_mode: str = "local"):
        self.sessions_root = sessions_root
        self.bind_mode = bind_mode
        self.auth_tokens = set()
        self.operator_sessions = {}
        self.sse_tokens = {}

class MockHandler:
    def __init__(self, server: MockServer, path: str, command: str = "GET", headers: dict = None):
        self.server = server
        self.path = path
        self.command = command
        self.headers = headers or {}
        self.sent_responses = []
        self.current_operator_id = None

    def _send_json(self, status: int, obj):
        self.sent_responses.append((status, obj))

    def _require_control_auth(self) -> bool:
        return _ControlHandler._require_control_auth(self)


@pytest.fixture
def temp_sessions_root():
    tmpdir = tempfile.mkdtemp()
    sessions_dir = Path(tmpdir) / "sessions"
    sessions_dir.mkdir(parents=True, exist_ok=True)
    yield str(sessions_dir)
    shutil.rmtree(tmpdir)


def test_pbkdf2_hashing_format():
    # Verify PIN hashing and matching works correctly
    salt = "d3f4b2a1a1f3e8c9"
    pin = "1234"
    h = hash_pin(pin, salt)
    assert len(h) == 64  # sha256 output hex length is 64
    assert verify_pin(pin, salt, h)
    assert not verify_pin("4321", salt, h)


def test_bootstrapping_and_profile_creation(temp_sessions_root):
    server = MockServer(temp_sessions_root)

    # Initially 0 profiles exist
    db = load_operator_profiles(temp_sessions_root)
    assert len(db.get("profiles", {})) == 0

    # Create first profile (bootstrapping)
    body = {"display_name": "Dr. Sarah Connor", "initials": "SC", "pin": "1234"}
    status, payload = create_operator_profile(server, body)
    assert status == 200
    assert payload["ok"] is True
    operator_id = payload["operator"]["operator_id"]
    assert operator_id.startswith("op_")

    # DB should now contain 1 profile
    db = load_operator_profiles(temp_sessions_root)
    assert len(db.get("profiles", {})) == 1
    profile = db["profiles"][operator_id]
    assert profile["display_name"] == "Dr. Sarah Connor"
    assert profile["initials"] == "SC"
    assert "pin_hash" in profile
    assert profile["failed_attempts"] == 0

    # Subsequent profile creation is protected but functions correctly
    body2 = {"display_name": "John Connor", "initials": "JC", "pin": "5678"}
    status2, payload2 = create_operator_profile(server, body2)
    assert status2 == 200
    assert payload2["ok"] is True


def test_login_success_and_brute_force_lockout(temp_sessions_root):
    server = MockServer(temp_sessions_root)

    # Create a profile
    body = {"display_name": "Sarah Connor", "initials": "SC", "pin": "1234"}
    status, payload = create_operator_profile(server, body)
    op_id = payload["operator"]["operator_id"]

    # Test login success (should return 200 and operator_session_token)
    status_login, body_login = login_operator(server, op_id, "1234")
    assert status_login == 200
    token = body_login["token"]
    assert token in server.operator_sessions
    # TTL: 8 hours
    expires_at = server.operator_sessions[token]["expires_at"]
    assert expires_at - time.time() > 7.9 * 3600

    # Test login failure increments failed_attempts
    # First, let's check with some failed attempts
    for i in range(4):
        status_fail, body_fail = login_operator(server, op_id, "wrong_pin")
        assert status_fail == 401
        assert body_fail["ok"] is False

    # 5th attempt fails and triggers lockout
    status_lock, body_lock = login_operator(server, op_id, "wrong_pin")
    assert status_lock == 429
    assert body_lock["error"]["code"] == "LOCKOUT_ACTIVE"
    assert body_lock["error"]["retry_after_s"] == 30

    # While locked, even correct PIN returns 429 Lockout
    status_locked, body_locked = login_operator(server, op_id, "1234")
    assert status_locked == 429
    assert body_locked["error"]["code"] == "LOCKOUT_ACTIVE"


def test_gating_rules(temp_sessions_root):
    server = MockServer(temp_sessions_root, bind_mode="lan")

    # Gating - Public Endpoints (always allowed without any headers/params)
    for path in ["/api/health", "/api/version", "/api/update/manifest"]:
        handler = MockHandler(server, path, "GET")
        assert handler._require_control_auth() is True
        assert len(handler.sent_responses) == 0

    # Gating - Discovery Endpoints
    # In LAN mode: requires pairing token or operator session token
    # Let's try GET /api/server-info without token -> fails with 401
    handler = MockHandler(server, "/api/server-info", "GET")
    assert handler._require_control_auth() is False
    assert len(handler.sent_responses) == 1
    assert handler.sent_responses[0][0] == 401

    # Add pairing token to server and retry GET /api/server-info with token
    server.auth_tokens.add("pair_tok_123")
    handler = MockHandler(server, "/api/server-info", "GET", {"X-RVT-Auth": "pair_tok_123"})
    assert handler._require_control_auth() is True
    assert len(handler.sent_responses) == 0

    # Gating - Sensitive Endpoints
    # /api/session/start requires operator session token, pairing token is not enough
    handler = MockHandler(server, "/api/session/start", "POST", {"X-RVT-Auth": "pair_tok_123"})
    assert handler._require_control_auth() is False
    assert handler.sent_responses[0][0] == 401

    # Add valid operator session token
    server.operator_sessions["op_tok_123"] = {
        "operator_id": "op_01",
        "expires_at": time.time() + 1000.0,
    }
    handler = MockHandler(server, "/api/session/start", "POST", {"X-RVT-Auth": "op_tok_123"})
    assert handler._require_control_auth() is True

    # Test SSE query param validation
    # Without token -> 401
    handler = MockHandler(server, "/api/events/subscribe", "GET")
    assert handler._require_control_auth() is False

    # With SSE token -> allowed (and consumed)
    server.sse_tokens["sse_tok_123"] = {"expires_at": time.time() + 30.0}
    handler = MockHandler(server, "/api/events/subscribe?token=sse_tok_123", "GET")
    assert handler._require_control_auth() is True
    assert "sse_tok_123" not in server.sse_tokens


def test_operator_profiles_migration_and_save_failures(temp_sessions_root):
    # Test migration from old path (parent of sessions_root)
    old_path = Path(temp_sessions_root).resolve().parent / "operator_profiles.json"
    dummy_data = {
        "schema_version": "rvt-operator-profiles-v12.0",
        "profiles": {
            "op_migrated": {
                "operator_id": "op_migrated",
                "display_name": "Migrated Operator",
                "initials": "MO",
                "pin_hash": "dummy_hash",
                "pin_salt": "dummy_salt",
                "failed_attempts": 0,
                "locked_until": 0.0
            }
        }
    }
    
    with open(old_path, "w", encoding="utf-8") as f:
        json.dump(dummy_data, f)
        
    # Loading profiles should trigger migration
    db = load_operator_profiles(temp_sessions_root)
    assert "op_migrated" in db.get("profiles", {})
    assert not old_path.exists()  # Old path should be deleted
    
    # Save failures check
    with pytest.raises(IOError):
        save_operator_profiles("D:\\invalid?dir\\nonexistent", dummy_data)


def test_create_operator_profile_display_name_validation(temp_sessions_root):
    server = MockServer(temp_sessions_root)
    # Long display name (e.g. 65 chars)
    long_name = "a" * 65
    body = {"display_name": long_name, "initials": "SC", "pin": "1234"}
    status, payload = create_operator_profile(server, body)
    assert status == 400
    assert payload["error"]["code"] == "VALIDATION_FAILED"


def test_lockout_enforced_even_when_save_fails(temp_sessions_root):
    """P2-B: lockout 429 must fire even when save_operator_profiles raises."""
    server = MockServer(temp_sessions_root)

    # Create a profile with PIN 1234
    body = {"display_name": "Test Operator", "initials": "TO", "pin": "1234"}
    status, payload = create_operator_profile(server, body)
    assert status == 200
    op_id = payload["operator"]["operator_id"]

    # Burn through 4 bad PINs
    for _ in range(4):
        s, _ = login_operator(server, op_id, "0000")
        assert s == 401

    # Patch save_operator_profiles to always raise
    import unittest.mock as mock
    with mock.patch("rvt_trainer.api.auth.save_operator_profiles", side_effect=IOError("disk full")):
        # 5th bad PIN should still return 429 lockout, not 401
        s, b = login_operator(server, op_id, "0000")
        assert s == 429, f"Expected 429 lockout but got {s}"
        assert b["error"]["code"] == "LOCKOUT_ACTIVE"


def test_help_schema_is_public(temp_sessions_root):
    """P1-B: /api/help/schema should bypass auth like /api/health."""
    server = MockServer(temp_sessions_root)
    handler = MockHandler(server, "/api/help/schema", "GET")
    assert handler._require_control_auth() is True
    assert len(handler.sent_responses) == 0
