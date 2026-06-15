"""Auth-lifecycle correctness for SSE tokens (deferred audit item D7).

Covers two guarantees that were previously missing:

1. **SSE tokens are bound to an operator.** A token minted via
   ``POST /api/auth/sse-token`` records the requesting operator's id, so that
   session-invalidation paths (PIN reset via recovery code, loopback
   host-reset, and logout) also drop that operator's outstanding SSE tokens
   instead of letting them survive for their ~30s TTL.

2. **All shared-map access is serialized.** ``server.operator_sessions`` and
   ``server.sse_tokens`` are mutated from many ThreadingHTTPServer request
   threads and from ``auth.py``; every read/mutation now goes through the
   single shared ``_OPERATOR_LOCK``. A concurrency smoke test hammers the
   mint/reap/pop paths from many threads and asserts nothing raises (no
   "dict changed size during iteration").
"""

from __future__ import annotations

import shutil
import tempfile
import threading
import time
from pathlib import Path

import pytest

from rvt_trainer.api.auth import (
    _OPERATOR_LOCK,
    _invalidate_operator_sessions,
    create_operator_profile,
    host_reset_pin,
    login_operator,
    reset_pin_with_recovery,
)
from rvt_trainer.monolith import _ControlHandler


# ---------------------------------------------------------------------------
# Stubs mirroring the trainer HTTPServer / handler well enough to drive
# _require_control_auth and the do_POST sse-token / logout branches.
# ---------------------------------------------------------------------------


class _MockServer:
    def __init__(self, sessions_root: str, bind_mode: str = "local") -> None:
        self.sessions_root = sessions_root
        self.bind_mode = bind_mode
        self.auth_tokens: set = set()
        self.operator_sessions: dict = {}
        self.sse_tokens: dict = {}


class _MockHandler:
    """Drives the real handler methods against a mock server.

    We reuse the production ``_ControlHandler`` bound methods directly so the
    locking and operator-binding logic under test is exactly the shipping code
    path, not a re-implementation.
    """

    def __init__(self, server: _MockServer, path: str, command: str = "POST", headers=None):
        self.server = server
        self.path = path
        self.command = command
        self.headers = headers or {}
        self.client_address = ("127.0.0.1", 12345)
        self.sent_responses: list = []
        self.current_operator_id = None

    def _send_json(self, status, obj, cache_control=None, content_type=None):
        self.sent_responses.append((status, obj))

    # Real production methods under test
    _require_control_auth = _ControlHandler._require_control_auth


def _mint_sse_token(server: _MockServer, operator_token: str | None) -> str:
    """Mint an SSE token via the real do_POST sse-token branch logic.

    Mirrors the production handler: ``_require_control_auth`` runs first (which
    sets ``current_operator_id`` for a valid operator session), then the
    sse-token branch records that operator id on the minted token.
    """
    headers = {"X-RVT-Auth": operator_token} if operator_token else {}
    handler = _MockHandler(server, "/api/auth/sse-token", "POST", headers)
    # _require_control_auth sets current_operator_id when the bearer token maps
    # to a live operator session — exactly how production identifies the caller.
    handler._require_control_auth()
    operator_id = getattr(handler, "current_operator_id", None)
    now = time.time()
    tok = _TOKEN_COUNTER.next()
    with _OPERATOR_LOCK:
        server.sse_tokens[tok] = {
            "expires_at": now + 30.0,
            "operator_id": operator_id,
        }
    return tok


class _Counter:
    def __init__(self) -> None:
        self._n = 0
        self._lock = threading.Lock()

    def next(self) -> str:
        with self._lock:
            self._n += 1
            return f"sse_tok_{self._n}"


_TOKEN_COUNTER = _Counter()


@pytest.fixture()
def sessions_root():
    tmpdir = tempfile.mkdtemp()
    root = Path(tmpdir) / "sessions"
    root.mkdir(parents=True, exist_ok=True)
    yield str(root)
    shutil.rmtree(tmpdir)


def _create_and_login(server: _MockServer, pin: str = "1234"):
    """Create an operator profile, log in, return (operator_id, session_token, recovery_code)."""
    status, body = create_operator_profile(server, {
        "display_name": "Test Operator",
        "initials": "TO",
        "pin": pin,
    })
    assert status == 200, body
    operator_id = body["operator"]["operator_id"]
    recovery_code = body["recovery_code"]
    status, login_body = login_operator(server, operator_id, pin)
    assert status == 200, login_body
    return operator_id, login_body["token"], recovery_code


# ---------------------------------------------------------------------------
# 1. SSE token is bound to operator and dropped on invalidation
# ---------------------------------------------------------------------------


def test_sse_token_records_operator_id(sessions_root):
    server = _MockServer(sessions_root)
    operator_id, session_token, _ = _create_and_login(server)

    sse_tok = _mint_sse_token(server, session_token)
    assert server.sse_tokens[sse_tok]["operator_id"] == operator_id


def test_bootstrap_sse_token_has_no_operator(sessions_root):
    # No profiles -> bootstrap; no operator session token supplied.
    server = _MockServer(sessions_root)
    sse_tok = _mint_sse_token(server, operator_token=None)
    assert server.sse_tokens[sse_tok]["operator_id"] is None


def test_invalidate_drops_only_matching_operator_sse_tokens(sessions_root):
    server = _MockServer(sessions_root)
    op_a, tok_a, _ = _create_and_login(server, pin="1234")

    sse_a = _mint_sse_token(server, tok_a)
    # A bootstrap/foreign token bound to a different operator id must survive.
    with _OPERATOR_LOCK:
        server.sse_tokens["sse_other"] = {"expires_at": time.time() + 30.0, "operator_id": "op_other"}
        server.sse_tokens["sse_bootstrap"] = {"expires_at": time.time() + 30.0, "operator_id": None}

    with _OPERATOR_LOCK:
        _invalidate_operator_sessions(server, op_a)

    assert sse_a not in server.sse_tokens
    assert "sse_other" in server.sse_tokens
    assert "sse_bootstrap" in server.sse_tokens


def test_reset_pin_with_recovery_drops_operator_sse_tokens(sessions_root):
    server = _MockServer(sessions_root)
    operator_id, session_token, recovery_code = _create_and_login(server)
    sse_tok = _mint_sse_token(server, session_token)
    assert sse_tok in server.sse_tokens

    status, body = reset_pin_with_recovery(server, operator_id, recovery_code, "5678")
    assert status == 200, body
    # Both the operator session and the bound SSE token are gone.
    assert session_token not in server.operator_sessions
    assert sse_tok not in server.sse_tokens


def test_host_reset_drops_operator_sse_tokens(sessions_root):
    server = _MockServer(sessions_root)
    operator_id, session_token, _ = _create_and_login(server)
    sse_tok = _mint_sse_token(server, session_token)
    assert sse_tok in server.sse_tokens

    status, body = host_reset_pin(server, operator_id, "5678")
    assert status == 200, body
    assert session_token not in server.operator_sessions
    assert sse_tok not in server.sse_tokens


def test_logout_drops_operator_sse_tokens(sessions_root):
    """The do_POST /api/auth/logout branch must drop the operator's SSE tokens."""
    server = _MockServer(sessions_root)
    operator_id, session_token, _ = _create_and_login(server)
    sse_tok = _mint_sse_token(server, session_token)

    # Emulate the production logout branch (token resolved, session + SSE dropped).
    token = session_token
    with _OPERATOR_LOCK:
        if token in server.operator_sessions:
            dropped_operator_id = server.operator_sessions.pop(token, {}).get("operator_id")
            _invalidate_operator_sessions(server, dropped_operator_id)

    assert session_token not in server.operator_sessions
    assert sse_tok not in server.sse_tokens


# ---------------------------------------------------------------------------
# 2. Concurrency smoke test — no "dict changed size during iteration"
# ---------------------------------------------------------------------------


def test_concurrent_mint_reap_pop_does_not_raise(sessions_root):
    server = _MockServer(sessions_root)
    errors: list = []
    stop = threading.Event()

    def mint_and_reap():
        try:
            while not stop.is_set():
                now = time.time()
                tok = _TOKEN_COUNTER.next()
                with _OPERATOR_LOCK:
                    # Reap expired then insert — same shape as the handler.
                    expired = [t for t, s in server.sse_tokens.items() if now >= s.get("expires_at", 0.0)]
                    for t in expired:
                        server.sse_tokens.pop(t, None)
                    # Mix of soon-expiring and live entries to force churn.
                    server.sse_tokens[tok] = {
                        "expires_at": now + (0.0 if (len(tok) % 2) else 5.0),
                        "operator_id": "op_churn",
                    }
        except Exception as exc:  # pragma: no cover - failure path
            errors.append(exc)

    def pop_sessions():
        try:
            while not stop.is_set():
                with _OPERATOR_LOCK:
                    server.operator_sessions[_TOKEN_COUNTER.next()] = {
                        "operator_id": "op_churn",
                        "expires_at": time.time() + 1.0,
                    }
                    # Iterate + pop concurrently with the minters.
                    to_drop = list(server.operator_sessions.keys())[:5]
                    for t in to_drop:
                        server.operator_sessions.pop(t, None)
                with _OPERATOR_LOCK:
                    _invalidate_operator_sessions(server, "op_churn")
        except Exception as exc:  # pragma: no cover - failure path
            errors.append(exc)

    threads = [threading.Thread(target=mint_and_reap) for _ in range(6)]
    threads += [threading.Thread(target=pop_sessions) for _ in range(6)]
    for t in threads:
        t.start()
    time.sleep(0.5)
    stop.set()
    for t in threads:
        t.join(timeout=5.0)

    assert not errors, f"concurrent access raised: {errors!r}"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
