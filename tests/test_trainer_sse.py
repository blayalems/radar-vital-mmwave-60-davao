"""Unit tests for rvt_trainer.api.sse — EventSource telemetry subscription,
pinging, stopped states, and client disconnection resilience.
"""

from __future__ import annotations

import time
import json
from typing import Optional, Dict, Any
from pathlib import Path
import pytest
from unittest.mock import patch

from rvt_trainer.api.sse import handle_sse_subscription
from rvt_trainer.monolith import FEATURE_FLAGS


class _StubWFile:
    def __init__(self, raise_after: Optional[int] = None) -> None:
        self.buffer: list[bytes] = []
        self.raise_after = raise_after
        self.write_count = 0

    def write(self, b: bytes) -> None:
        self.write_count += 1
        if self.raise_after is not None and self.write_count > self.raise_after:
            raise BrokenPipeError("Mocked client disconnect")
        self.buffer.append(b)

    def flush(self) -> None:
        pass


class _StubSupervisor:
    def __init__(self) -> None:
        self.active_session: Optional[Dict[str, Any]] = None

    def current(self) -> Optional[Dict[str, Any]]:
        return self.active_session


class _StubServer:
    def __init__(self) -> None:
        self.mock = False
        self.supervisor = _StubSupervisor()


class _StubHandler:
    def __init__(self, server: _StubServer, raise_after: Optional[int] = None) -> None:
        self.server = server
        self.wfile = _StubWFile(raise_after=raise_after)
        self.response_status: Optional[int] = None
        self.headers: Dict[str, str] = {}
        self.ended_headers = False
        self.json_status: Optional[int] = None
        self.json_body: Optional[Dict[str, Any]] = None

    def send_response(self, status: int) -> None:
        self.response_status = status

    def send_header(self, name: str, value: str) -> None:
        self.headers[name] = value

    def end_headers(self) -> None:
        self.ended_headers = True

    def _send_json(self, status: int, body: Dict[str, Any]) -> None:
        self.json_status = status
        self.json_body = body


def test_sse_disabled_returns_404():
    server = _StubServer()
    handler = _StubHandler(server)
    
    with patch.dict(FEATURE_FLAGS, {"enable_sse": False}):
        handle_sse_subscription(handler)
        
    assert handler.json_status == 404
    assert handler.json_body is not None
    assert handler.json_body["error"]["code"] == "SSE_DISABLED"


@patch("time.sleep", return_value=None)
def test_sse_headers_and_initial_ping(mock_sleep):
    server = _StubServer()
    # Raise BrokenPipeError after the initial ping write (1 write)
    handler = _StubHandler(server, raise_after=1)
    
    with patch.dict(FEATURE_FLAGS, {"enable_sse": True}):
        handle_sse_subscription(handler)
        
    assert handler.response_status == 200
    assert handler.headers["Content-Type"] == "text/event-stream; charset=utf-8"
    assert handler.headers["Cache-Control"] == "no-cache"
    assert handler.headers["Connection"] == "keep-alive"
    assert handler.ended_headers is True
    
    # We should have written exactly 1 event (the ping) before disconnect
    assert len(handler.wfile.buffer) == 1
    event_text = handler.wfile.buffer[0].decode("utf-8")
    assert event_text.startswith("event: ping\n")


@patch("time.sleep", return_value=None)
def test_sse_mock_live_events(mock_sleep):
    server = _StubServer()
    server.mock = True
    # Write initial ping (1), then first mock event (2), then raise BrokenPipeError (3)
    handler = _StubHandler(server, raise_after=2)
    
    with patch.dict(FEATURE_FLAGS, {"enable_sse": True}):
        handle_sse_subscription(handler)
        
    assert len(handler.wfile.buffer) == 2
    event1 = handler.wfile.buffer[0].decode("utf-8")
    event2 = handler.wfile.buffer[1].decode("utf-8")
    
    assert event1.startswith("event: ping\n")
    assert event2.startswith("event: live\n")
    assert "schema_version" in event2


@patch("time.sleep", return_value=None)
def test_sse_stopped_when_session_not_active(mock_sleep):
    server = _StubServer()
    server.supervisor.active_session = None
    # Write initial ping (1), thenstopped session check (2) -> exits handler cleanly
    handler = _StubHandler(server, raise_after=5)
    
    with patch.dict(FEATURE_FLAGS, {"enable_sse": True}):
        handle_sse_subscription(handler, session_id_hint="my-session")
        
    # Exits loop naturally because a stopped event was sent
    assert len(handler.wfile.buffer) == 2
    event1 = handler.wfile.buffer[0].decode("utf-8")
    event2 = handler.wfile.buffer[1].decode("utf-8")
    
    assert event1.startswith("event: ping\n")
    assert event2.startswith("event: stopped\n")
    assert "session_not_active" in event2


def test_sse_deadline_warning_payload_is_exactly_sixty_seconds():
    server = _StubServer()
    handler = _StubHandler(server, raise_after=2)
    monotonic_values = iter([
        100.0,   # last_emit_monotonic
        100.0,   # initial ping last_emit_monotonic
        100.0,   # deadline base
        43240.1, # loop condition: still before 12h deadline
        43240.1, # time_remaining is 59.9s; payload must stay contractual
        43240.1, # write_event last_emit_monotonic
        43240.1, # stopped event last_emit_monotonic
    ])

    with patch.dict(FEATURE_FLAGS, {"enable_sse": True}), \
            patch("time.monotonic", side_effect=lambda: next(monotonic_values)), \
            patch("time.sleep", return_value=None):
        handle_sse_subscription(handler, session_id_hint="test-session")

    assert len(handler.wfile.buffer) == 2
    warning = handler.wfile.buffer[1].decode("utf-8")
    assert warning.startswith("event: session_warning\n")
    payload = json.loads(warning.split("data: ", 1)[1])
    assert payload["reason"] == "deadline_approaching"
    assert payload["seconds_remaining"] == 60


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
