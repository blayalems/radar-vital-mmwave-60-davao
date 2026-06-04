"""Control server and SSE API classes."""

from __future__ import annotations
import json
import time
from pathlib import Path
from typing import Optional, Dict, Any

from ..monolith import _ControlHandler as ControlHandler
from ..monolith import _SessionSupervisor as SessionSupervisor

SSE_DEADLINE_WARN_BEFORE_S = 60


def handle_sse_subscription(handler, session_id_hint: Optional[str] = None):
    """Handle an EventSource subscription for live telemetry streams.

    Delegates low-level HTTP socket writing and headers to the calling HTTP
    handler, keeping the streaming loop isolated and testable.
    """
    # Import necessary helpers dynamically to avoid circular imports
    from ..monolith import (
        FEATURE_FLAGS,
        LIVE_EVENT_SCHEMA_VERSION,
        _mock_live_payload,
        _iso_now,
        nan_safe,
        _read_json_if_exists
    )

    if not FEATURE_FLAGS.get("enable_sse", True):
        handler._send_json(404, {"ok": False, "error": {"code": "SSE_DISABLED", "message": "SSE is disabled by feature flag"}})
        return

    handler.send_response(200)
    handler.send_header("Content-Type", "text/event-stream; charset=utf-8")
    handler.send_header("Cache-Control", "no-cache")
    handler.send_header("Connection", "keep-alive")
    handler.end_headers()

    last_mtime = None
    seq = 0
    last_emit_monotonic = time.monotonic()
    had_active_session = False
    ping_interval_s = 5.0
    last_session_id = None

    def write_event(name: str, data: Dict[str, Any]):
        nonlocal seq, last_emit_monotonic
        seq += 1
        payload = dict(data)
        payload.setdefault("schema_version", LIVE_EVENT_SCHEMA_VERSION)
        payload.setdefault("seq", seq)
        raw = f"event: {name}\ndata: {json.dumps(nan_safe(payload), allow_nan=False)}\n\n".encode("utf-8")
        handler.wfile.write(raw)
        handler.wfile.flush()
        last_emit_monotonic = time.monotonic()

    try:
        write_event("ping", {"at": _iso_now()})
        deadline = time.monotonic() + 12 * 60 * 60  # 12 hr — hard kill
        warned = False
        while time.monotonic() < deadline:
            time_remaining = deadline - time.monotonic()
            if time_remaining <= SSE_DEADLINE_WARN_BEFORE_S and not warned:
                write_event("session_warning", {
                    "reason": "deadline_approaching",
                    "seconds_remaining": SSE_DEADLINE_WARN_BEFORE_S
                })
                warned = True

            if getattr(handler.server, "mock", False):
                write_event("live", _mock_live_payload(seq))
                time.sleep(1.0)
                continue
            cur = handler.server.supervisor.current()
            if cur:
                had_active_session = True
                live_path = Path(str(cur.get("session_dir", ""))) / "live_dashboard.json"
                current_sid = str(cur.get("session_id") or "")
                if current_sid:
                    last_session_id = current_sid
                if session_id_hint and current_sid and session_id_hint != current_sid:
                    write_event("stopped", {"reason": "different_active_session", "session_id": session_id_hint, "active_session_id": current_sid})
                    return
                if live_path.exists():
                    mtime = live_path.stat().st_mtime_ns
                    if mtime != last_mtime:
                        last_mtime = mtime
                        payload = _read_json_if_exists(str(live_path)) or {}
                        if isinstance(payload, dict):
                            payload.setdefault("session_id", cur.get("session_id"))
                            payload.setdefault("revision", mtime)
                        write_event("live", payload if isinstance(payload, dict) else {"payload": payload})
                        write_event("data_update", {"session_id": cur.get("session_id"), "revision": mtime})
            elif session_id_hint and not had_active_session and last_mtime is None:
                write_event("stopped", {"reason": "session_not_active", "session_id": session_id_hint})
                return
            elif had_active_session or last_mtime is not None:
                write_event("stopped", {"reason": "no_active_session", "session_id": session_id_hint or last_session_id})
                return
            if (time.monotonic() - last_emit_monotonic) >= ping_interval_s:
                write_event("ping", {"at": _iso_now(), "session_id": session_id_hint})
            time.sleep(1.0)
    except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
        return


__all__ = ["ControlHandler", "SessionSupervisor", "SSE_DEADLINE_WARN_BEFORE_S", "handle_sse_subscription"]
