"""
test_trainer_lifecycle.py
-------------------------
Integration tests for the _ControlServer lifecycle (mock=True) covering:
  /api/status          – shape and required fields
  /api/session/current – mock response shape
  /api/sessions        – list response shape
  /api/session/current/live_dashboard.json – payload structure in mock mode
"""
from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
from pathlib import Path


from rvt_trainer.monolith import _ControlServer


def _get(base: str, path: str) -> tuple[int, object]:
    req = urllib.request.Request(base + path, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as err:
        return err.code, json.loads(err.read().decode("utf-8"))


def _start_mock_server(tmp_path: Path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    return server, base


# ---------------------------------------------------------------------------
# /api/status
# ---------------------------------------------------------------------------

class TestApiStatus:
    def test_status_ok_true(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            status, payload = _get(base, "/api/status")
            assert status == 200
            assert payload["ok"] is True
        finally:
            server.stop()

    def test_status_has_trainer_version(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/status")
            assert "trainer_version" in payload
            assert isinstance(payload["trainer_version"], str)
            assert len(payload["trainer_version"]) > 0
        finally:
            server.stop()

    def test_status_has_firmware_expected(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/status")
            assert "firmware_expected" in payload
        finally:
            server.stop()

    def test_status_has_active_session_field(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/status")
            # In mock mode active_session is a dict with session_id='mock'
            assert "active_session" in payload
            active = payload["active_session"]
            assert active is not None
            assert active.get("session_id") == "mock"
        finally:
            server.stop()

    def test_status_has_feature_flags(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/status")
            assert "feature_flags" in payload
            assert isinstance(payload["feature_flags"], dict)
        finally:
            server.stop()

    def test_status_has_control_server_started_at(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/status")
            assert "control_server_started_at" in payload
        finally:
            server.stop()


# ---------------------------------------------------------------------------
# /api/session/current
# ---------------------------------------------------------------------------

class TestApiSessionCurrent:
    def test_session_current_returns_200_in_mock(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            status, payload = _get(base, "/api/session/current")
            assert status == 200
        finally:
            server.stop()

    def test_session_current_has_mock_session_id(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current")
            assert payload.get("session_id") == "mock"
        finally:
            server.stop()

    def test_session_current_mock_flag_true(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current")
            assert payload.get("mock") is True
        finally:
            server.stop()

    def test_session_current_has_started_at(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current")
            assert "started_at" in payload
        finally:
            server.stop()


# ---------------------------------------------------------------------------
# /api/sessions
# ---------------------------------------------------------------------------

class TestApiSessions:
    def test_sessions_returns_200(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            status, payload = _get(base, "/api/sessions")
            assert status == 200
        finally:
            server.stop()

    def test_sessions_has_items_list(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/sessions")
            assert "items" in payload
            assert isinstance(payload["items"], list)
        finally:
            server.stop()

    def test_sessions_has_root_field(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/sessions")
            assert "root" in payload
            assert isinstance(payload["root"], str)
        finally:
            server.stop()

    def test_sessions_items_are_empty_with_no_sessions(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/sessions")
            # No sessions dirs exist — items should be an empty list
            assert payload["items"] == []
        finally:
            server.stop()


# ---------------------------------------------------------------------------
# /api/session/current/live_dashboard.json  (mock mode)
# ---------------------------------------------------------------------------

class TestApiLiveDashboardMock:
    def test_live_dashboard_returns_200_in_mock(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            status, payload = _get(base, "/api/session/current/live_dashboard.json")
            assert status == 200
        finally:
            server.stop()

    def test_live_dashboard_has_radar_key(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current/live_dashboard.json")
            assert "radar" in payload
            assert isinstance(payload["radar"], dict)
        finally:
            server.stop()

    def test_live_dashboard_has_series_key(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current/live_dashboard.json")
            assert "series" in payload
            assert isinstance(payload["series"], dict)
        finally:
            server.stop()

    def test_live_dashboard_has_meta_key(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current/live_dashboard.json")
            assert "meta" in payload
            assert isinstance(payload["meta"], dict)
        finally:
            server.stop()

    def test_live_dashboard_radar_has_pqi_heart(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current/live_dashboard.json")
            radar = payload["radar"]
            assert "pqi_heart" in radar
            pqi = radar["pqi_heart"]
            # Must be a finite float in [0, 1]
            assert isinstance(pqi, float)
            assert not math.isnan(pqi)
            assert 0.0 <= pqi <= 1.0
        finally:
            server.stop()

    def test_live_dashboard_radar_has_reported_hr(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current/live_dashboard.json")
            radar = payload["radar"]
            assert "reported_hr" in radar
            hr = radar["reported_hr"]
            assert isinstance(hr, (int, float))
            assert not math.isnan(float(hr))
        finally:
            server.stop()

    def test_live_dashboard_meta_has_status(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current/live_dashboard.json")
            meta = payload["meta"]
            assert "status" in meta
        finally:
            server.stop()

    def test_live_dashboard_session_id_is_mock(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current/live_dashboard.json")
            assert payload.get("session_id") == "mock"
        finally:
            server.stop()

    def test_live_dashboard_series_has_time_array(self, tmp_path):
        server, base = _start_mock_server(tmp_path)
        try:
            _, payload = _get(base, "/api/session/current/live_dashboard.json")
            series = payload["series"]
            assert "t" in series
            t = series["t"]
            assert isinstance(t, list)
            assert len(t) > 0
        finally:
            server.stop()
