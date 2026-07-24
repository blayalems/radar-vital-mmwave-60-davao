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
import os
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch


from rvt_trainer.monolith import _ControlServer
from rvt_trainer.monolith import _analysis_job_status
from rvt_trainer.monolith import _rerun_session_analysis
from rvt_trainer.monolith import _scan_sessions_root
from rvt_trainer.monolith import save_json


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


def _start_live_server(tmp_path: Path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=False)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    return server, base, sessions


def test_control_server_shutdown_owns_session_reap_without_analysis(tmp_path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer(
        "127.0.0.1",
        0,
        str(sessions),
        bind_mode="local",
        mock=False,
    )
    calls = []

    with (
        patch.object(
            server.supervisor,
            "close_start_gate",
            side_effect=lambda: calls.append("gate"),
        ) as close_gate,
        patch.object(
            server.httpd,
            "shutdown",
            side_effect=lambda: calls.append("http_shutdown"),
        ) as http_shutdown,
        patch.object(
            server.supervisor,
            "stop",
            side_effect=lambda **_kwargs: calls.append("session_stop"),
        ) as session_stop,
        patch.object(
            server.httpd,
            "server_close",
            side_effect=lambda: calls.append("http_close"),
        ) as http_close,
    ):
        server.stop()
        server.stop()

    assert calls == ["gate", "http_shutdown", "session_stop", "http_close"]
    close_gate.assert_called_once_with()
    http_shutdown.assert_called_once_with()
    session_stop.assert_called_once_with(
        reason="server_shutdown",
        auto_analyse=False,
        missing_ok=True,
    )
    http_close.assert_called_once_with()


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


class TestSessionHistorySelfHealing:
    def test_sessions_infer_timestamp_and_subject_from_files_and_profile(self, tmp_path):
        sessions = tmp_path / "sessions"
        session = sessions / "s13"
        session.mkdir(parents=True)
        radar = session / "radar.csv"
        radar.write_text("t_s,hr_bpm,rr_bpm\n1,72,16\n", encoding="utf-8")
        ref = session / "ref.csv"
        ref.write_text("t_s,ref_hr,ref_rr\n", encoding="utf-8")
        save_json({"session_id": "s13", "subject_profile_id": "adult_default"}, str(session / "session_manifest.json"))
        expected_mtime = 1_719_999_123
        os.utime(radar, (expected_mtime, expected_mtime))
        os.utime(ref, (expected_mtime, expected_mtime))

        items = _scan_sessions_root(str(sessions))

        assert len(items) == 1
        item = items[0]
        assert item["session_id"] == "s13"
        assert item["timestamp_source"] == "session_file_mtime"
        assert item["started_ms"] == expected_mtime * 1000
        assert item["started_at"].startswith("2024-07-03T")
        assert item["subject_label"] == "Adult Default"
        assert item["subject_profile_id"] == "adult_default"

    def test_sessions_infer_duration_from_radar_timestamp_ms(self, tmp_path):
        sessions = tmp_path / "sessions"
        session = sessions / "s13"
        session.mkdir(parents=True)
        (session / "radar.csv").write_text("timestamp_ms,reported_hr,reported_rr\n10285,0,0\n304421,63,11\n", encoding="utf-8")
        (session / "ref.csv").write_text("t_s,ref_hr,ref_rr\n", encoding="utf-8")
        save_json({"session_id": "s13", "subject_profile_id": "adult_default"}, str(session / "session_manifest.json"))

        items = _scan_sessions_root(str(sessions))

        assert items[0]["duration_s"] == 294

    def test_rerun_analysis_with_empty_ble_reference_records_radar_only_status(self, tmp_path):
        sessions = tmp_path / "sessions"
        session = sessions / "s13"
        session.mkdir(parents=True)
        (session / "radar.csv").write_text("t_s,hr_bpm,rr_bpm\n1,72,16\n2,73,16\n", encoding="utf-8")
        (session / "ref.csv").write_text("t_s,ref_hr,ref_rr\n", encoding="utf-8")
        save_json(
            {"error": "Device with address 10:22:33:9E:8F:63 was not found.", "parsed_rows": 0},
            str(session / "ref_ble_summary.json"),
        )
        save_json({"session_id": "s13", "subject_profile_id": "adult_default"}, str(session / "session_manifest.json"))

        payload = _rerun_session_analysis(str(session))

        assert payload["status"] == "complete"
        assert payload["analysis_status"] == "radar_only"
        assert payload["progress_pct"] == 100
        assert "10:22:33:9E:8F:63" in payload["last_line"]
        status_path = session / "analysis" / "analyse_status.json"
        assert status_path.exists()
        status_payload = _analysis_job_status(str(sessions), "s13")
        assert status_payload["analysis_status"] == "radar_only"


class TestApiLiveDashboardReal:
    def test_live_dashboard_returns_idle_payload_without_active_session(self, tmp_path):
        server, base, _ = _start_live_server(tmp_path)
        try:
            status, payload = _get(base, "/api/session/current/live_dashboard.json")
            assert status == 200
            assert payload["meta"]["status"] == "waiting"
            assert payload["meta"]["active"] is False
        finally:
            server.stop()

    def test_live_dashboard_returns_latest_payload_without_active_session(self, tmp_path):
        server, base, sessions = _start_live_server(tmp_path)
        try:
            session_dir = sessions / "s01"
            session_dir.mkdir()
            save_json({
                "schema_version": "rvt-live-events-v12.0",
                "session_id": "s01",
                "meta": {"status": "analysis complete", "session_id": "s01"},
                "radar": {"rows": 44},
                "ble": {"rows": 22},
                "series": {},
            }, str(session_dir / "live_dashboard.json"))

            status, payload = _get(base, "/api/session/current/live_dashboard.json")
            assert status == 200
            assert payload["session_id"] == "s01"
            assert payload["radar"]["rows"] == 44
            assert payload["meta"]["active"] is False
            assert payload["meta"]["latest_session"] is True
        finally:
            server.stop()
