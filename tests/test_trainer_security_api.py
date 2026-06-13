from __future__ import annotations

import json
import time
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace
import urllib.error
import urllib.request

from rvt_trainer.monolith import _ControlHandler, _ControlServer


def _request(base: str, path: str, method: str = "GET", token: str = "", payload=None):
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    headers = {"Content-Type": "application/json"} if data else {}
    if token:
        headers["X-RVT-Auth"] = token
    request = urllib.request.Request(base + path, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read().decode("utf-8"))


def _request_raw(base: str, path: str, method: str = "GET", headers: dict = None):
    request = urllib.request.Request(base + path, headers=headers or {}, method=method)
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response
    except urllib.error.HTTPError as error:
        return error


def test_cors_headers_are_restricted_by_default(tmp_path: Path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    # By default, cors_origin is empty. We test if it emits no Access-Control-Allow-Origin
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True, cors_origin="")
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    try:
        response = _request_raw(base, "/api/health", headers={"Origin": "http://malicious.com"})
        assert response.status == 200
        assert "Access-Control-Allow-Origin" not in response.headers
    finally:
        server.stop()


def test_cors_headers_are_emitted_when_origin_matches(tmp_path: Path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    # When explicitly allowed:
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True, cors_origin="http://allowed.com")
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    try:
        response = _request_raw(base, "/api/health", headers={"Origin": "http://allowed.com"})
        assert response.status == 200
        assert response.headers.get("Access-Control-Allow-Origin") == "http://allowed.com"
        assert response.headers.get("Vary") == "Origin"

        response_bad = _request_raw(base, "/api/health", headers={"Origin": "http://malicious.com"})
        assert response_bad.status == 200
        assert "Access-Control-Allow-Origin" not in response_bad.headers
    finally:
        server.stop()


def test_lan_sensitive_reads_require_token_and_public_shell_remains_available(tmp_path: Path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="lan", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    try:
        assert _request(base, "/api/health")[0] == 200
        # Loopback clients (the EXE shell) bypass the LAN pairing gate for
        # discovery routes; sensitive endpoints below still require an
        # operator session. Network peers keep the 401 pairing requirement,
        # which these loopback-only tests cannot exercise directly.
        assert _request(base, "/api/server-info")[0] == 200
        assert _request(base, "/api/status")[0] == 401
        assert _request(base, "/api/subject-profiles")[0] == 401
        assert _request(base, "/api/events/subscribe")[0] == 401
        
        # Add a pairing token
        server.httpd.auth_tokens.add("accepted-pairing-token")
        assert _request(base, "/api/server-info", token="accepted-pairing-token")[0] == 200
        
        # Verify that pairing token is NOT enough for sensitive endpoints under PR48
        assert _request(base, "/api/status", token="accepted-pairing-token")[0] == 401
        assert _request(base, "/api/subject-profiles", token="accepted-pairing-token")[0] == 401
        
        # Add a valid operator session token
        if not hasattr(server.httpd, "operator_sessions"):
            server.httpd.operator_sessions = {}
        server.httpd.operator_sessions["accepted-op-token"] = {
            "operator_id": "op_test",
            "expires_at": time.time() + 3600
        }
        
        # Verify operator session token allows access to sensitive endpoints
        assert _request(base, "/api/status", token="accepted-op-token")[0] == 200
        assert _request(base, "/api/subject-profiles", token="accepted-op-token")[0] == 200
    finally:
        server.stop()


def test_repo_private_and_arbitrary_paths_are_not_static_files(tmp_path: Path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    try:
        assert _request(base, "/.rvt_tls/key.pem")[0] == 404
        assert _request(base, "/rvt_trainer/monolith.py")[0] == 404
    finally:
        server.stop()


def test_review_summary_and_signoff_round_trip(tmp_path: Path):
    sessions = tmp_path / "sessions"
    session = sessions / "SESSION-01"
    session.mkdir(parents=True)
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=False)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    try:
        status, _ = _request(base, "/api/sessions/SESSION-01/notes", "PUT", payload={"review_summary": "Reviewed while seated."})
        assert status == 200
        status, notes = _request(base, "/api/sessions/SESSION-01/notes")
        payload = notes.get("data", notes)
        assert status == 200
        assert payload["review_summary"] == "Reviewed while seated."

        status, signoff = _request(base, "/api/sessions/SESSION-01/signoff", "PUT", payload={
            "operator_name": "Field Operator",
            "initials": "fo",
            "validation_comment": "Reference trace inspected."
        })
        payload = signoff.get("data", signoff)
        assert status == 200
        assert payload["initials"] == "FO"
        assert payload["signed_at"]
    finally:
        server.stop()


def test_headerless_request_does_not_crash(tmp_path: Path):
    import socket
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    port = server.httpd.server_port
    try:
        # Send a raw malformed/headerless request (HTTP/0.9)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("127.0.0.1", port))
        s.sendall(b"GET /api/health\r\n\r\n")
        try:
            s.recv(1024)
        except Exception:
            pass
        s.close()

        # Check server remains healthy and running
        base = f"http://127.0.0.1:{port}"
        status, resp = _request(base, "/api/health")
        assert status == 200
        assert resp["ok"] is True
    finally:
        server.stop()


def test_api_version_additive_product_and_schema_fields(tmp_path: Path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    try:
        status, payload = _request(base, "/api/version")
        assert status == 200
        assert payload["trainer"] == "16.3.0"
        assert payload["dashboard"] == "16.3.0"
        assert payload["product_version"] == "16.3.0"
        assert payload["firmware_expected"] == "v16.3.0"
        expected_schema_versions = {
            "control_api": "rvt-control-api-v12.0",
            "session_notes": "rvt-session-notes-v12.0",
            "session_signoff": "rvt-session-signoff-v12.0",
            "training_progress": "rvt-training-progress-v12.0",
            "live_events": "rvt-live-events-v12.0",
            "session_manifest": "rvt-session-manifest-v12.0",
            "chart_annotations": "rvt-chart-annotations-v12.0",
            "subject_profile": "rvt-subject-profiles-v12.0",
        }
        for key, value in expected_schema_versions.items():
            assert payload["schema_versions"][key] == value
        assert payload["schema_versions"]["live_event"] == "rvt-live-events-v12.0"
        assert payload["update_manifest_url"].endswith("/rvt-latest.json")
    finally:
        server.stop()


def test_control_handler_end_headers_tolerates_missing_headers_attribute():
    handler = object.__new__(_ControlHandler)
    handler.server = SimpleNamespace(
        cors_origin="",
        content_security_policy="default-src 'self'",
        tls_trusted=False,
    )
    handler._headers_buffer = []
    handler.request_version = "HTTP/1.1"
    handler.protocol_version = "HTTP/1.0"
    handler.wfile = BytesIO()

    handler.end_headers()

    raw_headers = handler.wfile.getvalue().decode("iso-8859-1")
    assert "Cache-Control: no-store, no-cache, must-revalidate, max-age=0" in raw_headers
    assert "Content-Security-Policy: default-src 'self'" in raw_headers
    assert "Access-Control-Allow-Origin" not in raw_headers


def test_update_manifest_proxy_success(tmp_path: Path):
    from unittest.mock import patch, MagicMock
    from rvt_trainer.monolith import _manifest_cache

    # Reset cache to force network call
    _manifest_cache["data"] = None
    _manifest_cache["ts"] = 0

    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"

    mock_data = {
        "product_version": "16.3.0",
        "minimum_supported": "16.0.0",
        "released_at": "2026-06-06T00:00:00Z",
        "artifacts": {}
    }

    original_urlopen = urllib.request.urlopen

    mock_response = MagicMock()
    mock_response.read.return_value = json.dumps(mock_data).encode("utf-8")
    mock_response.__enter__.return_value = mock_response

    def side_effect(req, *args, **kwargs):
        url = req.full_url if hasattr(req, "full_url") else str(req)
        if "rvt-latest.json" in url:
            return mock_response
        return original_urlopen(req, *args, **kwargs)

    try:
        with patch("urllib.request.urlopen", side_effect=side_effect):
            status, data = _request(base, "/api/update/manifest")
            assert status == 200
            assert data["product_version"] == "16.3.0"
    finally:
        server.stop()


def test_update_manifest_proxy_failure(tmp_path: Path):
    from unittest.mock import patch
    from rvt_trainer.monolith import _manifest_cache

    # Reset cache to force network call
    _manifest_cache["data"] = None
    _manifest_cache["ts"] = 0

    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"

    original_urlopen = urllib.request.urlopen

    def side_effect(req, *args, **kwargs):
        url = req.full_url if hasattr(req, "full_url") else str(req)
        if "rvt-latest.json" in url:
            raise urllib.error.URLError("Network error")
        return original_urlopen(req, *args, **kwargs)

    try:
        with patch("urllib.request.urlopen", side_effect=side_effect):
            status, data = _request(base, "/api/update/manifest")
            assert status == 502
            assert data["ok"] is False
            assert "PROXY_ERROR" in data["error"]["code"]
    finally:
        server.stop()


def test_update_manifest_proxy_caching_and_expiration(tmp_path: Path):
    from unittest.mock import patch, MagicMock
    from rvt_trainer.monolith import _manifest_cache
    import time

    # 1. Reset cache
    _manifest_cache["data"] = None
    _manifest_cache["ts"] = 0

    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"

    call_count = 0
    original_urlopen = urllib.request.urlopen

    def side_effect(req, *args, **kwargs):
        url = req.full_url if hasattr(req, "full_url") else str(req)
        if "rvt-latest.json" in url:
            nonlocal call_count
            call_count += 1
            mock_response = MagicMock()
            mock_data = {
                "product_version": f"16.3.0-{call_count}",
                "minimum_supported": "16.0.0",
                "released_at": "2026-06-06T00:00:00Z",
                "artifacts": {}
            }
            mock_response.read.return_value = json.dumps(mock_data).encode("utf-8")
            mock_response.__enter__.return_value = mock_response
            return mock_response
        return original_urlopen(req, *args, **kwargs)

    try:
        with patch("urllib.request.urlopen", side_effect=side_effect):
            # First request: should trigger fetch and get 16.3.0-1
            status1, data1 = _request(base, "/api/update/manifest")
            assert status1 == 200
            assert data1["product_version"] == "16.3.0-1"
            assert call_count == 1

            # Second request immediately: should return cached data and NOT increment call_count
            status2, data2 = _request(base, "/api/update/manifest")
            assert status2 == 200
            assert data2["product_version"] == "16.3.0-1"
            assert call_count == 1

            # Simulate cache expiration: move the cached timestamp back by 301 seconds
            _manifest_cache["ts"] = time.time() - 301

            # Third request: cache has expired, should trigger a new fetch and get 16.3.0-2
            status3, data3 = _request(base, "/api/update/manifest")
            assert status3 == 200
            assert data3["product_version"] == "16.3.0-2"
            assert call_count == 2
    finally:
        server.stop()


def test_public_server_info_never_exposes_pin_material(tmp_path: Path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="lan", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    try:
        # Loopback discovery is allowed in LAN bind, but must stay metadata-only.
        status_bare, payload_bare = _request(base, "/api/server-info")
        assert status_bare == 200
        assert "active_pin" not in payload_bare
        assert "qr_png_base64" not in payload_bare
        server.httpd.auth_tokens.add("paired-token")
        status, payload = _request(base, "/api/server-info", token="paired-token")
        assert status == 200
        assert "active_pin" not in payload
        assert "qr_png_base64" not in payload

        # The legacy ?format=qr variant must also stay metadata-only.
        status_qr, payload_qr = _request(base, "/api/server-info?format=qr", token="paired-token")
        assert status_qr == 200
        assert "active_pin" not in payload_qr
        assert "qr_png_base64" not in payload_qr
    finally:
        server.stop()


def test_native_pairing_qr_is_lan_gated(tmp_path: Path):
    import base64 as _b64

    sessions = tmp_path / "sessions"
    sessions.mkdir()

    # Local bind: pairing info responds but never mints QR material.
    local_server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    local_server.start()
    base = f"http://127.0.0.1:{local_server.httpd.server_port}"
    try:
        status, payload = _request(base, "/api/native-pairing-info?format=qr")
        assert status == 200
        assert payload["bind_mode"] == "local"
        assert "qr_png_base64" not in payload
    finally:
        local_server.stop()

    # LAN bind: format=qr returns a PNG (base64) pointing at the pairing URL.
    lan_server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="lan", mock=True)
    lan_server.start()
    base = f"http://127.0.0.1:{lan_server.httpd.server_port}"
    try:
        status, payload = _request(base, "/api/native-pairing-info?format=qr")
        assert status == 200
        assert payload["bind_mode"] == "lan"
        assert payload.get("qr_png_base64"), "LAN pairing info should include QR material"
        png = _b64.b64decode(payload["qr_png_base64"])
        assert png[:8] == b"\x89PNG\r\n\x1a\n"
        assert payload["qr_target_url"].startswith("http")
    finally:
        lan_server.stop()
