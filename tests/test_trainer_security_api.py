from __future__ import annotations

import json
from pathlib import Path
import urllib.error
import urllib.request

from rvt_trainer.monolith import _ControlServer


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
        assert _request(base, "/api/server-info")[0] == 200
        assert _request(base, "/api/status")[0] == 401
        assert _request(base, "/api/subject-profiles")[0] == 401
        assert _request(base, "/api/events/subscribe")[0] == 401
        server.httpd.auth_tokens.add("accepted-token")
        assert _request(base, "/api/status", token="accepted-token")[0] == 200
        assert _request(base, "/api/subject-profiles", token="accepted-token")[0] == 200
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
        "product_version": "16.0.1",
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
            assert data["product_version"] == "16.0.1"
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

