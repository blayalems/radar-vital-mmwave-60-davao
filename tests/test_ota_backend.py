from __future__ import annotations

import json
import os
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch, MagicMock

import pytest
from rvt_trainer.monolith import _ControlServer, _manifest_cache

def _request(base: str, path: str, method: str = "GET", token: str = "", payload=None, headers=None):
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    req_headers = {"Content-Type": "application/json"} if data else {}
    if token:
        req_headers["X-RVT-Auth"] = token
    if headers:
        req_headers.update(headers)
    request = urllib.request.Request(base + path, data=data, headers=req_headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, json.loads(response.read().decode("utf-8")), response.headers
    except urllib.error.HTTPError as error:
        try:
            return error.code, json.loads(error.read().decode("utf-8")), error.headers
        except Exception:
            return error.code, None, error.headers


def test_backend_manifest_proxy_success(tmp_path: Path):
    # Reset cache
    _manifest_cache["data"] = None
    _manifest_cache["ts"] = 0

    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"

    mock_data = {
        "product_version": "16.1.0",
        "minimum_supported": "16.0.0",
        "released_at": "2026-06-06T00:00:00Z",
        "artifacts": {}
    }

    original_urlopen = urllib.request.urlopen
    mock_response = MagicMock()
    mock_response.read.return_value = json.dumps(mock_data).encode("utf-8")
    mock_response.__enter__.return_value = mock_response

    def mock_fetch(req, *args, **kwargs):
        url = req.full_url if hasattr(req, "full_url") else str(req)
        if "rvt-latest.json" in url:
            return mock_response
        return original_urlopen(req, *args, **kwargs)

    try:
        with patch("urllib.request.urlopen", side_effect=mock_fetch):
            status, data, _ = _request(base, "/api/update/manifest")
            assert status == 200
            assert data["product_version"] == "16.1.0"
    finally:
        server.stop()


def test_backend_manifest_proxy_caching_and_expiration(tmp_path: Path):
    # Reset cache
    _manifest_cache["data"] = None
    _manifest_cache["ts"] = 0

    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"

    call_count = 0
    original_urlopen = urllib.request.urlopen

    def mock_fetch(req, *args, **kwargs):
        nonlocal call_count
        url = req.full_url if hasattr(req, "full_url") else str(req)
        if "rvt-latest.json" in url:
            call_count += 1
            mock_resp = MagicMock()
            mock_data = {
                "product_version": f"16.1.0-{call_count}",
                "minimum_supported": "16.0.0",
                "released_at": "2026-06-06T00:00:00Z",
                "artifacts": {}
            }
            mock_resp.read.return_value = json.dumps(mock_data).encode("utf-8")
            mock_resp.__enter__.return_value = mock_resp
            return mock_resp
        return original_urlopen(req, *args, **kwargs)

    try:
        with patch("urllib.request.urlopen", side_effect=mock_fetch):
            # First fetch (should call network)
            status1, data1, _ = _request(base, "/api/update/manifest")
            assert status1 == 200
            assert data1["product_version"] == "16.1.0-1"
            assert call_count == 1

            # Second fetch within 5 minutes (should be cached)
            status2, data2, _ = _request(base, "/api/update/manifest")
            assert status2 == 200
            assert data2["product_version"] == "16.1.0-1"
            assert call_count == 1

            # Expire cache (move ts back by 301s)
            _manifest_cache["ts"] = time.time() - 301

            # Third fetch (should hit network again)
            status3, data3, _ = _request(base, "/api/update/manifest")
            assert status3 == 200
            assert data3["product_version"] == "16.1.0-2"
            assert call_count == 2
    finally:
        server.stop()


def test_backend_manifest_proxy_upstream_error_handling(tmp_path: Path):
    # Reset cache
    _manifest_cache["data"] = None
    _manifest_cache["ts"] = 0

    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"

    original_urlopen = urllib.request.urlopen

    def mock_fetch_error(req, *args, **kwargs):
        url = req.full_url if hasattr(req, "full_url") else str(req)
        if "rvt-latest.json" in url:
            raise urllib.error.URLError("Connection refused")
        return original_urlopen(req, *args, **kwargs)

    try:
        with patch("urllib.request.urlopen", side_effect=mock_fetch_error):
            status, data, _ = _request(base, "/api/update/manifest")
            assert status == 502
            assert data["ok"] is False
            assert "PROXY_ERROR" in data["error"]["code"]
    finally:
        server.stop()


def test_backend_manifest_proxy_auth_checks(tmp_path: Path):
    # Reset cache
    _manifest_cache["data"] = None
    _manifest_cache["ts"] = 0

    sessions = tmp_path / "sessions"
    sessions.mkdir()

    # Run in LAN bind mode to enforce control API authentication checks
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="lan", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"

    mock_data = {
        "product_version": "16.1.0",
        "minimum_supported": "16.0.0",
        "released_at": "2026-06-06T00:00:00Z",
        "artifacts": {}
    }

    original_urlopen = urllib.request.urlopen
    mock_response = MagicMock()
    mock_response.read.return_value = json.dumps(mock_data).encode("utf-8")
    mock_response.__enter__.return_value = mock_response

    def mock_fetch(req, *args, **kwargs):
        url = req.full_url if hasattr(req, "full_url") else str(req)
        if "rvt-latest.json" in url:
            return mock_response
        return original_urlopen(req, *args, **kwargs)

    try:
        with patch("urllib.request.urlopen", side_effect=mock_fetch):
            # Health and Manifest endpoints are part of public_api_paths, so they don't require token auth
            status, data, _ = _request(base, "/api/update/manifest")
            assert status == 200
            assert data["product_version"] == "16.1.0"

            # A protected endpoint should require token auth in LAN mode
            status_protected, data_protected, _ = _request(base, "/api/status")
            assert status_protected == 401
    finally:
        server.stop()


def test_backend_manifest_proxy_malformed_query_or_headers(tmp_path: Path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _ControlServer("127.0.0.1", 0, str(sessions), bind_mode="local", mock=True)
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"

    try:
        # Check malformed headers or query parameters do not crash the proxy
        status, data, _ = _request(base, "/api/update/manifest?version=invalid&foo=%ff", headers={"X-Malformed-Header": "\xff"})
        assert status in [200, 502]  # Depending on network availability, we expect either success or upstream error, not a server crash
    finally:
        server.stop()


def test_generate_rvt_latest_generates_both_manifests(tmp_path: Path):
    # Run scripts/generate-rvt-latest.mjs via subprocess with --self-test and assert success
    script_path = Path(__file__).resolve().parent.parent / "scripts" / "generate-rvt-latest.mjs"
    res = subprocess.run(["node", str(script_path), "--self-test"], capture_output=True, text=True)
    assert res.returncode == 0
    assert "Self-test validation passed successfully!" in res.stdout
