from __future__ import annotations

import argparse
import json
from pathlib import Path
import urllib.error
import urllib.request

import pytest

from rvt_trainer.monolith import _start_control_server


def _request_raw(base: str, path: str, headers: dict = None):
    request = urllib.request.Request(base + path, headers=headers or {}, method="GET")
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            body = response.read()
            return response.status, response.headers, body
    except urllib.error.HTTPError as error:
        return error.code, error.headers, error.read()


def _serve_args(tmp_path: Path, *, allow_wildcard_cors_lan: bool):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    return argparse.Namespace(
        sessions_root=str(sessions),
        bind="lan",
        host="127.0.0.1",
        control_port=0,
        cors_origin="*",
        allow_wildcard_cors_lan=allow_wildcard_cors_lan,
        mock=True,
        tls=None,
        tls_trusted=False,
    )


def test_lan_wildcard_cors_requires_explicit_lab_override(tmp_path: Path):
    with pytest.raises(RuntimeError, match="Refusing --cors-origin '\\*' with --bind lan"):
        _start_control_server(_serve_args(tmp_path, allow_wildcard_cors_lan=False))


def test_lan_wildcard_cors_override_still_emits_header(tmp_path: Path):
    server = _start_control_server(_serve_args(tmp_path, allow_wildcard_cors_lan=True))
    server.start()
    base = f"http://127.0.0.1:{server.httpd.server_port}"
    try:
        status, headers, body = _request_raw(base, "/api/health", headers={"Origin": "http://example.com"})
        assert status == 200
        assert headers.get("Access-Control-Allow-Origin") == "*"
        assert json.loads(body.decode("utf-8"))["ok"] is True
    finally:
        server.stop()
