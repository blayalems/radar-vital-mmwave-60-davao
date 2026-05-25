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
