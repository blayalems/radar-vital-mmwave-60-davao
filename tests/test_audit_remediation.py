"""Regression tests for the 13-agent audit remediation waves.

Covers the high-confidence backend/CI fixes:

* Wave 1 — bounded/validated JSON request bodies (413/415/400) and the
  public-route / firmware-workflow-path guards.
* Wave 2 — fail-closed operator-profile DB load and display-name sanitization.
* Wave 4 — NaN/Inf JSON sanitation and current firmware preflight guidance.
"""

from __future__ import annotations

import http.client
import json
import re
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from rvt_trainer.api.auth import (
    create_operator_profile,
    load_operator_profiles,
    save_operator_profiles,
    _sanitize_display_name,
)
from rvt_trainer.monolith import (
    MAX_JSON_BODY_BYTES,
    _json_safe_response,
    _ControlServer,
    _ControlHandler,
)

ROOT = Path(__file__).resolve().parents[1]


# --------------------------------------------------------------------------- #
# Server fixtures / helpers
# --------------------------------------------------------------------------- #
class _Server:
    """Boot a mock control server on an ephemeral port and clean it up."""

    def __init__(self, sessions: Path, bind_mode: str = "local"):
        self.server = _ControlServer(
            "127.0.0.1", 0, str(sessions), bind_mode=bind_mode, mock=True, cors_origin=""
        )
        self.server.start()
        self.port = self.server.httpd.server_port

    def raw(self, method: str, path: str, body: bytes | None = None, headers: dict | None = None):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        try:
            conn.request(method, path, body=body, headers=headers or {})
            resp = conn.getresponse()
            data = resp.read()
            return resp.status, data
        finally:
            conn.close()

    def stop(self):
        self.server.stop()


@pytest.fixture
def server(tmp_path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    srv = _Server(sessions)
    try:
        yield srv
    finally:
        srv.stop()


# --------------------------------------------------------------------------- #
# Wave 1 — request body hardening
# --------------------------------------------------------------------------- #
def test_read_body_rejects_oversized_payload(server):
    # The DoS vector is a client-declared Content-Length: the server must reject
    # on the declared length *before* reading the body, so we send a tiny body
    # with an oversized declared length (and never stream a real megabyte).
    status, _ = server.raw(
        "POST", "/api/auth/login", body=b'{}',
        headers={"Content-Type": "application/json", "Content-Length": str(MAX_JSON_BODY_BYTES + 1)},
    )
    assert status == 413


def test_read_body_rejects_malformed_json(server):
    bad = b'{"operator_id": "op", '  # truncated / invalid JSON
    status, data = server.raw(
        "POST", "/api/auth/login", body=bad,
        headers={"Content-Type": "application/json", "Content-Length": str(len(bad))},
    )
    assert status == 400
    assert json.loads(data)["error"]["code"] == "BAD_JSON"


def test_read_body_rejects_wrong_content_type(server):
    payload = b'{"operator_id": "op", "pin": "123456"}'
    status, data = server.raw(
        "POST", "/api/auth/login", body=payload,
        headers={"Content-Type": "text/plain", "Content-Length": str(len(payload))},
    )
    assert status == 415
    assert json.loads(data)["error"]["code"] == "UNSUPPORTED_MEDIA_TYPE"


def test_read_body_rejects_non_object_json(server):
    payload = b'[1, 2, 3]'
    status, data = server.raw(
        "POST", "/api/auth/login", body=payload,
        headers={"Content-Type": "application/json", "Content-Length": str(len(payload))},
    )
    assert status == 400
    assert json.loads(data)["error"]["code"] == "BAD_JSON"


def test_bodyless_post_still_works(server):
    # /api/auth/sse-token carries no body; on a fresh (bootstrap) local server it
    # must still succeed — the body hardening must not break bodyless POSTs.
    status, data = server.raw("POST", "/api/auth/sse-token")
    assert status == 200
    assert "sse_token" in json.loads(data)


def test_valid_json_body_accepted(server):
    payload = json.dumps({"operator_id": "nope", "pin": "0000"}).encode("utf-8")
    status, _ = server.raw(
        "POST", "/api/auth/login", body=payload,
        headers={"Content-Type": "application/json; charset=utf-8", "Content-Length": str(len(payload))},
    )
    # Login fails (no such operator) but the body is parsed — not a 400/413/415.
    assert status not in (400, 413, 415)


def test_public_routes_need_no_auth(server):
    for path in ("/api/health", "/api/version"):
        status, _ = server.raw("GET", path)
        assert status == 200, path


# --------------------------------------------------------------------------- #
# Wave 1/6 — firmware workflow path guard
# --------------------------------------------------------------------------- #
def _quoted_ino_patterns(text: str):
    import re
    return re.findall(r"['\"]([^'\"]*\.ino)['\"]", text)


def test_workflow_firmware_paths_resolve():
    """Every quoted *.ino path filter in a workflow must match >=1 real file.

    Guards against the stale `radar_vital_v16_2_0.ino` path filter that silently
    stopped triggering the EXE gate on firmware edits.
    """
    workflows = sorted((ROOT / ".github" / "workflows").glob("*.yml"))
    assert workflows, "no workflow files found"
    checked = 0
    for wf in workflows:
        for pattern in _quoted_ino_patterns(wf.read_text(encoding="utf-8")):
            checked += 1
            matches = list(ROOT.glob(pattern))
            assert matches, f"{wf.name}: firmware path filter '{pattern}' matches no file"
    assert checked > 0, "expected at least one firmware path filter to validate"


# --------------------------------------------------------------------------- #
# Wave 2 — fail-closed operator profile DB
# --------------------------------------------------------------------------- #
def test_absent_profiles_db_is_clean_bootstrap(tmp_path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    db = load_operator_profiles(str(sessions))
    assert db.get("profiles") == {}
    assert "_load_error" not in db  # absent file is a legitimate bootstrap


def test_corrupt_profiles_db_fails_closed(tmp_path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    (sessions / "operator_profiles.json").write_text("{ this is : not json", encoding="utf-8")
    db = load_operator_profiles(str(sessions))
    assert db.get("_load_error"), "corrupt DB must surface a load error"
    assert db.get("profiles") == {}
    # The corrupt file must be preserved (not destroyed) for recovery/forensics.
    assert (sessions / "operator_profiles.json").exists()


def test_bad_schema_profiles_db_fails_closed(tmp_path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    (sessions / "operator_profiles.json").write_text(json.dumps({"unexpected": 1}), encoding="utf-8")
    db = load_operator_profiles(str(sessions))
    assert db.get("_load_error") == "bad_schema"


def test_corrupt_legacy_profiles_db_fails_closed(tmp_path):
    # New-path absent, legacy parent-level DB present but corrupt: migration must
    # fail closed rather than collapse into an empty bootstrap.
    root = tmp_path / "root"
    sessions = root / "sessions"
    sessions.mkdir(parents=True)
    (root / "operator_profiles.json").write_text("@@bad@@", encoding="utf-8")
    db = load_operator_profiles(str(sessions))
    assert db.get("_load_error") == "legacy_unreadable"
    assert db.get("profiles") == {}
    # The legacy file must be preserved (not migrated/destroyed) on error.
    assert (root / "operator_profiles.json").exists()


def test_bad_schema_legacy_profiles_db_fails_closed(tmp_path):
    root = tmp_path / "root"
    sessions = root / "sessions"
    sessions.mkdir(parents=True)
    (root / "operator_profiles.json").write_text(json.dumps({"nope": True}), encoding="utf-8")
    db = load_operator_profiles(str(sessions))
    assert db.get("_load_error") == "legacy_bad_schema"
    assert (root / "operator_profiles.json").exists()


def test_valid_legacy_profiles_db_still_migrates(tmp_path):
    # Sanity: a *valid* legacy DB must still migrate (and not be treated as error).
    root = tmp_path / "root"
    sessions = root / "sessions"
    sessions.mkdir(parents=True)
    legacy = {"schema_version": "rvt-operator-profiles-v12.0", "profiles": {"op_x": {"operator_id": "op_x"}}}
    (root / "operator_profiles.json").write_text(json.dumps(legacy), encoding="utf-8")
    db = load_operator_profiles(str(sessions))
    assert "_load_error" not in db
    assert "op_x" in db.get("profiles", {})
    # Migrated to the new path; old copy removed.
    assert (sessions / "operator_profiles.json").exists()
    assert not (root / "operator_profiles.json").exists()


class _MockServer:
    def __init__(self, sessions_root, bind_mode="lan"):
        self.sessions_root = sessions_root
        self.bind_mode = bind_mode
        self.operator_sessions = {}
        self.sse_tokens = {}


class _MockHandler:
    """Minimal handler shim to exercise _require_control_auth bootstrap logic."""

    def __init__(self, server, path, command="POST"):
        self.server = server
        self.path = path
        self.command = command
        self.headers = {}
        # Loopback (EXE-shell / same-machine) client: this passes the LAN pairing
        # gate, so the bootstrap-vs-_load_error decision is what actually gates
        # the request — exactly the logic under test.
        self.client_address = ("127.0.0.1", 5555)
        self.current_operator_id = None
        self.sent = []

    def _send_json(self, status, obj, **kw):
        self.sent.append((status, obj))


def test_corrupt_db_blocks_lan_bootstrap_admin_create(tmp_path):
    """A corrupt DB must not open the unauthenticated LAN admin-create window."""
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    (sessions / "operator_profiles.json").write_text("@@corrupt@@", encoding="utf-8")
    server = _MockServer(str(sessions), bind_mode="lan")
    handler = _MockHandler(server, "/api/operator-profiles", command="POST")
    allowed = _ControlHandler._require_control_auth(handler)
    assert allowed is False, "corrupt DB should fail closed, not grant bootstrap"
    assert handler.sent and handler.sent[-1][0] == 401


def test_empty_db_allows_lan_bootstrap_admin_create(tmp_path):
    """Sanity: a genuinely empty (absent) DB still permits first-admin bootstrap."""
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _MockServer(str(sessions), bind_mode="lan")
    handler = _MockHandler(server, "/api/operator-profiles", command="POST")
    allowed = _ControlHandler._require_control_auth(handler)
    assert allowed is True


# --------------------------------------------------------------------------- #
# Wave 2 — display-name sanitization
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("bad", [
    "Alice‮Bob",      # right-to-left override (bidi spoof)
    "Bob\x00Smith",        # NUL control char
    "Eve​Mallory",    # zero-width space (invisible)
    "Tab\tName",           # control whitespace
    "Bell\x07",            # bell control
])
def test_display_name_rejects_unsafe_codepoints(bad):
    assert _sanitize_display_name(bad) == ""


@pytest.mark.parametrize("ok", [
    "Dr. José Älvarez",
    "李 医生",
    "Mary-Jane O'Brien",
])
def test_display_name_accepts_normal_unicode(ok):
    assert _sanitize_display_name(ok) != ""


def test_create_profile_rejects_bidi_display_name(tmp_path):
    sessions = tmp_path / "sessions"
    sessions.mkdir()
    server = _MockServer(str(sessions), bind_mode="local")
    status, payload = create_operator_profile(
        server, {"display_name": "Ann‮evil", "initials": "AE", "pin": "123456"}
    )
    assert status == 400
    assert payload["error"]["code"] == "VALIDATION_FAILED"


# --------------------------------------------------------------------------- #
# Wave 4 — NaN/Inf JSON sanitation
# --------------------------------------------------------------------------- #
def test_json_safe_response_sanitizes_nan_and_inf():
    payload = {
        "nan": float("nan"),
        "pos_inf": float("inf"),
        "neg_inf": float("-inf"),
        "np_inf": np.float64("inf"),
        "finite": 12.5,
        "nested": [float("inf"), {"x": float("nan")}],
    }
    raw = _json_safe_response(payload)
    # Must be valid, parseable JSON with no NaN/Infinity tokens.
    text = raw.decode("utf-8")
    assert "Infinity" not in text and "NaN" not in text
    parsed = json.loads(text)  # strict parse would raise on NaN/Infinity
    assert parsed["nan"] is None
    assert parsed["pos_inf"] is None
    assert parsed["neg_inf"] is None
    assert parsed["np_inf"] is None
    assert parsed["finite"] == 12.5
    assert parsed["nested"][0] is None
    assert parsed["nested"][1]["x"] is None


# --------------------------------------------------------------------------- #
# Wave 4 — preflight firmware guidance is current
# --------------------------------------------------------------------------- #
# --------------------------------------------------------------------------- #
# Wave 3 — legacy compare-modal XSS regression
# --------------------------------------------------------------------------- #
LEGACY_PATCHES = ROOT / "web-legacy" / "modules" / "patches" / "legacy-patches.js"


def _legacy_src() -> str:
    return LEGACY_PATCHES.read_text(encoding="utf-8")


def test_compare_modal_sinks_are_escaped():
    """The three compare-modal innerHTML sinks must route dynamic values through
    the escape helper — guards against a regression that drops the escaping."""
    src = _legacy_src()
    assert "function escHtml(v)" in src, "compare-modal escape helper missing"
    for expected in (
        "escHtml(snap.label",
        "escHtml(snap.ts",
        "escHtml(key)",
        "escHtml(val)",
        "escHtml(delta)",
    ):
        assert expected in src, f"compare sink not escaped: {expected}"
    # The pre-fix raw concatenations must be gone.
    assert "'<tr><td>' + key + '</td><td>'" not in src
    assert "'<p class=\"rvt-compare-side-label\">' + (snap.label" not in src


@pytest.mark.skipif(shutil.which("node") is None, reason="node not available")
def test_legacy_escHtml_neutralizes_xss_payload():
    """Run the *actual* escHtml() from source through Node and prove a stored-XSS
    payload becomes inert escaped text (not an executable element)."""
    src = _legacy_src()
    match = re.search(r"function escHtml\(v\) \{.*?\n  \}", src, re.DOTALL)
    assert match, "could not extract escHtml() from legacy-patches.js"
    fn = match.group(0)
    payload = "<img src=x onerror=alert(1)>"
    script = fn + "\nprocess.stdout.write(escHtml(process.argv[1]));"
    out = subprocess.run(
        ["node", "-e", script, payload],
        capture_output=True, text=True, timeout=20, check=True,
    ).stdout
    assert "<img" not in out
    assert "onerror=alert(1)" not in out or "&lt;img" in out
    assert "&lt;img src=x onerror=alert(1)&gt;" == out


def test_preflight_firmware_guidance_points_to_current_firmware():
    runner_src = (ROOT / "rvt_trainer" / "audit" / "runner.py").read_text(encoding="utf-8")
    assert "radar_vital_v16_3_0.ino" in runner_src
    # The stale v15.0.0 firmware-file guidance that contradicts the 219-column
    # contract must be gone.
    assert "v15.0.0 firmware file" not in runner_src
    assert "radar_vital_v15_0_0.ino" not in runner_src
