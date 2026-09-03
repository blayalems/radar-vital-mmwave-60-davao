"""Operational probe executed with the isolated, installed wheel interpreter."""

from __future__ import annotations

import http.client
import json
import tempfile
from pathlib import Path

from rvt_trainer.monolith import _ControlServer
from rvt_trainer.runtime_paths import runtime_root


def request(port: int, path: str) -> tuple[int, dict[str, str], bytes]:
    connection = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
    try:
        connection.request("GET", path)
        response = connection.getresponse()
        return response.status, dict(response.getheaders()), response.read()
    finally:
        connection.close()


def main() -> None:
    root = runtime_root()
    required_resources = (
        root / "radar_vital_live_dashboard_v12_for_v16_0.html",
        root / "radar_vital_v16_6_3.ino",
        root / "assets" / "fonts" / "rvt-fonts.css",
        root / "assets" / "icons" / "icon-192.png",
        root / "assets" / "sw.js",
        root / "quality" / "statistical-analysis-plan.json",
    )
    missing = [str(path) for path in required_resources if not path.is_file()]
    assert not missing, f"installed wheel runtime resources are missing: {missing}"

    with tempfile.TemporaryDirectory(prefix="rvt-wheel-probe-") as temp_dir:
        sessions = Path(temp_dir) / "sessions"
        server = _ControlServer(
            "127.0.0.1",
            0,
            str(sessions),
            bind_mode="local",
            mock=True,
        )
        server.start()
        try:
            port = server.httpd.server_port

            status, _, body = request(port, "/api/health")
            assert status == 200
            assert json.loads(body)["ok"] is True

            status, _, body = request(port, "/")
            assert status == 200
            assert b"<!doctype html" in body[:256].lower()

            status, headers, body = request(port, "/fonts/rvt-fonts.css")
            assert status == 200
            assert headers.get("Content-Type", "").startswith("text/css")
            assert b"@import" in body or b"font-family" in body

            status, headers, body = request(port, "/icons/icon-192.png")
            assert status == 200
            assert headers.get("Content-Type") == "image/png"
            assert body.startswith(b"\x89PNG\r\n\x1a\n")

            status, headers, body = request(port, "/sw.js")
            assert status == 200
            assert "javascript" in headers.get("Content-Type", "")
            assert b"serviceWorker" in body or b"addEventListener" in body

            status, _, body = request(port, "/api/study/readiness")
            assert status == 200
            readiness = json.loads(body)
            assert readiness["authorized"] is False
            assert readiness["plan_status"] == "draft"
            assert "authorization_record_missing_or_invalid" in readiness["blockers"]
        finally:
            server.stop()


if __name__ == "__main__":
    main()
