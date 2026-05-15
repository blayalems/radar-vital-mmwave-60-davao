import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DASH = ROOT / "radar_vital_live_dashboard_v12_for_v16_0.html"
TRAINER = ROOT / "radar_vital_trainer_v12_for_v16_0.py"
FW = ROOT / "radar_vital_v16_0_0.ino"
SW = ROOT / "assets" / "sw.js"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def test_dashboard_pwa_contract():
    html = text(DASH)
    assert "viewport-fit=cover" in html
    assert "interactive-widget=resizes-content" in html
    assert '<link rel="manifest" href="/manifest.webmanifest">' in html
    assert "/sw.js" in html
    assert "demoBanner" in html
    assert "RVT-v12" in html
    assert "rvt-pair-token" in html
    assert "apiES('/api/events/subscribe')" in html
    assert "X-RVT-Auth" in html
    assert "S.apiBase" in html
    assert "EventSource(base + path)" in html
    assert "min-device-memory" not in html
    assert "fonts.googleapis" not in html
    assert "fonts.gstatic" not in html
    assert "cdn.jsdelivr" not in html


def test_service_worker_contract():
    sw = text(SW)
    assert "rvt-shell-v12.0.0" in sw
    assert "text/event-stream" in sw
    assert "SKIP_WAITING" in sw
    assert "SW_UPDATED" in sw
    assert "/api/session/current/live_dashboard.json" in sw
    assert "/fonts/material-symbols-rounded.woff2" in sw


def test_trainer_routes_and_security_contract():
    py = text(TRAINER)
    compile(py, str(TRAINER), "exec")
    for route in [
        "/manifest.webmanifest",
        "/sw.js",
        "/rvt-sw.js",
        "/api/server-info",
        "/api/auth/exchange",
        "/api/events/subscribe",
        "/pair",
    ]:
        assert route in py
    assert "Port {start_port} in use. Stop the other instance or pass --port <N>." in py
    assert "WWW-Authenticate" in py and "RVT-Token" in py
    assert "_PIN_TTL_S = 300" in py
    assert "Content-Security-Policy" in py
    assert "http: https:" not in py
    assert ".rvt_tls" in py and "target.relative_to(private_root)" in py
    assert "_qr_png_bytes" in py


def test_manifest_payload_is_plain_manifest():
    py = text(TRAINER)
    assert '"id": "/"' in py
    assert '"start_url": "./live_dashboard.html"' in py
    assert '"scope": "./"' in py
    assert 'json.dumps(_manifest_payload(self.server)' in py
    assert 'content_type="application/manifest+json' not in py


def test_firmware_ble_contract():
    ino = text(FW)
    assert '#define FW_VERSION "v16.0.0"' in ino
    assert "#define ENABLE_BLE false" in ino
    assert "NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN" in ino
    assert "bleSuppressUntilMs = millis() + 500UL" in ino
    assert "delay(500)" not in ino
    assert "hrsValid = validHr" in ino
    assert "hrBpm >= 1.0f && hrBpm <= 254.0f" in ino


def test_packaging_scaffolds_exist():
    assert (ROOT / "packaging" / "capacitor" / "capacitor.config.ts").exists()
    assert (ROOT / "packaging" / "tauri" / "tauri.conf.json").exists()
    cap_pkg = json.loads((ROOT / "packaging" / "capacitor" / "package.json").read_text(encoding="utf-8"))
    assert "@capacitor-community/http" in cap_pkg["dependencies"]
    tauri = json.loads((ROOT / "packaging" / "tauri" / "tauri.conf.json").read_text(encoding="utf-8"))
    assert tauri["bundle"]["windows"]["webviewInstallMode"]["type"] == "downloadBootstrapper"
