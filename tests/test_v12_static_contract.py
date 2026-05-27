import ast
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DASH = ROOT / "radar_vital_live_dashboard_v12_for_v16_0.html"
APP = ROOT / "web" / "src" / "app" / "app.ts"
LAYOUT = ROOT / "web" / "src" / "app" / "components" / "layout" / "layout.component.html"
API = ROOT / "web" / "src" / "app" / "services" / "api.service.ts"
TELEMETRY = ROOT / "web" / "src" / "app" / "services" / "telemetry.service.ts"
BLUETOOTH = ROOT / "web" / "src" / "app" / "services" / "bluetooth.service.ts"
STATE = ROOT / "web" / "src" / "app" / "services" / "state.service.ts"
TRAINER = ROOT / "radar_vital_trainer_v12_for_v16_0.py"
TRAINER_MONOLITH = ROOT / "rvt_trainer" / "monolith.py"
FW = ROOT / "radar_vital_v16_0_0.ino"
SW = ROOT / "assets" / "sw.js"
STYLES = ROOT / "web" / "src" / "styles.scss"
BUILD_ANGULAR = ROOT / "scripts" / "build-angular.mjs"
PAGES_WORKFLOW = ROOT / ".github" / "workflows" / "pages.yml"
PLAYWRIGHT_WORKFLOW = ROOT / ".github" / "workflows" / "playwright.yml"
EXE_WORKFLOW = ROOT / ".github" / "workflows" / "build-exe.yml"
TAURI_MAIN = ROOT / "src-tauri" / "src" / "main.rs"
VISUAL_SNAPSHOTS = ROOT / "tests" / "visual" / "rvt-v12.spec.ts-snapshots"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def test_dashboard_pwa_contract():
    html = text(DASH)
    app = text(APP)
    layout = text(LAYOUT)
    api = text(API)
    telemetry = text(TELEMETRY)
    state = text(STATE)
    assert "viewport-fit=cover" in html
    assert "interactive-widget=resizes-content" in html
    assert '<link rel="manifest" href="./manifest.webmanifest">' in html
    assert "register('./sw.js')" in app
    assert "Dashboard update available. Refresh when monitoring is paused." in app
    assert "onAction()" in app
    assert 'id="demoBanner"' in layout
    assert 'role="alert"' in layout
    assert 'href="#mainContent"' in layout
    assert "rvt-pair-token" in api
    assert "X-RVT-Auth" in api
    assert "/api/events/subscribe" in telemetry
    assert "new EventSource" in telemetry
    assert "document.documentElement.dataset['theme']" in state
    assert "this.state.demoMode() || this.state.autoDemoActive()" in telemetry
    assert "PersistenceService" in state
    assert "indexedDB" in text(ROOT / "web" / "src" / "app" / "services" / "persistence.service.ts")
    assert "min-device-memory" not in html
    assert "fonts.googleapis" not in html
    assert "fonts.gstatic" not in html
    assert "cdn.jsdelivr" not in html


def test_service_worker_contract():
    sw = text(SW)
    assert "rvt-shell-v12.0.2" in sw
    assert "text/event-stream" in sw
    assert "SKIP_WAITING" in sw
    assert "SW_UPDATED" in sw
    assert "/api/session/current/live_dashboard.json" in sw
    assert "/fonts/material-symbols-rounded.woff2" in sw


def test_pages_publishes_angular_pwa_shell_not_documentation():
    builder = text(BUILD_ANGULAR)
    workflow = text(PAGES_WORKFLOW)

    assert re.search(r"path\.join\(WWW, '404\.html'\),\s*indexHtml", builder)
    assert 'meta http-equiv="refresh"' not in builder
    assert "path: ./www" in workflow
    assert "grep -q '<app-root></app-root>' www/index.html" in workflow
    assert "grep -q '<app-root></app-root>' www/404.html" in workflow
    assert "markdown-body|README[.]md" in workflow
    assert "cmp -s www/index.html www/404.html" in workflow
    assert "const DASHBOARD = './index.html';" in workflow


def test_visual_ci_runs_on_the_committed_snapshot_platform():
    workflow = text(PLAYWRIGHT_WORKFLOW)
    snapshots = list(VISUAL_SNAPSHOTS.glob("*.png"))

    assert snapshots
    assert all(("-win32.png" in snapshot.name or "-linux.png" in snapshot.name) for snapshot in snapshots)
    assert "name: Visual regression (committed Windows baselines)" in workflow
    assert re.search(r"\n  visual:\n(?:.*\n){1,3}    runs-on: windows-latest", workflow)
    assert "npx playwright install --with-deps chromium webkit" in workflow
    assert "npx playwright install chromium webkit" in workflow
    assert "npx playwright test tests/visual" in workflow


def test_dark_inverse_hc_owns_angular_home_surfaces():
    styles = text(STYLES)

    for token in [
        "--shell-card-bg-solid: #000 !important;",
        "--shell-card-border: #fff !important;",
        "--shell-ink: #fff !important;",
        "--v11-home-card-bg: #000 !important;",
        "--v11-home-control-bg: #000 !important;",
    ]:
        assert token in styles
    assert "body[data-view='home'] app-home :is(" in styles
    assert ".home-stat-card" in styles
    assert ".radar-scope-card" in styles
    assert ".preflight-card-stack" in styles


def test_trainer_routes_and_security_contract():
    py = text(TRAINER_MONOLITH)
    compile(py, str(TRAINER_MONOLITH), "exec")
    for route in [
        "/manifest.webmanifest",
        "/sw.js",
        "/rvt-sw.js",
        "/api/server-info",
        "/api/auth/exchange",
        "/api/events/subscribe",
        "/api/serial/ports",
        "/pair",
    ]:
        assert route in py
    assert "Port {start_port} in use. Stop the other instance or pass --port <N>." in py
    assert "WWW-Authenticate" in py and "RVT-Token" in py
    
    auth_py = text(ROOT / "rvt_trainer" / "api" / "auth.py")
    assert "_PIN_TTL_S = 300" in auth_py
    assert "_PAIR_FAILURE_LIMIT = 5" in auth_py
    assert "PAIRING_RATE_LIMITED" in auth_py
    assert "_exchange_pair_pin(self.server, str(body.get(\"pin\") or \"\"), client_ip)" in py
    
    assert "Content-Security-Policy" in py
    assert "http: https:" not in py
    
    static_py = text(ROOT / "rvt_trainer" / "assets" / "static.py")
    assert ".rvt_tls" in static_py and "target.relative_to(private_root)" in static_py
    
    assert "_qr_png_bytes" in py
    assert '"/api/sessions/") and path.endswith("/signoff")' in py
    assert 'public resource not found' in py
    assert "return super().do_GET()" not in py


def test_manifest_payload_is_plain_manifest():
    py = text(TRAINER_MONOLITH)
    server_info_py = text(ROOT / "rvt_trainer" / "api" / "server_info.py")
    assert '"id": "/"' in server_info_py
    assert '"start_url": "./"' in server_info_py
    assert '"scope": "./"' in server_info_py
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


def test_native_ble_commands_allowlist_reference_gatt_profile():
    bluetooth = text(BLUETOOTH)
    rust = text(TAURI_MAIN)
    workflow = text(EXE_WORKFLOW)

    assert "requireNotificationProfile" in bluetooth
    assert "validateReferenceNotification" in bluetooth
    assert "Only the configured AiLink notification profile is permitted." in bluetooth
    assert "0000ffe0-0000-1000-8000-00805f9b34fb" in bluetooth
    assert "0000ffe2-0000-1000-8000-00805f9b34fb" in bluetooth
    assert 'const AILINK_SERVICE_UUID: &str = "0000ffe0-0000-1000-8000-00805f9b34fb";' in rust
    assert 'const AILINK_NOTIFY_UUID: &str = "0000ffe2-0000-1000-8000-00805f9b34fb";' in rust
    assert "allowed_notification_profile" in rust
    assert "require_active_ble_device" in rust
    assert "permits_only_the_ailink_notification_profile" in rust
    assert "cargo test --manifest-path src-tauri/Cargo.toml --verbose" in workflow


def test_frozen_serial_protocol_contract():
    ino = text(FW)
    trainer = text(TRAINER_MONOLITH)
    module = ast.parse(trainer)
    columns_assignment = next(
        node for node in module.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "RADAR_LOG_COLUMNS" for target in node.targets)
    )
    columns = ast.literal_eval(columns_assignment.value)

    assert len(columns) == 207
    assert columns[-1] == "correction_params_hash"
    assert "EXPECTED_RADAR_LOG_COLUMN_COUNT = 207" in trainer
    assert "#define CSV_COLUMN_COUNT 207" in ino
    assert re.search(r"\bSerial\.begin\(115200\)", ino)


def test_packaging_scaffolds_exist():
    assert (ROOT / "packaging" / "capacitor" / "capacitor.config.ts").exists()
    assert (ROOT / "packaging" / "tauri" / "tauri.conf.json").exists()
    cap_pkg = json.loads((ROOT / "packaging" / "capacitor" / "package.json").read_text(encoding="utf-8"))
    assert "@capacitor-community/http" in cap_pkg["dependencies"]
    tauri = json.loads((ROOT / "packaging" / "tauri" / "tauri.conf.json").read_text(encoding="utf-8"))
    assert tauri["bundle"]["windows"]["webviewInstallMode"]["type"] == "downloadBootstrapper"
    desktop_tauri = json.loads((ROOT / "src-tauri" / "tauri.conf.json").read_text(encoding="utf-8"))
    assert desktop_tauri["bundle"]["windows"]["webviewInstallMode"]["type"] == "downloadBootstrapper"
    assert "connect-src 'self'" in desktop_tauri["app"]["security"]["csp"]
    assert "http://127.0.0.1" not in desktop_tauri["app"]["security"]["csp"]
