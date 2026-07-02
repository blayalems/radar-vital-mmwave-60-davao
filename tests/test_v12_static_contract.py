import ast
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DASH = ROOT / "radar_vital_live_dashboard_v12_for_v16_0.html"
APP = ROOT / "web" / "src" / "app" / "app.ts"
LAYOUT = ROOT / "web" / "src" / "app" / "components" / "layout" / "layout.component.html"
LAYOUT_TS = ROOT / "web" / "src" / "app" / "components" / "layout" / "layout.component.ts"
API = ROOT / "web" / "src" / "app" / "services" / "api.service.ts"
APP_META = ROOT / "web" / "src" / "app" / "services" / "app-meta.ts"
TELEMETRY = ROOT / "web" / "src" / "app" / "services" / "telemetry.service.ts"
BLUETOOTH = ROOT / "web" / "src" / "app" / "services" / "bluetooth.service.ts"
SW_UPDATE = ROOT / "web" / "src" / "app" / "services" / "sw-update.service.ts"
STATE = ROOT / "web" / "src" / "app" / "services" / "state.service.ts"
TRAINER = ROOT / "radar_vital_trainer_v12_for_v16_0.py"
TRAINER_MONOLITH = ROOT / "rvt_trainer" / "monolith.py"
FW = ROOT / "radar_vital_v16_4_0.ino"
SW = ROOT / "assets" / "sw.js"
STYLES = ROOT / "web" / "src" / "styles.scss"
BUILD_ANGULAR = ROOT / "scripts" / "build-angular.mjs"
RELEASE_MANIFEST_GENERATOR = ROOT / "scripts" / "generate-rvt-latest.mjs"
PAGES_WORKFLOW = ROOT / ".github" / "workflows" / "pages.yml"
PLAYWRIGHT_WORKFLOW = ROOT / ".github" / "workflows" / "playwright.yml"
RELEASE_ARTIFACTS_WORKFLOW = ROOT / ".github" / "workflows" / "release-artifacts.yml"
EXE_WORKFLOW = ROOT / ".github" / "workflows" / "build-exe.yml"
TAURI_MAIN = ROOT / "src-tauri" / "src" / "main.rs"
VISUAL_SNAPSHOTS = ROOT / "tests" / "visual" / "rvt-v12.spec.ts-snapshots"
PATCH_ANDROID_SHELL = ROOT / "scripts" / "patch-android-shell.mjs"
ROOT_PACKAGE = ROOT / "package.json"
ROOT_PACKAGE_LOCK = ROOT / "package-lock.json"
TAURI_CARGO = ROOT / "src-tauri" / "Cargo.toml"
TAURI_CONF = ROOT / "src-tauri" / "tauri.conf.json"
PACKAGING_TAURI_CONF = ROOT / "packaging" / "tauri" / "tauri.conf.json"
PACKAGING_CAP_PACKAGE = ROOT / "packaging" / "capacitor" / "package.json"
PACKAGING_CAP_PACKAGE_LOCK = ROOT / "packaging" / "capacitor" / "package-lock.json"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def test_dashboard_pwa_contract():
    html = text(DASH)
    app = text(APP)
    layout = text(LAYOUT)
    layout_ts = text(LAYOUT_TS)
    api = text(API)
    telemetry = text(TELEMETRY)
    state = text(STATE)
    assert "viewport-fit=cover" in html
    assert "interactive-widget=resizes-content" in html
    assert '<link rel="manifest" href="./manifest.webmanifest">' in html
    sw_update = text(SW_UPDATE)
    assert "register('./sw.js')" in sw_update
    assert "ServiceWorkerRegistration" in sw_update
    assert "registration.update()" in sw_update
    assert "SKIP_WAITING" in sw_update
    assert "afterClosed()" in sw_update
    assert 'id="demoBanner"' in layout
    assert 'role="alert"' in layout
    assert 'href="#mainContent"' in layout
    assert "rvt-pair-token" in api or "rvt-pair-token" in text(ROOT / "web" / "src" / "app" / "services" / "rvt-storage-keys.ts")
    assert "X-RVT-Auth" in api
    assert "/api/events/subscribe" in telemetry
    assert "new EventSource" in telemetry
    assert "document.documentElement.dataset['theme']" in state
    assert "this.state.demoMode() || this.state.autoDemoActive()" in telemetry
    assert "PersistenceService" in state
    assert "indexedDB" in text(ROOT / "web" / "src" / "app" / "services" / "persistence.service.ts")
    # M3 three-tier navigation: bottom nav <600px, collapsed icon rail
    # 600–1023px, labelled rail >=1024px. The rail is shown from 600px and
    # forced into its icon-only form across the tablet band.
    assert "this.breakpointObserver.observe('(min-width: 600px)')" in layout_ts
    assert "(min-width: 600px) and (max-width: 1023.98px)" in layout_ts
    assert "min-device-memory" not in html
    assert "fonts.googleapis" not in html
    assert "fonts.gstatic" not in html
    assert "cdn.jsdelivr" not in html


def test_service_worker_contract():
    sw = text(SW)
    # Cache version bumped v12.0.5 -> v12.0.6 (kept in the v12 lineage) so the
    # activate handler purges stale pre-redesign shells and re-precaches the
    # self-hosted icon/UI fonts — fixes icons rendering as ligature text on
    # already-cached PWA clients.
    assert "rvt-shell-v12.0.6" in sw
    assert "text/event-stream" in sw
    assert "SKIP_WAITING" in sw
    assert "SW_UPDATED" in sw
    assert "request.mode === 'navigate'" in sw
    assert "/api/session/current/live_dashboard.json" in sw
    assert "/fonts/material-symbols-rounded.woff2" in sw


def test_pr46_product_identity_bump_preserves_v12_lineage():
    trainer = text(TRAINER_MONOLITH)
    firmware = text(FW)
    app_meta = text(APP_META)
    api = text(API)
    sw = text(SW)

    assert json.loads(text(ROOT_PACKAGE))["version"] == "16.4.0"
    assert json.loads(text(ROOT_PACKAGE_LOCK))["version"] == "16.4.0"
    assert json.loads(text(TAURI_CONF))["version"] == "16.4.0"
    assert json.loads(text(PACKAGING_TAURI_CONF))["version"] == "16.4.0"
    assert json.loads(text(PACKAGING_CAP_PACKAGE))["version"] == "16.4.0"
    assert json.loads(text(PACKAGING_CAP_PACKAGE_LOCK))["version"] == "16.4.0"
    assert re.search(r'^version = "16\.4\.0"$', text(TAURI_CARGO), re.MULTILINE)

    assert 'VERSION = "16.4.0"' in trainer
    assert 'DASHBOARD_VERSION = "16.4.0"' in trainer
    assert 'FIRMWARE_VERSION_EXPECTED = "v16.4.0"' in trainer
    assert "PRODUCT_VERSION = '16.4.0';" in app_meta
    assert "PRODUCT_VERSION_SHORT = 'v16.4';" in app_meta
    assert "PRODUCT_VERSION_LABEL = 'App v16.4';" in app_meta
    assert "import { PRODUCT_VERSION } from './app-meta';" in api
    assert "product_version: PRODUCT_VERSION" in api
    assert '#define FW_VERSION "v16.4.0"' in firmware
    assert "#define SKETCH_VERSION_MAJOR 16" in firmware
    assert "#define SKETCH_VERSION_SUB 4" in firmware
    assert "#define SKETCH_VERSION_MOD 0" in firmware

    for schema_id in [
        "rvt-control-api-v12.0",
        "rvt-session-notes-v12.0",
        "rvt-session-signoff-v12.0",
        "rvt-training-progress-v12.0",
        "rvt-live-events-v12.0",
        "rvt-session-manifest-v12.0",
        "rvt-chart-annotations-v12.0",
        "rvt-subject-profiles-v12.0",
    ]:
        assert schema_id in trainer
    assert "rvt-shell-v12.0.6" in sw
    assert "rvt-shell-v16" not in sw


def test_release_manifest_contract_and_ci_validation():
    generator = text(RELEASE_MANIFEST_GENERATOR)
    release_workflow = text(RELEASE_ARTIFACTS_WORKFLOW)
    playwright_workflow = text(PLAYWRIGHT_WORKFLOW)

    assert "artifact_entries" in generator
    assert "size_bytes" in generator
    assert "sha256" in generator
    assert "release_tag" in generator
    assert "release_version" in generator
    assert "build_number" in generator
    assert "version_code" in generator
    assert "compatibility" in generator
    assert "manual_download_guidance" in generator
    assert "process.env.RELEASE_TAG" in generator
    assert "process.env.ANDROID_VERSION_CODE" in generator
    assert "releases/download/${options.releaseTag}" in generator

    assert "RELEASE_TAG: ${{ needs.release_metadata.outputs.release_tag }}" in release_workflow
    assert "ANDROID_VERSION_CODE: ${{ needs.release_metadata.outputs.android_version_code }}" in release_workflow
    assert "node scripts/generate-rvt-latest.mjs" in release_workflow
    assert "dist/rvt-latest.json" in release_workflow
    assert "startsWith(github.ref_name, 'v16.4.0-alpha')" not in release_workflow
    assert "contains(github.ref_name, '-alpha')" in release_workflow
    assert "contains(github.ref_name, '-rc')" in release_workflow
    assert "actions/deploy-pages@v4" in release_workflow
    assert "group: pages" in release_workflow
    assert "gh-pages" not in release_workflow
    assert "Preserve latest release manifest" in text(PAGES_WORKFLOW)
    assert "rvt-latest.json" in text(PAGES_WORKFLOW)
    assert "node scripts/generate-rvt-latest.mjs --self-test" in playwright_workflow


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
    assert all("-win32.png" in snapshot.name for snapshot in snapshots)
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
        "/api/server-info",
        "/api/auth/exchange",
        "/api/events/subscribe",
        "/api/serial/ports",
        "/pair",
    ]:
        assert route in py
    assert 'path == "/rvt-sw.js"' in py
    assert '"application/javascript; charset=utf-8"' in py
    assert "LEGACY_SW_TOMBSTONE_NOT_FOUND" in py
    assert "SERVICE_WORKER_TOMBSTONE_REMOVED" not in py
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
    assert '#define FW_VERSION "v16.4.0"' in ino
    assert "#define ENABLE_BLE false" in ino
    assert "NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN" in ino
    assert "bleSuppressUntilMs = millis() + 500UL" in ino
    assert "delay(500)" not in ino
    assert "hrsValid = validHr" in ino
    assert "hrBpm >= 1.0f && hrBpm <= 254.0f" in ino


def test_firmware_new_audit_requirements():
    ino = text(FW)
    # 1. CSV Column Count Runtime Enforcement
    assert "int _csvColCount = 0;" in ino or "static int _csvColCount = 0;" in ino
    assert "_csvColCount++" in ino
    assert "[CONTRACT] CSV column count mismatch" in ino
    assert "_csvFirstEmitDone" in ino

    # 2. No Duplicate BLE Define
    assert ino.count("#define ENABLE_BLE false") == 1
    assert ino.count("#ifndef ENABLE_BLE") == 1

    # 3. Deprecated Constant Removed
    assert "CHIP_HR_BIAS_CORRECTION_BPM" not in ino or "removed" in ino
    assert "static const float CHIP_HR_BIAS_CORRECTION_BPM" not in ino

    # 4. BLE callbacks stubbed
    assert "// STUB — no-op until BLE path activates" in ino

    # 5. Legacy alias cleanup
    assert "// REMOVE at v17 — use RLS_LAMBDA_HR_BASE directly" in ino


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

    assert len(columns) == 222
    assert columns[206] == "correction_params_hash"
    assert columns[-15:] == [
        "loop_dt_mean_ms",
        "loop_dt_max_ms",
        "heap_free_kb",
        "heap_min_free_kb",
        "radar_uart_overflow_count",
        "radar_crc_err_count",
        "i2c_recover_count",
        "lcd_reinit_count",
        "wdt_near_miss_count",
        "cmd_rx_count",
        "cmd_err_count",
        "uart_rx_high_water",
        "hr_publish_tier",
        "rr_publish_tier",
        "fw_uptime_s",
    ]
    assert "EXPECTED_RADAR_LOG_COLUMN_COUNT = 222" in trainer
    assert "LEGACY_V15_1_COLUMN_COUNT = 219" in trainer
    assert "LEGACY_V15_COLUMN_COUNT = 207" in trainer
    assert "#define CSV_COLUMN_COUNT 222" in ino
    assert re.search(r"\bSerial\.begin\(115200\)", ino)


def test_trainer_pads_legacy_207_rows_to_v15_1_contract():
    from rvt_trainer import monolith as m

    kind, row, detail = m._parse_radar_data_line(
        "DATA," + ",".join(["1"] * m.LEGACY_V15_COLUMN_COUNT),
        m.RADAR_LOG_COLUMNS,
    )

    assert kind == "data", detail
    assert row is not None
    assert len(row) == m.EXPECTED_RADAR_LOG_COLUMN_COUNT
    assert row[: m.LEGACY_V15_COLUMN_COUNT] == ["1"] * m.LEGACY_V15_COLUMN_COUNT
    assert row[m.LEGACY_V15_COLUMN_COUNT :] == [""] * (
        m.EXPECTED_RADAR_LOG_COLUMN_COUNT - m.LEGACY_V15_COLUMN_COUNT
    )


def test_firmware_pr58_robustness_guards_present():
    ino = text(FW)

    assert "PERIPH_BACKOFF_MIN_MS = 3000UL" in ino
    assert "PERIPH_BACKOFF_MAX_MS = 300000UL" in ino
    assert "periphBackoffFailure(lcdBackoff" in ino
    assert "periphBackoffFailure(mlxBackoff" in ino
    assert "periphBackoffFailure(bh1750Backoff" in ino
    assert "periphBackoffReady(lcdBackoff" in ino
    assert "periphBackoffReady(mlxBackoff" in ino
    assert "periphBackoffReady(bh1750Backoff" in ino
    assert '[BOOT] reset_reason=%d (%s)' in ino
    assert 'prefs.putUInt("rstSlot"' in ino
    assert "nvsWriteFailureStreak >= 3" in ino
    assert "nvsWriteDisabledForBoot = true" in ino
    assert "[NVS] write_count_boot=" in ino


def test_firmware_pr59_power_thermal_guards_present():
    ino = text(FW)

    assert "#define RV_POWER_SAVE 0" in ino
    assert "#define RV_LCD_LUX_BACKLIGHT RV_POWER_SAVE" in ino
    assert "const unsigned long BH1750_RETRY_INTERVAL_MS" not in ino
    assert "lastBh1750RetryMs =" not in ino
    assert "POWER_SAVE_ABSENT_MS = 60000UL" in ino
    assert "POWER_SAVE_MLX_INTERVAL_MS = 10000UL" in ino
    assert "setCpuFrequencyMhz(80)" in ino
    assert "getCpuFrequencyMhz()" in ino
    assert "cpu frequency read as 0 before idle power save" in ino
    assert "setLcdBacklightDimmed(true)" in ino
    assert "setLcdBacklightDimmed(false)" in ino
    assert "if (powerSaveActive && brite > 4) brite = 4;" in ino
    assert "if (powerSaveActive) mlxReadInterval = POWER_SAVE_MLX_INTERVAL_MS;" in ino
    assert "THERMAL_SAMPLE_INTERVAL_MS = 10000UL" in ino
    assert "THERMAL_WARN_C = 75.0f" in ino
    assert "temperatureRead()" in ino
    assert "[THERMAL] chip_temp_c=" in ino
    assert "LCD_DIM_LUX_ON = 8.0f" in ino
    assert "LCD_DIM_LUX_OFF = 15.0f" in ino
    assert "#define CSV_COLUMN_COUNT 222" in ino


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


def test_android_package_does_not_backup_local_session_state():
    patcher = text(PATCH_ANDROID_SHELL)
    workflow = text(ROOT / ".github" / "workflows" / "build-apk.yml")

    assert "node scripts/patch-android-shell.mjs" in workflow
    assert "ensureApplicationAttribute(manifest, 'android:allowBackup', 'false')" in patcher
    assert "ensureApplicationAttribute(manifest, 'android:dataExtractionRules', '@xml/data_extraction_rules')" in patcher
    assert '<cloud-backup disableIfNoEncryptionCapabilities="true">' in patcher
    assert '<device-transfer>' in patcher
    assert patcher.count('<exclude domain="root" path="." />') == 2


def test_207_column_row_parses_without_warning():
    from rvt_trainer.monolith import _parse_radar_data_line, RADAR_LOG_COLUMNS
    # Construct a 207-column CSV row (DATA prefix + 207 fields)
    # 207 fields means 206 commas after DATA.
    line = "DATA,1000" + "," * 206
    kind, row, detail = _parse_radar_data_line(line, RADAR_LOG_COLUMNS)
    assert kind == "data"
    assert row is not None
    assert len(row) == 222
    assert detail == ""
    assert row[0] == "1000"
    # Verify that the added columns (indices 207 to 221) are padded with empty strings
    for i in range(207, 222):
        assert row[i] == ""

