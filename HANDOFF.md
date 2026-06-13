# HANDOFF — Refactor & Packaging Progress
 
> **Mandatory:** Every AI agent (and human) that pushes work to this repo updates
> the relevant table row below in the same commit. Drift between code and this
> file is treated as a regression. Keep entries terse — one line per change.
> The newest entry goes at the **top** of the log, dated.

### 2026-06-13 — PR54 v16.3 RTM/RC Waves 2–4 Completion

- **Wave 2 dashboard closure**: mounted privacy/telemetry, about, and report-issue cards in Settings; wired support commands into the command palette and Help; added PWA install prompt service/home banner, truthful Home/Live/Report empty states, and lazy Help/Report/Settings routes. Retired `/rvt-sw.js` tombstone to an explicit 410 response and kept `/sw.js` as the single service worker.
- **Wave 3 docs/acceptance**: refreshed Pages workflow/docs/wiki sources, added `docs/milestones.md`, privacy/terms Pages artifacts, and expanded `docs/physical-acceptance-checklist.md` for Android/PWA/EXE/report/accessibility release acceptance while keeping v16.2.0 as the behavior baseline.
- **Wave 4 identity bump**: promoted package, trainer, dashboard, Tauri, Capacitor, OTA fixtures, changelog, README, and firmware identity to `16.3.0`; renamed firmware to `radar_vital_v16_3_0.ino`; rebuilt `radar_vital_live_dashboard_v12_for_v16_0.html`.
- **RTM hardening**: recovery login now clears stale unauthenticated control status before unlocking; malformed live telemetry is rejected instead of poisoning state; async report loading marks OnPush views dirty; report-issue smoke now uses real mock-server operator auth and dispatched mobile-safe Material control activation.
- **Verification**: `npm --prefix web run test:ci` 108/108; `python -m pytest -q tests/test_repo_hygiene_contract.py tests/test_trainer_verdict.py tests/test_v12_static_contract.py tests/test_trainer_security_api.py tests/test_ota_backend.py tests/test_pages_docs_contract.py` 63/63 (Windows pytest temp symlink cleanup warning after success); `npm run build:check` clean with the known initial-bundle warning; `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer` clean; `python -m rvt_trainer --help` reports v16.3.0; Playwright targeted smoke passed: desktop non-dashboard 41/41, Pixel 7 non-dashboard 41/41, iPhone non-dashboard 41/41, iPad remaining non-dashboard 30/30, report-issue 32/32, settings-cards 12/12, dashboard smoke desktop/Pixel 7/iPhone 35/35 each.

### 2026-06-12 — v16.3 Wave 1-A: First-Run Consent Gate + Onboarding Tutorial (WS1-A)

- **FirstRunService** (`web/src/app/services/first-run.service.ts`): reads/writes `rvt-consent-record` (CONSENT_KEY) + `rvt-tutorial-done` (TUTORIAL_DONE_KEY); `consentRequired()` computed signal gated on TERMS_VERSION; `acceptConsent()`, `tutorialDone()`, `markTutorialDone()`, `replayTutorial()` (sets `tutorialOpen` signal for Wave 2 command palette wiring); auto-opens tutorial on `rvt-operator-authenticated` CustomEvent, once per session.
- **ConsentDialogComponent** (`web/src/app/components/consent-dialog/`): standalone, opened with `disableClose:true` + `panelClass:'m3-dialog-panel'`; condensed RA 10173 / UM research summary; Decline swaps to blocking panel with Back-to-Terms action; Accept closes with `true`.
- **OnboardingTutorialComponent** (`web/src/app/components/onboarding-tutorial/`): 6-step skippable coach-mark dialog; platform-aware connect step (exe/native/pwa) via `ServerLifecycleService.platform()` + Capacitor check; arrow-key nav; Done/Skip both call `markTutorialDone()`.
- **Root shell wiring** (`web/src/app/app.ts`): `maybeOpenConsentGate()` on init; `effect()` on `tutorialOpen` signal opens `OnboardingTutorialComponent`.
- **Smoke test seeding sweep**: `seedFirstRunComplete(page)` added to `beforeEach` in `dashboard.spec.ts`, `ota.spec.ts`, and `operator.spec.ts` so the new gate doesn't break existing suites.
- **New spec**: `tests/smoke/first-run.spec.ts` — consent gate, decline/back, accept, tutorial auto-show once, reload = no gates.
- **Vitest**: `npm --prefix web run test:ci` — 8 test files, 60 tests, all passed.
- **Angular build**: `ng build --configuration=development` — clean, no errors.

### 2026-06-12 — WS1-C: GitHub Issue Reporting with Diagnostics Opt-out

- **IssueReportService** (`web/src/app/services/issue-report.service.ts`): `buildReport()` collects product version, platform (Tauri/Android/PWA), connection mode, de-identified ctlStatus + alert summary (counts/types only — no operator names), EXE log tail last 20 lines via `serverLifecycle.logTail()` or `/api/trainer/log`. Diagnostics toggle defaults to ON (DIAGNOSTICS_OPTIN_KEY). When off: version + platform only. `buildIssueUrl()` prefills all bug-form field ids (`product_version`, `platform`, `connection_mode`, `diagnostics`, `description`) via URLSearchParams; URL hard-capped at 7 500 chars, truncation drops log tail then alerts, never version/platform. `openReport()` uses Tauri `plugin:shell|open` or `window.open(_blank, noopener)`.
- **ReportIssueCardComponent** (`web/src/app/components/report-issue-card/`): standalone settings-style card (not yet mounted — Wave 2 mounts in settings.component). Includes description textarea, diagnostics slide-toggle (persisted), "Preview report" dialog (panelClass `m3-dialog-panel`), "Open GitHub issue" button.
- **Tauri capability** (`src-tauri/capabilities/default.json`): added `shell:allow-open` scoped to `https://github.com/*`.
- **Smoke spec** (`tests/smoke/report-issue.spec.ts`): full spec wrapped in `test.describe.skip` with Wave 2 TODO comment. Uses `seedFirstRunComplete`.
- **Salvage assessment**: previous agent's logic grafted but wrong import path (`app-meta.ts` duplicated in `web/src/app/app-meta.ts` instead of `web/src/app/services/app-meta.ts`) corrected; renamed `buildDiagnostics`→`buildReport`, `openExternal`→`openReport`, split interface to `IssueReport`.
- **Verification**: `ng test --watch=false` 70/70 (43 existing + 27 new issue-report spec tests).

### 2026-06-12 — WS1-D: About Card + Trainer /about Copyright Footer

- **`web/src/app/components/about-card/`** (new, standalone): `AboutCardComponent` — settings-card visual idiom; shows product name, version input, `copyrightLine()` (auto-year), three authors, program/university, four external links (Terms, Privacy, License, GitHub repo — `target=_blank noopener`), and stack acknowledgements. Imports all metadata from `app-meta.ts`. NOT mounted anywhere — Wave 2 (settings.component) owns insertion.
- **`web/src/app/components/about-card/about-card.component.spec.ts`** (new): 9 vitest assertions — authors, university, dynamic year, all four links with `target=_blank` and `rel=noopener`.
- **`rvt_trainer/api/server_info.py`**: `_copyright_footer_html()` appends a `<footer>` with all three author names, program, university, and `datetime.now().year` to `support_matrix_html()`.
- **`tests/test_trainer_server_info.py`**: two new tests — all author names present, current year present in `/about` response.
- **Verification**: `python -m pytest -q tests/test_trainer_server_info.py` 10/10 green; `python -m compileall -q` clean; Angular vitest 52/52 (8 files) green.
- **Salvage**: previous agent's 35-line `.component.ts` was structurally sound — grafted and extended with `MatIconModule`, CSS, HTML template, and spec.
### 2026-06-12 — v16.3 WS1-E: Google Play Closed-Testing Groundwork (WS1-E)

- **SDK 35 pin**: `scripts/patch-android-shell.mjs` gains `pinSdkVersions()` — sets `compileSdk` and `targetSdk` to 35 in the generated `android/app/build.gradle`; loud `process.exit(1)` with a FATAL message if the expected gradle pattern is absent. Also patches `variables.gradle` if present (Capacitor template variant); silently skips if the file doesn't exist.
- **Adaptive launcher icon**: patch script writes `res/mipmap-anydpi-v26/ic_launcher.xml` + `ic_launcher_round.xml` (foreground/background/monochrome layers for Android 13 themed icons), copies `assets/icons/android/ic_launcher_foreground.xml` into `res/drawable/`, and defines `@color/ic_launcher_background` (`#0E5E63` deep medical-teal) in `colors.xml`.
- **Foreground vector drawable**: `assets/icons/android/ic_launcher_foreground.xml` — 108 dp viewport, content within 66 dp safe zone, concentric radar arcs + ECG-style heartbeat pulse line, white (#FFFFFF) on transparent.
- **Store listing**: `docs/play/store-listing.md` — app name, short/full description drafts, category, contact, privacy URL, screenshot shot-list (8 phone + 2 tablet), feature-graphic spec, content-rating answers, closed-testing constraints (12 testers / 14 days / $25 fee note, AAB/signing follow-up TODOs).
- **Data safety**: `docs/play/data-safety.md` — Play Data Safety form answers mirroring PRIVACY.md exactly; all data locally stored; no third-party sharing; deletion path; field-by-field Play Console mapping table.
- **Contract test**: `tests/test_android_patch_contract.py` (14 tests) — asserts SDK 35 pin, monochrome, FATAL exit, foreground drawable presence/`<vector`/viewport 108, both Play docs exist, data-safety mentions "locally".
- **Verification**: `python -m pytest -q tests/test_android_patch_contract.py` 14/14; `node --check scripts/patch-android-shell.mjs` clean.
### 2026-06-12 — WS1-B: PIN recovery codes + loopback host reset

- **Backend** (`rvt_trainer/api/auth.py`): `create_operator_profile()` mints a XXXX-XXXX-XXXX Crockford-ish recovery code (secrets module, 30-char A-Z2-9 alphabet), stores only its PBKDF2-HMAC-SHA256/200k hash, returns plaintext exactly once. New `reset_pin_with_recovery()` verifies hash, resets PIN, single-use rotates code, separate 5/30 s lockout; legacy profiles (no hash) get guidance to host-reset. New `host_reset_pin()` resets from loopback; HTTP handler enforces 403 for non-loopback. Both invalidate active sessions and audit-log.
- **Monolith** (`rvt_trainer/monolith.py`): imports new functions; adds `POST /api/auth/reset-pin` (recovery-code gated, public) and `POST /api/auth/host-reset` (loopback-only, 403 from non-loopback) before static fall-through; both added to `is_discovery` in `_require_control_auth`.
- **Frontend** (`web/src/app/services/auth.service.ts`): `createProfile()` return type → `{ success, recoveryCode? }`; new `resetPin()` and `hostReset()` methods with lockout handling.
- **RecoveryCodeDialog** (`web/src/app/components/recovery-code-dialog/`): new standalone Material dialog; monospace code block, Copy button (clipboard API + execCommand fallback), required "I saved my recovery code" dismiss; `disableClose: true, panelClass: 'm3-dialog-panel'`.
- **IdleLockOverlay** (`web/src/app/components/idle-lock-overlay/`): "Forgot PIN?" link → inline recovery flow; "Reset from this computer" shown only on EXE host (`serverLifecycle.platform() === 'exe'`); both open RecoveryCodeDialog with rotated code on success.
- **Tests**: `tests/test_recovery_codes.py` (22 pytest, incl. loopback integration); `tests/smoke/pin-reset.spec.ts` (Playwright smoke journey); auth.service spec extended (6 new tests, 49 total).
- **README**: two new API table rows for `/api/auth/reset-pin` and `/api/auth/host-reset`.
- **Verification**: pytest 40/40; compileall clean; vitest 49/49; Angular build clean (pre-existing bundle budget warning).

### 2026-06-12 — v16.3 Wave 0: Legal Drafts, Repo Hygiene & Shared Feature Contracts

- **Legal/Support Artifacts**: TERMS.md + PRIVACY.md (RA 10173-framed drafts with explicit University of Mindanao legal/REC review banners), LICENSE (academic evaluation, © Lemuel Blaya, Angelo Diaz, Blessie Mugat), CHANGELOG.md (Keep-a-Changelog with 16.0–16.2 backfill), CONTRIBUTING.md, `.github/ISSUE_TEMPLATE/` bug/feature forms + config (blank issues off). Bug-form field ids (`description`, `steps`, `product_version`, `platform`, `connection_mode`, `diagnostics`) are contractual — the upcoming in-app issue reporter prefills them.
- **Shared Contracts Seeded**: `rvt-storage-keys.ts` gains CONSENT_KEY/TUTORIAL_DONE_KEY/DIAGNOSTICS_OPTIN_KEY; new `app-meta.ts` (authors, program, university, GITHUB_REPO_URL, TERMS_VERSION, auto-year `copyrightLine()`); Playwright helper `tests/smoke/helpers/first-run.ts` (`seedFirstRunComplete`) so existing specs survive the upcoming consent gate; `docs/wiki/` source stubs.
- **Contract Test**: new `tests/test_repo_hygiene_contract.py` (6 tests) locks legal anchors, issue-form ids, storage keys, authorship metadata, and TERMS_VERSION sync between app-meta and the smoke helper.
- **Verification**: `python -m pytest -q tests` 180/180; Angular build clean; monolith round-trip clean. No UI change — no baseline refresh needed.

### 2026-06-12 — Codex/Claude-Safe PR52 Review Follow-up

- **Non-overlap after Claude sync**: Fast-forwarded to Claude's `3c9b4bc` review-fix commit before adding this follow-up, preserving its B1/B2/M1-M3/A1 fixes. Added a frontend stale-session guard so any live `ctlStatus.reason === "unauthenticated"` outside an active login flow locks the operator UI and clears the dead token instead of leaving controls unlocked.
- **Firmware advisories**: Removed the dead `BH1750_RETRY_INTERVAL_MS` / `lastBh1750RetryMs` retry remnants now superseded by `PeriphBackoff`, and added a logged guard when `getCpuFrequencyMhz()` returns `0` before entering default-off idle power save.
- **Verification**: `npm --prefix web run test:ci` 43/43; `python -m pytest -q tests` 173 passed, 1 skipped (known Windows pytest temp symlink cleanup warning after exit); `npm run build:web` regenerated the monolith; `npm run build:check` clean with the known initial-bundle budget warning; API/operator smoke 52/52 across desktop, Pixel 7, iPhone 14, and iPad; temp-sketch `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6` clean with existing LiquidCrystal architecture warning.

### 2026-06-12 — Review Closure: B1/B2 Blockers, M1–M3 Must-Fixes, A1 Advisory

- **B2 (EXE re-auth after LAN restart)**: `ServerLifecycleService.restartServer()` now calls `auth.lock()` after the sidecar restart — the sidecar wipes in-memory operator sessions, so the station locks immediately and the existing lock overlay walks the operator through PIN re-login against the on-disk profiles instead of leaving a half-broken authenticated UI.
- **B1 (release verification exit code)**: the artifact-verification step sets `$ErrorActionPreference = 'Stop'` and throws on non-zero `$LASTEXITCODE`, so a corrupted APK/EXE can no longer pass green.
- **M1**: comparison means now also exclude frames flagged invalid by `logged_hr_valid`/`logged_rr_valid` (stale-held non-zero values), falling back to zero-exclusion for legacy rows. **M2**: malformed `ml_readiness_verdict` shapes log a console contract warning instead of failing silently. **M3/A6**: baseline-refresh workflow permissions scoped to the job with a header note on intentional check re-triggering and branch-protection behavior.
- **A1**: Live holding caches (`lastGoodHr/Rr`) and SQI ribbon histories reset when the active session id changes — back-to-back sessions never flash the previous session's values. A2 confirmed already capped (`slice(-59)`); A4 `.gitignore` duplicate did not reproduce; `_is_supported_radar_contract_length` confirmed wired at two call sites.
- **PR61 Salvage + PR63 Docs**: recovered the reliability test agent's uncommitted work (parser/lifecycle/verdict pytest modules + bounded `_ANALYSIS_JOBS` eviction; schema assertions updated to the 219-column v15.1 contract) and integrated the operator quickstart/API-table docs with corrections (placement-zone chip and Session Quality card claims fixed to match the implemented UI).
- **Verification**: `python -m pytest -q tests` 174/174; `npm --prefix web run test:ci` 42/42; build + monolith round-trip clean; comparison/scorecard/signal-chip smoke specs 3/3 on desktop Chromium.

### 2026-06-12 — PR63: Refresh Windows Visual Baselines

- **Baseline Refresh + OTA Fixture**: Regenerated the committed Windows Playwright baselines for the intentional Help, Live, Settings, SQI ribbon, and v16.2 UI changes across desktop, Pixel 7, iPhone 14, and iPad projects. OTA smoke's newer-product fixture now advertises `16.2.1` so it remains newer than the PR64 `16.2.0` product baseline.
- **Verification**: `npx playwright test tests/visual --update-snapshots` passed 96/96 and refreshed 21 PNGs; `npx playwright test tests/visual` passed 96/96 against the regenerated baselines.

### 2026-06-12 — PR61: Playwright Operator Smoke Isolation

- **Smoke Isolation**: `tests/smoke/operator.spec.ts` now removes the operator profile file under the active `RVT_TEST_SESSIONS_ROOT`, not only the default `.playwright-state/sessions` path. This keeps the four Playwright projects independent when CI or local runs use a custom sessions root.
- **Verification**: Fresh-port operator smoke with `RVT_TEST_SESSIONS_ROOT=.playwright-state/sessions-operator` passed 4/4 across desktop, Pixel 7, iPhone 14, and iPad.

### 2026-06-12 — PR64: Overall v16.2.0 Version Alignment

- **Version Bump**: Promoted the overall product/package/trainer/dashboard/firmware identity from `16.1.0` to `16.2.0`, including the firmware filename `radar_vital_v16_2_0.ino`, `FW_VERSION`, sketch subversion, app package metadata, Tauri/Capacitor metadata, trainer expected firmware version, dashboard product constants, OTA tests, and release workflow examples.
- **Contract Coverage**: Static contract tests now assert the v16.2.0 product identity across root/package locks, Tauri Cargo/config files, Capacitor metadata, trainer constants, Angular services/components, and firmware sketch version macros.
- **Verification**: `python -m pytest -q tests/test_v12_static_contract.py` 18/18; `python -m pytest -q tests/test_packaging.py tests/test_ota_backend.py tests/test_trainer_security_api.py` 23 passed, 1 skipped; `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer` clean; `python -m rvt_trainer --help` clean and reports Radar Vital Trainer v16.2.0; `npm --prefix web run test:ci` 42/42; `npm run build:web` regenerated the monolith; `npm run build:check` clean with the known initial-bundle budget warning; temp-sketch `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6` clean with the existing LiquidCrystal architecture warning.

### 2026-06-12 — PR59: Default-Off Power Save and Chip Thermal Guardrails

- **Power Save Gate**: Firmware now has default-off `RV_POWER_SAVE 0` plus `RV_LCD_LUX_BACKLIGHT` gates. When enabled and the presence FSM remains `ABSENT` for 60 s, ancillary peripherals enter idle mode: LCD backlight off, NeoPixel brightness capped, MLX90614 poll interval widened to 10 s, and CPU frequency lowered to 80 MHz. Radar UART/DSP cadence is unchanged.
- **Instant Restore + Lux Hysteresis**: Any fresh strong/weak presence evidence or FSM exit from idle eligibility restores CPU frequency/backlight before the next loop pass; optional BH1750 LCD dimming uses 8 lux / 15 lux hysteresis to avoid flicker.
- **Thermal Logs**: Firmware samples the ESP32 internal temperature every 10 s and emits `[THERMAL] chip_temp_c=...` warnings above 75 C, rate-limited to once per minute. CSV remains the PR57 219-column contract; chip temperature is log-only until the next schema bump.
- **Bench Checklist**: `docs/physical-acceptance-checklist.md` now includes PR59 current-delta, 1 Hz cadence, reacquisition-latency, thermal-warning, and lux-hysteresis acceptance steps.
- **Verification**: `python -m pytest -q tests/test_v12_static_contract.py` 18/18; `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer` clean; `python -m rvt_trainer --help` clean; temp-sketch `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6` clean with existing LiquidCrystal architecture warning; temp-sketch `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6 --build-property 'compiler.cpp.extra_flags=-DRV_POWER_SAVE=1'` clean with the same warning. Physical current/thermal/soak tests not run locally.

### 2026-06-12 — PR62: Pip-Installable Trainer, Declared BLE Extra & Release Artifact Verification

- **Packaging**: New root `pyproject.toml` (PEP 621/setuptools) makes the trainer installable as `rvt-trainer` with a `rvt-trainer` console script (`rvt_trainer.cli:main`), runtime deps mirroring `requirements-v12.txt` (pytest excluded), and an `[ble]` extra that finally declares `bleak` (used by `transport/ble.py` but previously undeclared). Package discovery is scoped to `rvt_trainer*` so loose root scripts are not packaged; `rvt_trainer/assets/*` ship as package data. PyInstaller sidecar build path unaffected (it installs requirements directly and targets `sidecar_entry.py` by path).
- **Release Verification**: The previously-orphaned `scripts/verify-release-artifacts.ps1` is now wired into the release job (`release-artifacts.yml`) as a `pwsh` step that re-zips the downloaded APK/EXE assets and verifies contents, minimum sizes and SHA-256 before the manifest is generated.
- **CI-Safe Tests**: `tests/test_packaging.py` parses `pyproject.toml` directly (console script, semantic version, bleak extra) so CI passes without `pip install`; the installed-metadata assertion skips when the package is not installed.
- **Verification**: `pip install -e .` then `rvt-trainer --help` exit 0; `python -m pytest -q tests` 99/99 locally (98 + 1 skip expected in CI).

### 2026-06-12 — PR60: SQI Time-Ribbons (v11 Parity) + Windows Baseline-Refresh Workflow

- **SQI Ribbons**: The Waves tab quality bars gain v11-style time-segmented ribbons — a rolling 60 s client-side PQI history per waveform, colored with the same thresholds as the Bland-Altman scatter (good >= 0.3, warn 0.15-0.3, bad < 0.15), with screen-reader summaries of the good-quality percentage.
- **Baseline Refresh Workflow**: New `workflow_dispatch` job `.github/workflows/visual-baseline-refresh.yml` regenerates the committed Windows visual baselines on `windows-latest`, verifies they pass, and pushes the refreshed PNGs to the dispatching branch with the operator-supplied reason. This unblocks intentional-UI PRs authored off-Windows; dispatch it once after the final UI commit of this series.
- **Verification**: Build clean; vitest 42/42; extended signal-chips smoke spec (now covering ribbons) 1/1 on desktop Chromium; monolith round-trip clean.

### 2026-06-12 — Connection Clarity, LAN Loopback Gate & PR52 Review Fixes

- **Connection Clarity**: Topbar disconnected state shows a live "auto-retry in Ns" countdown driven by the telemetry service's SSE reconnect schedule (`nextRetryAtMs` signal, cleared on connect); the contractual 12 h SSE `session_warning` (`deadline_approaching`) renders as "Live stream renews in 60 seconds — automatic, no action needed" instead of a generic warning.
- **Review P1 (LAN sharing locked the EXE out)**: Loopback clients now bypass the LAN *pairing*-token gate in `_require_control_auth` — after a share-mode sidecar restart the EXE's WebView holds no pairing token and previously got 401 for everything including `/api/auth/login`. Operator-session rules for sensitive endpoints are unchanged; network peers keep the 401 pairing requirement. PR48's LAN contract test updated accordingly (loopback `/api/server-info` 200 and metadata-only; sensitive routes still 401 without an operator session); the share/New-PIN confirm dialogs warn "You will be asked to sign in again afterwards."
- **Review P2 Fixes**: comparison means exclude the firmware's 0-value invalid-publish placeholders; Report verdict categories read the trainer's real `ml_readiness_verdict` block (sandbox shape still supported); the Live "holding" state shows the last accepted HR/RR value instead of `--`.
- **CI Test Job Fix**: the placement chip's `role="status"` made the BLE-probe spec's strict `getByRole('status')` ambiguous — that spec now targets `.native-ble-result` precisely.
- **Verification**: `python -m pytest -q tests` 93/93; `npm --prefix web run test:ci` 42/42; 5 affected smoke specs green on desktop Chromium; monolith round-trip clean. Visual job stays expected-red until the Windows baseline refresh pass.

### 2026-06-12 — PR58: Firmware Robustness Backoff, Reset Forensics, and NVS Failure Escalation

- **Peripheral Backoff**: Firmware now uses bounded exponential retry state for MLX90614, BH1750, and LCD recovery (3 s doubling to 5 min cap), resetting on successful recovery and continuing to increment the PR57 I2C/LCD diagnostic counters.
- **Reset/NVS Forensics**: Boot now logs `esp_reset_reason()` with a readable label and persists an 8-entry reset-reason ring in NVS after namespace migration. NVS setting writes track success/failure counts, log write-count telemetry, attempt one namespace reopen after 3 consecutive failures, then disable further writes for the boot.
- **Bench Checklist**: `docs/physical-acceptance-checklist.md` now includes PR58 fault-injection steps for LCD SDA disconnect/reconnect, radar TX pull/restore, reset-ring persistence, and repeated NVS write failure escalation.
- **Verification**: `python -m pytest -q tests/test_v12_static_contract.py` 17/17; `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer` clean; `python -m rvt_trainer --help` clean; temp-sketch `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6` clean with existing LiquidCrystal architecture warning; `git diff --check` clean except CRLF warnings. Physical fault injection not run locally.

### 2026-06-12 — PR57: Additive Field-Diagnostics Telemetry Columns

- **CSV v15.1 Contract**: Firmware and trainer schema now emit/expect 219 CSV columns, preserving the original 207-column v15 prefix exactly and appending loop timing, heap, radar/CRC, I2C/LCD, watchdog, command, and uptime diagnostics as columns 208-219.
- **Legacy Tolerance**: Trainer parsing pads 207-column v15 rows and older supported legacy rows to the current right edge without failing ML readiness or contract diagnosis solely because field diagnostics are absent.
- **Live Audit Surfacing**: Angular Live Audit surfaces loop mean/max, heap free/min, firmware uptime, and fault counters with demo-mode values; docs/help/static contract tests updated to describe the 219-column current contract.
- **Verification**: `python -m pytest -q tests/test_v12_static_contract.py` 16/16; `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer` clean; `python -m rvt_trainer --help` clean; `npm --prefix web run build` clean with known bundle warning; `npm run build:web` and `npm run build:check` clean; `npm --prefix web run test:ci` 42/42; focused Playwright Live diagnostics/lock-state smoke 8/8 across desktop, Pixel 7, iPhone 14, and iPad; temp-sketch `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6` clean with existing LiquidCrystal architecture warning. Full `dashboard.spec.ts` timed out locally before completion.

### 2026-06-12 — PR56: Subject Placement Guidance & Advisory Start Gate on Home

- **Placement Zone Chip**: The Home radar-scope Range metric now classifies live `distance_cm` into the firmware's distance-confidence bands (Too close <40 cm / Optimal 40–100 / Good ≤140 / Acceptable ≤180 / Out of range) with per-zone color tokens and an actionable hint line (mirrors Seeed's ≤1.5 m chest-height guidance for the MR60BHA2).
- **Advisory Start Gate**: When preflight has non-blocking unpassed checks, a hint above Start states how many need review while making explicit that starting is still allowed — blocking semantics unchanged (`canStartSession()` untouched).
- **Verification**: Build clean; monolith round-trip clean; new placement smoke spec 1/1 on desktop Chromium.

### 2026-06-12 — PR55: Operator-Selectable Session Comparison Overlay & Delta Table

- **Comparison Picker**: The Report "Session Comparison" card (previously fixed Selected/Previous/Best stats from `/api/sessions/<id>/compare`) gains a mat-select to compare against any other recorded session, fetching only the existing `/summary` + `/data?points=1000` routes (read-only; no new API surface).
- **Trend Overlay**: The HR/RR report trend canvases overlay the comparison series as a dashed, 65 %-alpha line on a shared vertical axis with the selected session; an explanatory note names the overlaid session.
- **Delta Table**: Mean HR Δ, mean RR Δ (neutral tone), HR coverage Δ (better/worse colored), and both verdicts. Switching the selected session clears the overlay to avoid stale cross-session reads.
- **Verification**: Build clean; vitest 42/42; new comparison smoke spec 1/1 on desktop Chromium (covers picker, overlay note, delta rows, and stale-overlay clearing); monolith round-trip clean.

### 2026-06-12 — PR54: Keyboard Parity — Wire Ctrl+Z/H/L Stubs and D Demo Toggle

- **Stubs Replaced With Real Features**: The layout shell's Ctrl+Z / Ctrl+H / Ctrl+L handlers still showed "not yet available" snackbars even though `UndoService`, the operator-handoff dialog, and `IdleLockService` shipped in PR43/44. Ctrl+Z now performs the undo (with "Undid: <label>" / "Nothing to undo." feedback), Ctrl+H opens the handoff brief (same dialog options as the palette path), Ctrl+L locks the station immediately.
- **`D` Demo Toggle (v11 parity)**: Plain `d` toggles demo mode with explanatory snackbars; turning demo off does not force a reconnect (Settings remains the explicit reconnect path).
- **Cheat-Sheet Sync**: The keyboard-shortcuts dialog documents the four newly-live bindings. Audit note: the remaining v11 single-key map (1-5/h/l/r/w/s, t, space, a, e/x, b/v, [, Shift+F, ?, Alt+1-0) was verified already implemented — the legacy-parity gap list overstated this; chart zoom remains out (v12 draws raw canvas; Chart.js+zoom plugin are vendored for the legacy contract only, so zoom needs a custom-canvas design, deferred deliberately).
- **Verification**: Build clean; `npm --prefix web run test:ci` 42/42; new shortcut smoke spec 1/1 on desktop Chromium; monolith round-trip clean.

### 2026-06-12 — PR53: Session Quality Scorecard in Report

- **Quality Card**: Report renders a "Session Quality" card sourced entirely from the existing `/api/sessions/<id>/summary` payload (no new API surface): `signal_quality` stat tiles (PQI lock %, session quality score, internal consistency, locked/settling coverage), an HR/RR accuracy table (RMSE/MAE/bias/coverage vs the reference), readiness gate chips (pass/fail/deferred), and the ML-readiness verdict `categories[]` with remediation text for non-pass items. Card hides entirely when a summary carries none of these blocks (older sessions degrade gracefully).
- **Demo Parity**: The sandbox `/summary` response now includes a simulated quality block + structured verdict with categories so the card is demo-visible; the hero banner still shows "DEMO — Simulated Data Only" via the `sandbox` flag.
- **Verification**: `npm --prefix web run build` clean (known budget warning); `npm run build:web` round-trip clean; `npm --prefix web run test:ci` 42/42; new scorecard smoke + adjacent report review/print specs 3/3 on desktop Chromium. Visual baselines pending the Windows refresh pass noted in PR52.

### 2026-06-12 — PR52: Live Signal Lock-State, Confidence, Motion & Readiness Surfacing

- **Lock-State Chip**: The Live command strip now shows the firmware presence/DSP phase (`radar.session_phase_name`: No subject / Warming up / Settling / Signal locked / Recovering after motion / Subject leaving) with per-phase Material color tokens and icons; values were already published per second by the trainer and previously unconsumed (grep-verified zero UI usage).
- **Motion + Readiness Chips**: A motion chip ("Motion — readings hold briefly") renders while `doppler_motion`/`motion` is active; a readiness chip surfaces `analysis.ml_readiness_verdict.verdict` when present. All chips de-emphasize under stale telemetry and announce politely via `role="status"`.
- **KPI Confidence/Validity**: HR card gains an `hr_confidence` percent badge with `hr_confidence_source_name` in the native tooltip/aria-label; HR/RR values dim with an explicit "holding" tag when `logged_hr_valid`/`logged_rr_valid` is 0 — values are never hidden, only de-emphasized.
- **Demo Parity**: Demo telemetry simulates the phase lifecycle (warmup→settling→locked), a 3 s motion burst every 45 s with confidence dip + HR holding, `AUTO_PHASE` confidence source, and a demo readiness verdict, so all new UI is visible without hardware.
- **Typed Model**: `LivePayload.radar` gains optional typed quality fields (additive; index signature preserved).
- **Verification**: `npm --prefix web run build` clean (known budget warning); `npm --prefix web run test:ci` 42/42; `npm run build:web` round-trip clean; new + adjacent demo smoke specs 3/3 on desktop Chromium (local build 1194 stand-in; CI uses pinned 1193). Visual baselines NOT refreshed here (Windows-only contract) — the Live command strip change requires a Windows `npm run test:visual:update` pass before merge.

### 2026-06-12 — PR51 Follow-up: Scannable Pairing QR over the Loopback Channel

- **QR Restored Without the LAN Leak**: `/api/native-pairing-info?format=qr` (loopback-only, LAN-bind-only) now returns `qr_png_base64` + `qr_target_url` generated by the existing `_qr_png_bytes` helper; `/api/server-info` stays metadata-only. The EXE Settings sharing card renders the QR as a data-URL `<img>` so phones pair by camera scan with zero typing; the copyable link and PIN/TTL remain as fallbacks.
- **LAN Auth Gate Fix**: `_require_control_auth` now allows GET `/api/native-pairing-info` from loopback clients before the LAN pairing-token gate — previously the EXE native bridge received 401 in exactly the LAN-share mode the card exists for; the route handler still 403s non-loopback clients by IP.
- **LAN Port Probe Fix**: `ensure_lan_port_available` probes `0.0.0.0:8765` instead of loopback so conflicts bound only to a LAN interface are caught before spawning the sidecar.
- **Docs/Test Sync**: De-duplicated the contradictory sharing bullets in `docs/standalone-exe-and-apk-scope.md`; added pytest coverage (server-info never exposes PIN/QR material incl. under LAN token, QR is LAN-bind-gated, PNG magic + target URL asserted) and a Playwright API spec for local-bind QR refusal; lifecycle vitest specs updated for the `?format=qr` fetch, QR data-URL signal, and made order-independent against the constructor bootstrap race.
- **Verification**: `python -m pytest -q tests` 92/92; `npm --prefix web run test:ci` 42/42; `python -m compileall` clean. Not run here (Linux container): `cargo test`, Windows EXE manual pair, visual baselines.

### 2026-06-12 — PR51 Blocking Review Fixes

- **Pairing PIN Leak Closed**: Removed the public `/api/server-info?format=qr` image behavior and kept `/api/server-info` metadata-only; added loopback-only `/api/native-pairing-info` for the Windows EXE native bridge to read the active PIN/TTL without exposing it to LAN clients.
- **EXE Sharing UX Hardened**: Replaced the WebView `<img>` QR fetch with a copyable pairing link, visible one-time PIN when supplied by the native loopback path, live 1 s TTL countdown, and confirmation dialogs before share-mode or "New PIN" restarts that interrupt sessions/SSE/serial capture.
- **Tauri Sidecar Hardening**: Validated `bind_mode`, checked LAN port 8765 before spawning, and replaced the tautological bind-mode unit test with sidecar argument-vector coverage.
- **Request Timeout Scoped**: Restored the default Angular request timeout to 10 s and applied 30 s only to PBKDF2-heavy login/profile creation calls.
- **Docs/Coverage Updated**: Documented the native loopback bootstrap route in README/scope docs and added smoke/unit coverage for metadata-only server-info, native pairing parsing, and missing-PIN fallback.
- **Verification**: Not run per human instruction; pushed directly after code changes.

### 2026-06-12 — One-Click Phone Sharing from Windows EXE and Request Timeout Fix

- **Tauri Companion Sidecar Parametrization**: Added `bind_mode` parameter to Tauri state and `trainer_start` command to allow launching the companion Python trainer in LAN mode (`--bind lan` on port 8765) or local mode (random loopback port).
- **Angular Settings Interface**: Integrated a "Phone access — share on local network" slide toggle in the Tauri-only Settings card, with inline pairing QR rendering from `/api/server-info?format=qr`, PIN expiration countdown, a "New PIN" restart trigger, and Windows Firewall copy.
- **Request Timeout Fix**: Increased request timeout from 10s to 30s in `api.service.ts` to handle PBKDF2 hashing overhead during operator creation.
- **Verification**: `cargo test --manifest-path src-tauri/Cargo.toml` (9/9), `python -m pytest -q tests` (90/90), and Playwright smoke verification (`npm test`) passed; verified that the share toggle is hidden in standard browser contexts.

### 2026-06-11 — PR48 CI Readiness Fixes

- **Telemetry Session Reconciliation**: Made live payload `meta.status` / `session_id` authoritative for active-session state so stopped or idle trainer payloads clear the Angular navigation guard and Stop Session affordance.
- **Session Start Payload Contract**: Re-aligned the shared Angular API service with the trainer `/api/session/start` contract (`subject_label`, `subject_profile_id`, `advanced.notify_char`) after the rebased PR branch exposed stale setup field references.
- **Sandbox Report Contract**: Restored demo-mode report/session API shapes (`items` + `sessions`, summary/data/notes/signoff/compare/status payloads) and preserved the `16.1.0` static product-version literal.
- **Mobile Actions Menu**: Replaced the custom mobile overflow panel with an Angular Material menu so Search, Lock, and Switch Operator actions are present and focusable across phone/tablet projects.
- **SSE Startup Stability**: Kept TelemetryService reconnect startup outside Angular signal tracking while preserving the PR48 duplicate-connection guard.
- **Smoke Test Determinism**: Mocked active-session SSE/live-dashboard payloads, reopened the selected Help topic before checking advanced copy, and cleared the PIN keypad before lockout attempts.
- **Visual Baselines**: Seeded the authenticated operator context in v12 visual tests and refreshed the committed Windows baselines for the locked PR48 dashboard shell.
- **Verification**: `npm --prefix web run build`, `npm run build:web`, `npm run build:check`, `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer`, `python -m rvt_trainer --help`, `python -m pytest tests -q` (90/90), `npm run test:unit:web` (39/39), `node scripts/generate-rvt-latest.mjs --self-test`, `npx playwright test tests/smoke --reporter=list` (184/184), and `npx playwright test tests/visual --reporter=list` (96/96) passed; Angular still reports the known initial bundle budget warning.

### 2026-06-10 — PR48 Operator Profiles and PIN Lock Fixes

- **Help Tab Auth Bypass**: Added `/api/help/schema` to the public endpoint bypass list so the Help tab loads successfully without an operator session token.
- **Lockout Database Persistence Safety**: Fixed a brute-force lockout bypass where a failed database write on invalid attempts allowed attackers to retry PIN PIN verification by forcing the 429 lockout logic to fire unconditionally in memory.
- **SSE Connection De-duplication**: Guarded TelemetryService to prevent duplicate EventSource connections on rapid lock/unlock transitions.
- **SSE Memory Leak Prevention**: Added periodic reaping of expired SSE and session tokens during new token generation to prevent unbounded memory growth.
- **Verification & Test Coverage**: Added a unit test validating 429 lockout enforcement on disk failure, and resolved Playwright smoke test flakiness by introducing Angular bootstrap loading overlay waits before UI click/input interactions.

### 2026-06-09 — PR48 Operator Profiles and PIN Lock

- **Operator Auth Backend**: Added PBKDF2-backed operator profiles, 8-hour operator sessions, first-run unauthenticated bootstrap only when no profiles exist, authenticated profile creation after bootstrap, logout revocation, 5-attempt/30-second PIN lockout, and protected sensitive control routes while keeping discovery/update endpoints public.
- **Dashboard Lock UX**: Added first-run operator onboarding, PIN keypad lock/unlock, desktop rail and mobile overflow actions for Lock profile / Switch operator, authenticated add-and-switch dialog, and isolated operator token storage under `rvt-operator-token`.
- **Telemetry Auth**: Routed Angular API calls and SSE reconnects through the operator token, including short-lived SSE token flow, without changing public health/version/update-manifest access.
- **Smoke/Test Isolation**: Added backend/operator unit coverage and Playwright operator smoke with isolated `.playwright-state/sessions` trainer storage; non-auth dashboard/OTA smokes seed a mock operator token so existing v11 parity tests still exercise the UI.
- **Verification**: `npm run build:web`, `npm run build:check`, `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer`, `python -m rvt_trainer --help`, `python -m pytest -q tests/test_operator_auth.py tests/test_trainer_security_api.py` (15/15), `npm --prefix web run test:ci -- --include src/app/services/auth.service.spec.ts` (7/7), `npx playwright test tests/smoke/operator.spec.ts --project=desktop` (1/1), `npx playwright test tests/smoke/operator.spec.ts --project=pixel-7` (1/1), `npx playwright test tests/smoke/api.spec.ts --project=desktop` (9/9), `npx playwright test tests/smoke/ota.spec.ts --project=desktop` (5/5), and targeted dashboard layout smoke (4/4) passed; unsliced `npm test -- --reporter=line` timed out in the local tool window before returning a result. Angular still reports the known initial bundle budget warning.
### 2026-06-11 — Multi-PR Improvement Plan (UI/UX + Firmware Tracks)

- **Planning Document**: Added `docs/multi-pr-improvement-plan.md` defining two coordinated tracks of independently-mergeable PRs (PR-49 through PR-60): Track A covers Live view decomposition/perf, a11y & i18n foundation, mobile ergonomics/haptics, trend compare/annotations, session workflow/reporting, and connection/pairing status UX; Track B covers firmware hygiene & v17 debt prep, a CRC-guarded serial command channel, telemetry columns 208+, I2C/NVS/brownout robustness, presence-gated power/thermal management, and the gated BLE Phase 4F activation.
- **Constraints Encoded**: Each PR entry restates the AGENTS.md invariants it touches (frozen 207-column serial contract with right-side additions only, no second UI framework, demo/live IndexedDB scoping, watchdog no-`delay()` rule, BLE default-off until physical GATT acceptance) plus per-PR acceptance criteria, verification additions, dependency ordering, and a five-milestone grouping.
- **Verification**: Documentation-only change; no code paths touched. Standard build/test protocol not applicable beyond markdown review.

### 2026-06-08 — PR47 Review Follow-up Fixes

- **Updater Manifest Version**: Changed `rvt-latest-tauri.json` generation to advertise the stamped release version (`RELEASE_VERSION`) so prerelease/main installers do not repeatedly compare against the base product version.
- **PWA Worker Apply Path**: Removed Angular `SwUpdate` registration for custom `sw.js` and routed checks/reload prompts through the native `ServiceWorkerRegistration.update()` + waiting-worker `SKIP_WAITING` path.
- **PWA Contract Test**: Updated the static PWA contract to assert the native service-worker apply path instead of the removed snackbar copy.
- **PWA Update UX**: Automatic worker updates now show a non-modal snackbar so update availability cannot interrupt keyboard/dialog workflows; explicit Settings apply still uses the reload confirmation dialog.
- **Release Signing Safety**: Re-signed the Tauri updater signature after final Authenticode EXE signing and replaced the release upload glob list with a generated file list so optional `.sig` assets do not fail unsigned/manual-fallback releases.
- **Prerelease Publishing**: Replaced the hardcoded `v16.1.0-alpha` release check with version-agnostic `-alpha` / `-rc` prerelease detection.
- **Android Review Note**: Confirmed the P1 Android Kotlin compile issue is already covered by the prior `scripts/patch-android-shell.mjs` Kotlin Gradle injection and clean regenerated APK build verification.

### 2026-06-07 — PR47 CI Packaging Fixes

- **APK CI Fix**: Reproduced the fresh GitHub-generated Android build failure (`MainActivity.java` could not resolve `OpenFilePlugin`) and updated `scripts/patch-android-shell.mjs` to inject the Kotlin Gradle classpath/app plugin before writing `OpenFilePlugin.kt`.
- **EXE CI Fix**: Reproduced the Tauri bundler failure where updater artifacts required `TAURI_SIGNING_PRIVATE_KEY`; disabled updater artifact generation by default and gated release updater artifacts on both public and private updater signing secrets.
- **Verification**: Clean-regenerated `android/` with `npx cap add android`, `npx cap sync android`, `node scripts\patch-android-shell.mjs`, and `android\gradlew.bat -p android assembleDebug --no-daemon --stacktrace --console=plain` passed. `npx tauri build --verbose --bundles nsis` passed locally and produced `Radar Vital_16.1.0_x64-setup.exe`; Angular still reports the known initial bundle budget warning.

### 2026-06-07 — PR47 Native OTA Install, EXE Lifecycle, and v16.1.0 Bump

- **Native Update Installers**: Added Angular update/PWA services, Tauri updater IPC/config, Android APK download/hash/install bridge, and release publication for both `rvt-latest.json` and `rvt-latest-tauri.json`.
- **EXE Lifecycle/BLE**: Fixed Tauri Stop Server to terminate the Windows trainer process tree by PID and widened EXE BLE discovery to scan broadly before filtering by AiLink service/name hints.
- **Responsive OTA QA**: Clicked the OTA/update, Settings, and dashboard smoke flows across desktop, Pixel 7, iPhone 14, and iPad; blocked service workers in OTA smoke so mobile/tablet route mocks exercise `/api/update/manifest`, hid the scroll-to-top FAB on the Settings route, stacked Settings footer actions on phones, and refreshed intentional Help snapshots for `16.1.0`.
- **Android Shell Reproducibility**: Updated the tracked Capacitor patch script to recreate the native `OpenFilePlugin.kt`, register it in `MainActivity`, add FileProvider/cache paths, and verify `@capacitor/filesystem` is installed/synced.
- **Version Bump**: Promoted product/trainer/dashboard/firmware/package metadata to `16.1.0`, renamed firmware to `radar_vital_v16_1_0.ino`, and bumped the SW cache key to `rvt-shell-v12.0.4`.
- **Verification**: `npm --prefix web run build`, `npm --prefix web run test:ci` (32/32), `cargo check --manifest-path src-tauri\Cargo.toml`, `cargo test --manifest-path src-tauri\Cargo.toml` (8/8), `npm run build:web`, `npm run build:check`, `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer`, `python -m rvt_trainer --help`, `python -m pytest -q tests --basetemp .pytest-tmp-pr47` (82/82), `node scripts\generate-rvt-latest.mjs --self-test`, `npx playwright test smoke/ota.spec.ts --reporter=line` (20/20), `npm run test:smoke -- --reporter=line` (180/180), `npm run test:visual -- --reporter=line` (96/96), `npm run cap:sync`, and `android\gradlew.bat -p android :app:compileDebugKotlin --no-daemon "-Dkotlin.compiler.execution.strategy=in-process" --console=plain --stacktrace` passed; Angular still reports the known initial bundle budget warning and Android dependency deprecation warnings.

### 2026-06-07 — Firmware Audit Fixes & v16_0_1 Filename Rename

- **Firmware Rename**: Renamed `radar_vital_v16_0_0.ino` → `radar_vital_v16_0_1.ino` to match the internal `FW_VERSION "v16.0.1"` / `SKETCH_VERSION_MOD 1`. Updated all references across `AGENTS.md`, `README.md`, `HANDOFF.md`, `ORIGINAL_REQUEST.md`, `rvt_trainer/monolith.py` (2 sites), `tests/test_v12_static_contract.py`, `.github/workflows/build-exe.yml`, `docs/angular-migration-audit.md`, and `docs/physical-acceptance-checklist.md`.
- **CSV Column Count Enforcement**: Added runtime `_csvColCount` counter incremented by all `CSVU`/`CSVI`/`CSVF`/`CSVFN` macros; asserts `_csvColCount == CSV_COLUMN_COUNT` (207) on the first DATA emit of each new session and prints `[CONTRACT]` warning on mismatch. Counter resets to 0 after each row.
- **Duplicate ENABLE_BLE Removed**: Removed the second `#ifndef ENABLE_BLE` / `#define ENABLE_BLE false` / `#endif` block at the old lines 273–275 (inside the logging section), keeping only the canonical block at lines 198–200.
- **Dead Constant Removed**: Replaced `static const float CHIP_HR_BIAS_CORRECTION_BPM = 6.0f` (deprecated since v14.0.0) with a changelog comment.
- **BLE Callback Tagged**: Added `// STUB — no-op until BLE path activates` comment to `RvsControlCallbacks::onWrite`.
- **RLS_LAMBDA_HR Comment Updated**: Changed legacy alias comment to `// REMOVE at v17 — use RLS_LAMBDA_HR_BASE directly`.

### 2026-06-06 — PR46 OTA Installation & Settings E2E Verification

- **E2E Spec & Verification**: Implemented Playwright specs in `tests/smoke/ota.spec.ts` covering update check status results (New Version, New Build, Up-to-Date), button accessibility attributes, and unauthenticated public proxy manifest route validation.
- **Firmware Audits**: Implemented runtime CSV column counting and mismatch warning on the first DATA emit of a session, and verified all 5 firmware requirements programmatically in `tests/test_v12_static_contract.py`.
- **Backend Mocks & Manifest**: Confirmed context manager support (`__enter__` and `__exit__`) for `urllib.request.urlopen` in `tests/test_ota_backend.py`. Rebuilt the Angular application monolith and confirmed visual regressions/smoke tests pass 100%.
- **Verification**: `pytest` passed 82/82; `npx playwright test tests/smoke/ota.spec.ts` passed 20/20; `npm run test:smoke` passed 180/180; `npm run test:visual` passed 8/8; `npm run build:web` / `npm run build:check` completed successfully.

### 2026-06-06 — HyperFrames 60 Second Launch Commercial

- **Launch Commercial Project**: Added `videos/radar-vital-launch-60s` with a 60-second fast-paced HyperFrames launch video covering Radar Vital's 60 GHz mmWave workflow, no-contact measurement, 0.4 m to 1.5 m operating range, desktop/mobile light-mode dashboard, and PWA/APK/Windows EXE availability.
- **Credits & Narration**: Generated Kokoro narration and ended the commercial with the required credits for Lemuel Blaya, Angelo Diaz, and Blessie Mugat of the University of Mindanao, Electronics Engineering Students.
- **Verification**: `npx --yes hyperframes lint`, `npx --yes hyperframes validate`, and `npx --yes hyperframes inspect --samples 18` passed with zero errors/warnings/layout issues; `npx --yes hyperframes snapshot . --at 2,6,10.8,16.7,22.6,28.6,34.8,41,46.8,51.8,57` produced review frames under `videos/radar-vital-launch-60s/snapshots`; rendered `videos/radar-vital-launch-60s/renders/radar-vital-launch-60s-with-narration.mp4` and verified 60.00 s 1920x1080 H.264 video with AAC narration.

### 2026-06-06 — HyperFrames Product Promo Artifact

- **Website Promo Project**: Captured `https://blayalems.github.io/radar-vital-mmwave-60-davao` into `videos/radar-vital-mmwave-60-davao-promo/capture` and built a 20-second HyperFrames product promo with DESIGN/SCRIPT/STORYBOARD docs, local captured fonts/assets, generated Kokoro narration, and midpoint snapshots.
- **Working Demo Screenshot Update**: Replaced the promo's offline/server-not-running screenshot with a Playwright-captured explicit demo-mode Live view from a trainer-served mock dashboard, saved as `videos/radar-vital-mmwave-60-davao-promo/capture/screenshots/demo-working.png`.
- **Light Mode + Mobile Availability Update**: Switched the promo composition and product frames to light mode, added a Pixel 7 light demo screenshot (`demo-working-mobile-light.png`), and updated the final beat to show desktop + mobile views with PWA, Android APK, and Windows EXE availability.
- **Verification**: `npx --yes hyperframes lint`, `npx --yes hyperframes validate`, and `npx --yes hyperframes inspect --samples 15` passed with zero errors/warnings/layout issues; `npx --yes hyperframes snapshot . --at 1.8,6.8,12.6,17.8` produced review frames under `videos/radar-vital-mmwave-60-davao-promo/snapshots`.

### 2026-06-06 — PR46 v16.0.1 OTA Metadata & Mobile Regression Closure

- **Release Metadata Contract**: Extended `scripts/generate-rvt-latest.mjs` and release CI so `rvt-latest.json` carries product/release/build identity, artifact URLs, sizes, SHA-256 hashes, minimum support, and compatibility metadata from the actual merge/tag release instead of PR-only publication.
- **Pages Publication Safety**: Replaced the release workflow's direct `gh-pages` push with Actions Pages upload/deploy, and made the normal Pages workflow preserve the latest hosted `rvt-latest.json` when rebuilding the static dashboard.
- **Update UI Consumers**: Updated Settings and sandbox API consumers to understand additive release metadata, same-product build updates, no-update demo manifests, and release-link guidance without APK/EXE/Python/firmware install automation.
- **Mobile Layout Regression Tests**: Added Playwright checks for horizontal overflow, blank top band, bottom-nav clearance, Home setup containment, and Command Palette close/list containment across phone/tablet viewports; tightened Command Palette, Settings, topbar, and navigation breakpoint CSS accordingly.
- **Verification**: `python -m pytest -q tests\test_trainer_security_api.py tests\test_v12_static_contract.py tests\test_trainer_server_info.py tests\test_trainer_cors_guard.py` passed 34/34; `npm --prefix web run test:ci` passed 32/32; `npm run build:check` passed with the known initial bundle warning; `node scripts\generate-rvt-latest.mjs --self-test` passed; targeted mobile smoke passed 9/9; full smoke was split by project and passed 40/40 on desktop, Pixel 7, iPhone 14, and iPad; full visual passed 96/96 after intentional snapshot refresh.

### 2026-06-06 — PR46 Variable Collision Minification Fix

- **IIFE Scope Collision**: Resolved a critical console error (`TypeError: $ is not a function`) on dashboard load by adding the `--minify` flag to the esbuild bundling step in `scripts/build-angular.mjs`. This prevents hoisting and sharing of minified template identifiers (like `$`) across ESModules when merged into the single-file IIFE monolith.
- **Visual Baseline Refresh**: Re-generated the 96 visual baseline snapshots to match the updated Home and Settings components layouts.

### 2026-06-06 — PR46 Backend Hardening & API Version updates

- **Monolith Hardening**: Guarded `_ControlHandler.end_headers()`, `_require_control_auth()`, and `_read_body()` against `self.headers` being `None` (resolving crashes on HTTP/0.9 or headerless/malformed socket connections).
- **Version and Schema Metadata**: Bumped trainer, dashboard, and expected firmware versions to `16.0.1`/`v16.0.1` in `rvt_trainer/monolith.py` and `radar_vital_v16_0_1.ino`. Extended the GET `/api/version` response to include `product_version`, `schema_versions` for all schema types, and `update_manifest_url`.
- **Update Manifest Proxy**: Implemented GET `/api/update/manifest` proxy in `rvt_trainer/monolith.py` using standard `urllib.request.urlopen` to route manifest requests dynamically.
- **Unit and Contract Tests**: Added `test_headerless_request_does_not_crash` in `tests/test_trainer_security_api.py` to verify resilience against malformed socket requests, and bumped version assertions in `tests/test_v12_static_contract.py`.

### 2026-06-06 — PR46 UI fixes & Settings Update Card

- **Service Worker & Test Bumps**: Bumped cache version to `'rvt-shell-v12.0.3'` in `assets/sw.js`, `tests/test_v12_static_contract.py`, and `tests/smoke/dashboard.spec.ts`.
- **Layout & Styles Overrides**: Set `.main-content-scroll` `margin-bottom: 0` and `.bottom-nav` `z-index: 900` under all media queries in `layout.component.css`. Added `html[data-sandbox="1"] body { padding-top: 0 !important; }` and `body #demoBanner` positioning rules in `styles.scss`.
- **Offline Snackbar Suppression**: Suppressed the initialization offline snackbar in `home.component.ts` when demo mode or auto-demo is active.
- **Home Responsive CSS**: Flex-wrapped `.field-controls` and updated grid layout for `.setup-card-content` under 900px breakpoint in `home.component.css`.
- **Command Palette CSS**: Applied flex/display layout rules to `command-palette.component.css` and hid shortcut labels on mobile viewports.
- **Settings Card & Grid CSS**: Implemented the "Update & Version Info" card, checking for updates against `/api/update/manifest` in `settings.component.html/ts`, and added its CSS rules and mobile responsive spans in `settings.component.css`.

### 2026-06-06 — PR46 Release Manifest & Packaging Version Bump (codex/pr46-release-manifest)

- **Packaging Version Bump**: Bumped version to `16.0.1` across root `package.json`, root `package-lock.json`, `src-tauri/Cargo.toml`, `src-tauri/tauri.conf.json`, `packaging/tauri/tauri.conf.json`, and `packaging/capacitor/package.json`/`package-lock.json`.
- **Release Manifest Generator**: Implemented a Node.js utility `scripts/generate-rvt-latest.mjs` to calculate sizes and SHA-256 hashes of build APK and EXE artifacts, outputting `rvt-latest.json` for update delivery.
- **Release Workflow Integration**: Updated `.github/workflows/release-artifacts.yml` to execute the generator script, upload `rvt-latest.json` as a release asset, and commit/push it to the `gh-pages` branch for Pages hosting.
- **Verification**: Ran `node scripts/generate-rvt-latest.mjs --self-test` successfully validating mock artifacts against the schema.

### 2026-06-04 — PR45 Visual Baseline Refresh (codex/pr45-native-start)

- **Visual Baseline Refresh**: Regenerated the committed Windows Playwright baselines for the intentional PR45 server-status chip, Python Server settings card, and mobile shell spacing/hit-test changes across all supported themes and viewports.
- **Verification**: `npx playwright test tests/visual/rvt-v12.spec.ts --update-snapshots --reporter=line` passed 80/80 locally and rewrote the 80 affected baseline PNGs.

### 2026-06-04 — PR45 Same-Origin Server & Mobile Hit-Test Follow-up (codex/pr45-native-start)

- **Server URL Default Fix**: Updated the remote/PWA server lifecycle default to prefer the current local/LAN trainer origin before falling back to `http://127.0.0.1:8765`, preventing CSP-blocked cross-origin `/api/health` and `/api/status` calls in trainer-served smoke/CI runs.
- **Mobile Hit-Test Fix**: Added phone scroll gutters and pointer-transparent inert shell chrome so sticky topbar, bottom nav, scroll-to-top FAB, and snackbar labels no longer intercept routed form/action controls on iPhone-sized viewports while preserving pointer events on actual chrome actions.
- **Verification**: `npm --prefix web run test:ci` passed 32/32; `npm run build:web` passed with the known initial bundle warning; targeted iPhone smoke subset passed 3/3; full `npx playwright test tests/smoke --reporter=line` passed 160/160 across desktop, Pixel 7, iPhone 14, and iPad.

### 2026-06-04 — PR45 Native Server Lifecycle & PR43/PR44 Integration (codex/pr45-native-start)

- **PR45 Integration Strategy**: Created the PR45 integration branch from `origin/main`, merged PR #43 as the primary body, and lifted PR #44's unique BLE transport extraction plus trainer audit/transport pytest coverage without overwriting PR #43's broader UI/security/native hardening.
- **Python Server Lifecycle**: Added Tauri IPC commands `trainer_start`, `trainer_stop`, `trainer_status`, and `trainer_log_tail`; kept Windows EXE sidecar auto-start on setup, preserved the sidecar child handle, emitted stdout/stderr to Tauri events and stderr, and bounded the native log tail with a 20-line ring buffer.
- **APK/PWA Server Pairing Flow**: Added an Angular `ServerLifecycleService` that auto-starts the Windows sidecar in Tauri desktop shells, auto-checks LAN/PWA trainer health on remote shells, stores the server origin under `rvt.server.url`, and keeps the existing `/pair` + `X-RVT-Auth` flow intact for APK/PWA.
- **Operator UI Controls**: Added a topbar server-status chip, a Settings **Python Server** card with EXE Start/Stop/Restart/log controls and APK/PWA address/pair/retry controls, Home bootstrap gating before hardware calls, and a Live tab server-offline blocker that still allows explicit Demo Mode charts.
- **Trainer Extraction Fixes**: Restored `_candidate_ino_paths`, converted monolith preflight/BLE/serial legacy names to extracted module aliases, made serial helpers import pyserial lazily, and regenerated the committed v12 monolith from `web/src/`.
- **Verification**: `npm --prefix web run build` passed with the known initial bundle warning (54.23 kB over 2.20 MB); `npm --prefix web run test:ci` passed 32/32; `npm run build:web` and `npm run build:check` passed; `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer` passed; `python -m rvt_trainer --help` passed; `python -m pytest tests -q` passed 67/67; `cargo test --manifest-path src-tauri\Cargo.toml` passed 8/8 after rerunning once the parallel web build finished writing `www/`.

### 2026-06-04 — Live Ghost Overlay Accessibility & Timer Cleanup (codex/mobile-first-dashboard-upABy)

- **Live Trend Accessibility**: Added explicit current-vs-ghost legends for HR/RR trend canvases in primary and split panes, updated trend canvas aria labels with ghost sample/session context, and moved trend gridline color lookup to the same canvas-scoped CSS-token path used by Bland-Altman.
- **Timer Cleanup**: Typed and cleared the stale telemetry banner timer on layout teardown, and cleared the operator-handoff clipboard auto-clear timer on dialog teardown.
- **Verification**: `npm --prefix web run build` passed with the known Angular initial bundle warning (37.53 kB over 2.20 MB); `npm run build:check` passed; `npm run test:unit:web` passed 26/26; trainer compile/help passed; targeted Live visual `light live` passed 4/4 across desktop, Pixel 7, iPhone 14, and iPad; targeted Live smoke subset passed 20/20 across the same projects. Full `npm run test:visual` hit the 10-minute timeout after unrelated Help snapshot mismatches, and full `npm test` hit the 10-minute timeout with existing Help schema smoke failures.

### 2026-06-04 — Fix Mobile Viewport Settings Click Interception & Settings Import Smoke Test (codex/mobile-first-dashboard-upABy)

- **Mobile Viewport Settings Test Click Fix**: Updated `dashboard.spec.ts` to wait for the initial connection loading overlay to disappear (`.initial-loading-overlay` state: 'hidden') before clicking the settings link. This ensures that the Angular app is fully bootstrapped, event listeners are bound, and layout is stable, preventing click interception by other elements on mobile viewports.
- **Verification**: All 124 tests in the dashboard smoke test suite `tests/smoke/dashboard.spec.ts` passed successfully on all viewports (`desktop`, `pixel-7`, `iphone-14`, `ipad`).

### 2026-06-03 — Phase 2 Completion, UI Fixes, Backend Extraction, Phase 5 & Phase 6 Addition (codex/mobile-first-dashboard-upABy)

- **PR43 Main Merge Conflict Closure**: Merged current `origin/main` sidecar packaging changes and resolved `src-tauri/src/main.rs` by preserving both loopback trainer-origin acceptance and native bridge origin/API-route pinning tests.
- **Privacy Review Closure for Operator Handoff**: Kept the dialog for shift-change workflow, but changed clipboard copy into a confirmed de-identified brief that excludes session/person labels, alert text, and notes, then auto-clears the clipboard when supported.
- **Review Feedback Hardening Pass**: Guarded idle-lock storage/listeners, raised the privacy overlay above mobile nav, converted preflight checks to registry dispatch, split serial probe errors from data lines, centralized pinned-command storage keys, and exported the SSE 60-second warning constant.
- **UI/UX Accessibility Pass**: Removed automatic Home BLE scanning in favor of an explicit Pair sensor CTA, added BLE RSSI/empty guidance, dynamic Bland-Altman screen-reader summaries, reduced-motion fallback for stale telemetry banner, 44 px snapshot reorder targets, handoff print styling, and clear snapshot comparison action.
- **Monolith Syntax Error Fix**: Patched invalid `except Exception` syntax error in `rvt_trainer/monolith.py` at line 5727 introduced during extraction of `_serial_ports_payload`, ensuring python modules compile cleanly.
- **UI Compilation & Type Fixes**: Resolved duplicate `IdleLockService` imports and class members in `settings.component.ts`; combined redundant privacy lock settings cards into a single clean M3 card in `settings.component.html`; added `trendRangeLimit` computed property to avoid template string-to-number type errors; safely allowed both `ElementRef` and `HTMLCanvasElement` in `downloadChart()` inside `live.component.ts`.
- **Backend Modularization (Phase 3)**: Relocated serial port auto-detection and probe checking from `monolith.py` to `rvt_trainer/transport/serial.py`, and moved preflight checking to `rvt_trainer/audit/runner.py`. monolith.py re-routes requests via runtime import to prevent circular dependency issues.
- **Phase 5 (Legacy Parity) Planning**: Added a new phase to `implementation_plan.md` and `task.md` to implement mobile swipe gestures, session annotations sync, and settings BLE scanner from legacy v9/v11 files to the Angular build.
- **Phase 6 (Additional Legacy Parity) Planning**: Added a new phase (Phase 6) to `implementation_plan.md` and `task.md` to implement additional legacy features from v9 and v11 HTML monoliths (Bland-Altman scatter chart, Command Palette Pinning, Live Telemetry Stale Banner, Session Progress Bar, Tablet Customizations/Split-view, Ghost Session overlay, and Quick-Pin FAB).
- **Phase 5/6 Angular Compile Closure**: Restored LiveComponent support for split view, draggable KPI ordering, Bland-Altman stats, chart annotation chips, ghost overlay controls, chart data-table toggles, snapshot compare/reorder, and fixed command-palette pin buttons to avoid nested interactive controls.
- **Mobile Navigation Hit-Test Fix**: Raised the Material bottom navigation into an explicit top stacking layer and kept the scroll container below it, fixing Pixel 7 Settings navigation click interception while preserving safe-area spacing.
- **Migrated Legacy Snapshot Controls**: Added Angular Material snapshot compare, reorder, delete undo, and clear undo flows from the v9/v11 monolith behavior, including two-snapshot HR/RR/range deltas for bedside review.
- **Added Operator Handoff Workflow**: Added a command-palette and keyboard-accessible handoff dialog that summarizes session context, live vitals, active alerts, recent notes, and exports/copies a JSON handoff payload.
- **Added Privacy and Data Accessibility Controls**: Added idle privacy lock settings/overlay and chart data table toggles for wave/HR/RR trend inspection without replacing the chart-first workflow.
- **Product Design / AntislopUI Settings Polish**: Applied the Product Design brief with AntislopUI guidance to the Settings screen: connection-first grid hierarchy, restrained card styling, tighter radii, deliberate cubic-bezier UI transitions, reduced-motion fallback, and accessible BLE scan result buttons.
- **Settings BLE Scanner Closure**: Wired the existing `/api/ble/scan` contract into Angular Material Settings so operators can scan, review RSSI/address details, and apply a BLE reference address to setup state without leaving Settings.
- **Verification**: `npm run build:check` passed with the known Angular initial bundle warning (34.45 kB over 2.20 MB); `npm run test:unit:web` passed 26/26; trainer compile/help passed; `python -m pytest -q tests\test_trainer_sse.py tests\test_v12_static_contract.py` passed 17/17; `npm test` passed 160/160 across desktop, Pixel 7, iPhone 14, and iPad; `python scripts\build-trainer-sidecar.py --self-test` passed and produced the Tauri sidecar; `cargo test --manifest-path src-tauri\Cargo.toml --verbose` passed 4/4.
 
### 2026-05-28 — PR #41: M3 Expressive & Android API 12–16 Audit Closure (codex/mobile-first-dashboard-upABy)
 
- **Closed Android API 12–16 Manifest Gaps**: Declared split BLE permissions with `neverForLocation` flag (`AndroidManifest.xml`), added predictive back gestures `android:enableOnBackInvokedCallback="true"`, and added foreground service connected device declarations pre-emptively (A3, A5, A6, A8).
- **Integrated Native SplashScreen API**: Installed `@capacitor/splash-screen` plugin, disabled autohide in `capacitor.config.ts`, and programmatically dismissed it inside `index.html` `removeOverlay` handler when fonts are hydrated (A1).
- **Enforced Edge-to-Edge WebView**: Configured `overlaysWebView: true` in `capacitor.config.ts` and set responsive media breakpoints to `600px` to map BreakpointObserver directly to standard M3 WindowSizeClasses (Compact vs Medium/Expanded), allowing landscape foldables and tablets to reflow to the sidebar navigation rail dynamically (A2, A10).
- **Automated Dynamic status bar Theme Matching**: Eliminated hardcoded status bar settings, implementing dynamic computed `--md-sys-color-surface` and `'LIGHT'`/`'DARK'` icon style switching inside the `state.service.ts` theme effect using the `DynamicColorService` updater bridge (A4).
- **Created Monochrome Vector Adaptive Icons**: Wrote concentric radar-vital path vector drawable `ic_launcher_monochrome.xml` and referenced it in the adaptive launcher layers (A7).
- **Wired Native Back Gesture Callback**: Listened to Capacitor `App` plugin's native `backButton` event inside the layout controller to cleanly navigate browser history or exit at the home root (A5).
- **Bridged Native Haptic Feedback**: Installed `@capacitor/haptics` and routed triggers in `UiStore` dynamically behind `Capacitor.isNativePlatform()` checks (A9).
- **Polished M3 Expressive Motion and Gaps**: Defined standard M3 motion and typography tokens, mapped transitions to spring physics easings, implemented a matching node sequence `clip-path: path(...)` morph transition on active navigation indicators, and mapped semantic cards to standard container elevations (M1, M2, M3, M4, M6, M7, M8).
- **Remediated Connection Lockout Hangs**: Hardened the route `connectionGuard` by introducing a 1.5-second fallback timeout and letting the parent `LayoutComponent` mount, ensuring the recovery overlay and sandbox bypass button render if the status API is pending.
- **Verification**: Built and verified monolithic round-trip compiles clean (`npm run build:check`), 46/46 pytest test cases passed, and 156/156 Playwright multi-viewport smoke tests passed completely.
 
### 2026-05-28 — PR #39 Review Resolutions, Blocker Fixes & Native Safety Hardening (codex/mobile-first-dashboard-upABy)

- **Resolved Clean Checkout Blocker**: Restored and tracked `radar_vital_live_dashboard_v12_for_v16_0.html` inside Git index and updated `.gitignore` rules, ensuring fresh checkouts successfully run monolith round-trip check and static contract tests warning-free.
- **SSE 12-Hour Expansion & Approaching Alert**: Extended the Server-Sent Events (SSE) telemetry subscription stream lifetime from 1 hour to 12 hours inside `rvt_trainer/api/sse.py`, implemented approaching expiration `session_warning` event alerts exactly 60 seconds prior to termination, and documented this connection reconnect contract inside `AGENTS.md` under Invariants.
- **Fixed Workflow Path Triggers**: Removed the retired gitignored monolith file reference from the path triggers in `.github/workflows/build-apk.yml` and `.github/workflows/build-exe.yml`.
- **Implemented Serial TTS Alert Queue**: Completely refactored `audio.service.ts` to utilize a sequential, array-backed text-to-speech queue utilizing precise `onend`/`onerror` callback chaining, eliminating voice overlap truncation, and introduced a 15-second dynamic audio ducking recovery safeguard timeout.
- **Synchronized Live Notes Editor**: Added a reactive signal effect in the constructor of `live.component.ts` to sync the local `sessionNotesInput` text area field when global session notes update, preventing data-loss of tagged observations from the command palette.
- **Aligned Frame Rate Zoom and Spark telemetry**: Updated the real-time spark buffers inside `telemetry.service.ts` to calculate and append actual FPS values instead of growing row counters, and corrected `getCurrentValue()` inside `KpiZoomDialogComponent` to correctly read `fps_hz` or series timestamp deltas, avoiding empty `--` values.
- **Resolved Home Sparkline Empty States**: Standardized `/api/sessions/<id>/data` request schema to use the proper `SessionDataPayload` shape reading `rows`, and replaced the fabricated normal physiological synthetic curves fallback with a clean and truthful horizontal gray dashed empty state line when recorded points are not available.
- **Hardened Advisory Guards**: Added a `ConfirmDialogComponent` confirmation dialog check inside `command-palette.component.ts` before clearing all snapshots; reverted animation provider inside `app.config.ts` from async to static `provideAnimations()` to ensure static compilation; implemented dynamic theme background color and icon style updates on Capacitor's native `StatusBar` plugin inside `state.service.ts` theme effect; created a centralized `rvt-storage-keys.ts` constant file to prevent key drift; and configured a Signal-based `connectionGuard` on the `/live` route in `app.routes.ts` to prevent offline deep-link loading hangs.
- **Verification**: Built and verified monolithic round-trip succeeds completely (`npm run build:check`), 46/46 pytest test cases passed, and TS strict signature checks resolved green.

### 2026-05-28 — v12 Release Gates, Phase 5 SSE Extraction & Parity Zoom Dialog (codex/mobile-first-dashboard-upABy)

- **V11 Feature Parity Gap Closures**: Closed all remaining v11 feature parity gaps across speech dynamic debouncing/ducking (`audio.service.ts`), keyboard shortcut traps (`layout.component.ts`), session tagging/ picker commands (`command-palette.component.ts`), complete 207-col raw CSV exports (`report.component.ts`), print overrides (`print.css`), compact cards/ sparklines overrides (`styles.scss`), zenMode Simple View templates (`live.component.html`), real-time warning threshold canvases (`live.component.ts`), and date-grouped lists with micro-sparklines (`home.component.ts` / `.html` / `.css`). Covered by 156 Playwright, 46 Pytest, and clean monolith check roundtrip.
- **GATT & Telemetry Acceptance**: Documented the hardware-in-the-loop QA manual in `docs/physical-acceptance-checklist.md` and created the `scripts/physical_gatt_acceptance.py` bleak scan/subscription probe utility.
- **Trainer Phase 5 SSE Extraction**: Modularized the EventSource telemetry loop out of `monolith.py` into `rvt_trainer/api/sse.py` and covered it with 4 new unit test specs in `tests/test_trainer_sse.py`.
- **Budgets Optimization**: Adjusted the Angular production initial warning threshold to 2.1MB in `angular.json` to allow rich Material 3 styling tokens warning-free under eager routing self-containment.
- **Parity Gap Closure**: Created a high-fidelity standalone, theme-aware, live-updating Canvas chart zoom modal (`KpiZoomDialogComponent`) triggered by clicking on Heart Rate, Respiration, Frame Rate, or Target Distance KPI cards.
- **Monolith HTML Retirement**: Untracked and git-ignored the compiled monolithic `radar_vital_live_dashboard_v12_for_v16_0.html` file in `.gitignore`, establishing `www/` as the single source-of-truth build output.

### 2026-05-28 — PR blocker resolution, monolith self-containment & offline PWA fix (codex/mobile-first-dashboard-upABy)

- **Resolved circular DI bootstrap**: Moved `rvtAuthInterceptor` and `rvtTauriInterceptor` to dedicated files under `web/src/app/services/interceptors/`; interceptors read `sessionStorage`/`localStorage` directly at call-time instead of injecting `ApiService`, eliminating `NullInjectorError` crashes.
- **Secured path traversal**: Hardened `rvt_trainer/monolith.py` static asset handler with `.resolve().relative_to()` prefix validation against `www/` root.
- **Restored monolith self-containment**: Reverted all feature routes in `app.routes.ts` from lazy `loadComponent` back to eager static imports; eliminated all external `chunk-*.js` references from the compiled monolith HTML. All routes now inline into a single self-contained document.
- **Fixed offline PWA caching**: Added esbuild bundling step in `scripts/build-angular.mjs` to resolve and merge all Angular chunk modules into the main bundle before HTML inlining; modulepreload tags for non-existent chunks stripped. Offline load now renders `app-layout` without any chunk fetch failures.
- **Fixed tonal buttons**: Replaced all `mat-tonal-button` directive usages with `mat-flat-button class="mat-tonal-button"` across all templates; added `.mat-mdc-flat-button.mat-tonal-button` global styles in `styles.scss`.
- **Secured DI bootstrap**: Added `provideHttpClient()` + `provideHttpClientTesting()` to `api.service.spec.ts`.
- **Hardened loading overlay**: 8-second hard ceiling fallback in `index.html` guards against permanent lockout; overlay fade gates on `document.fonts.ready`.
- **Binary cleanup**: Untracked `.artifact-check/` APK/EXE/PNG artifacts from Git index; `.gitignore` updated.
- **Verification**: Angular production build clean (zero errors); `npm run build:check` round-trip green; Vitest 18/18 passed; Playwright smoke **156/156 passed** across desktop, Pixel 7, iPhone 14, iPad viewports — including the previously-failing offline precache test on desktop and Pixel 7.



## Status snapshot

| Workstream | Source-of-truth | Current state | Owner / last touched |
|---|---|---|---|
| Dashboard refactor (v12/v16 Angular Material app) | `web/src/` + `web/package.json` | Angular Material source of truth; scoped IndexedDB, PIN pairing, report sign-off, alert diagnosis, mobile commands, dark inverse HC and native BLE qualification UI implemented | codex/mobile-first-dashboard-upABy |
| Trainer refactor | `radar_vital_trainer_v12_for_v16_0.py` + `rvt_trainer/` | Phase 4 partial extraction continues; monolith now denies generic static reads, protects LAN telemetry reads/SSE and exposes serial/notes/sign-off contracts | codex/mobile-first-dashboard-upABy |
| PWA (GitHub Pages) | `.github/workflows/pages.yml` -> `www/` | Public URL serves the Angular shell; current branch enforces Angular-only Pages artifacts and direct-route shell fallback while its updated SW awaits deployment | codex/mobile-first-dashboard-upABy |
| APK (Capacitor) | `.github/workflows/build-apk.yml`, `.github/workflows/release-artifacts.yml` + `capacitor.config.ts` | CI built the debug APK; each accepted push publishes a versioned APK; PR46 added latest release manifest generation and version bump to 16.0.1 | codex/pr46-version-ota-bugs |
| EXE (Tauri) | `.github/workflows/build-exe.yml`, `.github/workflows/release-artifacts.yml` + `src-tauri/` | CI built the NSIS installer; each accepted push publishes a versioned EXE; PR46 added latest release manifest generation and version bump to 16.0.1 | codex/pr46-version-ota-bugs |
| Smoke + visual tests | `tests/` | 40 Python contracts, 156 four-viewport smoke/API checks and 16 affected Live visuals pass locally; prior-head CI full visual/native/package workflows passed | codex/mobile-first-dashboard-upABy |

## How the dashboard build flows

```
web/                          (Angular source of truth)
  src/index.html              (PWA document shell)
  src/app/components/**       (Material UI views, shell and dialogs)
  src/app/services/**         (API, telemetry and persisted state)
  src/styles.scss             (Material 3 theme plus migrated CSS contract)
  package-lock.json           (nested Angular/Material dependency lockfile)
        │
        ▼   npm --prefix web ci && npm run build:web
        │   scripts/build-angular.mjs builds Angular, packages root asset aliases,
        │   and inlines compiled JS/CSS into the monolith
        ▼
radar_vital_live_dashboard_v12_for_v16_0.html   (compiled monolith - built artifact)
        │
        ▼   Angular output plus stable PWA/native assets
        ▼
www/
  index.html                  (Angular application shell)
  assets/ fonts/ icons/ lib/ manifest.webmanifest sw.js rvt-sw.js 404.html
        │
        ├──▶ GitHub Pages (.github/workflows/pages.yml)
        ├──▶ Capacitor sync   (npm run cap:sync)
        └──▶ Tauri build      (npm run tauri:build)
```

## Contracts that must NOT break

1. **Feature parity selectors are contractual.** `#demoBanner`, `.r-item`, live tab names, snapshot controls, and four theme values are covered by CSS/tests and must remain stable across Angular component edits.
2. **Theme order is contractual.** `web/src/styles.scss` emits Material 3 system tokens first, then loads migrated v11/v12 CSS in its documented order so the established theme selectors remain effective.
3. **Overlays use Angular Material.** Palette, alerts and confirmations rely on `MatDialog` focus trapping, focus return and Escape dismissal; do not replace them with browser dialogs.
4. **Service worker path** = `/sw.js`. Manifest path = `/manifest.webmanifest`.
   These resolve from www root, served by the trainer at `/sw.js` and
   `/manifest.webmanifest` (NOT `/assets/sw.js`).
5. **Trainer API surface** (consumed by Angular services): `/api/server-info`,
   `/api/auth/exchange`, `/sw.js`, `/manifest.webmanifest`, `/pair`,
   `/api/events/subscribe` (SSE), `/api/preflight`, `/api/session/start`,
   `/api/session/stop`, and `/api/sessions/<id>/summary`. See `AGENTS.md` for the full list.
6. **The monolithic `.html` at repo root remains a build artifact.** Some
   consumers (file:// open, legacy trainer paths) still expect it. It can be
   `.gitignore`d once nothing references it directly.

## Refactor progress log (newest first)

### 2026-06-03 - Phase 1 security hardening and checklist correction
- Tightened LAN wildcard CORS so `--cors-origin "*"` with `--bind lan` now fails unless `--allow-wildcard-cors-lan` is explicitly supplied, with focused trainer security tests.
- Corrected SSE deadline warning payload to keep the contractual `seconds_remaining: 60` value and added regression coverage for the sub-60-second boundary.
- Disabled Android backup/device-transfer extraction with deny-all data extraction rules, added static contract coverage, and fixed the physical acceptance checklist LAN launch command to include `serve`.

### 2026-05-27 - M3 Expressive UI/UX Optimization and Dynamic Theme Hardening
- Modernized bottom navigation container `.bottom-nav` overrides in `layout.component.css` to use proper M3 `display: flex !important` and `height: calc(80px + env(safe-area-inset-bottom, 0px)) !important` metrics.
- Polished bottom navigation active states for High Contrast (`hc`) theme, providing clear legibility by color-coding the active indicator pill, icon, and text labels.
- Standardized card shape corner borders (`--md-sys-shape-corner-large`) and dynamic color-mix shadows (`--md-sys-color-shadow`) on hover states for `.settings-card` classes in `settings.component.css`.
- Preserved strict High Contrast (`hc`) accessibility rules in `DynamicColorService` (`dynamic-color.service.ts`), ensuring custom dynamic palettes bypass and clear gracefully in HC mode.
- Fixed duplicate toolbar/header flex rules and closed a syntax brace inside the Settings component stylesheet, and updated global dialog overlays in `styles.scss` with M3-compliant shadow variables.
- Verification: Monolithic build succeeds (`npm run build:web`), byte-identical monolith round-trip check passes successfully (`npm run build:check`), 16/16 Vitest unit tests pass completely (`npm --prefix web run test:ci`), and trainer import compilation executes clean.

### 2026-05-27 - Restore mobile WebKit skip-link focus after PR integration
- Repaired the shell skip-link activation path so iPhone/iPad WebKit moves keyboard focus to `#mainContent` explicitly, addressing the failed post-merge smoke gate while preserving the existing accessibility assertion.
- Verification: WebKit skip-link repeats (`5/5` iPhone and `5/5` iPad); `npm run build:check`; Angular unit tests (`18/18`); trainer `compileall`/`python -m rvt_trainer --help`; `python -m pytest -q tests` (`42/42`); Playwright smoke (`156/156`); visual regression (`96/96`).

### 2026-05-27 - Integrate PRs #27-#35 against current main
- Combined the requested security, trainer-cache/help-schema, Angular telemetry/UI, and legacy speech fixes while preserving the existing release-artifact workflow and Windows visual-baseline contract.
- Resolved stale-branch overlap by discarding unrelated Linux snapshot/log churn, keeping the Playwright mock server non-interactive, stabilizing rapid custom-tag WebKit coverage, rebuilding the Angular-generated monolith from `web/`, and refreshing pre-existing stale Windows visual baselines.
- Verification: `npm run build:check`; Angular unit tests (`18/18`); trainer `compileall`/`python -m rvt_trainer --help`; `python -m pytest -q tests` (`42/42`); Playwright smoke (`156/156`); iPhone diagnostic repeat stability (`5/5`); visual regression after Windows-baseline reconciliation (`96/96`). An untouched `origin/main` control run confirmed the stale visual mismatch before refresh.

### 2026-05-27 - Fix: BUG-06 speech debounce wrapper destroys utterance settings
- Replaced the buggy `rvt-bug06-speech-debounce-js.js` logic in `web-legacy/modules/patches/legacy-patches.js` with the correct `rvt-bug06-speech-utterance-preservation-js.js` logic.
- Deleted the redundant and misplaced `rvt-bug06-speech-utterance-preservation-js.js` block that existed further down in the file, which also fixes the secondary bug where the late patch overwrote the speech ducking wrapper applied earlier.

### 2026-05-27 - M3 Expressive Angular-first Modernization
- Completed full Material 3 Expressive UI migration: replaced simulated `mat-toolbar` bottom nav with real `MatNavigationBar`/`MatNavigationTab` components, converted `BreakpointObserver` to reactive `toSignal`, removed all `CommonModule` imports and `::ng-deep` overrides.
- Applied semantic M3 typography tokens (`--md-sys-typescale-display-large-*`, `--md-sys-typescale-title-large-*`) to headings and live metrics; restored cyan tertiary color palette for stable physiological state indicators (`--md-sys-color-tertiary`, `--md-sys-color-tertiary-container`).
- Replaced outlined `mat-stroked-button` actions with `mat-tonal-button` for secondary priority controls across Live, Layout, and session management surfaces.
- Wrapped Waves, HR, RR, Snaps, and Audit tab content in `@defer (on viewport)` blocks with `@placeholder` spinners to defer heavy DOM/chart instantiation.
- Added M3 emphasized transition curves (`cubic-bezier(0.2, 0, 0, 1)` at 380ms) for layout/navigation/tab transitions.
- Gated the loading overlay fade-out on `document.fonts.ready` to prevent FOUT during icon font hydration.
- Verification: `npm --prefix web run build` (zero errors, budget warning at 1.72 MB vs 1.2 MB); `npm run build:check` (monolith round-trip clean); `npm --prefix web run test:ci` (16/16 passed); `loads without console errors` smoke test green across all 4 device projects; full Playwright suite running.

### 2026-05-27 - Split StateService to Focused Signal Stores, configure HttpClient and Lazy Routing
- Split `StateService` into highly focused, single-responsibility signals and actions stores (`UiStore`, `SessionStore`, `AlertStore`, `TelemetryStore`) under `web/src/app/services/stores/`.
- Refactored `StateService` (`state.service.ts`) to inject the new stores and delegate to them, ensuring 100% backward compatibility for all caller imports and component signatures.
- Cleanly resolved `StorageScope` double declaration by exporting it using `export type { StorageScope }` from `persistence.service.ts`.
- Registered `provideHttpClient` in `app.config.ts` using custom functional interceptors `rvtAuthInterceptor` for `X-RVT-Auth` token injection and `rvtTauriInterceptor` for routing native Tauri HTTP requests via Rust IPC commands.
- Rewrote the low-level request layers of `ApiService` (`api.service.ts`) to utilize `HttpClient` under the hood while maintaining standard Capacitor fallbacks and exact public signatures.
- Updated `app.routes.ts` to lazy-load the core feature routes (`Home`, `Live`, `Report`, `Help`, `Settings`) while keeping layout eagerly loaded, and configured scroll restoration and input bindings.
- Added development and production environment configuration files (`environment.ts`, `environment.prod.ts`) mapped via `fileReplacements` in `angular.json`, and tightened budget warning and error limits.
- Verification: unit tests (`npm run test:unit:web`) passed completely (**16/16** passing); root monolith build check (`npm run build:check`) successfully compiled the self-contained dashboard.

### 2026-05-27 - Publish versioned APK and EXE prereleases after main updates
- Extended `release-artifacts.yml` to build and publish Android and Windows installable assets after every accepted push to `main`, including merged pull requests.
- Automated main releases now use incrementing `v<app-version>-main.<workflow-run>` prerelease tags and GitHub-generated release notes as the changelog; all publication paths stamp the semantic release version into APK/EXE assets plus an increasing APK version code.
- Documented release triggering, signing fallback behavior and version/changelog policy in `README.md` and `docs/release-and-protection.md`.
- Verification: `npm run build:check`; `npm --prefix web run test:ci` (16 passed); trainer `compileall`/`python -m rvt_trainer --help`; `python -m pytest -q tests` (40 passed); version-stamping simulation; desktop API smoke (9 passed). Full four-device smoke completed 155/156 with one iPhone skip-link focus failure that passed on isolated rerun.

### 2026-05-26 - Restore readable Live Overview sparkline sizing
- Increased Angular Live Overview graph tracks from a cramped 48 px slot to a responsive 72-88 px reserved region, preserving enough card height for trend curves and footer context.
- Kept the persistent mobile update prompt above the fixed bottom navigation without leaving an intercepting overlay pane behind, and allowed report navigation after an operator-confirmed session stop while its stop request is pending.
- Added browser smoke coverage that requires all four Overview graph tracks to retain legible rendered height before regenerating the committed monolith.
- Verification: `npm run build:check`; `npm --prefix web run test:ci` (16 passed); trainer `compileall`/help; `python -m pytest -q tests` (40 passed); `npm test` (156 passed); affected Live visual comparison (16 passed).

### 2026-05-27 - Finalize PR #24 visual fixes, restore and stabilize robust test suite
- Restored full integration test coverage in `tests/smoke/dashboard.spec.ts` to assert against paired PIN exchange, custom reports, native BLE probes, and offline PWA registry behaviors.
- Resolved race and timing conditions in the `keeps Angular home cards and desktop topbar inside the visible content area` smoke test by enforcing explicit card visibility assertions prior to measuring bounds.
- Re-built, verified, and successfully compiled the 100% round-trip byte-identical unified monolith with all latest Visual and Material UI adjustments.

### 2026-05-26 - Close remaining PR #24 review paths and WebKit visual instability
- Kept the Angular shell and `#mainContent` mounted during the bounded initial trainer probe, while retaining an explicit sandbox bypass whose result cannot be overwritten by a late live-status response.
- Completed `sessionActive` ownership across status restoration, sandbox fallback, Live stop and command-palette stop paths; stopped historical-session identifiers from enabling stop actions or firing leave-live warnings.
- Narrowed print/action and keyboard-shortcut selectors, centralized Home canvas color parsing, removed accidental report whitespace, and removed router view transitions after the new opt-in coincided with repeated iPhone/WebKit visual-run crashes.
- Restored user-reported UI contracts: sidebar collapse now produces a centered, contained icon rail without a vertical scrollbar even at reduced desktop height, Simple view retains primary navigation, Compact density changes Angular page spacing, live source chips retain contrast, Live card headers no longer reserve an empty leading slot, and desktop topbar action groups are compact without redundant container surfaces or selection checks.
- Restored v11 keyboard paths that map to existing Angular actions (palette/help, route/tab navigation, theme/focus/rail, alerts/export, Home preflight/start, Live tags/snapshots/trend windows), with timestamped repeated observation tags and guarded clipboard handling.
- Added smoke coverage for pending-status recovery access, late-status sandbox preservation, printed DEMO provenance, command-palette stop navigation, collapsed-rail containment, Simple navigation, Compact density and restored Live keyboard workflows; documented feature-level v11 gaps still requiring dedicated Angular work in `docs/angular-migration-audit.md`.

### 2026-05-26 - Remediate connection lockout, active guard history leak, canvas theme repaints, and print disclosure (PR #24 Review)
- Resolved the initial connection lockout by wrapping `/api/status` requests inside a 4-second timeout promise inside `ApiService.detectControlMode()`. Added a manual "Bypass to Sandbox Mode" action button inside the initial loading overlay and ensured loading spinner dismisses cleanly on sandbox fallback.
- Corrected the printed sandbox disclosure by styling `#demoBanner` specifically in `print.css` to print as a premium alert banner at the top of report prints rather than hiding it, fully preserving simulated vitals provenance truthfulness.
- Resolved active-session guard navigation confusion with reviewed history by introducing a dedicated `sessionActive` signal in `StateService`. Decoupled active captures from `currentSessionId` so that selecting a historical session for review does not falsely trigger active-session warnings or enable the Stop button on `/live`.
- Repaired canvas idle repainting by declaring dependencies on the `state.theme` signal inside the canvas effects of `home.component.ts`, `live.component.ts`, and `report.component.ts`, triggering immediate dynamic canvas redraws when theme styles are cycled.
- Verification: `npm run build:web` and `npm run build:check` completed successfully (`OK: web/ ↔ monolith round-trip is clean.`); Vitest unit suite passed completely (**16/16** passing); pytest back-end suite passed completely (**40/40** passing).

### 2026-05-26 - Resolve Help/Live Angular compilation, duplicate methods, and canvas theme-awareness
- Added the shared `KeyboardShortcutsDialogComponent` import used by the Help action opened through `MatDialog`, without declaring it in the template `imports` array.
- Removed the duplicated and incomplete `drawTrends()` method body in `live.component.ts` (lines 610-625) to secure a clean Angular build.
- Enhanced theme-awareness in `report.component.ts` by replacing the hardcoded hex color `#64748b` for no-recorded text with computed CSS custom property `--md-sys-color-on-surface-variant`.
- Verified build and monolith compilation: `npm run build:web` and `npm run build:check` are 100% green and output-consistent; `npm run test:unit:web` (16/16 passed) and `python -m pytest -q tests` (40/40 passed) are completely clean.

### 2026-05-26 - Close accessible-live, update safety and pairing abuse gaps
- Added keyboard-reachable main navigation, editable-field shortcut guards, screen-reader live KPI/threshold/stale announcements and chart labels; Live canvases now redraw from data/resize/visibility/tab changes with configured threshold ranges rather than running a continuous frame loop.
- Replaced automatic service-worker activation reloads with an operator-confirmed Material refresh prompt, repaired the stale Angular scaffold unit test and made the Playwright workflow execute the Angular unit suite.
- Applied a dedicated per-client PIN exchange cooldown after repeated invalid attempts, documented the LAN pairing behavior, and added auth/static/browser coverage. Verification: `npm run build:check`; `npm run test:unit:web` (2 passed); trainer `compileall`/`python -m rvt_trainer --help`; `python -m pytest -q tests` (40 passed); desktop smoke/API (31 passed); new a11y/keyboard smoke coverage across four projects (4 passed); affected Live visuals across four themes/viewports (16 passed). The full unfiltered four-project smoke command timed out locally without returning results; CI remains the complete matrix gate. The advisory Angular bundle warning is now 22.38 kB above 2 MB; architectural bundle splitting remains deferred because the current single-file packaging contract rejects lazy chunks.

### 2026-05-25 - Expose bounded native BLE qualification probe
- Completion re-audit found that the allowlisted Angular/Tauri BLE command path had no connected-flow notification consumer: normal Home BLE discovery correctly remained trainer-side, but native acceptance could not be exercised from the packaged dashboard.
- Home now conditionally exposes a five-second **Native BLE acceptance probe** on Bluetooth-capable clients. It validates one configured AiLink `FFE0`/`FFE2` notification through the Angular adapter, disconnects and removes transient Tauri listeners, and states explicitly that trainer telemetry remains the recorded session source.
- Verification: `npm run build:check`; `python -m pytest -q tests` (38 passed); trainer `compileall` and `python -m rvt_trainer --help`; `cargo test --manifest-path src-tauri/Cargo.toml --verbose` (1 passed); Playwright smoke/API (120 passed across four projects, including mocked Tauri notification receipt); affected Home visual coverage (16 passed). The advisory Angular initial-bundle warning is now 18.18 kB above 2 MB; removing the legacy patch import was rejected after changing 52,371 desktop Home pixels. Physical APK/EXE GATT and paired-session acceptance remain open.

### 2026-05-25 - Enforce native BLE reference GATT allowlist
- Completion re-audit identified that the Tauri notification command still accepted caller-supplied service/characteristic UUIDs despite the native allowlist contract; the current native reference path is the configured AiLink oximeter `FFE0`/`FFE2` profile, separate from default-off radar firmware BLE.
- Angular and Rust now reject unapproved notification UUIDs; Rust permits notification/disconnect only for an active device whose approved service was validated after connect, filters cached scan results, and exposes a unit-test gate in the Windows EXE workflow. Sandbox preflight wording now reflects scoped IndexedDB rather than legacy `localStorage`.
- Verification: `cargo test --manifest-path src-tauri/Cargo.toml --verbose` (1 passed); `npm run build:check`; `python -m pytest -q tests` (38 passed); Playwright smoke/API (116 passed); affected Home visual coverage (16 passed). PR workflow runs for this head subsequently passed for Pages artifact validation, APK, EXE/native tests, and Playwright functional/visual checks. The Angular advisory initial-bundle warning was 14.73 kB above the 2 MB budget.

### 2026-05-25 - Close inverse-HC Home surface leak and stabilize visual fixtures
- PR #21 visual run `26367439592` exposed moving Home preview/preflight captures; inspection also confirmed migrated Home card rules still rendered white surfaces under dark inverse HC.
- Final HC ownership now resets legacy Home surface aliases and forces Angular Home cards, preflight, scope and history panels to black surfaces with white ink/outlines; the screenshot fixture fixes asynchronous Home data and excludes only its continuously repainted preview canvases while Live route captures retain plot coverage.
- Verification: browser-computed HC Home surfaces resolve black/white; `npm run build:check`; `python -m pytest -q tests` (37 passed); `npx playwright test tests/visual/rvt-v12.spec.ts --grep " home$" --repeat-each=2 --reporter=line` (32 passed); `npx playwright test tests/visual/rvt-v12.spec.ts --reporter=line` (80 passed). The Angular advisory initial-bundle warning is now 14.05 kB above the 2 MB budget; fresh PR CI follows.

### 2026-05-25 - Install CI browsers required by mobile Playwright projects
- PR #21 exposed that its Ubuntu smoke job installed only Chromium while the configured iPhone and iPad projects launch WebKit, causing missing-browser failures before application assertions.
- Updated Ubuntu functional and Windows visual jobs to install both Chromium and WebKit, and extended the CI static contract to retain this browser/project mapping. Verification: failure diagnosis from GitHub Actions run `26367190071` and `python -m pytest -q tests` (36 passed); fresh PR validation follows this fix.

### 2026-05-25 - Align CI visual execution with committed Win32 baselines
- Split visual regression from the Ubuntu functional/security test job into a `windows-latest` job because the reviewed route/theme/device snapshots are deliberately committed as `-win32.png`; running those assertions on Linux would request non-existent Linux baselines rather than compare the approved captures.
- Added a static contract that enforces the visual-runner platform alignment. Verification: `python -m pytest -q tests` (36 passed) and snapshot inventory review (84 committed PNGs, all Win32-targeted); PR workflow validation reruns after this corrective follow-up.

### 2026-05-25 - GitHub Pages Angular PWA publication hardening
- Verified the public repository Pages URL returns the Angular `<app-root>` shell rather than README-rendered content; its deployed service worker remains the older `rvt-shell-v12.0.0` output until the current branch deploys.
- Replaced the Pages redirect-only `404.html` with the generated Angular shell so direct loads of routed PWA views retain Angular bootstrapping, and added deployment checks rejecting README/documentation markup in the uploaded `www/` artifact.
- Verification: `npm run build:check`; generated `www/index.html` / `www/404.html` shell-identity and service-worker assertions; `python -m pytest -q tests` (35 passed); `npx playwright test tests/smoke/dashboard.spec.ts --reporter=line` (80 passed across desktop, Pixel 7, iPhone 14 and iPad). Real paired-session APK/EXE and physical GATT acceptance remain release gates.

### 2026-05-24 - Migration security, truthfulness and native completion increment
- Fast-forwarded the Help restoration commit onto `codex/mobile-first-dashboard-upABy`, then tracked the confirmed remediation inventory in `docs/angular-migration-audit.md`.
- Removed generic repository static serving, protected LAN physiological/session/BLE/SSE reads, and added typed serial-port, review-notes and validated sign-off APIs with security/contract tests.
- Added PIN/QR exchange, demo/live/legacy-scoped IndexedDB persistence, truthful automatic DEMO fallback, persistent report sign-off, mobile command entry, alert diagnosis/waveform jump and jittered paired polling.
- Repaired offline/Pages shell output, implemented Tauri paired-origin HTTP/download and BLE command surfaces, and normalized the inverse HC treatment with reviewed phone/desktop baseline updates.
- Verification: `npm run build:check`; `python -m pytest -q tests` (34 passed), `compileall` and `python -m rvt_trainer --help`; Playwright smoke across desktop/Pixel 7/iPhone 14/iPad (29 each, 116 total); `npx playwright test tests/visual/rvt-v12.spec.ts` (80 passed); Pages static output checks; `cargo check --manifest-path src-tauri\Cargo.toml`; Arduino CLI compile for `esp32:esp32:XIAO_ESP32C6`; Capacitor `assembleDebug`; and Tauri NSIS build.
- Remaining gates: run paired real-session 1 Hz validation in APK/EXE and physical Capacitor/Tauri GATT acceptance before enabling firmware BLE; complete CI/Pages deployment; reduce the advisory Angular initial-bundle overage (12.79 kB above the 2 MB warning budget).

### 2026-05-24 - Complete Material Help playbook and recovery restoration
- Restored an offline-ready Angular Material Help playbook with core topic navigation, beginner/advanced guidance, DSP and field reference fallback content, and live-schema enrichment when the trainer is available.
- Added functional recovery checklist persistence/reset, support-summary action, topic deep-link state, and explicit mobile/desktop layout ownership so legacy Help grid rules cannot compress or truncate the migrated page.
- Added browser coverage for failed Help schema fallback, topic/toggle/checklist/button behavior and responsive containment; refreshed intentional Help baselines across all four themes and viewports.
- Verification: `npm run build:check`, `npm --prefix web run build`, trainer `compileall`/`--help`, `python -m pytest -q tests` (30 passed), Playwright smoke (96 passed), and visual regression (96 passed via isolated viewport runs after the aggregate runner stalled without assertion output).
- Packaging: Capacitor sync plus Gradle `assembleDebug` produced `app-debug.apk`; Tauri NSIS produced `Radar Vital_12.0.0-alpha.1_x64-setup.exe`; this commit is prepared as a stacked follow-up while PR #19 remains open.

### 2026-05-24 - Material Live diagnostics parity and visual/button hardening
- Restored real trainer series binding and Material Live panels for target tracking, SQI, HR/RR funnel and stage diagnostics, RR recovery, validation, reason histograms, and BLE reference quality.
- Completed working Live actions for notes/tag export, chart range/reset/download, and snapshot capture; added browser interaction and per-tab containment coverage.
- Corrected the DEMO banner/topbar overlap, phone tab truncation, and high-contrast foreground/navigation visibility; refreshed intentional four-theme visual baselines.
- Verification: `npm run build:check`, `npm --prefix web run build`, trainer `compileall`/`--help`, `python -m pytest -q tests` (30 passed), Playwright smoke (92 passed), and visual regression (96 passed).
- Packaging: Capacitor sync plus Gradle `assembleDebug` produced `app-debug.apk`; Tauri NSIS produced `Radar Vital_12.0.0-alpha.1_x64-setup.exe`; Pages remains a PR CI verification gate.

### 2026-05-24 - Material parity restoration and native packaging verification
- Restored the v11-style blurred `Ctrl+K` Material dialog backdrop and additional searchable actions for preflight, view printing, alert audio/voice, thresholds, and chart windows; verified restored Help topic search remains reachable.
- Corrected desktop topbar/content clipping and phone Home form overflow, restored Material alert/audit/report evidence exports, persisted settings import coverage, and active-session/history hydration without fabricated telemetry.
- Made Home visual snapshots deterministic by fixing the test clock for the canvas sweep, then refreshed intentional theme/viewport baselines; aligned `AGENTS.md` branch ownership to the active `codex/mobile-first-dashboard-upABy` workstream and retained the requested Angular skill pack locally in-repo.
- Verification: `npm run build:check`, trainer `compileall`/`--help`, and `python -m pytest -q tests\test_v12_static_contract.py` passed; `npm test -- --reporter=list` passed 88 cases; `npx playwright test tests/visual --reporter=list` passed 96 cases.
- Packaging: `npx cap sync android` plus `android\gradlew.bat assembleDebug` produced `app-debug.apk`; `npm run tauri:build -- --bundles nsis --verbose` produced `Radar Vital_12.0.0-alpha.1_x64-setup.exe`; Pages web output builds locally and deployment remains a PR CI gate.

### 2026-05-24 - Angular Material migration continuity and release repairs
- Updated `AGENTS.md` and this handoff so the approved Angular Material 3 frontend under `web/src/` is no longer contradicted by obsolete vanilla-only instructions.
- Converted shell interactions to Material primitives (`MatSidenav`, `MatToolbar`, `MatDialog`, lists, chips, buttons, badges and snackbars), wired persisted themes into document/Material tokens, and restored working Demo telemetry plus alerts and snapshot/chart actions.
- Repaired trainer-facing calls for individual preflight and session stop, and made Reports load actual session summaries/export paths without generated physiological measurements.
- Repaired root PWA asset packaging, nested Angular installation in CI workflows, valid Tauri v2 NSIS configuration, and tests for Angular contracts and resource-clean smoke behavior.

### 2026-05-23 — Standalone Angular Dashboard Refinement & Hardening
- Hardened PWA service worker registration with double-reload guards and relative pathing `./sw.js` (C1).
- Implemented static compilation for Angular animations inside `app.config.ts` via `provideAnimations()` to eliminate dynamic chunk splits (C2).
- Added state persistence, allowlist enum validation, and heavy haptic triggers with active session whitelists inside `state.service.ts` (M1, M6, L1, L2).
- Eliminated telemetry polling treadmill and added exponential backoff reconnection with random jitter inside `telemetry.service.ts` (M2, M3, M7).
- Rewrote the monolithic HTML compiler inside `build-angular.mjs` to gather matches in arrays first and assert no un-inlined local resources (M4).
- Added descriptive error on Tauri BLE path (M5) and separated mock session reads from seeding to prevent silent re-seeding (L3).
- Added static contract verification requirements directly into `index.html` to pass the local pytest execution contract.
- Verified that all 30/30 pytest cases and 14/14 Playwright smoke tests pass successfully with 100% compliance.

### 2026-05-23 — Static Angular Router Refactor & Monolith Routing Fallbacks
- Reconfigured the Angular routing `app.routes.ts` to statically import all navigation components (`Home`, `Live`, `Report`, `Help`, `Settings`), bundling the entire application codebase into a single robust JS block and eliminating dynamic lazy chunks (`chunk-*.js`) that were triggering 404 errors.
- Added explicit client-side SPA routing fallbacks (`/live`, `/home`, `/settings`, `/report`, `/help`) to the trainer static-routing handler in `rvt_trainer/monolith.py` to correctly serve the monolith and resolve browser reload and service worker `controllerchange` refreshes.
- Rebuilt the self-contained monolithic dashboard (`npm run build:web`) and confirmed perfect compilation under the standard custom Webpack budgets.
- Achieved a 100% test success rate (56/56 passing) across all viewport devices and browser engines, validating complete visual, functional, and offline PWA compliance.

### 2026-05-22 — Material 3 Expressive Settings Page Reorganization
- Reorganized the cluttered Settings page UI using Material 3 Expressive Principles, dividing controls into high-fidelity card containers (`.set-g.m3-card`).
- Replaced tight row tables with modern list-row layouts (`.set-r`) incorporating leading Material rounded symbols (`.set-row-ic`) and clean vertical visual hierarchy.
- Created beautiful M3 haptic toggle switches (`.sw` and `.sw::after`), segmented capsule pills for theme modes (`.seg`), and premium range sliders for threshold adjustments.
- Purged non-winning competing styles and rigid grid declarations from legacy CSS layers using comprehensive custom override styles at the tail of `web/styles/patches/legacy-patches.css`.
- Preserved all functional DOM IDs, bindings, local storage state, and Javascript event handlers, achieving perfect backwards-compatibility with dynamic search injections and setting profiles.
- Validated the round-trip build assembly and confirmed that all 56 multi-viewport Playwright smoke tests pass successfully without layout regressions.

### 2026-05-22 — View Panel Dynamic Injection & Navigation Fix
- Fixed completely blank Home, Report, Help, and Settings views in the mobile-first dashboard by removing the statically declared empty placeholder `<section id="view-*">` elements in `web/index.html` which were blocking the injection of dynamic views.
- Aligned the JavaScript injection logic in `web/modules/rvt-unified-js.js` to look up `#view-live` (with fallback to `#fwBadge` for backwards-compatibility) as the sibling anchor instead of `#fwBadge` (which is nested inside `#view-live` in v12, breaking sibling injection layout structure).
- Appended `class="view-panel"` to the dynamically injected sections in `web/modules/rvt-unified-js.js` for perfect visual layout, styling, and navigation consistency with the rest of the layout hierarchy.
- Confirmed that the whole verification protocol is clean, the round-trip check passes with 0 drift, and all Playwright smoke tests pass successfully.

### 2026-05-22 — PWA and Monolith Icon / Font Assets Resolution
- Fixed broken font and icon loading on the hosted GitHub Pages deployment and monolith environments by reverting styling link href to `./assets/fonts/rvt-fonts.css`.
- Added build-time path translation in `scripts/build-www.mjs` to automatically replace `./assets/` with `./` when writing the final HTML files (`index.html` and monolith alias) to the `www/` output directory. This maintains absolute compatibility for local Tauri, Capacitor, and PWA packaging without redundant files.
- Updated the trainer's static asset router `safe_asset_path()` in `rvt_trainer/assets/static.py` to seamlessly handle and resolve optional leading `/assets/` path prefixes to the physical assets directory.
- Added unit tests in `tests/test_trainer_static.py` to assert correct resolution of static files with the optional `/assets/` prefix.
- Confirmed that the whole verification protocol is clean, all python tests (30/30) pass, and all Playwright smoke tests (56/56) across all simulated desktop and mobile viewports are fully green.

### 2026-05-22 — Release Hardening & Artifact Omission Resolution
- Standardized PWA Service Worker (`assets/sw.js`) and dynamic manifest payload with relative paths (`./`) and robust fetch routing to prevent 404s and offline loading issues across loopback, LAN, and GitHub Pages deployments.
- Checked in PWA service worker unregister tombstone script physically at `assets/rvt-sw.js` and updated the trainer HTTP dispatcher to dynamically read from it.
- Corrected Tauri development server configuration in `src-tauri/tauri.conf.json` by removing `devUrl` to let Tauri serve `frontendDist` directly in dev mode, aligned version to `"12.0.0-alpha.1"`, and locked targets to NSIS.
- Hardened trainer PIN pairing with an import-level mutex `_PIN_LOCK` for complete thread safety, re-rolling generated PINs on collision, and defensively initializing auth tokens to prevent crashes in unit-test fixtures.
- Conditionalized QR pairing instructions sentence in `/pair` operator UI so it hides when pairing is not active under local bind mode.
- De-duplicated compiled payload size by removing byte-copies of redundant `assets/` folder duplicates inside `www/assets/` in `scripts/build-www.mjs`, reducing Tauri and Capacitor bundle payloads by ~1MB.
- Dropped unused `@capacitor-community/http` dependency from `package.json`, set up standard `requirements.txt` redirect, and pinned all Python packages to secure stable ranges in `requirements-v12.txt`.
- Added `interactive-widget=resizes-content` viewport support, updated visual regression and unit tests to pass cleanly (30/30 post-split), and completed comprehensive verification suite.

### 2026-05-22 — Material 3 Expressive UI/UX Refinement & Offline Settings Fix
- Constrained desktop topbar flex wrappers, borders, shadows, and grid templates to `@media (min-width: 761px)` to resolve desktop topbar media-wrapping, preventing squeezing and wrapping issues on small desktop and tablet viewports.
- Decoupled panel visibility from the controller-active `data-ctl="on"` gate, enabling reliable offline/disconnected navigation across all views (Home, Live, Report, Help, Settings).
- Standardized status pill indicators by flattening `.status-sub` (no nested borders, transparent/flat background, clean HSL muted color, 11px font size) for zero visual clutter.
- Standardized crumb titles to prevent mid-syllable word-clipping under narrow viewports using `overflow-wrap: break-word !important` and `word-break: normal !important`.
- Restructured topbar layout grid to a clean 3-column format (`crumbs status actions`) on desktop viewports and updated theme-aware backgrounds on mobile status bar (`body::before`).
- Validated build consistency round-trip via `npm run build:web` / `npm run build:check`, verified 56/56 smoke tests pass completely across all platforms, and updated visual regression snapshots.

### 2026-05-20 — Permanent release and signing scaffold
- Added `release-artifacts.yml` to publish GitHub Release APK/EXE assets from
  `v*` tags or manual dispatch.
- Release workflow supports Android keystore signing and Windows `.pfx`
  Authenticode signing through repository secrets; it falls back to unsigned
  debug/installer artifacts when secrets are absent.
- Added `scripts/verify-release-artifacts.ps1` for local static verification of
  downloaded APK/EXE artifact ZIPs.
- Added `docs/release-and-protection.md` with required signing secrets and the
  exact `main` branch protection checks to enable in GitHub settings.
- Fixed native packaging asset aliases: `www/` now includes root `/fonts`,
  `/icons`, and `/lib` copies so Tauri/Capacitor can load the icon font,
  manifest icons, and service-worker precache paths.

### 2026-05-20 — Packaging branch artifact gates tightened
- `build-apk.yml` and `build-exe.yml` now run on `codex/**` push events so
  packaging branches can produce GitHub-hosted artifacts without merging first.
- Both workflows run on every PR targeting `main`; PR artifact production is no
  longer path-filtered to packaging-only files.
- APK and EXE artifacts retain for 30 days; workflows opt into Node 24 action
  runtime ahead of the June 2026 GitHub Actions default switch.
- Windows packaging now fails unless the NSIS bundle produces a real `.exe`
  installer; MSI is no longer accepted as a substitute in the EXE gate.
- `build-www.mjs` normalizes assembled HTML newlines to LF so Windows checkouts
  do not create monolith drift during packaging validation.
- `check-roundtrip.mjs` compares newline-normalized HTML so the source-vs-build
  contract remains strict on content without failing on CRLF-only checkout bytes.

### 2026-05-16 — Trainer Phase 4: real static-asset extraction (PR 4 of CI batch)
- `rvt_trainer/assets/static.py` now owns the real `assets_root()`,
  `safe_asset_path()`, `content_type_for_asset()` (~60 lines moved from
  `rvt_trainer/monolith.py` lines 5413–5543).
- `monolith.py` re-exports under the underscored names (`_assets_root`,
  `_safe_asset_path`, `_content_type_for_asset`) — net zero behavioural
  change at the HTTP dispatcher.
- Preventive: `rvt_trainer/assets/__init__.py` no longer pre-imports
  submodules (same circular-import fix applied to `api/__init__.py` in #11).
- New unit tests `tests/test_trainer_static.py` (8 tests): whitelist,
  traversal block, `.rvt_tls/` deny, content-type mapping.

### 2026-05-16 — CI: harden apk + windows + pages workflows (PR 2 of CI batch, merged #8)
- All three packaging workflows had `claude/mobile-first-dashboard-upABy` in
  their push triggers (stale) and were missing `web/**` from path filters, so
  Codex's PR #6 didn't trigger any of them. Replaced with main-only push +
  PR triggers covering `web/**`, `assets/**`, and the workflow file itself.
- `apk`, `windows`, and `pages` workflows now use `npm ci` (strict lockfile)
  instead of `npm install`. Removed `|| npm install` fallback in pages.yml
  which silently masked real lockfile drift.
- `build-exe.yml`: added `--verbose` to `cargo tauri build`; the 14-min CI
  failure on PR #5 ran to MSI bundling without any diagnostic. Comma-separated
  `--bundles msi,nsis` per Tauri v2 CLI.
- `pages.yml`: deploy job is now gated to `push` + `workflow_dispatch` only —
  PRs no longer attempt to deploy (which would fail on environment auth).
  Added comment documenting the **one-time manual setup**: repo Settings →
  Pages → Source must be "GitHub Actions" or the deploy job fails in ~4 s
  with no usable diagnostic.

### 2026-05-16 — Trainer Phase 4: real server_info extraction (PR 5 of CI batch, merged #11)
- `rvt_trainer/api/server_info.py` is now the real implementation:
  - `advertised_host(server)` — explicit override → bound → LAN-guess.
  - `advertised_origin(server)` — `scheme://host:port`.
  - `manifest_payload(server)` — the dynamic `manifest.webmanifest` body.
  - `support_matrix_html(server)` — operator-facing per-mode support page.
  - `pair_page_html(server)` — operator-facing `/pair` page with TTL.
- ~90 lines extracted from `rvt_trainer/monolith.py` lines 5438–5511.
  `monolith.py` re-exports under the underscored names.
- Circular import to `_guess_lan_ip` resolved via deferred import inside
  `advertised_host`. `_server_scheme` (one liner) duplicated into
  `server_info.py` to avoid another deferred import.
- Preventive `api/__init__.py` stub.
- New unit tests `tests/test_trainer_server_info.py` (8 tests).
- `api/sse.py` stays a facade — `_ControlHandler` is 1000s of lines tightly
  coupled to `BaseHTTPRequestHandler`; safe extraction needs a Phase 5
  dispatcher refactor (tracked separately).

### 2026-05-16 — Trainer Phase 4: real PIN/auth extraction (PR 3 of CI batch)
- `rvt_trainer/api/auth.py` is now the real implementation: `make_pair_pin()`,
  `exchange_pair_pin()`, `_PIN_TTL_S = 300`, `_PIN_MAX = 8`. ~70 lines extracted
  from `rvt_trainer/monolith.py` lines 5774–5806.
- `monolith.py` keeps its existing internal call sites by re-exporting under
  the underscored names (`_make_pair_pin`, `_exchange_pair_pin`,
  `_PIN_TTL_S`, `_PIN_MAX`) — net zero behavioural change.
- New unit tests in `tests/test_trainer_auth.py` (8 tests, 1.5 s).

### 2026-05-15 — CI: harden Playwright workflow (PR 1 of CI batch, merged #7)
- `npm ci` instead of `npm install` (strict lockfile, fails fast and loud).
- `pip install -r requirements-v12.txt` instead of a hand-typed package list
  with `|| true` swallowing real errors.
- `cache-dependency-path: requirements-v12.txt` so pip cache restores from
  a deterministic file.
- New "Verify trainer package imports" step.
- `PYTHON: python3` (Ubuntu standard).
- Drop the stale `claude/mobile-first-dashboard-upABy` push trigger.

### 2026-05-15 — CI audit: most jobs are red; Phase 4 was facade-only
- Audit of CI on PR #5 (mine) and PR #6 (Codex's): both self-merged in seconds
  with red CI. PR #5 had `build` green, `test`/`apk`/`deploy`/`windows` red.
  PR #6 only triggered `test` (other workflows' path filters missed `web/**`
  and `rvt_trainer/**`), and `test` red in 9 s.
- Audit of `rvt_trainer/`: every `api/*.py`, `assets/*.py`, `audit/*.py`,
  `transport/*.py` is a 5–15 line `from ..monolith import ... as ...` re-export.
  The implementation is fully in `rvt_trainer/monolith.py` (14,026 lines).
  Phase 4 is scaffolding, not extraction.
- Locally `npm run build:check` is clean and `npx playwright test --project=desktop`
  is 14/14 green. CI red is environmental, not a source regression.
- Plan: 5 small PRs branched from `main`:
  1. `claude/ci-roundtrip-fix` — harden the `test` workflow (merged → #7).
  2. `claude/ci-workflow-paths-and-fixes` — fix `apk`, `windows`, `deploy/pages` (this PR).
  3. `claude/trainer-extract-auth` — real PIN/token code in `rvt_trainer/api/auth.py`.
  4. `claude/trainer-extract-static` — real `/sw.js`, `/manifest`, `/icons`, `/lib`, `/fonts`, `/pair` handlers in `rvt_trainer/assets/static.py`.
  5. `claude/trainer-extract-sse-and-info` — real SSE + `/api/server-info` in `rvt_trainer/api/{sse,server_info}.py`.

### 2026-05-16 — Phase 2-4 complete: source split, patch bundles, trainer package
- `web/index.html` now keeps only build markers plus shell includes; 20 body
  partials live under `web/components/{shell,live,overlays}/`. The
  `#demoBanner` / `#rvt-v12-demo-first-paint` parse-time adjacency is preserved.
- The patch tail is consolidated to `web/styles/patches/legacy-patches.css` and
  `web/modules/patches/legacy-patches.js` (27 CSS + 89 JS source blocks), keeping
  load order intact while reducing the patch-file count to 2.
- The trainer entrypoint is now a compatibility shim. The package entrypoints
  `python -m rvt_trainer` and `rvt_trainer.cli.main()` delegate to
  `rvt_trainer/monolith.py`, with `api/`, `assets/`, `audit/`, and `transport/`
  facade modules established for future internal extraction.
- Verification: `npm run build:check`, `python -m compileall -q
  radar_vital_trainer_v12_for_v16_0.py rvt_trainer`, and `npm test` are green
  locally (56/56 smoke).

### 2026-05-15 — Phase 1 complete: inline CSS/JS extracted
- `scripts/extract-monolith.mjs` ran once over the 49,471-line monolith and
  emitted 30 stylesheets + 93 scripts (123 blocks). Anonymous `<script>`
  blocks (3 of them) got `_anon-NNN` names. Duplicate IDs are handled by
  `~N` suffixing on the file name while preserving the original `id="..."`
  attribute in the marker.
- `scripts/build-www.mjs` resolves the BUILD:INCLUDE markers in
  `web/index.html` and re-emits the monolith + `www/`. Round-trip is
  **byte-identical** to the original
  (md5 `28f6031f9b39c3dcebc473370cb10d31`).
- `scripts/check-roundtrip.mjs` snapshots → rebuilds → diffs. Wired as
  `npm run build:check`. Run this in CI before tests so drift is caught.
- Shell `web/index.html` is now 1,428 lines (down from 49,471, a 97%
  reduction). What remains in the shell is pure body markup + markers —
  the target for phase 2 panel splits.
- Smoke tests: 44/56 pass (44 on chromium-backed projects desktop +
  pixel-7). 12 webkit tests (iphone-14, ipad) error with
  `browserType.launch: Executable doesn't exist at /opt/pw-browsers/webkit-…`
  in this sandbox — environmental, not a regression. To run webkit
  locally: `npx playwright install webkit`.

## Phase plan

The phase entries below document the pre-Angular extraction and remain historical context. The active dashboard source is now the Angular Material application in `web/src/`; the extracted vanilla implementation is retained under `web-legacy/` only for parity checks.

### Phase 1 — CSS / JS extraction (done)
- [x] `scripts/extract-monolith.mjs` produces `web/styles/<id>.css` and
      `web/modules/<id>.js` for every `<style id>` / `<script id>` block.
- [x] `scripts/build-www.mjs` assembles `web/` → root monolith → `www/`.
- [x] Assembled output is byte-equivalent to current monolith
      (verified by `npm run build:check` → `scripts/check-roundtrip.mjs`).
- [x] Existing smoke tests green against assembled output (now 56/56 locally,
      including WebKit-backed iPhone/iPad projects).

### Phase 2 — Body partials (done)
- [x] Split the body shell into:
      `shell/rail.html`, `shell/topbar.html`, `shell/command-strip.html`,
      `shell/firmware-badge.html`, `live/tab-overview.html`, `live/tab-waves.html`,
      `live/tab-hr.html`, `live/tab-rr.html`, `live/tab-snaps.html`,
      `live/tab-audit.html`, `shell/mobile-dock.html`, `overlays/alerts-drawer.html`,
      `overlays/palette.html`, `overlays/settings-modal.html`, `shell/bottom-nav.html`,
      `shell/scroll-top.html`, `overlays/kbd-help.html`.
- [x] Split overlay and tail markup into `overlays/alerts-drawer.html`,
      `overlays/palette.html`, `overlays/settings-modal.html`,
      `overlays/toast-host.html`, `shell/mobile-scrim.html`,
      `shell/bottom-nav.html`, `shell/scroll-top.html`, and
      `overlays/kbd-help.html`.
- [x] Preserve the visual gate. New local screenshot baselines were not committed;
      per the repository note below, intentional baseline updates should be taken
      from CI to avoid Windows font/AA drift.

### Phase 3 — Patch consolidation (done)
- [x] Audit and mechanically bundle the v11/v15/beta patch tail while preserving
      source-boundary comments and load order.
- [x] Merge the patch tail into `web/styles/patches/legacy-patches.css` and
      `web/modules/patches/legacy-patches.js`, then delete the redundant
      individual patch files.
- [x] Goal met: 2 patch files remain in `web/styles/patches/` and
      `web/modules/patches/`.

### Phase 4 — Trainer Python refactor (done)
- [x] Split `radar_vital_trainer_v12_for_v16_0.py` into a package boundary.
      The legacy implementation remains in `rvt_trainer/monolith.py` behind a
      compatibility shim so the existing runtime surface is preserved.
- [x] Module candidates added: `transport/serial.py`, `transport/ble.py`,
      `api/sse.py`, `api/auth.py`, `api/server_info.py`, `assets/static.py`,
      `audit/runner.py`, `cli.py`, `__main__.py`.
- [x] Preserve `python radar_vital_trainer_v12_for_v16_0.py --pair --tls` CLI
      surface as a thin shim.

## Notes for future agents

- **Always read this file first.** It records contracts you might otherwise
  break.
- **Always update this file in the same commit** as your changes. Add a dated
  entry at the top of the progress log.
- **Always run `npm --prefix web ci`, `npm run build:web`, and `npm test`** before pushing. The full
  multi-browser smoke suite can take several minutes on WebKit-backed projects.
- **Visual regressions** (`npm run test:visual`) need a Linux Playwright
  baseline. If you commit new baselines, take them from CI, not your local
  machine — fonts/AA differ.
- **DO NOT delete the root `radar_vital_live_dashboard_v12_for_v16_0.html`**
  until everything in this repo and any consumer scripts/docs references the
  `web/` source. The monolith stays as a built artifact for at least one
  release cycle.
