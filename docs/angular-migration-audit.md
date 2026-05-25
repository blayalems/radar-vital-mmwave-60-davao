# Angular Material v12/v16 Migration Remediation Audit

Date: 2026-05-24
Updated: 2026-05-25
Branch: `codex/mobile-first-dashboard-upABy`
Baseline: v11 operator workflows preserved through Angular Material 3 source in `web/src/`.

## Disposition

| Severity | Finding | Disposition / Acceptance Evidence |
|---|---|---|
| Critical | Repository-root static fallback could expose `.rvt_tls/` keys and source files. | Fixed: trainer returns 404 outside an explicit public shell/asset route allowlist. Covered by `tests/test_trainer_security_api.py`. |
| Critical | Demo snapshots and notes used unscoped synchronous browser storage and automatic sandbox was unlabeled. | Fixed: scoped IndexedDB stores use `demo`/`live`, legacy records are quarantined as `legacy-unclassified`, and automatic fallback sets the DEMO state/banner. |
| High | LAN mode protected writes but exposed physiological reads, BLE scan, reports, and SSE. | Fixed: only health/version/server-info/help bootstrap reads remain public; profile, session, device and physiological reads require the paired token in LAN mode. |
| High | Angular had no PIN exchange workflow. | Fixed: Settings accepts a six-digit PIN, QR `?pair=` links are consumed once, issued tokens remain session-only, and the PIN is removed from browser history. |
| High | Tauri EXE lacked a secure LAN bridge and BLE implementation surface. | Fixed in code: native HTTP/download traffic is pinned to the paired origin; native BLE scan/connect/notification commands allowlist the configured AiLink reference GATT profile and bind notify/disconnect operations to the validated active device. Real-device GATT acceptance remains a release gate. |
| High | Report review summaries and operator sign-off were not persisted. | Fixed: protected historical notes/sign-off routes, Material sign-off UI, validation bounds, and explicit unsigned export warnings. |
| High | Packaged/Pages PWA output did not reliably precache a bootable shell or preserve direct Angular route loads. | Fixed: build emits one self-contained entry document for root, `www/index.html` and `www/404.html`; relative manifest/scope URLs work under repository Pages paths; SW installation fails visibly if required assets are missing; Pages CI rejects non-Angular/README artifacts. The public Pages root was verified to serve Angular HTML on 2026-05-25, while deployment of the current worker remains pending. |
| High | CI did not exercise stated viewport/visual/static contracts, initially targeted Win32 visual baselines from Ubuntu, and installed Chromium alone despite configured WebKit mobile projects. | Fixed: Ubuntu CI installs Chromium and WebKit for complete functional/API viewport checks; reviewed `-win32.png` visual baselines run on `windows-latest` with both required engines installed. A follow-up makes Home captures deterministic by fixing preflight/stream inputs and excluding only continuously repainted preview canvases, while Live retains plot coverage. Native build workflows retain artifact gates. |
| Medium | HC left bright unowned surfaces/plot backgrounds in an inverse theme, with legacy Home aliases still painting white KPI/panel cards. | Fixed: the final HC layer resets legacy Home aliases and explicitly owns Angular Home cards, scope/preflight/history panels and shell tokens as black surfaces with white ink/outlines. Browser-computed styles and reviewed phone/desktop captures confirm the inverse result. |
| Medium | Serial scan, mobile command entry, alert review controls and polling resilience were incomplete. | Fixed: typed serial route, touch command action, alert filter/snooze/wave jump flow, and bounded jittered HTTP polling retry. |

## Acceptance Mapping

| Area | Implementation evidence | Verification evidence |
|---|---|---|
| Static/TLS denial and LAN read authorization | [`rvt_trainer/monolith.py`](../rvt_trainer/monolith.py) | [`tests/test_trainer_security_api.py`](../tests/test_trainer_security_api.py) |
| PIN exchange and scoped telemetry persistence | [`web/src/app/services/api.service.ts`](../web/src/app/services/api.service.ts), [`web/src/app/services/persistence.service.ts`](../web/src/app/services/persistence.service.ts), [`web/src/app/services/state.service.ts`](../web/src/app/services/state.service.ts) | [`tests/smoke/dashboard.spec.ts`](../tests/smoke/dashboard.spec.ts), [`tests/test_v12_static_contract.py`](../tests/test_v12_static_contract.py) |
| Notes, sign-off and serial discovery | [`rvt_trainer/monolith.py`](../rvt_trainer/monolith.py), [`web/src/app/components/report/report.component.ts`](../web/src/app/components/report/report.component.ts), [`web/src/app/components/home/home.component.ts`](../web/src/app/components/home/home.component.ts) | [`tests/test_trainer_security_api.py`](../tests/test_trainer_security_api.py), [`tests/smoke/dashboard.spec.ts`](../tests/smoke/dashboard.spec.ts) |
| Mobile workflow and HC visual parity | [`web/src/app/components/topbar/topbar.component.ts`](../web/src/app/components/topbar/topbar.component.ts), [`web/src/app/components/alerts-dialog/alerts-dialog.component.ts`](../web/src/app/components/alerts-dialog/alerts-dialog.component.ts), [`web/src/styles.scss`](../web/src/styles.scss) | [`tests/visual/rvt-v12.spec.ts`](../tests/visual/rvt-v12.spec.ts) and its committed screenshots |
| Offline shell and Pages scope | [`assets/sw.js`](../assets/sw.js), [`scripts/build-angular.mjs`](../scripts/build-angular.mjs), [`.github/workflows/pages.yml`](../.github/workflows/pages.yml) | [`tests/smoke/dashboard.spec.ts`](../tests/smoke/dashboard.spec.ts), [`tests/test_v12_static_contract.py`](../tests/test_v12_static_contract.py), generated Pages shell-identity checks |
| Native paired transport and BLE gate | [`src-tauri/src/main.rs`](../src-tauri/src/main.rs), [`web/src/app/services/bluetooth.service.ts`](../web/src/app/services/bluetooth.service.ts) | Rust GATT-allowlist unit test, `cargo check`, local NSIS/APK packaging; physical GATT acceptance remains open |
| Frozen firmware protocol | [`radar_vital_v16_0_0.ino`](../radar_vital_v16_0_0.ino), [`rvt_trainer/monolith.py`](../rvt_trainer/monolith.py) | [`tests/test_v12_static_contract.py`](../tests/test_v12_static_contract.py), Arduino CLI compile |

## Public Contracts Added Or Tightened

- `GET /api/serial/ports` returns `{ ports: [{ device, label }], selected }` and is protected in LAN mode.
- `GET|PUT /api/sessions/<id>/notes` persists `review_summary` at a maximum of 4000 characters without replacing timestamped observations.
- `GET|PUT /api/sessions/<id>/signoff` persists validated operator name, `^[A-Z]{2,5}$` initials, up to 500 validation-comment characters, and server-issued `signed_at`.
- Native Tauri requests may bootstrap only `/api/server-info` and `/api/auth/exchange` before pairing; subsequent `/api/*` access and downloads must match the recorded paired origin.
- Native Tauri BLE reference operations allow only the configured AiLink `FFE0`/`FFE2` notification profile and require a successfully validated active device; optional radar-firmware GATT promotion remains gated separately.
- Firmware serial output remains frozen at 207 columns and 115200 baud; firmware BLE remains default-off until physical GATT acceptance passes.

## Remaining Release Gates

- Run APK and EXE paired-LAN 1 Hz telemetry validation against a real trainer.
- Run Capacitor and Tauri GATT acceptance with the target BLE device before considering firmware BLE promotion.
- Deploy the current Pages workflow output so the public site advances from its verified Angular shell with the older `rvt-shell-v12.0.0` worker to the remediation worker and direct-route fallback.
- Confirm current-head APK/EXE workflow runs in CI after the GATT allowlist correction and reduce the advisory Angular initial bundle warning (currently 14.73 kB over the 2 MB warning budget).

## Verification Evidence

- `npm run build:check` passed; `web/` and the committed monolith round-trip cleanly.
- `python -m pytest -q tests` passed with 38 tests, including LAN authorization/static denial, Pages Angular-shell publication, CI snapshot-platform alignment, native BLE GATT allowlist ownership, dark inverse HC Home ownership, and frozen 207-column/115200 protocol assertions.
- `cargo test --manifest-path src-tauri/Cargo.toml --verbose` passed the executable native allowlist test, rejecting unapproved BLE notification UUID combinations.
- `npx playwright test tests/smoke/dashboard.spec.ts tests/smoke/api.spec.ts --reporter=line` passed 116 current checks across desktop, Pixel 7, iPhone 14 and iPad; Chromium verifies offline launch, while WebKit verifies precaching because Playwright offline navigation emulation reports an internal engine error.
- `npx playwright test tests/visual/rvt-v12.spec.ts --grep " home$" --reporter=line` passed the 16 affected Home visual comparisons for the current increment.
- `npx playwright test tests/visual/rvt-v12.spec.ts --grep " home$" --repeat-each=2 --reporter=line` passed 32 repeated Home comparisons after stabilizing its streamed/preflight fixture; `npx playwright test tests/visual/rvt-v12.spec.ts --reporter=line` passed all 80 current route/theme/device baselines after intentional inverse-HC updates. All 84 committed snapshot files are Win32-targeted and CI visual comparison runs on `windows-latest`.
- Generated Pages output checks confirmed identical Angular shells in `www/index.html` and `www/404.html`, no README rendering marker, and the `rvt-shell-v12.0.2` service worker. A direct public probe on 2026-05-25 confirmed the root returns Angular HTML rather than README markup, while its still-deployed `rvt-shell-v12.0.0` worker and `404` response for `/settings` prove the current direct-route fallback has not yet been deployed. `cargo check --manifest-path src-tauri\Cargo.toml`, Arduino CLI compile for `esp32:esp32:XIAO_ESP32C6`, Android `assembleDebug`, and Tauri NSIS build passed locally in the earlier remediation increment.
