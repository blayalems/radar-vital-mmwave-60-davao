# Angular Material v12/v16 Migration Remediation Audit

Date: 2026-05-24
Branch: `codex/mobile-first-dashboard-upABy`
Baseline: v11 operator workflows preserved through Angular Material 3 source in `web/src/`.

## Disposition

| Severity | Finding | Disposition / Acceptance Evidence |
|---|---|---|
| Critical | Repository-root static fallback could expose `.rvt_tls/` keys and source files. | Fixed: trainer returns 404 outside an explicit public shell/asset route allowlist. Covered by `tests/test_trainer_security_api.py`. |
| Critical | Demo snapshots and notes used unscoped synchronous browser storage and automatic sandbox was unlabeled. | Fixed: scoped IndexedDB stores use `demo`/`live`, legacy records are quarantined as `legacy-unclassified`, and automatic fallback sets the DEMO state/banner. |
| High | LAN mode protected writes but exposed physiological reads, BLE scan, reports, and SSE. | Fixed: only health/version/server-info/help bootstrap reads remain public; profile, session, device and physiological reads require the paired token in LAN mode. |
| High | Angular had no PIN exchange workflow. | Fixed: Settings accepts a six-digit PIN, QR `?pair=` links are consumed once, issued tokens remain session-only, and the PIN is removed from browser history. |
| High | Tauri EXE lacked a secure LAN bridge and BLE implementation surface. | Fixed in code: native HTTP/download traffic is pinned to the paired origin and native BLE scan/connect/notification commands are exposed; real-device GATT acceptance remains a release gate. |
| High | Report review summaries and operator sign-off were not persisted. | Fixed: protected historical notes/sign-off routes, Material sign-off UI, validation bounds, and explicit unsigned export warnings. |
| High | Packaged/Pages PWA output did not reliably precache a bootable shell. | Fixed: build emits one self-contained entry document for root and `www/`, relative manifest/scope URLs work under repository Pages paths, and SW installation fails visibly if required assets are missing. |
| High | CI did not exercise stated viewport/visual/static contracts. | Fixed: CI runs complete Python tests, all Playwright viewport projects, and visual regressions. Native build workflows retain artifact gates. |
| Medium | HC left bright unowned surfaces/plot backgrounds in an inverse theme. | Fixed: HC establishes all surface tokens, canvas/nav/safe-area backgrounds, and dark color-scheme ownership. Phone and desktop HC captures were visually inspected before accepting updated baselines. |
| Medium | Serial scan, mobile command entry, alert review controls and polling resilience were incomplete. | Fixed: typed serial route, touch command action, alert filter/snooze/wave jump flow, and bounded jittered HTTP polling retry. |

## Acceptance Mapping

| Area | Implementation evidence | Verification evidence |
|---|---|---|
| Static/TLS denial and LAN read authorization | [`rvt_trainer/monolith.py`](../rvt_trainer/monolith.py) | [`tests/test_trainer_security_api.py`](../tests/test_trainer_security_api.py) |
| PIN exchange and scoped telemetry persistence | [`web/src/app/services/api.service.ts`](../web/src/app/services/api.service.ts), [`web/src/app/services/persistence.service.ts`](../web/src/app/services/persistence.service.ts), [`web/src/app/services/state.service.ts`](../web/src/app/services/state.service.ts) | [`tests/smoke/dashboard.spec.ts`](../tests/smoke/dashboard.spec.ts), [`tests/test_v12_static_contract.py`](../tests/test_v12_static_contract.py) |
| Notes, sign-off and serial discovery | [`rvt_trainer/monolith.py`](../rvt_trainer/monolith.py), [`web/src/app/components/report/report.component.ts`](../web/src/app/components/report/report.component.ts), [`web/src/app/components/home/home.component.ts`](../web/src/app/components/home/home.component.ts) | [`tests/test_trainer_security_api.py`](../tests/test_trainer_security_api.py), [`tests/smoke/dashboard.spec.ts`](../tests/smoke/dashboard.spec.ts) |
| Mobile workflow and HC visual parity | [`web/src/app/components/topbar/topbar.component.ts`](../web/src/app/components/topbar/topbar.component.ts), [`web/src/app/components/alerts-dialog/alerts-dialog.component.ts`](../web/src/app/components/alerts-dialog/alerts-dialog.component.ts), [`web/src/styles.scss`](../web/src/styles.scss) | [`tests/visual/rvt-v12.spec.ts`](../tests/visual/rvt-v12.spec.ts) and its committed screenshots |
| Offline shell and Pages scope | [`assets/sw.js`](../assets/sw.js), [`scripts/build-angular.mjs`](../scripts/build-angular.mjs), [`.github/workflows/pages.yml`](../.github/workflows/pages.yml) | [`tests/smoke/dashboard.spec.ts`](../tests/smoke/dashboard.spec.ts), local Pages static output checks |
| Native paired transport and BLE gate | [`src-tauri/src/main.rs`](../src-tauri/src/main.rs), [`web/src/app/services/bluetooth.service.ts`](../web/src/app/services/bluetooth.service.ts) | `cargo check`, local NSIS/APK packaging; physical GATT acceptance remains open |
| Frozen firmware protocol | [`radar_vital_v16_0_0.ino`](../radar_vital_v16_0_0.ino), [`rvt_trainer/monolith.py`](../rvt_trainer/monolith.py) | [`tests/test_v12_static_contract.py`](../tests/test_v12_static_contract.py), Arduino CLI compile |

## Public Contracts Added Or Tightened

- `GET /api/serial/ports` returns `{ ports: [{ device, label }], selected }` and is protected in LAN mode.
- `GET|PUT /api/sessions/<id>/notes` persists `review_summary` at a maximum of 4000 characters without replacing timestamped observations.
- `GET|PUT /api/sessions/<id>/signoff` persists validated operator name, `^[A-Z]{2,5}$` initials, up to 500 validation-comment characters, and server-issued `signed_at`.
- Native Tauri requests may bootstrap only `/api/server-info` and `/api/auth/exchange` before pairing; subsequent `/api/*` access and downloads must match the recorded paired origin.
- Firmware serial output remains frozen at 207 columns and 115200 baud; firmware BLE remains default-off until physical GATT acceptance passes.

## Remaining Release Gates

- Run APK and EXE paired-LAN 1 Hz telemetry validation against a real trainer.
- Run Capacitor and Tauri GATT acceptance with the target BLE device before considering firmware BLE promotion.
- Confirm APK/EXE/Pages workflows in CI and reduce the advisory Angular initial bundle warning (currently 12.79 kB over the 2 MB warning budget).

## Verification Evidence

- `npm run build:check` passed; `web/` and the committed monolith round-trip cleanly.
- `python -m pytest -q tests` passed with 34 tests, including LAN authorization/static denial and frozen 207-column/115200 protocol assertions.
- Playwright smoke passed across desktop, Pixel 7, iPhone 14 and iPad at 29 checks each (116 total); Chromium verifies offline launch, while WebKit verifies precaching because Playwright offline navigation emulation reports an internal engine error.
- `npx playwright test tests/visual/rvt-v12.spec.ts` passed all 80 current route/theme/device baselines after intentional HC/settings updates.
- Pages static output checks, `cargo check --manifest-path src-tauri\Cargo.toml`, Arduino CLI compile for `esp32:esp32:XIAO_ESP32C6`, Android `assembleDebug`, and Tauri NSIS build passed locally.
