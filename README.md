# Radar Vital — mmWave 60 GHz Trainer & Dashboard

Mobile-first PWA + desktop console for the **Radar Vital** stack: a Seeed XIAO ESP32-C6 carrier driving a Seeed MR60BHA2 60 GHz pulsed-coherent radar that recovers heart rate, respiratory rate, presence, and ML-readiness verdicts.

The repository ships three coupled artefacts:

| Component | File | Role |
|---|---|---|
| **Firmware** | [`radar_vital_v16_5_5.ino`](./radar_vital_v16_5_5.ino) | XIAO ESP32-C6 + MR60BHA2 driver. Emits the v15.2 222-column CSV at 115 200 baud over USB; the first 207 columns remain the frozen v15 contract, 208-219 preserve v15.1 diagnostics, and 220-222 retain the audit fields introduced in v16.4. `ENABLE_BLE` stays off by default; passive module-firmware readback after radar boot/recovery lets session truthfulness record the MR60BHA2 version. |
| **Trainer** | [`radar_vital_trainer_v12_for_v16_0.py`](./radar_vital_trainer_v12_for_v16_0.py) + [`rvt_trainer/`](./rvt_trainer/) | Python 3.11+ `ThreadingHTTPServer`. The root script is a compatibility shim over the package entrypoint. It reads the firmware CSV, manages sessions, runs preflight/ML-readiness/audit, writes `live_dashboard.json`, serves REST/SSE APIs, handles COM7/COM10 serial capture, and captures AiLink BLE reference data through `bleak` when available. |
| **Dashboard** | [`web/src/`](./web/src/) -> [`radar_vital_live_dashboard_v12_for_v16_0.html`](./radar_vital_live_dashboard_v12_for_v16_0.html) | Standalone Angular 21 + Material 3 application compiled to a committed single-file PWA artefact and `www/` packages. Polls or subscribes to `/api/events/subscribe`, renders live KPIs, bounded waveforms/Doppler plots, alerts, reports, pairing, preflight progress, and scoped offline state. |

The mobile-first redesign plan that this branch implements is documented in [`AGENTS.md`](./AGENTS.md).
For a non-developer operator setup guide (EXE/APK/PWA pairing, placement, signal quality, troubleshooting) see [`docs/operator-quickstart.md`](./docs/operator-quickstart.md).
The canonical hardware–firmware–trainer–dashboard feedback loop, shared GBR/1-D CNN experiment path, statistical outputs, and manuscript integration contract live in [`docs/system-feedback-loop.md`](./docs/system-feedback-loop.md).
The ranked successor-PR plan and acceptance gates are tracked in [`docs/v16-5-high-yield-roadmap.md`](./docs/v16-5-high-yield-roadmap.md).

## Quality and change control

The repository uses the documented-information and change-control procedure in
[`quality/qms-policy.md`](./quality/qms-policy.md). It is aligned to relevant
ISO 9001:2015 controls for traceability, design change, verification, release,
monitoring, and corrective action; it does not claim certification. The
machine-readable [document register](./quality/document-register.json) and
[requirements ledger](./quality/requirements.json) bind stable IDs/revisions
to implementation and tests. Run `npm run test:qms-contract -- --base-ref
<PR-base>` before publishing a PR.

Release builds publish `qms-release-record.json` and `SHA256SUMS` alongside the
installers. Those records bind the approved source/workflow, controlled
document register, verification state, artifact byte sizes/hashes, and signing
state. GitHub build-provenance attestations are additional evidence; none of
these records represents medical-device approval or independent conformity
assessment.

---

## Current PR72/PR71 state

- **PR72 session-data audit fixes**: trainer truthfulness now measures the on-disk CSV contract width instead of loader-added columns, accepts both canonical and raw module firmware field names, runs adaptive-correction shadow metrics on suffixed 1 Hz features, runs v15 PQI shadow checks on raw radar rows, and computes BLE reference quality from time-based coverage instead of treating AiLink protocol gaps as decode failures. The BLE logger snapshots `ref_ble_summary.json` during capture so Windows child-process termination does not lose summary metrics.
- **Firmware readback**: `radar_vital_v16_5_5.ino` passively polls the MR60BHA2 module firmware version immediately after `mmWave.begin()` and after radar recovery, so captures can populate `module_fw_*` / `module_fw_valid`.
- **PR71 live-session recovery**: the trainer creates startup/standby `live_dashboard.json` payloads, waits longer for session start, avoids nested dashboard port conflicts, and keeps radar-only sessions when BLE is absent instead of dropping the manifest.
- **PR71 Home/Live UX recovery**: preflight rows persist across refresh/navigation and show progress, advisory hardware/package checks no longer block Start, history infers missing timestamps/durations/subjects from session files, standby `0 bpm` values no longer spam alerts, and Live chart/Doppler containers are bounded to stop vertical scroll growth.

Data-quality note: short captures and weak oximeter contact can still produce conditional/not-ready verdicts. For training/secondary gates, collect at least 10 minutes with stable placement and good finger contact; sessions with low HR coverage dominated by `NO_PHASE`/`PHASE_HOLDOFF` are a DSP/placement issue, not a trainer parsing failure.

---

## Quick start — operator console (default `127.0.0.1`)

```bash
# 1. Install Python deps
python3 -m pip install -r requirements.txt   # pyserial, pandas, numpy, etc.

# 2. Plug in the XIAO ESP32-C6 over USB-C (or run mock mode without hardware)
python3 radar_vital_trainer_v12_for_v16_0.py serve --mock        # trainer-provided mock data, no serial
python3 radar_vital_trainer_v12_for_v16_0.py serve               # operator starts live sessions from the dashboard
# Equivalent package entrypoint:
python3 -m rvt_trainer serve --mock

# 3. Open the URL printed in the console
#    http://127.0.0.1:8765/
```

The first launch will register the service worker (`/sw.js`), wire up the manifest (`/manifest.webmanifest`), and prompt a PWA install on Chrome.

`serve --mock` publishes a sandbox preview, not an active recording. Its
`/api/status` response keeps `active_session` null until an actual capture
starts and exposes mock preview identity separately, so Stop controls and
navigation guards remain truthful. In live mode, a browser offline event marks
telemetry stale immediately; reconnecting restarts transport without changing
the bound participant/session identity.

### Install as a package

The trainer is also pip-installable for environments where a development checkout is not convenient:

```bash
# From PyPI (once published) or directly from the repo:
pip install rvt-trainer                   # runtime deps, including bleak for AiLink BLE reference capture
pip install "rvt-trainer[ble]"            # accepted compatibility extra; also includes bleak

# Editable install from a checkout:
pip install -e .

# The console script is then available:
rvt-trainer serve --mock
rvt-trainer --help
```

Requires Python 3.11+. `bleak>=0.21` is part of the packaged trainer dependency set so the Windows sidecar can capture the configured AiLink oximeter without a manual BLE install.

---

## LAN access from phones — opt-in, PIN-paired

By default the trainer binds `127.0.0.1` so nothing leaks to the network. Opt into PIN-paired LAN mode explicitly:

```bash
python3 radar_vital_trainer_v12_for_v16_0.py serve --bind lan
```

`--bind lan` generates a six-digit PIN (five-minute TTL, single-use), prints the pairing page URL, and supplies a QR link encoding `http://<lan-ip>:8765/?pair=<PIN>`. The public `/api/server-info` route is metadata-only and does not serve a QR image or expose the PIN. The Windows EXE Settings card reads PIN details through the native bridge from loopback-only `/api/native-pairing-info`; phone/APK/PWA clients use the printed QR, `/pair`, or manual PIN entry. The Angular Settings view keeps the issued `X-RVT-Auth` token in session storage only. Five invalid PIN exchanges from one client within a minute trigger a one-minute pairing cooldown; reopen the pairing flow after the cooldown or mint a new PIN if an operator mistyped repeatedly. Protected session APIs also require an operator session token after bootstrap on local and LAN serves.

| Endpoint set | Auth | Routes (owned by `rvt_trainer.api.route_registry`) |
|---|---|---|
| Bootstrap/public | None | shell assets, `/pair`, `/api/health`, `/api/version`, `/api/update/manifest`, `/api/server-info`, `/api/auth/exchange`, `/api/help/schema` |
| EXE native loopback bootstrap | Loopback-only native bridge | `/api/native-pairing-info` (GET; `?format=qr` adds `qr_png_base64` in LAN bind) |
| Auth / operator management | Operator session token (`X-RVT-Auth`) | `/api/auth/validate` (GET), `/api/auth/login` (POST), `/api/auth/logout` (POST), `/api/auth/sse-token` (POST), `/api/operator-profiles` (GET/POST), `/api/subject-profiles` (GET), `/api/participants` (GET/POST), `/api/defaults` (GET/POST) |
| PIN recovery | Recovery code (no session token needed) | `/api/auth/reset-pin` (POST — body: `{operator_id, recovery_code, new_pin}`; verifies PBKDF2 recovery-code hash; rotates code on success; separate 5-attempt/30 s lockout) |
| Host PIN reset | Loopback-only (127.0.0.1 / ::1); no token | `/api/auth/host-reset` (POST — body: `{operator_id, new_pin}`; 403 from any non-loopback address; re-mints recovery code; use for legacy profiles or when recovery code is lost) |
| Physiological / session / hardware | `X-RVT-Auth` operator token after bootstrap | `/api/status`, `/api/events/subscribe`, `/api/session/events`, `/api/session/current`, `/api/session/current/live_dashboard.json`, `/api/session/buffer`, `/api/sessions`, `/api/sessions/<id>/summary`, `/api/sessions/<id>/data`, `/api/sessions/<id>/notes` (GET), `/api/sessions/<id>/signoff` (GET), `/api/sessions/<id>/annotations` (GET), `/api/sessions/<id>/compare`, `/api/sessions/<id>/analyse/status`, `/api/sessions/<id>/training/status`, `/api/sessions/<id>/predict`, `/api/sessions/<id>/files/<rel>`, `/api/ble/scan`, `/api/serial/ports`, `/api/preflight`, `/api/preflight/<id>` (single-check rerun), `/api/trainer/log`, `/api/report/export` |
| Control / mutation | `X-RVT-Auth` operator token after bootstrap | `/api/session/start` (POST), `/api/session/stop` (POST), `/api/session/annotate` (POST), `/api/session/annotations` (POST), `/api/participants/<id>` (PUT — lifecycle status only), `/api/sessions/<id>/notes` (PUT), `/api/sessions/<id>/signoff` (PUT), `/api/sessions/<id>/tags` (PUT), `/api/sessions/<id>/analyse` (POST — rerun; returns `radar_only` status when reference CSV/BLE data is absent), `/api/sessions/<id>` (DELETE — soft-trashes to `.trash/`) |

Backend service ownership is split without changing the public entrypoint:

- `rvt_trainer.session.SessionSupervisor` owns capture-process start, stop,
  reap, session locks, and stop markers.
- `rvt_trainer.api.route_registry` owns route names, methods, groups, and
  authorization policies; the compatibility handler dispatches by those names.
- `rvt_trainer.api.common` owns strict JSON responses, stable API errors,
  atomic JSON persistence, and bounded process waits.
- `rvt_trainer.monolith` retains historical import aliases while downstream
  callers migrate; `python -m rvt_trainer` and the root trainer script remain
  equivalent packaged entrypoints.

Tokens live in the trainer's memory only — re-pair after every trainer restart.

Study sessions use a pseudonymous participant profile that is separate from the
optional physiology subject profile. A real confirmatory start must bind one
participant code to one immutable trial assignment: distance 0.6, 0.8, or 1.0 m;
barrier `none` or `cardboard`; trial 1–3; and planned duration 150 seconds.
Exploratory starts may use 0.5–1.0 m but remain explicitly labelled
`exploratory`, so they cannot enter confirmatory statistics accidentally.
Participant IDs and release/firmware/protocol provenance are persisted with the
session and must not be reassigned after capture starts.

Before a real Start, the dashboard sends a compatibility handshake containing
its product/dashboard version, serial protocol and width, and required API
schemas. The trainer compares that request with its own release identity and
known firmware truthfulness. Known mismatches return a stable conflict response
with reload/restart/flash guidance; older clients without the handshake remain
operational but are recorded as unverified. If a capture process fails after a
session directory is allocated, its manifest remains as a failed-start record
with the participant/trial, release/model provenance, failure reason, and
timestamps instead of becoming an anonymous placeholder.

An explicit `/api/session/stop` first lets the detached capture child flush and
exit, escalates through bounded terminate/kill waits if needed, and only then
may enqueue eligible paired analysis. Stopping the control server uses the same
bounded reap but never starts analysis; already-captured files are preserved.
If the child cannot be reaped, its current-session markers remain intact and
the stop reports a failure instead of advertising an idle trainer.

Every current dashboard Start sends an `idempotency_key` (the equivalent
`Idempotency-Key` header is also accepted). The trainer reserves that key before
preflight and stores a hash of the normalized effective request. Concurrent or
response-loss retries with the same key and request replay the original result
without allocating another participant/trial directory; reusing the key with a
different request returns `409 IDEMPOTENCY_KEY_REUSED`. Definitive failed starts
are replayed with their original failure/session evidence, while legacy clients
without a key retain the pre-v16.5.4 behavior for one compatibility release.

For TLS, pass `--tls` (auto-generates a self-signed cert under `.rvt_tls/`, which is git-ignored). HSTS is **not** sent under self-signed certs; pass `--tls-trusted` only when serving a CA-signed cert.

---

## Hosted PWA (GitHub Pages)

[`.github/workflows/pages.yml`](./.github/workflows/pages.yml) publishes the self-contained Angular dashboard shell to GitHub Pages on every push to `main`. Its relative manifest and service-worker paths remain within the repository Pages scope. Settings accepts a trainer origin and one-time PIN for LAN pairing.

The hosted shell **cannot read serial** — it's a thin client. The trainer still has to run on the operator's machine.

---

## Native packages

### Android APK — Capacitor

```bash
npm install                           # installs @capacitor/cli, @capacitor/android
npm run build:web                     # copies dashboard + assets/ into www/
npx cap sync android
npx cap open android                  # opens Android Studio for signing/release
```

LAN HTTP traffic in the APK routes through the Capacitor native HTTP stack (via `CapacitorHttp` configuration in `capacitor.config.ts`) so the WebView's mixed-content rules never apply. Telemetry-derived offline records are segregated by `demo` versus `live` IndexedDB scope. Where local Bluetooth is available, Home exposes a **Native BLE acceptance probe** that validates one allowlisted AiLink notification; it is a hardware qualification check and does not replace trainer-side reference capture for a recorded session. Preflight BLE/device warnings are advisory for Start; collection can proceed as radar-only and the trainer records that status if the oximeter is unavailable.

CI: [`.github/workflows/build-apk.yml`](./.github/workflows/build-apk.yml) produces an unsigned debug APK for validation. After each accepted push to `main`, [`.github/workflows/release-artifacts.yml`](./.github/workflows/release-artifacts.yml) publishes a versioned GitHub prerelease with an APK asset and generated changelog; every release stamps its semantic version and increasing Android version code into the APK, using signing secrets when configured.

### Windows EXE — Tauri v2

```bash
cargo install tauri-cli --version '^2.0'
npm install
npm run build:web
cargo tauri build                     # produces src-tauri/target/release/*.exe
```

Tauri uses Microsoft Edge WebView2 and keeps WebView network policy at `connect-src 'self'`. Paired LAN API/download calls run through native Rust commands pinned to the explicitly paired origin. Native BLE reference commands allowlist the configured AiLink oximeter notify profile (`FFE0` service / `FFE2` characteristic); Home's bounded Native BLE acceptance probe consumes that command path and reports whether a notification was received without claiming it supplied session telemetry. The EXE does not rely on Chromium Web Bluetooth prompts for local-device discovery; BLE capture is handled by the bundled Python/WinRT sidecar path. The separate radar-firmware GATT path remains disabled by default pending physical acceptance. Windows 11 ships WebView2 preinstalled; the installer uses `downloadBootstrapper` for other systems.

CI: [`.github/workflows/build-exe.yml`](./.github/workflows/build-exe.yml) builds the EXE on `windows-latest`. After each accepted push to `main`, [`.github/workflows/release-artifacts.yml`](./.github/workflows/release-artifacts.yml) attaches the NSIS installer to the versioned GitHub prerelease and generated changelog; every release stamps the same semantic version into the EXE and signs it when certificate secrets are configured.

---

## Model training

The Python trainer keeps `gradient_boosting` as its default correction model and
offers an explicit, optional `cnn_1d` research path with causal
session-bounded windows. The CNN requires TensorFlow and enforces a
500-valid-window floor per target unless a clearly labelled experimental
override is supplied.

See [`docs/model-family-guide.md`](./docs/model-family-guide.md) for commands,
data-readiness requirements, artifact behavior, and thesis-claim boundaries.

---

## Testing

```bash
npm install
npx playwright install --with-deps chromium
npm test                              # runs tests/smoke/*.spec.ts
npm run test:unit:web                 # Angular unit/source-integrity/contrast checks
python -m pytest -q tests/test_session_data_regressions.py tests/test_trainer_audit.py tests/test_trainer_verdict.py
```

Playwright covers:
- **Smoke** — dashboard loads, no uncaught script errors, core API routes answer, and PWA install criteria pass.
- **Visual** — screenshot regressions across mobile/tablet/desktop viewports and the light/dark/night/hc themes via `npm run test:visual`.
- **API** — `/api/health`, `/manifest.webmanifest`, `/sw.js`, `/api/server-info`, `/api/auth/exchange`, LAN authorization, static-key denial, notes/sign-off, serial discovery, preflight, and session-analysis contracts.
- **Session-data regressions** — PR72 pins on-disk contract width, module firmware truthfulness, adaptive-correction shadow metrics, v15 PQI shadow, and BLE reference-quality calculations against recorded session failures.

CI: [`.github/workflows/playwright.yml`](./.github/workflows/playwright.yml) runs Python contracts, smoke/API coverage across desktop, Pixel 7, iPhone 14 and iPad projects, plus visual regression.

---

## Branches & history

| Branch | Purpose |
|---|---|
| `main` | Operator-stable mobile-first v12 dashboard / v12 trainer / v16 firmware path. |
| `archive/legacy-v8-to-v11` | Frozen snapshot of every legacy `.ino` / `.py` / `.html` (v8.8 → v11). Recover historical baselines from this branch only. |

Pre-mobile baseline tag: `v15.0.0-pre-mobile` — rollback point for the redesign work.

---

## Repository layout

```
.
├── radar_vital_v16_5_5.ino                          # firmware (v16.5.5; v15.2 222-column USB contract, BLE gated off)
├── radar_vital_trainer_v12_for_v16_0.py             # trainer compatibility shim
├── rvt_trainer/                                     # trainer package facade + legacy monolith
├── radar_vital_live_dashboard_v12_for_v16_0.html    # PWA dashboard (single file)
├── assets/
│   ├── sw.js                                        # service worker (registered at /sw.js)
│   ├── manifest.webmanifest                         # static fallback (trainer overrides at /manifest.webmanifest)
│   ├── icons/                                       # PWA icons (192, 512, 512-maskable, apple-touch)
│   ├── lib/                                         # self-hosted Chart.js, Hammer.js, jsqr (Phase 1a)
│   └── fonts/                                       # self-hosted Inter, JetBrains Mono, Material Symbols (Phase 1a)
├── android/                                         # Capacitor Android project (generated)
├── src-tauri/                                       # Tauri Windows wrapper
├── tools/extract-icons.py                           # Material Symbols codepoint subsetter
├── tests/
│   ├── smoke/                                       # Playwright smoke + API tests
│   └── visual/                                      # Playwright screenshot regressions
├── .github/workflows/                               # CI: pages.yml, build-apk.yml, build-exe.yml, playwright.yml
├── .rvt_tls/                                        # git-ignored self-signed cert material
├── AGENTS.md                                        # AI agent contract for this repo
└── README.md
```

---

## License

Same license as upstream — see repository settings.
