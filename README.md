# Radar Vital — mmWave 60 GHz Trainer & Dashboard

Mobile-first PWA + desktop console for the **Radar Vital** stack: a Seeed XIAO ESP32-C6 carrier driving a Seeed MR60BHA2 60 GHz pulsed-coherent radar that recovers heart rate, respiratory rate, presence, and ML-readiness verdicts.

The repository ships three coupled artefacts:

| Component | File | Role |
|---|---|---|
| **Firmware** | [`radar_vital_v16_2_0.ino`](./radar_vital_v16_2_0.ino) | XIAO ESP32-C6 + MR60BHA2 driver. Emits the v15.1 219-column CSV at 115 200 baud over USB; the first 207 columns remain the frozen v15 contract and columns 208-219 are right-edge diagnostics. v16 gates an optional NimBLE GATT path behind `ENABLE_BLE` (Phase 4F). |
| **Trainer** | [`radar_vital_trainer_v12_for_v16_0.py`](./radar_vital_trainer_v12_for_v16_0.py) + [`rvt_trainer/`](./rvt_trainer/) | Python 3.11 `ThreadingHTTPServer`. The root script is a compatibility shim over the package entrypoint. It reads the firmware CSV, manages sessions, runs preflight/ML-readiness/audit, writes `live_dashboard.json` once per second, and serves the dashboard plus its REST/SSE API. |
| **Dashboard** | [`web/src/`](./web/src/) -> [`radar_vital_live_dashboard_v12_for_v16_0.html`](./radar_vital_live_dashboard_v12_for_v16_0.html) | Standalone Angular 21 + Material 3 application compiled to a committed single-file PWA artefact and `www/` packages. Polls or subscribes to `/api/events/subscribe`, renders live KPIs, waveforms, alerts, reports, pairing and scoped offline state. |

The mobile-first redesign plan that this branch implements is documented in [`AGENTS.md`](./AGENTS.md).
For a non-developer operator setup guide (EXE/APK/PWA pairing, placement, signal quality, troubleshooting) see [`docs/operator-quickstart.md`](./docs/operator-quickstart.md).

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
#    http://127.0.0.1:8765/radar_vital_live_dashboard_v12_for_v16_0.html
```

The first launch will register the service worker (`/sw.js`), wire up the manifest (`/manifest.webmanifest`), and prompt a PWA install on Chrome.

### Install as a package

The trainer is also pip-installable for environments where a development checkout is not convenient:

```bash
# From PyPI (once published) or directly from the repo:
pip install rvt-trainer                   # runtime deps only
pip install "rvt-trainer[ble]"            # include bleak for BLE transport

# Editable install from a checkout:
pip install -e .

# The console script is then available:
rvt-trainer serve --mock
rvt-trainer --help
```

Requires Python 3.11+. The optional `ble` extra (`bleak>=0.21`) is only needed when using `--transport ble`.

---

## LAN access from phones — opt-in, PIN-paired

By default the trainer binds `127.0.0.1` so nothing leaks to the network. Opt into PIN-paired LAN mode explicitly:

```bash
python3 radar_vital_trainer_v12_for_v16_0.py serve --bind lan
```

`--bind lan` generates a six-digit PIN (five-minute TTL, single-use), prints the pairing page URL, and supplies a QR link encoding `http://<lan-ip>:8765/?pair=<PIN>`. The public `/api/server-info` route is metadata-only and does not serve a QR image or expose the PIN. The Windows EXE Settings card reads PIN details through the native bridge from loopback-only `/api/native-pairing-info`; phone/APK/PWA clients use the printed QR, `/pair`, or manual PIN entry. The Angular Settings view keeps the issued `X-RVT-Auth` token in session storage only. Five invalid PIN exchanges from one client within a minute trigger a one-minute pairing cooldown; reopen the pairing flow after the cooldown or mint a new PIN if an operator mistyped repeatedly.

| Endpoint set | Auth | Routes (verified against `rvt_trainer/monolith.py`) |
|---|---|---|
| Bootstrap/public | None | shell assets, `/pair`, `/api/health`, `/api/version`, `/api/update/manifest`, `/api/server-info`, `/api/auth/exchange`, `/api/help/schema` |
| EXE native loopback bootstrap | Loopback-only native bridge | `/api/native-pairing-info` (GET; `?format=qr` adds `qr_png_base64` in LAN bind) |
| Auth / operator management | Operator session token (`X-RVT-Auth`) | `/api/auth/validate` (GET), `/api/auth/login` (POST), `/api/auth/logout` (POST), `/api/auth/sse-token` (POST), `/api/operator-profiles` (GET/POST), `/api/subject-profiles` (GET), `/api/defaults` (GET/POST) |
| PIN recovery | Recovery code (no session token needed) | `/api/auth/reset-pin` (POST — body: `{operator_id, recovery_code, new_pin}`; verifies PBKDF2 recovery-code hash; rotates code on success; separate 5-attempt/30 s lockout) |
| Host PIN reset | Loopback-only (127.0.0.1 / ::1); no token | `/api/auth/host-reset` (POST — body: `{operator_id, new_pin}`; 403 from any non-loopback address; re-mints recovery code; use for legacy profiles or when recovery code is lost) |
| Physiological / session / hardware | `X-RVT-Auth` required in LAN mode | `/api/status`, `/api/events/subscribe`, `/api/session/events`, `/api/session/current`, `/api/session/current/live_dashboard.json`, `/api/session/buffer`, `/api/sessions`, `/api/sessions/<id>/summary`, `/api/sessions/<id>/data`, `/api/sessions/<id>/notes` (GET), `/api/sessions/<id>/signoff` (GET), `/api/sessions/<id>/annotations` (GET), `/api/sessions/<id>/compare`, `/api/sessions/<id>/analyse/status`, `/api/sessions/<id>/training/status`, `/api/sessions/<id>/predict`, `/api/sessions/<id>/files/<rel>`, `/api/ble/scan`, `/api/serial/ports`, `/api/preflight`, `/api/preflight/<id>` (single-check rerun), `/api/trainer/log`, `/api/report/export` |
| Control / mutation | `X-RVT-Auth` required in LAN mode | `/api/session/start` (POST), `/api/session/stop` (POST), `/api/session/annotate` (POST), `/api/session/annotations` (POST), `/api/sessions/<id>/notes` (PUT), `/api/sessions/<id>/signoff` (PUT), `/api/sessions/<id>/tags` (PUT), `/api/sessions/<id>/analyse` (POST — rerun), `/api/sessions/<id>` (DELETE — soft-trashes to `.trash/`) |

Tokens live in the trainer's memory only — re-pair after every trainer restart.

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

LAN HTTP traffic in the APK routes through the Capacitor native HTTP stack (via `CapacitorHttp` configuration in `capacitor.config.ts`) so the WebView's mixed-content rules never apply. Telemetry-derived offline records are segregated by `demo` versus `live` IndexedDB scope. Where local Bluetooth is available, Home exposes a **Native BLE acceptance probe** that validates one allowlisted AiLink notification; it is a hardware qualification check and does not replace trainer-side reference capture for a recorded session.

CI: [`.github/workflows/build-apk.yml`](./.github/workflows/build-apk.yml) produces an unsigned debug APK for validation. After each accepted push to `main`, [`.github/workflows/release-artifacts.yml`](./.github/workflows/release-artifacts.yml) publishes a versioned GitHub prerelease with an APK asset and generated changelog; every release stamps its semantic version and increasing Android version code into the APK, using signing secrets when configured.

### Windows EXE — Tauri v2

```bash
cargo install tauri-cli --version '^2.0'
npm install
npm run build:web
cargo tauri build                     # produces src-tauri/target/release/*.exe
```

Tauri uses Microsoft Edge WebView2 and keeps WebView network policy at `connect-src 'self'`. Paired LAN API/download calls run through native Rust commands pinned to the explicitly paired origin. Native BLE reference commands allowlist the configured AiLink oximeter notify profile (`FFE0` service / `FFE2` characteristic); Home's bounded Native BLE acceptance probe consumes that command path and reports whether a notification was received without claiming it supplied session telemetry. The separate radar-firmware GATT path remains disabled by default pending physical acceptance. Windows 11 ships WebView2 preinstalled; the installer uses `downloadBootstrapper` for other systems.

CI: [`.github/workflows/build-exe.yml`](./.github/workflows/build-exe.yml) builds the EXE on `windows-latest`. After each accepted push to `main`, [`.github/workflows/release-artifacts.yml`](./.github/workflows/release-artifacts.yml) attaches the NSIS installer to the versioned GitHub prerelease and generated changelog; every release stamps the same semantic version into the EXE and signs it when certificate secrets are configured.

---

## Testing

```bash
npm install
npx playwright install --with-deps chromium
npm test                              # runs tests/smoke/*.spec.ts
```

Playwright covers:
- **Smoke** — dashboard loads, no uncaught script errors, core API routes answer, and PWA install criteria pass.
- **Visual** — screenshot regressions across mobile/tablet/desktop viewports and the light/dark/night/hc themes via `npm run test:visual`.
- **API** — `/api/health`, `/manifest.webmanifest`, `/sw.js`, `/api/server-info`, `/api/auth/exchange`, LAN authorization, static-key denial, notes/sign-off and serial discovery contracts.

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
├── radar_vital_v16_2_0.ino                          # firmware (v16; v15 behavior with optional BLE gated off)
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
