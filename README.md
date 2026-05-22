# Radar Vital — mmWave 60 GHz Trainer & Dashboard

Mobile-first PWA + desktop console for the **Radar Vital** stack: a Seeed XIAO ESP32-C6 carrier driving a Seeed MR60BHA2 60 GHz pulsed-coherent radar that recovers heart rate, respiratory rate, presence, and ML-readiness verdicts.

The repository ships three coupled artefacts:

| Component | File | Role |
|---|---|---|
| **Firmware** | [`radar_vital_v16_0_0.ino`](./radar_vital_v16_0_0.ino) | XIAO ESP32-C6 + MR60BHA2 driver. Emits a 207-column CSV at 115 200 baud over USB. v16 gates an optional NimBLE GATT path behind `ENABLE_BLE` (Phase 4F). |
| **Trainer** | [`radar_vital_trainer_v12_for_v16_0.py`](./radar_vital_trainer_v12_for_v16_0.py) + [`rvt_trainer/`](./rvt_trainer/) | Python 3.11 `ThreadingHTTPServer`. The root script is a compatibility shim over the package entrypoint. It reads the firmware CSV, manages sessions, runs preflight/ML-readiness/audit, writes `live_dashboard.json` once per second, and serves the dashboard plus its REST/SSE API. |
| **Dashboard** | [`radar_vital_live_dashboard_v12_for_v16_0.html`](./radar_vital_live_dashboard_v12_for_v16_0.html) | Single-file PWA: vanilla HTML/CSS/JS, Chart.js, Hammer.js. Polls the trainer at 1 Hz, subscribes to `/api/session/events`, renders five live KPIs + waveforms + alerts + reports. Installable on Chrome desktop and Android Chrome. |

The mobile-first redesign plan that this branch implements is documented in [`AGENTS.md`](./AGENTS.md).

---

## Quick start — operator console (default `127.0.0.1`)

```bash
# 1. Install Python deps
python3 -m pip install -r requirements.txt   # pyserial, pandas, numpy, etc.

# 2. Plug in the XIAO ESP32-C6 over USB-C (or run mock mode without hardware)
python3 radar_vital_trainer_v12_for_v16_0.py serve --mock        # demo data, no serial
python3 radar_vital_trainer_v12_for_v16_0.py serve --port COM10  # live capture (Linux: /dev/ttyACM0)
# Equivalent package entrypoint:
python3 -m rvt_trainer serve --mock

# 3. Open the URL printed in the console
#    http://127.0.0.1:8765/radar_vital_live_dashboard_v12_for_v16_0.html
```

The first launch will register the service worker (`/sw.js`), wire up the manifest (`/manifest.webmanifest`), and prompt a PWA install on Chrome.

---

## LAN access from phones — opt-in, PIN-paired

By default the trainer binds `127.0.0.1` so nothing leaks to the network. Pass `--host 0.0.0.0` to allow LAN clients:

```bash
python3 radar_vital_trainer_v12_for_v16_0.py serve --host 0.0.0.0 --pair
```

`--pair` generates a 6-digit PIN (5 min TTL, single-use), prints a QR encoding `http://<lan-ip>:8765/?pair=<PIN>`, and gates control endpoints behind an `X-RVT-Auth` token issued on PIN exchange.

| Endpoint set | Auth | Examples |
|---|---|---|
| Read-only | None | `/api/health`, `/api/status`, `/api/session/current`, `/api/session/events`, `/api/session/current/live_dashboard.json` |
| Control | `X-RVT-Auth` header required when `--pair` is on | `/api/session/start`, `/api/session/stop`, `/api/defaults` (POST), `/api/preflight/*` |

Tokens live in the trainer's memory only — re-pair after every trainer restart.

For TLS, pass `--tls` (auto-generates a self-signed cert under `.rvt_tls/`, which is git-ignored). HSTS is **not** sent under self-signed certs; pass `--tls-trusted` only when serving a CA-signed cert.

---

## Hosted PWA (GitHub Pages)

[`.github/workflows/pages.yml`](./.github/workflows/pages.yml) publishes the dashboard shell to GitHub Pages on every push to `main`. The hosted PWA points its API base at `S.apiBase` from `localStorage`; first launch shows a pairing screen that accepts a `http(s)://<host>:<port>` URL plus an optional PIN.

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

LAN HTTP traffic in the APK routes through the Capacitor native HTTP stack (via `CapacitorHttp` configuration in `capacitor.config.ts`) so the WebView's mixed-content rules never apply. No `network_security_config.xml` cleartext exception is required.

CI: [`.github/workflows/build-apk.yml`](./.github/workflows/build-apk.yml) produces an unsigned debug APK on every push and attaches it to the workflow artifacts.

### Windows EXE — Tauri v2

```bash
cargo install tauri-cli --version '^2.0'
npm install
npm run build:web
cargo tauri build                     # produces src-tauri/target/release/*.exe
```

Tauri uses Microsoft Edge WebView2. Windows 11 ships it preinstalled; the bundler configures `downloadBootstrapper` so the installer fetches WebView2 at install time on Windows 10. To embed WebView2 in the installer for offline installs, switch to `embedBootstrapper` in [`src-tauri/tauri.conf.json`](./src-tauri/tauri.conf.json).

CI: [`.github/workflows/build-exe.yml`](./.github/workflows/build-exe.yml) cross-compiles the EXE on `windows-latest`.

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
- **API** — `/api/health`, `/api/manifest.webmanifest`, `/api/sw.js`, `/api/server-info`, `/api/auth/exchange` (PIN happy-path + expiry + replay).

CI: [`.github/workflows/playwright.yml`](./.github/workflows/playwright.yml) runs on every push.

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
├── radar_vital_v16_0_0.ino                          # firmware (v16; v15 behavior with optional BLE gated off)
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
