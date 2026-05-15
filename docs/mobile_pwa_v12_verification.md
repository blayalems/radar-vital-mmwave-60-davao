# Radar Vital v12 Mobile PWA Verification

This document tracks the implementation gates for v12 dashboard, v12 trainer, v16 firmware, and packaging.

## Implemented in this branch

- v12 dashboard and v12 trainer cloned from v11, with v11/v15 files preserved.
- v16 firmware cloned from v15, with `ENABLE_BLE=false` by default and optional NimBLE service scaffolding.
- Fixed trainer default port `8765`; explicit `--port 0` keeps auto-pick behavior.
- Dynamic `/manifest.webmanifest`, `/sw.js`, immutable `/icons`, `/lib`, `/fonts`, `/pair`, `/about`, `/api/server-info`, `/api/auth/exchange`, and `/api/events/subscribe`.
- Legacy `/rvt-sw.js` tombstone unregisters the old service worker for one release.
- Header token auth for LAN control endpoints; read endpoints remain unauthenticated.
- Pairing PIN TTL is 5 minutes and single-use.
- CSP is mode-specific and keeps LAN `connect-src` to the literal advertised origin.
- Dashboard registers `/sw.js`, uses `/api/events/subscribe`, keeps EventSource token-free, and bridges legacy API fetches through `api()`.
- DEMO-mode banner is non-dismissable and reinjected into sheets.
- Offline last-known-good store uses `RVT-v12` with separate `live` and `demo` keys.
- Capacitor and Tauri packaging scaffolds are present.

## Hardware-gated acceptance still requiring a device lab

- Android APK install and native HTTP/BLE verification.
- Tauri MSI install on Windows 10/11.
- nRF Connect BLE acceptance tests.
- Pixel 6a performance and Lighthouse numbers on the target network profile.
- TalkBack/VoiceOver pass on row-card tables and sheets.

Record those results in this file before cutting a production release.
