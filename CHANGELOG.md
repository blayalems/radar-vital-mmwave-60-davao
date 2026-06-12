# Changelog

All notable changes to Radar Vital are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
product's semantic `MAJOR.MINOR.PATCH` scheme.

## [Unreleased]

### Added
- Terms & Conditions, Privacy Notice (RA 10173-framed drafts), LICENSE,
  CONTRIBUTING guide, issue templates, and wiki source pages.
- PIN recovery codes: each operator profile now receives a one-time XXXX-XXXX-XXXX
  recovery code at creation (only the PBKDF2 hash is stored). `POST /api/auth/reset-pin`
  accepts `{operator_id, recovery_code, new_pin}`; on success the old code is consumed
  and a fresh code is returned (single-use rotation). A separate 5-attempt/30 s lockout
  tracks recovery attempts independently of the PIN lockout. Legacy profiles without a
  recovery code receive a clear error directing to host-reset.
- `POST /api/auth/host-reset`: loopback-only (`127.0.0.1`/`::1`) PIN reset; returns a
  new recovery code; 403 from any other IP even with a valid session token. Useful for
  local EXE operators who have lost their recovery code.
- Recovery-code dialog (`RecoveryCodeDialogComponent`): shown after profile creation and
  after any reset; large monospace display, one-click copy (clipboard API + fallback),
  "I saved my recovery code" required to dismiss (`disableClose: true`).
- "Forgot PIN?" link on the lock overlay → inline 3-step recovery flow (operator picker →
  recovery code → new PIN). EXE/Tauri hosts also show "Reset from this computer" →
  host-reset flow.

## [16.2.0] — 2026-06-12

### Added
- One-click "Phone access — share on local network" toggle in the Windows EXE
  with a scannable pairing QR served over a loopback-only channel.
- Live signal lock-state, HR confidence, motion, and readiness chips; SQI
  time-ribbons on the Waves tab; subject placement-zone guidance on Home.
- Report "Session Quality" scorecard and operator-selectable session
  comparison overlay with validity-aware delta table.
- Keyboard parity: Ctrl+Z undo, Ctrl+H handoff, Ctrl+L lock, D demo toggle.
- Firmware field-diagnostics CSV columns 208–219 (v15.1 contract) with legacy
  207/199-column tolerance; peripheral recovery backoff, reset-reason and NVS
  forensics; default-off presence-gated power management.
- Pip-installable trainer (`rvt-trainer` console script, `[ble]` extra),
  release artifact integrity verification, Windows visual-baseline refresh
  workflow, operator quickstart documentation.

### Fixed
- LAN pairing-gate lockout of the EXE's own WebView after share-mode restarts
  (loopback bypass + forced operator re-login).
- Public `/api/server-info` no longer exposes pairing PIN/QR material.
- Comparison means exclude invalid publishes; Live "holding" state shows the
  last accepted value instead of `--`.

## [16.1.0] — 2026-06-08

### Added
- Native OTA install integration: release manifests (`rvt-latest.json`,
  `rvt-latest-tauri.json`), Tauri updater wiring, Android APK download/install
  bridge; Settings "Update & Version Info" card.
- Operator profiles with PBKDF2 PINs, station lock overlay, and operator
  session tokens (merged to main alongside 16.2.0 work).

## [16.0.x] — 2026-05 → 2026-06

### Added
- Mobile-first Angular 21 + Material 3 dashboard (v12) replacing the legacy
  monolith; PWA install, Capacitor Android APK, Tauri Windows EXE with bundled
  Python trainer sidecar; PIN-paired LAN mode; demo sandbox; preflight and
  ML-readiness audit pipeline; Playwright smoke/visual CI across four
  viewports.

[Unreleased]: https://github.com/blayalems/radar-vital-mmwave-60-davao/compare/v16.2.0...HEAD
