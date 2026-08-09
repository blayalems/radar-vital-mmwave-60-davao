# Changelog

All notable changes to Radar Vital are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow the
product's semantic `MAJOR.MINOR.PATCH` scheme.

## [Unreleased]

### v16.5.11 evidence and CI hardening (in progress)

- Upgraded vulnerable Python and Angular dependency ranges and made Python plus
  both npm dependency audits blocking instead of advisory-only.
- Reworked Objective 3 to qualify only non-overlapping, capture-backed
  no-subject trials under a locked 72 x 150-second protocol with corroborated
  firmware/rules/threshold identity, and to generate the observed false-alarm
  rate, two-sided 95% Clopper–Pearson interval, and one-sided exact binomial
  decision directly from exactly 72 unique trial-ledger records.
- Contained analysis-worker exceptions in durable failed job records rather
  than leaking unhandled daemon-thread errors.
- Split CI contracts from browser shards, added the stable aggregate `test`
  check, narrowed workflow permissions, preserved the complete Pages release
  evidence plus PWA/legal bundle, honored manual release tags on main, stopped
  swallowing Capacitor failures, and reduced transient artifact retention.
- Coordinated firmware, trainer, Angular/PWA, Capacitor, and Tauri carriers at
  v16.5.11 while preserving the v15.2/222-column serial contract and existing
  study-session/statistical-plan schema identities.

### v16.5.10 model-analysis readiness (in progress)

- Promoted the coordinated firmware, trainer, Angular, PWA, Capacitor, and
  Tauri carriers to v16.5.10 while keeping the v16.5.9 session/protocol schema
  identities stable for replay and migration.
- Added backend-authoritative study-analysis jobs with bounded admission,
  cancellation, durable status recovery, participant-cohort discovery, LOSO
  gating, artifact/hash checks, and a statistics phase that fails closed when
  the draft plan or optional dependencies are unavailable.
- Kept capture model-agnostic by default (`model_family: "none"`) and moved
  GBR/experimental 1-D CNN selection into Model Lab for same-cohort comparison;
  the frontend now exposes RR and HR objectives, progress, provenance state,
  and actionable worker evidence.
- Separated attempted from qualified no-subject controls and documented the
  hardware/software/statistics feedback loop for manuscript reproduction.

### v16.5.9 protocol-attempt provenance (in progress)

- Advance the coordinated firmware, trainer, dashboard, APK, and EXE release
  identity from v16.5.8 to v16.5.9.
- Begin the implementation-plan follow-up with immutable capture
  provenance, canonical protocol attempts, and participant completion evidence.
- Keep GBR as the default model family and 1-D CNN explicitly experimental;
  the confirmatory analysis plan remains draft until research/QMS approval.

### v16.5.8 statistical validation and OOF evidence

- Added a versioned statistical-analysis plan for the proposal's six confirmatory conditions, 30-second/5-second endpoint aggregation, primary 1.0 m/no-cardboard RR TOST, Holm-adjusted secondary tests, participant-balanced agreement, coverage, and exact false-alarm evidence.
- Added `rvt-statistics` JSON/CSV/LaTeX exports with report hashes and provenance hooks, while retaining GBR as the default and CNN as an explicit host-side experimental option.
- Persisted participant-held-out `outer_oof_predictions.csv` with raw and causal post-processed predictions so model comparisons and manuscript tables use the same evidence.
- Contained the QMS fixture base-ref environment leak and recorded that hardware completion remains pending controlled signoff evidence.

### Added
- Added release-aware, pseudonymous participant profiles and immutable trial assignment for confirmatory and exploratory study sessions.
- Added explicit confirmatory validation for the proposal's 0.6/0.8/1.0 m, no-barrier/cardboard, three-trial, 150-second protocol while retaining labelled exploratory captures from 0.5 to 1.0 m.
- Added a cross-stack compatibility handshake that records dashboard, trainer, firmware, serial protocol, API/schema, source, and model identity before capture.
- Added ISO 9001:2015-aligned controlled-document, requirement/test traceability, PR change-control, release authorization, checksum, and build-provenance records without claiming certification.

### Changed
- Promoted every shipped product-version carrier exactly one patch to v16.5.7, renamed the active firmware source accordingly, and advanced only the controlled-document revisions affected by the dual-model readiness release.
- Promoted the firmware, trainer, Angular dashboard, Capacitor package, Tauri package, and issue-report identity together to v16.5.5 while preserving the frozen v15.2 / 222-column serial contract and stable v12 compatibility entrypoints.
- Added an executable cross-language version contract so Python, TypeScript, Rust, C++, package locks, help text, and packaging metadata cannot drift silently.
- Added a canonical hardware–software feedback-loop and dual-model reproducibility figure contract for GBR, experimental 1-D CNN, shared statistical reporting, and direct manuscript integration.

### Fixed
- Made route wildcards segment-aware, blocked inherited `HEAD` filesystem probing, authenticated protected POSTs before body parsing, rejected invalid session-root aliases, and standardized unknown API errors.
- Invalidated cached participant consent identity when the current roster is unavailable, missing the participant, or marks them withdrawn; Start now remains blocked until an active roster selection is verified.
- Corrected adversarial session-state and onboarding truthfulness issues found with the real mock trainer on desktop and Pixel-sized viewports.
- Blocked known release/protocol/schema splits before capture and retained immutable study provenance plus failure evidence when session startup fails.
- Made session Start retry-safe so duplicate browser activation or response-loss retries cannot allocate multiple sessions for one participant/trial intent.

## [16.4.0] — 2026-07-05

### Added
- v15.2 / 222-column radar CSV contract with right-edge v16.4 audit fields while preserving the original 207-column prefix and 219-column v15.1 replay compatibility.
- PR72 session-data regression coverage for on-disk contract width, module firmware truthfulness, adaptive-correction shadow metrics, v15 PQI shadow checks, and BLE reference-quality calculations.
- Passive MR60BHA2 module firmware version polling during firmware boot and radar recovery.

### Fixed
- Trainer truthfulness now measures the on-disk CSV header width instead of loader-added feature columns, avoiding false firmware rejection on valid v16.4 sessions.
- BLE reference quality now uses time-based coverage and preserves `ref_ble_summary.json` during Windows-side capture.
- PR71 live-session start recovery creates standby/startup `live_dashboard.json`, preserves radar-only sessions when BLE is absent, persists Home preflight state, infers legacy session dates/durations/subjects, suppresses standby `0 bpm` alert spam, and bounds Live chart/Doppler growth.

## [16.3.0] — 2026-06-12

### Added
- One-tap GitHub issue reporting (`IssueReportService`, `ReportIssueCardComponent`):
  pre-fills all bug-form fields (version, platform, connection mode, de-identified
  diagnostics, log tail); diagnostics toggle defaults on and persists under
  `rvt-diagnostics-optin`; URL hard-capped at 7 500 chars with safe truncation order
  (log tail → alerts → none); Tauri opens via `plugin:shell|open`, PWA/APK via
  `window.open`. Card not yet mounted (Wave 2 task).
- Tauri `shell:allow-open` capability scoped to `https://github.com/*`.
- Terms & Conditions, Privacy Notice (RA 10173-framed drafts), LICENSE,
  CONTRIBUTING guide, issue templates, and wiki source pages.
- First-run consent gate: `FirstRunService` reads/writes localStorage under
  `CONSENT_KEY`/`TUTORIAL_DONE_KEY`; consent dialog (standalone, Material 3,
  `disableClose:true`) with RA 10173 summary, Decline → blocking panel with
  Back-to-Terms, Accept records consent; terms re-prompt on TERMS_VERSION bump.
- Onboarding tutorial: 6-step skippable coach-mark dialog (no MatStepper);
  platform-aware connect step (exe/native/pwa); arrow-key navigation;
  auto-triggered once after `rvt-operator-authenticated`; re-playable via
  `FirstRunService.replayTutorial()` (Wave 2 wires to command palette).
- `AboutCardComponent` (standalone, settings-card idiom): product name, version
  input, auto-year copyright, three authors, program/university, links to Terms,
  Privacy, License, and GitHub repo, stack acknowledgements. Ready for Wave 2
  insertion into settings.component after the "Update & Version Info" card.
- Trainer `/about` page (`support_matrix_html`) now includes a copyright footer
  with all three author names, program, university, and dynamically computed year.
- Google Play closed-testing groundwork: `patch-android-shell.mjs` pins
  `compileSdk`/`targetSdk` to 35 (fails loudly if gradle pattern is absent),
  installs adaptive launcher icon (mipmap-anydpi-v26 XMLs with monochrome layer,
  foreground vector drawable, `#0E5E63` background color resource);
  `docs/play/store-listing.md` and `docs/play/data-safety.md` covering app
  identity, descriptions, screenshot shot-list, content-rating answers, closed-
  testing constraints, and Play Data Safety form answers mirroring PRIVACY.md.
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

[Unreleased]: https://github.com/blayalems/radar-vital-mmwave-60-davao/compare/v16.4.0...HEAD
[16.4.0]: https://github.com/blayalems/radar-vital-mmwave-60-davao/compare/v16.3.0...v16.4.0
[16.3.0]: https://github.com/blayalems/radar-vital-mmwave-60-davao/compare/v16.2.0...v16.3.0
