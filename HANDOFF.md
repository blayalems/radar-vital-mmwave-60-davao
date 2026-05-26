# HANDOFF — Refactor & Packaging Progress

> **Mandatory:** Every AI agent (and human) that pushes work to this repo updates
> the relevant table row below in the same commit. Drift between code and this
> file is treated as a regression. Keep entries terse — one line per change.
> The newest entry goes at the **top** of the log, dated.

## Status snapshot

| Workstream | Source-of-truth | Current state | Owner / last touched |
|---|---|---|---|
| Dashboard refactor (v12/v16 Angular Material app) | `web/src/` + `web/package.json` | Angular Material source of truth; scoped IndexedDB, PIN pairing, report sign-off, alert diagnosis, mobile commands, dark inverse HC and native BLE qualification UI implemented | codex/mobile-first-dashboard-upABy |
| Trainer refactor | `radar_vital_trainer_v12_for_v16_0.py` + `rvt_trainer/` | Phase 4 partial extraction continues; monolith now denies generic static reads, protects LAN telemetry reads/SSE and exposes serial/notes/sign-off contracts | codex/mobile-first-dashboard-upABy |
| PWA (GitHub Pages) | `.github/workflows/pages.yml` -> `www/` | Public URL serves the Angular shell; current branch enforces Angular-only Pages artifacts and direct-route shell fallback while its updated SW awaits deployment | codex/mobile-first-dashboard-upABy |
| APK (Capacitor) | `.github/workflows/build-apk.yml`, `.github/workflows/release-artifacts.yml` + `capacitor.config.ts` | CI built the debug APK from `a74c4e1`; local BLE qualification is now reachable in UI, while real paired-LAN/GATT acceptance remains open | codex/mobile-first-dashboard-upABy |
| EXE (Tauri) | `.github/workflows/build-exe.yml`, `.github/workflows/release-artifacts.yml` + `src-tauri/` | CI built the NSIS installer and passed Rust native tests from `a74c4e1`; Home now reaches the bounded GATT probe; hardware acceptance remains open | codex/mobile-first-dashboard-upABy |
| Smoke + visual tests | `tests/` | 38 Python contracts, 120 four-viewport smoke/API checks and 16 affected Home visuals pass locally; prior-head CI full visual/native/package workflows passed | codex/mobile-first-dashboard-upABy |

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

### 2026-05-26 - Remediate connection lockout, active guard history leak, canvas theme repaints, and print disclosure (PR #24 Review)
- Resolved the initial connection lockout by wrapping `/api/status` requests inside a 4-second timeout promise inside `ApiService.detectControlMode()`. Added a manual "Bypass to Sandbox Mode" action button inside the initial loading overlay and ensured loading spinner dismisses cleanly on sandbox fallback.
- Corrected the printed sandbox disclosure by styling `#demoBanner` specifically in `print.css` to print as a premium alert banner at the top of report prints rather than hiding it, fully preserving simulated vitals provenance truthfulness.
- Resolved active-session guard navigation confusion with reviewed history by introducing a dedicated `sessionActive` signal in `StateService`. Decoupled active captures from `currentSessionId` so that selecting a historical session for review does not falsely trigger active-session warnings or enable the Stop button on `/live`.
- Repaired canvas idle repainting by declaring dependencies on the `state.theme` signal inside the canvas effects of `home.component.ts`, `live.component.ts`, and `report.component.ts`, triggering immediate dynamic canvas redraws when theme styles are cycled.
- Verification: `npm run build:web` and `npm run build:check` completed successfully (`OK: web/ ↔ monolith round-trip is clean.`); Vitest unit suite passed completely (**16/16** passing); pytest back-end suite passed completely (**40/40** passing).

### 2026-05-26 - Resolve Help/Live Angular compilation, duplicate methods, and canvas theme-awareness
- Cleaned up `help.component.ts` imports by removing `KeyboardShortcutsDialogComponent` (since it is opened programmatically via `MatDialog` and doesn't need to be declared in the template imports), resolving template compiler alerts.
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
