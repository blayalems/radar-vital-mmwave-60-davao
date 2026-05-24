# HANDOFF — Refactor & Packaging Progress

> **Mandatory:** Every AI agent (and human) that pushes work to this repo updates
> the relevant table row below in the same commit. Drift between code and this
> file is treated as a regression. Keep entries terse — one line per change.
> The newest entry goes at the **top** of the log, dated.

## Status snapshot

| Workstream | Source-of-truth | Current state | Owner / last touched |
|---|---|---|---|
| Dashboard refactor (v12/v16 Angular Material app) | `web/src/` + `web/package.json` | Angular Material 3 source of truth; command/help/export parity, Live diagnostic restoration, real-series rendering, and responsive theme containment repaired | codex/mobile-first-dashboard-upABy |
| Trainer refactor | `radar_vital_trainer_v12_for_v16_0.py` + `rvt_trainer/` | Phase 4 **partial extraction in progress**. `api/server_info.py` (#11) + `api/auth.py` (#9) + `assets/static.py` (this PR) own real code. `api/sse.py` still a facade. | claude/trainer-extract-static |
| PWA (GitHub Pages) | `.github/workflows/pages.yml` -> `www/` | Stable asset packaging and Angular build pass locally; PR Pages build/deploy preview remains CI-verified | codex/mobile-first-dashboard-upABy |
| APK (Capacitor) | `.github/workflows/build-apk.yml`, `.github/workflows/release-artifacts.yml` + `capacitor.config.ts` | Local `assembleDebug` reverified after Live parity work and produced `app-debug.apk` on 2026-05-24; CI artifact verification follows in PR | codex/mobile-first-dashboard-upABy |
| EXE (Tauri) | `.github/workflows/build-exe.yml`, `.github/workflows/release-artifacts.yml` + `src-tauri/` | Local Tauri NSIS build reverified after Live parity work and produced the Windows setup EXE on 2026-05-24; CI artifact verification follows in PR | codex/mobile-first-dashboard-upABy |
| Smoke + visual tests | `tests/` | Live Material action and containment coverage restored; 92 smoke and 96 visual cases pass on all four projects | codex/mobile-first-dashboard-upABy |

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
