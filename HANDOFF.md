# HANDOFF — Refactor & Packaging Progress

> **Mandatory:** Every AI agent (and human) that pushes work to this repo updates
> the relevant table row below in the same commit. Drift between code and this
> file is treated as a regression. Keep entries terse — one line per change.
> The newest entry goes at the **top** of the log, dated.

## Status snapshot

| Workstream | Source-of-truth | Current state | Owner / last touched |
|---|---|---|---|
| Dashboard refactor (v12/v16 monolith → split) | `web/` | Phase 3 complete — body partials split; legacy patch tail bundled into 2 patch files | codex/complete-handoff-phases |
| Trainer refactor | `radar_vital_trainer_v12_for_v16_0.py` + `rvt_trainer/` | Phase 4 **partial extraction in progress**. `api/server_info.py` owns the real manifest/origin/support-matrix/pair-page builders (PR 5). `api/auth.py`/`assets/static.py` will own real code on PR 3/4 merge. `api/sse.py` stays a facade — the `ControlHandler` dispatcher is too coupled to move without a Phase 5 refactor. | claude/trainer-extract-sse-and-info |
| PWA (GitHub Pages) | `.github/workflows/pages.yml` → `www/` | **Red on PR #5 head** (`deploy` failed in 4 s). Untriggered on PR #6 (no path-filter match). | unstable |
| APK (Capacitor) | `.github/workflows/build-apk.yml` + `capacitor.config.ts` | **Red on PR #5 head** (`apk` failed in 16 s). Untriggered on PR #6. | unstable |
| EXE (Tauri) | `.github/workflows/build-exe.yml` + `src-tauri/` | **Red on PR #5 head** (`windows` ran 14 min then failed — real Tauri MSI error). Untriggered on PR #6. | unstable |
| Smoke + visual tests | `tests/` | Locally 14/14 desktop green, 44/56 with WebKit projects (sandbox lacks webkit). **CI `test` job red on both PR #5 and PR #6** in 6–9 s — under investigation in `claude/ci-roundtrip-fix`. | claude/ci-roundtrip-fix |

## How the dashboard build flows

```
web/                          (source of truth, split files)
  index.html                  (shell with BUILD:INCLUDE markers)
  styles/*.css                (one file per <style id="...">)
  modules/*.js                (one file per <script id="...">)
  components/**/*.html        (panel partials + overlays)
        │
        ▼   npm run build:web
        │   scripts/build-www.mjs resolves markers, concats in order
        ▼
radar_vital_live_dashboard_v12_for_v16_0.html   (assembled monolith — built artifact)
        │
        ▼   build-www.mjs also copies to www/
        ▼
www/
  index.html                  (= assembled monolith)
  radar_vital_live_dashboard_v12_for_v16_0.html  (alias for deep links)
  assets/ manifest.webmanifest sw.js 404.html
        │
        ├──▶ GitHub Pages (.github/workflows/pages.yml)
        ├──▶ Capacitor sync   (npm run cap:sync)
        └──▶ Tauri build      (npm run tauri:build)
```

## Contracts that must NOT break

1. **DOM element IDs and class names are frozen.** `#tab-overview`, `#fwBadge`,
   `#demoBanner`, `.r-item`, etc. are referenced by scripts and CSS selectors.
   Renaming them silently breaks the cascade.
2. **Load order is contractual.** See `web/index.html` for the canonical order.
   `v11-*` and patch blocks intentionally come AFTER the consolidated CSS/JS to
   override the cascade and monkey-patch behaviour. Do not reorder.
3. **`#rvt-v12-demo-first-paint` must be adjacent to `#demoBanner`** — the
   script reads the banner element synchronously at parse time.
4. **Service worker path** = `/sw.js`. Manifest path = `/manifest.webmanifest`.
   These resolve from www root, served by the trainer at `/sw.js` and
   `/manifest.webmanifest` (NOT `/assets/sw.js`).
5. **Trainer API surface** (consumed by dashboard JS): `/api/server-info`,
   `/api/auth/exchange`, `/sw.js`, `/manifest.webmanifest`, `/pair`,
   `/api/telemetry` (SSE). See `AGENTS.md` for the full list.
6. **The monolithic `.html` at repo root remains a build artifact.** Some
   consumers (file:// open, legacy trainer paths) still expect it. It can be
   `.gitignore`d once nothing references it directly.

## Refactor progress log (newest first)

### 2026-05-16 — Trainer Phase 4: real server_info extraction (PR 5 of CI batch)
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
- Same preventive `api/__init__.py` stub as PR 3 (this branch was off main
  before PR 3 merged; the fix applies cleanly).
- New unit tests `tests/test_trainer_server_info.py` (8 tests):
  - explicit `advertised_host` override, bound fallback, 0.0.0.0→LAN guess
  - origin format http vs https
  - manifest PWA contract (id, display, start_url, scope, 192/512 icons,
    maskable purpose, Start/Open-last-report shortcuts)
  - support matrix lists all 5 modes
  - pair page shows PIN + "Expires in" + "consumed after first use"
  - pair page without active PIN shows the local-bind notice instead
- `api/sse.py` stays a facade — `_ControlHandler` is 1000s of lines tightly
  coupled to `BaseHTTPRequestHandler`; safe extraction needs a Phase 5
  dispatcher refactor (tracked separately).

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
  1. `claude/ci-roundtrip-fix` — diagnose + harden the `test` workflow (this PR).
  2. `claude/ci-workflow-paths-and-fixes` — fix `apk`, `windows`, `deploy/pages`.
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
- **Always run `npm run build:web && npm test`** before pushing. The full
  multi-browser smoke suite can take several minutes on WebKit-backed projects.
- **Visual regressions** (`npm run test:visual`) need a Linux Playwright
  baseline. If you commit new baselines, take them from CI, not your local
  machine — fonts/AA differ.
- **DO NOT delete the root `radar_vital_live_dashboard_v12_for_v16_0.html`**
  until everything in this repo and any consumer scripts/docs references the
  `web/` source. The monolith stays as a built artifact for at least one
  release cycle.
