# AGENTS.md — contract for AI coding agents working on this repo

This file is the operating manual for AI agents (Claude Code, Codex, Cursor, etc.) editing this repository. Humans should read [`README.md`](./README.md) first.

**Before editing anything**, read [`HANDOFF.md`](./HANDOFF.md). It tracks live progress on the v12/v16 dashboard refactor, the EXE/APK/PWA packaging, and the trainer split. **Update HANDOFF.md in the same commit as your changes** — every agent and every commit. Drift between the code and the handoff file is treated as a regression.

## Scope

The current focus of this branch is the mobile-first Angular Material 3 redesign that turns the v11 desktop-only dashboard into v12 — a single built document that installs as a PWA on phones, ships as an Android APK via Capacitor, and ships as a Windows EXE via Tauri.

The design plan lives at `/root/.claude/plans/take-a-look-at-fizzy-taco.md` on the working machine. The condensed contract below restates what plans cannot encode: invariants, file-touching rules, and the verification protocol.

## Invariants — do not break these

1. **Three coupled artefacts.** Every change must be evaluated against all three: firmware (`.ino`), trainer (`.py`), dashboard (`.html`). Don't ship a dashboard call to an endpoint the trainer doesn't serve. Don't ship a trainer endpoint nobody calls.
2. **Branch discipline.** Develop PR71/PR72 production fixes on `codex/pr71-exe-production-fixes` unless the human explicitly switches branches. Never push to `main` or `archive/*` without an explicit human instruction. The archive branch is frozen.
3. **No regression in the v11 features.** All KPI cards, chart tabs (Overview/Waves/HR/RR/Snaps/Audit), alerts drawer, snapshot capture, export, sliders, preflight, session-start/stop, and the four themes (light/dark/night/hc) must keep working. Use Playwright visual regression against the `v15.0.0-pre-mobile` baseline before deleting any CSS block.
4. **Serial protocol is frozen.** The current v15.2 CSV is 222 columns at 115 200 baud; the original 207-column prefix is contractual, v15.1 columns 208-219 remain stable diagnostics, and v16.4 columns 220-222 are right-edge audit additions. Don't change column order, names, or units. Add new columns only as additions on the right, and keep the trainer tolerant of 207-column legacy rows plus 219-column v15.1 rows for replay.
5. **Approved frontend framework migration.** The v12 dashboard UI is a standalone Angular application using Angular Material 3 under `web/src/`. Continue new UI work in Angular/Material; do not introduce a second UI framework or externally hosted runtime dependencies. Non-Angular browser assets remain self-hosted under `assets/`.
6. **Secrets stay out of `assets/`.** TLS material lives in `.rvt_tls/` (git-ignored). The trainer's path-traversal guard denies any URL that resolves into `.rvt_tls/` or any path containing `..`. Never serve private keys from a publicly mounted directory.
7. **CSP is per-mode, not blanket.** `--bind local` ⇒ `connect-src 'self'`. `--bind lan` ⇒ `'self' <advertised-lan-origin>` (the literal trainer URL chosen at boot). Capacitor/Tauri WebViews keep `'self'` because LAN traffic flows through native HTTP, not the WebView fetch path.
8. **HSTS only when explicit.** Never send `Strict-Transport-Security` under a self-signed cert. Gated to `--tls-trusted` only.
9. **Backward-compatibility tombstone.** The pre-existing `/rvt-sw.js` route stays for one release, but the body is a self-unregister stub. The new SW is `/sw.js`. Don't remove the tombstone until v12.1.
10. **No marketing model names in artefacts.** Don't write the model ID this agent runs on into commit messages, PR bodies, code comments, or docs.
11. **SSE Telemetry Reconnect Contract.** The trainer's Server-Sent Events (SSE) `/api/events/subscribe` connection has a hard 12-hour session lifetime limit. The trainer must emit a `session_warning` event with payload `{"reason": "deadline_approaching", "seconds_remaining": 60}` exactly 60 seconds before termination. The Angular client's EventSource or custom reconnect driver must handle this warning or catch the native browser `disconnect`/`close` to re-establish the event stream without session-state clobbering.
12. **QMS change control.** Follow [`quality/qms-policy.md`](./quality/qms-policy.md) as the ISO 9001:2015-aligned documented-information and change-control procedure; this is not a certification claim. Every material PR references active IDs from `quality/requirements.json`, updates affected controlled-document revisions in `quality/document-register.json`, advances exactly one product patch or minor, and records objective verification plus rollback/release impact. Unchanged schema IDs do not advance mechanically.
13. **Feedback-loop and manuscript figure contract.** [`docs/system-feedback-loop.md`](./docs/system-feedback-loop.md) is the canonical hardware, firmware, trainer, dashboard, GBR, and experimental 1-D CNN process diagram. Any change to acquisition, serial parsing, feature construction, dataset grouping, model training/evaluation, artifact promotion, prediction, operator feedback, or version/protocol identity must update that document in the same commit. Both model families must use the same recorded inputs, outer holdout assignments, and statistical report contract so comparisons are reproducible and manuscript figures/tables can be regenerated without manual relabelling.
14. **One release increment per PR.** Every pull request that changes shipped code, firmware, model/statistics behavior, session schemas, packaging, or user-facing documentation must advance the product version exactly once relative to its base: either one patch (`16.5.0` -> `16.5.1`) or one minor with patch reset (`16.5.x` -> `16.6.0`). Major jumps require explicit human approval. Run `npm run test:pr-version -- --base-ref <base-ref>`; every version carrier must still pass `npm run test:version-contract`.
15. **Participant- and release-bound sessions.** Every newly captured or simulated study session must immutably record a pseudonymous `participant_id`, `trial_id`, condition, distance, barrier state, trial number, product/trainer/dashboard versions, expected and observed firmware versions, serial protocol/width, source commit, and model-bundle identity when inference is used. A participant profile is not a demographic `subject_profile`; do not reuse one for the other. Reassignment after capture is forbidden. Legacy sessions may be imported only with an explicit `legacy_unassigned` provenance state and must not enter confirmatory GBR/CNN evaluation.

## File-touching rules

| Path | Edit policy |
|---|---|
| `web/` | **Source of truth** for the Angular dashboard. `web/src/app/**` owns standalone components and services; `web/src/styles/**` preserves migrated v11 CSS contracts; `web/package.json` owns Angular/Material dependencies. Run `npm --prefix web ci` in a clean checkout, then `npm run build:web` regenerates the root monolith and `www/`. |
| `web-legacy/` | Compatibility reference for pre-Angular v12 behavior and v11 feature parity. Do not add new UI behavior here; migrate required behavior into Angular components/services. |
| `radar_vital_live_dashboard_v12_for_v16_0.html` | **Built artifact** — do not edit by hand. Regenerated by `scripts/build-angular.mjs` from `web/`. Still committed so trainer-served and local artifact flows keep working. `npm run build:check` in CI catches drift. |
| `scripts/extract-monolith.mjs` | Historical one-shot extractor for the vanilla source archive. Do not run it for ordinary Angular work. |
| `radar_vital_trainer_v12_for_v16_0.py` + `rvt_trainer/` | The root file is a compatibility shim. `rvt_trainer/monolith.py` still owns the legacy `_ControlHandler` dispatcher until the next deep extraction; package facades in `rvt_trainer/api`, `assets`, `audit`, and `transport` expose the new module boundary. Add new routes before the legacy static-file fall-through, document them in `README.md`, and cover them with a Playwright API test. |
| `radar_vital_v16_5_11.ino` | Firmware. `ENABLE_BLE` is `false` by default; the active USB contract is v15.2 / 222 columns. Keep radar BLE/GATT work gated until accepted. Watchdog must never `delay()` in the DSP loop; use `bleSuppressUntilMs = millis() + N` instead. |
| `assets/sw.js` | Single service worker. HTML/manifest are **network-first** (2 s timeout). Static `/lib/`, `/fonts/`, `/icons/` are cache-first. `/api/*` is network-only with an explicit passthrough for `text/event-stream`. |
| `assets/manifest.webmanifest` | Static fallback. The trainer overrides with a dynamic, port-aware manifest at `/manifest.webmanifest`. The hosted GitHub Pages build uses the static file. |
| `docs/system-feedback-loop.md` | Canonical cross-stack and dual-model figure source. Keep hardware/software feedback arrows, version labels, reproducibility fields, statistics outputs, and manuscript-ready captions synchronized with implementation. Run `npm run docs:export-feedback-loop` after changing its Mermaid blocks. |
| `.rvt_tls/` | Git-ignored. Auto-populated on first `--tls` launch. Never commit. |
| Legacy `radar_vital_v8_*.ino`, `_v9_*.ino`, `_v10_*.ino`, `_v11_for_v15_0.html`, `_v10_for_v14_1.html`, trainer `_v10_*.py`, `_v11_*.py` | Removed from this branch. Available on `archive/legacy-v8-to-v11`. Do not restore. |

## Verification protocol — every commit

1. **HANDOFF.md updated** — every commit gets a dated entry at the top of the progress log in [`HANDOFF.md`](./HANDOFF.md).
2. **QMS contract clean.** Run `npm run test:qms-contract -- --base-ref <PR-base>`; controlled documents, requirement-to-test traceability, per-commit handoff evidence, and release-record schemas must pass.
3. **Build round-trip clean.** In a clean checkout run `npm --prefix web ci`, then `npm run build:check`; the assembled `radar_vital_live_dashboard_v12_for_v16_0.html` must match what the Angular project would produce.
4. **Trainer imports clean.** `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer` and `python -m rvt_trainer --help` both exit 0.
5. **Playwright smoke passes.** `npm test` reports zero uncaught script errors across desktop, Pixel 7, iPhone 14, and iPad projects.
6. **Visual regression.** `npx playwright test --update-snapshots` only when an intentional UI change ships; otherwise the baseline matches.
7. **Frontend compilation.** `npm --prefix web run build` exits 0. ESLint is not configured; review TypeScript/template changes and cover interactive contracts with Playwright.
8. **APK build green.** `.github/workflows/build-apk.yml` produces an installable APK artifact.
9. **EXE build green.** `.github/workflows/build-exe.yml` produces an NSIS `.exe` installer on the `windows-latest` runner.
10. **Pages preview green.** `.github/workflows/pages.yml` builds and deploys to `gh-pages` (or `pages` env) without errors.
11. **Feedback-loop export clean.** `npm run docs:export-feedback-loop -- --check` confirms both canonical Mermaid figures and manuscript caption snippets can be reproduced from `docs/system-feedback-loop.md`.
12. **PR release step valid.** On pull requests, `npm run test:pr-version -- --base-ref origin/$GITHUB_BASE_REF` proves the branch advances exactly one patch or minor version from its base.

## What "done" looks like for the redesign

- Dashboard installs as a PWA on Android Chrome and Chrome desktop.
- Bottom-nav primary on phones (< 1024 px), rail returns at ≥ 1024 px, full desktop chrome at ≥ 1280 px.
- All four themes pass visual regression.
- Sheets (palette, alerts, detail) trap focus, return focus on close, dismiss via Escape and back-button.
- DEMO banner is visible whenever sandbox mode is active. IndexedDB key prefix `demo:` keeps demo data segregated from `live:`.
- Trainer fail-fast on port conflict (default 8765; pass `--port 0` only when an auto-picked port is explicitly wanted).
- Capacitor APK + Tauri NSIS EXE both report a real session at 1 Hz against a LAN-bound trainer.
- Radar-firmware BLE/GATT work lands in a follow-up only when the GATT acceptance suite is green. AiLink oximeter BLE reference capture is already handled by the trainer/sidecar path and may be advisory for Start.

## What "done" does **not** mean

- Bit-identical desktop layout. The v12 desktop view is mobile-first scaled up — visually close, not pixel-identical to v11.
- iOS Safari Web Bluetooth. Safari does not support it; iOS BLE only via the Capacitor iOS shell (v16.1+).
- A second UI framework or externally hosted charting/runtime dependency beyond the approved Angular/Material application.
- Subnet scanning. The pairing model is QR + PIN only.

## How to ask for help

If you hit ambiguity, **stop and ask the human via `AskUserQuestion`** before guessing. Specifically:
- Conflicting feature requests (e.g., "preserve v11 exactly" vs "redesign mobile-first").
- API shape changes that would break existing consumers.
- New external network dependencies.
- Any action with a destructive blast radius beyond a single commit on this branch.

The plan file is a contract, not a suggestion. If a request contradicts the plan, surface the conflict instead of silently breaking the plan.
