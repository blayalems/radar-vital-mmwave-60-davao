# Cross-stack audit and improvement plan

Date: 2026-07-17

Baseline: `main` at `2d2259e` (`v16.4.0-main.55`)

Scope: firmware, Angular/PWA frontend, Python control backend, build tooling,
dependencies, and staged refactoring.

## Executive summary

The product contracts are fundamentally healthy: the firmware/trainer CSV
schema is still frozen at 222 columns, BLE remains disabled by default, the
Python and Angular unit suites pass, and the generated dashboard remains
self-contained. The audit nevertheless found several high-impact lifecycle
races where a healthy-looking UI could disagree with the real trainer.

This hardening PR fixes the highest-confidence issues that can be verified
without physical hardware:

- a real recording can no longer be silently switched to simulated controls;
- SSE connection creation is single-flight, cancellable, and renews its
  one-use token at the 12-hour warning;
- session start no longer treats the supervisor's own placeholder dashboard as
  proof that the child process started;
- corrupt operator-profile storage fails closed and is never overwritten by a
  profile mutation;
- radar recovery no longer spends up to 400 ms waiting for optional module
  metadata inside the live loop;
- runtime Angular, build tooling, and pytest dependency floors include the
  available security fixes; and
- monolith generation uses a pinned esbuild API and fails if chunk bundling
  fails.

The remaining items below are intentionally separated into smaller follow-up
PRs. Firmware presence/LCD changes require hardware or protocol fixtures;
frontend report/preflight races require controlled deferred-response tests; and
backend shutdown semantics need an explicit analysis policy.

## Findings and disposition

| ID | Severity | Area | Finding | Disposition |
|---|---|---|---|---|
| WEB-01 | High | Source safety | Settings, command palette, keyboard, connect wizard, and disconnect fallback could enter demo while a real session was active. Stop was then handled by the sandbox while the trainer kept recording. | Fixed here with centralized guarded source transitions plus a real-session Stop defense. |
| WEB-02 | High | Source truth | Trainer-reported `mode=sandbox` did not participate in every banner/simulation decision, and the banner offered an exit action that cannot disable a server sandbox. | Fixed here with `demoSourceActive` and truthful trainer-sandbox copy. |
| WEB-03 | High | Telemetry | Concurrent SSE token mints could leak multiple streams; Stop/lock/demo could not cancel an in-flight mint; the 12-hour warning did not rotate the one-use token. | Fixed here with single-flight/generation guards and immediate token/stream renewal that preserves session state. |
| WEB-04 | High | Reports | Slow session A responses can overwrite newly selected session B; saves read mutable selection/form state after awaits. | Follow-up PR 1: epoch guards and captured immutable save payloads with out-of-order response tests. |
| WEB-05 | High | Preflight | Overlapping port/setup probes can apply stale results and briefly clear `preflightRunning`, allowing Start to use checks for a different setup. | Follow-up PR 1: setup fingerprint plus latest-request generation and change-during-Start tests. |
| WEB-06 | High | Privacy/PWA | The service worker caches navigation URLs including `?pair=PIN`; issue diagnostics can include raw `ctl.error` text. | Follow-up PR 1: canonical shell cache key and category-only/sanitized control errors. |
| WEB-07 | Medium-high | Authentication | Arbitrary absolute `/api/` URLs can receive trainer auth headers because trust is path-based rather than exact-origin-based. | Follow-up PR 1: exact configured-origin policy with explicit native-loopback handling and exfiltration tests. |
| WEB-08 | Medium | Accessibility | Interactive KPI cards use status/live-region semantics, causing ambiguous actions and possible 1 Hz screen-reader chatter. | Follow-up PR 2 with the Live component split: button semantics and one debounced aggregate status channel. |
| WEB-09 | Medium | Settings/accessibility | The Text Scale slider was a `0 x 0` flex item inside a column row, so its absolute thumb covered the valid `100%` output and the input had no accessible name. | Fixed in the browser-audit follow-up: a full-width control row, percentage formatter, accessible name, and geometry regression. |
| WEB-10 | Medium | Mobile accessibility | The mobile demo exit action measured 20 px high, Live actions/mode controls measured 40 px, and quick tags measured 32-36 px. | Fixed in the browser-audit follow-up with scoped 44 px mobile targets and a no-overflow regression. |
| WEB-11 | Medium | Visual trust | Home can show populated demo KPIs while the Radar Trend surface appears empty, which can read as a stalled stream. | Follow-up PR 2: render an honest warming/no-samples state from the real trend-buffer state. |
| WEB-12 | Medium | Mobile hierarchy | Demo provenance, page chrome, recording actions, and fixed navigation leave little first-screen room for Live KPIs at 390 x 844. | Follow-up PR 2: characterize and compact/collapse the status/action header without hiding source provenance or Stop. |
| WEB-13 | Low | Discoverability | The desktop Live snapshot FAB is visually unlabeled and appears to duplicate nearby snapshot/bookmark actions. | Follow-up PR 2: verify its unique purpose, tooltip, focus behavior, and placement before retaining or removing it. |
| BE-01 | High | Session lifecycle | `_SessionSupervisor.start()` created `live_dashboard.json`, immediately saw that same file, and returned success before checking whether the child had exited. | Fixed here: supervisor placeholder is marked, written before spawn, ignored for readiness, and child exit is checked first. |
| BE-02 | High | Auth integrity | An authenticated profile-create request could replace a corrupt profile DB with a new one-profile database despite the loader's fail-closed contract. | Fixed here: every auth mutation returns `503 OPERATOR_STORE_UNAVAILABLE` and preserves the original bytes. |
| BE-03 | High | Shutdown | `_ControlServer.stop()` closed HTTP but did not stop/reap an active detached session child, leaving recording and stale current-session state behind. | Fixed in follow-up PR 1: shutdown closes the start gate, preserves captured files, performs a bounded reap, retains truthful markers on terminal failure, and never launches analysis. |
| FW-01 | High | Loop timing | Radar recovery invoked a blocking firmware-version window with `delay(25)` for up to 400 ms, consuming loop time and potentially parser frames. | Fixed here: optional metadata is probed once per normal parser pump without waiting. |
| FW-02 | High | Presence FSM | Valid absent packets can refresh a timestamp later treated as presence evidence; the derived FSM result is then written back into raw evidence, potentially latching presence indefinitely. | Follow-up firmware/bench PR: separate raw, derived, and published presence state; replay present-to-absent packets. |
| FW-03 | High | LCD recovery | Successful runtime rescans do not restore `lcdConnected`; failed probes can leave allocation state inconsistent before placement-new reconstruction. | Follow-up firmware/bench PR: one idempotent attach/teardown owner and connect-fail-reconnect fixture. |
| BUILD-01 | High | Reproducibility | Root `npx esbuild` was floating and bundling failure silently fell back to an unresolved Angular entry chunk. | Fixed here with exact `esbuild` dependency, JS API use, and fatal build failure. |
| DEP-01 | High | Supply chain | The baseline audit reported Angular production advisories, vulnerable build transitive packages, and pytest `PYSEC-2026-1845`. | Fixed here; both npm trees and the Python requirements audit are expected to report zero after verification. |

## Browser walkthrough evidence

The frontend audit used the installed Chrome browser against the locally served
self-contained dashboard. It captured the same seeded demo/operator state at
1440 x 1000 and 390 x 844 across Home, Live Simple, Live Advanced, Report,
Settings, Home mobile, and Live mobile.

- No audited route introduced horizontal document overflow at either width.
- Before the fix, the Settings Text Scale row measured 313 x 94 while the
  slider host measured 0 x 0; the thumb overlapped the `100%` output. After the
  fix, the slider measures 233 x 48 and the separate 52 px output no longer
  overlaps.
- On mobile, the demo exit action, primary Live actions, both mode controls,
  and every quick tag now measure at least 44 x 44. The post-fix 390 px capture
  still has no horizontal overflow.
- Home/Live/Report/Settings remain visually consistent with the existing
  Material 3 palette, demo provenance is persistent, Stop remains prominent,
  and the Report summary remains easy to scan.
- The automated clipped-text probe over-reported Material icon ligatures and
  scroll dimensions. Those entries were not treated as bugs without matching
  screenshot evidence.

The browser regressions live in `tests/smoke/dashboard.spec.ts` and
`tests/smoke/settings-cards.spec.ts`; both focused tests pass in installed
Chrome.

## Refactoring plan

### PR 1 — Lifecycle, race, and privacy closure

Risk: medium-high. This PR should remain behavior-focused and precede component
decomposition.

1. Backend shutdown owns the active child lifecycle:
   - snapshot the current supervisor state;
   - request a graceful stop with a bounded terminate/kill fallback;
   - clear current markers idempotently; and
   - `server_shutdown` preserves captured files but never launches analysis;
     explicit user stop may analyse only after a successful reap and cleanup.
2. Report uses independent monotonically increasing epochs for primary and
   comparison loads. Saves capture session ID, note, and sign-off before the
   request and update visible state only if the same session is still selected.
3. Home preflight captures a setup fingerprint. Only the latest generation may
   apply results, and Start aborts when the setup changed during validation.
4. Service-worker navigations use one canonical scope shell key; query strings,
   pairing PINs, and route variants never become CacheStorage keys.
5. Issue diagnostics omit raw control errors or map them to safe category/code
   fields. The newest five alerts are selected in the correct order.
6. Authentication headers attach only to the exact configured trainer origin,
   same-origin relative APIs, or a narrowly defined native loopback origin.

Acceptance:

- deferred A/B report and COM3/COM4 preflight tests resolve out of order;
- server shutdown leaves no process or current-session marker;
- CacheStorage contains no `pair`, PIN, or route-specific shell entries;
- diagnostic reports exclude paths, bearer tokens, PINs, subject/operator
  labels, and raw server messages; and
- `https://untrusted.example/api/x` receives no trainer credentials.

#### Implementation status — PR #77 (2026-07-22)

Complete for source-controlled and automated acceptance. All six implementation
items above are present on `codex/lifecycle-race-privacy-closure`.

| Acceptance requirement | Authoritative implementation and verification evidence | Status |
|---|---|---|
| Deferred Report A/B loads and COM3/COM4-style setup probes resolve out of order without stale UI state | `ReportComponent` owns independent primary/comparison epochs and immutable save payloads; `HomeComponent` owns a setup fingerprint and latest-generation gate. `report.component.spec.ts` covers late A after B for primary/comparison plus both saves; `home.component.race.spec.ts` covers newest-response ownership and setup changes during Start. | Complete |
| Server shutdown leaves no child process or owned current-session marker | `_ControlServer.stop()` closes the supervisor gate and calls `_SessionSupervisor.stop(reason="server_shutdown", auto_analyse=False)` with bounded reap and owned-marker cleanup. `test_trainer_lifecycle.py::test_control_server_shutdown_owns_session_reap_without_analysis` and the stop/reap cases in `test_session_isolation.py` cover ordering, escalation, idempotence, and failed-reap truthfulness. | Complete in automated process fixtures |
| CacheStorage contains no pairing PIN or route-specific shell entries | `assets/sw.js` uses the scope-qualified `NAVIGATION_CACHE_KEY`, refreshes it only from `DASHBOARD`, and keeps pairing/support navigations network-only. `tests/service-worker-cache.test.mjs` executes online, offline, and pairing cases and asserts that neither `pair=` nor route keys are written or read. | Complete |
| Diagnostic reports exclude paths, bearer tokens, PINs, identities, and raw server messages | `IssueReportService` allowlists control fields, reduces failures to categories, sorts the newest five alerts, and applies contextual PIN/token/identity/path redaction to previews and URLs. `issue-report.service.spec.ts` covers raw control errors, false-positive auth terms, 3-digit PINs, paths, tokens, recovery codes, operator IDs, subject/operator labels, and caller-supplied legacy reports. | Complete |
| Untrusted absolute API targets receive no trainer credentials | `api-target-policy.ts` requires the exact configured origin, with only the explicit native loopback exception. `api.service.spec.ts`, `auth.interceptor.spec.ts`, and `tauri.interceptor.spec.ts` cover untrusted API-looking URLs, origin-prefix lookalikes, caller-supplied auth headers, downloads, and native routing. | Complete |

The remaining gates are external release validation, not unimplemented PR #77
scope: packaged Windows EXE child-process stop/start behavior and the native
12-hour reconnect soak still require their target runtime.

### PR 2 — Characterization-first frontend decomposition

Risk: medium. Preserve visual and behavioral baselines.

1. Extract a source-mode policy service from `StateService`/`ApiService`; it
   becomes the only owner of live, manual demo, auto-demo, and server sandbox
   transitions.
2. Extract the SSE driver from `TelemetryService` behind a small event interface
   (connect, renew, cancel, backoff, transport state).
3. Extract Report loading/saving and Home preflight coordination before changing
   their templates.
4. Split Live by tab into presentational OnPush components only after its
   keyboard, chart, and alert behaviors have characterization tests.
5. Add a shared chart scheduler so hidden tabs do not repaint and visible
   sparklines are rate-limited.

Acceptance:

- no extracted component exceeds 400 lines;
- visual baselines have no unintended changes;
- one owner exists for each async workflow;
- the initial bundle warning is reduced and then locked with a realistic budget;
- KPI controls have button semantics and screen readers receive state
  transitions rather than the raw telemetry cadence.

### PR 3 — Backend service boundaries

Risk: medium-high because `rvt_trainer.monolith` is a compatibility surface.

1. Add characterization tests around `_SessionSupervisor`, auth/profile
   persistence, SSE token use, static assets, and the route matrix.
2. Move session lifecycle into `rvt_trainer/session/supervisor.py`.
3. Replace the giant method route cascade with small route registries grouped
   by auth, sessions, telemetry, reports, and static assets.
4. Keep import shims in `rvt_trainer.monolith` until downstream callers and
   packaging tests are migrated.
5. Extract common JSON error/timeout/atomic-write helpers so security policy is
   enforced once.

Acceptance:

- public CLI, schema IDs, error codes, routes, and packaged entrypoint remain
  unchanged;
- monolith compatibility tests import both old and new locations;
- route authorization is table-tested; and
- process, file, and HTTP cleanup is idempotent.

### PR 4 — Firmware state ownership and recovery

Risk: high; requires XIAO ESP32-C6 + MR60BHA2 bench validation. Keep the
single-sketch policy and frozen serial layout.

1. Separate `radarPresenceRaw`, `presenceFsmState`, and published effective
   presence. Derived state must never refresh raw evidence timestamps/scores.
2. Model firmware-version capture as a bounded nonblocking state (`idle`,
   `armed`, `captured`, `expired`) re-armed after `mmWave.begin()`.
3. Centralize LCD object attach/teardown, clear allocation/connectivity state
   together, and make repeated recovery idempotent.
4. Replace the remaining DSP-adjacent delays with deadline-driven states where
   hardware allows; document setup-only delays.

Acceptance:

- Arduino compile for `esp32:esp32:XIAO_ESP32C6`;
- 222 CSV fields in the exact existing order;
- `ENABLE_BLE false` remains the default;
- present-to-absent replay reaches LEAVING/ABSENT within configured bounds;
- connect-fail-reconnect LCD test balances destruction/reconstruction; and
- loop timing shows no metadata-induced stall.

## Verification matrix

| Layer | Automated gate | Physical/manual gate |
|---|---|---|
| Dependencies | root/web `npm audit`; `pip-audit` for both requirement files | none |
| Frontend | source integrity, contrast, Angular unit tests, production build, monolith round trip, Playwright smoke | native reconnect and 12-hour renewal soak |
| Backend | compileall, CLI help, full pytest suite | child-process stop/start behavior in packaged Windows EXE |
| Firmware | static protocol/loop guards; Arduino CI compile | radar recovery, absent-person transition, LCD reconnect, CSV capture |

The physical firmware and packaged-native gates remain required before a
release even when this PR's source-level checks pass.
