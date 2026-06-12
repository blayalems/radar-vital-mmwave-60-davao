# Multi-PR Improvement Plan — UI/UX, Features, and Firmware

---

## Status update (2026-06-12)

### Track A — Dashboard (PR-49 … PR-54)

The PR #52 stacked series has landed, superseding the individual PR-49 through
PR-54 plan items. Specifically:

- **Signal-quality surfacing** — SQI progress-bar ribbons below the Waves tab
  waveforms, PQI confidence colouring on the Bland-Altman scatter plot, and
  funnel-survival / gate-reason histograms in the Live Audit tab are present in
  `live.component.html` / `live.component.ts`.
- **Quality scorecard** — the Analysis Readiness Outcome banner (READY /
  CONDITIONAL / NOT READY) and Session Comparison (HR RMSE, HR r) are in
  `report.component.html`.
- **Comparison overlay** — ghost-session overlay on the HR and RR trend charts
  (layers toggle, `ghostSessionActive()`, `ghostPointCount()`) is in
  `live.component.html`.
- **Keyboard parity** — keyboard controls, shortcut actions, and
  `aria-label` coverage are present across the Live template.
- **Connection clarity** — the Publishing Policy card, Payload freshness
  indicator (LIVE / STALE / WAITING), and server-blocker screen in
  `live.component.html` surface connection state.
- **Placement guidance** — the Home Live Radar Scope Preview ("placement cues"
  subtitle, live `distance_cm` readout) and the Preflight Pipeline card provide
  placement feedback; sensor distance guidance is documented in
  `docs/operator-quickstart.md`.
- **SQI ribbons** — `pqi_breath` / `pqi_heart` progress bars with `qualityLabel`
  / `qualityPercent` and the `.sqi-ribbon` CSS are confirmed in the Waves tab.

### Track B — Firmware (PR-55 … PR-60)

- **PR-57 and PR-58 landed**: field diagnostics columns 208–219 (loop timing,
  heap health, radar UART/CRC error counters, I2C/LCD recovery counts, WDT near-
  miss count, command channel stats, firmware uptime) are implemented per the plan.
  Robustness and NVS forensics (exponential backoff for peripheral recovery,
  reset-reason ring in NVS via `esp_reset_reason()`, NVS write-failure escalation,
  radar UART staged escalation) are also landed.
- **PR-59 in flight**: power and thermal management (`RV_POWER_SAVE` gate,
  idle-state duty cycling, chip temperature telemetry, LCD backlight scheduling)
  is in progress.
- **PR-55 retired by policy**: the `.ino` header-tab split (moving cohesive
  sections into `.h` Arduino IDE tabs) is retired. No `.ino` file restructuring
  will proceed. The serial contract, `setup()`/`loop()` structure, and
  single-file layout are preserved as-is.

---

Date: 2026-06-11
Baseline: `main` @ PR #47 merged (OTA install, v16.2.0)
Scope: Two coordinated tracks of small, independently-mergeable PRs.

- **Track A — Dashboard UI/UX & feature improvements** (Angular 21 + Material 3, `web/src/`)
- **Track B — Firmware improvements** (`radar_vital_v16_2_0.ino`, XIAO ESP32-C6 + MR60BHA2)

Every PR in this plan honors the invariants in [`AGENTS.md`](../AGENTS.md):
the 207-column / 115 200-baud serial contract is **frozen** (additions only on the
right), no second UI framework, no externally hosted runtime dependencies,
per-mode CSP, HANDOFF.md updated in the same commit, and the full verification
protocol (build round-trip, trainer compileall, Playwright smoke/visual, APK/EXE
CI) green before merge.

---

## How to read this plan

| Field | Meaning |
|---|---|
| **Depends on** | PRs that must merge first. PRs without dependencies can land in any order. |
| **Risk** | Blast radius if it regresses: Low = view-local, Medium = cross-view/service, High = contract or hardware-coupled. |
| **Verification** | Required checks beyond the standard AGENTS.md protocol. |

Numbering continues the repository's PR sequence (last merged: #47; `codex/pr48-operator-profiles` exists on the remote, so this plan starts at PR-49).

### Sequencing overview

```
Track A (UI/UX)                       Track B (Firmware)
─────────────────                     ──────────────────
PR-49  Live decomposition + perf      PR-55  Hygiene & v17 debt prep
PR-50  A11y & i18n foundation         PR-56  Serial command channel (RX)
PR-51  Mobile ergonomics & haptics    PR-57  Telemetry quality additions
PR-52  Trends, compare & annotations  PR-58  Robustness: brownout/I2C/NVS
PR-53  Session workflow & reporting   PR-59  Power & thermal management
PR-54  Connection & status UX         PR-60  BLE Phase 4F activation (gated)
         │                                      ▲
         └── PR-54 unblocks ───────────────────┘ (dashboard BLE status surface)
```

PR-49 → PR-52 have an internal dependency chain; PR-50/51 are parallelizable
after PR-49. Track B is independent of Track A except PR-60, which should land
after PR-54 so the dashboard can display the BLE bridge state.

---

# Track A — Dashboard UI/UX & Feature Improvements

## PR-49 — Live view decomposition & render performance

**Problem.** `live.component.ts` is 1 536 lines and owns KPI cards, six chart
tabs, sparklines, thresholds, zen mode, snapshots, and 27 canvases in one
component. Change risk is concentrated, change detection is broad, and canvas
repaints are not centrally budgeted.

**Depends on:** none. **Risk:** Medium (view-local but largest view).

### Work items

1. **Component split (no behavior change).** Extract standalone child components
   under `web/src/app/components/live/`:
   - `kpi-strip` — the four KPI metric cards + sparklines.
   - `wave-charts` — Waves tab canvases (heart/breath phase, fused).
   - `trend-charts` — HR/RR trend tabs.
   - `snapshots-panel` — Snaps tab list + capture.
   - `audit-panel` — Audit tab table.
   - `live-toolbar` — trend-range selector, zen toggle, snapshot button.
   Each child takes signals/inputs from the parent; the parent keeps the
   telemetry subscription and tab state so SSE wiring stays single-owner.
2. **Canvas render budget.** Introduce one `requestAnimationFrame` scheduler
   service (`chart-scheduler.service.ts`) that coalesces all chart redraws into
   a single rAF tick, skips redraw for hidden tabs (`document.visibilityState`
   + active-tab gating), and caps sparkline repaint at 2 Hz. Today each canvas
   schedules independently.
3. **Bundle budget.** Address the standing advisory overage (12.79 kB above the
   2 MB initial-bundle warning noted in HANDOFF). Lazy-load `Report`, `Help`,
   and `Settings` routes (`loadComponent`), keeping Live/Home eager because
   they are the operational path. Re-measure and tighten
   `web/angular.json` budgets to lock in the gain.
4. **OnPush + signals audit.** Confirm every extracted child uses
   `ChangeDetectionStrategy.OnPush` and signal inputs; remove any residual
   zone-driven bindings in the Live tree.

### Acceptance criteria

- Zero visual diff: `npm run test:visual` passes against the **existing**
  baselines (no `--update-snapshots`).
- All Live keyboard shortcuts, zen mode, snapshot capture, threshold overlays,
  and tab persistence behave identically (covered by existing smoke specs;
  add specs for any uncovered shortcut before refactoring).
- Initial bundle ≤ 2 MB warning budget; `npm run build:check` clean.
- No child component exceeds 400 lines.

### Verification

Standard protocol + a new Playwright performance smoke: assert ≤ 1 chart
redraw per frame via an instrumented test hook on the scheduler, and assert
hidden-tab canvases receive no draw calls for 5 s.

---

## PR-50 — Accessibility hardening & i18n foundation

**Problem.** The Live template has good per-card ARIA coverage, but there is no
live-region strategy for KPI changes, no reduced-motion handling audit, and the
codebase has **zero** i18n scaffolding — all operator-facing strings are
hard-coded English.

**Depends on:** PR-49 (touches the extracted Live children). **Risk:** Low–Medium.

### Work items

1. **Screen-reader telemetry channel.** Add a polite `aria-live` region that
   announces KPI state *transitions* only (e.g., "Heart rate locked, 72 BPM",
   "Respiration signal lost") — never the 1 Hz stream. Debounce ≥ 10 s per
   metric; alert-severity changes announce assertively.
2. **Reduced-motion compliance.** Honor `prefers-reduced-motion`: disable the
   heart pulse animation, chart easing, sheet slide transitions, and scroll
   smoothing. Add a manual override in Settings (`Motion: system / full /
   reduced`) persisted alongside theme.
3. **Focus & contrast sweep.** Run an axe-core pass (added as a dev-only
   Playwright project) across all five routes × four themes. Fix findings;
   gate CI on zero serious/critical violations. Verify HC theme keeps ≥ 7:1
   for KPI numerals.
4. **i18n foundation (no translation yet).** Adopt Angular's built-in `$localize`
   / `i18n` attributes for all operator-facing template strings, extract
   `messages.xlf`, and wire `ng extract-i18n` into CI as a drift check. Runtime
   stays single-locale English; this PR only makes the catalog exist so a
   future locale is a translation task, not an engineering task. (Self-hosted,
   build-time — complies with the no-external-runtime-deps invariant.)
5. **Touch target audit.** Enforce ≥ 44 × 44 px on all interactive elements at
   phone widths; fix the known small targets (trend-range chips, alert filter
   chips, table row actions).

### Acceptance criteria

- axe-core: 0 serious/critical across 20 route×theme combinations.
- `ng extract-i18n` produces a stable catalog; CI fails if templates add
  untagged user-facing strings (lint script greps for bare text in new code).
- VoiceOver/TalkBack manual pass documented in `docs/` for the Live view.

---

## PR-51 — Mobile ergonomics & haptics

**Problem.** The app is mobile-first in layout but not yet in *feel*: haptics
exist in only a few places (22 references), there's no one-handed reach
consideration for the primary action, and pull-to-refresh / swipe gestures are
absent where operators expect them.

**Depends on:** PR-49. **Risk:** Low.

### Work items

1. **Haptics service.** Centralize vibration in a `haptics.service.ts` with
   semantic methods (`tick`, `success`, `warning`, `alarm`) mapping to
   `navigator.vibrate` patterns on web/APK and a no-op on Tauri. Wire to:
   session start/stop confirm, alert raise, snapshot capture, pairing success,
   preflight pass/fail. Respect a Settings toggle (default on for APK,
   off for browser).
2. **Swipe navigation between Live tabs.** Hammer.js is already self-hosted;
   add horizontal swipe to move across Overview/Waves/HR/RR/Snaps/Audit with
   the standard Material edge-resistance affordance. Must not conflict with
   horizontal chart pan — swipe only initiates from the tab-header zone or
   non-canvas whitespace.
3. **Thumb-reachable primary action.** On phones, float the contextual primary
   action (Start session on Home, Snapshot on Live, Stop on active session)
   as an extended FAB bottom-right, above the bottom nav, replacing
   topbar-only placement. Keyboard/desktop placement unchanged.
4. **Bottom-sheet pattern for phone dialogs.** Convert Alerts and KPI-zoom
   dialogs to `MatBottomSheet` below 600 px (dialog retained at ≥ 600 px).
   Sheets must keep the existing focus-trap/Escape/back-button contracts from
   AGENTS.md "done" criteria.
5. **Safe-area & display-cutout polish.** Audit `env(safe-area-inset-*)` usage
   for the FAB, bottom nav, and snackbars on notched devices; add Playwright
   viewport assertions for the iPhone 14 project.

### Acceptance criteria

- All gestures discoverable: first-run coach-marks (one-time, dismissible,
  stored per-scope in IndexedDB) for swipe and FAB.
- No gesture conflicts: chart pan, swipe-nav, and pull regions covered by
  Playwright touch tests on the Pixel 7 project.
- Visual baselines intentionally refreshed (FAB + sheet layouts) in the same PR.

---

## PR-52 — Trend history, session compare & annotations

**Problem.** Operators can see live trends and per-session reports, but cannot
overlay two sessions, mark moments during capture beyond snapshots, or view
long-horizon trends past the in-memory window.

**Depends on:** PR-49 (chart components), PR-51 (gesture model). **Risk:** Medium
(adds trainer API surface).

### Work items

1. **In-session annotations.** A one-tap "Mark" action (FAB long-press +
   keyboard `M`) records a timestamped annotation with optional quick-pick
   label (Movement / Talking / Position change / Custom…). Annotations render
   as vertical markers on all charts and export with the session.
   - Trainer: extend the existing notes route family with
     `POST /api/session/annotations` (protected, LAN-auth) appending to the
     session JSON; document in README; add Playwright API + pytest coverage.
2. **Session compare view.** In Report, allow selecting a second session of the
   same subject scope; render HR/RR trend overlays (current solid, comparison
   dashed) with aligned elapsed-time axes and a delta summary table
   (mean HR Δ, mean RR Δ, coverage Δ, ML-readiness verdict both).
   Read-only — no new mutation endpoints.
3. **Longer trend ranges.** Extend the Live trend-range selector with 8 h and
   session-max options backed by trainer-side decimated history
   (`GET /api/session/trend?bucket=10s`) instead of growing browser memory;
   the trainer already owns per-second history in the session store.
4. **Chart interaction parity.** Pinch-zoom (touch) and wheel-zoom (desktop)
   on trend charts with a double-tap/double-click reset, using the existing
   self-hosted Chart.js + Hammer.js stack only.

### Acceptance criteria

- Annotations survive trainer restart (persisted in the session record),
  appear in CSV/JSON export, and are scoped demo:/live: in offline cache.
- Compare view renders correctly with sessions of unequal duration and across
  themes; visual baselines added.
- Decimated trend endpoint returns ≤ 5 000 points for any range; payload
  documented in README API table.

---

## PR-53 — Session workflow & reporting upgrades

**Problem.** Sign-off, notes, and tags exist, but the operator journey around
them still has friction: no pre-session checklist enforcement summary, no
shift-handoff report, and exports require manual assembly for the audit binder.

**Depends on:** PR-52 (annotations feed reports). **Risk:** Medium.

### Work items

1. **Guided pre-session checklist.** Home gains an explicit checklist card that
   reflects preflight machine state (radar link, serial rate, LCD, MLX, scope,
   subject profile) with one-tap rerun per item (the single-check rerun
   API already exists) and a clear "Ready" gate animating into the Start button.
2. **Shift-handoff summary.** Extend the existing operator-handoff dialog into
   a generated handoff sheet: active session state, last 3 alerts, unsigned
   sessions count, pairing/TLS mode, trainer uptime, pending annotations.
   Printable via the existing print stylesheet.
3. **Audit-binder export.** One action exports a ZIP per session: 207-col raw
   CSV, session JSON (incl. annotations + sign-off), alert log, snapshots
   (PNG), and a rendered PDF summary. Assemble client-side (JSZip vendored
   under `assets/lib/` — self-hosted, complies with invariant 5); trainer
   adds nothing new except an existing-data manifest route if needed.
4. **Unsigned-session nudges.** Badge count on the Report nav icon for
   completed-but-unsigned sessions in the active scope; snackbar nudge on
   session stop with a "Sign now" action.

### Acceptance criteria

- Checklist state is purely derived from existing preflight APIs (no new
  trainer mutation surface).
- Export ZIP opens on Windows/Android; PDF renders all four themes' print
  styles correctly; smoke test downloads and inspects the ZIP manifest.
- Unsigned badge updates live via the existing SSE event stream.

---

## PR-54 — Connection, pairing & status UX

**Problem.** Pairing and connection states are functional but terse: operators
see disconnects without progressive context, the SSE 12-hour deadline handling
is invisible until it happens, and BLE (future) has no status surface.

**Depends on:** none (parallelizable). **Risk:** Medium (touches reconnect driver).

### Work items

1. **Connection status center.** A persistent, compact status chip in the
   topbar (color + icon) expanding to a panel: transport (SSE/poll), latency,
   last event age, auth mode (local / LAN-paired / native-pinned), TLS state,
   SSE session age vs the 12 h deadline with a countdown once < 30 min.
2. **Graceful SSE deadline UX.** On the contractual `session_warning`
   (`seconds_remaining: 60`), show a non-modal "Stream renewing in 60 s"
   indicator, pre-warm the replacement EventSource, and swap without UI gap;
   log the renewal in the status panel history. (The reconnect driver already
   handles the event — this PR makes it *visible* and pre-warmed.)
3. **Pairing flow polish.** QR-scan path gets a live viewfinder overlay with
   corner guides and explicit camera-permission recovery copy; manual PIN
   entry gets segmented 6-digit boxes with paste support; cooldown state shows
   a countdown instead of a bare error.
4. **Offline-first messaging.** When the trainer is unreachable, the shell
   states *which* origin failed, last-success time, and a one-tap retry +
   "Switch to demo" pair of actions — replacing the current generic recovery
   overlay copy.
5. **BLE status placeholder surface.** A status-center row for the firmware
   BLE bridge (Hidden / Disabled / Advertising / Connected) driven by trainer
   `/api/status` fields, so PR-60 only has to light it up.

### Acceptance criteria

- SSE renewal produces zero missed events in a 13 h mock-clock Playwright test
  (trainer mock emits the warning; client swap verified by sequence numbers).
- All pairing error states have distinct, actionable copy; cooldown countdown
  matches the trainer's one-minute lockout.
- Status center is keyboard-accessible and announced as a landmark.

---

# Track B — Firmware Improvement Plan (`radar_vital_v16_2_0.ino`)

The serial contract is frozen: **207 columns, 115 200 baud, additions only on
the right.** Each firmware PR ships an Arduino CLI compile for
`esp32:esp32:XIAO_ESP32C6` with `ENABLE_BLE` both off and on, and bench notes
in `docs/physical-acceptance-checklist.md` where hardware behavior changes.

## PR-55 — Hygiene, structure & v17 debt preparation (no behavior change)

**Problem.** The sketch is 7 078 lines in one file with declared-but-deferred
debt: the `RLS_LAMBDA_HR` alias is tagged "REMOVE at v17", the BLE control
callback is a stub, and section navigation relies on banner comments.

**Risk:** Low. **Depends on:** none.

### Work items

1. **Header tab split (compile-identical).** Move cohesive sections into `.h`
   files in the sketch folder (Arduino IDE tabs — still one translation unit,
   no build-system change): `rv_pins.h`, `rv_buzzer.h`, `rv_lcd.h`, `rv_dsp.h`,
   `rv_presence.h`, `rv_csv.h`, `rv_ble.h`. The `.ino` keeps `setup()`/`loop()`
   and the include ladder. Verify with a binary-size and `objdump` symbol diff
   that codegen is unchanged.
2. **v17 debt sweep, staged behind no-ops.**
   - Add `#define RV_V17_PREP 0`; when 0, everything compiles exactly as today.
   - Mark every `RLS_LAMBDA_HR` use site with a one-line comment naming its
     v17 replacement so the v17 removal is mechanical.
   - Promote the deprecated `CHIP_HR_BIAS_CORRECTION_BPM` changelog comment
     into a single canonical "deprecated constants" block.
3. **Compile-time contract guards.** Add `static_assert`-style checks
   (via `char[1 - 2*(cond)]` where C++11 `static_assert` is available — it is
   on ESP32 Arduino): `CSV_COLUMN_COUNT == 207`, buffer sizes vs window
   constants, buzzer pattern table length vs enum count. The existing
   *runtime* `_csvColCount` check stays as the second line of defense.
4. **`delay()` audit.** Eight `delay()` sites exist. Classify each: boot-time
   (acceptable), I2C recovery (acceptable if < watchdog margin), DSP-loop
   adjacent (must convert to `millis()` deadlines per the AGENTS.md watchdog
   rule). Convert any in the third class; document the rest inline as
   `// delay OK: setup-only, wdt not yet armed`.

### Acceptance criteria

- `arduino-cli compile` binary diff: identical `.text` size ±0 bytes with
  `RV_V17_PREP 0` (comment/structure-only change).
- All static contract tests in `tests/test_v12_static_contract.py` still pass
  (they grep the firmware file — update path-based greps to scan the sketch
  folder recursively in the same PR).

---

## PR-56 — Serial command channel (trainer → firmware RX path)

**Problem.** The firmware is transmit-only over USB. Every tuning change
(thresholds, buzzer enable, display mode) requires a reflash. The trainer
already has a serial write handle; the firmware never reads it
(`Serial.read` appears only for the radar UART flush).

**Risk:** Medium-High (new input surface on the frozen link — TX contract
untouched, RX is new). **Depends on:** PR-55 (lands in `rv_cmd.h`).

### Work items

1. **Line-oriented command parser.** Non-blocking read in `loop()` (bounded:
   ≤ 64 bytes/iteration, 96-byte line buffer, drop-on-overflow with a
   `[CMD] overflow` log). Commands are `$CMD,<verb>[,args]*<CRC8>\n`:
   - `$CMD,PING` → `[CMD] PONG <FW_VERSION>`
   - `$CMD,BUZZER,0|1` — runtime buzzer mute (NVS-persisted)
   - `$CMD,LCDMODE,0|1|2` — vitals / diagnostics / off
   - `$CMD,HRALERT,<lo>,<hi>` — alert thresholds (validated 30–200, NVS)
   - `$CMD,RESETVITALS` — calls existing `resetVitals()`
   - `$CMD,IDENT` — one-shot identity block (sketch + module versions)
   - Unknown verb → `[CMD] ERR unknown` (never silent).
2. **Safety rails.** Commands never touch DSP constants in v16 (tuning stays
   compile-time); rate-limit to 5 commands/s; CRC8 required — bad CRC logs and
   drops; all responses go through the existing log channel so the 207-column
   DATA stream is never interleaved mid-row (emit responses only at row
   boundaries via a pending-response queue).
3. **Trainer integration.** `rvt_trainer` gains
   `POST /api/device/command` (LAN-auth protected, allowlisted verbs only) and
   surfaces `PONG`/`IDENT` in `/api/status`. Dashboard Settings gains a
   "Device" card (buzzer mute, LCD mode, HR alert bounds) — feature-flagged on
   the trainer reporting command-channel support, so old firmware degrades
   gracefully (PING timeout ⇒ card hidden).
4. **Contract tests.** pytest: verb allowlist, CRC validation, row-boundary
   interleaving (assert no DATA row is ever split). Static contract: DATA
   column count untouched.

### Acceptance criteria

- A DATA stream captured during a 1 000-command soak shows zero malformed rows.
- Firmware with no trainer attached behaves exactly as v16.2.0 (parser idles).
- All commands idempotent and safe during active capture.

---

## PR-57 — Telemetry quality additions (columns 208+)

**Problem.** Diagnosing field issues still requires bench access for some
signals: loop timing health, heap headroom, radar UART error counts, I2C bus
recovery counts, and command-channel stats are not in the CSV.

**Risk:** Low (right-side additions are contractual). **Depends on:** PR-56
(command stats column), PR-55 (csv tab).

### Work items

1. **Append columns 208–219** (names final before merge, schema bumped to
   v15.1.0 in the header comment and `/api/help/schema`):
   - `loop_dt_mean_ms`, `loop_dt_max_ms` (1 s window)
   - `heap_free_kb`, `heap_min_free_kb`
   - `radar_uart_overflow_count`, `radar_crc_err_count`
   - `i2c_recover_count`, `lcd_reinit_count`
   - `wdt_near_miss_count` (loop iterations > 50 % of the 8 s timeout)
   - `cmd_rx_count`, `cmd_err_count`
   - `fw_uptime_s`
2. **Update both counters.** `CSV_COLUMN_COUNT` 207 → 219, runtime
   `_csvColCount` assert and the compile-time guard from PR-55 updated
   together; trainer parser extends its column map additively and tolerates
   207-column rows from older firmware (version-gated by the sketch identity
   columns already in the stream).
3. **Dashboard surfacing.** New columns flow to the Live Audit tab
   diagnostics table and the connection status center (PR-54) — heap and loop
   health get warn thresholds with alert-log entries.

### Acceptance criteria

- Old trainer + new firmware: trainer logs a schema-width notice and continues
  parsing the first 207 columns (explicitly tested).
- New trainer + old firmware: zero warnings, full function.
- pytest schema tests updated in the same PR; README column-count references
  updated everywhere (`README.md`, `AGENTS.md` quote the number).

---

## PR-58 — Robustness: I2C storm recovery, NVS wear, brownout telemetry

**Problem.** Single-fault recovery exists (i2cRecover, lcdReInit, radar
watchdog), but repeated-fault behavior is unbounded: an LCD that fails
persistently re-probes forever at full rate, NVS writes are time-throttled
(10 min) but not wear-leveled or failure-escalated, and brownouts/resets leave
no trace in telemetry.

**Risk:** Medium. **Depends on:** PR-57 (uses the new counters).

### Work items

1. **Exponential backoff for peripheral recovery.** I2C/LCD/MLX re-probe
   intervals back off 3 s → 6 s → 12 s → … capped at 5 min after consecutive
   failures, reset on success. Prevents a dead LCD from stealing I2C bus time
   from the MLX path every cycle.
2. **Reset-reason forensics.** On boot, read `esp_reset_reason()` and the
   brownout detector status; print a `[BOOT] reset_reason=<n> (<name>)` line
   and persist a small ring (last 8 reasons + uptime-at-reset) in NVS;
   expose the latest via the `$CMD,IDENT` response and a future CSV column.
3. **NVS hygiene.** Track write-failure streaks; after 3 consecutive
   `putFloat` failures, attempt `nvs_flash_erase`-free namespace re-open once,
   then stop retrying for the boot (log loudly). Add a daily write-count log
   line so wear is observable over long deployments.
4. **Radar UART resync hardening.** After the existing silence watchdog fires,
   add a staged escalation: flush+regrace (today) → module soft-command reinit
   → log `[RADAR] escalation exhausted` with the diagnosis snapshot. Each
   stage increments the PR-57 counters.

### Acceptance criteria

- Bench test in `docs/physical-acceptance-checklist.md`: disconnect LCD SDA
  mid-session → DATA stream cadence unaffected (±5 ms loop budget), backoff
  observable in logs; reconnect → recovery within one backoff interval.
- Pull radar TX mid-session → staged escalation visible; restore → lock
  re-acquired without reboot.
- Power-cycle test shows reset-reason ring persisting across 8 boots.

---

## PR-59 — Power & thermal management

**Problem.** The C6 runs full-tilt continuously. For battery-backed or
enclosure-constrained deployments there is no idle power reduction, and chip
temperature is never observed even though sustained enclosure heat shifts
radar and MLX accuracy.

**Risk:** Medium (must not disturb DSP timing). **Depends on:** PR-57.

### Work items

1. **Idle-state duty cycling (presence-gated, default off).**
   `#define RV_POWER_SAVE 0` gate. When enabled and the presence FSM has been
   `ABSENT` ≥ 60 s: drop LCD backlight, reduce LED duty, lower MLX poll rate
   to 1/10, and call `setCpuFrequencyMhz(80)`; restore instantly (< 1 frame)
   on the first presence vote. The radar UART and DSP path are **never**
   slowed — only ancillary peripherals. Document measured mA deltas.
2. **Chip temperature telemetry.** Read the ESP32-C6 internal temperature
   sensor each 10 s; reserve a CSV column in the next schema bump (with
   PR-57's block if sequenced together) and emit a `[THERMAL]` log warning
   above 75 °C.
3. **LCD backlight schedule.** Honor the BH1750 lux input that already exists
   for LED brightness: dim the LCD backlight in dark rooms (PWM if the carrier
   wiring allows; otherwise binary at a lux threshold) with hysteresis to
   avoid flicker.

### Acceptance criteria

- With `RV_POWER_SAVE 1`: measured idle current reduction documented;
  presence reacquisition latency unchanged vs v16.2.0 (bench-timed).
- With the default `RV_POWER_SAVE 0`: binary behavior identical (size/symbol
  diff as in PR-55).
- No change in DATA cadence (1 Hz ±2 %) in either mode across a 2 h soak.

---

## PR-60 — BLE Phase 4F activation (gated, last)

**Problem.** The NimBLE GATT bridge is fully scaffolded behind
`ENABLE_BLE false` (service UUIDs, vitals/phase/control characteristics,
DIS/HRS) but the control callback is a stub and physical GATT acceptance has
never run — it is the standing release gate noted in HANDOFF.

**Risk:** High (radio + DSP coexistence on one core). **Depends on:** PR-56
(shared command verb table), PR-54 (dashboard status surface), and the
physical acceptance checklist hardware session.

### Work items

1. **Control characteristic — real implementation.** Replace the stub with the
   PR-56 verb table (same allowlist, same validation, same rate limit) so
   serial and BLE control share one code path (`rv_cmd.h` gains a transport
   parameter). Writes remain `WRITE_ENC | WRITE_AUTHEN` only.
2. **Coexistence guard hardening.** Audit every BLE notify site for the
   AGENTS.md rule: no `delay()` in the DSP loop — use `bleSuppressUntilMs`
   deadlines. Add a measured guard: if a BLE notify burst pushes
   `loop_dt_max_ms` (PR-57) beyond 25 ms, auto-throttle phase-frame notifies
   from 10 Hz to 2 Hz and log `[BLE] throttled`.
3. **GATT acceptance suite.** Execute and check off the physical acceptance
   checklist: pairing/bonding flow, MTU 247 negotiation, vitals notify
   cadence, phase-frame integrity (sequence-numbered, gap detection),
   control-write round-trip, and a 4 h dual-path soak (serial DATA + BLE
   notifies simultaneously) with zero watchdog resets and zero malformed CSV
   rows. Results recorded in `docs/physical-acceptance-checklist.md`.
4. **Flip the default — separate, trivially-revertable commit.** Only after
   the suite is green: `ENABLE_BLE` default → `true` in a one-line commit at
   the top of the PR's final stack, so a field regression reverts one line.
5. **Trainer + dashboard lights-on.** Trainer reports BLE bridge state in
   `/api/status`; the PR-54 status row goes live; the Home native BLE
   acceptance probe gains a radar-firmware profile option alongside the
   existing AiLink path (per the documented gating in README).

### Acceptance criteria

- 4 h soak: 0 WDT resets, 0 CSV contract violations, BLE notify delivery
  ≥ 99 % (sequence-gap audit), serial DATA cadence 1 Hz ±2 % throughout.
- With BLE enabled but no central connected: ≤ 1 ms added `loop_dt_mean_ms`.
- Revert path proven: reverting the default-flip commit restores
  v16-equivalent behavior (CI compile both ways stays in the matrix forever).

---

# Cross-cutting policies for every PR in this plan

1. **HANDOFF.md** — dated entry at the top in the same commit (AGENTS.md
   mandate).
2. **Branching** — one branch per PR, named `codex/pr<NN>-<slug>` to match
   repository convention; PRs target `main`; never push `main` or `archive/*`.
3. **Verification floor** — `npm --prefix web ci && npm run build:check`,
   `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer`,
   `python -m rvt_trainer --help`, `npm test`, visual regression (refresh only
   for intentional UI changes), APK/EXE/Pages CI green. Firmware PRs add the
   Arduino CLI compile matrix (`ENABLE_BLE` off/on; `RV_POWER_SAVE` off/on
   after PR-59).
4. **Schema discipline** — any CSV column addition bumps the schema version in
   the firmware header, `/api/help/schema`, the trainer column map, and the
   static contract tests in one PR. Never split a schema change across PRs.
5. **Demo/live scoping** — every new persisted dashboard artifact (coach-mark
   flags, annotations cache, compare selections) uses the existing
   `demo:`/`live:` IndexedDB scope prefixes.
6. **No marketing model names** in commits, PR bodies, comments, or docs
   (AGENTS.md invariant 10).

# Suggested milestone grouping

| Milestone | PRs | Theme |
|---|---|---|
| M1 — Foundation | PR-49, PR-55 | Decomposition & hygiene, zero behavior change |
| M2 — Operator feel | PR-50, PR-51, PR-56 | A11y, mobile ergonomics, device control |
| M3 — Insight | PR-52, PR-57, PR-54 | Trends/compare, telemetry, connection UX |
| M4 — Hardening | PR-53, PR-58, PR-59 | Workflow, robustness, power |
| M5 — Radio | PR-60 | BLE Phase 4F (release-gated) |
