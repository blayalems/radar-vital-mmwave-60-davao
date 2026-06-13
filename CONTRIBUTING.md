# Contributing to Radar Vital

Radar Vital is a University of Mindanao BS Electronics Engineering thesis
project. Contributions are welcome for review and discussion; note the
[LICENSE](./LICENSE) terms and that agents/humans must follow
[AGENTS.md](./AGENTS.md).

## Repository shape

| Part | Where | Build |
|---|---|---|
| Dashboard (source of truth) | `web/src/` | `npm --prefix web ci && npm run build:web` (regenerates the committed monolith + `www/`) |
| Trainer | `rvt_trainer/` (+ root shim) | `pip install -r requirements-v12.txt` or `pip install -e .` |
| Firmware | `radar_vital_v16_2_0.ino` | Arduino CLI, `esp32:esp32:XIAO_ESP32C6` |
| Windows EXE | `src-tauri/` | `npm run tauri:build` |
| Android APK | Capacitor (generated `android/`) | `npm run cap:sync` |

## Verification protocol (every change)

```bash
python -m pytest -q tests
npm --prefix web run test:ci
npm run build:web && npm run build:check
npm test                       # Playwright smoke, 4 viewports
python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer
```

Visual baselines are Windows-generated; after intentional UI changes dispatch
the **Refresh visual baselines (Windows)** workflow instead of committing
Linux/macOS screenshots.

## Rules that are easy to trip over

- The serial CSV contract is frozen (219 columns; additions right-side only).
- The root dashboard `.html` is a **build artifact** — never edit by hand.
- Every commit adds a dated entry at the top of `HANDOFF.md`.
- New trainer API routes are registered before the static fall-through,
  documented in README's API table, and covered by a Playwright API test.
- User-visible features ship with tests (pytest / vitest / Playwright).

## Reporting issues

Use the issue forms (Bug report / Feature request). The app's
**Settings → Report an issue** card pre-fills the bug form with diagnostics
you can review before submitting.
