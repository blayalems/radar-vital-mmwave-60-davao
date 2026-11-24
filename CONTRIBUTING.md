# Contributing to Radar Vital

Radar Vital is a University of Mindanao BS Electronics Engineering thesis
project distributed under the **Radar Vital — Academic Evaluation License**
([`LICENSE`](./LICENSE)).

- **Permitted under `LICENSE`**: viewing, building, and running this software
  for academic evaluation, thesis review, research demonstration, and personal
  study.
- **Requires prior written permission from the copyright holders**:
  redistribution of the software or substantial portions of it, commercial use,
  publication of derivative works, and use of the "Radar Vital" name or the
  authors' names to endorse derived products.
- **Contribution boundaries**: issue reports, defect reproductions, and review
  discussions are welcome from evaluators; modifications to the repository,
  shipped builds, or controlled documents require author authorization and must
  follow [`AGENTS.md`](./AGENTS.md) and [`quality/qms-policy.md`](./quality/qms-policy.md).

## Repository shape

| Part | Where | Build |
|---|---|---|
| Dashboard (source of truth) | `web/src/` | `npm --prefix web ci && npm run build:web` (regenerates the committed monolith + `www/`) |
| Trainer | `rvt_trainer/` (+ root shim) | `pip install -r requirements.txt` or `pip install -e .` |
| Firmware | `radar_vital.ino` | Pinned toolchain in `packaging/firmware/arduino-toolchain.json`, `esp32:esp32:XIAO_ESP32C6` |
| Windows EXE | `src-tauri/` | `npm run tauri:build` |
| Android APK | Capacitor (generated `android/`) | `npm run cap:sync` |

## Verification protocol (every change)

```bash
python -m pytest -q tests
npm --prefix web run test:ci
npm run build:web && npm run build:check
npm test                       # Playwright smoke, 4 viewports
npm run lint && npm run typecheck && npm run lint:py && npm run typecheck:py
npm run test:version-contract
npm run test:qms-contract -- --base-ref <PR-base>
python -m compileall -q radar_vital_trainer.py rvt_trainer
```

Visual baselines are Windows-generated; after intentional UI changes dispatch
the **Refresh visual baselines (Windows)** workflow instead of committing
Linux/macOS screenshots.

## Rules that are easy to trip over

- The serial CSV contract is frozen at v15.2 / 222 columns; the original 207-column prefix and v15.1 219-column replay compatibility must remain intact. Additions are right-side only.
- The root dashboard `.html` is a **build artifact** — never edit by hand.
- Every commit adds a dated entry at the top of `HANDOFF.md`.
- New trainer API routes are registered before the static fall-through,
  documented in README's API table, and covered by a Playwright API test.
- User-visible features ship with tests (pytest / vitest / Playwright).

## Reporting issues

Use the issue forms (Bug report / Feature request). The app's
**Settings → Report an issue** card pre-fills the bug form with diagnostics
you can review before submitting.
