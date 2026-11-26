# Radar Vital v16.9 Thesis Readiness Roadmap (PR #171–PR #175)

Date: 2026-11-25  
Status: active engineering roadmap; no research or release authorization  
Baseline: PR #170 (`v16.9.0`)

This roadmap extends `docs/v16-6-thesis-readiness-roadmap.md` for post-PR #170 delivery toward UMERC ethics readiness, hardware acceptance, confirmatory data lock, and thesis manuscript regeneration (`docs/manuscript/manuscript.tex`). All frozen protocol contracts (including the 222-column serial CSV contract and fail-closed `quality/precollection-readiness.json` gating) remain binding across every slice.

## Release Sequence Summary

| PR | Version | Focus | Key Deliverables | Thesis Traceability | Dependencies | Verification & Abort Criteria | External UMERC / Hardware Gates |
|---|---|---|---|---|---|---|---|
| **PR #171** | `v16.10.0` | **Frontend Consistency, 1 Hz Render Perf & CSS Consolidation** | Migrate remaining Live, Home, Report, and strip components to `ChangeDetectionStrategy.OnPush` with memoized `computed()` signals; enforce canvas redraw throttling via `ChartRenderSchedulerService`; unify duplicated styles across `rvt-consolidated-css.css`, `rvt-v12-mobile-pwa-css.css`, and component stylesheets across all four themes; replace filler copy with concise triage microcopy. | `RVT-UI-001`, `RVT-THESIS-003` (GUI stability during continuous monitoring), `RVT-DOC-001` | PR #170 (`v16.9.0`) | `npm run test:unit:web`, `npm run build:check`, Playwright smoke suite, and 4-theme visual regression. **Abort/Defer**: If visual regression fails twice on a CSS slice, isolate or defer that CSS slice past thesis bench lock. | Physical Pixel touch and edge-to-edge verification (`docs/physical-acceptance-checklist.md` Phase G) remains an external sign-off. |
| **PR #172** | `v16.11.0` | **Physical Latency Bench, Sync Provenance & 8-Hr Burn-In Auditor** | Add `rvt-trainer latency-bench` CLI producing hashed `latency_characterization.json` (USB/radar/reference offset, jitter, numeric uncertainty in seconds, SHA-256); validate `sync_anchors.json` and `alignment_report.json` linkage; add `rvt-trainer burnin-audit` validating 30-min isolated soaks and 8-hr continuous `1 Hz ± 2%` logs (Phase J); allow authorized 150 s session-backed `no_subject` start (`participant_id: null`). | `RVT-THESIS-003` (soaks and 8-hr burn-in), `RVT-THESIS-004` (`objective_3_false_alarm`, `objective_4_hr` timing gate), `RVT-RES-001` | PR #171 (`v16.10.0`) | `pytest` coverage for latency schema, 8-hr burn-in pass/fail fixtures, `412` blocking on unauthorized `no_subject` start, and unchanged 222-column contract. **Abort**: Stop if any code assumes a timing offset without bench evidence or weakens `412` gates. | **Hard Physical Gates**: KiCad PCB continuity (`RVT-THESIS-001`/`002`), physical USB/radar/oximeter bench latency run, 30-min subsystem soaks, and 8-hr continuous burn-in (Checklist Phases D/E/H/I/J). |
| **PR #173** | `v16.12.0` | **Frozen GBR C-Branch Codegen & `objective_2_temperature` Pipeline** | Add `rvt-trainer export-gbr-c` generating deterministic C99 branch-comparison code from signed GBR bundles, host C parity verification (`<= 1e-5` delta), and ESP32-C6 flash/RAM/latency profiler (`gbr_c_parity_report.json`); implement `objective_2_temperature` (`d060_none`) repeated-measures Bland-Altman/RMSE/MAE/missingness evaluator in `study_evidence.py` and `statistics.py`; enforce explicit `"cnn_unavailable_insufficient_data"` stopping rule. | `RVT-THESIS-001`, `RVT-THESIS-003` (embedded GBR inference), `RVT-THESIS-004` (`objective_1_rr`, `objective_2_temperature`, `objective_4_hr`), `RVT-ML-001`, `RVT-STA-001` | PR #172 (`v16.11.0`) | Python-vs-C99 parity tests; `objective_2_temperature` rejection of `cardboard` or `distance_m != 0.6`; CNN data-floor enforcement test. **Abort**: Never deploy CNN on ESP32-C6 or tune GBR after outer-OOF inspection; keep `radar_vital.ino` 222-column contract frozen. | Flashing updated frozen GBR C-branches into `radar_vital.ino` requires Phase J soak re-signoff; confirmatory GBR training remains gated on UMERC human data. |
| **PR #174** | `v16.13.0` | **Pre-Collection Authorization Tooling, Withdrawal SOP & Native Release Hardening** | Add `rvt-trainer authorize-check` verifying all four role approvals (`research_lead`, `quality_manager`, `REC_or_ethics_authority`, `privacy_reviewer`), `statistical-analysis-plan.json` approval, withdrawal SOP reference, and `latency_characterization_sha256` (checked-in repository state remains `blocked`); add withdrawal reconciliation CLI/UI status without false crypto-shredding claims; harden Capacitor APK and Tauri EXE LAN/BLE smoke coverage. | `RVT-RES-001`, `RVT-SES-001`, `RVT-SES-003`, `RVT-SEC-003`, `RVT-REL-001` | PR #173 (`v16.12.0`) | `test_precollection_readiness.py`, `test_collection_authorization.py`, `test_qms_release_record.py`, security audit, APK build, and EXE build. **Abort**: Reject any commit flipping checked-in readiness or plan state to `ready`/`approved` without dated external evidence or weakening native signing. | **Hard Governance Gates**: UMERC ethics approval, Privacy Reviewer withdrawal disposition SOP, Research Lead and Quality Manager sign-off on `RVT-STA-PLAN-16.5.8`, and signed physical acceptance checklist (Phases A–J) prior to recruitment. |
| **PR #175** | `v16.14.0` | **Confirmatory Data-Lock Bundle & Manuscript Table/Figure Regeneration** | Add `rvt-trainer confirmatory-bundle` producing `confirmatory_data_lock.json` (SHA-256 bound) and running all four endpoints (`objective_1_rr` TOST + 9 Holm secondary, `objective_2_temperature` `d060_none`, `objective_3_false_alarm` 72-trial Clopper-Pearson, `objective_4_hr` 10-condition Bland-Altman/RMSE/MAE); export `build/manuscript-tables/*.tex` and `build/manuscript-figures/` (accuracy/coverage vs distance and barrier, Bland-Altman plots, GBR vs CNN outer-OOF) for `docs/manuscript/manuscript.tex`. | Full closure of `RVT-THESIS-001`–`004` (`manuscript.tex` Results, Discussion, Reproducibility Appendix), `RVT-DOC-001`, `RVT-STA-001` | PR #174 (`v16.13.0`) + completed collection | Confirmatory `test_statistical_analysis.py` (`>= 19` primary estimates, `>= 38` complete participants, `72` unique non-overlapping no-subject captures), `docs:export-feedback-loop -- --check`, and QMS release record. **Abort**: Emit `"inconclusive"` if `< 19` primary estimates or `< 38` complete participants; withhold CNN if data floor is unmet; block `objective_4_hr` timing claims if latency bench evidence is absent. | **Hard Post-Collection Gates**: 40 recruited / `>= 38` protocol-complete participants (`>= 1,140` trials), `72` qualified no-subject trials (`10,800 s`), dual-observer RR adjudication on all eligible trials, reconciled withdrawal tombstones, and human confirmatory data lock. |

## Detailed Slice Specifications

### PR #171 (`v16.10.0`) — Frontend Consistency, 1 Hz Render Perf & CSS Consolidation
- **Objective**: Keep the UI deterministic at 1 Hz during multi-hour monitoring and 8-hour burn-in runs across desktop, tablet, and mobile viewports in `light`, `dark`, `night`, and `hc` themes.
- **Scope**:
  - Complete `ChangeDetectionStrategy.OnPush` and memoized `computed()` migration across shell, Home, Live, and Report surfaces so 1 Hz telemetry updates do not reallocate arrays or re-run DOM scans (`querySelectorAll`).
  - Route all live and report canvases through `ChartRenderSchedulerService` with `ResizeObserver` visibility gating.
  - Consolidate duplicated legacy stylesheets (`rvt-consolidated-css.css`, `legacy-patches.css`, `rvt-v12-mobile-pwa-css.css`) without regressing existing smoke or visual regression baselines.

### PR #172 (`v16.11.0`) — Physical Latency Bench, Sync Provenance & 8-Hr Burn-In Auditor
- **Objective**: Supply quantitative hardware timing and long-duration soak evidence required before `objective_4_hr` and `objective_3_false_alarm` confirmatory evaluation.
- **Scope**:
  - `rvt-trainer latency-bench`: Measures and hashes end-to-end USB serial, radar processing, and BLE reference latency offsets and jitter into `latency_characterization.json`.
  - `rvt-trainer burnin-audit`: Audits 30-minute isolated subsystem soaks and 8-hour continuous `1 Hz ± 2%` logs for frame drops, UART overflows, watchdog near-misses, and heap stability.
  - Enable session-backed `150 s` `no_subject` start (`participant_id: null`) strictly behind collection authorization (`412` when unauthorized).

### PR #173 (`v16.12.0`) — Frozen GBR C-Branch Codegen & `objective_2_temperature` Pipeline
- **Objective**: Bridge offline signed GBR bundles to embedded ESP32-C6 inference verification while enforcing statistical stopping rules.
- **Scope**:
  - `rvt-trainer export-gbr-c`: Emits deterministic C99 decision-tree branch code from signed GBR artifacts and verifies Python-vs-C99 parity within `1e-5` alongside flash/RAM/latency profiling (`gbr_c_parity_report.json`).
  - `objective_2_temperature`: Evaluates repeated-measures agreement exclusively on `d060_none` trials, rejecting `cardboard` or `distance_m != 0.6` observations.
  - CNN stopping rule: Explicitly emits `"cnn_unavailable_insufficient_data"` when predeclared sample floors are not satisfied.

### PR #174 (`v16.13.0`) — Pre-Collection Authorization Tooling, Withdrawal SOP & Native Release Hardening
- **Objective**: Enforce external governance prerequisites programmatically while keeping checked-in repository state fail-closed.
- **Scope**:
  - `rvt-trainer authorize-check`: Validates all four independent role sign-offs, statistical analysis plan approval (`RVT-STA-PLAN-16.5.8`), privacy withdrawal disposition SOP reference, and `latency_characterization_sha256`.
  - Withdrawal reconciliation status surfaced in CLI and dashboard without making unsupported cryptographic erasure claims.
  - Capacitor Android APK and Tauri Windows EXE LAN/BLE smoke verification under release signing policies.

### PR #175 (`v16.14.0`) — Confirmatory Data-Lock Bundle & Manuscript Table/Figure Regeneration
- **Objective**: Generate immutable, SHA-256 bound confirmatory statistical tables and publication figures directly for `docs/manuscript/manuscript.tex`.
- **Scope**:
  - `rvt-trainer confirmatory-bundle`: Computes TOST + Holm-Bonferroni (`objective_1_rr`), line-of-sight thermal agreement (`objective_2_temperature`), Clopper-Pearson false-alarm rate (`objective_3_false_alarm`), and 10-condition exploratory HR performance (`objective_4_hr`).
  - Exports `build/manuscript-tables/*.tex` and `build/manuscript-figures/` using the exact condition-stratified IEEE LaTeX schema introduced in PR #170.
  - Any unmet sample floor (`< 19` primary estimates, `< 38` protocol-complete participants, `< 72` unique non-overlapping no-subject captures) or missing bench latency artifact fails closed to `"inconclusive"`.

## Governance & Merge Boundary

- Every PR increments the product version once per PR in lockstep with `quality/document-register.json` and `HANDOFF.md`.
- No PR modifies the frozen 222-column serial CSV contract or weakens `quality/precollection-readiness.json` fail-closed defaults.
- All physical bench runs, UMERC ethics approvals, participant recruitment, and confirmatory data lock remain external human-gated milestones.
