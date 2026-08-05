# Radar Vital v16.5 High-Yield Engineering Roadmap

Date: 2026-08-03
Audit base: integrated `main` at `a35da6e`
Release identity: firmware, trainer, dashboard, APK, and EXE `16.5.10` work in progress
Wire compatibility: v15.2 CSV, 222 columns, 207/219-column replay support

## 2026-08-03 audit disposition

The attached 2026-08-01 implementation plan was audited against the repository
and accepted as the governing direction, with its status table corrected here:
PR79 and PR93–PR100 are merged; the integrated main commit is the v16.5.8
baseline. The next controlled patch is v16.5.10, continuing the versioned
statistical-analysis plan, participant-balanced RMSE/TOST/agreement/coverage,
outer-OOF prediction retention, and JSON/CSV/LaTeX exports. The confirmatory
design is 0.6/0.8/1.0 m × none/cardboard × three 150-second trials, with the
1.0 m/no-cardboard RR TOST as primary and five Holm-adjusted secondary tests.
Hardware acceptance is recorded as project-team reported until the controlled
checklist contains tester, date, device IDs, flashed SHA, log hashes, results,
and authorization. Local thesis/manual PDFs remain outside this repository.

This roadmap ranks the hardest changes by long-term safety, reproducibility,
and maintenance yield. Each slice is a separate reviewable pull request built
on the accepted predecessor. Every commit updates `HANDOFF.md` and
`docs/system-feedback-loop.md` when its data or modeling path changes.

## Delivery order

| Order | Pull-request slice | Why it precedes later work | Acceptance evidence |
|---:|---|---|---|
| 1 | Coordinated v16.5 identity and version graph | Every successor must inherit one product/firmware identity without relabelling the frozen serial schema. | Cross-language version checker; clean Angular/trainer/firmware/static contracts; regenerated dashboard |
| 2 | Segment-safe route matching | The current glob matcher lets `*` consume `/`, so malformed nested DELETE paths can reach the whole-session delete handler. | Template compiler uses one segment per parameter; ambiguity checks; nested/encoded/trailing-slash black-box tests |
| 3 | Durable authentication attempt state | Repeated profile-store failures can discard in-memory PIN/recovery attempts and prevent lockout. | Five failures from the first write still lock; atomic store only; concurrent/crash/permission tests |
| 4 | Leakage-safe dual-model experiment and statistics core | GBR and experimental 1-D CNN are not comparable until they share participant-grouped splits, causal transforms, finite labels, and paired statistics. | Immutable dataset/split manifests; identical folds; RMSE/MAE/bias/correlation/coverage/CI/TOST outputs; raw and causal-postprocessed predictions; deterministic GBR and optional CNN tests |
| 5 | Model job API and Angular Model Lab | Operators need capability/readiness/progress/provenance, but the UI must consume a stable backend contract and never imply on-device inference. | Typed start/cancel/status/promote/rollback API; refresh-safe jobs; mobile/keyboard/screen-reader browser tests; report prediction overlay |
| 6 | Capture parser and quality ledger | Serial parsing remains embedded in the trainer monolith and does not produce a durable reason ledger for corrupt/mixed-width/gapped captures. | Extracted parser; atomic `capture_quality.json`; 207/219/222 replay matrix; byte-corruption/reset/gap tests; unchanged emitted CSV |
| 7 | Scope-safe, chunked frontend delivery | Angular routes are lazy in source but the build re-bundles every route into one IIFE; the service worker can also delete caches belonging to other apps on the same Pages origin. | Separate compatibility monolith and chunked `www/`; scope-owned cache cleanup; cold/warm/offline browser evidence; measured initial-load reduction |
| 8 | Durable asynchronous session supervision and bounded HTTP | PID reuse, restart adoption, a lock held through startup, and unbounded HTTP threads are lifecycle/resource risks. | Owner nonce/process fingerprint; STARTING/RUNNING/STOPPING/FAILED states; cancelable start; socket/body deadlines; connection cap tests |
| 9 | Route-handler extraction from the trainer monolith | Registry metadata exists, but the giant HTTP dispatcher still owns handler behavior. | Typed request/response adapter; one route group per commit; old/new black-box status/body/header conformance |
| 10 | Host-testable firmware schedulers | The 7,600-line sketch still contains blocking-risk I2C scanning and pure state logic that cannot be exercised outside the board. | One-address-per-tick LCD scan; loop-latency instrumentation; native presence/publish/recovery tests; Arduino compile; physical unplug/noisy-bus gate |
| 11 | Packaging consolidation and provenance | Duplicate Capacitor/Tauri package roots can drift, and release/debug artifacts do not share one provenance path. | One generated packaging graph; version-stamped debug and release artifacts; SBOM/provenance; install checks |
| 12 | Repository branch-retention policy | The remote has a long tail of merged/retry agent branches, but automatic deletion would be destructive without an explicit retention decision. | Document keep/delete rules; produce a dry-run merged-branch inventory; delete only after human approval; optional post-merge automation |

## Dual-model and statistics contract

Gradient boosting regression remains the default production research model.
The 1-D CNN remains optional and experimental; TensorFlow must not become a
runtime dependency of the lightweight trainer/EXE unless explicitly selected.

Both families must:

1. Consume the same immutable dataset manifest and consent-safe participant
   groups.
2. Use the same outer participant holdouts and recorded inner validation
   assignments.
3. Fit imputation, scaling, feature selection, and normalization only on the
   corresponding training groups.
4. Reject non-finite targets and prevent windows from crossing session
   boundaries, timestamp resets, or declared gaps.
5. Retain row/window identities and raw predictions before any clipping,
   smoothing, slew limiting, or abstention.
6. Publish pooled held-out predictions plus participant/session macro results,
   not only an average of fold scores.
7. Generate stable JSON and CSV statistics with sample, participant, and
   session counts; RMSE; MAE; bias; correlation; coverage; confidence
   intervals; and two one-sided equivalence tests (TOST).
8. Declare TOST margins and alpha before training. A non-significant
   difference test is never reported as equivalence.
9. Record source commit, package versions, seeds, feature/window schemas,
   hyperparameters, fold assignments, and artifact/report hashes.
10. Produce manuscript-ready table data from the same machine-readable report
    consumed by the API and UI.

The data contract supports distances from 0.5 to 1.0 m, but the current
2026-07-22 proposal freezes confirmatory conditions at 0.6, 0.8, and 1.0 m,
with and without one nominal 3-mm cardboard panel, three 150-s trials per
configuration. Each trial is immutably bound to one pseudonymous participant
profile; participant-independent claims require participant-held-out folds.

## Browser and packaging evidence

Frontend-affecting slices are accepted only after fresh captures and interaction
tests at desktop `1440x1000`, Pixel `390x844`, iPhone, and iPad breakpoints in
light, dark, night, and high-contrast themes. The core journey is:

`Connect -> operator unlock -> Home preflight -> Start -> Live -> Stop -> Report
-> Model Lab -> Settings`.

Acceptance includes no uncaught console errors, no horizontal overflow, 44 px
mobile targets, keyboard-only operation, focus return, 200% zoom/reflow,
reduced-motion behavior, offline cold/warm launch, and visible
source/version/model provenance.

## Feature candidates after integrity work

These ideas from the external review are valuable, but they follow the
security, study-validity, and lifecycle foundations above:

- A read-only observer role that can view live status without mutation rights.
- An interrupted-session recovery prompt backed by durable capture evidence,
  not merely a stale PID marker.
- A typed SSE device-health stream for radar quality, serial/BLE link state,
  and capture degradation.
- Multi-station aggregation for a coordinator view, with explicit station
  identity, per-device authentication, bounded fan-out, and acuity rules that
  are clinically/ethically reviewed before use.
- Spatial-zone configuration for presence/bed occupancy, gated by measured
  MR60BHA2 capability and physical acceptance.

The stable root trainer/dashboard compatibility filenames are intentionally
retained; moving the large generated dashboard or firmware source to another
directory would break documented artifact flows and requires an alias and
deprecation cycle. The short-lived SSE exchange token and ephemeral loopback
sidecar port are already resolved contracts and should not be re-opened without
new failing evidence.

## External gates

Automated green checks do not replace:

- physical ESP32-C6/MR60BHA2 compile/flash and serial capture;
- radar unplug/recovery and noisy/stuck I2C bench scenarios;
- AiLink reference-device acceptance where used;
- install/launch tests for the generated APK and Windows NSIS package;
- predeclared study statistical margins and advisor/ethics approval of
  manuscript claims.
