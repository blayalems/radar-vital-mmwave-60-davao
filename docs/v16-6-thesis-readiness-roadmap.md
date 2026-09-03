# Radar Vital v16.6 thesis-readiness roadmap

Date: 2026-08-19  
Status: active engineering roadmap; no research or release authorization

This roadmap supersedes `docs/v16-5-high-yield-roadmap.md` as the ordering and
ownership record. Completed v16.5 work remains historical evidence. Unfinished
items are either carried below, explicitly deferred, or retained as external
gates; they are not silently dropped.

| Order | Slice | Owner role | Estimate | Abort/defer criterion | Status |
|---:|---|---|---:|---|---|
| 1 | Fail-closed credential persistence with live-session availability | security/backend maintainer | 3–5 days | Stop if existing authenticated sessions cannot safely stop a capture | Draft PR #113 |
| 2 | Separate run, plan, study-protocol, and session-schema provenance | research software maintainer | 2–3 days | Do not approve the plan or relabel frozen study identities | Draft PR #114 |
| 3 | Pre-collection calendar, authorization, withdrawal, and synchronization gates | research lead + quality manager | 2–4 days | No recruitment/collection while any gate is pending | Draft PR #115 |
| 4 | CI gate attestation, build-once promotion, release immutability, and reproducible runner baseline | release maintainer | 5–8 days | Preserve or improve time-to-green while adding missing gates; split if release verification guarantees weaken | In progress |
| 5 | Packaging/dependency graph consolidation | packaging maintainer | 5–8 days | No destructive package-root removal until every requirement/workflow consumer is migrated and reviewed | Planned separately |
| 6 | Lazy package boundary and monolith public-surface/route inventory | backend maintainer | 5–10 days | Defer extraction if public-symbol or route golden contracts cannot remain stable | Planned |
| 7 | Parser, supervisor, and route-handler extraction | backend maintainer | 8–15 days | Freeze after two lifecycle/route conformance regressions and ship current behavior | Planned in reviewable slices |
| 8 | Generated frontend contract and BackendPort, then sandbox maps, Live extraction, and CSS ownership | frontend maintainer | 10–20 days | If the visual suite regresses twice for one slice, defer that slice past thesis submission | Planned as separate PRs |
| 9 | GBR/CNN stopping rule and manuscript-ready exports | research software maintainer | 4–7 days | Report CNN unavailable when the predeclared data floor is not met; do not tune after outcomes | Planned |

The estimates are planning ranges, not promises. Calendar-bound approval,
recruitment, 38-participant protocol completion, at least 19 independent
primary estimates, three hours minimum no-subject capture, physical latency
characterization, packaging installation, signing, and hardware acceptance are
external gates.

## v16.5 disposition

- Carried: capture parser/quality ledger, session supervisor owner-nonce and
  lifecycle work, route extraction, host-testable firmware schedulers,
  packaging consolidation, and branch-retention governance.
- Split: model/statistics provenance and pre-collection readiness move before
  infrastructure because recruitment and capture are calendar-bound.
- Deferred unless new evidence raises priority: scope-safe chunking and broad
  UI decomposition beyond the isolated frontend slices above.
- Branch retention remains read-only planning: produce a dry-run inventory and
  obtain human approval before any deletion. The archive branch remains frozen.
- Firmware scheduler work remains a physical-bench slice and does not block the
  pre-collection documentation PR; it does block claims that depend on its
  unresolved timing or recovery behavior.

## Merge and release boundary

Every slice is a draft, reviewable PR with one exact product step and a HANDOFF
update in every commit. No slice merges, tags, publishes, deploys, approves a
study plan, or changes repository protection without explicit human action.
Generated-dashboard and HANDOFF conflicts are expected in the stack and must be
resolved after each predecessor lands; a future merge-queue design may reduce
reruns but is not assumed here.
