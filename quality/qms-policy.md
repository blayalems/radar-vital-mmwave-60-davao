# RVT-QMS-POL-001 — Documented Information and Change Control

| Control | Value |
|---|---|
| Document ID | `RVT-QMS-POL-001` |
| Revision | `R03` |
| Status | Active when the approving pull request is merged |
| Owner role | Quality manager |
| Approver roles | Quality manager and technical lead |
| Effective product version | `16.5.8` |
| Review interval | 12 months or before a controlled process changes |
| Retention | Project lifetime plus five years; review before disposal |

## Purpose and limits

This procedure establishes an ISO 9001:2015-aligned documented-information,
design-change, verification, and release-control layer for Radar Vital. It
supports clauses 7.5, 8.3.3–8.3.6, 8.5.6, 8.6, 9.1, and 10.2. It does **not**
claim that the project, repository, laboratory, or organization is ISO 9001
certified.

Version control applies to every shipped product carrier, controlled document,
schema, requirement, test record, release artifact, model bundle, and
statistical report. This means that each item has a stable identity, an
explicit revision or schema version, and a traceable change record. It does not
mean rewriting historical records or advancing an unchanged schema whenever
the product patch changes.

## Authoritative records

- `quality/document-register.json` identifies current controlled documents,
  their owners, approvals, revisions, confidentiality, and retention.
- `quality/requirements.json` traces high-value requirements to implementation,
  verification, and affected artifact types.
- The pull request is the change record. It identifies requirements, risk,
  verification evidence, migration, rollback, document revisions, and release
  impact.
- Git commits are immutable implementation records. Every commit updates
  `HANDOFF.md`; commits must be reviewable and limited to one coherent change.
- `CHANGELOG.md` is the user-facing change history. `HANDOFF.md` is the
  engineering progress record. Existing entries are not rewritten.
- A release record binds the approved source commit, controlled-document
  revisions, verification evidence, and final artifact hashes. Product release
  records use the independent schema `rvt-qms-release-record-v1`.

## Roles and responsibilities

| Role | Responsibility |
|---|---|
| Contributor | Defines scope, links requirement IDs, implements the change, and records objective verification evidence. |
| Technical lead | Reviews architecture, interfaces, compatibility, risk controls, and rollback feasibility. |
| Quality manager | Reviews documented-information control, traceability, required evidence, deviations, and release readiness. |
| Research lead | Approves protocol, participant, model-evaluation, statistical, and manuscript-affecting changes. |
| Release manager | Confirms the approved commit, required checks, signing state, hashes, release record, and authorization. |
| Security owner | Reviews authentication, secrets, privacy, signing, network exposure, and supply-chain changes. |

Roles describe responsibilities and may be held by the same person in this
student project. The pull request and protected-environment records must show
the actual reviewer or authorizer; the repository must not imply separation of
duties that did not occur.

## Document control

1. A controlled document uses one stable document ID for its lifetime.
2. A content or control change advances its `RNN` revision in the document
   register. A moved document keeps its ID and records the new path.
3. The changed entry records the product version in which it becomes effective.
   Unchanged documents retain their existing effective product version.
4. The pull request lists every revised document ID and old/new revision.
5. Approval is the recorded pull-request review and merge. Draft text is not
   represented as approved evidence.
6. Superseded or obsolete entries remain in the register with their final
   revision, status, replacement where applicable, and retention rule.
7. Public documents must contain no participant identity, authentication
   secret, private key, recovery material, or restricted research data.
8. Generated copies are identifiable as uncontrolled unless they include the
   matching document ID, revision, source commit, and integrity hash.

Local-only LaTeX manuscripts, manuals, and trade-off reports remain outside
Git. Their output directory must contain an untracked `document-control.json`
recording document ID, revision, status, TeX and PDF SHA-256 hashes, build
command and tool versions, repository commit, creation time, and approval
evidence. Those local files must not be pushed unless a later approved change
explicitly brings them into repository scope.

## Requirement and design-change control

Each material change references at least one active requirement in
`quality/requirements.json`. If no registered requirement applies, the pull
request must justify `N/A` and either add a requirement or explain why the
change is administrative only.

The requirement record states the controlled input, responsible role, risk,
implementation paths, verification references, acceptance criteria, and
affected artifact types. High- and critical-risk changes require objective
automated evidence plus any named physical or manual acceptance evidence.

Changes that affect firmware, trainer, dashboard, ML/statistics behavior,
session or API schemas, packaging, or user-facing controlled documentation
advance the product exactly one patch or one minor release relative to the PR
base. Schema identifiers advance only when their contract changes. The frozen
serial prefix and replay contracts are changed only through explicit protocol,
migration, test, and manuscript review.

## Verification, nonconformity, and corrective action

- Acceptance evidence must identify the command/check, result, execution
  environment when relevant, and any skipped or external gate.
- A failing required check blocks release. An unavailable physical or browser
  environment is recorded as an open release gate, not silently treated as a
  pass.
- Defects and deviations are recorded in an issue, review thread, or PR with
  containment, cause, correction, regression evidence, and affected releases.
- Corrective actions reference the originating record and requirement IDs.
  Repeated failures trigger review of the requirement, process, and test—not
  only another local patch.
- Verification records and release records are retained according to the
  register and must remain readable for their retention period.

## Frozen analysis-plan and deviation control

The confirmatory statistical inputs are controlled in
`quality/statistical-analysis-plan.json`. A research-lead approval is required
before changing distances, barriers, trial duration/count, window rules,
equivalence margins, multiplicity handling, exclusions, or the primary
condition. A report records the plan ID and hash; a changed plan requires a
new product increment and manuscript/protocol review. If an external gate is
unavailable or a planned acceptance scenario is incomplete, the PR records the
deviation, containment, owner, and closure evidence. A project-team statement
of hardware completion is not substituted for a signed controlled acceptance
record.

## Release authorization and rollback

The release manager verifies that the tag matches the authoritative product
version, the source commit is approved, required checks pass, controlled
documents are current, deviations are resolved or explicitly accepted, and
final artifacts have recorded byte sizes and SHA-256 hashes. Signing state must
be reported truthfully. An unsigned diagnostic build must not be described as a
signed production artifact.

Authorization is recorded through the protected release environment or an
equivalent durable approval. The release record identifies the authorizer and
time. Rollback never replaces an existing tag or artifact in place; it promotes
a previously approved immutable build or creates a new controlled release with
its own version and record.

## Control review

The quality manager reviews this procedure and both registers at least every
12 months, after a major incident, or before changing the version, build,
release, research, or retention processes. Review results are captured in a
pull request even when no revision is required.
