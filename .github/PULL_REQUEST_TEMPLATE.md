<!--
RVT-QMS-TPL-001, revision R01.
This pull request is the durable change record. Replace every placeholder.
Do not claim a check passed when it remains an external or unavailable gate.
-->

## Change identity

- Change title:
- Related issue, review, CAPA, or decision:
- Requirement IDs from `quality/requirements.json`:
- Administrative-only `N/A` justification, if applicable:
- Intended product version:
- Release step from base: patch / minor

## Purpose and scope

Describe the problem, controlled input, intended result, and explicit
out-of-scope items.

## Impact assessment

Mark each area and explain every checked item.

- [ ] Firmware or hardware
- [ ] Trainer, API, or session lifecycle
- [ ] Angular dashboard or generated web artifact
- [ ] Serial, API, session, model, or document schema
- [ ] Participant protocol or research data
- [ ] GBR, experimental 1-D CNN, features, or inference
- [ ] Statistical analysis or manuscript output
- [ ] APK, EXE, PWA, signing, updater, or release packaging
- [ ] Security, privacy, authentication, or secrets
- [ ] Controlled documentation

Risk class: low / medium / high / critical

Failure modes, affected users/data/artifacts, and controls:

## Traceability

| Requirement ID | Implementation path(s) | Verification reference(s) | Affected artifact(s) |
|---|---|---|---|
| `RVT-…` | | | |

## Controlled documents

List every changed controlled document. Use `None` only when the diff contains
no path registered by `quality/document-register.json`.

| Document ID | Path | Old revision | New revision | Effective product version |
|---|---|---:|---:|---|
| `RVT-QMS-…` | | | | |

## Compatibility and migration

- Backward-compatible behavior:
- Legacy data/session/schema treatment:
- Data or configuration migration:
- Explicitly unsupported combinations:

## Verification evidence

| Check or command | Result | Environment/evidence link |
|---|---|---|
| `npm run test:qms-contract -- --base-ref <PR-base>` | | |
| `npm run test:version-contract` | | |
| `npm run test:pr-version -- --base-ref …` | | |
| Python/Angular/browser/firmware/package checks as applicable | | |

External, skipped, or unavailable gates and why they remain release blockers or
accepted deviations:

## Release and rollback

- Release artifact or provenance impact:
- Signing/attestation impact:
- Rollback strategy:
- Previous compatible release or data format:
- Irreversible action, if any:

## Review and authorization

- Technical review role:
- Quality review role:
- Research/security/release review roles when applicable:
- Required protected-environment approval:

## Completion checklist

- [ ] The change is limited to the stated scope.
- [ ] Every commit updates `HANDOFF.md`.
- [ ] The product advances exactly one patch or one minor when required.
- [ ] Unchanged schema identifiers remain unchanged.
- [ ] Requirement, implementation, test, commit, and artifact links are complete.
- [ ] Controlled-document revisions and effective versions are current.
- [ ] Generated artifacts were regenerated from their authoritative sources.
- [ ] Verification evidence is objective; unavailable gates are not called passes.
- [ ] No participant identity, secret, private key, or restricted data is included.
- [ ] Migration and rollback are documented and feasible.
