# Pre-collection readiness and external gates

Status: **blocked**  
Schedule status: **proposed, not authorized**  
Manifest: `quality/precollection-readiness.json`

This document turns the study calendar, authorization boundary, withdrawal
handling, and synchronization evidence into reviewable gates. Merging software
does not approve the statistical plan, satisfy the REC, or authorize
recruitment, collection, exclusion, confirmatory evaluation, or manuscript
claims. `RVT-STA-001` and the statistical plan remain draft; draft controls may
gate software preparation only.

## Scale and calendar

The current proposal targets 40 recruited participants, at least 38
protocol-complete participants, and at least 19 independent estimates for the
primary RR TOST. A complete participant has six conditions with three
150-second trials each: 18 captures, for 684 minimum protocol-complete captures
and 720 captures of capacity at 40 recruits. Objective 3 separately requires
72 unique 150-second no-subject captures, or 10,800 seconds (three hours) of
capture time before setup, review, reruns, and evidence reconciliation.

The manifest assigns owner roles, earliest dates, target dates, and an abort or
replan rule to approvals, synchronization bench work, no-subject collection,
recruitment, protocol completion, and confirmatory data lock. The dates are
planning assumptions as of 2026-08-19 in Asia/Manila, not evidence that a gate
was completed. A dated evidence reference and all named approvals are required
before the manifest can become ready.

## Withdrawal and erasure boundary

Append-only means audit and control metadata, not an unconditional promise to
retain participant payloads. On withdrawal, the system may retain a
non-identifying tombstone that records the authority, consent revision,
artifact classes, and disposition. The participant data itself must follow the
REC/privacy-approved disposition: retention under an approved basis,
restriction of processing, verified physical deletion, or verified key
destruction when encryption architecture actually supports it.

The current repository does not provide per-participant cryptographic erasure.
Therefore it must not claim that withdrawal data was crypto-shredded. The
withdrawal gate remains blocked until the REC/privacy decision, operational
procedure, responsible role, verification evidence, and treatment of derived
artifacts are recorded. Legal and REC review, rather than this engineering
document, determines the applicable treatment.

## Synchronization and uncertainty

Host monotonic receive anchors remain outside the frozen 222-column serial
contract. Each study session must retain `sync_anchors.json` with the clock,
anchor method, source commit, anchor observations, and uncertainty model. The
derived `alignment_report.json` must state the method, estimated offset,
numeric uncertainty in seconds, latency-characterization reference, and
limitations. An offset without a method and uncertainty is not sufficient
provenance.

USB scheduling and buffering jitter are expected to matter less for the
30-second-window RR analysis than for HR agreement, but this is a sensitivity
assessment rather than measured evidence. Objective 4 timing precision and
alignment claims remain blocked until a physical bench characterizes latency
and uncertainty. The uncertainty must be propagated into the limitations and
manuscript wording; the report must not imply clock accuracy beyond the bench
evidence.

## Readiness decision

The machine-readable manifest is intentionally fail closed. Changing its
status to `ready` requires a separate controlled change after all approvals,
physical synchronization evidence, withdrawal disposition, frozen protocol
configuration, operator training, and collection materials have dated evidence
references. The software tests assert the blocked state and critical arithmetic
so an ordinary release bump cannot silently authorize participant work.
