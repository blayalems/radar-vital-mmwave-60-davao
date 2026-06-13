# Radar Vital v16.3.0 Milestones

This file tracks release-readiness milestones for PR #54. It is intentionally
honest about gates that code cannot solve by itself: University of Mindanao
legal / Research Ethics Committee review, Google Play policy timing, signing
identity validation, and SmartScreen reputation.

## v16.3.0-rc

Goal: produce the first release-candidate build for lab-side validation.

Exit criteria:

- Wave 1 and Wave 2 features are merged without ownership conflicts.
- Monolith is rebuilt once by the integrator and `npm run build:check` is clean.
- Python tests, Angular unit tests, Playwright smoke, and committed visual
  baselines are green.
- Pages publishes `/privacy.html` and `/terms.html`.
- RC release artifacts include Android and Windows outputs plus update manifest.
- Physical acceptance checklist is scheduled on the target radar hardware.

## v16.3.0 RTM

Goal: tag the production-ready thesis release after RC defects are closed.

Exit criteria:

- Version bump lands across package metadata, trainer constants, dashboard
  version display, firmware filename/defines, tests, README, and CHANGELOG.
- No known blocker remains in session capture, export, operator auth, pairing,
  or update discovery.
- Release workflow publishes permanent artifacts for the tag.
- `rvt-latest.json` points older 16.2.x clients to the new release.
- RTM notes repeat that Radar Vital is not a medical device.

## Play closed testing

Goal: make the Android app eligible for Play production promotion later.

Exit criteria:

- Signed AAB is produced by CI and uploaded to the closed-testing track.
- Privacy URL resolves at `https://blayalems.github.io/radar-vital-mmwave-60-davao/privacy.html`.
- Data Safety, content rating, short description, full description, screenshots,
  and feature graphic are complete.
- At least 12 testers opt in and actively test for at least 14 days.
- Testers understand that the Android app is a companion client for a LAN-bound
  trainer, not a standalone radar reader.

Risk: Play policy and review timing are external. Passing CI does not guarantee
Play approval or production eligibility on a specific date.

## Signing identity

Goal: give Windows and Android artifacts a verifiable publisher identity.

Exit criteria:

- Android keystore secrets are present and documented in repository settings.
- Windows signing path is selected: Azure Trusted Signing as the preferred path,
  with PFX signing as a fallback when applicable.
- Release workflow handles missing secrets honestly instead of pretending debug
  artifacts are production-signed.
- Installer documentation explains SmartScreen reality: signing is necessary for
  identity, but reputation may still produce warnings until trust accrues.

Risk: identity validation can take days or weeks and may require human documents.
No code change can guarantee immediate SmartScreen trust.

## Legal review sign-off

Goal: replace draft legal language with approved thesis-use wording.

Exit criteria:

- University of Mindanao legal / Research Ethics Committee reviewers approve or
  revise the Terms and Privacy Notice.
- RA 10173 language is counsel-reviewed before being called final.
- Any wording change updates `TERMS_VERSION` and re-prompts first-run consent.
- Play Data Safety answers are checked against the final Privacy Notice.

Risk: the current RA 10173 text is a draft. Treat it as release-blocking for RTM
distribution if the study protocol requires formal approval before use.

## v16.4 backlog

Candidate follow-ups after v16.3.0 RTM:

- BLE acceptance suite and any native BLE activation work that was intentionally
  kept out of the v16.3 blast radius.
- Installer reputation and Defender false-positive submission loop hardening.
- More automated Pages/wiki publication checks after the GitHub wiki is synced.
- Post-closed-testing Play feedback fixes.
- Additional operator export and retention controls requested by legal / REC.
- Accessibility audit expansion for all dialogs and bottom-sheet flows.
