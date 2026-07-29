## Summary
<!-- What does this PR do and why? Link to any design docs or discussions. -->

## Linked Issues
<!-- Use "Closes #XX" to auto-close issues on merge. -->

## Changes
<!-- Group changes by component (firmware, trainer, web, tests, docs). -->

## Verification
<!-- Which verification steps from CONTRIBUTING.md did you run? -->
- [ ] `python -m pytest -q tests`
- [ ] `npm --prefix web run test:ci`
- [ ] `npm run build:web && npm run build:check`
- [ ] `npm test` (Playwright smoke, 4 viewports)
- [ ] `python -m compileall -q radar_vital_trainer_v12_for_v16_0.py rvt_trainer`

## Checklist
- [ ] Tests pass locally
- [ ] CHANGELOG.md updated (if user-facing change)
- [ ] manifest-lock.json updated (if version change)
- [ ] HANDOFF.md entry added (dated, top of file)
- [ ] No new lint warnings
- [ ] Reviewer assigned (or self-review documented for thesis workflow)

## Review Notes
<!-- Anything the reviewer should focus on? Security implications? ML correctness? -->
