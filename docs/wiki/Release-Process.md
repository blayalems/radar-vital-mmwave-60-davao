# Release Process

Radar Vital releases must keep firmware, trainer, dashboard, Android, Windows,
and Pages artifacts aligned. A release is not ready just because one build
passes locally.

## RC process

1. Merge Wave 2 work without hand-editing the built monolith.
2. Rebuild the monolith once in the integration pass.
3. Run Python tests, Angular unit tests, Playwright smoke, visual baselines, and
   build round-trip checks.
4. Publish an RC prerelease and verify APK/AAB, EXE, Pages, privacy URL, and
   update manifest artifacts.
5. Side-load on lab devices and run the physical acceptance checklist.

## GitHub Pages

Pages must publish the Angular PWA shell plus static legal pages at
`/privacy.html` and `/terms.html`. The privacy URL must resolve before Play
closed-testing submission.

## Play

Play closed testing requires a signed AAB, data-safety answers, content rating,
privacy policy, listing assets, and opted-in testers. For personal accounts,
plan around at least 12 testers and 14 days of active closed testing before
production promotion can be requested.

## Windows

The NSIS installer should carry the EULA, publisher identity, and signing when
available. Azure Trusted Signing or PFX signing proves publisher identity, but
SmartScreen reputation can still take time and Defender false positives may need
manual submission.

## Legal

Terms and privacy text are draft material until University of Mindanao legal /
REC review signs off. Any legal wording change requires a terms-version bump and
first-run consent re-prompt.
