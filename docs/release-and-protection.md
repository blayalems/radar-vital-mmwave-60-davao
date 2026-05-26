# Release, Signing, and Branch Protection

## Permanent releases

The `Release APK and EXE` workflow publishes permanent GitHub Release assets
when either of these happens:

- A pull request merge or other accepted update is pushed to `main`.
- A tag matching `v*` is pushed.
- The workflow is manually dispatched with a `tag_name`.

Every `main` publication is an automated prerelease tagged
`v<app-version>-main.<workflow-run>`, for example `v12.0.0-main.42`. The
workflow run number increases on each release workflow invocation, so each
published main build receives a new version. All release routes require a
semantic-version tag and stamp that tag's version into the APK `versionName`
and Tauri installer version. The APK `versionCode` uses the increasing
workflow-run value so later signed APKs can upgrade earlier ones.
GitHub-generated release notes are enabled for every publication and serve as
the build changelog.

The release attaches:

- `radar-vital-release.apk` when Android signing secrets are configured.
- `radar-vital-debug.apk` as the fallback when Android signing secrets are not
  configured.
- `radar-vital-windows-installer.exe`, signed only when Windows certificate
  secrets are configured.

## Signing secrets

Add these repository secrets before treating artifacts as public production
installers:

- `ANDROID_KEYSTORE_BASE64`: base64-encoded Android keystore.
- `ANDROID_KEYSTORE_PASSWORD`: Android keystore password.
- `ANDROID_KEY_ALIAS`: Android signing key alias.
- `ANDROID_KEY_PASSWORD`: Android signing key password.
- `WINDOWS_CERTIFICATE_BASE64`: base64-encoded `.pfx` code-signing certificate.
- `WINDOWS_CERTIFICATE_PASSWORD`: `.pfx` password.

Without those secrets, CI still proves that APK and EXE packaging works, but the
published files are not production-signed.

## Required branch protection for `main`

Configure GitHub branch protection or a ruleset for `main` with these required
checks:

- `Playwright tests / test`
- `Build Android APK (Capacitor) / apk`
- `Build Windows EXE (Tauri) / windows`

Recommended settings:

- Require pull request before merging.
- Require status checks to pass before merging.
- Require branches to be up to date before merging.
- Block force pushes and branch deletion.

## Artifact verification

For a local static check of downloaded GitHub Actions artifacts:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/verify-release-artifacts.ps1 `
  -ApkZip .\radar-vital-main-apk.zip `
  -ExeZip .\radar-vital-main-exe.zip
```

This verifies each artifact ZIP contains a non-trivial `.apk` or `.exe` and
prints SHA-256 hashes for the contained installers.
