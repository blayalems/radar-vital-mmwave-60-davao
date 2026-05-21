# Release, Signing, and Branch Protection

## Permanent releases

The `Release APK and EXE` workflow publishes permanent GitHub Release assets
when either of these happens:

- A tag matching `v*` is pushed.
- The workflow is manually dispatched with a `tag_name`.

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
