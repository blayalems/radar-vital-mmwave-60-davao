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
- `radar-vital-release.aab` for Google Play closed testing when Android signing
  secrets are configured.
- `radar-vital-debug.apk` as the fallback when Android signing secrets are not
  configured.
- `radar-vital-windows-installer.exe`, signed only when Windows certificate
  secrets are configured.

## Signing secrets

Add these repository secrets before treating artifacts as public production
installers.

Android APK/AAB signing:

- `ANDROID_KEYSTORE_BASE64`: base64-encoded Android keystore.
- `ANDROID_KEYSTORE_PASSWORD`: Android keystore password.
- `ANDROID_KEY_ALIAS`: Android signing key alias.
- `ANDROID_KEY_PASSWORD`: Android signing key password.

The release workflow builds both `radar-vital-release.apk` and
`radar-vital-release.aab` when all four Android signing secrets are present.
When none are present it publishes the existing debug APK fallback. A partial
Android signing configuration fails the workflow instead of silently publishing
an incorrectly signed release.

Windows signing, preferred path (Azure Trusted Signing / Artifact Signing):

- `AZURE_CLIENT_ID`: Microsoft Entra application/client ID configured for
  GitHub OIDC.
- `AZURE_TENANT_ID`: Microsoft Entra tenant ID.
- `AZURE_SUBSCRIPTION_ID`: Azure subscription ID.
- `AZURE_TRUSTED_SIGNING_ENDPOINT`: regional code-signing endpoint, for example
  `https://eus.codesigning.azure.net/`.
- `AZURE_TRUSTED_SIGNING_ACCOUNT`: Trusted Signing account name.
- `AZURE_TRUSTED_SIGNING_CERTIFICATE_PROFILE`: certificate profile name.

Windows signing, fallback path:

- `WINDOWS_CERTIFICATE_BASE64`: base64-encoded `.pfx` code-signing certificate.
- `WINDOWS_CERTIFICATE_PASSWORD`: `.pfx` password.

If Azure Trusted Signing secrets are complete, the workflow signs the final NSIS
installer with Azure and skips the `.pfx` fallback. If Azure is not configured,
the existing `.pfx` path is used when its certificate secrets are present. If no
Windows signing secrets are configured, CI still proves EXE packaging but leaves
the installer unsigned. A partial Azure Trusted Signing configuration fails the
workflow instead of silently falling back to a different signing identity.

## Code-signing reality

Code signing improves provenance: Windows can verify that the installer was
signed by the configured identity and that the bytes were not modified after
signing. It does not guarantee an immediate no-warning install experience.
Microsoft Defender SmartScreen reputation accrues over time per certificate,
publisher, and download reputation. New student/research installers can still
show warnings even when they are correctly signed.

For release candidates, keep this loop explicit:

- Sign with Azure Trusted Signing when available; use the `.pfx` fallback only
  when Azure is unavailable.
- Re-sign the Tauri updater artifact after the final installer bytes are signed.
- If Defender flags the PyInstaller sidecar or NSIS installer, submit a false
  positive through Microsoft Security Intelligence with the exact release hash.
- Document any remaining SmartScreen warning in the release notes instead of
  promising that signing removes all warnings.

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
  -AabZip .\radar-vital-main-aab.zip `
  -ExeZip .\radar-vital-main-exe.zip
```

This verifies each artifact ZIP contains a non-trivial `.apk`, optional `.aab`,
or `.exe` and prints SHA-256 hashes for the contained installers.
