# Release, Signing, and Branch Protection

| Control | Value |
|---|---|
| Document ID | `RVT-QMS-PRO-001` |
| Revision | `R07` |
| Owner role | Release manager |
| Approver roles | Release manager, quality manager, and security owner |
| Effective product version | `16.5.11` |
| Retention | Project lifetime plus five years; archive then review |

This procedure implements the release portion of the
[ISO 9001:2015-aligned documented-information policy](../quality/qms-policy.md).
It is a project control and is not a claim of ISO 9001 certification.

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

Each publication also retains the QMS release record, detached `SHA256SUMS`,
and build-provenance evidence. The release record binds the final artifact
bytes to the approved source commit, workflow run, controlled-document
register revision, verification results, signing state, authorization, and
rollback strategy. Hashes must be calculated after the final signing step.

The release tag's base semantic version must equal the authoritative product
version. A syntactically valid but different tag is rejected. Tags and
published assets are immutable; a correction receives a new controlled version
instead of replacing evidence in place.

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
- `Security Audit / audit`
- `Build Android APK (Capacitor) / apk`
- `Build Windows EXE (Tauri) / windows`

The test job must include the QMS document/requirement contract, the exact
one-step product-version check, trainer and Angular tests, the release-manifest
self-test, and the generated-dashboard round trip. If the QMS check later moves
to a separate named job, add that job to the required checks before merging the
workflow change.

GitHub Pages must use the `GitHub Actions` publishing source. The custom Pages
workflow builds the Angular PWA and preserves the complete updater/QMS evidence
bundle (`rvt-latest.json`, `rvt-latest-tauri.json`, `qms-release-record.json`,
`controlled-document-revisions.json`, and `SHA256SUMS`) atomically; it must not
fall back to the legacy branch/Jekyll publisher or publish a partial evidence
set.

The dependency audit is a blocking gate. Python advisories and
moderate-or-higher findings in either npm lockfile fail the workflow. Any future
exception requires an advisory ID, owner, rationale, compensating control, and
expiry instead of a blanket `continue-on-error`.

Required ruleset settings:

- Require pull request before merging.
- Require at least one approving review.
- Require review from CODEOWNERS for owned paths.
- Dismiss stale approvals when new commits are pushed.
- Require status checks to pass before merging.
- Require branches to be up to date before merging.
- Require conversation resolution.
- Block force pushes and branch deletion.

Use the pull-request template as the change record. It must contain applicable
requirement IDs (or a justified administrative `N/A`), the impact and risk
assessment, implementation/test/artifact traceability, controlled-document
revision changes, objective verification, migration, and rollback. The
repository owner may hold more than one project role; the record must identify
the actual reviewer and must not imply independent approval that did not occur.

Configure the release environment with required reviewers for production or
thesis-evidence publication. Environment approval is the durable release
authorization. An automated prerelease can be generated without production
promotion, but its release record must keep authorization `pending` and label
the artifact accordingly.

The repository's `release-production` environment is restricted to `main`,
requires a named reviewer, disables administrator bypass, and holds the
`QMS_PRODUCTION_RELEASE_APPROVAL` sentinel. Production dispatch still fails
closed unless the source commit is reachable from `main` and the workflow run's
durable approval history names that reviewer.

## Controlled release evidence

The release evidence set uses `rvt-qms-release-record-v1` and contains:

- Product version, release tag, source repository, exact 40-character commit,
  and source ref.
- Workflow name, run ID, run attempt, builder identity, and UTC build time.
- Controlled-document register ID, revision, and SHA-256.
- Required check IDs, conclusions, and evidence URLs. Unavailable physical or
  browser gates use `external_gate`; they are not called passes.
- Final artifact name, type, byte size, SHA-256, and truthful signing state.
- Authorization state, actual authorizer, authorization time, rollback
  strategy, and previous compatible release.

The independent QMS schema version changes only when this evidence contract
changes; it does not advance on every product patch. Existing release
consumers may ignore additive provenance fields. A legacy release without a
QMS record remains historical and is identified as unverified rather than
retrofitted with invented evidence.

Release authorization requires:

1. The PR and source commit are approved and immutable.
2. The tag matches the authoritative product version.
3. Required checks pass and all external gates are resolved or explicitly
   accepted by the authorized role.
4. Controlled-document and requirements registers validate.
5. APK, AAB when applicable, EXE, updater signature, release manifests,
   checksums, and attestations describe the same final bytes.
6. Signing configuration is complete or the release is explicitly classified
   as unsigned/non-production.
7. Migration, compatibility, support limits, and rollback are documented.

## Artifact verification

For a local static check of downloaded GitHub Actions artifacts:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/verify-release-artifacts.ps1 `
  -ApkZip .\radar-vital-main-apk.zip `
  -AabZip .\radar-vital-main-aab.zip `
  -ExeZip .\radar-vital-main-exe.zip
```

This verifies each artifact ZIP contains a non-trivial `.apk`, optional `.aab`,
or `.exe` and prints SHA-256 hashes for the contained installers. For a
controlled release, also verify the detached checksums against the downloaded
bytes and confirm that the same values, source commit, workflow run, signing
state, and authorization appear in the QMS release record. A mismatch is a
release nonconformity and blocks promotion.
