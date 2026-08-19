"""Supply-chain, packaging, and required-check workflow contracts.

Guards the wiring added for supply-chain hardening so the workflows cannot
silently regress:

  D9  - the dependency audit is blocking and covers Python, Rust, and all npm trees.
  D8  - Pages preserves the complete release-evidence bundle atomically while
        distinguishing a first-release 404 from transient/network errors.
  CI  - expensive contracts run once, browser shards stay browser-only, and one
        stable aggregate ``test`` check represents the complete workflow.

Everything is parsed as plain text: PyYAML may be absent in CI, so we never
import it here. Assertions are kept robust to minor formatting (whitespace,
quote style) by normalising before matching.
"""

from __future__ import annotations

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS = ROOT / ".github" / "workflows"
PLAYWRIGHT = WORKFLOWS / "playwright.yml"
PAGES = WORKFLOWS / "pages.yml"
BUILD_APK = WORKFLOWS / "build-apk.yml"
BUILD_EXE = WORKFLOWS / "build-exe.yml"
RELEASE = WORKFLOWS / "release-artifacts.yml"
DEPENDABOT = ROOT / ".github" / "dependabot.yml"


def _text(path: Path) -> str:
    assert path.exists(), f"expected workflow file missing: {path}"
    return path.read_text(encoding="utf-8")


def _normalise(s: str) -> str:
    """Lowercase and collapse whitespace so matches survive reformatting."""
    return " ".join(s.lower().split())


def test_third_party_actions_are_immutable_and_monitored():
    mutable = []
    for workflow in WORKFLOWS.glob("*.yml"):
        for line_number, line in enumerate(_text(workflow).splitlines(), 1):
            match = re.search(r"\buses:\s+([^\s#]+)", line)
            if not match or match.group(1).startswith("./"):
                continue
            ref = match.group(1).rsplit("@", 1)[-1]
            if not re.fullmatch(r"[0-9a-f]{40}", ref):
                mutable.append(f"{workflow.name}:{line_number}:{match.group(1)}")
    assert mutable == [], f"workflow actions must use commit SHAs: {mutable}"

    dependabot = _text(DEPENDABOT)
    assert "package-ecosystem: github-actions" in dependabot
    assert "directory: /" in dependabot


# --- D9: dependency-audit workflow -------------------------------------------


def test_security_audit_workflow_exists_and_runs_audits():
    text = _text(WORKFLOWS / "security-audit.yml")
    norm = _normalise(text)

    # Python, npm, and Rust audits must all be present.
    assert "pip-audit" in norm, "security-audit.yml must run pip-audit"
    assert "npm audit" in norm, "security-audit.yml must run npm audit"
    assert "cargo audit" in norm, "security-audit.yml must run cargo-audit"

    # requirements.txt includes the canonical file, so audit it once.
    assert "requirements-v12.txt" in text
    assert "python -m pip_audit -r requirements-v12.txt" in norm

    # The web and standalone Capacitor package trees are audited too.
    assert "npm --prefix web audit" in norm
    assert "npm --prefix packaging/capacitor audit" in norm

    # Rust uses a pinned, checksum-verified auditor. The one platform-specific
    # exception must retain its advisory ID, rationale, owner, and expiry.
    assert "taiki-e/install-action@7f4eb899022d8fe70b20c4f3de697aa85c309026" in text
    assert "cargo-audit@0.22.2" in text
    assert "--ignore RUSTSEC-2024-0429" in text
    assert "Owner: desktop" in text
    assert "Expiry: 2026-11-10" in text

    # Findings are a hard gate; exceptions must remain explicit and scoped.
    assert "continue-on-error" not in norm
    assert "--audit-level=moderate" in norm

    # Expected triggers and runner.
    assert "workflow_dispatch" in norm
    assert "pull_request" in norm
    assert "ubuntu-latest" in norm


# --- D9: npm cache key includes the web lockfile -----------------------------


def test_build_exe_npm_cache_includes_web_lockfile():
    text = _text(WORKFLOWS / "build-exe.yml")

    assert "cache-dependency-path" in text, (
        "build-exe.yml setup-node must declare cache-dependency-path"
    )
    # The web lockfile must be part of the npm cache key.
    assert "web/package-lock.json" in text, (
        "build-exe.yml npm cache key must include web/package-lock.json"
    )
    # The root lockfile should still be present.
    assert "package-lock.json" in text


# --- D8: resilient manifest preservation -------------------------------------


def test_pages_manifest_step_is_resilient():
    text = _text(PAGES)
    norm = _normalise(text)

    # Preserve the complete updater/QMS bundle, not just the web manifest.
    for file_name in (
        "rvt-latest.json",
        "rvt-latest-tauri.json",
        "qms-release-record.json",
        "controlled-document-revisions.json",
        "SHA256SUMS",
    ):
        assert file_name in text

    # Captures the HTTP status code rather than collapsing all failures.
    assert "%{http_code}" in text, (
        "pages.yml manifest step must capture the curl http_code"
    )

    # Retries on transient errors.
    assert "retry" in norm or "retries" in norm or "attempt" in norm, (
        "pages.yml manifest step must retry transient failures"
    )

    # Distinguishes a legitimate 404 from other errors.
    assert "404" in text, (
        "pages.yml manifest step must special-case a 404 (not yet published)"
    )

    # Fails hard on persistent failure instead of silently proceeding.
    assert "exit 1" in norm, (
        "pages.yml manifest step must fail (exit 1) on persistent fetch errors"
    )

    # A partially published evidence set must never be copied into www/.
    assert "release_files=(" in text
    assert 'if [ "$present" -eq 0 ]' in text
    assert 'if [ "$missing" -ne 0 ]' in text
    assert 'cp "$bundle_dir"/* www/' in text


def test_playwright_contracts_run_once_and_publish_stable_test_check():
    text = _text(PLAYWRIGHT)

    assert text.count("python -m pytest tests -v") == 1
    assert text.count("npm run test:unit:web") == 1
    assert text.count("npm run build:check") == 1
    assert "\n  contracts:\n" in text
    assert "\n  smoke:\n" in text
    assert "\n  visual:\n" in text
    assert "\n  test:\n    name: test\n" in text
    assert "CONTRACTS_RESULT: ${{ needs.contracts.result }}" in text
    assert "SMOKE_RESULT: ${{ needs.smoke.result }}" in text
    assert "VISUAL_RESULT: ${{ needs.visual.result }}" in text
    assert "FIRMWARE_RESULT: ${{ needs.firmware.result }}" in text

    # Browser jobs run on isolated runners, so the verified build must be
    # transferred rather than assumed to exist after the contracts job.
    assert text.count("name: verified-web-bundle") == 3
    assert text.count("actions/download-artifact@") == 2
    assert "Upload verified web bundle for browser shards" in text
    assert "retention-days: 1" in text

    # Successful shards should not consume report storage; cancellation still
    # gets a best-effort diagnostic upload.
    assert text.count("if: ${{ failure() || cancelled() }}") == 2
    assert text.count("retention-days: 3") == 2
    assert text.count("if: always()") == 1


def test_workflow_tokens_and_intermediate_artifacts_are_scoped():
    playwright = _text(PLAYWRIGHT)
    pages = _text(PAGES)
    apk = _text(BUILD_APK)
    exe = _text(BUILD_EXE)
    release = _text(RELEASE)

    for workflow in (playwright, pages, apk, exe, release):
        assert "permissions:\n  contents: read" in workflow

    assert "pages: write" not in pages
    assert "actions/deploy-pages" not in pages
    assert "Production deployment has one owner" in pages
    assert "actions: read\n      attestations: write\n      contents: write" in release
    assert playwright.count("persist-credentials: false") == 4
    assert release.count("persist-credentials: false") == 5

    # Generated platforms should suppress only the known already-present case;
    # a real `cap add` failure must stop packaging.
    for workflow in (apk, release):
        assert "cap add android ||" not in workflow
        assert "if [ ! -d android ]; then" in workflow

    assert "retention-days: 7" in apk
    assert "retention-days: 7" in exe
    assert release.count("retention-days: 1") == 4


def test_manual_release_dispatch_honors_the_requested_tag_without_main_publication():
    release = _text(RELEASE)

    dispatch_guard = 'if [ "$GITHUB_EVENT_NAME" = "workflow_dispatch" ]; then'
    assert dispatch_guard in release
    assert 'release_tag="$REQUESTED_TAG"' in release
    assert 'release_tag="v${base_version}-main.' not in release
    assert "branches:\n      - main" not in release


def test_release_is_intentional_immutable_and_builds_web_once():
    release = _text(RELEASE)
    apk = _text(BUILD_APK)
    exe = _text(BUILD_EXE)

    assert "tags:\n      - 'v*'" in release
    assert "scripts/check-release-source.mjs" in release
    assert release.index("scripts/check-release-source.mjs") < release.index("\n  android:\n")
    assert "cancel-in-progress: false" in release
    assert "required-check-evidence.json" in release
    assert release.count("npm run build:check") == 1
    assert "python -m pytest tests -q" not in release
    assert "npm run test:unit:web" not in release
    assert "npx playwright test tests/smoke" not in release
    assert release.count("name: verified-release-web") == 4
    assert '$config.build.beforeBuildCommand = ""' in release

    for workflow in (apk, exe):
        assert "'codex/**'" not in workflow
        assert "'gpt55/**'" not in workflow
        assert "pull_request:" in workflow
    assert "npm run build:web" not in exe


def test_firmware_compile_uses_the_pinned_toolchain_contract():
    import json

    workflow = _text(PLAYWRIGHT)
    lock = json.loads((ROOT / "packaging" / "firmware" / "arduino-toolchain.json").read_text(encoding="utf-8"))
    assert lock["arduino_cli"] == "1.5.1"
    assert lock["fqbn"] == "esp32:esp32:XIAO_ESP32C6"
    assert lock["core"] == "esp32:esp32@3.3.11"
    assert lock["sketch"] == "radar_vital_v16_6_3.ino"
    assert lock["libraries"]["NimBLE-Arduino"] == "2.5.0"
    assert len(lock["seeed_mmwave_commit"]) == 40
    assert all("main" not in str(value) and "latest" not in str(value) for value in lock.values())
    assert f"arduino/setup-arduino-cli@{lock['setup_action_commit']}" in workflow
    assert "arduino-cli core install \"${{ steps.toolchain.outputs.core }}\"" in workflow
    assert "sed -i 's/^#define ENABLE_BLE false$/#define ENABLE_BLE true/'" in workflow
    assert "grep -qx '#define ENABLE_BLE true'" in workflow
    assert "extra_flags=-DENABLE_BLE=1" not in workflow
    assert "- firmware" in workflow


def test_contracts_install_and_exercise_wheel_outside_checkout():
    workflow = _text(PLAYWRIGHT)
    assert "python -m pip wheel --no-deps" in workflow
    assert "cd \"$RUNNER_TEMP\"" in workflow
    assert "env -u PYTHONPATH" in workflow
    assert 'RVT_REQUIRE_INSTALLED_PACKAGE: \'1\'' in workflow
    assert '"$venv/bin/rvt-trainer" --help' in workflow
    assert '"$venv/bin/rvt-statistics" --help' in workflow
    assert 'tests/installed_wheel_probe.py' in workflow


def test_production_release_pages_artifact_keeps_site_and_metadata_parity():
    release = _text(RELEASE)

    for command in (
        "cp docs/privacy.html www/privacy.html",
        "cp docs/terms.html www/terms.html",
        "touch www/.nojekyll",
        "cat > www/robots.txt",
    ):
        assert command in release

    for file_name in (
        "index.html",
        "404.html",
        "sw.js",
        "privacy.html",
        "terms.html",
        "rvt-latest.json",
        "rvt-latest-tauri.json",
        "qms-release-record.json",
        "controlled-document-revisions.json",
        "SHA256SUMS",
        ".nojekyll",
        "robots.txt",
    ):
        assert file_name in release

    assert "Verify complete production Pages artifact" in release
    assert 'test -f "www/${file}"' in release
    assert "cmp -s www/index.html www/404.html" in release
    assert "grep -q 'Radar Vital Privacy Notice' www/privacy.html" in release
    assert "grep -q 'Radar Vital Terms and Conditions' www/terms.html" in release
