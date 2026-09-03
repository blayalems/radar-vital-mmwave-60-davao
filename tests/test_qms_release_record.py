"""Release-provenance and integrity-evidence contract tests."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
QMS_GENERATOR = ROOT / "scripts" / "generate-qms-release-record.mjs"
LATEST_GENERATOR = ROOT / "scripts" / "generate-rvt-latest.mjs"
VERIFIER = ROOT / "scripts" / "verify-release-artifacts.ps1"
WORKFLOW = ROOT / ".github" / "workflows" / "release-artifacts.yml"
UPDATER_SIGNATURE_VERIFIER = (
    ROOT / "src-tauri" / "examples" / "verify_updater_signature.rs"
)


def _version() -> str:
    return json.loads((ROOT / "package.json").read_text(encoding="utf-8"))[
        "version"
    ]


def _release_env(**overrides: str) -> dict[str, str]:
    env = {
        **os.environ,
        "RELEASE_TAG": f"v{_version()}",
        "RELEASE_VERSION": _version(),
        "RELEASED_AT": "2026-07-29T00:00:00Z",
        "GITHUB_REPOSITORY": "example/radar-vital",
        "GITHUB_SHA": "0123456789abcdef0123456789abcdef01234567",
        "GITHUB_REF": f"refs/tags/v{_version()}",
        "GITHUB_RUN_ID": "1234",
        "GITHUB_RUN_ATTEMPT": "2",
        "ANDROID_SIGNATURE_STATE": "signed_release",
        "WINDOWS_SIGNATURE_STATE": "authenticode_verified",
    }
    env.update(overrides)
    return env


REQUIRED_WORKFLOWS = (
    ("Playwright tests", ".github/workflows/playwright.yml"),
    ("Security Audit", ".github/workflows/security-audit.yml"),
    ("Build Android APK (Capacitor)", ".github/workflows/build-apk.yml"),
    ("Build Windows EXE (Tauri)", ".github/workflows/build-exe.yml"),
)


def _write_required_check_evidence(
    dist: Path,
    *,
    source_sha: str = "0123456789abcdef0123456789abcdef01234567",
) -> Path:
    evidence_path = dist / "required-check-evidence.json"
    evidence_path.write_text(
        json.dumps(
            {
                "schema_version": "rvt-required-check-evidence-v1",
                "release_tag": f"v{_version()}",
                "source_sha": source_sha,
                "generated_at": "2026-08-19T00:00:00Z",
                "tag_state": "new_tag_identity",
                "checks": [
                    {
                        "workflow_name": name,
                        "workflow_id": str(index + 100),
                        "path": workflow_path,
                        "event": "push",
                        "conclusion": "success",
                        "run_id": str(index),
                        "run_attempt": 1,
                        "run_url": f"https://example.invalid/runs/{index}",
                        "completed_at": "2026-08-19T00:00:00Z",
                    }
                    for index, (name, workflow_path) in enumerate(
                        REQUIRED_WORKFLOWS, start=1
                    )
                ],
            }
        ),
        encoding="utf-8",
    )
    return evidence_path


def test_qms_generator_self_test_and_tag_gate():
    result = subprocess.run(
        ["node", str(QMS_GENERATOR), "--self-test"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "QMS release record self-test passed." in result.stdout


def test_qms_generator_rejects_tag_without_v_prefix(tmp_path: Path):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(b"exe-fixture")
    result = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(tmp_path)],
        cwd=ROOT,
        env=_release_env(RELEASE_TAG=_version()),
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "must start with v" in result.stderr


def test_qms_generator_rejects_invalid_document_register(tmp_path: Path):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(b"exe-fixture")
    invalid_register = tmp_path / "invalid-document-register.json"
    invalid_register.write_text(
        json.dumps(
            {
                "$schema": "./schemas/document-register.schema.json",
                "schema_version": "rvt-qms-document-register-v1",
                "register_id": "RVT-QMS-REG-001",
            }
        ),
        encoding="utf-8",
    )
    result = subprocess.run(
        [
            "node",
            str(QMS_GENERATOR),
            "--dist",
            str(tmp_path),
            "--document-register",
            str(invalid_register),
        ],
        cwd=ROOT,
        env=_release_env(),
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "document register failed schema validation" in result.stderr


def test_qms_generator_does_not_mark_not_verified_windows_as_signed(
    tmp_path: Path,
):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(b"exe-fixture")
    _write_required_check_evidence(tmp_path)
    result = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(tmp_path)],
        cwd=ROOT,
        env=_release_env(WINDOWS_SIGNATURE_STATE="not_verified_notsigned"),
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    record = json.loads(
        (tmp_path / "qms-release-record.json").read_text(encoding="utf-8")
    )
    artifacts = {item["name"]: item for item in record["artifacts"]}
    assert (
        artifacts["radar-vital-windows-installer.exe"]["signing_state"]
        == "unsigned"
    )


def test_release_record_schema_couples_authorization_state_and_identity():
    schema = json.loads(
        (ROOT / "quality" / "schemas" / "release-record.schema.json").read_text(
            encoding="utf-8"
        )
    )
    authorization = schema["properties"]["authorization"]
    assert authorization["oneOf"][0]["properties"] == {
        "state": {"const": "pending"},
        "authorized_by": {"type": "null"},
        "authorized_at": {"type": "null"},
    }
    assert authorization["oneOf"][1]["properties"]["state"] == {
        "enum": ["authorized", "rejected"]
    }
    assert authorization["oneOf"][1]["properties"]["authorized_by"] == {
        "type": "string",
        "minLength": 1,
    }


def test_required_check_evidence_schema_is_strict_and_controlled():
    schema_path = (
        ROOT / "quality" / "schemas" / "required-check-evidence.schema.json"
    )
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    checks = schema["properties"]["checks"]
    assert schema["additionalProperties"] is False
    assert checks["minItems"] == checks["maxItems"] == 4
    assert checks["items"]["additionalProperties"] is False
    assert {
        "workflow_id",
        "path",
        "event",
        "run_attempt",
    }.issubset(checks["items"]["required"])
    assert set(checks["items"]["properties"]["workflow_name"]["enum"]) == set(
        name for name, _ in REQUIRED_WORKFLOWS
    )

    register = json.loads(
        (ROOT / "quality" / "document-register.json").read_text(encoding="utf-8")
    )
    entry = next(
        document
        for document in register["documents"]
        if document["path"]
        == "quality/schemas/required-check-evidence.schema.json"
    )
    assert entry["document_id"] == "RVT-QMS-SCH-007"
    assert entry["effective_product_version"] == _version()


def test_qms_record_captures_source_docs_artifacts_and_checksums(tmp_path: Path):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"signed-apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(
        b"windows-exe-fixture"
    )
    _write_required_check_evidence(tmp_path)
    env = _release_env(QMS_ATTESTATION_CONFIGURED="true")
    result = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(tmp_path)],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr

    record = json.loads(
        (tmp_path / "qms-release-record.json").read_text(encoding="utf-8")
    )
    assert record["schema_version"] == "rvt-qms-release-record-v1"
    assert record["release_id"] == f"RVT-REL-{_version()}"
    assert record["product_version"] == _version()
    assert record["source"] == {
        "repository": "example/radar-vital",
        "commit_sha": "0123456789abcdef0123456789abcdef01234567",
        "ref": f"refs/tags/v{_version()}",
    }
    assert record["build"]["run_id"] == "1234"
    assert record["build"]["run_attempt"] == 2
    assert record["controlled_documents"]["register_id"] == "RVT-QMS-REG-001"
    assert record["controlled_documents"]["documents"]
    assert all(
        len(document["sha256"]) == 64
        for document in record["controlled_documents"]["documents"]
    )
    artifacts = {item["name"]: item for item in record["artifacts"]}
    assert artifacts["radar-vital-release.apk"]["signing_state"] == "signed"
    assert artifacts["radar-vital-windows-installer.exe"]["signing_state"] == "signed"
    snapshot = json.loads(
        (tmp_path / "controlled-document-revisions.json").read_text(
            encoding="utf-8"
        )
    )
    assert snapshot["register"]["schema_version"] == (
        "rvt-qms-document-register-v1"
    )
    assert snapshot["documents"]
    assert all(len(item["sha256"]) == 64 for item in snapshot["documents"])

    sums = (tmp_path / "SHA256SUMS").read_text(encoding="utf-8")
    assert "  radar-vital-release.apk" in sums
    assert "  radar-vital-windows-installer.exe" in sums
    assert "  qms-release-record.json" in sums
    assert "  controlled-document-revisions.json" in sums

    pwsh = shutil.which("pwsh") or shutil.which("powershell")
    if not pwsh:
        pytest.skip("PowerShell 7 is not available for direct evidence verification")
    verified = subprocess.run(
        [pwsh, "-File", str(VERIFIER), "-DistDirectory", str(tmp_path)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert verified.returncode == 0, verified.stderr


def test_qms_record_captures_exact_source_required_check_evidence(tmp_path: Path):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(b"exe-fixture")
    evidence_path = _write_required_check_evidence(tmp_path)
    result = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(tmp_path)],
        cwd=ROOT,
        env=_release_env(REQUIRED_CHECK_EVIDENCE_PATH=str(evidence_path)),
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    record = json.loads(
        (tmp_path / "qms-release-record.json").read_text(encoding="utf-8")
    )
    protected = [
        item for item in record["verification"]
        if item["check_id"].startswith("protected-workflow-")
    ]
    assert len(protected) == 4
    assert all(item["conclusion"] == "passed" for item in protected)
    assert "required-check-evidence.json" in {
        item["name"] for item in record["artifacts"]
    }


def test_qms_record_rejects_required_check_evidence_for_other_source(tmp_path: Path):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(b"exe-fixture")
    evidence_path = _write_required_check_evidence(tmp_path, source_sha="f" * 40)
    result = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(tmp_path)],
        cwd=ROOT,
        env=_release_env(REQUIRED_CHECK_EVIDENCE_PATH=str(evidence_path)),
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "does not match" in result.stderr


def test_qms_record_rejects_schema_invalid_required_check_evidence(tmp_path: Path):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(b"exe-fixture")
    evidence_path = _write_required_check_evidence(tmp_path)
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    evidence["checks"][0]["uncontrolled_field"] = True
    evidence_path.write_text(json.dumps(evidence), encoding="utf-8")

    result = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(tmp_path)],
        cwd=ROOT,
        env=_release_env(),
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "Required-check evidence failed schema validation" in result.stderr
    assert "unexpected uncontrolled_field" in result.stderr


def test_qms_record_rejects_duplicate_protected_workflow_evidence(tmp_path: Path):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(b"exe-fixture")
    evidence_path = _write_required_check_evidence(tmp_path)
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    evidence["checks"][-1]["workflow_name"] = evidence["checks"][0][
        "workflow_name"
    ]
    evidence_path.write_text(json.dumps(evidence), encoding="utf-8")

    result = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(tmp_path)],
        cwd=ROOT,
        env=_release_env(),
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "Required-check evidence entry is invalid" in result.stderr


def test_qms_generator_requires_evidence_unless_fixture_escape_is_explicit(
    tmp_path: Path,
):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(b"exe-fixture")
    missing = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(tmp_path)],
        cwd=ROOT,
        env=_release_env(),
        capture_output=True,
        text=True,
        check=False,
    )
    assert missing.returncode != 0
    assert "Required-check evidence is required" in missing.stderr

    fixture = subprocess.run(
        [
            "node",
            str(QMS_GENERATOR),
            "--dist",
            str(tmp_path),
            "--allow-missing-required-check-evidence-for-nonpublication-fixture",
        ],
        cwd=ROOT,
        env=_release_env(),
        capture_output=True,
        text=True,
        check=False,
    )
    assert fixture.returncode == 0, fixture.stderr
    record = json.loads(
        (tmp_path / "qms-release-record.json").read_text(encoding="utf-8")
    )
    assert record["authorization"]["state"] == "pending"
    assert not any(
        check["check_id"].startswith("protected-workflow-")
        for check in record["verification"]
    )

    publication = subprocess.run(
        [
            "node",
            str(QMS_GENERATOR),
            "--dist",
            str(tmp_path),
            "--allow-missing-required-check-evidence-for-nonpublication-fixture",
        ],
        cwd=ROOT,
        env=_release_env(
            QMS_AUTHORIZATION_STATE="authorized",
            QMS_AUTHORIZED_BY="release-manager",
            QMS_AUTHORIZED_AT="2026-08-19T00:00:00Z",
        ),
        capture_output=True,
        text=True,
        check=False,
    )
    assert publication.returncode != 0
    assert "only valid for pending release records" in publication.stderr


def test_release_workflow_attests_and_publishes_qms_evidence():
    workflow = WORKFLOW.read_text(encoding="utf-8")
    latest = LATEST_GENERATOR.read_text(encoding="utf-8")
    verifier = VERIFIER.read_text(encoding="utf-8")

    assert "attestations: write" in workflow
    assert "actions: read" in workflow
    assert "actions/attest-build-provenance@" in workflow
    assert "node scripts/generate-qms-release-record.mjs" in workflow
    assert "dist/qms-release-record.json" in workflow
    assert "dist/SHA256SUMS" in workflow
    assert "GITHUB_RUN_ATTEMPT: ${{ github.run_attempt }}" in workflow
    assert 'tag_product_version="${release_version%%[-+]*}"' in workflow
    assert "does not claim regulatory certification" in workflow
    assert "ref: ${{ github.sha }}" in workflow
    assert 'actual_sha="$(git rev-parse HEAD)"' in workflow
    assert workflow.count("node scripts/check-release-source.mjs") == 2
    assert "node scripts/check-release-source.mjs --verify-only" in workflow
    assert 'gh release create "${release_args[@]}"' in workflow
    assert "softprops/action-gh-release" not in workflow
    assert "dist/required-check-evidence.json" in workflow
    assert "REQUIRED_CHECK_EVIDENCE_PATH" in workflow
    assert "python -m pytest tests -q" not in workflow
    assert "npx playwright test tests/smoke" not in workflow
    assert "npm run build:check" in workflow
    assert "QMS_PRODUCTION_RELEASE_APPROVAL" in workflow
    assert 'git merge-base --is-ancestor "$GITHUB_SHA" origin/main' in workflow
    assert "/actions/runs/${GITHUB_RUN_ID}/approvals" in workflow
    assert '.name == "release-production"' in workflow
    assert "authorized_by=${authorizer}" in workflow
    assert "Diagnostic publication remains pending" in workflow
    assert "release_args+=(--prerelease)" in workflow
    assert "if: steps.authorization.outputs.state == 'authorized'" in workflow

    assert "qms_release_record" in latest
    assert "sourceCommit" in latest
    assert "workflowRunAttempt" in latest
    assert "signature_state" in latest
    assert "QmsReleaseRecord" in verifier
    assert "$record.product_version" in verifier
    assert "$expectedChecksumNames.Add('qms-release-record.json')" in verifier
    assert "--verify-record" in verifier
    assert "Duplicate recorded release artifact" in verifier
    assert "Duplicate SHA256SUMS entry" in verifier
    assert "SHA256SUMS coverage mismatch" in verifier


def test_production_release_requires_every_native_signature_proof():
    workflow = WORKFLOW.read_text(encoding="utf-8")

    assert (
        "updater_signature_state: "
        "${{ steps.windows_updater_signature_state.outputs.signature_state }}"
        in workflow
    )
    assert (
        "WINDOWS_UPDATER_SIGNATURE_STATE: "
        "${{ needs.windows.outputs.updater_signature_state }}"
        in workflow
    )
    assert '"signature_state=signed" >> $env:GITHUB_OUTPUT' in workflow
    assert '"signature_state=missing" >> $env:GITHUB_OUTPUT' in workflow
    android_gate = 'if [ "$ANDROID_SIGNATURE_STATE" != "signed_release" ]; then'
    windows_gate = (
        'if [ "$WINDOWS_SIGNATURE_STATE" != "authenticode_verified" ]; then'
    )
    updater_gate = 'if [ "$WINDOWS_UPDATER_SIGNATURE_STATE" != "signed" ]; then'
    authorization = 'echo "state=authorized" >> "$GITHUB_OUTPUT"'
    assert android_gate in workflow
    assert windows_gate in workflow
    assert updater_gate in workflow
    assert workflow.index("Diagnostic publication remains pending") < workflow.index(
        android_gate
    )
    assert workflow.index(android_gate) < workflow.index(authorization)
    assert workflow.index(windows_gate) < workflow.index(authorization)
    assert workflow.index(updater_gate) < workflow.index(authorization)


def test_updater_signed_state_requires_cryptographic_verification():
    workflow = WORKFLOW.read_text(encoding="utf-8")
    verifier = UPDATER_SIGNATURE_VERIFIER.read_text(encoding="utf-8")
    stage_start = workflow.index("- name: Stage EXE release asset")
    stage_end = workflow.index(
        "- name: Record Windows Authenticode signature state", stage_start
    )
    stage = workflow[stage_start:stage_end]
    verify_command = (
        "cargo run --quiet --release --locked --manifest-path "
        "src-tauri/Cargo.toml --example verify_updater_signature"
    )
    signed_output = '"signature_state=signed" >> $env:GITHUB_OUTPUT'

    assert (
        "TAURI_UPDATER_PUBLIC_KEY: ${{ secrets.TAURI_UPDATER_PUBLIC_KEY }}"
        in stage
    )
    assert '$sigPath = "$($exe.FullName).sig"' in stage
    assert verify_command in stage
    assert "$LASTEXITCODE -ne 0" in stage
    assert stage.index(verify_command) < stage.index(signed_output)
    assert '"dist\\radar-vital-windows-installer.exe"' in stage
    assert '"dist\\radar-vital-windows-installer.exe.sig"' in stage
    assert "PublicKey::from_base64" in verifier
    assert "Signature::from_file" in verifier
    assert ".verify_stream(signature)" in verifier
    assert ".finalize()" in verifier
    assert "rejects_signature_for_modified_bytes" in verifier
    assert "rejects_signature_from_different_public_key" in verifier


def test_windows_nsis_build_retries_transient_dependency_downloads():
    workflow = WORKFLOW.read_text(encoding="utf-8")

    retry_limit = "$maxAttempts = 3"
    build_command = "cargo tauri build --verbose --bundles nsis"
    terminal_failure = (
        'throw "Tauri NSIS build failed after $maxAttempts attempts '
        '(exit code $exitCode)."'
    )
    assert retry_limit in workflow
    assert "for ($attempt = 1; $attempt -le $maxAttempts; $attempt++)" in workflow
    assert build_command in workflow
    assert "Start-Sleep -Seconds $delaySeconds" in workflow
    assert terminal_failure in workflow
    assert workflow.index(retry_limit) < workflow.index(build_command)
    assert workflow.index(build_command) < workflow.index(terminal_failure)


def _verify_qms_evidence(dist: Path) -> subprocess.CompletedProcess[str]:
    pwsh = shutil.which("pwsh") or shutil.which("powershell")
    if not pwsh:
        pytest.skip("PowerShell is not available for direct evidence verification")
    return subprocess.run(
        [pwsh, "-NoProfile", "-File", str(VERIFIER), "-DistDirectory", str(dist)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def _generate_qms_evidence(dist: Path) -> dict:
    (dist / "radar-vital-release.apk").write_bytes(b"signed-apk-fixture")
    (dist / "radar-vital-windows-installer.exe").write_bytes(
        b"windows-exe-fixture"
    )
    _write_required_check_evidence(dist)
    result = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(dist)],
        cwd=ROOT,
        env=_release_env(),
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return json.loads(
        (dist / "qms-release-record.json").read_text(encoding="utf-8")
    )


def test_qms_verifier_rejects_schema_invalid_record(tmp_path: Path):
    record = _generate_qms_evidence(tmp_path)
    del record["build"]["run_attempt"]
    (tmp_path / "qms-release-record.json").write_text(
        json.dumps(record), encoding="utf-8"
    )
    verified = _verify_qms_evidence(tmp_path)
    assert verified.returncode != 0
    assert "schema validation" in (verified.stdout + verified.stderr).lower()


@pytest.mark.parametrize("artifact_name", ["../outside.apk", "nested/file.apk"])
def test_qms_verifier_rejects_artifact_path_escape(
    tmp_path: Path, artifact_name: str
):
    record = _generate_qms_evidence(tmp_path)
    record["artifacts"][0]["name"] = artifact_name
    (tmp_path / "qms-release-record.json").write_text(
        json.dumps(record), encoding="utf-8"
    )
    verified = _verify_qms_evidence(tmp_path)
    assert verified.returncode != 0
    assert "contained basename" in (verified.stdout + verified.stderr)


def test_qms_verifier_rejects_duplicate_artifacts(tmp_path: Path):
    record = _generate_qms_evidence(tmp_path)
    record["artifacts"].append(dict(record["artifacts"][0]))
    (tmp_path / "qms-release-record.json").write_text(
        json.dumps(record), encoding="utf-8"
    )
    verified = _verify_qms_evidence(tmp_path)
    assert verified.returncode != 0
    assert "Duplicate recorded release artifact" in (
        verified.stdout + verified.stderr
    )


def test_qms_verifier_requires_exact_unique_checksum_coverage(tmp_path: Path):
    _generate_qms_evidence(tmp_path)
    sums_path = tmp_path / "SHA256SUMS"
    lines = sums_path.read_text(encoding="utf-8").splitlines()

    sums_path.write_text("\n".join([*lines, lines[0]]) + "\n", encoding="utf-8")
    duplicate = _verify_qms_evidence(tmp_path)
    assert duplicate.returncode != 0
    assert "Duplicate SHA256SUMS entry" in (duplicate.stdout + duplicate.stderr)

    sums_path.write_text("\n".join(lines[1:]) + "\n", encoding="utf-8")
    missing = _verify_qms_evidence(tmp_path)
    assert missing.returncode != 0
    assert "SHA256SUMS coverage mismatch" in (missing.stdout + missing.stderr)

    sums_path.write_text(
        "\n".join([*lines, f"{'0' * 64}  unexpected.bin"]) + "\n",
        encoding="utf-8",
    )
    unexpected = _verify_qms_evidence(tmp_path)
    assert unexpected.returncode != 0
    assert "SHA256SUMS coverage mismatch" in (
        unexpected.stdout + unexpected.stderr
    )


def test_qms_generator_requires_complete_authorization_evidence(tmp_path: Path):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"apk")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(b"exe")
    incomplete = subprocess.run(
        ["node", str(QMS_GENERATOR), "--dist", str(tmp_path)],
        cwd=ROOT,
        env=_release_env(QMS_AUTHORIZATION_STATE="authorized"),
        capture_output=True,
        text=True,
        check=False,
    )
    assert incomplete.returncode != 0
    assert "requires an authorizer" in incomplete.stderr
