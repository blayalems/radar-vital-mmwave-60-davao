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


def test_qms_record_captures_source_docs_artifacts_and_checksums(tmp_path: Path):
    (tmp_path / "radar-vital-release.apk").write_bytes(b"signed-apk-fixture")
    (tmp_path / "radar-vital-windows-installer.exe").write_bytes(
        b"windows-exe-fixture"
    )
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


def test_release_workflow_attests_and_publishes_qms_evidence():
    workflow = WORKFLOW.read_text(encoding="utf-8")
    latest = LATEST_GENERATOR.read_text(encoding="utf-8")
    verifier = VERIFIER.read_text(encoding="utf-8")

    assert "attestations: write" in workflow
    assert "actions: read" in workflow
    assert "actions/attest-build-provenance@v3" in workflow
    assert "node scripts/generate-qms-release-record.mjs" in workflow
    assert "dist/qms-release-record.json" in workflow
    assert "dist/SHA256SUMS" in workflow
    assert "GITHUB_RUN_ATTEMPT: ${{ github.run_attempt }}" in workflow
    assert 'tag_product_version="${release_version%%[-+]*}"' in workflow
    assert "does not claim regulatory certification" in workflow
    assert "ref: ${{ github.sha }}" in workflow
    assert 'actual_sha="$(git rev-parse HEAD)"' in workflow
    assert "npm run test:source-integrity" in workflow
    assert "npm run test:qms-contract" in workflow
    assert "python -m pytest tests -q" in workflow
    assert "npm run test:unit:web" in workflow
    assert "npx playwright test tests/smoke" in workflow
    assert "npm run build:check" in workflow
    assert "QMS_PRODUCTION_RELEASE_APPROVAL" in workflow
    assert 'git merge-base --is-ancestor "$GITHUB_SHA" origin/main' in workflow
    assert "/actions/runs/${GITHUB_RUN_ID}/approvals" in workflow
    assert '.name == "release-production"' in workflow
    assert "authorized_by=${authorizer}" in workflow
    assert "Diagnostic publication remains pending" in workflow
    assert "prerelease: ${{ steps.authorization.outputs.prerelease }}" in workflow
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
