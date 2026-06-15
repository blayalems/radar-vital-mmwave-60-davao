"""Tests for the model integrity manifest (audit item D4).

These cover the manifest writer (`_write_model_manifest` / `_build_model_manifest`)
and the pre-unpickle verifier (`_verify_model_dir`) in rvt_trainer.monolith. They
exercise the helpers directly against tiny fake .pkl artifacts so they stay fast
and avoid the heavy training path.
"""

import json
import os
import pickle

import pytest

from rvt_trainer.monolith import (
    MODEL_ARTIFACT_NAMES,
    MODEL_MANIFEST_FILENAME,
    MODEL_MANIFEST_SCHEMA_VERSION,
    REQUIRE_MODEL_MANIFEST_ENV,
    MODEL_SIGNING_KEY_ENV,
    _build_model_manifest,
    _file_sha256,
    _verify_model_dir,
    _write_model_manifest,
)


def _make_fake_model_dir(tmp_path, names=MODEL_ARTIFACT_NAMES):
    """Create a model dir with tiny pickled artifacts and return its path."""
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    for name in names:
        with open(model_dir / name, "wb") as f:
            pickle.dump({"artifact": name, "payload": list(range(8))}, f)
    return str(model_dir)


@pytest.fixture(autouse=True)
def _clean_manifest_env(monkeypatch):
    """Ensure manifest-related env vars never leak between tests."""
    monkeypatch.delenv(MODEL_SIGNING_KEY_ENV, raising=False)
    monkeypatch.delenv(REQUIRE_MODEL_MANIFEST_ENV, raising=False)
    yield


def test_write_model_manifest_records_correct_hashes(tmp_path):
    model_dir = _make_fake_model_dir(tmp_path)
    manifest_path = _write_model_manifest(model_dir)

    assert manifest_path == os.path.join(model_dir, MODEL_MANIFEST_FILENAME)
    assert os.path.exists(manifest_path)

    with open(manifest_path, "r", encoding="utf-8") as f:
        manifest = json.load(f)

    assert manifest["schema_version"] == MODEL_MANIFEST_SCHEMA_VERSION
    assert "created_at" in manifest and manifest["created_at"]
    assert "trainer_version" in manifest

    artifacts = manifest["artifacts"]
    assert set(artifacts) == set(MODEL_ARTIFACT_NAMES)
    for name, recorded in artifacts.items():
        assert recorded == _file_sha256(os.path.join(model_dir, name))

    # No signing key configured -> no HMAC field.
    assert "hmac_sha256" not in manifest


def test_build_manifest_only_lists_present_artifacts(tmp_path):
    model_dir = _make_fake_model_dir(tmp_path, names=("model_hr.pkl", "preprocessor.pkl"))
    manifest = _build_model_manifest(model_dir)
    assert set(manifest["artifacts"]) == {"model_hr.pkl", "preprocessor.pkl"}


def test_verify_model_dir_passes_for_intact_dir(tmp_path):
    model_dir = _make_fake_model_dir(tmp_path)
    _write_model_manifest(model_dir)
    # Should not raise.
    _verify_model_dir(model_dir)


def test_verify_model_dir_raises_on_tampered_artifact(tmp_path):
    model_dir = _make_fake_model_dir(tmp_path)
    _write_model_manifest(model_dir)

    # Mutate one artifact after the manifest was written.
    with open(os.path.join(model_dir, "model_hr.pkl"), "ab") as f:
        f.write(b"tampered-bytes")

    with pytest.raises(ValueError, match="integrity check FAILED"):
        _verify_model_dir(model_dir)


def test_verify_model_dir_raises_on_missing_artifact(tmp_path):
    model_dir = _make_fake_model_dir(tmp_path)
    _write_model_manifest(model_dir)

    os.remove(os.path.join(model_dir, "model_rr.pkl"))

    with pytest.raises(ValueError, match="missing"):
        _verify_model_dir(model_dir)


def test_verify_model_dir_warns_and_proceeds_without_manifest(tmp_path, capsys):
    model_dir = _make_fake_model_dir(tmp_path)
    # No manifest written, no enforcement env -> warn to stderr, do not raise.
    _verify_model_dir(model_dir)
    err = capsys.readouterr().err
    assert "WARN" in err
    assert MODEL_MANIFEST_FILENAME in err


def test_verify_model_dir_raises_without_manifest_when_required(tmp_path, monkeypatch):
    model_dir = _make_fake_model_dir(tmp_path)
    monkeypatch.setenv(REQUIRE_MODEL_MANIFEST_ENV, "1")
    with pytest.raises(ValueError, match="No model_manifest.json"):
        _verify_model_dir(model_dir)


def test_verify_model_dir_raises_on_missing_dir(tmp_path):
    with pytest.raises(ValueError, match="does not exist"):
        _verify_model_dir(str(tmp_path / "nope"))


def test_verify_model_dir_raises_on_corrupt_manifest(tmp_path):
    model_dir = _make_fake_model_dir(tmp_path)
    with open(os.path.join(model_dir, MODEL_MANIFEST_FILENAME), "w", encoding="utf-8") as f:
        f.write("{not valid json")
    with pytest.raises(ValueError):
        _verify_model_dir(model_dir)


def test_hmac_signed_manifest_round_trip(tmp_path, monkeypatch):
    monkeypatch.setenv(MODEL_SIGNING_KEY_ENV, "unit-test-signing-key")
    model_dir = _make_fake_model_dir(tmp_path)
    manifest_path = _write_model_manifest(model_dir)

    with open(manifest_path, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    assert isinstance(manifest.get("hmac_sha256"), str) and manifest["hmac_sha256"]

    # Correct key -> passes.
    _verify_model_dir(model_dir)


def test_hmac_verification_fails_with_wrong_key(tmp_path, monkeypatch):
    monkeypatch.setenv(MODEL_SIGNING_KEY_ENV, "correct-key")
    model_dir = _make_fake_model_dir(tmp_path)
    _write_model_manifest(model_dir)

    monkeypatch.setenv(MODEL_SIGNING_KEY_ENV, "attacker-key")
    with pytest.raises(ValueError, match="HMAC verification FAILED"):
        _verify_model_dir(model_dir)


def test_hmac_required_but_absent_in_manifest(tmp_path, monkeypatch):
    # Manifest written WITHOUT a key (no HMAC), then a key is configured at verify time.
    model_dir = _make_fake_model_dir(tmp_path)
    _write_model_manifest(model_dir)

    monkeypatch.setenv(MODEL_SIGNING_KEY_ENV, "now-required")
    with pytest.raises(ValueError, match="no HMAC signature"):
        _verify_model_dir(model_dir)


# ---------------------------------------------------------------------------
# Structural artifact-set validation (PR #58 review: close model-integrity
# bypass where a manifest omits preprocessor.pkl or lists unknown / path-like
# names, while cmd_predict ALWAYS unpickles preprocessor.pkl).
# ---------------------------------------------------------------------------

def _make_dir_with_manifest_artifacts(tmp_path, file_names, manifest_artifacts):
    """Create a model dir with real .pkl files `file_names` and a manifest that
    declares exactly `manifest_artifacts` (name -> recorded sha256 of an
    on-disk file). Returns the model dir path.
    """
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    for name in file_names:
        with open(model_dir / os.path.basename(name), "wb") as f:
            pickle.dump({"artifact": name, "payload": list(range(8))}, f)

    artifacts = {}
    for name in manifest_artifacts:
        # Hash a real on-disk file so the only thing under test is the
        # structural check, not a hash mismatch. Path-like names hash whatever
        # is at model_dir/basename so the test reaches the structural reject.
        path = os.path.join(str(model_dir), os.path.basename(name))
        digest = _file_sha256(path)
        artifacts[name] = digest if digest is not None else "0" * 64

    manifest = {
        "schema_version": MODEL_MANIFEST_SCHEMA_VERSION,
        "trainer_version": "test",
        "created_at": "2026-01-01T00:00:00Z",
        "artifacts": artifacts,
    }
    with open(model_dir / MODEL_MANIFEST_FILENAME, "w", encoding="utf-8") as f:
        json.dump(manifest, f)
    return str(model_dir)


def test_verify_raises_when_manifest_missing_preprocessor(tmp_path):
    # cmd_predict always unpickles preprocessor.pkl; a manifest that omits it
    # must be rejected BEFORE any artifact is trusted/unpickled.
    model_dir = _make_dir_with_manifest_artifacts(
        tmp_path,
        file_names=("model_hr.pkl", "model_rr.pkl", "preprocessor.pkl"),
        manifest_artifacts=("model_hr.pkl", "model_rr.pkl"),
    )
    with pytest.raises(ValueError, match="preprocessor.pkl"):
        _verify_model_dir(model_dir)


def test_verify_raises_on_unknown_traversal_artifact_name(tmp_path):
    # A path-like / traversal name in the manifest must be rejected.
    model_dir = _make_dir_with_manifest_artifacts(
        tmp_path,
        file_names=("model_hr.pkl", "preprocessor.pkl"),
        manifest_artifacts=("model_hr.pkl", "preprocessor.pkl", "../evil.pkl"),
    )
    with pytest.raises(ValueError, match="unknown artifact"):
        _verify_model_dir(model_dir)


def test_verify_raises_on_unknown_plain_artifact_name(tmp_path):
    # An unrelated but non-path name must also be rejected.
    model_dir = _make_dir_with_manifest_artifacts(
        tmp_path,
        file_names=("model_hr.pkl", "preprocessor.pkl"),
        manifest_artifacts=("model_hr.pkl", "preprocessor.pkl", "model_xx.pkl"),
    )
    with pytest.raises(ValueError, match="unknown artifact"):
        _verify_model_dir(model_dir)


def test_verify_passes_for_hr_only_with_preprocessor(tmp_path):
    # HR-only (model_hr.pkl + preprocessor.pkl) is valid.
    model_dir = _make_dir_with_manifest_artifacts(
        tmp_path,
        file_names=("model_hr.pkl", "preprocessor.pkl"),
        manifest_artifacts=("model_hr.pkl", "preprocessor.pkl"),
    )
    _verify_model_dir(model_dir)  # should not raise


def test_verify_passes_for_rr_only_with_preprocessor(tmp_path):
    # RR-only (model_rr.pkl + preprocessor.pkl) is valid.
    model_dir = _make_dir_with_manifest_artifacts(
        tmp_path,
        file_names=("model_rr.pkl", "preprocessor.pkl"),
        manifest_artifacts=("model_rr.pkl", "preprocessor.pkl"),
    )
    _verify_model_dir(model_dir)  # should not raise


def test_verify_raises_when_manifest_has_only_preprocessor(tmp_path):
    # preprocessor.pkl alone (no model) cannot satisfy prediction; reject.
    model_dir = _make_dir_with_manifest_artifacts(
        tmp_path,
        file_names=("preprocessor.pkl",),
        manifest_artifacts=("preprocessor.pkl",),
    )
    with pytest.raises(ValueError, match="no model artifact"):
        _verify_model_dir(model_dir)
