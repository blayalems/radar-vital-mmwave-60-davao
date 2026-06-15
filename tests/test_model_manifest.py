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
