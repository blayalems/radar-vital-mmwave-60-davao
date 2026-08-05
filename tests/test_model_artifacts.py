import builtins
import json
import pickle

import pytest

from rvt_trainer.model_artifacts import (
    FEATURE_CONTRACT_SCHEMA_VERSION,
    MODEL_BUNDLE_SCHEMA_VERSION,
    FeatureContract,
    assert_bundle_inference_compatible,
    assert_feature_contract_compatible,
    build_model_bundle_metadata,
    build_reproducibility_record,
    capture_dependency_versions,
    deployment_feasibility,
    stable_contract_hash,
)
from rvt_trainer.modeling import (
    MODEL_FAMILY_CNN_1D,
    MODEL_FAMILY_GRADIENT_BOOSTING,
)
from rvt_trainer.monolith import (
    MODEL_BUNDLE_METADATA_FILENAME,
    _read_model_bundle_metadata,
    _verify_model_dir,
    _write_model_manifest,
)


def _contract():
    return FeatureContract.create(
        base_feature_cols=["range", "phase"],
        expanded_feature_cols=["range", "phase", "phase__missing"],
        feature_mode="safe",
        feature_engineering_version="17",
        feature_schema_version="v3",
        feature_schema_hash="a" * 64,
    )


def test_feature_contract_hash_is_stable_and_order_sensitive():
    contract = _contract()
    assert contract.as_dict()["schema_version"] == FEATURE_CONTRACT_SCHEMA_VERSION
    assert contract.contract_hash == stable_contract_hash(contract.semantic_dict())

    reordered = FeatureContract.create(
        base_feature_cols=["phase", "range"],
        expanded_feature_cols=["phase", "range", "phase__missing"],
        feature_mode="safe",
        feature_engineering_version="17",
        feature_schema_version="v3",
        feature_schema_hash="a" * 64,
    )
    assert reordered.contract_hash != contract.contract_hash


def test_inference_contract_rejects_reordered_features():
    with pytest.raises(ValueError, match="feature order differs"):
        assert_feature_contract_compatible(
            ["phase", "range", "phase__missing"], _contract()
        )


def test_inference_contract_reports_missing_and_extra_features():
    with pytest.raises(ValueError, match=r"missing=.*phase__missing.*extra=.*noise"):
        assert_feature_contract_compatible(["range", "phase", "noise"], _contract())


def test_gbr_metadata_is_default_runtime_without_tensorflow_import(monkeypatch):
    imported = []
    original_import = builtins.__import__

    def recording_import(name, *args, **kwargs):
        imported.append(name)
        if name == "tensorflow":
            raise AssertionError("GBR metadata must not import TensorFlow")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", recording_import)
    bundle = build_model_bundle_metadata(
        trainer_version="16.5.8",
        model_family=MODEL_FAMILY_GRADIENT_BOOSTING,
        trained_targets=["hr", "rr"],
        feature_contract=_contract(),
        training_config={"n_estimators": 150},
        seed=73,
        created_at="2026-07-29T00:00:00Z",
    )

    assert bundle["schema_version"] == MODEL_BUNDLE_SCHEMA_VERSION
    assert bundle["model_family"] == MODEL_FAMILY_GRADIENT_BOOSTING
    assert bundle["reproducibility"]["random_seed"] == 73
    assert bundle["dependencies"]["scikit-learn"]
    assert "tensorflow" not in bundle["dependencies"]
    assert "tensorflow" not in imported


def test_cnn_metadata_requires_and_checks_window_contract():
    with pytest.raises(ValueError, match="sequence.window_size"):
        build_model_bundle_metadata(
            trainer_version="16.5.8",
            model_family=MODEL_FAMILY_CNN_1D,
            trained_targets=["hr"],
            feature_contract=_contract(),
            training_config={},
        )

    bundle = build_model_bundle_metadata(
        trainer_version="16.5.8",
        model_family=MODEL_FAMILY_CNN_1D,
        trained_targets=["hr"],
        feature_contract=_contract(),
        training_config={"sequence": {"window_size": 32}},
        created_at="2026-07-29T00:00:00Z",
    )
    with pytest.raises(ValueError, match="window mismatch"):
        assert_bundle_inference_compatible(
            bundle,
            actual_columns=_contract().expanded_feature_cols,
            requested_model_family=MODEL_FAMILY_CNN_1D,
            sequence_window_size=16,
        )


def test_bundle_rejects_corrupt_feature_contract_hash():
    bundle = build_model_bundle_metadata(
        trainer_version="16.5.8",
        model_family=MODEL_FAMILY_GRADIENT_BOOSTING,
        trained_targets=["rr"],
        feature_contract=_contract(),
        training_config={"n_estimators": 100},
        created_at="2026-07-29T00:00:00Z",
    )
    bundle = json.loads(json.dumps(bundle))
    bundle["feature_contract"]["contract_hash"] = "0" * 64
    with pytest.raises(ValueError, match="contract hash is invalid"):
        assert_bundle_inference_compatible(
            bundle,
            actual_columns=_contract().expanded_feature_cols,
        )


def test_bundle_rejects_metadata_changed_after_contract_was_created():
    bundle = build_model_bundle_metadata(
        trainer_version="16.5.8",
        model_family=MODEL_FAMILY_GRADIENT_BOOSTING,
        trained_targets=["hr"],
        feature_contract=_contract(),
        training_config={"n_estimators": 100},
        created_at="2026-07-29T00:00:00Z",
    )
    bundle["training_config"]["n_estimators"] = 101
    with pytest.raises(ValueError, match="bundle contract hash is invalid"):
        assert_bundle_inference_compatible(
            bundle,
            actual_columns=_contract().expanded_feature_cols,
        )


def test_reproducibility_record_discloses_determinism_limits(monkeypatch):
    monkeypatch.delenv("PYTHONHASHSEED", raising=False)
    record = build_reproducibility_record(42, model_family=MODEL_FAMILY_CNN_1D)
    assert record["target_seed_policy"] == {"hr": 42, "rr": 43}
    assert record["python_hash_seed_fixed_at_process_start"] is False
    assert record["tensorflow_deterministic_ops_requested"] is True
    assert "dependency versions" in record["determinism_limit"]


def test_dependency_capture_reports_missing_optional_tensorflow_without_import():
    def fake_version(name):
        if name == "tensorflow":
            from importlib import metadata

            raise metadata.PackageNotFoundError(name)
        return "1.2.3"

    versions = capture_dependency_versions(
        MODEL_FAMILY_CNN_1D, distribution_version=fake_version
    )
    assert versions == {
        "numpy": "1.2.3",
        "pandas": "1.2.3",
        "scikit-learn": "1.2.3",
        "tensorflow": None,
    }


@pytest.mark.parametrize(
    ("family", "host", "embedded"),
    [
        (MODEL_FAMILY_GRADIENT_BOOSTING, "supported", "not_qualified"),
        (MODEL_FAMILY_CNN_1D, "experimental", "not_qualified"),
    ],
)
def test_deployment_feasibility_is_conservative(family, host, embedded):
    result = deployment_feasibility(family)
    assert result["host_inference"] == host
    assert result["embedded_inference"] == embedded


def test_bundle_metadata_is_in_signed_artifact_boundary_and_loads(tmp_path):
    model_dir = tmp_path / "model"
    model_dir.mkdir()
    for name in ("model_hr.pkl", "preprocessor.pkl"):
        with (model_dir / name).open("wb") as handle:
            pickle.dump({"artifact": name}, handle)
    bundle = build_model_bundle_metadata(
        trainer_version="16.5.8",
        model_family=MODEL_FAMILY_GRADIENT_BOOSTING,
        trained_targets=["hr"],
        feature_contract=_contract(),
        training_config={"model_params": {"n_estimators": 100}, "sequence": None},
        created_at="2026-07-29T00:00:00Z",
    )
    (model_dir / MODEL_BUNDLE_METADATA_FILENAME).write_text(
        json.dumps(bundle), encoding="utf-8"
    )

    manifest_path = _write_model_manifest(str(model_dir))
    manifest = json.loads(
        (model_dir / "model_manifest.json").read_text(encoding="utf-8")
    )

    assert manifest_path == str(model_dir / "model_manifest.json")
    assert MODEL_BUNDLE_METADATA_FILENAME in manifest["artifacts"]
    _verify_model_dir(str(model_dir))
    assert _read_model_bundle_metadata(str(model_dir))["bundle_contract_hash"] == (
        bundle["bundle_contract_hash"]
    )


def test_legacy_verified_bundle_without_v2_metadata_remains_supported(tmp_path):
    model_dir = tmp_path / "legacy"
    model_dir.mkdir()
    for name in ("model_rr.pkl", "preprocessor.pkl"):
        with (model_dir / name).open("wb") as handle:
            pickle.dump({"artifact": name}, handle)
    _write_model_manifest(str(model_dir))

    _verify_model_dir(str(model_dir))
    assert _read_model_bundle_metadata(str(model_dir)) is None
