"""Reproducible model-bundle metadata and inference compatibility checks.

This module deliberately has no TensorFlow import.  Gradient boosting is the
default deployment family, while the optional CNN runtime remains lazy.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
from importlib import metadata
import json
import os
import platform
import sys
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence

from .modeling import (
    MODEL_FAMILY_CNN_1D,
    MODEL_FAMILY_GRADIENT_BOOSTING,
    MODEL_FAMILIES,
)


MODEL_BUNDLE_SCHEMA_VERSION = "rvt-model-bundle-v2"
MODEL_BUNDLE_METADATA_FILENAME = "model_bundle_metadata.json"
FEATURE_CONTRACT_SCHEMA_VERSION = "rvt-feature-contract-v1"
DEFAULT_RANDOM_SEED = 42


def _canonical_json(value: object) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def stable_contract_hash(value: object) -> str:
    """Return the SHA-256 of canonical JSON for a semantic contract."""

    return hashlib.sha256(_canonical_json(value)).hexdigest()


def _ordered_unique_strings(values: Iterable[object], field: str) -> tuple[str, ...]:
    result = tuple(str(value) for value in values)
    if not result or any(not value for value in result):
        raise ValueError(f"{field} must contain non-empty feature names")
    if len(set(result)) != len(result):
        raise ValueError(f"{field} contains duplicate feature names")
    return result


@dataclass(frozen=True)
class FeatureContract:
    """The exact, ordered feature interface consumed by a saved model."""

    base_feature_cols: tuple[str, ...]
    expanded_feature_cols: tuple[str, ...]
    feature_mode: str
    feature_engineering_version: str
    feature_schema_version: str
    feature_schema_hash: str
    allow_policy_features: bool = False

    @classmethod
    def create(
        cls,
        *,
        base_feature_cols: Sequence[object],
        expanded_feature_cols: Sequence[object],
        feature_mode: object,
        feature_engineering_version: object,
        feature_schema_version: object,
        feature_schema_hash: object,
        allow_policy_features: bool = False,
    ) -> "FeatureContract":
        contract = cls(
            base_feature_cols=_ordered_unique_strings(
                base_feature_cols, "base_feature_cols"
            ),
            expanded_feature_cols=_ordered_unique_strings(
                expanded_feature_cols, "expanded_feature_cols"
            ),
            feature_mode=str(feature_mode),
            feature_engineering_version=str(feature_engineering_version),
            feature_schema_version=str(feature_schema_version),
            feature_schema_hash=str(feature_schema_hash),
            allow_policy_features=bool(allow_policy_features),
        )
        contract.validate()
        return contract

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> "FeatureContract":
        return cls.create(
            base_feature_cols=value.get("base_feature_cols", ()),
            expanded_feature_cols=value.get("expanded_feature_cols", ()),
            feature_mode=value.get("feature_mode", ""),
            feature_engineering_version=value.get(
                "feature_engineering_version", ""
            ),
            feature_schema_version=value.get("feature_schema_version", ""),
            feature_schema_hash=value.get("feature_schema_hash", ""),
            allow_policy_features=bool(value.get("allow_policy_features", False)),
        )

    def validate(self) -> None:
        if not self.feature_mode:
            raise ValueError("feature_mode must be recorded")
        if not self.feature_engineering_version:
            raise ValueError("feature_engineering_version must be recorded")
        if not self.feature_schema_version or not self.feature_schema_hash:
            raise ValueError("feature schema version and hash must be recorded")
        unknown_base = set(self.base_feature_cols) - set(self.expanded_feature_cols)
        if unknown_base:
            raise ValueError(
                "expanded_feature_cols omits base feature(s): "
                + ", ".join(sorted(unknown_base))
            )

    def semantic_dict(self) -> Dict[str, object]:
        value = asdict(self)
        value["base_feature_cols"] = list(self.base_feature_cols)
        value["expanded_feature_cols"] = list(self.expanded_feature_cols)
        return {
            "schema_version": FEATURE_CONTRACT_SCHEMA_VERSION,
            **value,
        }

    @property
    def contract_hash(self) -> str:
        return stable_contract_hash(self.semantic_dict())

    def as_dict(self) -> Dict[str, object]:
        return {
            **self.semantic_dict(),
            "contract_hash": self.contract_hash,
        }


def assert_feature_contract_compatible(
    actual_columns: Sequence[object],
    contract: FeatureContract | Mapping[str, object],
) -> None:
    """Fail closed if inference features differ in membership or order."""

    expected_contract = (
        contract
        if isinstance(contract, FeatureContract)
        else FeatureContract.from_mapping(contract)
    )
    actual = tuple(str(column) for column in actual_columns)
    expected = expected_contract.expanded_feature_cols
    if actual == expected:
        return
    missing = [column for column in expected if column not in actual]
    extra = [column for column in actual if column not in expected]
    if not missing and not extra:
        detail = "feature order differs"
    else:
        detail = f"missing={missing}, extra={extra}"
    raise ValueError(
        "Inference feature contract mismatch: "
        f"{detail}; expected_hash={expected_contract.contract_hash}"
    )


def capture_dependency_versions(
    model_family: str,
    *,
    distribution_version=metadata.version,
) -> Dict[str, Optional[str]]:
    """Capture installed versions without importing optional ML runtimes."""

    if model_family not in MODEL_FAMILIES:
        raise ValueError(f"Unsupported model family: {model_family}")
    names = ["numpy", "pandas", "scikit-learn"]
    if model_family == MODEL_FAMILY_CNN_1D:
        names.append("tensorflow")
    versions: Dict[str, Optional[str]] = {}
    for name in names:
        try:
            versions[name] = str(distribution_version(name))
        except metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def build_reproducibility_record(
    seed: int = DEFAULT_RANDOM_SEED,
    *,
    model_family: str = MODEL_FAMILY_GRADIENT_BOOSTING,
) -> Dict[str, object]:
    """Describe deterministic controls, including limits that need disclosure."""

    if model_family not in MODEL_FAMILIES:
        raise ValueError(f"Unsupported model family: {model_family}")
    hash_seed = os.environ.get("PYTHONHASHSEED")
    return {
        "random_seed": int(seed),
        "target_seed_policy": {
            "hr": int(seed),
            "rr": int(seed) + 1,
        },
        "python_hash_seed": hash_seed,
        "python_hash_seed_fixed_at_process_start": hash_seed is not None,
        "numpy_seeded": True,
        "estimator_random_state_seeded": True,
        "tensorflow_seeded": model_family == MODEL_FAMILY_CNN_1D,
        "tensorflow_deterministic_ops_requested": (
            model_family == MODEL_FAMILY_CNN_1D
        ),
        "determinism_limit": (
            "Bit-identical results still depend on matching dependency versions, "
            "hardware, runtime, and deterministic kernel availability."
        ),
    }


def deployment_feasibility(model_family: str) -> Dict[str, object]:
    """Return conservative deployment claims for the selected family."""

    if model_family == MODEL_FAMILY_GRADIENT_BOOSTING:
        return {
            "host_inference": "supported",
            "host_runtime": "scikit-learn CPU",
            "embedded_inference": "not_qualified",
            "embedded_next_gate": (
                "Convert or distill, then measure flash, RAM, latency, numeric "
                "parity, and physical accuracy on the target ESP32-C6."
            ),
        }
    if model_family == MODEL_FAMILY_CNN_1D:
        return {
            "host_inference": "experimental",
            "host_runtime": "TensorFlow CPU or accelerator",
            "embedded_inference": "not_qualified",
            "embedded_next_gate": (
                "Produce an integer-quantized TFLite candidate, then measure "
                "operator support, flash, tensor arena, latency, parity, and "
                "physical accuracy on the target ESP32-C6."
            ),
        }
    raise ValueError(f"Unsupported model family: {model_family}")


def build_model_bundle_metadata(
    *,
    trainer_version: str,
    model_family: str,
    trained_targets: Sequence[str],
    feature_contract: FeatureContract,
    training_config: Mapping[str, object],
    seed: int = DEFAULT_RANDOM_SEED,
    artifact_hashes: Optional[Mapping[str, str]] = None,
    source_commit: Optional[str] = None,
    created_at: Optional[str] = None,
) -> Dict[str, object]:
    """Build auditable metadata for a dual-family model bundle."""

    if model_family not in MODEL_FAMILIES:
        raise ValueError(f"Unsupported model family: {model_family}")
    targets = tuple(dict.fromkeys(str(target) for target in trained_targets))
    if not targets or any(target not in {"hr", "rr"} for target in targets):
        raise ValueError("trained_targets must contain hr and/or rr")
    if model_family == MODEL_FAMILY_CNN_1D:
        sequence = training_config.get("sequence")
        if not isinstance(sequence, Mapping) or int(sequence.get("window_size", 0)) < 2:
            raise ValueError("cnn_1d bundles require sequence.window_size >= 2")
    feature_contract.validate()
    timestamp = created_at or datetime.now(timezone.utc).isoformat().replace(
        "+00:00", "Z"
    )
    metadata_record: Dict[str, object] = {
        "schema_version": MODEL_BUNDLE_SCHEMA_VERSION,
        "created_at": timestamp,
        "trainer_version": str(trainer_version),
        "model_family": model_family,
        "trained_targets": list(targets),
        "feature_contract": feature_contract.as_dict(),
        "training_config": dict(training_config),
        "reproducibility": build_reproducibility_record(
            seed, model_family=model_family
        ),
        "dependencies": capture_dependency_versions(model_family),
        "runtime": {
            "python": platform.python_version(),
            "python_implementation": platform.python_implementation(),
            "platform": platform.platform(),
            "machine": platform.machine(),
            "byteorder": sys.byteorder,
        },
        "deployability": deployment_feasibility(model_family),
        "artifacts": dict(artifact_hashes or {}),
        "source_commit": source_commit,
    }
    metadata_record["bundle_contract_hash"] = stable_contract_hash(
        {
            key: value
            for key, value in metadata_record.items()
            if key not in {"created_at", "artifacts", "bundle_contract_hash"}
        }
    )
    return metadata_record


def assert_bundle_inference_compatible(
    bundle: Mapping[str, object],
    *,
    actual_columns: Sequence[object],
    requested_model_family: Optional[str] = None,
    sequence_window_size: Optional[int] = None,
) -> None:
    """Validate the bundle contract before invoking either inference runtime."""

    if bundle.get("schema_version") != MODEL_BUNDLE_SCHEMA_VERSION:
        raise ValueError(
            f"Unsupported model bundle schema: {bundle.get('schema_version')!r}"
        )
    supplied_bundle_hash = bundle.get("bundle_contract_hash")
    recomputed_bundle_hash = stable_contract_hash(
        {
            key: value
            for key, value in bundle.items()
            if key not in {"created_at", "artifacts", "bundle_contract_hash"}
        }
    )
    if supplied_bundle_hash != recomputed_bundle_hash:
        raise ValueError("Model bundle contract hash is invalid")
    family = str(bundle.get("model_family", ""))
    if family not in MODEL_FAMILIES:
        raise ValueError(f"Unsupported model family in bundle: {family!r}")
    if requested_model_family is not None and requested_model_family != family:
        raise ValueError(
            "Requested model family does not match bundle: "
            f"requested={requested_model_family}, bundle={family}"
        )
    feature_value = bundle.get("feature_contract")
    if not isinstance(feature_value, Mapping):
        raise ValueError("Model bundle has no feature_contract")
    supplied_hash = feature_value.get("contract_hash")
    contract = FeatureContract.from_mapping(feature_value)
    if supplied_hash != contract.contract_hash:
        raise ValueError("Model bundle feature contract hash is invalid")
    assert_feature_contract_compatible(actual_columns, contract)
    if family == MODEL_FAMILY_CNN_1D:
        config = bundle.get("training_config")
        sequence = config.get("sequence") if isinstance(config, Mapping) else None
        expected_window = (
            int(sequence.get("window_size", 0))
            if isinstance(sequence, Mapping)
            else 0
        )
        if expected_window < 2:
            raise ValueError("CNN bundle has no valid sequence window contract")
        if (
            sequence_window_size is not None
            and int(sequence_window_size) != expected_window
        ):
            raise ValueError(
                "CNN inference window mismatch: "
                f"actual={sequence_window_size}, expected={expected_window}"
            )
