"""Model-family helpers, alignment, LOSO evaluation, and offline ML commands.

Gradient boosting remains the lightweight default.  The optional 1-D CNN path
uses causal windows that never cross session boundaries and imports TensorFlow
only when the CNN family is selected.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
import pickle
import random
import re
import subprocess
import sys
import time
from html import escape as html_escape
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Set, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.metrics import mean_squared_error

from rvt_trainer.api.common import (
    atomic_write_json as save_json,
    nan_safe,
    read_json_if_exists as _read_json_if_exists,
)
from rvt_trainer.audit.truthfulness import (
    _build_agc_anomaly_flags,
    _build_gate_audit,
    _compare_to_golden,
    _compute_internal_consistency_score,
    _compute_session_quality_score,
    _coverage_loss_ledger,
    _derive_raw_rescue_reject_count,
    _experimental_hr_gate_metrics,
    _ml_gate_block_from_baseline,
    _oracle_candidate_audit,
    _phase_subset_summary,
    _radar_csv_on_disk_contract_length,
    _truthfulness_from_radar,
)
from rvt_trainer.audit.verdict import (
    _build_adaptive_correction_shadow,
    _build_ml_readiness_verdict,
    _build_pqi_v15_shadow,
    _coverage_penalized_hr_gate,
    baseline_summary,
    compute_hr_bias_estimate,
    compute_raw_hr_bias_estimate,
    compute_raw_hr_sumfreq_error_estimate,
    finite_pair_mask,
    get_valid_pair_arrays,
    maybe_plot_bland_altman,
    maybe_plot_phase_signals,
    maybe_plot_timeseries,
    model_summary,
    print_metric_block,
    safe_corr,
    save_feature_importance,
    validity_masks,
)
from rvt_trainer.constants import (
    ALIGNMENT_REPORT_SCHEMA_VERSION,
    CONFIRMATORY_ANALYSIS_PLAN_ID,
    CONFIRMATORY_RUN_MANIFEST_SCHEMA_VERSION,
    FEATURE_ENGINEERING_VERSION,
    FEATURE_SCHEMA_VERSION,
    HR_GATE_REASON_NAMES,
    HR_PUBLISH_REASON_NAMES,
    HR_PUBLISH_SOURCE_NAMES,
    HR_RANGE,
    MODEL_ARTIFACT_NAMES,
    MODEL_FAMILIES,
    MODEL_FAMILY_CNN_1D,
    MODEL_FAMILY_GRADIENT_BOOSTING,
    MODEL_MANIFEST_FILENAME,
    MODEL_MANIFEST_SCHEMA_VERSION,
    MODEL_SIGNING_KEY_ENV,
    REQUIRE_MODEL_MANIFEST_ENV,
    REQUIRE_MODEL_SIGNATURE_ENV,
    RR_GATE_REASON_NAMES,
    RR_PUBLISH_REASON_NAMES,
    RR_RANGE,
    SECONDARY_GATE_MIN_DURATION_S,
    SYNC_ANCHORS_SCHEMA_VERSION,
    TRAINING_PROGRESS_SCHEMA_VERSION,
    VERSION,
    _REPO_ROOT,
    _bold,
    _env_flag_enabled,
    _file_sha256,
    _first_existing,
    _green,
    _histogram_from_series,
    _iso_now,
    _red,
    _report_stamp,
    _safe_float_val,
    _safe_int,
    _scoring_weights_hash,
    _session_duration_s,
    _to_num,
    _yellow,
    bytes_to_c_array,
    safe_series,
    tensorflow_is_available,
    warn,
    write_text_report,
)
from rvt_trainer.evaluation_split import (
    attach_study_metadata as _attach_study_metadata,
    has_any_study_metadata as _has_any_study_metadata,
    has_complete_study_metadata as _has_complete_study_metadata,
    participant_split as _participant_split,
)
from rvt_trainer.features import (
    BASE_FEATURES_CORE,
    BASE_FEATURES_FULL,
    FEATURE_CATEGORY,
    LABEL_FEATURES,
    POLICY_FEATURE_EXACT,
    POLICY_FEATURE_PREFIXES,
    TEMPORAL_BASE_COLS_CORE,
    TEMPORAL_BASE_COLS_FULL,
    TEMPORAL_SUFFIXES_CORE,
    TEMPORAL_SUFFIXES_FULL,
    TRAINING_EXCLUDE_COLS,
    assert_ml_feature_schema,
    engineer_temporal_features,
    feature_category,
    feature_schema_hash,
    label_feature_columns,
    pick_feature_columns,
    policy_feature_columns,
    prepare_feature_matrix,
    transform_feature_matrix,
)
from rvt_trainer.model_artifacts import (
    MODEL_BUNDLE_METADATA_FILENAME,
    FeatureContract,
    assert_bundle_inference_compatible,
    build_model_bundle_metadata,
)
from rvt_trainer.session.manifest import (
    _session_root_for_outputs,
    _warn_on_manifest_mismatch,
    _write_session_manifest,
)
from rvt_trainer.session.study_contract import (
    STUDY_SESSION_SCHEMA_VERSION,
    source_commit as _source_commit,
)
from rvt_trainer.session.study_evidence import STUDY_PROTOCOL_ID
from rvt_trainer.statistics import DEFAULT_ANALYSIS_PLAN, analysis_plan_sha256
from rvt_trainer.transport.ble import _compute_ble_ref_quality
from rvt_trainer.transport.serial import (
    _check_arduino_cli,
    _log_serial_for_duration,
    _prepare_arduino_sketch_dir,
)


def build_causal_windows(
    features: np.ndarray,
    session_ids: Iterable[object],
    window_size: int,
    endpoint_mask: Optional[Sequence[bool]] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return left-padded causal windows and their source-row endpoints.

    Windows are built over contiguous runs of the same session ID. The first
    observation in a run is repeated for cold-start padding, which preserves a
    prediction for every requested endpoint without leaking future samples.
    """

    values = np.asarray(features, dtype=np.float32)
    if values.ndim != 2:
        raise ValueError(f"features must be 2-D [rows, channels], got shape={values.shape}")
    if int(window_size) < 2:
        raise ValueError("window_size must be at least 2 for a 1-D CNN")
    sessions = np.asarray(list(session_ids), dtype=object)
    if len(sessions) != len(values):
        raise ValueError(
            f"session_ids length ({len(sessions)}) must match feature rows ({len(values)})"
        )
    if endpoint_mask is None:
        selected = np.ones(len(values), dtype=bool)
    else:
        selected = np.asarray(endpoint_mask, dtype=bool)
        if len(selected) != len(values):
            raise ValueError(
                f"endpoint_mask length ({len(selected)}) must match feature rows ({len(values)})"
            )

    endpoints = np.flatnonzero(selected).astype(np.int64)
    windows = np.empty((len(endpoints), int(window_size), values.shape[1]), dtype=np.float32)
    if not len(endpoints):
        return windows, endpoints

    run_start = np.zeros(len(values), dtype=np.int64)
    start = 0
    for row in range(len(values)):
        if row and sessions[row] != sessions[row - 1]:
            start = row
        run_start[row] = start

    width = int(window_size)
    for out_idx, endpoint in enumerate(endpoints):
        first = int(run_start[endpoint])
        source_start = max(first, int(endpoint) - width + 1)
        observed = values[source_start : int(endpoint) + 1]
        pad_count = width - len(observed)
        if pad_count:
            windows[out_idx, :pad_count] = values[first]
        windows[out_idx, pad_count:] = observed
    return windows, endpoints


@dataclass(frozen=True)
class Cnn1DConfig:
    window_size: int = 32
    filters: int = 32
    kernel_size: int = 5
    dropout: float = 0.20
    learning_rate: float = 1e-3
    epochs: int = 100
    batch_size: int = 64
    patience: int = 12

    def validate(self) -> None:
        if self.window_size < 2:
            raise ValueError("cnn_window_size must be at least 2")
        if self.filters < 4:
            raise ValueError("cnn_filters must be at least 4")
        if self.kernel_size < 2 or self.kernel_size > self.window_size:
            raise ValueError("cnn_kernel_size must be between 2 and cnn_window_size")
        if not 0.0 <= self.dropout < 1.0:
            raise ValueError("cnn_dropout must be in [0, 1)")
        if self.learning_rate <= 0:
            raise ValueError("cnn_learning_rate must be positive")
        if self.epochs < 1 or self.batch_size < 1 or self.patience < 1:
            raise ValueError("cnn_epochs, cnn_batch_size, and cnn_patience must be positive")

    def as_dict(self) -> Dict[str, Any]:
        return {
            "window_size": int(self.window_size),
            "filters": int(self.filters),
            "kernel_size": int(self.kernel_size),
            "dropout": float(self.dropout),
            "learning_rate": float(self.learning_rate),
            "epochs": int(self.epochs),
            "batch_size": int(self.batch_size),
            "patience": int(self.patience),
        }


class Keras1DCNNRegressor:
    """Pickle-compatible, lazily reconstructed Keras 1-D CNN regressor."""

    model_family = MODEL_FAMILY_CNN_1D

    def __init__(self, config: Cnn1DConfig, random_state: int = 42):
        config.validate()
        self.config = config
        self.random_state = int(random_state)
        self.feature_mean_: Optional[np.ndarray] = None
        self.feature_scale_: Optional[np.ndarray] = None
        self.model_json_: Optional[str] = None
        self.model_weights_: Optional[list[np.ndarray]] = None
        self.history_: Dict[str, list[float]] = {}
        self._model: Any = None

    @staticmethod
    def _tensorflow():
        try:
            import tensorflow as tf
        except Exception as exc:
            raise RuntimeError(
                "The cnn_1d model family requires TensorFlow. "
                "Install it with: pip install tensorflow"
            ) from exc
        return tf

    def _build_model(self, input_shape: Tuple[int, int]):
        tf = self._tensorflow()
        tf.keras.backend.clear_session()
        random.seed(self.random_state)
        np.random.seed(self.random_state)
        tf.keras.utils.set_random_seed(self.random_state)
        try:
            tf.config.experimental.enable_op_determinism()
        except (AttributeError, RuntimeError):
            # Older TensorFlow releases or already-initialized accelerators may
            # not expose/allow this switch. The bundle records that deterministic
            # ops were requested, not that every platform guarantees bit identity.
            pass
        inputs = tf.keras.Input(shape=input_shape, name="radar_window")
        x = tf.keras.layers.Conv1D(
            self.config.filters,
            self.config.kernel_size,
            padding="causal",
            activation="relu",
            name="temporal_conv_1",
        )(inputs)
        x = tf.keras.layers.BatchNormalization(name="temporal_norm_1")(x)
        x = tf.keras.layers.Conv1D(
            self.config.filters * 2,
            3,
            padding="causal",
            activation="relu",
            name="temporal_conv_2",
        )(x)
        x = tf.keras.layers.GlobalAveragePooling1D(name="temporal_pool")(x)
        x = tf.keras.layers.Dropout(self.config.dropout, name="dropout")(x)
        x = tf.keras.layers.Dense(self.config.filters, activation="relu", name="dense")(x)
        output = tf.keras.layers.Dense(1, name="vital")(x)
        model = tf.keras.Model(inputs, output, name="radar_vital_1d_cnn")
        model.compile(
            optimizer=tf.keras.optimizers.Adam(self.config.learning_rate),
            loss=tf.keras.losses.Huber(),
            metrics=["mae"],
        )
        return model

    def _normalize_fit(self, windows: np.ndarray) -> np.ndarray:
        self.feature_mean_ = windows.mean(axis=(0, 1), dtype=np.float64).astype(np.float32)
        scale = windows.std(axis=(0, 1), dtype=np.float64).astype(np.float32)
        self.feature_scale_ = np.where(scale > 1e-6, scale, 1.0).astype(np.float32)
        return self._normalize(windows)

    def _normalize(self, windows: np.ndarray) -> np.ndarray:
        if self.feature_mean_ is None or self.feature_scale_ is None:
            raise ValueError("CNN regressor has not been fitted")
        return ((windows - self.feature_mean_) / self.feature_scale_).astype(np.float32)

    def fit(
        self,
        train_windows: np.ndarray,
        y_train: np.ndarray,
        validation_data: Optional[Tuple[np.ndarray, np.ndarray]] = None,
        sample_weight: Optional[np.ndarray] = None,
    ) -> "Keras1DCNNRegressor":
        tf = self._tensorflow()
        train_windows = np.asarray(train_windows, dtype=np.float32)
        y_train = np.asarray(y_train, dtype=np.float32).reshape(-1)
        if train_windows.ndim != 3 or len(train_windows) != len(y_train):
            raise ValueError("CNN training data must be [samples, timesteps, channels]")
        normalized_train = self._normalize_fit(train_windows)
        normalized_validation = None
        if validation_data is not None:
            val_windows, y_val = validation_data
            normalized_validation = (
                self._normalize(np.asarray(val_windows, dtype=np.float32)),
                np.asarray(y_val, dtype=np.float32).reshape(-1),
            )
        self._model = self._build_model(
            (int(train_windows.shape[1]), int(train_windows.shape[2]))
        )
        callbacks = []
        if normalized_validation is not None and len(normalized_validation[0]):
            callbacks = [
                tf.keras.callbacks.EarlyStopping(
                    monitor="val_loss",
                    patience=self.config.patience,
                    restore_best_weights=True,
                ),
                tf.keras.callbacks.ReduceLROnPlateau(
                    monitor="val_loss",
                    patience=max(2, self.config.patience // 2),
                    factor=0.5,
                    min_lr=1e-6,
                ),
            ]
        history = self._model.fit(
            normalized_train,
            y_train,
            validation_data=normalized_validation,
            sample_weight=sample_weight,
            epochs=self.config.epochs,
            batch_size=min(self.config.batch_size, max(1, len(normalized_train))),
            verbose=0,
            callbacks=callbacks,
        )
        self.history_ = {
            str(name): [float(value) for value in values]
            for name, values in history.history.items()
        }
        self.model_json_ = self._model.to_json()
        self.model_weights_ = [np.asarray(weight) for weight in self._model.get_weights()]
        return self

    def _ensure_model(self):
        if self._model is not None:
            return self._model
        if not self.model_json_ or self.model_weights_ is None:
            raise ValueError("CNN model state is incomplete")
        tf = self._tensorflow()
        model = tf.keras.models.model_from_json(self.model_json_)
        model.set_weights(self.model_weights_)
        self._model = model
        return model

    def predict_windows(self, windows: np.ndarray) -> np.ndarray:
        normalized = self._normalize(np.asarray(windows, dtype=np.float32))
        prediction = self._ensure_model().predict(
            normalized,
            batch_size=self.config.batch_size,
            verbose=0,
        )
        return np.asarray(prediction, dtype=np.float32).reshape(-1)

    def predict_aligned(self, frame, features) -> np.ndarray:
        values = (
            features.to_numpy(dtype=np.float32)
            if hasattr(features, "to_numpy")
            else np.asarray(features, dtype=np.float32)
        )
        sessions = (
            frame["session_id"].to_numpy(dtype=object)
            if "session_id" in frame
            else np.repeat("session", len(values))
        )
        windows, endpoints = build_causal_windows(
            values,
            sessions,
            self.config.window_size,
        )
        result = np.full(len(values), np.nan, dtype=np.float32)
        result[endpoints] = self.predict_windows(windows)
        return result

    def __getstate__(self):
        state = self.__dict__.copy()
        if self._model is not None:
            state["model_json_"] = self._model.to_json()
            state["model_weights_"] = [
                np.asarray(weight) for weight in self._model.get_weights()
            ]
        state["_model"] = None
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        self._model = None



def _combine_radar_frames(paths: Sequence[str]) -> pd.DataFrame:
    frames = []
    for idx, path in enumerate(paths):
        df = load_radar(path)
        if "session_id" not in df.columns:
            df["session_id"] = f"session_{idx+1}"
        frames.append(df)
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True)


def _model_signing_key() -> Optional[bytes]:
    """Return the HMAC signing key bytes from the environment, or None if unset/blank."""
    raw = os.environ.get(MODEL_SIGNING_KEY_ENV)
    if not raw:
        return None
    return raw.encode("utf-8")


def _model_artifact_hashes(model_dir: str) -> "Dict[str, str]":
    """Map of artifact filename -> sha256 hex for every present model artifact in model_dir."""
    hashes: Dict[str, str] = {}
    for name in MODEL_ARTIFACT_NAMES:
        digest = _file_sha256(os.path.join(model_dir, name))
        if digest is not None:
            hashes[name] = digest
    return hashes


def _model_manifest_hmac(artifact_hashes: "Dict[str, str]", key: bytes) -> str:
    """HMAC-SHA256 over the artifact hashes, stable across dict ordering."""
    import hmac as _hmac

    payload = "\n".join(f"{name}={artifact_hashes[name]}" for name in sorted(artifact_hashes))
    return _hmac.new(key, payload.encode("utf-8"), hashlib.sha256).hexdigest()


def _build_model_manifest(model_dir: str) -> "Dict[str, object]":
    """Build the model manifest dict for the artifacts written into model_dir."""
    artifact_hashes = _model_artifact_hashes(model_dir)
    manifest: Dict[str, object] = {
        "schema_version": MODEL_MANIFEST_SCHEMA_VERSION,
        "trainer_version": VERSION,
        "created_at": _iso_now(),
        "artifacts": artifact_hashes,
    }
    key = _model_signing_key()
    if key is not None:
        manifest["hmac_sha256"] = _model_manifest_hmac(artifact_hashes, key)
    return manifest


def _write_model_manifest(model_dir: str) -> str:
    """Write model_manifest.json into model_dir and return its path."""
    manifest = _build_model_manifest(model_dir)
    manifest_path = os.path.join(model_dir, MODEL_MANIFEST_FILENAME)
    save_json(manifest, manifest_path)
    return manifest_path


def _read_model_bundle_metadata(model_dir: str) -> Optional[Dict[str, object]]:
    """Read and internally validate optional v2 bundle metadata.

    The caller must run ``_verify_model_dir`` first so this JSON is covered by
    the same SHA-256/HMAC boundary as the pickle artifacts. Legacy bundles have
    no metadata file and retain the existing compatibility behavior.
    """

    path = os.path.join(model_dir, MODEL_BUNDLE_METADATA_FILENAME)
    if not os.path.exists(path):
        return None
    bundle = _read_json_if_exists(path)
    if not isinstance(bundle, dict):
        raise ValueError(f"Model bundle metadata is not valid JSON: {path}")
    feature_contract = bundle.get("feature_contract")
    if not isinstance(feature_contract, dict):
        raise ValueError(f"Model bundle metadata has no feature contract: {path}")
    expected_columns = feature_contract.get("expanded_feature_cols")
    if not isinstance(expected_columns, list):
        raise ValueError(f"Model bundle feature order is invalid: {path}")
    family = bundle.get("model_family")
    sequence = bundle.get("training_config")
    sequence = sequence.get("sequence") if isinstance(sequence, dict) else None
    window_size = (
        sequence.get("window_size") if isinstance(sequence, dict) else None
    )
    assert_bundle_inference_compatible(
        bundle,
        actual_columns=expected_columns,
        requested_model_family=str(family),
        sequence_window_size=window_size,
    )
    return bundle


def _verify_model_dir(model_dir: str) -> None:
    """Verify the integrity of pickled model artifacts before they are unpickled.

    Behaviour:
      * Resolves model_dir to a real absolute path.
      * If model_manifest.json is present, first validates the *structure* of
        the declared artifact set: every listed name must be one of the allowed
        model artifacts, 'preprocessor.pkl' (always loaded at predict time) must
        be present, and at least one of model_hr.pkl / model_rr.pkl must be
        present. This runs before any file is trusted, so a manifest that omits
        preprocessor.pkl or lists unknown / path-like names is rejected rather
        than silently letting an unverified pickle be loaded.
      * It then recomputes each listed artifact's sha256 and raises on a missing
        artifact or hash mismatch (tamper detection). If RVT_MODEL_SIGNING_KEY is
        set, the manifest HMAC is also verified.
      * If no manifest is present, this is a legacy/unsigned model dir: emit a
        warning to stderr and proceed, UNLESS RVT_REQUIRE_MODEL_MANIFEST=1 is
        set, in which case it raises.

    Strict-signature mode (RVT_REQUIRE_MODEL_SIGNATURE=1):
        An unsigned manifest is self-attesting: an attacker who controls the
        model dir can rewrite both a .pkl and the manifest hash, and an unsigned
        sha256-only manifest still verifies. sha256 alone is therefore only
        accidental-corruption / local-drift detection. Strict mode upgrades this
        to tamper/authenticity protection BEFORE pickle.load by requiring an
        authenticated (HMAC-signed) manifest:
          * RVT_MODEL_SIGNING_KEY MUST be set (authenticity is unverifiable
            without a key) -> raise if unset.
          * The manifest MUST carry an 'hmac_sha256' field -> raise if absent
            (an unsigned manifest is not acceptable in strict mode).
          * The HMAC is verified with hmac.compare_digest and FAILS CLOSED on
            mismatch (wrong key / tampered manifest -> raise).
          * It also implies manifest-required: a missing manifest raises, exactly
            like RVT_REQUIRE_MODEL_MANIFEST.
        When the flag is NOT set, behaviour is unchanged: an unsigned manifest
        still passes (corruption/drift detection), and the HMAC is verified
        opportunistically only when a key is configured.

    Raises:
        ValueError: on missing manifest (when required/strict), missing artifact,
            hash mismatch, missing key/signature in strict mode, or HMAC mismatch.
    """
    real_dir = os.path.realpath(os.path.abspath(model_dir))
    if not os.path.isdir(real_dir):
        raise ValueError(f"model_dir does not exist or is not a directory: {model_dir}")

    strict_signature = _env_flag_enabled(REQUIRE_MODEL_SIGNATURE_ENV)

    manifest_path = os.path.join(real_dir, MODEL_MANIFEST_FILENAME)
    if not os.path.exists(manifest_path):
        # Strict-signature mode implies manifest-required: a missing manifest
        # can never be authenticated, so it must fail closed regardless of
        # RVT_REQUIRE_MODEL_MANIFEST.
        require = strict_signature or _env_flag_enabled(REQUIRE_MODEL_MANIFEST_ENV)
        msg = (
            f"No {MODEL_MANIFEST_FILENAME} found in model dir '{real_dir}'. "
            "Model integrity cannot be verified before unpickling."
        )
        if require:
            raise ValueError(
                msg + f" Refusing to load because {REQUIRE_MODEL_MANIFEST_ENV} is set. "
                "Re-train to generate a manifest, or unset the variable to allow legacy loads."
            )
        print(
            "[WARN] " + msg + " Proceeding with an UNVERIFIED legacy model dir; "
            f"set {REQUIRE_MODEL_MANIFEST_ENV}=1 to refuse unsigned models.",
            file=sys.stderr,
        )
        return

    manifest = _read_json_if_exists(manifest_path)
    if not isinstance(manifest, dict):
        raise ValueError(f"Model manifest is not valid JSON or not an object: {manifest_path}")

    expected = manifest.get("artifacts")
    if not isinstance(expected, dict) or not expected:
        raise ValueError(f"Model manifest has no 'artifacts' hashes: {manifest_path}")

    # Structural validation of the artifact set BEFORE any file is trusted.
    # cmd_predict ALWAYS unpickles preprocessor.pkl and at least one of the
    # model artifacts, so a manifest that omits them (or that lists unrelated /
    # path-like names) cannot be the complete, verified set of files. Reject
    # such manifests outright to close the model-integrity bypass.
    allowed_artifact_names = set(MODEL_ARTIFACT_NAMES)  # {"model_hr.pkl", "model_rr.pkl", "preprocessor.pkl"}
    unknown = sorted(name for name in expected if name not in allowed_artifact_names)
    if unknown:
        raise ValueError(
            f"Model manifest lists unknown artifact name(s) {unknown} in '{manifest_path}'. "
            f"Only {sorted(allowed_artifact_names)} are permitted; refusing to unpickle."
        )
    if "preprocessor.pkl" not in expected:
        raise ValueError(
            f"Model manifest does not list 'preprocessor.pkl' in '{manifest_path}', "
            "but it is always loaded at predict time. Refusing to unpickle an unverified preprocessor."
        )
    if "model_hr.pkl" not in expected and "model_rr.pkl" not in expected:
        raise ValueError(
            f"Model manifest lists no model artifact (model_hr.pkl / model_rr.pkl) in '{manifest_path}'. "
            "At least one model is required for prediction; refusing to unpickle."
        )

    for name, expected_hash in expected.items():
        artifact_path = os.path.join(real_dir, name)
        actual_hash = _file_sha256(artifact_path)
        if actual_hash is None:
            raise ValueError(
                f"Model artifact listed in manifest is missing: '{name}' in '{real_dir}'."
            )
        if actual_hash != expected_hash:
            raise ValueError(
                f"Model artifact integrity check FAILED for '{name}' in '{real_dir}': "
                f"expected sha256 {expected_hash}, got {actual_hash}. "
                "The model file may be corrupted or tampered with; refusing to unpickle."
            )

    key = _model_signing_key()

    if strict_signature:
        # Strict mode: authenticity is mandatory BEFORE pickle.load. Without a
        # key we cannot verify the manifest at all; without an HMAC field the
        # manifest is unsigned (self-attesting) and unacceptable. Fail closed.
        if key is None:
            raise ValueError(
                f"{REQUIRE_MODEL_SIGNATURE_ENV} is set but {MODEL_SIGNING_KEY_ENV} is unset for "
                f"'{real_dir}'. A signing key is required to verify model manifest authenticity; "
                "refusing to unpickle an unauthenticated model."
            )
        signed_hmac = manifest.get("hmac_sha256")
        if not isinstance(signed_hmac, str) or not signed_hmac:
            raise ValueError(
                f"{REQUIRE_MODEL_SIGNATURE_ENV} is set but model manifest has no HMAC signature "
                f"(unsigned manifest): {manifest_path}. Re-train with {MODEL_SIGNING_KEY_ENV} set to "
                "produce a signed manifest; refusing to unpickle in strict mode."
            )

    if key is not None:
        signed_hmac = manifest.get("hmac_sha256")
        if not isinstance(signed_hmac, str) or not signed_hmac:
            raise ValueError(
                f"{MODEL_SIGNING_KEY_ENV} is set but model manifest has no HMAC signature: {manifest_path}."
            )
        recomputed = _model_manifest_hmac({str(k): str(v) for k, v in expected.items()}, key)
        import hmac as _hmac

        if not _hmac.compare_digest(recomputed, signed_hmac):
            raise ValueError(
                f"Model manifest HMAC verification FAILED for '{real_dir}'. "
                "The manifest may be tampered with or signed with a different key; refusing to unpickle."
            )


def _plot_phase_signal(df: pd.DataFrame, col: str, title: str, out_path: str, enabled: bool = True):
    if not enabled or col not in df.columns:
        return
    y = safe_series(df, col)
    if not y.notna().any():
        return
    x = safe_series(df, "timestamp_s") if "timestamp_s" in df.columns else pd.Series(np.arange(len(df)), index=df.index, dtype=float)
    plt.figure(figsize=(10, 3.6))
    plt.plot(x, y, linewidth=1.0)
    plt.xlabel("Time (s)")
    plt.ylabel(col)
    plt.title(title)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def save_feature_importance_plot(model, feature_names, out_path, top_n: int = 20):
    if not hasattr(model, "feature_importances_"):
        return
    imp = pd.DataFrame({"feature": list(feature_names), "importance": model.feature_importances_})
    imp = imp.sort_values("importance", ascending=False).head(int(top_n))
    if imp.empty:
        return
    plt.figure(figsize=(10, max(4, 0.28 * len(imp))))
    plt.barh(imp["feature"].iloc[::-1], imp["importance"].iloc[::-1])
    plt.xlabel("Importance")
    plt.title(f"Top {len(imp)} Feature Importances")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def _write_simple_html_report(title: str, sections: List[Tuple[str, str]], image_paths: List[Tuple[str, str]], out_path: str):
    parts = [
        "<html><head><meta charset='utf-8'><title>%s</title>" % html_escape(title),
        "<style>body{font-family:Arial,sans-serif;margin:24px;line-height:1.45;}"
        "h1,h2{margin-bottom:0.35em;}pre{background:#f6f8fa;padding:12px;border-radius:8px;overflow:auto;}"
        "img{max-width:100%%;height:auto;border:1px solid #ddd;border-radius:8px;margin:12px 0;}"
        ".muted{color:#666}</style></head><body>",
        f"<h1>{html_escape(title)}</h1>",
        f"<p class='muted'>Generated: {html_escape(_report_stamp())}</p>",
    ]
    for heading, body in sections:
        parts.append(f"<h2>{html_escape(heading)}</h2><pre>{html_escape(body)}</pre>")
    for caption, img_path in image_paths:
        if os.path.exists(img_path):
            rel = os.path.basename(img_path)
            parts.append(f"<h2>{html_escape(caption)}</h2><img src='{html_escape(rel)}' alt='{html_escape(caption)}'>")
    parts.append("</body></html>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))


RADAR_SYNONYMS = {
    "smooth_hr": "reported_hr",
    "smooth_rr": "reported_rr",
    "humandetected": "human_detected",
    "radarispresent": "radar_is_present",
    "hrstate": "hr_state",
    "hr_trust_fresh": "trusted_hr_fresh",
    "heart_phase_stabilized": "heart_phase",
    "breath_phase_stabilized": "breath_phase",
    "module_fw_major": "fw_major",
    "module_fw_sub": "fw_sub",
    "module_fw_mod": "fw_mod",
}
REF_SYNONYMS = {
    "hr": "ref_hr",
    "rr": "ref_rr",
    "heart_rate": "ref_hr",
    "resp_rate": "ref_rr",
}


def lower_columns(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df.columns = [str(c).strip().lower() for c in df.columns]
    return df


def canonicalize_with_synonyms(df: pd.DataFrame, synonyms: Dict[str, str], source_name: str) -> pd.DataFrame:
    df = df.copy()
    for src, dst in synonyms.items():
        if src not in df.columns:
            continue
        if dst not in df.columns:
            df[dst] = df[src]
        else:
            dst_num = pd.to_numeric(df[dst], errors="coerce")
            src_num = pd.to_numeric(df[src], errors="coerce")
            overlap = dst_num.notna() & src_num.notna()
            if overlap.any():
                disagreements = np.abs(dst_num[overlap] - src_num[overlap]) > 1e-9
                if disagreements.any():
                    warn(f"{source_name}: columns '{dst}' and '{src}' both exist and disagree "
                         f"on {int(disagreements.sum())} row(s); preferring '{dst}' where present.")
            df[dst] = dst_num.where(dst_num.notna(), src_num)
        if src != dst:
            df.drop(columns=[src], inplace=True, errors="ignore")
    return df


def _check_monotonic_before_sort(df: pd.DataFrame, path: str):
    ts = pd.to_numeric(df["timestamp_s"], errors="coerce")
    if (ts.diff() < 0).any():
        warn(f"Non-monotonic timestamps in {path}; rows will be sorted by timestamp_s.")


def _radar_parse_stats(timestamp_s: pd.Series, path: str) -> Dict[str, int]:
    """Return CSV parse-drop metrics for a radar frame's ``timestamp_s`` column.

    ``timestamp_s`` must already be coerced with ``pd.to_numeric(..., errors='coerce')``
    so unparseable values are NaN. Reports how many rows will be discarded by the
    subsequent ``dropna(subset=['timestamp_s'])`` in :func:`load_radar`.

    No PHI is included — only counts and the file basename.
    """
    total = int(len(timestamp_s))
    dropped = int(timestamp_s.isna().sum())
    if dropped:
        warn(
            f"load_radar: dropped {dropped}/{total} rows with unparseable "
            f"timestamp_s from {os.path.basename(path)}"
        )
    return {"parse_total_rows": total, "parse_dropped_rows": dropped}


def load_radar(path: str) -> pd.DataFrame:
    df = lower_columns(pd.read_csv(path))
    df = canonicalize_with_synonyms(df, RADAR_SYNONYMS, source_name=os.path.basename(path))

    # v8.4.0: materialize widened telemetry for older logs and create stable aliases.
    for col in ("hr_trusted_phase_anchor", "hr_anchor_err_bpm"):
        if col not in df.columns:
            df[col] = np.nan
    if "hr_anchor_source" not in df.columns:
        df["hr_anchor_source"] = 0
    if "hr_raw_high_bias_suspect" not in df.columns:
        df["hr_raw_high_bias_suspect"] = 0
    if "heart_phase" not in df.columns and "heart_phase_stabilized" in df.columns:
        df["heart_phase"] = df["heart_phase_stabilized"]
    if "breath_phase" not in df.columns and "breath_phase_stabilized" in df.columns:
        df["breath_phase"] = df["breath_phase_stabilized"]
    if "fw_major" not in df.columns and "module_fw_major" in df.columns:
        df["fw_major"] = df["module_fw_major"]
    if "fw_sub" not in df.columns and "module_fw_sub" in df.columns:
        df["fw_sub"] = df["module_fw_sub"]
    if "fw_mod" not in df.columns and "module_fw_mod" in df.columns:
        df["fw_mod"] = df["module_fw_mod"]
    for col, default in (
        ("dsp_ran_this_frame", 0),
        ("hr_confidence_source", 0),
        ("sketch_major", np.nan),
        ("sketch_sub", np.nan),
        ("sketch_mod", np.nan),
        ("rr_seed_from_raw_used", 0),
        ("trusted_hr_fresh", 0),
        ("allow_logged_hr_vitals", 0),
        ("allow_logged_rr_vitals", 0),
        ("logged_hr_quality_gate", 0),
        ("logged_rr_quality_gate", 0),
        ("phase_valid_run_len", 0),
        ("phase_invalid_run_len", 0),
        ("hr_phase_backed_update_count", 0),
        ("rr_phase_backed_update_count", 0),
        ("near_field_reflector_suspect", 0),
        ("agc_floor_suspect", 0),
        ("phase_backed_publish_ready", 0),
        ("hr_anchor_drift_suspect", 0),
        ("phase_gap_fill_count", 0),
        ("clutter_rewarm_count", 0),
        ("rewarm_triggered", 0),
        ("rewarm_reason", 0),
        ("experimental_profile_enabled", 0),
        ("fs_effective", np.nan),
        ("fs_snap_used", np.nan),
        ("hr_fs_guard_min", np.nan),
        ("hr_autocorr_best_conf", np.nan),
        ("phase_zero_fill_pct", 0),
        ("fs_fallback_used", 0),
        ("module_fw_valid", 0),
        ("hr_trust_age_ms", np.nan),
        ("rr_trust_age_ms", np.nan),
        ("skipdsp_run_len", 0),
        ("agc_floor_run_len", 0),
        ("buffer_zero_injected", 0),
        ("hr_raw_minus_anchor_bpm", np.nan),
        ("hr_phase_minus_anchor_bpm", np.nan),
        ("hr_raw_minus_phase_bpm", np.nan),
        ("rr_candidate_present", 0),
        ("rr_candidate_source", 0),
        ("rr_source_reject_reason", 0),
        ("rr_source_latched_ok", 0),
        ("hr_publish_block_stage", 0),
        ("rr_publish_block_stage", 0),
        ("rr_phase_backed_publish_ready", 0),
        ("hr_raw_age_ms", np.nan),
        ("hr_raw_disagree_subreason", 0),
    ):
        if col not in df.columns:
            df[col] = default
    if "hr_trust_fresh" not in df.columns and "trusted_hr_fresh" in df.columns:
        df["hr_trust_fresh"] = df["trusted_hr_fresh"]
    if "trusted_hr_fresh" not in df.columns and "hr_trust_fresh" in df.columns:
        df["trusted_hr_fresh"] = df["hr_trust_fresh"]

    if "timestamp_s" not in df.columns:
        if "timestamp_ms" not in df.columns:
            raise ValueError(f"Radar file {path} must contain timestamp_ms or timestamp_s")
        df["timestamp_s"] = pd.to_numeric(df["timestamp_ms"], errors="coerce") / 1000.0
    else:
        df["timestamp_s"] = pd.to_numeric(df["timestamp_s"], errors="coerce")

    if "distance_m" in df.columns:
        dist_m = pd.to_numeric(df["distance_m"], errors="coerce") * 100.0
        if "reflector_distance_cm" in df.columns:
            canon = pd.to_numeric(df["reflector_distance_cm"], errors="coerce")
            overlap = canon.notna() & dist_m.notna()
            if overlap.any():
                disagreements = np.abs(canon[overlap] - dist_m[overlap]) > 1e-6
                if disagreements.any():
                    warn(f"{os.path.basename(path)}: reflector_distance_cm and distance_m disagree "
                         f"on {int(disagreements.sum())} row(s); preferring reflector_distance_cm.")
            df["reflector_distance_cm"] = canon.where(canon.notna(), dist_m)
        else:
            df["reflector_distance_cm"] = dist_m
        df.drop(columns=["distance_m"], inplace=True, errors="ignore")

    required = ["timestamp_s", "reported_hr", "reported_rr", "pqi_heart", "pqi_breath"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing radar columns in {path}: {missing}")

    numeric_candidates = [
        "timestamp_ms", "heart_phase", "breath_phase", "raw_hr", "raw_hr_uncorrected", "raw_hr_corrected", "raw_rr", "raw_rr_effective", "raw_rr_likely_harmonic",
        "hr_zc_bpm", "hr_zc_conf", "hr_spec_bpm", "hr_spec_mag", "hr_triple_agree",
        "rr_zc_bpm", "rr_zc_conf", "rr_spec_bpm", "rr_spec_conf", "rr_triple_agree",
        "reported_hr", "reported_rr", "candidate_hr", "candidate_rr",
        "candidate_hr_conf", "candidate_rr_conf",
        "pqi_heart", "pqi_breath", "hr_confidence",
        "hr_gate_pqi_used", "rr_gate_pqi_used",
        "reflector_distance_cm", "reported_distance_cm", "in_motion",
        "human_detected", "radar_is_present", "hr_state", "ghost_suspect",
        "dist_sd_cm", "disp_state", "dsp_task", "present_votes", "absent_votes",
        "phase_fresh", "trusted_vital_fresh", "logged_hr_valid", "logged_rr_valid",
        "hr_publish_reason", "rr_publish_reason", "skipdsp_misses",
        "hr_band_min", "hr_band_max", "radar_gain",
        "hr_publish_source", "rr_publish_source", "hr_publish_block_stage", "rr_publish_block_stage",
        "hr_arbiter_corrected", "hr_rejectphase_rejected", "hr_coherence_rejected",
        "hr_raw_source", "hr_raw_agree", "hr_agree_err_bpm",
        "hr_bypass_pqi_ok", "hr_bypass_conf_ok", "hr_bypass_gate_ok", "hr_bypass_active",
        "hr_grace_eligible", "hr_grace_active", "hr_trust_fresh",
        "rr_anchor_fresh", "rr_outlier_persist", "rr_raw_agree_ok",
        "rr_pre_acceptphase", "rr_post_acceptphase", "rr_post_blend", "rr_post_bias_correction", "rr_post_kalman", "rr_final_publish_candidate",
        "rr_anchor_value", "rr_anchor_age_ms", "rr_anchor_source", "rr_anchor_confidence",
        "rr_fundamental_recovery_count", "rr_fundamental_recovery_triggered", "rr_raw_seed_consistent_count",
        "rr_midsession_raw_reanchor_allowed", "rr_midsession_raw_reanchor_blocked", "rr_midsession_raw_reanchor_reason", "rr_raw_anchor_err_bpm",
        "hr_pre_rejectphase", "hr_post_rejectphase", "hr_post_blend", "hr_post_coherence",
        "hr_final_publish_candidate",
        "hr_arbiter_anchor_used", "hr_arbiter_anchor_value",
        "hr_rejectphase_anchor_used", "hr_rejectphase_anchor_value",
        "hr_raw_looks_like_half_rate", "hr_trusted_anchor_value",
        "hr_age_ms", "candidate_hr_age_ms", "candidate_rr_age_ms",
        "hr_updated_this_cycle", "hr_update_source", "latched_raw_hr", "trusted_rr_fresh", "radar_is_present_raw",
        "harmonic_mode", "session_phase",
        "point_cloud_ok", "target_info_ok", "num_targets", "max_dop_abs", "max_dop_speed_cms", "doppler_motion", "cluster_anomaly", "multi_target",
        "primary_x", "primary_y", "primary_dop", "primary_dop_speed_cms", "primary_cluster", "spatial_source", "spatial_age_ms", "position_radius_cm",
        "phase_warmup_complete", "clutter_warmup_count", "current_clutter_alpha", "use_fast_path", "phase_valid_this_frame", "hr_path_source", "fw_major", "fw_sub", "fw_mod",
        "trusted_hr_fresh", "allow_logged_hr_vitals", "allow_logged_rr_vitals", "logged_hr_quality_gate", "logged_rr_quality_gate",
        "phase_valid_run_len", "phase_invalid_run_len", "hr_phase_backed_update_count", "rr_phase_backed_update_count",
        "near_field_reflector_suspect", "agc_floor_suspect", "phase_backed_publish_ready", "hr_anchor_drift_suspect",
        "phase_gap_fill_count", "clutter_rewarm_count", "rewarm_triggered", "rewarm_reason", "experimental_profile_enabled",
        "hr_raw_disagree_subreason",
    ]
    for col in [c for c in numeric_candidates if c in df.columns]:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    # Surface (log + count) rows discarded for an unparseable timestamp_s rather
    # than dropping them silently.
    _radar_parse_stats(df["timestamp_s"], path)
    df = df.dropna(subset=["timestamp_s"]).copy()
    _check_monotonic_before_sort(df, path)
    if "in_motion" in df.columns:
        df = df[df["in_motion"].fillna(0) == 0].copy()
    return df.sort_values("timestamp_s").reset_index(drop=True)


def load_reference(path: str) -> pd.DataFrame:
    df = lower_columns(pd.read_csv(path))
    df = canonicalize_with_synonyms(df, REF_SYNONYMS, source_name=os.path.basename(path))

    if "timestamp_s" not in df.columns:
        if "timestamp_ms" not in df.columns:
            raise ValueError(f"Reference file {path} must contain timestamp_ms or timestamp_s")
        df["timestamp_s"] = pd.to_numeric(df["timestamp_ms"], errors="coerce") / 1000.0
    else:
        df["timestamp_s"] = pd.to_numeric(df["timestamp_s"], errors="coerce")

    for col in ["ref_hr", "ref_rr", "ref_spo2"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    if "ref_hr" not in df.columns:
        df["ref_hr"] = np.nan
    if "ref_rr" not in df.columns:
        df["ref_rr"] = np.nan

    if not (df["ref_hr"].notna().any() or df["ref_rr"].notna().any()):
        raise ValueError(f"Reference file {path} must contain at least one of ref_hr or ref_rr")

    df = df.dropna(subset=["timestamp_s"]).copy()
    _check_monotonic_before_sort(df, path)
    return df.sort_values("timestamp_s").reset_index(drop=True)


def normalize_offsets(n: int, offsets: Optional[Sequence[float]]) -> List[float]:
    if offsets is None or len(offsets) == 0:
        return [0.0] * n
    if len(offsets) == 1 and n > 1:
        return [float(offsets[0])] * n
    if len(offsets) != n:
        raise ValueError(f"Expected 1 or {n} offsets, got {len(offsets)}")
    return [float(x) for x in offsets]


def align_reference_time(
    radar_df: pd.DataFrame,
    ref_df: pd.DataFrame,
    ref_offset_s: float = 0.0,
    auto_align_start: bool = False,
) -> pd.DataFrame:
    ref_df = ref_df.copy()
    shift = float(ref_offset_s)
    if auto_align_start:
        shift += float(radar_df["timestamp_s"].min() - ref_df["timestamp_s"].min())
    ref_df["timestamp_s"] = ref_df["timestamp_s"] + shift
    return ref_df


def circular_mean(arr_like) -> float:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").dropna().to_numpy(dtype=float)
    if len(arr) == 0:
        return np.nan
    z = np.exp(1j * arr)
    return float(np.angle(np.mean(z)))


def circular_std(arr_like) -> float:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").dropna().to_numpy(dtype=float)
    if len(arr) == 0:
        return np.nan
    R = np.abs(np.mean(np.exp(1j * arr)))
    if R <= 1e-12:
        return np.nan
    return float(np.sqrt(max(0.0, -2.0 * np.log(R))))


def last_valid(arr_like) -> float:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").dropna().to_numpy(dtype=float)
    if len(arr) == 0:
        return np.nan
    return float(arr[-1])


def unwrapped_excursion(arr_like) -> float:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").dropna().to_numpy(dtype=float)
    if len(arr) == 0:
        return np.nan
    uw = np.unwrap(arr)
    return float(np.max(uw) - np.min(uw))


def phase_to_radians(arr_like, unit: str = "rad") -> np.ndarray:
    arr = pd.to_numeric(pd.Series(arr_like), errors="coerce").to_numpy(dtype=float)
    if len(arr) == 0:
        return arr
    valid = np.isfinite(arr)
    out = arr.astype(float, copy=True)
    if unit == "deg":
        out[valid] = np.deg2rad(out[valid])
    return out


def finite_phase_series_with_ts(ts_like, phase_like, unit: str = "rad") -> Tuple[np.ndarray, np.ndarray]:
    ts = pd.to_numeric(pd.Series(ts_like), errors="coerce").to_numpy(dtype=float)
    phase = phase_to_radians(phase_like, unit=unit)
    mask = np.isfinite(ts) & np.isfinite(phase)
    return ts[mask], phase[mask]


def spectral_entropy(power: np.ndarray) -> float:
    power = np.asarray(power, dtype=float)
    power = power[np.isfinite(power) & (power > 0)]
    if len(power) == 0:
        return np.nan
    p = power / np.sum(power)
    ent = -np.sum(p * np.log(p + 1e-12))
    return float(ent / np.log(len(p))) if len(p) > 1 else 0.0


def phase_band_features(
    ts_like, phase_like, band: Tuple[float, float],
    phase_unit: str = "rad",
    broader_band: Optional[Tuple[float, float]] = None,
) -> Dict[str, float]:
    ts, phase = finite_phase_series_with_ts(ts_like, phase_like, unit=phase_unit)
    empty = {
        "dom_freq_hz": np.nan, "peak_power": np.nan, "bandpower": np.nan,
        "spectral_entropy": np.nan, "peak_ratio": np.nan, "band_contrast": np.nan,
    }
    if len(ts) < 8:
        return empty
    order = np.argsort(ts)
    ts = ts[order]
    phase = phase[order]
    dt = np.diff(ts)
    dt = dt[np.isfinite(dt) & (dt > 1e-6)]
    if len(dt) == 0:
        return empty
    step = float(np.median(dt))
    if (not np.isfinite(step)) or step <= 0:
        return empty
    grid = np.arange(ts[0], ts[-1] + 0.5 * step, step)
    if len(grid) < 8:
        return empty
    uw = np.unwrap(phase)
    interp = np.interp(grid, ts, uw)
    t0 = grid - grid[0]
    if len(interp) >= 2:
        coeff = np.polyfit(t0, interp, deg=1)
        detrended = interp - np.polyval(coeff, t0)
    else:
        detrended = interp - np.nanmean(interp)
    detrended = detrended - np.nanmean(detrended)
    window = np.hanning(len(detrended)) if len(detrended) >= 4 else np.ones(len(detrended))
    spec = np.abs(np.fft.rfft(detrended * window)) ** 2
    freqs = np.fft.rfftfreq(len(detrended), d=step)
    band_mask = (freqs >= float(band[0])) & (freqs <= float(band[1]))
    if not np.any(band_mask):
        return empty
    band_freqs = freqs[band_mask]
    band_power = spec[band_mask]
    if len(band_power) == 0 or np.all(~np.isfinite(band_power)):
        return empty
    peak_idx = int(np.nanargmax(band_power))
    peak_power = float(band_power[peak_idx])
    dom_freq = float(band_freqs[peak_idx])
    total_bandpower = float(np.nansum(band_power))
    finite_power = band_power[np.isfinite(band_power)]
    if len(finite_power) >= 2:
        sorted_power = np.sort(finite_power)[::-1]
        second = float(sorted_power[1])
        peak_ratio = float(peak_power / (second + 1e-12))
    elif len(finite_power) == 1:
        peak_ratio = 1.0
    else:
        peak_ratio = np.nan
    entropy = spectral_entropy(finite_power)
    if broader_band is None:
        broader_band = (0.05, max(3.0, float(band[1]) * 1.5))
    broad_mask = (freqs >= float(broader_band[0])) & (freqs <= float(broader_band[1]))
    noise_mask = broad_mask & (~band_mask)
    noise_power = spec[noise_mask]
    noise_mean = float(np.nanmean(noise_power)) if np.any(np.isfinite(noise_power)) else np.nan
    band_contrast = float(peak_power / (noise_mean + 1e-12)) if np.isfinite(noise_mean) else np.nan
    return {
        "dom_freq_hz": dom_freq, "peak_power": peak_power, "bandpower": total_bandpower,
        "spectral_entropy": entropy, "peak_ratio": peak_ratio, "band_contrast": band_contrast,
    }


def aggregate_reference_1hz(ref_df: pd.DataFrame) -> pd.DataFrame:
    ref_df = ref_df.copy()
    ref_df["sec_bin"] = np.floor(ref_df["timestamp_s"]).astype(int)
    agg_map = {"ref_timestamp_s": ("timestamp_s", "mean")}
    if "ref_hr" in ref_df.columns:
        agg_map["ref_hr"] = ("ref_hr", "mean")
    if "ref_rr" in ref_df.columns:
        agg_map["ref_rr"] = ("ref_rr", "mean")
    if "ref_spo2" in ref_df.columns:
        agg_map["ref_spo2"] = ("ref_spo2", "mean")
    out = ref_df.groupby("sec_bin", as_index=False).agg(**agg_map)
    if "ref_hr" not in out.columns:
        out["ref_hr"] = np.nan
    if "ref_rr" not in out.columns:
        out["ref_rr"] = np.nan
    return out.sort_values("ref_timestamp_s").reset_index(drop=True)


def aggregate_radar_1hz(
    radar_df: pd.DataFrame,
    phase_unit: str = "rad",
    heart_fft_window_s: float = 5.0,
    breath_fft_window_s: float = 10.0,
) -> pd.DataFrame:
    df = radar_df.copy()
    df["sec_bin"] = np.floor(df["timestamp_s"]).astype(int)

    if "logged_hr_valid" in df.columns:
        df["row_hr_valid"] = (safe_series(df, "logged_hr_valid", default=0.0) > 0).astype(float)
    else:
        df["row_hr_valid"] = (safe_series(df, "reported_hr") > 10).astype(float)

    if "logged_rr_valid" in df.columns:
        df["row_rr_valid"] = (safe_series(df, "logged_rr_valid", default=0.0) > 0).astype(float)
    else:
        df["row_rr_valid"] = (safe_series(df, "reported_rr") > 0).astype(float)

    rows = []
    phase_cache = {}
    phase_ts_cache = {}
    for phase_col in [c for c in ["heart_phase", "breath_phase"] if c in df.columns]:
        pts, pvals = finite_phase_series_with_ts(df["timestamp_s"], df[phase_col], unit=phase_unit)
        phase_ts_cache[phase_col] = pts
        phase_cache[phase_col] = pvals

    for sec_bin, g in df.groupby("sec_bin", sort=True):
        row = {
            "sec_bin": int(sec_bin),
            "timestamp_s": float(pd.to_numeric(g["timestamp_s"], errors="coerce").dropna().iloc[-1]),
            "samples_per_sec": int(len(g)),
        }

        def add_numeric(prefix: str, col: str, stats: Sequence[str], *, nonzero_only: bool = False):
            if col not in g.columns:
                return
            s = pd.to_numeric(g[col], errors="coerce")
            if nonzero_only:
                s = s.where(s > 0, np.nan)
                row[f"{prefix}_nonzero_count"] = int(s.notna().sum())
            arr = s.dropna().to_numpy(dtype=float)
            if "mean"   in stats:
                row[f"{prefix}_mean"]   = float(np.mean(arr))   if len(arr) else np.nan
            if "std"    in stats:
                row[f"{prefix}_std"]    = float(np.std(arr, ddof=0)) if len(arr) else np.nan
            if "min"    in stats:
                row[f"{prefix}_min"]    = float(np.min(arr))    if len(arr) else np.nan
            if "max"    in stats:
                row[f"{prefix}_max"]    = float(np.max(arr))    if len(arr) else np.nan
            if "median" in stats:
                row[f"{prefix}_median"] = float(np.median(arr)) if len(arr) else np.nan
            if "last"   in stats:
                row[f"{prefix}_last"]   = float(arr[-1])        if len(arr) else np.nan
            if "sum"    in stats:
                row[f"{prefix}_sum"]    = float(np.sum(arr))    if len(arr) else np.nan

        g = g.copy()
        g["reported_hr__valid_only"] = pd.to_numeric(g["reported_hr"], errors="coerce").where(g["row_hr_valid"] > 0.0, np.nan)
        g["reported_rr__valid_only"] = pd.to_numeric(g["reported_rr"], errors="coerce").where(g["row_rr_valid"] > 0.0, np.nan)
        g["reported_hr__all"] = pd.to_numeric(g["reported_hr"], errors="coerce")
        g["reported_rr__all"] = pd.to_numeric(g["reported_rr"], errors="coerce")
        add_numeric("reported_hr", "reported_hr__valid_only", ["mean", "std", "min", "max", "median"])
        add_numeric("reported_rr", "reported_rr__valid_only", ["mean", "std", "min", "max", "median"])
        add_numeric("reported_hr_all", "reported_hr__all", ["mean", "std", "min", "max", "median"])
        add_numeric("reported_rr_all", "reported_rr__all", ["mean", "std", "min", "max", "median"])
        add_numeric("candidate_hr", "candidate_hr", ["mean", "std", "min", "max", "median"])
        add_numeric("candidate_rr", "candidate_rr", ["mean", "std", "min", "max", "median"])
        add_numeric("candidate_hr_conf", "candidate_hr_conf", ["mean", "std", "max"])
        add_numeric("candidate_rr_conf", "candidate_rr_conf", ["mean", "std", "max"])
        add_numeric("raw_hr",      "raw_hr",      ["mean", "std"], nonzero_only=True)
        add_numeric("raw_hr_uncorrected", "raw_hr_uncorrected", ["mean", "std"], nonzero_only=True)
        add_numeric("raw_hr_corrected", "raw_hr_corrected", ["mean", "std"], nonzero_only=True)
        add_numeric("raw_rr",      "raw_rr",      ["mean", "std"], nonzero_only=True)
        add_numeric("raw_rr_effective", "raw_rr_effective", ["mean", "std"], nonzero_only=True)
        add_numeric("hr_zc_bpm", "hr_zc_bpm", ["mean", "std", "last"])
        add_numeric("hr_zc_conf", "hr_zc_conf", ["mean", "std", "max"])
        add_numeric("hr_spec_bpm", "hr_spec_bpm", ["mean", "std", "last"])
        add_numeric("hr_spec_mag", "hr_spec_mag", ["mean", "std", "max"])
        add_numeric("hr_triple_agree", "hr_triple_agree", ["mean"])
        add_numeric("rr_zc_bpm", "rr_zc_bpm", ["mean", "std", "last"])
        add_numeric("rr_zc_conf", "rr_zc_conf", ["mean", "std", "max"])
        add_numeric("rr_spec_bpm", "rr_spec_bpm", ["mean", "std", "last"])
        add_numeric("rr_spec_conf", "rr_spec_conf", ["mean", "std", "max"])
        add_numeric("rr_triple_agree", "rr_triple_agree", ["mean"])
        add_numeric("pqi_heart",   "pqi_heart",   ["mean", "std", "max"])
        add_numeric("pqi_breath",  "pqi_breath",  ["mean", "std", "max"])
        add_numeric("hr_confidence","hr_confidence",["mean","std","max"])
        add_numeric("hr_gate_pqi_used", "hr_gate_pqi_used", ["mean", "std", "max"])
        add_numeric("rr_gate_pqi_used", "rr_gate_pqi_used", ["mean", "std", "max"])
        add_numeric("reported_distance_cm",  "reported_distance_cm",  ["mean", "std"])
        add_numeric("reflector_distance_cm", "reflector_distance_cm", ["mean", "std"])
        add_numeric("dist_sd_cm",  "dist_sd_cm",  ["mean", "std"])
        add_numeric("hr_state",    "hr_state",    ["last"])
        add_numeric("ghost_suspect","ghost_suspect",["mean"])
        add_numeric("human_detected","human_detected",["mean"])
        add_numeric("radar_is_present","radar_is_present",["mean"])
        add_numeric("present_votes","present_votes",["mean"])
        add_numeric("absent_votes", "absent_votes", ["mean"])
        add_numeric("row_hr_valid", "row_hr_valid", ["mean", "sum"])
        add_numeric("row_rr_valid", "row_rr_valid", ["mean", "sum"])
        for col in ["phase_fresh", "trusted_vital_fresh", "logged_hr_valid", "logged_rr_valid"]:
            add_numeric(col, col, ["mean"])

        add_numeric("hr_gate_reason", "hr_gate_reason", ["mean", "last"])
        add_numeric("rr_gate_reason", "rr_gate_reason", ["mean", "last"])
        add_numeric("hr_publish_reason", "hr_publish_reason", ["mean", "last"])
        add_numeric("rr_publish_reason", "rr_publish_reason", ["mean", "last"])
        add_numeric("hr_age_ms", "hr_age_ms", ["mean", "last"])
        add_numeric("candidate_hr_age_ms", "candidate_hr_age_ms", ["mean", "last", "max"])
        add_numeric("candidate_rr_age_ms", "candidate_rr_age_ms", ["mean", "last", "max"])
        add_numeric("hr_updated_this_cycle", "hr_updated_this_cycle", ["mean", "last"])

        add_numeric("point_cloud_ok", "point_cloud_ok", ["mean"])
        add_numeric("target_info_ok", "target_info_ok", ["mean"])
        add_numeric("num_targets", "num_targets", ["mean", "max", "last"])
        add_numeric("max_dop_abs", "max_dop_abs", ["mean", "max", "last"])
        add_numeric("max_dop_speed_cms", "max_dop_speed_cms", ["mean", "max", "last"])
        add_numeric("doppler_motion", "doppler_motion", ["mean"])
        add_numeric("cluster_anomaly", "cluster_anomaly", ["mean"])
        add_numeric("multi_target", "multi_target", ["mean"])
        add_numeric("primary_x", "primary_x", ["mean", "last"])
        add_numeric("primary_y", "primary_y", ["mean", "last"])
        add_numeric("primary_dop", "primary_dop", ["mean", "last"])
        add_numeric("primary_dop_speed_cms", "primary_dop_speed_cms", ["mean", "max", "last"])
        add_numeric("primary_cluster", "primary_cluster", ["last"])
        add_numeric("spatial_source", "spatial_source", ["last"])
        add_numeric("spatial_age_ms", "spatial_age_ms", ["mean", "last"])
        add_numeric("position_radius_cm", "position_radius_cm", ["mean", "std", "last"])
        add_numeric("harmonic_mode", "harmonic_mode", ["mean", "max", "last"])
        add_numeric("session_phase", "session_phase", ["mean", "last"])
        add_numeric("phase_valid_this_frame", "phase_valid_this_frame", ["mean"])
        add_numeric("dsp_ran_this_frame", "dsp_ran_this_frame", ["mean"])
        add_numeric("hr_confidence_source", "hr_confidence_source", ["last"])
        add_numeric("current_clutter_alpha", "current_clutter_alpha", ["mean", "last"])
        add_numeric("use_fast_path", "use_fast_path", ["mean"])
        add_numeric("hr_path_source", "hr_path_source", ["last"])
        add_numeric("module_fw_major", "module_fw_major", ["last"])
        add_numeric("module_fw_sub", "module_fw_sub", ["last"])
        add_numeric("module_fw_mod", "module_fw_mod", ["last"])
        add_numeric("sketch_major", "sketch_major", ["last"])
        add_numeric("sketch_sub", "sketch_sub", ["last"])
        add_numeric("sketch_mod", "sketch_mod", ["last"])
        add_numeric("hr_trusted_phase_anchor", "hr_trusted_phase_anchor", ["mean", "last"])
        add_numeric("hr_anchor_source", "hr_anchor_source", ["last"])
        add_numeric("hr_anchor_err_bpm", "hr_anchor_err_bpm", ["mean", "last", "max"])
        add_numeric("hr_raw_high_bias_suspect", "hr_raw_high_bias_suspect", ["mean", "last", "max"])
        add_numeric("trusted_hr_fresh", "trusted_hr_fresh", ["mean", "last"])
        add_numeric("allow_logged_hr_vitals", "allow_logged_hr_vitals", ["mean", "last"])
        add_numeric("allow_logged_rr_vitals", "allow_logged_rr_vitals", ["mean", "last"])
        add_numeric("logged_hr_quality_gate", "logged_hr_quality_gate", ["mean", "last"])
        add_numeric("logged_rr_quality_gate", "logged_rr_quality_gate", ["mean", "last"])
        add_numeric("phase_valid_run_len", "phase_valid_run_len", ["mean", "max", "last"])
        add_numeric("phase_invalid_run_len", "phase_invalid_run_len", ["mean", "max", "last"])
        add_numeric("hr_phase_backed_update_count", "hr_phase_backed_update_count", ["mean", "max", "last"])
        add_numeric("rr_phase_backed_update_count", "rr_phase_backed_update_count", ["mean", "max", "last"])
        add_numeric("near_field_reflector_suspect", "near_field_reflector_suspect", ["mean", "last", "max"])
        add_numeric("agc_floor_suspect", "agc_floor_suspect", ["mean", "last", "max"])
        add_numeric("phase_backed_publish_ready", "phase_backed_publish_ready", ["mean", "last", "max"])
        add_numeric("rr_phase_backed_publish_ready", "rr_phase_backed_publish_ready", ["mean", "last", "max"])
        add_numeric("hr_anchor_drift_suspect", "hr_anchor_drift_suspect", ["mean", "last", "max"])
        add_numeric("phase_gap_fill_count", "phase_gap_fill_count", ["mean", "max", "last"])
        add_numeric("clutter_rewarm_count", "clutter_rewarm_count", ["mean", "max", "last"])
        add_numeric("rewarm_triggered", "rewarm_triggered", ["mean", "max", "last"])
        add_numeric("rewarm_reason", "rewarm_reason", ["mean", "last"])
        add_numeric("experimental_profile_enabled", "experimental_profile_enabled", ["mean", "last", "max"])
        add_numeric("fs_effective", "fs_effective", ["mean", "last", "min", "max"])
        add_numeric("fs_snap_used", "fs_snap_used", ["mean", "last", "min", "max"])
        add_numeric("hr_fs_guard_min", "hr_fs_guard_min", ["mean", "last", "min", "max"])
        add_numeric("hr_autocorr_best_conf", "hr_autocorr_best_conf", ["mean", "last", "max"])
        add_numeric("phase_zero_fill_pct", "phase_zero_fill_pct", ["mean", "last", "max"])
        add_numeric("fs_fallback_used", "fs_fallback_used", ["mean", "last", "max"])
        add_numeric("module_fw_valid", "module_fw_valid", ["mean", "last", "max"])
        add_numeric("hr_trust_age_ms", "hr_trust_age_ms", ["mean", "last", "min", "max"])
        add_numeric("rr_trust_age_ms", "rr_trust_age_ms", ["mean", "last", "min", "max"])
        add_numeric("skipdsp_run_len", "skipdsp_run_len", ["mean", "last", "max"])
        add_numeric("agc_floor_run_len", "agc_floor_run_len", ["mean", "last", "max"])
        add_numeric("buffer_zero_injected", "buffer_zero_injected", ["mean", "last", "max"])
        add_numeric("hr_raw_minus_anchor_bpm", "hr_raw_minus_anchor_bpm", ["mean", "last", "max"])
        add_numeric("hr_phase_minus_anchor_bpm", "hr_phase_minus_anchor_bpm", ["mean", "last", "max"])
        add_numeric("hr_raw_minus_phase_bpm", "hr_raw_minus_phase_bpm", ["mean", "last", "max"])
        add_numeric("rr_candidate_present", "rr_candidate_present", ["mean", "last", "max"])
        add_numeric("rr_candidate_source", "rr_candidate_source", ["mean", "last"])
        add_numeric("rr_source_reject_reason", "rr_source_reject_reason", ["mean", "last"])
        add_numeric("rr_source_latched_ok", "rr_source_latched_ok", ["mean", "last", "max"])
        add_numeric("rr_source_current_ok", "rr_source_current_ok", ["mean", "last", "max"])
        add_numeric("logged_rr_valid_current", "logged_rr_valid_current", ["mean", "last", "max"])
        add_numeric("logged_rr_valid_latched", "logged_rr_valid_latched", ["mean", "last", "max"])
        add_numeric("hr_publish_source_class", "hr_publish_source_class", ["mean", "last"])
        add_numeric("rr_publish_source_class", "rr_publish_source_class", ["mean", "last"])
        add_numeric("hr_freeze_suspect", "hr_freeze_suspect", ["mean", "last", "max"])
        add_numeric("hr_value_frozen_confirmed", "hr_value_frozen_confirmed", ["mean", "last", "max"])
        add_numeric("hr_freeze_duration_ms", "hr_freeze_duration_ms", ["mean", "last", "max"])
        add_numeric("hr_no_fresh_update_duration_ms", "hr_no_fresh_update_duration_ms", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_anchor_drift_suspect", "hr_raw_disagree_anchor_drift_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_phase_stale_suspect", "hr_raw_disagree_phase_stale_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_high_bias_suspect", "hr_raw_disagree_high_bias_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_low_dynamic_range_suspect", "hr_raw_disagree_low_dynamic_range_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_sumfreq_suspect", "hr_raw_disagree_sumfreq_suspect", ["mean", "last", "max"])
        add_numeric("hr_raw_disagree_rr_harmonic_k", "hr_raw_disagree_rr_harmonic_k", ["mean", "last", "max"])
        add_numeric("hr_publish_block_stage", "hr_publish_block_stage", ["mean", "last"])
        add_numeric("rr_publish_block_stage", "rr_publish_block_stage", ["mean", "last"])
        add_numeric("hr_raw_age_ms", "hr_raw_age_ms", ["mean", "last", "min", "max"])
        add_numeric("hr_raw_disagree_subreason", "hr_raw_disagree_subreason", ["mean", "last"])

        for phase_col in ["heart_phase", "breath_phase"]:
            if phase_col in g.columns:
                _, phase_arr_rad = finite_phase_series_with_ts(
                    g["timestamp_s"], g[phase_col], unit=phase_unit)
                row[f"{phase_col}_mean"]      = circular_mean(phase_arr_rad)
                row[f"{phase_col}_std"]       = circular_std(phase_arr_rad)
                row[f"{phase_col}_last"]      = last_valid(phase_arr_rad)
                row[f"{phase_col}_excursion"] = unwrapped_excursion(phase_arr_rad)

        t_end = row["timestamp_s"]
        if "heart_phase" in phase_cache:
            pts   = phase_ts_cache["heart_phase"]
            pvals = phase_cache["heart_phase"]
            si = np.searchsorted(pts, t_end - float(heart_fft_window_s), side="left")
            ei = np.searchsorted(pts, t_end + 1e-9, side="right")
            feats = phase_band_features(
                pts[si:ei], pvals[si:ei], band=(0.8, 2.5),
                phase_unit="rad", broader_band=(0.1, 3.0))
            for k, v in feats.items():
                row[f"heart_phase_{k}"] = v

        if "breath_phase" in phase_cache:
            pts   = phase_ts_cache["breath_phase"]
            pvals = phase_cache["breath_phase"]
            si = np.searchsorted(pts, t_end - float(breath_fft_window_s), side="left")
            ei = np.searchsorted(pts, t_end + 1e-9, side="right")
            feats = phase_band_features(
                pts[si:ei], pvals[si:ei], band=(0.10, 0.60),
                phase_unit="rad", broader_band=(0.05, 1.0))
            for k, v in feats.items():
                row[f"breath_phase_{k}"] = v

        rows.append(row)

    out = pd.DataFrame(rows)
    out = out.rename(columns={
        "row_hr_valid_mean": "hr_valid_frac",
        "row_rr_valid_mean": "rr_valid_frac",
        "row_hr_valid_sum":  "hr_valid_count",
        "row_rr_valid_sum":  "rr_valid_count",
    })
    return out.sort_values("timestamp_s").reset_index(drop=True)


def align_one_session(
    radar_path: str, ref_path: str, session_id: str,
    tolerance_s: float, ref_offset_s: float,
    auto_align_start: bool, merge_direction: str,
    phase_unit: str, heart_fft_window_s: float, breath_fft_window_s: float,
) -> pd.DataFrame:
    radar = load_radar(radar_path)
    ref   = load_reference(ref_path)
    ref   = align_reference_time(radar, ref, ref_offset_s=ref_offset_s,
                                  auto_align_start=auto_align_start)
    radar_1hz = aggregate_radar_1hz(
        radar, phase_unit=phase_unit,
        heart_fft_window_s=heart_fft_window_s,
        breath_fft_window_s=breath_fft_window_s)
    ref_1hz   = aggregate_reference_1hz(ref)
    merged = pd.merge_asof(
        radar_1hz.sort_values("timestamp_s"),
        ref_1hz.sort_values("ref_timestamp_s"),
        left_on="timestamp_s", right_on="ref_timestamp_s",
        tolerance=tolerance_s, direction=merge_direction,
    )
    merged["ref_dt_s"] = merged["timestamp_s"] - merged["ref_timestamp_s"]
    ref_cols = [c for c in ["ref_hr", "ref_rr"] if c in merged.columns]
    if not ref_cols:
        raise ValueError("Merged dataset has no reference columns after alignment.")
    merged = merged.dropna(subset=ref_cols, how="all").copy()
    if "ref_hr" in merged.columns:
        merged = merged[(merged["ref_hr"].isna()) | ((merged["ref_hr"] >= 30) & (merged["ref_hr"] <= 220))]
    if "ref_rr" in merged.columns:
        merged = merged[(merged["ref_rr"].isna()) | ((merged["ref_rr"] >= 4) & (merged["ref_rr"] <= 60))]
    merged["session_id"] = session_id
    merged["radar_file"] = os.path.basename(radar_path)
    merged["ref_file"]   = os.path.basename(ref_path)
    return merged.reset_index(drop=True)


def load_and_align_multiple_base(
    radar_paths: Sequence[str], ref_paths: Sequence[str],
    tolerance_s: float, ref_offsets_s: Optional[Sequence[float]],
    auto_align_start: bool, merge_direction: str,
    phase_unit: str, heart_fft_window_s: float, breath_fft_window_s: float,
) -> pd.DataFrame:
    offsets = normalize_offsets(len(radar_paths), ref_offsets_s)
    frames = []
    for idx, (rp, refp, off) in enumerate(zip(radar_paths, ref_paths, offsets), start=1):
        sid = f"session_{idx:02d}"
        merged = align_one_session(
            rp, refp, sid, tolerance_s, off, auto_align_start,
            merge_direction, phase_unit, heart_fft_window_s, breath_fft_window_s)
        merged = _attach_study_metadata(merged, rp)
        frames.append(merged)
        print(
            f"[ALIGN] {sid}: {len(merged)} paired 1 Hz rows | "
            f"radar={os.path.basename(rp)} | ref={os.path.basename(refp)} | "
            f"offset={off:+.3f}s | direction={merge_direction}")
    return pd.concat(frames, ignore_index=True).reset_index(drop=True)


def load_and_aggregate_radar_only(
    radar_paths: Sequence[str],
    phase_unit: str, heart_fft_window_s: float, breath_fft_window_s: float,
) -> pd.DataFrame:
    frames = []
    for idx, rp in enumerate(radar_paths, start=1):
        sid = f"session_{idx:02d}"
        radar = load_radar(rp)
        agg   = aggregate_radar_1hz(
            radar, phase_unit=phase_unit,
            heart_fft_window_s=heart_fft_window_s,
            breath_fft_window_s=breath_fft_window_s)
        agg["session_id"]  = sid
        agg["radar_file"]  = os.path.basename(rp)
        frames.append(agg)
    return pd.concat(frames, ignore_index=True).reset_index(drop=True)


def chronological_split_single_session(
    df: pd.DataFrame, val_ratio: float = 0.2, purge_gap_s: float = 10.0,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    df = df.sort_values("timestamp_s").reset_index(drop=True)
    t_min = float(df["timestamp_s"].min())
    t_max = float(df["timestamp_s"].max())
    if not np.isfinite(t_min) or not np.isfinite(t_max) or t_max <= t_min:
        raise ValueError("Single-session timestamps are not valid for chronological splitting.")
    split_time = t_min + (1.0 - val_ratio) * (t_max - t_min)
    train_df = df[df["timestamp_s"] <= split_time].copy()
    val_df   = df[df["timestamp_s"] > split_time + purge_gap_s].copy()
    if len(train_df) == 0 or len(val_df) == 0:
        raise ValueError("Validation set became empty after purge gap. Reduce --purge-gap-s.")
    if len(val_df) < 10:
        raise ValueError("Validation set too small. Use longer capture or reduce --purge-gap-s.")
    print(f"[SPLIT] single-session | train={len(train_df)} | val={len(val_df)} | "
          f"split_time={split_time:.2f}s | purge_gap={purge_gap_s:.1f}s")
    return train_df.reset_index(drop=True), val_df.reset_index(drop=True)


def split_sessions(df: pd.DataFrame, val_ratio: float, purge_gap_s: float) -> Tuple[pd.DataFrame, pd.DataFrame]:
    session_ids = list(dict.fromkeys(df["session_id"].tolist()))
    if len(session_ids) >= 2:
        warn("Multi-session hold-out uses input file order. Pass files in chronological order.")
        train_ids = session_ids[:-1]
        val_id    = session_ids[-1]
        train_df  = df[df["session_id"].isin(train_ids)].copy()
        val_df    = df[df["session_id"] == val_id].copy()
        print(f"[SPLIT] hold-out last session | train={train_ids} | val={val_id} | "
              f"n_train={len(train_df)} | n_val={len(val_df)}")
        return train_df.reset_index(drop=True), val_df.reset_index(drop=True)
    return chronological_split_single_session(df, val_ratio=val_ratio, purge_gap_s=purge_gap_s)


def split_three_way(
    df: pd.DataFrame, test_ratio: float, early_stop_ratio: float, purge_gap_s: float,
) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, Dict[str, object]]:
    session_ids = list(dict.fromkeys(df["session_id"].tolist()))
    info: Dict[str, object] = {"mode": "three_way", "session_ids": session_ids}

    if len(session_ids) >= 3:
        warn("Three-way multi-session split uses input file order.")
        train_ids = session_ids[:-2]
        stop_id   = session_ids[-2]
        test_id   = session_ids[-1]
        train_df  = df[df["session_id"].isin(train_ids)].copy()
        stop_df   = df[df["session_id"] == stop_id].copy()
        test_df   = df[df["session_id"] == test_id].copy()
        info.update({"train_sessions": train_ids, "stop_session": stop_id, "test_session": test_id})
        return train_df.reset_index(drop=True), stop_df.reset_index(drop=True), test_df.reset_index(drop=True), info

    if len(session_ids) == 2:
        train_source = df[df["session_id"] == session_ids[0]].copy()
        test_df      = df[df["session_id"] == session_ids[1]].copy()
        train_df, stop_df = chronological_split_single_session(
            train_source, val_ratio=early_stop_ratio, purge_gap_s=purge_gap_s)
        info.update({"train_source_session": session_ids[0], "test_session": session_ids[1]})
        return train_df.reset_index(drop=True), stop_df.reset_index(drop=True), test_df.reset_index(drop=True), info

    train_stop_df, test_df = chronological_split_single_session(df, val_ratio=test_ratio, purge_gap_s=purge_gap_s)
    train_df, stop_df = chronological_split_single_session(train_stop_df, val_ratio=early_stop_ratio, purge_gap_s=purge_gap_s)
    info.update({"mode": "three_way_single_session"})
    return train_df.reset_index(drop=True), stop_df.reset_index(drop=True), test_df.reset_index(drop=True), info


def resolve_model_params(args) -> Dict[str, float]:
    if args.light_mode:
        return {"n_estimators": 150, "early_stop_max_estimators": 250,
                "max_depth": 2, "learning_rate": 0.05, "subsample": 0.8}
    return {
        "n_estimators":             args.n_estimators,
        "early_stop_max_estimators": args.early_stop_max_estimators,
        "max_depth":                args.max_depth,
        "learning_rate":            args.learning_rate,
        "subsample":                args.subsample,
    }


def resolve_cnn_config(args) -> Cnn1DConfig:
    return Cnn1DConfig(
        window_size=int(getattr(args, "cnn_window_size", 32)),
        filters=int(getattr(args, "cnn_filters", 32)),
        kernel_size=int(getattr(args, "cnn_kernel_size", 5)),
        dropout=float(getattr(args, "cnn_dropout", 0.20)),
        learning_rate=float(getattr(args, "cnn_learning_rate", 1e-3)),
        epochs=int(getattr(args, "cnn_epochs", 100)),
        batch_size=int(getattr(args, "cnn_batch_size", 64)),
        patience=int(getattr(args, "cnn_patience", 12)),
    )


def build_model(random_state, params, n_estimators):
    return GradientBoostingRegressor(
        n_estimators=int(n_estimators), max_depth=int(params["max_depth"]),
        learning_rate=float(params["learning_rate"]), subsample=float(params["subsample"]),
        random_state=random_state,
    )


def get_sample_weights(train_rows, target, mode):
    if mode == "none":
        return None
    q = safe_series(train_rows, "pqi_heart_mean" if target == "hr" else "pqi_breath_mean", default=0.5)
    return q.clip(lower=0.05, upper=1.0).to_numpy(dtype=np.float32)


def choose_best_n_estimators(model, X_val, y_val):
    scores = [float(np.sqrt(mean_squared_error(y_val, pred)))
              for pred in model.staged_predict(X_val)]
    best_idx = int(np.argmin(scores))
    return best_idx + 1, float(scores[best_idx])


def truncate_gbr_model(model, best_n):
    best_n = int(best_n)
    model.estimators_ = model.estimators_[:best_n]
    model.n_estimators = best_n
    if hasattr(model, "n_estimators_"):
        model.n_estimators_ = best_n
    for attr in ["train_score_", "oob_improvement_", "oob_scores_"]:
        if hasattr(model, attr):
            try:
                setattr(model, attr, getattr(model, attr)[:best_n])
            except Exception:
                pass
    return model


def fit_target_model(
    df_train, df_val, target, X_train_all, X_val_all, params,
    random_state=42, sample_weight_mode="none",
    use_early_stopping=True, early_stop_strategy="retrain",
):
    valid_col  = f"{target}_valid_for_eval"
    ref_col    = f"ref_{target}"
    train_mask = df_train[valid_col].fillna(False).to_numpy(dtype=bool)
    val_mask   = df_val[valid_col].fillna(False).to_numpy(dtype=bool)
    train_rows = df_train.loc[train_mask].copy()
    val_rows   = df_val.loc[val_mask].copy()
    if len(train_rows) < 20:
        raise ValueError(f"Not enough valid {target.upper()} rows to train ({len(train_rows)}).")
    X_train  = X_train_all.loc[train_mask].to_numpy(dtype=np.float32)
    y_train  = train_rows[ref_col].to_numpy(dtype=np.float32)
    X_val    = X_val_all.loc[val_mask].to_numpy(dtype=np.float32)
    y_val    = val_rows[ref_col].to_numpy(dtype=np.float32)
    weights  = get_sample_weights(train_rows, target, sample_weight_mode)
    meta = {"train_rows": int(len(train_rows)), "val_rows": int(len(val_rows)),
            "used_early_stopping": False,
            "best_n_estimators": int(params["n_estimators"]),
            "best_val_rmse": float("nan")}
    def fit(m):
        if weights is None:
            m.fit(X_train, y_train)
        else:
            m.fit(X_train, y_train, sample_weight=weights)
        return m
    if use_early_stopping and len(val_rows) >= 10:
        max_n      = int(params["early_stop_max_estimators"])
        model_full = fit(build_model(random_state, params, max_n))
        best_n, best_val_rmse = choose_best_n_estimators(model_full, X_val, y_val)
        if early_stop_strategy == "truncate":
            final_model = truncate_gbr_model(model_full, best_n)
        else:
            final_model = fit(build_model(random_state, params, best_n))
        meta.update({"used_early_stopping": True,
                     "best_n_estimators": int(best_n),
                     "best_val_rmse": float(best_val_rmse)})
        return final_model, meta
    return fit(build_model(random_state, params, int(params["n_estimators"]))), meta


def fit_cnn_target_model(
    df_train,
    df_val,
    target,
    X_train_all,
    X_val_all,
    config,
    random_state=42,
    sample_weight_mode="none",
    min_valid_windows=500,
    allow_small_dataset=False,
):
    valid_col = f"{target}_valid_for_eval"
    ref_col = f"ref_{target}"
    train_mask = df_train[valid_col].fillna(False).to_numpy(dtype=bool)
    val_mask = df_val[valid_col].fillna(False).to_numpy(dtype=bool)
    train_rows = df_train.loc[train_mask].copy()
    val_rows = df_val.loc[val_mask].copy()
    required_rows = 20 if allow_small_dataset else max(20, int(min_valid_windows))
    if len(train_rows) < required_rows:
        override = (
            " Pass --allow-small-cnn-dataset only for an explicitly labelled "
            "overfit/feasibility experiment."
            if not allow_small_dataset
            else ""
        )
        raise ValueError(
            f"Not enough valid {target.upper()} windows for cnn_1d "
            f"({len(train_rows)} available, {required_rows} required). "
            "Fix upstream publish coverage or keep --model-family gradient_boosting."
            + override
        )

    train_sessions = (
        df_train["session_id"].to_numpy(dtype=object)
        if "session_id" in df_train
        else np.repeat("train", len(df_train))
    )
    val_sessions = (
        df_val["session_id"].to_numpy(dtype=object)
        if "session_id" in df_val
        else np.repeat("validation", len(df_val))
    )
    train_windows, train_endpoints = build_causal_windows(
        X_train_all.to_numpy(dtype=np.float32),
        train_sessions,
        config.window_size,
        endpoint_mask=train_mask,
    )
    val_windows, val_endpoints = build_causal_windows(
        X_val_all.to_numpy(dtype=np.float32),
        val_sessions,
        config.window_size,
        endpoint_mask=val_mask,
    )
    y_train = df_train.iloc[train_endpoints][ref_col].to_numpy(dtype=np.float32)
    y_val = df_val.iloc[val_endpoints][ref_col].to_numpy(dtype=np.float32)
    weights = get_sample_weights(train_rows, target, sample_weight_mode)
    model = Keras1DCNNRegressor(config=config, random_state=random_state)
    validation_data = (val_windows, y_val) if len(val_windows) >= 10 else None
    model.fit(
        train_windows,
        y_train,
        validation_data=validation_data,
        sample_weight=weights,
    )
    trained_epochs = max((len(values) for values in model.history_.values()), default=0)
    best_val_rmse = float("nan")
    if len(val_windows):
        val_pred = model.predict_windows(val_windows)
        best_val_rmse = float(np.sqrt(mean_squared_error(y_val, val_pred)))
    meta = {
        "model_family": MODEL_FAMILY_CNN_1D,
        "train_rows": int(len(train_rows)),
        "val_rows": int(len(val_rows)),
        "window_size": int(config.window_size),
        "input_channels": int(X_train_all.shape[1]),
        "trained_epochs": int(trained_epochs),
        "best_val_rmse": best_val_rmse,
        "used_early_stopping": validation_data is not None,
    }
    return model, meta


def fit_selected_target_model(
    df_train,
    df_val,
    target,
    X_train_all,
    X_val_all,
    params,
    args,
    random_state,
):
    model_family = getattr(args, "model_family", MODEL_FAMILY_GRADIENT_BOOSTING)
    if model_family == MODEL_FAMILY_CNN_1D:
        return fit_cnn_target_model(
            df_train,
            df_val,
            target,
            X_train_all,
            X_val_all,
            config=resolve_cnn_config(args),
            random_state=random_state,
            sample_weight_mode=args.sample_weight_mode,
            min_valid_windows=int(getattr(args, "cnn_min_valid_windows", 500)),
            allow_small_dataset=bool(getattr(args, "allow_small_cnn_dataset", False)),
        )
    return fit_target_model(
        df_train,
        df_val,
        target,
        X_train_all,
        X_val_all,
        params=params,
        random_state=random_state,
        sample_weight_mode=args.sample_weight_mode,
        use_early_stopping=not args.disable_early_stopping,
        early_stop_strategy=args.early_stop_strategy,
    )


def apply_causal_slew_limit(df, col, max_delta_per_s):
    if max_delta_per_s is None or not np.isfinite(max_delta_per_s) or max_delta_per_s <= 0:
        return df
    df = df.copy()
    pieces = []
    for _, g in df.groupby("session_id", sort=False):
        g    = g.sort_values("timestamp_s").copy()
        vals = pd.to_numeric(g[col], errors="coerce").to_numpy(dtype=float)
        ts   = pd.to_numeric(g["timestamp_s"], errors="coerce").to_numpy(dtype=float)
        fi   = np.where(np.isfinite(vals) & np.isfinite(ts))[0]
        if len(fi) >= 1:
            first_idx = fi[0]
            # The first output may only depend on the current sample (or a
            # persisted prior state supplied by a future streaming wrapper).
            # Seeding from the first three predictions would look ahead into
            # t+1/t+2 and make the declared causal postprocessor non-causal.
            anchor = float(vals[first_idx])
            vals[first_idx] = anchor
            prev_time = ts[first_idx]
            prev_out = vals[first_idx]
            for idx in fi[1:]:
                dt        = max(1e-6, ts[idx] - prev_time)
                max_delta = float(max_delta_per_s) * dt
                vals[idx] = np.clip(vals[idx], prev_out - max_delta, prev_out + max_delta)
                prev_out = vals[idx]
                prev_time = ts[idx]
        g[col] = vals
        pieces.append(g)
    return pd.concat(pieces, ignore_index=True)


def add_predictions(df, X_all, model_hr=None, model_rr=None, hr_slew_limit=None, rr_slew_limit=None):
    df = df.copy()
    X  = X_all.to_numpy(dtype=np.float32)
    if model_hr is not None:
        raw_hr = (
            model_hr.predict_aligned(df, X_all)
            if hasattr(model_hr, "predict_aligned")
            else model_hr.predict(X)
        )
        df["pred_hr_raw"] = np.asarray(raw_hr, dtype=float)
        df["pred_hr"] = np.clip(raw_hr, HR_RANGE[0], HR_RANGE[1])
        df = apply_causal_slew_limit(df, "pred_hr", hr_slew_limit)
    else:
        df["pred_hr_raw"] = np.nan
        df["pred_hr"] = np.nan
    if model_rr is not None:
        raw_rr = (
            model_rr.predict_aligned(df, X_all)
            if hasattr(model_rr, "predict_aligned")
            else model_rr.predict(X)
        )
        df["pred_rr_raw"] = np.asarray(raw_rr, dtype=float)
        df["pred_rr"] = np.clip(raw_rr, RR_RANGE[0], RR_RANGE[1])
        df = apply_causal_slew_limit(df, "pred_rr", rr_slew_limit)
    else:
        df["pred_rr_raw"] = np.nan
        df["pred_rr"] = np.nan
    return df


def enforce_baseline_gate(hr_baseline, args):
    if not args.require_baseline_gate:
        return
    r    = hr_baseline["valid_only"]["r"]
    rmse = hr_baseline["valid_only"]["rmse"]
    if ((np.isfinite(args.gate_hr_min_r) and (not np.isfinite(r)   or r    < args.gate_hr_min_r)) or
        (np.isfinite(args.gate_hr_max_rmse) and (not np.isfinite(rmse) or rmse > args.gate_hr_max_rmse))):
        raise ValueError(
            f"Baseline gate FAILED: r={r:.3f}, RMSE={rmse:.2f}. "
            f"Required: r>={args.gate_hr_min_r:.3f}, RMSE<={args.gate_hr_max_rmse:.2f}. "
            f"Do NOT train until signal quality improves.")


def fit_linear_embedded_surrogate(X, y):
    Xn    = X.to_numpy(dtype=np.float64)
    X_aug = np.column_stack([np.ones(len(Xn)), Xn])
    coeffs, *_ = np.linalg.lstsq(X_aug, y.astype(np.float64), rcond=None)
    return coeffs[1:, :].astype(np.float32), coeffs[0, :].astype(np.float32)


def export_linear_embedded_header(feature_names, weights, bias, out_path):
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("#pragma once\n\n")
        f.write(f"// Auto-generated by radar_vital_trainer_v{VERSION}\n")
        f.write(f"#define RVT_EMBEDDED_INPUT_DIM {len(feature_names)}\n\n")
        f.write("static const float RVT_EMBEDDED_BIAS[2] = {")
        f.write(", ".join(f"{float(x):.8g}f" for x in bias.tolist()))
        f.write("};\n\n")
        f.write(f"static const char* RVT_EMBEDDED_FEATURE_NAMES[{len(feature_names)}] = {{\n")
        for name in feature_names:
            f.write(f'  "{name}",\n')
        f.write("};\n\n")
        f.write(f"static const float RVT_EMBEDDED_WEIGHTS[{len(feature_names)}][2] = {{\n")
        for row in weights:
            f.write("  {" + ", ".join(f"{float(x):.8g}f" for x in row.tolist()) + "},\n")
        f.write("};\n")


def build_tiny_surrogate(input_dim):
    import tensorflow as tf
    inp = tf.keras.Input(shape=(input_dim,), name="features")
    x   = tf.keras.layers.Dense(32, activation="relu")(inp)
    x   = tf.keras.layers.Dense(16, activation="relu")(x)
    out = tf.keras.layers.Dense(2, name="vitals")(x)
    m   = tf.keras.Model(inp, out, name="radar_vital_surrogate")
    m.compile(optimizer=tf.keras.optimizers.Adam(1e-3), loss="mse", metrics=["mae"])
    return m


def export_tflite_surrogate(X_train, Y_train, X_val, Y_val, feature_names, out_dir):
    try:
        import tensorflow as tf
    except Exception as e:
        raise RuntimeError("TensorFlow required for TFLite export. pip install tensorflow") from e
    model = build_tiny_surrogate(X_train.shape[1])
    model.fit(X_train.to_numpy(dtype=np.float32), Y_train.astype(np.float32),
              validation_data=(X_val.to_numpy(dtype=np.float32), Y_val.astype(np.float32)),
              epochs=250, batch_size=min(64, max(8, len(X_train)//8)), verbose=0,
              callbacks=[
                  tf.keras.callbacks.EarlyStopping(monitor="val_loss", patience=15, restore_best_weights=True),
                  tf.keras.callbacks.ReduceLROnPlateau(monitor="val_loss", patience=7, factor=0.5, min_lr=1e-5),
              ])
    pred_val = model.predict(X_val.to_numpy(dtype=np.float32), verbose=0)
    surrogate_val_rmse = float(np.sqrt(np.mean((pred_val - Y_val.astype(np.float32))**2)))
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.target_spec.supported_types = [tf.float16]
    tflite_model = converter.convert()
    tflite_path  = os.path.join(out_dir, "embedded_surrogate.tflite")
    with open(tflite_path, "wb") as f:
        f.write(tflite_model)
    header_path = os.path.join(out_dir, "embedded_surrogate_tflite.h")
    with open(header_path, "w", encoding="utf-8") as f:
        f.write("#pragma once\n\n")
        f.write(bytes_to_c_array(tflite_model, "g_radar_vital_surrogate_tflite"))
    save_json({"feature_names": list(feature_names), "input_dim": int(X_train.shape[1]),
               "tflite_bytes": int(len(tflite_model)), "surrogate_val_rmse": surrogate_val_rmse},
              os.path.join(out_dir, "embedded_surrogate_manifest.json"))
    return {"tflite_path": tflite_path, "header_path": header_path,
            "surrogate_val_rmse": surrogate_val_rmse, "tflite_bytes": int(len(tflite_model))}


def maybe_export_embedded(args, out_dir, X_train_all, X_eval_all, train_pred, eval_pred,
                           eval_target_name="validation"):
    mode = getattr(args, "embedded_export", "none")
    if mode == "none":
        return {"mode": "none"}
    teacher_train = np.column_stack([train_pred["pred_hr"].to_numpy(dtype=np.float32),
                                     train_pred["pred_rr"].to_numpy(dtype=np.float32)])
    teacher_eval  = np.column_stack([eval_pred["pred_hr"].to_numpy(dtype=np.float32),
                                     eval_pred["pred_rr"].to_numpy(dtype=np.float32)])
    info = {"mode": mode, "eval_target": eval_target_name}
    if mode in {"linear", "both"}:
        weights, bias = fit_linear_embedded_surrogate(X_train_all, teacher_train)
        linear_header = os.path.join(out_dir, "embedded_linear_surrogate.h")
        export_linear_embedded_header(list(X_train_all.columns), weights, bias, linear_header)
        linear_eval = X_eval_all.to_numpy(dtype=np.float32) @ weights + bias
        linear_teacher_rmse = float(np.sqrt(np.mean((linear_eval - teacher_eval)**2)))
        linear_eval_df = eval_pred.copy()
        linear_eval_df["pred_hr"] = np.clip(linear_eval[:, 0], HR_RANGE[0], HR_RANGE[1])
        linear_eval_df["pred_rr"] = np.clip(linear_eval[:, 1], RR_RANGE[0], RR_RANGE[1])
        info.update({"linear_header": linear_header, "linear_teacher_rmse": linear_teacher_rmse,
                     "linear_ref_hr": model_summary(linear_eval_df, "hr"),
                     "linear_ref_rr": model_summary(linear_eval_df, "rr")})
    if mode in {"tflite", "both"}:
        if not tensorflow_is_available():
            msg = "TensorFlow not installed. pip install tensorflow"
            if mode == "tflite":
                raise RuntimeError(msg)
            warn(msg + " Skipping TFLite export.")
            info["tflite_skipped"] = msg
        else:
            info.update(export_tflite_surrogate(
                X_train_all, teacher_train, X_eval_all, teacher_eval,
                list(X_train_all.columns), out_dir))
    save_json(info, os.path.join(out_dir, "embedded_export_summary.json"))
    return info


def cmd_predict(args):
    os.makedirs(args.out, exist_ok=True)
    _verify_model_dir(args.model_dir)
    bundle_metadata = _read_model_bundle_metadata(args.model_dir)
    model_hr = None
    model_rr = None
    hr_path = os.path.join(args.model_dir, "model_hr.pkl")
    rr_path = os.path.join(args.model_dir, "model_rr.pkl")
    if os.path.exists(hr_path):
        with open(hr_path, "rb") as f:
            model_hr = pickle.load(f)
    if os.path.exists(rr_path):
        with open(rr_path, "rb") as f:
            model_rr = pickle.load(f)
    if model_hr is None and model_rr is None:
        raise ValueError("No model_hr.pkl or model_rr.pkl found in model_dir.")
    with open(os.path.join(args.model_dir, "preprocessor.pkl"), "rb") as f:
        pre = pickle.load(f)
    feature_mode          = pre.get("feature_mode", "full")
    phase_unit            = pre.get("phase_unit", "rad")
    heart_fft_window_s    = float(pre.get("heart_fft_window_s", 5.0))
    breath_fft_window_s   = float(pre.get("breath_fft_window_s", 10.0))
    missing_flag_cols     = list(pre.get("missing_flag_cols", []))
    feature_cols          = list(pre["base_feature_cols"])
    expanded_feature_cols = list(pre["expanded_feature_cols"])
    impute_values         = {str(k): float(v) for k, v in pre.get("impute_values", {}).items()}
    trained_targets       = list(pre.get("trained_targets", []))
    if bundle_metadata is not None:
        sequence = pre.get("sequence")
        sequence_window_size = (
            sequence.get("window_size") if isinstance(sequence, dict) else None
        )
        assert_bundle_inference_compatible(
            bundle_metadata,
            actual_columns=expanded_feature_cols,
            requested_model_family=pre.get(
                "model_family", MODEL_FAMILY_GRADIENT_BOOSTING
            ),
            sequence_window_size=sequence_window_size,
        )

    base_df = load_and_aggregate_radar_only(
        args.radar, phase_unit=phase_unit,
        heart_fft_window_s=heart_fft_window_s, breath_fft_window_s=breath_fft_window_s)
    feat_df = validity_masks(engineer_temporal_features(base_df, feature_mode=feature_mode))
    X_all   = transform_feature_matrix(feat_df, feature_cols, impute_values, missing_flag_cols)
    X_all   = X_all.reindex(columns=expanded_feature_cols, fill_value=0.0).astype(np.float32)
    if bundle_metadata is not None:
        assert_bundle_inference_compatible(
            bundle_metadata,
            actual_columns=list(X_all.columns),
            requested_model_family=pre.get(
                "model_family", MODEL_FAMILY_GRADIENT_BOOSTING
            ),
            sequence_window_size=(
                pre.get("sequence", {}).get("window_size")
                if isinstance(pre.get("sequence"), dict)
                else None
            ),
        )
    pred_df = add_predictions(feat_df, X_all, model_hr=model_hr, model_rr=model_rr,
                              hr_slew_limit=pre.get("slew_limit_hr_per_s"),
                              rr_slew_limit=pre.get("slew_limit_rr_per_s"))
    out_csv = os.path.join(args.out, "predictions.csv")
    pred_df.to_csv(out_csv, index=False)
    if model_hr is not None:
        maybe_plot_timeseries(pred_df.assign(ref_hr=np.nan, ref_rr=np.nan), "hr",
                              os.path.join(args.out, "predict_hr_overlay.png"),
                              pred_col="pred_hr", enabled=not args.no_plots)
    if model_rr is not None:
        maybe_plot_timeseries(pred_df.assign(ref_hr=np.nan, ref_rr=np.nan), "rr",
                              os.path.join(args.out, "predict_rr_overlay.png"),
                              pred_col="pred_rr", enabled=not args.no_plots)
    summary = {
        "version": VERSION,
        "model_family": pre.get("model_family", MODEL_FAMILY_GRADIENT_BOOSTING),
        "sequence": pre.get("sequence"),
        "rows": int(len(pred_df)),
        "sessions": list(dict.fromkeys(pred_df["session_id"].tolist())),
        "feature_mode": feature_mode,
        "phase_unit": phase_unit,
        "heart_fft_window_s": heart_fft_window_s,
        "breath_fft_window_s": breath_fft_window_s,
        "trained_targets": trained_targets if trained_targets else [t for t, m in [("hr", model_hr), ("rr", model_rr)] if m is not None],
        "output_csv": out_csv,
    }
    save_json(summary, os.path.join(args.out, "predict_summary.json"))
    write_text_report(os.path.join(args.out, "predict_report.txt"), summary)
    print(f"[OUT] Predictions saved to {out_csv}")


def cmd_analyse(args):
    os.makedirs(args.out, exist_ok=True)
    session_root = _session_root_for_outputs(args.out)
    base_df = load_and_align_multiple_base(
        args.radar, args.ref,
        tolerance_s=args.tolerance_s, ref_offsets_s=args.ref_offset_s,
        auto_align_start=args.auto_align_start, merge_direction=args.merge_direction,
        phase_unit=args.phase_unit, heart_fft_window_s=args.heart_fft_window_s,
        breath_fft_window_s=args.breath_fft_window_s)
    feat_df = validity_masks(engineer_temporal_features(base_df, feature_mode=args.feature_mode))
    hr_base = baseline_summary(feat_df, "hr")
    rr_base = baseline_summary(feat_df, "rr")
    raw_radar_df = _combine_radar_frames(args.radar)
    hr_locked_base, ml_gate_locked = _phase_subset_summary(feat_df, "phase_locked")
    hr_settling_base, ml_gate_settling = _phase_subset_summary(feat_df, "phase_settling")
    ml_gate_combined = _ml_gate_block_from_baseline(hr_base)
    coverage_locked = float(hr_locked_base.get("coverage_pct", float("nan")))
    coverage_settling = float(hr_settling_base.get("coverage_pct", float("nan")))
    gate_audit = _build_gate_audit(raw_radar_df, feat_df)
    locked_mask = safe_series(feat_df, "phase_locked", default=0.0) >= 0.5
    settling_mask = safe_series(feat_df, "phase_settling", default=0.0) >= 0.5
    hr_eval_mask = safe_series(feat_df, "hr_valid_for_eval", default=0.0).to_numpy(dtype=bool)
    gate_audit["valid_pairs"]["locked_hr_pairs"] = int(np.sum(finite_pair_mask(feat_df, "ref_hr", "reported_hr_mean") & hr_eval_mask & locked_mask.to_numpy(dtype=bool)))
    gate_audit["valid_pairs"]["settling_hr_pairs"] = int(np.sum(finite_pair_mask(feat_df, "ref_hr", "reported_hr_mean") & hr_eval_mask & settling_mask.to_numpy(dtype=bool)))
    publish_reason_histogram = {
        "hr": _histogram_from_series(raw_radar_df.get("hr_publish_reason", pd.Series(dtype=float)), HR_PUBLISH_REASON_NAMES),
        "rr": _histogram_from_series(raw_radar_df.get("rr_publish_reason", pd.Series(dtype=float)), RR_PUBLISH_REASON_NAMES),
    }
    hr_gate_reason_histogram = _histogram_from_series(raw_radar_df.get("hr_gate_reason", pd.Series(dtype=float)), HR_GATE_REASON_NAMES)
    rr_gate_reason_histogram = _histogram_from_series(raw_radar_df.get("rr_gate_reason", pd.Series(dtype=float)), RR_GATE_REASON_NAMES)
    agc_anomaly_flags = _build_agc_anomaly_flags(raw_radar_df)
    fw_truthfulness = _truthfulness_from_radar(
        raw_radar_df,
        contract_length=_radar_csv_on_disk_contract_length(args.radar),
    )
    coverage_ledger = _coverage_loss_ledger(raw_radar_df)
    oracle_audit = _oracle_candidate_audit(feat_df)
    hr_publish_source_hist = _histogram_from_series(raw_radar_df.get("hr_publish_source", pd.Series(dtype=float)), HR_PUBLISH_SOURCE_NAMES)
    _warn_on_manifest_mismatch(session_root, fw_truthfulness)
    session_start_s = float(safe_series(feat_df, "timestamp_s", default=np.nan).dropna().min()) if len(feat_df) else 0.0
    early_mask = (safe_series(feat_df, "timestamp_s", default=np.nan) < (session_start_s + 30.0)).to_numpy(dtype=bool)
    suspect_half_rate_mask = (
        (safe_series(feat_df, "reported_hr_mean", default=0.0) < 70.0) &
        (safe_series(feat_df, "ref_hr", default=0.0) > 85.0)
    ).to_numpy(dtype=bool)
    all_mask = np.ones(len(feat_df), dtype=bool)
    experimental_audit = {
        "experimental_profile_default": False,
        "early_window_excluded_n": int(np.sum(early_mask)),
        "suspect_half_rate_excluded_n": int(np.sum(suspect_half_rate_mask)),
        "combined_all": _experimental_hr_gate_metrics(feat_df, all_mask),
        "combined_without_early_window": _experimental_hr_gate_metrics(feat_df, ~early_mask),
        "combined_without_suspect_half_rate": _experimental_hr_gate_metrics(feat_df, ~suspect_half_rate_mask),
        "combined_without_early_or_suspect_half_rate": _experimental_hr_gate_metrics(feat_df, ~(early_mask | suspect_half_rate_mask)),
    }
    print("\n== BASELINE SUMMARY ==")
    print_metric_block("HR baseline", hr_base)
    print_metric_block("RR baseline", rr_base)

    r = hr_base["valid_only"]["r"]
    rmse = hr_base["valid_only"]["rmse"]
    gate_pass = np.isfinite(r) and r > 0.4 and np.isfinite(rmse) and rmse < 8.0
    secondary_gate = _coverage_penalized_hr_gate(feat_df, window_s=600.0)
    session_duration_s = _session_duration_s(feat_df)
    short_session = np.isfinite(session_duration_s) and session_duration_s < SECONDARY_GATE_MIN_DURATION_S
    primary_gate = dict(ml_gate_combined)
    primary_gate["min_n"] = 15
    primary_gate["strict_ml_passed"] = bool(gate_pass and _safe_int(primary_gate.get("n"), 0) >= 15)
    primary_gate["secondary_gate"] = secondary_gate
    if short_session:
        primary_gate["passed"] = False
        primary_gate["conditional_passed"] = False
        primary_gate["ml_ready"] = False
        primary_gate["status"] = "DEFERRED_TOO_SHORT_TO_JUDGE"
        primary_gate["reason"] = "too_short_to_judge"
        primary_gate["operator_message"] = "Session duration is below the minimum threshold for ML judgment."
    elif _safe_int(primary_gate.get("n"), 0) < 15:
        if secondary_gate.get("kind") == "deferred":
            primary_gate["passed"] = False
            primary_gate["conditional_passed"] = False
            primary_gate["ml_ready"] = False
            primary_gate["status"] = "DEFERRED_TOO_SHORT_TO_JUDGE"
            primary_gate["reason"] = "too_short_to_judge"
            primary_gate["operator_message"] = secondary_gate.get("operator_message")
        elif secondary_gate.get("passed"):
            primary_gate["passed"] = True
            primary_gate["conditional_passed"] = True
            primary_gate["ml_ready"] = False
            primary_gate["status"] = "CONDITIONAL_PASS_N_TOO_SMALL_FOR_ML"
            primary_gate["reason"] = "valid_hr_pairs_below_min_n_but_coverage_penalized_rmse_passed"
            primary_gate["operator_message"] = secondary_gate.get("operator_message")
        else:
            primary_gate["passed"] = False
            primary_gate["conditional_passed"] = False
            primary_gate["ml_ready"] = False
            primary_gate["status"] = "INSUFFICIENT_DATA"
            primary_gate["reason"] = "valid_hr_pairs_below_min_n"
    else:
        primary_gate["ml_ready"] = bool(gate_pass)
        primary_gate["status"] = "PASS" if gate_pass else "FAIL"
    print(f"\n{'='*50}")
    print(f"ML gate status:         {primary_gate.get('status', 'FAIL')}  (n={_safe_int(primary_gate.get('n'), 0)}, min_n={_safe_int(primary_gate.get('min_n'), 0)})")
    print(f"ML gate  r>0.4:    {_green('PASS') if (np.isfinite(r) and r>0.4) else _red('FAIL')}  (r={r:.3f})")
    print(f"ML gate  RMSE<8:   {_green('PASS') if (np.isfinite(rmse) and rmse<8) else _red('FAIL')}  (RMSE={rmse:.2f})")
    if secondary_gate.get("kind") == "deferred":
        print("ML gate secondary:  DEFERRED  (too short to judge)")
    else:
        print(f"ML gate conditional 10min cov-pen RMSE<8: "
              f"{_green('PASS') if secondary_gate.get('passed') else _red('FAIL')}  "
              f"(cov_pen={_to_num(secondary_gate.get('coverage_penalized_rmse'), float('nan')):.2f}, "
              f"n={_safe_int(secondary_gate.get('n'), 0)}, cov={_to_num(secondary_gate.get('coverage_pct'), float('nan')):.1f}%)")
    print(f"ML gate locked audit:    r={_to_num(ml_gate_locked.get('r'), float('nan')):.3f}  RMSE={_to_num(ml_gate_locked.get('rmse'), float('nan')):.2f}  n={_safe_int(ml_gate_locked.get('n'), 0)}")
    print(f"ML gate settling audit:  r={_to_num(ml_gate_settling.get('r'), float('nan')):.3f}  RMSE={_to_num(ml_gate_settling.get('rmse'), float('nan')):.2f}  n={_safe_int(ml_gate_settling.get('n'), 0)}")
    if primary_gate.get("status") == "CONDITIONAL_PASS_N_TOO_SMALL_FOR_ML":
        print(_yellow("-> Conditional pass: n is too small for ML, but the 10-minute coverage-penalized HR RMSE passed."))
        print("  Treat this as an operator-facing signal-quality pass, not a training-ready dataset.")
    elif primary_gate.get("status") == "INSUFFICIENT_DATA":
        print(_red("-> Gate INSUFFICIENT_DATA. Do NOT run `train` yet."))
        print("  Collect more valid locked rows or fix firmware coverage before training.")
    elif gate_pass:
        print(_green("-> Gate PASSED. You may proceed to `train`."))
    else:
        print(_red("-> Gate FAILED. Do NOT run `train` yet."))
        print("  Investigate pqi_heart, firmware fixes, or collect more sessions.")
    print(f"{'='*50}\n")

    out_csv = os.path.join(args.out, "aligned_1hz_features.csv")
    feat_df.to_csv(out_csv, index=False)
    feat_df.to_csv(os.path.join(args.out, "aligned_1hz.csv"), index=False)
    hr_overlay = os.path.join(args.out, "analyse_hr_overlay.png")
    rr_overlay = os.path.join(args.out, "analyse_rr_overlay.png")
    hr_ba = os.path.join(args.out, "analyse_hr_bland_altman.png")
    rr_ba = os.path.join(args.out, "analyse_rr_bland_altman.png")
    phase_png = os.path.join(args.out, "analyse_phase_signals.png")
    maybe_plot_timeseries(feat_df, "hr", hr_overlay, enabled=not args.no_plots)
    maybe_plot_timeseries(feat_df, "rr", rr_overlay, enabled=not args.no_plots)
    maybe_plot_phase_signals(feat_df, phase_png, enabled=not args.no_plots)
    y_true_hr, y_pred_hr = get_valid_pair_arrays(feat_df, "hr", "reported_hr_mean", valid_only=True)
    y_true_rr, y_pred_rr = get_valid_pair_arrays(feat_df, "rr", "reported_rr_mean", valid_only=True)
    maybe_plot_bland_altman(y_true_hr, y_pred_hr,
                            "HR Bland-Altman (reported vs ref, valid-only)",
                            hr_ba,
                            enabled=not args.no_plots, annotate=args.annotate_bland_altman)
    maybe_plot_bland_altman(y_true_rr, y_pred_rr,
                            "RR Bland-Altman (reported vs ref, valid-only)",
                            rr_ba,
                            enabled=not args.no_plots, annotate=args.annotate_bland_altman)

    ref_dt = safe_series(feat_df, "ref_dt_s")
    pqi = safe_series(feat_df, "pqi_heart_mean")
    pqi_lock_pct = float((pqi >= 0.35).mean() * 100.0) if pqi.notna().any() else float("nan")
    raw_hr_bias_estimate = compute_raw_hr_bias_estimate(feat_df)
    raw_hr_uncorrected_bias_estimate = compute_hr_bias_estimate(feat_df, ["raw_hr_uncorrected_mean", "raw_hr_mean", "raw_hr_nonzero_mean"], series_label="raw_hr_uncorrected")
    raw_hr_corrected_bias_estimate = compute_hr_bias_estimate(feat_df, ["raw_hr_corrected_mean"], series_label="raw_hr_corrected")
    raw_hr_sumfreq_estimate = compute_raw_hr_sumfreq_error_estimate(feat_df)
    ble_ref_quality = _compute_ble_ref_quality(args.ref)
    raw_hr_uncorrected_bias_bpm = float(raw_hr_uncorrected_bias_estimate.get("bias_bpm", float("nan")))
    raw_hr_corrected_bias_bpm = float(raw_hr_corrected_bias_estimate.get("bias_bpm", float("nan")))
    if {"raw_hr_corrected", "raw_hr_uncorrected"}.issubset(raw_radar_df.columns):
        corrected = pd.to_numeric(raw_radar_df["raw_hr_corrected"], errors="coerce")
        uncorrected = pd.to_numeric(raw_radar_df["raw_hr_uncorrected"], errors="coerce")
        correction_mask = corrected.notna() & uncorrected.notna() & (corrected != uncorrected)
        raw_hr_correction_coverage_pct = float(100.0 * correction_mask.mean()) if len(raw_radar_df) else float("nan")
    else:
        raw_hr_correction_coverage_pct = float("nan")
    hr_rescue_publish_count = _safe_int(hr_publish_source_hist.get("corrected_raw_rescue"), 0) if isinstance(hr_publish_source_hist, dict) else 0
    hr_rescue_publish_coverage_pct = float(100.0 * hr_rescue_publish_count / max(len(raw_radar_df), 1)) if len(raw_radar_df) else float("nan")
    raw_rescue_reject_count = _derive_raw_rescue_reject_count(raw_radar_df)
    summary = {
        "version": VERSION, "n_rows": int(len(feat_df)),
        "sessions": list(dict.fromkeys(feat_df["session_id"].tolist())),
        "tolerance_s": args.tolerance_s, "merge_direction": args.merge_direction,
        "auto_align_start": bool(args.auto_align_start),
        "ref_offset_s": normalize_offsets(len(args.radar), args.ref_offset_s),
        "feature_mode": args.feature_mode, "phase_unit": args.phase_unit,
        "feature_engineering_version": FEATURE_ENGINEERING_VERSION,
        "feature_schema_hash": feature_schema_hash(),
        "scoring_weights_hash": _scoring_weights_hash(),
        "heart_fft_window_s": args.heart_fft_window_s,
        "breath_fft_window_s": args.breath_fft_window_s,
        "alignment_dt_s": {
            "mean": float(ref_dt.mean()) if ref_dt.notna().any() else float("nan"),
            "max_abs": float(ref_dt.abs().max()) if ref_dt.notna().any() else float("nan"),
        },
        "pqi_lock_pct": pqi_lock_pct,
        "ml_gate": primary_gate,
        "ml_gate_policy": primary_gate,
        "ml_gate_secondary": secondary_gate,
        "ml_gate_combined": ml_gate_combined,
        "ml_gate_locked": ml_gate_locked,
        "ml_gate_settling": ml_gate_settling,
        "hr_baseline": hr_base, "rr_baseline": rr_base,
        "hr_baseline_locked": hr_locked_base,
        "hr_baseline_settling": hr_settling_base,
        "coverage_locked": coverage_locked,
        "coverage_settling": coverage_settling,
        "raw_hr_bias_estimate": raw_hr_bias_estimate,
        "raw_hr_bias_audit": raw_hr_bias_estimate,
        "raw_hr_uncorrected_bias_estimate": raw_hr_uncorrected_bias_estimate,
        "raw_hr_corrected_bias_estimate": raw_hr_corrected_bias_estimate,
        "raw_hr_uncorrected_bias_bpm": raw_hr_uncorrected_bias_bpm,
        "raw_hr_corrected_bias_bpm": raw_hr_corrected_bias_bpm,
        "raw_hr_correction_coverage_pct": raw_hr_correction_coverage_pct,
        "raw_hr_sumfreq_estimate": raw_hr_sumfreq_estimate,
        "ble_ref_quality": ble_ref_quality,
        "fw_truthfulness": fw_truthfulness,
        "gate_audit": gate_audit,
        "coverage_ledger": coverage_ledger,
        "hr_coverage_loss_by_stage": coverage_ledger.get("hr", {}).get("histogram", {}),
        "rr_coverage_loss_by_stage": coverage_ledger.get("rr", {}).get("histogram", {}),
        "hr_salvageable_frames_pct": coverage_ledger.get("hr", {}).get("salvageable_pct"),
        "rr_salvageable_frames_pct": coverage_ledger.get("rr", {}).get("salvageable_pct"),
        "oracle_candidate_audit": oracle_audit,
        "hr_oracle_candidate_mae_bpm": oracle_audit.get("hr", {}).get("oracle_candidate_mae_bpm"),
        "hr_oracle_candidate_rmse_bpm": oracle_audit.get("hr", {}).get("oracle_candidate_rmse_bpm"),
        "hr_oracle_candidate_coverage_pct": oracle_audit.get("hr", {}).get("oracle_candidate_coverage_pct"),
        "rr_oracle_candidate_mae_bpm": oracle_audit.get("rr", {}).get("oracle_candidate_mae_bpm"),
        "rr_oracle_candidate_rmse_bpm": oracle_audit.get("rr", {}).get("oracle_candidate_rmse_bpm"),
        "rr_oracle_candidate_coverage_pct": oracle_audit.get("rr", {}).get("oracle_candidate_coverage_pct"),
        "hr_best_candidate_source_histogram": oracle_audit.get("hr", {}).get("best_candidate_source_histogram", {}),
        "rr_best_candidate_source_histogram": oracle_audit.get("rr", {}).get("best_candidate_source_histogram", {}),
        "hr_rescue_publish_count": hr_rescue_publish_count,
        "hr_rescue_publish_coverage_pct": hr_rescue_publish_coverage_pct,
        "raw_rescue_reject_count": raw_rescue_reject_count,
        "hr_publish_source_histogram": hr_publish_source_hist,
        "publish_reason_histogram": publish_reason_histogram,
        "hr_gate_reason_histogram": hr_gate_reason_histogram,
        "rr_gate_reason_histogram": rr_gate_reason_histogram,
        "agc_anomaly_flags": agc_anomaly_flags,
        "experimental_audit": experimental_audit,
    }
    summary["internal_consistency_score"] = _compute_internal_consistency_score(summary)
    summary["session_quality_score"] = _compute_session_quality_score(summary, feat_df)
    has_ref = any(float((blk.get("all_frames", {}) if isinstance(blk.get("all_frames"), dict) else {}).get("n", 0)) > 0 for blk in (hr_base, rr_base))
    ghost_session = False
    if not has_ref:
        human_mean = safe_series(feat_df, "human_detected_mean")
        cand_hr_mean = safe_series(feat_df, "candidate_hr_mean")
        cand_rr_mean = safe_series(feat_df, "candidate_rr_mean")
        ghost_session = human_mean.notna().any() and human_mean.fillna(0).mean() > 0.5 and (((cand_hr_mean.fillna(0) > 0).sum() > 0) or ((cand_rr_mean.fillna(0) > 0).sum() > 0))
    review_flags = []
    rr_bias = float(((rr_base.get("valid_only") or {}).get("bias", float("nan"))))
    rr_r = float(((rr_base.get("valid_only") or {}).get("r", float("nan"))))
    hr_cov = float(hr_base.get("coverage_pct", float("nan")))
    raw_hr_bias = float((raw_hr_bias_estimate or {}).get("bias_bpm", float("nan")))
    raw_hr_sumfreq_mae = float((raw_hr_sumfreq_estimate or {}).get("mean_abs_error_bpm", float("nan")))
    raw_hr_corrected_bias = float((raw_hr_corrected_bias_estimate or {}).get("bias_bpm", float("nan")))
    if np.isfinite(rr_bias) and abs(rr_bias) > 3.5:
        review_flags.append("rr_bias_high")
    if np.isfinite(rr_r) and rr_r < 0.0:
        review_flags.append("rr_negative_correlation")
    if np.isfinite(hr_cov) and hr_cov < 20.0:
        review_flags.append("hr_coverage_low")
    if np.isfinite(raw_hr_bias) and abs(raw_hr_bias) > 8.0:
        review_flags.append("raw_hr_bias_high")
    if np.isfinite(raw_hr_sumfreq_mae) and raw_hr_sumfreq_mae <= 4.0:
        review_flags.append("raw_hr_sumfreq_match")
    if np.isfinite(raw_hr_corrected_bias) and np.isfinite(raw_hr_bias) and abs(raw_hr_corrected_bias) < abs(raw_hr_bias):
        review_flags.append("raw_hr_correction_improves_bias")
    if ghost_session:
        review_flags.append("ghost_target_session")
    summary["has_reference"] = bool(has_ref)
    summary["ghost_target_session"] = bool(ghost_session)
    summary["review_required"] = bool(review_flags)
    summary["review_flags"] = review_flags

    # v15.0 Stage C: pqi_v15 shadow analysis. Surfaces whether the gap-aware
    # PQI would unlock more candidate-eligible frames vs the v14 PQI, AND
    # whether it would inflate on motion frames (the go/no-go safety check).
    # PR72 session-data audit: the v15 PQI columns are never aggregated into
    # the 1 Hz feature frame, so running the shadow on feat_df always claimed
    # "columns not present (firmware <v15.0)" even for v16.4 logs that carry
    # the reserved columns with sentinel -1.0. Run it on the raw radar frame.
    summary["pqi_v15_shadow"] = _build_pqi_v15_shadow(
        raw_radar_df if isinstance(raw_radar_df, pd.DataFrame) and len(raw_radar_df) else feat_df
    )

    # v15.0 Stage E: adaptive correction shadow. Dual-domain (reference + deployment)
    # piecewise-linear fit. Push to firmware is DEFERRED to v12 — this is analysis-only.
    summary["adaptive_correction_shadow"] = _build_adaptive_correction_shadow(feat_df)

    readiness = _build_ml_readiness_verdict(summary)
    summary["ml_readiness_verdict"] = readiness
    summary["readiness_kind"] = readiness.get("readiness_kind")
    summary["schema_verdict"] = readiness.get("schema_verdict")
    summary["signal_verdict"] = readiness.get("signal_verdict")
    summary["reference_verdict"] = readiness.get("reference_verdict")
    summary["training_verdict"] = readiness.get("training_verdict")
    summary["readiness_headline"] = readiness.get("headline")

    golden_result = None
    if getattr(args, "golden_json", None):
        try:
            golden_result = _compare_to_golden(summary, args.golden_json,
                                               getattr(args, "golden_hr_rmse_tol", 0.25),
                                               getattr(args, "golden_hr_r_tol", 0.03))
            summary["golden_check"] = golden_result
            status = _green("PASS") if golden_result.get("passed") else _red("FAIL")
            print(_bold("== GOLDEN REGRESSION CHECK =="))
            print(f"Status: {status}")
            for chk in golden_result.get("checks", []):
                print(f"  {chk.get('metric')}: gold={chk.get('golden')} current={chk.get('current')} ok={chk.get('ok')}")
        except Exception as e:
            warn(f"Golden comparison failed: {e}")

    save_json(summary, os.path.join(args.out, "analyse_summary.json"))
    write_text_report(os.path.join(args.out, "analyse_report.txt"), summary)

    sections = [
        ("Baseline summary", json.dumps(nan_safe({
            "ml_gate": summary.get("ml_gate"),
            "session_quality_score": summary.get("session_quality_score"),
            "internal_consistency_score": summary.get("internal_consistency_score"),
            "review_required": summary.get("review_required"),
            "review_flags": summary.get("review_flags"),
            "pqi_lock_pct": summary.get("pqi_lock_pct"),
            "alignment_dt_s": summary.get("alignment_dt_s"),
            "hr_baseline": hr_base,
            "rr_baseline": rr_base,
            "raw_hr_bias_audit": raw_hr_bias_estimate,
            "ble_ref_quality": ble_ref_quality,
            "golden_check": golden_result,
        }), indent=2, allow_nan=False)),
    ]
    images = []
    for label, path in [
        ("HR Overlay", hr_overlay),
        ("RR Overlay", rr_overlay),
        ("Phase Signals", phase_png),
        ("HR Bland-Altman", hr_ba),
        ("RR Bland-Altman", rr_ba),
    ]:
        if os.path.exists(path):
            images.append((label, path))
    _write_simple_html_report(f"Radar Vital Trainer Analyse - {', '.join(summary['sessions'])}",
                              sections, images,
                              os.path.join(args.out, "analyse_report.html"))
    _write_session_manifest(session_root, args.radar, args.ref, args.out, fw_truthfulness)
    print(f"[OUT] Analysis outputs saved to {args.out}")
    return summary


def _run_loso_evaluation(base_df: pd.DataFrame, args, params, available_targets):
    participant_aware = _has_complete_study_metadata(base_df)
    group_column = "participant_id" if participant_aware else "session_id"
    group_ids = list(dict.fromkeys(base_df[group_column].tolist()))
    expected_groups = [str(value) for value in group_ids]
    if len(group_ids) < 3:
        return {
            "enabled": False,
            "reason": f"need at least 3 {group_column} groups",
            "group_column": group_column,
            "expected_groups": expected_groups,
            "completed_groups": [],
            "skipped_groups": expected_groups,
            "complete": False,
        }
    folds = []
    outer_oof_frames = []
    research_identity = _controlled_research_identity()
    completed_groups = []
    skipped_groups = []
    for i, holdout in enumerate(group_ids):
        train_source = base_df[base_df[group_column] != holdout].copy().reset_index(drop=True)
        eval_base = base_df[base_df[group_column] == holdout].copy().reset_index(drop=True)
        if train_source.empty or eval_base.empty:
            skipped_groups.append(str(holdout))
            folds.append({
                f"holdout_{group_column}": str(holdout),
                "skipped": True,
                "reason": "empty_training_or_holdout_partition",
            })
            continue
        fold_split = None
        if participant_aware:
            loo_train_base, loo_stop_base, _, fold_split = _participant_split(
                train_source,
                test_ratio=args.val_ratio,
                early_stop_ratio=args.early_stop_ratio,
                three_way=False,
                random_state=args.random_state + i,
                require_confirmatory=False,
            )
        else:
            loo_train_base, loo_stop_base = split_sessions(
                train_source,
                val_ratio=args.val_ratio,
                purge_gap_s=args.purge_gap_s,
            )
        loo_train_df = validity_masks(engineer_temporal_features(loo_train_base, feature_mode=args.feature_mode))
        loo_stop_df = validity_masks(engineer_temporal_features(loo_stop_base, feature_mode=args.feature_mode))
        loo_eval_df = validity_masks(engineer_temporal_features(eval_base, feature_mode=args.feature_mode))
        fold_feature_cols = pick_feature_columns(
            loo_train_df,
            feature_mode=args.feature_mode,
            max_nan_frac=args.max_nan_frac,
            min_variance=args.min_variance,
            allow_policy_features=bool(getattr(args, "allow_policy_features", False)),
        )
        if not fold_feature_cols:
            skipped_groups.append(str(holdout))
            folds.append({
                f"holdout_{group_column}": str(holdout),
                "skipped": True,
                "reason": "no numeric features selected from the fold training sessions",
            })
            continue
        assert_ml_feature_schema(
            fold_feature_cols,
            allow_policy_features=bool(getattr(args, "allow_policy_features", False)),
        )
        X_train, X_stop, fold_impute_values, fold_missing_flag_cols = prepare_feature_matrix(
            loo_train_df,
            loo_stop_df,
            fold_feature_cols,
        )
        fold_expanded_feature_cols = list(X_train.columns)
        X_eval = transform_feature_matrix(
            loo_eval_df,
            fold_feature_cols,
            fold_impute_values,
            fold_missing_flag_cols,
        )
        X_eval = X_eval.reindex(
            columns=fold_expanded_feature_cols,
            fill_value=0.0,
        ).astype(np.float32)
        model_hr = model_rr = None
        if "hr" in available_targets and loo_train_df.get("hr_valid_for_eval", pd.Series(False, index=loo_train_df.index)).fillna(False).sum() >= 20:
            model_hr, _ = fit_selected_target_model(
                loo_train_df, loo_stop_df, "hr", X_train, X_stop,
                params=params, args=args, random_state=args.random_state + i)
        if "rr" in available_targets and loo_train_df.get("rr_valid_for_eval", pd.Series(False, index=loo_train_df.index)).fillna(False).sum() >= 20:
            model_rr, _ = fit_selected_target_model(
                loo_train_df, loo_stop_df, "rr", X_train, X_stop,
                params=params, args=args, random_state=args.random_state + 100 + i)
        missing_targets = [
            target for target, model in (("hr", model_hr), ("rr", model_rr))
            if target in available_targets and model is None
        ]
        if missing_targets:
            skipped_groups.append(str(holdout))
            folds.append({
                f"holdout_{group_column}": str(holdout),
                "skipped": True,
                "reason": "insufficient_training_rows_for_target",
                "missing_targets": missing_targets,
            })
            continue
        pred = add_predictions(loo_eval_df, X_eval, model_hr=model_hr, model_rr=model_rr,
                               hr_slew_limit=args.slew_limit_hr_per_s, rr_slew_limit=args.slew_limit_rr_per_s)
        oof = pred.copy()
        oof["outer_fold"] = int(i)
        oof["outer_holdout_group"] = str(holdout)
        oof["model_family"] = str(getattr(args, "model_family", MODEL_FAMILY_GRADIENT_BOOSTING))
        oof["participant_disjoint"] = bool(participant_aware)
        oof["source_commit"] = _source_commit()
        for identity_field, identity_value in research_identity.items():
            oof[identity_field] = identity_value
        outer_oof_frames.append(oof)
        fold = {
            f"holdout_{group_column}": holdout,
            "participant_disjoint": bool(participant_aware),
        }
        if fold_split is not None:
            fold["inner_split_ledger_sha256"] = fold_split.get("sha256")
        if model_hr is not None:
            fold["hr_model"] = model_summary(pred, "hr")
        if model_rr is not None:
            fold["rr_model"] = model_summary(pred, "rr")
        folds.append(fold)
        completed_groups.append(str(holdout))
    out: Dict[str, Any] = {
        "enabled": True,
        "mode": (
            "leave_one_participant_out"
            if participant_aware
            else "leave_one_session_out"
        ),
        "group_column": group_column,
        "folds": folds,
        "expected_groups": expected_groups,
        "completed_groups": completed_groups,
        "skipped_groups": skipped_groups,
        "complete": bool(not skipped_groups and set(completed_groups) == set(expected_groups)),
    }
    output_root = getattr(args, "out", None)
    if outer_oof_frames and output_root:
        oof_columns = [
            "outer_fold", "outer_holdout_group", "model_family", "participant_disjoint",
            "participant_id", "session_id", "trial_id", "condition_id", "distance_m",
            "barrier_type", "timestamp_s", "ref_hr", "pred_hr_raw", "pred_hr",
            "hr_valid_for_eval", "ref_rr", "pred_rr_raw", "pred_rr", "rr_valid_for_eval",
            "confirmatory_eligible", "logical_trial_id", "attempt_id", "source_commit",
            "run_product_version", "analysis_plan_id", "analysis_plan_sha256",
            "study_protocol_id", "study_session_schema_version",
        ]
        oof = pd.concat(outer_oof_frames, ignore_index=True)
        for column in oof_columns:
            if column not in oof.columns:
                oof[column] = np.nan
        oof_path = os.path.join(str(output_root), "outer_oof_predictions.csv")
        oof.loc[:, oof_columns].to_csv(oof_path, index=False)
        out["outer_oof_predictions"] = {
            "path": os.path.abspath(oof_path),
            "rows": int(len(oof)),
            "sha256": _file_sha256(oof_path),
            "columns": oof_columns,
            "prediction_contract": {
                "raw_columns": ["pred_hr_raw", "pred_rr_raw"],
                "postprocessed_columns": ["pred_hr", "pred_rr"],
                "postprocessing": "causal_slew_and_range_clip",
            },
        }
    if folds:
        for target in ("hr", "rr"):
            rmses = []
            rs = []
            for f in folds:
                block = f.get(f"{target}_model")
                if isinstance(block, dict):
                    vo = block.get("valid_only", {})
                    if np.isfinite(float(vo.get("rmse", float("nan")))):
                        rmses.append(float(vo.get("rmse")))
                    if np.isfinite(float(vo.get("r", float("nan")))):
                        rs.append(float(vo.get("r")))
            out[f"{target}_mean_rmse"] = float(np.mean(rmses)) if rmses else float("nan")
            out[f"{target}_mean_r"] = float(np.mean(rs)) if rs else float("nan")
    return out


def _validate_confirmatory_loso(
    loo_eval: Mapping[str, object],
    *,
    required_targets: Sequence[str] = ("rr",),
) -> None:
    """Fail closed unless a confirmatory run produced complete participant LOSO OOF."""

    outer = loo_eval.get("outer_oof_predictions")
    failures = []
    if (
        not bool(loo_eval.get("enabled"))
        or loo_eval.get("mode") != "leave_one_participant_out"
        or not bool(loo_eval.get("complete"))
        or not isinstance(outer, Mapping)
    ):
        failures.append("complete participant-disjoint LOSO metadata")
    def _group_set(key: str) -> Set[str]:
        raw_groups = loo_eval.get(key)
        if not isinstance(raw_groups, Iterable) or isinstance(raw_groups, (str, bytes)):
            return set()
        return {str(value) for value in raw_groups}

    expected = _group_set("expected_groups")
    completed = _group_set("completed_groups")
    skipped = _group_set("skipped_groups")
    if not expected or expected != completed or skipped:
        failures.append("exact outer-fold coverage with no skipped groups")
    if isinstance(outer, Mapping):
        path_value = str(outer.get("path") or "").strip()
        path = Path(path_value).resolve() if path_value else None
        expected_hash = str(outer.get("sha256") or "").strip().lower()
        try:
            row_count = int(outer.get("rows") or 0)
        except (TypeError, ValueError):
            row_count = 0
        if path is None or not path.is_file() or not re.fullmatch(r"[0-9a-f]{64}", expected_hash) or _file_sha256(str(path)) != expected_hash or row_count < 1:
            failures.append("non-empty hashed outer_oof_predictions.csv")
        else:
            try:
                oof = pd.read_csv(path)
            except Exception as exc:
                failures.append(f"readable outer OOF artifact ({exc})")
            else:
                required_columns = {
                    "outer_holdout_group", "participant_id", "session_id", "trial_id",
                    "participant_disjoint", "confirmatory_eligible",
                }
                for target in required_targets:
                    required_columns.update({
                        f"ref_{target}", f"pred_{target}_raw", f"pred_{target}",
                        f"{target}_valid_for_eval",
                    })
                missing = sorted(required_columns - set(oof.columns))
                if missing:
                    failures.append("OOF columns: " + ", ".join(missing))
                else:
                    if len(oof) != row_count or oof.empty:
                        failures.append("OOF row count is positive and matches the manifest")
                    if not oof["participant_disjoint"].map(lambda value: value is True or str(value).strip().lower() == "true").all():
                        failures.append("all OOF rows are participant-disjoint")
                    if not oof["confirmatory_eligible"].map(lambda value: value is True or str(value).strip().lower() == "true").all():
                        failures.append("all OOF rows are confirmatory eligible")
                    for target in required_targets:
                        if not np.isfinite(pd.to_numeric(oof[f"ref_{target}"], errors="coerce")).any() or not np.isfinite(pd.to_numeric(oof[f"pred_{target}_raw"], errors="coerce")).any():
                            failures.append(f"OOF contains finite {target.upper()} reference and raw predictions")
                    if expected and set(oof["outer_holdout_group"].astype(str)) != expected:
                        failures.append("OOF holdout groups match the LOSO ledger")
    if failures:
        raise ValueError(
            "Confirmatory evaluation requires a complete participant-disjoint LOSO "
            "run with a valid outer OOF artifact: " + "; ".join(failures)
        )


def _controlled_analysis_plan() -> Dict[str, object]:
    """Load the controlled plan when present, preserving wheel compatibility."""

    plan_path = _REPO_ROOT / "quality" / "statistical-analysis-plan.json"
    if plan_path.is_file():
        loaded = json.loads(plan_path.read_text(encoding="utf-8"))
        if not isinstance(loaded, dict):
            raise ValueError("controlled statistical analysis plan must be a JSON object")
        return loaded
    return json.loads(json.dumps(DEFAULT_ANALYSIS_PLAN))


def _controlled_research_identity() -> Dict[str, str]:
    plan = _controlled_analysis_plan()
    identity = {
        "run_product_version": VERSION,
        "analysis_plan_id": str(plan.get("plan_id") or ""),
        "analysis_plan_sha256": analysis_plan_sha256(plan),
        "study_protocol_id": str(plan.get("study_protocol_id") or ""),
        "study_session_schema_version": str(plan.get("study_session_schema_version") or ""),
    }
    expected = {
        "analysis_plan_id": CONFIRMATORY_ANALYSIS_PLAN_ID,
        "study_protocol_id": STUDY_PROTOCOL_ID,
        "study_session_schema_version": STUDY_SESSION_SCHEMA_VERSION,
    }
    mismatched = [key for key, value in expected.items() if identity[key] != value]
    if mismatched:
        raise ValueError(
            "controlled research identity mismatch: " + ", ".join(mismatched)
        )
    return identity


def _write_confirmatory_run_manifest(
    args,
    *,
    split_info: Mapping[str, Any],
    loo_eval: Mapping[str, Any],
    available_targets: Sequence[str],
) -> Dict[str, object]:
    """Persist one auditable identity joining inputs, folds, OOF, and models."""

    radar_paths = [str(path) for path in getattr(args, "radar", [])]
    ref_paths = [str(path) for path in getattr(args, "ref", [])]
    input_hashes = {
        "radar": {os.path.abspath(path): _file_sha256(path) for path in radar_paths},
        "reference": {os.path.abspath(path): _file_sha256(path) for path in ref_paths},
    }
    outer_oof = loo_eval.get("outer_oof_predictions")
    outer_oof = outer_oof if isinstance(outer_oof, Mapping) else {}
    research_identity = _controlled_research_identity()
    manifest: Dict[str, object] = {
        "schema_version": CONFIRMATORY_RUN_MANIFEST_SCHEMA_VERSION,
        "status": "ready",
        **research_identity,
        "source_commit": _source_commit(),
        "analysis_job_id": str(os.environ.get("RVT_STUDY_ANALYSIS_JOB_ID") or "") or None,
        "model_family": str(getattr(args, "model_family", MODEL_FAMILY_GRADIENT_BOOSTING)),
        "targets": [str(target) for target in available_targets],
        # This is intentionally the predeclared, controlled analysis-plan
        # identity.  It is not changed merely because the product patch moved.
        "split_ledger_sha256": split_info.get("sha256"),
        "outer_oof_predictions": {
            "path": outer_oof.get("path"),
            "sha256": outer_oof.get("sha256"),
            "rows": outer_oof.get("rows"),
        },
        "input_file_sha256": input_hashes,
        "model_artifact_sha256": _model_artifact_hashes(str(getattr(args, "out", ""))),
        "folds": {
            "group_column": loo_eval.get("group_column"),
            "expected_groups": list(loo_eval.get("expected_groups") or []),
            "completed_groups": list(loo_eval.get("completed_groups") or []),
            "skipped_groups": list(loo_eval.get("skipped_groups") or []),
            "complete": bool(loo_eval.get("complete")),
        },
        "created_at": _iso_now(),
    }
    path = os.path.join(str(getattr(args, "out", "")), "confirmatory_run_manifest.json")
    save_json(manifest, path)
    manifest["path"] = os.path.abspath(path)
    manifest["sha256"] = _file_sha256(path)
    return manifest


def cmd_train(args):
    os.makedirs(args.out, exist_ok=True)
    progress_path = os.path.join(args.out, "training_progress.json")
    model_family = getattr(args, "model_family", MODEL_FAMILY_GRADIENT_BOOSTING)
    target_label = ",".join(getattr(args, "targets", ["hr", "rr"]))
    progress_units_per_target = (
        int(getattr(args, "cnn_epochs", 0) or 0)
        if model_family == MODEL_FAMILY_CNN_1D
        else int(getattr(args, "n_estimators", 0) or 0)
    )
    n_estimators_total = progress_units_per_target
    started_at = _iso_now()
    save_json({
        "schema_version": TRAINING_PROGRESS_SCHEMA_VERSION,
        "status": "running",
        "target": target_label,
        "n_estimators_done": 0,
        "n_estimators_total": n_estimators_total,
        "train_loss": None,
        "val_loss": None,
        "elapsed_s": 0.0,
        "started_at": started_at,
        "updated_at": started_at,
        "model_family": model_family,
        "progress_unit": "epochs" if model_family == MODEL_FAMILY_CNN_1D else "estimators",
    }, progress_path)
    train_started_t = time.monotonic()
    def _write_training_progress(status="running", n_done=0, train_loss=None, val_loss=None, **extra):
        payload = {
            "schema_version": TRAINING_PROGRESS_SCHEMA_VERSION,
            "status": status,
            "target": target_label,
            "n_estimators_done": int(max(0, n_done)),
            "n_estimators_total": int(max(0, n_estimators_total)),
            "train_loss": None if train_loss is None else float(train_loss),
            "val_loss": None if val_loss is None else float(val_loss),
            "elapsed_s": round(time.monotonic() - train_started_t, 3),
            "started_at": started_at,
            "updated_at": _iso_now(),
            "model_family": model_family,
            "progress_unit": "epochs" if model_family == MODEL_FAMILY_CNN_1D else "estimators",
        }
        payload.update(extra)
        save_json(payload, progress_path)

    _write_training_progress(status="running", n_done=0, phase="aligning")
    base_df = load_and_align_multiple_base(
        args.radar, args.ref,
        tolerance_s=args.tolerance_s, ref_offsets_s=args.ref_offset_s,
        auto_align_start=args.auto_align_start, merge_direction=args.merge_direction,
        phase_unit=args.phase_unit, heart_fft_window_s=args.heart_fft_window_s,
        breath_fft_window_s=args.breath_fft_window_s)
    _write_training_progress(status="running", n_done=0, phase="feature_engineering")

    require_confirmatory = bool(getattr(args, "confirmatory_evaluation", False))
    if require_confirmatory and not bool(getattr(args, "three_way_split", False)):
        raise ValueError(
            "Confirmatory evaluation requires --three-way-split so the held-out "
            "participant test partition remains separate from training and early stopping."
        )
    # A manuscript-confirmatory run is never allowed to silently fall back to
    # the pooled validation metrics.  LOSO is forced here even when an older
    # caller omitted --loo-eval; the run manifest below records that it was
    # required by the confirmatory contract.
    if require_confirmatory:
        args.loo_eval = True
    if _has_any_study_metadata(base_df) and not _has_complete_study_metadata(base_df):
        raise ValueError(
            "Study metadata is present for only part of the training input. "
            "Every radar file must resolve to a complete session_manifest.json."
        )
    if require_confirmatory:
        participant_count = int(base_df["participant_id"].astype(str).nunique()) if "participant_id" in base_df else 0
        if participant_count < 38:
            raise ValueError(
                "Confirmatory evaluation requires at least 38 independent participants "
                f"from the approved protocol; found {participant_count}."
            )
    if _has_complete_study_metadata(base_df):
        train_base, stop_base, eval_base, split_info = _participant_split(
            base_df,
            test_ratio=args.val_ratio,
            early_stop_ratio=args.early_stop_ratio,
            three_way=bool(args.three_way_split),
            random_state=args.random_state,
            require_confirmatory=require_confirmatory,
        )
        eval_name = "test" if args.three_way_split else "validation"
    elif require_confirmatory:
        raise ValueError(
            "Confirmatory evaluation requires complete participant, trial, condition, "
            "distance, barrier, duration, and eligibility metadata from each session manifest."
        )
    elif args.three_way_split:
        train_base, stop_base, eval_base, split_info = split_three_way(
            base_df, test_ratio=args.val_ratio,
            early_stop_ratio=args.early_stop_ratio, purge_gap_s=args.purge_gap_s)
        split_info["confirmatory"] = {
            "requested": False,
            "eligible": False,
            "reasons": ["participant-bound study metadata is unavailable"],
        }
        eval_name = "test"
    else:
        train_base, eval_base = split_sessions(base_df, val_ratio=args.val_ratio,
                                               purge_gap_s=args.purge_gap_s)
        stop_base  = eval_base
        split_info = {"mode": "two_way",
                      "sessions": list(dict.fromkeys(base_df["session_id"].tolist())),
                      "confirmatory": {
                          "requested": False,
                          "eligible": False,
                          "reasons": ["participant-bound study metadata is unavailable"],
                      }}
        eval_name  = "validation"
    save_json(split_info, os.path.join(args.out, "split_ledger.json"))

    train_df = validity_masks(engineer_temporal_features(train_base, feature_mode=args.feature_mode))
    stop_df  = validity_masks(engineer_temporal_features(stop_base,  feature_mode=args.feature_mode))
    eval_df  = validity_masks(engineer_temporal_features(eval_base,  feature_mode=args.feature_mode))

    hr_base_train = baseline_summary(train_df, "hr")
    rr_base_train = baseline_summary(train_df, "rr")
    hr_base_stop  = baseline_summary(stop_df,  "hr")
    rr_base_stop  = baseline_summary(stop_df,  "rr")
    hr_base_eval  = baseline_summary(eval_df,  "hr")
    rr_base_eval  = baseline_summary(eval_df,  "rr")

    gate_source = hr_base_stop if args.three_way_split else hr_base_eval
    enforce_baseline_gate(gate_source, args)

    requested_targets = list(dict.fromkeys(args.targets))
    available_targets = []
    skipped_targets = []
    for target in requested_targets:
        ref_col = f"ref_{target}"
        valid_col = f"{target}_valid_for_eval"
        has_ref = ref_col in train_df.columns and train_df[ref_col].notna().any()
        has_valid = valid_col in train_df.columns and bool(train_df[valid_col].fillna(False).sum() >= 20)
        if has_ref and has_valid:
            available_targets.append(target)
        else:
            skipped_targets.append(target)

    if not available_targets:
        raise ValueError(
            "No requested targets have enough valid training rows. "
            f"Requested={requested_targets}, skipped={skipped_targets}")
    if skipped_targets:
        warn(f"Skipping targets without enough valid reference rows: {skipped_targets}")
    n_estimators_total = progress_units_per_target * max(1, len(available_targets))
    _write_training_progress(status="running", n_done=0, phase="training_targets")

    feature_cols = pick_feature_columns(
        train_df, feature_mode=args.feature_mode,
        max_nan_frac=args.max_nan_frac, min_variance=args.min_variance,
        allow_policy_features=bool(getattr(args, "allow_policy_features", False)))
    if not feature_cols:
        raise ValueError("No numeric feature columns available for training.")
    assert_ml_feature_schema(
        feature_cols,
        allow_policy_features=bool(getattr(args, "allow_policy_features", False)),
    )

    X_train_all, X_stop_all, impute_values, missing_flag_cols = prepare_feature_matrix(
        train_df, stop_df, feature_cols)
    X_eval_all = transform_feature_matrix(eval_df, feature_cols, impute_values, missing_flag_cols)
    expanded_feature_cols = list(X_train_all.columns)
    assert_ml_feature_schema(
        expanded_feature_cols,
        allow_policy_features=bool(getattr(args, "allow_policy_features", False)),
    )
    X_eval_all = X_eval_all.reindex(columns=expanded_feature_cols, fill_value=0.0).astype(np.float32)

    valid_counts = {
        "hr": int(train_df["hr_valid_for_eval"].fillna(False).sum()),
        "rr": int(train_df["rr_valid_for_eval"].fillna(False).sum()),
    }
    for target in available_targets:
        if len(expanded_feature_cols) > max(1, valid_counts[target] // max(1, args.min_samples_per_feature)):
            msg = (f"Feature/sample ratio high for {target}: features={len(expanded_feature_cols)} | "
                   f"valid_rows={valid_counts[target]}")
            if args.strict_feature_ratio:
                raise ValueError(msg)
            warn(msg)

    params = resolve_model_params(args)
    model_hr = None
    model_rr = None
    hr_fit_meta = {"skipped": True}
    rr_fit_meta = {"skipped": True}
    n_estimators_done = 0

    if "hr" in available_targets:
        model_hr, hr_fit_meta = fit_selected_target_model(
            train_df, stop_df, "hr", X_train_all, X_stop_all,
            params=params, args=args, random_state=args.random_state)
        n_estimators_done += int(
            hr_fit_meta.get(
                "trained_epochs",
                hr_fit_meta.get("best_n_estimators", params.get("n_estimators", 0)),
            )
            or 0
        )
        _write_training_progress(status="running", n_done=n_estimators_done, phase="trained_hr", val_loss=hr_fit_meta.get("best_val_rmse"))
    if "rr" in available_targets:
        model_rr, rr_fit_meta = fit_selected_target_model(
            train_df, stop_df, "rr", X_train_all, X_stop_all,
            params=params, args=args, random_state=args.random_state + 1)
        n_estimators_done += int(
            rr_fit_meta.get(
                "trained_epochs",
                rr_fit_meta.get("best_n_estimators", params.get("n_estimators", 0)),
            )
            or 0
        )
        _write_training_progress(status="running", n_done=n_estimators_done, phase="trained_rr", val_loss=rr_fit_meta.get("best_val_rmse"))

    train_pred = add_predictions(train_df, X_train_all, model_hr=model_hr, model_rr=model_rr,
                                 hr_slew_limit=args.slew_limit_hr_per_s,
                                 rr_slew_limit=args.slew_limit_rr_per_s)
    stop_pred  = add_predictions(stop_df,  X_stop_all,  model_hr=model_hr, model_rr=model_rr,
                                 hr_slew_limit=args.slew_limit_hr_per_s,
                                 rr_slew_limit=args.slew_limit_rr_per_s)
    eval_pred  = add_predictions(eval_df,  X_eval_all,  model_hr=model_hr, model_rr=model_rr,
                                 hr_slew_limit=args.slew_limit_hr_per_s,
                                 rr_slew_limit=args.slew_limit_rr_per_s)

    hr_model_train = model_summary(train_pred, "hr") if model_hr is not None else None
    rr_model_train = model_summary(train_pred, "rr") if model_rr is not None else None
    hr_model_stop  = model_summary(stop_pred,  "hr") if model_hr is not None else None
    rr_model_stop  = model_summary(stop_pred,  "rr") if model_rr is not None else None
    hr_model_eval  = model_summary(eval_pred,  "hr") if model_hr is not None else None
    rr_model_eval  = model_summary(eval_pred,  "rr") if model_rr is not None else None

    print("\n== TRAIN SUMMARY ==")
    print_metric_block("HR baseline train", hr_base_train)
    if hr_model_train is not None:
        print_metric_block("HR model train", hr_model_train)
    else:
        print("\n[HR model train]\n  skipped")
    if rr_model_train is not None:
        print_metric_block("RR model train", rr_model_train)
    else:
        print("\n[RR model train]\n  skipped")
    print(f"\n== {'STOP' if args.three_way_split else 'VALIDATION'} SUMMARY ==")
    print_metric_block("HR baseline stop", hr_base_stop)
    if hr_model_stop is not None:
        print_metric_block("HR model stop", hr_model_stop)
    else:
        print("\n[HR model stop]\n  skipped")
    print_metric_block("RR baseline stop", rr_base_stop)
    if rr_model_stop is not None:
        print_metric_block("RR model stop", rr_model_stop)
    else:
        print("\n[RR model stop]\n  skipped")

    if args.three_way_split:
        print("\n== TEST SUMMARY ==")
        print_metric_block("HR baseline test", hr_base_eval)
        if hr_model_eval is not None:
            print_metric_block("HR model test", hr_model_eval)
        else:
            print("\n[HR model test]\n  skipped")
        print_metric_block("RR baseline test", rr_base_eval)
        if rr_model_eval is not None:
            print_metric_block("RR model test", rr_model_eval)
        else:
            print("\n[RR model test]\n  skipped")

    feature_manifest = {
        "model_family": model_family,
        "split_ledger_sha256": split_info.get("sha256"),
        "sequence": (
            resolve_cnn_config(args).as_dict()
            if model_family == MODEL_FAMILY_CNN_1D
            else None
        ),
        "cnn_training_policy": (
            {
                "minimum_valid_windows": int(args.cnn_min_valid_windows),
                "small_dataset_override": bool(args.allow_small_cnn_dataset),
                "status": "experimental",
            }
            if model_family == MODEL_FAMILY_CNN_1D
            else None
        ),
        "feature_mode": args.feature_mode,
        "feature_engineering_version": FEATURE_ENGINEERING_VERSION,
        "feature_schema_version": FEATURE_SCHEMA_VERSION,
        "feature_schema_hash": feature_schema_hash(),
        "allow_policy_features": bool(getattr(args, "allow_policy_features", False)),
        "policy_features_in_matrix": policy_feature_columns(expanded_feature_cols),
        "base_feature_count": len(feature_cols),
        "expanded_feature_count": len(expanded_feature_cols),
        "base_feature_cols": feature_cols,
        "expanded_feature_cols": expanded_feature_cols,
        "missing_flag_cols": missing_flag_cols,
        "requested_targets": requested_targets,
        "trained_targets": available_targets,
        "skipped_targets": skipped_targets,
    }
    save_json(feature_manifest, os.path.join(args.out, "feature_manifest.json"))
    if model_hr is not None:
        with open(os.path.join(args.out, "model_hr.pkl"), "wb") as f:
            pickle.dump(model_hr, f)
    if model_rr is not None:
        with open(os.path.join(args.out, "model_rr.pkl"), "wb") as f:
            pickle.dump(model_rr, f)
    with open(os.path.join(args.out, "preprocessor.pkl"), "wb") as f:
        pickle.dump({
            "version": VERSION, "feature_mode": args.feature_mode,
            "model_family": model_family,
            "split_ledger_sha256": split_info.get("sha256"),
            "sequence": (
                resolve_cnn_config(args).as_dict()
                if model_family == MODEL_FAMILY_CNN_1D
                else None
            ),
            "cnn_training_policy": (
                {
                    "minimum_valid_windows": int(args.cnn_min_valid_windows),
                    "small_dataset_override": bool(args.allow_small_cnn_dataset),
                    "status": "experimental",
                }
                if model_family == MODEL_FAMILY_CNN_1D
                else None
            ),
            "feature_engineering_version": FEATURE_ENGINEERING_VERSION,
            "feature_schema_version": FEATURE_SCHEMA_VERSION,
            "feature_schema_hash": feature_schema_hash(),
            "allow_policy_features": bool(getattr(args, "allow_policy_features", False)),
            "base_feature_cols": feature_cols, "expanded_feature_cols": expanded_feature_cols,
            "impute_values": impute_values, "missing_flag_cols": missing_flag_cols,
            "max_nan_frac": args.max_nan_frac, "min_variance": args.min_variance,
            "phase_unit": args.phase_unit,
            "heart_fft_window_s": args.heart_fft_window_s,
            "breath_fft_window_s": args.breath_fft_window_s,
            "slew_limit_hr_per_s": args.slew_limit_hr_per_s,
            "slew_limit_rr_per_s": args.slew_limit_rr_per_s,
            "trained_targets": available_targets,
        }, f)
    feature_contract = FeatureContract.create(
        base_feature_cols=feature_cols,
        expanded_feature_cols=expanded_feature_cols,
        feature_mode=args.feature_mode,
        feature_engineering_version=FEATURE_ENGINEERING_VERSION,
        feature_schema_version=FEATURE_SCHEMA_VERSION,
        feature_schema_hash=feature_schema_hash(),
        allow_policy_features=bool(getattr(args, "allow_policy_features", False)),
    )
    sequence_config = (
        resolve_cnn_config(args).as_dict()
        if model_family == MODEL_FAMILY_CNN_1D
        else None
    )
    bundle_metadata = build_model_bundle_metadata(
        trainer_version=VERSION,
        model_family=model_family,
        trained_targets=available_targets,
        feature_contract=feature_contract,
        training_config={
            "model_params": sequence_config if sequence_config is not None else params,
            "sequence": sequence_config,
            "sample_weight_mode": args.sample_weight_mode,
            "split_ledger_sha256": split_info.get("sha256"),
            "preprocessing": {
                "fit_partition": "train",
                "imputation": "training_partition_only",
                "missing_indicator_selection": "training_partition_only",
                "cnn_normalization": (
                    "training_partition_only"
                    if model_family == MODEL_FAMILY_CNN_1D
                    else None
                ),
                "max_nan_frac": args.max_nan_frac,
                "min_variance": args.min_variance,
            },
        },
        seed=args.random_state,
        artifact_hashes={
            name: digest
            for name, digest in _model_artifact_hashes(args.out).items()
            if name != MODEL_BUNDLE_METADATA_FILENAME
        },
        source_commit=_source_commit(),
    )
    save_json(
        bundle_metadata,
        os.path.join(args.out, MODEL_BUNDLE_METADATA_FILENAME),
    )
    # Integrity manifest: hashes the pickle artifacts and bundle metadata after
    # their exact bytes exist. _verify_model_dir checks this signed boundary
    # before any pickle.load at predict time.
    _write_model_manifest(args.out)
    if model_hr is not None and hasattr(model_hr, "feature_importances_"):
        save_feature_importance(model_hr, expanded_feature_cols,
                                os.path.join(args.out, "feature_importance_hr.csv"))
        save_feature_importance_plot(model_hr, expanded_feature_cols,
                                     os.path.join(args.out, "feature_importance_hr.png"))
    if model_rr is not None and hasattr(model_rr, "feature_importances_"):
        save_feature_importance(model_rr, expanded_feature_cols,
                                os.path.join(args.out, "feature_importance_rr.csv"))
        save_feature_importance_plot(model_rr, expanded_feature_cols,
                                     os.path.join(args.out, "feature_importance_rr.png"))
    train_pred.to_csv(os.path.join(args.out, "train_predictions.csv"), index=False)
    stop_pred.to_csv(os.path.join(args.out,
                                  f"{'stop' if args.three_way_split else 'validation'}_predictions.csv"),
                     index=False)
    eval_pred.to_csv(os.path.join(args.out, f"{eval_name}_predictions.csv"), index=False)
    if model_hr is not None:
        maybe_plot_timeseries(eval_pred, "hr",
                              os.path.join(args.out, f"{eval_name}_hr_overlay.png"),
                              pred_col="pred_hr", enabled=not args.no_plots)
        y_true_hr, y_pred_hr = get_valid_pair_arrays(eval_pred, "hr", "pred_hr", valid_only=True)
        maybe_plot_bland_altman(y_true_hr, y_pred_hr,
                                f"HR Bland-Altman (model vs ref, {eval_name}, valid-only)",
                                os.path.join(args.out, f"{eval_name}_hr_bland_altman.png"),
                                enabled=not args.no_plots, annotate=args.annotate_bland_altman)
    if model_rr is not None:
        maybe_plot_timeseries(eval_pred, "rr",
                              os.path.join(args.out, f"{eval_name}_rr_overlay.png"),
                              pred_col="pred_rr", enabled=not args.no_plots)
        y_true_rr, y_pred_rr = get_valid_pair_arrays(eval_pred, "rr", "pred_rr", valid_only=True)
        maybe_plot_bland_altman(y_true_rr, y_pred_rr,
                                f"RR Bland-Altman (model vs ref, {eval_name}, valid-only)",
                                os.path.join(args.out, f"{eval_name}_rr_bland_altman.png"),
                                enabled=not args.no_plots, annotate=args.annotate_bland_altman)

    if model_hr is not None and model_rr is not None:
        embedded_info = maybe_export_embedded(args, args.out, X_train_all, X_eval_all,
                                              train_pred, eval_pred, eval_target_name=eval_name)
    else:
        embedded_info = {
            "mode": "skipped",
            "reason": "embedded export currently requires both HR and RR models",
        }
        save_json(embedded_info, os.path.join(args.out, "embedded_export_summary.json"))

    loo_eval = _run_loso_evaluation(base_df, args, params, available_targets) \
        if getattr(args, "loo_eval", False) else {"enabled": False}
    if require_confirmatory:
        _validate_confirmatory_loso(loo_eval, required_targets=available_targets)

    confirmatory_manifest = (
        _write_confirmatory_run_manifest(
            args,
            split_info=split_info,
            loo_eval=loo_eval,
            available_targets=available_targets,
        )
        if require_confirmatory
        else None
    )

    summary = {
        "version": VERSION, "split": split_info,
        "model_family": model_family,
        "cnn_config": (
            resolve_cnn_config(args).as_dict()
            if model_family == MODEL_FAMILY_CNN_1D
            else None
        ),
        "cnn_training_policy": (
            {
                "minimum_valid_windows": int(args.cnn_min_valid_windows),
                "small_dataset_override": bool(args.allow_small_cnn_dataset),
                "status": "experimental",
            }
            if model_family == MODEL_FAMILY_CNN_1D
            else None
        ),
        "alignment": {
            "tolerance_s": args.tolerance_s, "merge_direction": args.merge_direction,
            "auto_align_start": bool(args.auto_align_start),
            "ref_offset_s": normalize_offsets(len(args.radar), args.ref_offset_s),
            "phase_unit": args.phase_unit,
            "heart_fft_window_s": args.heart_fft_window_s,
            "breath_fft_window_s": args.breath_fft_window_s,
        },
        "model_params": (
            resolve_cnn_config(args).as_dict()
            if model_family == MODEL_FAMILY_CNN_1D
            else params
        ),
        "feature_manifest": feature_manifest,
        "train_valid_counts": {
            "hr_valid_train_rows": valid_counts["hr"],
            "rr_valid_train_rows": valid_counts["rr"],
        },
        "sample_weight_mode": args.sample_weight_mode,
        "fit_meta": {"hr": hr_fit_meta, "rr": rr_fit_meta},
        "slew_limits": {"hr_per_s": args.slew_limit_hr_per_s, "rr_per_s": args.slew_limit_rr_per_s},
        "embedded_export": embedded_info,
        "loo_evaluation": loo_eval,
        "confirmatory_contract": {
            "requested": require_confirmatory,
            "loso_required": require_confirmatory,
            "loso_forced": bool(require_confirmatory),
            "outer_oof_required": require_confirmatory,
            "status": "ready" if require_confirmatory else "not_requested",
        },
        "confirmatory_run_manifest": confirmatory_manifest,
        "loo_eval": loo_eval,
        "train": {
            "hr_baseline": hr_base_train, "hr_model": hr_model_train,
            "rr_baseline": rr_base_train, "rr_model": rr_model_train,
        },
        "stop": {
            "hr_baseline": hr_base_stop, "hr_model": hr_model_stop,
            "rr_baseline": rr_base_stop, "rr_model": rr_model_stop,
        },
        eval_name: {
            "hr_baseline": hr_base_eval, "hr_model": hr_model_eval,
            "rr_baseline": rr_base_eval, "rr_model": rr_model_eval,
        },
    }
    save_json(summary, os.path.join(args.out, "train_summary.json"))
    hr_train_rmse = _to_num((hr_model_train or {}).get("valid_only", {}).get("rmse"), default=None) if isinstance(hr_model_train, dict) else None
    rr_train_rmse = _to_num((rr_model_train or {}).get("valid_only", {}).get("rmse"), default=None) if isinstance(rr_model_train, dict) else None
    hr_eval_rmse = _to_num((hr_model_eval or {}).get("valid_only", {}).get("rmse"), default=None) if isinstance(hr_model_eval, dict) else None
    rr_eval_rmse = _to_num((rr_model_eval or {}).get("valid_only", {}).get("rmse"), default=None) if isinstance(rr_model_eval, dict) else None
    train_losses = [x for x in (hr_train_rmse, rr_train_rmse) if x is not None]
    val_losses = [x for x in (hr_eval_rmse, rr_eval_rmse) if x is not None]
    _write_training_progress(
        status="complete",
        n_done=n_estimators_total,
        train_loss=(sum(train_losses) / len(train_losses)) if train_losses else None,
        val_loss=(sum(val_losses) / len(val_losses)) if val_losses else None,
        completed_at=_iso_now(),
        target=",".join(available_targets),
        phase="done",
    )
    write_text_report(os.path.join(args.out, "train_report.txt"), summary)
    print(f"\n[OUT] Training outputs saved to {args.out}")


def _alignment_provenance(args, offsets) -> Dict[str, object]:
    """Describe the applied sync method without overstating clock precision."""

    normalized_offsets = [float(value) for value in offsets]
    if bool(args.auto_align_start):
        method = "first_sample_exploratory"
    elif any(abs(value) > 0 for value in normalized_offsets):
        method = "operator_manual_offset"
    else:
        method = "nearest_receive_time_uncharacterized"
    return {
        "method": method,
        "source_clocks": ["radar_csv_time", "reference_csv_time"],
        "anchor_clock": "host_monotonic_receive_not_recorded_in_legacy_input",
        "applied_ref_offset_s": normalized_offsets,
        "uncertainty_s": float(args.tolerance_s),
        "uncertainty_basis": "merge_tolerance_upper_bound_not_physical_latency_uncertainty",
        "physical_latency_characterization": {
            "status": "missing",
            "evidence_ref": None,
            "sha256": None,
        },
        "confirmatory_timing_eligible": False,
        "limitations": [
            "ref_dt_s is a merge residual, not USB or reference-device latency uncertainty",
            "Objective 4 HR timing precision requires physical latency and jitter characterization",
        ],
    }


def cmd_align(args):
    """
    Sync a radar CSV to a reference CSV and write the merged output.

    This is the same alignment performed inside `analyse` and `train`, exposed
    as a standalone step so you can inspect the merged CSV before training.

    Outputs:
      <out>/aligned_merged.csv   - merged 1 Hz radar+ref rows
      <out>/alignment_report.txt - sync quality metrics
    """
    os.makedirs(args.out, exist_ok=True)

    if len(args.radar) != len(args.ref):
        sys.exit("[ALIGN] --radar and --ref must have the same number of files.")

    offsets = normalize_offsets(len(args.radar), args.ref_offset_s)
    alignment_provenance = _alignment_provenance(args, offsets)
    frames  = []
    for idx, (rp, refp, off) in enumerate(zip(args.radar, args.ref, offsets), start=1):
        sid    = f"session_{idx:02d}"
        merged = align_one_session(
            rp, refp, sid, args.tolerance_s, off,
            args.auto_align_start, args.merge_direction,
            args.phase_unit, args.heart_fft_window_s, args.breath_fft_window_s)
        frames.append(merged)
        ref_dt = merged["ref_dt_s"].dropna()
        print(f"[ALIGN] {sid}: {len(merged)} rows | "
              f"dt mean={ref_dt.mean():.3f}s  max_abs={ref_dt.abs().max():.3f}s")

        # Warn about poor alignment
        if ref_dt.abs().max() > 2.0:
            print(_yellow(f"[ALIGN] WARNING: max alignment gap {ref_dt.abs().max():.1f}s > 2s. "
                          f"Consider --ref-offset-s or --auto-align-start."))
        if len(merged) < 30:
            print(_yellow(f"[ALIGN] WARNING: only {len(merged)} paired rows. "
                          f"Session may be too short or timing offset too large."))

    combined = pd.concat(frames, ignore_index=True)
    out_csv  = os.path.join(args.out, "aligned_merged.csv")
    combined.to_csv(out_csv, index=False)

    report = {
        "schema_version": ALIGNMENT_REPORT_SCHEMA_VERSION,
        "version": VERSION, "n_sessions": len(args.radar),
        "total_paired_rows": int(len(combined)),
        "tolerance_s": args.tolerance_s, "merge_direction": args.merge_direction,
        "ref_offset_s": offsets, "auto_align_start": bool(args.auto_align_start),
        **alignment_provenance,
        "sessions": [],
    }
    for frame in frames:
        dt = frame["ref_dt_s"].dropna()
        report["sessions"].append({
            "session_id":  frame["session_id"].iloc[0],
            "n_rows":      int(len(frame)),
            "dt_mean_s":   float(dt.mean())       if len(dt) else float("nan"),
            "dt_max_abs_s": float(dt.abs().max()) if len(dt) else float("nan"),
        })

    save_json(report, os.path.join(args.out, "alignment_report.json"))
    save_json(
        {
            "schema_version": SYNC_ANCHORS_SCHEMA_VERSION,
            "status": "insufficient_for_confirmatory_timing",
            "method": alignment_provenance["method"],
            "clock": alignment_provenance["anchor_clock"],
            "anchors": [],
            "applied_ref_offset_s": alignment_provenance["applied_ref_offset_s"],
            "uncertainty_model": {
                "uncertainty_s": alignment_provenance["uncertainty_s"],
                "basis": alignment_provenance["uncertainty_basis"],
            },
            "source_commit": _source_commit(),
            "limitations": alignment_provenance["limitations"],
        },
        os.path.join(args.out, "sync_anchors.json"),
    )
    write_text_report(os.path.join(args.out, "alignment_report.txt"), report)
    print(f"\n[OUT] Merged CSV: {out_csv}")
    print(f"[OUT] Report:     {os.path.join(args.out, 'alignment_report.txt')}")
    print("\nNext step:")
    print(f"  python radar_vital_trainer.py analyse --radar {args.radar[0]} "
          f"--ref {args.ref[0]} --out <analysis_dir>")


def cmd_sweep(args):
    """
    Iterates a params_sweep.json, flashing and logging each variant.

    params_sweep.json format:
    [
      {"name": "baseline",    "params": {"RLS_LAMBDA": "0.97f"}, "duration_s": 920},
      {"name": "shorter_mem", "params": {"RLS_LAMBDA": "0.94f"}, "duration_s": 920}
    ]

    For each entry:
      1. Patches .ino constants and flashes firmware
      2. Waits for you to get into position (press Enter)
      3. Logs serial data for duration_s seconds
      4. Optionally runs analyse if a ref CSV is provided
    """
    _check_arduino_cli()

    if not os.path.exists(args.sweep_json):
        sys.exit(f"[SWEEP] Sweep file not found: {args.sweep_json}")

    experiments = json.load(open(args.sweep_json, encoding="utf-8"))
    if not isinstance(experiments, list) or len(experiments) == 0:
        sys.exit("[SWEEP] params_sweep.json must be a non-empty JSON array.")

    os.makedirs(args.out_dir, exist_ok=True)
    fqbn  = args.fqbn
    port  = args.port
    total = len(experiments)

    print(_bold(f"\n=== Sweep: {total} experiment(s) - {args.sweep_json} ===\n"))

    results = []

    for i, exp in enumerate(experiments, start=1):
        name       = exp.get("name", f"exp_{i:02d}")
        params     = exp.get("params", {})
        duration_s = int(exp.get("duration_s", args.default_duration_s))
        exp_dir    = os.path.join(args.out_dir, name)
        os.makedirs(exp_dir, exist_ok=True)

        print(_bold(f"\n[{i}/{total}] {name}"))
        print(f"  Params:   {params}")
        print(f"  Duration: {duration_s}s ({duration_s/60:.1f} min)")
        print(f"  Output:   {exp_dir}")

        # 1. Flash
        print("\n  Flashing firmware...")
        build_dir = os.path.join(exp_dir, "build")
        os.makedirs(build_dir, exist_ok=True)

        compile_sketch = _prepare_arduino_sketch_dir(args.sketch, build_dir, params)

        compile_cmd = ["arduino-cli", "compile", "--fqbn", fqbn,
                       "--build-path", os.path.join(build_dir, "build_out"), compile_sketch]
        result = subprocess.run(compile_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(_red(f"  Compile FAILED for {name}. Skipping."))
            print(result.stderr[-1000:])
            results.append({"name": name, "status": "compile_failed"})
            continue
        print(_green("  Compile OK"))

        upload_cmd = ["arduino-cli", "upload", "--fqbn", fqbn, "--port", port,
                      "--input-dir", os.path.join(build_dir, "build_out")]
        result = subprocess.run(upload_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(_red(f"  Upload FAILED for {name}. Skipping."))
            print(result.stderr[-500:])
            results.append({"name": name, "status": "upload_failed"})
            continue
        print(_green(f"  Upload OK -> {port}"))

        print(f"\n  Device booting ({args.boot_wait}s)...")
        time.sleep(args.boot_wait)

        # 2. Wait for user
        print(f"\n  {_bold('GET INTO POSITION:')} sit {args.distance_hint}m from the radar.")
        print(f"  Session will log for {duration_s}s (~{duration_s/60:.1f} min) once you press Enter.")
        if args.ref:
            print(f"  {_yellow('REMINDER:')} run  python radar_vital_trainer.py reflog  in a second terminal!")
        try:
            input("  Press Enter when ready... ")
        except KeyboardInterrupt:
            print(_yellow("\n  Skipped by user."))
            results.append({"name": name, "status": "skipped"})
            continue

        # 3. Log
        radar_csv = os.path.join(exp_dir, "radar.csv")
        print(f"\n  Logging to {radar_csv} ...")
        n_frames  = _log_serial_for_duration(port, duration_s, radar_csv)
        print(_green(f"  Session complete. {n_frames} frames saved."))

        # 4. Analyse (if ref provided)
        exp_result = {"name": name, "status": "logged", "n_frames": n_frames,
                      "radar_csv": radar_csv, "params": params}
        if args.ref and os.path.exists(args.ref):
            print(f"\n  Analysing against {args.ref} ...")
            try:
                merged = align_one_session(
                    radar_csv, args.ref, "session_01",
                    args.tolerance_s, 0.0, args.auto_align_start,
                    "nearest", "rad", 5.0, 10.0)
                feat_df  = validity_masks(engineer_temporal_features(merged, feature_mode="core"))
                feat_df.to_csv(os.path.join(exp_dir, "aligned_1hz_features.csv"), index=False)
                feat_df.to_csv(os.path.join(exp_dir, "aligned_1hz.csv"), index=False)
                hr_base  = baseline_summary(feat_df, "hr")
                rr_base  = baseline_summary(feat_df, "rr")
                r    = hr_base["valid_only"]["r"]
                rmse = hr_base["valid_only"]["rmse"]
                bias = hr_base["valid_only"]["bias"]
                gate = "PASS" if (np.isfinite(r) and r > 0.4 and
                                  np.isfinite(rmse) and rmse < 8.0) else "FAIL"
                exp_result.update({"status": "analysed", "hr_rmse": float(rmse),
                                   "hr_bias": float(bias), "hr_r": float(r), "ml_gate": gate})
                save_json({"version": VERSION, "params": params, "hr_baseline": hr_base, "rr_baseline": rr_base},
                          os.path.join(exp_dir, "analyse_summary.json"))
                gate_str = _green(gate) if gate == "PASS" else _red(gate)
                print(f"  HR: RMSE={rmse:.2f}  bias={bias:+.2f}  r={r:.3f}  gate={gate_str}")
            except Exception as e:
                warn(f"Analysis failed for {name}: {e}")
                exp_result["status"] = "analyse_failed"
        results.append(exp_result)

    # Summary table
    print(_bold(f"\n\n{'='*60}"))
    print(_bold(f"  SWEEP SUMMARY - {total} experiments"))
    print(f"{'='*60}")
    print(f"  {'Name':<20} {'Status':<15} {'RMSE':>6} {'Bias':>6} {'r':>6} {'Gate'}")
    print(f"  {'-'*56}")
    for r in results:
        rmse = r.get("hr_rmse", float("nan"))
        bias = r.get("hr_bias", float("nan"))
        rv   = r.get("hr_r",   float("nan"))
        gate = r.get("ml_gate", "-")
        gate_str = _green(gate) if gate == "PASS" else (_red(gate) if gate == "FAIL" else gate)
        rmse_str = f"{rmse:.2f}" if np.isfinite(rmse) else "-"
        bias_str = f"{bias:+.2f}" if np.isfinite(bias) else "-"
        r_str    = f"{rv:.3f}"   if np.isfinite(rv)   else "-"
        print(f"  {r.get('name','?'):<20} {r.get('status','?'):<15} "
              f"{rmse_str:>6} {bias_str:>6} {r_str:>6} {gate_str}")
    print(f"{'='*60}")

    save_json({"version": VERSION, "sweep_json": args.sweep_json,
               "results": results},
              os.path.join(args.out_dir, "sweep_results.json"))
    print(f"\n[OUT] Sweep results saved to {args.out_dir}")

    if args.ref:
        print("\nTo compare all sessions:")
        print(f"  python radar_vital_trainer.py compare --sessions-dir {args.out_dir} --out report.html")


def _metric_samples_from_aligned(path: Optional[str]) -> Optional[Dict[str, np.ndarray]]:
    if not path or not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path)
    except Exception:
        return None
    if not {"ref_hr", "reported_hr_mean"}.issubset(df.columns):
        return None
    mask = finite_pair_mask(df, "ref_hr", "reported_hr_mean")
    if "hr_valid_for_eval" in df.columns:
        mask = mask & df["hr_valid_for_eval"].fillna(False).to_numpy(dtype=bool)
    if not mask.any():
        return None
    ref = safe_series(df, "ref_hr").to_numpy(dtype=float)[mask]
    pred = safe_series(df, "reported_hr_mean").to_numpy(dtype=float)[mask]
    finite = np.isfinite(ref) & np.isfinite(pred)
    if finite.sum() == 0:
        return None
    return {"ref": ref[finite], "pred": pred[finite]}


def _sample_rmse_r(samples: Dict[str, np.ndarray], rng: np.random.Generator) -> Tuple[float, float]:
    ref = np.asarray(samples["ref"], dtype=float)
    pred = np.asarray(samples["pred"], dtype=float)
    n = len(ref)
    if n == 0:
        return float("nan"), float("nan")
    idx = rng.integers(0, n, size=n)
    ref_b = ref[idx]
    pred_b = pred[idx]
    return float(np.sqrt(np.mean((pred_b - ref_b) ** 2))), safe_corr(ref_b, pred_b)


def _bootstrap_delta_ci(
    baseline_samples: Optional[Dict[str, np.ndarray]],
    experiment_samples: Optional[Dict[str, np.ndarray]],
    n_boot: int,
    seed: int,
) -> Dict[str, object]:
    if baseline_samples is None or experiment_samples is None:
        return {"available": False, "reason": "aligned feature CSV missing or has no valid HR pairs"}
    rng = np.random.default_rng(int(seed))
    d_rmse = []
    d_r = []
    for _ in range(max(1, int(n_boot))):
        b_rmse, b_r = _sample_rmse_r(baseline_samples, rng)
        e_rmse, e_r = _sample_rmse_r(experiment_samples, rng)
        if np.isfinite(b_rmse) and np.isfinite(e_rmse):
            d_rmse.append(e_rmse - b_rmse)
        if np.isfinite(b_r) and np.isfinite(e_r):
            d_r.append(e_r - b_r)

    def block(vals):
        arr = np.asarray(vals, dtype=float)
        arr = arr[np.isfinite(arr)]
        if len(arr) == 0:
            return {"mean": float("nan"), "ci_low": float("nan"), "ci_high": float("nan"), "n_boot": 0}
        return {
            "mean": float(np.mean(arr)),
            "ci_low": float(np.percentile(arr, 2.5)),
            "ci_high": float(np.percentile(arr, 97.5)),
            "n_boot": int(len(arr)),
        }

    return {"available": True, "delta_rmse": block(d_rmse), "delta_r": block(d_r)}


def _format_ci(block: Optional[Dict[str, object]], fmt: str = ".2f") -> str:
    if not isinstance(block, dict):
        return "-"
    mean = _to_num(block.get("mean"), float("nan"))
    lo = _to_num(block.get("ci_low"), float("nan"))
    hi = _to_num(block.get("ci_high"), float("nan"))
    if not (np.isfinite(mean) and np.isfinite(lo) and np.isfinite(hi)):
        return "-"
    return f"{format(mean, fmt)} [{format(lo, fmt)}, {format(hi, fmt)}]"


def _load_sweep_param_map(sessions_dir: str) -> Dict[str, Dict[str, object]]:
    data = _read_json_if_exists(os.path.join(sessions_dir, "sweep_results.json"))
    if not isinstance(data, dict):
        return {}
    out: Dict[str, Dict[str, object]] = {}
    for row in data.get("results", []):
        if isinstance(row, dict) and row.get("name"):
            out[str(row["name"])] = dict(row.get("params", {}) or {})
    return out


def _build_sweep_delta_rows(rows: List[Dict], baseline_name: Optional[str], n_boot: int, seed: int) -> List[Dict[str, object]]:
    if not rows:
        return []
    baseline = None
    if baseline_name:
        baseline = next((r for r in rows if str(r.get("name")) == str(baseline_name)), None)
    if baseline is None:
        baseline = next((r for r in rows if "baseline" in str(r.get("name", "")).lower()), rows[0])
    base_params = baseline.get("params", {}) if isinstance(baseline.get("params"), dict) else {}
    base_samples = _metric_samples_from_aligned(baseline.get("aligned_features_path"))
    base_rmse = _to_num(baseline.get("hr_rmse"), float("nan"))
    base_r = _to_num(baseline.get("hr_r"), float("nan"))
    deltas: List[Dict[str, object]] = []
    for row in rows:
        if row is baseline:
            continue
        params = row.get("params", {}) if isinstance(row.get("params"), dict) else {}
        changed = []
        for key in sorted(set(base_params.keys()) | set(params.keys())):
            if base_params.get(key) != params.get(key):
                changed.append({"parameter": key, "baseline_value": base_params.get(key), "experiment_value": params.get(key)})
        if not changed and params:
            changed = [{"parameter": "(params)", "baseline_value": base_params, "experiment_value": params}]
        if not changed:
            changed = [{"parameter": "(session)", "baseline_value": baseline.get("name"), "experiment_value": row.get("name")}]
        exp_samples = _metric_samples_from_aligned(row.get("aligned_features_path"))
        boot = _bootstrap_delta_ci(base_samples, exp_samples, n_boot=n_boot, seed=seed + len(deltas) + 1)
        exp_rmse = _to_num(row.get("hr_rmse"), float("nan"))
        exp_r = _to_num(row.get("hr_r"), float("nan"))
        point_rmse = exp_rmse - base_rmse if np.isfinite(base_rmse) and np.isfinite(exp_rmse) else float("nan")
        point_r = exp_r - base_r if np.isfinite(base_r) and np.isfinite(exp_r) else float("nan")
        for change in changed:
            deltas.append({
                "baseline": baseline.get("name"),
                "experiment": row.get("name"),
                "parameter": change["parameter"],
                "baseline_value": change["baseline_value"],
                "experiment_value": change["experiment_value"],
                "delta_rmse_point": point_rmse,
                "delta_r_point": point_r,
                "bootstrap": boot,
            })
    return deltas


def _generate_compare_html(rows: List[Dict], out_path: str, delta_rows: Optional[List[Dict[str, object]]] = None):
    """Generates a self-contained HTML comparison report."""

    def cell_color(val, thresholds, colors, reverse=False):
        """Returns inline CSS color for a metric value."""
        if val is None:
            return ""
        try:
            f = float(val)
            if not np.isfinite(f):
                return ""
        except Exception:
            return ""
        if reverse:
            if f > thresholds[0]:
                return f"color:{colors[0]}"
            if f > thresholds[1]:
                return f"color:{colors[1]}"
            return f"color:{colors[2]}"
        else:
            if f < thresholds[0]:
                return f"color:{colors[0]}"
            if f < thresholds[1]:
                return f"color:{colors[1]}"
            return f"color:{colors[2]}"

    gate_counts = {"PASS": 0, "FAIL": 0, "-": 0}
    for r in rows:
        g = r.get("ml_gate", "-")
        gate_counts[g if g in gate_counts else "-"] += 1

    timestamp = _report_stamp()
    best_rmse = min((float(r["hr_rmse"]) for r in rows
                     if r.get("hr_rmse") and np.isfinite(float(r["hr_rmse"]))),
                    default=float("nan"))

    table_rows = ""
    for r in sorted(rows, key=lambda x: float(x.get("hr_rmse") or 999)):
        gate = r.get("ml_gate", "-")
        gate_style = ("color:#3b6d11;font-weight:500" if gate == "PASS"
                      else "color:#a32d2d;font-weight:500" if gate == "FAIL" else "color:#888")
        rmse = r.get("hr_rmse")
        r_v  = r.get("hr_r")
        bias = r.get("hr_bias")
        pqi  = r.get("pqi_mean")
        rr_rmse = r.get("rr_rmse")

        rmse_style  = cell_color(rmse, [5, 8, 99],  ["#3b6d11", "#854f0b", "#a32d2d"])
        r_style     = cell_color(r_v,  [0.4, 0.2, -1], ["#3b6d11", "#854f0b", "#a32d2d"], reverse=True)
        pqi_style   = cell_color(pqi,  [0.3, 0.15, -1], ["#3b6d11", "#854f0b", "#a32d2d"], reverse=True)
        rr_style    = cell_color(rr_rmse, [3, 5, 99], ["#3b6d11", "#854f0b", "#a32d2d"])

        table_rows += f"""
        <tr>
          <td style="font-weight:500">{r.get('name','-')}</td>
          <td>{r.get('firmware','-')}</td>
          <td>{r.get('distance','-')}</td>
          <td>{r.get('n_pairs','-')}</td>
          <td style="{rmse_style}">{_safe_float_val(rmse)}</td>
          <td>{_safe_float_val(bias, '+.2f')}</td>
          <td style="{r_style}">{_safe_float_val(r_v, '.3f')}</td>
          <td style="{rr_style}">{_safe_float_val(rr_rmse)}</td>
          <td style="{pqi_style}">{_safe_float_val(pqi, '.4f')}</td>
          <td style="{pqi_style}">{_safe_float_val(r.get('pqi_lock_pct'), '.1f')}%</td>
          <td style="{gate_style}">{gate}</td>
        </tr>"""

    delta_rows = delta_rows or []
    delta_table_rows = ""
    for d in delta_rows:
        boot = d.get("bootstrap", {}) if isinstance(d.get("bootstrap"), dict) else {}
        delta_table_rows += f"""
        <tr>
          <td>{d.get('baseline','-')}</td>
          <td>{d.get('experiment','-')}</td>
          <td>{d.get('parameter','-')}</td>
          <td>{html_escape(str(d.get('baseline_value','-')))}</td>
          <td>{html_escape(str(d.get('experiment_value','-')))}</td>
          <td>{_safe_float_val(d.get('delta_rmse_point'), '+.2f')}</td>
          <td>{_format_ci(boot.get('delta_rmse') if isinstance(boot, dict) else None, '+.2f')}</td>
          <td>{_safe_float_val(d.get('delta_r_point'), '+.3f')}</td>
          <td>{_format_ci(boot.get('delta_r') if isinstance(boot, dict) else None, '+.3f')}</td>
        </tr>"""
    delta_section = ""
    if delta_rows:
        delta_section = f"""
<p class="sec">Sweep deltas vs baseline with bootstrap 95% CIs</p>
<table>
  <thead>
    <tr>
      <th>Baseline</th><th>Experiment</th><th>Parameter</th><th>Baseline value</th><th>Experiment value</th>
      <th>Delta RMSE</th><th>Delta RMSE 95% CI</th><th>Delta r</th><th>Delta r 95% CI</th>
    </tr>
  </thead>
  <tbody>{delta_table_rows}</tbody>
</table>
"""

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Radar Vital Monitor - Session Comparison</title>
<style>
  body {{ font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
          background: #f9f9f7; color: #1a1a18; margin: 0; padding: 2rem; font-size: 14px; }}
  h1   {{ font-size: 20px; font-weight: 500; margin: 0 0 4px; }}
  .meta {{ color: #73726c; font-size: 13px; margin-bottom: 24px; }}
  .cards {{ display: grid; grid-template-columns: repeat(4, minmax(0,1fr)); gap: 10px; margin-bottom: 24px; }}
  .card {{ background: #f1efe8; border-radius: 8px; padding: 12px 14px; }}
  .card-label {{ font-size: 11px; color: #73726c; margin: 0 0 3px; }}
  .card-val {{ font-size: 20px; font-weight: 500; margin: 0; }}
  .card-sub {{ font-size: 11px; color: #73726c; margin: 2px 0 0; }}
  .sec {{ font-size: 11px; font-weight: 500; color: #73726c; letter-spacing: .06em;
          text-transform: uppercase; margin: 20px 0 8px; }}
  table {{ width: 100%; border-collapse: collapse; background: #fff;
           border: 0.5px solid #d3d1c7; border-radius: 8px; overflow: hidden; }}
  th {{ background: #f1efe8; font-size: 11px; font-weight: 500; color: #73726c;
        letter-spacing: .04em; padding: 8px 10px; text-align: left; border-bottom: 0.5px solid #d3d1c7; }}
  td {{ padding: 7px 10px; border-bottom: 0.5px solid #e8e6df; font-size: 13px; }}
  tr:last-child td {{ border-bottom: none; }}
  tr:hover td {{ background: #faf9f6; }}
  .legend {{ margin-top: 16px; font-size: 12px; color: #73726c; }}
  .dot {{ display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 4px; }}
  footer {{ margin-top: 32px; font-size: 11px; color: #b4b2a9; }}
</style>
</head>
<body>

<h1>Radar Vital Monitor - Session Comparison</h1>
<p class="meta">Generated: {timestamp} &nbsp;-&nbsp; Trainer v{VERSION} &nbsp;-&nbsp;
{len(rows)} session(s)</p>

<div class="cards">
  <div class="card">
    <p class="card-label">Sessions</p>
    <p class="card-val">{len(rows)}</p>
    <p class="card-sub">total analysed</p>
  </div>
  <div class="card">
    <p class="card-label">Best HR RMSE</p>
    <p class="card-val" style="color:{'#3b6d11' if best_rmse<8 else '#a32d2d'}">{
        _safe_float_val(best_rmse)} BPM</p>
    <p class="card-sub">{'gate range' if best_rmse<8 else 'above gate'}</p>
  </div>
  <div class="card">
    <p class="card-label">ML gate PASS</p>
    <p class="card-val" style="color:{'#3b6d11' if gate_counts['PASS']>0 else '#a32d2d'}">{
        gate_counts['PASS']}</p>
    <p class="card-sub">of {len(rows)} sessions</p>
  </div>
  <div class="card">
    <p class="card-label">Gate criteria</p>
    <p class="card-val" style="font-size:14px;padding-top:3px">r &gt; 0.4 &amp; RMSE &lt; 8</p>
    <p class="card-sub">HR valid-only</p>
  </div>
</div>

<p class="sec">All sessions - sorted by HR RMSE</p>
<table>
  <thead>
    <tr>
      <th>Session</th>
      <th>Firmware</th>
      <th>Distance</th>
      <th>n pairs</th>
      <th>HR RMSE</th>
      <th>HR bias</th>
      <th>Pearson r</th>
      <th>RR RMSE</th>
      <th>pqi_heart</th>
      <th>pqi lock %</th>
      <th>ML gate</th>
    </tr>
  </thead>
  <tbody>
    {table_rows}
  </tbody>
</table>

{delta_section}

<div class="legend">
  <span class="dot" style="background:#3b6d11"></span> AAMI target &nbsp;
  <span class="dot" style="background:#854f0b"></span> borderline &nbsp;
  <span class="dot" style="background:#a32d2d"></span> needs work &nbsp;&nbsp;
  HR RMSE targets: &lt;5 AAMI / &lt;8 ML gate &nbsp;&nbsp;
  RR RMSE target: &lt;3 br/min &nbsp;&nbsp;
  pqi lock: >=0.35
</div>

<footer>radar_vital_trainer_v{VERSION} &nbsp;-&nbsp;
XIAO ESP32-C6 + MR60BHA2 60 GHz FMCW &nbsp;-&nbsp; {timestamp}</footer>
</body>
</html>"""

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"[OUT] HTML report: {out_path}")


def cmd_compare(args):
    """
    Scans a directory of session folders and generates an HTML comparison report.

    Each session folder should contain:
      - analyse_summary.json   (written by `analyse`)  OR
      - radar.csv + ref.csv    (will run alignment+analysis on the fly)

    The folder name is used as the session name.
    """
    sessions_dir = args.sessions_dir
    if not os.path.isdir(sessions_dir):
        sys.exit(f"[COMPARE] Directory not found: {sessions_dir}")

    print(f"[COMPARE] Scanning {sessions_dir} ...")
    rows = []
    sweep_params = _load_sweep_param_map(sessions_dir)

    for entry in sorted(os.scandir(sessions_dir), key=lambda e: e.name):
        if not entry.is_dir():
            continue
        name = entry.name
        d    = entry.path

        # Try reading pre-computed analyse_summary.json first
        summary_path = _first_existing([
            os.path.join(d, "analyse_summary.json"),
            os.path.join(d, "analysis", "analyse_summary.json"),
        ])
        if summary_path and os.path.exists(summary_path):
            try:
                s = json.load(open(summary_path, encoding="utf-8"))
                hr = s.get("hr_baseline", {}).get("valid_only", {})
                rr = s.get("rr_baseline", {}).get("valid_only", {})

                # Try reading pqi from aligned CSV
                pqi_mean = pqi_lock = None
                aligned_csv = _first_existing([
                    os.path.join(d, "aligned_1hz_features.csv"),
                    os.path.join(d, "aligned_1hz.csv"),
                    os.path.join(d, "analysis", "aligned_1hz_features.csv"),
                    os.path.join(d, "analysis", "aligned_1hz.csv"),
                ])
                if aligned_csv and os.path.exists(aligned_csv):
                    try:
                        df = pd.read_csv(aligned_csv)
                        if "pqi_heart_mean" in df.columns:
                            still = df
                            pqi_mean = float(still["pqi_heart_mean"].mean())
                            pqi_lock = float(100 * (still["pqi_heart_mean"] >= 0.35).mean())
                    except Exception:
                        pass

                gate = s.get("ml_gate", {})
                gate_str = ("PASS" if gate.get("passed") else
                            "FAIL" if gate.get("passed") is False else "-")
                params_for_session = sweep_params.get(name, s.get("params", {}) if isinstance(s.get("params"), dict) else {})
                rows.append({
                    "name":        name,
                    "firmware":    s.get("sessions", ["?"])[0] if s.get("sessions") else "?",
                    "distance":    "-",
                    "n_pairs":     hr.get("n", "-"),
                    "hr_rmse":     hr.get("rmse"),
                    "hr_bias":     hr.get("bias"),
                    "hr_r":        hr.get("r"),
                    "rr_rmse":     rr.get("rmse"),
                    "pqi_mean":    pqi_mean,
                    "pqi_lock_pct": pqi_lock,
                    "ml_gate":     gate_str,
                    "params": params_for_session,
                    "aligned_features_path": aligned_csv,
                })
                print(f"  [OK]  {name}  (from analyse_summary.json)")
                continue
            except Exception as e:
                warn(f"  Could not read {summary_path}: {e}")

        # Fall back: find radar.csv + ref.csv and run on the fly
        radar_csv = None
        ref_csv   = None
        for fname in os.listdir(d):
            fl = fname.lower()
            if fl.endswith(".csv"):
                if "ref" in fl:
                    ref_csv = os.path.join(d, fname)
                elif "radar" in fl or "session" in fl or "run" in fl:
                    radar_csv = os.path.join(d, fname)

        if radar_csv and ref_csv:
            try:
                print(f"  [RUN] {name}  (analysing {os.path.basename(radar_csv)} + "
                      f"{os.path.basename(ref_csv)}) ...")
                merged  = align_one_session(radar_csv, ref_csv, "s01",
                                            0.9, 0.0, False, "nearest", "rad", 5.0, 10.0)
                feat_df = validity_masks(engineer_temporal_features(merged, feature_mode="core"))
                hr_base = baseline_summary(feat_df, "hr")
                rr_base = baseline_summary(feat_df, "rr")
                hr      = hr_base["valid_only"]
                rr      = rr_base["valid_only"]
                pqi_mean = float(feat_df["pqi_heart_mean"].mean()) \
                           if "pqi_heart_mean" in feat_df.columns else None
                pqi_lock = float(100 * (feat_df["pqi_heart_mean"] >= 0.35).mean()) \
                           if "pqi_heart_mean" in feat_df.columns else None
                gate = ("PASS" if (np.isfinite(hr.get("r", float("nan"))) and
                                   hr.get("r", 0) > 0.4 and
                                   np.isfinite(hr.get("rmse", float("nan"))) and
                                   hr.get("rmse", 99) < 8.0) else "FAIL")
                rows.append({
                    "name": name, "firmware": "?", "distance": "-",
                    "n_pairs": hr.get("n", "-"),
                    "hr_rmse": hr.get("rmse"), "hr_bias": hr.get("bias"),
                    "hr_r": hr.get("r"), "rr_rmse": rr.get("rmse"),
                    "pqi_mean": pqi_mean, "pqi_lock_pct": pqi_lock, "ml_gate": gate,
                    "params": sweep_params.get(name, {}),
                    "aligned_features_path": None,
                })
            except Exception as e:
                warn(f"  Analysis failed for {name}: {e}")
        else:
            print(f"  [SKIP] {name}  (no analyse_summary.json or radar+ref CSV pair found)")

    if not rows:
        print(_yellow("[COMPARE] No sessions found. Run `analyse` on each session first."))
        return

    print(f"\n[COMPARE] {len(rows)} session(s) found.")
    delta_rows = _build_sweep_delta_rows(
        rows,
        baseline_name=getattr(args, "baseline_name", None),
        n_boot=int(getattr(args, "bootstrap_n", 1000)),
        seed=int(getattr(args, "bootstrap_seed", 42)),
    )
    _generate_compare_html(rows, args.out, delta_rows=delta_rows)
    compare_json = os.path.splitext(args.out)[0] + ".json"
    save_json({
        "version": VERSION,
        "sessions_dir": os.path.abspath(sessions_dir),
        "rows": rows,
        "sweep_delta_rows": delta_rows,
        "bootstrap_n": int(getattr(args, "bootstrap_n", 1000)),
    }, compare_json)

    # Print quick text table
    print(f"\n{'Name':<22} {'RMSE':>6} {'Bias':>6} {'r':>6} {'Gate'}")
    print("-" * 52)
    for r in sorted(rows, key=lambda x: float(x.get("hr_rmse") or 999)):
        gate = r.get("ml_gate", "-")
        g    = _green(gate) if gate == "PASS" else _red(gate) if gate == "FAIL" else gate
        print(f"  {r.get('name','?'):<20} "
              f"{_safe_float_val(r.get('hr_rmse')):>6} "
              f"{_safe_float_val(r.get('hr_bias'), '+.2f'):>6} "
              f"{_safe_float_val(r.get('hr_r'), '.3f'):>6}  {g}")
    if delta_rows:
        print(f"\n{'Experiment':<20} {'Param':<18} {'dRMSE [95% CI]':>24} {'dr [95% CI]':>22}")
        print("-" * 90)
        for d in delta_rows:
            boot = d.get("bootstrap", {}) if isinstance(d.get("bootstrap"), dict) else {}
            print(f"  {str(d.get('experiment','?')):<18} {str(d.get('parameter','?')):<18} "
                  f"{_format_ci(boot.get('delta_rmse') if isinstance(boot, dict) else None, '+.2f'):>24} "
                  f"{_format_ci(boot.get('delta_r') if isinstance(boot, dict) else None, '+.3f'):>22}")

__all__ = [
    "BASE_FEATURES_CORE",
    "BASE_FEATURES_FULL",
    "Cnn1DConfig",
    "FEATURE_CATEGORY",
    "Keras1DCNNRegressor",
    "LABEL_FEATURES",
    "MODEL_FAMILIES",
    "MODEL_FAMILY_CNN_1D",
    "MODEL_FAMILY_GRADIENT_BOOSTING",
    "POLICY_FEATURE_EXACT",
    "POLICY_FEATURE_PREFIXES",
    "RADAR_SYNONYMS",
    "REF_SYNONYMS",
    "TEMPORAL_BASE_COLS_CORE",
    "TEMPORAL_BASE_COLS_FULL",
    "TEMPORAL_SUFFIXES_CORE",
    "TEMPORAL_SUFFIXES_FULL",
    "TRAINING_EXCLUDE_COLS",
    "add_predictions",
    "aggregate_radar_1hz",
    "aggregate_reference_1hz",
    "align_one_session",
    "align_reference_time",
    "apply_causal_slew_limit",
    "assert_ml_feature_schema",
    "build_causal_windows",
    "build_model",
    "build_tiny_surrogate",
    "canonicalize_with_synonyms",
    "choose_best_n_estimators",
    "chronological_split_single_session",
    "circular_mean",
    "circular_std",
    "cmd_align",
    "cmd_analyse",
    "cmd_compare",
    "cmd_predict",
    "cmd_sweep",
    "cmd_train",
    "enforce_baseline_gate",
    "engineer_temporal_features",
    "export_linear_embedded_header",
    "export_tflite_surrogate",
    "feature_category",
    "feature_schema_hash",
    "finite_phase_series_with_ts",
    "fit_cnn_target_model",
    "fit_linear_embedded_surrogate",
    "fit_selected_target_model",
    "fit_target_model",
    "get_sample_weights",
    "label_feature_columns",
    "last_valid",
    "load_and_aggregate_radar_only",
    "load_and_align_multiple_base",
    "load_radar",
    "load_reference",
    "lower_columns",
    "maybe_export_embedded",
    "normalize_offsets",
    "phase_band_features",
    "phase_to_radians",
    "pick_feature_columns",
    "policy_feature_columns",
    "prepare_feature_matrix",
    "resolve_cnn_config",
    "resolve_model_params",
    "save_feature_importance_plot",
    "spectral_entropy",
    "split_sessions",
    "split_three_way",
    "transform_feature_matrix",
    "truncate_gbr_model",
    "unwrapped_excursion",
]
