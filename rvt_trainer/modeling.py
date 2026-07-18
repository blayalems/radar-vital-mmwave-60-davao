"""Model-family helpers for the Radar Vital trainer.

Gradient boosting remains the lightweight default.  The optional 1-D CNN path
uses causal windows that never cross session boundaries and imports TensorFlow
only when the CNN family is selected.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np


MODEL_FAMILY_GRADIENT_BOOSTING = "gradient_boosting"
MODEL_FAMILY_CNN_1D = "cnn_1d"
MODEL_FAMILIES = (MODEL_FAMILY_GRADIENT_BOOSTING, MODEL_FAMILY_CNN_1D)


def build_causal_windows(
    features: np.ndarray,
    session_ids: Sequence[object],
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
        self._model = None

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
        tf.keras.utils.set_random_seed(self.random_state)
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
