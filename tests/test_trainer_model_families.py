import builtins
import inspect

import numpy as np
import pandas as pd
import pytest

from rvt_trainer.modeling import (
    Cnn1DConfig,
    Keras1DCNNRegressor,
    MODEL_FAMILY_CNN_1D,
    MODEL_FAMILY_GRADIENT_BOOSTING,
    build_causal_windows,
)
from rvt_trainer.monolith import (
    _controlled_analysis_plan,
    _controlled_research_identity,
    _run_loso_evaluation,
    _validate_confirmatory_loso,
    apply_causal_slew_limit,
    build_parser,
    fit_cnn_target_model,
)
from rvt_trainer.statistics import DEFAULT_ANALYSIS_PLAN, analysis_plan_sha256


def test_causal_windows_are_left_padded_and_session_bounded():
    features = np.asarray([[1.0], [2.0], [3.0], [10.0], [11.0]], dtype=np.float32)
    windows, endpoints = build_causal_windows(
        features,
        session_ids=["a", "a", "a", "b", "b"],
        window_size=3,
    )

    assert endpoints.tolist() == [0, 1, 2, 3, 4]
    assert windows[:, :, 0].tolist() == [
        [1.0, 1.0, 1.0],
        [1.0, 1.0, 2.0],
        [1.0, 2.0, 3.0],
        [10.0, 10.0, 10.0],
        [10.0, 10.0, 11.0],
    ]


def test_causal_windows_only_materialize_selected_endpoints():
    features = np.arange(12, dtype=np.float32).reshape(6, 2)
    windows, endpoints = build_causal_windows(
        features,
        session_ids=["s"] * 6,
        window_size=3,
        endpoint_mask=[False, True, False, False, True, False],
    )

    assert endpoints.tolist() == [1, 4]
    assert windows.shape == (2, 3, 2)
    np.testing.assert_array_equal(windows[1], features[2:5])


def test_causal_slew_limit_is_prefix_invariant():
    prefix = pd.DataFrame({
        "session_id": ["s", "s"],
        "timestamp_s": [0.0, 1.0],
        "prediction": [10.0, 100.0],
    })
    extended = pd.concat(
        [prefix, pd.DataFrame({
            "session_id": ["s"],
            "timestamp_s": [2.0],
            "prediction": [100.0],
        })],
        ignore_index=True,
    )
    prefix_out = apply_causal_slew_limit(prefix, "prediction", 1.0)
    extended_out = apply_causal_slew_limit(extended, "prediction", 1.0)
    np.testing.assert_array_equal(
        prefix_out["prediction"].to_numpy(),
        extended_out["prediction"].to_numpy()[: len(prefix)],
    )
    np.testing.assert_allclose(
        extended_out["prediction"].to_numpy(),
        [10.0, 11.0, 12.0],
    )


@pytest.mark.parametrize("window_size", [0, 1])
def test_cnn_config_rejects_non_temporal_windows(window_size):
    with pytest.raises(ValueError, match="window_size"):
        Cnn1DConfig(window_size=window_size).validate()


def test_train_cli_keeps_gradient_boosting_as_default():
    args = build_parser().parse_args(
        ["train", "--radar", "radar.csv", "--ref", "ref.csv"]
    )
    assert args.model_family == MODEL_FAMILY_GRADIENT_BOOSTING


def test_train_cli_accepts_1d_cnn_as_explicit_option():
    args = build_parser().parse_args(
        [
            "train",
            "--radar",
            "radar.csv",
            "--ref",
            "ref.csv",
            "--model-family",
            MODEL_FAMILY_CNN_1D,
            "--cnn-window-size",
            "48",
        ]
    )
    assert args.model_family == MODEL_FAMILY_CNN_1D
    assert args.cnn_window_size == 48


def test_train_cli_exposes_fail_closed_confirmatory_evaluation():
    args = build_parser().parse_args(
        [
            "train",
            "--radar",
            "radar.csv",
            "--ref",
            "ref.csv",
            "--three-way-split",
            "--confirmatory-evaluation",
        ]
    )
    assert args.model_family == MODEL_FAMILY_GRADIENT_BOOSTING
    assert args.three_way_split is True
    assert args.confirmatory_evaluation is True


def test_controlled_research_identity_keeps_plan_and_run_versions_independent():
    plan = _controlled_analysis_plan()
    identity = _controlled_research_identity()

    assert plan == DEFAULT_ANALYSIS_PLAN
    assert plan["effective_product_version"] == "16.5.8"
    assert identity["run_product_version"] == "16.6.4"
    assert identity["analysis_plan_id"] == plan["plan_id"]
    assert identity["analysis_plan_sha256"] == analysis_plan_sha256(plan)
    assert identity["study_protocol_id"] == plan["study_protocol_id"]
    assert identity["study_session_schema_version"] == plan["study_session_schema_version"]


def test_confirmatory_loso_validator_rejects_incomplete_or_non_participant_runs(tmp_path):
    with pytest.raises(ValueError, match="complete participant-disjoint LOSO"):
        _validate_confirmatory_loso({
            "enabled": True,
            "mode": "leave_one_participant_out",
            "complete": False,
            "outer_oof_predictions": {"path": "missing.csv"},
        })

    with pytest.raises(ValueError, match="complete participant-disjoint LOSO"):
        _validate_confirmatory_loso({
            "enabled": True,
            "mode": "leave_one_session_out",
            "complete": True,
            "outer_oof_predictions": {"path": "session-oof.csv"},
        })

    oof = tmp_path / "outer_oof_predictions.csv"
    import hashlib
    oof.write_text(
        "outer_holdout_group,participant_id,session_id,trial_id,participant_disjoint,confirmatory_eligible,ref_rr,pred_rr_raw,pred_rr,rr_valid_for_eval\n"
        "P-001,P-001,s01,t1,true,true,15,15,15,true\n",
        encoding="utf-8",
    )
    _validate_confirmatory_loso({
        "enabled": True,
        "mode": "leave_one_participant_out",
        "complete": True,
        "expected_groups": ["P-001"],
        "completed_groups": ["P-001"],
        "skipped_groups": [],
        "outer_oof_predictions": {
            "path": str(oof),
            "rows": 1,
            "sha256": hashlib.sha256(oof.read_bytes()).hexdigest(),
        },
    })


def test_cnn_dependency_error_is_actionable(monkeypatch):
    original_import = builtins.__import__

    def reject_tensorflow(name, *args, **kwargs):
        if name == "tensorflow":
            raise ModuleNotFoundError("tensorflow unavailable in unit test")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_tensorflow)
    with pytest.raises(RuntimeError, match="pip install tensorflow"):
        Keras1DCNNRegressor._tensorflow()


def test_cnn_normalization_is_fitted_on_training_windows_only():
    model = Keras1DCNNRegressor(Cnn1DConfig(window_size=2, kernel_size=2))
    train = np.asarray(
        [[[0.0, 10.0], [2.0, 14.0]], [[4.0, 18.0], [6.0, 22.0]]],
        dtype=np.float32,
    )
    validation = np.full((1, 2, 2), 1_000_000.0, dtype=np.float32)

    normalized_train = model._normalize_fit(train)
    normalized_validation = model._normalize(validation)

    np.testing.assert_allclose(model.feature_mean_, [3.0, 16.0])
    np.testing.assert_allclose(normalized_train.mean(axis=(0, 1)), [0.0, 0.0])
    assert np.all(normalized_validation > 100_000)


def test_cnn_refuses_starved_dataset_before_loading_tensorflow():
    rows = 30
    frame = pd.DataFrame(
        {
            "session_id": ["s"] * rows,
            "hr_valid_for_eval": [True] * rows,
            "ref_hr": np.linspace(70.0, 75.0, rows),
            "pqi_heart_mean": [0.8] * rows,
        }
    )
    features = pd.DataFrame({"feature": np.linspace(0.0, 1.0, rows)})

    with pytest.raises(ValueError, match="Fix upstream publish coverage"):
        fit_cnn_target_model(
            frame,
            frame,
            "hr",
            features,
            features,
            Cnn1DConfig(window_size=8),
            min_valid_windows=500,
        )


def test_loso_preprocessing_is_fit_inside_each_training_fold():
    source = inspect.getsource(_run_loso_evaluation)
    assert "pick_feature_columns(" in source
    assert "prepare_feature_matrix(" in source
    assert "fold_impute_values" in source
    assert "fold_missing_flag_cols" in source


def test_loso_retains_outer_oof_raw_and_postprocessed_prediction_contract():
    source = inspect.getsource(_run_loso_evaluation)
    assert "outer_oof_predictions.csv" in source
    assert "pred_hr_raw" in source and "pred_rr_raw" in source
    assert "prediction_contract" in source
