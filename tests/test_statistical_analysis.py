import json

import numpy as np
import pandas as pd
import pytest

from rvt_trainer.statistics import (
    StatisticalInputError,
    aggregate_confirmatory_frame,
    analyze_frame,
    exact_proportion,
    holm_adjust,
    paired_metrics,
    paired_tost,
    repeated_measures_agreement,
    write_statistical_outputs,
)


def test_paired_metrics_are_oriented_estimate_minus_reference():
    result = paired_metrics([10, 12, 14], [11, 10, 15])
    assert result["n_pairs"] == 3
    assert result["bias_estimate_minus_reference"] == pytest.approx(0.0)
    assert result["mae"] == pytest.approx(4 / 3)
    assert result["rmse"] == pytest.approx(np.sqrt(2))


def test_paired_tost_declares_equivalence_only_inside_predeclared_margin():
    equivalent = paired_tost(
        [100, 101, 99, 100, 102, 98],
        [100.2, 100.7, 99.1, 100.1, 101.4, 98.5],
        lower=-2.0,
        upper=2.0,
    )
    not_equivalent = paired_tost(
        [100, 100, 100, 100, 100, 100],
        [104, 104, 104, 104, 104, 104],
        lower=-2.0,
        upper=2.0,
    )
    assert equivalent["equivalent"] is True
    assert equivalent["ci_low"] > -2.0
    assert equivalent["ci_high"] < 2.0
    assert not_equivalent["equivalent"] is False
    assert not_equivalent["mean_difference"] == pytest.approx(4.0)


def test_exact_proportion_uses_clopper_pearson_interval():
    result = exact_proportion(8, 10)
    assert result["method"] == "clopper_pearson_exact"
    assert result["proportion"] == pytest.approx(0.8)
    assert 0.0 < result["ci_low"] < result["proportion"] < result["ci_high"] < 1.0
    one_sided = exact_proportion(1, 10, null_proportion=0.05, alternative="less")
    assert one_sided["alternative"] == "less"
    assert one_sided["ci_low"] < one_sided["proportion"] < one_sided["ci_high"]


def test_holm_adjust_is_monotone_and_preserves_input_order():
    assert holm_adjust([0.01, 0.04, 0.2]) == pytest.approx([0.03, 0.08, 0.2])


def test_repeated_measures_agreement_clusters_bootstrap_by_participant():
    reference = [10, 10, 20, 20, 30, 30]
    estimate = [11, 9, 21, 19, 29, 31]
    participants = ["P-001", "P-001", "P-002", "P-002", "P-003", "P-003"]
    result = repeated_measures_agreement(
        reference, estimate, participants, bootstrap_reps=250, seed=123
    )
    repeated = repeated_measures_agreement(
        reference, estimate, participants, bootstrap_reps=250, seed=123
    )
    assert result["method"] == "repeated_measures_clustered_bland_altman"
    assert result["n_pairs"] == 6
    assert result["n_participants"] == 3
    assert result["bias"] == pytest.approx(0.0)
    assert result["loa_lower"] < result["loa_upper"]
    assert result["bias_ci_low"] == pytest.approx(repeated["bias_ci_low"])
    assert result["loa_upper_ci_high"] == pytest.approx(repeated["loa_upper_ci_high"])


def test_analyze_frame_emits_tost_and_agreement_policy():
    frame = pd.DataFrame(
        {
            "ref_hr": [60, 61, 70, 71, 80, 81],
            "pred_hr": [60.2, 60.8, 70.1, 71.2, 79.9, 81.1],
            "participant_id": ["P-001", "P-001", "P-002", "P-002", "P-003", "P-003"],
        }
    )
    report = analyze_frame(
        frame,
        reference_column="ref_hr",
        estimate_column="pred_hr",
        tost_margin=2.0,
        bootstrap_reps=250,
        seed=42,
    )
    assert report["analysis_schema"] == "rvt-statistical-report-v1"
    assert report["tost"]["lower_bound"] == -2.0
    assert report["agreement"]["n_participants"] == 3
    assert report["analysis_policy"]["participant_clustered"] is True
    assert report["coverage"]["n_attempted"] == 6
    assert report["denominators"]["n_participants"] == 3


def test_confirmatory_aggregation_enforces_windows_and_trials_and_holm_conditions(tmp_path):
    rows = []
    conditions = ["d100_none", "d060_none", "d080_none", "d060_cardboard", "d080_cardboard", "d100_cardboard"]
    for participant_index in range(3):
        participant = f"P-{participant_index + 1:03d}"
        for condition in conditions:
            for trial in range(2):
                for window in range(15):
                    reference = 20.0 + participant_index
                    rows.append(
                        {
                            "participant_id": participant,
                            "trial_id": f"{participant}-{condition}-T{trial + 1}",
                            "condition_id": condition,
                            "ref_rr": reference,
                            "pred_rr": reference + 0.2,
                        }
                    )
    frame = pd.DataFrame(rows)
    summary, diagnostics = aggregate_confirmatory_frame(
        frame,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        participant_column="participant_id",
    )
    assert diagnostics["eligible_trial_count"] == 36
    assert diagnostics["eligible_participant_condition_count"] == 18
    assert len(summary) == 18
    report = analyze_frame(
        frame,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        participant_column="participant_id",
        tost_margin=2.0,
        bootstrap_reps=200,
        confirmatory=True,
    )
    assert report["aggregation"]["minimum_valid_windows_per_trial"] == 15
    assert report["coverage"]["denominator_unit"] == "trial"
    assert report["coverage"]["n_attempted"] == 36
    assert report["condition_tost"]["primary"]["status"] == "tested"
    assert len(report["condition_tost"]["secondary"]) == 5
    assert report["condition_effects"]["status"] in {"unavailable_optional_dependency", "tested", "fit_failed"}
    outputs = write_statistical_outputs(
        report,
        tmp_path / "report.json",
        csv_path=tmp_path / "report.csv",
        latex_path=tmp_path / "report.tex",
    )
    assert outputs["json"].exists() and outputs["csv"].exists() and outputs["latex"].exists()
    saved = json.loads(outputs["json"].read_text(encoding="utf-8"))
    assert len(saved["report_sha256"]) == 64


@pytest.mark.parametrize(
    "call",
    [
        lambda: paired_metrics([1], [1]),
        lambda: paired_tost([1, 2], [1, 2], lower=1, upper=-1),
        lambda: exact_proportion(3, 2),
        lambda: repeated_measures_agreement([1, 2], [1, 2], ["P-001", "P-001"]),
    ],
)
def test_statistical_tools_fail_closed_on_invalid_designs(call):
    with pytest.raises(StatisticalInputError):
        call()
