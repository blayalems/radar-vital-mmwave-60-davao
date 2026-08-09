import json

import numpy as np
import pandas as pd
import pytest

from rvt_trainer.statistics import (
    StatisticalInputError,
    DEFAULT_ANALYSIS_PLAN,
    aggregate_confirmatory_frame,
    analyze_frame,
    coverage_report,
    exact_proportion,
    holm_adjust,
    paired_metrics,
    paired_tost,
    repeated_measures_agreement,
    write_statistical_outputs,
    _condition_tost,
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
                # 21 endpoints at 5-second spacing span 100 seconds and
                # therefore yield exactly 15 valid 30-second windows.
                for window in range(21):
                    reference = 20.0 + participant_index
                    rows.append(
                        {
                            "participant_id": participant,
                            "trial_id": f"{participant}-{condition}-T{trial + 1}",
                            "condition_id": condition,
                            "session_id": f"S-{participant}-{condition}-T{trial + 1}",
                            "timestamp_s": float(window * 5),
                            "ref_rr": reference,
                            "pred_rr": reference + 0.2,
                            "rr_valid_for_eval": True,
                            "confirmatory_eligible": True,
                            "participant_disjoint": True,
                            "model_family": "gradient_boosting",
                            "outer_fold": participant_index,
                            "outer_holdout_group": participant,
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
    plan = json.loads(json.dumps(DEFAULT_ANALYSIS_PLAN))
    plan["status"] = "approved"
    ledger = pd.DataFrame([
        {
            "participant_id": participant,
            "trial_id": f"{participant}-{condition}-T{trial + 1}",
            "condition_id": condition,
            "attempt_type": "subject",
            "eligible": True,
        }
        for participant_index in range(3)
        for participant in [f"P-{participant_index + 1:03d}"]
        for condition in conditions
        for trial in range(2)
    ] + [
        {
            "participant_id": "NO-SUBJECT",
            "trial_id": f"NO-SUBJECT-{index + 1:03d}",
            "condition_id": "no_subject",
            "attempt_type": "no_subject",
            "eligible": True,
        }
        for index in range(72)
    ])
    report = analyze_frame(
        frame,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        participant_column="participant_id",
        bootstrap_reps=200,
        analysis_plan=plan,
        provenance={
            "source_commit": "c" * 40,
            "model_family": "gradient_boosting",
            "split_ledger_sha256": "a" * 64,
            "prediction_file_sha256": "b" * 64,
            "product_version": "16.5.8",
            "protocol_id": "RVT-STA-PLAN-16.5.8",
        },
        attempt_ledger=ledger,
        confirmatory=True,
    )
    assert report["aggregation"]["minimum_valid_windows_per_trial"] == 15
    assert report["coverage"]["denominator_unit"] == "trial"
    assert report["coverage"]["n_attempted"] == 108
    assert report["coverage"]["n_with_output"] == 36
    assert report["condition_tost"]["primary"]["status"].startswith("inconclusive")
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


def test_confirmatory_aggregation_rejects_short_endpoint_bursts():
    frame = pd.DataFrame({
        "participant_id": ["P-001"] * 30,
        "trial_id": ["T-001"] * 30,
        "condition_id": ["d100_none"] * 30,
        "session_id": ["S-001"] * 30,
        "timestamp_s": np.arange(30, dtype=float),
        "ref_rr": [20.0] * 30,
        "pred_rr": [20.1] * 30,
    })
    summary, diagnostics = aggregate_confirmatory_frame(
        frame,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        participant_column="participant_id",
    )
    assert summary.empty
    assert diagnostics["validated_window_count"] == 0


def test_confirmatory_requires_approved_plan_and_provenance():
    frame = pd.DataFrame({
        "participant_id": ["P-001", "P-002"],
        "session_id": ["S-001", "S-002"],
        "trial_id": ["T-001", "T-002"],
        "condition_id": ["d100_none", "d100_none"],
        "timestamp_s": [0.0, 0.0],
        "ref_rr": [20.0, 20.0],
        "pred_rr": [20.0, 20.0],
        "rr_valid_for_eval": [True, True],
        "confirmatory_eligible": [True, True],
        "participant_disjoint": [True, True],
        "model_family": ["gradient_boosting", "gradient_boosting"],
    })
    with pytest.raises(StatisticalInputError, match="explicit approved"):
        analyze_frame(
            frame,
            reference_column="ref_rr",
            estimate_column="pred_rr",
            confirmatory=True,
        )


def _small_confirmatory_inputs():
    plan = json.loads(json.dumps(DEFAULT_ANALYSIS_PLAN))
    plan["status"] = "approved"
    frame = pd.DataFrame(
        {
            "participant_id": ["P-001", "P-002", "P-003"],
            "session_id": ["S-001", "S-002", "S-003"],
            "trial_id": ["T-001", "T-002", "T-003"],
            "condition_id": ["d100_none", "d100_none", "d100_none"],
            "timestamp_s": [0.0, 0.0, 0.0],
            "ref_rr": [20.0, 20.0, 20.0],
            "pred_rr": [20.1, 20.1, 20.1],
            "rr_valid_for_eval": [True, True, True],
            "confirmatory_eligible": [True, True, True],
            "participant_disjoint": [True, True, True],
            "model_family": ["gradient_boosting", "gradient_boosting", "gradient_boosting"],
            "outer_fold": [0, 1, 2],
            "outer_holdout_group": ["P-001", "P-002", "P-003"],
        }
    )
    ledger = pd.DataFrame(
        [
            {
                "participant_id": "P-001",
                "trial_id": "T-001",
                "condition_id": "d100_none",
                "attempt_type": "subject",
                "eligible": True,
            },
            {
                "participant_id": "P-002",
                "trial_id": "T-002",
                "condition_id": "d100_none",
                "attempt_type": "subject",
                "eligible": True,
            },
            {
                "participant_id": "P-003",
                "trial_id": "T-003",
                "condition_id": "d100_none",
                "attempt_type": "subject",
                "eligible": True,
            },
        ]
        + [
            {
                "participant_id": "NO-SUBJECT",
                "trial_id": f"NS-{index + 1:03d}",
                "condition_id": "no_subject",
                "attempt_type": "no_subject",
                "eligible": True,
            }
            for index in range(72)
        ]
    )
    provenance = {
        "source_commit": "c" * 40,
        "model_family": "gradient_boosting",
        "split_ledger_sha256": "a" * 64,
        "prediction_file_sha256": "b" * 64,
        "product_version": "16.5.8",
        "protocol_id": "RVT-STA-PLAN-16.5.8",
    }
    return frame, ledger, plan, provenance


def _primary_vs_secondary_confirmatory_inputs(participant_count=19):
    rows = []
    ledger_rows = []
    condition_biases = {"d100_none": 3.0, "d060_none": 0.0}
    for participant_index in range(participant_count):
        participant = f"P-{participant_index + 1:03d}"
        for condition, bias in condition_biases.items():
            for trial_number in (1, 2):
                trial_id = f"{participant}-{condition}-T{trial_number}"
                session_id = f"S-{trial_id}"
                ledger_rows.append({
                    "participant_id": participant,
                    "trial_id": trial_id,
                    "condition_id": condition,
                    "attempt_type": "subject",
                    "eligible": True,
                })
                for endpoint in range(21):
                    rows.append({
                        "participant_id": participant,
                        "session_id": session_id,
                        "trial_id": trial_id,
                        "condition_id": condition,
                        "timestamp_s": float(endpoint * 5),
                        "ref_rr": 20.0 + participant_index / 100.0,
                        "pred_rr": 20.0 + participant_index / 100.0 + bias,
                        "rr_valid_for_eval": True,
                        "confirmatory_eligible": True,
                        "participant_disjoint": True,
                        "model_family": "gradient_boosting",
                        "outer_fold": participant_index,
                        "outer_holdout_group": participant,
                    })
    ledger_rows.extend({
        "participant_id": "NO-SUBJECT",
        "trial_id": f"NS-{index + 1:03d}",
        "condition_id": "no_subject",
        "attempt_type": "no_subject",
        "eligible": True,
    } for index in range(72))
    plan = json.loads(json.dumps(DEFAULT_ANALYSIS_PLAN))
    plan["status"] = "approved"
    provenance = {
        "source_commit": "c" * 40,
        "model_family": "gradient_boosting",
        "split_ledger_sha256": "a" * 64,
        "prediction_file_sha256": "b" * 64,
        "product_version": "16.5.8",
        "protocol_id": "RVT-STA-PLAN-16.5.8",
    }
    return pd.DataFrame(rows), pd.DataFrame(ledger_rows), plan, provenance


def test_confirmatory_flags_are_strict_and_aggregation_excludes_ineligible_rows():
    frame, ledger, plan, provenance = _small_confirmatory_inputs()
    frame["confirmatory_eligible"] = frame["confirmatory_eligible"].astype(object)
    frame.loc[0, "confirmatory_eligible"] = "false"
    report = analyze_frame(
        frame,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        analysis_plan=plan,
        provenance=provenance,
        attempt_ledger=ledger,
        confirmatory=True,
    )
    assert report["exclusions"]["ineligible"] == 1
    assert report["denominators"]["n_participants"] == 2

    poisoned = pd.concat(
        [
            frame.assign(
                confirmatory_eligible=True,
                rr_valid_for_eval=True,
                participant_disjoint=True,
            ),
            frame.assign(
                participant_id="P-POISON",
                confirmatory_eligible="false",
                rr_valid_for_eval=True,
                participant_disjoint=True,
            ),
        ],
        ignore_index=True,
    )
    summary, _ = aggregate_confirmatory_frame(
        poisoned,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        participant_column="participant_id",
        require_eligibility=True,
    )
    assert "P-POISON" not in set(summary.get("participant_id", []))


def test_confirmatory_filters_invalid_rr_endpoints_before_all_metrics():
    frame, ledger, plan, provenance = _small_confirmatory_inputs()
    frame.loc[0, "rr_valid_for_eval"] = False
    frame.loc[0, "pred_rr"] = 999.0

    report = analyze_frame(
        frame,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        analysis_plan=plan,
        provenance=provenance,
        attempt_ledger=ledger,
        confirmatory=True,
    )

    assert report["exclusions"]["invalid_rr"] == 1
    assert report["metrics"]["n_pairs"] == 2
    assert report["metrics"]["estimate_mean"] == pytest.approx(20.1)


def test_confirmatory_rejects_non_disjoint_eligible_oof_rows():
    frame, ledger, plan, provenance = _small_confirmatory_inputs()
    frame.loc[0, "participant_disjoint"] = False

    with pytest.raises(StatisticalInputError, match="participant-disjoint"):
        analyze_frame(
            frame,
            reference_column="ref_rr",
            estimate_column="pred_rr",
            analysis_plan=plan,
            provenance=provenance,
            attempt_ledger=ledger,
            confirmatory=True,
        )


def test_confirmatory_rejects_relabelled_oof_holdout_group():
    frame, ledger, plan, provenance = _small_confirmatory_inputs()
    frame.loc[0, "outer_holdout_group"] = "P-999"

    with pytest.raises(StatisticalInputError, match="holdout group must equal"):
        analyze_frame(
            frame,
            reference_column="ref_rr",
            estimate_column="pred_rr",
            analysis_plan=plan,
            provenance=provenance,
            attempt_ledger=ledger,
            confirmatory=True,
        )


def test_confirmatory_rejects_uncontrolled_participant_identity():
    frame, ledger, plan, provenance = _small_confirmatory_inputs()
    frame.loc[0, "participant_id"] = "subject-1"
    frame.loc[0, "outer_holdout_group"] = "subject-1"

    with pytest.raises(StatisticalInputError, match="P-NNN"):
        analyze_frame(
            frame,
            reference_column="ref_rr",
            estimate_column="pred_rr",
            analysis_plan=plan,
            provenance=provenance,
            attempt_ledger=ledger,
            confirmatory=True,
        )


def test_confirmatory_accepts_canonical_raw_rr_but_rejects_lookalike_columns():
    frame, ledger, plan, provenance = _small_confirmatory_inputs()
    raw_frame = frame.rename(columns={"pred_rr": "pred_rr_raw"})
    report = analyze_frame(
        raw_frame,
        reference_column="ref_rr",
        estimate_column="pred_rr_raw",
        analysis_plan=plan,
        provenance=provenance,
        attempt_ledger=ledger,
        confirmatory=True,
    )
    assert report["estimate_column"] == "pred_rr_raw"

    lookalike = frame.assign(uncontrolled_rr=frame["pred_rr"])
    with pytest.raises(StatisticalInputError, match="pred_rr or pred_rr_raw"):
        analyze_frame(
            lookalike,
            reference_column="ref_rr",
            estimate_column="uncontrolled_rr",
            analysis_plan=plan,
            provenance=provenance,
            attempt_ledger=ledger,
            confirmatory=True,
        )


def test_confirmatory_rejects_uncontrolled_plan_identity():
    frame, ledger, plan, provenance = _small_confirmatory_inputs()
    plan["plan_id"] = "RVT-STA-PLAN-HACK"
    provenance["protocol_id"] = plan["plan_id"]

    with pytest.raises(StatisticalInputError, match="plan_id"):
        analyze_frame(
            frame,
            reference_column="ref_rr",
            estimate_column="pred_rr",
            analysis_plan=plan,
            provenance=provenance,
            attempt_ledger=ledger,
            confirmatory=True,
        )


def test_confirmatory_report_uses_only_primary_condition_tost(tmp_path):
    frame, ledger, plan, provenance = _primary_vs_secondary_confirmatory_inputs()
    report = analyze_frame(
        frame,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        analysis_plan=plan,
        provenance=provenance,
        attempt_ledger=ledger,
        bootstrap_reps=200,
        confirmatory=True,
    )

    primary = report["condition_tost"]["primary"]
    assert primary["condition_id"] == "d100_none"
    assert primary["n_participants"] == 19
    assert primary["mean_difference"] == pytest.approx(3.0)
    assert primary["decision"] == "not_equivalent"
    assert report["tost"] == primary

    outputs = write_statistical_outputs(
        report,
        tmp_path / "report.json",
        csv_path=tmp_path / "report.csv",
    )
    flat = pd.read_csv(outputs["csv"])
    primary_decision = flat.loc[
        flat["metric"] == "d100_none.decision", "value"
    ].tolist()
    assert primary_decision == ["not_equivalent"]


def test_confirmatory_provenance_rejects_relabelled_model_family():
    frame, ledger, plan, provenance = _small_confirmatory_inputs()
    provenance["model_family"] = "cnn_1d"
    with pytest.raises(StatisticalInputError, match="model_family"):
        analyze_frame(
            frame,
            reference_column="ref_rr",
            estimate_column="pred_rr",
            analysis_plan=plan,
            provenance=provenance,
            attempt_ledger=ledger,
            confirmatory=True,
        )


def test_no_subject_rows_are_retained_for_false_alarm_analysis():
    frame = pd.DataFrame(
        {
            "participant_id": ["P-001", "NO-SUBJECT"],
            "trial_id": ["T-001", "NS-001"],
            "condition_id": ["d100_none", "no_subject"],
            "ref_rr": [20.0, np.nan],
            "pred_rr": [20.1, np.nan],
            "false_alarm": [False, True],
        }
    )
    ledger = pd.DataFrame(
        [
            {
                "participant_id": "P-001",
                "trial_id": "T-001",
                "condition_id": "d100_none",
                "attempt_type": "subject",
                "eligible": True,
            },
            {
                "participant_id": "NO-SUBJECT",
                "trial_id": "NS-001",
                "condition_id": "no_subject",
                "attempt_type": "no_subject",
                "eligible": True,
            },
        ]
    )
    report = coverage_report(
        frame,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        false_alarm_column="false_alarm",
        attempt_ledger=ledger,
        minimum_valid_windows_per_trial=15,
    )
    assert report["no_subject_denominator"] == 1
    assert report["no_subject_false_alarms"] == 1
    assert report["false_alarm_exact"]["alternative"] == "less"


def test_condition_tost_primary_is_inconclusive_below_predeclared_n():
    rows = []
    for index in range(3):
        participant = f"P-{index + 1:03d}"
        for condition in ["d100_none", "d060_none", "d080_none", "d060_cardboard", "d080_cardboard", "d100_cardboard"]:
            rows.append({
                "participant_id": participant,
                "condition_id": condition,
                "ref_rr": 20.0,
                "pred_rr": 20.1,
            })
    result = _condition_tost(
        pd.DataFrame(rows),
        reference_column="ref_rr",
        estimate_column="pred_rr",
        participant_column="participant_id",
        margin=2.0,
        alpha=0.05,
        analysis_plan=DEFAULT_ANALYSIS_PLAN,
    )
    assert result["primary"]["status"] == "inconclusive_insufficient_independent_estimates"
    assert result["primary"]["minimum_required"] == 19
    assert all(item["decision_holm"] == "inconclusive" for item in result["secondary"])


def test_coverage_uses_attempt_ledger_for_no_subject_denominator():
    frame = pd.DataFrame({
        "participant_id": ["P-001", "NO-SUBJECT-001"],
        "trial_id": ["T-001", "NS-001"],
        "condition_id": ["d100_none", "no_subject"],
        "ref_rr": [20.0, np.nan],
        "pred_rr": [20.0, 1.0],
        "false_alarm": [False, True],
    })
    ledger = pd.DataFrame([
        {"participant_id": "P-001", "trial_id": "T-001", "condition_id": "d100_none", "attempt_type": "subject", "eligible": True},
        {"participant_id": "NO-SUBJECT-001", "trial_id": "NS-001", "condition_id": "no_subject", "attempt_type": "no_subject", "eligible": True},
        {"participant_id": "NO-SUBJECT-002", "trial_id": "NS-002", "condition_id": "no_subject", "attempt_type": "no_subject", "eligible": True},
    ])
    result = coverage_report(
        frame,
        reference_column="ref_rr",
        estimate_column="pred_rr",
        false_alarm_column="false_alarm",
        attempt_ledger=ledger,
    )
    assert result["n_attempted"] == 3
    assert result["n_with_output"] == 2
    assert result["no_subject_denominator"] == 2
    assert result["no_subject_false_alarms"] == 1
    assert result["false_alarm_exact"]["alternative"] == "less"


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
