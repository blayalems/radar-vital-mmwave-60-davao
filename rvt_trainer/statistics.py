"""Reproducible statistical analysis for Radar Vital validation.

The analysis unit is an aligned prediction/reference pair.  Participant IDs
are treated as the cluster for repeated-measures agreement and bootstrap
resampling; callers must not interpret row counts as independent participants.
SciPy is imported lazily so the GBR trainer remains usable without the
statistical extras until a report is requested.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

import numpy as np


# This mirrors the controlled JSON plan shape.  It is useful for descriptive
# previews, but confirmatory callers must provide an explicit approved copy.
DEFAULT_ANALYSIS_PLAN = {
    "$schema": "./schemas/statistical-analysis-plan.schema.json",
    "schema_version": "rvt-analysis-plan-v1",
    "plan_id": "RVT-STA-PLAN-16.5.8",
    "effective_product_version": "16.5.8",
    "status": "draft",
    "owner_role": "research_lead",
    "approval_required": ["research_lead", "quality_manager"],
    "protocol": {
        "confirmatory_distances_m": [0.6, 0.8, 1.0],
        "barriers": ["none", "cardboard"],
        "trials_per_condition": 3,
        "planned_duration_s": 150,
        "window_duration_s": 30,
        "window_stride_s": 5,
        "minimum_valid_windows_per_trial": 15,
        "minimum_valid_trials_per_participant_condition": 2,
        "target_recruited_participants": 40,
        "minimum_protocol_complete_participants": 38,
        "no_subject_trial_count": 72,
    },
    "primary": {
        "condition_id": "d100_none",
        "target": "rr",
        "margin_bpm": 2.0,
        "alpha": 0.05,
        "confidence_level": 0.90,
        "minimum_independent_estimates": 19,
    },
    "secondary": {
        "condition_ids": [
            "d060_none",
            "d080_none",
            "d060_cardboard",
            "d080_cardboard",
            "d100_cardboard",
        ],
        "multiplicity_adjustment": "holm",
    },
    "aggregation": {
        "input": "outer_oof_predictions.csv",
        "unit": "participant_condition",
        "sequence": [
            "30-second windows at 5-second stride",
            "median of at least 15 valid windows to trial summary",
            "mean of at least 2 valid trials to participant-condition summary",
        ],
        "participant_balanced": True,
    },
    "reports": {
        "retain_raw_predictions": True,
        "retain_raw_and_postprocessed_columns": True,
        "metrics": ["rmse", "mae", "bias", "pearson_r", "spearman_rho"],
        "agreement": "repeated_measures_bland_altman_with_participant_bootstrap",
        "coverage_denominator": "every_eligible_protocol_attempted_trial_including_non_output",
        "false_alarm_test": "exact_binomial_vs_0.05_with_separate_two_sided_95_percent_clopper_pearson_interval",
        "exports": ["json", "csv", "latex"],
        "provenance": ["source_commit", "model_family", "split_ledger_sha256", "prediction_file_sha256"],
    },
    "exclusions": [
        "legacy_unassigned",
        "participant_reassigned",
        "outside_confirmatory_condition_set",
        "missing_release_or_protocol_provenance",
    ],
    "interpretation": {
        "equivalence_requires_both_one_sided_tost_tests": True,
        "non_significant_difference_is_not_equivalence": True,
        "fewer_than_minimum_independent_estimates": "inconclusive",
        "cnn_deployment_claim": "not_permitted_without_separate_tinyml_hardware_acceptance",
    },
}


class StatisticalInputError(ValueError):
    """Raised when a statistical contract is incomplete or invalid."""


def _validate_analysis_plan(
    plan: Mapping[str, Any],
    *,
    require_approved: bool = False,
) -> dict[str, Any]:
    """Validate the frozen plan without making ``jsonschema`` mandatory.

    The trainer's lightweight runtime intentionally does not depend on a JSON
    schema package.  These exact-value checks mirror the controlled schema and
    fail closed when a caller attempts a confirmatory run with a draft or
    altered protocol.
    """

    if not isinstance(plan, Mapping):
        raise StatisticalInputError("analysis_plan must be a JSON object")
    normalized = dict(plan)
    if not str(normalized.get("$schema", "")).strip():
        raise StatisticalInputError("analysis_plan $schema is required")
    if normalized.get("schema_version") != "rvt-analysis-plan-v1":
        raise StatisticalInputError("analysis_plan schema_version is unsupported")
    if not re.fullmatch(
        r"RVT-STA-PLAN-[0-9]+\.[0-9]+\.[0-9]+",
        str(normalized.get("plan_id", "")),
    ):
        raise StatisticalInputError("analysis_plan plan_id is invalid")
    if not re.fullmatch(
        r"[0-9]+\.[0-9]+\.[0-9]+",
        str(normalized.get("effective_product_version", "")),
    ):
        raise StatisticalInputError("analysis_plan effective_product_version is invalid")
    if normalized.get("owner_role") != "research_lead":
        raise StatisticalInputError("analysis_plan owner_role must be research_lead")
    approvals = normalized.get("approval_required")
    if approvals != ["research_lead", "quality_manager"]:
        raise StatisticalInputError(
            "analysis_plan approval_required must name research_lead and quality_manager"
        )
    if normalized.get("status") not in {"draft", "approved", "superseded"}:
        raise StatisticalInputError("analysis_plan status is invalid")
    if require_approved and normalized.get("status") != "approved":
        raise StatisticalInputError(
            "confirmatory analysis requires an approved statistical analysis plan"
        )
    protocol = normalized.get("protocol")
    primary = normalized.get("primary")
    secondary = normalized.get("secondary")
    aggregation = normalized.get("aggregation")
    reports = normalized.get("reports")
    interpretation = normalized.get("interpretation")
    if not all(isinstance(value, Mapping) for value in (protocol, primary, secondary, aggregation, reports, interpretation)):
        raise StatisticalInputError("analysis_plan is missing a required section")
    expected_protocol = {
        "confirmatory_distances_m": [0.6, 0.8, 1.0],
        "barriers": ["none", "cardboard"],
        "trials_per_condition": 3,
        "planned_duration_s": 150,
        "window_duration_s": 30,
        "window_stride_s": 5,
        "minimum_valid_windows_per_trial": 15,
        "minimum_valid_trials_per_participant_condition": 2,
        "target_recruited_participants": 40,
        "minimum_protocol_complete_participants": 38,
        "no_subject_trial_count": 72,
    }
    for key, expected in expected_protocol.items():
        if protocol.get(key) != expected:
            raise StatisticalInputError(
                f"analysis_plan protocol.{key} must remain {expected!r}"
            )
    if primary.get("condition_id") != "d100_none" or primary.get("target") != "rr":
        raise StatisticalInputError("analysis_plan primary must be d100_none RR")
    if primary.get("margin_bpm") != 2.0 or primary.get("alpha") != 0.05:
        raise StatisticalInputError("analysis_plan primary margin/alpha are not frozen")
    if primary.get("confidence_level") != 0.90 or primary.get("minimum_independent_estimates") != 19:
        raise StatisticalInputError("analysis_plan primary sample-size contract is not frozen")
    expected_secondary = [
        "d060_none", "d080_none", "d060_cardboard", "d080_cardboard", "d100_cardboard"
    ]
    if secondary.get("condition_ids") != expected_secondary or secondary.get("multiplicity_adjustment") != "holm":
        raise StatisticalInputError("analysis_plan secondary Holm family is not frozen")
    sequence = aggregation.get("sequence")
    expected_sequence = DEFAULT_ANALYSIS_PLAN["aggregation"]["sequence"]
    if aggregation.get("input") != "outer_oof_predictions.csv" or aggregation.get("unit") != "participant_condition" or aggregation.get("participant_balanced") is not True or sequence != expected_sequence:
        raise StatisticalInputError("analysis_plan aggregation contract is incomplete")
    if reports.get("retain_raw_predictions") is not True or reports.get("retain_raw_and_postprocessed_columns") is not True:
        raise StatisticalInputError("analysis_plan must retain raw and postprocessed predictions")
    expected_reports = DEFAULT_ANALYSIS_PLAN["reports"]
    for key in (
        "metrics", "agreement", "coverage_denominator", "false_alarm_test", "provenance",
    ):
        if reports.get(key) != expected_reports[key]:
            raise StatisticalInputError(f"analysis_plan reports.{key} is not frozen")
    if reports.get("exports") != ["json", "csv", "latex"]:
        raise StatisticalInputError("analysis_plan export contract is not frozen")
    if normalized.get("exclusions") != DEFAULT_ANALYSIS_PLAN["exclusions"]:
        raise StatisticalInputError("analysis_plan exclusion contract is not frozen")
    if interpretation != DEFAULT_ANALYSIS_PLAN["interpretation"]:
        raise StatisticalInputError("analysis_plan interpretation contract is incomplete")
    return normalized


def _plan_primary(plan: Mapping[str, Any]) -> Mapping[str, Any]:
    primary = plan.get("primary")
    if isinstance(primary, Mapping):
        return primary
    # Accept the pre-v16.5.9 flat shape for descriptive, non-confirmatory
    # callers only.  Confirmatory callers are validated above.
    return {
        "condition_id": plan.get("primary_condition", "d100_none"),
        "target": "rr",
        "margin_bpm": float(dict(plan.get("tost") or {}).get("margin", 2.0)),
        "alpha": float(dict(plan.get("tost") or {}).get("alpha", 0.05)),
        "minimum_independent_estimates": 19,
    }


def _plan_secondary(plan: Mapping[str, Any]) -> list[str]:
    secondary = plan.get("secondary")
    if isinstance(secondary, Mapping):
        return [str(value) for value in secondary.get("condition_ids", [])]
    return [str(value) for value in plan.get("secondary_conditions", [])]


def _as_1d(values: Iterable[Any], *, dtype: Any = None) -> np.ndarray:
    """Materialise array-like and generator inputs without treating generators as scalars."""

    if isinstance(values, np.ndarray):
        raw = values
    else:
        raw = np.asarray(values)
        if raw.ndim == 0 and not np.isscalar(values):
            raw = np.asarray(list(values))
    raw = raw.reshape(-1)
    return raw.astype(dtype) if dtype is not None else raw


def _strict_bool_mask(values: Iterable[Any], *, name: str) -> np.ndarray:
    """Parse a contract boolean column without Python's truthiness trap.

    ``bool("false")`` is ``True``.  That conversion is unsafe for eligibility,
    validity, and ledger fields because a CSV string can silently admit an
    ineligible row into a confirmatory analysis.  Only the explicit JSON/CSV
    spellings below are accepted; missing or ambiguous values fail closed.
    """

    raw = _as_1d(values, dtype=object)
    parsed: list[bool] = []
    for value in raw:
        if isinstance(value, (bool, np.bool_)):
            parsed.append(bool(value))
            continue
        if isinstance(value, (int, np.integer)) and int(value) in (0, 1):
            parsed.append(bool(int(value)))
            continue
        if isinstance(value, (float, np.floating)) and math.isfinite(float(value)) and float(value) in (0.0, 1.0):
            parsed.append(bool(int(value)))
            continue
        if isinstance(value, str):
            token = value.strip().lower()
            if token in {"true", "1"}:
                parsed.append(True)
                continue
            if token in {"false", "0"}:
                parsed.append(False)
                continue
        raise StatisticalInputError(
            f"{name} must contain only strict boolean values (true/false or 1/0)"
        )
    return np.asarray(parsed, dtype=bool)


def _strict_attempt_types(values: Iterable[Any], *, name: str = "attempt_type") -> np.ndarray:
    """Parse the two protocol-attempt classes used by denominators."""

    raw = _as_1d(values, dtype=object)
    parsed: list[str] = []
    for value in raw:
        token = value.strip().lower() if isinstance(value, str) else ""
        if token not in {"subject", "no_subject"}:
            raise StatisticalInputError(
                f"{name} must contain only subject or no_subject values"
            )
        parsed.append(token)
    return np.asarray(parsed, dtype=object)


def _validate_confirmatory_provenance(
    frame: Any,
    provenance: Mapping[str, Any],
    plan: Mapping[str, Any],
    *,
    participant_column: str,
) -> dict[str, str]:
    """Verify that predictions carry the same immutable OOF provenance.

    Confirmatory reports must not trust a caller-supplied model label alone.
    The prediction frame therefore carries the OOF fold and holdout-group
    columns, and any duplicated provenance columns must agree with the signed
    run metadata.  This keeps a relabelled or cross-participant prediction file
    from being promoted to a manuscript result.
    """

    required = (
        "source_commit", "model_family", "split_ledger_sha256",
        "prediction_file_sha256", "product_version", "protocol_id",
    )
    if not isinstance(provenance, Mapping) or any(not str(provenance.get(key, "")).strip() for key in required):
        raise StatisticalInputError(
            "confirmatory analysis requires source/model/split/prediction/protocol provenance"
        )
    expected_version = str(plan.get("effective_product_version", "")).strip()
    if str(provenance["product_version"]).strip() != expected_version:
        raise StatisticalInputError("confirmatory provenance product_version does not match the approved plan")
    if str(provenance["protocol_id"]).strip() != str(plan.get("plan_id", "")).strip():
        raise StatisticalInputError("confirmatory provenance protocol_id does not match the approved plan")
    source_commit = str(provenance["source_commit"]).strip()
    if not re.fullmatch(r"[0-9a-fA-F]{7,64}", source_commit):
        raise StatisticalInputError("confirmatory provenance source_commit is not a commit hash")
    for key in ("split_ledger_sha256", "prediction_file_sha256"):
        if not re.fullmatch(r"[0-9a-fA-F]{64}", str(provenance[key]).strip()):
            raise StatisticalInputError(f"confirmatory provenance {key} is not a SHA-256 hash")
    model_family = str(provenance["model_family"]).strip().lower()
    if model_family not in {"gradient_boosting", "cnn_1d"}:
        raise StatisticalInputError("confirmatory provenance model_family is unsupported")
    required_oof = ["outer_fold", "outer_holdout_group"]
    missing_oof = [column for column in required_oof if column not in frame]
    if missing_oof:
        raise StatisticalInputError(
            "confirmatory input is missing OOF provenance columns " + ", ".join(missing_oof)
        )
    frame_families = frame["model_family"].map(lambda value: str(value).strip().lower())
    if frame_families.empty or bool((frame_families == "").any()) or bool((frame_families != model_family).any()):
        raise StatisticalInputError("confirmatory frame model_family does not match provenance")
    for column in ("source_commit", "split_ledger_sha256", "prediction_file_sha256", "product_version", "protocol_id"):
        if column in frame:
            values = frame[column].map(lambda value: str(value).strip())
            expected = str(provenance[column]).strip()
            if bool((values == "").any()) or bool((values != expected).any()):
                raise StatisticalInputError(f"confirmatory frame {column} does not match provenance")
    holdout = frame["outer_holdout_group"].map(lambda value: str(value).strip())
    if bool((holdout == "").any()):
        raise StatisticalInputError("confirmatory OOF holdout groups must be non-empty")
    participants = frame[participant_column].map(lambda value: str(value).strip())
    if bool((participants == "").any()):
        raise StatisticalInputError("confirmatory participant IDs must be non-empty")
    participant_groups = frame[[participant_column]].copy()
    participant_groups["_holdout_group"] = holdout.to_numpy()
    if bool((participant_groups.groupby(participant_column, dropna=False)["_holdout_group"].nunique() > 1).any()):
        raise StatisticalInputError("confirmatory participant appears in multiple OOF holdout groups")
    eligible = _strict_bool_mask(
        frame["confirmatory_eligible"], name="confirmatory_eligible"
    )
    if any(
        not re.fullmatch(r"P-[0-9]{3}", participant)
        for participant in participants[eligible]
    ):
        raise StatisticalInputError(
            "confirmatory-eligible participant IDs must use the P-NNN format"
        )
    if bool((holdout[eligible].to_numpy() != participants[eligible].to_numpy()).any()):
        raise StatisticalInputError(
            "confirmatory OOF holdout group must equal participant_id for every eligible row"
        )
    return {
        "source_commit": source_commit,
        "model_family": model_family,
        "split_ledger_sha256": str(provenance["split_ledger_sha256"]).strip().lower(),
        "prediction_file_sha256": str(provenance["prediction_file_sha256"]).strip().lower(),
        "product_version": expected_version,
        "protocol_id": str(plan.get("plan_id", "")).strip(),
    }


def _scipy_stats():
    try:
        from scipy import stats
    except ImportError as exc:  # pragma: no cover - exercised in minimal installs
        raise StatisticalInputError(
            "statistical reports require scipy; install the trainer statistical extras"
        ) from exc
    return stats


def _finite_pairs(reference: Iterable[Any], estimate: Iterable[Any]) -> tuple[np.ndarray, np.ndarray]:
    ref = _as_1d(reference, dtype=float)
    pred = _as_1d(estimate, dtype=float)
    if ref.size != pred.size:
        raise StatisticalInputError("reference and estimate must have equal lengths")
    mask = np.isfinite(ref) & np.isfinite(pred)
    ref, pred = ref[mask], pred[mask]
    if ref.size < 2:
        raise StatisticalInputError("at least two finite paired observations are required")
    return ref, pred


def _finite_differences(reference: Iterable[Any], estimate: Iterable[Any]) -> np.ndarray:
    ref, pred = _finite_pairs(reference, estimate)
    return pred - ref


def _float(value: Any) -> Optional[float]:
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def rmse(reference: Iterable[Any], estimate: Iterable[Any]) -> float:
    ref, pred = _finite_pairs(reference, estimate)
    return float(np.sqrt(np.mean(np.square(pred - ref))))


def mae(reference: Iterable[Any], estimate: Iterable[Any]) -> float:
    ref, pred = _finite_pairs(reference, estimate)
    return float(np.mean(np.abs(pred - ref)))


def paired_metrics(reference: Iterable[Any], estimate: Iterable[Any]) -> dict[str, Any]:
    """Return error metrics with the explicit orientation ``estimate - reference``."""

    ref, pred = _finite_pairs(reference, estimate)
    difference = pred - ref
    stats = _scipy_stats()
    pearson = stats.pearsonr(ref, pred) if np.ptp(ref) and np.ptp(pred) else None
    spearman = stats.spearmanr(ref, pred) if np.ptp(ref) and np.ptp(pred) else None
    return {
        "n_pairs": int(ref.size),
        "reference_mean": float(np.mean(ref)),
        "estimate_mean": float(np.mean(pred)),
        "rmse": float(np.sqrt(np.mean(np.square(difference)))),
        "mae": float(np.mean(np.abs(difference))),
        "bias_estimate_minus_reference": float(np.mean(difference)),
        "difference_sd": float(np.std(difference, ddof=1)),
        "pearson_r": _float(pearson.statistic if pearson is not None else None),
        "pearson_p": _float(pearson.pvalue if pearson is not None else None),
        "spearman_rho": _float(spearman.statistic if spearman is not None else None),
        "spearman_p": _float(spearman.pvalue if spearman is not None else None),
    }


def paired_tost(
    reference: Iterable[Any],
    estimate: Iterable[Any],
    *,
    lower: float,
    upper: float,
    alpha: float = 0.05,
) -> dict[str, Any]:
    """Run paired TOST on the mean difference ``estimate - reference``.

    Equivalence is declared only when both one-sided tests reject the null of
    a difference at or beyond the predeclared bounds.  The equivalent
    two-sided confidence interval is ``100*(1-2*alpha)%``.
    """

    lower, upper, alpha = float(lower), float(upper), float(alpha)
    if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
        raise StatisticalInputError("TOST bounds must be finite with lower < upper")
    if not 0.0 < alpha < 0.5:
        raise StatisticalInputError("TOST alpha must be between 0 and 0.5")
    difference = _finite_differences(reference, estimate)
    n = int(difference.size)
    df = n - 1
    mean = float(np.mean(difference))
    sd = float(np.std(difference, ddof=1))
    stats = _scipy_stats()
    if sd == 0.0:
        p_lower = 0.0 if mean > lower else 1.0
        p_upper = 0.0 if mean < upper else 1.0
        t_lower = math.inf if mean > lower else -math.inf
        t_upper = -math.inf if mean < upper else math.inf
        ci_low = ci_high = mean
    else:
        sem = sd / math.sqrt(n)
        t_lower = (mean - lower) / sem
        t_upper = (mean - upper) / sem
        p_lower = float(stats.t.sf(t_lower, df))
        p_upper = float(stats.t.cdf(t_upper, df))
        critical = float(stats.t.ppf(1.0 - alpha, df))
        ci_low, ci_high = mean - critical * sem, mean + critical * sem
    pvalue = max(float(p_lower), float(p_upper))
    return {
        "n_pairs": n,
        "degrees_of_freedom": df,
        "alpha": alpha,
        "lower_bound": lower,
        "upper_bound": upper,
        "mean_difference": mean,
        "ci_level": 1.0 - 2.0 * alpha,
        "ci_low": float(ci_low),
        "ci_high": float(ci_high),
        "lower_test_t": _float(t_lower),
        "lower_test_p": float(p_lower),
        "upper_test_t": _float(t_upper),
        "upper_test_p": float(p_upper),
        "p_value": pvalue,
        "equivalent": bool(pvalue < alpha),
        "decision": "equivalent" if pvalue < alpha else "not_equivalent",
    }


def exact_proportion(
    successes: int,
    trials: int,
    *,
    confidence: float = 0.95,
    null_proportion: float = 0.5,
    alternative: str = "two-sided",
) -> dict[str, Any]:
    """Return an exact Clopper-Pearson interval for a Bernoulli proportion."""

    successes, trials = int(successes), int(trials)
    confidence = float(confidence)
    if trials < 1 or successes < 0 or successes > trials:
        raise StatisticalInputError("successes/trials must satisfy 0 <= successes <= trials")
    if not 0.0 < confidence < 1.0:
        raise StatisticalInputError("confidence must be between 0 and 1")
    null_proportion = float(null_proportion)
    if not 0.0 <= null_proportion <= 1.0:
        raise StatisticalInputError("null_proportion must be between 0 and 1")
    if alternative not in {"two-sided", "greater", "less"}:
        raise StatisticalInputError("alternative must be two-sided, greater, or less")
    stats = _scipy_stats()
    result = stats.binomtest(
        successes, trials, p=null_proportion, alternative=alternative
    )
    # The hypothesis test may be one-sided, but the reported interval is always
    # the separately requested two-sided Clopper--Pearson interval.  This keeps
    # a one-sided false-alarm test from silently changing the manuscript CI.
    interval = stats.binomtest(
        successes, trials, p=null_proportion, alternative="two-sided"
    ).proportion_ci(confidence_level=confidence, method="exact")
    return {
        "successes": successes,
        "trials": trials,
        "proportion": successes / trials,
        "confidence": confidence,
        "null_proportion": null_proportion,
        "alternative": alternative,
        "p_value": float(result.pvalue),
        "ci_low": float(interval.low),
        "ci_high": float(interval.high),
        "method": "clopper_pearson_exact",
    }


def _agreement_once(difference: np.ndarray, groups: np.ndarray) -> dict[str, float]:
    unique = np.unique(groups)
    if unique.size < 2:
        raise StatisticalInputError("repeated-measures agreement requires at least two participants")
    means = np.asarray([np.mean(difference[groups == group]) for group in unique], dtype=float)
    counts = np.asarray([np.sum(groups == group) for group in unique], dtype=float)
    overall = float(np.mean(difference))
    within_ss = float(sum(np.sum(np.square(difference[groups == group] - mean))
                          for group, mean in zip(unique, means)))
    within_df = int(difference.size - unique.size)
    within_var = within_ss / within_df if within_df > 0 else 0.0
    mean_var = float(np.var(means, ddof=1)) if unique.size > 1 else 0.0
    # The subject-mean variance contains less within-subject variance when
    # sessions have multiple rows.  This estimates the variance of a new row.
    between_var = max(0.0, mean_var - within_var * float(np.mean(1.0 / counts)))
    total_sd = math.sqrt(max(0.0, within_var + between_var))
    return {
        "bias": overall,
        "within_subject_sd": math.sqrt(max(0.0, within_var)),
        "between_subject_sd": math.sqrt(max(0.0, between_var)),
        "agreement_sd": total_sd,
        "loa_lower": overall - 1.96 * total_sd,
        "loa_upper": overall + 1.96 * total_sd,
        "n_pairs": float(difference.size),
        "n_participants": float(unique.size),
    }


def repeated_measures_agreement(
    reference: Iterable[Any],
    estimate: Iterable[Any],
    participant_ids: Sequence[Any],
    *,
    bootstrap_reps: int = 2000,
    seed: int = 42,
) -> dict[str, Any]:
    """Estimate clustered Bland--Altman limits and participant-bootstrap CIs."""

    ref_raw = _as_1d(reference, dtype=float)
    pred_raw = _as_1d(estimate, dtype=float)
    groups_raw = _as_1d(participant_ids, dtype=object)
    if ref_raw.size != pred_raw.size:
        raise StatisticalInputError("reference and estimate must have equal lengths")
    if groups_raw.size != ref_raw.size:
        raise StatisticalInputError("participant_ids must match the original pair length")
    mask = np.isfinite(ref_raw) & np.isfinite(pred_raw)
    ref, pred = ref_raw[mask], pred_raw[mask]
    if ref.size < 2:
        raise StatisticalInputError("at least two finite paired observations are required")
    groups = groups_raw[mask]
    if groups.size != ref.size or any(str(group).strip() == "" for group in groups):
        raise StatisticalInputError("participant_ids must be present for every finite pair")
    difference = pred - ref
    point = _agreement_once(difference, groups)
    bootstrap_reps = int(bootstrap_reps)
    if bootstrap_reps < 200:
        raise StatisticalInputError("bootstrap_reps must be at least 200 for agreement CIs")
    rng = np.random.default_rng(int(seed))
    unique = np.unique(groups)
    draws = {key: [] for key in ("bias", "loa_lower", "loa_upper")}
    for _ in range(bootstrap_reps):
        sampled = rng.choice(unique, size=unique.size, replace=True)
        cluster_indexes = [np.flatnonzero(groups == group) for group in sampled]
        indexes = np.concatenate(cluster_indexes)
        # Relabel each draw by its bootstrap cluster position.  Keeping the
        # source participant ID would collapse duplicate draws and can produce
        # a false one-participant sample when a cluster is drawn repeatedly.
        boot_groups = np.concatenate([
            np.full(cluster.size, draw_index, dtype=np.int64)
            for draw_index, cluster in enumerate(cluster_indexes)
        ])
        sample = _agreement_once(difference[indexes], boot_groups)
        for key in draws:
            draws[key].append(sample[key])
    return {
        **{key: int(value) if key.startswith("n_") else float(value) for key, value in point.items()},
        "method": "repeated_measures_clustered_bland_altman",
        "bootstrap_reps": bootstrap_reps,
        "bootstrap_seed": int(seed),
        "bootstrap_ci_level": 0.95,
        "bias_ci_low": float(np.quantile(draws["bias"], 0.025)),
        "bias_ci_high": float(np.quantile(draws["bias"], 0.975)),
        "loa_lower_ci_low": float(np.quantile(draws["loa_lower"], 0.025)),
        "loa_lower_ci_high": float(np.quantile(draws["loa_lower"], 0.975)),
        "loa_upper_ci_low": float(np.quantile(draws["loa_upper"], 0.025)),
        "loa_upper_ci_high": float(np.quantile(draws["loa_upper"], 0.975)),
    }


def holm_adjust(p_values: Sequence[float]) -> list[float]:
    """Return step-down Holm adjusted p-values in the input order."""

    values = [float(value) for value in p_values]
    if any(not math.isfinite(value) or value < 0.0 or value > 1.0 for value in values):
        raise StatisticalInputError("p-values must be finite and between 0 and 1")
    order = sorted(range(len(values)), key=lambda index: values[index])
    adjusted_sorted: list[float] = []
    running = 0.0
    count = len(values)
    for rank, index in enumerate(order):
        running = max(running, min(1.0, (count - rank) * values[index]))
        adjusted_sorted.append(running)
    adjusted = [0.0] * count
    for rank, index in enumerate(order):
        adjusted[index] = adjusted_sorted[rank]
    return adjusted


def _condition_id(distance_m: Any, barrier_type: Any) -> Optional[str]:
    try:
        distance = float(distance_m)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(distance):
        return None
    barrier = str(barrier_type).strip().lower()
    if not barrier:
        return None
    return f"d{int(round(distance * 100)):03d}_{barrier}"


def _participant_condition_summary(
    frame: Any,
    *,
    reference_column: str,
    estimate_column: str,
    participant_column: str,
) -> Any:
    """Collapse endpoints to trial summaries, then equal-weight participant/config rows."""

    try:
        import pandas as pd
    except ImportError as exc:  # pragma: no cover - pandas is a trainer dependency
        raise StatisticalInputError("participant-balanced analysis requires pandas") from exc

    columns = [participant_column, reference_column, estimate_column]
    for optional in ("condition_id", "distance_m", "barrier_type", "trial_id", "session_id"):
        if optional in frame.columns and optional not in columns:
            columns.append(optional)
    work = frame.loc[:, columns].copy()
    work[reference_column] = pd.to_numeric(work[reference_column], errors="coerce")
    work[estimate_column] = pd.to_numeric(work[estimate_column], errors="coerce")
    work = work[work[participant_column].notna()]
    work = work[work[participant_column].astype(str).str.strip() != ""]
    if "condition_id" not in work.columns and {"distance_m", "barrier_type"}.issubset(work.columns):
        work["condition_id"] = [
            _condition_id(distance, barrier)
            for distance, barrier in zip(work["distance_m"], work["barrier_type"])
        ]
    condition_columns = [column for column in ("condition_id", "distance_m", "barrier_type") if column in work.columns]
    trial_column = "trial_id" if "trial_id" in work.columns else (
        "session_id" if "session_id" in work.columns else None
    )
    trial_keys = [participant_column, *condition_columns]
    if trial_column and trial_column not in trial_keys:
        trial_keys.append(trial_column)
    # If neither a trial nor session key is present, each source row is already
    # the smallest available observation and will be balanced at participant level.
    trial_summary = (
        work.groupby(trial_keys, dropna=False, sort=True, observed=True)[
            [reference_column, estimate_column]
        ]
        .mean()
        .reset_index()
    )
    participant_keys = [participant_column, *condition_columns]
    participant_summary = (
        trial_summary.groupby(participant_keys, dropna=False, sort=True, observed=True)[
            [reference_column, estimate_column]
        ]
        .mean()
        .reset_index()
    )
    return participant_summary


def _fixed_window_rows(
    group: Any,
    *,
    reference_column: str,
    estimate_column: str,
    participant_column: str,
    trial_id: Any,
    condition_id: Any,
    session_id: Any,
    window_duration_s: float,
    window_stride_s: float,
) -> list[dict[str, Any]]:
    """Reduce endpoint rows to unique fixed-duration analysis windows.

    OOF predictions are commonly emitted at 1 Hz, but the controlled plan's
    independent analysis unit is a 30-second window every 5 seconds.  This
    helper refuses to call a short burst of endpoint rows fifteen windows and
    de-duplicates repeated timestamps before applying the finite-pair rule.
    """

    import pandas as pd

    if "timestamp_s" not in group.columns:
        raise StatisticalInputError("confirmatory aggregation requires timestamp_s")
    work = group.copy()
    work["timestamp_s"] = pd.to_numeric(work["timestamp_s"], errors="coerce")
    work[reference_column] = pd.to_numeric(work[reference_column], errors="coerce")
    work[estimate_column] = pd.to_numeric(work[estimate_column], errors="coerce")
    work = work[np.isfinite(work["timestamp_s"].to_numpy(dtype=float))].sort_values("timestamp_s")
    if work.empty:
        return []
    timestamps = work["timestamp_s"].to_numpy(dtype=float)
    first = float(timestamps[0])
    last = float(timestamps[-1])
    if last - first + 1e-9 < float(window_duration_s):
        return []
    count = int(math.floor((last - first - float(window_duration_s)) / float(window_stride_s) + 1e-9)) + 1
    rows: list[dict[str, Any]] = []
    for window_index in range(max(0, count)):
        end = first + float(window_duration_s) + window_index * float(window_stride_s)
        start = end - float(window_duration_s)
        segment = work[(work["timestamp_s"] >= start - 1e-6) & (work["timestamp_s"] <= end + 1e-6)]
        # Duplicate timestamps cannot create extra independent windows or
        # inflate a valid-window count.
        segment = segment.drop_duplicates(subset=["timestamp_s"], keep="last")
        finite = segment[
            np.isfinite(segment[reference_column].to_numpy(dtype=float))
            & np.isfinite(segment[estimate_column].to_numpy(dtype=float))
        ]
        if len(finite) < 2:
            continue
        finite_ts = finite["timestamp_s"].to_numpy(dtype=float)
        if float(np.max(finite_ts) - np.min(finite_ts)) + 1e-6 < float(window_duration_s):
            continue
        rows.append({
            participant_column: str(group[participant_column].iloc[0]),
            "trial_id": trial_id,
            "condition_id": condition_id,
            "session_id": session_id,
            "window_id": f"{session_id}:{trial_id}:{condition_id}:W{window_index + 1:03d}",
            "window_start_s": start,
            "window_end_s": end,
            reference_column: float(np.median(finite[reference_column].to_numpy(dtype=float))),
            estimate_column: float(np.median(finite[estimate_column].to_numpy(dtype=float))),
        })
    return rows


def aggregate_confirmatory_frame(
    frame: Any,
    *,
    reference_column: str,
    estimate_column: str,
    participant_column: str,
    analysis_plan: Optional[Mapping[str, Any]] = None,
    require_eligibility: bool = False,
) -> tuple[Any, dict[str, Any]]:
    """Apply the frozen window/trial/participant aggregation contract.

    The input contains aligned outer-OOF endpoints.  Each trial is first
    reduced to unique fixed-duration windows at the plan's stride; only then
    are the minimum-window and minimum-trial rules applied.
    """

    try:
        import pandas as pd
    except ImportError as exc:  # pragma: no cover
        raise StatisticalInputError("confirmatory aggregation requires pandas") from exc
    plan = dict(analysis_plan or DEFAULT_ANALYSIS_PLAN)
    protocol = dict(plan.get("protocol") or {})
    min_windows = int(protocol.get("minimum_valid_windows_per_trial", 15))
    min_trials = int(protocol.get("minimum_valid_trials_per_participant_condition", 2))
    window_duration_s = float(protocol.get("window_duration_s", 30))
    window_stride_s = float(protocol.get("window_stride_s", 5))
    required = [participant_column, "trial_id", "timestamp_s", "session_id"]
    if "condition_id" not in frame.columns and not {"distance_m", "barrier_type"}.issubset(frame.columns):
        required.append("condition_id")
    missing = [column for column in required if column not in frame.columns]
    if missing:
        raise StatisticalInputError(
            "confirmatory aggregation requires " + ", ".join(missing)
        )
    columns = [participant_column, "trial_id", reference_column, estimate_column, "timestamp_s", "session_id"]
    for optional in (
        "condition_id", "distance_m", "barrier_type", "model_family", "window_id",
        "rr_valid_for_eval", "confirmatory_eligible", "participant_disjoint",
    ):
        if optional in frame.columns and optional not in columns:
            columns.append(optional)
    work = frame.loc[:, columns].copy()
    if require_eligibility:
        eligibility_columns = ["rr_valid_for_eval", "confirmatory_eligible", "participant_disjoint"]
        missing_eligibility = [column for column in eligibility_columns if column not in work.columns]
        if missing_eligibility:
            raise StatisticalInputError(
                "confirmatory aggregation requires " + ", ".join(missing_eligibility)
            )
        eligibility = np.ones(len(work), dtype=bool)
        for column in eligibility_columns:
            eligibility &= _strict_bool_mask(work[column], name=column)
        work = work.loc[eligibility].copy()
    work[reference_column] = pd.to_numeric(work[reference_column], errors="coerce")
    work[estimate_column] = pd.to_numeric(work[estimate_column], errors="coerce")
    if "condition_id" not in work.columns:
        work["condition_id"] = [
            _condition_id(distance, barrier)
            for distance, barrier in zip(work["distance_m"], work["barrier_type"])
        ]
    work = work[
        work[participant_column].notna()
        & work["trial_id"].notna()
        & work["condition_id"].notna()
    ]
    finite = np.isfinite(work[reference_column].to_numpy(dtype=float)) & np.isfinite(
        work[estimate_column].to_numpy(dtype=float)
    )
    work = work.loc[finite].copy()
    trial_keys = [participant_column, "trial_id", "condition_id", "session_id"]
    trial_rows = []
    for key, group in work.groupby(trial_keys, sort=True, dropna=False, observed=True):
        window_rows = _fixed_window_rows(
            group,
            reference_column=reference_column,
            estimate_column=estimate_column,
            participant_column=participant_column,
            trial_id=key[1],
            condition_id=key[2],
            session_id=key[3],
            window_duration_s=window_duration_s,
            window_stride_s=window_stride_s,
        )
        if len(window_rows) < min_windows:
            continue
        trial_rows.extend(window_rows)
    # Collapse the validated windows to one trial summary.  Keeping the
    # window rows in a separate intermediate frame makes the denominator
    # auditable and prevents source-row counts from masquerading as windows.
    window_frame = pd.DataFrame(trial_rows)
    if not window_frame.empty:
        trial_summary = (
            window_frame.groupby([participant_column, "trial_id", "condition_id", "session_id"], sort=True, observed=True)
            .agg({
                reference_column: "median",
                estimate_column: "median",
                "window_id": "nunique",
            })
            .reset_index()
            .rename(columns={"window_id": "valid_window_count"})
        )
    else:
        trial_summary = pd.DataFrame()
    participant_rows = []
    if not trial_summary.empty:
        for key, group in trial_summary.groupby(
            [participant_column, "condition_id"], sort=True, dropna=False, observed=True
        ):
            if len(group) < min_trials:
                continue
            participant_rows.append({
                participant_column: key[0],
                "condition_id": key[1],
                reference_column: float(np.mean(group[reference_column].to_numpy(dtype=float))),
                estimate_column: float(np.mean(group[estimate_column].to_numpy(dtype=float))),
                "valid_trial_count": int(len(group)),
                "valid_window_count": int(group["valid_window_count"].sum()),
            })
    summary = pd.DataFrame(participant_rows)
    return summary, {
        "input_rows": int(len(frame)),
        "finite_endpoint_rows": int(len(work)),
        "validated_window_count": int(len(window_frame)),
        "eligible_trial_count": int(len(trial_summary)),
        "eligible_participant_condition_count": int(len(summary)),
        "minimum_valid_windows_per_trial": min_windows,
        "minimum_valid_trials_per_participant_condition": min_trials,
        "window_duration_s": window_duration_s,
        "window_stride_s": window_stride_s,
        "eligibility_filtered": bool(require_eligibility),
        "sequence": "30s_window_median_then_trial_median_then_participant_condition_mean",
    }


def _condition_tost(
    frame: Any,
    *,
    reference_column: str,
    estimate_column: str,
    participant_column: str,
    margin: float,
    alpha: float,
    analysis_plan: Optional[Mapping[str, Any]] = None,
    summary_frame: Any = None,
) -> dict[str, Any]:
    """Run the proposal's primary plus five Holm-adjusted secondary condition tests."""

    plan = dict(analysis_plan or DEFAULT_ANALYSIS_PLAN)
    primary_spec = _plan_primary(plan)
    primary_condition = str(primary_spec.get("condition_id", "d100_none"))
    secondary_conditions = _plan_secondary(plan)
    minimum_independent_estimates = int(primary_spec.get("minimum_independent_estimates", 19))
    if "condition_id" not in frame.columns and not {
        "distance_m", "barrier_type"
    }.issubset(frame.columns):
        return {
            "status": "not_tested_missing_condition_columns",
            "primary_condition": primary_condition,
            "secondary_conditions": secondary_conditions,
        }
    summary = summary_frame if summary_frame is not None else _participant_condition_summary(
        frame,
        reference_column=reference_column,
        estimate_column=estimate_column,
        participant_column=participant_column,
    )
    if "condition_id" not in summary.columns:
        return {
            "status": "not_tested_missing_condition_ids",
            "primary_condition": primary_condition,
            "secondary_conditions": secondary_conditions,
        }

    def run(condition: str) -> dict[str, Any]:
        block = summary[summary["condition_id"].astype(str) == condition]
        finite = np.isfinite(block[reference_column].to_numpy(dtype=float)) & np.isfinite(
            block[estimate_column].to_numpy(dtype=float)
        )
        participant_block = (
            block.loc[finite, [participant_column, reference_column, estimate_column]]
            .groupby(participant_column, dropna=False, sort=True, observed=True)[
                [reference_column, estimate_column]
            ]
            .mean()
            .reset_index()
        )
        participants = sorted(
            str(value) for value in participant_block[participant_column]
        )
        base = {
            "condition_id": condition,
            "n_participants": len(participants),
            "minimum_required": minimum_independent_estimates,
            "status": "tested" if len(participants) >= minimum_independent_estimates else "inconclusive_insufficient_independent_estimates",
        }
        if len(participants) < minimum_independent_estimates:
            return base
        result = paired_tost(
            participant_block[reference_column].to_numpy(dtype=float),
            participant_block[estimate_column].to_numpy(dtype=float),
            lower=-margin,
            upper=margin,
            alpha=alpha,
        )
        return {**base, **result}

    primary = run(primary_condition)
    secondary = [run(condition) for condition in secondary_conditions]
    # Keep the predeclared family size fixed at five.  Missing secondary
    # conditions are inconclusive and contribute p=1 rather than shrinking
    # the multiplicity correction around whichever files happened to exist.
    adjusted = holm_adjust([
        float(item.get("p_value", 1.0)) if item.get("status") == "tested" else 1.0
        for item in secondary
    ]) if secondary else []
    for item, adjusted_p in zip(secondary, adjusted):
        item["p_value_holm"] = adjusted_p
        if item.get("status") == "tested":
            item["equivalent_holm"] = bool(adjusted_p < alpha)
            item["decision_holm"] = "equivalent" if item["equivalent_holm"] else "not_equivalent"
        else:
            item["equivalent_holm"] = False
            item["decision_holm"] = "inconclusive"
    return {
        "status": "tested" if primary.get("status") == "tested" else primary.get("status"),
        "primary": primary,
        "secondary": secondary,
        "primary_condition": primary_condition,
        "secondary_conditions": secondary_conditions,
        "multiple_testing": "holm",
        "alpha": float(alpha),
        "margin": float(margin),
        "aggregation": "trial_then_participant_condition",
    }


def coverage_report(
    frame: Any,
    *,
    reference_column: str,
    estimate_column: str,
    false_alarm_column: Optional[str] = None,
    false_alarm_null: float = 0.05,
    attempt_ledger: Any = None,
    minimum_valid_windows_per_trial: Optional[int] = None,
    window_duration_s: float = 30.0,
    window_stride_s: float = 5.0,
) -> dict[str, Any]:
    """Report output/missing-output denominators and optional exact false alarms."""

    if false_alarm_column and false_alarm_column not in frame:
        raise StatisticalInputError(f"input is missing requested {false_alarm_column!r}")
    ref = _as_1d(frame[reference_column], dtype=float)
    estimate = _as_1d(frame[estimate_column], dtype=float)
    if ref.size != estimate.size:
        raise StatisticalInputError("reference and estimate must have equal lengths")
    eligible = np.isfinite(ref)
    output = eligible & np.isfinite(estimate)
    denominator_unit = "row"
    if attempt_ledger is not None:
        try:
            import pandas as pd
        except ImportError as exc:  # pragma: no cover
            raise StatisticalInputError("attempt-ledger coverage requires pandas") from exc
        ledger = attempt_ledger.copy() if hasattr(attempt_ledger, "copy") else pd.DataFrame(attempt_ledger)
        key_columns = ["participant_id", "trial_id", "condition_id"]
        missing = [column for column in key_columns if column not in ledger.columns]
        if missing:
            raise StatisticalInputError("attempt ledger is missing " + ", ".join(missing))
        if "eligible" in ledger.columns:
            eligible_mask = _strict_bool_mask(ledger["eligible"], name="attempt_ledger.eligible")
            ledger = ledger.loc[eligible_mask].copy()
        ledger = ledger.drop_duplicates(subset=key_columns)
        if ledger.empty:
            raise StatisticalInputError("attempt ledger has no eligible attempted trials")
        output = frame.copy()
        output[estimate_column] = pd.to_numeric(output[estimate_column], errors="coerce")
        output = output[np.isfinite(output[estimate_column].to_numpy(dtype=float))]
        grouping = [column for column in key_columns if column in output.columns]
        if len(grouping) != len(key_columns):
            raise StatisticalInputError("prediction frame lacks attempt-ledger keys")
        output_keys = set()
        grouped = output.groupby(key_columns, dropna=False, sort=False, observed=True)
        for key, group in grouped:
            if minimum_valid_windows_per_trial is not None:
                if "window_id" in group.columns:
                    count = int(group["window_id"].nunique())
                elif {"timestamp_s", "session_id"}.issubset(group.columns):
                    window_rows = _fixed_window_rows(
                        group,
                        reference_column=reference_column,
                        estimate_column=estimate_column,
                        participant_column="participant_id",
                        trial_id=key[1],
                        condition_id=key[2],
                        session_id=group["session_id"].iloc[0],
                        window_duration_s=window_duration_s,
                        window_stride_s=window_stride_s,
                    )
                    count = len(window_rows)
                else:
                    count = int(len(group))
                if count < int(minimum_valid_windows_per_trial):
                    continue
            output_keys.add(tuple(key if isinstance(key, tuple) else (key,)))
        attempted_keys = {
            tuple(row[column] for column in key_columns)
            for _, row in ledger.iterrows()
        }
        with_output = attempted_keys.intersection(output_keys)
        attempted = len(attempted_keys)
        result: dict[str, Any] = {
            "n_attempted": attempted,
            "n_with_output": len(with_output),
            "n_missing_output": attempted - len(with_output),
            "coverage": len(with_output) / attempted,
            "denominator_unit": "trial",
            "coverage_exact": exact_proportion(
                len(with_output), attempted, null_proportion=0.95, alternative="two-sided"
            ),
            "missing_output_exact": exact_proportion(
                attempted - len(with_output), attempted, null_proportion=0.05, alternative="greater"
            ),
        }
        if false_alarm_column:
            if "attempt_type" not in ledger.columns:
                raise StatisticalInputError("false-alarm coverage requires attempt_type in the ledger")
            attempt_types = _strict_attempt_types(ledger["attempt_type"])
            no_subject_keys = {
                tuple(row[column] for column in key_columns)
                for _, row in ledger.loc[attempt_types == "no_subject"].iterrows()
            }
            alarm_keys = set()
            # False alarms are trial-level events.  They must remain countable
            # even when a no-subject capture has no RR estimate or cannot meet
            # the subject window-count gate.
            alarm_source = frame
            for key, group in alarm_source.groupby(key_columns, dropna=False, sort=False, observed=True):
                normalized_key = tuple(key if isinstance(key, tuple) else (key,))
                if normalized_key not in no_subject_keys:
                    continue
                values = group[false_alarm_column]
                truth = values.map(lambda value: value is True or str(value).strip().lower() in {"1", "true", "yes", "alarm"})
                if bool(truth.any()):
                    alarm_keys.add(normalized_key)
            result["no_subject_denominator"] = len(no_subject_keys)
            result["no_subject_false_alarms"] = len(no_subject_keys.intersection(alarm_keys))
            result["false_alarm_exact"] = exact_proportion(
                result["no_subject_false_alarms"],
                len(no_subject_keys),
                null_proportion=false_alarm_null,
                alternative="less",
            )
        return result
    if "trial_id" in frame:
        try:
            import pandas as pd
            grouping = [column for column in ("participant_id", "trial_id", "condition_id") if column in frame]
            if grouping:
                work = frame.loc[eligible, [*grouping, estimate_column]].copy()
                work[estimate_column] = pd.to_numeric(work[estimate_column], errors="coerce")
                grouped = work.groupby(grouping, dropna=False, sort=False, observed=True)
                attempted = int(grouped.ngroups)
                with_output = int(sum(np.isfinite(group[estimate_column].to_numpy(dtype=float)).any() for _, group in grouped))
                denominator_unit = "trial"
            else:
                attempted = int(np.sum(eligible))
                with_output = int(np.sum(output))
        except ImportError:  # pragma: no cover - pandas is a trainer dependency
            attempted = int(np.sum(eligible))
            with_output = int(np.sum(output))
    else:
        attempted = int(np.sum(eligible))
        with_output = int(np.sum(output))
    missing = attempted - with_output
    if attempted < 1:
        raise StatisticalInputError("coverage requires at least one eligible attempt")
    result: dict[str, Any] = {
        "n_attempted": attempted,
        "n_with_output": with_output,
        "n_missing_output": missing,
        "coverage": with_output / attempted,
        "denominator_unit": denominator_unit,
        "coverage_exact": exact_proportion(
            with_output, attempted, null_proportion=0.95, alternative="two-sided"
        ),
        "missing_output_exact": exact_proportion(
            missing, attempted, null_proportion=0.05, alternative="greater"
        ),
    }
    if false_alarm_column:
        alarms = _as_1d(frame[false_alarm_column], dtype=object)
        if alarms.size != ref.size:
            raise StatisticalInputError("false alarm column must match the input length")
        truth = np.asarray([
            value is True or str(value).strip().lower() in {"1", "true", "yes", "alarm"}
            for value in alarms
        ], dtype=bool)
        eligible_alarm = np.isfinite(ref)
        result["false_alarm_exact"] = exact_proportion(
            int(np.sum(truth & eligible_alarm)),
            int(np.sum(eligible_alarm)),
            null_proportion=false_alarm_null,
            alternative="less",
        )
    return result


def mixed_effects_condition_analysis(
    frame: Any,
    *,
    reference_column: str,
    estimate_column: str,
    participant_column: str,
    summary_frame: Any = None,
) -> dict[str, Any]:
    """Fit the optional participant-random-intercept distance/barrier model."""

    try:
        import pandas as pd
    except ImportError as exc:  # pragma: no cover
        raise StatisticalInputError("mixed-effects analysis requires pandas") from exc
    summary = summary_frame.copy() if summary_frame is not None else _participant_condition_summary(
        frame,
        reference_column=reference_column,
        estimate_column=estimate_column,
        participant_column=participant_column,
    )
    if "condition_id" not in summary.columns:
        return {"status": "not_tested_missing_condition_ids"}
    if "distance_m" not in summary.columns or "barrier_type" not in summary.columns:
        parsed = summary["condition_id"].astype(str).str.extract(r"^d(?P<distance>[0-9]+)_(?P<barrier>.+)$")
        summary["distance_m"] = pd.to_numeric(parsed["distance"], errors="coerce") / 100.0
        summary["barrier_type"] = parsed["barrier"]
    summary["difference"] = pd.to_numeric(summary[estimate_column], errors="coerce") - pd.to_numeric(summary[reference_column], errors="coerce")
    summary["cardboard"] = (summary["barrier_type"].astype(str).str.lower() == "cardboard").astype(int)
    summary = summary[np.isfinite(summary["difference"].to_numpy(dtype=float)) & np.isfinite(summary["distance_m"].to_numpy(dtype=float))]
    if summary[participant_column].astype(str).nunique() < 3 or len(summary) < 6:
        return {
            "status": "not_tested_insufficient_participants",
            "n_participants": int(summary[participant_column].astype(str).nunique()),
            "n_rows": int(len(summary)),
        }
    try:
        import statsmodels.formula.api as smf
    except ImportError:
        return {
            "status": "unavailable_optional_dependency",
            "dependency": "statsmodels",
            "install": "pip install 'rvt-trainer[stats]'",
            "n_participants": int(summary[participant_column].astype(str).nunique()),
            "n_rows": int(len(summary)),
        }
    try:
        model = smf.mixedlm(
            "difference ~ distance_m * cardboard",
            summary,
            groups=summary[participant_column].astype(str),
        )
        fitted = model.fit(reml=False, method="lbfgs", disp=False)
        fixed = {}
        for name, coefficient in fitted.fe_params.items():
            fixed[str(name)] = {
                "coefficient": float(coefficient),
                "p_value": _float(fitted.pvalues.get(name)),
            }
        return {
            "status": "tested",
            "method": "participant_random_intercept_mixed_effects",
            "n_participants": int(summary[participant_column].astype(str).nunique()),
            "n_rows": int(len(summary)),
            "fixed_effects": fixed,
            "aic": _float(fitted.aic),
            "bic": _float(fitted.bic),
        }
    except Exception as exc:  # pragma: no cover - depends on optional solver/data
        return {
            "status": "fit_failed",
            "method": "participant_random_intercept_mixed_effects",
            "error": str(exc),
            "n_participants": int(summary[participant_column].astype(str).nunique()),
            "n_rows": int(len(summary)),
        }


def _report_rows(report: Mapping[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for section, value in report.items():
        if isinstance(value, Mapping):
            for metric, metric_value in value.items():
                if isinstance(metric_value, (str, int, float, bool)) or metric_value is None:
                    rows.append({"section": section, "metric": metric, "value": metric_value})
        elif isinstance(value, (str, int, float, bool)) or value is None:
            rows.append({"section": "report", "metric": section, "value": value})
    condition = report.get("condition_tost")
    if isinstance(condition, Mapping):
        for group in ("primary", "secondary"):
            values = condition.get(group, [])
            values = values if isinstance(values, list) else [values]
            for item in values:
                if isinstance(item, Mapping):
                    for key in (
                        "condition_id", "status", "n_participants", "minimum_required",
                        "mean_difference", "ci_low", "ci_high", "lower_test_p",
                        "upper_test_p", "p_value", "p_value_holm", "equivalent",
                        "decision", "equivalent_holm", "decision_holm",
                    ):
                        if key in item:
                            rows.append({"section": f"condition_tost.{group}", "metric": f"{item.get('condition_id')}.{key}", "value": item[key]})
    return rows


def _json_hash(payload: Mapping[str, Any]) -> str:
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def write_statistical_outputs(
    report: Mapping[str, Any],
    json_path: Path | str,
    *,
    csv_path: Optional[Path | str] = None,
    latex_path: Optional[Path | str] = None,
) -> dict[str, Path]:
    """Write stable JSON plus optional flat CSV and manuscript LaTeX table."""

    payload = dict(report)
    payload.pop("report_sha256", None)
    payload["report_sha256"] = _json_hash(payload)
    json_target = Path(json_path)
    json_target.parent.mkdir(parents=True, exist_ok=True)
    json_target.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    outputs = {"json": json_target}
    rows = _report_rows(payload)
    if csv_path:
        target = Path(csv_path)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=["section", "metric", "value"])
            writer.writeheader()
            writer.writerows(rows)
        outputs["csv"] = target
    if latex_path:
        target = Path(latex_path)
        target.parent.mkdir(parents=True, exist_ok=True)
        def escape(value: Any) -> str:
            return str(value).replace("\\", "\\textbackslash{}").replace("_", "\\_").replace("%", "\\%")
        lines = [
            "% Generated by rvt_trainer.statistics; do not edit by hand.",
            "\\begin{tabular}{lll}",
            "\\toprule",
            "Section & Metric & Value \\\\",
            "\\midrule",
        ]
        lines.extend(
            f"{escape(row['section'])} & {escape(row['metric'])} & {escape(row['value'])} \\\\" 
            for row in rows
        )
        lines.extend(["\\bottomrule", "\\end{tabular}", ""])
        target.write_text("\n".join(lines), encoding="utf-8")
        outputs["latex"] = target
    return outputs


def analyze_frame(
    frame: Any,
    *,
    reference_column: str,
    estimate_column: str,
    participant_column: Optional[str] = "participant_id",
    tost_margin: Optional[float] = None,
    alpha: float = 0.05,
    bootstrap_reps: int = 2000,
    seed: int = 42,
    false_alarm_column: Optional[str] = None,
    analysis_plan: Optional[Mapping[str, Any]] = None,
    provenance: Optional[Mapping[str, Any]] = None,
    attempt_ledger: Any = None,
    confirmatory: bool = False,
) -> dict[str, Any]:
    """Analyze a dataframe-like object and return JSON-safe report data."""

    if reference_column not in frame or estimate_column not in frame:
        raise StatisticalInputError(
            f"input is missing {reference_column!r} or {estimate_column!r}"
        )
    plan = dict(analysis_plan or DEFAULT_ANALYSIS_PLAN)
    analysis_frame = frame
    exclusions: dict[str, int] = {}
    if confirmatory:
        if analysis_plan is None:
            raise StatisticalInputError(
                "confirmatory analysis requires an explicit approved analysis plan"
            )
        plan = _validate_analysis_plan(plan, require_approved=True)
        if not participant_column or participant_column not in frame:
            raise StatisticalInputError("confirmatory analysis requires participant_id")
        primary_spec = _plan_primary(plan)
        target = str(primary_spec.get("target", ""))
        allowed_estimates = {"pred_rr", "pred_rr_raw"}
        if target != "rr" or reference_column != "ref_rr" or estimate_column not in allowed_estimates:
            raise StatisticalInputError(
                "confirmatory primary analysis requires ref_rr with pred_rr or pred_rr_raw"
            )
        required_columns = [
            str(participant_column), "session_id", "trial_id", "condition_id",
            "timestamp_s", "rr_valid_for_eval", "confirmatory_eligible",
            "participant_disjoint", "model_family",
        ]
        missing = [column for column in required_columns if column not in frame]
        if missing:
            raise StatisticalInputError(
                "confirmatory input is missing " + ", ".join(missing)
            )
        provenance = _validate_confirmatory_provenance(
            frame,
            provenance if isinstance(provenance, Mapping) else {},
            plan,
            participant_column=str(participant_column),
        )
        if attempt_ledger is None:
            raise StatisticalInputError(
                "confirmatory analysis requires an explicit attempted-trial ledger"
            )
        if not hasattr(attempt_ledger, "columns") or "attempt_type" not in attempt_ledger.columns:
            raise StatisticalInputError("confirmatory attempt ledger requires attempt_type")
        ledger_for_validation = attempt_ledger.copy()
        if "eligible" in ledger_for_validation.columns:
            ledger_for_validation = ledger_for_validation.loc[
                _strict_bool_mask(ledger_for_validation["eligible"], name="attempt_ledger.eligible")
            ].copy()
        attempt_types = _strict_attempt_types(ledger_for_validation["attempt_type"])
        no_subject_count = int(np.sum(attempt_types == "no_subject"))
        expected_no_subject = int(dict(plan.get("protocol") or {}).get("no_subject_trial_count", 72))
        if no_subject_count != expected_no_subject:
            raise StatisticalInputError(
                f"confirmatory attempt ledger requires {expected_no_subject} no-subject trials; found {no_subject_count}"
            )
        if false_alarm_column and false_alarm_column not in frame:
            raise StatisticalInputError(f"input is missing requested {false_alarm_column!r}")
        rr_valid = _strict_bool_mask(frame["rr_valid_for_eval"], name="rr_valid_for_eval")
        confirmatory_eligible = _strict_bool_mask(
            frame["confirmatory_eligible"], name="confirmatory_eligible"
        )
        participant_disjoint = _strict_bool_mask(
            frame["participant_disjoint"], name="participant_disjoint"
        )
        if bool((confirmatory_eligible & ~participant_disjoint).any()):
            raise StatisticalInputError(
                "confirmatory-eligible rows must come from participant-disjoint OOF folds"
            )
        valid = rr_valid & confirmatory_eligible & participant_disjoint
        exclusions = {
            "invalid_rr": int((~rr_valid).sum()),
            "ineligible": int((~confirmatory_eligible).sum()),
            "participant_overlap": int((~participant_disjoint).sum()),
        }
        analysis_frame = frame.loc[valid].copy()
        if analysis_frame.empty:
            raise StatisticalInputError("confirmatory input has no eligible RR predictions")
    coverage_frame = analysis_frame
    if confirmatory and false_alarm_column and hasattr(attempt_ledger, "columns"):
        # Keep explicit no-subject captures in the false-alarm denominator even
        # though they are intentionally absent from the subject RR analysis.
        ledger_for_coverage = attempt_ledger.copy()
        if "eligible" in ledger_for_coverage.columns:
            ledger_for_coverage = ledger_for_coverage.loc[
                _strict_bool_mask(ledger_for_coverage["eligible"], name="attempt_ledger.eligible")
            ].copy()
        coverage_attempt_types = _strict_attempt_types(ledger_for_coverage["attempt_type"])
        key_columns = ["participant_id", "trial_id", "condition_id"]
        if all(column in frame.columns for column in key_columns):
            no_subject_keys = {
                tuple(str(row[column]) for column in key_columns)
                for _, row in ledger_for_coverage.loc[coverage_attempt_types == "no_subject"].iterrows()
            }
            frame_keys = frame[key_columns].apply(
                lambda row: tuple(str(row[column]) for column in key_columns), axis=1
            )
            no_subject_rows = frame.loc[frame_keys.isin(no_subject_keys)].copy()
            if not no_subject_rows.empty:
                try:
                    import pandas as pd
                except ImportError as exc:  # pragma: no cover
                    raise StatisticalInputError("confirmatory coverage requires pandas") from exc
                coverage_frame = pd.concat([analysis_frame, no_subject_rows], ignore_index=True)
    participant_present = bool(participant_column and participant_column in analysis_frame)
    report: dict[str, Any] = {
        "analysis_schema": "rvt-statistical-report-v1",
        "reference_column": reference_column,
        "estimate_column": estimate_column,
        "metrics": paired_metrics(analysis_frame[reference_column], analysis_frame[estimate_column]),
        "coverage": coverage_report(
            coverage_frame,
            reference_column=reference_column,
            estimate_column=estimate_column,
            false_alarm_column=false_alarm_column,
            attempt_ledger=attempt_ledger if confirmatory else None,
            minimum_valid_windows_per_trial=(
                int(dict(plan.get("protocol") or {}).get("minimum_valid_windows_per_trial", 15))
                if confirmatory else None
            ),
            window_duration_s=float(dict(plan.get("protocol") or {}).get("window_duration_s", 30)),
            window_stride_s=float(dict(plan.get("protocol") or {}).get("window_stride_s", 5)),
        ),
        "analysis_plan": plan,
    }
    if exclusions:
        report["exclusions"] = exclusions
    denominators = {"n_rows": int(len(frame))}
    for label, column in (
        ("n_participants", participant_column),
        ("n_trials", "trial_id"),
        ("n_sessions", "session_id"),
    ):
        if column and column in analysis_frame:
            denominators[label] = int(analysis_frame[column].dropna().astype(str).nunique())
    report["denominators"] = denominators
    balanced_frame = None
    aggregation_diagnostics = None
    if participant_present:
        if confirmatory:
            balanced_frame, aggregation_diagnostics = aggregate_confirmatory_frame(
                analysis_frame,
                reference_column=reference_column,
                estimate_column=estimate_column,
                participant_column=str(participant_column),
                analysis_plan=plan,
                require_eligibility=True,
            )
            report["aggregation"] = aggregation_diagnostics
        else:
            balanced_frame = _participant_condition_summary(
                analysis_frame,
                reference_column=reference_column,
                estimate_column=estimate_column,
                participant_column=str(participant_column),
            )
        if balanced_frame is not None and len(balanced_frame) >= 2:
            report["participant_balanced_metrics"] = paired_metrics(
                balanced_frame[reference_column], balanced_frame[estimate_column]
            )
        else:
            report["participant_balanced_metrics"] = {
                "status": "not_tested_insufficient_participant_condition_values",
                "n_pairs": int(len(balanced_frame)) if balanced_frame is not None else 0,
            }
    if confirmatory:
        primary_spec = _plan_primary(plan)
        plan_margin = float(primary_spec["margin_bpm"])
        plan_alpha = float(primary_spec["alpha"])
        if tost_margin is not None and not math.isclose(float(tost_margin), plan_margin, rel_tol=0.0, abs_tol=1e-9):
            raise StatisticalInputError("confirmatory TOST margin must come from the approved plan")
        if not math.isclose(float(alpha), plan_alpha, rel_tol=0.0, abs_tol=1e-9):
            raise StatisticalInputError("confirmatory alpha must come from the approved plan")
        tost_margin = plan_margin
        alpha = plan_alpha
    if tost_margin is not None:
        margin = abs(float(tost_margin))
        if not margin:
            raise StatisticalInputError("tost_margin must be non-zero")
        if participant_present:
            report["condition_tost"] = _condition_tost(
                analysis_frame,
                reference_column=reference_column,
                estimate_column=estimate_column,
                participant_column=str(participant_column),
                margin=margin,
                alpha=alpha,
                analysis_plan=plan,
                summary_frame=balanced_frame,
            )
            if confirmatory:
                # The plan's primary condition is the sole primary result;
                # never pool the six conditions into a manuscript claim.
                report["tost"] = dict(report["condition_tost"].get("primary") or {})
            else:
                tost_frame = balanced_frame if balanced_frame is not None else analysis_frame
                report["tost"] = paired_tost(
                    tost_frame[reference_column], tost_frame[estimate_column],
                    lower=-margin, upper=margin, alpha=alpha,
                )
        else:
            tost_frame = balanced_frame if balanced_frame is not None else analysis_frame
            report["tost"] = paired_tost(
                tost_frame[reference_column], tost_frame[estimate_column],
                lower=-margin, upper=margin, alpha=alpha,
            )
    if participant_present and (
        "condition_id" in analysis_frame
        or {"distance_m", "barrier_type"}.issubset(analysis_frame.columns)
    ):
        report["condition_effects"] = mixed_effects_condition_analysis(
            analysis_frame,
            reference_column=reference_column,
            estimate_column=estimate_column,
            participant_column=str(participant_column),
            summary_frame=balanced_frame if confirmatory else None,
        )
    if participant_present and balanced_frame is not None and len(balanced_frame) >= 2:
        report["agreement"] = repeated_measures_agreement(
            balanced_frame[reference_column],
            balanced_frame[estimate_column],
            balanced_frame[str(participant_column)],
            bootstrap_reps=bootstrap_reps, seed=seed,
        )
    report["analysis_policy"] = {
        "error_orientation": "estimate_minus_reference",
        "participant_clustered": participant_present,
        "aggregation": "30s_window_median_then_trial_median_then_participant_condition_mean" if confirmatory else ("trial_then_participant_condition" if participant_present else "row_pair"),
        "confirmatory_primary_condition": _plan_primary(plan).get("condition_id", "d100_none"),
        "confirmatory_primary_target": _plan_primary(plan).get("target", "rr"),
        "confirmatory_secondary_holm_conditions": _plan_secondary(plan),
        "alpha": float(alpha),
        "bootstrap_seed": int(seed),
        "confirmatory": bool(confirmatory),
    }
    if provenance is not None:
        report["provenance"] = dict(provenance)
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a Radar Vital statistical validation report")
    parser.add_argument("--input", required=True, help="prediction CSV")
    parser.add_argument("--reference-column", default="ref_hr")
    parser.add_argument("--estimate-column", default="pred_hr")
    parser.add_argument("--participant-column", default="participant_id")
    parser.add_argument("--tost-margin", type=float, default=None)
    parser.add_argument("--alpha", type=float, default=0.05)
    parser.add_argument("--bootstrap-reps", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--false-alarm-column", default=None)
    parser.add_argument("--analysis-plan", default=None, help="versioned analysis-plan JSON")
    parser.add_argument("--provenance-json", default=None)
    parser.add_argument("--attempt-ledger", default=None, help="eligible attempted-trial ledger CSV")
    parser.add_argument("--confirmatory", action="store_true", help="enforce the versioned window/trial aggregation contract")
    parser.add_argument("--out", default="statistical_report.json")
    parser.add_argument("--csv-out", default=None)
    parser.add_argument("--latex-out", default=None)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    import pandas as pd
    frame = pd.read_csv(args.input)
    analysis_plan = None
    if args.analysis_plan:
        analysis_plan = json.loads(Path(args.analysis_plan).read_text(encoding="utf-8"))
    provenance = None
    if args.provenance_json:
        provenance = json.loads(Path(args.provenance_json).read_text(encoding="utf-8"))
    attempt_ledger = None
    if args.attempt_ledger:
        attempt_ledger = pd.read_csv(args.attempt_ledger)
    report = analyze_frame(
        frame,
        reference_column=args.reference_column,
        estimate_column=args.estimate_column,
        participant_column=args.participant_column or None,
        tost_margin=args.tost_margin,
        alpha=args.alpha,
        bootstrap_reps=args.bootstrap_reps,
        seed=args.seed,
        false_alarm_column=args.false_alarm_column,
        analysis_plan=analysis_plan,
        provenance=provenance,
        attempt_ledger=attempt_ledger,
        confirmatory=args.confirmatory,
    )
    outputs = write_statistical_outputs(
        report,
        args.out,
        csv_path=args.csv_out,
        latex_path=args.latex_out,
    )
    print(f"Statistical report written to {outputs['json'].resolve()}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
