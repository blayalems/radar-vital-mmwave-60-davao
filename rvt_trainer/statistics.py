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
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

import numpy as np


# This is deliberately data, not a hidden statistical constant.  A caller can
# replace it with a reviewed analysis-plan JSON before running a report.
DEFAULT_ANALYSIS_PLAN = {
    "schema": "rvt-analysis-plan-v1",
    "plan_id": "RVT-STA-PLAN-16.5.8",
    "product_version": "16.5.8",
    "primary_condition": "d100_none",
    "secondary_conditions": [
        "d060_none",
        "d080_none",
        "d060_cardboard",
        "d080_cardboard",
        "d100_cardboard",
    ],
    "confirmatory_distances_m": [0.6, 0.8, 1.0],
    "barriers": ["none", "cardboard"],
    "trials_per_condition": 3,
    "planned_duration_s": 150,
    "tost": {"margin": 2.0, "alpha": 0.05, "ci_level": 0.90},
    "multiple_testing": "holm",
    "aggregation": "trial_then_participant_condition",
    "exclusions": [
        "legacy_unassigned",
        "participant_reassigned",
        "outside_confirmatory_condition_set",
        "missing_release_or_protocol_provenance",
    ],
}


class StatisticalInputError(ValueError):
    """Raised when a statistical contract is incomplete or invalid."""


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


def aggregate_confirmatory_frame(
    frame: Any,
    *,
    reference_column: str,
    estimate_column: str,
    participant_column: str,
    analysis_plan: Optional[Mapping[str, Any]] = None,
) -> tuple[Any, dict[str, Any]]:
    """Apply the frozen window/trial/participant aggregation contract.

    The input is expected to contain one aligned outer-OOF endpoint/window per
    row.  Each trial contributes only when it has the predeclared minimum
    finite windows; each participant-condition contributes only when it has
    the predeclared minimum valid trials.  The returned frame is therefore
    safe for participant-balanced paired inference.
    """

    try:
        import pandas as pd
    except ImportError as exc:  # pragma: no cover
        raise StatisticalInputError("confirmatory aggregation requires pandas") from exc
    plan = dict(analysis_plan or DEFAULT_ANALYSIS_PLAN)
    protocol = dict(plan.get("protocol") or {})
    min_windows = int(protocol.get("minimum_valid_windows_per_trial", 15))
    min_trials = int(protocol.get("minimum_valid_trials_per_participant_condition", 2))
    required = [participant_column, "trial_id"]
    if "condition_id" not in frame.columns and not {"distance_m", "barrier_type"}.issubset(frame.columns):
        required.append("condition_id")
    missing = [column for column in required if column not in frame.columns]
    if missing:
        raise StatisticalInputError(
            "confirmatory aggregation requires " + ", ".join(missing)
        )
    columns = [participant_column, "trial_id", reference_column, estimate_column]
    for optional in ("condition_id", "distance_m", "barrier_type", "session_id", "timestamp_s", "model_family"):
        if optional in frame.columns and optional not in columns:
            columns.append(optional)
    work = frame.loc[:, columns].copy()
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
    trial_keys = [participant_column, "trial_id", "condition_id"]
    trial_rows = []
    for key, group in work.groupby(trial_keys, sort=True, dropna=False, observed=True):
        if len(group) < min_windows:
            continue
        row = {
            participant_column: key[0],
            "trial_id": key[1],
            "condition_id": key[2],
            reference_column: float(np.median(group[reference_column].to_numpy(dtype=float))),
            estimate_column: float(np.median(group[estimate_column].to_numpy(dtype=float))),
            "valid_window_count": int(len(group)),
        }
        trial_rows.append(row)
    trial_summary = pd.DataFrame(trial_rows)
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
        "finite_window_rows": int(len(work)),
        "eligible_trial_count": int(len(trial_summary)),
        "eligible_participant_condition_count": int(len(summary)),
        "minimum_valid_windows_per_trial": min_windows,
        "minimum_valid_trials_per_participant_condition": min_trials,
        "sequence": "window_median_then_trial_mean",
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
    primary_condition = str(
        plan.get("primary_condition")
        or dict(plan.get("primary") or {}).get("condition_id")
        or DEFAULT_ANALYSIS_PLAN["primary_condition"]
    )
    secondary_conditions = list(
        plan.get("secondary_conditions")
        or dict(plan.get("secondary") or {}).get("condition_ids")
        or DEFAULT_ANALYSIS_PLAN["secondary_conditions"]
    )
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
        participants = sorted({str(value) for value in block.loc[finite, participant_column]})
        base = {
            "condition_id": condition,
            "n_participants": len(participants),
            "status": "tested" if int(np.sum(finite)) >= 2 else "not_tested_insufficient_participants",
        }
        if int(np.sum(finite)) < 2:
            return base
        result = paired_tost(
            block.loc[finite, reference_column].to_numpy(dtype=float),
            block.loc[finite, estimate_column].to_numpy(dtype=float),
            lower=-margin,
            upper=margin,
            alpha=alpha,
        )
        return {**base, **result}

    primary = run(primary_condition)
    secondary = [run(condition) for condition in secondary_conditions]
    tested = [item for item in secondary if item.get("status") == "tested"]
    adjusted = holm_adjust([float(item["p_value"]) for item in tested]) if tested else []
    for item, adjusted_p in zip(tested, adjusted):
        item["p_value_holm"] = adjusted_p
        item["equivalent_holm"] = bool(adjusted_p < alpha)
        item["decision_holm"] = "equivalent" if item["equivalent_holm"] else "not_equivalent"
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
) -> dict[str, Any]:
    """Report output/missing-output denominators and optional exact false alarms."""

    ref = _as_1d(frame[reference_column], dtype=float)
    estimate = _as_1d(frame[estimate_column], dtype=float)
    if ref.size != estimate.size:
        raise StatisticalInputError("reference and estimate must have equal lengths")
    eligible = np.isfinite(ref)
    output = eligible & np.isfinite(estimate)
    denominator_unit = "row"
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
    if false_alarm_column and false_alarm_column in frame:
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
            alternative="greater",
        )
    return result


def mixed_effects_condition_analysis(
    frame: Any,
    *,
    reference_column: str,
    estimate_column: str,
    participant_column: str,
) -> dict[str, Any]:
    """Fit the optional participant-random-intercept distance/barrier model."""

    try:
        import pandas as pd
    except ImportError as exc:  # pragma: no cover
        raise StatisticalInputError("mixed-effects analysis requires pandas") from exc
    summary = _participant_condition_summary(
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
                    for key in ("condition_id", "status", "n_participants", "p_value", "p_value_holm", "equivalent_holm"):
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
    confirmatory: bool = False,
) -> dict[str, Any]:
    """Analyze a dataframe-like object and return JSON-safe report data."""

    if reference_column not in frame or estimate_column not in frame:
        raise StatisticalInputError(
            f"input is missing {reference_column!r} or {estimate_column!r}"
        )
    participant_present = bool(participant_column and participant_column in frame)
    report: dict[str, Any] = {
        "analysis_schema": "rvt-statistical-report-v1",
        "reference_column": reference_column,
        "estimate_column": estimate_column,
        "metrics": paired_metrics(frame[reference_column], frame[estimate_column]),
        "coverage": coverage_report(
            frame,
            reference_column=reference_column,
            estimate_column=estimate_column,
            false_alarm_column=false_alarm_column,
        ),
        "analysis_plan": dict(analysis_plan or DEFAULT_ANALYSIS_PLAN),
    }
    denominators = {"n_rows": int(len(frame))}
    for label, column in (
        ("n_participants", participant_column),
        ("n_trials", "trial_id"),
        ("n_sessions", "session_id"),
    ):
        if column and column in frame:
            denominators[label] = int(frame[column].dropna().astype(str).nunique())
    report["denominators"] = denominators
    plan = dict(analysis_plan or DEFAULT_ANALYSIS_PLAN)
    balanced_frame = None
    aggregation_diagnostics = None
    if participant_present:
        if confirmatory:
            balanced_frame, aggregation_diagnostics = aggregate_confirmatory_frame(
                frame,
                reference_column=reference_column,
                estimate_column=estimate_column,
                participant_column=str(participant_column),
                analysis_plan=plan,
            )
            report["aggregation"] = aggregation_diagnostics
        else:
            balanced_frame = _participant_condition_summary(
                frame,
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
    if tost_margin is not None:
        margin = abs(float(tost_margin))
        if not margin:
            raise StatisticalInputError("tost_margin must be non-zero")
        tost_frame = balanced_frame if balanced_frame is not None else frame
        report["tost"] = paired_tost(
            tost_frame[reference_column], tost_frame[estimate_column],
            lower=-margin, upper=margin, alpha=alpha,
        )
        if participant_present:
            report["condition_tost"] = _condition_tost(
                frame,
                reference_column=reference_column,
                estimate_column=estimate_column,
                participant_column=str(participant_column),
                margin=margin,
                alpha=alpha,
                analysis_plan=plan,
                summary_frame=balanced_frame,
            )
    if participant_present and (
        "condition_id" in frame
        or {"distance_m", "barrier_type"}.issubset(frame.columns)
    ):
        report["condition_effects"] = mixed_effects_condition_analysis(
            frame,
            reference_column=reference_column,
            estimate_column=estimate_column,
            participant_column=str(participant_column),
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
        "aggregation": "trial_then_participant_condition" if participant_present else "row_pair",
        "confirmatory_primary_condition": DEFAULT_ANALYSIS_PLAN["primary_condition"],
        "confirmatory_secondary_holm_conditions": DEFAULT_ANALYSIS_PLAN["secondary_conditions"],
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
