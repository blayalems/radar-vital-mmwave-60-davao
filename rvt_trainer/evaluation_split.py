"""Participant-aware, model-family-neutral evaluation split contracts."""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple

import pandas as pd
import numpy as np


PARTICIPANT_PATTERN = re.compile(r"^P-\d{3}$")
CONFIRMATORY_DISTANCES_M = (0.6, 0.8, 1.0)
CONFIRMATORY_BARRIERS = ("none", "cardboard")
CONFIRMATORY_CONDITIONS = frozenset(
    f"d{int(distance * 100):03d}_{barrier}"
    for distance in CONFIRMATORY_DISTANCES_M
    for barrier in CONFIRMATORY_BARRIERS
)
STUDY_COLUMNS = (
    "participant_id",
    "trial_id",
    "condition_id",
    "distance_m",
    "barrier_type",
    "trial_number",
    "planned_duration_s",
    "study_classification",
    "confirmatory_eligible",
)


def _stable_rank(value: object, random_state: int) -> str:
    return hashlib.sha256(f"{int(random_state)}:{value}".encode("utf-8")).hexdigest()


def _canonical_condition(distance_m: float, barrier_type: str) -> str:
    return f"d{int(round(float(distance_m) * 100)):03d}_{barrier_type}"


def load_study_metadata(radar_path: str) -> Optional[Dict[str, object]]:
    """Load the nearest session manifest without guessing study identity."""

    path = Path(radar_path).resolve()
    candidates = [path.parent / "session_manifest.json"]
    candidates.extend(parent / "session_manifest.json" for parent in path.parents[1:4])
    for candidate in candidates:
        if not candidate.is_file():
            continue
        try:
            payload = json.loads(candidate.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"Study manifest is unreadable: {candidate}: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError(f"Study manifest must contain a JSON object: {candidate}")
        return {column: payload.get(column) for column in STUDY_COLUMNS}
    return None


def attach_study_metadata(frame: pd.DataFrame, radar_path: str) -> pd.DataFrame:
    result = frame.copy()
    metadata = load_study_metadata(radar_path)
    for column in STUDY_COLUMNS:
        result[column] = metadata.get(column) if metadata is not None else None
    return result


def _one_value(frame: pd.DataFrame, column: str, session_id: object) -> object:
    values = frame[column].dropna().unique().tolist() if column in frame else []
    if len(values) != 1:
        raise ValueError(
            f"Session {session_id!r} must have exactly one {column}; found {values!r}"
        )
    return values[0]


def _session_records(df: pd.DataFrame) -> list[Dict[str, object]]:
    records = []
    if "session_id" not in df:
        raise ValueError("Participant-aware evaluation requires session_id.")
    for session_id, frame in df.groupby("session_id", sort=False):
        participant_id = str(_one_value(frame, "participant_id", session_id))
        if not PARTICIPANT_PATTERN.fullmatch(participant_id):
            raise ValueError(
                f"Session {session_id!r} has invalid participant_id {participant_id!r}"
            )
        distance_raw = _one_value(frame, "distance_m", session_id)
        if isinstance(distance_raw, bool):
            raise ValueError(f"Session {session_id!r} has a non-numeric distance_m")
        distance_m = float(distance_raw)
        if not math.isfinite(distance_m):
            raise ValueError(f"Session {session_id!r} has a non-finite distance_m")
        barrier_type = str(_one_value(frame, "barrier_type", session_id))
        condition_id = str(_one_value(frame, "condition_id", session_id))
        if condition_id != _canonical_condition(distance_m, barrier_type):
            raise ValueError(
                f"Session {session_id!r} condition_id {condition_id!r} does not match "
                f"distance/barrier {_canonical_condition(distance_m, barrier_type)!r}"
            )
        trial_id = str(_one_value(frame, "trial_id", session_id)).strip()
        if not trial_id:
            raise ValueError(f"Session {session_id!r} has an empty trial_id")
        trial_raw = _one_value(frame, "trial_number", session_id)
        duration_raw = _one_value(frame, "planned_duration_s", session_id)
        if (
            isinstance(trial_raw, bool)
            or not float(trial_raw).is_integer()
            or isinstance(duration_raw, bool)
            or not float(duration_raw).is_integer()
        ):
            raise ValueError(
                f"Session {session_id!r} trial_number and planned_duration_s "
                "must be integers"
            )
        classification = str(
            _one_value(frame, "study_classification", session_id)
        )
        if classification not in {"confirmatory", "exploratory"}:
            raise ValueError(
                f"Session {session_id!r} has invalid study_classification "
                f"{classification!r}"
            )
        eligible_raw = _one_value(frame, "confirmatory_eligible", session_id)
        if not isinstance(eligible_raw, (bool, np.bool_)):
            raise ValueError(
                f"Session {session_id!r} confirmatory_eligible must be boolean"
            )
        timestamp_min = float(frame["timestamp_s"].min())
        timestamp_max = float(frame["timestamp_s"].max())
        if not math.isfinite(timestamp_min) or not math.isfinite(timestamp_max):
            raise ValueError(f"Session {session_id!r} has non-finite timestamps")
        records.append(
            {
                "session_id": str(session_id),
                "participant_id": participant_id,
                "trial_id": trial_id,
                "condition_id": condition_id,
                "distance_m": distance_m,
                "barrier_type": barrier_type,
                "trial_number": int(trial_raw),
                "planned_duration_s": int(duration_raw),
                "study_classification": classification,
                "confirmatory_eligible": bool(eligible_raw),
                "rows": int(len(frame)),
                "timestamp_min_s": timestamp_min,
                "timestamp_max_s": timestamp_max,
            }
        )
    return records


def _condition_sets(
    records: Sequence[Mapping[str, object]],
) -> Dict[str, frozenset[str]]:
    result: Dict[str, set[str]] = {}
    for record in records:
        result.setdefault(str(record["participant_id"]), set()).add(
            str(record["condition_id"])
        )
    return {participant: frozenset(conditions) for participant, conditions in result.items()}


def _select_holdout(
    candidates: Iterable[str],
    count: int,
    condition_sets: Mapping[str, frozenset[str]],
    random_state: int,
) -> list[str]:
    remaining = set(candidates)
    selected: list[str] = []
    covered: set[str] = set()
    condition_frequency: Dict[str, int] = {}
    for participant in remaining:
        for condition in condition_sets[participant]:
            condition_frequency[condition] = condition_frequency.get(condition, 0) + 1
    while remaining and len(selected) < count:
        participant = min(
            remaining,
            key=lambda item: (
                -len(condition_sets[item] - covered),
                -sum(
                    1.0 / condition_frequency[condition]
                    for condition in condition_sets[item]
                ),
                _stable_rank(item, random_state + len(selected)),
            ),
        )
        selected.append(participant)
        covered.update(condition_sets[participant])
        remaining.remove(participant)
    return selected


def _split_block(
    df: pd.DataFrame,
    records: Sequence[Mapping[str, object]],
    participants: Sequence[str],
) -> Tuple[pd.DataFrame, Dict[str, object]]:
    participant_set = set(participants)
    block = df[df["participant_id"].isin(participant_set)].copy().reset_index(drop=True)
    block_records = [
        dict(record)
        for record in records
        if str(record["participant_id"]) in participant_set
    ]
    block_records.sort(key=lambda record: str(record["session_id"]))
    return block, {
        "participant_ids": sorted(participant_set),
        "session_ids": sorted(str(record["session_id"]) for record in block_records),
        "condition_ids": sorted(
            {str(record["condition_id"]) for record in block_records}
        ),
        "distance_m": sorted({float(record["distance_m"]) for record in block_records}),
        "barrier_types": sorted(
            {str(record["barrier_type"]) for record in block_records}
        ),
        "rows": int(len(block)),
        "sessions": block_records,
    }


def participant_split(
    df: pd.DataFrame,
    *,
    test_ratio: float,
    early_stop_ratio: float,
    three_way: bool,
    random_state: int,
    require_confirmatory: bool,
) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame, Dict[str, object]]:
    """Split by participant and emit one ledger shared by GBR and 1-D CNN."""

    records = _session_records(df)
    participants = sorted({str(record["participant_id"]) for record in records})
    minimum = 3 if three_way else 2
    if len(participants) < minimum:
        raise ValueError(
            f"Participant-aware {'three-way' if three_way else 'two-way'} evaluation "
            f"requires at least {minimum} participants; found {len(participants)}."
        )

    condition_sets = _condition_sets(records)
    test_count = max(1, min(len(participants) - (2 if three_way else 1),
                            int(math.ceil(len(participants) * float(test_ratio)))))
    test_participants = _select_holdout(
        participants, test_count, condition_sets, random_state
    )
    remaining = [item for item in participants if item not in set(test_participants)]
    if three_way:
        stop_count = max(
            1,
            min(
                len(remaining) - 1,
                int(math.ceil(len(remaining) * float(early_stop_ratio))),
            ),
        )
        stop_participants = _select_holdout(
            remaining, stop_count, condition_sets, random_state + 10_000
        )
    else:
        stop_participants = list(test_participants)
    train_participants = [
        item
        for item in remaining
        if item not in set(stop_participants)
    ]

    train_df, train_info = _split_block(
        df, records, train_participants
    )
    stop_df, stop_info = _split_block(
        df, records, stop_participants
    )
    test_df, test_info = _split_block(
        df, records, test_participants
    )
    reasons = []
    if require_confirmatory and not three_way:
        reasons.append("confirmatory evaluation requires --three-way-split")
    for record in records:
        if record["study_classification"] != "confirmatory":
            reasons.append(
                f"{record['session_id']} is not classified confirmatory"
            )
        if not record["confirmatory_eligible"]:
            reasons.append(
                f"{record['session_id']} is not confirmatory eligible"
            )
        if float(record["distance_m"]) not in CONFIRMATORY_DISTANCES_M:
            reasons.append(f"{record['session_id']} has a non-protocol distance")
        if record["barrier_type"] not in CONFIRMATORY_BARRIERS:
            reasons.append(f"{record['session_id']} has a non-protocol barrier")
        if int(record["trial_number"]) not in (1, 2, 3):
            reasons.append(f"{record['session_id']} has a non-protocol trial")
        if int(record["planned_duration_s"]) != 150:
            reasons.append(f"{record['session_id']} is not a 150-second trial")
    trial_ids = [str(record["trial_id"]) for record in records]
    if len(set(trial_ids)) != len(trial_ids):
        reasons.append("trial_id values are not unique across input sessions")
    required_cells = {
        (condition, trial_number)
        for condition in CONFIRMATORY_CONDITIONS
        for trial_number in (1, 2, 3)
    }
    for participant in participants:
        observed_cells = {
            (str(record["condition_id"]), int(record["trial_number"]))
            for record in records
            if str(record["participant_id"]) == participant
        }
        missing_cells = required_cells - observed_cells
        if require_confirmatory and missing_cells:
            reasons.append(
                f"{participant} lacks {len(missing_cells)} of 18 confirmatory "
                "condition/trial cells"
            )
    for name, block in (
        ("train", train_info),
        ("early_stop", stop_info),
        ("test", test_info),
    ):
        missing = CONFIRMATORY_CONDITIONS - set(block["condition_ids"])
        if require_confirmatory and missing:
            reasons.append(
                f"{name} lacks confirmatory condition(s): {', '.join(sorted(missing))}"
            )
    participant_overlap = {
        "train_early_stop": sorted(
            set(train_participants) & set(stop_participants)
        ),
        "train_test": sorted(set(train_participants) & set(test_participants)),
        "early_stop_test": sorted(
            set(stop_participants) & set(test_participants)
        ),
    }
    if any(participant_overlap.values()):
        reasons.append("participant groups overlap across evaluation partitions")
    reasons = list(dict.fromkeys(reasons))
    ledger = {
        "schema_version": "rvt-evaluation-split-v1",
        "mode": "participant_group_three_way" if three_way else "participant_group_two_way",
        "assignment_algorithm": "condition_coverage_greedy_sha256_v1",
        "random_state": int(random_state),
        "model_families": ["gradient_boosting", "cnn_1d"],
        "temporal_boundary": (
            "feature lags, rolling statistics, and CNN windows are bounded by session_id; "
            "sessions never cross participant partitions"
        ),
        "stratification": ["condition_id", "distance_m", "barrier_type"],
        "participant_overlap": participant_overlap,
        "train": train_info,
        "early_stop": stop_info,
        "test": test_info,
        "confirmatory": {
            "requested": bool(require_confirmatory),
            "eligible": bool(require_confirmatory and not reasons),
            "required_conditions": sorted(CONFIRMATORY_CONDITIONS),
            "required_trials_per_condition": [1, 2, 3],
            "required_sessions_per_participant": 18,
            "reasons": reasons,
        },
    }
    encoded = json.dumps(ledger, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ledger["sha256"] = hashlib.sha256(encoded).hexdigest()
    if require_confirmatory and reasons:
        raise ValueError(
            "Confirmatory evaluation is ineligible: " + "; ".join(reasons)
        )
    return train_df, stop_df, test_df, ledger


def has_any_study_metadata(df: pd.DataFrame) -> bool:
    return "participant_id" in df and bool(df["participant_id"].notna().any())


def has_complete_study_metadata(df: pd.DataFrame) -> bool:
    return all(column in df and not df[column].isna().any() for column in STUDY_COLUMNS)


__all__ = [
    "CONFIRMATORY_CONDITIONS",
    "STUDY_COLUMNS",
    "attach_study_metadata",
    "has_any_study_metadata",
    "has_complete_study_metadata",
    "load_study_metadata",
    "participant_split",
]
