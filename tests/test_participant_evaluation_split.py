import json
import pandas as pd
import pytest

from rvt_trainer.evaluation_split import (
    CONFIRMATORY_CONDITIONS,
    attach_study_metadata,
    participant_split,
)


def _study_frame(participants=("P-001", "P-002", "P-003"), complete=True):
    rows = []
    conditions = sorted(CONFIRMATORY_CONDITIONS)
    for participant in participants:
        for condition in conditions:
            distance_m = int(condition[1:4]) / 100.0
            barrier = condition.split("_", 1)[1]
            trials = (1, 2, 3) if complete else (1,)
            for trial in trials:
                session = f"{participant}-{condition}-t{trial}"
                for timestamp in range(4):
                    rows.append(
                        {
                            "session_id": session,
                            "timestamp_s": float(timestamp),
                            "participant_id": participant,
                            "trial_id": session,
                            "condition_id": condition,
                            "distance_m": distance_m,
                            "barrier_type": barrier,
                            "trial_number": trial,
                            "planned_duration_s": 150,
                            "study_classification": "confirmatory",
                            "confirmatory_eligible": True,
                        }
                    )
    return pd.DataFrame(rows)


def test_participant_split_is_disjoint_deterministic_and_condition_stratified():
    frame = _study_frame()
    first = participant_split(
        frame,
        test_ratio=0.2,
        early_stop_ratio=0.2,
        three_way=True,
        random_state=42,
        require_confirmatory=True,
    )
    second = participant_split(
        frame.sample(frac=1.0, random_state=7),
        test_ratio=0.2,
        early_stop_ratio=0.2,
        three_way=True,
        random_state=42,
        require_confirmatory=True,
    )

    train, stop, test, ledger = first
    _, _, _, repeated_ledger = second
    groups = [
        set(train["participant_id"]),
        set(stop["participant_id"]),
        set(test["participant_id"]),
    ]
    assert all(groups)
    assert not (groups[0] & groups[1] or groups[0] & groups[2] or groups[1] & groups[2])
    assert ledger["participant_overlap"] == {
        "train_early_stop": [],
        "train_test": [],
        "early_stop_test": [],
    }
    assert ledger["confirmatory"]["eligible"] is True
    assert ledger["model_families"] == ["gradient_boosting", "cnn_1d"]
    assert ledger["sha256"] == repeated_ledger["sha256"]
    for name in ("train", "early_stop", "test"):
        assert set(ledger[name]["condition_ids"]) == CONFIRMATORY_CONDITIONS


def test_confirmatory_split_fails_closed_without_all_three_trials():
    with pytest.raises(ValueError, match="lacks 12 of 18"):
        participant_split(
            _study_frame(complete=False),
            test_ratio=0.2,
            early_stop_ratio=0.2,
            three_way=True,
            random_state=42,
            require_confirmatory=True,
        )


def test_confirmatory_split_requires_independent_early_stop_group():
    with pytest.raises(ValueError, match="requires --three-way-split"):
        participant_split(
            _study_frame(),
            test_ratio=0.2,
            early_stop_ratio=0.2,
            three_way=False,
            random_state=42,
            require_confirmatory=True,
        )


@pytest.mark.parametrize("bad_value", ["false", 0, 1])
def test_confirmatory_eligibility_requires_a_json_boolean(bad_value):
    frame = _study_frame()
    frame["confirmatory_eligible"] = bad_value
    with pytest.raises(ValueError, match="must be boolean"):
        participant_split(
            frame,
            test_ratio=0.2,
            early_stop_ratio=0.2,
            three_way=True,
            random_state=42,
            require_confirmatory=True,
        )


def test_participant_split_ledger_records_temporal_session_boundaries():
    _, _, _, ledger = participant_split(
        _study_frame(),
        test_ratio=0.2,
        early_stop_ratio=0.2,
        three_way=True,
        random_state=11,
        require_confirmatory=False,
    )
    sessions = [
        session
        for name in ("train", "early_stop", "test")
        for session in ledger[name]["sessions"]
    ]
    assert sessions
    assert all(session["timestamp_min_s"] == 0.0 for session in sessions)
    assert all(session["timestamp_max_s"] == 3.0 for session in sessions)
    assert "bounded by session_id" in ledger["temporal_boundary"]


def test_manifest_metadata_is_attached_without_model_dependencies(tmp_path):
    session_dir = tmp_path / "sessions" / "session-1"
    session_dir.mkdir(parents=True)
    radar = session_dir / "radar.csv"
    radar.write_text("timestamp_s\n0\n", encoding="utf-8")
    manifest = {
        "participant_id": "P-001",
        "trial_id": "P-001-d060_none-t1",
        "condition_id": "d060_none",
        "distance_m": 0.6,
        "barrier_type": "none",
        "trial_number": 1,
        "planned_duration_s": 150,
        "study_classification": "confirmatory",
        "confirmatory_eligible": True,
    }
    (session_dir / "session_manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8"
    )

    frame = attach_study_metadata(
        pd.DataFrame({"session_id": ["s"], "timestamp_s": [0.0]}), str(radar)
    )

    assert frame.loc[0, "participant_id"] == "P-001"
    assert frame.loc[0, "condition_id"] == "d060_none"
