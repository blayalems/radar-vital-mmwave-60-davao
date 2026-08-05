import json
from pathlib import Path

import pytest

from rvt_trainer.monolith import _write_session_manifest, save_json, VERSION
from rvt_trainer.session.protocol_ledger import (
    append_session_attempt_event,
    canonical_logical_trial_id,
    completion_matrix,
    initialize_session_attempt,
    register_protocol_attempt,
)
from rvt_trainer.session.study_contract import (
    create_participant_profile,
    update_participant_status,
    validate_study_assignment,
)


def _manifest(participant_id="P-001", trial_number=1):
    return {
        "participant_id": participant_id,
        "trial_id": f"P-001-d060-none-t{trial_number}",
        "logical_trial_id": f"P-001-d060-none-t{trial_number}",
        "condition_id": "d060_none",
        "trial_number": trial_number,
        "attempt_type": "subject",
        "attempt_id": "AT-fixed",
        "product_version": "16.5.9",
        "study_session_schema_version": "rvt-study-session-v16.5.9",
    }


def test_canonical_logical_trial_key_is_stable_and_condition_bound():
    assert canonical_logical_trial_id("p-001", "d060_none", 1) == "P-001-d060-none-t1"
    with pytest.raises(ValueError):
        canonical_logical_trial_id("P-001", "invalid", 1)


def test_session_ledger_is_append_only_and_terminal(tmp_path: Path):
    session = tmp_path / "sessions" / "s01"
    session.mkdir(parents=True)
    initialize_session_attempt(str(session), _manifest())
    append_session_attempt_event(str(session), "collecting")
    record = append_session_attempt_event(str(session), "completed", reason="done")
    assert record["terminal"] is True
    assert [event["status"] for event in record["events"]] == [
        "allocated",
        "collecting",
        "completed",
    ]
    with pytest.raises(ValueError, match="terminal"):
        append_session_attempt_event(str(session), "aborted")


def test_no_subject_attempt_is_explicit_and_counted_in_matrix(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    register_protocol_attempt(
        str(sessions_root),
        {
            "attempt_type": "no_subject",
            "condition_id": "d060_none",
            "trial_number": 1,
            "status": "completed",
        },
    )
    matrix = completion_matrix(str(sessions_root))
    assert matrix["no_subject_attempt_count"] == 1
    assert matrix["no_subject_expected"] == 72


def test_no_subject_qualification_requires_session_backed_duration_and_hash(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    session = sessions_root / "no-subject-001"
    session.mkdir()
    (session / "session_manifest.json").write_text(
        json.dumps({"session_id": session.name, "status": "completed"}),
        encoding="utf-8",
    )
    register_protocol_attempt(
        str(sessions_root),
        {
            "attempt_type": "no_subject",
            "condition_id": "d060_none",
            "trial_number": 1,
            "status": "completed",
            "session_id": session.name,
            "duration_s": 150,
            "frozen_configuration_hash": "a" * 64,
            "false_alarm_count": 0,
        },
    )
    matrix = completion_matrix(str(sessions_root))
    assert matrix["no_subject_qualified_count"] == 1
    assert matrix["no_subject_unqualified_count"] == 0


def test_participant_status_history_retains_withdrawal_and_reversal(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    create_participant_profile(str(sessions_root), {})
    update_participant_status(
        str(sessions_root),
        "P-001",
        "withdrawn",
        actor="OP-001",
        reason="participant request",
        consent_revision="2026-06-12.1",
    )
    profile = update_participant_status(
        str(sessions_root),
        "P-001",
        "active",
        actor="OP-001",
        reason="withdrawal reversal approved",
        consent_revision="2026-06-12.1",
    )
    assert profile["status"] == "active"
    assert [(row["from_status"], row["to_status"]) for row in profile["status_history"]] == [
        (None, "active"),
        ("active", "withdrawn"),
        ("withdrawn", "active"),
    ]
    assert profile["status_history"][1]["reason"] == "participant request"


def test_manifest_reanalysis_keeps_capture_identity_and_appends_analysis_run(tmp_path: Path):
    session = tmp_path / "sessions" / "s01"
    analysis = session / "analysis"
    analysis.mkdir(parents=True)
    radar = session / "radar.csv"
    reference = session / "ref.csv"
    radar.write_text("x\n1\n", encoding="utf-8")
    reference.write_text("x\n1\n", encoding="utf-8")
    save_json(
        {
            **_manifest(),
            "generated_at": "2026-08-01T00:00:00Z",
            "product_version": "16.5.8",
            "capture_provenance": {
                "captured_at": "2026-08-01T00:00:00Z",
                "product_version": "16.5.8",
                "trainer_version": "16.5.8",
                "dashboard_version": "16.5.8",
                "firmware_expected": "v16.5.8",
                "source_commit": "capture-commit",
                "attempt_id": "AT-fixed",
            },
            "analysis_runs": [],
        },
        str(session / "session_manifest.json"),
    )
    manifest = _write_session_manifest(
        str(session),
        [str(radar)],
        [str(reference)],
        str(analysis),
        {"version": "v16.5.9", "contract_length": 222},
    )
    assert manifest["product_version"] == "16.5.8"
    assert manifest["capture_provenance"]["product_version"] == "16.5.8"
    assert manifest["capture_provenance"]["source_commit"] == "capture-commit"
    assert manifest["analysis_runs"]
    assert manifest["analysis_runs"][-1]["trainer_version"] == VERSION
    ledger = json.loads((session / "protocol_attempt.json").read_text(encoding="utf-8"))
    assert ledger["attempt_id"] == "AT-fixed"
    assert ledger["status"] == "completed"


def test_confirmatory_assignment_exposes_logical_trial_id(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    create_participant_profile(str(sessions_root), {})
    assignment = validate_study_assignment(
        {
            "participant_id": "P-001",
            "trial_id": "P-001-d060-none-t1",
            "condition_id": "d060_none",
            "distance_m": 0.6,
            "barrier_type": "none",
            "trial_number": 1,
            "planned_duration_s": 150,
            "duration_s": 150,
            "study_classification": "confirmatory",
        },
        sessions_root=str(sessions_root),
    )
    assert assignment["logical_trial_id"] == "P-001-d060-none-t1"
    assert assignment["attempt_type"] == "subject"
