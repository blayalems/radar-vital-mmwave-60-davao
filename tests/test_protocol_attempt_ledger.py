import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from rvt_trainer.monolith import _write_session_manifest, save_json, VERSION
from rvt_trainer.session.protocol_ledger import (
    append_session_attempt_event,
    canonical_logical_trial_id,
    completion_matrix,
    frozen_configuration_hash,
    initialize_session_attempt,
    register_protocol_attempt,
)
from rvt_trainer.session.study_evidence import objective_report, save_protocol
from rvt_trainer.session.study_objectives import study_objectives_payload
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


def _valid_no_subject_configuration() -> dict:
    return {
        "firmware": "v16.5.11",
        "artifact_rules": "frozen-rules-1",
        "alert_threshold": 0.80,
    }


def _lock_no_subject_protocol(
    sessions_root: Path,
    *,
    trial_count: int = 72,
    configuration: dict | None = None,
) -> str:
    configuration = (
        _valid_no_subject_configuration()
        if configuration is None
        else configuration
    )
    save_protocol(
        str(sessions_root),
        {
            "state": "locked",
            "no_subject": {
                "trial_count": trial_count,
                "planned_duration_s": 150,
                "frozen_configuration": configuration,
            },
        },
        actor="OP-001",
    )
    return frozen_configuration_hash(configuration)


def _register_no_subject_capture(
    sessions_root: Path,
    index: int,
    configuration_hash: str,
    *,
    false_alarm_count: int = 0,
    start_offset_s: int | None = None,
    manifest_updates: dict | None = None,
):
    session_id = f"no-subject-{index:03d}"
    attempt_id = f"AT-no-subject-{index:03d}"
    session = sessions_root / session_id
    session.mkdir()
    offset_s = (index - 1) * 180 if start_offset_s is None else start_offset_s
    captured_at = datetime(2026, 8, 10, tzinfo=timezone.utc) + timedelta(
        seconds=offset_s
    )
    ended_at = captured_at + timedelta(seconds=150)
    captured_at_text = captured_at.isoformat().replace("+00:00", "Z")
    ended_at_text = ended_at.isoformat().replace("+00:00", "Z")
    manifest = {
        "schema_version": "rvt-session-manifest-v1",
        "session_id": session_id,
        "status": "completed",
        "terminal": True,
        "attempt_id": attempt_id,
        "attempt_type": "no_subject",
        "participant_id": None,
        "duration_s": 150,
        "frozen_configuration_hash": configuration_hash,
        "false_alarm_count": false_alarm_count,
        "product_version": "16.5.11",
        "trainer_version": "16.5.11",
        "dashboard_version": "16.5.11",
        "firmware_expected": "v16.5.11",
        "firmware_observed": "v16.5.11",
        "source_commit": "a" * 40,
        "ended_at": ended_at_text,
        "capture_provenance": {
            "captured_at": captured_at_text,
            "source": "session_start",
            "attempt_id": attempt_id,
            "product_version": "16.5.11",
            "trainer_version": "16.5.11",
            "dashboard_version": "16.5.11",
            "firmware_expected": "v16.5.11",
            "firmware_observed": "v16.5.11",
            "source_commit": "a" * 40,
        },
    }
    manifest.update(manifest_updates or {})
    (session / "session_manifest.json").write_text(
        json.dumps(manifest),
        encoding="utf-8",
    )
    return register_protocol_attempt(
        str(sessions_root),
        {
            "attempt_id": attempt_id,
            "attempt_type": "no_subject",
            "condition_id": "d060_none",
            "trial_number": index,
            "status": "completed",
            "session_id": session_id,
            "duration_s": 150,
            "frozen_configuration_hash": configuration_hash,
            "false_alarm_count": false_alarm_count,
        },
    )


def test_canonical_logical_trial_key_is_stable_and_condition_bound():
    assert canonical_logical_trial_id("p-001", "d060_none", 1) == "P-001-d060-none-t1"
    with pytest.raises(ValueError):
        canonical_logical_trial_id("P-001", "invalid", 1)


@pytest.mark.parametrize(
    ("trial_count", "configuration", "message"),
    [
        (71, _valid_no_subject_configuration(), "exactly 72 no-subject trials"),
        (
            72,
            {"artifact_rules": "rules-1", "alert_threshold": 0.8},
            "firmware must use",
        ),
        (
            72,
            {"firmware": "v16.5.11", "alert_threshold": 0.8},
            "artifact_rules identity is required",
        ),
        (
            72,
            {"firmware": "v16.5.11", "artifact_rules": "rules-1"},
            "alert_threshold must be finite",
        ),
    ],
)
def test_protocol_lock_rejects_incomplete_no_subject_evidence_contract(
    tmp_path: Path,
    trial_count: int,
    configuration: dict,
    message: str,
):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    with pytest.raises(ValueError, match=message):
        _lock_no_subject_protocol(
            sessions_root,
            trial_count=trial_count,
            configuration=configuration,
        )


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


def test_no_subject_qualification_requires_protocol_and_capture_bound_evidence(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    configuration_hash = _lock_no_subject_protocol(sessions_root)
    _register_no_subject_capture(sessions_root, 1, configuration_hash)
    matrix = completion_matrix(str(sessions_root))
    assert matrix["no_subject_qualified_count"] == 1
    assert matrix["no_subject_unqualified_count"] == 0


@pytest.mark.parametrize(
    "manifest_updates",
    [
        {"attempt_type": "subject"},
        {"duration_s": 149},
        {"frozen_configuration_hash": "b" * 64},
        {"false_alarm_count": 1},
        {"firmware_observed": None},
        {"firmware_observed": "v16.5.10"},
        {"capture_provenance": None},
    ],
)
def test_no_subject_qualification_rejects_uncorroborated_evidence(
    tmp_path: Path,
    manifest_updates: dict,
):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    configuration_hash = _lock_no_subject_protocol(sessions_root)
    _register_no_subject_capture(
        sessions_root,
        1,
        configuration_hash,
        manifest_updates=manifest_updates,
    )
    matrix = completion_matrix(str(sessions_root))
    assert matrix["no_subject_attempt_count"] == 1
    assert matrix["no_subject_qualified_count"] == 0
    assert matrix["no_subject_unqualified_count"] == 1


def test_objective_three_report_is_built_directly_from_72_qualified_trials(
    tmp_path: Path,
):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    configuration_hash = _lock_no_subject_protocol(sessions_root)
    for index in range(1, 73):
        _register_no_subject_capture(sessions_root, index, configuration_hash)

    result = objective_report(
        "objective_3_false_alarm",
        objectives=study_objectives_payload(),
        sessions_root=str(sessions_root),
    )

    assert result["status"] == "ready"
    assert result["exclusions"] == []
    assert result["report"]["planned_trial_count"] == 72
    assert result["report"]["qualified_trial_count"] == 72
    assert result["report"]["false_alarm_trial_count"] == 0
    assert result["report"]["observed_false_alarm_rate"] == 0.0
    assert result["report"]["confidence_interval_95"]["method"] == "clopper_pearson_exact"
    assert result["report"]["confidence_interval_95"]["low"] == 0.0
    assert 0.0 < result["report"]["confidence_interval_95"]["high"] < 0.05
    exact_test = result["report"]["exact_binomial_test"]
    assert exact_test["alternative"] == "less"
    assert exact_test["null_proportion"] == 0.05
    assert exact_test["p_value"] < 0.05
    assert exact_test["reject_null"] is True
    assert result["provenance"]["frozen_configuration_hash"] == configuration_hash
    assert result["provenance"]["no_subject_attempt_count"] == 72
    assert result["provenance"]["capture_interval_count"] == 72


def test_objective_three_rejects_overlapping_control_trial_intervals(
    tmp_path: Path,
):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    configuration_hash = _lock_no_subject_protocol(sessions_root)
    for index in range(1, 73):
        _register_no_subject_capture(
            sessions_root,
            index,
            configuration_hash,
            start_offset_s=0,
        )

    result = objective_report(
        "objective_3_false_alarm",
        objectives=study_objectives_payload(),
        sessions_root=str(sessions_root),
    )

    assert result["status"] == "inconclusive"
    assert result["report"] is None
    assert result["exclusions"][0]["reason"] == (
        "overlapping_no_subject_capture_intervals"
    )


def test_objective_three_reports_non_rejection_with_one_false_alarm(
    tmp_path: Path,
):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    configuration_hash = _lock_no_subject_protocol(sessions_root)
    for index in range(1, 73):
        _register_no_subject_capture(
            sessions_root,
            index,
            configuration_hash,
            false_alarm_count=1 if index == 1 else 0,
        )

    result = objective_report(
        "objective_3_false_alarm",
        objectives=study_objectives_payload(),
        sessions_root=str(sessions_root),
    )

    assert result["status"] == "ready"
    assert result["report"]["false_alarm_trial_count"] == 1
    assert result["report"]["observed_false_alarm_rate"] == pytest.approx(1 / 72)
    exact_test = result["report"]["exact_binomial_test"]
    assert exact_test["p_value"] == pytest.approx(0.11923050191296)
    assert exact_test["reject_null"] is False
    assert exact_test["decision"] == (
        "does_not_support_false_alarm_rate_below_threshold"
    )


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
