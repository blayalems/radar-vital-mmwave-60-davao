import json
from datetime import datetime, timezone
from pathlib import Path

import pytest

from rvt_trainer.session.collection_authorization import (
    CollectionAuthorizationError,
    collection_authorization_status,
    require_collection_authorization,
)


def test_collection_authorization_is_absent_and_fail_closed_by_default(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()

    status = collection_authorization_status(str(sessions_root))

    assert status["authorized"] is False
    assert status["plan_status"] == "draft"
    assert "authorization_record_missing_or_invalid" in status["blockers"]
    assert "protocol_configuration_not_locked" in status["blockers"]
    assert "statistical_analysis_plan_not_approved" in status["blockers"]
    with pytest.raises(CollectionAuthorizationError) as exc:
        require_collection_authorization(str(sessions_root))
    assert exc.value.code == "STUDY_COLLECTION_NOT_AUTHORIZED"


def test_protocol_lock_alone_never_authorizes_collection(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    (sessions_root / "study_protocol.json").write_text(
        json.dumps(
            {
                "schema_version": "rvt-study-protocol-v2",
                "protocol_id": "RVT-THESIS-16.5.9",
                "state": "locked",
                "conditions": [],
                "no_subject": {"trial_count": 72, "planned_duration_s": 150},
            }
        ),
        encoding="utf-8",
    )

    status = collection_authorization_status(
        str(sessions_root), now=datetime(2026, 8, 19, tzinfo=timezone.utc)
    )

    assert status["protocol_state"] == "locked"
    assert status["authorized"] is False
    assert "protocol_configuration_not_locked" not in status["blockers"]
    assert "authorization_record_missing_or_invalid" in status["blockers"]
    assert "statistical_analysis_plan_not_approved" in status["blockers"]


def test_unresolved_withdrawal_is_an_independent_authorization_blocker(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    (tmp_path / "participant_profiles.json").write_text(
        json.dumps(
            {
                "profiles": {
                    "P-001": {"participant_id": "P-001", "status": "withdrawn"}
                }
            }
        ),
        encoding="utf-8",
    )

    status = collection_authorization_status(str(sessions_root))

    assert status["unresolved_withdrawal_count"] == 1
    assert "unresolved_participant_withdrawal" in status["blockers"]
