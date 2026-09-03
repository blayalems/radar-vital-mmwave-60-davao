"""Adversarial release-handshake and failed-start provenance regressions."""

import http.client
import json
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

from rvt_trainer.api.compatibility import (
    build_compatibility_handshake,
    validate_client_compatibility,
)
from rvt_trainer.monolith import (
    CONTROL_API_SCHEMA_VERSION,
    DASHBOARD_VERSION,
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    FIRMWARE_VERSION_EXPECTED,
    SESSION_MANIFEST_SCHEMA_VERSION,
    VERSION,
    _ControlServer,
    _SessionSupervisor,
)
from rvt_trainer.session.study_contract import (
    STUDY_SESSION_SCHEMA_VERSION,
    create_participant_profile,
)


def _matching_client() -> dict:
    return {
        "product_version": VERSION,
        "dashboard_version": DASHBOARD_VERSION,
        "serial_protocol": "v15.2",
        "serial_width_expected": EXPECTED_RADAR_LOG_COLUMN_COUNT,
        "schema_versions": {
            "control_api": CONTROL_API_SCHEMA_VERSION,
            "study_session": STUDY_SESSION_SCHEMA_VERSION,
        },
    }


def _server_handshake() -> dict:
    return build_compatibility_handshake(
        product_version=VERSION,
        trainer_version=VERSION,
        dashboard_version=DASHBOARD_VERSION,
        firmware_expected=FIRMWARE_VERSION_EXPECTED,
        schema_versions={
            "control_api": CONTROL_API_SCHEMA_VERSION,
            "study_session": STUDY_SESSION_SCHEMA_VERSION,
            "session_manifest": SESSION_MANIFEST_SCHEMA_VERSION,
        },
    )


def _validate(client: dict | None) -> dict:
    payload = {} if client is None else {"client_handshake": client}
    return validate_client_compatibility(
        payload,
        product_version=VERSION,
        dashboard_version=DASHBOARD_VERSION,
        control_api_schema=CONTROL_API_SCHEMA_VERSION,
        study_session_schema=STUDY_SESSION_SCHEMA_VERSION,
        session_manifest_schema=SESSION_MANIFEST_SCHEMA_VERSION,
    )


def _reason_codes(result: dict) -> set[str]:
    reasons = result.get("reasons", result.get("reason_codes", []))
    if not reasons and result.get("reason_code"):
        reasons = [result["reason_code"]]
    return {
        str(item.get("code") if isinstance(item, dict) else item)
        for item in reasons
    }


def _confirmatory() -> dict:
    return {
        "participant_id": "P-001",
        "trial_id": "P-001-d060-none-t1",
        "condition_id": "d060_none",
        "distance_m": 0.6,
        "barrier_type": "none",
        "trial_number": 1,
        "planned_duration_s": 150,
        "duration_s": 150,
        "study_classification": "confirmatory",
        "model_family": "gradient_boosting",
        "model_bundle": "gbr-locked-001",
    }


def test_server_handshake_advertises_release_and_frozen_protocol_identity():
    handshake = _server_handshake()
    identity = handshake["identity"]

    assert identity["product_version"] == VERSION
    assert identity["trainer_version"] == VERSION
    assert identity["dashboard_version"] == DASHBOARD_VERSION
    assert identity["firmware_expected"] == FIRMWARE_VERSION_EXPECTED
    assert identity["serial_protocol"] == "v15.2"
    assert identity["serial_width_expected"] == 222
    assert handshake["schema_versions"]["control_api"] == CONTROL_API_SCHEMA_VERSION
    assert (
        handshake["schema_versions"]["study_session"]
        == STUDY_SESSION_SCHEMA_VERSION
    )
    assert (
        handshake["schema_versions"]["session_manifest"]
        == SESSION_MANIFEST_SCHEMA_VERSION
    )


def test_matching_client_handshake_is_verified_and_compatible():
    result = _validate(_matching_client())

    assert result["decision"] == "compatible"
    assert result["verified"] is True
    assert result.get("blocks_start", False) is False
    assert _reason_codes(result) == set()


@pytest.mark.parametrize(
    ("patch", "reason_code"),
    [
        ({"product_version": "16.5.2"}, "PRODUCT_VERSION_MISMATCH"),
        ({"dashboard_version": "16.5.2"}, "DASHBOARD_VERSION_MISMATCH"),
        ({"serial_protocol": "v15.1"}, "SERIAL_PROTOCOL_MISMATCH"),
        ({"serial_width_expected": 219}, "SERIAL_WIDTH_MISMATCH"),
        (
            {
                "schema_versions": {
                    "control_api": "rvt-control-api-v11.0",
                    "study_session": STUDY_SESSION_SCHEMA_VERSION,
                }
            },
            "CONTROL_API_SCHEMA_MISMATCH",
        ),
        (
            {
                "schema_versions": {
                    "control_api": CONTROL_API_SCHEMA_VERSION,
                    "study_session": "rvt-study-session-v16.4.0",
                }
            },
            "STUDY_SESSION_SCHEMA_MISMATCH",
        ),
    ],
)
def test_any_supplied_release_or_protocol_mismatch_blocks_start(
    patch: dict,
    reason_code: str,
):
    result = _validate({**_matching_client(), **patch})

    assert result["decision"] == "incompatible"
    assert result["verified"] is False
    assert result["blocks_start"] is True
    assert reason_code in _reason_codes(result)


def test_legacy_client_without_handshake_is_explicitly_unverified_not_compatible():
    result = _validate(None)

    assert result["decision"] == "unverified"
    assert result["verified"] is False
    assert result.get("blocks_start", False) is False
    assert "CLIENT_METADATA_MISSING" in _reason_codes(result)
    assert result.get("confirmatory_eligible", False) is False


@patch("subprocess.Popen")
def test_legacy_confirmatory_request_is_downgraded_in_durable_manifest(
    mock_popen: MagicMock,
    tmp_path: Path,
):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    create_participant_profile(str(sessions_root), {})
    proc = MagicMock()
    proc.pid = 54101
    proc.poll.return_value = None

    def spawn(_argv, **_kwargs):
        (sessions_root / "s01" / "live_dashboard.json").write_text(
            json.dumps({"session_id": "s01", "radar": {}}),
            encoding="utf-8",
        )
        return proc

    mock_popen.side_effect = spawn
    supervisor = _SessionSupervisor(str(sessions_root))
    try:
        with patch(
            "rvt_trainer.session.supervisor.require_collection_authorization",
            return_value={"authorized": True},
        ):
            supervisor.start(timeout_s=0.2, **_confirmatory())
        manifest = json.loads(
            (sessions_root / "s01" / "session_manifest.json").read_text(
                encoding="utf-8"
            )
        )
    finally:
        supervisor._reset_runtime_state()

    assert manifest["study_classification"] == "confirmatory"
    assert manifest["confirmatory_eligible"] is False
    assert manifest["release_compatibility_state"] == "unverified"
    assert manifest["release_compatibility_verified"] is False
    assert manifest["client_compatibility"]["decision"] == "unverified"
    assert "CLIENT_METADATA_MISSING" in _reason_codes(
        manifest["client_compatibility"]
    )
    assert manifest["participant_id"] == "P-001"
    assert manifest["condition_id"] == "d060_none"
    assert manifest["model_family"] == "gradient_boosting"
    assert manifest["model_bundle"] == "gbr-locked-001"


def _request_json(
    control: _ControlServer,
    method: str,
    path: str,
    payload: dict,
) -> tuple[int, dict]:
    body = json.dumps(payload).encode("utf-8")
    conn = http.client.HTTPConnection(
        "127.0.0.1",
        control.httpd.server_port,
        timeout=5,
    )
    try:
        conn.request(
            method,
            path,
            body=body,
            headers={"Content-Type": "application/json"},
        )
        response = conn.getresponse()
        return response.status, json.loads(response.read())
    finally:
        conn.close()


@patch(
    "rvt_trainer.monolith._run_preflight_all",
    return_value={"checks": []},
)
def test_api_rejects_mismatched_dashboard_before_preflight(
    mock_preflight: MagicMock,
    tmp_path: Path,
):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    control = _ControlServer(
        "127.0.0.1",
        0,
        str(sessions_root),
        mock=True,
    )
    control.start()
    try:
        client = {**_matching_client(), "dashboard_version": "16.5.2"}
        with patch.object(
            control.supervisor,
            "start",
            return_value={"session_id": "must-not-start"},
        ) as mock_start:
            status, response = _request_json(
                control,
                "POST",
                "/api/session/start",
                {"client_handshake": client},
            )
    finally:
        control.stop()

    assert status == 409
    assert response["error"]["code"] == "RELEASE_COMPATIBILITY_MISMATCH"
    compatibility = response["error"]["details"]["compatibility"]
    assert compatibility["decision"] == "incompatible"
    assert "DASHBOARD_VERSION_MISMATCH" in _reason_codes(compatibility)
    assert any(
        "reload" in str(reason.get("remediation", "")).lower()
        for reason in compatibility["reasons"]
    )
    mock_preflight.assert_not_called()
    mock_start.assert_not_called()


@patch("subprocess.Popen", side_effect=OSError("capture executable denied"))
def test_process_launch_failure_preserves_full_study_release_and_failure_evidence(
    _mock_popen: MagicMock,
    tmp_path: Path,
):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    create_participant_profile(str(sessions_root), {})
    compatibility = _validate(_matching_client())
    supervisor = _SessionSupervisor(str(sessions_root))

    with pytest.raises(RuntimeError, match="capture executable denied"):
        with patch(
            "rvt_trainer.session.supervisor.require_collection_authorization",
            return_value={"authorized": True},
        ):
            supervisor.start(
                timeout_s=0.1,
                client_compatibility=compatibility,
                **_confirmatory(),
            )

    manifest = json.loads(
        (sessions_root / "s01" / "session_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["status"] == "failed_start"
    assert manifest["terminal"] is True
    assert manifest["ended_at"]
    assert manifest["participant_id"] == "P-001"
    assert manifest["trial_id"] == "P-001-d060-none-t1"
    assert manifest["condition_id"] == "d060_none"
    assert manifest["distance_m"] == 0.6
    assert manifest["barrier_type"] == "none"
    assert manifest["trial_number"] == 1
    assert manifest["product_version"] == VERSION
    assert manifest["trainer_version"] == VERSION
    assert manifest["dashboard_version"] == DASHBOARD_VERSION
    assert manifest["firmware_expected"] == FIRMWARE_VERSION_EXPECTED
    assert manifest["serial_protocol"] == "v15.2"
    assert manifest["serial_width_expected"] == 222
    assert manifest["model_family"] == "gradient_boosting"
    assert manifest["model_bundle"] == "gbr-locked-001"
    assert manifest["client_compatibility"]["decision"] == "compatible"
    assert manifest["failure"]["stage"] == "process_launch"
    assert manifest["failure"]["code"] == "SPAWN_ERROR"
    assert "capture executable denied" in manifest["failure"]["reason"]
    assert manifest["failure"]["failed_at"]
    assert not (sessions_root / "current_session.json").exists()
