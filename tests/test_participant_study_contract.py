import json
import http.client
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

import rvt_trainer.monolith as monolith
import rvt_trainer.session.supervisor as supervisor_module
from rvt_trainer.api.route_registry import match_route
from rvt_trainer.monolith import (
    FIRMWARE_VERSION_EXPECTED,
    _ControlServer,
    _SessionSupervisor,
    _write_session_manifest,
    save_json,
)
from rvt_trainer.session.study_contract import (
    PARTICIPANT_REGISTRY_SCHEMA_VERSION,
    StudyContractError,
    canonical_condition_id,
    create_participant_profile,
    load_participant_registry,
    merge_immutable_study_assignment,
    release_provenance,
    update_participant_status,
    validate_study_assignment,
)


@pytest.fixture
def sessions_root(tmp_path: Path) -> Path:
    root = tmp_path / "sessions"
    root.mkdir()
    return root


def _confirmatory(participant_id: str = "P-001") -> dict:
    return {
        "participant_id": participant_id,
        "trial_id": "P-001-d060-none-t1",
        "condition_id": "d060_none",
        "distance_m": 0.6,
        "barrier_type": "none",
        "trial_number": 1,
        "planned_duration_s": 150,
        "duration_s": 150,
        "study_classification": "confirmatory",
    }


def test_participant_registry_allocates_codes_without_identity_fields(
    sessions_root: Path,
):
    first = create_participant_profile(str(sessions_root), {})
    second = create_participant_profile(str(sessions_root), {})

    assert first["participant_id"] == "P-001"
    assert first["display_code"] == "P-001"
    assert first["profile_code"] == "P-001"
    assert first["status"] == "active"
    assert second["participant_id"] == "P-002"
    registry = load_participant_registry(str(sessions_root))
    assert registry["schema_version"] == PARTICIPANT_REGISTRY_SCHEMA_VERSION
    assert set(registry["profiles"]) == {"P-001", "P-002"}
    assert not any("name" in key for key in first)


def test_participant_registry_allocates_unique_codes_concurrently(
    sessions_root: Path,
):
    with ThreadPoolExecutor(max_workers=8) as executor:
        profiles = list(
            executor.map(
                lambda _index: create_participant_profile(
                    str(sessions_root),
                    {},
                ),
                range(40),
            )
        )
    codes = [profile["participant_id"] for profile in profiles]
    assert len(set(codes)) == 40
    assert set(codes) == {f"P-{number:03d}" for number in range(1, 41)}
    assert len(load_participant_registry(str(sessions_root))["profiles"]) == 40


@pytest.mark.parametrize("field", ["name", "display_name", "email", "phone"])
def test_participant_registry_rejects_direct_identifiers(
    sessions_root: Path,
    field: str,
):
    with pytest.raises(StudyContractError, match="pseudonymous") as exc:
        create_participant_profile(str(sessions_root), {field: "private"})
    assert exc.value.code == "PII_FIELD_FORBIDDEN"


def test_participant_status_is_lifecycle_only_and_withdrawn_cannot_start(
    sessions_root: Path,
):
    create_participant_profile(str(sessions_root), {})
    profile = update_participant_status(str(sessions_root), "P-001", "withdrawn")
    assert profile["status"] == "withdrawn"
    with pytest.raises(StudyContractError) as exc:
        validate_study_assignment(
            _confirmatory(),
            sessions_root=str(sessions_root),
        )
    assert exc.value.code == "PARTICIPANT_NOT_ACTIVE"


def test_confirmatory_contract_accepts_frozen_proposal_conditions(
    sessions_root: Path,
):
    create_participant_profile(str(sessions_root), {})
    assignment = validate_study_assignment(
        _confirmatory(),
        sessions_root=str(sessions_root),
    )
    assert assignment["participant_id"] == "P-001"
    assert assignment["condition_id"] == "d060_none"
    assert assignment["confirmatory_eligible"] is True
    assert assignment["planned_duration_s"] == 150.0


@pytest.mark.parametrize(
    ("patch", "code"),
    [
        ({"participant_id": ""}, "PARTICIPANT_REQUIRED"),
        ({"distance_m": 0.7, "condition_id": "d070_none"}, "INVALID_CONFIRMATORY_DISTANCE"),
        ({"trial_number": 4}, "INVALID_CONFIRMATORY_TRIAL"),
        ({"planned_duration_s": 149, "duration_s": 149}, "INVALID_CONFIRMATORY_DURATION"),
        ({"duration_s": 30}, "CAPTURE_DURATION_MISMATCH"),
        ({"condition_id": "wrong"}, "CONDITION_MISMATCH"),
    ],
)
def test_confirmatory_contract_rejects_protocol_drift(
    sessions_root: Path,
    patch: dict,
    code: str,
):
    create_participant_profile(str(sessions_root), {})
    payload = {**_confirmatory(), **patch}
    with pytest.raises(StudyContractError) as exc:
        validate_study_assignment(payload, sessions_root=str(sessions_root))
    assert exc.value.code == code


def test_exploratory_contract_supports_half_metre_capture(
    sessions_root: Path,
):
    create_participant_profile(str(sessions_root), {})
    assignment = validate_study_assignment(
        {
            **_confirmatory(),
            "trial_id": "P-001-exploratory-half-metre",
            "condition_id": canonical_condition_id(0.5, "cardboard"),
            "distance_m": 0.5,
            "barrier_type": "cardboard",
            "trial_number": 5,
            "planned_duration_s": 60,
            "duration_s": 60,
            "study_classification": "exploratory",
        },
        sessions_root=str(sessions_root),
    )
    assert assignment["distance_m"] == 0.5
    assert assignment["confirmatory_eligible"] is False
    assert assignment["provenance_state"] == "assigned"


def test_study_mode_alias_is_normalized_for_frontend_compatibility(
    sessions_root: Path,
):
    create_participant_profile(str(sessions_root), {})
    payload = _confirmatory()
    payload["study_mode"] = payload.pop("study_classification")
    assignment = validate_study_assignment(
        payload,
        sessions_root=str(sessions_root),
    )
    assert assignment["study_classification"] == "confirmatory"


def test_unassigned_legacy_request_remains_operational_but_not_confirmatory():
    assignment = validate_study_assignment({})
    assert assignment == {
        "schema_version": "rvt-study-session-v16.5.9",
        "study_classification": "operational",
        "provenance_state": "legacy_unassigned",
        "confirmatory_eligible": False,
    }


def test_partial_study_metadata_requires_explicit_classification():
    with pytest.raises(StudyContractError) as exc:
        validate_study_assignment({"participant_id": "P-001"})
    assert exc.value.code == "STUDY_CLASSIFICATION_REQUIRED"


def test_session_assignment_cannot_be_reassigned():
    existing = _confirmatory()
    with pytest.raises(StudyContractError) as exc:
        merge_immutable_study_assignment(
            existing,
            {**existing, "participant_id": "P-002"},
        )
    assert exc.value.code == "SESSION_ASSIGNMENT_IMMUTABLE"


def test_release_provenance_carries_cross_stack_and_model_identity(monkeypatch):
    monkeypatch.setenv("RVT_SOURCE_COMMIT", "abc123")
    import rvt_trainer.session.study_contract as contract

    monkeypatch.setattr(contract, "_SOURCE_COMMIT", None)
    monkeypatch.setattr(supervisor_module, "require_collection_authorization", lambda _root: {"authorized": True})
    payload = release_provenance(
        product_version="16.5.8",
        trainer_version="16.5.8",
        dashboard_version="16.5.8",
        firmware_expected="v16.5.8",
        firmware_observed="v16.5.8",
        serial_width_observed=222,
        model_family="gradient_boosting",
        model_bundle="bundle-sha256",
    )
    assert payload == {
        "product_version": "16.5.8",
        "trainer_version": "16.5.8",
        "dashboard_version": "16.5.8",
        "firmware_expected": "v16.5.8",
        "firmware_observed": "v16.5.8",
        "serial_protocol": "v15.2",
        "serial_width_expected": 222,
        "serial_width_observed": 222,
        "source_commit": "abc123",
        "model_family": "gradient_boosting",
        "model_bundle": "bundle-sha256",
    }


@patch("subprocess.Popen")
def test_supervisor_persists_assignment_before_child_and_passes_cli_flags(
    mock_popen,
    sessions_root: Path,
    monkeypatch,
):
    monkeypatch.setenv("RVT_SOURCE_COMMIT", "test-commit")
    import rvt_trainer.session.study_contract as contract

    monkeypatch.setattr(contract, "_SOURCE_COMMIT", None)
    monkeypatch.setattr(monolith, "_require_collection_authorization", lambda _root: {"authorized": True})
    monkeypatch.setattr(supervisor_module, "require_collection_authorization", lambda _root: {"authorized": True})
    create_participant_profile(str(sessions_root), {})
    proc = MagicMock()
    proc.pid = 12345
    proc.poll.return_value = None

    def spawn(argv, **_kwargs):
        session_dir = sessions_root / "s01"
        manifest = json.loads(
            (session_dir / "session_manifest.json").read_text(encoding="utf-8")
        )
        assert manifest["participant_id"] == "P-001"
        save_json({"radar": {}}, str(session_dir / "live_dashboard.json"))
        return proc

    mock_popen.side_effect = spawn
    supervisor = _SessionSupervisor(str(sessions_root))
    result = supervisor.start(
        timeout_s=1,
        **_confirmatory(),
        model_family="gradient_boosting",
        model_bundle="bundle-001",
    )
    assert result["session_id"] == "s01"
    argv = mock_popen.call_args.args[0]
    assert argv[argv.index("--participant-id") + 1] == "P-001"
    assert argv[argv.index("--condition-id") + 1] == "d060_none"
    assert argv[argv.index("--model-bundle") + 1] == "bundle-001"
    manifest = json.loads(
        (sessions_root / "s01" / "session_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    assert manifest["serial_protocol"] == "v15.2"
    assert manifest["serial_width_expected"] == 222
    assert manifest["firmware_expected"] == FIRMWARE_VERSION_EXPECTED


def test_analysis_manifest_rewrite_preserves_study_assignment(
    sessions_root: Path,
):
    session_dir = sessions_root / "s01"
    analysis_dir = session_dir / "analysis"
    analysis_dir.mkdir(parents=True)
    radar = session_dir / "radar.csv"
    ref = session_dir / "ref.csv"
    radar.write_text("x\n1\n", encoding="utf-8")
    ref.write_text("x\n1\n", encoding="utf-8")
    save_json(
        {
            "schema_version": "rvt-session-manifest-v16.5.1",
            "study_session_schema_version": "rvt-study-session-v16.5.1",
            **{
                key: value
                for key, value in _confirmatory().items()
                if key != "duration_s"
            },
            "provenance_state": "assigned",
            "confirmatory_eligible": True,
            "model_family": "cnn_1d",
            "model_bundle": "bundle-cnn",
        },
        str(session_dir / "session_manifest.json"),
    )
    manifest = _write_session_manifest(
        str(session_dir),
        [str(radar)],
        [str(ref)],
        str(analysis_dir),
        {"version": FIRMWARE_VERSION_EXPECTED, "contract_length": 222},
    )
    assert manifest["participant_id"] == "P-001"
    assert manifest["trial_id"] == "P-001-d060-none-t1"
    assert manifest["model_family"] == "cnn_1d"
    assert manifest["firmware_observed"] == FIRMWARE_VERSION_EXPECTED
    assert manifest["serial_width_observed"] == 222


def test_participant_routes_are_operator_protected():
    assert match_route("GET", "/api/participants").name == "participants_list"
    assert match_route("POST", "/api/participants").name == "participants_create"
    assert (
        match_route("PUT", "/api/participants/P-001").name
        == "participant_status"
    )


@patch("rvt_trainer.monolith._run_preflight_all", return_value={"checks": []})
@patch("subprocess.Popen")
def test_api_start_persists_release_bound_participant_manifest(
    mock_popen,
    _mock_preflight,
    sessions_root: Path,
    monkeypatch,
):
    monkeypatch.setenv("RVT_SOURCE_COMMIT", "api-test-commit")
    import rvt_trainer.session.study_contract as contract

    monkeypatch.setattr(contract, "_SOURCE_COMMIT", None)
    monkeypatch.setattr(monolith, "_require_collection_authorization", lambda _root: {"authorized": True})
    monkeypatch.setattr(supervisor_module, "require_collection_authorization", lambda _root: {"authorized": True})
    proc = MagicMock()
    proc.pid = 54321
    proc.poll.return_value = None

    def spawn(_argv, **_kwargs):
        save_json(
            {"radar": {}},
            str(sessions_root / "s01" / "live_dashboard.json"),
        )
        return proc

    mock_popen.side_effect = spawn
    control = _ControlServer(
        "127.0.0.1",
        0,
        str(sessions_root),
        mock=True,
    )
    control.start()

    def request(method: str, path: str, payload: dict):
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

    try:
        status, created = request("POST", "/api/participants", {})
        assert status == 201
        assert created["participant"]["participant_id"] == "P-001"

        payload = _confirmatory()
        payload["model_family"] = "gradient_boosting"
        payload["model_bundle"] = "gbr-production-001"
        status, started = request("POST", "/api/session/start", payload)
        assert status == 200
        assert started["session_id"] == "s01"

        manifest = json.loads(
            (sessions_root / "s01" / "session_manifest.json").read_text(
                encoding="utf-8"
            )
        )
        assert manifest["participant_id"] == "P-001"
        assert manifest["trial_id"] == "P-001-d060-none-t1"
        assert manifest["condition_id"] == "d060_none"
        assert manifest["planned_duration_s"] == 150.0
        assert manifest["product_version"] == manifest["trainer_version"]
        assert manifest["dashboard_version"] == manifest["trainer_version"]
        assert manifest["firmware_expected"] == FIRMWARE_VERSION_EXPECTED
        assert manifest["firmware_observed"] is None
        assert manifest["serial_protocol"] == "v15.2"
        assert manifest["serial_width_expected"] == 222
        assert manifest["serial_width_observed"] is None
        assert manifest["source_commit"] == "api-test-commit"
        assert manifest["model_family"] == "gradient_boosting"
        assert manifest["model_bundle"] == "gbr-production-001"
    finally:
        control.supervisor._reset_runtime_state()
        control.stop()
