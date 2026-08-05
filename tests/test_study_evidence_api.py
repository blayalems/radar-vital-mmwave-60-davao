"""Focused HTTP contracts for versioned study-control evidence routes."""

from __future__ import annotations

import http.client
import json
from pathlib import Path

import pytest

from rvt_trainer.monolith import _ControlServer
from rvt_trainer.session.study_contract import create_participant_profile


def _request(
    control: _ControlServer,
    method: str,
    path: str,
    payload: dict | None = None,
) -> tuple[int, dict]:
    body = json.dumps(payload).encode("utf-8") if payload is not None else None
    headers = {"Content-Type": "application/json"} if body is not None else {}
    conn = http.client.HTTPConnection(
        "127.0.0.1",
        control.httpd.server_port,
        timeout=5,
    )
    try:
        conn.request(method, path, body=body, headers=headers)
        response = conn.getresponse()
        return response.status, json.loads(response.read())
    finally:
        conn.close()


@pytest.fixture
def study_server(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    create_participant_profile(str(sessions_root), {})
    (sessions_root / "s01").mkdir()
    control = _ControlServer(
        "127.0.0.1",
        0,
        str(sessions_root),
        bind_mode="local",
        mock=True,
    )
    control.start()
    try:
        yield control
    finally:
        control.stop()


def test_protocol_schedule_and_no_subject_attempt_contract(study_server: _ControlServer):
    status, response = _request(study_server, "GET", "/api/study/protocol")
    assert status == 200
    assert response["protocol"]["schema_version"] == "rvt-study-protocol-v2"
    assert response["protocol"]["protocol_id"] == "RVT-THESIS-16.5.9"

    status, response = _request(
        study_server,
        "PUT",
        "/api/study/protocol",
        {"state": "locked", "actor": "OP-001"},
    )
    assert status == 200
    assert response["protocol"]["state"] == "locked"

    status, response = _request(
        study_server,
        "GET",
        "/api/study/schedule?participant_id=P-001",
    )
    assert status == 200
    assert response["participant_id"] == "P-001"
    assert len(response["entries"]) == 6
    assert {tuple(entry["trial_numbers"]) for entry in response["entries"]} == {
        (1, 2, 3)
    }

    status, response = _request(
        study_server,
        "POST",
        "/api/study/attempts",
        {
            "attempt_type": "no_subject",
            "condition_id": "d060_none",
            "trial_number": 1,
            "status": "completed",
        },
    )
    assert status == 201
    assert response["attempt"]["attempt_type"] == "no_subject"

    status, matrix = _request(
        study_server,
        "GET",
        "/api/study/completion-matrix",
    )
    assert status == 200
    assert matrix["no_subject_attempt_count"] == 1


def test_reference_adjudication_analysis_and_objective_report_contract(
    study_server: _ControlServer,
):
    for observer_id, value in (("OBS-A", 15), ("OBS-B", 16)):
        status, response = _request(
            study_server,
            "POST",
            "/api/sessions/s01/references",
            {
                "kind": "rr_observer",
                "observer_id": observer_id,
                "value": value,
                "unit": "breaths/min",
                "duration_s": 150,
            },
        )
        assert status == 201
        assert response["session_id"] == "s01"

    status, response = _request(
        study_server,
        "POST",
        "/api/sessions/s01/references/rr-adjudication",
        {"final_value": 15.5, "rationale": "dual-observer review"},
    )
    assert status == 200
    assert response["rr_adjudication"]["final_value"] == 15.5

    status, response = _request(
        study_server,
        "GET",
        "/api/sessions/s01/references",
    )
    assert status == 200
    assert response["session_id"] == "s01"
    assert len(response["references"]) == 2
    assert response["rr_adjudication"]["locked_at"]

    status, response = _request(
        study_server,
        "POST",
        "/api/study/analysis",
        {"objective_id": "objective_1_rr", "model_family": "gbr"},
    )
    assert status == 202
    job_id = response["job"]["job_id"]
    assert response["job"]["schema_version"] == "rvt-study-analysis-v1"
    assert response["job"]["model_family"] == "gradient_boosting"

    status, response = _request(
        study_server,
        "GET",
        f"/api/study/analysis/{job_id}",
    )
    assert status == 200
    assert response["job"]["job_id"] == job_id

    status, response = _request(study_server, "GET", "/api/study/analysis?limit=5")
    assert status == 200
    assert response["jobs"][0]["job_id"] == job_id

    status, response = _request(
        study_server,
        "GET",
        "/api/study/objectives/objective_1_rr/report",
    )
    assert status == 200
    assert response["objective_id"] == "objective_1_rr"
    assert response["status"] == "inconclusive"
    assert response["provenance"]["product_version"] == "16.5.10"


def test_study_evidence_routes_are_operator_protected(study_server: _ControlServer):
    from rvt_trainer.api.route_registry import match_route
    from rvt_trainer.api.route_registry.types import AuthPolicy

    cases = (
        ("GET", "/api/study/protocol"),
        ("PUT", "/api/study/protocol"),
        ("GET", "/api/study/schedule"),
        ("POST", "/api/sessions/s01/references"),
        ("GET", "/api/sessions/s01/references"),
        ("POST", "/api/sessions/s01/references/rr-adjudication"),
        ("POST", "/api/study/analysis"),
        ("GET", "/api/study/analysis"),
        ("GET", "/api/study/analysis/JOB-1"),
        ("DELETE", "/api/study/analysis/JOB-1"),
        ("GET", "/api/study/objectives/objective_1_rr/report"),
    )
    for method, path in cases:
        route = match_route(method, path)
        assert route is not None
        assert route.auth is AuthPolicy.OPERATOR
