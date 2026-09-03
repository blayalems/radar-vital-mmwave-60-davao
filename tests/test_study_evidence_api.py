"""Focused HTTP contracts for versioned study-control evidence routes."""

from __future__ import annotations

import http.client
import json
import threading
from pathlib import Path

import pytest

import rvt_trainer.monolith as monolith
from rvt_trainer.monolith import _ControlServer
from rvt_trainer.session.study_contract import create_participant_profile
from rvt_trainer.session.study_evidence import (
    _validate_completed_job_artifacts,
    create_analysis_job,
    load_analysis_job,
)


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
    assert status == 400
    assert response["error"]["code"] == "INVALID_STUDY_PROTOCOL"

    status, response = _request(
        study_server,
        "PUT",
        "/api/study/protocol",
        {
            "state": "locked",
            "actor": "OP-001",
            "no_subject": {
                "trial_count": 72,
                "planned_duration_s": 150,
                "frozen_configuration": {
                    "firmware": "v16.5.12",
                    "artifact_rules": "frozen-rules-1",
                    "alert_threshold": 0.8,
                },
            },
        },
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
    assert response["provenance"]["product_version"] == "16.6.1"


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


def test_analysis_worker_exception_becomes_durable_failed_job(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    job = create_analysis_job(
        str(sessions_root),
        {"objective_id": "objective_1_rr", "model_family": "gbr"},
    )
    semaphore = threading.BoundedSemaphore(1)
    monkeypatch.setattr(monolith, "_STUDY_ANALYSIS_SEMAPHORE", semaphore)

    def fail_before_inner_worker_guard(_sessions_root: str, _job_id: str) -> None:
        raise RuntimeError("synthetic top-level worker failure")

    monkeypatch.setattr(
        monolith,
        "_run_study_analysis_job_once",
        fail_before_inner_worker_guard,
    )
    monolith._run_study_analysis_job(str(sessions_root), str(job["job_id"]))

    persisted = load_analysis_job(str(sessions_root), job["job_id"])
    assert persisted is not None
    assert persisted["status"] == "failed"
    assert persisted["completed_at"]
    assert persisted["error"] == "RuntimeError: synthetic top-level worker failure"
    assert "failed before terminal state" in persisted["last_line"]
    assert semaphore.acquire(blocking=False) is True
    semaphore.release()


def test_completed_job_promotion_rejects_report_manifest_identity_mismatch(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    output_dir = sessions_root / "study_analysis" / "job-1"
    output_dir.mkdir(parents=True)
    identity = {
        "run_product_version": "16.6.1",
        "analysis_plan_id": "RVT-STA-PLAN-16.5.8",
        "analysis_plan_sha256": "a" * 64,
        "study_protocol_id": "RVT-THESIS-16.5.9",
        "study_session_schema_version": "rvt-study-session-v16.5.9",
    }
    (output_dir / "confirmatory_run_manifest.json").write_text(
        json.dumps(
            {
                **identity,
                "analysis_job_id": "job-1",
                "model_family": "gradient_boosting",
                "targets": ["rr"],
                "source_commit": "c" * 40,
                "folds": {"complete": True},
            }
        ),
        encoding="utf-8",
    )
    (output_dir / "statistical_report.json").write_text(
        json.dumps(
            {
                "confirmatory": True,
                "provenance": {**identity, "analysis_plan_sha256": "b" * 64},
            }
        ),
        encoding="utf-8",
    )

    valid, reasons, *_ = _validate_completed_job_artifacts(
        {
            "job_id": "job-1",
            "output_dir": str(output_dir),
            "model_family": "gradient_boosting",
            "statistics_status": "completed",
            "objective_id": "objective_1_rr",
            "request": {"confirmatory": True},
        },
        sessions_root=str(sessions_root),
        product_version="16.6.1",
    )

    assert valid is False
    assert "statistical_report_identity_mismatch" in {
        reason["reason"] for reason in reasons
    }
