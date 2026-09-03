"""Adversarial regressions for exactly-once control-API session starts."""

from concurrent.futures import ThreadPoolExecutor
import http.client
import json
from pathlib import Path
import threading
import time
from unittest.mock import MagicMock, patch

import pytest

import rvt_trainer.monolith as monolith
import rvt_trainer.session.supervisor as supervisor_module
from rvt_trainer.monolith import _ControlServer
from rvt_trainer.session.start_idempotency import (
    StartIdempotencyError,
    StartIdempotencyStore,
)
from rvt_trainer.session.study_contract import create_participant_profile


def _confirmatory_payload(key: str) -> dict:
    return {
        "idempotency_key": key,
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
        "timeout_s": 1,
    }


def _request_json(
    control: _ControlServer,
    payload: dict,
    *,
    headers: dict[str, str] | None = None,
) -> tuple[int, dict]:
    body = json.dumps(payload).encode("utf-8")
    request_headers = {"Content-Type": "application/json", **(headers or {})}
    conn = http.client.HTTPConnection(
        "127.0.0.1",
        control.httpd.server_port,
        timeout=5,
    )
    try:
        conn.request(
            "POST",
            "/api/session/start",
            body=body,
            headers=request_headers,
        )
        response = conn.getresponse()
        return response.status, json.loads(response.read())
    finally:
        conn.close()


def _spawn_that_becomes_ready(
    sessions_root: Path,
    calls: list[str],
    started: threading.Event | None = None,
):
    def spawn(argv, **_kwargs):
        session_dir = Path(argv[argv.index("--session-dir") + 1])
        calls.append(session_dir.name)
        # Widen the race window so the second HTTP worker reaches the same-key
        # reservation while the first worker owns the lifecycle operation.
        time.sleep(0.05)
        (session_dir / "live_dashboard.json").write_text(
            json.dumps({"session_id": session_dir.name, "radar": {}}),
            encoding="utf-8",
        )
        if started is not None:
            started.set()
        proc = MagicMock()
        proc.pid = 54000 + len(calls)
        proc.poll.return_value = None
        return proc

    return spawn


@pytest.fixture
def control(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    create_participant_profile(str(sessions_root), {})
    monkeypatch.setattr(monolith, "_require_collection_authorization", lambda _root: {"authorized": True})
    monkeypatch.setattr(supervisor_module, "require_collection_authorization", lambda _root: {"authorized": True})
    server = _ControlServer("127.0.0.1", 0, str(sessions_root), mock=True)
    server.start()
    try:
        yield server
    finally:
        server.supervisor._reset_runtime_state()
        server.stop()


def test_unauthorized_confirmatory_start_is_non_mutating(tmp_path: Path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()
    create_participant_profile(str(sessions_root), {})
    server = _ControlServer("127.0.0.1", 0, str(sessions_root), mock=True)
    server.start()
    try:
        status, response = _request_json(
            server, _confirmatory_payload("blocked-confirmatory-P001-t1")
        )
    finally:
        server.stop()

    assert status == 412
    assert response["error"]["code"] == "STUDY_COLLECTION_NOT_AUTHORIZED"
    assert not list(sessions_root.glob("s*"))
    assert not (sessions_root / ".session_start_idempotency.json").exists()
    assert not (sessions_root / ".logical_trial_reservations.json").exists()


@patch("rvt_trainer.monolith._run_preflight_all", return_value={"checks": []})
@patch("subprocess.Popen")
def test_concurrent_same_key_same_payload_allocates_and_spawns_once(
    mock_popen: MagicMock,
    mock_preflight: MagicMock,
    control: _ControlServer,
):
    spawn_calls: list[str] = []
    mock_popen.side_effect = _spawn_that_becomes_ready(
        Path(control.sessions_root),
        spawn_calls,
    )
    payload = _confirmatory_payload("start-concurrent-P001-t1")

    with ThreadPoolExecutor(max_workers=2) as pool:
        responses = list(
            pool.map(lambda _index: _request_json(control, payload), range(2))
        )

    assert responses[0] == responses[1]
    assert responses[0][0] == 200
    assert responses[0][1]["session_id"] == "s01"
    assert spawn_calls == ["s01"]
    capture_launches = [
        call
        for call in mock_popen.call_args_list
        if "--session-dir" in list(call.args[0])
    ]
    assert len(capture_launches) == 1
    assert mock_preflight.call_count == 1
    assert sorted(
        path.name
        for path in Path(control.sessions_root).glob("s*")
        if path.is_dir()
    ) == ["s01"]


@patch("rvt_trainer.monolith._run_preflight_all", return_value={"checks": []})
@patch("subprocess.Popen")
def test_same_key_different_payload_returns_stable_conflict_before_reallocation(
    mock_popen: MagicMock,
    mock_preflight: MagicMock,
    control: _ControlServer,
):
    spawn_calls: list[str] = []
    mock_popen.side_effect = _spawn_that_becomes_ready(
        Path(control.sessions_root),
        spawn_calls,
    )
    payload = _confirmatory_payload("start-reuse-P001-t1")
    status, started = _request_json(control, payload)
    conflicting = {**payload, "model_bundle": "gbr-different"}
    conflict_status, conflict = _request_json(control, conflicting)

    assert status == 200
    assert started["session_id"] == "s01"
    assert conflict_status == 409
    assert conflict["error"]["code"] == "IDEMPOTENCY_KEY_REUSED"
    assert spawn_calls == ["s01"]
    capture_launches = [
        call
        for call in mock_popen.call_args_list
        if "--session-dir" in list(call.args[0])
    ]
    assert len(capture_launches) == 1
    assert mock_preflight.call_count == 1


@patch("rvt_trainer.monolith._run_preflight_all", return_value={"checks": []})
@patch("subprocess.Popen")
def test_different_keys_cannot_reserve_the_same_logical_trial_concurrently(
    mock_popen: MagicMock,
    mock_preflight: MagicMock,
    control: _ControlServer,
):
    spawn_calls: list[str] = []
    mock_popen.side_effect = _spawn_that_becomes_ready(
        Path(control.sessions_root),
        spawn_calls,
    )
    first = _confirmatory_payload("logical-trial-client-a")
    second = {**first, "idempotency_key": "logical-trial-client-b"}

    with ThreadPoolExecutor(max_workers=2) as pool:
        responses = list(
            pool.map(
                lambda payload: _request_json(control, payload),
                (first, second),
            )
        )

    assert sorted(status for status, _payload in responses) == [200, 409]
    conflict = next(payload for status, payload in responses if status == 409)
    assert conflict["error"]["code"] == "LOGICAL_TRIAL_RESERVED"
    assert conflict["error"]["logical_trial_id"] == "P-001-d060-none-t1"
    assert spawn_calls == ["s01"]
    assert mock_preflight.call_count == 1


@patch("rvt_trainer.monolith._run_preflight_all", return_value={"checks": []})
@patch("subprocess.Popen")
def test_terminal_session_releases_logical_trial_for_a_deliberate_new_attempt(
    mock_popen: MagicMock,
    mock_preflight: MagicMock,
    control: _ControlServer,
):
    spawn_calls: list[str] = []
    mock_popen.side_effect = _spawn_that_becomes_ready(
        Path(control.sessions_root),
        spawn_calls,
    )

    status, first = _request_json(
        control,
        _confirmatory_payload("logical-trial-first-attempt"),
    )
    assert status == 200
    assert first["session_id"] == "s01"

    control.supervisor.proc.poll.return_value = 0
    assert control.supervisor.current() is None

    status, second = _request_json(
        control,
        _confirmatory_payload("logical-trial-second-attempt"),
    )
    assert status == 200
    assert second["session_id"] == "s02"
    assert spawn_calls == ["s01", "s02"]
    assert mock_preflight.call_count == 2


@patch("rvt_trainer.monolith._run_preflight_all", return_value={"checks": []})
@patch("subprocess.Popen")
def test_retry_after_response_loss_replays_original_result(
    mock_popen: MagicMock,
    mock_preflight: MagicMock,
    control: _ControlServer,
):
    spawn_calls: list[str] = []
    started = threading.Event()
    mock_popen.side_effect = _spawn_that_becomes_ready(
        Path(control.sessions_root),
        spawn_calls,
        started,
    )
    payload = _confirmatory_payload("start-lost-response-P001-t1")
    body = json.dumps(payload).encode("utf-8")
    conn = http.client.HTTPConnection(
        "127.0.0.1",
        control.httpd.server_port,
        timeout=5,
    )
    conn.request(
        "POST",
        "/api/session/start",
        body=body,
        headers={"Content-Type": "application/json"},
    )
    conn.close()  # Simulate navigation/network loss before reading the reply.
    assert started.wait(timeout=3)

    status, replay = _request_json(control, payload)

    assert status == 200
    assert replay["session_id"] == "s01"
    assert spawn_calls == ["s01"]
    capture_launches = [
        call
        for call in mock_popen.call_args_list
        if "--session-dir" in list(call.args[0])
    ]
    assert len(capture_launches) == 1
    assert mock_preflight.call_count == 1


def test_store_expires_terminal_records_and_prunes_to_a_bounded_ledger(
    tmp_path: Path,
):
    now = [100.0]
    store = StartIdempotencyStore(
        str(tmp_path),
        retention_s=10,
        max_records=2,
        clock=lambda: now[0],
    )
    for index in range(3):
        key = f"bounded-{index}"
        request_hash = f"hash-{index}"
        store.begin(key, request_hash)
        store.mark_succeeded(
            key,
            request_hash,
            {"session_id": f"s{index + 1:02d}"},
        )
        now[0] += 1

    assert store.lookup("bounded-0", "hash-0") == ("missing", None)
    assert store.lookup("bounded-2", "hash-2")[0] == "replay"

    now[0] += 11
    assert store.prune() == 2
    assert store.lookup("bounded-1", "hash-1") == ("missing", None)
    assert store.lookup("bounded-2", "hash-2") == ("missing", None)


def test_store_never_silently_accepts_same_key_with_a_different_hash(
    tmp_path: Path,
):
    store = StartIdempotencyStore(str(tmp_path))
    store.begin("same-key", "original-hash")

    with pytest.raises(StartIdempotencyError) as exc:
        store.lookup("same-key", "changed-hash")

    assert exc.value.code == "IDEMPOTENCY_KEY_REUSED"
