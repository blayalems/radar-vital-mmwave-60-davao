import os
import json
import subprocess
import time
from pathlib import Path
from unittest.mock import MagicMock, patch
import pytest

from rvt_trainer.monolith import (
    _SessionSupervisor,
    _consume_supervisor_stop_request,
    _latest_live_dashboard_payload,
    _next_session_dir,
    _session_is_active,
    _supervisor_stop_path,
    _write_supervisor_stop_request,
    save_json,
)

@pytest.fixture
def temp_sessions_root(tmp_path):
    root = tmp_path / "sessions"
    root.mkdir()
    return root

def test_next_session_dir_isolation(temp_sessions_root):
    # Test directory naming sequencing s01, s02, etc.
    dir1 = _next_session_dir(str(temp_sessions_root))
    assert Path(dir1).name == "s01"
    
    Path(dir1).mkdir(parents=True, exist_ok=True)
    dir2 = _next_session_dir(str(temp_sessions_root))
    assert Path(dir2).name == "s02"

@patch("rvt_trainer.monolith._pid_alive")
@patch("subprocess.Popen")
def test_active_session_locking_isolation(mock_popen, mock_pid_alive, temp_sessions_root):
    # Mock processes
    mock_proc = MagicMock()
    mock_proc.pid = 12345
    mock_proc.poll.return_value = None  # process is running
    mock_popen.return_value = mock_proc
    mock_pid_alive.return_value = True

    supervisor = _SessionSupervisor(str(temp_sessions_root))

    # Helper: write live_dashboard.json immediately so start doesn't timeout
    def mock_spawn(*args, **kwargs):
        s01_dir = temp_sessions_root / "s01"
        s01_dir.mkdir(parents=True, exist_ok=True)
        live = s01_dir / "live_dashboard.json"
        save_json({"radar": {}}, str(live))
        return mock_proc

    mock_popen.side_effect = mock_spawn

    # Start first session
    res = supervisor.start(timeout_s=1.0)
    assert res["session_id"] == "s01"
    
    # Try starting another session while s01 is active
    with pytest.raises(RuntimeError) as excinfo:
        supervisor.start(timeout_s=1.0)
    assert "SESSION_IN_PROGRESS" in str(excinfo.value)

@patch("rvt_trainer.monolith._pid_alive")
@patch("subprocess.Popen")
def test_supervisor_placeholder_does_not_report_child_ready(mock_popen, mock_pid_alive, temp_sessions_root):
    mock_proc = MagicMock()
    mock_proc.pid = 12345
    mock_proc.poll.return_value = None
    mock_popen.return_value = mock_proc
    mock_pid_alive.return_value = True

    supervisor = _SessionSupervisor(str(temp_sessions_root))
    with pytest.raises(TimeoutError):
        supervisor.start(duration_s=60, radar_port="COM7", timeout_s=0.05)

    argv = mock_popen.call_args.args[0]
    assert "--dashboard-port" in argv
    assert argv[argv.index("--dashboard-port") + 1] == "0"
    live = temp_sessions_root / "s01" / "live_dashboard.json"
    assert live.exists()
    payload = json.loads(live.read_text(encoding="utf-8"))
    assert payload["meta"]["status"] == "starting"
    assert payload["meta"]["remaining_s"] == 60.0
    assert payload["radar"]["rows"] == 0
    assert bool(payload["_supervisor_placeholder"]) is True
    assert not (temp_sessions_root / "current_session.json").exists()


@patch("rvt_trainer.monolith._pid_alive")
@patch("subprocess.Popen")
def test_supervisor_rejects_exited_child_even_when_placeholder_exists(mock_popen, mock_pid_alive, temp_sessions_root):
    mock_proc = MagicMock()
    mock_proc.pid = 12345
    mock_proc.poll.return_value = 1
    mock_popen.return_value = mock_proc
    mock_pid_alive.return_value = False

    supervisor = _SessionSupervisor(str(temp_sessions_root))
    with pytest.raises(RuntimeError, match="SPAWN_ERROR"):
        supervisor.start(timeout_s=0.1)

    assert supervisor.proc is None
    assert not (temp_sessions_root / "current_session.json").exists()

def test_latest_live_dashboard_payload_marks_stale_active_payload_as_waiting(temp_sessions_root):
    s01 = temp_sessions_root / "s01"
    s01.mkdir(parents=True)
    save_json({
        "schema_version": "rvt-live-events-v12.0",
        "session_id": "s01",
        "meta": {"status": "collecting", "session_id": "s01"},
        "radar": {"rows": 12},
        "ble": {"rows": 3},
        "series": {},
    }, str(s01 / "live_dashboard.json"))

    payload = _latest_live_dashboard_payload(str(temp_sessions_root))

    assert payload is not None
    assert payload["session_id"] == "s01"
    assert payload["meta"]["status"] == "waiting"
    assert payload["meta"]["active"] is False
    assert payload["meta"]["latest_session"] is True

@patch("rvt_trainer.monolith._pid_alive")
@patch("subprocess.Popen")
def test_session_clean_stop_isolation(mock_popen, mock_pid_alive, temp_sessions_root):
    mock_proc = MagicMock()
    mock_proc.pid = 12345
    mock_proc.poll.return_value = None
    mock_popen.return_value = mock_proc
    mock_pid_alive.return_value = True

    supervisor = _SessionSupervisor(str(temp_sessions_root))

    # Mock spawn to write live_dashboard.json
    def mock_spawn_s01(*args, **kwargs):
        s01_dir = temp_sessions_root / "s01"
        s01_dir.mkdir(parents=True, exist_ok=True)
        save_json({"radar": {}}, str(s01_dir / "live_dashboard.json"))
        return mock_proc

    mock_popen.side_effect = mock_spawn_s01

    # Start first session (s01)
    res = supervisor.start(timeout_s=1.0)
    assert res["session_id"] == "s01"
    assert (temp_sessions_root / "current_session.json").exists()

    # Stop first session (simulating process exit)
    mock_proc.poll.return_value = 0 # finished
    supervisor.stop()
    assert supervisor.current() is None
    assert supervisor.proc is None
    assert not (temp_sessions_root / "current_session.json").exists()

    # Mock spawn for s02
    def mock_spawn_s02(*args, **kwargs):
        s02_dir = temp_sessions_root / "s02"
        s02_dir.mkdir(parents=True, exist_ok=True)
        save_json({"radar": {}}, str(s02_dir / "live_dashboard.json"))
        # Update process mock for the new call
        proc2 = MagicMock()
        proc2.pid = 12346
        proc2.poll.return_value = None
        return proc2

    mock_popen.side_effect = mock_spawn_s02
    mock_proc.poll.return_value = None # reset running state for new checks

    # Start second session (s02)
    res2 = supervisor.start(timeout_s=1.0)
    assert res2["session_id"] == "s02"
    assert (temp_sessions_root / "current_session.json").exists()


def _prime_supervisor(supervisor, sessions_root, proc):
    session_dir = sessions_root / "s01"
    session_dir.mkdir(parents=True, exist_ok=True)
    supervisor.proc = proc
    supervisor.session_dir = str(session_dir)
    supervisor.started_at = "2026-07-17T00:00:00Z"
    supervisor.started_monotonic = time.monotonic()
    supervisor.params = {"duration_s": 60}
    supervisor._write_current()
    save_json(
        {
            "pid": proc.pid,
            "session_dir": str(session_dir),
            "started_at": supervisor.started_at,
        },
        str(sessions_root / ".session.lock"),
    )
    return session_dir


@patch("rvt_trainer.monolith._spawn_auto_analyse")
def test_supervisor_graceful_stop_reaps_before_cleanup_and_analysis(
    mock_auto_analyse,
    temp_sessions_root,
):
    proc = MagicMock()
    proc.pid = 12345
    proc.poll.return_value = None
    proc.wait.return_value = 0
    supervisor = _SessionSupervisor(str(temp_sessions_root))
    session_dir = _prime_supervisor(supervisor, temp_sessions_root, proc)

    def assert_clean_then_analyse(path, reason):
        assert path == str(session_dir)
        assert reason == "user_request"
        assert supervisor.proc is None
        assert not (temp_sessions_root / "current_session.json").exists()
        assert not (temp_sessions_root / ".session.lock").exists()
        assert not _supervisor_stop_path(session_dir).exists()
        return {"status": "started"}

    mock_auto_analyse.side_effect = assert_clean_then_analyse

    result = supervisor.stop()

    proc.send_signal.assert_called_once()
    proc.wait.assert_called_once_with(timeout=supervisor._stop_grace_s)
    proc.terminate.assert_not_called()
    proc.kill.assert_not_called()
    mock_auto_analyse.assert_called_once_with(str(session_dir), reason="user_request")
    assert result["session_id"] == "s01"
    assert result["auto_analyse"] == {"status": "started"}


@patch("rvt_trainer.monolith._spawn_auto_analyse")
def test_supervisor_stop_escalates_through_terminate_and_kill(
    mock_auto_analyse,
    temp_sessions_root,
):
    proc = MagicMock()
    proc.pid = 12345
    proc.poll.return_value = None
    proc.wait.side_effect = [
        subprocess.TimeoutExpired("session", 10.0),
        subprocess.TimeoutExpired("session", 3.0),
        0,
    ]
    supervisor = _SessionSupervisor(str(temp_sessions_root))
    _prime_supervisor(supervisor, temp_sessions_root, proc)

    result = supervisor.stop(reason="server_shutdown", auto_analyse=False)

    assert [call.kwargs["timeout"] for call in proc.wait.call_args_list] == [
        supervisor._stop_grace_s,
        supervisor._terminate_grace_s,
        supervisor._kill_grace_s,
    ]
    proc.terminate.assert_called_once_with()
    proc.kill.assert_called_once_with()
    mock_auto_analyse.assert_not_called()
    assert result["reason"] == "server_shutdown"
    assert result["auto_analyse"] is None
    assert supervisor.proc is None


@patch("rvt_trainer.monolith._spawn_auto_analyse")
def test_supervisor_failed_reap_preserves_truthful_state_and_markers(
    mock_auto_analyse,
    temp_sessions_root,
):
    proc = MagicMock()
    proc.pid = 12345
    proc.poll.return_value = None
    proc.wait.side_effect = subprocess.TimeoutExpired("session", 1.0)
    supervisor = _SessionSupervisor(str(temp_sessions_root))
    session_dir = _prime_supervisor(supervisor, temp_sessions_root, proc)

    with pytest.raises(RuntimeError, match="SESSION_STOP_FAILED"):
        supervisor.stop()

    assert supervisor.proc is proc
    assert supervisor.session_dir == str(session_dir)
    assert (temp_sessions_root / "current_session.json").exists()
    assert (temp_sessions_root / ".session.lock").exists()
    assert _supervisor_stop_path(session_dir).exists()
    mock_auto_analyse.assert_not_called()


@patch("rvt_trainer.monolith._spawn_auto_analyse")
def test_supervisor_preserves_marker_replaced_by_another_owner(
    mock_auto_analyse,
    temp_sessions_root,
):
    proc = MagicMock()
    proc.pid = 12345
    proc.poll.return_value = None
    supervisor = _SessionSupervisor(str(temp_sessions_root))
    session_dir = _prime_supervisor(supervisor, temp_sessions_root, proc)
    other_dir = temp_sessions_root / "s02"

    def replace_current_marker(timeout):
        save_json(
            {
                "pid": 99999,
                "session_dir": str(other_dir),
                "started_at": "2026-07-17T00:01:00Z",
            },
            str(temp_sessions_root / "current_session.json"),
        )
        return 0

    proc.wait.side_effect = replace_current_marker

    with pytest.raises(RuntimeError, match="SESSION_CLEANUP_CONFLICT"):
        supervisor.stop()

    marker = json.loads(
        (temp_sessions_root / "current_session.json").read_text(encoding="utf-8")
    )
    assert marker["pid"] == 99999
    assert marker["session_dir"] == str(other_dir)
    assert supervisor.proc is None
    assert not (temp_sessions_root / ".session.lock").exists()
    assert not _supervisor_stop_path(session_dir).exists()
    mock_auto_analyse.assert_not_called()


@patch("subprocess.Popen")
def test_supervisor_close_gate_blocks_new_starts_and_missing_stop_is_idempotent(
    mock_popen,
    temp_sessions_root,
):
    supervisor = _SessionSupervisor(str(temp_sessions_root))
    supervisor.close_start_gate()

    with pytest.raises(RuntimeError, match="SUPERVISOR_CLOSING"):
        supervisor.start(timeout_s=0.01)

    first = supervisor.stop(
        reason="server_shutdown",
        auto_analyse=False,
        missing_ok=True,
    )
    second = supervisor.stop(
        reason="server_shutdown",
        auto_analyse=False,
        missing_ok=True,
    )

    mock_popen.assert_not_called()
    assert first["already_stopped"] is True
    assert second["already_stopped"] is True


def test_supervisor_stop_marker_suppresses_only_parent_driven_exit(
    temp_sessions_root,
):
    session_dir = temp_sessions_root / "s01"
    session_dir.mkdir()

    assert _consume_supervisor_stop_request(str(session_dir)) is None

    request = _write_supervisor_stop_request(
        str(session_dir),
        reason="server_shutdown",
        session_pid=12345,
        auto_analyse=False,
    )
    consumed = _consume_supervisor_stop_request(str(session_dir))

    assert consumed == request
    assert bool(consumed["suppress_inline_auto_analyse"]) is True
    assert bool(consumed["parent_auto_analyse"]) is False
    assert not _supervisor_stop_path(session_dir).exists()


def test_session_data_and_notes_isolation(temp_sessions_root):
    # Test file system isolation of session data files
    s01_dir = temp_sessions_root / "s01"
    s02_dir = temp_sessions_root / "s02"
    s01_dir.mkdir(parents=True, exist_ok=True)
    s02_dir.mkdir(parents=True, exist_ok=True)

    # Write separate notes / data
    notes_1 = {"session_id": "s01", "notes": [{"note": "Session 1 Note"}]}
    notes_2 = {"session_id": "s02", "notes": [{"note": "Session 2 Note"}]}
    
    save_json(notes_1, str(s01_dir / "session_notes.json"))
    save_json(notes_2, str(s02_dir / "session_notes.json"))

    # Assert they are isolated
    n1 = json.loads((s01_dir / "session_notes.json").read_text())
    n2 = json.loads((s02_dir / "session_notes.json").read_text())
    
    assert n1["session_id"] == "s01"
    assert n1["notes"][0]["note"] == "Session 1 Note"
    
    assert n2["session_id"] == "s02"
    assert n2["notes"][0]["note"] == "Session 2 Note"
    assert n1 != n2
