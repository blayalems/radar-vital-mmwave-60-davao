import os
import json
import time
from pathlib import Path
from unittest.mock import MagicMock, patch
import pytest

from rvt_trainer.monolith import _SessionSupervisor, _session_is_active, _next_session_dir, save_json

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
