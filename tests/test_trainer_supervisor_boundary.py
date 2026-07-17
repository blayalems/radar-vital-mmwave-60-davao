"""Import and compatibility contracts for the extracted session supervisor."""

from __future__ import annotations

from rvt_trainer import monolith
from rvt_trainer.session import SessionSupervisor
from rvt_trainer.session import supervisor as service


def test_legacy_and_service_supervisor_imports_are_identical():
    assert monolith._SessionSupervisor is SessionSupervisor
    assert SessionSupervisor.__module__ == "rvt_trainer.session.supervisor"


def test_legacy_lock_and_stop_marker_helpers_are_service_aliases():
    names = [
        "_check_stale_session_lock",
        "_clear_supervisor_stop_request",
        "_consume_supervisor_stop_request",
        "_lock_path",
        "_read_session_lock",
        "_release_session_lock",
        "_release_session_lock_if_owned",
        "_same_session_dir",
        "_session_is_active",
        "_session_marker_owned_by",
        "_supervisor_stop_path",
        "_write_session_lock",
        "_write_supervisor_stop_request",
    ]

    for name in names:
        assert getattr(monolith, name) is getattr(service, name)


def test_direct_service_import_keeps_idempotent_stop_schema(tmp_path):
    supervisor = SessionSupervisor(str(tmp_path / "sessions"))

    payload = supervisor.stop(
        reason="server_shutdown",
        auto_analyse=False,
        missing_ok=True,
    )

    assert payload["schema_version"] == monolith.CONTROL_API_SCHEMA_VERSION
    assert payload["already_stopped"] is True
    assert payload["reason"] == "server_shutdown"
