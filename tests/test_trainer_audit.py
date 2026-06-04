"""Unit tests for rvt_trainer.audit.runner — the Phase 5–extracted
preflight wizard backing ``/api/preflight``.

We exercise the structural (no-hardware) checks that run deterministically
in CI, the aggregation/summary contract of ``run_preflight_all``, the
``include`` filter, and the never-raises guarantee of a single check.
"""

from __future__ import annotations

import rvt_trainer.monolith as monolith
from rvt_trainer.audit import runner


def test_clock_monotonic_check_is_ok():
    res = runner.run_preflight_check("clock_monotonic_sanity")
    assert res["id"] == "clock_monotonic_sanity"
    assert res["status"] == "ok"
    assert "duration_ms" in res


def test_schema_hash_check_is_ok_and_stable():
    a = runner.run_preflight_check("schema_hash_consistency")
    b = runner.run_preflight_check("schema_hash_consistency")
    assert a["status"] == "ok"
    # The scoring-weights hash is part of the frozen contract; two reads
    # within one process must agree.
    assert a["detail"] == b["detail"]


def test_session_folder_writable_check(tmp_path):
    res = runner.run_preflight_check("session_folder_writable", sessions_root=str(tmp_path))
    assert res["status"] == "ok"
    assert str(tmp_path) in res["detail"]


def test_unknown_check_is_skipped():
    res = runner.run_preflight_check("does_not_exist")
    assert res["status"] == "skip"
    assert res["detail"] == "Unknown check id"


def test_run_preflight_all_include_filters_checks():
    report = runner.run_preflight_all(
        sessions_root="sessions",
        include=["schema_hash_consistency", "clock_monotonic_sanity"],
    )
    assert sorted(report.keys()) == ["checks", "ran_at", "refs", "summary"]
    ids = [c["id"] for c in report["checks"]]
    assert ids == ["schema_hash_consistency", "clock_monotonic_sanity"]
    summary = report["summary"]
    assert summary["ok"] == 2
    assert summary["overall"] == "ok"
    assert summary["fail"] == 0


def test_run_preflight_all_summary_overall_precedence(monkeypatch):
    # Force a warn result and confirm overall escalates to warn (not fail).
    def fake_check(check_id, **params):
        status = "warn" if check_id == "clock_monotonic_sanity" else "ok"
        return {"id": check_id, "label": check_id, "status": status,
                "detail": "", "remediation": "", "duration_ms": 0.0}

    monkeypatch.setattr(runner, "run_preflight_check", fake_check)
    report = runner.run_preflight_all(
        include=["schema_hash_consistency", "clock_monotonic_sanity"],
    )
    assert report["summary"]["warn"] == 1
    assert report["summary"]["overall"] == "warn"


def test_run_preflight_all_refs_carry_port_and_address():
    report = runner.run_preflight_all(
        include=["clock_monotonic_sanity"],
        port="COM7",
        address="AA:BB:CC:DD:EE:FF",
    )
    assert report["refs"]["port"] == "COM7"
    assert report["refs"]["address"] == "AA:BB:CC:DD:EE:FF"


def test_monolith_reexports_point_at_extracted_runner():
    assert monolith._run_preflight_all is runner.run_preflight_all
    assert monolith._run_preflight_check is runner.run_preflight_check


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__, "-v"]))
