"""Background auto-analyse and study-analysis worker orchestration."""

from __future__ import annotations

import csv
import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

from rvt_trainer._compat import monolith_override
from rvt_trainer.api.common import (
    atomic_write_json as save_json,
    read_json_if_exists as _read_json_if_exists,
)
from rvt_trainer.constants import (
    CONTROL_API_SCHEMA_VERSION,
    FEATURE_FLAGS,
    MODEL_FAMILY_GRADIENT_BOOSTING,
    _REPO_ROOT,
    _append_trainer_log,
    _iso_now,
    _schema_wrap,
    _trainer_command_prefix,
)
from rvt_trainer.session.manifest import _write_session_manifest
from rvt_trainer.session.storage import _session_path
from rvt_trainer.session.study_evidence import (
    finish_analysis_job as _finish_analysis_job,
    load_analysis_job as _load_analysis_job,
    update_analysis_job as _update_analysis_job,
)

_ANALYSIS_JOBS: Dict[str, Dict[str, Any]] = {}
_ANALYSIS_JOBS_MAX = 32
_STUDY_ANALYSIS_PROCS: Dict[str, subprocess.Popen] = {}
_STUDY_ANALYSIS_LOCK = threading.RLock()
_STUDY_ANALYSIS_SEMAPHORE = threading.BoundedSemaphore(1)
_CANONICAL_STUDY_ANALYSIS_SEMAPHORE = _STUDY_ANALYSIS_SEMAPHORE


def _evict_completed_analysis_jobs() -> None:
    """Evict completed (non-running) entries from _ANALYSIS_JOBS when the dict
    exceeds _ANALYSIS_JOBS_MAX, keeping the most-recently-added jobs."""
    if len(_ANALYSIS_JOBS) <= _ANALYSIS_JOBS_MAX:
        return
    completed_keys = [
        k for k, v in _ANALYSIS_JOBS.items()
        if (v.get("proc") is not None and v["proc"].poll() is not None)
        or v.get("proc") is None
    ]
    evict_count = max(0, len(_ANALYSIS_JOBS) - _ANALYSIS_JOBS_MAX)
    for key in completed_keys[:evict_count]:
        _ANALYSIS_JOBS.pop(key, None)


def _study_analysis_output_dir(sessions_root: str, job_id: str) -> Path:
    root = (Path(sessions_root).resolve() / "study_analysis" / str(job_id)).resolve()
    root.parent.mkdir(parents=True, exist_ok=True)
    return root


def _write_attempt_ledger_csv(sessions_root: str, output_dir: Path) -> Optional[Path]:
    """Materialize the append-only attempt ledger for the stats CLI."""

    aggregate = _read_json_if_exists(str(Path(sessions_root).resolve() / "protocol_attempts.json"))
    attempts = aggregate.get("attempts") if isinstance(aggregate, dict) else None
    if not isinstance(attempts, list) or not attempts:
        return None
    rows = [row for row in attempts if isinstance(row, dict)]
    if not rows:
        return None
    fields = sorted({str(key) for row in rows for key in row.keys()})
    path = output_dir / "attempt_ledger.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})
    return path


def _run_study_statistics_job(sessions_root: str, output_dir: Path, job: Mapping[str, object]) -> Dict[str, object]:
    """Run the controlled statistical phase after confirmatory training."""

    status_path = output_dir / "statistics_status.json"
    raw_request = job.get("request")
    request = raw_request if isinstance(raw_request, dict) else {}
    if not bool(request.get("confirmatory")):
        not_requested: Dict[str, object] = {"status": "not_requested", "reason": "exploratory job"}
        save_json(not_requested, str(status_path))
        return not_requested
    manifest_path = output_dir / "confirmatory_run_manifest.json"
    manifest = _read_json_if_exists(str(manifest_path))
    oof = manifest.get("outer_oof_predictions") if isinstance(manifest, dict) else None
    oof_path = str(oof.get("path") or "") if isinstance(oof, dict) else ""
    ledger_path = _write_attempt_ledger_csv(sessions_root, output_dir)
    if not isinstance(manifest, dict) or not oof_path or ledger_path is None:
        result: Dict[str, object] = {
            "status": "blocked",
            "reason": "confirmatory statistics require outer OOF predictions and an attempted-trial ledger",
        }
        save_json(result, str(status_path))
        return result
    provenance_path = output_dir / "statistics_provenance.json"
    provenance = {
        "source_commit": manifest.get("source_commit"),
        "run_product_version": manifest.get("run_product_version"),
        "analysis_plan_id": manifest.get("analysis_plan_id"),
        "analysis_plan_sha256": manifest.get("analysis_plan_sha256"),
        "study_protocol_id": manifest.get("study_protocol_id"),
        "study_session_schema_version": manifest.get("study_session_schema_version"),
        "model_family": manifest.get("model_family"),
        "split_ledger_sha256": manifest.get("split_ledger_sha256"),
        "prediction_file_sha256": (oof or {}).get("sha256") if isinstance(oof, dict) else None,
    }
    save_json(provenance, str(provenance_path))
    stats_path = output_dir / "statistical_report.json"
    stats_log = output_dir / "statistics.log"
    argv = [
        sys.executable,
        "-m",
        "rvt_trainer.statistics",
        "--input",
        oof_path,
        "--reference-column",
        "ref_rr",
        "--estimate-column",
        "pred_rr_raw",
        "--analysis-plan",
        str(_REPO_ROOT / "quality" / "statistical-analysis-plan.json"),
        "--provenance-json",
        str(provenance_path),
        "--attempt-ledger",
        str(ledger_path),
        "--confirmatory",
        "--out",
        str(stats_path),
    ]
    with stats_log.open("w", encoding="utf-8", errors="replace") as handle:
        completed = subprocess.run(
            argv,
            cwd=str(_REPO_ROOT),
            stdout=handle,
            stderr=subprocess.STDOUT,
            check=False,
        )
    final_result: Dict[str, object] = {
        "status": "completed" if completed.returncode == 0 and stats_path.is_file() else "blocked",
        "exit_code": int(completed.returncode),
        "report_path": str(stats_path) if stats_path.is_file() else None,
        "log_path": str(stats_log),
    }
    save_json(final_result, str(status_path))
    return final_result


def _run_study_analysis_job_once(sessions_root: str, job_id: str) -> None:
    """Execute one explicit study-analysis request and persist truthful state."""

    job = _load_analysis_job(sessions_root, job_id)
    if not isinstance(job, dict):
        return
    if str(job.get("status") or "") in {"cancelling", "cancelled"} or job.get("cancel_requested") is True:
        return
    raw_request = job.get("request")
    request = raw_request if isinstance(raw_request, dict) else {}
    session_ids = [str(value) for value in request.get("session_ids", []) if str(value).strip()]
    if not session_ids:
        _update_analysis_job(
            sessions_root,
            job_id,
            {
                "status": "blocked",
                "error": "Select at least one recorded session before starting study analysis.",
                "last_line": "Blocked: no recorded session IDs were supplied.",
            },
        )
        return

    radar_paths: list[str] = []
    ref_paths: list[str] = []
    try:
        for session_id in session_ids:
            session_dir = _session_path(sessions_root, session_id)
            radar = session_dir / "radar.csv"
            ref = session_dir / "ref.csv"
            if not radar.is_file():
                raise FileNotFoundError(f"{session_id}: radar.csv is missing")
            if not ref.is_file():
                raise FileNotFoundError(f"{session_id}: ref.csv is missing")
            radar_paths.append(str(radar))
            ref_paths.append(str(ref))
    except (FileNotFoundError, ValueError) as exc:
        _update_analysis_job(
            sessions_root,
            job_id,
            {"status": "failed", "error": str(exc), "last_line": str(exc)},
        )
        return

    # Cancellation can arrive after cohort validation but before the child
    # process is spawned. Re-read the durable CAS state so a cancelled queued
    # job never consumes trainer resources.
    current = _load_analysis_job(sessions_root, job_id) or {}
    if str(current.get("status") or "") in {"cancelling", "cancelled"} or current.get("cancel_requested") is True:
        return

    output_dir = _study_analysis_output_dir(sessions_root, job_id)
    output_dir.mkdir(parents=True, exist_ok=True)
    family = str(job.get("model_family") or MODEL_FAMILY_GRADIENT_BOOSTING)
    objective_id = str(job.get("objective_id") or "")
    target = str(request.get("target") or ("rr" if objective_id == "objective_1_rr" else "hr"))
    
    args_file = output_dir / "analysis_args.txt"
    with args_file.open("w", encoding="utf-8") as f:
        f.write("train\n")
        f.write("--radar\n")
        for path in radar_paths:
            f.write(f"{path}\n")
        f.write("--ref\n")
        for path in ref_paths:
            f.write(f"{path}\n")
        f.write("--out\n")
        f.write(f"{output_dir}\n")
        f.write("--model-family\n")
        f.write(f"{family}\n")
        f.write("--targets\n")
        f.write(f"{target}\n")
        f.write("--no-plots\n")
        if bool(request.get("confirmatory")):
            f.write("--three-way-split\n")
            f.write("--confirmatory-evaluation\n")

    argv = [
        *_trainer_command_prefix(),
        f"@{args_file}",
    ]

    log_path = output_dir / "study_analysis.log"
    log_handle = None
    try:
        log_handle = log_path.open("w", encoding="utf-8", errors="replace")
        child_env = os.environ.copy()
        child_env["RVT_STUDY_ANALYSIS_JOB_ID"] = job_id
        proc = subprocess.Popen(
            argv,
            cwd=str(_REPO_ROOT),
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            env=child_env,
        )
    except OSError as exc:
        if log_handle is not None:
            try:
                log_handle.close()
            except Exception:
                pass
        _update_analysis_job(
            sessions_root,
            job_id,
            {"status": "failed", "error": str(exc), "last_line": str(exc)},
        )
        return

    with _STUDY_ANALYSIS_LOCK:
        _STUDY_ANALYSIS_PROCS[job_id] = proc
    try:
        _update_analysis_job(
            sessions_root,
            job_id,
            {
                "status": "running",
                "pid": proc.pid,
                "argv": argv,
                "output_dir": str(output_dir),
                "log_path": str(log_path),
                "started_at": _iso_now(),
                "progress_pct": 1,
                "last_line": "Trainer worker started.",
            },
        )
        while proc.poll() is None:
            progress = _read_json_if_exists(str(output_dir / "training_progress.json"))
            if isinstance(progress, dict):
                total = max(0, int(progress.get("n_estimators_total") or 0))
                done = max(0, int(progress.get("n_estimators_done") or 0))
                pct = min(99, int(round(done * 100 / total))) if total else 1
                _update_analysis_job(
                    sessions_root,
                    job_id,
                    {
                        "progress_pct": pct,
                        "phase": progress.get("phase"),
                        "last_line": f"{progress.get('phase') or progress.get('status') or 'running'}",
                    },
                )
            time.sleep(0.5)
        exit_code = int(proc.returncode or 0)
        current = _load_analysis_job(sessions_root, job_id) or {}
        if str(current.get("status") or "") in {"cancelling", "cancelled"}:
            return
        statistics = _run_study_statistics_job(sessions_root, output_dir, job) if exit_code == 0 else {"status": "not_run"}
        _finish_analysis_job(
            sessions_root,
            job_id,
            {
                "status": "completed" if exit_code == 0 else "failed",
                "exit_code": exit_code,
                "progress_pct": 100 if exit_code == 0 else int(current.get("progress_pct") or 0),
                "last_line": "Trainer worker completed." if exit_code == 0 else "Trainer worker failed; inspect study_analysis.log.",
                "completed_at": _iso_now(),
                "statistics_status": statistics.get("status"),
                "statistics_report": statistics.get("report_path"),
            },
        )
    except Exception as exc:
        try:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=3.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        try:
            _finish_analysis_job(
                sessions_root,
                job_id,
                {
                    "status": "failed",
                    "error": str(exc),
                    "last_line": "Study analysis worker failed before terminal status was written.",
                    "completed_at": _iso_now(),
                },
            )
        except Exception:
            pass
    finally:
        if log_handle is not None:
            try:
                log_handle.close()
            except Exception:
                pass
        with _STUDY_ANALYSIS_LOCK:
            _STUDY_ANALYSIS_PROCS.pop(job_id, None)


_CANONICAL_RUN_STUDY_ANALYSIS_JOB_ONCE = _run_study_analysis_job_once


def _run_study_analysis_job(sessions_root: str, job_id: str) -> None:
    acquired = False
    try:
        semaphore = monolith_override(
            "_STUDY_ANALYSIS_SEMAPHORE",
            _STUDY_ANALYSIS_SEMAPHORE,
            canonical=_CANONICAL_STUDY_ANALYSIS_SEMAPHORE,
        )
        acquired = semaphore.acquire(blocking=False)
        if not acquired:
            _update_analysis_job(
                sessions_root,
                job_id,
                {
                    "status": "blocked",
                    "error": "Another study analysis is already running; retry after it completes.",
                    "last_line": "Blocked by the single-worker analysis admission limit.",
                },
            )
            return
        run_once = monolith_override(
            "_run_study_analysis_job_once",
            _run_study_analysis_job_once,
            canonical=_CANONICAL_RUN_STUDY_ANALYSIS_JOB_ONCE,
        )
        run_once(sessions_root, job_id)
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"[:1000]
        try:
            _finish_analysis_job(
                sessions_root,
                job_id,
                {
                    "status": "failed",
                    "error": error,
                    "last_line": "Study analysis worker failed before terminal state was written.",
                    "completed_at": _iso_now(),
                },
            )
        except Exception as persistence_exc:
            # A disappearing/unwritable sessions root cannot be repaired by a
            # daemon thread.  Contain both failures so they never surface as an
            # unhandled thread exception, and retain the diagnostic in memory.
            _append_trainer_log(
                f"[STUDY_ANALYSIS] {job_id} could not persist failure state: "
                f"{type(persistence_exc).__name__}: {persistence_exc}"
            )
        _append_trainer_log(f"[STUDY_ANALYSIS] {job_id} failed: {error}")
    finally:
        if acquired:
            semaphore.release()


def _start_study_analysis_job(sessions_root: str, job: Mapping[str, object]) -> None:
    job_id = str(job.get("job_id") or "").strip()
    if not job_id:
        return
    with _STUDY_ANALYSIS_LOCK:
        existing = _STUDY_ANALYSIS_PROCS.get(job_id)
        if existing is not None and existing.poll() is None:
            return
    threading.Thread(
        target=_run_study_analysis_job,
        args=(sessions_root, job_id),
        name=f"study-analysis-{job_id}",
        daemon=True,
    ).start()


def _count_csv_data_rows(path: Path, limit: int = 61) -> int:
    if not path.exists():
        return 0
    count = 0
    try:
        with path.open("r", encoding="utf-8", errors="ignore") as f:
            first = True
            for line in f:
                if first:
                    first = False
                    continue
                if line.strip():
                    count += 1
                    if count >= limit:
                        break
    except Exception:
        return 0
    return count


def _auto_analyse_sentinel(session_dir: Path) -> Path:
    return session_dir / "analysis" / "_auto_analyse_running"


def _spawn_auto_analyse(session_dir: str, reason: str = "session_stop") -> Optional[Dict[str, object]]:
    if not FEATURE_FLAGS.get("enable_auto_analyse", True):
        return None
    root = Path(session_dir)
    radar = root / "radar.csv"
    ref = root / "ref.csv"
    out = root / "analysis"
    summary = out / "analyse_summary.json"
    sentinel = _auto_analyse_sentinel(root)
    if summary.exists() or sentinel.exists():
        return None
    if _count_csv_data_rows(radar) <= 60 or _count_csv_data_rows(ref) <= 0:
        return None
    free_mb = shutil.disk_usage(str(root)).free / (1024 * 1024)
    if free_mb < 500:
        msg = f"Disk space low ({free_mb:.0f} MB free); auto-analyse skipped"
        _append_trainer_log(f"[WARN] {msg}")
        return {"status": "skipped", "session_id": root.name, "reason": "disk_space_low", "message": msg}
    out.mkdir(parents=True, exist_ok=True)
    save_json(
        {
            "schema_version": CONTROL_API_SCHEMA_VERSION,
            "status": "running",
            "reason": reason,
            "session_id": root.name,
            "started_at": _iso_now(),
        },
        str(sentinel),
    )
    argv = [*_trainer_command_prefix(), "analyse", "--radar", str(radar), "--ref", str(ref), "--out", str(out)]

    def _runner():
        code = 1
        try:
            proc = subprocess.Popen(argv)
            _evict_completed_analysis_jobs()
            _ANALYSIS_JOBS[root.name] = {
                "job_id": root.name,
                "session_id": root.name,
                "analysis_dir": str(out),
                "started_at": _iso_now(),
                "pid": proc.pid,
                "proc": proc,
                "argv": argv,
            }
            code = proc.wait()
        finally:
            try:
                if sentinel.exists():
                    sentinel.unlink()
            except Exception:
                pass
            _append_trainer_log(f"[AUTO_ANALYSE] {root.name} exited with {code}")

    threading.Thread(target=_runner, daemon=True).start()
    return {"status": "started", "session_id": root.name, "analysis_dir": str(out), "reason": reason}


def _rerun_session_analysis(session_dir: str) -> Dict[str, object]:
    root = Path(session_dir)
    out = root / "analysis"
    out.mkdir(parents=True, exist_ok=True)
    status_path = out / "analyse_status.json"
    radar = root / "radar.csv"
    ref = root / "ref.csv"
    radar_rows = _count_csv_data_rows(radar)
    ref_rows = _count_csv_data_rows(ref)
    if radar_rows <= 0:
        payload = {
            "schema_version": CONTROL_API_SCHEMA_VERSION,
            "session_id": root.name,
            "status": "failed",
            "analysis_status": "missing_radar",
            "progress_pct": 0,
            "analysis_dir": str(out),
            "completed_at": _iso_now(),
            "radar_rows": radar_rows,
            "ref_rows": ref_rows,
            "last_line": "Radar CSV is missing or contains no data rows; analysis was not started.",
        }
        save_json(payload, str(status_path))
        return payload
    if ref_rows <= 0:
        ble_summary = _read_json_if_exists(str(root / "ref_ble_summary.json")) or {}
        message = "BLE reference is missing or empty; paired HR/RR analysis was skipped and radar data was preserved."
        if isinstance(ble_summary, dict) and ble_summary.get("error"):
            message = f"{message} BLE error: {ble_summary.get('error')}"
        payload = {
            "schema_version": CONTROL_API_SCHEMA_VERSION,
            "session_id": root.name,
            "status": "complete",
            "analysis_status": "radar_only",
            "progress_pct": 100,
            "analysis_dir": str(out),
            "completed_at": _iso_now(),
            "radar_rows": radar_rows,
            "ref_rows": ref_rows,
            "last_line": message,
        }
        save_json(payload, str(status_path))
        _write_session_manifest(str(root), [str(radar)], [], str(out), None)
        return payload
    try:
        if status_path.exists():
            status_path.unlink()
    except Exception:
        pass
    argv = [*_trainer_command_prefix(), "analyse", "--radar", str(radar), "--ref", str(ref), "--out", str(out)]
    proc = subprocess.Popen(argv)
    job_id = root.name
    started_at = _iso_now()
    _evict_completed_analysis_jobs()
    _ANALYSIS_JOBS[job_id] = {
        "job_id": job_id,
        "session_id": root.name,
        "analysis_dir": str(out),
        "started_at": started_at,
        "pid": proc.pid,
        "proc": proc,
        "argv": argv,
    }
    return {"status": "started", "job_id": job_id, "session_id": root.name, "pid": proc.pid, "analysis_dir": str(out), "started_at": started_at}


def _analysis_job_status(sessions_root: str, session_id: str) -> Dict[str, object]:
    job = _ANALYSIS_JOBS.get(session_id)
    session_dir = Path(sessions_root) / session_id
    summary_path = session_dir / "analysis" / "analyse_summary.json"
    status_path = session_dir / "analysis" / "analyse_status.json"
    sentinel_path = _auto_analyse_sentinel(session_dir)
    if job:
        proc = job.get("proc")
        code = proc.poll() if proc is not None else None
        status = "running" if code is None else ("complete" if code == 0 else "failed")
        payload = {
            "job_id": job.get("job_id"),
            "session_id": session_id,
            "status": status,
            "analysis_status": status,
            "pid": job.get("pid"),
            "started_at": job.get("started_at"),
            "analysis_dir": job.get("analysis_dir"),
            "exit_code": code,
            "progress_pct": 50 if code is None else (100 if code == 0 else 0),
            "last_line": "",
        }
        if code is not None:
            payload["completed_at"] = _iso_now()
        return _schema_wrap(payload)
    if sentinel_path.exists():
        sent = _read_json_if_exists(str(sentinel_path)) or {}
        return _schema_wrap({
            "session_id": session_id,
            "status": "running",
            "analysis_status": "running",
            "progress_pct": 35,
            "last_line": "auto-analyse is running",
            "sentinel": sent,
        })
    if summary_path.exists():
        return _schema_wrap({"session_id": session_id, "status": "complete", "analysis_status": "complete", "progress_pct": 100, "last_line": "analyse_summary.json exists"})
    status_payload = _read_json_if_exists(str(status_path))
    if isinstance(status_payload, dict):
        return _schema_wrap(status_payload)
    return _schema_wrap({"session_id": session_id, "status": "idle", "analysis_status": "idle", "progress_pct": 0, "last_line": "No analysis job is tracked for this session."})
