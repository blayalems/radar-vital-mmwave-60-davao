"""Interactive CLI commands for capture sessions and standalone dashboards."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from collections import deque
from types import SimpleNamespace
from typing import Deque, List

from rvt_trainer.api.common import (
    atomic_write_json as save_json,
    read_json_if_exists as _read_json_if_exists,
)
from rvt_trainer.constants import (
    DASHBOARD_VERSION,
    DEFAULT_RADAR_PORT,
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    FIRMWARE_VERSION_EXPECTED,
    SESSION_MANIFEST_SCHEMA_VERSION,
    SESSION_MANIFEST_VERSION,
    VERSION,
    _REPO_ROOT,
    _bold,
    _dim,
    _green,
    _report_stamp,
    _stream_subprocess_output,
    _terminate_subprocess,
    _yellow,
    warn,
)
from rvt_trainer.modeling import cmd_analyse
from rvt_trainer.session.live_dashboard import (
    _CSVAppendTracker,
    _dashboard_html_path,
    _format_dashboard_event,
    _render_live_dashboard,
    _start_dashboard_server,
    _write_live_dashboard_html,
    _write_live_dashboard_json,
)
from rvt_trainer.session.manifest import _write_session_manifest
from rvt_trainer.session.protocol_ledger import (
    ATTEMPT_LEDGER_SCHEMA_VERSION,
    allocate_attempt_id as _allocate_attempt_id,
    initialize_session_attempt as _initialize_session_attempt,
)
from rvt_trainer.session.storage import (
    _dashboard_analysis_payload,
    _next_session_dir,
    _print_and_save_session_report,
)
from rvt_trainer.session.study_contract import (
    merge_immutable_study_assignment as _merge_immutable_study_assignment,
    release_provenance as _release_provenance,
    validate_study_assignment as _validate_study_assignment,
)
from rvt_trainer.session.supervisor import (
    _check_stale_session_lock,
    _consume_supervisor_stop_request,
    _lock_path,
    _release_session_lock,
    _session_is_active,
    _write_session_lock,
)
from rvt_trainer.transport.serial import (
    auto_detect_radar_port as _auto_detect_radar_port,
    cmd_log,
)


def cmd_session(args):
    if str(getattr(args, "port", "")).strip().lower() in {"auto", "autodetect", "auto-detect"}:
        args.port = _auto_detect_radar_port(DEFAULT_RADAR_PORT)
    auto_session_dir = str(args.session_dir).strip().lower() == "auto"
    if str(args.session_dir).strip().lower() == "auto":
        session_dir = _next_session_dir(os.path.abspath(args.sessions_root), args.session_prefix, args.session_digits)
    else:
        session_dir = os.path.abspath(args.session_dir)
    os.makedirs(session_dir, exist_ok=True)
    lock_root = os.path.abspath(args.sessions_root) if auto_session_dir else os.path.dirname(session_dir)
    study_payload = {
        "participant_id": getattr(args, "participant_id", None),
        "trial_id": getattr(args, "trial_id", None),
        "condition_id": getattr(args, "condition_id", None),
        "distance_m": getattr(args, "distance_m", None),
        "barrier_type": getattr(args, "barrier_type", None),
        "trial_number": getattr(args, "trial_number", None),
        "planned_duration_s": getattr(args, "planned_duration_s", None),
        "study_classification": getattr(args, "study_classification", None),
        "duration_s": getattr(args, "duration_s", None),
    }
    study_assignment = _validate_study_assignment(
        study_payload,
        sessions_root=lock_root,
    )
    attempt_id = str(getattr(args, "attempt_id", None) or _allocate_attempt_id())
    attempt_type = str(getattr(args, "attempt_type", None) or "subject")
    _check_stale_session_lock(lock_root)
    if _session_is_active(lock_root):
        raise RuntimeError(f"SESSION_IN_PROGRESS: active session lock exists at {_lock_path(lock_root)}")
    _write_session_lock(lock_root, session_dir)
    lock_acquired = True
    analysis_dir = os.path.join(session_dir, "analysis")
    radar_out = os.path.join(session_dir, "radar.csv")
    ref_out = os.path.join(session_dir, "ref.csv")
    initial_manifest = {
        "schema_version": SESSION_MANIFEST_SCHEMA_VERSION,
        "manifest_version": SESSION_MANIFEST_VERSION,
        "generated_at": _report_stamp(),
        "attempt_id": attempt_id,
        "attempt_type": attempt_type,
        "attempt_ledger_schema_version": ATTEMPT_LEDGER_SCHEMA_VERSION,
        "study_session_schema_version": study_assignment.pop("schema_version"),
        **study_assignment,
        **_release_provenance(
            product_version=VERSION,
            trainer_version=VERSION,
            dashboard_version=DASHBOARD_VERSION,
            firmware_expected=FIRMWARE_VERSION_EXPECTED,
            serial_width_expected=EXPECTED_RADAR_LOG_COLUMN_COUNT,
            model_family=getattr(args, "model_family", None),
            model_bundle=getattr(args, "model_bundle", None),
        ),
        "auto_analysed": False,
        "tags": [],
        "notes_count": 0,
        "subject_profile_id": getattr(args, "subject_profile_id", "adult_default"),
    }
    initial_manifest["capture_provenance"] = {
        "captured_at": initial_manifest["generated_at"],
        "source": "session_start",
        "product_version": initial_manifest.get("product_version"),
        "trainer_version": initial_manifest.get("trainer_version"),
        "dashboard_version": initial_manifest.get("dashboard_version"),
        "firmware_expected": initial_manifest.get("firmware_expected"),
        "firmware_observed": initial_manifest.get("firmware_observed"),
        "serial_protocol": initial_manifest.get("serial_protocol"),
        "serial_width_expected": initial_manifest.get("serial_width_expected"),
        "source_commit": initial_manifest.get("source_commit"),
        "model_family": initial_manifest.get("model_family"),
        "model_bundle": initial_manifest.get("model_bundle"),
        "participant_id": initial_manifest.get("participant_id"),
        "logical_trial_id": initial_manifest.get("logical_trial_id"),
        "attempt_id": attempt_id,
    }
    initial_manifest["analysis_runs"] = []
    existing_manifest = _read_json_if_exists(
        os.path.join(session_dir, "session_manifest.json")
    ) or {}
    initial_manifest = _merge_immutable_study_assignment(
        existing_manifest,
        initial_manifest,
    )
    try:
        save_json(initial_manifest, os.path.join(session_dir, "session_manifest.json"))
        _initialize_session_attempt(session_dir, initial_manifest)
    except Exception:
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
        raise

    ble_cmd = [sys.executable, "-u", "-m", "rvt_trainer", "ble_reflog",
               "--out", ref_out]
    if getattr(args, "address", None):
        ble_cmd += ["--address", args.address]
    if getattr(args, "ble_profile", None):
        ble_cmd += ["--ble-profile", args.ble_profile]
    if getattr(args, "notify_char", None):
        ble_cmd += ["--notify-char", args.notify_char]
    if args.echo_raw:
        ble_cmd.append("--echo-raw")
    if args.probe_services:
        ble_cmd.append("--probe-services")
    if args.subscribe_all:
        ble_cmd.append("--subscribe-all")

    dashboard_enabled = bool(getattr(args, "live_dashboard", False) or getattr(args, "open_dashboard", True))
    radar_tracker = _CSVAppendTracker(radar_out, keep_rows=480)
    ref_tracker = _CSVAppendTracker(ref_out, keep_rows=480)
    raw_tracker = _CSVAppendTracker(os.path.join(session_dir, "ref_ble_raw.csv"), keep_rows=480)
    recent_events: Deque[str] = deque(["[INFO] Session initialized"], maxlen=24)
    event_lock = threading.Lock()
    dashboard_server = None

    def event_cb(prefix: str, line: str):
        msg = _format_dashboard_event(prefix, line)
        with event_lock:
            recent_events.append(msg)

    print(_bold("\n=== Combined Session Runner ==="))
    print(f"Session dir: {session_dir}")
    print(f"Radar CSV:   {radar_out}")
    print(f"Ref CSV:     {ref_out}")
    print(f"Dashboard:   {_dashboard_html_path(session_dir)}")
    if args.duration_s:
        print(f"Auto-stop:   {args.duration_s:.1f}s")
    else:
        print("Auto-stop:   manual (Ctrl-C)")
    print(f"Analyse:     {analysis_dir}\n")

    try:
        if dashboard_enabled:
            _write_live_dashboard_html(session_dir)
            _write_live_dashboard_json(session_dir, "starting", 0.0, args.duration_s, radar_tracker, ref_tracker, raw_tracker, ["[INFO] Session starting"], analysis_summary=_dashboard_analysis_payload(session_dir))
            try:
                dashboard_server = _start_dashboard_server(session_dir, port=int(getattr(args, "dashboard_port", 8765) or 0))
                print(_green(f"[DASHBOARD] {dashboard_server.url}"))
                if getattr(args, "open_dashboard", True):
                    try:
                        import webbrowser
                        webbrowser.open(dashboard_server.url)
                    except Exception as e:
                        warn(f"Could not open dashboard in browser: {e}")
            except Exception as e:
                port = int(getattr(args, "dashboard_port", 8765) or 0)
                if port != 0:
                    print(f"Port {port} in use. Stop the other instance or pass --dashboard-port <N>.", file=sys.stderr)
                    raise SystemExit(2)
                dashboard_enabled = False
                warn(f"Could not start dashboard server: {e}")
    except Exception:
        if dashboard_server is not None:
            dashboard_server.stop()
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
        raise

    radar_proc = None
    ble_proc = None
    radar_thread = None
    ble_thread = None
    radar_done_event = threading.Event()
    radar_error: List[str] = []
    stop_reason = "completed"
    ble_failure_reported = False
    start_t = time.time()

    def run_radar_logger_inline():
        try:
            cmd_log(SimpleNamespace(port=args.port, out=radar_out, duration_s=args.duration_s))
        except BaseException as e:
            radar_error.append(str(e))
            event_cb("[RADAR]", f"inline logger exited: {e}")
        finally:
            radar_done_event.set()

    def csv_data_row_count(path: str) -> int:
        if not os.path.exists(path):
            return 0
        try:
            with open(path, "r", encoding="utf-8", errors="ignore", newline="") as f:
                return max(0, sum(1 for _ in f) - 1)
        except Exception:
            return 0

    try:
        radar_thread = threading.Thread(
            target=run_radar_logger_inline,
            daemon=True
        )
        radar_thread.start()

        ble_proc = subprocess.Popen(
            ble_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, universal_newlines=True, cwd=str(_REPO_ROOT)
        )
        ble_thread = threading.Thread(
            target=_stream_subprocess_output,
            args=(ble_proc.stdout, "[BLE]", event_cb, not dashboard_enabled),
            daemon=True
        )
        ble_thread.start()

        while True:
            time.sleep(max(0.15, float(getattr(args, "dashboard_refresh_s", 1.0))))
            elapsed = time.time() - start_t
            if dashboard_enabled:
                radar_tracker.poll()
                ref_tracker.poll()
                raw_tracker.poll()
                _render_live_dashboard(session_dir, start_t, args.duration_s, radar_tracker, ref_tracker, raw_tracker,
                                       recent_events, event_lock, "collecting", analysis_summary=_dashboard_analysis_payload(session_dir))

            if args.duration_s and elapsed >= args.duration_s:
                stop_reason = f"duration reached ({args.duration_s:.1f}s)"
                break

            radar_done = radar_done_event.is_set()
            ble_done = (ble_proc is not None and ble_proc.poll() is not None)
            if radar_done:
                if radar_error:
                    stop_reason = f"radar logger exited: {radar_error[-1]}"
                elif args.duration_s and elapsed >= max(0.0, float(args.duration_s) - 1.0):
                    stop_reason = f"duration reached ({args.duration_s:.1f}s)"
                else:
                    stop_reason = "radar logger exited"
                break
            if ble_done:
                if not ble_failure_reported:
                    ble_failure_reported = True
                    code = ble_proc.returncode
                    msg = f"[BLE] Reference logger exited with code {code}; radar collection continues."
                    warn(msg)
                    with event_lock:
                        recent_events.append(msg)
                ble_proc = None

    except KeyboardInterrupt:
        stop_reason = "user interrupt"
        print(_yellow("\n[SESSION] Stop requested by user."))
    finally:
        print(_yellow(f"[SESSION] Stopping child processes ({stop_reason})..."))
        _terminate_subprocess(radar_proc, "RADAR")
        _terminate_subprocess(ble_proc, "BLE")
        if radar_thread is not None:
            radar_thread.join(timeout=3.0)
        if ble_thread is not None:
            ble_thread.join(timeout=3.0)
        radar_tracker.poll()
        ref_tracker.poll()
        raw_tracker.poll()
        if dashboard_enabled:
            _render_live_dashboard(session_dir, start_t, args.duration_s, radar_tracker, ref_tracker, raw_tracker,
                                   recent_events, event_lock, f"stopped ({stop_reason})", analysis_summary=_dashboard_analysis_payload(session_dir))

    supervisor_stop_request = _consume_supervisor_stop_request(session_dir)
    radar_rows = csv_data_row_count(radar_out)
    ref_rows = csv_data_row_count(ref_out)
    radar_paths = [radar_out] if radar_rows > 0 else []
    ref_paths = [ref_out] if ref_rows > 0 else []
    if radar_rows <= 0:
        warn("Radar CSV was not created or has no data rows; skipping analyse.")
        _write_session_manifest(session_dir, radar_paths, ref_paths, analysis_dir, None)
        if dashboard_server is not None:
            dashboard_server.stop()
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
        return
    if ref_rows <= 0:
        warn("BLE reference CSV was not created or has no data rows; saved radar-only session and skipped paired analyse.")
        _write_session_manifest(session_dir, radar_paths, ref_paths, analysis_dir, None)
        if dashboard_server is not None:
            dashboard_server.stop()
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
        return

    if (
        isinstance(supervisor_stop_request, dict)
        and bool(supervisor_stop_request.get("suppress_inline_auto_analyse"))
    ):
        warn(
            "Inline auto analyse suppressed for a supervisor-owned stop; "
            "the parent will apply the requested post-reap policy."
        )
        _write_session_manifest(session_dir, radar_paths, ref_paths, analysis_dir, None)
        if dashboard_server is not None:
            dashboard_server.stop()
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
        return

    if not getattr(args, "auto_analyse", True):
        warn("Auto analyse disabled by --no-auto-analyse.")
        _write_session_manifest(session_dir, radar_paths, ref_paths, analysis_dir, None)
        if dashboard_server is not None:
            dashboard_server.stop()
        if lock_acquired:
            _release_session_lock(lock_root)
        return

    print(_bold("\n=== Auto Analyse ==="))
    analyse_args = argparse.Namespace(
        radar=[radar_out],
        ref=[ref_out],
        out=analysis_dir,
        tolerance_s=args.tolerance_s,
        merge_direction=args.merge_direction,
        ref_offset_s=None,
        auto_align_start=args.auto_align_start,
        feature_mode=args.feature_mode,
        phase_unit=args.phase_unit,
        heart_fft_window_s=args.heart_fft_window_s,
        breath_fft_window_s=args.breath_fft_window_s,
        no_plots=args.no_plots,
        annotate_bland_altman=args.annotate_bland_altman,
        golden_json=args.golden_json,
        golden_hr_rmse_tol=args.golden_hr_rmse_tol,
        golden_hr_r_tol=args.golden_hr_r_tol,
    )
    try:
        cmd_analyse(analyse_args)
    except Exception as e:
        warn(f"Auto analyse failed: {e}")
        if dashboard_server is not None:
            dashboard_server.stop()
        if lock_acquired:
            _release_session_lock(lock_root)
            lock_acquired = False
        return

    _print_and_save_session_report(session_dir, analysis_dir, radar_out, ref_out)
    analysis_payload = _dashboard_analysis_payload(session_dir)
    with event_lock:
        recent_events.append("[ANALYSIS] Summary ready")
    if dashboard_enabled:
        radar_tracker.poll()
        ref_tracker.poll()
        raw_tracker.poll()
        _render_live_dashboard(session_dir, start_t, args.duration_s, radar_tracker, ref_tracker, raw_tracker,
                               recent_events, event_lock, "analysis complete", analysis_summary=analysis_payload)
        hold_s = max(0.0, float(getattr(args, "dashboard_hold_s", 6.0) or 0.0))
        if dashboard_server is not None and hold_s > 0:
            print(_dim(f"[DASHBOARD] Holding live server for {hold_s:.1f}s so the browser can pull the final analysis summary..."))
            time.sleep(hold_s)
    if not dashboard_enabled:
        try:
            radar_tracker.poll()
            ref_tracker.poll()
            raw_tracker.poll()
            _write_live_dashboard_html(session_dir)
            _write_live_dashboard_json(session_dir, "analysis complete", time.time() - start_t, 0.0,
                                       radar_tracker, ref_tracker, raw_tracker, list(recent_events),
                                       analysis_summary=analysis_payload)
        except Exception as e:
            warn(f"Final dashboard artifact write failed: {e}")
    analyse_summary = _read_json_if_exists(os.path.join(analysis_dir, "analyse_summary.json")) or {}
    _write_session_manifest(
        session_dir,
        [radar_out],
        [ref_out],
        analysis_dir,
        analyse_summary.get("fw_truthfulness") if isinstance(analyse_summary, dict) else None,
    )
    if dashboard_server is not None:
        print(_dim(f"[DASHBOARD] Served live during the run at {dashboard_server.url}"))
        dashboard_server.stop()
    if lock_acquired:
        _release_session_lock(lock_root)


def cmd_dashboard(args):
    session_dir = os.path.abspath(args.session_dir)
    radar_tracker = _CSVAppendTracker(os.path.join(session_dir, "radar.csv"), keep_rows=480)
    ref_tracker = _CSVAppendTracker(os.path.join(session_dir, "ref.csv"), keep_rows=480)
    raw_tracker = _CSVAppendTracker(os.path.join(session_dir, "ref_ble_raw.csv"), keep_rows=480)
    recent_events: Deque[str] = deque(["[INFO] Watching existing CSV files"], maxlen=24)
    event_lock = threading.Lock()
    _write_live_dashboard_html(session_dir)
    dashboard_server = None
    try:
        dashboard_server = _start_dashboard_server(session_dir, port=int(getattr(args, "dashboard_port", 8765) or 0))
        print(_green(f"[DASHBOARD] {dashboard_server.url}"))
        if getattr(args, "open_dashboard", True):
            try:
                import webbrowser
                webbrowser.open(dashboard_server.url)
            except Exception as e:
                warn(f"Could not open dashboard in browser: {e}")
    except Exception as e:
        port = int(getattr(args, "dashboard_port", 8765) or 0)
        if port != 0:
            print(f"Port {port} in use. Stop the other instance or pass --dashboard-port <N>.", file=sys.stderr)
            raise SystemExit(2)
        warn(f"Could not start dashboard server: {e}")
    start_t = time.time()
    try:
        while True:
            time.sleep(max(0.15, float(args.dashboard_refresh_s)))
            radar_tracker.poll()
            ref_tracker.poll()
            raw_tracker.poll()
            _render_live_dashboard(session_dir, start_t, args.duration_s, radar_tracker, ref_tracker, raw_tracker,
                                   recent_events, event_lock, "watching", analysis_summary=_dashboard_analysis_payload(session_dir))
            if args.duration_s and (time.time() - start_t) >= args.duration_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if dashboard_server is not None:
            dashboard_server.stop()
