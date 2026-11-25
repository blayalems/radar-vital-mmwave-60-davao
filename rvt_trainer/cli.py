"""CLI entrypoints for the Radar Vital trainer."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from rvt_trainer.api.common import (
    nan_safe,
    read_json_if_exists as _read_json_if_exists,
)
from rvt_trainer.api.server import _start_control_server, cmd_serve
from rvt_trainer.constants import (
    CONTROL_API_SCHEMA_VERSION,
    DEFAULT_BLE_ADDRESS,
    DEFAULT_RADAR_PORT,
    MODEL_FAMILIES,
    MODEL_FAMILY_GRADIENT_BOOSTING,
    SESSION_MANIFEST_SCHEMA_VERSION,
    VERSION,
    _bold,
    _dim,
    _green,
    _red,
    _yellow,
    write_text_report,
)
from rvt_trainer.modeling import (
    cmd_align,
    cmd_analyse,
    cmd_compare,
    cmd_predict,
    cmd_sweep,
    cmd_train,
)
from rvt_trainer.session.commands import cmd_dashboard, cmd_session
from rvt_trainer.transport.ble import cmd_ble_analyse_raw, cmd_ble_reflog
from rvt_trainer.transport.serial import cmd_flash, cmd_log, cmd_reflog

_QUICKSTART_TEXT = """
+==================================================================+
|     Radar Vital Trainer v{ver} - Quick-Start Guide              |
+==================================================================+

RECOMMENDED THESIS WORKFLOW (single command per session)

STEP 0 - Check your setup
  python radar_vital_trainer.py doctor

STEP 1 - Run one full session
  python radar_vital_trainer.py session \
    --port COM10 \
    --address 10:22:33:9E:8F:63 \
    --duration-s 480 \
    --open-dashboard

  What this does automatically:
    - starts radar logging
    - starts BLE oximeter reference logging
    - starts a localhost dashboard server
    - opens the Material 3 Expressive live dashboard in your browser
    - stops both collectors together
    - runs analyse immediately after capture
    - prints a quick session report right away

  Session tips:
    - Sit still and face the radar chest-on
    - Use a fixed distance (recommended default: ~60 cm)
    - Keep the pulse oximeter on the same finger each run
    - Avoid fans, talking, and large body motion
    - Start with 8-minute sessions (480 s)

STEP 2 - Review the outputs
  In the new session folder you will get:
    - radar.csv
    - ref.csv
    - ref_ble_raw.csv
    - ref_ble_summary.json
    - live_dashboard.html
    - live_dashboard.json
    - analysis/analyse_summary.json
    - analysis/analyse_report.txt
    - analysis/analyse_report.html
    - session_quick_report.txt

STEP 3 - Compare sessions after you collect several runs
  python radar_vital_trainer.py compare \
    --sessions-dir sessions/ --out report.html

STEP 4 - Train a correction model after baseline quality is acceptable
  python radar_vital_trainer.py train \
    --radar sessions/s01/radar.csv --ref sessions/s01/ref.csv \
    --feature-mode core --require-baseline-gate \
    --out model_s01/

ADVANCED / MANUAL WORKFLOW

Manual radar logging:
  python radar_vital_trainer.py log --port COM10 --out sessions/s01/radar.csv

Manual BLE reference logging:
  python radar_vital_trainer.py ble_reflog \
    --out sessions/s01/ref.csv \
    --address 10:22:33:9E:8F:63 \
    --ble-profile ailink_oximeter \
    --notify-char 0000ffe2-0000-1000-8000-00805f9b34fb

Manual analysis:
  python radar_vital_trainer.py analyse \
    --radar sessions/s01/radar.csv --ref sessions/s01/ref.csv \
    --out sessions/s01/analysis/

DSP sweep:
  python radar_vital_trainer.py sweep \
    --sweep-json params_sweep.json \
    --port COM10 --sketch radar_vital.ino \
    --out-dir sweep_results/

Run  python radar_vital_trainer.py doctor  to verify your environment.
For BLE pulse oximeter logging, also install:  pip install bleak
""".format(ver=VERSION)


def cmd_quickstart(args):
    print(_QUICKSTART_TEXT)


def cmd_doctor(args):
    if getattr(args, "json", False):
        checks = []

        def add_check(check_id, ok, detail=""):
            checks.append({"id": check_id, "status": "ok" if ok else "fail", "detail": detail})

        for name, import_name in [("numpy", "numpy"), ("pandas", "pandas"), ("scipy", "scipy"), ("scikit-learn", "sklearn"), ("matplotlib", "matplotlib"), ("pyserial", "serial")]:
            spec = importlib.util.find_spec(import_name)
            add_check(import_name, spec is not None, name if spec is not None else f"pip install {name}")
        cli_path = shutil.which("arduino-cli")
        add_check("arduino_cli", bool(cli_path), cli_path or "install arduino-cli and esp32 core")
        add_check("control_api_schema", True, CONTROL_API_SCHEMA_VERSION)
        legacy = []
        sessions_dir = Path("sessions")
        if sessions_dir.exists():
            for manifest_path in sessions_dir.glob("*/session_manifest.json"):
                manifest = _read_json_if_exists(str(manifest_path)) or {}
                if manifest.get("schema_version") != SESSION_MANIFEST_SCHEMA_VERSION:
                    legacy.append(manifest_path.parent.name)
        add_check("legacy_session_manifests", not legacy, "none" if not legacy else ",".join(legacy[:20]))
        print(json.dumps(nan_safe({"schema_version": CONTROL_API_SCHEMA_VERSION, "version": VERSION, "checks": checks}), indent=2, allow_nan=False))
        return
    print(_bold(f"\n=== Radar Vital Trainer v{VERSION} - Dependency Check ===\n"))
    ok_all = True

    def check_pkg(name, import_name=None, extra=""):
        nonlocal ok_all
        mod = importlib.util.find_spec(import_name or name)
        if mod is not None:
            try:
                m = __import__(import_name or name)
                ver = getattr(m, "__version__", "?")
            except Exception:
                ver = "?"
            print(f"  {_green('[OK]')}  {name:<20} {_dim(ver)}")
            return True
        else:
            print(f"  {_red('[MISSING]')}  {name:<20} {_yellow('pip install ' + name)}{extra}")
            ok_all = False
            return False

    print(_bold("Core (required for analyse / train / predict):"))
    check_pkg("numpy")
    check_pkg("pandas")
    check_pkg("scipy")
    check_pkg("scikit-learn", "sklearn")
    check_pkg("matplotlib")

    print(_bold("\nSerial (required for log / reflog / flash / sweep):"))
    check_pkg("pyserial", "serial")

    print(_bold("\nHardware (required for flash / sweep):"))
    cli_path = shutil.which("arduino-cli")
    if cli_path:
        try:
            result = subprocess.run(["arduino-cli", "version"], capture_output=True, text=True, timeout=5)
            ver_line = result.stdout.strip().split("\n")[0] if result.stdout else "?"
        except Exception:
            ver_line = "found"
        print(f"  {_green('[OK]')}  {'arduino-cli':<20} {_dim(ver_line)}")
    else:
        print(f"  {_red('[MISSING]')}  {'arduino-cli':<20} "
              f"{_yellow('https://arduino.github.io/arduino-cli/latest/installation/')}")
        print(f"            {'':20} After installing: arduino-cli core install esp32:esp32")
        ok_all = False

    print(_bold("\nOptional (for TFLite embedded export):"))
    mod = importlib.util.find_spec("tensorflow")
    if mod:
        try:
            import tensorflow as tf
            print(f"  {_green('[OK]')}  {'tensorflow':<20} {_dim(tf.__version__)}")
        except Exception:
            print(f"  {_green('[OK]')}  {'tensorflow':<20} {_dim('(version unknown)')}")
    else:
        print(f"  {_dim('[--]')}  {'tensorflow':<20} not installed  "
              f"{_dim('(optional - pip install tensorflow)')}")

    print()
    if ok_all:
        print(_green("All required dependencies satisfied. You are ready to go!"))
        print(f"Run  {_bold('python radar_vital_trainer.py quickstart')}  for the workflow guide.\n")
    else:
        print(_red("Some required dependencies are missing. Install them and re-run doctor.\n"))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        fromfile_prefix_chars='@',
        description=f"Radar Vital Trainer v{VERSION} - full session pipeline + ML training",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Quick start:\n"
            "  python radar_vital_trainer.py doctor      # check dependencies\n"
            "  python radar_vital_trainer.py quickstart  # full workflow guide\n"
        ),
    )
    parser.add_argument("--version", action="version", version=VERSION)
    sub = parser.add_subparsers(dest="cmd", required=True)

    # -- doctor ----------------------------------------------------------------
    p_doc = sub.add_parser("doctor", help="check all dependencies and print install instructions")
    p_doc.add_argument("--json", action="store_true", help="emit machine-readable dependency/session checks")
    p_doc.set_defaults(func=cmd_doctor)

    # -- quickstart ------------------------------------------------------------
    p_qs = sub.add_parser("quickstart", help="print the full new-user workflow guide")
    p_qs.set_defaults(func=cmd_quickstart)

    # -- log -------------------------------------------------------------------
    p_log = sub.add_parser("log",
        help="capture serial DATA lines to CSV (run in Terminal A during session)")
    p_log.add_argument("--port", default=DEFAULT_RADAR_PORT,
                       help=f"serial port (default: {DEFAULT_RADAR_PORT}), e.g. COM10 (Windows) or /dev/ttyACM0 (Linux/Mac)")
    p_log.add_argument("--out",  default="session.csv",
                       help="output CSV path (default: session.csv)")
    p_log.set_defaults(func=cmd_log)

    # -- reflog ----------------------------------------------------------------
    p_rl = sub.add_parser("reflog",
        help="live keyboard HR/RR reference logger (run in Terminal B during session)")
    p_rl.add_argument("--out", default="ref.csv",
                      help="output reference CSV path (default: ref.csv)")
    p_rl.set_defaults(func=cmd_reflog)

    # -- ble_reflog ------------------------------------------------------------
    p_ble = sub.add_parser("ble_reflog",
        help="capture BLE pulse oximeter data into trainer CSV + raw protocol logs")
    p_ble.add_argument("--out", default="ref.csv",
                       help="parsed reference CSV path (default: ref.csv)")
    p_ble.add_argument("--raw-out", default=None,
                       help="raw BLE packet CSV path (default: alongside --out)")
    p_ble.add_argument("--summary-out", default=None,
                       help="BLE session summary JSON path (default: alongside --out)")
    p_ble.add_argument("--address", default=DEFAULT_BLE_ADDRESS,
                       help=f"BLE MAC address / Windows DEV token (default: {DEFAULT_BLE_ADDRESS})")
    p_ble.add_argument("--name-hint", nargs="*", default=None,
                       help="scan name substrings, e.g. Oximeter Wilcare PC-60")
    p_ble.add_argument("--notify-char", default=None,
                       help="preferred notification characteristic UUID (auto-ranked if omitted)")
    p_ble.add_argument("--ble-profile", choices=["none", "ailink_oximeter"], default="none",
                       help="apply a known BLE field map preset (default: none)")
    p_ble.add_argument("--subscribe-all", action="store_true",
                       help="subscribe to all notify-capable characteristics and log them all")
    p_ble.add_argument("--scan-timeout", type=float, default=8.0,
                       help="BLE scan timeout in seconds when --address is not supplied")
    p_ble.add_argument("--duration-s", type=float, default=None,
                       help="optional fixed logging duration in seconds")
    p_ble.add_argument("--hr-byte-index", type=int, default=None,
                       help="explicit byte index containing HR")
    p_ble.add_argument("--hr-scale", type=float, default=None,
                       help="scale factor applied to the HR byte (default: 1.0, or profile preset)")
    p_ble.add_argument("--rr-byte-index", type=int, default=None,
                       help="explicit byte index containing RR if your BLE device provides it")
    p_ble.add_argument("--rr-scale", type=float, default=None,
                       help="scale factor applied to the RR byte (example: 0.1 for 17.0 stored as 170)")
    p_ble.add_argument("--spo2-byte-index", type=int, default=None,
                       help="explicit byte index containing SpO2 if available")
    p_ble.add_argument("--spo2-scale", type=float, default=None,
                       help="scale factor applied to the SpO2 byte (default: 1.0, or profile preset)")
    p_ble.add_argument("--pi-byte-index", type=int, default=None,
                       help="explicit byte index containing perfusion index (PI) if available")
    p_ble.add_argument("--pi-scale", type=float, default=None,
                       help="scale factor applied to the PI byte (example: 0.1 for PI x10)")
    p_ble.add_argument("--auto-parse", action="store_true",
                       help="allow heuristic HR/SpO2 guesses when explicit byte indices are unknown")
    p_ble.add_argument("--raw-only", action="store_true",
                       help="disable parsed HR/RR inference; capture raw BLE only")
    p_ble.add_argument("--write-char", action="append", default=None,
                       help="characteristic UUID to write to before or during capture; repeatable")
    p_ble.add_argument("--write-hex", action="append", default=None,
                       help="hex payload for the matching --write-char; repeatable")
    p_ble.add_argument("--write-with-response", action="store_true",
                       help="use GATT write-with-response for all configured writes")
    p_ble.add_argument("--post-write-wait-ms", type=int, default=250,
                       help="wait after each write operation in milliseconds (default: 250)")
    p_ble.add_argument("--min-log-interval-ms", type=int, default=250,
                       help="minimum interval between parsed log rows (default: 250 ms)")
    p_ble.add_argument("--dedupe", action="store_true",
                       help="skip rows when parsed HR/RR did not change")
    p_ble.add_argument("--echo-raw", action="store_true",
                       help="print raw packets even when no valid HR/RR is parsed")
    p_ble.add_argument("--probe-services", action="store_true",
                       help="print BLE services/characteristics before subscribing")
    p_ble.set_defaults(func=cmd_ble_reflog)

    # -- ble_analyse_raw -------------------------------------------------------
    p_bar = sub.add_parser("ble_analyse_raw",
        help="rank likely HR/RR/SpO2 fields from a raw BLE packet CSV")
    p_bar.add_argument("--raw", required=True,
                       help="raw BLE CSV produced by ble_reflog")
    p_bar.add_argument("--out", default="ble_raw_analysis",
                       help="output directory for ranked field candidates")
    p_bar.add_argument("--sender", default=None,
                       help="optional source_uuid/sender filter")
    p_bar.add_argument("--top-k", type=int, default=5,
                       help="number of top candidates to print per source (default: 5)")
    p_bar.set_defaults(func=cmd_ble_analyse_raw)


    # -- session ---------------------------------------------------------------
    p_ss = sub.add_parser("session",
        help="run radar logging + BLE reference logging together, then auto-analyse")
    p_ss.add_argument("--session-dir", default="auto",
                      help="session directory that will receive radar.csv, ref.csv, analysis/ (default: auto-increment)")
    p_ss.add_argument("--sessions-root", default="sessions",
                      help="root directory used when --session-dir auto (default: sessions)")
    p_ss.add_argument("--session-prefix", default="s",
                      help="prefix for auto session names (default: s)")
    p_ss.add_argument("--session-digits", type=int, default=2,
                      help="zero-padding for auto session names (default: 2)")
    p_ss.add_argument("--port", default=DEFAULT_RADAR_PORT,
                      help=f"serial port for the radar logger (default: {DEFAULT_RADAR_PORT})")
    p_ss.add_argument("--address", default=DEFAULT_BLE_ADDRESS,
                      help=f"BLE MAC address / Windows DEV token for the oximeter (default: {DEFAULT_BLE_ADDRESS})")
    p_ss.add_argument("--ble-profile", default="ailink_oximeter",
                      choices=["none", "ailink_oximeter"],
                      help="BLE parser profile (default: ailink_oximeter)")
    p_ss.add_argument("--notify-char", default="0000ffe2-0000-1000-8000-00805f9b34fb",
                      help="notify characteristic UUID (default: working AiLink/Wilcare FFE2)")
    p_ss.add_argument("--duration-s", type=float, default=None,
                      help="optional auto-stop duration in seconds; omit to stop with Ctrl-C")
    p_ss.add_argument("--subject-profile-id", default="adult_default",
                      help="subject profile id from subject_profiles.json (default: adult_default)")
    p_ss.add_argument("--participant-id", default=None,
                      help="pseudonymous study participant code, for example P-001")
    p_ss.add_argument("--trial-id", default=None,
                      help="immutable study trial identifier")
    p_ss.add_argument("--condition-id", default=None,
                      help="condition identifier derived from distance/barrier, for example d060_none")
    p_ss.add_argument("--distance-m", type=float, default=None,
                      help="study distance in metres (0.5 through 3.0)")
    p_ss.add_argument("--barrier-type", choices=["none", "cardboard"], default=None,
                      help="study barrier condition")
    p_ss.add_argument("--trial-number", type=int, default=None,
                      help="within-condition trial number")
    p_ss.add_argument("--planned-duration-s", type=float, default=None,
                      help="protocol-planned capture duration")
    p_ss.add_argument("--study-classification",
                      choices=["confirmatory", "exploratory"], default=None,
                      help="study protocol classification; required when study fields are supplied")
    p_ss.add_argument("--model-family", choices=sorted(MODEL_FAMILIES), default=None,
                      help="model family active during inference, if any")
    p_ss.add_argument("--model-bundle", default=None,
                      help="immutable model-bundle identifier active during inference")
    p_ss.add_argument("--auto-analyse", dest="auto_analyse", action="store_true", default=True,
                      help="run analysis after capture (default: enabled)")
    p_ss.add_argument("--no-auto-analyse", dest="auto_analyse", action="store_false",
                      help="skip automatic analysis after capture")
    p_ss.add_argument("--echo-raw", action="store_true",
                      help="echo raw BLE packets during the session")
    p_ss.add_argument("--probe-services", action="store_true",
                      help="print GATT services before subscribing (debug)")
    p_ss.add_argument("--subscribe-all", action="store_true",
                      help="subscribe to multiple notify candidates (debug)")
    p_ss.add_argument("--tolerance-s", type=float, default=0.9,
                      help="analyse merge tolerance in seconds (default: 0.9)")
    p_ss.add_argument("--merge-direction", choices=["nearest","backward","forward"],
                      default="nearest",
                      help="analyse merge_asof direction (default: nearest)")
    p_ss.add_argument("--auto-align-start", action="store_true", default=True,
                      help="shift reference start to radar start before analyse (default: enabled)")
    p_ss.add_argument("--feature-mode", choices=["core","full"], default="full",
                      help="feature mode passed to analyse (default: full)")
    p_ss.add_argument("--phase-unit", choices=["rad","deg"], default="rad")
    p_ss.add_argument("--heart-fft-window-s", type=float, default=5.0)
    p_ss.add_argument("--breath-fft-window-s", type=float, default=10.0)
    p_ss.add_argument("--no-plots", action="store_true",
                      help="skip analyse plots for faster turnaround")
    p_ss.add_argument("--annotate-bland-altman", action="store_true")
    p_ss.add_argument("--golden-json", default=None,
                      help="optional analyse_summary.json baseline to compare against immediately after session")
    p_ss.add_argument("--golden-hr-rmse-tol", type=float, default=0.25)
    p_ss.add_argument("--golden-hr-r-tol", type=float, default=0.03)
    p_ss.add_argument("--dashboard-refresh-s", type=float, default=1.0,
                      help="live dashboard refresh period in seconds (default: 1.0)")
    p_ss.add_argument("--dashboard-port", type=int, default=8765,
                      help="dashboard localhost port (default: 8765; use 0 for auto)")
    p_ss.add_argument("--live-dashboard", dest="live_dashboard", action="store_true", default=True,
                      help="show live clean dashboard during session (default: enabled)")
    p_ss.add_argument("--no-dashboard", dest="live_dashboard", action="store_false",
                      help="disable live dashboard and show only final report")
    p_ss.add_argument("--open-dashboard", dest="open_dashboard", action="store_true", default=True,
                      help="open the live dashboard automatically in your default browser (default: enabled)")
    p_ss.add_argument("--no-open-dashboard", dest="open_dashboard", action="store_false",
                      help="do not auto-open the dashboard browser tab")
    p_ss.set_defaults(func=cmd_session)

    # ── serve ────────────────────────────────────────────────────────────────
    p_srv = sub.add_parser("serve",
        help="localhost dashboard control server")
    p_srv.add_argument("--host", default=None,
                       help="control server bind host (default: 127.0.0.1 local, 0.0.0.0 lan)")
    p_srv.add_argument("--port", dest="control_port", type=int, default=8765,
                       help="control server port (default: 8765; use 0 for auto)")
    p_srv.add_argument("--control-port", dest="control_port", type=int,
                       help="deprecated alias for --port")
    p_srv.add_argument("--bind", choices=["local", "lan"], default="local",
                       help="bind mode: local loopback or LAN with pairing token controls (default: local)")
    p_srv.add_argument("--tls", nargs="*", default=None,
                       help="serve HTTPS; omit paths to auto-generate .rvt_tls/cert.pem and key.pem")
    p_srv.add_argument("--tls-trusted", action="store_true",
                       help="send HSTS only when --tls uses a CA-trusted certificate")
    p_srv.add_argument("--sessions-root", default="sessions",
                       help="sessions root served by the control server (default: sessions)")
    p_srv.add_argument("--cors-origin", default="",
                       help="Access-Control-Allow-Origin value for local dashboard requests (default: \"\", no wildcards by default)")
    p_srv.add_argument("--allow-wildcard-cors-lan", action="store_true",
                       help="permit --cors-origin '*' with --bind lan on an isolated lab network")
    p_srv.add_argument("--mock", action="store_true",
                       help="serve an integrated virtual radar stream over the live APIs and SSE")
    p_srv.add_argument("--no-browser", action="store_true",
                       help="do not open the dashboard browser tab")
    p_srv.set_defaults(func=cmd_serve)

    # -- dashboard --------------------------------------------------------------
    p_db = sub.add_parser("dashboard",
        help="live clean dashboard for an existing session directory")
    p_db.add_argument("--session-dir", required=True,
                      help="session directory containing radar.csv and/or ref.csv")
    p_db.add_argument("--duration-s", type=float, default=None,
                      help="optional auto-stop duration")
    p_db.add_argument("--dashboard-refresh-s", type=float, default=1.0)
    p_db.add_argument("--dashboard-port", type=int, default=8765,
                      help="dashboard localhost port (default: 8765; use 0 for auto)")
    p_db.add_argument("--open-dashboard", dest="open_dashboard", action="store_true", default=True,
                      help="open the live dashboard automatically in your default browser (default: enabled)")
    p_db.add_argument("--no-open-dashboard", dest="open_dashboard", action="store_false",
                      help="do not auto-open the dashboard browser tab")
    p_db.set_defaults(func=cmd_dashboard)

    # -- flash -----------------------------------------------------------------
    p_fl = sub.add_parser("flash",
        help="patch .ino DSP constants, compile, and upload via arduino-cli")
    p_fl.add_argument("--sketch",    required=True,
                      help="path to .ino sketch file")
    p_fl.add_argument("--port",      default=None,
                      help="serial port for upload (omit for build-only)")
    p_fl.add_argument("--fqbn",      default="esp32:esp32:XIAO_ESP32C6",
                      help="arduino-cli FQBN (default: esp32:esp32:XIAO_ESP32C6)")
    p_fl.add_argument("--params",    default=None,
                      help="JSON file with constant overrides, e.g. {\"RLS_LAMBDA\": \"0.94f\"}")
    p_fl.add_argument("--build-dir", default="build",
                      help="temporary build directory (default: build/)")
    p_fl.add_argument("--boot-wait", type=float, default=3.0,
                      help="seconds to wait after upload for device boot (default: 3)")
    p_fl.add_argument("--verbose",   action="store_true",
                      help="print full arduino-cli output")
    p_fl.set_defaults(func=cmd_flash)

    # -- align -----------------------------------------------------------------
    p_al = sub.add_parser("align",
        help="sync radar CSV to reference CSV and write merged output for inspection")
    p_al.add_argument("--radar",  nargs="+", required=True)
    p_al.add_argument("--ref",    nargs="+", required=True)
    p_al.add_argument("--out",    default="align_out",
                      help="output directory (default: align_out/)")
    p_al.add_argument("--ref-offset-s",    nargs="*", type=float, default=None,
                      help="manual timestamp offset(s) in seconds (one per session)")
    p_al.add_argument("--tolerance-s",     type=float, default=0.9)
    p_al.add_argument("--merge-direction", choices=["nearest","backward","forward"],
                      default="nearest")
    p_al.add_argument("--auto-align-start", action="store_true",
                      help="shift ref timestamps so they start at radar start time")
    p_al.add_argument("--phase-unit",  choices=["rad","deg"], default="rad")
    p_al.add_argument("--heart-fft-window-s",  type=float, default=5.0)
    p_al.add_argument("--breath-fft-window-s", type=float, default=10.0)
    p_al.set_defaults(func=cmd_align)

    # -- predict ---------------------------------------------------------------
    p_pr = sub.add_parser("predict",
        help="apply a saved trained model to new radar-only CSV(s)")
    p_pr.add_argument("--radar",     nargs="+", required=True)
    p_pr.add_argument("--model-dir", required=True,
                      help="directory containing model_hr.pkl, model_rr.pkl, preprocessor.pkl")
    p_pr.add_argument("--out",       default="predict_out")
    p_pr.add_argument("--no-plots",  action="store_true")
    p_pr.set_defaults(func=cmd_predict)

    # -- shared args for analyse + train ---------------------------------------
    def add_common(p):
        p.add_argument("--radar",  nargs="+", required=True)
        p.add_argument("--ref",    nargs="+", required=True)
        p.add_argument("--out",    default="model_out")
        p.add_argument("--tolerance-s",     type=float, default=0.9)
        p.add_argument("--merge-direction", choices=["nearest","backward","forward"],
                       default="nearest",
                       help="merge_asof direction (default: nearest; use backward only when you explicitly want stricter delayed-reference matching)")
        p.add_argument("--ref-offset-s",    nargs="*", type=float, default=None)
        p.add_argument("--auto-align-start", action="store_true")
        p.add_argument("--feature-mode",    choices=["core","full"], default="full",
                       help="core=~86 features (safe for <15min); full=~120+ (needs 15+ min)")
        p.add_argument("--phase-unit",      choices=["rad","deg"], default="rad")
        p.add_argument("--heart-fft-window-s",  type=float, default=5.0)
        p.add_argument("--breath-fft-window-s", type=float, default=10.0)
        p.add_argument("--no-plots",             action="store_true")
        p.add_argument("--annotate-bland-altman", action="store_true")

    # -- analyse ---------------------------------------------------------------
    p_an = sub.add_parser("analyse",
        help="baseline RMSE/r/pqi analysis with ML gate check")
    add_common(p_an)
    p_an.add_argument("--golden-json", default=None,
                      help="optional analyse_summary.json baseline to compare current metrics against")
    p_an.add_argument("--golden-hr-rmse-tol", type=float, default=0.25,
                      help="allowed HR RMSE regression versus golden before FAIL (default: 0.25)")
    p_an.add_argument("--golden-hr-r-tol", type=float, default=0.03,
                      help="allowed HR correlation drop versus golden before FAIL (default: 0.03)")
    p_an.set_defaults(func=cmd_analyse)

    # -- train -----------------------------------------------------------------
    p_tr = sub.add_parser("train",
        help="train HR/RR correction models with gradient boosting or an optional 1-D CNN")
    add_common(p_tr)
    p_tr.add_argument(
        "--model-family",
        choices=MODEL_FAMILIES,
        default=MODEL_FAMILY_GRADIENT_BOOSTING,
        help="estimator family (default: gradient_boosting; cnn_1d requires TensorFlow)",
    )
    p_tr.add_argument("--val-ratio",         type=float, default=0.2)
    p_tr.add_argument("--three-way-split",   action="store_true")
    p_tr.add_argument(
        "--confirmatory-evaluation",
        action="store_true",
        help=(
            "fail closed unless a participant-disjoint three-way split covers every "
            "0.6/0.8/1.0 m x none/cardboard confirmatory condition"
        ),
    )
    p_tr.add_argument("--early-stop-ratio",  type=float, default=0.15)
    p_tr.add_argument("--purge-gap-s",       type=float, default=10.0)
    p_tr.add_argument("--random-state",      type=int,   default=42)
    p_tr.add_argument("--sample-weight-mode",choices=["none","pqi"], default="none")
    p_tr.add_argument("--disable-early-stopping", action="store_true")
    p_tr.add_argument("--early-stop-strategy", choices=["retrain","truncate"], default="retrain")
    p_tr.add_argument("--light-mode",        action="store_true",
                      help="fast/small model for quick iteration")
    p_tr.add_argument("--n-estimators",      type=int,   default=250)
    p_tr.add_argument("--early-stop-max-estimators", type=int, default=500)
    p_tr.add_argument("--max-depth",         type=int,   default=3)
    p_tr.add_argument("--learning-rate",     type=float, default=0.04)
    p_tr.add_argument("--subsample",         type=float, default=0.8)
    p_tr.add_argument("--cnn-window-size",   type=int, default=32,
                      help="causal samples per 1-D CNN input window")
    p_tr.add_argument("--cnn-filters",       type=int, default=32)
    p_tr.add_argument("--cnn-kernel-size",   type=int, default=5)
    p_tr.add_argument("--cnn-dropout",       type=float, default=0.20)
    p_tr.add_argument("--cnn-learning-rate", type=float, default=1e-3)
    p_tr.add_argument("--cnn-epochs",        type=int, default=100)
    p_tr.add_argument("--cnn-batch-size",    type=int, default=64)
    p_tr.add_argument("--cnn-patience",      type=int, default=12)
    p_tr.add_argument("--cnn-min-valid-windows", type=int, default=500,
                      help="minimum valid endpoint windows per target before CNN training")
    p_tr.add_argument("--allow-small-cnn-dataset", action="store_true",
                      help="research-only override for CNN feasibility/overfit experiments")
    p_tr.add_argument("--max-nan-frac",      type=float, default=0.70)
    p_tr.add_argument("--min-variance",      type=float, default=1e-8)
    p_tr.add_argument("--min-samples-per-feature", type=int, default=10)
    p_tr.add_argument("--strict-feature-ratio", action="store_true")
    p_tr.add_argument("--require-baseline-gate", action="store_true",
                      help="abort training if r<gate_hr_min_r or RMSE>gate_hr_max_rmse")
    p_tr.add_argument("--gate-hr-min-r",    type=float, default=0.4)
    p_tr.add_argument("--gate-hr-max-rmse", type=float, default=8.0)
    p_tr.add_argument("--slew-limit-hr-per-s", type=float, default=None)
    p_tr.add_argument("--slew-limit-rr-per-s", type=float, default=None)
    p_tr.add_argument("--embedded-export",  choices=["none","linear","tflite","both"],
                      default="none")
    p_tr.add_argument("--targets", nargs="+", choices=["hr", "rr"], default=["hr", "rr"],
                      help="targets to train. Use --targets hr for BLE pulse-ox reference sessions")
    p_tr.add_argument("--allow-policy-features", action="store_true",
                      help="explicit ablation flag: allow policy/gate telemetry into the ML feature matrix")
    p_tr.add_argument("--loo-eval", action="store_true",
                      help=(
                          "run leave-one-participant-out evaluation when study metadata "
                          "is available, otherwise leave-one-session-out"
                      ))
    p_tr.set_defaults(func=cmd_train)

    # -- sweep -----------------------------------------------------------------
    p_sw = sub.add_parser("sweep",
        help="automated DSP parameter sweep: flash -> log -> analyse each variant")
    p_sw.add_argument("--sweep-json",  required=True,
                      help="JSON array of experiment configs (see quickstart for format)")
    p_sw.add_argument("--sketch",      required=True,
                      help="path to .ino sketch file")
    p_sw.add_argument("--port",        required=True,
                      help="serial port, e.g. COM3 or /dev/ttyACM0")
    p_sw.add_argument("--ref",         default=None,
                      help="reference CSV to use for all variants (optional but recommended)")
    p_sw.add_argument("--out-dir",     default="sweep_out",
                      help="output directory for all experiment results (default: sweep_out/)")
    p_sw.add_argument("--fqbn",        default="esp32:esp32:XIAO_ESP32C6")
    p_sw.add_argument("--default-duration-s", type=int, default=920,
                      help="fallback session duration if not set per-experiment (default: 920s)")
    p_sw.add_argument("--boot-wait",   type=float, default=3.0)
    p_sw.add_argument("--distance-hint", default="0.7-0.9",
                      help="distance reminder shown to user (default: 0.7-0.9)")
    p_sw.add_argument("--tolerance-s",  type=float, default=0.9)
    p_sw.add_argument("--auto-align-start", action="store_true")
    p_sw.set_defaults(func=cmd_sweep)

    # -- compare ---------------------------------------------------------------
    p_cmp = sub.add_parser("compare",
        help="generate HTML comparison report across all sessions in a directory")
    p_cmp.add_argument("--sessions-dir", required=True,
                       help="directory containing session sub-folders")
    p_cmp.add_argument("--out", default="compare_report.html",
                       help="output HTML report path (default: compare_report.html)")
    p_cmp.add_argument("--baseline-name", default=None,
                       help="baseline session/experiment name for sweep deltas (default: name containing 'baseline', else first session)")
    p_cmp.add_argument("--bootstrap-n", type=int, default=1000,
                       help="bootstrap replicates for delta RMSE/r confidence intervals (default: 1000)")
    p_cmp.add_argument("--bootstrap-seed", type=int, default=42,
                       help="random seed for bootstrap confidence intervals")
    p_cmp.set_defaults(func=cmd_compare)

    return parser


def _run_self_tests() -> int:
    import urllib.request

    test_report = tempfile.NamedTemporaryFile(delete=False, suffix=".txt")
    test_report.close()
    try:
        write_text_report(test_report.name, {"x": {"y": float("nan"), "z": float("inf")}})
        txt = Path(test_report.name).read_text(encoding="utf-8")
        assert "NaN" not in txt and "Infinity" not in txt
    finally:
        try:
            os.remove(test_report.name)
        except Exception:
            pass

    tmp = tempfile.mkdtemp(prefix="rvt-selftest-")
    server = None
    try:
        args = argparse.Namespace(host="127.0.0.1", control_port=0, sessions_root=tmp, no_browser=True, cors_origin="", mock=True, bind="local", tls=None, tls_trusted=False)
        server = _start_control_server(args)
        server.start()
        base = f"http://127.0.0.1:{server.httpd.server_port}"
        with urllib.request.urlopen(base + "/api/health", timeout=5) as r:
            assert r.status == 200
            assert r.headers.get("Access-Control-Allow-Origin") is None
            data = json.loads(r.read().decode("utf-8"))
            assert data.get("version") == VERSION
        with urllib.request.urlopen(base + "/api/session/current/live_dashboard.json", timeout=5) as r:
            assert r.status == 200
            data = json.loads(r.read().decode("utf-8"))
            assert data.get("session_id") == "mock"
        print("PASS: self-tests")
        return 0
    finally:
        if server is not None:
            try:
                server.stop()
            except Exception:
                pass
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    if "--test" in sys.argv:
        raise SystemExit(_run_self_tests())
    parser = build_parser()
    args   = parser.parse_args()

    if hasattr(args, "radar") and hasattr(args, "ref") and args.ref is not None:
        if len(args.radar) != len(args.ref):
            parser.error("--radar and --ref must have the same number of files")

    args.func(args)


if __name__ == "__main__":
    main()

__all__ = [
    "build_parser",
    "cmd_doctor",
    "cmd_quickstart",
    "main",
]
