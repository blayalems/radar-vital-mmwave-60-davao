"""Serial transport helpers, firmware flashing, and CSV line parsing."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, cast

from rvt_trainer._compat import monolith_module, monolith_override
from rvt_trainer.constants import (
    CONTROL_API_SCHEMA_VERSION,
    DEFAULT_RADAR_PORT,
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    EXPECTED_RADAR_LOG_TAIL,
    LEGACY_RADAR_LOG_COLUMN_COUNT,
    LEGACY_V14_COLUMN_COUNT,
    LEGACY_V15_1_COLUMN_COUNT,
    LEGACY_V15_COLUMN_COUNT,
    RADAR_LOG_COLUMNS,
    SCHEMA_VERSION_MAP,
    _REPO_ROOT,
    _bold,
    _cyan,
    _dim,
    _green,
    _is_relative_to_path,
    _red,
    _yellow,
    warn,
)
from rvt_trainer.protocol_contract import (
    load_protocol_contract as _load_protocol_contract,
    protocol_contract_candidates as _protocol_contract_candidates,
)

_RADAR_PORT_HINTS = (
    "xiao",
    "esp32",
    "usb serial",
    "usb-serial",
    "wch",
    "cp210",
    "ch340",
    "arduino",
)

_FIRMWARE_CONTRACT_CACHE: Optional[Tuple[str, Tuple[str, ...]]] = None


def auto_detect_radar_port(default: str = DEFAULT_RADAR_PORT) -> str:
    try:
        import serial.tools.list_ports

        ports = list(serial.tools.list_ports.comports())
    except Exception:
        return default
    if not ports:
        return default
    scored = []
    for p in ports:
        text = " ".join(
            str(x or "")
            for x in [
                getattr(p, "device", ""),
                getattr(p, "description", ""),
                getattr(p, "manufacturer", ""),
                getattr(p, "hwid", ""),
            ]
        ).lower()
        score = sum(1 for h in _RADAR_PORT_HINTS if h in text)
        scored.append((score, str(getattr(p, "device", "") or "")))
    scored = [x for x in scored if x[1]]
    if not scored:
        return default
    scored.sort(key=lambda x: (-x[0], x[1]))
    if scored[0][0] > 0:
        return scored[0][1]
    return scored[0][1] if len(scored) == 1 else default


def serial_ports_payload(selected: str = DEFAULT_RADAR_PORT) -> Dict[str, object]:
    ports: List[Dict[str, str]] = []
    try:
        import serial.tools.list_ports

        for item in serial.tools.list_ports.comports():
            device = str(getattr(item, "device", "") or "")
            if not device:
                continue
            label = str(getattr(item, "description", "") or device)
            ports.append({"device": device, "label": label})
    except Exception:
        ports = []
    return {
        "ok": True,
        "schema_version": CONTROL_API_SCHEMA_VERSION,
        "ports": ports,
        "selected": selected,
    }


_serial_ports_payload = serial_ports_payload


def probe_serial_port(port: str, timeout_s: float = 4.0) -> Tuple[List[str], Optional[str]]:
    lines = []
    try:
        import serial

        ser = serial.Serial(port, 115200, timeout=timeout_s)
        with ser as s:
            for _ in range(20):
                raw = s.readline()
                if not raw:
                    break
                lines.append(raw.decode(errors="ignore").strip())
    except Exception as e:
        return lines, str(e)
    return lines, None


def _detect_csv_schema_version(columns) -> str:
    """Detect firmware schema version from column count.
    Returns 'v15.1' for 219, 'v15.0' for 207, 'v14.1' for 199,
    'unknown-N' otherwise. Used by row padding logic to decide how to fill
    missing right-edge columns.
    """
    try:
        n = len(columns)
    except TypeError:
        return "unknown-?"
    return SCHEMA_VERSION_MAP.get(n, f"unknown-{n}")


def _is_supported_radar_contract_length(length: object) -> bool:
    try:
        n = int(cast(Any, length))
    except Exception:
        return False
    return n in {EXPECTED_RADAR_LOG_COLUMN_COUNT, LEGACY_V15_1_COLUMN_COUNT, LEGACY_V15_COLUMN_COUNT, LEGACY_V14_COLUMN_COUNT}


def _firmware_contract_candidates() -> List[Path]:
    roots = [
        _REPO_ROOT,
        Path(os.getcwd()),
    ]
    out: List[Path] = []
    seen = set()
    for root in roots:
        path = (root / "radar_vital.ino").resolve()
        key = str(path).lower()
        if key in seen:
            continue
        seen.add(key)
        out.append(path)
    return out


def _extract_firmware_data_header_tokens(ino_path: Path) -> List[str]:
    text = ino_path.read_text(encoding="utf-8", errors="ignore")
    text = re.sub(r"//.*", "", text)
    for match in re.finditer(r'Serial\.println\(\s*((?:"[^"]*"\s*)+)\);', text, flags=re.S):
        literals = re.findall(r'"([^"]*)"', match.group(1))
        header = "".join(literals)
        if header.startswith("DATA,"):
            return header.split(",")[1:]
    raise RuntimeError(f"Could not locate DATA header in firmware file: {ino_path}")


_CANONICAL_PROTOCOL_CONTRACT_CANDIDATES = _protocol_contract_candidates
_CANONICAL_LOAD_PROTOCOL_CONTRACT = _load_protocol_contract


def _assert_radar_log_contract(allow_missing_source: bool = False) -> Optional[Path]:
    global _FIRMWARE_CONTRACT_CACHE
    expected = list(RADAR_LOG_COLUMNS)
    if len(expected) != EXPECTED_RADAR_LOG_COLUMN_COUNT:
        raise RuntimeError(
            f"RADAR_LOG_COLUMNS contract drift: expected {EXPECTED_RADAR_LOG_COLUMN_COUNT} columns, got {len(expected)}"
        )
    if tuple(expected[-len(EXPECTED_RADAR_LOG_TAIL):]) != EXPECTED_RADAR_LOG_TAIL:
        raise RuntimeError(
            f"RADAR_LOG_COLUMNS tail drift: expected {EXPECTED_RADAR_LOG_TAIL}, got {tuple(expected[-len(EXPECTED_RADAR_LOG_TAIL):])}"
        )
    monolith_mod = monolith_module()
    cache = (
        getattr(monolith_mod, "_FIRMWARE_CONTRACT_CACHE", _FIRMWARE_CONTRACT_CACHE)
        if monolith_mod is not None
        else _FIRMWARE_CONTRACT_CACHE
    )
    if cache is not None and list(cache[1]) == expected:
        return Path(cache[0])

    searched = []
    mismatches = []
    candidates_fn = monolith_override(
        "_protocol_contract_candidates",
        _protocol_contract_candidates,
        canonical=_CANONICAL_PROTOCOL_CONTRACT_CANDIDATES,
    )
    load_fn = monolith_override(
        "_load_protocol_contract",
        _load_protocol_contract,
        canonical=_CANONICAL_LOAD_PROTOCOL_CONTRACT,
    )
    for path in candidates_fn():
        searched.append(str(path))
        if not path.is_file():
            continue
        contract = load_fn(path)
        actual = [str(col) for col in contract.get("columns", [])]
        if actual == expected:
            _FIRMWARE_CONTRACT_CACHE = (str(path), tuple(actual))
            if monolith_mod is not None:
                setattr(monolith_mod, "_FIRMWARE_CONTRACT_CACHE", _FIRMWARE_CONTRACT_CACHE)
            return path
        mismatch_idx = next(
            (i for i, (trainer_col, contract_col) in enumerate(zip(expected, actual)) if trainer_col != contract_col),
            min(len(expected), len(actual)),
        )
        trainer_col = expected[mismatch_idx] if mismatch_idx < len(expected) else "<end>"
        contract_col = actual[mismatch_idx] if mismatch_idx < len(actual) else "<end>"
        mismatches.append((path, mismatch_idx, trainer_col, contract_col, len(actual)))

    if mismatches:
        details = []
        for path, mismatch_idx, trainer_col, contract_col, actual_len in mismatches[:4]:
            details.append(
                f"file={path}, index={mismatch_idx}, trainer={trainer_col!r}, contract={contract_col!r}, "
                f"trainer_count={len(expected)}, contract_count={actual_len}"
            )
        raise RuntimeError(
            "Protocol contract mismatch: no candidate matched the trainer contract. "
            + " | ".join(details)
            + f". Searched: {', '.join(searched)}"
        )

    if allow_missing_source:
        return None
    raise RuntimeError(
        "Could not locate config/protocol_contract.json for contract verification. "
        f"Tried: {', '.join(searched)}"
    )


def _parse_radar_data_line(line: str, cols: Sequence[str]) -> Tuple[str, Optional[List[str]], str]:
    if not line.startswith("DATA,"):
        return "skip", None, ""
    payload = line.split(",")[1:]
    if payload == list(cols) or (payload and payload[0] == "timestamp_ms"):
        return "header", None, ""
    # v12 trainer accepts v15.2 (222), v15.1 (219), v15.0 (207), and v14.1
    # (199) schemas. Older legacy counts retained for archived sessions.
    accepted_legacy_counts = (LEGACY_RADAR_LOG_COLUMN_COUNT, 136, 180, 195, LEGACY_V14_COLUMN_COUNT, LEGACY_V15_COLUMN_COUNT, LEGACY_V15_1_COLUMN_COUNT)
    if len(payload) not in ((len(cols),) + accepted_legacy_counts):
        return "reject", None, f"{len(payload)} fields, expected {accepted_legacy_counts} or {len(cols)}"
    if len(payload) == LEGACY_V15_1_COLUMN_COUNT:
        # v15.1 ended with fw_uptime_s. v15.2 firmware inserted three audit
        # fields before that right-edge value, so preserve old uptime position.
        payload = payload[:-1] + ["", "", ""] + payload[-1:]
    elif len(payload) in accepted_legacy_counts:
        payload = payload + [""] * (EXPECTED_RADAR_LOG_COLUMN_COUNT - len(payload))
    try:
        float(payload[0])
    except Exception:
        return "reject", None, "non-numeric timestamp_ms field"
    return "data", payload, ""


def _log_serial_for_duration(port: str, duration_s: int, out_csv: str) -> int:
    """Logs serial DATA frames for duration_s seconds. Returns number of frames."""
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not found. Run: pip install pyserial")

    cols = list(RADAR_LOG_COLUMNS)

    os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)
    accepted    = 0
    header_lines = 0
    t_start     = time.time()
    t_end       = t_start + duration_s
    first_frame = False

    with serial.Serial(port, 115200, timeout=1) as ser, \
         open(out_csv, "w", encoding="utf-8") as f:
        f.write(",".join(cols) + "\n")
        while time.time() < t_end:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("SESSION_SUMMARY") or line.startswith("SESSION,"):
                print(f"  {_dim(line)}")
                continue
            kind, row, detail = _parse_radar_data_line(line, cols)
            if kind == "skip":
                continue
            if kind == "header":
                header_lines += 1
                continue
            if kind == "data" and row is not None:
                f.write(",".join(row) + "\n")
                f.flush()
                accepted += 1
                if not first_frame:
                    print(_green("  First frame received - logging active."))
                    first_frame = True
                elapsed = time.time() - t_start
                remaining = max(0, duration_s - elapsed)
                if accepted % 50 == 0:
                    print(f"  {accepted} frames | {remaining:.0f}s remaining...", end="\r")
    print(f"  {accepted} frames logged in {duration_s}s")
    return accepted


def cmd_log(args):
    start_wall = time.time()
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not found. Run: pip install pyserial")

    cols = list(RADAR_LOG_COLUMNS)
    contract_path = _assert_radar_log_contract(allow_missing_source=True)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    if contract_path is not None:
        print(f"[LOG] Firmware contract OK: {contract_path.name} ({len(cols)} columns)")
    else:
        print(f"[LOG] Firmware contract source unavailable; using built-in trainer schema ({len(cols)} columns)")
    print(f"[LOG] Opening {args.port} @ 115200 baud")
    print(f"[LOG] Writing to {args.out}")
    print("[LOG] Waiting for first DATA frame... (Ctrl-C to stop)\n")
    accepted = 0
    dropped = 0
    header_lines = 0
    first_sample = False
    legacy_warned = False
    with serial.Serial(args.port, 115200, timeout=1) as ser, \
         open(args.out, "w", encoding="utf-8") as f:
        f.write(",".join(cols) + "\n")
        try:
            while True:
                if getattr(args, "duration_s", None) and (time.time() - start_wall) >= float(args.duration_s):
                    print(f"\n[LOG] Auto-stop reached at {float(args.duration_s):.1f}s")
                    break
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("SESSION_SUMMARY") or line.startswith("SESSION,"):
                    print(f"  {_dim(line)}")
                    continue
                kind, row, detail = _parse_radar_data_line(line, cols)
                if kind == "skip":
                    continue
                if kind == "header":
                    header_lines += 1
                    continue

                if kind == "data" and row is not None:
                    pad_len = EXPECTED_RADAR_LOG_COLUMN_COUNT - LEGACY_RADAR_LOG_COLUMN_COUNT
                    if (not legacy_warned) and len(row) == len(cols) and row[-pad_len:] == [""] * pad_len:
                        print(_yellow(f"[LOG] Legacy 131-column firmware detected; padding {pad_len} anchor telemetry fields with blanks."))
                        legacy_warned = True
                    f.write(",".join(row) + "\n")
                    f.flush()
                    accepted += 1
                    if not first_sample:
                        print(_green("[LOG] First sample received - logging active."))
                        first_sample = True
                    if accepted % 100 == 0:
                        print(f"[LOG] {accepted} frames logged...", end="\r")
                else:
                    dropped += 1
                    if dropped <= 5:
                        print(f"[LOG] Dropped malformed DATA line ({detail})")
        except KeyboardInterrupt:
            print("\n[LOG] Stopped.")
            print(f"[LOG] Accepted: {_green(str(accepted))} frames")
            if header_lines:
                print(f"[LOG] Skipped firmware header lines: {header_lines}")
            if dropped:
                print(f"[LOG] Dropped:  {_yellow(str(dropped))} malformed lines")
            print(f"[LOG] Saved to: {args.out}")


def cmd_reflog(args):
    """
    Live keyboard-driven reference logger.
    Run this in a second terminal alongside `log`.

    Usage:
      Type  HR=72  then Enter  to log a heart rate reading.
      Type  RR=14  then Enter  to log a respiration rate reading.
      Press Ctrl-C to stop and save.
    """
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    print(_bold("\n=== Reference Logger ==="))
    print(f"Output: {args.out}")
    print()
    print("Type readings and press Enter:")
    print(f"  {_cyan('HR=72')}  -> log heart rate")
    print(f"  {_cyan('RR=14')}  -> log respiration rate")
    print(f"  {_cyan('Ctrl-C')} -> stop and save\n")
    print(_dim("TIP: Log HR every 30 seconds, RR every 60 seconds.\n"))

    start_ms = int(time.time() * 1000)
    rows: List[Dict] = []

    try:
        while True:
            try:
                line = input("> ").strip().upper().replace(" ", "")
            except EOFError:
                break
            t = int(time.time() * 1000) - start_ms
            elapsed = t / 1000.0

            if line.startswith("HR="):
                try:
                    val = float(line[3:])
                    if not (30 <= val <= 220):
                        print(_yellow(f"  Ignored - HR {val:.0f} out of range [30-220]"))
                        continue
                    rows.append({"timestamp_ms": t, "ref_hr": val, "ref_rr": ""})
                    print(_green(f"  t={elapsed:.1f}s  HR={val:.0f} BPM logged"))
                except ValueError:
                    print(_red(f"  Could not parse: {line}  (example: HR=72)"))

            elif line.startswith("RR="):
                try:
                    val = float(line[3:])
                    if not (4 <= val <= 60):
                        print(_yellow(f"  Ignored - RR {val:.0f} out of range [4-60]"))
                        continue
                    rows.append({"timestamp_ms": t, "ref_hr": "", "ref_rr": val})
                    print(_green(f"  t={elapsed:.1f}s  RR={val:.0f} br/min logged"))
                except ValueError:
                    print(_red(f"  Could not parse: {line}  (example: RR=14)"))

            elif line in ("Q", "QUIT", "EXIT"):
                break
            elif line == "":
                pass
            else:
                print(_dim(f"  Unknown: '{line}'  (use HR=<number> or RR=<number>)"))

    except KeyboardInterrupt:
        pass

    if not rows:
        print(_yellow("\n[REFLOG] No readings captured."))
        return

    import csv
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["timestamp_ms", "ref_hr", "ref_rr"])
        writer.writeheader()
        writer.writerows(rows)

    hr_rows = [r for r in rows if r["ref_hr"] != ""]
    rr_rows = [r for r in rows if r["ref_rr"] != ""]
    duration_s = (rows[-1]["timestamp_ms"] - rows[0]["timestamp_ms"]) / 1000 if len(rows) > 1 else 0

    print(f"\n{_green('[REFLOG] Saved')} {len(rows)} readings to {args.out}")
    print(f"  HR readings: {len(hr_rows)}")
    print(f"  RR readings: {len(rr_rows)}")
    print(f"  Duration:    {duration_s:.0f}s")
    if hr_rows:
        hr_vals = [float(r["ref_hr"]) for r in hr_rows]
        print(f"  HR range:    {min(hr_vals):.0f} - {max(hr_vals):.0f} BPM "
              f"(mean {sum(hr_vals)/len(hr_vals):.1f})")


def _check_arduino_cli() -> str:
    """Returns path to arduino-cli or exits with friendly instructions."""
    path = shutil.which("arduino-cli")
    if path:
        return path
    print(_red("\n[FLASH] arduino-cli not found.\n"))
    print("Install it from: https://arduino.github.io/arduino-cli/latest/installation/\n")
    print("Quick install (Linux/Mac):")
    print("  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/"
          "master/install.sh | sh")
    print("\nAfter installing, add the ESP32 core:")
    print("  arduino-cli core update-index")
    print("  arduino-cli core install esp32:esp32")
    print("\nThen re-run this command.\n")
    sys.exit(1)


def _patch_ino_constants(sketch_path: str, params: Dict[str, str], out_path: str):
    """
    Reads sketch_path, replaces constant definitions matching params keys,
    writes result to out_path.

    Matches patterns like:
      const float  RLS_LAMBDA = 0.97f;
      const int    ABSENT_VOTES = 5;
      #define      ABSENT_VOTES 5
    """
    src = open(sketch_path, "r", encoding="utf-8", errors="ignore").read()
    patched = 0
    for key, val in params.items():
        # Match:  const <type> KEY = <old_value>;
        pattern_const = rf"(const\s+\w+\s+{re.escape(key)}\s*=\s*)([^;]+)(;)"
        new_src, n = re.subn(pattern_const, lambda m: f"{m.group(1)}{val}{m.group(3)}", src)
        if n:
            src = new_src
            patched += n
            continue
        # Match:  #define KEY <old_value>
        pattern_define = rf"(#define\s+{re.escape(key)}\s+)(\S+)"
        new_src, n = re.subn(pattern_define, lambda m: f"{m.group(1)}{val}", src)
        if n:
            src = new_src
            patched += n
            continue
        warn(f"Constant '{key}' not found in sketch - skipping")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    open(out_path, "w", encoding="utf-8").write(src)
    print(f"[FLASH] Patched {patched} constant(s) in sketch")
    return patched


def _prepare_arduino_sketch_dir(sketch_path: str, build_root: str, params: Optional[Dict[str, str]] = None) -> str:
    """Stage a valid Arduino sketch folder named after the .ino stem."""
    src = Path(sketch_path).resolve()
    if src.suffix.lower() != ".ino":
        raise ValueError(f"[FLASH] Expected an .ino sketch, got: {src}")
    root = Path(build_root).resolve()
    work_dir = root / "staged_sketch" / src.stem
    if work_dir.exists():
        if not _is_relative_to_path(work_dir, root):
            raise RuntimeError(f"[FLASH] Refusing to clean sketch folder outside build root: {work_dir}")
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    staged_ino = work_dir / f"{src.stem}.ino"
    if params:
        _patch_ino_constants(str(src), params, str(staged_ino))
    else:
        shutil.copy2(src, staged_ino)
    for sibling in src.parent.iterdir():
        if sibling == src or not sibling.is_file():
            continue
        if sibling.suffix.lower() in {".h", ".hpp", ".hh", ".c", ".cpp", ".cc", ".cxx", ".s"}:
            shutil.copy2(sibling, work_dir / sibling.name)
    return str(work_dir)


def cmd_flash(args):
    _check_arduino_cli()

    # Load params
    if args.params:
        if not os.path.exists(args.params):
            sys.exit(f"[FLASH] params file not found: {args.params}")
        params = json.load(open(args.params, encoding="utf-8"))
        print(f"[FLASH] Loaded {len(params)} param(s) from {args.params}")
    else:
        params = {}
        print("[FLASH] No params file - compiling sketch as-is")

    sketch_path = args.sketch
    if not os.path.exists(sketch_path):
        sys.exit(f"[FLASH] Sketch not found: {sketch_path}")

    build_path = os.path.join(args.build_dir, "build")
    os.makedirs(build_path, exist_ok=True)
    compile_sketch = _prepare_arduino_sketch_dir(sketch_path, args.build_dir, params)

    fqbn = args.fqbn

    # Compile
    print(f"\n[FLASH] Compiling for {fqbn} ...")
    compile_cmd = [
        "arduino-cli", "compile",
        "--fqbn", fqbn,
        "--build-path", build_path,
        compile_sketch,
    ]
    if args.verbose:
        compile_cmd.append("--verbose")
    result = subprocess.run(compile_cmd, capture_output=not args.verbose, text=True)
    if result.returncode != 0:
        print(_red("[FLASH] Compilation FAILED"))
        if not args.verbose:
            print(result.stderr[-2000:] if result.stderr else "(no output)")
        sys.exit(1)
    print(_green("[FLASH] Compilation successful"))

    # Upload
    if args.port:
        print(f"[FLASH] Uploading to {args.port} ...")
        upload_cmd = [
            "arduino-cli", "upload",
            "--fqbn", fqbn,
            "--port", args.port,
            "--input-dir", build_path,
        ]
        if args.verbose:
            upload_cmd.append("--verbose")
        result = subprocess.run(upload_cmd, capture_output=not args.verbose, text=True)
        if result.returncode != 0:
            print(_red("[FLASH] Upload FAILED"))
            if not args.verbose:
                print(result.stderr[-2000:] if result.stderr else "(no output)")
            sys.exit(1)
        print(_green(f"[FLASH] Upload successful -> {args.port}"))
        print(f"[FLASH] Waiting {args.boot_wait}s for device to boot...")
        time.sleep(args.boot_wait)
    else:
        print(_yellow("[FLASH] No --port specified - skipping upload (build only)"))

    print(_green("\n[FLASH] Done."))
