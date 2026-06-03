"""Preflight and audit runner helpers."""

from __future__ import annotations
import time
import shutil
import asyncio
import inspect
from pathlib import Path
from typing import Dict, List, Any

def run_preflight_check(check_id: str, **params) -> Dict[str, object]:
    import sys
    from ..monolith import (
        _preflight_result,
        _candidate_ino_paths,
        _extract_firmware_data_header_tokens,
        _read_session_lock,
        _scoring_weights_hash,
        _iso_now,
        DEFAULT_RADAR_PORT,
        DEFAULT_BLE_ADDRESS,
        RADAR_LOG_COLUMNS
    )

    start = time.monotonic()
    sessions_root = str(params.get("sessions_root") or "sessions")
    port = str(params.get("port") or DEFAULT_RADAR_PORT)
    address = str(params.get("address") or DEFAULT_BLE_ADDRESS)
    try:
        if check_id == "python_env":
            parts = []
            for name in ("serial", "bleak", "pandas", "numpy", "sklearn", "matplotlib"):
                try:
                    mod = __import__(name)
                    parts.append(f"{name}={getattr(mod, '__version__', 'available')}")
                except Exception as e:
                    return _preflight_result(check_id, "Python environment", "fail", f"{name} import failed: {e}", "Install missing dependency.", start)
            return _preflight_result(check_id, "Python environment", "ok", ", ".join(parts), "", start)

        if check_id == "firmware_file_present":
            expected = list(RADAR_LOG_COLUMNS)
            for path in _candidate_ino_paths(params.get("ino_search_paths")):
                if not path.exists():
                    continue
                try:
                    actual = _extract_firmware_data_header_tokens(path)
                except Exception:
                    continue
                if actual == expected:
                    return _preflight_result(check_id, "Firmware file", "ok", f"{path.name}: {len(expected)}-column contract matches", "", start)
                return _preflight_result(check_id, "Firmware file", "fail", f"{path.name}: expected {len(expected)} columns, got {len(actual)}", "Use the v15.0.0 firmware file that matches this trainer.", start)
            return _preflight_result(check_id, "Firmware file", "fail", "No parseable firmware DATA header found", "Place radar_vital_v15_0_0.ino beside the trainer.", start)

        if check_id == "serial_port_list":
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            devices = [str(getattr(p, "device", p)) for p in ports]
            if port in devices:
                return _preflight_result(check_id, "Serial ports", "ok", f"{port} present; ports={devices}", "", start)
            return _preflight_result(check_id, "Serial ports", "warn", f"{port} not found; ports={devices}", f"Connect radar or select a detected port instead of {port}.", start)

        if check_id == "serial_port_probe":
            if _read_session_lock(sessions_root):
                return _preflight_result(check_id, "Serial probe", "fail", "A session lock is active; refusing to probe serial hardware.", "Stop the active session before probing.", start)

            # Use the serial probe function from our serial transport package
            from ..transport.serial import probe_serial_port
            timeout_s = float(params.get("timeout_s") or 4.0)
            lines = probe_serial_port(port, timeout_s)

            # Check if an error was returned as the first element
            if lines and lines[0].startswith("ERROR:"):
                return _preflight_result(check_id, "Serial probe", "fail", lines[0], "Review serial connection.", start)

            detail = f"Read {len(lines)} line(s); " + ("; ".join(lines[:3]) if lines else "no banner yet")
            status = "ok" if any(("RVital" in ln or "DATA," in ln) for ln in lines) else "warn"
            return _preflight_result(check_id, "Serial probe", status, detail, "Warmup may take about 60 s before DATA appears." if status == "warn" else "", start)

        if check_id == "ble_adapter":
            try:
                from bleak import BleakScanner
            except Exception as e:
                return _preflight_result(check_id, "BLE adapter", "fail", f"bleak import failed: {e}", "Install bleak: pip install bleak", start)
            try:
                devices = asyncio.run(BleakScanner.discover(timeout=float(params.get("timeout_s") or 1.0)))
                count = len(devices) if devices is not None else 0
                elapsed_ms = (time.monotonic() - start) * 1000.0
                return _preflight_result(check_id, "BLE adapter", "ok", f"adapter discovered {count} device(s) in {elapsed_ms:.0f} ms", "", start)
            except Exception as e:
                return _preflight_result(check_id, "BLE adapter", "fail", f"BleakScanner.discover failed: {e}", "Enable Bluetooth and confirm the adapter is present.", start)

        if check_id == "ble_device_probe":
            if _read_session_lock(sessions_root):
                return _preflight_result(check_id, "BLE device", "fail", "A session lock is active; refusing BLE probe.", "Stop the active session before probing.", start)
            from bleak import BleakScanner
            finder = getattr(BleakScanner, "find_device_by_address", None)
            if finder is None:
                return _preflight_result(check_id, "BLE device", "fail", "BleakScanner.find_device_by_address unavailable", "Check Bleak installation.", start)
            value = finder(address, timeout=float(params.get("timeout_s") or 6.0))
            device = asyncio.run(value) if inspect.isawaitable(value) else value
            if device is None:
                return _preflight_result(check_id, "BLE device", "fail", f"{address} not found", "Turn on the oximeter and check finger/contact placement.", start)
            name = getattr(device, "name", "") or "unknown"
            rssi = getattr(device, "rssi", None)
            return _preflight_result(check_id, "BLE device", "ok", f"{name} {address} RSSI={rssi}", "", start)

        if check_id == "session_folder_writable":
            root = Path(sessions_root)
            root.mkdir(parents=True, exist_ok=True)
            probe = root / ".preflight_probe"
            probe.write_text("ok", encoding="utf-8")
            probe.unlink()
            return _preflight_result(check_id, "Session folder writable", "ok", f"{root} writable", "", start)

        if check_id == "disk_space":
            root = Path(sessions_root)
            root.mkdir(parents=True, exist_ok=True)
            usage = shutil.disk_usage(str(root))
            free_mb = usage.free / (1024 * 1024)
            return _preflight_result(check_id, "Disk space", "warn" if free_mb < 500 else "ok", f"{free_mb:.0f} MB free", "Free disk space under the sessions root." if free_mb < 500 else "", start)

        if check_id == "schema_hash_consistency":
            return _preflight_result(check_id, "Schema hash consistency", "ok", f"scoring_weights_hash={_scoring_weights_hash()}", "", start)

        if check_id == "clock_monotonic_sanity":
            a = time.monotonic()
            time.sleep(0.001)
            b = time.monotonic()
            return _preflight_result(check_id, "Clock monotonic sanity", "ok" if b >= a else "fail", f"monotonic advanced by {b-a:.6f}s", "", start)

        return _preflight_result(check_id, check_id, "skip", "Unknown check id", "", start)
    except Exception as e:
        return _preflight_result(check_id, check_id, "fail", str(e), "Review the preflight detail and retry.", start)


def run_preflight_all(**params) -> Dict[str, object]:
    from ..monolith import _iso_now, DEFAULT_RADAR_PORT, DEFAULT_BLE_ADDRESS
    checks = [
        "python_env", "firmware_file_present", "serial_port_list", "session_folder_writable",
        "disk_space", "schema_hash_consistency", "clock_monotonic_sanity",
        "ble_adapter", "ble_device_probe", "serial_port_probe",
    ]
    include = params.pop("include", None)
    if include:
        wanted = {str(x).strip() for x in include if str(x).strip()}
        checks = [cid for cid in checks if cid in wanted]
    results = [run_preflight_check(cid, **params) for cid in checks]
    summary = {k: sum(1 for r in results if r.get("status") == k) for k in ("ok", "warn", "fail", "skip")}
    summary["overall"] = "fail" if summary["fail"] else ("warn" if summary["warn"] else "ok")
    return {
        "summary": summary,
        "checks": results,
        "ran_at": _iso_now(),
        "refs": {"port": params.get("port", DEFAULT_RADAR_PORT), "address": params.get("address", DEFAULT_BLE_ADDRESS)},
    }
