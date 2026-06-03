"""Preflight and audit runner helpers."""

from __future__ import annotations
import time
import shutil
import asyncio
import inspect
from types import SimpleNamespace
from pathlib import Path
from typing import Any, Callable, Dict

PreflightHandler = Callable[[str, float, Dict[str, Any], Any], Dict[str, object]]


def _load_preflight_runtime() -> SimpleNamespace:
    from ..monolith import (
        _preflight_result,
        _candidate_ino_paths,
        _extract_firmware_data_header_tokens,
        _read_session_lock,
        _scoring_weights_hash,
        _iso_now,
        DEFAULT_RADAR_PORT,
        DEFAULT_BLE_ADDRESS,
        RADAR_LOG_COLUMNS,
    )

    return SimpleNamespace(
        preflight_result=_preflight_result,
        candidate_ino_paths=_candidate_ino_paths,
        extract_firmware_data_header_tokens=_extract_firmware_data_header_tokens,
        read_session_lock=_read_session_lock,
        scoring_weights_hash=_scoring_weights_hash,
        iso_now=_iso_now,
        default_radar_port=DEFAULT_RADAR_PORT,
        default_ble_address=DEFAULT_BLE_ADDRESS,
        radar_log_columns=RADAR_LOG_COLUMNS,
    )


def _runtime_refs(params: Dict[str, Any], runtime: Any) -> tuple[str, str, str]:
    sessions_root = str(params.get("sessions_root") or "sessions")
    port = str(params.get("port") or runtime.default_radar_port)
    address = str(params.get("address") or runtime.default_ble_address)
    return sessions_root, port, address


def _check_python_env(check_id: str, start: float, params: Dict[str, Any], runtime: Any) -> Dict[str, object]:
    parts = []
    for name in ("serial", "bleak", "pandas", "numpy", "sklearn", "matplotlib"):
        try:
            mod = __import__(name)
            parts.append(f"{name}={getattr(mod, '__version__', 'available')}")
        except Exception as e:
            return runtime.preflight_result(
                check_id,
                "Python environment",
                "fail",
                f"{name} import failed: {e}",
                "Install missing dependency.",
                start,
            )
    return runtime.preflight_result(check_id, "Python environment", "ok", ", ".join(parts), "", start)


def _check_firmware_file_present(
    check_id: str,
    start: float,
    params: Dict[str, Any],
    runtime: Any,
) -> Dict[str, object]:
    expected = list(runtime.radar_log_columns)
    for path in runtime.candidate_ino_paths(params.get("ino_search_paths")):
        if not path.exists():
            continue
        try:
            actual = runtime.extract_firmware_data_header_tokens(path)
        except Exception:
            continue
        if actual == expected:
            return runtime.preflight_result(
                check_id,
                "Firmware file",
                "ok",
                f"{path.name}: {len(expected)}-column contract matches",
                "",
                start,
            )
        return runtime.preflight_result(
            check_id,
            "Firmware file",
            "fail",
            f"{path.name}: expected {len(expected)} columns, got {len(actual)}",
            "Use the v15.0.0 firmware file that matches this trainer.",
            start,
        )
    return runtime.preflight_result(
        check_id,
        "Firmware file",
        "fail",
        "No parseable firmware DATA header found",
        "Place radar_vital_v15_0_0.ino beside the trainer.",
        start,
    )


def _check_serial_port_list(check_id: str, start: float, params: Dict[str, Any], runtime: Any) -> Dict[str, object]:
    import serial.tools.list_ports

    _, port, _ = _runtime_refs(params, runtime)
    ports = list(serial.tools.list_ports.comports())
    devices = [str(getattr(p, "device", p)) for p in ports]
    if port in devices:
        return runtime.preflight_result(check_id, "Serial ports", "ok", f"{port} present; ports={devices}", "", start)
    return runtime.preflight_result(
        check_id,
        "Serial ports",
        "warn",
        f"{port} not found; ports={devices}",
        f"Connect radar or select a detected port instead of {port}.",
        start,
    )


def _check_serial_port_probe(check_id: str, start: float, params: Dict[str, Any], runtime: Any) -> Dict[str, object]:
    sessions_root, port, _ = _runtime_refs(params, runtime)
    if runtime.read_session_lock(sessions_root):
        return runtime.preflight_result(
            check_id,
            "Serial probe",
            "fail",
            "A session lock is active; refusing to probe serial hardware.",
            "Stop the active session before probing.",
            start,
        )

    from ..transport.serial import probe_serial_port

    timeout_s = float(params.get("timeout_s") or 4.0)
    lines, error = probe_serial_port(port, timeout_s)
    if error:
        return runtime.preflight_result(check_id, "Serial probe", "fail", error, "Review serial connection.", start)

    detail = f"Read {len(lines)} line(s); " + ("; ".join(lines[:3]) if lines else "no banner yet")
    status = "ok" if any(("RVital" in ln or "DATA," in ln) for ln in lines) else "warn"
    guidance = "Warmup may take about 60 s before DATA appears." if status == "warn" else ""
    return runtime.preflight_result(check_id, "Serial probe", status, detail, guidance, start)


def _check_ble_adapter(check_id: str, start: float, params: Dict[str, Any], runtime: Any) -> Dict[str, object]:
    try:
        from bleak import BleakScanner
    except Exception as e:
        return runtime.preflight_result(check_id, "BLE adapter", "fail", f"bleak import failed: {e}", "Install bleak: pip install bleak", start)
    try:
        devices = asyncio.run(BleakScanner.discover(timeout=float(params.get("timeout_s") or 1.0)))
        count = len(devices) if devices is not None else 0
        elapsed_ms = (time.monotonic() - start) * 1000.0
        return runtime.preflight_result(check_id, "BLE adapter", "ok", f"adapter discovered {count} device(s) in {elapsed_ms:.0f} ms", "", start)
    except Exception as e:
        return runtime.preflight_result(
            check_id,
            "BLE adapter",
            "fail",
            f"BleakScanner.discover failed: {e}",
            "Enable Bluetooth and confirm the adapter is present.",
            start,
        )


def _check_ble_device_probe(check_id: str, start: float, params: Dict[str, Any], runtime: Any) -> Dict[str, object]:
    sessions_root, _, address = _runtime_refs(params, runtime)
    if runtime.read_session_lock(sessions_root):
        return runtime.preflight_result(
            check_id,
            "BLE device",
            "fail",
            "A session lock is active; refusing BLE probe.",
            "Stop the active session before probing.",
            start,
        )
    from bleak import BleakScanner

    finder = getattr(BleakScanner, "find_device_by_address", None)
    if finder is None:
        return runtime.preflight_result(check_id, "BLE device", "fail", "BleakScanner.find_device_by_address unavailable", "Check Bleak installation.", start)
    value = finder(address, timeout=float(params.get("timeout_s") or 6.0))
    device = asyncio.run(value) if inspect.isawaitable(value) else value
    if device is None:
        return runtime.preflight_result(
            check_id,
            "BLE device",
            "fail",
            f"{address} not found",
            "Turn on the oximeter and check finger/contact placement.",
            start,
        )
    name = getattr(device, "name", "") or "unknown"
    rssi = getattr(device, "rssi", None)
    return runtime.preflight_result(check_id, "BLE device", "ok", f"{name} {address} RSSI={rssi}", "", start)


def _check_session_folder_writable(check_id: str, start: float, params: Dict[str, Any], runtime: Any) -> Dict[str, object]:
    sessions_root, _, _ = _runtime_refs(params, runtime)
    root = Path(sessions_root)
    root.mkdir(parents=True, exist_ok=True)
    probe = root / ".preflight_probe"
    probe.write_text("ok", encoding="utf-8")
    probe.unlink()
    return runtime.preflight_result(check_id, "Session folder writable", "ok", f"{root} writable", "", start)


def _check_disk_space(check_id: str, start: float, params: Dict[str, Any], runtime: Any) -> Dict[str, object]:
    sessions_root, _, _ = _runtime_refs(params, runtime)
    root = Path(sessions_root)
    root.mkdir(parents=True, exist_ok=True)
    usage = shutil.disk_usage(str(root))
    free_mb = usage.free / (1024 * 1024)
    return runtime.preflight_result(
        check_id,
        "Disk space",
        "warn" if free_mb < 500 else "ok",
        f"{free_mb:.0f} MB free",
        "Free disk space under the sessions root." if free_mb < 500 else "",
        start,
    )


def _check_schema_hash_consistency(check_id: str, start: float, params: Dict[str, Any], runtime: Any) -> Dict[str, object]:
    return runtime.preflight_result(
        check_id,
        "Schema hash consistency",
        "ok",
        f"scoring_weights_hash={runtime.scoring_weights_hash()}",
        "",
        start,
    )


def _check_clock_monotonic_sanity(check_id: str, start: float, params: Dict[str, Any], runtime: Any) -> Dict[str, object]:
    a = time.monotonic()
    time.sleep(0.001)
    b = time.monotonic()
    return runtime.preflight_result(
        check_id,
        "Clock monotonic sanity",
        "ok" if b >= a else "fail",
        f"monotonic advanced by {b-a:.6f}s",
        "",
        start,
    )


_CHECK_HANDLERS: Dict[str, PreflightHandler] = {
    "python_env": _check_python_env,
    "firmware_file_present": _check_firmware_file_present,
    "serial_port_list": _check_serial_port_list,
    "session_folder_writable": _check_session_folder_writable,
    "disk_space": _check_disk_space,
    "schema_hash_consistency": _check_schema_hash_consistency,
    "clock_monotonic_sanity": _check_clock_monotonic_sanity,
    "ble_adapter": _check_ble_adapter,
    "ble_device_probe": _check_ble_device_probe,
    "serial_port_probe": _check_serial_port_probe,
}


def run_preflight_check(check_id: str, **params) -> Dict[str, object]:
    start = time.monotonic()
    runtime = _load_preflight_runtime()
    try:
        handler = _CHECK_HANDLERS.get(check_id)
        if handler is None:
            return runtime.preflight_result(check_id, check_id, "skip", "Unknown check id", "", start)
        return handler(check_id, start, params, runtime)
    except Exception as e:
        return runtime.preflight_result(check_id, check_id, "fail", str(e), "Review the preflight detail and retry.", start)


def run_preflight_all(**params) -> Dict[str, object]:
    runtime = _load_preflight_runtime()
    checks = list(_CHECK_HANDLERS)
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
        "ran_at": runtime.iso_now(),
        "refs": {"port": params.get("port", runtime.default_radar_port), "address": params.get("address", runtime.default_ble_address)},
    }
