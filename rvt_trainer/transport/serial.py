"""Serial transport helpers."""

from __future__ import annotations
import serial
import serial.tools.list_ports
from typing import Dict, List, Optional, Tuple

DEFAULT_RADAR_PORT = "COM10"
CONTROL_API_SCHEMA_VERSION = "rvt-control-api-v12.0"

def auto_detect_radar_port(default: str = DEFAULT_RADAR_PORT) -> str:
    try:
        ports = list(serial.tools.list_ports.comports())
    except Exception:
        return default
    if not ports:
        return default
    scored = []
    hints = ("xiao", "esp32", "usb serial", "usb-serial", "wch", "cp210", "ch340", "arduino")
    for p in ports:
        text = " ".join(str(x or "") for x in [
            getattr(p, "device", ""),
            getattr(p, "description", ""),
            getattr(p, "manufacturer", ""),
            getattr(p, "hwid", "")
        ]).lower()
        score = sum(1 for h in hints if h in text)
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


def probe_serial_port(port: str, timeout_s: float = 4.0) -> Tuple[List[str], Optional[str]]:
    lines = []
    try:
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
