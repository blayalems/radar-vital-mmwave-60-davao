"""Serial transport helpers for the trainer.

This module owns the real implementation of radar serial-port
auto-detection used at session start and by the preflight wizard. The
heuristic scores each enumerated COM/tty port against a list of known
USB-UART chipset hints (the XIAO ESP32 radar board advertises one of
these) and returns the best match, falling back to a caller-supplied
default when nothing scores.

* :func:`auto_detect_radar_port` — choose the most likely radar serial
  port, or return ``default`` when detection is inconclusive.
"""

from __future__ import annotations

# USB-UART bridge / dev-board identifiers that strongly suggest the
# attached device is the radar board rather than an unrelated serial
# peripheral. Matched case-insensitively against the joined
# device/description/manufacturer/hwid text.
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


def auto_detect_radar_port(default: str = "COM10") -> str:
    """Return the most likely radar serial port, or ``default``.

    Enumerates the host's serial ports and scores each by how many radar
    chipset hints appear in its metadata. The highest-scoring named port
    wins. When :mod:`pyserial` is unavailable, no ports exist, or nothing
    scores, ``default`` is returned unchanged so the caller's configured
    port is preserved.
    """
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


__all__ = ["auto_detect_radar_port"]
