"""BLE transport helpers for the trainer.

This module owns the real implementation of the Bluetooth Low Energy
scan used by the control server's ``/api/ble/scan`` endpoint. It returns
a JSON-serialisable payload describing the discovered peripherals (the
oximeter advertises here) without depending on the rest of the trainer
monolith at import time — the single monolith helper it needs
(:func:`_iso_now`) is imported lazily inside the function to keep this
package import-cheap and free of circular-import hazards.

* :func:`scan_ble_devices_payload` — discover nearby BLE peripherals and
  return ``{"ok", "devices", "count", "scanned_at"}`` (or an error
  payload that never raises to the caller).
"""

from __future__ import annotations

import asyncio
from typing import Dict, List

# Discovery timeout is clamped to a sane window so a wedged adapter can't
# hang the HTTP worker, and a fat-fingered query string can't spin for
# minutes.
_MIN_SCAN_TIMEOUT_S = 0.5
_MAX_SCAN_TIMEOUT_S = 12.0


def scan_ble_devices_payload(timeout_s: float = 3.0) -> Dict[str, object]:
    """Scan for nearby BLE peripherals and return a UI-ready payload.

    The function never raises: hardware/driver failures are captured and
    returned as ``{"ok": False, ..., "error": {...}}`` so the HTTP layer
    can always emit a clean ``200`` with a structured body.
    """
    # Deferred import avoids a circular dependency (monolith imports this
    # module back under its legacy underscored name).
    from ..monolith import _iso_now

    try:
        from bleak import BleakScanner

        timeout = max(_MIN_SCAN_TIMEOUT_S, min(float(timeout_s), _MAX_SCAN_TIMEOUT_S))
        devices = asyncio.run(BleakScanner.discover(timeout=timeout))
        out: List[Dict[str, object]] = []
        for d in devices or []:
            out.append(
                {
                    "address": str(getattr(d, "address", "") or ""),
                    "name": str(getattr(d, "name", "") or "BLE device"),
                    "rssi": getattr(d, "rssi", None),
                }
            )
        return {"ok": True, "devices": out, "count": len(out), "scanned_at": _iso_now()}
    except Exception as e:  # noqa: BLE001 — surfaced as structured error payload
        return {
            "ok": False,
            "devices": [],
            "count": 0,
            "error": {"code": "BLE_SCAN_FAILED", "message": str(e)},
            "scanned_at": _iso_now(),
        }


__all__ = ["scan_ble_devices_payload"]
