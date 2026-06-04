"""Unit tests for rvt_trainer.transport.{ble,serial}.

These modules own the Phase 5–extracted hardware-discovery helpers that
back the trainer's BLE scan and radar serial-port auto-detection. The
real hardware path is exercised in the field; here we pin the pure logic:

* serial port scoring + fallback semantics (no real radar attached),
* BLE scan returning a structured, never-raising payload,
* the monolith re-export aliases stay wired to these implementations.
"""

from __future__ import annotations

import sys
import types

import rvt_trainer.monolith as monolith
from rvt_trainer.transport import ble, serial


# --------------------------------------------------------------------------
# serial.auto_detect_radar_port
# --------------------------------------------------------------------------

class _StubPort:
    def __init__(self, device, description="", manufacturer="", hwid=""):
        self.device = device
        self.description = description
        self.manufacturer = manufacturer
        self.hwid = hwid


def _install_fake_comports(monkeypatch, ports):
    """Install a fake serial.tools.list_ports.comports() returning *ports*."""
    fake_list_ports = types.ModuleType("serial.tools.list_ports")
    fake_list_ports.comports = lambda: list(ports)
    fake_tools = types.ModuleType("serial.tools")
    fake_tools.list_ports = fake_list_ports
    fake_serial = types.ModuleType("serial")
    fake_serial.tools = fake_tools
    monkeypatch.setitem(sys.modules, "serial", fake_serial)
    monkeypatch.setitem(sys.modules, "serial.tools", fake_tools)
    monkeypatch.setitem(sys.modules, "serial.tools.list_ports", fake_list_ports)


def test_auto_detect_prefers_known_chipset_hint(monkeypatch):
    ports = [
        _StubPort("/dev/ttyS0", description="standard serial"),
        _StubPort("/dev/ttyUSB0", description="XIAO ESP32-C3 USB Serial", manufacturer="WCH"),
    ]
    _install_fake_comports(monkeypatch, ports)
    assert serial.auto_detect_radar_port("COM10") == "/dev/ttyUSB0"


def test_auto_detect_returns_default_when_no_ports(monkeypatch):
    _install_fake_comports(monkeypatch, [])
    assert serial.auto_detect_radar_port("COM10") == "COM10"


def test_auto_detect_single_unscored_port_is_returned(monkeypatch):
    # A lone serial port with no chipset hints is still the best guess.
    _install_fake_comports(monkeypatch, [_StubPort("/dev/ttyS0", description="plain")])
    assert serial.auto_detect_radar_port("COM10") == "/dev/ttyS0"


def test_auto_detect_multiple_unscored_ports_fall_back_to_default(monkeypatch):
    ports = [
        _StubPort("/dev/ttyS0", description="plain a"),
        _StubPort("/dev/ttyS1", description="plain b"),
    ]
    _install_fake_comports(monkeypatch, ports)
    assert serial.auto_detect_radar_port("COM10") == "COM10"


def test_auto_detect_returns_default_when_pyserial_missing(monkeypatch):
    # Simulate pyserial not being importable.
    monkeypatch.setitem(sys.modules, "serial", None)
    assert serial.auto_detect_radar_port("COM10") == "COM10"


def test_auto_detect_highest_score_wins(monkeypatch):
    ports = [
        _StubPort("/dev/ttyUSB0", description="cp210 usb serial"),                # 2 hints
        _StubPort("/dev/ttyUSB1", description="xiao esp32 usb-serial wch ch340"),  # 5 hints
    ]
    _install_fake_comports(monkeypatch, ports)
    assert serial.auto_detect_radar_port("COM10") == "/dev/ttyUSB1"


# --------------------------------------------------------------------------
# ble.scan_ble_devices_payload
# --------------------------------------------------------------------------

def test_ble_scan_returns_structured_error_when_bleak_missing(monkeypatch):
    # No bleak in the sandbox → structured failure payload, never raises.
    monkeypatch.setitem(sys.modules, "bleak", None)
    payload = ble.scan_ble_devices_payload(timeout_s=0.5)
    assert payload["ok"] is False
    assert payload["devices"] == []
    assert payload["count"] == 0
    assert payload["error"]["code"] == "BLE_SCAN_FAILED"
    assert "scanned_at" in payload


def test_ble_scan_maps_discovered_devices(monkeypatch):
    class _Dev:
        def __init__(self, address, name, rssi):
            self.address = address
            self.name = name
            self.rssi = rssi

    class _FakeScanner:
        @staticmethod
        async def discover(timeout):  # noqa: D401 - async stub
            return [
                _Dev("AA:BB:CC:DD:EE:FF", "Oximeter", -55),
                _Dev("11:22:33:44:55:66", "", -80),  # unnamed → default label
            ]

    fake_bleak = types.ModuleType("bleak")
    fake_bleak.BleakScanner = _FakeScanner
    monkeypatch.setitem(sys.modules, "bleak", fake_bleak)

    payload = ble.scan_ble_devices_payload(timeout_s=1.0)
    assert payload["ok"] is True
    assert payload["count"] == 2
    first, second = payload["devices"]
    assert first == {"address": "AA:BB:CC:DD:EE:FF", "name": "Oximeter", "rssi": -55}
    assert second["name"] == "BLE device"  # empty name defaulted


# --------------------------------------------------------------------------
# monolith re-export wiring (back-compat for legacy underscored names)
# --------------------------------------------------------------------------

def test_monolith_reexports_point_at_extracted_modules():
    assert monolith._scan_ble_devices_payload is ble.scan_ble_devices_payload
    assert monolith._auto_detect_radar_port is serial.auto_detect_radar_port


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__, "-v"]))
