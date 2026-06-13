"""Unit tests for rvt_trainer.api.server_info.

The manifest is a contract for the dashboard's installability + the PWA
loader path — small drift breaks PWA install. The pair page is the only
operator-facing UI for LAN pairing; its TTL display and consumption note
are part of the operator workflow.
"""

from __future__ import annotations

import time

from rvt_trainer.api.server_info import (
    advertised_host,
    advertised_origin,
    manifest_payload,
    pair_page_html,
    support_matrix_html,
)


class _StubServer:
    def __init__(self, host: str = "", bound: str = "127.0.0.1",
                 port: int = 8765, tls: bool = False,
                 active_pin: str = "", expires_at: float = 0.0) -> None:
        self.advertised_host: str = host
        self.server_address = (bound, port)
        self.server_port: int = port
        self.tls_enabled: bool = tls
        self.active_pin: str = active_pin
        self.active_pin_expires_at: float = expires_at


def test_advertised_host_uses_explicit_override():
    s = _StubServer(host="trainer.local")
    assert advertised_host(s) == "trainer.local"


def test_advertised_host_falls_back_to_bound_address():
    s = _StubServer(bound="192.168.1.42")
    assert advertised_host(s) == "192.168.1.42"


def test_advertised_host_resolves_wildcard_to_lan_ip():
    # 0.0.0.0 must resolve to *some* concrete IP, not the wildcard itself.
    s = _StubServer(bound="0.0.0.0")
    host = advertised_host(s)
    assert host != "0.0.0.0"
    assert host != ""


def test_advertised_origin_format():
    s = _StubServer(host="example.lan", port=8765, tls=False)
    assert advertised_origin(s) == "http://example.lan:8765"
    s_tls = _StubServer(host="example.lan", port=8443, tls=True)
    assert advertised_origin(s_tls) == "https://example.lan:8443"


def test_manifest_payload_is_pwa_contract():
    s = _StubServer()
    m = manifest_payload(s)
    # PWA spec essentials
    assert m["id"] == "/"
    assert m["display"] == "standalone"
    assert m["start_url"] == "./"
    assert m["scope"] == "./"
    # Required icon sizes for Chrome installability
    sizes = {icon["sizes"] for icon in m["icons"]}
    assert "192x192" in sizes
    assert "512x512" in sizes
    # Maskable icon present for Android adaptive icons
    purposes = [icon.get("purpose", "") for icon in m["icons"]]
    assert "maskable" in purposes
    # Shortcuts for "Start session" + "Open last report"
    shortcut_names = {s["name"] for s in m["shortcuts"]}
    assert "Start session" in shortcut_names
    assert "Open last report" in shortcut_names


def test_support_matrix_html_lists_all_modes():
    s = _StubServer(host="loopback.local")
    page = support_matrix_html(s)
    # Five modes from the plan §0 support matrix
    for marker in (
        "Desktop browser",
        "Phone browser",
        "Capacitor APK",
        "Tauri EXE",
    ):
        assert marker in page
    assert "loopback.local" in page


def test_pair_page_html_with_active_pin():
    expires = time.time() + 60.0
    s = _StubServer(host="lan.local", active_pin="123456", expires_at=expires)
    page = pair_page_html(s)
    assert "123456" in page
    assert "lan.local" in page
    assert "Expires in" in page
    assert "consumed after first use" in page


def test_pair_page_html_without_active_pin():
    s = _StubServer(host="loopback.local", active_pin="")
    page = pair_page_html(s)
    assert "local" in page
    # When no PIN is active, the "consumed after first use" line shouldn't appear
    assert "LAN pairing is not active in local bind mode." in page


# ---------------------------------------------------------------------------
# /about copyright footer — WS1-D
# ---------------------------------------------------------------------------

def test_about_page_contains_all_author_names():
    """The /about (support matrix) page must name every project author."""
    s = _StubServer(host="loopback.local")
    page = support_matrix_html(s)
    for author in ("Lemuel Blaya", "Angelo Diaz", "Blessie Mugat"):
        assert author in page, f"Expected author '{author}' in /about response"


def test_about_page_contains_current_year():
    """The /about page must include the dynamically computed current year."""
    from datetime import datetime

    s = _StubServer(host="loopback.local")
    page = support_matrix_html(s)
    assert str(datetime.now().year) in page, (
        f"Expected year {datetime.now().year} in /about response"
    )


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__, "-v"]))
