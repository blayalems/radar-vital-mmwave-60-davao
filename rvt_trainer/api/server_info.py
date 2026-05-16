"""Server-info, manifest, and pairing-page response helpers.

These build JSON/HTML payloads served by the trainer's HTTP dispatcher.
Pure helpers — no I/O, no socket access. The caller (dispatcher in
:mod:`rvt_trainer.monolith`) reads the resulting payload and writes the
HTTP response.

* :func:`advertised_host` — host the trainer prints in QR/manifest. Resolves
  ``0.0.0.0``/``::`` to a guessed LAN address; otherwise returns the bound
  address.
* :func:`advertised_origin` — ``scheme://host:port`` string for QR codes
  and CSP ``connect-src``.
* :func:`manifest_payload` — the dynamic ``manifest.webmanifest`` body.
* :func:`support_matrix_html` — operator-facing HTML page enumerating PWA /
  SW / BLE / LAN-API support per launch mode.
* :func:`pair_page_html` — operator-facing ``/pair`` HTML showing the
  current PIN and consumption rules.
"""

from __future__ import annotations

import time
from html import escape as html_escape
from typing import Dict, List, Tuple


def _server_scheme(server) -> str:
    return "https" if bool(getattr(server, "tls_enabled", False)) else "http"


def advertised_host(server) -> str:
    """Pick the host string the trainer should print in its QR + manifest.

    ``server.advertised_host`` (set by the CLI when the operator passed an
    explicit ``--host`` override) wins; otherwise the bound address is used,
    with ``0.0.0.0``/``::`` resolved to the best-guess LAN IP.
    """
    host = str(getattr(server, "advertised_host", "") or "")
    if host:
        return host
    bound = server.server_address[0] if getattr(server, "server_address", None) else "127.0.0.1"
    if bound in {"", "0.0.0.0", "::"}:
        # Deferred import: rvt_trainer.monolith imports BACK from this module,
        # so a top-level import would cycle. _guess_lan_ip is a pure stdlib
        # helper that doesn't change behavior across the boundary.
        from rvt_trainer.monolith import _guess_lan_ip
        return _guess_lan_ip()
    return bound


def advertised_origin(server) -> str:
    """``scheme://host:port`` — the canonical URL of this trainer."""
    return f"{_server_scheme(server)}://{advertised_host(server)}:{int(server.server_port)}"


def manifest_payload(server) -> Dict[str, object]:
    """The body of the dynamic ``/manifest.webmanifest`` response.

    Identical for every bind mode — the manifest deliberately uses relative
    paths so the same payload works on loopback, LAN, hosted GitHub Pages,
    Capacitor, and Tauri.
    """
    return {
        "id": "/",
        "name": "Radar Vital Trainer",
        "short_name": "Radar Vital",
        "start_url": "./live_dashboard.html",
        "scope": "./",
        "display": "standalone",
        "background_color": "#f4f6fb",
        "theme_color": "#3b82f6",
        "orientation": "any",
        "categories": ["health", "medical"],
        "icons": [
            {"src": "./icons/icon-192.png", "sizes": "192x192", "type": "image/png"},
            {"src": "./icons/icon-512.png", "sizes": "512x512", "type": "image/png"},
            {"src": "./icons/icon-maskable-512.png", "sizes": "512x512", "type": "image/png", "purpose": "maskable"},
        ],
        "shortcuts": [
            {"name": "Start session", "url": "./live_dashboard.html#start"},
            {"name": "Open last report", "url": "./live_dashboard.html#report"},
        ],
    }


def _support_matrix_rows() -> List[Tuple[str, str, str, str, str]]:
    return [
        ("Desktop browser -> http://127.0.0.1:8765", "yes", "yes", "yes, Chromium", "yes"),
        ("Phone browser -> http://<lan-ip>:8765", "no", "no", "no", "yes, read-only without PIN"),
        ("Phone browser -> https://<lan-ip>:8765 (--tls)", "yes after cert trust", "yes", "yes, Android Chrome", "yes"),
        ("Capacitor APK", "native", "local cache", "BLE plugin", "native HTTP bridge"),
        ("Tauri EXE", "native", "local cache", "plugin or trainer HTTP", "yes"),
    ]


def support_matrix_html(server) -> str:
    """Operator-facing HTML page documenting the per-mode support matrix."""
    rows = "\n".join(
        "<tr>" + "".join(f"<td>{html_escape(cell)}</td>" for cell in row) + "</tr>"
        for row in _support_matrix_rows()
    )
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Radar Vital Support Matrix</title>
<style>body{{font-family:system-ui,sans-serif;margin:32px;line-height:1.45;color:#0b1220;background:#f4f6fb}}table{{border-collapse:collapse;width:100%;background:#fff}}th,td{{border:1px solid #dde3ee;padding:10px;text-align:left}}th{{background:#eef2f8}}</style>
</head><body><h1>Radar Vital v12 Support Matrix</h1><p>Origin: {html_escape(advertised_origin(server))}</p>
<table><thead><tr><th>Mode</th><th>PWA install</th><th>Service Worker</th><th>Web Bluetooth</th><th>LAN HTTP API</th></tr></thead><tbody>{rows}</tbody></table>
</body></html>"""


def pair_page_html(server) -> str:
    """Operator-facing ``/pair`` page showing the active PIN and TTL."""
    pin = getattr(server, "active_pin", "") or ""
    origin = advertised_origin(server)
    expires_at = float(getattr(server, "active_pin_expires_at", 0.0) or 0.0)
    ttl = max(0, int(expires_at - time.time())) if pin else 0
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Pair Radar Vital</title>
<style>body{{font-family:system-ui,sans-serif;margin:0;min-height:100vh;display:grid;place-items:center;background:#0b1220;color:#e6eefc}}main{{width:min(560px,calc(100vw - 32px));background:#121c31;border:1px solid #26344f;border-radius:18px;padding:24px}}.pin{{font-size:48px;letter-spacing:.18em;font-weight:800}}code{{background:#0b1220;padding:2px 6px;border-radius:6px}}</style>
</head><body><main><h1>Pair Radar Vital</h1><p>Server URL: <code>{html_escape(origin)}</code></p>
<p>PIN</p><div class="pin">{html_escape(pin or "local")}</div>
<p>{'Expires in ' + str(ttl) + ' seconds and is consumed after first use.' if pin else 'LAN pairing is not active in local bind mode.'}</p>
<p>Scan the trainer QR or open <code>{html_escape(origin)}/?pair={html_escape(pin)}</code> on the phone, then keep the trainer running.</p>
</main></body></html>"""


__all__ = [
    "advertised_host",
    "advertised_origin",
    "manifest_payload",
    "support_matrix_html",
    "pair_page_html",
]
