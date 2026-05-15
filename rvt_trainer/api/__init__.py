"""HTTP API helpers for the Radar Vital trainer."""

from .auth import exchange_pair_pin, make_pair_pin
from .server_info import manifest_payload, pair_page_html, support_matrix_html
from .sse import ControlHandler, SessionSupervisor

__all__ = [
    "ControlHandler",
    "SessionSupervisor",
    "exchange_pair_pin",
    "make_pair_pin",
    "manifest_payload",
    "pair_page_html",
    "support_matrix_html",
]
