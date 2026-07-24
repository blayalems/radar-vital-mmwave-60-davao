"""Health, hardware, and live-telemetry route declarations."""

from .types import AuthPolicy, RouteGroup, route

ROUTES = (
    route("health", "GET", "/api/health", RouteGroup.TELEMETRY, AuthPolicy.PUBLIC),
    route("version", "GET", "/api/version", RouteGroup.TELEMETRY, AuthPolicy.PUBLIC),
    route("update_manifest", "GET", "/api/update/manifest", RouteGroup.TELEMETRY, AuthPolicy.PUBLIC),
    route("help_schema", "GET", "/api/help/schema", RouteGroup.TELEMETRY, AuthPolicy.PUBLIC),
    route("ble_scan", "GET", "/api/ble/scan", RouteGroup.TELEMETRY, AuthPolicy.OPERATOR),
    route("trainer_log", "GET", "/api/trainer/log", RouteGroup.TELEMETRY, AuthPolicy.OPERATOR),
    route("status", "GET", "/api/status", RouteGroup.TELEMETRY, AuthPolicy.OPERATOR),
    route("defaults", "GET|POST", "/api/defaults", RouteGroup.TELEMETRY, AuthPolicy.OPERATOR),
    route("serial_ports", "GET", "/api/serial/ports", RouteGroup.TELEMETRY, AuthPolicy.OPERATOR),
    route("preflight_all", "GET", "/api/preflight", RouteGroup.TELEMETRY, AuthPolicy.OPERATOR),
    route("preflight_one", "POST", "/api/preflight/*", RouteGroup.TELEMETRY, AuthPolicy.OPERATOR),
    route("session_events", "GET", "/api/session/events", RouteGroup.TELEMETRY, AuthPolicy.SSE),
    route("events_subscribe", "GET", "/api/events/subscribe", RouteGroup.TELEMETRY, AuthPolicy.SSE),
    route("recorded_session_events", "GET", "/api/sessions/*/events", RouteGroup.TELEMETRY, AuthPolicy.SSE),
    route("session_current", "GET", "/api/session/current", RouteGroup.TELEMETRY, AuthPolicy.OPERATOR),
    route(
        "session_live_dashboard",
        "GET",
        "/api/session/current/live_dashboard.json",
        RouteGroup.TELEMETRY,
        AuthPolicy.OPERATOR,
    ),
    route("session_buffer", "GET", "/api/session/buffer", RouteGroup.TELEMETRY, AuthPolicy.OPERATOR),
)

__all__ = ["ROUTES"]
