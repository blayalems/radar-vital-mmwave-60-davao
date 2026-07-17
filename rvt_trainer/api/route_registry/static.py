"""PWA, shell, and static-asset route declarations."""

from .types import AuthPolicy, RouteGroup, route

ROUTES = (
    route("legacy_service_worker", "GET", "/rvt-sw.js", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("service_worker", "GET", "/sw.js", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("manifest", "GET", "/manifest.webmanifest", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("about", "GET", "/about", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("pair", "GET", "/pair", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("icons", "GET", "/icons/*", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("libraries", "GET", "/lib/*", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("fonts", "GET", "/fonts/*", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("shell_root", "GET", "/", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("shell_index", "GET", "/index.html", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("shell_connect", "GET", "/connect", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("shell_dashboard", "GET", "/dashboard", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("shell_live", "GET", "/live", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("shell_home", "GET", "/home", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("shell_settings", "GET", "/settings", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("shell_report", "GET", "/report", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("shell_help", "GET", "/help", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    route("legacy_live_dashboard", "GET", "/live_dashboard.html", RouteGroup.STATIC, AuthPolicy.PUBLIC),
)

__all__ = ["ROUTES"]
