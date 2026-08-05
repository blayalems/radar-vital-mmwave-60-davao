"""Authentication and operator-management route declarations."""

from .types import AuthPolicy, RouteGroup, route

ROUTES = (
    route("auth_exchange", "POST", "/api/auth/exchange", RouteGroup.AUTH, AuthPolicy.PUBLIC),
    route("auth_validate", "GET", "/api/auth/validate", RouteGroup.AUTH, AuthPolicy.OPERATOR),
    route("auth_login", "POST", "/api/auth/login", RouteGroup.AUTH, AuthPolicy.DISCOVERY),
    route("auth_logout", "POST", "/api/auth/logout", RouteGroup.AUTH, AuthPolicy.OPERATOR),
    route("auth_sse_token", "POST", "/api/auth/sse-token", RouteGroup.AUTH, AuthPolicy.OPERATOR),
    route("auth_reset_pin", "POST", "/api/auth/reset-pin", RouteGroup.AUTH, AuthPolicy.RECOVERY),
    route("auth_host_reset", "POST", "/api/auth/host-reset", RouteGroup.AUTH, AuthPolicy.LOOPBACK),
    route("operator_profiles_get", "GET", "/api/operator-profiles", RouteGroup.AUTH, AuthPolicy.DISCOVERY),
    route("operator_profiles_create", "POST", "/api/operator-profiles", RouteGroup.AUTH, AuthPolicy.BOOTSTRAP),
    route("subject_profiles", "GET", "/api/subject-profiles", RouteGroup.AUTH, AuthPolicy.OPERATOR),
    route("subject_profiles_put", "PUT", "/api/subject-profiles", RouteGroup.AUTH, AuthPolicy.OPERATOR),
    route("server_info", "GET", "/api/server-info", RouteGroup.AUTH, AuthPolicy.DISCOVERY),
    route("native_pairing_info", "GET", "/api/native-pairing-info", RouteGroup.AUTH, AuthPolicy.DISCOVERY),
)

__all__ = ["ROUTES"]
