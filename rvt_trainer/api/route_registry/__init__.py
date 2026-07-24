"""Grouped control-API route and authorization registry."""

from __future__ import annotations

from typing import Optional, Tuple

from . import auth, reports, sessions, static, telemetry
from .types import AuthPolicy, RouteGroup, RouteSpec

ROUTES: Tuple[RouteSpec, ...] = (
    *auth.ROUTES,
    *sessions.ROUTES,
    *telemetry.ROUTES,
    *reports.ROUTES,
    *static.ROUTES,
)


def match_route(method: str, path: str) -> Optional[RouteSpec]:
    for spec in ROUTES:
        if spec.matches(method, path):
            return spec
    return None


def authorization_for(method: str, path: str) -> AuthPolicy:
    spec = match_route(method, path)
    if spec is not None:
        return spec.auth
    return AuthPolicy.OPERATOR if str(path).startswith("/api/") else AuthPolicy.PUBLIC


def group_for(method: str, path: str) -> Optional[RouteGroup]:
    spec = match_route(method, path)
    return spec.group if spec is not None else None


__all__ = [
    "AuthPolicy",
    "ROUTES",
    "RouteGroup",
    "RouteSpec",
    "authorization_for",
    "group_for",
    "match_route",
]
