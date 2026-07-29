"""Typed route-registry primitives."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import re
from typing import FrozenSet


class AuthPolicy(str, Enum):
    PUBLIC = "public"
    DISCOVERY = "discovery"
    BOOTSTRAP = "bootstrap"
    RECOVERY = "recovery"
    LOOPBACK = "loopback"
    OPERATOR = "operator"
    SSE = "sse"


class RouteGroup(str, Enum):
    AUTH = "auth"
    SESSIONS = "sessions"
    TELEMETRY = "telemetry"
    REPORTS = "reports"
    STATIC = "static"


@dataclass(frozen=True)
class RouteSpec:
    name: str
    methods: FrozenSet[str]
    pattern: str
    group: RouteGroup
    auth: AuthPolicy

    def matches(self, method: str, path: str) -> bool:
        if str(method).upper() not in self.methods:
            return False
        # Route wildcards are path aware: ``*`` is exactly one non-empty
        # segment, while ``**`` is an explicitly recursive tail.  fnmatch's
        # plain ``*`` also consumes "/", which previously let malformed paths
        # such as /api/sessions/a/b/summary select a one-session handler.
        expression = re.escape(self.pattern)
        expression = expression.replace(r"\*\*", r".+")
        expression = expression.replace(r"\*", r"[^/]+")
        return re.fullmatch(expression, str(path)) is not None


def route(
    name: str,
    methods: str,
    pattern: str,
    group: RouteGroup,
    auth: AuthPolicy,
) -> RouteSpec:
    return RouteSpec(
        name=name,
        methods=frozenset(part.strip().upper() for part in methods.split("|")),
        pattern=pattern,
        group=group,
        auth=auth,
    )


__all__ = ["AuthPolicy", "RouteGroup", "RouteSpec", "route"]
