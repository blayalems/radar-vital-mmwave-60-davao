"""Typed route-registry primitives."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fnmatch import fnmatchcase
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
        return str(method).upper() in self.methods and fnmatchcase(
            str(path),
            self.pattern,
        )


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
