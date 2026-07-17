"""Table-driven contracts for grouped trainer routes and authorization."""

from __future__ import annotations

import ast
import inspect
import textwrap
from dataclasses import dataclass
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from rvt_trainer.api.route_registry import (
    AuthPolicy,
    ROUTES,
    RouteGroup,
    authorization_for,
    group_for,
    match_route,
)
from rvt_trainer.monolith import _ControlHandler


@pytest.mark.parametrize(
    ("method", "path", "group", "auth"),
    [
        ("POST", "/api/auth/exchange", RouteGroup.AUTH, AuthPolicy.PUBLIC),
        ("GET", "/api/server-info", RouteGroup.AUTH, AuthPolicy.DISCOVERY),
        ("POST", "/api/operator-profiles", RouteGroup.AUTH, AuthPolicy.BOOTSTRAP),
        ("POST", "/api/auth/reset-pin", RouteGroup.AUTH, AuthPolicy.RECOVERY),
        ("POST", "/api/auth/host-reset", RouteGroup.AUTH, AuthPolicy.LOOPBACK),
        ("GET", "/api/events/subscribe", RouteGroup.TELEMETRY, AuthPolicy.SSE),
        ("GET", "/api/sessions/s01/events", RouteGroup.TELEMETRY, AuthPolicy.SSE),
        ("POST", "/api/session/start", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
        ("PUT", "/api/sessions/s01/signoff", RouteGroup.SESSIONS, AuthPolicy.OPERATOR),
        ("GET", "/api/sessions/s01/summary", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
        ("POST", "/api/sessions/s01/analyse", RouteGroup.REPORTS, AuthPolicy.OPERATOR),
        ("GET", "/sw.js", RouteGroup.STATIC, AuthPolicy.PUBLIC),
    ],
)
def test_route_matrix(method, path, group, auth):
    spec = match_route(method, path)

    assert spec is not None
    assert spec.group == group
    assert spec.auth == auth
    assert group_for(method, path) == group
    assert authorization_for(method, path) == auth


def test_registry_entries_are_unique_and_match_their_own_pattern():
    keys = [(tuple(sorted(spec.methods)), spec.pattern) for spec in ROUTES]

    assert len(keys) == len(set(keys))
    for spec in ROUTES:
        sample = spec.pattern.replace("*", "s01", 1).replace("*", "file.json")
        for method in spec.methods:
            assert spec.matches(method, sample)


def test_http_dispatch_uses_registry_names_instead_of_route_pattern_copies():
    source = "\n".join(
        textwrap.dedent(inspect.getsource(getattr(_ControlHandler, method)))
        for method in ("do_GET", "do_POST", "do_PUT", "do_DELETE")
    )
    tree = ast.parse(source)
    dispatched_names = set()
    path_prefixes = []

    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Compare)
            and isinstance(node.left, ast.Name)
            and node.left.id == "route_name"
        ):
            for candidate in node.comparators:
                if isinstance(candidate, ast.Constant) and isinstance(candidate.value, str):
                    dispatched_names.add(candidate.value)
                elif isinstance(candidate, (ast.List, ast.Set, ast.Tuple)):
                    dispatched_names.update(
                        element.value
                        for element in candidate.elts
                        if isinstance(element, ast.Constant)
                        and isinstance(element.value, str)
                    )
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr in {"startswith", "endswith"}
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "path"
        ):
            path_prefixes.extend(
                argument.value
                for argument in node.args
                if isinstance(argument, ast.Constant)
                and isinstance(argument.value, str)
            )

    assert dispatched_names == {spec.name for spec in ROUTES}
    assert set(path_prefixes) == {"/api/"}


def test_unknown_api_routes_fail_closed_but_unknown_static_paths_stay_public():
    assert authorization_for("GET", "/api/not-a-real-route") == AuthPolicy.OPERATOR
    assert authorization_for("GET", "/compiled-angular-chunk.js") == AuthPolicy.PUBLIC


@dataclass
class _AuthCase:
    method: str
    path: str
    expected: bool


class _AuthHandler:
    _require_control_auth = _ControlHandler._require_control_auth

    def __init__(self, method: str, path: str):
        self.command = method
        self.path = path
        self.headers = {}
        self.client_address = ("127.0.0.1", 12345)
        self.responses = []
        self.server = SimpleNamespace(
            sessions_root="unused",
            bind_mode="local",
            operator_sessions={},
            sse_tokens={},
            auth_tokens=set(),
        )

    def _send_json(self, status, payload, **_kwargs):
        self.responses.append((status, payload))


@pytest.mark.parametrize(
    "case",
    [
        _AuthCase("GET", "/api/health", True),
        _AuthCase("GET", "/api/server-info", True),
        _AuthCase("POST", "/api/auth/login", True),
        _AuthCase("POST", "/api/auth/reset-pin", True),
        _AuthCase("POST", "/api/auth/host-reset", True),
        _AuthCase("GET", "/api/session/current", False),
        _AuthCase("POST", "/api/operator-profiles", False),
        _AuthCase("GET", "/api/not-a-real-route", False),
    ],
)
def test_registry_drives_handler_authorization_without_schema_drift(case):
    handler = _AuthHandler(case.method, case.path)
    with patch(
        "rvt_trainer.monolith._load_operator_profiles",
        return_value={"profiles": {"op_1": {"operator_id": "op_1"}}},
    ):
        allowed = handler._require_control_auth()

    assert allowed is case.expected
    if case.expected:
        assert handler.responses == []
    else:
        assert handler.responses[-1][0] == 401
        assert handler.responses[-1][1]["error"]["code"] == "UNAUTHORIZED"


def test_operator_and_sse_tokens_keep_existing_authorization_semantics():
    operator = _AuthHandler("POST", "/api/session/stop")
    operator.headers["X-RVT-Auth"] = "operator-token"
    operator.server.operator_sessions["operator-token"] = {
        "operator_id": "op_1",
        "expires_at": 9_999_999_999,
    }

    sse = _AuthHandler("GET", "/api/events/subscribe?token=sse-token")
    sse.server.sse_tokens["sse-token"] = {
        "operator_id": "op_1",
        "expires_at": 9_999_999_999,
    }

    with patch(
        "rvt_trainer.monolith._load_operator_profiles",
        return_value={"profiles": {"op_1": {"operator_id": "op_1"}}},
    ):
        assert operator._require_control_auth() is True
        assert sse._require_control_auth() is True

    assert operator.current_operator_id == "op_1"
    assert "sse-token" not in sse.server.sse_tokens
