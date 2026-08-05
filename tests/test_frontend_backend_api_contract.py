"""Keep the Angular API surface in lock-step with the Python route registry."""

from __future__ import annotations

import re
from pathlib import Path

from rvt_trainer.api.route_registry import ROUTES
from rvt_trainer.monolith import _load_subject_profiles, _save_subject_profiles


ROOT = Path(__file__).resolve().parents[1]
FRONTEND_CONTRACT = ROOT / "web" / "src" / "app" / "services" / "backend-api.contract.ts"


def _frontend_routes() -> set[tuple[str, frozenset[str], str]]:
    source = FRONTEND_CONTRACT.read_text(encoding="utf-8")
    pattern = re.compile(
        r"\{ name: '([^']+)', methods: \[([^\]]*)\], path: '([^']+)' \},"
    )
    routes = set()
    for name, methods, path in pattern.findall(source):
        parsed_methods = frozenset(
            item.strip().strip("'")
            for item in methods.split(",")
            if item.strip()
        )
        routes.add((name, parsed_methods, path))
    return routes


def test_every_python_api_route_is_declared_in_the_angular_contract():
    backend = {
        (spec.name, frozenset(spec.methods), spec.pattern)
        for spec in ROUTES
        if spec.pattern.startswith("/api/")
    }
    frontend = _frontend_routes()

    assert frontend == backend


def test_subject_profile_put_round_trips_the_frontend_envelope(tmp_path):
    sessions_root = tmp_path / "sessions"
    sessions_root.mkdir()

    saved = _save_subject_profiles(
        str(sessions_root),
        {"profiles": {"adult_default": {"label": "Adult", "expected_hr_range": [50, 120]}}},
    )

    assert saved["profiles"]["adult_default"]["label"] == "Adult"
    loaded = _load_subject_profiles(str(sessions_root))
    assert loaded["profiles"]["adult_default"]["expected_hr_range"] == [50.0, 120.0]
