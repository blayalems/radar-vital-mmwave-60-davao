"""Supply-chain CI contract (deferred audit items D9 + D8).

Guards the wiring added for supply-chain hardening so the workflows cannot
silently regress:

  D9  - a non-blocking dependency-audit workflow exists (pip-audit + npm audit),
        and the npm cache key in build-exe.yml includes the web/ lockfile.
  D8  - the Pages "Preserve latest release manifest" step distinguishes a 404
        (manifest not published yet) from transient/network errors, retrying and
        failing hard on persistent failure instead of silently shipping.

Everything is parsed as plain text: PyYAML may be absent in CI, so we never
import it here. Assertions are kept robust to minor formatting (whitespace,
quote style) by normalising before matching.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS = ROOT / ".github" / "workflows"


def _text(path: Path) -> str:
    assert path.exists(), f"expected workflow file missing: {path}"
    return path.read_text(encoding="utf-8")


def _normalise(s: str) -> str:
    """Lowercase and collapse whitespace so matches survive reformatting."""
    return " ".join(s.lower().split())


# --- D9: dependency-audit workflow -------------------------------------------


def test_security_audit_workflow_exists_and_runs_audits():
    text = _text(WORKFLOWS / "security-audit.yml")
    norm = _normalise(text)

    # Python and npm audits must both be present.
    assert "pip-audit" in norm, "security-audit.yml must run pip-audit"
    assert "npm audit" in norm, "security-audit.yml must run npm audit"

    # Both requirements files are audited.
    assert "requirements.txt" in text
    assert "requirements-v12.txt" in text

    # The web/ package tree is audited too.
    assert "npm --prefix web audit" in norm

    # Advisory, not a hard gate.
    assert "continue-on-error: true" in norm

    # Expected triggers and runner.
    assert "workflow_dispatch" in norm
    assert "pull_request" in norm
    assert "ubuntu-latest" in norm


# --- D9: npm cache key includes the web lockfile -----------------------------


def test_build_exe_npm_cache_includes_web_lockfile():
    text = _text(WORKFLOWS / "build-exe.yml")

    assert "cache-dependency-path" in text, (
        "build-exe.yml setup-node must declare cache-dependency-path"
    )
    # The web lockfile must be part of the npm cache key.
    assert "web/package-lock.json" in text, (
        "build-exe.yml npm cache key must include web/package-lock.json"
    )
    # The root lockfile should still be present.
    assert "package-lock.json" in text


# --- D8: resilient manifest preservation -------------------------------------


def test_pages_manifest_step_is_resilient():
    text = _text(WORKFLOWS / "pages.yml")
    norm = _normalise(text)

    # Still targets the same manifest (happy path unchanged).
    assert "rvt-latest.json" in text

    # Captures the HTTP status code rather than collapsing all failures.
    assert "%{http_code}" in text, (
        "pages.yml manifest step must capture the curl http_code"
    )

    # Retries on transient errors.
    assert "retry" in norm or "retries" in norm or "attempt" in norm, (
        "pages.yml manifest step must retry transient failures"
    )

    # Distinguishes a legitimate 404 from other errors.
    assert "404" in text, (
        "pages.yml manifest step must special-case a 404 (not yet published)"
    )

    # Fails hard on persistent failure instead of silently proceeding.
    assert "exit 1" in norm, (
        "pages.yml manifest step must fail (exit 1) on persistent fetch errors"
    )
