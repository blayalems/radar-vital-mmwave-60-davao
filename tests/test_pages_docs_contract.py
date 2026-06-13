"""Static contract checks for the GitHub Pages legal URLs and wiki sources."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WIKI = ROOT / "docs" / "wiki"


def text(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def test_pages_workflow_publishes_stable_legal_urls():
    workflow = text(".github/workflows/pages.yml")

    assert "cp docs/privacy.html www/privacy.html" in workflow
    assert "cp docs/terms.html www/terms.html" in workflow
    assert "test -f www/privacy.html" in workflow
    assert "test -f www/terms.html" in workflow
    assert "grep -q 'Radar Vital Privacy Notice' www/privacy.html" in workflow
    assert "grep -q 'Radar Vital Terms and Conditions' www/terms.html" in workflow
    assert "grep -q 'radar-vital-mmwave-60-davao/privacy.html' www/privacy.html" in workflow
    assert "grep -q 'radar-vital-mmwave-60-davao/terms.html' www/terms.html" in workflow
    assert "'docs/privacy.html'" in workflow
    assert "'docs/terms.html'" in workflow


def test_static_legal_pages_match_draft_contract():
    privacy = text("docs/privacy.html")
    terms = text("docs/terms.html")

    assert "Radar Vital Privacy Notice" in privacy
    assert "Radar Vital Terms and Conditions" in terms
    assert "DRAFT 2026-06-12.1" in privacy
    assert "DRAFT 2026-06-12.1" in terms
    assert "Republic Act No. 10173" in privacy
    assert "Republic Act No. 10173" in terms
    assert "has not yet been counsel-verified" in privacy
    assert "has not yet been counsel-verified" in terms
    assert "https://blayalems.github.io/radar-vital-mmwave-60-davao/privacy.html" in privacy
    assert "https://blayalems.github.io/radar-vital-mmwave-60-davao/terms.html" in terms
    for author in ("Lemuel Blaya", "Angelo Diaz", "Blessie Mugat"):
        assert author in privacy
        assert author in terms


def test_wiki_sources_are_filled_not_stubs():
    pages = sorted(WIKI.glob("*.md"))
    assert pages
    for page in pages:
        body = page.read_text(encoding="utf-8")
        assert "Stub" not in body
        assert "content filled in the Wave 2" not in body
        assert len(body.split()) >= 80, f"{page.name} is too thin for operator docs"


def test_milestones_track_release_blockers_honestly():
    body = text("docs/milestones.md")

    for milestone in (
        "v16.3.0-rc",
        "v16.3.0 RTM",
        "Play closed testing",
        "Signing identity",
        "Legal review sign-off",
        "v16.4 backlog",
    ):
        assert milestone in body
    assert "12 testers" in body
    assert "14 days" in body
    assert "RA 10173" in body
    assert "draft" in body.lower()
    assert "SmartScreen" in body
