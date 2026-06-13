"""Repo-hygiene contract: legal/support artifacts exist with the agreed
content anchors, and the cross-file constants other features rely on stay in
sync (issue-form field ids prefilled by the in-app reporter, storage keys,
authorship metadata, and the smoke-helper TERMS_VERSION mirror)."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
AUTHORS = ["Lemuel Blaya", "Angelo Diaz", "Blessie Mugat"]


def text(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def test_legal_files_exist_with_required_anchors():
    license_text = text("LICENSE")
    for author in AUTHORS:
        assert author in license_text
    assert "University of Mindanao" in license_text
    assert "NOT A MEDICAL DEVICE" in " ".join(license_text.upper().split())

    terms = text("TERMS.md")
    privacy = text("PRIVACY.md")
    for doc in (terms, privacy):
        assert "10173" in doc, "RA 10173 reference required"
        assert "University of Mindanao" in doc
        assert "DRAFT" in doc, "legal drafts must carry the review banner"
    assert "National Privacy Commission" in privacy
    assert "not a medical device" in terms.lower()


def test_changelog_and_contributing_exist():
    changelog = text("CHANGELOG.md")
    assert "[Unreleased]" in changelog
    assert "16.3.0" in changelog
    assert "Verification protocol" in text("CONTRIBUTING.md")


def test_bug_report_form_field_ids_are_stable():
    """The in-app issue reporter prefills these ids via URL params — renaming
    a field id silently breaks prefill, so the ids are contractual.

    Asserted textually (no PyYAML in requirements-v12.txt): GitHub form ids
    appear as `id: <name>` lines, which is stable enough for the contract.
    """
    form = text(".github/ISSUE_TEMPLATE/bug_report.yml")
    ids = set(re.findall(r"^\s*id:\s*(\S+)\s*$", form, flags=re.MULTILINE))
    for required in ("description", "steps", "product_version", "platform",
                     "connection_mode", "diagnostics"):
        assert required in ids, f"bug form lost contractual field id {required!r}"
    assert "name:" in text(".github/ISSUE_TEMPLATE/feature_request.yml")
    assert re.search(r"^blank_issues_enabled:\s*false\s*$",
                     text(".github/ISSUE_TEMPLATE/config.yml"),
                     flags=re.MULTILINE), "blank issues must stay disabled"


def test_storage_keys_and_app_meta_contract():
    keys = text("web/src/app/services/rvt-storage-keys.ts")
    for const in ("CONSENT_KEY = 'rvt-consent-record'",
                  "TUTORIAL_DONE_KEY = 'rvt-tutorial-done'",
                  "DIAGNOSTICS_OPTIN_KEY = 'rvt-diagnostics-optin'"):
        assert const in keys

    meta = text("web/src/app/services/app-meta.ts")
    for author in AUTHORS:
        assert author in meta
    assert "University of Mindanao" in meta
    assert "getFullYear()" in meta, "copyright year must auto-update"


def test_terms_version_mirrors_smoke_helper():
    meta_version = re.search(r"TERMS_VERSION = '([^']+)'", text("web/src/app/services/app-meta.ts"))
    helper_version = re.search(r"TERMS_VERSION = '([^']+)'", text("tests/smoke/helpers/first-run.ts"))
    assert meta_version and helper_version
    assert meta_version.group(1) == helper_version.group(1), (
        "app-meta TERMS_VERSION and the Playwright first-run helper drifted")


def test_wiki_sources_exist():
    wiki = ROOT / "docs" / "wiki"
    pages = {p.name for p in wiki.glob("*.md")}
    for page in ("Home.md", "Install-Windows.md", "Install-Android.md",
                 "Install-PWA.md", "Operator-Guide.md", "Privacy-and-Data.md",
                 "Troubleshooting.md", "Release-Process.md"):
        assert page in pages
    assert all(author in text("docs/wiki/Home.md") for author in AUTHORS)
