"""Android Play groundwork contract tests.

Asserts that the patch script, icon asset, and Play store docs meet the
requirements introduced in the WS1-E wave (SDK 35 pin, adaptive icon,
store/data-safety docs).
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def text(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# patch-android-shell.mjs
# ---------------------------------------------------------------------------

def test_patch_script_pins_target_sdk_35():
    """patch-android-shell.mjs must reference targetSdk 35."""
    src = text("scripts/patch-android-shell.mjs")
    assert "targetSdk" in src and "35" in src, (
        "patch-android-shell.mjs must pin targetSdk to 35"
    )


def test_patch_script_pins_compile_sdk_35():
    """patch-android-shell.mjs must reference compileSdk 35."""
    src = text("scripts/patch-android-shell.mjs")
    assert "compileSdk" in src and "35" in src, (
        "patch-android-shell.mjs must pin compileSdk to 35"
    )


def test_patch_script_contains_monochrome():
    """patch-android-shell.mjs must include a monochrome adaptive-icon layer."""
    src = text("scripts/patch-android-shell.mjs")
    assert "monochrome" in src, (
        "patch-android-shell.mjs must define a monochrome layer for Android 13 themed icons"
    )


def test_patch_script_fails_loudly_on_missing_sdk_pattern():
    """pinSdkVersions must call process.exit(1) when the gradle pattern is missing."""
    src = text("scripts/patch-android-shell.mjs")
    # The loud-failure branch must be present
    assert "process.exit(1)" in src, (
        "patch-android-shell.mjs must call process.exit(1) on missing SDK pattern"
    )
    assert "FATAL" in src, (
        "patch-android-shell.mjs must emit a FATAL message when SDK pattern is absent"
    )


# ---------------------------------------------------------------------------
# Adaptive icon foreground drawable
# ---------------------------------------------------------------------------

def test_foreground_drawable_exists():
    """assets/icons/android/ic_launcher_foreground.xml must exist."""
    p = ROOT / "assets" / "icons" / "android" / "ic_launcher_foreground.xml"
    assert p.exists(), f"Missing: {p.relative_to(ROOT)}"


def test_foreground_drawable_is_vector():
    """The foreground drawable must be a <vector> element."""
    src = text("assets/icons/android/ic_launcher_foreground.xml")
    assert "<vector" in src, (
        "ic_launcher_foreground.xml must contain a <vector> root element"
    )


def test_foreground_drawable_viewport_108():
    """The foreground drawable viewport must be 108 × 108 dp (adaptive icon spec)."""
    src = text("assets/icons/android/ic_launcher_foreground.xml")
    assert 'android:viewportWidth="108"' in src, (
        "ic_launcher_foreground.xml must declare android:viewportWidth=\"108\""
    )
    assert 'android:viewportHeight="108"' in src, (
        "ic_launcher_foreground.xml must declare android:viewportHeight=\"108\""
    )


# ---------------------------------------------------------------------------
# Play store docs
# ---------------------------------------------------------------------------

def test_store_listing_exists():
    """docs/play/store-listing.md must exist."""
    p = ROOT / "docs" / "play" / "store-listing.md"
    assert p.exists(), f"Missing: {p.relative_to(ROOT)}"


def test_data_safety_exists():
    """docs/play/data-safety.md must exist."""
    p = ROOT / "docs" / "play" / "data-safety.md"
    assert p.exists(), f"Missing: {p.relative_to(ROOT)}"


def test_data_safety_mentions_locally():
    """data-safety.md must state that data is stored locally (mirrors PRIVACY.md § 2-3)."""
    src = text("docs/play/data-safety.md")
    assert "locally" in src.lower(), (
        "docs/play/data-safety.md must mention that data is stored locally"
    )


def test_data_safety_mirrors_privacy_no_third_party():
    """data-safety.md must confirm no third-party sharing."""
    src = text("docs/play/data-safety.md")
    assert "No" in src or "no" in src, (
        "docs/play/data-safety.md must state no data is shared with third parties"
    )
    # The word "third" must appear (in the context of third-party sharing)
    assert "third" in src.lower(), (
        "docs/play/data-safety.md must address third-party data sharing"
    )


def test_store_listing_has_app_name():
    """store-listing.md must contain the app name 'Radar Vital'."""
    src = text("docs/play/store-listing.md")
    assert "Radar Vital" in src, (
        "docs/play/store-listing.md must contain the app name 'Radar Vital'"
    )


def test_store_listing_references_privacy_url():
    """store-listing.md must contain the privacy policy URL."""
    src = text("docs/play/store-listing.md")
    assert "blayalems.github.io/radar-vital-mmwave-60-davao/privacy" in src, (
        "docs/play/store-listing.md must reference the privacy policy URL"
    )


def test_store_listing_references_university():
    """store-listing.md must reference University of Mindanao."""
    src = text("docs/play/store-listing.md")
    assert "University of Mindanao" in src, (
        "docs/play/store-listing.md must reference University of Mindanao"
    )
