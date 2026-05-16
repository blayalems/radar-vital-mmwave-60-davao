"""Unit tests for rvt_trainer.assets.static — path-traversal guard,
whitelist enforcement, content-type mapping.

The full HTTP path (Cache-Control headers, status codes) is exercised
by the Playwright smoke suite; these tests are the fast feedback for
the pure-helper logic.
"""

from __future__ import annotations

from pathlib import Path

from rvt_trainer.assets.static import (
    assets_root,
    content_type_for_asset,
    safe_asset_path,
)


def test_assets_root_is_repo_assets_dir():
    root = assets_root()
    assert root.is_dir(), f"{root} is not a directory"
    assert root.name == "assets"
    # sw.js is a known canonical file under the assets root.
    assert (root / "sw.js").is_file()


def test_safe_asset_path_resolves_real_files():
    # These three should all exist in the shipped assets tree.
    assert safe_asset_path("/icons/icon-192.png") is not None
    assert safe_asset_path("/lib/jsqr.min.js") is not None
    # fonts directory may or may not have a stable file name across releases;
    # at minimum the rvt-fonts.css linker should be there.
    assert safe_asset_path("/fonts/rvt-fonts.css") is not None


def test_safe_asset_path_rejects_paths_outside_whitelist():
    # sw.js, manifest, and root files are NOT served via /assets/<x>;
    # the dispatcher handles them separately.
    assert safe_asset_path("/sw.js") is None
    assert safe_asset_path("/manifest.webmanifest") is None
    assert safe_asset_path("/secret/key.pem") is None


def test_safe_asset_path_blocks_traversal():
    assert safe_asset_path("/icons/../../etc/passwd") is None
    assert safe_asset_path("/lib/../../../../etc/passwd") is None
    assert safe_asset_path("/fonts/../../../.rvt_tls/cert.pem") is None


def test_safe_asset_path_blocks_rvt_tls_directly():
    # Even if .rvt_tls/ were exposed via /icons/..., the explicit
    # private-root deny check stops it. Synthesize the path:
    assert safe_asset_path("/icons/../.rvt_tls/cert.pem") is None


def test_safe_asset_path_returns_none_for_missing_files():
    assert safe_asset_path("/icons/this-file-does-not-exist.png") is None
    assert safe_asset_path("/lib/nope.js") is None


def test_content_type_known_extensions():
    assert content_type_for_asset(Path("a.js")) == "application/javascript; charset=utf-8"
    assert content_type_for_asset(Path("b.css")) == "text/css; charset=utf-8"
    assert content_type_for_asset(Path("c.woff2")) == "font/woff2"
    assert content_type_for_asset(Path("d.png")) == "image/png"


def test_content_type_falls_back_to_mimetypes_or_octet_stream():
    # html is in mimetypes by default → text/html
    ct = content_type_for_asset(Path("e.html"))
    assert "text/html" in ct
    # Unknown extension → octet-stream.
    assert content_type_for_asset(Path("z.weird")) == "application/octet-stream"


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__, "-v"]))
