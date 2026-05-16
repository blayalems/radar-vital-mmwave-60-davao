"""Static asset routing helpers for the trainer HTTP server.

The trainer serves three public asset trees: ``/icons/*``, ``/lib/*``,
and ``/fonts/*``. These helpers resolve a request path safely (no
``..`` escapes, no leakage into the private ``.rvt_tls/`` directory)
and pick a sensible ``Content-Type`` header.

* :func:`assets_root` — public asset root directory (``<repo>/assets``).
* :func:`safe_asset_path` — resolve ``url_path`` to an absolute file
  path under ``assets_root()``, or ``None`` if the path escapes the
  whitelist (``icons/``, ``lib/``, ``fonts/``) or resolves into the
  private TLS directory.
* :func:`content_type_for_asset` — MIME type for the response.
"""

from __future__ import annotations

import mimetypes
from pathlib import Path
from typing import Optional
from urllib.parse import unquote

_PACKAGE_ROOT = Path(__file__).resolve().parent.parent       # rvt_trainer/
_REPO_ROOT = _PACKAGE_ROOT.parent                            # repo root


def assets_root() -> Path:
    """Return the public assets directory shipped with the repo."""
    return _REPO_ROOT / "assets"


def safe_asset_path(url_path: str) -> Optional[Path]:
    """Resolve ``url_path`` to a real file under :func:`assets_root`.

    Returns ``None`` (caller should send 404) when the path:

    * Doesn't start with one of the public trees (``icons/``, ``lib/``,
      ``fonts/``).
    * Escapes the assets root via ``..`` or absolute paths.
    * Lands inside the private ``.rvt_tls/`` directory.
    * Doesn't point at an existing file.
    """
    root = assets_root().resolve()
    rel = unquote(url_path.lstrip("/")).replace("\\", "/")
    if not (rel.startswith("icons/") or rel.startswith("lib/") or rel.startswith("fonts/")):
        return None
    target = (root / rel).resolve()
    try:
        target.relative_to(root)
    except ValueError:
        return None
    private_root = (_REPO_ROOT / ".rvt_tls").resolve()
    try:
        target.relative_to(private_root)
        return None
    except ValueError:
        pass
    return target if target.is_file() else None


def content_type_for_asset(path: Path) -> str:
    """Pick a ``Content-Type`` for the given asset path."""
    suffix = path.suffix.lower()
    if suffix == ".js":
        return "application/javascript; charset=utf-8"
    if suffix == ".css":
        return "text/css; charset=utf-8"
    if suffix == ".woff2":
        return "font/woff2"
    if suffix == ".png":
        return "image/png"
    return mimetypes.guess_type(str(path))[0] or "application/octet-stream"


__all__ = ["assets_root", "safe_asset_path", "content_type_for_asset"]
