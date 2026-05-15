"""Static asset helpers for the trainer HTTP server."""

from ..monolith import _assets_root as assets_root
from ..monolith import _content_type_for_asset as content_type_for_asset
from ..monolith import _safe_asset_path as safe_asset_path

__all__ = ["assets_root", "content_type_for_asset", "safe_asset_path"]
