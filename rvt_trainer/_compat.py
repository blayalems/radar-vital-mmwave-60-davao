"""Monkeypatch compatibility helpers for the retired monolith facade."""

from __future__ import annotations

import sys
from types import ModuleType
from typing import Optional, TypeVar, cast

T = TypeVar("T")


_UNSET = object()


def monolith_module() -> Optional[ModuleType]:
    """Return ``rvt_trainer.monolith`` if already imported, without importing it."""
    return sys.modules.get("rvt_trainer.monolith")


def monolith_override(name: str, default: T, *, canonical: object = _UNSET) -> T:
    """Resolve *name* from ``rvt_trainer.monolith`` when overridden, else *default*.

    When *canonical* is supplied, a locally patched *default* wins immediately
    even if ``rvt_trainer.monolith`` is already loaded in ``sys.modules``.
    """
    if canonical is not _UNSET and default is not canonical:
        return default
    mod = sys.modules.get("rvt_trainer.monolith")
    if mod is None:
        return default
    candidate = getattr(mod, name, default)
    if canonical is not _UNSET and candidate is canonical:
        return default
    return cast(T, candidate)


__all__ = ["monolith_module", "monolith_override"]
