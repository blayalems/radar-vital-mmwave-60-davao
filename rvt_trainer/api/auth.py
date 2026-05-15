"""Pairing and token helpers.

These exports keep authentication code discoverable while the v12 trainer is
being decomposed from the legacy monolith.
"""

from ..monolith import _exchange_pair_pin as exchange_pair_pin
from ..monolith import _make_pair_pin as make_pair_pin

__all__ = ["exchange_pair_pin", "make_pair_pin"]
