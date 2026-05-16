"""HTTP API helpers for the Radar Vital trainer.

Submodules are not pre-imported here: that would force every submodule to
load before any one of them is usable, which creates circular imports with
:mod:`rvt_trainer.monolith` while the legacy implementation still lives
there. Import the specific submodule you need:

    from rvt_trainer.api import auth, server_info
    from rvt_trainer.api.auth import make_pair_pin, exchange_pair_pin
    from rvt_trainer.api.server_info import manifest_payload
"""
