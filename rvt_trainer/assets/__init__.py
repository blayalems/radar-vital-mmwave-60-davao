"""Static asset serving helpers.

Submodules are not pre-imported here to keep the package init free of
circular dependencies on :mod:`rvt_trainer.monolith` while the
implementation transitions. Import the submodule you need:

    from rvt_trainer.assets import static
    from rvt_trainer.assets.static import safe_asset_path
"""
