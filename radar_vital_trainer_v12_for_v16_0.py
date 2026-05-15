"""Compatibility shim for the Radar Vital v12 trainer.

The implementation lives under :mod:`rvt_trainer` so packaging and tests can
import stable modules without depending on a single 800 KB script. Keep this
file executable for the documented CLI:

    python radar_vital_trainer_v12_for_v16_0.py ...
"""

from rvt_trainer.cli import main


if __name__ == "__main__":
    main()
