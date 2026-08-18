from types import SimpleNamespace

from rvt_trainer.monolith import _alignment_provenance


def _args(*, auto_align_start=False, tolerance_s=1.0):
    return SimpleNamespace(auto_align_start=auto_align_start, tolerance_s=tolerance_s)


def test_alignment_provenance_records_method_numeric_uncertainty_and_limitations():
    provenance = _alignment_provenance(_args(tolerance_s=0.75), [0.0, 0.25])

    assert provenance["method"] == "operator_manual_offset"
    assert provenance["applied_ref_offset_s"] == [0.0, 0.25]
    assert provenance["uncertainty_s"] == 0.75
    assert provenance["uncertainty_basis"] == (
        "merge_tolerance_upper_bound_not_physical_latency_uncertainty"
    )
    assert provenance["physical_latency_characterization"]["status"] == "missing"
    assert provenance["confirmatory_timing_eligible"] is False
    assert any("Objective 4 HR" in item for item in provenance["limitations"])


def test_first_sample_alignment_is_explicitly_exploratory_not_confirmatory_precision():
    provenance = _alignment_provenance(
        _args(auto_align_start=True, tolerance_s=1.0), [0.0]
    )

    assert provenance["method"] == "first_sample_exploratory"
    assert provenance["confirmatory_timing_eligible"] is False
    assert "not_physical_latency" in provenance["uncertainty_basis"]
