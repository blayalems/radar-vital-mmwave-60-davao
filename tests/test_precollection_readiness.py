import json
from datetime import date
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _manifest():
    return json.loads((ROOT / "quality" / "precollection-readiness.json").read_text(encoding="utf-8"))


def test_precollection_manifest_is_fail_closed_and_dates_every_workstream():
    manifest = _manifest()
    boundary = manifest["authorization_boundary"]
    assert manifest["status"] == "blocked"
    assert manifest["schedule_status"] == "proposed_not_authorized"
    assert boundary["software_merge_authorizes_collection"] is False
    assert boundary["draft_plan_authorizes_collection"] is False
    assert boundary["draft_requirement_gates_software_only"] is True
    assert {gate["role"] for gate in boundary["required_approvals"]} == {
        "research_lead", "quality_manager", "REC_or_ethics_authority", "privacy_reviewer"
    }
    assert all(gate["status"] == "pending" and gate["evidence_ref"] is None for gate in boundary["required_approvals"])
    for stream in manifest["workstreams"]:
        assert stream["owner_role"]
        assert date.fromisoformat(stream["earliest_start_date"]) <= date.fromisoformat(stream["target_date"])
        assert stream["status"] == "blocked"
        assert len(stream["abort_or_replan"]) >= 20


def test_precollection_manifest_records_recruitment_and_capture_arithmetic():
    scale = _manifest()["study_scale"]
    assert scale["target_recruited_participants"] == 40
    assert scale["minimum_protocol_complete_participants"] == 38
    assert scale["minimum_independent_primary_estimates"] == 19
    assert scale["trials_per_protocol_complete_participant"] == 6 * 3 == 18
    assert scale["minimum_protocol_complete_trial_captures"] == 38 * 18 == 684
    assert scale["target_recruited_trial_capacity"] == 40 * 18 == 720
    assert scale["no_subject_minimum_capture_s"] == 72 * 150 == 10800
    assert scale["no_subject_minimum_capture_hours"] == 3


def test_withdrawal_policy_does_not_claim_raw_data_is_append_only_or_erased():
    withdrawal = _manifest()["withdrawal_reconciliation"]
    assert withdrawal["status"] == "blocked_pending_REC_and_privacy_review"
    assert withdrawal["append_only_scope"] == "audit_and_control_metadata_only"
    assert withdrawal["participant_data_is_unconditionally_append_only"] is False
    assert withdrawal["current_crypto_shredding_available"] is False
    assert withdrawal["tombstone_required"] is True
    assert "authority_reference" in withdrawal["required_tombstone_fields"]
    assert "independently verified" in withdrawal["prohibited_claim"]


def test_synchronization_contract_requires_method_uncertainty_and_physical_latency_evidence():
    sync = _manifest()["synchronization"]
    assert sync["serial_contract_change"] is False
    assert sync["anchor_clock"] == "host_monotonic_receive"
    assert sync["required_artifacts"] == ["sync_anchors.json", "alignment_report.json"]
    assert sync["method_must_be_recorded"] is True
    assert sync["uncertainty_must_be_numeric_seconds"] is True
    assert sync["usb_buffering_latency_physical_bench_required"] is True
    assert "uncertainty_model" in sync["sync_anchors_required_fields"]
    assert "latency_characterization_ref" in sync["alignment_report_required_fields"]
    assert sync["objective_sensitivity"]["objective_4_hr"].startswith("higher_timing_sensitivity")


def test_draft_statistical_controls_prepare_software_but_do_not_authorize_collection():
    plan = json.loads((ROOT / "quality" / "statistical-analysis-plan.json").read_text(encoding="utf-8"))
    requirements = json.loads((ROOT / "quality" / "requirements.json").read_text(encoding="utf-8"))
    statistical = next(item for item in requirements["requirements"] if item["requirement_id"] == "RVT-STA-001")
    assert plan["status"] == "draft"
    assert statistical["status"] == "draft"
    assert _manifest()["authorization_boundary"]["draft_requirement_gates_software_only"] is True
