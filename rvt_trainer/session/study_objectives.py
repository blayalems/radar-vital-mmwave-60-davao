"""Machine-readable study objectives from the approved manuscript protocol.

The approved September 2026 manuscript (``docs/manuscript/manuscript.tex``)
states four specific thesis objectives: design, build, integrate, and test.
The analysis endpoints below keep their stable historical identifiers
(``objective_1_rr`` ... ``objective_4_hr``) because routes, analysis jobs, and
reports are keyed by them; each endpoint is explicitly mapped to thesis
Objective 4 (test) and manuscript Section II-F (Functionality testing) so the
endpoint ``number`` is never confused with the thesis objective number.
"""

from __future__ import annotations

import copy
from typing import Dict, List

STUDY_OBJECTIVES_SCHEMA_VERSION = "rvt-study-objectives-v16.6.5"

MANUSCRIPT_IDENTITY: Dict[str, object] = {
    "path": "docs/manuscript/manuscript.tex",
    "document_id": "RVT-QMS-MAN-001",
    "approval_state": "approved_manuscript_2026_09",
    "testing_section": "II-F",
    "testing_heading": "Functionality testing",
}

_THESIS_OBJECTIVES: List[Dict[str, object]] = [
    {
        "number": 1,
        "phase": "design",
        "statement": (
            "Design a non-contact vital signs monitoring system that uses 60-GHz millimeter-wave "
            "sensing and an ESP32 microcontroller to detect respiratory rate, heart rate, and body temperature."
        ),
        "requirement_id": "RVT-THESIS-001",
        "manuscript_sections": ["I", "II-C", "II-D"],
    },
    {
        "number": 2,
        "phase": "build",
        "statement": (
            "Build the individual hardware components of the system, including the millimeter-wave "
            "sensing module, ESP32-based processing unit, and user-interface module."
        ),
        "requirement_id": "RVT-THESIS-002",
        "manuscript_sections": ["I", "II-D", "II-E", "II-F"],
    },
    {
        "number": 3,
        "phase": "integrate",
        "statement": (
            "Integrate the built components into a single functional prototype capable of continuous "
            "non-contact vital signs monitoring."
        ),
        "requirement_id": "RVT-THESIS-003",
        "manuscript_sections": ["I", "II-F"],
    },
    {
        "number": 4,
        "phase": "test",
        "statement": (
            "Test the performance of the integrated prototype in detecting respiratory rate, heart rate, "
            "and body temperature by comparing its readings against a reference measurement device under "
            "controlled test conditions."
        ),
        "requirement_id": "RVT-THESIS-004",
        "manuscript_sections": ["I", "II-F"],
        "analysis_endpoint_ids": [
            "objective_1_rr",
            "objective_2_temperature",
            "objective_3_false_alarm",
            "objective_4_hr",
        ],
    },
]

_OBJECTIVES: List[Dict[str, object]] = [
    {
        "id": "objective_1_rr",
        "number": 1,
        "thesis_objective": 4,
        "manuscript_section": "II-F",
        "outcome": "rr",
        "label": "GBR-assisted respiration-rate equivalence",
        "role": "confirmatory",
        "primary_condition_id": "d100_none",
        "secondary_condition_count": 9,
        "equivalence_margin_bpm": 2.0,
        "confidence_level": 0.90,
        "minimum_independent_estimates": 19,
        "reference": "adjudicated dual-observer full-trial count",
        "required_routes": [
            "/api/session/start",
            "/api/study/completion-matrix",
            "/api/sessions/*/summary",
            "/api/sessions/*/data",
        ],
    },
    {
        "id": "objective_2_temperature",
        "number": 2,
        "thesis_objective": 4,
        "manuscript_section": "II-F",
        "outcome": "temperature",
        "label": "Unobstructed skin-surface-temperature agreement",
        "role": "exploratory",
        "conditions": ["d060_none"],
        "reference": "calibrated infrared thermometer",
        "metrics": ["bias", "rmse", "mae", "limits_of_agreement", "missing_output_rate"],
        "required_routes": ["/api/sessions/*/summary", "/api/sessions/*/data"],
    },
    {
        "id": "objective_3_false_alarm",
        "number": 3,
        "thesis_objective": 4,
        "manuscript_section": "II-F",
        "outcome": "false_alarm",
        "label": "No-subject false-alarm rate",
        "role": "confirmatory",
        "threshold": 0.05,
        "trial_count": 72,
        "trial_duration_s": 150,
        "reference": "no-subject alert state",
        "required_routes": ["/api/study/attempts", "/api/study/completion-matrix", "/api/sessions/*/summary"],
    },
    {
        "id": "objective_4_hr",
        "number": 4,
        "thesis_objective": 4,
        "manuscript_section": "II-F",
        "outcome": "hr",
        "label": "GBR-assisted heart-rate accuracy and agreement",
        "role": "exploratory",
        "conditions": [
            "d060_none",
            "d080_none",
            "d100_none",
            "d200_none",
            "d300_none",
            "d060_cardboard",
            "d080_cardboard",
            "d100_cardboard",
            "d200_cardboard",
            "d300_cardboard",
        ],
        "reference": "pulse-oximeter HR",
        "metrics": ["rmse", "mae", "bias", "limits_of_agreement", "valid_output_rate"],
        "required_routes": [
            "/api/sessions/*/training/status",
            "/api/sessions/*/predict",
            "/api/sessions/*/summary",
            "/api/sessions/*/data",
        ],
    },
]


def study_objectives_payload() -> Dict[str, object]:
    """Return a deep copy so callers cannot mutate the controlled contract.

    ``control_status`` remains ``draft_not_authorized``: manuscript approval
    does not authorize recruitment or confirmatory collection, which stay gated
    on UMERC certification and the pre-collection readiness record.
    """

    return {
        "schema_version": STUDY_OBJECTIVES_SCHEMA_VERSION,
        "control_status": "draft_not_authorized",
        "product_version": "16.9.0",
        "manuscript": copy.deepcopy(MANUSCRIPT_IDENTITY),
        "thesis_objectives": copy.deepcopy(_THESIS_OBJECTIVES),
        "confirmatory_conditions": [
            "d060_none",
            "d080_none",
            "d100_none",
            "d200_none",
            "d300_none",
            "d060_cardboard",
            "d080_cardboard",
            "d100_cardboard",
            "d200_cardboard",
            "d300_cardboard",
        ],
        "trials_per_condition": 3,
        "planned_duration_s": 150,
        "target_recruited_participants": 40,
        "minimum_protocol_complete_participants": 38,
        "objectives": copy.deepcopy(_OBJECTIVES),
    }


__all__ = ["MANUSCRIPT_IDENTITY", "STUDY_OBJECTIVES_SCHEMA_VERSION", "study_objectives_payload"]
