"""Machine-readable study objectives from the approved manuscript protocol."""

from __future__ import annotations

from typing import Dict, List

STUDY_OBJECTIVES_SCHEMA_VERSION = "rvt-study-objectives-v16.5.9"

_OBJECTIVES: List[Dict[str, object]] = [
    {
        "id": "objective_1_rr",
        "number": 1,
        "outcome": "rr",
        "label": "GBR-assisted respiration-rate equivalence",
        "role": "confirmatory",
        "primary_condition_id": "d100_none",
        "secondary_condition_count": 5,
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
        "outcome": "temperature",
        "label": "Unobstructed skin-surface-temperature agreement",
        "role": "exploratory",
        "conditions": ["d060_none", "d080_none", "d100_none"],
        "reference": "calibrated infrared thermometer",
        "metrics": ["bias", "rmse", "mae", "limits_of_agreement", "missing_output_rate"],
        "required_routes": ["/api/sessions/*/summary", "/api/sessions/*/data"],
    },
    {
        "id": "objective_3_false_alarm",
        "number": 3,
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
        "outcome": "hr",
        "label": "GBR-assisted heart-rate accuracy and agreement",
        "role": "exploratory",
        "conditions": [
            "d060_none",
            "d080_none",
            "d100_none",
            "d060_cardboard",
            "d080_cardboard",
            "d100_cardboard",
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
    """Return a copy so callers cannot mutate the controlled contract."""

    return {
        "schema_version": STUDY_OBJECTIVES_SCHEMA_VERSION,
        "product_version": "16.5.12",
        "confirmatory_conditions": [
            "d060_none",
            "d080_none",
            "d100_none",
            "d060_cardboard",
            "d080_cardboard",
            "d100_cardboard",
        ],
        "trials_per_condition": 3,
        "planned_duration_s": 150,
        "target_recruited_participants": 40,
        "minimum_protocol_complete_participants": 38,
        "objectives": [dict(objective) for objective in _OBJECTIVES],
    }


__all__ = ["STUDY_OBJECTIVES_SCHEMA_VERSION", "study_objectives_payload"]
