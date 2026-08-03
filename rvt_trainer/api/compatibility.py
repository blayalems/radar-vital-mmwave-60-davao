"""Cross-release compatibility decisions for control API clients and hardware.

The dashboard and trainer are normally released together, but cached PWAs and
long-running native shells can retain an older dashboard.  This module keeps
the compatibility comparison independent from the HTTP handler so the same
stable decision/reason contract can be used by status, version, and session
start paths.
"""

from __future__ import annotations

from typing import Dict, Mapping, Optional


SERIAL_PROTOCOL_VERSION = "v15.2"
SERIAL_WIDTH_EXPECTED = 222
COMPATIBILITY_SCHEMA_VERSION = "rvt-release-compatibility-v1"

_CLIENT_FIELDS = {
    "product_version": ("product_version", "PRODUCT_VERSION_MISMATCH"),
    "dashboard_version": ("dashboard_version", "DASHBOARD_VERSION_MISMATCH"),
    "serial_protocol": ("serial_protocol", "SERIAL_PROTOCOL_MISMATCH"),
    "serial_width_expected": ("serial_width_expected", "SERIAL_WIDTH_MISMATCH"),
    "control_api_schema": ("control_api_schema", "CONTROL_API_SCHEMA_MISMATCH"),
    "study_session_schema": (
        "study_session_schema",
        "STUDY_SESSION_SCHEMA_MISMATCH",
    ),
    "session_manifest_schema": (
        "session_manifest_schema",
        "SESSION_MANIFEST_SCHEMA_MISMATCH",
    ),
}
_REQUIRED_CLIENT_FIELDS = frozenset(
    {
        "product_version",
        "dashboard_version",
        "serial_protocol",
        "serial_width_expected",
        "control_api_schema",
        "study_session_schema",
    }
)


def _reason(
    code: str,
    message: str,
    *,
    expected: object = None,
    observed: object = None,
    remediation: str = "",
) -> Dict[str, object]:
    item: Dict[str, object] = {"code": code, "message": message}
    if expected is not None:
        item["expected"] = expected
    if observed is not None:
        item["observed"] = observed
    if remediation:
        item["remediation"] = remediation
    return item


def server_compatibility_decision(
    *,
    firmware_expected: str,
    firmware_observed: object = None,
    serial_width_expected: int = SERIAL_WIDTH_EXPECTED,
    serial_width_observed: object = None,
) -> Dict[str, object]:
    """Compare observed hardware identity with this trainer's frozen contract."""

    reasons = []
    if firmware_observed in (None, "", "unknown"):
        reasons.append(
            _reason(
                "FIRMWARE_NOT_OBSERVED",
                "Firmware identity is not available until hardware telemetry is observed.",
                expected=firmware_expected,
                remediation="Run hardware preflight or start a verified capture.",
            )
        )
    elif str(firmware_observed) != str(firmware_expected):
        reasons.append(
            _reason(
                "FIRMWARE_VERSION_MISMATCH",
                "Observed firmware does not match this trainer release.",
                expected=firmware_expected,
                observed=firmware_observed,
                remediation="Flash the expected firmware before confirmatory capture.",
            )
        )

    if serial_width_observed not in (None, "", "unknown"):
        try:
            observed_width = int(serial_width_observed)
        except (TypeError, ValueError):
            observed_width = serial_width_observed
        if observed_width != int(serial_width_expected):
            reasons.append(
                _reason(
                    "SERIAL_WIDTH_MISMATCH",
                    "Observed serial row width does not match the active capture contract.",
                    expected=int(serial_width_expected),
                    observed=observed_width,
                    remediation="Use v15.2/222-column firmware for new captures.",
                )
            )

    incompatible = any(
        item["code"] in {"FIRMWARE_VERSION_MISMATCH", "SERIAL_WIDTH_MISMATCH"}
        for item in reasons
    )
    decision = "incompatible" if incompatible else ("unverified" if reasons else "compatible")
    return {
        "schema_version": COMPATIBILITY_SCHEMA_VERSION,
        "decision": decision,
        "verified": decision == "compatible",
        "blocks_start": decision == "incompatible",
        "confirmatory_eligible": decision == "compatible",
        "reasons": reasons,
    }


def build_compatibility_handshake(
    *,
    product_version: str,
    trainer_version: str,
    dashboard_version: str,
    firmware_expected: str,
    schema_versions: Mapping[str, object],
    source_commit: object = None,
    firmware_observed: object = None,
    serial_protocol: str = SERIAL_PROTOCOL_VERSION,
    serial_width_expected: int = SERIAL_WIDTH_EXPECTED,
    serial_width_observed: object = None,
) -> Dict[str, object]:
    """Return the structured server/hardware release handshake."""

    identity = {
        "product_version": product_version,
        "trainer_version": trainer_version,
        "dashboard_version": dashboard_version,
        "firmware_expected": firmware_expected,
        "firmware_observed": firmware_observed,
        "serial_protocol": serial_protocol,
        "serial_width_expected": int(serial_width_expected),
        "serial_width_observed": serial_width_observed,
        "source_commit": source_commit,
    }
    return {
        "schema_version": COMPATIBILITY_SCHEMA_VERSION,
        "identity": identity,
        "schema_versions": dict(schema_versions),
        "compatibility": server_compatibility_decision(
            firmware_expected=firmware_expected,
            firmware_observed=firmware_observed,
            serial_width_expected=serial_width_expected,
            serial_width_observed=serial_width_observed,
        ),
    }


def _normalise_client_metadata(payload: Mapping[str, object]) -> Optional[Dict[str, object]]:
    client = payload.get("client_handshake")
    if not isinstance(client, Mapping):
        client = payload.get("client")
    if not isinstance(client, Mapping):
        client = payload.get("client_compatibility")
    if not isinstance(client, Mapping):
        return None
    out = dict(client)
    schema_versions = out.get("schema_versions")
    if isinstance(schema_versions, Mapping):
        out.setdefault("control_api_schema", schema_versions.get("control_api"))
        out.setdefault("study_session_schema", schema_versions.get("study_session"))
        out.setdefault(
            "session_manifest_schema",
            schema_versions.get("session_manifest"),
        )
    if "serial_width_expected" not in out and "serial_width" in out:
        out["serial_width_expected"] = out.get("serial_width")
    return out


def validate_client_compatibility(
    payload: Mapping[str, object],
    *,
    product_version: str,
    dashboard_version: str,
    control_api_schema: str,
    study_session_schema: str,
    session_manifest_schema: str,
    serial_protocol: str = SERIAL_PROTOCOL_VERSION,
    serial_width_expected: int = SERIAL_WIDTH_EXPECTED,
) -> Dict[str, object]:
    """Validate optional start-request metadata without breaking legacy clients."""

    expected = {
        "product_version": product_version,
        "dashboard_version": dashboard_version,
        "serial_protocol": serial_protocol,
        "serial_width_expected": int(serial_width_expected),
        "control_api_schema": control_api_schema,
        "study_session_schema": study_session_schema,
        "session_manifest_schema": session_manifest_schema,
    }
    raw_client = payload.get("client_handshake")
    if not isinstance(raw_client, Mapping):
        raw_client = payload.get("client")
    if not isinstance(raw_client, Mapping):
        raw_client = payload.get("client_compatibility")
    client = _normalise_client_metadata(payload)
    if client is None:
        return {
            "schema_version": COMPATIBILITY_SCHEMA_VERSION,
            "decision": "unverified",
            "verified": False,
            "blocks_start": False,
            "confirmatory_eligible": False,
            "client_handshake": None,
            "client_metadata": None,
            "reasons": [
                _reason(
                    "CLIENT_METADATA_MISSING",
                    "Legacy client supplied no release compatibility metadata.",
                    remediation="Reload or restart the dashboard before confirmatory capture.",
                )
            ],
        }

    reasons = []
    present = []
    for field, (_, reason_code) in _CLIENT_FIELDS.items():
        if field not in client or client.get(field) in (None, ""):
            continue
        present.append(field)
        observed = client.get(field)
        expected_value = expected[field]
        if field == "serial_width_expected":
            try:
                observed = int(observed)
            except (TypeError, ValueError):
                pass
        if observed != expected_value:
            reasons.append(
                _reason(
                    reason_code,
                    f"Client {field} does not match the running trainer.",
                    expected=expected_value,
                    observed=observed,
                    remediation="Reload the PWA or restart the native app and trainer, then retry.",
                )
            )

    if reasons:
        decision = "incompatible"
    elif not _REQUIRED_CLIENT_FIELDS.issubset(present):
        decision = "unverified"
        missing = sorted(_REQUIRED_CLIENT_FIELDS - set(present))
        reasons.append(
            _reason(
                "CLIENT_METADATA_INCOMPLETE",
                "Client release metadata is incomplete.",
                observed={"present": sorted(present), "missing": missing},
                remediation="Reload or restart the dashboard to send the complete release handshake.",
            )
        )
    else:
        decision = "compatible"

    return {
        "schema_version": COMPATIBILITY_SCHEMA_VERSION,
        "decision": decision,
        "verified": decision == "compatible",
        "blocks_start": decision == "incompatible",
        "confirmatory_eligible": decision == "compatible",
        "client_handshake": dict(raw_client) if isinstance(raw_client, Mapping) else None,
        "client_metadata": client,
        "expected": expected,
        "reasons": reasons,
    }


__all__ = [
    "COMPATIBILITY_SCHEMA_VERSION",
    "SERIAL_PROTOCOL_VERSION",
    "SERIAL_WIDTH_EXPECTED",
    "build_compatibility_handshake",
    "server_compatibility_decision",
    "validate_client_compatibility",
]
