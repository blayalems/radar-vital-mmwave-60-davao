"""Control HTTP server, request dispatcher, TLS/QR helpers, and ``serve`` command."""

from __future__ import annotations

import base64
import json
import mimetypes
import os
import re
import secrets
import shutil
import ssl
import sys
import threading
import time
import urllib.request
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple, cast
from urllib.parse import parse_qs, unquote, urlparse

from rvt_trainer._compat import monolith_override
from rvt_trainer.api.auth import (
    _OPERATOR_LOCK,
    create_operator_profile as _create_operator_profile,
    exchange_pair_pin as _exchange_pair_pin,
    host_reset_pin as _host_reset_pin,
    load_operator_profiles as _load_operator_profiles,
    login_operator as _login_operator,
    make_pair_pin as _make_pair_pin,
    reset_pin_with_recovery as _reset_pin_with_recovery,
    _invalidate_operator_sse_tokens,
)
from rvt_trainer.api.common import (
    api_error as _api_error,
    atomic_write_json as save_json,
    json_safe_response as _json_safe_response,
    read_json_if_exists as _read_json_if_exists,
)
from rvt_trainer.api.compatibility import (
    build_compatibility_handshake as _build_compatibility_handshake,
    validate_client_compatibility as _validate_client_compatibility,
)
from rvt_trainer.api.route_registry import (
    AuthPolicy as _RouteAuthPolicy,
    authorization_for as _route_authorization_for,
    match_route as _match_route,
)
from rvt_trainer.api.server_info import (
    _guess_lan_ip,
    advertised_origin as _advertised_origin,
    manifest_payload as _manifest_payload,
    pair_page_html as _pair_page_html,
    support_matrix_html as _support_matrix_html,
)
from rvt_trainer.assets.static import (
    assets_root as _assets_root,
    content_type_for_asset as _content_type_for_asset,
    safe_asset_path as _safe_asset_path,
)
from rvt_trainer.audit.runner import (
    SESSION_START_PREFLIGHT_IDS,
    _session_start_blocking_failures,
    run_preflight_all as _run_preflight_all,
    run_preflight_check as _run_preflight_check,
)
from rvt_trainer.constants import (
    CHART_ANNOTATIONS_SCHEMA_VERSION,
    CONTROL_API_SCHEMA_VERSION,
    DASHBOARD_VERSION,
    DEFAULT_BLE_ADDRESS,
    DEFAULT_RADAR_PORT,
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    FEATURE_FLAGS,
    FIRMWARE_VERSION_EXPECTED,
    HR_GATE_REASON_NAMES,
    HR_PUBLISH_REASON_NAMES,
    HR_RAW_DISAGREE_SUBREASON_NAMES,
    LIVE_EVENT_SCHEMA_VERSION,
    MAX_JSON_BODY_BYTES,
    PUBLISH_BLOCK_STAGE_NAMES,
    RR_GATE_REASON_NAMES,
    RR_PUBLISH_REASON_NAMES,
    RR_SOURCE_NAMES,
    RR_SOURCE_REJECT_REASON_NAMES,
    SESSION_MANIFEST_SCHEMA_VERSION,
    SESSION_MANIFEST_VERSION,
    SESSION_NOTES_SCHEMA_VERSION,
    SESSION_SIGNOFF_SCHEMA_VERSION,
    SUBJECT_PROFILE_SCHEMA_VERSION,
    TRAINING_PROGRESS_SCHEMA_VERSION,
    UPDATE_MANIFEST_URL,
    VERSION,
    _PACKAGE_ROOT,
    _REPO_ROOT,
    _TRAINER_LOG,
    _append_trainer_log,
    _cleanup_json_temp_files,
    _iso_now,
    _pid_alive,
    _sanitize_user_string,
    _schema_wrap,
    _yellow,
    warn,
)
from rvt_trainer.session.analysis_jobs import (
    _STUDY_ANALYSIS_LOCK,
    _STUDY_ANALYSIS_PROCS,
    _analysis_job_status,
    _rerun_session_analysis,
    _start_study_analysis_job,
)
from rvt_trainer.session.collection_authorization import (
    CollectionAuthorizationError,
    collection_authorization_status as _collection_authorization_status,
    require_collection_authorization as _require_collection_authorization,
)
from rvt_trainer.session.live_dashboard import (
    _DASHBOARD_HTML_FALLBACK_NAMES,
    _DASHBOARD_HTML_NAME,
    _dashboard_html_template_path,
    _idle_live_payload,
    _load_dashboard_template_text,
    _mock_live_payload,
)
from rvt_trainer.session.protocol_ledger import (
    completion_matrix as _completion_matrix,
    register_protocol_attempt as _register_protocol_attempt,
)
from rvt_trainer.session.start_idempotency import StartIdempotencyError
from rvt_trainer.session.storage import (
    _build_report_export_html,
    _compare_session_payload,
    _latest_live_dashboard_payload,
    _load_session_annotations_payload,
    _load_session_summary,
    _load_subject_profiles,
    _save_subject_profiles,
    _scan_sessions_root,
    _session_data_payload,
    _session_path,
    _upsert_session_annotation,
)
from rvt_trainer.session.study_contract import (
    PARTICIPANT_REGISTRY_SCHEMA_VERSION,
    STUDY_SESSION_SCHEMA_VERSION,
    StudyContractError,
    create_participant_profile as _create_participant_profile,
    load_participant_registry as _load_participant_registry,
    source_commit as _source_commit,
    update_participant_status as _update_participant_status,
    validate_study_assignment as _validate_study_assignment,
)
from rvt_trainer.session.study_evidence import (
    adjudicate_rr as _adjudicate_rr,
    append_reference as _append_reference,
    cancel_analysis_job as _cancel_analysis_job,
    create_analysis_job as _create_analysis_job,
    list_analysis_jobs as _list_analysis_jobs,
    load_analysis_job as _load_analysis_job,
    load_protocol as _load_study_protocol,
    load_references as _load_references,
    objective_report as _objective_report,
    request_analysis_cancel as _request_analysis_cancel,
    save_protocol as _save_study_protocol,
    schedule_for_participant as _schedule_for_participant,
)
from rvt_trainer.session.study_objectives import study_objectives_payload as _study_objectives_payload
from rvt_trainer.session.supervisor import (
    SessionStartPreflightError,
    SessionSupervisor as _SessionSupervisor,
    _lock_path,
)
from rvt_trainer.transport.ble import scan_ble_devices_payload as _scan_ble_devices_payload
from rvt_trainer.transport.serial import (
    _serial_ports_payload,
    auto_detect_radar_port as _auto_detect_radar_port,
)

_manifest_cache = {"data": None, "ts": 0}
_manifest_cache_lock = threading.Lock()
_RATE_LIMIT: Dict[str, Tuple[float, float]] = {}
_RATE_LIMIT_LOCK = threading.Lock()
_RATE_LIMIT_MAX = 1024


def _process_memory_mb() -> float:
    try:
        import psutil
        return float(psutil.Process(os.getpid()).memory_info().rss / (1024 * 1024))
    except Exception:
        try:
            import resource
            getrusage = getattr(resource, "getrusage", None)
            rusage_self = getattr(resource, "RUSAGE_SELF", None)
            if getrusage is None or rusage_self is None:
                raise AttributeError("resource.getrusage unavailable")
            rss = float(getrusage(rusage_self).ru_maxrss)
            return rss / 1024.0 if rss > 1024 * 10 else rss
        except Exception:
            return float("nan")


def _system_metrics(path: str) -> Dict[str, object]:
    usage = shutil.disk_usage(path)
    metrics: Dict[str, object] = {
        "memory_mb": _process_memory_mb(),
        "disk_free_mb": float(usage.free / (1024 * 1024)),
    }
    try:
        import psutil
        metrics["cpu_percent"] = float(psutil.cpu_percent(interval=0.0))
    except Exception:
        try:
            getloadavg = getattr(os, "getloadavg", None)
            if getloadavg is None:
                raise AttributeError("os.getloadavg unavailable")
            metrics["cpu_percent"] = float(getloadavg()[0])
        except Exception:
            metrics["cpu_percent"] = None
    return metrics


def _server_scheme(server) -> str:
    return "https" if bool(getattr(server, "tls_enabled", False)) else "http"


def _qr_png_bytes(text: str, scale: int = 8, border: int = 4) -> bytes:
    """Return a small QR-L PNG for pairing URLs without third-party dependencies."""
    import binascii
    import struct
    import zlib

    data = text.encode("utf-8")
    specs = {
        1: (21, 19, 7, []),
        2: (25, 34, 10, [6, 18]),
        3: (29, 55, 15, [6, 22]),
        4: (33, 80, 20, [6, 26]),
    }
    version = next((v for v, (_, data_cw, _, _) in specs.items() if len(data) + 3 <= data_cw), 4)
    size, data_cw, ecc_cw, align_centers = specs[version]

    bits: List[int] = []

    def append_bits(value: int, width: int):
        for shift in range(width - 1, -1, -1):
            bits.append((value >> shift) & 1)

    append_bits(0x4, 4)  # byte mode
    append_bits(len(data), 8)
    for b in data:
        append_bits(b, 8)
    append_bits(0, min(4, data_cw * 8 - len(bits)))
    while len(bits) % 8:
        bits.append(0)
    codewords: List[int] = []
    for i in range(0, len(bits), 8):
        value = 0
        for bit in bits[i:i + 8]:
            value = (value << 1) | bit
        codewords.append(value)
    for pad in (0xEC, 0x11) * data_cw:
        if len(codewords) >= data_cw:
            break
        codewords.append(pad)

    exp = [0] * 512
    log = [0] * 256
    x = 1
    for i in range(255):
        exp[i] = x
        log[x] = i
        x <<= 1
        if x & 0x100:
            x ^= 0x11D
    for i in range(255, 512):
        exp[i] = exp[i - 255]

    def gf_mul(a: int, b: int) -> int:
        return 0 if a == 0 or b == 0 else exp[log[a] + log[b]]

    gen = [0] * (ecc_cw - 1) + [1]
    root = 1
    for _ in range(ecc_cw):
        for j in range(ecc_cw):
            gen[j] = gf_mul(gen[j], root)
            if j + 1 < ecc_cw:
                gen[j] ^= gen[j + 1]
        root = gf_mul(root, 2)
    rem = [0] * ecc_cw
    for b in codewords:
        factor = b ^ rem[0]
        rem = rem[1:] + [0]
        for i, coef in enumerate(gen):
            rem[i] ^= gf_mul(coef, factor)
    all_cw = codewords + rem

    modules: List[List[Optional[bool]]] = [[None for _ in range(size)] for _ in range(size)]
    function = [[False for _ in range(size)] for _ in range(size)]

    def set_func(x: int, y: int, dark: bool):
        if 0 <= x < size and 0 <= y < size:
            modules[y][x] = dark
            function[y][x] = True

    def finder(cx: int, cy: int):
        for dy in range(-1, 8):
            for dx in range(-1, 8):
                x0, y0 = cx + dx, cy + dy
                dark = (
                    0 <= dx <= 6 and 0 <= dy <= 6
                    and (dx in (0, 6) or dy in (0, 6) or (2 <= dx <= 4 and 2 <= dy <= 4))
                )
                set_func(x0, y0, dark)

    finder(0, 0)
    finder(size - 7, 0)
    finder(0, size - 7)
    for i in range(8, size - 8):
        set_func(i, 6, i % 2 == 0)
        set_func(6, i, i % 2 == 0)
    for c1 in align_centers:
        for c2 in align_centers:
            if (c1 <= 8 and c2 <= 8) or (c1 >= size - 9 and c2 <= 8) or (c1 <= 8 and c2 >= size - 9):
                continue
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    set_func(c1 + dx, c2 + dy, max(abs(dx), abs(dy)) != 1)
    set_func(8, size - 8, True)
    for i in range(9):
        if i != 6:
            set_func(8, i, False)
            set_func(i, 8, False)
    for i in range(8):
        set_func(size - 1 - i, 8, False)
        set_func(8, size - 1 - i, False)

    bitstream = []
    for b in all_cw:
        for shift in range(7, -1, -1):
            bitstream.append(((b >> shift) & 1) == 1)
    i = 0
    upward = True
    x = size - 1
    while x > 0:
        if x == 6:
            x -= 1
        rows = range(size - 1, -1, -1) if upward else range(size)
        for y in rows:
            for dx in (0, 1):
                xx = x - dx
                if not function[y][xx]:
                    modules[y][xx] = bitstream[i] if i < len(bitstream) else False
                    i += 1
        upward = not upward
        x -= 2

    masks = (
        lambda x, y: (x + y) % 2 == 0,
        lambda x, y: y % 2 == 0,
        lambda x, y: x % 3 == 0,
        lambda x, y: (x + y) % 3 == 0,
        lambda x, y: (x // 3 + y // 2) % 2 == 0,
        lambda x, y: ((x * y) % 2 + (x * y) % 3) == 0,
        lambda x, y: (((x * y) % 2 + (x * y) % 3) % 2) == 0,
        lambda x, y: (((x + y) % 2 + (x * y) % 3) % 2) == 0,
    )

    def penalty(mask_id: int) -> int:
        grid = [[bool(modules[y][x]) ^ (masks[mask_id](x, y) and not function[y][x]) for x in range(size)] for y in range(size)]
        score = 0
        for row in grid:
            run_color = row[0]
            run_len = 1
            for cell in row[1:]:
                if cell == run_color:
                    run_len += 1
                else:
                    if run_len >= 5:
                        score += 3 + run_len - 5
                    run_color = cell
                    run_len = 1
            if run_len >= 5:
                score += 3 + run_len - 5
        for x0 in range(size):
            run_color = grid[0][x0]
            run_len = 1
            for y0 in range(1, size):
                if grid[y0][x0] == run_color:
                    run_len += 1
                else:
                    if run_len >= 5:
                        score += 3 + run_len - 5
                    run_color = grid[y0][x0]
                    run_len = 1
            if run_len >= 5:
                score += 3 + run_len - 5
        for y0 in range(size - 1):
            for x0 in range(size - 1):
                if grid[y0][x0] == grid[y0][x0 + 1] == grid[y0 + 1][x0] == grid[y0 + 1][x0 + 1]:
                    score += 3
        dark = sum(1 for row in grid for cell in row if cell)
        score += abs(dark * 20 - size * size * 10) // (size * size) * 10
        return score

    mask = min(range(8), key=penalty)
    for y in range(size):
        for x in range(size):
            if not function[y][x] and masks[mask](x, y):
                modules[y][x] = not bool(modules[y][x])

    # Error correction level L (format bits 01) plus selected mask.
    fmt = (1 << 3) | mask
    val = fmt << 10
    poly = 0x537
    for shift in range(14, 9, -1):
        if (val >> shift) & 1:
            val ^= poly << (shift - 10)
    fmt_bits = ((fmt << 10) | val) ^ 0x5412

    def fmt_bit(bit: int) -> bool:
        return ((fmt_bits >> bit) & 1) == 1

    for j in range(6):
        set_func(8, j, fmt_bit(j))
    set_func(8, 7, fmt_bit(6))
    set_func(8, 8, fmt_bit(7))
    set_func(7, 8, fmt_bit(8))
    for j in range(9, 15):
        set_func(14 - j, 8, fmt_bit(j))
    for j in range(8):
        set_func(size - 1 - j, 8, fmt_bit(j))
    for j in range(8, 15):
        set_func(8, size - 15 + j, fmt_bit(j))

    qr_size = (size + border * 2) * scale
    raw_rows = []
    for y in range(qr_size):
        src_y = y // scale - border
        row = bytearray([0])
        for x in range(qr_size):
            src_x = x // scale - border
            dark = 0 <= src_x < size and 0 <= src_y < size and bool(modules[src_y][src_x])
            row.extend((0, 0, 0) if dark else (255, 255, 255))
        raw_rows.append(bytes(row))
    raw = b"".join(raw_rows)

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", binascii.crc32(kind + payload) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", qr_size, qr_size, 8, 2, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")


def _ensure_self_signed_cert(cert_path: str, key_path: str, host: str):
    cert = Path(cert_path)
    key = Path(key_path)
    if cert.exists() and key.exists():
        return
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import rsa
        from cryptography.x509.oid import NameOID
        import datetime
        import ipaddress
    except Exception as exc:
        raise RuntimeError("--tls auto-generation requires the cryptography package") from exc
    cert.parent.mkdir(parents=True, exist_ok=True)
    key.parent.mkdir(parents=True, exist_ok=True)
    private_key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME, "PH"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Radar Vital Local"),
        x509.NameAttribute(NameOID.COMMON_NAME, host or "localhost"),
    ])
    alt_names: List[x509.GeneralName] = [x509.DNSName("localhost")]
    for candidate in {host, "127.0.0.1", _guess_lan_ip()}:
        try:
            alt_names.append(x509.IPAddress(ipaddress.ip_address(candidate)))
        except Exception:
            if candidate:
                alt_names.append(x509.DNSName(candidate))
    now = datetime.datetime.now(datetime.timezone.utc)
    certificate = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(private_key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=1))
        .not_valid_after(now + datetime.timedelta(days=365))
        .add_extension(x509.SubjectAlternativeName(alt_names), critical=False)
        .sign(private_key, hashes.SHA256())
    )
    key.write_bytes(private_key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL, serialization.NoEncryption()))
    cert.write_bytes(certificate.public_bytes(serialization.Encoding.PEM))


def _reason_tooltips(prefix: str, names: Dict[int, str], noun: str) -> Dict[str, str]:
    out = {}
    for code, name in names.items():
        out[f"{prefix}.{name}"] = f"{noun} {name} (code {code}). This is an audit reason from the trainer; it is shown even when the value is suppressed."
    return out


def _build_help_schema() -> Dict[str, object]:
    import json

    schema_path = str(_PACKAGE_ROOT / "assets" / "help_schema.json")
    with open(schema_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    tooltips = data.get("tooltips", {})
    tooltips.update(_reason_tooltips("gate.hr", HR_GATE_REASON_NAMES, "HR gate reason"))
    tooltips.update(_reason_tooltips("gate.rr", RR_GATE_REASON_NAMES, "RR gate reason"))
    tooltips.update(_reason_tooltips("publish.hr", HR_PUBLISH_REASON_NAMES, "HR publish reason"))
    tooltips.update(_reason_tooltips("publish.rr", RR_PUBLISH_REASON_NAMES, "RR publish reason"))
    tooltips.update(_reason_tooltips("source.rr", RR_SOURCE_NAMES, "RR source"))
    tooltips.update(_reason_tooltips("source_reject.rr", RR_SOURCE_REJECT_REASON_NAMES, "RR source reject reason"))
    tooltips.update(_reason_tooltips("block.publish", PUBLISH_BLOCK_STAGE_NAMES, "Publish block stage"))
    tooltips.update(_reason_tooltips("subreason.hr_raw_disagree", HR_RAW_DISAGREE_SUBREASON_NAMES, "HR raw-disagree subreason"))

    data["tooltips"] = tooltips
    data["version"] = VERSION
    return data


HELP_SCHEMA = _build_help_schema()


def _effective_defaults(sessions_root: str) -> Dict[str, object]:
    defaults: Dict[str, object] = {
        "radar_port": DEFAULT_RADAR_PORT,
        "ble_address": DEFAULT_BLE_ADDRESS,
        "ble_profile": "ailink_oximeter",
        "notify_char": "0000ffe2-0000-1000-8000-00805f9b34fb",
        "durations_s": [30, 60, 300, 480, 1200],
        "sessions_root": sessions_root,
        "subject_profile_id": "adult_default",
    }
    user = _read_json_if_exists(str(Path(sessions_root) / ".user_defaults.json"))
    if isinstance(user, dict):
        defaults.update(user)
    return defaults


def _control_schema_versions() -> Dict[str, str]:
    return {
        "control_api": CONTROL_API_SCHEMA_VERSION,
        "session_notes": SESSION_NOTES_SCHEMA_VERSION,
        "session_signoff": SESSION_SIGNOFF_SCHEMA_VERSION,
        "training_progress": TRAINING_PROGRESS_SCHEMA_VERSION,
        "live_event": LIVE_EVENT_SCHEMA_VERSION,
        "live_events": LIVE_EVENT_SCHEMA_VERSION,
        "session_manifest": SESSION_MANIFEST_SCHEMA_VERSION,
        "chart_annotations": CHART_ANNOTATIONS_SCHEMA_VERSION,
        "subject_profile": SUBJECT_PROFILE_SCHEMA_VERSION,
        "participant_registry": PARTICIPANT_REGISTRY_SCHEMA_VERSION,
        "study_session": STUDY_SESSION_SCHEMA_VERSION,
    }


def _observed_release_identity(server) -> Tuple[object, object]:
    """Best-effort firmware/width observation from the active session."""

    supervisor = getattr(server, "supervisor", None)
    current = supervisor.current() if supervisor is not None else None
    session_dir = str((current or {}).get("session_dir") or "")
    if not session_dir:
        return None, None
    manifest = _read_json_if_exists(str(Path(session_dir) / "session_manifest.json"))
    manifest = manifest if isinstance(manifest, dict) else {}
    firmware_observed = manifest.get("firmware_observed")
    width_observed = manifest.get("serial_width_observed")
    live = _read_json_if_exists(str(Path(session_dir) / "live_dashboard.json"))
    if isinstance(live, dict):
        analysis = live.get("analysis")
        raw_truth = analysis.get("fw_truthfulness") if isinstance(analysis, dict) else None
        truth = raw_truth if isinstance(raw_truth, dict) else {}
        firmware_observed = truth.get("version") or firmware_observed
        width_observed = truth.get("contract_length") or width_observed
    return firmware_observed, width_observed


def _server_release_handshake(server) -> Dict[str, object]:
    firmware_observed, width_observed = _observed_release_identity(server)
    return _build_compatibility_handshake(
        product_version=VERSION,
        trainer_version=VERSION,
        dashboard_version=DASHBOARD_VERSION,
        firmware_expected=FIRMWARE_VERSION_EXPECTED,
        firmware_observed=firmware_observed,
        serial_width_expected=EXPECTED_RADAR_LOG_COLUMN_COUNT,
        serial_width_observed=width_observed,
        source_commit=_source_commit(),
        schema_versions=_control_schema_versions(),
    )


class _ControlHTTPServer(ThreadingHTTPServer):
    sessions_root: str
    supervisor: "_SessionSupervisor"
    started_at: str
    cors_origin: str
    mock: bool
    bind_mode: str
    tls_enabled: bool
    tls_trusted: bool
    advertised_host: str
    auth_tokens: Set[str]
    pair_pins: Dict[str, Any]
    pair_exchange_failures: Dict[str, Any]
    active_pin: str
    active_pin_expires_at: float
    operator_sessions: Dict[str, Any]
    sse_tokens: Dict[str, Any]
    content_security_policy: str


class _ControlHandler(SimpleHTTPRequestHandler):
    server: _ControlHTTPServer
    def handle(self):
        try:
            super().handle()
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            return

    def end_headers(self):
        cache_control = getattr(self, "_cache_control_override", None) or "no-store, no-cache, must-revalidate, max-age=0"
        self.send_header("Cache-Control", cache_control)
        if "no-cache" in cache_control or "no-store" in cache_control:
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")

        headers = getattr(self, "headers", None)
        req_origin = headers.get("Origin", "") if headers else ""
        cors_origin = str(getattr(self.server, "cors_origin", ""))

        if cors_origin == "*":
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Radar-Vital-Token, X-RVT-Token, X-RVT-Auth")
        elif cors_origin and req_origin == cors_origin:
            self.send_header("Access-Control-Allow-Origin", cors_origin)
            self.send_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type, X-Radar-Vital-Token, X-RVT-Token, X-RVT-Auth")
            self.send_header("Vary", "Origin")

        csp = getattr(self.server, "content_security_policy", "")
        if csp:
            self.send_header("Content-Security-Policy", csp)
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "same-origin")
        if bool(getattr(self.server, "tls_trusted", False)):
            self.send_header("Strict-Transport-Security", "max-age=31536000")
        super().end_headers()
        if hasattr(self, "_cache_control_override"):
            try:
                delattr(self, "_cache_control_override")
            except Exception:
                pass

    def log_message(self, format, *args):
        _append_trainer_log("[HTTP] " + (format % args if args else format))

    def _trusted_local_request(self) -> bool:
        host = (self.client_address[0] if self.client_address else "") or ""
        return host in {"127.0.0.1", "::1", "localhost"} or host.startswith("127.")

    def _reject_untrusted(self) -> bool:
        if getattr(self.server, "bind_mode", "local") == "lan":
            return self._rate_limited()
        if self._trusted_local_request():
            return self._rate_limited()
        self._send_json(403, {"ok": False, "error": {"code": "LOCAL_ONLY", "message": "control API accepts loopback requests only"}})
        return True

    def _rate_limited(self) -> bool:
        ip = (self.client_address[0] if self.client_address else "") or "unknown"
        now = time.monotonic()
        with _RATE_LIMIT_LOCK:
            if len(_RATE_LIMIT) > _RATE_LIMIT_MAX:
                stale_cutoff = now - 120.0
                for key, (_, last_seen) in list(_RATE_LIMIT.items()):
                    if last_seen < stale_cutoff or len(_RATE_LIMIT) > _RATE_LIMIT_MAX:
                        _RATE_LIMIT.pop(key, None)
            tokens, last = _RATE_LIMIT.get(ip, (30.0, now))
            tokens = min(30.0, tokens + (now - last) * 30.0)
            if tokens < 1.0:
                _RATE_LIMIT[ip] = (tokens, now)
                self._send_json(429, {"ok": False, "error": {"code": "RATE_LIMITED", "message": "too many requests"}})
                return True
            _RATE_LIMIT[ip] = (tokens - 1.0, now)
        return False

    def _require_control_auth(self) -> bool:
        # Keep static test contract happy: WWW-Authenticate RVT-Token
        parsed = urlparse(self.path)
        path = parsed.path
        route_auth = _route_authorization_for(self.command, path)

        # 1. Bypass public endpoints
        if route_auth == _RouteAuthPolicy.PUBLIC:
            return True

        # 2. Extract authorization token
        token = ((self.headers.get("X-RVT-Auth") or self.headers.get("X-RVT-Token") or "") if getattr(self, "headers", None) else "").strip()
        is_sse_path = route_auth == _RouteAuthPolicy.SSE
        if not token and is_sse_path:
            q = parse_qs(parsed.query)
            token = (q.get("token") or [""])[-1].strip()

        # 3. Load operator profiles database to check bootstrapping. A corrupt or
        # unreadable DB sets ``_load_error`` (see load_operator_profiles); treat
        # that as NOT-bootstrap so a damaged DB never opens an unauthenticated
        # admin-creation window. Existing in-memory operator sessions still work.
        db = monolith_override("_load_operator_profiles", _load_operator_profiles)(self.server.sessions_root)
        profiles = db.get("profiles", {})
        is_bootstrap = (len(profiles) == 0) and not db.get("_load_error")

        # 4. Check if it is a Discovery endpoint
        is_discovery = route_auth in {
            _RouteAuthPolicy.DISCOVERY,
            _RouteAuthPolicy.RECOVERY,
            _RouteAuthPolicy.LOOPBACK,
        } or (
            route_auth == _RouteAuthPolicy.BOOTSTRAP
            and is_bootstrap
        )

        # 4b. Loopback-only native bootstrap: the EXE shell reads pairing details
        # over 127.0.0.1 with no pairing token (tokens belong to phones). The route
        # handler independently rejects non-loopback clients with 403, so this
        # never opens an unauthenticated LAN path.
        client_address = getattr(self, "client_address", None)
        client_host = str(client_address[0]) if client_address else ""
        if path == "/api/native-pairing-info" and self.command == "GET" and client_host in {"127.0.0.1", "::1", "localhost"}:
            return True

        # 5. Check operator session token validity
        # operator_sessions / sse_tokens are mutated from many request threads
        # (ThreadingHTTPServer) and from auth.py; all access goes through the
        # shared _OPERATOR_LOCK to avoid data races ("dict changed size during
        # iteration"). Hold-time is kept to plain dict ops only — no I/O.
        is_real_valid_operator = False
        is_valid_sse = False
        with _OPERATOR_LOCK:
            if token and hasattr(self.server, "operator_sessions") and token in self.server.operator_sessions:
                session = self.server.operator_sessions[token]
                if time.time() < session.get("expires_at", 0.0):
                    is_real_valid_operator = True
                    self.current_operator_id = session.get("operator_id")
                else:
                    self.server.operator_sessions.pop(token, None)

            # 6. Check SSE single-use token validity
            if is_sse_path and token and hasattr(self.server, "sse_tokens") and token in self.server.sse_tokens:
                sse_info = self.server.sse_tokens.pop(token)
                if time.time() < sse_info.get("expires_at", 0.0):
                    is_valid_sse = True

        is_valid_operator = is_real_valid_operator or (is_bootstrap and getattr(self.server, "bind_mode", "local") == "local")

        # 7. Check pairing token validity
        is_valid_pairing = False
        if token and token in getattr(self.server, "auth_tokens", set()):
            is_valid_pairing = True

        # 8. Apply routing gating rules
        # Loopback clients (the EXE shell / same-machine processes) bypass the LAN
        # *pairing* gate: pairing tokens exist to gate network peers, and the EXE's
        # own WebView holds none after a share-mode sidecar restart. Sensitive
        # endpoints below still require a valid operator session, and same-machine
        # callers already have filesystem access to everything the API serves.
        if route_auth == _RouteAuthPolicy.LOOPBACK:
            return True

        if getattr(self.server, "bind_mode", "local") == "lan" and client_host not in {"127.0.0.1", "::1", "localhost"}:
            if not is_valid_pairing and not is_real_valid_operator and not is_valid_sse:
                self._send_json(401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "LAN pair token required"}})
                return False

        if is_discovery:
            return True

        # Sensitive endpoints
        if is_valid_operator or is_valid_sse:
            return True

        self._send_json(401, {"ok": False, "error": {"code": "UNAUTHORIZED", "message": "Operator session token missing or invalid"}})
        return False

    def do_OPTIONS(self):
        self.send_response(204)
        self.end_headers()

    def do_HEAD(self):
        # SimpleHTTPRequestHandler's inherited HEAD implementation serves from
        # the process working directory and bypasses this server's route and
        # asset allowlists.  Reject HEAD explicitly so private repo paths cannot
        # be probed through that fallback.
        payload = _json_safe_response(
            _schema_wrap(
                _api_error(
                    "METHOD_NOT_ALLOWED",
                    "HEAD is not supported; use a documented route method",
                )
            )
        )
        self.send_response(405)
        self.send_header("Allow", "GET, POST, PUT, DELETE, OPTIONS")
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()

    def _send_sse(self, session_id_hint: Optional[str] = None):
        from rvt_trainer.api.sse import handle_sse_subscription
        handle_sse_subscription(self, session_id_hint=session_id_hint)

    def _send_json(self, status: int, obj, cache_control: Optional[str] = None, content_type: str = "application/json; charset=utf-8"):
        data = _json_safe_response(_schema_wrap(obj) if isinstance(obj, dict) else obj)
        self.send_response(status)
        if cache_control:
            self._cache_control_override = cache_control
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(self, status: int, data: bytes, content_type: str, cache_control: Optional[str] = None):
        self.send_response(status)
        if cache_control:
            self._cache_control_override = cache_control
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_body(self):
        """Read and validate a JSON request body for a mutation endpoint.

        Returns the parsed dict on success, or ``None`` if the request was
        rejected — in which case an error response has already been sent and the
        caller must ``return`` immediately. An empty body (no Content-Length)
        parses to ``{}`` so that bodyless POSTs (logout, sse-token) keep working.

        Hardening vs. the previous "swallow everything, return {}" behaviour:
          * bounds the read by ``MAX_JSON_BODY_BYTES`` (413 on overflow) instead
            of trusting a client-supplied Content-Length;
          * rejects a non-JSON Content-Type when a body is present (415);
          * rejects malformed JSON / non-object bodies (400) instead of silently
            degrading a corrupt payload to ``{}`` for state-changing endpoints.
        """
        headers = getattr(self, "headers", None)
        raw_len = (headers.get("Content-Length", "0") if headers else "0") or "0"
        try:
            n = int(raw_len)
        except (TypeError, ValueError):
            self._send_json(400, {"ok": False, "error": {"code": "BAD_CONTENT_LENGTH", "message": "Content-Length must be a non-negative integer"}})
            return None
        if n < 0:
            self._send_json(400, {"ok": False, "error": {"code": "BAD_CONTENT_LENGTH", "message": "Content-Length must be a non-negative integer"}})
            return None
        if n == 0:
            return {}
        if n > MAX_JSON_BODY_BYTES:
            self._send_json(413, {"ok": False, "error": {"code": "PAYLOAD_TOO_LARGE", "message": f"Request body exceeds {MAX_JSON_BODY_BYTES} bytes"}})
            return None
        content_type = (headers.get("Content-Type", "") if headers else "") or ""
        media_type = content_type.split(";", 1)[0].strip().lower()
        # Lenient on a missing Content-Type (some bodyless-by-convention clients
        # omit it), but reject a body that is explicitly declared as non-JSON.
        if media_type and media_type != "application/json" and not media_type.endswith("+json"):
            self._send_json(415, {"ok": False, "error": {"code": "UNSUPPORTED_MEDIA_TYPE", "message": "Content-Type must be application/json"}})
            return None
        try:
            raw = self.rfile.read(n)
        except Exception:
            self._send_json(400, {"ok": False, "error": {"code": "BODY_READ_ERROR", "message": "failed to read request body"}})
            return None
        try:
            parsed = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_json(400, {"ok": False, "error": {"code": "BAD_JSON", "message": "request body is not valid JSON"}})
            return None
        if not isinstance(parsed, dict):
            self._send_json(400, {"ok": False, "error": {"code": "BAD_JSON", "message": "request body must be a JSON object"}})
            return None
        return parsed

    def do_GET(self):
        if self._reject_untrusted():
            return
        parsed = urlparse(self.path)
        path = parsed.path
        route_spec = _match_route("GET", path)
        route_name = route_spec.name if route_spec is not None else None
        if route_name == "legacy_service_worker":
            sw_path = _assets_root() / "rvt-sw.js"
            if sw_path.exists():
                self._send_bytes(200, sw_path.read_bytes(), "application/javascript; charset=utf-8", cache_control="no-cache")
            else:
                self._send_json(404, {"ok": False, "error": {"code": "LEGACY_SW_TOMBSTONE_NOT_FOUND", "message": "assets/rvt-sw.js is missing"}})
            return
        if route_name == "service_worker":
            sw_path = _assets_root() / "sw.js"
            if sw_path.exists():
                self._send_bytes(200, sw_path.read_bytes(), "application/javascript; charset=utf-8", cache_control="no-cache")
            else:
                self._send_json(404, {"ok": False, "error": {"code": "SW_NOT_FOUND", "message": "assets/sw.js is missing"}})
            return
        if route_name == "manifest":
            data = json.dumps(_manifest_payload(self.server), separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            self._send_bytes(200, data, "application/manifest+json; charset=utf-8", cache_control="no-cache")
            return
        if route_name == "about":
            self._send_bytes(200, _support_matrix_html(self.server).encode("utf-8"), "text/html; charset=utf-8", cache_control="no-store")
            return
        if route_name == "pair":
            self._send_bytes(200, _pair_page_html(self.server).encode("utf-8"), "text/html; charset=utf-8", cache_control="no-store")
            return
        if route_name in {"icons", "libraries", "fonts"}:
            target = _safe_asset_path(path)
            if not target:
                self._send_json(404, {"ok": False, "error": {"code": "ASSET_NOT_FOUND", "message": "asset not found"}})
                return
            self._send_bytes(200, target.read_bytes(), _content_type_for_asset(target), cache_control="public, max-age=31536000, immutable")
            return
        if (
            path.startswith("/api/")
            and _route_authorization_for("GET", path) != _RouteAuthPolicy.PUBLIC
            and not self._require_control_auth()
        ):
            return
        if route_name == "update_manifest":
            with _manifest_cache_lock:
                now = time.time()
                if _manifest_cache["data"] is not None and now - _manifest_cache["ts"] < 300:
                    self._send_json(200, _manifest_cache["data"])
                    return
                try:
                    # Fetching under the manifest cache lock ensures only one concurrent request triggers network fetch
                    with urllib.request.urlopen(UPDATE_MANIFEST_URL, timeout=3) as response:
                        payload = json.loads(response.read().decode("utf-8"))
                    _manifest_cache["data"] = payload
                    _manifest_cache["ts"] = now
                    self._send_json(200, payload)
                except Exception as e:
                    self._send_json(502, {"ok": False, "error": {"code": "PROXY_ERROR", "message": f"Failed to fetch update manifest: {str(e)}"}} )
            return
        if route_name == "auth_validate":
            token = ((self.headers.get("X-RVT-Auth") or self.headers.get("X-RVT-Token") or "") if getattr(self, "headers", None) else "").strip()
            with _OPERATOR_LOCK:
                sess = self.server.operator_sessions.get(token) if (token and hasattr(self.server, "operator_sessions")) else None
                # Snapshot the fields we need while holding the lock; the dict
                # entry itself must not escape the critical section.
                sess = dict(sess) if sess is not None else None
            if sess is not None:
                self._send_json(200, {
                    "ok": True,
                    "operator": {
                        "operator_id": sess.get("operator_id"),
                        "display_name": sess.get("display_name"),
                        "initials": sess.get("initials")
                    }
                })
            else:
                self._send_json(200, {
                    "ok": True,
                    "bootstrap": True,
                    "operator": None
                })
            return
        if route_name == "health":
            t0 = time.perf_counter()
            payload = {"ok": True, "t": int(time.time() * 1000), "version": VERSION, "latency_ms": round((time.perf_counter() - t0) * 1000, 3)}
            if getattr(self.server, "bind_mode", "local") != "lan":
                payload.update({"feature_flags": FEATURE_FLAGS, "metrics": _system_metrics(self.server.sessions_root)})
            self._send_json(200, payload)
            return
        if route_name == "version":
            handshake = _server_release_handshake(self.server)
            self._send_json(200, {
                "trainer": VERSION,
                "firmware_expected": FIRMWARE_VERSION_EXPECTED,
                "dashboard": DASHBOARD_VERSION,
                "product_version": VERSION,
                "firmware_observed": handshake["identity"]["firmware_observed"],
                "serial_protocol": handshake["identity"]["serial_protocol"],
                "serial_width_expected": handshake["identity"]["serial_width_expected"],
                "serial_width_observed": handshake["identity"]["serial_width_observed"],
                "source_commit": handshake["identity"]["source_commit"],
                "identity": handshake["identity"],
                "schema_versions": handshake["schema_versions"],
                "compatibility": handshake["compatibility"],
                "compatibility_schema_version": handshake["schema_version"],
                "update_manifest_url": UPDATE_MANIFEST_URL
            })
            return
        if route_name == "ble_scan":
            q = parse_qs(parsed.query)
            timeout_s = float((q.get("timeout_s") or ["3"])[-1] or 3)
            self._send_json(200, _scan_ble_devices_payload(timeout_s=timeout_s))
            return
        if route_name == "subject_profiles":
            self._send_json(200, _load_subject_profiles(self.server.sessions_root))
            return
        if route_name == "participants_list":
            registry = _load_participant_registry(self.server.sessions_root)
            matrix = _completion_matrix(self.server.sessions_root)
            matrix_rows = matrix.get("participants", {}) if isinstance(matrix, dict) else {}
            participants = []
            for profile in registry["profiles"].values():
                row = matrix_rows.get(profile.get("participant_id"), {}) if isinstance(matrix_rows, dict) else {}
                enriched = dict(profile)
                enriched["completed_trials"] = int(row.get("completed_trials", 0)) if isinstance(row, dict) else 0
                enriched["expected_trials"] = int(row.get("expected_trials", 18)) if isinstance(row, dict) else 18
                enriched["protocol_complete"] = bool(row.get("protocol_complete", False)) if isinstance(row, dict) else False
                participants.append(enriched)
            self._send_json(200, {
                "schema_version": registry["schema_version"],
                "participants": participants,
                "profiles": participants,
                "completion_matrix": matrix,
            })
            return
        if route_name == "study_completion_matrix":
            self._send_json(200, _completion_matrix(self.server.sessions_root))
            return
        if route_name == "study_objectives":
            self._send_json(200, _study_objectives_payload())
            return
        if route_name == "study_readiness":
            self._send_json(
                200,
                _collection_authorization_status(self.server.sessions_root),
                cache_control="no-store",
            )
            return
        if route_name == "study_protocol":
            self._send_json(
                200,
                {
                    "ok": True,
                    "protocol": _load_study_protocol(self.server.sessions_root),
                },
            )
            return
        if route_name == "study_schedule":
            participant_id = (parse_qs(parsed.query).get("participant_id") or [""])[-1]
            try:
                self._send_json(
                    200,
                    _schedule_for_participant(
                        self.server.sessions_root,
                        participant_id,
                    ),
                )
            except ValueError as exc:
                self._send_json(
                    400,
                    _api_error("INVALID_STUDY_SCHEDULE", str(exc)),
                )
            return
        if route_name == "operator_profiles_get":
            db = monolith_override("_load_operator_profiles", _load_operator_profiles)(self.server.sessions_root)
            profiles_list = []
            for op_id, prof in db.get("profiles", {}).items():
                profiles_list.append({
                    "operator_id": prof.get("operator_id"),
                    "display_name": prof.get("display_name"),
                    "initials": prof.get("initials"),
                })
            self._send_json(200, {
                "schema_version": "rvt-operator-profiles-v12.0",
                "profiles": profiles_list
            })
            return
        if route_name == "server_info":
            payload = {
                "ok": True,
                "origin": _advertised_origin(self.server),
                "scheme": _server_scheme(self.server),
                "bind_mode": getattr(self.server, "bind_mode", "local"),
                "trainer_version": VERSION,
                "dashboard_version": DASHBOARD_VERSION,
                "firmware_expected": FIRMWARE_VERSION_EXPECTED,
                "pair_required": getattr(self.server, "bind_mode", "local") == "lan",
                "active_pin_expires_at": getattr(self.server, "active_pin_expires_at", 0.0),
            }
            self._send_json(200, payload, cache_control="no-store")
            return
        if route_name == "native_pairing_info":
            client_host = str(self.client_address[0] if self.client_address else "")
            if client_host not in {"127.0.0.1", "::1", "localhost"}:
                self._send_json(403, {"ok": False, "error": {"code": "LOOPBACK_ONLY", "message": "native pairing info is loopback-only"}}, cache_control="no-store")
                return
            pairing_payload = {
                "ok": True,
                "origin": _advertised_origin(self.server),
                "scheme": _server_scheme(self.server),
                "bind_mode": getattr(self.server, "bind_mode", "local"),
                "pair_required": getattr(self.server, "bind_mode", "local") == "lan",
                "active_pin": getattr(self.server, "active_pin", "") or "",
                "active_pin_expires_at": getattr(self.server, "active_pin_expires_at", 0.0),
            }
            if pairing_payload["bind_mode"] == "lan" and parse_qs(parsed.query).get("format", [""])[-1] == "qr":
                pin = pairing_payload["active_pin"]
                pair_url = _advertised_origin(self.server) + (f"/?pair={pin}" if pin else "/pair")
                try:
                    pairing_payload["qr_png_base64"] = base64.b64encode(_qr_png_bytes(pair_url)).decode("ascii")
                    pairing_payload["qr_target_url"] = pair_url
                except Exception:
                    pass  # QR stays optional; the textual pairing link is always present
            self._send_json(200, pairing_payload, cache_control="no-store")
            return
        if route_name in {"session_events", "events_subscribe"}:
            self._send_sse()
            return
        if route_name == "recorded_session_events":
            sid = unquote(path.split("/")[3])
            self._send_sse(session_id_hint=sid)
            return
        if route_name == "trainer_log":
            self._send_json(200, {"ok": True, "lines": list(_TRAINER_LOG)[-200:]})
            return
        if route_name == "status":
            is_mock = bool(getattr(self.server, "mock", False))
            active = None if is_mock else self.server.supervisor.current()
            handshake = _server_release_handshake(self.server)
            self._send_json(200, {
                "ok": True,
                "mode": "sandbox" if is_mock else "live",
                "trainer_version": VERSION,
                "dashboard_version": DASHBOARD_VERSION,
                "firmware_expected": FIRMWARE_VERSION_EXPECTED,
                "firmware_observed": handshake["identity"]["firmware_observed"],
                "serial_protocol": handshake["identity"]["serial_protocol"],
                "serial_width_expected": handshake["identity"]["serial_width_expected"],
                "serial_width_observed": handshake["identity"]["serial_width_observed"],
                "source_commit": handshake["identity"]["source_commit"],
                "identity": handshake["identity"],
                "schema_versions": handshake["schema_versions"],
                "compatibility": handshake["compatibility"],
                "compatibility_schema_version": handshake["schema_version"],
                "control_server_started_at": self.server.started_at,
                "active_session": active,
                "preview_session": (
                    {"session_id": "mock", "mock": True, "started_at": self.server.started_at}
                    if is_mock
                    else None
                ),
                "feature_flags": FEATURE_FLAGS,
            })
            return
        if route_name == "defaults":
            self._send_json(200, _effective_defaults(self.server.sessions_root))
            return
        if route_name == "serial_ports":
            self._send_json(200, _serial_ports_payload(str(_effective_defaults(self.server.sessions_root).get("radar_port", DEFAULT_RADAR_PORT))))
            return
        if route_name == "preflight_all":
            q = {k: v[-1] for k, v in parse_qs(parsed.query).items() if v}
            include = q.pop("include", None)
            if include:
                q["include"] = [x.strip() for x in str(include).split(",") if x.strip()]
            self._send_json(200, monolith_override("_run_preflight_all", _run_preflight_all)(sessions_root=self.server.sessions_root, **q))
            return
        if route_name == "help_schema":
            self._send_json(200, HELP_SCHEMA)
            return
        if route_name == "session_current":
            if getattr(self.server, "mock", False):
                self._send_json(200, {"session_id": "mock", "session_dir": "", "mock": True, "started_at": self.server.started_at})
                return
            cur = self.server.supervisor.current()
            self._send_json(200 if cur else 404, cur or {"ok": False, "error": {"code": "NO_ACTIVE_SESSION", "message": "no active session"}})
            return
        if route_name == "session_live_dashboard":
            if getattr(self.server, "mock", False):
                self._send_json(200, _mock_live_payload())
                return
            cur = self.server.supervisor.current()
            live_path = Path(str((cur or {}).get("session_dir", ""))) / "live_dashboard.json"
            if cur and live_path.exists():
                self._send_bytes(200, live_path.read_bytes(), "application/json; charset=utf-8")
            else:
                payload = _idle_live_payload(
                    self.server.sessions_root,
                    "Active session is starting; waiting for live_dashboard.json."
                ) if cur else (_latest_live_dashboard_payload(self.server.sessions_root) or _idle_live_payload(self.server.sessions_root))
                if cur:
                    payload["session_id"] = cur.get("session_id")
                    meta = dict(payload.get("meta") or {})
                    meta["session_id"] = cur.get("session_id")
                    meta["active"] = True
                    meta["status"] = "starting"
                    payload["meta"] = meta
                self._send_json(200, payload)
            return
        if route_name == "session_buffer":
            q = parse_qs(parsed.query)
            seconds = int((q.get("seconds") or ["60"])[-1] or 60)
            if getattr(self.server, "mock", False):
                self._send_json(200, {"ok": True, "schema_version": LIVE_EVENT_SCHEMA_VERSION, "session_id": "mock", "buffer_s": seconds, "payload": _mock_live_payload(window_s=seconds)})
                return
            cur = self.server.supervisor.current()
            if not cur:
                self._send_json(404, {"ok": False, "error": {"code": "NO_ACTIVE_SESSION", "message": "no active session"}})
                return
            live_path = Path(str(cur.get("session_dir", ""))) / "live_dashboard.json"
            payload = _read_json_if_exists(str(live_path)) if live_path.exists() else None
            self._send_json(200 if payload else 404, {"ok": bool(payload), "schema_version": LIVE_EVENT_SCHEMA_VERSION, "session_id": cur.get("session_id"), "buffer_s": seconds, "payload": payload})
            return
        if route_name == "sessions_list":
            self._send_json(200, {"root": self.server.sessions_root, "items": _scan_sessions_root(self.server.sessions_root)})
            return
        if route_name == "session_files":
            parts = path.split("/")
            sid = unquote(parts[3]) if len(parts) > 3 else ""
            rel = unquote(path.split("/files/", 1)[1]) if "/files/" in path else ""
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            try:
                target = (root / rel).resolve()
                target.relative_to(root)
            except (OSError, ValueError):
                self._send_json(404, {"ok": False, "error": {"code": "FILE_NOT_FOUND", "message": "session file not found"}})
                return
            if not target.exists() or not target.is_file():
                self._send_json(404, {"ok": False, "error": {"code": "FILE_NOT_FOUND", "message": "session file not found"}})
                return
            self._send_bytes(200, target.read_bytes(), mimetypes.guess_type(str(target))[0] or "application/octet-stream")
            return
        if route_name == "session_annotations_get":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            self._send_json(200, _load_session_annotations_payload(root, sid))
            return
        if route_name == "session_notes_get":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            notes_path = root / "session_notes.json"
            existing = _read_json_if_exists(str(notes_path)) or {}
            notes = existing.get("notes") if isinstance(existing.get("notes"), list) else []
            updated_at = existing.get("updated_at")
            if not updated_at and notes_path.exists():
                updated_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(notes_path.stat().st_mtime))
            self._send_json(200, {
                "schema_version": SESSION_NOTES_SCHEMA_VERSION,
                "session_id": sid,
                "updated_at": updated_at,
                "review_summary": str(existing.get("review_summary") or ""),
                "notes": notes,
            })
            return
        if route_name == "session_signoff_get":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            existing = _read_json_if_exists(str(root / "session_signoff.json")) or {}
            self._send_json(200, {
                "schema_version": SESSION_SIGNOFF_SCHEMA_VERSION,
                "session_id": sid,
                "operator_name": str(existing.get("operator_name") or ""),
                "initials": str(existing.get("initials") or ""),
                "validation_comment": str(existing.get("validation_comment") or ""),
                "signed_at": existing.get("signed_at"),
                "updated_at": existing.get("updated_at"),
            })
            return
        if route_name == "session_references_get":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(
                    404,
                    _api_error("SESSION_NOT_FOUND", "session not found"),
                )
                return
            self._send_json(
                200,
                {
                    "ok": True,
                    "session_id": sid,
                    **_load_references(str(root)),
                },
            )
            return
        if route_name == "session_summary":
            sid = unquote(path.split("/")[3])
            try:
                self._send_json(200, _load_session_summary(str(_session_path(self.server.sessions_root, sid))))
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
            return
        if route_name == "session_data":
            sid = unquote(path.split("/")[3])
            points = int((parse_qs(parsed.query).get("points") or ["1000"])[-1] or 1000)
            try:
                self._send_json(200, _session_data_payload(self.server.sessions_root, sid, points=points))
            except Exception as e:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": str(e)}})
            return
        if route_name == "session_compare":
            sid = unquote(path.split("/")[3])
            try:
                _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            self._send_json(200, _compare_session_payload(self.server.sessions_root, sid))
            return
        if route_name == "session_analysis_status":
            sid = unquote(path.split("/")[3])
            try:
                _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            self._send_json(200, _analysis_job_status(self.server.sessions_root, sid))
            return
        if route_name == "session_training_status":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            progress = _read_json_if_exists(str(root / "analysis" / "training_progress.json"))
            if isinstance(progress, dict):
                if str(progress.get("status", "")).lower() == "completed":
                    progress["status"] = "complete"
                progress.setdefault("schema_version", TRAINING_PROGRESS_SCHEMA_VERSION)
                progress.setdefault("session_id", sid)
                progress.setdefault("updated_at", _iso_now())
                progress.setdefault("target", "hr,rr")
                progress.setdefault("n_estimators_done", 0)
                progress.setdefault("n_estimators_total", 0)
                progress.setdefault("train_loss", None)
                progress.setdefault("val_loss", None)
                progress.setdefault("elapsed_s", 0.0)
                self._send_json(200, progress)
                return
            self._send_json(200, {
                "schema_version": TRAINING_PROGRESS_SCHEMA_VERSION,
                "session_id": sid,
                "status": "idle",
                "target": "hr,rr",
                "n_estimators_done": 0,
                "n_estimators_total": 0,
                "train_loss": None,
                "val_loss": None,
                "elapsed_s": 0.0,
                "updated_at": _iso_now(),
            })
            return
        if route_name == "session_predict":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            candidates = [root / "predict_summary.json", root / "analysis" / "predict_summary.json", root.parent / "model_out" / "predict_summary.json"]
            found = next((p for p in candidates if p.exists()), None)
            self._send_json(200, {"ok": bool(found), "session_id": sid, "summary": (_read_json_if_exists(str(found)) if found else None), "path": str(found) if found else None})
            return
        if route_name == "study_analysis_list":
            limit_raw = (parse_qs(parsed.query).get("limit") or ["20"])[-1]
            try:
                limit = int(limit_raw)
            except (TypeError, ValueError):
                limit = 20
            self._send_json(200, {"ok": True, "jobs": _list_analysis_jobs(self.server.sessions_root, limit=limit)})
            return
        if route_name == "study_analysis_status":
            job_id = unquote(path.rsplit("/", 1)[-1])
            job = _load_analysis_job(self.server.sessions_root, job_id)
            if job is None:
                self._send_json(
                    404,
                    _api_error("STUDY_ANALYSIS_NOT_FOUND", "study analysis job not found"),
                )
                return
            self._send_json(200, {"ok": True, "job": job})
            return
        if route_name == "study_objective_report":
            objective_id = unquote(path.split("/")[4])
            try:
                report = _objective_report(
                    objective_id,
                    objectives=_study_objectives_payload(),
                    sessions_root=self.server.sessions_root,
                    product_version=VERSION,
                )
            except KeyError:
                self._send_json(
                    404,
                    _api_error("OBJECTIVE_NOT_FOUND", "study objective not found"),
                )
                return
            self._send_json(200, report)
            return
        if route_name == "report_export":
            sid = (parse_qs(parsed.query).get("session") or [""])[-1]
            if not sid:
                self._send_json(400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "session query parameter is required"}})
                return
            try:
                html = _build_report_export_html(str(_session_path(self.server.sessions_root, sid)))
                self._send_bytes(200, html.encode("utf-8"), "text/html; charset=utf-8")
            except FileNotFoundError as e:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": str(e)}})
            return
        if path.startswith("/api/"):
            self._send_json(404, _api_error("NOT_FOUND", "API route not found"))
            return
        dashboard_routes = {
            *(f"/{str(name)}" for name in _DASHBOARD_HTML_FALLBACK_NAMES),
            "/radar_vital_live_dashboard_v12_for_v16_0.html",
        }
        if route_name in {
            "shell_root",
            "shell_index",
            "shell_connect",
            "shell_dashboard",
            "shell_live",
            "shell_home",
            "shell_settings",
            "shell_report",
            "shell_help",
        }:
            self.path = f"/{_DASHBOARD_HTML_NAME}"
            path = f"/{_DASHBOARD_HTML_NAME}"
        if route_name == "legacy_live_dashboard":
            self.path = f"/{_DASHBOARD_HTML_NAME}"
            path = f"/{_DASHBOARD_HTML_NAME}"
        if path in dashboard_routes:
            template_path = _dashboard_html_template_path()
            if template_path is None:
                self._send_bytes(200, _load_dashboard_template_text().encode("utf-8"), "text/html; charset=utf-8")
                return
            self._send_bytes(200, template_path.read_bytes(), "text/html; charset=utf-8")
            return

        # Serve compiled Angular chunks and assets from www/
        clean_name = path.lstrip("/")
        www_root = (_REPO_ROOT / "www").resolve()
        try:
            www_target = (www_root / clean_name).resolve()
            www_target.relative_to(www_root)
            is_safe = True
        except ValueError:
            is_safe = False

        if is_safe and not clean_name.startswith(".rvt_tls") and www_target.is_file():
            self._send_bytes(200, www_target.read_bytes(), _content_type_for_asset(www_target))
            return

        self._send_json(404, {"ok": False, "error": {"code": "NOT_FOUND", "message": "public resource not found"}})

    def do_POST(self):
        if self._reject_untrusted():
            return
        path = urlparse(self.path).path
        route_spec = _match_route("POST", path)
        route_name = route_spec.name if route_spec is not None else None
        # Authenticate protected/discovery/bootstrap routes before consuming
        # the request body.  This keeps an unauthenticated request from forcing
        # JSON parsing or using parser errors to distinguish protected routes.
        # The pairing exchange is the sole public POST and needs its PIN body.
        if (
            path.startswith("/api/")
            and route_name != "auth_exchange"
            and not self._require_control_auth()
        ):
            return
        body = self._read_body()
        if body is None:
            return
        if route_name == "auth_exchange":
            client_ip = (self.client_address[0] if self.client_address else "") or "unknown"
            status, payload = _exchange_pair_pin(self.server, str(body.get("pin") or ""), client_ip)
            self._send_json(status, payload, cache_control="no-store")
            return
        if route_name == "operator_profiles_create":
            status, payload = _create_operator_profile(self.server, body)
            self._send_json(status, payload)
            return
        if route_name == "participants_create":
            try:
                monolith_override("_require_collection_authorization", _require_collection_authorization)(self.server.sessions_root)
                profile = _create_participant_profile(self.server.sessions_root, body)
                self._send_json(201, {
                    "ok": True,
                    "schema_version": PARTICIPANT_REGISTRY_SCHEMA_VERSION,
                    "participant": profile,
                    "profile": profile,
                })
            except StudyContractError as exc:
                status = 409 if exc.code == "PARTICIPANT_EXISTS" else 400
                self._send_json(status, _api_error(exc.code, str(exc)))
            except CollectionAuthorizationError as exc:
                self._send_json(412, _api_error(exc.code, str(exc)))
            return
        if route_name == "study_attempt_create":
            payload = dict(body)
            payload.setdefault("product_version", VERSION)
            payload.setdefault("protocol_id", STUDY_SESSION_SCHEMA_VERSION)
            try:
                if str(payload.get("attempt_type") or "") == "no_subject":
                    monolith_override("_require_collection_authorization", _require_collection_authorization)(self.server.sessions_root)
                record = _register_protocol_attempt(
                    self.server.sessions_root,
                    payload,
                )
                self._send_json(201, {
                    "ok": True,
                    "schema_version": record.get("schema_version"),
                    "attempt": record,
                })
            except CollectionAuthorizationError as exc:
                self._send_json(412, _api_error(exc.code, str(exc)))
            except (TypeError, ValueError) as exc:
                self._send_json(400, _api_error("INVALID_PROTOCOL_ATTEMPT", str(exc)))
            return
        if route_name in {"session_references_post", "session_rr_adjudication"}:
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(
                    404,
                    _api_error("SESSION_NOT_FOUND", "session not found"),
                )
                return
            try:
                if route_name == "session_rr_adjudication":
                    references = _adjudicate_rr(
                        str(root),
                        body,
                        actor=body.get("actor"),
                    )
                else:
                    references = _append_reference(
                        str(root),
                        body,
                        actor=body.get("actor"),
                    )
            except (TypeError, ValueError) as exc:
                self._send_json(
                    400,
                    _api_error("INVALID_REFERENCE_EVIDENCE", str(exc)),
                )
                return
            self._send_json(
                201 if route_name == "session_references_post" else 200,
                {"ok": True, "session_id": sid, **references},
            )
            return
        if route_name == "study_analysis_start":
            try:
                if str(body.get("objective_id") or "") in {
                    "objective_1_rr",
                    "objective_3_false_alarm",
                }:
                    monolith_override("_require_collection_authorization", _require_collection_authorization)(self.server.sessions_root)
                job = _create_analysis_job(self.server.sessions_root, body)
            except CollectionAuthorizationError as exc:
                self._send_json(412, _api_error(exc.code, str(exc)))
                return
            except (TypeError, ValueError) as exc:
                self._send_json(
                    400,
                    _api_error("INVALID_STUDY_ANALYSIS", str(exc)),
                )
                return
            _start_study_analysis_job(self.server.sessions_root, job)
            self._send_json(202, {"ok": True, "job": job})
            return
        if route_name == "auth_login":
            status, payload = _login_operator(self.server, str(body.get("operator_id") or ""), str(body.get("pin") or ""))
            self._send_json(status, payload)
            return
        if route_name == "auth_logout":
            token = ((self.headers.get("X-RVT-Auth") or self.headers.get("X-RVT-Token") or "") if getattr(self, "headers", None) else "").strip()
            if not token:
                token = (parse_qs(urlparse(self.path).query).get("token") or [""])[-1].strip()
            # Logout revokes ONLY the presented session token, plus that
            # operator's short-lived SSE tokens (otherwise an SSE token outlives
            # the session for its ~30s TTL). It must NOT drop the operator's OTHER
            # active sessions — logging out one tab/device must not log out the
            # rest. (reset-pin / host-reset use the global _invalidate_operator_sessions.)
            # All operator_sessions / sse_tokens access goes through _OPERATOR_LOCK.
            with _OPERATOR_LOCK:
                if token and hasattr(self.server, "operator_sessions") and token in self.server.operator_sessions:
                    operator_id = self.server.operator_sessions.pop(token, {}).get("operator_id")
                    _invalidate_operator_sse_tokens(self.server, operator_id)
            self._send_json(200, {"ok": True})
            return
        if route_name == "auth_sse_token":
            token = secrets.token_urlsafe(24)
            # Bind the SSE token to the requesting operator so session
            # invalidation (reset / host-reset / logout) can also drop it.
            # _require_control_auth set current_operator_id when a valid operator
            # session authorised this request; bootstrap/no-operator => None.
            operator_id = getattr(self, "current_operator_id", None)
            now = time.time()
            with _OPERATOR_LOCK:
                if not hasattr(self.server, "sse_tokens"):
                    self.server.sse_tokens = {}
                # Reap expired SSE tokens to prevent unbounded memory growth
                expired = [t for t, s in self.server.sse_tokens.items() if now >= s.get("expires_at", 0.0)]
                for t in expired:
                    self.server.sse_tokens.pop(t, None)
                self.server.sse_tokens[token] = {
                    "expires_at": now + 30.0,
                    "operator_id": operator_id,
                }
            self._send_json(200, {"sse_token": token})
            return
        if route_name == "auth_reset_pin":
            status, payload = _reset_pin_with_recovery(
                self.server,
                str(body.get("operator_id") or ""),
                str(body.get("recovery_code") or ""),
                str(body.get("new_pin") or ""),
            )
            self._send_json(status, payload, cache_control="no-store")
            return
        if route_name == "auth_host_reset":
            client_host = str(self.client_address[0] if self.client_address else "")
            if client_host not in {"127.0.0.1", "::1", "localhost"}:
                self._send_json(403, {"ok": False, "error": {"code": "LOOPBACK_ONLY", "message": "host-reset is loopback-only"}}, cache_control="no-store")
                return
            status, payload = _host_reset_pin(
                self.server,
                str(body.get("operator_id") or ""),
                str(body.get("new_pin") or ""),
            )
            self._send_json(status, payload, cache_control="no-store")
            return
        if route_name == "defaults":
            current = _effective_defaults(self.server.sessions_root)
            current.update({k: v for k, v in body.items() if k in current})
            save_json(current, str(Path(self.server.sessions_root) / ".user_defaults.json"))
            self._send_json(200, current)
            return
        if route_name == "preflight_one":
            check_id = path.rsplit("/", 1)[-1]
            if check_id in {"serial_port_probe", "ble_device_probe"} and self.server.supervisor.current():
                self._send_json(409, {"ok": False, "error": {"code": "SESSION_IN_PROGRESS", "message": "hardware preflight is blocked while a session is active"}})
                return
            self._send_json(200, _run_preflight_check(check_id, sessions_root=self.server.sessions_root, **body))
            return
        if route_name == "session_start":
            radar_port = str(body.get("radar_port") or DEFAULT_RADAR_PORT).strip() or DEFAULT_RADAR_PORT
            if str(radar_port).strip().lower() in {"auto", "autodetect", "auto-detect"}:
                radar_port = _auto_detect_radar_port(DEFAULT_RADAR_PORT)
            ble_address = str(body.get("ble_address") or DEFAULT_BLE_ADDRESS).strip() or DEFAULT_BLE_ADDRESS
            header_key = (
                self.headers.get("Idempotency-Key")
                or self.headers.get("X-RVT-Idempotency-Key")
                or ""
            ).strip()
            body_key = str(body.get("idempotency_key") or "").strip()
            if header_key and body_key and header_key != body_key:
                self._send_json(
                    400,
                    _api_error(
                        "IDEMPOTENCY_KEY_MISMATCH",
                        "body idempotency_key and Idempotency-Key header must match",
                    ),
                )
                return
            idempotency_key = body_key or header_key or None
            client_compatibility = _validate_client_compatibility(
                body,
                product_version=VERSION,
                dashboard_version=DASHBOARD_VERSION,
                control_api_schema=CONTROL_API_SCHEMA_VERSION,
                study_session_schema=STUDY_SESSION_SCHEMA_VERSION,
                session_manifest_schema=SESSION_MANIFEST_SCHEMA_VERSION,
                serial_width_expected=EXPECTED_RADAR_LOG_COLUMN_COUNT,
            )
            if client_compatibility["blocks_start"]:
                self._send_json(
                    409,
                    _api_error(
                        "RELEASE_COMPATIBILITY_MISMATCH",
                        "Dashboard and trainer release contracts are incompatible. Reload the PWA or restart the native app and trainer, then retry.",
                        compatibility=client_compatibility,
                        details={"compatibility": client_compatibility},
                        remediation="Reload the PWA or restart the native app and trainer, then retry.",
                    ),
                )
                return
            if str(body.get("study_classification") or body.get("study_mode") or "").lower() == "confirmatory":
                try:
                    monolith_override("_require_collection_authorization", _require_collection_authorization)(self.server.sessions_root)
                except CollectionAuthorizationError as exc:
                    self._send_json(412, _api_error(exc.code, str(exc)))
                    return
            try:
                study_assignment = _validate_study_assignment(
                    {
                        **body,
                        "duration_s": body.get("duration_s"),
                    },
                    sessions_root=self.server.sessions_root,
                )
            except StudyContractError as exc:
                self._send_json(400, _api_error(exc.code, str(exc)))
                return
            duration_s = body.get(
                "duration_s",
                study_assignment.get("planned_duration_s"),
            )
            # Session capture is model-neutral unless a concrete bundle is
            # attached.  This protects provenance from stale client state and
            # keeps model-family choice inside the evidence-bound Model Lab
            # workflow.
            model_bundle = str(body.get("model_bundle") or "").strip()
            model_family = (
                str(body.get("model_family") or "").strip() or None
                if model_bundle
                else None
            )
            advanced = body.get("advanced") if isinstance(body.get("advanced"), dict) else {}
            timeout_s = float(body.get("timeout_s", 30.0) or 30.0)
            idempotency_payload = {
                "server_product_version": VERSION,
                "duration_s": duration_s,
                "radar_port": radar_port,
                "ble_address": ble_address,
                "ble_profile": body.get("ble_profile", "ailink_oximeter"),
                "timeout_s": timeout_s,
                "subject_label": body.get("subject_label"),
                "operator_label": body.get("operator_label"),
                "subject_profile_id": body.get("subject_profile_id", "adult_default"),
                "notify_char": advanced.get("notify_char"),
                "dashboard_refresh_s": advanced.get("dashboard_refresh_s"),
                **study_assignment,
                "model_family": model_family,
                "model_bundle": model_bundle or None,
                "client_compatibility": client_compatibility,
            }

            try:
                replay = self.server.supervisor.lookup_idempotent_start(
                    idempotency_key,
                    idempotency_payload,
                )
                if replay is not None:
                    self._send_json(200, replay)
                    return
                self.server.supervisor.lookup_logical_trial_start(
                    study_assignment.get("logical_trial_id")
                )
            except StartIdempotencyError as exc:
                status = 400 if exc.code == "INVALID_IDEMPOTENCY_KEY" else 409
                if exc.code == "PREFLIGHT_FAILED":
                    status = 424
                elif exc.code in {"PREFLIGHT_ERROR", "SPAWN_ERROR", "SPAWN_TIMEOUT"}:
                    status = 500
                error_details = (
                    dict(exc.details) if isinstance(exc.details, dict) else {}
                )
                self._send_json(
                    status,
                    _api_error(exc.code, str(exc), **error_details),
                )
                return

            if self.server.supervisor.current():
                self._send_json(409, {"ok": False, "error": {"code": "SESSION_IN_PROGRESS", "message": "active session already running"}})
                return

            def run_start_preflight():
                try:
                    report = monolith_override("_run_preflight_all", _run_preflight_all)(
                        sessions_root=self.server.sessions_root,
                        port=radar_port,
                        address=ble_address,
                        include=SESSION_START_PREFLIGHT_IDS,
                    )
                except Exception as exc:
                    raise SessionStartPreflightError(
                        "PREFLIGHT_ERROR",
                        str(exc),
                        http_status=500,
                    ) from exc
                failed = _session_start_blocking_failures(report)
                if failed:
                    raise SessionStartPreflightError(
                        "PREFLIGHT_FAILED",
                        "one or more required start checks failed",
                        details={"failed": failed},
                        http_status=424,
                    )
                return report

            try:
                result = self.server.supervisor.start(
                    duration_s=duration_s,
                    radar_port=radar_port,
                    ble_address=ble_address,
                    ble_profile=body.get("ble_profile", "ailink_oximeter"),
                    timeout_s=timeout_s,
                    idempotency_key=idempotency_key,
                    idempotency_payload=idempotency_payload,
                    preflight=run_start_preflight,
                    subject_label=body.get("subject_label"),
                    operator_label=body.get("operator_label"),
                    subject_profile_id=body.get("subject_profile_id", "adult_default"),
                    notify_char=advanced.get("notify_char"),
                    dashboard_refresh_s=advanced.get("dashboard_refresh_s"),
                    **study_assignment,
                    model_family=model_family,
                    model_bundle=model_bundle or None,
                    client_compatibility=client_compatibility,
                )
                self._send_json(200, result)
            except SessionStartPreflightError as e:
                details = dict(e.details) if isinstance(e.details, dict) else {}
                details["failed_session_id"] = e.failed_session_id
                self._send_json(
                    e.http_status,
                    _api_error(e.code, str(e), **details),
                )
            except StartIdempotencyError as e:
                status = 400 if e.code == "INVALID_IDEMPOTENCY_KEY" else 409
                if e.code in {"PREFLIGHT_FAILED"}:
                    status = 424
                elif e.code in {"PREFLIGHT_ERROR", "SPAWN_ERROR", "SPAWN_TIMEOUT"}:
                    status = 500
                error_details = dict(e.details) if isinstance(e.details, dict) else {}
                self._send_json(status, _api_error(e.code, str(e), **error_details))
            except RuntimeError as e:
                code = "SESSION_IN_PROGRESS" if "SESSION_IN_PROGRESS" in str(e) else "SPAWN_ERROR"
                self._send_json(409 if code == "SESSION_IN_PROGRESS" else 500, {"ok": False, "error": {"code": code, "message": str(e)}})
            except TimeoutError as e:
                self._send_json(500, {"ok": False, "error": {"code": "SPAWN_TIMEOUT", "message": str(e)}})
            return
        if route_name == "session_stop":
            try:
                self._send_json(200, self.server.supervisor.stop(reason=str(body.get("reason", "user_request"))))
            except RuntimeError as e:
                message = str(e)
                no_active = message == "no active session"
                self._send_json(
                    404 if no_active else 500,
                    _api_error(
                        "NO_ACTIVE_SESSION" if no_active else "SESSION_STOP_FAILED",
                        message,
                    ),
                )
            return
        if route_name == "session_annotate":
            cur = self.server.supervisor.current()
            if not cur:
                self._send_json(404, {"ok": False, "error": {"code": "NO_ACTIVE_SESSION", "message": "no active session"}})
                return
            notes_path = Path(str(cur.get("session_dir", ""))) / "session_notes.json"
            existing = _read_json_if_exists(str(notes_path)) or {}
            notes = existing.get("notes") if isinstance(existing.get("notes"), list) else []
            entry = {
                "id": f"note_{int(time.time() * 1000)}",
                "t_s": body.get("t_s"),
                "note": _sanitize_user_string(body.get("note", "")),
                "created_at": _iso_now(),
            }
            notes.append(entry)
            existing.update({"schema_version": SESSION_NOTES_SCHEMA_VERSION, "session_id": cur.get("session_id"), "notes": notes, "updated_at": _iso_now()})
            save_json(existing, str(notes_path))
            self._send_json(200, {"ok": True, "entry": entry})
            return
        if route_name == "session_annotations":
            cur = self.server.supervisor.current()
            if not cur:
                self._send_json(404, {"ok": False, "error": {"code": "NO_ACTIVE_SESSION", "message": "no active session"}})
                return
            session_id = str(cur.get("session_id") or "")
            session_dir = Path(str(cur.get("session_dir", "")))
            action = str(body.get("action") or "upsert").lower()
            chart_key = str(body.get("chart_key") or body.get("chartKey") or "chart")
            annotation = body.get("annotation") if isinstance(body.get("annotation"), dict) else {}
            entry = _upsert_session_annotation(session_dir, session_id, chart_key, annotation, action=action)
            self._send_json(200, {"ok": True, "entry": entry, "action": action})
            return
        if route_name == "session_analyse":
            sid = unquote(path.split("/")[3])
            try:
                self._send_json(200, _rerun_session_analysis(str(_session_path(self.server.sessions_root, sid))))
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
            return
        if path.startswith("/api/"):
            self._send_json(404, _api_error("NOT_FOUND", "API route not found"))
            return
        self._send_json(404, _api_error("NOT_FOUND", "route not found"))

    def do_PUT(self):
        if self._reject_untrusted():
            return
        if not self._require_control_auth():
            return
        path = urlparse(self.path).path
        route_spec = _match_route("PUT", path)
        route_name = route_spec.name if route_spec is not None else None
        body = self._read_body()
        if body is None:
            return
        if route_name == "subject_profiles_put":
            try:
                saved = _save_subject_profiles(self.server.sessions_root, body)
                self._send_json(200, saved)
            except (TypeError, ValueError) as exc:
                self._send_json(400, _api_error("INVALID_SUBJECT_PROFILES", str(exc)))
            return
        if route_name == "study_protocol":
            try:
                protocol = _save_study_protocol(
                    self.server.sessions_root,
                    body,
                    actor=body.get("actor"),
                )
            except (TypeError, ValueError) as exc:
                status = 409 if "locked" in str(exc).lower() else 400
                self._send_json(
                    status,
                    _api_error("INVALID_STUDY_PROTOCOL", str(exc)),
                )
                return
            self._send_json(200, {"ok": True, "protocol": protocol})
            return
        if route_name == "participant_status":
            participant_id = unquote(path.rsplit("/", 1)[-1])
            try:
                profile = _update_participant_status(
                    self.server.sessions_root,
                    participant_id,
                    body.get("status"),
                    actor=body.get("actor"),
                    reason=body.get("reason"),
                    consent_revision=body.get("consent_revision"),
                    withdrawal_disposition=body.get("withdrawal_disposition"),
                    authority_reference=body.get("authority_reference"),
                    evidence_ref=body.get("evidence_ref"),
                )
                self._send_json(200, {
                    "ok": True,
                    "schema_version": PARTICIPANT_REGISTRY_SCHEMA_VERSION,
                    "profile": profile,
                })
            except StudyContractError as exc:
                status = 404 if exc.code == "PARTICIPANT_NOT_FOUND" else 400
                self._send_json(status, _api_error(exc.code, str(exc)))
            return
        if route_name == "session_notes_put":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            review_summary = _sanitize_user_string(body.get("review_summary", ""), 4000).strip()
            notes_path = root / "session_notes.json"
            existing = _read_json_if_exists(str(notes_path)) or {}
            existing.update({
                "schema_version": SESSION_NOTES_SCHEMA_VERSION,
                "session_id": sid,
                "review_summary": review_summary,
                "notes": existing.get("notes") if isinstance(existing.get("notes"), list) else [],
                "updated_at": _iso_now(),
            })
            save_json(existing, str(notes_path))
            self._send_json(200, existing)
            return
        if route_name == "session_signoff_put":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            operator_name = _sanitize_user_string(body.get("operator_name", ""), 120).strip()
            initials = str(body.get("initials") or "").strip().upper()
            validation_comment = _sanitize_user_string(body.get("validation_comment", ""), 500).strip()
            if not operator_name or not re.fullmatch(r"[A-Z]{2,5}", initials):
                self._send_json(400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "operator_name and 2-5 uppercase initials are required"}})
                return
            signoff = {
                "schema_version": SESSION_SIGNOFF_SCHEMA_VERSION,
                "session_id": sid,
                "operator_name": operator_name,
                "initials": initials,
                "validation_comment": validation_comment,
                "signed_at": _iso_now(),
                "updated_at": _iso_now(),
            }
            save_json(signoff, str(root / "session_signoff.json"))
            self._send_json(200, signoff)
            return
        if route_name == "session_tags":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            manifest_path = root / "session_manifest.json"
            manifest = _read_json_if_exists(str(manifest_path)) or {}
            tags = body.get("tags", [])
            if not isinstance(tags, list):
                self._send_json(400, {"ok": False, "error": {"code": "VALIDATION_FAILED", "message": "tags must be a list"}})
                return
            manifest["schema_version"] = SESSION_MANIFEST_SCHEMA_VERSION
            manifest["manifest_version"] = SESSION_MANIFEST_VERSION
            manifest["tags"] = [_sanitize_user_string(t, 80).strip() for t in tags if str(t).strip()]
            manifest["updated_at"] = _iso_now()
            save_json(manifest, str(manifest_path))
            self._send_json(200, {"ok": True, "session_id": sid, "tags": manifest["tags"]})
            return
        self._send_json(404, _api_error("NOT_FOUND", "API route not found"))

    def do_DELETE(self):
        if self._reject_untrusted():
            return
        if not self._require_control_auth():
            return
        path = urlparse(self.path).path
        route_spec = _match_route("DELETE", path)
        route_name = route_spec.name if route_spec is not None else None
        if route_name == "study_analysis_cancel":
            job_id = unquote(path.rsplit("/", 1)[-1])
            _request_analysis_cancel(
                self.server.sessions_root,
                job_id,
                reason="Cancelled by operator.",
            )
            with _STUDY_ANALYSIS_LOCK:
                proc = _STUDY_ANALYSIS_PROCS.get(job_id)
            if proc is not None and proc.poll() is None:
                try:
                    proc.terminate()
                    proc.wait(timeout=3.0)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
            job = _cancel_analysis_job(
                self.server.sessions_root,
                job_id,
                reason="Cancelled by operator.",
            )
            if job is None:
                self._send_json(404, _api_error("STUDY_ANALYSIS_NOT_FOUND", "study analysis job not found"))
            else:
                self._send_json(200, {"ok": True, "job": job})
            return
        if route_name == "session_delete":
            sid = unquote(path.split("/")[3])
            try:
                root = _session_path(self.server.sessions_root, sid)
            except Exception:
                self._send_json(404, {"ok": False, "error": {"code": "SESSION_NOT_FOUND", "message": "session not found"}})
                return
            active = self.server.supervisor.current() if getattr(self.server, "supervisor", None) else None
            active_dir = os.path.abspath(str((active or {}).get("session_dir", ""))) if isinstance(active, dict) else ""
            if active_dir and active_dir == os.path.abspath(str(root)):
                self._send_json(409, {"ok": False, "error": {"code": "SESSION_ACTIVE", "message": "stop the active session before deleting it"}})
                return
            lock = _read_json_if_exists(_lock_path(self.server.sessions_root)) or {}
            lock_dir = os.path.abspath(str(lock.get("session_dir", ""))) if isinstance(lock, dict) else ""
            if lock_dir and lock_dir == os.path.abspath(str(root)):
                self._send_json(409, {"ok": False, "error": {"code": "SESSION_LOCKED", "message": "session lock is active; stop or clear the session before deleting it"}})
                return
            trash = Path(self.server.sessions_root) / ".trash"
            trash.mkdir(parents=True, exist_ok=True)
            target = trash / sid
            if target.exists():
                target = trash / f"{sid}_{int(time.time())}"
            shutil.move(str(root), str(target))
            self._send_json(200, {"ok": True, "session_id": sid, "trashed_path": str(target), "retention_hint": "soft-deleted; clean .trash after local retention review"})
            return
        self._send_json(404, _api_error("NOT_FOUND", "API route not found"))


class _ControlServer:
    def __init__(
        self,
        host: str,
        port: int,
        sessions_root: str,
        cors_origin: str = "",
        mock: bool = False,
        bind_mode: str = "local",
        tls_cert: Optional[str] = None,
        tls_key: Optional[str] = None,
        tls_trusted: bool = False,
    ):
        self.sessions_root = os.path.abspath(sessions_root)
        os.makedirs(self.sessions_root, exist_ok=True)
        _cleanup_json_temp_files(self.sessions_root)
        handler = partial(_ControlHandler, directory=str(_REPO_ROOT))
        self.httpd: _ControlHTTPServer = cast(_ControlHTTPServer, ThreadingHTTPServer((host, int(port)), handler))
        if tls_cert and tls_key:
            _ensure_self_signed_cert(tls_cert, tls_key, host if host not in {"0.0.0.0", "::"} else _guess_lan_ip())
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(tls_cert, tls_key)
            self.httpd.socket = ctx.wrap_socket(self.httpd.socket, server_side=True)
        self.httpd.sessions_root = self.sessions_root
        self.httpd.supervisor = _SessionSupervisor(self.sessions_root)
        self.httpd.started_at = _iso_now()
        self.httpd.cors_origin = cors_origin
        self.httpd.mock = bool(mock)
        self.httpd.bind_mode = bind_mode
        self.httpd.tls_enabled = bool(tls_cert and tls_key)
        self.httpd.tls_trusted = bool(tls_trusted)
        self.httpd.advertised_host = _guess_lan_ip() if bind_mode == "lan" else "127.0.0.1"
        self.httpd.auth_tokens = set()
        self.httpd.pair_pins = {}
        self.httpd.pair_exchange_failures = {}
        self.httpd.active_pin = ""
        self.httpd.active_pin_expires_at = 0.0
        self.httpd.operator_sessions = {}
        self.httpd.sse_tokens = {}
        if bind_mode == "lan":
            _make_pair_pin(self.httpd)
        origin = _advertised_origin(self.httpd)
        connect_src = "'self'" if bind_mode == "local" else f"'self' {origin}"
        self.httpd.content_security_policy = (
            "default-src 'self'; "
            "script-src 'self' 'unsafe-inline'; "
            "style-src 'self' 'unsafe-inline'; "
            "img-src 'self' data: blob:; "
            f"connect-src {connect_src}; "
            "font-src 'self'; worker-src 'self'; manifest-src 'self'; "
            "frame-ancestors 'none'; base-uri 'self'"
        )
        self.supervisor = self.httpd.supervisor
        self.sentinel = Path(self.sessions_root).parent / "control_server.json"
        save_json({"pid": os.getpid(), "port": self.httpd.server_port, "origin": origin, "started_at": self.httpd.started_at}, str(self.sentinel))
        _append_trainer_log(f"[CONTROL] server started on {host}:{self.httpd.server_port}")
        self._lifecycle_lock = threading.RLock()
        self._serving = False
        self._http_closed = False
        self._stopped = False
        self.thread = None
        original_serve_forever = self.httpd.serve_forever
        original_shutdown = self.httpd.shutdown

        def _serve_forever_with_flag(*args, **kwargs):
            self._serving = True
            try:
                return original_serve_forever(*args, **kwargs)
            finally:
                self._serving = False

        def _shutdown_with_cleanup():
            try:
                if self._serving:
                    original_shutdown()
            finally:
                try:
                    self.sentinel.unlink()
                except FileNotFoundError:
                    pass
        setattr(self.httpd, "serve_forever", _serve_forever_with_flag)
        setattr(self.httpd, "shutdown", _shutdown_with_cleanup)

    def start(self):
        with self._lifecycle_lock:
            if self._http_closed or self._stopped:
                raise RuntimeError("CONTROL_SERVER_CLOSED: server cannot be restarted")
            if self.thread is not None and self.thread.is_alive():
                return
            self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
            self.thread.start()

    def stop(self):
        with self._lifecycle_lock:
            if self._stopped:
                return

            # Close the start gate before the HTTP accept loop so an in-flight
            # request either finishes under the supervisor lock or observes a
            # deterministic SUPERVISOR_CLOSING error.
            self.supervisor.close_start_gate()
            shutdown_error = None
            supervisor_error = None
            try:
                if not self._http_closed:
                    try:
                        self.httpd.shutdown()
                    except Exception as exc:
                        shutdown_error = exc
                try:
                    self.supervisor.stop(
                        reason="server_shutdown",
                        auto_analyse=False,
                        missing_ok=True,
                    )
                except Exception as exc:
                    supervisor_error = exc
            finally:
                if not self._http_closed:
                    try:
                        self.httpd.server_close()
                    finally:
                        self._http_closed = True

            if supervisor_error is not None:
                raise supervisor_error
            if shutdown_error is not None:
                raise shutdown_error
            self._stopped = True


def _start_control_server(args):
    sessions_root = os.path.abspath(str(getattr(args, "sessions_root", "sessions")))
    sentinel = Path(sessions_root).parent / "control_server.json"
    if sentinel.exists():
        existing = _read_json_if_exists(str(sentinel)) or {}
        if monolith_override("_pid_alive", _pid_alive)(existing.get("pid")):
            raise RuntimeError(f"control server already running on port {existing.get('port')}")
        try:
            sentinel.unlink()
        except Exception:
            pass
    bind_mode = str(getattr(args, "bind", "local") or "local")
    requested_host = str(getattr(args, "host", "") or "")
    host = requested_host or ("0.0.0.0" if bind_mode == "lan" else "127.0.0.1")
    port_arg = getattr(args, "control_port", 8765)
    start_port = int(8765 if port_arg is None else port_arg)
    cors_origin = str(getattr(args, "cors_origin", "") or "")
    if bind_mode == "lan" and cors_origin == "*":
        if not bool(getattr(args, "allow_wildcard_cors_lan", False)):
            raise RuntimeError(
                "Refusing --cors-origin '*' with --bind lan. "
                "Use --allow-wildcard-cors-lan only for an isolated lab network."
            )
        print(_yellow("[WARN] Control server is bound to LAN interfaces with a wildcard CORS origin."))
        print(_yellow("       This allows cross-origin requests from any website opened on your browser."))
    mock = bool(getattr(args, "mock", False))
    tls_arg = getattr(args, "tls", None)
    tls_cert = None
    tls_key = None
    if tls_arg is not None:
        if isinstance(tls_arg, list) and len(tls_arg) >= 2:
            tls_cert, tls_key = tls_arg[0], tls_arg[1]
        else:
            tls_dir = _REPO_ROOT / ".rvt_tls"
            tls_cert, tls_key = str(tls_dir / "cert.pem"), str(tls_dir / "key.pem")
    tls_trusted = bool(getattr(args, "tls_trusted", False))
    last_error = None
    ports = [0] if start_port == 0 else [start_port]
    for port in ports:
        try:
            return _ControlServer(host, port, sessions_root, cors_origin=cors_origin, mock=mock, bind_mode=bind_mode, tls_cert=tls_cert, tls_key=tls_key, tls_trusted=tls_trusted)
        except OSError as e:
            last_error = e
            if start_port == 0:
                break
    if start_port != 0:
        raise RuntimeError(f"Port {start_port} in use. Stop the other instance or pass --port <N>.")
    raise RuntimeError(f"could not bind control server on an automatic port: {last_error}")


def cmd_serve(args):
    try:
        server = _start_control_server(args)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)
    url = f"{_advertised_origin(server.httpd)}/?v={int(time.time())}"
    print(f"[CONTROL] {url}")
    print(f"[CONTROL] support matrix: {_advertised_origin(server.httpd)}/about")
    if getattr(server.httpd, "bind_mode", "local") == "lan":
        pin = getattr(server.httpd, "active_pin", "")
        print(f"[PAIR] open {_advertised_origin(server.httpd)}/pair")
        print(f"[PAIR] one-time PIN: {pin} (expires in 5 minutes; consumed after first exchange)")
    if not getattr(args, "no_browser", False):
        try:
            import webbrowser
            webbrowser.open(url)
        except Exception as e:
            warn(f"Could not open dashboard in browser: {e}")
    try:
        server.httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
