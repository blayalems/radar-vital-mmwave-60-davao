"""BLE transport helpers, GATT packet parsing, and oximeter logging."""

from __future__ import annotations

import asyncio
import os
import re
import sys
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

from rvt_trainer.api.common import (
    atomic_write_json as save_json,
    nan_safe,
    read_json_if_exists as _read_json_if_exists,
)
from rvt_trainer.constants import (
    BLE_PI_QUALITY_THRESHOLD,
    VERSION,
    _bold,
    _dim,
    _first_existing,
    _green,
    _iso_now,
    _yellow,
)

_MIN_SCAN_TIMEOUT_S = 0.5
_MAX_SCAN_TIMEOUT_S = 12.0


def scan_ble_devices_payload(timeout_s: float = 3.0) -> Dict[str, object]:
    """Scan for nearby BLE peripherals and return a UI-ready payload.

    The function never raises: hardware/driver failures are captured and
    returned as ``{"ok": False, ..., "error": {...}}`` so the HTTP layer
    can always emit a clean ``200`` with a structured body.
    """
    try:
        from bleak import BleakScanner

        timeout = max(_MIN_SCAN_TIMEOUT_S, min(float(timeout_s), _MAX_SCAN_TIMEOUT_S))
        devices = asyncio.run(BleakScanner.discover(timeout=timeout))
        out: List[Dict[str, object]] = []
        for d in devices or []:
            out.append(
                {
                    "address": str(getattr(d, "address", "") or ""),
                    "name": str(getattr(d, "name", "") or "BLE device"),
                    "rssi": getattr(d, "rssi", None),
                }
            )
        return {"ok": True, "devices": out, "count": len(out), "scanned_at": _iso_now()}
    except Exception as e:
        return {
            "ok": False,
            "devices": [],
            "count": 0,
            "error": {"code": "BLE_SCAN_FAILED", "message": str(e)},
            "scanned_at": _iso_now(),
        }


def _normalize_ble_address(addr: Optional[str]) -> Optional[str]:
    if addr is None:
        return None
    s = str(addr).strip().upper()
    if not s:
        return None
    m = re.search(r"DEV[_-]?([0-9A-F]{12})", s)
    if m:
        s = m.group(1)
    s = re.sub(r"[^0-9A-F]", "", s)
    if len(s) == 12:
        return ":".join(s[i:i+2] for i in range(0, 12, 2))
    return addr


def _pick_notify_characteristics(services, preferred_uuid: Optional[str] = None) -> List[str]:
    preferred = preferred_uuid.lower() if preferred_uuid else None
    notify_chars: List[str] = []

    def score_uuid(uuid: str, props: set) -> Tuple[int, str]:
        u = uuid.lower()
        score = 0

        # Strongly prefer explicitly requested UUID.
        if preferred and u == preferred:
            score += 10000
        elif preferred and u.endswith(preferred.replace('-', '').lower()):
            score += 9000

        # Known common data-stream candidates.
        if u == "6e400003-b5a3-f393-e0a9-e50e24dcca9e":
            score += 8000
        if u == "0000ffe2-0000-1000-8000-00805f9b34fb":
            score += 7000
        if u == "0000ffe3-0000-1000-8000-00805f9b34fb":
            score += 6500
        if u == "49535343-1e4d-4bd9-ba61-23c647249616":
            score += 7500
        if u == "49535343-aca3-481c-91ec-d85e28a60318":
            score += 6000

        # Prefer vendor-specific / custom UUIDs over standard GATT housekeeping.
        if u.startswith("49535343-"):
            score += 4000
        if u.startswith("0000ffe"):
            score += 3500

        # Prefer notify over indicate-only.
        if "notify" in props:
            score += 500
        if "indicate" in props:
            score += 100

        # Penalize standard characteristics that are usually not the live vital stream.
        if u == "00002a05-0000-1000-8000-00805f9b34fb":  # Service Changed
            score -= 100000
        if u == "00002a19-0000-1000-8000-00805f9b34fb":  # Battery Level
            score -= 5000
        if u == "00002a07-0000-1000-8000-00805f9b34fb":  # Tx Power Level
            score -= 5000

        return score, u

    for service in services:
        for char in getattr(service, "characteristics", []):
            uuid = str(getattr(char, "uuid", ""))
            props = {str(p).lower() for p in getattr(char, "properties", [])}
            if "notify" in props or "indicate" in props:
                notify_chars.append(uuid)

    if not notify_chars:
        return []

    # Re-score with actual props preserved to break ties correctly.
    scored = []
    for service in services:
        for char in getattr(service, "characteristics", []):
            uuid = str(getattr(char, "uuid", ""))
            props = {str(p).lower() for p in getattr(char, "properties", [])}
            if uuid in notify_chars:
                scored.append((score_uuid(uuid, props), uuid))
    ordered = [uuid for _, uuid in sorted(scored, key=lambda x: x[0], reverse=True)]

    deduped: List[str] = []
    seen = set()
    for uuid in ordered:
        if uuid not in seen:
            deduped.append(uuid)
            seen.add(uuid)
    return deduped


def _parse_ble_vitals_packet(
    data: bytes,
    hr_byte_index: Optional[int] = None,
    spo2_byte_index: Optional[int] = None,
    rr_byte_index: Optional[int] = None,
    pi_byte_index: Optional[int] = None,
    hr_scale: float = 1.0,
    spo2_scale: float = 1.0,
    rr_scale: float = 1.0,
    pi_scale: float = 1.0,
    last_hr: Optional[float] = None,
    allow_heuristic: bool = False,
) -> Tuple[
    Optional[float], Optional[float], Optional[float], Optional[float],
    Optional[int], Optional[int], Optional[int], Optional[int]
]:
    hr = rr = spo2 = pi = None
    hr_idx = spo2_idx = rr_idx = pi_idx = None
    payload = list(data)

    def pick_value(index: Optional[int], scale: float, valid_min: float, valid_max: float):
        if index is None:
            return None, None
        if index < 0 or index >= len(payload):
            return None, None
        raw_val = int(payload[index])
        val = float(raw_val) * float(scale)
        if valid_min <= val <= valid_max:
            return float(val), index
        return None, index

    hr, hr_idx = pick_value(hr_byte_index, hr_scale, 30, 220)
    rr, rr_idx = pick_value(rr_byte_index, rr_scale, 4, 60)
    spo2, spo2_idx = pick_value(spo2_byte_index, spo2_scale, 70, 100)
    pi, pi_idx = pick_value(pi_byte_index, pi_scale, 0, 40)

    if allow_heuristic and hr is None:
        candidates = []
        seen = set()
        priority = []
        if len(payload) > 4:
            priority.extend([4, 5, 3])
        if len(payload) > 8:
            priority.extend([8, 9, 10])
        if len(payload) > 1:
            priority.append(1)
        if len(payload) > 0:
            priority.append(len(payload) - 1)
        for idx in priority:
            if idx in seen or idx < 0 or idx >= len(payload):
                continue
            seen.add(idx)
            val = float(int(payload[idx]))
            if 30 <= val <= 220:
                candidates.append((idx, val))
        if candidates:
            if last_hr is not None:
                hr_idx, hr = min(candidates, key=lambda p: abs(p[1] - last_hr))
            else:
                hr_idx, hr = candidates[0]

    if allow_heuristic and spo2 is None:
        for idx, val in enumerate(payload):
            if 70 <= int(val) <= 100:
                spo2 = float(val)
                spo2_idx = idx
                break

    return hr, rr, spo2, pi, hr_idx, rr_idx, spo2_idx, pi_idx


def _hex_to_bytes(raw_hex: str) -> Optional[bytes]:
    try:
        return bytes.fromhex(str(raw_hex).strip())
    except Exception:
        return None


def _prepare_ble_write_ops(write_chars: Optional[Sequence[str]], write_hexes: Optional[Sequence[str]]) -> List[Tuple[str, bytes]]:
    chars = [str(x).strip() for x in (write_chars or []) if str(x).strip()]
    hexes = [str(x).strip() for x in (write_hexes or []) if str(x).strip()]
    if not chars and not hexes:
        return []
    if len(chars) == 1 and len(hexes) > 1:
        chars = chars * len(hexes)
    if len(chars) != len(hexes):
        raise ValueError("--write-char and --write-hex must have equal counts, or provide one --write-char with multiple --write-hex values.")
    ops = []
    for char_uuid, hex_str in zip(chars, hexes):
        cleaned = re.sub(r"[^0-9A-Fa-f]", "", hex_str)
        if len(cleaned) % 2 != 0:
            raise ValueError(f"Write payload must contain whole bytes: {hex_str}")
        try:
            payload = bytes.fromhex(cleaned)
        except ValueError as e:
            raise ValueError(f"Invalid --write-hex payload '{hex_str}': {e}") from e
        ops.append((char_uuid, payload))
    return ops


def _apply_ble_profile_defaults(args):
    """
    Apply BLE profile defaults without clobbering profile-specific scales.

    v7.2.8e keeps the BLE/dashboard truthfulness fixes, corrects the remaining HR/RR gate-reason enum mappings, adds a non-gating physiological ratio review warning, and keeps nearest alignment as the default so stricter backward matching stays opt-in
    before the profile block ran, which prevented the AILink oximeter profile
    from applying its intended RR/PI scale of 0.1 unless the user overrode them
    explicitly on the command line.
    """
    profile = str(getattr(args, "ble_profile", "none") or "none").strip().lower()

    if profile == "ailink_oximeter":
        if not getattr(args, "notify_char", None):
            args.notify_char = "0000ffe2-0000-1000-8000-00805f9b34fb"
        if getattr(args, "hr_byte_index", None) is None:
            args.hr_byte_index = 7
        if getattr(args, "spo2_byte_index", None) is None:
            args.spo2_byte_index = 6
        if getattr(args, "rr_byte_index", None) is None:
            args.rr_byte_index = 11
        if getattr(args, "pi_byte_index", None) is None:
            args.pi_byte_index = 8
        if getattr(args, "hr_scale", None) is None:
            args.hr_scale = 1.0
        if getattr(args, "spo2_scale", None) is None:
            args.spo2_scale = 1.0
        if getattr(args, "rr_scale", None) is None:
            args.rr_scale = 0.1
        if getattr(args, "pi_scale", None) is None:
            args.pi_scale = 0.1

    if getattr(args, "hr_scale", None) is None:
        args.hr_scale = 1.0
    if getattr(args, "spo2_scale", None) is None:
        args.spo2_scale = 1.0
    if getattr(args, "rr_scale", None) is None:
        args.rr_scale = 1.0
    if getattr(args, "pi_scale", None) is None:
        args.pi_scale = 1.0

    return args


def _fmt_ref_value(value: Optional[float], decimals_if_needed: int = 1) -> str:
    if value is None or not np.isfinite(value):
        return ""
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{float(value):.{int(decimals_if_needed)}f}"


def _ble_sender_column(df: pd.DataFrame) -> str:
    if "source_uuid" in df.columns:
        return "source_uuid"
    if "sender" in df.columns:
        return "sender"
    raise ValueError("Raw BLE CSV must contain source_uuid or sender column")


def _score_ble_byte_candidate(row: pd.Series, target: str) -> float:
    unique_count = float(row.get("unique_count", 0.0) or 0.0)
    change_frac = float(row.get("change_frac", 0.0) or 0.0)
    dominant_frac = float(row.get("dominant_frac", 1.0) or 1.0)
    if target == "hr":
        frac = float(row.get("frac_in_hr_range", 0.0) or 0.0)
        center_bonus = float(row.get("frac_in_hr_core", 0.0) or 0.0)
        score = 100.0 * frac + 20.0 * center_bonus + 8.0 * min(unique_count, 20.0) + 30.0 * change_frac - 40.0 * dominant_frac
    elif target == "rr":
        frac = float(row.get("frac_in_rr_range", 0.0) or 0.0)
        scaled = float(row.get("frac_u16le_div10_in_rr_range", 0.0) or row.get("frac_div10_in_rr_range", 0.0) or 0.0)
        score = 100.0 * frac + 80.0 * scaled + 10.0 * min(unique_count, 12.0) + 30.0 * change_frac - 35.0 * dominant_frac
    else:
        frac = float(row.get("frac_in_spo2_range", 0.0) or 0.0)
        score = 100.0 * frac + 6.0 * min(unique_count, 8.0) + 10.0 * change_frac - 20.0 * dominant_frac
    if unique_count <= 1.0:
        score -= 50.0
    return float(score)


def _build_ble_byte_candidates(raw_df: pd.DataFrame) -> pd.DataFrame:
    sender_col = _ble_sender_column(raw_df)
    rows: List[Dict[str, object]] = []
    work = raw_df.copy()
    work["_payload"] = work["raw_hex"].map(_hex_to_bytes)
    work = work[work["_payload"].notna()].copy()
    if len(work) == 0:
        return pd.DataFrame()

    for sender, g in work.groupby(sender_col, sort=False):
        payloads = list(g["_payload"])
        max_len = max(len(p) for p in payloads)
        for idx in range(max_len):
            vals = [p[idx] for p in payloads if idx < len(p)]
            if not vals:
                continue
            s = pd.Series(vals, dtype=float)
            diffs = s.diff().fillna(0.0)
            vc = s.value_counts(dropna=False)
            dominant_val = float(vc.index[0]) if len(vc) else float("nan")
            dominant_frac = float(vc.iloc[0] / len(s)) if len(vc) else float("nan")
            row = {
                "source_uuid": sender,
                "field_type": "u8",
                "index": int(idx),
                "n": int(len(s)),
                "unique_count": int(s.nunique(dropna=True)),
                "min": float(s.min()),
                "max": float(s.max()),
                "mean": float(s.mean()),
                "std": float(s.std(ddof=0)) if len(s) > 1 else 0.0,
                "change_frac": float((diffs != 0).mean()) if len(s) > 1 else 0.0,
                "dominant_value": dominant_val,
                "dominant_frac": dominant_frac,
                "frac_in_hr_range": float(((s >= 30) & (s <= 220)).mean()),
                "frac_in_hr_core": float(((s >= 45) & (s <= 160)).mean()),
                "frac_in_rr_range": float(((s >= 4) & (s <= 60)).mean()),
                "frac_in_spo2_range": float(((s >= 70) & (s <= 100)).mean()),
            }
            row["score_hr"] = _score_ble_byte_candidate(pd.Series(row), "hr")
            row["score_rr"] = _score_ble_byte_candidate(pd.Series(row), "rr")
            row["score_spo2"] = _score_ble_byte_candidate(pd.Series(row), "spo2")
            rows.append(row)
    return pd.DataFrame(rows)


def _build_ble_word_candidates(raw_df: pd.DataFrame) -> pd.DataFrame:
    sender_col = _ble_sender_column(raw_df)
    rows: List[Dict[str, object]] = []
    work = raw_df.copy()
    work["_payload"] = work["raw_hex"].map(_hex_to_bytes)
    work = work[work["_payload"].notna()].copy()
    if len(work) == 0:
        return pd.DataFrame()

    for sender, g in work.groupby(sender_col, sort=False):
        payloads = list(g["_payload"])
        max_len = max(len(p) for p in payloads)
        for idx in range(max_len - 1):
            vals_le = [p[idx] + 256 * p[idx + 1] for p in payloads if idx + 1 < len(p)]
            vals_be = [256 * p[idx] + p[idx + 1] for p in payloads if idx + 1 < len(p)]
            for endian, vals in [("u16le", vals_le), ("u16be", vals_be)]:
                if not vals:
                    continue
                s = pd.Series(vals, dtype=float)
                diffs = s.diff().fillna(0.0)
                vc = s.value_counts(dropna=False)
                dominant_val = float(vc.index[0]) if len(vc) else float("nan")
                dominant_frac = float(vc.iloc[0] / len(s)) if len(vc) else float("nan")
                s_div10 = s / 10.0
                row = {
                    "source_uuid": sender,
                    "field_type": endian,
                    "index": int(idx),
                    "n": int(len(s)),
                    "unique_count": int(s.nunique(dropna=True)),
                    "min": float(s.min()),
                    "max": float(s.max()),
                    "mean": float(s.mean()),
                    "std": float(s.std(ddof=0)) if len(s) > 1 else 0.0,
                    "change_frac": float((diffs != 0).mean()) if len(s) > 1 else 0.0,
                    "dominant_value": dominant_val,
                    "dominant_frac": dominant_frac,
                    "frac_u16_in_hr_range": float(((s >= 30) & (s <= 220)).mean()),
                    "frac_u16_div10_in_hr_range": float(((s_div10 >= 30) & (s_div10 <= 220)).mean()),
                    "frac_u16_in_rr_range": float(((s >= 4) & (s <= 60)).mean()),
                    "frac_u16_div10_in_rr_range": float(((s_div10 >= 4) & (s_div10 <= 60)).mean()),
                    "frac_u16_in_spo2_range": float(((s >= 70) & (s <= 100)).mean()),
                }
                hr_score = 100.0 * row["frac_u16_in_hr_range"] + 80.0 * row["frac_u16_div10_in_hr_range"] + 8.0 * min(row["unique_count"], 20) + 25.0 * row["change_frac"] - 35.0 * row["dominant_frac"]
                rr_score = 100.0 * row["frac_u16_in_rr_range"] + 100.0 * row["frac_u16_div10_in_rr_range"] + 10.0 * min(row["unique_count"], 15) + 25.0 * row["change_frac"] - 35.0 * row["dominant_frac"]
                spo2_score = 100.0 * row["frac_u16_in_spo2_range"] + 5.0 * min(row["unique_count"], 8) + 10.0 * row["change_frac"] - 20.0 * row["dominant_frac"]
                row["score_hr"] = float(hr_score)
                row["score_rr"] = float(rr_score)
                row["score_spo2"] = float(spo2_score)
                rows.append(row)
    return pd.DataFrame(rows)


def _raw_ble_candidates_for_ref(ref_path: str) -> List[str]:
    base_dir = os.path.dirname(os.path.abspath(ref_path))
    stem = os.path.splitext(os.path.basename(ref_path))[0]
    return [
        os.path.join(base_dir, "ref_ble_raw.csv"),
        os.path.join(base_dir, f"{stem}_ble_raw.csv"),
        os.path.join(base_dir, f"{stem}_raw.csv"),
    ]


def _ble_summary_candidates_for_ref(ref_path: str) -> List[str]:
    base_dir = os.path.dirname(os.path.abspath(ref_path))
    stem = os.path.splitext(os.path.basename(ref_path))[0]
    return [
        os.path.join(base_dir, "ref_ble_summary.json"),
        os.path.join(base_dir, f"{stem}_ble_summary.json"),
        os.path.join(base_dir, f"{stem}_summary.json"),
    ]


def _compute_ble_ref_quality(ref_paths: Sequence[str], pi_threshold: float = BLE_PI_QUALITY_THRESHOLD) -> Dict[str, object]:
    totals = {
        "raw_packets": 0,
        "parsed_rows": 0,
        "summary_parsed_packets": 0,
        "summary_packet_count": 0,
        "pi_rows": 0,
        "pi_below_threshold_rows": 0,
        "packet_loss_estimated_missing": 0.0,
        "packet_loss_expected": 0.0,
    }
    sources = []
    # PR72 session-data audit: packet ratio (parsed_rows/raw_packets) is not a
    # coverage metric — the oximeter emits ~3x more raw notifications than
    # distilled reference rows by protocol design, which pinned the old
    # "coverage" at ~33% and decode_error_pct at ~66% on every session.
    # Track true time coverage (seconds with >=1 parsed row / capture span)
    # and the PI median that the readiness verdict already expects.
    pi_values_all: List[float] = []
    coverage_seconds_covered = 0
    coverage_span_seconds = 0.0
    for ref_path in ref_paths:
        ref_abs = os.path.abspath(ref_path)
        raw_path = _first_existing(_raw_ble_candidates_for_ref(ref_abs))
        summary_path = _first_existing(_ble_summary_candidates_for_ref(ref_abs))
        ref_rows = 0
        pi_rows = 0
        pi_low = 0
        ref_seconds: set = set()
        ref_span_s = 0.0
        try:
            ref_df = pd.read_csv(ref_abs)
            ref_rows = int(len(ref_df))
            if "ref_pi" in ref_df.columns:
                pi = pd.to_numeric(ref_df["ref_pi"], errors="coerce")
                pi_rows = int(pi.notna().sum())
                pi_low = int((pi.dropna() < float(pi_threshold)).sum())
                pi_values_all.extend(float(v) for v in pi.dropna().tolist())
            if "timestamp_ms" in ref_df.columns:
                ref_ts = pd.to_numeric(ref_df["timestamp_ms"], errors="coerce").dropna()
                if len(ref_ts):
                    ref_seconds = set((ref_ts // 1000).astype(int).tolist())
                    if len(ref_ts) >= 2:
                        ref_span_s = float((ref_ts.max() - ref_ts.min()) / 1000.0)
        except Exception:
            pass

        raw_rows = 0
        packet_loss_missing = 0.0
        packet_loss_expected = 0.0
        raw_span_s = 0.0
        if raw_path:
            try:
                raw_df = pd.read_csv(raw_path)
                raw_rows = int(len(raw_df))
                if "timestamp_ms" in raw_df.columns and raw_rows >= 3:
                    ts = pd.to_numeric(raw_df["timestamp_ms"], errors="coerce").dropna().sort_values().to_numpy(dtype=float)
                    if len(ts) >= 2:
                        raw_span_s = float((ts[-1] - ts[0]) / 1000.0)
                    diffs = np.diff(ts)
                    diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
                    if len(diffs):
                        med = float(np.median(diffs))
                        if np.isfinite(med) and med > 0:
                            expected = int(max(raw_rows, round((ts[-1] - ts[0]) / med) + 1))
                            packet_loss_expected = float(expected)
                            packet_loss_missing = float(max(0, expected - raw_rows))
            except Exception:
                pass
        span_s = raw_span_s if raw_span_s > 0 else ref_span_s
        if span_s > 0 and ref_seconds:
            coverage_span_seconds += span_s
            coverage_seconds_covered += min(len(ref_seconds), int(span_s) + 1)

        summary = _read_json_if_exists(summary_path) if summary_path else None
        summary_packets = 0
        summary_parsed = 0
        if isinstance(summary, dict):
            stats = summary.get("stats_by_source", {})
            if isinstance(stats, dict):
                for ent in stats.values():
                    if isinstance(ent, dict):
                        summary_packets += int(float(ent.get("packet_count", 0) or 0))
                        summary_parsed += int(float(ent.get("parsed_count", 0) or 0))
            summary_packets = summary_packets or int(float(summary.get("raw_packets", 0) or 0))
            summary_parsed = summary_parsed or int(float(summary.get("parsed_rows", 0) or 0))

        totals["raw_packets"] += raw_rows
        totals["parsed_rows"] += ref_rows
        totals["summary_packet_count"] += summary_packets
        totals["summary_parsed_packets"] += summary_parsed
        totals["pi_rows"] += pi_rows
        totals["pi_below_threshold_rows"] += pi_low
        totals["packet_loss_estimated_missing"] += packet_loss_missing
        totals["packet_loss_expected"] += packet_loss_expected
        sources.append({
            "ref_csv": ref_abs,
            "raw_csv": os.path.abspath(raw_path) if raw_path else None,
            "summary_json": os.path.abspath(summary_path) if summary_path else None,
            "raw_packets": raw_rows,
            "parsed_rows": ref_rows,
            "summary_packet_count": summary_packets,
            "summary_parsed_packets": summary_parsed,
            "pi_rows": pi_rows,
            "pi_below_threshold_rows": pi_low,
        })

    raw_denom = max(1, int(totals["raw_packets"]))
    # decode_error_pct is only measurable when the BLE logger's summary JSON
    # recorded true per-source parse counts. The old fallback divided distilled
    # reference rows by raw notification count, mislabelling protocol-normal
    # redundant packets as decode errors (~66% on every session).
    if int(totals["summary_packet_count"]) > 0:
        decode_packets = int(totals["summary_packet_count"])
        parsed_packets = int(totals["summary_parsed_packets"])
        decode_error_pct = float(100.0 * max(0, decode_packets - parsed_packets) / max(1, decode_packets))
    else:
        decode_error_pct = float("nan")
    packet_loss_pct = (
        float(100.0 * totals["packet_loss_estimated_missing"] / totals["packet_loss_expected"])
        if totals["packet_loss_expected"] > 0 else float("nan")
    )
    pi_below_pct = (
        float(100.0 * totals["pi_below_threshold_rows"] / totals["pi_rows"])
        if totals["pi_rows"] > 0 else float("nan")
    )
    distilled_pct = float(100.0 * int(totals["parsed_rows"]) / raw_denom) if totals["raw_packets"] else float("nan")
    coverage_pct = (
        float(100.0 * min(1.0, coverage_seconds_covered / max(coverage_span_seconds, 1e-9)))
        if coverage_span_seconds > 0 else float("nan")
    )
    pi_median = float(np.median(pi_values_all)) if pi_values_all else float("nan")
    return {
        "raw_packets": int(totals["raw_packets"]),
        "parsed_rows": int(totals["parsed_rows"]),
        "distilled_rows_pct_of_raw": distilled_pct,
        "coverage_pct": coverage_pct,
        "packet_loss_pct": packet_loss_pct,
        "decode_error_pct": decode_error_pct,
        "pi_median": pi_median,
        "pi_below_threshold_pct": pi_below_pct,
        "pi_threshold": float(pi_threshold),
        "sources": sources,
        "status": (
            "WARN"
            if ((np.isfinite(packet_loss_pct) and packet_loss_pct > 10.0) or
                (np.isfinite(decode_error_pct) and decode_error_pct > 25.0) or
                (np.isfinite(pi_below_pct) and pi_below_pct > 25.0))
            else "OK"
        ),
    }


def cmd_ble_analyse_raw(args):
    os.makedirs(args.out, exist_ok=True)
    raw_df = pd.read_csv(args.raw)
    sender_col = _ble_sender_column(raw_df)
    if args.sender:
        raw_df = raw_df[raw_df[sender_col].astype(str) == str(args.sender)].copy()
    if len(raw_df) == 0:
        raise ValueError("No raw BLE packets matched the requested filter.")

    byte_df = _build_ble_byte_candidates(raw_df)
    word_df = _build_ble_word_candidates(raw_df)

    if len(byte_df):
        byte_df = byte_df.sort_values(["source_uuid", "score_rr", "score_hr", "score_spo2"], ascending=[True, False, False, False]).reset_index(drop=True)
        byte_df.to_csv(os.path.join(args.out, "byte_candidates.csv"), index=False)
    if len(word_df):
        word_df = word_df.sort_values(["source_uuid", "score_rr", "score_hr", "score_spo2"], ascending=[True, False, False, False]).reset_index(drop=True)
        word_df.to_csv(os.path.join(args.out, "word_candidates.csv"), index=False)

    top_summary = {"bytes": {}, "words": {}}
    for name, df, key in [("bytes", byte_df, "index"), ("words", word_df, "index")]:
        if df is None or len(df) == 0:
            continue
        for sender, g in df.groupby("source_uuid", sort=False):
            top_summary[name][sender] = {
                "hr": g.sort_values("score_hr", ascending=False).head(int(args.top_k)).to_dict(orient="records"),
                "rr": g.sort_values("score_rr", ascending=False).head(int(args.top_k)).to_dict(orient="records"),
                "spo2": g.sort_values("score_spo2", ascending=False).head(int(args.top_k)).to_dict(orient="records"),
            }

    summary = {
        "version": VERSION,
        "raw_csv": os.path.abspath(args.raw),
        "rows": int(len(raw_df)),
        "source_column": sender_col,
        "sources": [str(x) for x in pd.Series(raw_df[sender_col]).dropna().astype(str).unique().tolist()],
        "top_k": int(args.top_k),
        "top_summary": nan_safe(top_summary),
    }
    save_json(summary, os.path.join(args.out, "ble_raw_analysis_summary.json"))

    print(_bold("\n=== BLE RAW ANALYSIS ==="))
    print(f"Rows: {len(raw_df)}")
    print(f"Sources: {', '.join(summary['sources'])}")
    if len(byte_df):
        print(_bold("\nTop byte candidates by source:"))
        for sender, g in byte_df.groupby("source_uuid", sort=False):
            print(f"  {sender}")
            top_hr = g.sort_values("score_hr", ascending=False).head(int(args.top_k))
            top_rr = g.sort_values("score_rr", ascending=False).head(int(args.top_k))
            print("    HR:")
            for _, row in top_hr.iterrows():
                print(f"      idx={int(row['index'])} score={row['score_hr']:.1f} unique={int(row['unique_count'])} hr_frac={row['frac_in_hr_range']:.2f} dom={row['dominant_frac']:.2f}")
            print("    RR:")
            for _, row in top_rr.iterrows():
                print(f"      idx={int(row['index'])} score={row['score_rr']:.1f} unique={int(row['unique_count'])} rr_frac={row['frac_in_rr_range']:.2f} dom={row['dominant_frac']:.2f}")
    if len(word_df):
        print(_bold("\nTop 16-bit candidates by source:"))
        for sender, g in word_df.groupby("source_uuid", sort=False):
            print(f"  {sender}")
            top_rr = g.sort_values("score_rr", ascending=False).head(int(args.top_k))
            for _, row in top_rr.iterrows():
                print(f"    {row['field_type']} @ {int(row['index'])}: score={row['score_rr']:.1f} rr_frac={row['frac_u16_in_rr_range']:.2f} rr_div10_frac={row['frac_u16_div10_in_rr_range']:.2f} unique={int(row['unique_count'])}")
    print(f"\n[OUT] Saved raw BLE field analysis to {args.out}")


def cmd_ble_reflog(args):
    """
    Automated BLE reference logger for pulse oximeters.

    Writes the standard reference CSV expected by align/analyse/train, while also
    saving raw packets for protocol reverse-engineering. In v6.2 this command can
    subscribe to all notify-capable characteristics, issue optional initialization
    writes, and run in conservative raw-only mode to avoid fabricating HR/RR fields.
    """
    try:
        import asyncio
        from bleak import BleakScanner, BleakClient
    except ImportError:
        sys.exit("The 'bleak' library is required for BLE logging. Run: pip install bleak")

    import csv

    args = _apply_ble_profile_defaults(args)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    raw_out = args.raw_out or os.path.join(
        os.path.dirname(os.path.abspath(args.out)),
        os.path.splitext(os.path.basename(args.out))[0] + "_ble_raw.csv")
    summary_out = args.summary_out or os.path.join(
        os.path.dirname(os.path.abspath(args.out)),
        os.path.splitext(os.path.basename(args.out))[0] + "_ble_summary.json")

    start_ms = int(time.time() * 1000)
    parsed_count = 0
    raw_count = 0
    last_logged_hr: Optional[float] = None
    last_logged_rr: Optional[float] = None
    last_log_ms = -10**12
    write_ops = _prepare_ble_write_ops(args.write_char, args.write_hex)

    name_hints = [h.strip() for h in (args.name_hint or []) if str(h).strip()] or ["Oxy", "PC-60", "Wilcare", "Oximeter"]
    preferred_notify = args.notify_char or "0000ffe2-0000-1000-8000-00805f9b34fb"
    normalized_address = _normalize_ble_address(args.address)

    print(_bold("\n=== BLE Reference Logger ==="))
    print(f"Reference CSV: {args.out}")
    print(f"Raw packet log: {raw_out}")
    print(f"Summary JSON:   {summary_out}")
    if args.raw_only:
        print(_yellow("Raw-only mode enabled: the trainer will not invent HR/RR from unknown packet bytes."))
    elif not args.auto_parse and args.hr_byte_index is None and args.rr_byte_index is None and args.spo2_byte_index is None and args.pi_byte_index is None:
        print(_yellow("No explicit byte indices supplied. Parsed CSV will stay sparse unless you enable --auto-parse."))
    if str(getattr(args, "ble_profile", "none") or "none").lower() != "none":
        print(_dim(f"BLE profile: {args.ble_profile}"))
    print(_dim("Press Ctrl-C to stop. Keep the oximeter awake with a finger inserted.\n"))

    parsed_f = open(args.out, "w", newline="", encoding="utf-8")
    raw_f = open(raw_out, "w", newline="", encoding="utf-8")
    parsed_writer = csv.DictWriter(parsed_f, fieldnames=[
        "timestamp_ms", "source_uuid", "ref_hr", "ref_rr", "ref_spo2", "ref_pi",
        "hr_byte_index", "rr_byte_index", "spo2_byte_index", "pi_byte_index", "raw_hex",
    ])
    raw_writer = csv.DictWriter(raw_f, fieldnames=[
        "timestamp_ms", "source_uuid", "sender", "packet_len", "raw_hex"
    ])
    parsed_writer.writeheader()
    raw_writer.writeheader()
    parsed_f.flush()
    raw_f.flush()

    stats: Dict[str, Dict[str, Any]] = {}
    parse_source_uuid: Optional[str] = None
    active_notify_chars: List[str] = []

    def record_stat(source_uuid: str, payload: bytes, parsed: bool):
        ent = stats.setdefault(source_uuid, {
            "packet_count": 0,
            "parsed_count": 0,
            "packet_lens": {},
            "first_timestamp_ms": None,
            "last_timestamp_ms": None,
        })
        ent["packet_count"] = int(ent["packet_count"]) + 1
        if parsed:
            ent["parsed_count"] = int(ent["parsed_count"]) + 1
        lens = dict(ent["packet_lens"])
        lens[str(len(payload))] = int(lens.get(str(len(payload)), 0)) + 1
        ent["packet_lens"] = lens
        now_ms = int(time.time() * 1000) - start_ms
        if ent["first_timestamp_ms"] is None:
            ent["first_timestamp_ms"] = now_ms
        ent["last_timestamp_ms"] = now_ms

    def make_notification_handler(source_uuid: str):
        def notification_handler(sender, data: bytearray):
            nonlocal parsed_count, raw_count, last_logged_hr, last_logged_rr, last_log_ms, parse_source_uuid
            t_ms = int(time.time() * 1000) - start_ms
            elapsed = t_ms / 1000.0
            payload = bytes(data)
            raw_hex = payload.hex()
            raw_writer.writerow({
                "timestamp_ms": t_ms,
                "source_uuid": source_uuid,
                "sender": str(sender),
                "packet_len": len(payload),
                "raw_hex": raw_hex,
            })
            raw_f.flush()
            raw_count += 1

            should_try_parse = (not args.raw_only) and (parse_source_uuid is None or source_uuid == parse_source_uuid)
            parsed_any = False
            hr = rr = spo2 = pi = None
            hr_idx = rr_idx = spo2_idx = pi_idx = None
            if should_try_parse:
                hr, rr, spo2, pi, hr_idx, rr_idx, spo2_idx, pi_idx = _parse_ble_vitals_packet(
                    payload,
                    hr_byte_index=args.hr_byte_index,
                    spo2_byte_index=args.spo2_byte_index,
                    rr_byte_index=args.rr_byte_index,
                    pi_byte_index=args.pi_byte_index,
                    hr_scale=float(args.hr_scale if args.hr_scale is not None else 1.0),
                    spo2_scale=float(args.spo2_scale if args.spo2_scale is not None else 1.0),
                    rr_scale=float(args.rr_scale if args.rr_scale is not None else 1.0),
                    pi_scale=float(args.pi_scale if args.pi_scale is not None else 1.0),
                    last_hr=last_logged_hr,
                    allow_heuristic=bool(args.auto_parse),
                )
                parsed_any = (hr is not None) or (rr is not None) or (spo2 is not None) or (pi is not None)

            record_stat(source_uuid, payload, parsed_any)

            should_log = parsed_any
            if should_log and args.min_log_interval_ms is not None:
                if (t_ms - last_log_ms) < int(args.min_log_interval_ms):
                    should_log = False
            if should_log and args.dedupe:
                same_hr = (hr is None and last_logged_hr is None) or (hr is not None and last_logged_hr is not None and abs(hr - last_logged_hr) < 1e-9)
                same_rr = (rr is None and last_logged_rr is None) or (rr is not None and last_logged_rr is not None and abs(rr - last_logged_rr) < 1e-9)
                if same_hr and same_rr:
                    should_log = False

            if should_log:
                if parse_source_uuid is None:
                    parse_source_uuid = source_uuid
                parsed_writer.writerow({
                    "timestamp_ms": t_ms,
                    "source_uuid": source_uuid,
                    "ref_hr": _fmt_ref_value(hr, decimals_if_needed=1),
                    "ref_rr": _fmt_ref_value(rr, decimals_if_needed=1),
                    "ref_spo2": _fmt_ref_value(spo2, decimals_if_needed=1),
                    "ref_pi": _fmt_ref_value(pi, decimals_if_needed=1),
                    "hr_byte_index": "" if hr_idx is None else hr_idx,
                    "rr_byte_index": "" if rr_idx is None else rr_idx,
                    "spo2_byte_index": "" if spo2_idx is None else spo2_idx,
                    "pi_byte_index": "" if pi_idx is None else pi_idx,
                    "raw_hex": raw_hex,
                })
                parsed_f.flush()
                parsed_count += 1
                last_log_ms = t_ms
                if hr is not None:
                    last_logged_hr = hr
                if rr is not None:
                    last_logged_rr = rr
                details = []
                if hr is not None:
                    details.append(f"HR={_fmt_ref_value(hr, decimals_if_needed=1)}")
                if rr is not None:
                    details.append(f"RR={_fmt_ref_value(rr, decimals_if_needed=1)}")
                if spo2 is not None:
                    details.append(f"SpO2={_fmt_ref_value(spo2, decimals_if_needed=1)}%")
                if pi is not None:
                    details.append(f"PI={_fmt_ref_value(pi, decimals_if_needed=1)}")
                details_s = "  ".join(details) if details else "parsed packet"
                print(_green(f"  t={elapsed:.1f}s  [{source_uuid}] {details_s}  (raw={raw_hex})"))
            elif args.echo_raw:
                print(_dim(f"  t={elapsed:.1f}s  [{source_uuid}] raw={raw_hex}"))
        return notification_handler

    async def find_device() -> str:
        if normalized_address:
            return normalized_address
        print(f"Scanning for BLE devices for up to {args.scan_timeout:.1f}s...")
        devices = await BleakScanner.discover(timeout=float(args.scan_timeout))
        for d in devices:
            name = getattr(d, "name", None) or ""
            if any(h.lower() in name.lower() for h in name_hints):
                print(_green(f"Found candidate: {name or '(unnamed)'} at {d.address}"))
                return d.address
        raise RuntimeError("BLE oximeter not found. Try --address, or adjust --name-hint.")

    async def run_ble():
        nonlocal parse_source_uuid, active_notify_chars
        target_address = await find_device()
        print(f"Connecting to {target_address}...")
        async with BleakClient(target_address) as client:
            if not client.is_connected:
                raise RuntimeError("BLE connection failed.")
            print(_green("Connected. Enumerating services..."))
            services = client.services
            if not services:
                services = await client.get_services()
            if args.probe_services:
                print(_bold("Available services / characteristics:"))
                for service in services:
                    print(f"  [SERVICE] {service.uuid}")
                    for char in getattr(service, "characteristics", []):
                        props = ",".join(str(p) for p in getattr(char, "properties", []))
                        print(f"    - {char.uuid}  ({props})")
            notify_candidates = _pick_notify_characteristics(services, preferred_uuid=preferred_notify)
            if not notify_candidates:
                raise RuntimeError("No notify-capable BLE characteristic found.")
            if args.probe_services:
                print(_dim("Notify candidates (best first):"))
                for c in notify_candidates:
                    print(_dim(f"  - {c}"))

            subscribe_targets = notify_candidates if args.subscribe_all else notify_candidates[:]
            if not args.subscribe_all:
                subscribe_targets = notify_candidates

            last_notify_error = None
            if args.subscribe_all:
                for candidate in subscribe_targets:
                    try:
                        print(f"Trying notify characteristic: {candidate}")
                        await client.start_notify(candidate, make_notification_handler(candidate))
                        active_notify_chars.append(candidate)
                        print(_green(f"Subscribed: {candidate}"))
                        if parse_source_uuid is None and (args.notify_char is None or candidate.lower() == preferred_notify.lower()):
                            parse_source_uuid = candidate
                    except Exception as e:
                        last_notify_error = e
                        print(_yellow(f"  start_notify failed on {candidate}: {e}"))
                if not active_notify_chars:
                    raise RuntimeError(
                        "Could not start notify on any candidate characteristic. "
                        f"Last error: {last_notify_error}")
                if parse_source_uuid is None:
                    parse_source_uuid = active_notify_chars[0]
                print(_green(f"Using parse source: {parse_source_uuid}"))
            else:
                for candidate in notify_candidates:
                    try:
                        print(f"Trying notify characteristic: {candidate}")
                        await client.start_notify(candidate, make_notification_handler(candidate))
                        active_notify_chars.append(candidate)
                        parse_source_uuid = candidate
                        print(_green(f"Using notify characteristic: {candidate}"))
                        break
                    except Exception as e:
                        last_notify_error = e
                        print(_yellow(f"  start_notify failed on {candidate}: {e}"))
                if not active_notify_chars:
                    raise RuntimeError(
                        "Could not start notify on any candidate characteristic. "
                        f"Last error: {last_notify_error}")

            if write_ops:
                print(_bold("Issuing BLE write operations:"))
                for char_uuid, payload in write_ops:
                    try:
                        await client.write_gatt_char(char_uuid, payload, response=bool(args.write_with_response))
                        print(_green(f"  wrote {payload.hex()} -> {char_uuid}"))
                        if args.post_write_wait_ms and args.post_write_wait_ms > 0:
                            await asyncio.sleep(float(args.post_write_wait_ms) / 1000.0)
                    except Exception as e:
                        print(_yellow(f"  write failed on {char_uuid}: {e}"))

            try:
                last_summary_snapshot = time.time()

                def _maybe_snapshot_summary() -> None:
                    nonlocal last_summary_snapshot
                    if time.time() - last_summary_snapshot >= 5.0:
                        last_summary_snapshot = time.time()
                        try:
                            _write_ble_summary(None)
                        except Exception:
                            pass

                if args.duration_s is not None and args.duration_s > 0:
                    end_time = time.time() + float(args.duration_s)
                    while time.time() < end_time:
                        await asyncio.sleep(0.25)
                        _maybe_snapshot_summary()
                else:
                    while True:
                        await asyncio.sleep(0.25)
                        _maybe_snapshot_summary()
            finally:
                for candidate in reversed(active_notify_chars):
                    try:
                        await client.stop_notify(candidate)
                    except Exception:
                        pass

    def _write_ble_summary(error_text_val: Optional[str]) -> None:
        # PR72 session-data audit: s08-s11 had no ref_ble_summary.json because
        # the session orchestrator hard-terminates this logger on Windows, so a
        # finally-only write never ran and analyse lost the true per-source
        # decode statistics. The summary is now also snapshotted periodically
        # from the wait loop, surviving a hard kill.
        summary = {
            "version": VERSION,
            "reference_csv": os.path.abspath(args.out),
            "raw_csv": os.path.abspath(raw_out),
            "parsed_rows": int(parsed_count),
            "raw_packets": int(raw_count),
            "notify_char_requested": args.notify_char,
            "notify_chars_active": active_notify_chars,
            "parse_source_uuid": parse_source_uuid,
            "address": normalized_address or args.address,
            "raw_only": bool(args.raw_only),
            "auto_parse": bool(args.auto_parse),
            "ble_profile": getattr(args, "ble_profile", "none"),
            "field_map": {
                "hr_byte_index": args.hr_byte_index,
                "hr_scale": args.hr_scale,
                "rr_byte_index": args.rr_byte_index,
                "rr_scale": args.rr_scale,
                "spo2_byte_index": args.spo2_byte_index,
                "spo2_scale": args.spo2_scale,
                "pi_byte_index": args.pi_byte_index,
                "pi_scale": args.pi_scale,
            },
            "write_ops": [{"char_uuid": c, "payload_hex": p.hex()} for c, p in write_ops],
            "stats_by_source": nan_safe(stats),
            "error": error_text_val,
        }
        save_json(summary, summary_out)

    error_text = None
    try:
        asyncio.run(run_ble())
    except KeyboardInterrupt:
        print("\n[BLE REFLOG] Stopped by user.")
    except Exception as e:
        error_text = str(e)
        raise
    finally:
        parsed_f.close()
        raw_f.close()
        _write_ble_summary(error_text)

    if parsed_count == 0:
        print(_yellow("\n[BLE REFLOG] No parsed HR/RR/SpO2 values were logged."))
        if args.raw_only:
            print(_dim("Raw-only mode intentionally disables parsed reference inference."))
        else:
            print(_yellow("Try --auto-parse for exploration, or run ble_analyse_raw on the raw packet CSV to rank candidate fields."))
        print(_yellow(f"Raw packet capture is still saved to: {raw_out}"))
        return

    duration_s = max(0.0, (int(time.time() * 1000) - start_ms) / 1000.0)
    print(f"\n{_green('[BLE REFLOG] Saved')} {parsed_count} parsed readings to {args.out}")
    print(f"  Raw packets: {raw_count}")
    print(f"  Duration:    {duration_s:.0f}s")
