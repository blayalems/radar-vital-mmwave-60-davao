"""
test_trainer_parsing.py
-----------------------
Unit tests for CSV row parsing helpers in rvt_trainer.monolith:
  _row_get  – fallback-column lookup on dict rows
  _to_num   – numeric coercion with NaN/garbage/None handling
  _detect_csv_schema_version – column-count → version string
"""
from __future__ import annotations

import math

import pytest

from rvt_trainer.monolith import (
    _row_get,
    _to_num,
    _detect_csv_schema_version,
    EXPECTED_RADAR_LOG_COLUMN_COUNT,
    LEGACY_V14_COLUMN_COUNT,
)


# ---------------------------------------------------------------------------
# _to_num
# ---------------------------------------------------------------------------

class TestToNum:
    def test_valid_int_string(self):
        assert _to_num("42") == pytest.approx(42.0)

    def test_valid_float_string(self):
        assert _to_num("3.14") == pytest.approx(3.14)

    def test_none_returns_default(self):
        result = _to_num(None)
        assert math.isnan(result)

    def test_none_custom_default(self):
        assert _to_num(None, default=0.0) == 0.0

    def test_empty_string_returns_default(self):
        result = _to_num("")
        assert math.isnan(result)

    def test_empty_string_custom_default(self):
        assert _to_num("", default=-1.0) == -1.0

    def test_garbage_string_returns_default(self):
        result = _to_num("not_a_number")
        assert math.isnan(result)

    def test_garbage_string_custom_default(self):
        assert _to_num("GARBAGE", default=99.0) == pytest.approx(99.0)

    def test_nan_string(self):
        # "nan" is a valid float literal in Python
        result = _to_num("nan")
        assert math.isnan(result)

    def test_inf_string(self):
        result = _to_num("inf")
        assert math.isinf(result)

    def test_actual_float(self):
        assert _to_num(1.5) == pytest.approx(1.5)

    def test_actual_int(self):
        assert _to_num(7) == pytest.approx(7.0)

    def test_zero_string(self):
        assert _to_num("0") == pytest.approx(0.0)

    def test_negative_float_string(self):
        assert _to_num("-2.71828") == pytest.approx(-2.71828)

    def test_whitespace_only_string(self):
        # A string that is not empty but not numeric — should return default.
        # "  " cannot be converted to float, so it returns default.
        result = _to_num("   ", default=0.0)
        # Python float("   ") actually strips whitespace and raises ValueError,
        # so the except branch fires — default is returned.
        assert result == pytest.approx(0.0)

    def test_object_type_returns_default(self):
        result = _to_num(object(), default=5.0)
        assert result == pytest.approx(5.0)


# ---------------------------------------------------------------------------
# _row_get
# ---------------------------------------------------------------------------

class TestRowGet:
    def test_first_key_found(self):
        row = {"a": "1", "b": "2"}
        assert _row_get(row, "a", "b") == "1"

    def test_fallback_to_second_key(self):
        # 'a' is missing entirely; 'b' should be returned
        row = {"b": "42"}
        assert _row_get(row, "a", "b") == "42"

    def test_skip_empty_string_and_fallback(self):
        # 'a' exists but is "" — should be skipped; 'b' is used
        row = {"a": "", "b": "99"}
        assert _row_get(row, "a", "b") == "99"

    def test_skip_none_value_and_fallback(self):
        # 'a' exists but is None — should be skipped; 'b' is used
        row = {"a": None, "b": "77"}
        assert _row_get(row, "a", "b") == "77"

    def test_all_keys_absent_returns_none(self):
        row = {"x": "10"}
        assert _row_get(row, "a", "b") is None

    def test_all_keys_empty_returns_none(self):
        row = {"a": "", "b": ""}
        assert _row_get(row, "a", "b") is None

    def test_all_keys_none_returns_none(self):
        row = {"a": None, "b": None}
        assert _row_get(row, "a", "b") is None

    def test_none_row_returns_none(self):
        assert _row_get(None, "a", "b") is None

    def test_empty_dict_returns_none(self):
        assert _row_get({}, "timestamp_ms") is None

    def test_no_keys_specified_returns_none(self):
        row = {"a": "1"}
        assert _row_get(row) is None

    def test_zero_string_is_not_skipped(self):
        # "0" is a non-empty, non-None string — it should be returned
        row = {"a": "0", "b": "1"}
        assert _row_get(row, "a", "b") == "0"

    def test_numeric_value_passthrough(self):
        # Numeric values stored as actual numbers (not strings) should pass through
        row = {"a": 42}
        assert _row_get(row, "a") == 42

    def test_single_key_found(self):
        row = {"timestamp_ms": "1234567890"}
        assert _row_get(row, "timestamp_ms") == "1234567890"


# ---------------------------------------------------------------------------
# _detect_csv_schema_version
# ---------------------------------------------------------------------------

class TestDetectCsvSchemaVersion:
    def test_current_column_count_returns_v15_1(self):
        # 219-column current contract (PR57 field diagnostics) maps to v15.1;
        # the legacy 207-column prefix maps to v15.0.
        cols = ["col"] * EXPECTED_RADAR_LOG_COLUMN_COUNT
        assert _detect_csv_schema_version(cols) == "v15.1"

    def test_legacy_v15_column_count_returns_v15_0(self):
        cols = ["col"] * 207
        assert _detect_csv_schema_version(cols) == "v15.0"

    def test_v14_column_count_returns_v14(self):
        cols = ["col"] * LEGACY_V14_COLUMN_COUNT
        assert _detect_csv_schema_version(cols) == "v14.1"

    def test_unknown_column_count(self):
        cols = ["col"] * 100
        result = _detect_csv_schema_version(cols)
        assert result.startswith("unknown-")
        assert "100" in result

    def test_empty_list(self):
        result = _detect_csv_schema_version([])
        assert result.startswith("unknown-") or result == "unknown-0"

    def test_non_iterable_returns_unknown(self):
        result = _detect_csv_schema_version(None)
        assert "unknown" in result
