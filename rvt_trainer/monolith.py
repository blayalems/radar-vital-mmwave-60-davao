"""Retired monolith compatibility facade.

All trainer logic has been extracted into cohesive submodules under
``rvt_trainer/``. This module re-exports historical public and underscored
names (plus ``json`` for test monkeypatching) without defining functions or
classes."""

from __future__ import annotations

import json

from rvt_trainer.api.auth import (
    _OPERATOR_LOCK, _PIN_MAX, _PIN_TTL_S, _invalidate_operator_sessions, _invalidate_operator_sse_tokens,
    create_operator_profile as _create_operator_profile, exchange_pair_pin as _exchange_pair_pin,
    host_reset_pin as _host_reset_pin, load_operator_profiles as _load_operator_profiles, login_operator as _login_operator,
    make_pair_pin as _make_pair_pin, reset_pin_with_recovery as _reset_pin_with_recovery,
)
from rvt_trainer.api.common import (
    api_error as _api_error, atomic_write_json as save_json, json_safe_response as _json_safe_response, nan_safe,
    read_json_if_exists as _read_json_if_exists, wait_for_process_exit as _wait_for_process_exit,
)
from rvt_trainer.api.compatibility import (
    build_compatibility_handshake as _build_compatibility_handshake,
    validate_client_compatibility as _validate_client_compatibility,
)
from rvt_trainer.api.route_registry import (
    AuthPolicy as _RouteAuthPolicy, authorization_for as _route_authorization_for, match_route as _match_route,
)
from rvt_trainer.api.server import (
    HELP_SCHEMA, _RATE_LIMIT, _RATE_LIMIT_LOCK, _RATE_LIMIT_MAX, _ControlHandler, _ControlHTTPServer, _ControlServer,
    _build_help_schema, _control_schema_versions, _effective_defaults, _ensure_self_signed_cert, _manifest_cache,
    _manifest_cache_lock, _observed_release_identity, _process_memory_mb, _qr_png_bytes, _reason_tooltips,
    _server_release_handshake, _server_scheme, _start_control_server, _system_metrics, cmd_serve,
)
from rvt_trainer.api.server_info import (
    _guess_lan_ip, advertised_host as _advertised_host, advertised_origin as _advertised_origin,
    manifest_payload as _manifest_payload, pair_page_html as _pair_page_html, support_matrix_html as _support_matrix_html,
)
from rvt_trainer.assets.static import (
    assets_root as _assets_root, content_type_for_asset as _content_type_for_asset, safe_asset_path as _safe_asset_path,
)
from rvt_trainer.audit.runner import (
    SESSION_START_PREFLIGHT_IDS, _candidate_ino_paths, _preflight_result, _session_start_blocking_failures,
    run_preflight_all as _run_preflight_all, run_preflight_check as _run_preflight_check,
)
from rvt_trainer.audit.truthfulness import (
    _build_agc_anomaly_flags, _build_gate_audit, _compare_to_golden, _compute_internal_consistency_score,
    _compute_session_quality_score, _coverage_loss_ledger, _coverage_stage_label, _derive_raw_rescue_reject_count,
    _experimental_hr_gate_metrics, _extract_golden_metrics, _first_populated_series, _fmt_quick_metrics, _format_sketch_version,
    _ml_gate_block_from_baseline, _oracle_candidate_audit, _phase_subset_summary, _radar_csv_on_disk_contract_length,
    _truthfulness_from_radar,
)
from rvt_trainer.audit.verdict import (
    _apply_piecewise3, _build_adaptive_correction_shadow, _build_ml_readiness_verdict, _build_pqi_v15_shadow,
    _confidently_wrong_curve, _coverage_penalized_hr_gate, _dynamic_range_gate, _evaluability_block, _fit_piecewise3,
    baseline_summary, compute_hr_bias_estimate, compute_metrics, compute_raw_hr_bias_estimate,
    compute_raw_hr_sumfreq_error_estimate, finite_pair_mask, get_valid_pair_arrays, lins_ccc, longest_false_run,
    maybe_plot_bland_altman, maybe_plot_phase_signals, maybe_plot_timeseries, model_summary, print_metric_block, safe_corr,
    save_feature_importance, validity_masks,
)
from rvt_trainer.cli import _QUICKSTART_TEXT, _run_self_tests, build_parser, cmd_doctor, cmd_quickstart, main
from rvt_trainer.constants import (
    ALIGNMENT_REPORT_SCHEMA_VERSION, BLE_PI_QUALITY_THRESHOLD, CHART_ANNOTATIONS_SCHEMA_VERSION, CONFIRMATORY_ANALYSIS_PLAN_ID,
    CONFIRMATORY_RUN_MANIFEST_SCHEMA_VERSION, CONTROL_API_SCHEMA_VERSION, DASHBOARD_VERSION, DEFAULT_BLE_ADDRESS,
    DEFAULT_RADAR_PORT, DEFAULT_SUBJECT_PROFILES, FEATURE_ENGINEERING_VERSION, FEATURE_FLAGS, FEATURE_SCHEMA_VERSION,
    FIRMWARE_HR_PUBLISH_RANGE, FIRMWARE_RR_PUBLISH_RANGE, FIRMWARE_VERSION_EXPECTED, HARMONIC_MODE_BIT_LABELS,
    HEART_PQI_GATE_FAR, HEART_PQI_GATE_MID, HEART_PQI_GATE_NEAR, HR_ANCHOR_SOURCE_NAMES, HR_CONFIDENCE_SOURCE_NAMES,
    HR_GATE_REASON_NAMES, HR_PATH_SOURCE_NAMES, HR_PUBLISH_REASON_NAMES, HR_PUBLISH_SOURCE_NAMES,
    HR_PUBLISH_SOURCE_NAMES_INVERSE, HR_RANGE, HR_RAW_DISAGREE_SUBREASON_NAMES, HR_RESCUE_PLAUSIBLE_MAX_BPM,
    HR_RESCUE_PLAUSIBLE_MIN_BPM, HR_VALID_MAX_AGE_MS, HR_VALID_PUBLISH_OK, LEGACY_RADAR_LOG_COLUMN_COUNT, LEGACY_RADAR_LOG_TAIL,
    LIVE_EVENT_SCHEMA_VERSION, MAX_JSON_BODY_BYTES, MIN_PHASE_VALID_RUN_LEN, ML_READY_MIN_BLE_COVERAGE_PCT,
    ML_READY_MIN_HR_BIAS_BPM, ML_READY_MIN_HR_COVERAGE_PCT, ML_READY_MIN_HR_MAE_BPM, ML_READY_MIN_RR_COVERAGE_PCT,
    ML_READY_MIN_RR_MAE_BPM, MODEL_ARTIFACT_NAMES, MODEL_MANIFEST_FILENAME, MODEL_MANIFEST_SCHEMA_VERSION,
    MODEL_SIGNING_KEY_ENV, PUBLISH_BLOCK_STAGE_NAMES, REFERENCE_PHYSIO_HR_RANGE, REFERENCE_PHYSIO_RR_RANGE,
    REQUIRE_MODEL_MANIFEST_ENV, REQUIRE_MODEL_SIGNATURE_ENV, RR_GATE_REASON_NAMES, RR_PUBLISH_REASON_NAMES,
    RR_PUBLISH_SOURCE_NAMES, RR_RANGE, RR_SOURCE_NAMES, RR_SOURCE_REJECT_REASON_NAMES, RR_VALID_MAX_AGE_MS, RR_VALID_PUBLISH_OK,
    SCORING_WEIGHTS, SECONDARY_GATE_MIN_DURATION_S, SECONDARY_GATE_MIN_PAIRS, SESSION_MANIFEST_SCHEMA_VERSION,
    SESSION_MANIFEST_VERSION, SESSION_NOTES_SCHEMA_VERSION, SESSION_PHASE_NAMES, SESSION_SIGNOFF_SCHEMA_VERSION,
    SUBJECT_PROFILE_SCHEMA_VERSION, SYNC_ANCHORS_SCHEMA_VERSION, TRAINING_PROGRESS_SCHEMA_VERSION, UPDATE_MANIFEST_URL, VERSION,
    _PACKAGE_ROOT, _REPO_ROOT, _TRAINER_ENTRYPOINT, _TRAINER_LOG, _USE_COLOR, _append_trainer_log, _bold, _c,
    _cleanup_json_temp_files, _clip01, _cyan, _decimate_records, _dim, _env_flag_enabled, _file_sha256, _first_existing, _green,
    _histogram_from_series, _is_relative_to_path, _iso_from_mtime, _iso_now, _json_hash, _numeric_or_none, _pid_alive, _red,
    _report_stamp, _row_get, _row_num, _row_text, _safe_float_val, _safe_int, _sanitize_user_string, _schema_wrap,
    _scoring_weights_hash, _session_duration_s, _summary_value, _to_num, _top_histogram_items, _trainer_command_prefix, _yellow,
    bytes_to_c_array, safe_series, tensorflow_is_available, warn, write_text_report,
)
from rvt_trainer.protocol_contract import (
    EXPECTED_RADAR_LOG_COLUMN_COUNT, EXPECTED_RADAR_LOG_TAIL, LEGACY_V14_COLUMN_COUNT, LEGACY_V14_RADAR_LOG_TAIL,
    LEGACY_V15_1_COLUMN_COUNT, LEGACY_V15_COLUMN_COUNT, RADAR_LOG_COLUMNS, SCHEMA_VERSION_MAP, V15_NEW_COLUMN_DEFAULTS,
    load_protocol_contract as _load_protocol_contract, protocol_contract_candidates as _protocol_contract_candidates,
)
from rvt_trainer.model_artifacts import (
    MODEL_BUNDLE_METADATA_FILENAME, FeatureContract, assert_bundle_inference_compatible, build_model_bundle_metadata,
)
from rvt_trainer.modeling import (
    BASE_FEATURES_CORE, BASE_FEATURES_FULL, FEATURE_CATEGORY, LABEL_FEATURES, MODEL_FAMILIES, MODEL_FAMILY_CNN_1D,
    MODEL_FAMILY_GRADIENT_BOOSTING, POLICY_FEATURE_EXACT, POLICY_FEATURE_PREFIXES, RADAR_SYNONYMS, REF_SYNONYMS,
    TEMPORAL_BASE_COLS_CORE, TEMPORAL_BASE_COLS_FULL, TEMPORAL_SUFFIXES_CORE, TEMPORAL_SUFFIXES_FULL, TRAINING_EXCLUDE_COLS,
    Cnn1DConfig, Keras1DCNNRegressor, _alignment_provenance, _bootstrap_delta_ci, _build_model_manifest,
    _build_sweep_delta_rows, _combine_radar_frames, _controlled_analysis_plan, _controlled_research_identity, _format_ci,
    _generate_compare_html, _load_sweep_param_map, _metric_samples_from_aligned, _model_artifact_hashes, _model_manifest_hmac,
    _model_signing_key, _plot_phase_signal, _read_model_bundle_metadata, _run_loso_evaluation, _sample_rmse_r,
    _validate_confirmatory_loso, _verify_model_dir, _write_confirmatory_run_manifest, _write_model_manifest,
    _write_simple_html_report, add_predictions, aggregate_radar_1hz, aggregate_reference_1hz, align_one_session,
    align_reference_time, apply_causal_slew_limit, assert_ml_feature_schema, build_causal_windows, build_model,
    build_tiny_surrogate, canonicalize_with_synonyms, choose_best_n_estimators, chronological_split_single_session,
    circular_mean, circular_std, cmd_align, cmd_analyse, cmd_compare, cmd_predict, cmd_sweep, cmd_train, enforce_baseline_gate,
    engineer_temporal_features, export_linear_embedded_header, export_tflite_surrogate, feature_category, feature_schema_hash,
    finite_phase_series_with_ts, fit_cnn_target_model, fit_linear_embedded_surrogate, fit_selected_target_model,
    fit_target_model, get_sample_weights, label_feature_columns, last_valid, load_and_aggregate_radar_only,
    load_and_align_multiple_base, load_radar, load_reference, lower_columns, maybe_export_embedded, normalize_offsets,
    phase_band_features, phase_to_radians, pick_feature_columns, policy_feature_columns, prepare_feature_matrix,
    resolve_cnn_config, resolve_model_params, save_feature_importance_plot, spectral_entropy, split_sessions, split_three_way,
    transform_feature_matrix, truncate_gbr_model, unwrapped_excursion,
)
from rvt_trainer.runtime_paths import runtime_root
from rvt_trainer.session.analysis_jobs import (
    _ANALYSIS_JOBS, _ANALYSIS_JOBS_MAX, _STUDY_ANALYSIS_LOCK, _STUDY_ANALYSIS_PROCS, _STUDY_ANALYSIS_SEMAPHORE,
    _analysis_job_status, _auto_analyse_sentinel, _count_csv_data_rows, _evict_completed_analysis_jobs, _rerun_session_analysis,
    _run_study_analysis_job, _run_study_analysis_job_once, _run_study_statistics_job, _spawn_auto_analyse,
    _start_study_analysis_job, _study_analysis_output_dir, _write_attempt_ledger_csv,
)
from rvt_trainer.session.collection_authorization import (
    CollectionAuthorizationError, collection_authorization_status as _collection_authorization_status,
    require_collection_authorization as _require_collection_authorization,
)
from rvt_trainer.session.commands import cmd_dashboard, cmd_session
from rvt_trainer.session.live_dashboard import (
    _DASHBOARD_HTML_FALLBACK_NAMES, _DASHBOARD_HTML_NAME, _DASHBOARD_TEMPLATE_EMBEDDED, _LEGACY_DASHBOARD_HTML_NAMES,
    _CSVAppendTracker, _DashboardServer, _SilentDashboardHandler, _age_from_tracker, _build_dashboard_faults,
    _coerce_boolish_series, _coerce_numeric_series, _coerce_text_series, _dashboard_html_path, _dashboard_html_template_path,
    _dashboard_html_template_paths, _dashboard_json_path, _dashboard_template_candidates, _decode_enum_series,
    _format_dashboard_event, _harmonic_mode_labels, _heart_pqi_gate, _hr_path_source_name, _idle_live_payload,
    _last_positive_or_nan, _last_valid_age_s, _load_dashboard_template_text, _mock_live_payload, _prefer_display_series,
    _recent_rate_hz, _render_live_dashboard, _resolve_live_distance_cm, _series_from_trackers, _start_dashboard_server,
    _tracker_dataframe, _write_live_dashboard_html, _write_live_dashboard_json,
)
from rvt_trainer.session.manifest import (
    _manifest_identity, _session_root_for_outputs, _warn_on_manifest_mismatch, _write_session_manifest,
)
from rvt_trainer.session.protocol_ledger import (
    ATTEMPT_LEDGER_SCHEMA_VERSION, allocate_attempt_id as _allocate_attempt_id,
    append_session_attempt_event as _append_session_attempt_event, completion_matrix as _completion_matrix,
    initialize_session_attempt as _initialize_session_attempt, register_protocol_attempt as _register_protocol_attempt,
)
from rvt_trainer.session.start_idempotency import StartIdempotencyError
from rvt_trainer.session.storage import (
    _build_report_export_html, _compare_session_payload, _contract_diagnosis, _dashboard_analysis_payload,
    _download_items_for_session, _duration_from_csv, _infer_session_duration_s, _infer_session_timestamp,
    _latest_live_dashboard_payload, _load_session_annotations_payload, _load_session_summary, _load_subject_profiles,
    _metric_summary, _next_session_dir, _print_and_save_session_report, _save_subject_profiles, _scan_sessions_root,
    _session_annotations_path, _session_data_payload, _session_path, _subject_profiles_path, _top_hist_items,
    _upsert_session_annotation,
)
from rvt_trainer.session.study_contract import (
    PARTICIPANT_REGISTRY_SCHEMA_VERSION, STUDY_SESSION_SCHEMA_VERSION, StudyContractError,
    create_participant_profile as _create_participant_profile, load_participant_registry as _load_participant_registry,
    merge_immutable_study_assignment as _merge_immutable_study_assignment, release_provenance as _release_provenance,
    source_commit as _source_commit, update_participant_status as _update_participant_status,
    validate_study_assignment as _validate_study_assignment,
)
from rvt_trainer.session.supervisor import (
    SessionStartPreflightError, SessionSupervisor as _SessionSupervisor, _check_stale_session_lock,
    _clear_supervisor_stop_request, _consume_supervisor_stop_request, _lock_path, _read_session_lock, _release_session_lock,
    _release_session_lock_if_owned, _same_session_dir, _session_is_active, _session_marker_owned_by, _supervisor_stop_path,
    _write_session_lock, _write_supervisor_stop_request,
)
from rvt_trainer.transport.ble import (
    _apply_ble_profile_defaults, _ble_sender_column, _ble_summary_candidates_for_ref, _build_ble_byte_candidates,
    _build_ble_word_candidates, _compute_ble_ref_quality, _fmt_ref_value, _hex_to_bytes, _normalize_ble_address,
    _parse_ble_vitals_packet, _pick_notify_characteristics, _prepare_ble_write_ops, _raw_ble_candidates_for_ref,
    _score_ble_byte_candidate, cmd_ble_analyse_raw, cmd_ble_reflog, scan_ble_devices_payload as _scan_ble_devices_payload,
)
from rvt_trainer.transport.serial import (
    _FIRMWARE_CONTRACT_CACHE, _assert_radar_log_contract, _check_arduino_cli, _detect_csv_schema_version,
    _extract_firmware_data_header_tokens, _firmware_contract_candidates, _is_supported_radar_contract_length,
    _log_serial_for_duration, _parse_radar_data_line, _patch_ino_constants, _prepare_arduino_sketch_dir, _serial_ports_payload,
    auto_detect_radar_port as _auto_detect_radar_port, cmd_flash, cmd_log, cmd_reflog,
)

__all__ = [
    "ALIGNMENT_REPORT_SCHEMA_VERSION", "ATTEMPT_LEDGER_SCHEMA_VERSION", "BASE_FEATURES_CORE", "BASE_FEATURES_FULL",
    "BLE_PI_QUALITY_THRESHOLD", "CHART_ANNOTATIONS_SCHEMA_VERSION", "CONFIRMATORY_ANALYSIS_PLAN_ID",
    "CONFIRMATORY_RUN_MANIFEST_SCHEMA_VERSION", "CONTROL_API_SCHEMA_VERSION", "Cnn1DConfig", "CollectionAuthorizationError",
    "DASHBOARD_VERSION", "DEFAULT_BLE_ADDRESS", "DEFAULT_RADAR_PORT", "DEFAULT_SUBJECT_PROFILES",
    "EXPECTED_RADAR_LOG_COLUMN_COUNT", "EXPECTED_RADAR_LOG_TAIL", "FEATURE_CATEGORY", "FEATURE_ENGINEERING_VERSION",
    "FEATURE_FLAGS", "FEATURE_SCHEMA_VERSION", "FIRMWARE_HR_PUBLISH_RANGE", "FIRMWARE_RR_PUBLISH_RANGE",
    "FIRMWARE_VERSION_EXPECTED", "FeatureContract", "HARMONIC_MODE_BIT_LABELS", "HEART_PQI_GATE_FAR", "HEART_PQI_GATE_MID",
    "HEART_PQI_GATE_NEAR", "HELP_SCHEMA", "HR_ANCHOR_SOURCE_NAMES", "HR_CONFIDENCE_SOURCE_NAMES", "HR_GATE_REASON_NAMES",
    "HR_PATH_SOURCE_NAMES", "HR_PUBLISH_REASON_NAMES", "HR_PUBLISH_SOURCE_NAMES", "HR_PUBLISH_SOURCE_NAMES_INVERSE", "HR_RANGE",
    "HR_RAW_DISAGREE_SUBREASON_NAMES", "HR_RESCUE_PLAUSIBLE_MAX_BPM", "HR_RESCUE_PLAUSIBLE_MIN_BPM", "HR_VALID_MAX_AGE_MS",
    "HR_VALID_PUBLISH_OK", "Keras1DCNNRegressor", "LABEL_FEATURES", "LEGACY_RADAR_LOG_COLUMN_COUNT", "LEGACY_RADAR_LOG_TAIL",
    "LEGACY_V14_COLUMN_COUNT", "LEGACY_V14_RADAR_LOG_TAIL", "LEGACY_V15_1_COLUMN_COUNT", "LEGACY_V15_COLUMN_COUNT",
    "LIVE_EVENT_SCHEMA_VERSION", "MAX_JSON_BODY_BYTES", "MIN_PHASE_VALID_RUN_LEN", "ML_READY_MIN_BLE_COVERAGE_PCT",
    "ML_READY_MIN_HR_BIAS_BPM", "ML_READY_MIN_HR_COVERAGE_PCT", "ML_READY_MIN_HR_MAE_BPM", "ML_READY_MIN_RR_COVERAGE_PCT",
    "ML_READY_MIN_RR_MAE_BPM", "MODEL_ARTIFACT_NAMES", "MODEL_BUNDLE_METADATA_FILENAME", "MODEL_FAMILIES",
    "MODEL_FAMILY_CNN_1D", "MODEL_FAMILY_GRADIENT_BOOSTING", "MODEL_MANIFEST_FILENAME", "MODEL_MANIFEST_SCHEMA_VERSION",
    "MODEL_SIGNING_KEY_ENV", "PARTICIPANT_REGISTRY_SCHEMA_VERSION", "POLICY_FEATURE_EXACT", "POLICY_FEATURE_PREFIXES",
    "PUBLISH_BLOCK_STAGE_NAMES", "RADAR_LOG_COLUMNS", "RADAR_SYNONYMS", "REFERENCE_PHYSIO_HR_RANGE",
    "REFERENCE_PHYSIO_RR_RANGE", "REF_SYNONYMS", "REQUIRE_MODEL_MANIFEST_ENV", "REQUIRE_MODEL_SIGNATURE_ENV",
    "RR_GATE_REASON_NAMES", "RR_PUBLISH_REASON_NAMES", "RR_PUBLISH_SOURCE_NAMES", "RR_RANGE", "RR_SOURCE_NAMES",
    "RR_SOURCE_REJECT_REASON_NAMES", "RR_VALID_MAX_AGE_MS", "RR_VALID_PUBLISH_OK", "SCHEMA_VERSION_MAP", "SCORING_WEIGHTS",
    "SECONDARY_GATE_MIN_DURATION_S", "SECONDARY_GATE_MIN_PAIRS", "SESSION_MANIFEST_SCHEMA_VERSION", "SESSION_MANIFEST_VERSION",
    "SESSION_NOTES_SCHEMA_VERSION", "SESSION_PHASE_NAMES", "SESSION_SIGNOFF_SCHEMA_VERSION", "SESSION_START_PREFLIGHT_IDS",
    "STUDY_SESSION_SCHEMA_VERSION", "SUBJECT_PROFILE_SCHEMA_VERSION", "SYNC_ANCHORS_SCHEMA_VERSION",
    "SessionStartPreflightError", "StartIdempotencyError", "StudyContractError", "TEMPORAL_BASE_COLS_CORE",
    "TEMPORAL_BASE_COLS_FULL", "TEMPORAL_SUFFIXES_CORE", "TEMPORAL_SUFFIXES_FULL", "TRAINING_EXCLUDE_COLS",
    "TRAINING_PROGRESS_SCHEMA_VERSION", "UPDATE_MANIFEST_URL", "V15_NEW_COLUMN_DEFAULTS", "VERSION", "_ANALYSIS_JOBS",
    "_ANALYSIS_JOBS_MAX", "_CSVAppendTracker", "_ControlHTTPServer", "_ControlHandler", "_ControlServer",
    "_DASHBOARD_HTML_FALLBACK_NAMES", "_DASHBOARD_HTML_NAME", "_DASHBOARD_TEMPLATE_EMBEDDED", "_DashboardServer",
    "_FIRMWARE_CONTRACT_CACHE", "_LEGACY_DASHBOARD_HTML_NAMES", "_OPERATOR_LOCK", "_PACKAGE_ROOT", "_PIN_MAX", "_PIN_TTL_S",
    "_QUICKSTART_TEXT", "_RATE_LIMIT", "_RATE_LIMIT_LOCK", "_RATE_LIMIT_MAX", "_REPO_ROOT", "_RouteAuthPolicy",
    "_STUDY_ANALYSIS_LOCK", "_STUDY_ANALYSIS_PROCS", "_STUDY_ANALYSIS_SEMAPHORE", "_SessionSupervisor",
    "_SilentDashboardHandler", "_TRAINER_ENTRYPOINT", "_TRAINER_LOG", "_USE_COLOR", "_advertised_host", "_advertised_origin",
    "_age_from_tracker", "_alignment_provenance", "_allocate_attempt_id", "_analysis_job_status", "_api_error",
    "_append_session_attempt_event", "_append_trainer_log", "_apply_ble_profile_defaults", "_apply_piecewise3",
    "_assert_radar_log_contract", "_assets_root", "_auto_analyse_sentinel", "_auto_detect_radar_port", "_ble_sender_column",
    "_ble_summary_candidates_for_ref", "_bold", "_bootstrap_delta_ci", "_build_adaptive_correction_shadow",
    "_build_agc_anomaly_flags", "_build_ble_byte_candidates", "_build_ble_word_candidates", "_build_compatibility_handshake",
    "_build_dashboard_faults", "_build_gate_audit", "_build_help_schema", "_build_ml_readiness_verdict",
    "_build_model_manifest", "_build_pqi_v15_shadow", "_build_report_export_html", "_build_sweep_delta_rows", "_c",
    "_candidate_ino_paths", "_check_arduino_cli", "_check_stale_session_lock", "_cleanup_json_temp_files",
    "_clear_supervisor_stop_request", "_clip01", "_coerce_boolish_series", "_coerce_numeric_series", "_coerce_text_series",
    "_collection_authorization_status", "_combine_radar_frames", "_compare_session_payload", "_compare_to_golden",
    "_completion_matrix", "_compute_ble_ref_quality", "_compute_internal_consistency_score", "_compute_session_quality_score",
    "_confidently_wrong_curve", "_consume_supervisor_stop_request", "_content_type_for_asset", "_contract_diagnosis",
    "_control_schema_versions", "_controlled_analysis_plan", "_controlled_research_identity", "_count_csv_data_rows",
    "_coverage_loss_ledger", "_coverage_penalized_hr_gate", "_coverage_stage_label", "_create_operator_profile",
    "_create_participant_profile", "_cyan", "_dashboard_analysis_payload", "_dashboard_html_path",
    "_dashboard_html_template_path", "_dashboard_html_template_paths", "_dashboard_json_path", "_dashboard_template_candidates",
    "_decimate_records", "_decode_enum_series", "_derive_raw_rescue_reject_count", "_detect_csv_schema_version", "_dim",
    "_download_items_for_session", "_duration_from_csv", "_dynamic_range_gate", "_effective_defaults",
    "_ensure_self_signed_cert", "_env_flag_enabled", "_evaluability_block", "_evict_completed_analysis_jobs",
    "_exchange_pair_pin", "_experimental_hr_gate_metrics", "_extract_firmware_data_header_tokens", "_extract_golden_metrics",
    "_file_sha256", "_firmware_contract_candidates", "_first_existing", "_first_populated_series", "_fit_piecewise3",
    "_fmt_quick_metrics", "_fmt_ref_value", "_format_ci", "_format_dashboard_event", "_format_sketch_version",
    "_generate_compare_html", "_green", "_guess_lan_ip", "_harmonic_mode_labels", "_heart_pqi_gate", "_hex_to_bytes",
    "_histogram_from_series", "_host_reset_pin", "_hr_path_source_name", "_idle_live_payload", "_infer_session_duration_s",
    "_infer_session_timestamp", "_initialize_session_attempt", "_invalidate_operator_sessions",
    "_invalidate_operator_sse_tokens", "_is_relative_to_path", "_is_supported_radar_contract_length", "_iso_from_mtime",
    "_iso_now", "_json_hash", "_json_safe_response", "_last_positive_or_nan", "_last_valid_age_s",
    "_latest_live_dashboard_payload", "_load_dashboard_template_text", "_load_operator_profiles", "_load_participant_registry",
    "_load_protocol_contract", "_load_session_annotations_payload", "_load_session_summary", "_load_subject_profiles",
    "_load_sweep_param_map", "_lock_path", "_log_serial_for_duration", "_login_operator", "_make_pair_pin", "_manifest_cache",
    "_manifest_cache_lock", "_manifest_identity", "_manifest_payload", "_match_route", "_merge_immutable_study_assignment",
    "_metric_samples_from_aligned", "_metric_summary", "_ml_gate_block_from_baseline", "_mock_live_payload",
    "_model_artifact_hashes", "_model_manifest_hmac", "_model_signing_key", "_next_session_dir", "_normalize_ble_address",
    "_numeric_or_none", "_observed_release_identity", "_oracle_candidate_audit", "_pair_page_html", "_parse_ble_vitals_packet",
    "_parse_radar_data_line", "_patch_ino_constants", "_phase_subset_summary", "_pick_notify_characteristics", "_pid_alive",
    "_plot_phase_signal", "_prefer_display_series", "_preflight_result", "_prepare_arduino_sketch_dir",
    "_prepare_ble_write_ops", "_print_and_save_session_report", "_process_memory_mb", "_protocol_contract_candidates",
    "_qr_png_bytes", "_radar_csv_on_disk_contract_length", "_raw_ble_candidates_for_ref", "_read_json_if_exists",
    "_read_model_bundle_metadata", "_read_session_lock", "_reason_tooltips", "_recent_rate_hz", "_red",
    "_register_protocol_attempt", "_release_provenance", "_release_session_lock", "_release_session_lock_if_owned",
    "_render_live_dashboard", "_report_stamp", "_require_collection_authorization", "_rerun_session_analysis",
    "_reset_pin_with_recovery", "_resolve_live_distance_cm", "_route_authorization_for", "_row_get", "_row_num", "_row_text",
    "_run_loso_evaluation", "_run_preflight_all", "_run_preflight_check", "_run_self_tests", "_run_study_analysis_job",
    "_run_study_analysis_job_once", "_run_study_statistics_job", "_safe_asset_path", "_safe_float_val", "_safe_int",
    "_same_session_dir", "_sample_rmse_r", "_sanitize_user_string", "_save_subject_profiles", "_scan_ble_devices_payload",
    "_scan_sessions_root", "_schema_wrap", "_score_ble_byte_candidate", "_scoring_weights_hash", "_serial_ports_payload",
    "_series_from_trackers", "_server_release_handshake", "_server_scheme", "_session_annotations_path",
    "_session_data_payload", "_session_duration_s", "_session_is_active", "_session_marker_owned_by", "_session_path",
    "_session_root_for_outputs", "_session_start_blocking_failures", "_source_commit", "_spawn_auto_analyse",
    "_start_control_server", "_start_dashboard_server", "_start_study_analysis_job", "_study_analysis_output_dir",
    "_subject_profiles_path", "_summary_value", "_supervisor_stop_path", "_support_matrix_html", "_system_metrics", "_to_num",
    "_top_hist_items", "_top_histogram_items", "_tracker_dataframe", "_trainer_command_prefix", "_truthfulness_from_radar",
    "_update_participant_status", "_upsert_session_annotation", "_validate_client_compatibility", "_validate_confirmatory_loso",
    "_validate_study_assignment", "_verify_model_dir", "_wait_for_process_exit", "_warn_on_manifest_mismatch",
    "_write_attempt_ledger_csv", "_write_confirmatory_run_manifest", "_write_live_dashboard_html", "_write_live_dashboard_json",
    "_write_model_manifest", "_write_session_lock", "_write_session_manifest", "_write_simple_html_report",
    "_write_supervisor_stop_request", "_yellow", "add_predictions", "aggregate_radar_1hz", "aggregate_reference_1hz",
    "align_one_session", "align_reference_time", "apply_causal_slew_limit", "assert_bundle_inference_compatible",
    "assert_ml_feature_schema", "baseline_summary", "build_causal_windows", "build_model", "build_model_bundle_metadata",
    "build_parser", "build_tiny_surrogate", "bytes_to_c_array", "canonicalize_with_synonyms", "choose_best_n_estimators",
    "chronological_split_single_session", "circular_mean", "circular_std", "cmd_align", "cmd_analyse", "cmd_ble_analyse_raw",
    "cmd_ble_reflog", "cmd_compare", "cmd_dashboard", "cmd_doctor", "cmd_flash", "cmd_log", "cmd_predict", "cmd_quickstart",
    "cmd_reflog", "cmd_serve", "cmd_session", "cmd_sweep", "cmd_train", "compute_hr_bias_estimate", "compute_metrics",
    "compute_raw_hr_bias_estimate", "compute_raw_hr_sumfreq_error_estimate", "enforce_baseline_gate",
    "engineer_temporal_features", "export_linear_embedded_header", "export_tflite_surrogate", "feature_category",
    "feature_schema_hash", "finite_pair_mask", "finite_phase_series_with_ts", "fit_cnn_target_model",
    "fit_linear_embedded_surrogate", "fit_selected_target_model", "fit_target_model", "get_sample_weights",
    "get_valid_pair_arrays", "json", "label_feature_columns", "last_valid", "lins_ccc", "load_and_aggregate_radar_only",
    "load_and_align_multiple_base", "load_radar", "load_reference", "longest_false_run", "lower_columns", "main",
    "maybe_export_embedded", "maybe_plot_bland_altman", "maybe_plot_phase_signals", "maybe_plot_timeseries", "model_summary",
    "nan_safe", "normalize_offsets", "phase_band_features", "phase_to_radians", "pick_feature_columns",
    "policy_feature_columns", "prepare_feature_matrix", "print_metric_block", "resolve_cnn_config", "resolve_model_params",
    "runtime_root", "safe_corr", "safe_series", "save_feature_importance", "save_feature_importance_plot", "save_json",
    "spectral_entropy", "split_sessions", "split_three_way", "tensorflow_is_available", "transform_feature_matrix",
    "truncate_gbr_model", "unwrapped_excursion", "validity_masks", "warn", "write_text_report",
]
