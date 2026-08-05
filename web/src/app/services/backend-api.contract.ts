/**
 * The browser-facing inventory of the Python trainer API.
 *
 * Keep this list in lock-step with rvt_trainer/api/route_registry.  It is
 * intentionally data-only so a static QMS check can compare the two route
 * registries without importing Angular or starting a server.  Dynamic session
 * and participant paths use `*` in the same way as the Python registry.
 */
export interface BackendApiRoute {
  readonly name: string;
  readonly methods: readonly string[];
  readonly path: string;
}

export const BACKEND_API_ROUTES: readonly BackendApiRoute[] = [
  { name: 'auth_exchange', methods: ['POST'], path: '/api/auth/exchange' },
  { name: 'auth_validate', methods: ['GET'], path: '/api/auth/validate' },
  { name: 'auth_login', methods: ['POST'], path: '/api/auth/login' },
  { name: 'auth_logout', methods: ['POST'], path: '/api/auth/logout' },
  { name: 'auth_sse_token', methods: ['POST'], path: '/api/auth/sse-token' },
  { name: 'auth_reset_pin', methods: ['POST'], path: '/api/auth/reset-pin' },
  { name: 'auth_host_reset', methods: ['POST'], path: '/api/auth/host-reset' },
  { name: 'operator_profiles_get', methods: ['GET'], path: '/api/operator-profiles' },
  { name: 'operator_profiles_create', methods: ['POST'], path: '/api/operator-profiles' },
  { name: 'subject_profiles', methods: ['GET'], path: '/api/subject-profiles' },
  { name: 'subject_profiles_put', methods: ['PUT'], path: '/api/subject-profiles' },
  { name: 'server_info', methods: ['GET'], path: '/api/server-info' },
  { name: 'native_pairing_info', methods: ['GET'], path: '/api/native-pairing-info' },
  { name: 'participants_list', methods: ['GET'], path: '/api/participants' },
  { name: 'participants_create', methods: ['POST'], path: '/api/participants' },
  { name: 'participant_status', methods: ['PUT'], path: '/api/participants/*' },
  { name: 'study_completion_matrix', methods: ['GET'], path: '/api/study/completion-matrix' },
  { name: 'study_objectives', methods: ['GET'], path: '/api/study/objectives' },
  { name: 'study_protocol', methods: ['GET', 'PUT'], path: '/api/study/protocol' },
  { name: 'study_schedule', methods: ['GET'], path: '/api/study/schedule' },
  { name: 'study_attempt_create', methods: ['POST'], path: '/api/study/attempts' },
  { name: 'study_analysis_start', methods: ['POST'], path: '/api/study/analysis' },
  { name: 'study_analysis_list', methods: ['GET'], path: '/api/study/analysis' },
  { name: 'study_analysis_status', methods: ['GET'], path: '/api/study/analysis/*' },
  { name: 'study_analysis_cancel', methods: ['DELETE'], path: '/api/study/analysis/*' },
  { name: 'study_objective_report', methods: ['GET'], path: '/api/study/objectives/*/report' },
  { name: 'session_start', methods: ['POST'], path: '/api/session/start' },
  { name: 'session_stop', methods: ['POST'], path: '/api/session/stop' },
  { name: 'session_annotate', methods: ['POST'], path: '/api/session/annotate' },
  { name: 'session_annotations', methods: ['POST'], path: '/api/session/annotations' },
  { name: 'sessions_list', methods: ['GET'], path: '/api/sessions' },
  { name: 'session_files', methods: ['GET'], path: '/api/sessions/*/files/**' },
  { name: 'session_annotations_get', methods: ['GET'], path: '/api/sessions/*/annotations' },
  { name: 'session_notes_get', methods: ['GET'], path: '/api/sessions/*/notes' },
  { name: 'session_notes_put', methods: ['PUT'], path: '/api/sessions/*/notes' },
  { name: 'session_signoff_get', methods: ['GET'], path: '/api/sessions/*/signoff' },
  { name: 'session_signoff_put', methods: ['PUT'], path: '/api/sessions/*/signoff' },
  { name: 'session_references_get', methods: ['GET'], path: '/api/sessions/*/references' },
  { name: 'session_references_post', methods: ['POST'], path: '/api/sessions/*/references' },
  { name: 'session_rr_adjudication', methods: ['POST'], path: '/api/sessions/*/references/rr-adjudication' },
  { name: 'session_tags', methods: ['PUT'], path: '/api/sessions/*/tags' },
  { name: 'session_delete', methods: ['DELETE'], path: '/api/sessions/*' },
  { name: 'health', methods: ['GET'], path: '/api/health' },
  { name: 'version', methods: ['GET'], path: '/api/version' },
  { name: 'update_manifest', methods: ['GET'], path: '/api/update/manifest' },
  { name: 'help_schema', methods: ['GET'], path: '/api/help/schema' },
  { name: 'ble_scan', methods: ['GET'], path: '/api/ble/scan' },
  { name: 'trainer_log', methods: ['GET'], path: '/api/trainer/log' },
  { name: 'status', methods: ['GET'], path: '/api/status' },
  { name: 'defaults', methods: ['GET', 'POST'], path: '/api/defaults' },
  { name: 'serial_ports', methods: ['GET'], path: '/api/serial/ports' },
  { name: 'preflight_all', methods: ['GET'], path: '/api/preflight' },
  { name: 'preflight_one', methods: ['POST'], path: '/api/preflight/*' },
  { name: 'session_events', methods: ['GET'], path: '/api/session/events' },
  { name: 'events_subscribe', methods: ['GET'], path: '/api/events/subscribe' },
  { name: 'recorded_session_events', methods: ['GET'], path: '/api/sessions/*/events' },
  { name: 'session_current', methods: ['GET'], path: '/api/session/current' },
  { name: 'session_live_dashboard', methods: ['GET'], path: '/api/session/current/live_dashboard.json' },
  { name: 'session_buffer', methods: ['GET'], path: '/api/session/buffer' },
  { name: 'session_summary', methods: ['GET'], path: '/api/sessions/*/summary' },
  { name: 'session_data', methods: ['GET'], path: '/api/sessions/*/data' },
  { name: 'session_compare', methods: ['GET'], path: '/api/sessions/*/compare' },
  { name: 'session_analysis_status', methods: ['GET'], path: '/api/sessions/*/analyse/status' },
  { name: 'session_training_status', methods: ['GET'], path: '/api/sessions/*/training/status' },
  { name: 'session_predict', methods: ['GET'], path: '/api/sessions/*/predict' },
  { name: 'session_analyse', methods: ['POST'], path: '/api/sessions/*/analyse' },
  { name: 'report_export', methods: ['GET'], path: '/api/report/export' },
];

export function sessionApiPath(sessionId: string, suffix: string): string {
  return `/api/sessions/${encodeURIComponent(sessionId)}${suffix}`;
}
