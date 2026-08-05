export type ThemeId = 'light' | 'dark' | 'night' | 'hc';
export type DensityId = 'comfortable' | 'compact';
export type PaletteId = 'azure' | 'bloom' | 'mint';
export type HapticMode = 'on' | 'off' | 'auto';
export type AlertSeverity = 'info' | 'warn' | 'critical';
export type StorageScope = 'demo' | 'live' | 'legacy-unclassified';
export type StudyMode = 'confirmatory' | 'exploratory';
export type BarrierType = 'none' | 'cardboard';

export interface ControlStatus {
  ok: boolean;
  mode?: 'sandbox' | 'loading' | 'live' | string;
  latency?: number;
  error?: string;
  reason?: string;
  message?: string;
  session?: SessionRecord | null;
  active_session?: SessionRecord | null;
  [key: string]: unknown;
}

export interface SetupState {
  duration_s: number;
  customDuration: number;
  customUnit: string;
  radar_port: string;
  ble_address: string;
  ble_profile: string;
  notify_char: string;
  subject_label: string;
  operator_label: string;
  station_label: string;
  subject_profile_id: string;
  participant_id: string;
  study_mode: StudyMode;
  condition_id: string;
  distance_m: number;
  barrier_type: BarrierType;
  trial_number: number;
  skip_countdown: boolean;
  /** Captures are model-agnostic unless a verified inference bundle is active. */
  model_family?: 'none' | 'gradient_boosting' | 'cnn_1d';
  model_bundle?: string;
}

export interface ParticipantProfile {
  participant_id: string;
  display_code: string;
  status: 'active' | 'completed' | 'withdrawn' | string;
  completed_trials?: number;
  expected_trials?: number;
  protocol_complete?: boolean;
  status_history?: Array<Record<string, unknown>>;
  created_at?: string;
}

export interface ParticipantProfilesResponse {
  ok?: boolean;
  schema_version?: string;
  participants?: ParticipantProfile[];
  items?: ParticipantProfile[];
  profiles?: ParticipantProfile[];
  completion_matrix?: CompletionMatrix;
}

export type ParticipantStatus = 'active' | 'completed' | 'withdrawn';
export type ProtocolAttemptType = 'subject' | 'no_subject';
export type ProtocolAttemptStatus =
  | 'allocated'
  | 'starting'
  | 'collecting'
  | 'completed'
  | 'stopped'
  | 'failed_start'
  | 'aborted'
  | 'invalid'
  | 'no_output';

export interface CompletionCell {
  status: ProtocolAttemptStatus | 'missing' | string;
  attempt_id?: string | null;
  attempt_type?: ProtocolAttemptType | string;
}

export interface CompletionParticipant {
  participant_id: string;
  status?: ParticipantStatus | string;
  completed_trials: number;
  expected_trials: number;
  protocol_complete: boolean;
  cells: Record<string, CompletionCell>;
}

export interface CompletionMatrix {
  schema_version?: string;
  conditions: string[];
  trials: number[];
  participants: Record<string, CompletionParticipant>;
  participant_count: number;
  protocol_complete_participant_count: number;
  no_subject_attempt_count: number;
  no_subject_qualified_count?: number;
  no_subject_unqualified_count?: number;
  no_subject_expected: number;
  attempt_count: number;
}

export interface ProtocolAttemptInput {
  attempt_type: ProtocolAttemptType;
  condition_id: string;
  trial_number?: number;
  participant_id?: string;
  logical_trial_id?: string;
  trial_id?: string;
  status?: ProtocolAttemptStatus;
  actor?: string;
  reason?: string;
  product_version?: string;
  protocol_id?: string;
  session_id?: string;
  duration_s?: number;
  frozen_configuration_hash?: string;
  false_alarm_count?: number;
}

export interface ProtocolAttemptRecord {
  schema_version?: string;
  attempt_id: string;
  attempt_type: ProtocolAttemptType;
  participant_id?: string | null;
  logical_trial_id?: string | null;
  trial_id?: string | null;
  condition_id: string;
  trial_number?: number | null;
  product_version?: string | null;
  protocol_id?: string | null;
  status: ProtocolAttemptStatus | string;
  terminal?: boolean;
  created_at?: string;
  updated_at?: string;
  events?: Array<Record<string, unknown>>;
}

export interface ProtocolAttemptResponse {
  ok: boolean;
  schema_version?: string;
  attempt: ProtocolAttemptRecord;
}

export interface StudyObjective {
  id: string;
  number: number;
  outcome: string;
  label: string;
  role: 'confirmatory' | 'exploratory' | string;
  primary_condition_id?: string;
  secondary_condition_count?: number;
  equivalence_margin_bpm?: number;
  confidence_level?: number;
  minimum_independent_estimates?: number;
  threshold?: number;
  trial_count?: number;
  trial_duration_s?: number;
  conditions?: string[];
  reference?: string;
  metrics?: string[];
  required_routes?: string[];
}

export interface StudyObjectivesResponse {
  schema_version: string;
  product_version: string;
  confirmatory_conditions: string[];
  trials_per_condition: number;
  planned_duration_s: number;
  target_recruited_participants: number;
  minimum_protocol_complete_participants: number;
  objectives: StudyObjective[];
}

export type StudyProtocolState = 'draft' | 'locked' | 'superseded' | string;

export interface StudyProtocolCondition {
  condition_id: string;
  distance_m: number;
  barrier_type: BarrierType;
  trial_count: number;
  planned_duration_s: number;
  confirmatory?: boolean;
  [key: string]: unknown;
}

export interface StudyProtocol {
  schema_version?: string;
  protocol_id: string;
  protocol_version: string;
  state: StudyProtocolState;
  locked_at?: string | null;
  locked_by?: string | null;
  randomization_seed?: string | null;
  conditions: StudyProtocolCondition[];
  no_subject?: {
    trial_count: number;
    planned_duration_s: number;
    frozen_configuration?: Record<string, unknown> | null;
  };
  [key: string]: unknown;
}

export interface StudyProtocolResponse {
  ok?: boolean;
  protocol: StudyProtocol;
}

export interface StudyScheduleEntry {
  participant_id: string;
  order: number;
  condition_id: string;
  trial_numbers: number[];
  status?: string;
  seed?: string | null;
  [key: string]: unknown;
}

export interface StudyScheduleResponse {
  ok?: boolean;
  schema_version?: string;
  participant_id?: string;
  seed?: string | null;
  entries: StudyScheduleEntry[];
}

export type ReferenceObservationKind = 'rr_observer' | 'temperature' | 'hr' | string;

export interface ReferenceObservation {
  observation_id: string;
  session_id: string;
  kind: ReferenceObservationKind;
  observer_id?: string | null;
  value?: number | null;
  unit?: string | null;
  observed_at?: string | null;
  duration_s?: number | null;
  device_id?: string | null;
  calibration_id?: string | null;
  uncertainty?: number | null;
  locked?: boolean;
  missing_reason?: string | null;
  [key: string]: unknown;
}

export interface ReferenceObservationInput {
  kind: ReferenceObservationKind;
  observer_id?: string;
  value?: number;
  unit?: string;
  duration_s?: number;
  device_id?: string;
  calibration_id?: string;
  uncertainty?: number;
  observed_at?: string;
  missing_reason?: string;
  [key: string]: unknown;
}

export interface StudyReferencesResponse {
  ok?: boolean;
  schema_version?: string;
  session_id: string;
  references: ReferenceObservation[];
  rr_adjudication?: Record<string, unknown> | null;
}

export interface RrAdjudicationInput {
  final_value: number;
  rationale: string;
  actor?: string;
}

export interface StudyAnalysisRequest {
  objective_id?: string;
  session_ids?: string[];
  /** Canonical trainer values are gradient_boosting and cnn_1d; short aliases remain accepted at the API boundary. */
  model_family?: 'gradient_boosting' | 'cnn_1d' | 'gbr' | 'cnn' | string;
  confirmatory?: boolean;
}

export interface StudyAnalysisJob {
  job_id: string;
  status: 'queued' | 'running' | 'completed' | 'failed' | string;
  objective_id?: string | null;
  model_family?: 'gradient_boosting' | 'cnn_1d' | string;
  progress_pct?: number;
  phase?: string | null;
  last_line?: string | null;
  error?: string | null;
  cohort_selection?: string | null;
  statistics_status?: string | null;
  request?: StudyAnalysisRequest;
  created_at?: string;
  updated_at?: string;
  [key: string]: unknown;
}

export interface StudyAnalysisJobsResponse {
  ok?: boolean;
  jobs: StudyAnalysisJob[];
}

export interface StudyAnalysisResponse {
  ok?: boolean;
  job: StudyAnalysisJob;
}

export interface StudyObjectiveReport {
  ok?: boolean;
  objective_id: string;
  schema_version?: string;
  status: 'ready' | 'inconclusive' | 'descriptive' | 'blocked' | string;
  report?: Record<string, unknown> | null;
  exclusions?: Array<Record<string, unknown>>;
  provenance?: Record<string, unknown>;
  [key: string]: unknown;
}

export interface ParticipantStatusResponse {
  ok: boolean;
  schema_version?: string;
  profile: ParticipantProfile;
}

export interface BackendDefaults {
  sandbox?: boolean;
  radar_port?: string;
  serial_ports?: string[];
  ble_address?: string;
  ble_profile?: string;
  durations_s?: number[];
  sessions_root?: string;
  [key: string]: unknown;
}

export interface TrainerVersionResponse {
  product_version?: string;
  trainer?: string;
  dashboard?: string;
  firmware_expected?: string;
  firmware_observed?: string;
  serial_protocol?: string;
  serial_width_expected?: number;
  serial_width_observed?: number;
  schema_versions?: Record<string, string>;
  [key: string]: unknown;
}

export interface SessionListResponse {
  ok?: boolean;
  root?: string;
  sessions?: SessionRecord[];
  items?: SessionRecord[];
}

export interface SessionTrainingStatus {
  schema_version?: string;
  session_id: string;
  status?: string;
  target?: string;
  n_estimators_done?: number;
  n_estimators_total?: number;
  train_loss?: number | null;
  val_loss?: number | null;
  elapsed_s?: number;
  updated_at?: string;
  [key: string]: unknown;
}

export interface SessionPredictionResponse {
  ok: boolean;
  session_id: string;
  summary?: Record<string, unknown> | null;
  path?: string | null;
}

export interface SessionTagsResponse {
  ok: boolean;
  session_id: string;
  tags: string[];
}

export interface SessionDeleteResponse {
  ok: boolean;
  session_id: string;
  trashed_path?: string;
  retention_hint?: string;
}

export interface SessionBufferResponse {
  ok: boolean;
  schema_version?: string;
  session_id?: string;
  buffer_s?: number;
  payload?: LivePayload | null;
}

export interface SessionEventStreamOptions {
  sessionId?: string;
  token?: string;
}

export interface AlertEvent {
  id: string;
  ts: number;
  msg: string;
  severity: AlertSeverity;
  source?: string;
  seekTimestamp?: number;
  dismissed?: boolean;
  snoozedUntil?: number;
}

export interface SessionNotesPayload {
  schema_version?: string;
  session_id: string;
  review_summary: string;
  notes?: Array<Record<string, unknown>>;
  updated_at?: string;
}

export interface SessionSignoff {
  schema_version?: string;
  session_id: string;
  operator_name: string;
  initials: string;
  validation_comment: string;
  signed_at?: string | null;
  updated_at?: string | null;
}

export interface SerialPortRecord {
  device: string;
  label?: string;
}

export interface SnapshotRecord {
  id: string;
  ts: number;
  reported_hr: number;
  reported_rr: number;
  distance_cm: number;
  ble_hr: number;
  ble_rr: number;
  sortOrder?: number;
}

export interface SessionRecord {
  session_id: string;
  started_at?: string;
  started_ms?: number;
  duration_s?: number;
  subject?: string;
  subject_label?: string;
  operator?: string;
  operator_label?: string;
  verdict?: string | Record<string, unknown>;
  summary?: string;
  analysis_status?: string;
  sandbox?: boolean;
  mock?: boolean;
  params?: Record<string, unknown>;
  downloads?: DownloadRecord[];
  [key: string]: unknown;
}

export interface DownloadRecord {
  label?: string;
  relpath?: string;
  path?: string;
}

export interface PreflightCheck {
  id: string;
  label?: string;
  status: 'good' | 'warn' | 'bad' | 'fail' | string;
  description?: string;
  detail?: string;
  remediation?: string;
}

export function normalizePreflightStatus(check: Partial<PreflightCheck> | null | undefined): string {
  return String(check?.status ?? '').toLowerCase();
}

export interface SubjectProfileRecord {
  label: string;
  age_group?: string;
  fitness_level?: string;
  expected_hr_range?: [number, number];
  notes?: string;
}

export interface BleScanDevice {
  id?: string;
  address?: string;
  name?: string;
  rssi?: number;
}

export interface TelemetrySeries {
  ts?: number[];
  t?: number[];
  hr?: number[];
  rr?: number[];
  reported_hr?: number[];
  reported_rr?: number[];
  candidate_hr?: number[];
  candidate_rr?: number[];
  raw_hr?: number[];
  raw_hr_uncorrected?: number[];
  raw_hr_corrected?: number[];
  ble_hr?: number[];
  ble_rr?: number[];
  breath?: number[];
  heart?: number[];
  breath_phase?: number[];
  heart_phase?: number[];
  [key: string]: unknown;
}

export interface LivePayload {
  meta: Record<string, unknown> & {
    status?: string;
    elapsed_s?: number;
    remaining_s?: number;
    sandbox?: boolean;
    received_at_ms?: number;
    stale?: boolean;
  };
  radar: Record<string, unknown> & {
    reported_hr?: number;
    reported_rr?: number;
    candidate_hr?: number;
    candidate_rr?: number;
    raw_hr?: number;
    distance_cm?: number;
    motion?: number;
    human?: boolean;
    rows?: number;
    session_phase?: number;
    session_phase_name?: string;
    hr_locked_live?: number;
    hr_confidence?: number | null;
    hr_confidence_source_name?: string;
    logged_hr_valid?: number;
    logged_rr_valid?: number;
    doppler_motion?: number;
    pqi_heart?: number;
    pqi_breath?: number;
    loop_dt_mean_ms?: number;
    loop_dt_max_ms?: number;
    heap_free_kb?: number;
    heap_min_free_kb?: number;
    radar_uart_overflow_count?: number;
    radar_crc_err_count?: number;
    i2c_recover_count?: number;
    lcd_reinit_count?: number;
    wdt_near_miss_count?: number;
    cmd_rx_count?: number;
    cmd_err_count?: number;
    fw_uptime_s?: number;
  };
  ble: Record<string, unknown> & {
    address?: string;
    hr?: number;
    rr?: number;
    connected?: boolean;
    perf?: number | string;
  };
  thresholds: Record<string, unknown>;
  faults: Array<string | Record<string, unknown>>;
  events: Array<Record<string, unknown> | string>;
  series: TelemetrySeries;
  analysis: Record<string, unknown> | null;
}

export interface SessionDataPayload {
  ok: boolean;
  rows?: Array<Record<string, number | string | null>>;
  error?: { message?: string };
}

export interface ChartAnnotation {
  id: string;
  chart_key: string;
  label: string;
  xPct: number;
  elapsed_s?: number;
  created_at?: string;
  updated_at?: string;
}

export interface OperatorProfile {
  operator_id: string;
  display_name: string;
  initials: string;
}

export interface OperatorProfilesResponse {
  schema_version: string;
  profiles: OperatorProfile[];
}

export interface LoginResponse {
  token: string;
  expires_at: number;
  operator: OperatorProfile;
}

