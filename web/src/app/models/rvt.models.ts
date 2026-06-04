export type ThemeId = 'light' | 'dark' | 'night' | 'hc';
export type DensityId = 'comfortable' | 'compact';
export type HapticMode = 'on' | 'off' | 'auto';
export type AlertSeverity = 'info' | 'warn' | 'critical';
export type StorageScope = 'demo' | 'live' | 'legacy-unclassified';

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
  skip_countdown: boolean;
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
