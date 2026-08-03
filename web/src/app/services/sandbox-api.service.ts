import { Injectable, inject } from '@angular/core';
import {
  CompletionMatrix,
  LoginResponse,
  OperatorProfile,
  OperatorProfilesResponse,
  ParticipantProfile,
  SessionRecord,
  SessionSignoff
} from '../models/rvt.models';
import { PRODUCT_VERSION } from './app-meta';
import { OPERATOR_TOKEN_KEY, SANDBOX_OPERATOR_PROFILES_KEY } from './rvt-storage-keys';
import { SessionStore } from './stores/session.store';

interface SandboxOperatorProfile extends OperatorProfile {
  pin: string;
  failed_attempts?: number;
  locked_until?: number;
}

const SANDBOX_OPERATOR_SESSIONS_KEY = 'demo:rvt-operator-sessions';
const SANDBOX_OPERATOR_SESSION_TTL_MS = 8 * 60 * 60 * 1000;
const SANDBOX_PARTICIPANTS_KEY = 'demo:rvt-participants-v16.5.1';
const SANDBOX_SUBJECT_PROFILES_KEY = 'demo:rvt-subject-profiles-v16.5.9';
const SANDBOX_PROTOCOL_ATTEMPTS_KEY = 'demo:rvt-protocol-attempts-v16.5.9';

@Injectable({
  providedIn: 'root'
})
export class SandboxApiService {
  private readonly sessionStore = inject(SessionStore);

  ensureSessions(): SessionRecord[] {
    const existing = this.readSessions();
    if (!existing.length) {
      const iso = (mins: number) => new Date(Date.now() - mins * 60000).toISOString();
      const items = [
        { session_id: 'sandbox_20260420_091800', started_at: iso(64), duration_s: 480, subject: 'demo-A', operator: 'Demo operator', verdict: 'demo', summary: 'Completed 8 min simulated telemetry preview; no physiological conclusion.', sandbox: true },
        { session_id: 'sandbox_20260419_141803', started_at: iso(1240), duration_s: 300, subject: 'demo-B', operator: 'Demo operator', verdict: 'demo', summary: 'Calibration sandbox with stable BLE coverage and simulated gate audit.', sandbox: true }
      ] as SessionRecord[];
      this.sessionStore.sessionItems.set(items);
      return items;
    }
    return existing;
  }

  request(path: string, init?: RequestInit): unknown {
    const url = new URL(path, window.location.origin);
    const method = String(init?.method || 'GET').toUpperCase();
    if (url.pathname === '/api/status') return { ok: true, mode: 'sandbox', active_session: this.sessionStore.sessionActive() ? { session_id: this.sessionStore.currentSessionId() || 'sandbox_active', sandbox: true } : null };
    if (url.pathname === '/api/health') return { ok: true, version: 'sandbox' };
    if (url.pathname === '/api/version') return {
      product_version: PRODUCT_VERSION,
      trainer: PRODUCT_VERSION,
      dashboard: PRODUCT_VERSION,
      firmware_expected: `v${PRODUCT_VERSION}`,
      firmware_observed: `v${PRODUCT_VERSION}`,
      serial_protocol: 'v15.2',
      serial_width_expected: 222,
      serial_width_observed: 222,
      schema_versions: {
        control_api: 'rvt-control-api-v12.0',
        study_session: 'rvt-study-session-v16.5.1'
      }
    };
    if (url.pathname === '/api/auth/validate') return this.validateOperator();
    if (url.pathname === '/api/auth/login' && method === 'POST') {
      const login = this.login(init);
      if ('error' in login) throw new Error(`${login.error.message} (${login.error.code})`);
      return login;
    }
    if (url.pathname === '/api/auth/logout' && method === 'POST') return this.logoutOperator(init);
    if (url.pathname === '/api/auth/sse-token') return { sse_token: `sandbox_sse_${Date.now().toString(36)}` };
    if (url.pathname === '/api/operator-profiles' && method === 'GET') return this.operatorProfilesResponse();
    if (url.pathname === '/api/operator-profiles' && method === 'POST') {
      const created = this.createOperatorProfile(init);
      if (created.error) throw new Error(`${created.error.message} (${created.error.code})`);
      return created;
    }
    if (url.pathname === '/api/subject-profiles' && method === 'GET') {
      return { schema_version: 'rvt-subject-profiles-v12.0', profiles: this.readSubjectProfiles() };
    }
    if (url.pathname === '/api/subject-profiles' && method === 'PUT') {
      const body = this.parseJsonBody<Record<string, unknown>>(init?.body);
      const profiles = (body['profiles'] && typeof body['profiles'] === 'object' ? body['profiles'] : body) as Record<string, unknown>;
      localStorage.setItem(SANDBOX_SUBJECT_PROFILES_KEY, JSON.stringify(profiles));
      return { schema_version: 'rvt-subject-profiles-v12.0', profiles };
    }
    if (url.pathname === '/api/participants' && method === 'GET') {
      const participants = this.readParticipants();
      return { ok: true, participants, items: participants, completion_matrix: this.completionMatrix() };
    }
    if (url.pathname === '/api/participants' && method === 'POST') {
      return { ok: true, participant: this.createParticipant() };
    }
    if (url.pathname.startsWith('/api/participants/') && method === 'PUT') {
      const participantId = decodeURIComponent(url.pathname.split('/').pop() || '');
      const body = this.parseJsonBody<{ status?: string }>(init?.body);
      const participants = this.readParticipants().map(item => item.participant_id === participantId
        ? { ...item, status: String(body.status || item.status), status_history: [...(item.status_history || []), { to_status: body.status, changed_at: new Date().toISOString() }] }
        : item);
      const profile = participants.find(item => item.participant_id === participantId);
      if (!profile) throw new Error('Participant not found');
      localStorage.setItem(SANDBOX_PARTICIPANTS_KEY, JSON.stringify(participants.filter(item => item.participant_id !== 'P-DEMO')));
      return { ok: true, schema_version: 'rvt-participant-profiles-v16.5.9', profile };
    }
    if (url.pathname === '/api/study/completion-matrix') return this.completionMatrix();
    if (url.pathname === '/api/study/objectives') return this.studyObjectives();
    if (url.pathname === '/api/study/attempts' && method === 'POST') {
      const body = this.parseJsonBody<Record<string, unknown>>(init?.body);
      const attempts = this.readProtocolAttempts();
      const attempt = {
        schema_version: 'rvt-protocol-attempt-ledger-v16.5.9',
        attempt_id: `AT-sandbox-${Date.now().toString(36)}`,
        attempt_type: body['attempt_type'] || 'no_subject',
        participant_id: body['participant_id'] || null,
        condition_id: body['condition_id'] || null,
        trial_number: body['trial_number'] || null,
        status: body['status'] || 'allocated',
        terminal: true,
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
        events: []
      };
      attempts.push(attempt);
      localStorage.setItem(SANDBOX_PROTOCOL_ATTEMPTS_KEY, JSON.stringify(attempts));
      return { ok: true, schema_version: 'rvt-protocol-attempts-v16.5.9', attempt };
    }
    if (url.pathname === '/api/sessions') {
      const sessions = this.ensureSessions();
      return { ok: true, sessions, items: sessions };
    }
    if (url.pathname === '/api/session/current') return this.sessionStore.sessionActive() ? { session_id: this.sessionStore.currentSessionId() || 'sandbox_active', sandbox: true } : { ok: false, error: { code: 'NO_ACTIVE_SESSION' } };
    if (url.pathname === '/api/session/current/live_dashboard.json') return { meta: { sandbox: true, status: this.sessionStore.sessionActive() ? 'running' : 'waiting' } };
    if (url.pathname === '/api/session/start' && method === 'POST') {
      const id = `sandbox_${Date.now()}`;
      this.sessionStore.sessionActive.set(true);
      this.sessionStore.currentSessionId.set(id);
      return { ok: true, session_id: id, sandbox: true };
    }
    if (url.pathname === '/api/session/stop' && method === 'POST') {
      this.sessionStore.sessionActive.set(false);
      const id = this.sessionStore.currentSessionId() || `sandbox_${Date.now()}`;
      this.sessionStore.currentSessionId.set(null);
      const setup = this.sessionStore.setup();
      const session: SessionRecord = {
        session_id: id,
        started_at: new Date(Date.now() - setup.duration_s * 1000).toISOString(),
        duration_s: setup.duration_s,
        subject: setup.subject_label || setup.participant_id || 'P-DEMO',
        subject_label: setup.subject_label || setup.participant_id || 'P-DEMO',
        participant_id: setup.participant_id || 'P-DEMO',
        trial_id: `${setup.participant_id || 'P-DEMO'}-${setup.condition_id}-t${setup.trial_number}`,
        study_mode: setup.study_mode,
        study_classification: setup.study_mode,
        condition_id: setup.condition_id,
        distance_m: setup.distance_m,
        barrier_type: setup.barrier_type,
        trial_number: setup.trial_number,
        planned_duration_s: setup.duration_s,
        operator: setup.operator_label || 'Demo operator',
        verdict: 'demo',
        summary: 'Sandbox session stopped locally; trainer was unavailable.',
        sandbox: true
      };
      this.sessionStore.sessionItems.update(items => [session, ...items.filter(item => item.session_id !== id)]);
      return { ok: true, session_id: id, session, sandbox: true };
    }
    if (url.pathname.startsWith('/api/sessions/')) {
      const parts = url.pathname.split('/');
      const sessionId = decodeURIComponent(parts[3] || '');
      const session = this.ensureSessions().find(item => item.session_id === sessionId)
        || { session_id: sessionId, sandbox: true, verdict: 'demo', summary: 'Sandbox session summary.' };
      if (url.pathname.endsWith('/summary')) {
        return {
          ...session,
          sandbox: true,
          signal_quality: { pqi_lock_pct: 84.2, session_quality_score: 8.7, internal_consistency_score: 9.1, coverage_locked: 81.5, coverage_settling: 11.2 },
          hr_metrics: { rmse: 2.41, mae: 1.92, bias: -0.4, coverage_pct: 88.1 },
          rr_metrics: { rmse: 0.82, mae: 0.61, bias: 0.1, coverage_pct: 90.4 },
          gates: {
            coverage: { passed: true, status: 'pass' },
            agreement: { passed: true, status: 'pass' },
            motion: { passed: true, status: 'pass' },
            reference: { passed: true, status: 'pass' }
          },
          verdict: {
            verdict: 'ready',
            readiness_kind: 'ready',
            categories: []
          }
        };
      }
      if (url.pathname.endsWith('/data')) {
        return {
          rows: [
            { t: 0, reported_hr: 72, reported_rr: 14 },
            { t: 1, reported_hr: 73, reported_rr: 15 },
            { t: 2, reported_hr: 71, reported_rr: 14 }
          ]
        };
      }
      if (url.pathname.endsWith('/compare')) return { selected: session, previous: null, best: session };
      if (url.pathname.endsWith('/analyse/status')) {
        return { status: 'complete', progress_pct: 100, last_line: 'Sandbox analysis complete.' };
      }
      if (url.pathname.endsWith('/training/status')) {
        return { schema_version: 'rvt-training-progress-v16.5.9', session_id: sessionId, status: 'complete', target: 'hr,rr', n_estimators_done: 100, n_estimators_total: 100, elapsed_s: 0.2 };
      }
      if (url.pathname.endsWith('/predict')) {
        return { ok: true, session_id: sessionId, summary: { model_family: 'gradient_boosting', status: 'complete', sandbox: true }, path: null };
      }
      if (url.pathname.endsWith('/tags') && method === 'PUT') {
        const body = this.parseJsonBody<{ tags?: string[] }>(init?.body);
        return { ok: true, session_id: sessionId, tags: Array.isArray(body.tags) ? body.tags : [] };
      }
      if (url.pathname.endsWith('/events')) return { ok: true, session_id: sessionId, sandbox: true };
      if (url.pathname.endsWith('/notes')) {
        const body = method === 'PUT' ? this.parseJsonBody<{ review_summary?: string }>(init?.body) : {};
        return { review_summary: body.review_summary || session.summary || '', sandbox: true };
      }
      if (url.pathname.endsWith('/signoff')) {
        const body = method === 'PUT' ? this.parseJsonBody<SessionSignoff>(init?.body) : {};
        return {
          session_id: sessionId,
          operator_name: body.operator_name || '',
          initials: body.initials || '',
          validation_comment: body.validation_comment || '',
          signed_at: method === 'PUT' ? new Date().toISOString() : null,
          sandbox: true
        };
      }
      if (method === 'DELETE' && url.pathname.split('/').length === 4) {
        this.sessionStore.sessionItems.update(items => items.filter(item => item.session_id !== sessionId));
        return { ok: true, session_id: sessionId, sandbox: true };
      }
    }
    if (url.pathname === '/api/defaults' && method === 'POST') return { ...this.sandboxDefaults(), ...this.parseJsonBody<Record<string, unknown>>(init?.body) };
    if (url.pathname === '/api/defaults') return this.sandboxDefaults();
    if (url.pathname === '/api/preflight') return { ok: true, checks: [
      { id: 'trainer', label: 'Trainer link', status: 'good', description: 'Demo trainer reachable — simulated control plane.' },
      { id: 'radar', label: 'Radar serial', status: 'good', description: 'COM4 — XIAO ESP32-S3 detected.' },
      { id: 'firmware', label: 'Firmware contract', status: 'good', description: 'Simulated 222-column contract intact.' },
      { id: 'ble', label: 'BLE reference', status: 'good', description: 'AiLink oximeter paired (simulated).' },
      { id: 'placement', label: 'Subject placement', status: 'good', description: 'Subject within the radar sweet spot.' },
      { id: 'coverage', label: 'Reference coverage', status: 'warn', description: 'Simulated BLE coverage 72% — review before trusting agreement.', remediation: 'Keep the oximeter within range for the full session.' }
    ] };
    if (url.pathname === '/api/ble/scan') return { ok: true, devices: [] };
    return { ok: true, sandbox: true };
  }

  private readSessions(): SessionRecord[] {
    return this.sessionStore.sessionItems().filter(item => item.sandbox || item.session_id.startsWith('sandbox_'));
  }

  private readParticipants(): ParticipantProfile[] {
    const demo: ParticipantProfile = {
      participant_id: 'P-DEMO',
      display_code: 'P-DEMO',
      status: 'active',
      completed_trials: 0
    };
    try {
      const stored = JSON.parse(localStorage.getItem(SANDBOX_PARTICIPANTS_KEY) || '[]');
      if (!Array.isArray(stored)) return [demo];
      const valid = stored.filter((item): item is ParticipantProfile =>
        Boolean(item && typeof item === 'object' && item.participant_id && item.display_code)
      );
      return [demo, ...valid.filter(item => item.participant_id !== demo.participant_id)].slice(0, 41);
    } catch {
      return [demo];
    }
  }

  private readSubjectProfiles(): Record<string, unknown> {
    try {
      const parsed = JSON.parse(localStorage.getItem(SANDBOX_SUBJECT_PROFILES_KEY) || '{}');
      return parsed && typeof parsed === 'object' ? parsed as Record<string, unknown> : {};
    } catch {
      return {};
    }
  }

  private readProtocolAttempts(): Array<Record<string, unknown>> {
    try {
      const parsed = JSON.parse(localStorage.getItem(SANDBOX_PROTOCOL_ATTEMPTS_KEY) || '[]');
      return Array.isArray(parsed) ? parsed.filter(item => item && typeof item === 'object') as Array<Record<string, unknown>> : [];
    } catch {
      return [];
    }
  }

  private completionMatrix(): CompletionMatrix {
    const conditions = ['d060_none', 'd080_none', 'd100_none', 'd060_cardboard', 'd080_cardboard', 'd100_cardboard'];
    const trials = [1, 2, 3];
    const attempts = this.readProtocolAttempts();
    const participants: CompletionMatrix['participants'] = {};
    for (const participant of this.readParticipants()) {
      const cells: Record<string, { status: string; attempt_type: string; attempt_id: string | null }> = {};
      let completed = 0;
      for (const condition of conditions) {
        for (const trial of trials) {
          const row = [...attempts].reverse().find(item => item['participant_id'] === participant.participant_id && item['condition_id'] === condition && Number(item['trial_number']) === trial);
          const status = String(row?.['status'] || 'missing');
          if (status === 'completed') completed++;
          cells[`${condition}:t${trial}`] = { status, attempt_type: String(row?.['attempt_type'] || 'subject'), attempt_id: row?.['attempt_id'] ? String(row['attempt_id']) : null };
        }
      }
      participants[participant.participant_id] = {
        participant_id: participant.participant_id,
        status: participant.status,
        completed_trials: completed,
        expected_trials: conditions.length * trials.length,
        protocol_complete: completed === conditions.length * trials.length,
        cells
      };
    }
    return {
      schema_version: 'rvt-protocol-attempts-v16.5.9',
      conditions,
      trials,
      participants,
      participant_count: Object.keys(participants).length,
      protocol_complete_participant_count: Object.values(participants).filter(item => item.protocol_complete).length,
      no_subject_attempt_count: attempts.filter(item => item['attempt_type'] === 'no_subject').length,
      no_subject_expected: 72,
      attempt_count: attempts.length
    };
  }

  private sandboxDefaults(): Record<string, unknown> {
    return { sandbox: true, radar_port: 'COM4', ble_address: '10:22:33:9E:8F:63', ble_profile: 'ailink_oximeter' };
  }

  private studyObjectives(): Record<string, unknown> {
    const conditions = ['d060_none', 'd080_none', 'd100_none', 'd060_cardboard', 'd080_cardboard', 'd100_cardboard'];
    return {
      schema_version: 'rvt-study-objectives-v16.5.9',
      product_version: '16.5.9',
      confirmatory_conditions: conditions,
      trials_per_condition: 3,
      planned_duration_s: 150,
      target_recruited_participants: 40,
      minimum_protocol_complete_participants: 38,
      objectives: [
        { id: 'objective_1_rr', number: 1, outcome: 'rr', label: 'GBR-assisted respiration-rate equivalence', role: 'confirmatory', primary_condition_id: 'd100_none', equivalence_margin_bpm: 2, confidence_level: 0.9, minimum_independent_estimates: 19 },
        { id: 'objective_2_temperature', number: 2, outcome: 'temperature', label: 'Unobstructed skin-surface-temperature agreement', role: 'exploratory' },
        { id: 'objective_3_false_alarm', number: 3, outcome: 'false_alarm', label: 'No-subject false-alarm rate', role: 'confirmatory', threshold: 0.05, trial_count: 72, trial_duration_s: 150 },
        { id: 'objective_4_hr', number: 4, outcome: 'hr', label: 'GBR-assisted heart-rate accuracy and agreement', role: 'exploratory', conditions }
      ]
    };
  }

  private createParticipant(): ParticipantProfile {
    const participants = this.readParticipants();
    if (participants.filter(item => item.participant_id !== 'P-DEMO').length >= 40) {
      throw new Error('The 40-participant registry is full.');
    }
    const used = new Set(participants.map(item => item.participant_id));
    let sequence = 1;
    while (used.has(`P-${String(sequence).padStart(3, '0')}`)) sequence++;
    const participant: ParticipantProfile = {
      participant_id: `P-${String(sequence).padStart(3, '0')}`,
      display_code: `P-${String(sequence).padStart(3, '0')}`,
      status: 'active',
      completed_trials: 0,
      created_at: new Date().toISOString()
    };
    const stored = [...participants.filter(item => item.participant_id !== 'P-DEMO'), participant];
    localStorage.setItem(SANDBOX_PARTICIPANTS_KEY, JSON.stringify(stored));
    return participant;
  }

  private readOperatorProfiles(): SandboxOperatorProfile[] {
    try {
      const payload = JSON.parse(localStorage.getItem(SANDBOX_OPERATOR_PROFILES_KEY) || '{}');
      const profiles = Array.isArray(payload?.profiles) ? payload.profiles : [];
      return profiles
        .filter((profile: Partial<SandboxOperatorProfile>) =>
          typeof profile.operator_id === 'string'
          && typeof profile.display_name === 'string'
          && typeof profile.initials === 'string'
          && typeof profile.pin === 'string'
        )
        .map((profile: SandboxOperatorProfile) => ({
          operator_id: profile.operator_id,
          display_name: profile.display_name,
          initials: profile.initials,
          pin: profile.pin,
          failed_attempts: Number(profile.failed_attempts || 0),
          locked_until: Number(profile.locked_until || 0)
        }));
    } catch (_) {
      return [];
    }
  }

  private writeOperatorProfiles(profiles: SandboxOperatorProfile[]): void {
    try {
      localStorage.setItem(SANDBOX_OPERATOR_PROFILES_KEY, JSON.stringify({
        schema_version: 'rvt-sandbox-operator-profiles-v12.0',
        profiles
      }));
    } catch (_) {}
  }

  private publicOperatorProfiles(): OperatorProfile[] {
    return this.readOperatorProfiles().map(({ operator_id, display_name, initials }) => ({
      operator_id,
      display_name,
      initials
    }));
  }

  private readOperatorSessions(): Record<string, { operator_id: string; expires_at: number }> {
    try {
      const payload = JSON.parse(sessionStorage.getItem(SANDBOX_OPERATOR_SESSIONS_KEY) || '{}');
      return payload && typeof payload === 'object' ? payload : {};
    } catch (_) {
      return {};
    }
  }

  private writeOperatorSessions(sessions: Record<string, { operator_id: string; expires_at: number }>): void {
    try {
      sessionStorage.setItem(SANDBOX_OPERATOR_SESSIONS_KEY, JSON.stringify(sessions));
    } catch (_) {}
  }

  private operatorProfilesResponse(): OperatorProfilesResponse {
    return {
      schema_version: 'rvt-sandbox-operator-profiles-v12.0',
      profiles: this.publicOperatorProfiles()
    };
  }

  private createOperatorProfile(init?: RequestInit): { ok: boolean; operator?: OperatorProfile; error?: { code: string; message: string } } {
    const body = this.parseJsonBody<{ display_name?: string; initials?: string; pin?: string }>(init?.body);
    const displayName = String(body.display_name || '').trim();
    const initials = String(body.initials || '').trim().toUpperCase();
    const pin = String(body.pin || '').trim();
    if (displayName.length < 3 || displayName.length > 64) {
      return { ok: false, error: { code: 'VALIDATION_FAILED', message: 'display_name must be 3 to 64 characters' } };
    }
    if (!/^[A-Z]{2,5}$/.test(initials)) {
      return { ok: false, error: { code: 'VALIDATION_FAILED', message: 'initials must be 2 to 5 uppercase letters' } };
    }
    if (!/^\d{6}$/.test(pin)) {
      return { ok: false, error: { code: 'VALIDATION_FAILED', message: 'pin must be exactly 6 digits' } };
    }
    const profiles = this.readOperatorProfiles();
    const operator: SandboxOperatorProfile = {
      operator_id: `sandbox_op_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`,
      display_name: displayName,
      initials,
      pin,
      failed_attempts: 0,
      locked_until: 0
    };
    this.writeOperatorProfiles([...profiles, operator]);
    const { pin: _pin, failed_attempts: _failed, locked_until: _locked, ...publicOperator } = operator;
    return { ok: true, operator: publicOperator };
  }

  private login(init?: RequestInit): LoginResponse | { ok: false; error: { code: string; message: string; retry_after_s?: number } } {
    const body = this.parseJsonBody<{ operator_id?: string; pin?: string }>(init?.body);
    const operatorId = String(body.operator_id || '').trim();
    const pin = String(body.pin || '').trim();
    const profiles = this.readOperatorProfiles();
    const profile = profiles.find(item => item.operator_id === operatorId);
    if (!profile) {
      return { ok: false, error: { code: 'UNAUTHORIZED', message: 'Invalid operator ID or PIN' } };
    }
    const now = Date.now();
    const lockedUntil = Number(profile.locked_until || 0);
    if (lockedUntil > now) {
      const retryAfter = Math.max(1, Math.ceil((lockedUntil - now) / 1000));
      return {
        ok: false,
        error: {
          code: 'LOCKOUT_ACTIVE',
          message: `Too many failed attempts. Try again in ${retryAfter} seconds.`,
          retry_after_s: retryAfter
        }
      };
    }
    if (profile.pin !== pin) {
      profile.failed_attempts = Number(profile.failed_attempts || 0) + 1;
      if (profile.failed_attempts >= 5) {
        profile.locked_until = now + 30_000;
      }
      this.writeOperatorProfiles(profiles);
      if (profile.failed_attempts >= 5) {
        return {
          ok: false,
          error: {
            code: 'LOCKOUT_ACTIVE',
            message: 'Too many failed attempts. Try again in 30 seconds.',
            retry_after_s: 30
          }
        };
      }
      return { ok: false, error: { code: 'UNAUTHORIZED', message: 'Invalid operator ID or PIN' } };
    }
    profile.failed_attempts = 0;
    profile.locked_until = 0;
    this.writeOperatorProfiles(profiles);
    const expiresAt = now + SANDBOX_OPERATOR_SESSION_TTL_MS;
    const token = `sandbox_op_token_${operatorId}_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 10)}`;
    const sessions = this.readOperatorSessions();
    sessions[token] = { operator_id: operatorId, expires_at: expiresAt };
    this.writeOperatorSessions(sessions);
    const operator = { operator_id: profile.operator_id, display_name: profile.display_name, initials: profile.initials };
    return { token, expires_at: Math.floor(expiresAt / 1000), operator };
  }

  private validateOperator(): { ok: boolean; bootstrap?: boolean; operator: OperatorProfile | null } {
    const token = this.operatorToken();
    const sessions = this.readOperatorSessions();
    const session = token ? sessions[token] : null;
    const now = Date.now();
    if (session && Number(session.expires_at || 0) > now) {
      const operator = this.publicOperatorProfiles().find(item => item.operator_id === session.operator_id);
      if (operator) {
        return { ok: true, operator };
      }
    }
    if (token && sessions[token]) {
      delete sessions[token];
      this.writeOperatorSessions(sessions);
    }
    return { ok: true, bootstrap: this.publicOperatorProfiles().length === 0, operator: null };
  }

  private operatorToken(): string {
    try {
      return sessionStorage.getItem(OPERATOR_TOKEN_KEY) || '';
    } catch (_) {
      return '';
    }
  }

  private tokenFromRequestInit(init?: RequestInit): string {
    try {
      return new Headers(init?.headers || {}).get('X-RVT-Auth') || this.operatorToken();
    } catch (_) {
      return this.operatorToken();
    }
  }

  private logoutOperator(init?: RequestInit): { ok: true } {
    const token = this.tokenFromRequestInit(init);
    if (token) {
      const sessions = this.readOperatorSessions();
      delete sessions[token];
      this.writeOperatorSessions(sessions);
    }
    return { ok: true };
  }

  private parseJsonBody<T extends object>(body: BodyInit | null | undefined): Partial<T> {
    if (typeof body !== 'string') return {};
    try {
      const parsed = JSON.parse(body);
      return typeof parsed === 'object' && parsed !== null ? parsed as Partial<T> : {};
    } catch (_) {
      return {};
    }
  }
}
