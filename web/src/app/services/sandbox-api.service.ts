import { Injectable, inject } from '@angular/core';
import {
  CompletionMatrix,
  LoginResponse,
  OperatorProfile,
  OperatorProfilesResponse,
  ParticipantProfile,
  ReferenceObservation,
  StudyAnalysisRequest,
  StudyObjectiveExportResponse,
  StudyObjectiveReport,
  StudyProtocol,
  SessionRecord,
  SessionSignoff,
  StudyReferencesResponse
} from '../models/rvt.models';
import { PRODUCT_VERSION } from './app-meta';
import { OPERATOR_TOKEN_KEY, SANDBOX_OPERATOR_PROFILES_KEY } from './rvt-storage-keys';
import { SessionStore } from './stores/session.store';

interface SandboxOperatorProfile extends OperatorProfile {
  pin: string;
  failed_attempts?: number;
  locked_until?: number;
  recovery_code?: string;
  recovery_failed_attempts?: number;
  recovery_locked_until?: number;
}

const SANDBOX_RECOVERY_ALPHABET = 'ABCDEFGHJKMNPQRSTVWXYZ23456789';

const SANDBOX_OPERATOR_SESSIONS_KEY = 'demo:rvt-operator-sessions';
const SANDBOX_OPERATOR_SESSION_TTL_MS = 8 * 60 * 60 * 1000;
const SANDBOX_PARTICIPANTS_KEY = 'demo:rvt-participants-v16.5.9';
const SANDBOX_SUBJECT_PROFILES_KEY = 'demo:rvt-subject-profiles-v16.5.9';
const SANDBOX_PROTOCOL_ATTEMPTS_KEY = 'demo:rvt-protocol-attempts-v16.5.9';
const LEGACY_SANDBOX_STUDY_PROTOCOL_KEY = 'demo:rvt-study-protocol-v16.5.9';
const SANDBOX_STUDY_PROTOCOL_KEY = 'demo:rvt-study-protocol-v16.6.5';
const SANDBOX_STUDY_ANALYSIS_KEY = 'demo:rvt-study-analysis-v16.5.9';
const LEGACY_SANDBOX_STUDY_SCHEDULE_PREFIX = 'demo:rvt-study-schedule-v16.5.9:';
const SANDBOX_STUDY_SCHEDULE_PREFIX = 'demo:rvt-study-schedule-v16.6.5:';
const SANDBOX_CONFIRMATORY_CONDITIONS = [
  'd060_none',
  'd080_none',
  'd100_none',
  'd200_none',
  'd300_none',
  'd060_cardboard',
  'd080_cardboard',
  'd100_cardboard',
  'd200_cardboard',
  'd300_cardboard'
];

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
        study_session: 'rvt-study-session-v16.5.9'
      }
    };
    if (url.pathname === '/api/auth/validate') return this.validateOperator();
    if (url.pathname === '/api/auth/login' && method === 'POST') {
      const login = this.login(init);
      if ('error' in login) throw new Error(`${login.error.message} (${login.error.code})`);
      return login;
    }
    if (url.pathname === '/api/auth/logout' && method === 'POST') return this.logoutOperator(init);
    if (url.pathname === '/api/auth/reset-pin' && method === 'POST') {
      const reset = this.resetPinWithRecovery(init);
      if ('error' in reset && reset.error) throw new Error(`${reset.error.message} (${reset.error.code})`);
      return reset;
    }
    if (url.pathname === '/api/auth/host-reset' && method === 'POST') {
      const reset = this.hostResetPin(init);
      if ('error' in reset && reset.error) throw new Error(`${reset.error.message} (${reset.error.code})`);
      return reset;
    }
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
      localStorage.setItem(SANDBOX_PARTICIPANTS_KEY, JSON.stringify(participants));
      return { ok: true, schema_version: 'rvt-participant-profiles-v16.5.9', profile };
    }
    if (url.pathname === '/api/study/completion-matrix') return this.completionMatrix();
    if (url.pathname === '/api/study/objectives') return this.studyObjectives();
    if (url.pathname === '/api/study/protocol' && method === 'GET') return { ok: true, protocol: this.studyProtocol() };
    if (url.pathname === '/api/study/readiness' && method === 'GET') {
      return {
        ok: true,
        schema_version: 'rvt-collection-readiness-v1',
        authorized: false,
        protocol_state: this.studyProtocol().state,
        plan_status: 'draft',
        authorization_record_present: false,
        blockers: ['authorization_record_missing_or_invalid', 'statistical_analysis_plan_not_approved'],
        unresolved_withdrawal_count: 0
      };
    }
    if (url.pathname === '/api/study/protocol' && method === 'PUT') {
      const current = this.studyProtocol();
      if (current.state === 'locked') throw new Error('study protocol is locked and cannot be edited');
      const body = this.parseJsonBody<Partial<StudyProtocol>>(init?.body);
      const conditions = Array.isArray(body.conditions) ? body.conditions : current.conditions;
      const conditionIds = conditions.map(item => String(item?.condition_id || ''));
      if (!this.hasCanonicalSandboxConditions(conditionIds)) {
        throw new Error('protocol conditions must contain the ten canonical condition IDs');
      }
      const state = body.state || current.state;
      if (state !== 'draft' && state !== 'locked') throw new Error('protocol state must be draft or locked');
      const protocol = {
        ...current,
        ...body,
        conditions,
        state,
        schema_version: 'rvt-study-protocol-v2',
        updated_at: new Date().toISOString(),
        ...(state === 'locked' ? { locked_at: new Date().toISOString(), locked_by: String(body['actor'] || '') || null } : {})
      } as StudyProtocol;
      localStorage.setItem(SANDBOX_STUDY_PROTOCOL_KEY, JSON.stringify(protocol));
      if (protocol.randomization_seed !== current.randomization_seed) {
        this.clearStoredSchedulesByPrefix(SANDBOX_STUDY_SCHEDULE_PREFIX);
      }
      return { ok: true, protocol };
    }
    if (url.pathname === '/api/study/schedule') {
      const participantId = url.searchParams.get('participant_id') || 'P-DEMO';
      return this.studySchedule(participantId);
    }
    if (url.pathname === '/api/study/analysis' && method === 'GET') {
      try {
        return { ok: true, jobs: [this.readSandboxStudyAnalysis('')] };
      } catch {
        return { ok: true, jobs: [] };
      }
    }
    if (url.pathname === '/api/study/analysis' && method === 'POST') {
      const input = this.parseJsonBody<StudyAnalysisRequest>(init?.body);
      const requestedFamily = String(input.model_family || 'gradient_boosting').toLowerCase();
      const modelFamily = requestedFamily === 'gbr' ? 'gradient_boosting' : requestedFamily === 'cnn' ? 'cnn_1d' : requestedFamily;
      if (!['gradient_boosting', 'cnn_1d'].includes(modelFamily)) throw new Error('model_family must be gradient_boosting/gbr or cnn_1d/cnn');
      const job = {
        job_id: `demo-study-${Date.now().toString(36)}`,
        status: 'completed',
        objective_id: input.objective_id || null,
        model_family: modelFamily,
        request: {
          ...input,
          session_ids: [],
          confirmatory: input.objective_id === 'objective_1_rr' || input.objective_id === 'objective_3_false_alarm'
        },
        progress_pct: 100,
        statistics_status: 'blocked',
        last_line: 'Sandbox analysis is descriptive only; no confirmatory statistics are fabricated.',
        created_at: new Date().toISOString(),
        updated_at: new Date().toISOString(),
        sandbox: true
      };
      localStorage.setItem(SANDBOX_STUDY_ANALYSIS_KEY, JSON.stringify(job));
      return { ok: true, job };
    }
    if (url.pathname.startsWith('/api/study/analysis/')) {
      if (method === 'DELETE') {
        const existing = this.readSandboxStudyAnalysis(url.pathname.split('/').pop() || '');
        const job = { ...existing, status: 'cancelled', last_line: 'Sandbox analysis cancelled.', updated_at: new Date().toISOString() };
        localStorage.setItem(SANDBOX_STUDY_ANALYSIS_KEY, JSON.stringify(job));
        return { ok: true, job };
      }
      const job = this.readSandboxStudyAnalysis(url.pathname.split('/').pop() || '');
      return { ok: true, job };
    }
    if (url.pathname.startsWith('/api/study/objectives/') && url.pathname.endsWith('/report')) {
      const objectiveId = decodeURIComponent(url.pathname.split('/')[4] || '');
      return this.sandboxObjectiveReport(objectiveId);
    }
    if (url.pathname.startsWith('/api/study/objectives/') && url.pathname.endsWith('/export')) {
      const objectiveId = decodeURIComponent(url.pathname.split('/')[4] || '');
      const fmt = (url.searchParams.get('format') || 'latex').toLowerCase() === 'csv' ? 'csv' : 'latex';
      return this.sandboxObjectiveExport(objectiveId, fmt);
    }
    if (url.pathname === '/api/study/attempts' && method === 'POST') {
      const body = this.parseJsonBody<Record<string, unknown>>(init?.body);
      const attempts = this.readProtocolAttempts();
      const status = String(body['status'] || 'allocated').toLowerCase();
      const terminal = ['completed', 'stopped', 'failed_start', 'aborted', 'invalid', 'no_output'].includes(status);
      const attempt = {
        schema_version: 'rvt-protocol-attempt-ledger-v16.5.9',
        attempt_id: `AT-sandbox-${Date.now().toString(36)}`,
        attempt_type: body['attempt_type'] || 'no_subject',
        participant_id: body['participant_id'] || null,
        condition_id: body['condition_id'] || null,
        trial_number: body['trial_number'] || null,
        status,
        session_id: body['session_id'] || null,
        duration_s: body['duration_s'] ?? null,
        frozen_configuration_hash: body['frozen_configuration_hash'] || null,
        false_alarm_count: body['false_alarm_count'] ?? null,
        terminal,
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
        model_family: setup.model_family,
        model_bundle: setup.model_bundle || undefined,
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
      if (url.pathname.endsWith('/references') && method === 'GET') return this.sessionReferences(sessionId);
      if (url.pathname.endsWith('/references') && method === 'POST') {
        const body = this.parseJsonBody<Record<string, unknown>>(init?.body);
        const kind = String(body['kind'] || 'rr_observer').toLowerCase();
        if (kind === 'temperature' && String(body['barrier_type'] || 'none').toLowerCase() === 'cardboard') {
          throw new Error('temperature reference capture is limited to unobstructed trials');
        }
        const existing = this.sessionReferences(sessionId);
        const observerId = String(body['observer_id'] || body['actor'] || '').trim();
        if (kind === 'rr_observer' && observerId) {
          const dup = existing.references.some(
            item => item.kind === 'rr_observer' && String(item.observer_id || '').trim().toLowerCase() === observerId.toLowerCase()
          );
          if (dup) {
            throw new Error(`duplicate rr_observer submission for observer_id ${observerId}`);
          }
        }
        const observation = {
          observation_id: `OBS-${Date.now().toString(36)}`,
          session_id: sessionId,
          kind,
          ...body,
          ...(observerId ? { observer_id: observerId } : {}),
          locked: true,
          observed_at: body['observed_at'] || new Date().toISOString()
        } as ReferenceObservation;
        const references = [...existing.references, observation];
        localStorage.setItem(`demo:rvt-references:${sessionId}`, JSON.stringify(references));
        return { ok: true, schema_version: 'rvt-reference-observations-v1', session_id: sessionId, references, rr_adjudication: existing.rr_adjudication || null } satisfies StudyReferencesResponse;
      }
      if (url.pathname.endsWith('/references/rr-adjudication') && method === 'POST') {
        const body = this.parseJsonBody<Record<string, unknown>>(init?.body);
        const existing = this.sessionReferences(sessionId);
        const lockedRr = existing.references.filter(item => item.kind === 'rr_observer' && item.locked);
        if (lockedRr.length < 2) throw new Error('RR adjudication requires two locked observer submissions');
        const response = { ok: true, schema_version: 'rvt-reference-observations-v1', session_id: sessionId, references: existing.references, rr_adjudication: { ...body, locked_at: new Date().toISOString(), sandbox: true } } satisfies StudyReferencesResponse;
        localStorage.setItem(`demo:rvt-references:${sessionId}:adjudication`, JSON.stringify(response.rr_adjudication));
        return response;
      }
      if (url.pathname.endsWith('/summary')) {
        const tags = this.readSessionTags(sessionId);
        return {
          ...session,
          sandbox: true,
          tags,
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
        const pointsRequested = Math.max(1, Math.min(10000, Number(url.searchParams.get('points') || 1000) || 1000));
        const rows = [
          { timestamp_s: 0, timestamp_ms: 0, reported_hr: 72, reported_rr: 14, ref_hr: 72, ble_hr: 72, ref_rr: 14, ble_rr: 14 },
          { timestamp_s: 1, timestamp_ms: 1000, reported_hr: 73, reported_rr: 15, ref_hr: 73, ble_hr: 73, ref_rr: 15, ble_rr: 15 },
          { timestamp_s: 2, timestamp_ms: 2000, reported_hr: 71, reported_rr: 14, ref_hr: 71, ble_hr: 71, ref_rr: 14, ble_rr: 14 }
        ];
        return {
          ok: true,
          schema_version: 'rvt-control-api-v12.0',
          session_id: sessionId,
          points_requested: pointsRequested,
          rows_returned: rows.length,
          rows
        };
      }
      if (url.pathname.endsWith('/compare')) return { selected: session, previous: null, best: session };
      if (url.pathname.endsWith('/analyse') && method === 'POST') {
        return { ok: true, session_id: sessionId, status: 'complete', progress_pct: 100, sandbox: true };
      }
      if (url.pathname.endsWith('/analyse/status')) {
        return { status: 'complete', progress_pct: 100, last_line: 'Sandbox analysis complete.' };
      }
      if (url.pathname.endsWith('/training/status')) {
        return { schema_version: 'rvt-training-progress-v16.5.9', session_id: sessionId, status: 'complete', target: 'hr,rr', n_estimators_done: 100, n_estimators_total: 100, elapsed_s: 0.2 };
      }
      if (url.pathname.endsWith('/predict')) {
        return { ok: true, session_id: sessionId, summary: { model_family: 'gradient_boosting', status: 'complete', sandbox: true }, path: null };
      }
      if (url.pathname.includes('/files/')) {
        const rel = decodeURIComponent(url.pathname.split('/files/')[1] || '');
        return {
          ok: true,
          session_id: sessionId,
          filename: rel || 'artifact.txt',
          content: `Sandbox preview artifact for ${sessionId}: ${rel || 'artifact.txt'}`,
          sandbox: true
        };
      }
      if (url.pathname.endsWith('/tags') && method === 'PUT') {
        const body = this.parseJsonBody<{ tags?: string[] }>(init?.body);
        const tags = Array.isArray(body.tags) ? body.tags.map(t => String(t).trim()).filter(Boolean) : [];
        localStorage.setItem(`demo:rvt-tags:${sessionId}`, JSON.stringify(tags));
        return { ok: true, session_id: sessionId, tags };
      }
      if (url.pathname.endsWith('/events')) return { ok: true, session_id: sessionId, sandbox: true };
      if (url.pathname.endsWith('/notes')) {
        const notesKey = `demo:rvt-notes:${sessionId}`;
        if (method === 'PUT') {
          const body = this.parseJsonBody<{ review_summary?: string; notes?: Array<Record<string, unknown>> }>(init?.body);
          const stored = {
            schema_version: 'rvt-session-notes-v12.0',
            session_id: sessionId,
            review_summary: String(body.review_summary ?? ''),
            notes: Array.isArray(body.notes) ? body.notes : this.readJsonStorage<{ notes?: Array<Record<string, unknown>> }>(notesKey)?.notes || [],
            updated_at: new Date().toISOString(),
            sandbox: true
          };
          localStorage.setItem(notesKey, JSON.stringify(stored));
          return stored;
        }
        const existing = this.readJsonStorage<{ review_summary?: string; notes?: Array<Record<string, unknown>>; updated_at?: string }>(notesKey);
        return {
          schema_version: 'rvt-session-notes-v12.0',
          session_id: sessionId,
          updated_at: existing?.updated_at ?? null,
          review_summary: existing?.review_summary ?? (session.summary || ''),
          notes: Array.isArray(existing?.notes) ? existing.notes : [],
          sandbox: true
        };
      }
      if (url.pathname.endsWith('/signoff')) {
        const signoffKey = `demo:rvt-signoff:${sessionId}`;
        if (method === 'PUT') {
          const body = this.parseJsonBody<SessionSignoff>(init?.body);
          const operator_name = String(body.operator_name || '').trim();
          const initials = String(body.initials || '').trim().toUpperCase();
          if (!operator_name || !/^[A-Z]{2,5}$/.test(initials)) {
            throw new Error('operator_name and 2-5 uppercase initials are required');
          }
          const now = new Date().toISOString();
          const signoff = {
            schema_version: 'rvt-session-signoff-v12.0',
            session_id: sessionId,
            operator_name,
            initials,
            validation_comment: String(body.validation_comment || '').trim(),
            signed_at: now,
            updated_at: now,
            sandbox: true
          };
          localStorage.setItem(signoffKey, JSON.stringify(signoff));
          return signoff;
        }
        const existing = this.readJsonStorage<Record<string, unknown>>(signoffKey) || {};
        return {
          schema_version: 'rvt-session-signoff-v12.0',
          session_id: sessionId,
          operator_name: String(existing['operator_name'] || ''),
          initials: String(existing['initials'] || ''),
          validation_comment: String(existing['validation_comment'] || ''),
          signed_at: existing['signed_at'] ?? null,
          updated_at: existing['updated_at'] ?? null,
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
    if (url.pathname === '/api/serial/ports') {
      return {
        ok: true,
        schema_version: 'rvt-control-api-v12.0',
        ports: [{ device: 'COM4', label: 'USB Serial Device (Sandbox)' }],
        selected: 'COM4'
      };
    }
    if (url.pathname === '/api/trainer/log') {
      return { ok: true, lines: ['[sandbox] Local demo control plane active (no hardware trainer attached).'] };
    }
    if (url.pathname === '/api/help/schema') {
      return { ok: true, version: PRODUCT_VERSION, glossary: [], tooltips: {}, troubleshooting: [], sandbox: true };
    }
    if (url.pathname === '/api/update/manifest') {
      return {
        product_version: PRODUCT_VERSION,
        release_tag: `v${PRODUCT_VERSION}`,
        release_version: PRODUCT_VERSION,
        minimum_supported: PRODUCT_VERSION,
        sandbox: true
      };
    }
    if (url.pathname === '/api/server-info') {
      return {
        ok: true,
        origin: window.location.origin,
        scheme: window.location.protocol.replace(':', ''),
        bind_mode: 'local',
        trainer_version: PRODUCT_VERSION,
        dashboard_version: PRODUCT_VERSION,
        firmware_expected: `v${PRODUCT_VERSION}`,
        pair_required: false,
        sandbox: true
      };
    }
    if (url.pathname === '/api/session/buffer') {
      const seconds = Math.max(1, Math.min(3600, Number(url.searchParams.get('seconds') || 60) || 60));
      return {
        ok: true,
        schema_version: 'rvt-live-event-v12.0',
        session_id: this.sessionStore.currentSessionId() || 'sandbox_active',
        buffer_s: seconds,
        payload: null,
        sandbox: true
      };
    }
    if (url.pathname === '/api/report/export') {
      const sid = url.searchParams.get('session') || 'sandbox_demo';
      return `<!doctype html><html><head><meta charset="utf-8"><title>RVT Sandbox Report ${sid}</title></head><body><h1>Sandbox Report (${sid})</h1><p>Demo export — not for clinical or manuscript claims.</p></body></html>`;
    }
    if (url.pathname === '/api/preflight') {
      return { ok: true, checks: this.sandboxPreflightChecks() };
    }
    if (url.pathname.startsWith('/api/preflight/') && method === 'POST') {
      const checkId = decodeURIComponent(url.pathname.split('/').pop() || '');
      const existing = this.sandboxPreflightChecks().find(c => c['id'] === checkId);
      return existing || {
        id: checkId,
        label: checkId,
        status: 'ok',
        detail: `Sandbox preflight check ${checkId} passed.`,
        description: `Sandbox preflight check ${checkId} passed.`,
        remediation: '',
        duration_ms: 1.0
      };
    }
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
      const storedDemo = valid.find(item => item.participant_id === demo.participant_id);
      return [storedDemo || demo, ...valid.filter(item => item.participant_id !== demo.participant_id)].slice(0, 41);
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
    const conditions = [...SANDBOX_CONFIRMATORY_CONDITIONS];
    const trials = [1, 2, 3];
    const attempts = this.readProtocolAttempts();
    const participants: CompletionMatrix['participants'] = {};
    for (const participant of this.readParticipants()) {
      const cells: CompletionMatrix['participants'][string]['cells'] = {};
      const conditions_summary: NonNullable<CompletionMatrix['participants'][string]['conditions_summary']> = {};
      let completed = 0;
      for (const condition of conditions) {
        let validTrials = 0;
        for (const trial of trials) {
          const row = [...attempts].reverse().find(item => item['participant_id'] === participant.participant_id && item['condition_id'] === condition && Number(item['trial_number']) === trial);
          const status = String(row?.['status'] || 'missing');
          if (status === 'completed') completed++;
          const sessionId = row?.['session_id'] ? String(row['session_id']) : null;
          const validWindows = typeof row?.['valid_windows'] === 'number' ? Number(row['valid_windows']) : null;
          const meetsWindowFloor = validWindows !== null ? validWindows >= 15 : null;
          if (status === 'completed' && meetsWindowFloor === true) validTrials++;
          cells[`${condition}:t${trial}`] = {
            status,
            attempt_type: String(row?.['attempt_type'] || 'subject'),
            attempt_id: row?.['attempt_id'] ? String(row['attempt_id']) : null,
            session_id: sessionId,
            valid_windows: validWindows,
            meets_window_floor: meetsWindowFloor
          };
        }
        conditions_summary[condition] = {
          valid_trials: validTrials,
          meets_trial_floor: validTrials >= 2
        };
      }
      participants[participant.participant_id] = {
        participant_id: participant.participant_id,
        status: participant.status,
        completed_trials: completed,
        expected_trials: conditions.length * trials.length,
        protocol_complete: completed === conditions.length * trials.length,
        cells,
        conditions_summary
      };
    }
    const noSubjectRows = attempts.filter(item => item['attempt_type'] === 'no_subject');
    const unqualifiedReasons: Record<string, number> = {};
    for (const row of noSubjectRows) {
      const status = String(row['status'] || '');
      const reason = status !== 'completed' ? 'not_completed' : 'missing_interval';
      unqualifiedReasons[reason] = (unqualifiedReasons[reason] || 0) + 1;
    }
    return {
      schema_version: 'rvt-protocol-attempts-v16.5.9',
      conditions,
      trials,
      participants,
      participant_count: Object.keys(participants).length,
      protocol_complete_participant_count: Object.values(participants).filter(item => item.protocol_complete).length,
      no_subject_attempt_count: noSubjectRows.length,
      no_subject_qualified_count: 0,
      no_subject_unqualified_count: noSubjectRows.length,
      no_subject_qualified_duration_s: 0,
      no_subject_target_duration_s: 10800,
      no_subject_unqualified_reasons: unqualifiedReasons,
      no_subject_expected: 72,
      attempt_count: attempts.length
    };
  }

  private sandboxPreflightChecks(): Array<Record<string, unknown>> {
    return [
      { id: 'firmware_file_present', label: 'Firmware file', status: 'ok', detail: 'Simulated 222-column contract intact.', description: 'Simulated 222-column contract intact.', remediation: '', duration_ms: 1.0 },
      { id: 'serial_port_list', label: 'Serial ports', status: 'ok', detail: 'COM4 present (sandbox).', description: 'COM4 present (sandbox).', remediation: '', duration_ms: 1.0 },
      { id: 'session_folder_writable', label: 'Session folder writable', status: 'ok', detail: 'Local browser sandbox storage writable.', description: 'Local browser sandbox storage writable.', remediation: '', duration_ms: 1.0 },
      { id: 'disk_space', label: 'Disk space', status: 'ok', detail: 'Sandbox storage available.', description: 'Sandbox storage available.', remediation: '', duration_ms: 1.0 },
      { id: 'schema_hash_consistency', label: 'Schema hash consistency', status: 'ok', detail: 'Simulated scoring_weights_hash consistent.', description: 'Simulated scoring_weights_hash consistent.', remediation: '', duration_ms: 1.0 },
      { id: 'ble_adapter', label: 'BLE adapter', status: 'warn', detail: 'Simulated BLE coverage preview — pair hardware for live sessions.', description: 'Simulated BLE coverage preview — pair hardware for live sessions.', remediation: 'Keep the oximeter within range for live sessions.', duration_ms: 1.0 }
    ];
  }

  private readJsonStorage<T>(key: string): T | null {
    try {
      const raw = localStorage.getItem(key);
      if (!raw) return null;
      return JSON.parse(raw) as T;
    } catch {
      return null;
    }
  }

  private readSessionTags(sessionId: string): string[] {
    const stored = this.readJsonStorage<unknown>(`demo:rvt-tags:${sessionId}`);
    return Array.isArray(stored) ? stored.map(item => String(item)).filter(Boolean) : [];
  }

  private sandboxDefaults(): Record<string, unknown> {
    return { sandbox: true, radar_port: 'COM4', ble_address: '10:22:33:9E:8F:63', ble_profile: 'ailink_oximeter' };
  }

  private studyObjectives(): Record<string, unknown> {
    const conditions = [...SANDBOX_CONFIRMATORY_CONDITIONS];
    return {
      schema_version: 'rvt-study-objectives-v16.6.5',
      control_status: 'draft_not_authorized',
      product_version: PRODUCT_VERSION,
      manuscript: {
        path: 'docs/manuscript/manuscript.tex',
        document_id: 'RVT-QMS-MAN-001',
        approval_state: 'approved_manuscript_2026_09',
        testing_section: 'II-F',
        testing_heading: 'Functionality testing'
      },
      thesis_objectives: [
        {
          number: 1,
          phase: 'design',
          statement: 'Design a non-contact vital signs monitoring system that uses 60-GHz millimeter-wave sensing and an ESP32 microcontroller to detect respiratory rate, heart rate, and body temperature.',
          requirement_id: 'RVT-THESIS-001',
          manuscript_sections: ['I', 'II-C', 'II-D']
        },
        {
          number: 2,
          phase: 'build',
          statement: 'Build the individual hardware components of the system, including the millimeter-wave sensing module, ESP32-based processing unit, and user-interface module.',
          requirement_id: 'RVT-THESIS-002',
          manuscript_sections: ['I', 'II-D', 'II-E', 'II-F']
        },
        {
          number: 3,
          phase: 'integrate',
          statement: 'Integrate the built components into a single functional prototype capable of continuous non-contact vital signs monitoring.',
          requirement_id: 'RVT-THESIS-003',
          manuscript_sections: ['I', 'II-F']
        },
        {
          number: 4,
          phase: 'test',
          statement: 'Test the performance of the integrated prototype in detecting respiratory rate, heart rate, and body temperature by comparing its readings against a reference measurement device under controlled test conditions.',
          requirement_id: 'RVT-THESIS-004',
          manuscript_sections: ['I', 'II-F'],
          analysis_endpoint_ids: ['objective_1_rr', 'objective_2_temperature', 'objective_3_false_alarm', 'objective_4_hr']
        }
      ],
      confirmatory_conditions: conditions,
      trials_per_condition: 3,
      planned_duration_s: 150,
      target_recruited_participants: 40,
      minimum_protocol_complete_participants: 38,
      objectives: [
        { id: 'objective_1_rr', number: 1, thesis_objective: 4, manuscript_section: 'II-F', outcome: 'rr', label: 'GBR-assisted respiration-rate equivalence', role: 'confirmatory', primary_condition_id: 'd100_none', secondary_condition_count: 9, equivalence_margin_bpm: 2, confidence_level: 0.9, minimum_independent_estimates: 19 },
        { id: 'objective_2_temperature', number: 2, thesis_objective: 4, manuscript_section: 'II-F', outcome: 'temperature', label: 'Unobstructed skin-surface-temperature agreement', role: 'exploratory', conditions: ['d060_none'] },
        { id: 'objective_3_false_alarm', number: 3, thesis_objective: 4, manuscript_section: 'II-F', outcome: 'false_alarm', label: 'No-subject false-alarm rate', role: 'confirmatory', threshold: 0.05, trial_count: 72, trial_duration_s: 150 },
        { id: 'objective_4_hr', number: 4, thesis_objective: 4, manuscript_section: 'II-F', outcome: 'hr', label: 'GBR-assisted heart-rate accuracy and agreement', role: 'exploratory', conditions }
      ]
    };
  }

  private defaultSandboxProtocol(): StudyProtocol {
    return {
      schema_version: 'rvt-study-protocol-v2',
      protocol_id: 'RVT-THESIS-16.5.9',
      protocol_version: '2',
      state: 'draft',
      randomization_seed: 'sandbox-seed-v16.5.9',
      conditions: [...SANDBOX_CONFIRMATORY_CONDITIONS].map(condition_id => {
        const distance_m = Number(condition_id.slice(1, 4)) / 100;
        return { condition_id, distance_m, barrier_type: condition_id.endsWith('cardboard') ? 'cardboard' : 'none', trial_count: 3, planned_duration_s: 150, confirmatory: true };
      }),
      no_subject: { trial_count: 72, planned_duration_s: 150, frozen_configuration: null }
    };
  }

  private hasCanonicalSandboxConditions(conditionIds: string[]): boolean {
    return (
      conditionIds.length === SANDBOX_CONFIRMATORY_CONDITIONS.length
      && new Set(conditionIds).size === SANDBOX_CONFIRMATORY_CONDITIONS.length
      && SANDBOX_CONFIRMATORY_CONDITIONS.every(id => conditionIds.includes(id))
    );
  }

  private clearStoredSchedulesByPrefix(prefix: string): void {
    const keysToRemove: string[] = [];
    for (let i = 0; i < localStorage.length; i++) {
      const key = localStorage.key(i);
      if (key?.startsWith(prefix)) keysToRemove.push(key);
    }
    for (const key of keysToRemove) {
      localStorage.removeItem(key);
    }
  }

  private studyProtocol(): StudyProtocol {
    const fallback = this.defaultSandboxProtocol();
    try {
      const rawCurrent = localStorage.getItem(SANDBOX_STUDY_PROTOCOL_KEY);
      if (rawCurrent !== null) {
        const stored = JSON.parse(rawCurrent);
        const ids = Array.isArray(stored?.conditions)
          ? stored.conditions.map((item: { condition_id?: unknown }) => String(item?.condition_id || ''))
          : [];
        if (stored && typeof stored === 'object' && this.hasCanonicalSandboxConditions(ids)) {
          return { ...fallback, ...stored } as StudyProtocol;
        }
        localStorage.removeItem(SANDBOX_STUDY_PROTOCOL_KEY);
        this.clearStoredSchedulesByPrefix(SANDBOX_STUDY_SCHEDULE_PREFIX);
      }
    } catch {
      localStorage.removeItem(SANDBOX_STUDY_PROTOCOL_KEY);
      this.clearStoredSchedulesByPrefix(SANDBOX_STUDY_SCHEDULE_PREFIX);
    }

    try {
      const rawLegacy = localStorage.getItem(LEGACY_SANDBOX_STUDY_PROTOCOL_KEY);
      if (rawLegacy !== null) {
        const legacy = JSON.parse(rawLegacy);
        if (legacy && typeof legacy === 'object') {
          const legacyIds = Array.isArray(legacy.conditions)
            ? legacy.conditions.map((item: { condition_id?: unknown }) => String(item?.condition_id || ''))
            : [];
          if (this.hasCanonicalSandboxConditions(legacyIds)) {
            const migrated = { ...fallback, ...legacy } as StudyProtocol;
            localStorage.setItem(SANDBOX_STUDY_PROTOCOL_KEY, JSON.stringify(migrated));
            localStorage.removeItem(LEGACY_SANDBOX_STUDY_PROTOCOL_KEY);
            this.clearStoredSchedulesByPrefix(LEGACY_SANDBOX_STUDY_SCHEDULE_PREFIX);
            return migrated;
          }
          // Legacy 6-condition demo protocols (even if locked) cannot satisfy the
          // 10-condition matrix or allow PUT repairs, so reset to a 10-condition draft.
          const migrated: StudyProtocol = {
            ...fallback,
            ...(typeof legacy.protocol_id === 'string' && legacy.protocol_id ? { protocol_id: legacy.protocol_id } : {}),
            ...(typeof legacy.protocol_version === 'string' && legacy.protocol_version ? { protocol_version: legacy.protocol_version } : {}),
            ...(typeof legacy.randomization_seed === 'string' ? { randomization_seed: legacy.randomization_seed } : {}),
            ...(legacy.no_subject && typeof legacy.no_subject === 'object' ? { no_subject: legacy.no_subject } : {}),
            conditions: fallback.conditions,
            state: 'draft',
            locked_at: null,
            locked_by: null,
            updated_at: new Date().toISOString()
          };
          localStorage.setItem(SANDBOX_STUDY_PROTOCOL_KEY, JSON.stringify(migrated));
          localStorage.removeItem(LEGACY_SANDBOX_STUDY_PROTOCOL_KEY);
          this.clearStoredSchedulesByPrefix(LEGACY_SANDBOX_STUDY_SCHEDULE_PREFIX);
          this.clearStoredSchedulesByPrefix(SANDBOX_STUDY_SCHEDULE_PREFIX);
          return migrated;
        }
        localStorage.removeItem(LEGACY_SANDBOX_STUDY_PROTOCOL_KEY);
      }
    } catch {
      localStorage.removeItem(LEGACY_SANDBOX_STUDY_PROTOCOL_KEY);
    }

    this.clearStoredSchedulesByPrefix(LEGACY_SANDBOX_STUDY_SCHEDULE_PREFIX);
    return fallback;
  }

  private studySchedule(participantId: string): Record<string, unknown> {
    const participant = participantId.trim().toUpperCase();
    if (!participant) throw new Error('participant_id is required');
    const protocol = this.studyProtocol();
    const seed = `${protocol.randomization_seed || ''}:${participant}`;
    const key = `${SANDBOX_STUDY_SCHEDULE_PREFIX}${participant}`;
    try {
      const stored = JSON.parse(localStorage.getItem(key) || 'null');
      const ids = Array.isArray(stored?.entries)
        ? stored.entries.map((item: { condition_id?: unknown }) => String(item?.condition_id || ''))
        : [];
      if (stored && typeof stored === 'object' && stored.seed === seed && this.hasCanonicalSandboxConditions(ids)) {
        return stored as Record<string, unknown>;
      }
    } catch {
      // Recreate a deterministic schedule when local storage contains invalid data.
    }
    const conditions = this.stableShuffle(protocol.conditions.map(item => item.condition_id), seed);
    const result = {
      ok: true,
      schema_version: 'rvt-study-schedule-v2',
      participant_id: participant,
      seed,
      entries: conditions.map((condition_id, index) => ({ participant_id: participant, order: index + 1, condition_id, trial_numbers: [1, 2, 3], status: 'missing', seed }))
    };
    localStorage.setItem(key, JSON.stringify(result));
    localStorage.removeItem(`${LEGACY_SANDBOX_STUDY_SCHEDULE_PREFIX}${participant}`);
    return result;
  }

  private sessionReferences(sessionId: string): StudyReferencesResponse {
    try {
      const stored = JSON.parse(localStorage.getItem(`demo:rvt-references:${sessionId}`) || '[]');
      const adjudication = JSON.parse(localStorage.getItem(`demo:rvt-references:${sessionId}:adjudication`) || 'null');
      return { ok: true, schema_version: 'rvt-reference-observations-v1', session_id: sessionId, references: Array.isArray(stored) ? stored : [], rr_adjudication: adjudication };
    } catch {
      return { ok: true, schema_version: 'rvt-reference-observations-v1', session_id: sessionId, references: [], rr_adjudication: null };
    }
  }

  private readSandboxStudyAnalysis(jobId: string): Record<string, unknown> {
    try {
      const stored = JSON.parse(localStorage.getItem(SANDBOX_STUDY_ANALYSIS_KEY) || 'null');
      if (stored && typeof stored === 'object' && (!jobId || stored.job_id === jobId)) return stored;
      throw new Error('study analysis job not found');
    } catch {
      throw new Error('study analysis job not found');
    }
  }

  private sandboxObjectiveExport(objectiveId: string, format: 'latex' | 'csv'): StudyObjectiveExportResponse {
    const report = this.sandboxObjectiveReport(objectiveId);
    const filename = `${objectiveId}_summary.${format === 'latex' ? 'tex' : 'csv'}`;
    const content_type = format === 'latex' ? 'application/x-tex; charset=utf-8' : 'text/csv; charset=utf-8';
    if (format === 'latex') {
      const rows = SANDBOX_CONFIRMATORY_CONDITIONS.map(
        cid => `${cid.replace(/_/g, '\\_')} & -- & -- & -- & -- & -- & -- & -- & inconclusive \\\\`
      );
      const content = [
        '% SANDBOX DEMO DATA — NOT FOR MANUSCRIPT SUBMISSION',
        `% Objective: ${objectiveId} (status=${report.status})`,
        '\\begin{table}[!t]',
        '\\centering',
        `\\caption{Sandbox summary for ${objectiveId.replace(/_/g, '\\_')} (${report.status})}`,
        `\\label{tab:${objectiveId.replace(/_/g, '\\_')}}`,
        '\\begin{tabular}{lcccccccl}',
        '\\toprule',
        'Condition & $n$ & Valid (\\%) & Bias & RMSE & MAE & BA LoA & 90\\% TOST CI & Holm Decision \\\\',
        '\\midrule',
        ...rows,
        '\\bottomrule',
        '\\end{tabular}',
        '\\end{table}',
        ''
      ].join('\n');
      return { ok: true, objective_id: objectiveId, format, filename, content_type, content, status: report.status };
    }
    const header = 'objective_id,status,condition_id,distance_m,barrier_type,role,n_participants,valid_output_rate_pct,bias,rmse,mae,ba_loa_low,ba_loa_high,tost_ci_low,tost_ci_high,p_value,p_value_holm,holm_decision';
    const csvRows = SANDBOX_CONFIRMATORY_CONDITIONS.map(cid => {
      const distanceM = Number(cid.slice(1, 4)) / 100;
      const barrier = cid.endsWith('cardboard') ? 'cardboard' : 'none';
      const role = cid === 'd100_none' ? 'primary' : 'secondary';
      return `${objectiveId},${report.status},${cid},${distanceM},${barrier},${role},--,--,--,--,--,--,--,--,--,--,--,inconclusive`;
    });
    return {
      ok: true,
      objective_id: objectiveId,
      format,
      filename,
      content_type,
      content: [header, ...csvRows].join('\n') + '\n',
      status: report.status
    };
  }

  private sandboxObjectiveReport(objectiveId: string): StudyObjectiveReport {
    const objective = this.studyObjectives()['objectives'] as Array<Record<string, unknown>>;
    const known = objective.some(item => item['id'] === objectiveId);
    if (!known) throw new Error('study objective not found');
    const status = objectiveId === 'objective_3_false_alarm' ? 'inconclusive' : 'descriptive';
    return {
      ok: true,
      objective_id: objectiveId,
      schema_version: 'rvt-study-report-v2',
      status: known ? status : 'blocked',
      report: known ? { sandbox: true, note: 'Demo evidence is excluded from confirmatory claims.', objective: objective.find(item => item['id'] === objectiveId) || null } : null,
      exclusions: [{ reason: objectiveId === 'objective_3_false_alarm' ? 'insufficient_qualified_no_subject_evidence' : 'sandbox_data', count: 1 }],
      provenance: { sandbox: true, product_version: PRODUCT_VERSION }
    };
  }

  private stableShuffle(values: string[], seed: string): string[] {
    let state = 2166136261;
    for (const char of seed) state = Math.imul(state ^ char.charCodeAt(0), 16777619) >>> 0;
    const result = [...values];
    for (let index = result.length - 1; index > 0; index--) {
      state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
      const swap = state % (index + 1);
      [result[index], result[swap]] = [result[swap], result[index]];
    }
    return result;
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
          locked_until: Number(profile.locked_until || 0),
          recovery_code: typeof profile.recovery_code === 'string' ? profile.recovery_code : undefined,
          recovery_failed_attempts: Number(profile.recovery_failed_attempts || 0),
          recovery_locked_until: Number(profile.recovery_locked_until || 0)
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

  private mintRecoveryCode(): string {
    const group = () => {
      let out = '';
      for (let i = 0; i < 4; i++) {
        out += SANDBOX_RECOVERY_ALPHABET[Math.floor(Math.random() * SANDBOX_RECOVERY_ALPHABET.length)];
      }
      return out;
    };
    return `${group()}-${group()}-${group()}`;
  }

  private invalidateOperatorSessions(operatorId: string): void {
    const sessions = this.readOperatorSessions();
    let changed = false;
    for (const [tok, sess] of Object.entries(sessions)) {
      if (sess?.operator_id === operatorId) {
        delete sessions[tok];
        changed = true;
      }
    }
    if (changed) this.writeOperatorSessions(sessions);
  }

  private createOperatorProfile(init?: RequestInit): { ok: boolean; operator?: OperatorProfile; recovery_code?: string; error?: { code: string; message: string } } {
    const body = this.parseJsonBody<{ display_name?: string; initials?: string; pin?: string }>(init?.body);
    const displayName = String(body.display_name || '').trim();
    const initials = String(body.initials || '').trim().toUpperCase();
    const pin = String(body.pin || '').trim();
    if (!displayName) {
      return { ok: false, error: { code: 'VALIDATION_FAILED', message: 'display_name is required' } };
    }
    if (displayName.length > 64) {
      return { ok: false, error: { code: 'VALIDATION_FAILED', message: 'display_name must not exceed 64 characters' } };
    }
    if (!/^[A-Z]{2,5}$/.test(initials)) {
      return { ok: false, error: { code: 'VALIDATION_FAILED', message: 'initials must be 2 to 5 uppercase letters' } };
    }
    if (!/^\d{6}$/.test(pin)) {
      return { ok: false, error: { code: 'VALIDATION_FAILED', message: 'pin must be exactly 6 digits' } };
    }
    const profiles = this.readOperatorProfiles();
    const recoveryCode = this.mintRecoveryCode();
    const operator: SandboxOperatorProfile = {
      operator_id: `sandbox_op_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`,
      display_name: displayName,
      initials,
      pin,
      failed_attempts: 0,
      locked_until: 0,
      recovery_code: recoveryCode,
      recovery_failed_attempts: 0,
      recovery_locked_until: 0
    };
    this.writeOperatorProfiles([...profiles, operator]);
    const {
      pin: _pin,
      failed_attempts: _failed,
      locked_until: _locked,
      recovery_code: _rc,
      recovery_failed_attempts: _rfailed,
      recovery_locked_until: _rlocked,
      ...publicOperator
    } = operator;
    return { ok: true, operator: publicOperator, recovery_code: recoveryCode };
  }

  private resetPinWithRecovery(init?: RequestInit): { ok: boolean; recovery_code?: string; error?: { code: string; message: string; retry_after_s?: number } } {
    const body = this.parseJsonBody<{ operator_id?: string; recovery_code?: string; new_pin?: string }>(init?.body);
    const operatorId = String(body.operator_id || '').trim();
    const recoveryCode = String(body.recovery_code || '').trim().toUpperCase();
    const newPin = String(body.new_pin || '').trim();
    if (!/^\d{6}$/.test(newPin)) {
      return { ok: false, error: { code: 'VALIDATION_FAILED', message: 'new_pin must be exactly 6 digits' } };
    }
    const profiles = this.readOperatorProfiles();
    const profile = profiles.find(item => item.operator_id === operatorId);
    if (!profile) {
      return { ok: false, error: { code: 'UNAUTHORIZED', message: 'Invalid operator ID' } };
    }
    const now = Date.now();
    const lockedUntil = Number(profile.recovery_locked_until || 0);
    if (lockedUntil > now) {
      const retryAfter = Math.max(1, Math.ceil((lockedUntil - now) / 1000));
      return {
        ok: false,
        error: {
          code: 'LOCKOUT_ACTIVE',
          message: `Too many failed recovery attempts. Try again in ${retryAfter} seconds.`,
          retry_after_s: retryAfter
        }
      };
    }
    if (!profile.recovery_code) {
      return {
        ok: false,
        error: {
          code: 'NO_RECOVERY_CODE',
          message: 'This operator profile has no recovery code. Use host-reset.'
        }
      };
    }
    if (profile.recovery_code !== recoveryCode) {
      profile.recovery_failed_attempts = Number(profile.recovery_failed_attempts || 0) + 1;
      if (profile.recovery_failed_attempts >= 5) {
        profile.recovery_locked_until = now + 30_000;
      }
      this.writeOperatorProfiles(profiles);
      if (profile.recovery_failed_attempts >= 5) {
        return {
          ok: false,
          error: {
            code: 'LOCKOUT_ACTIVE',
            message: 'Too many failed recovery attempts. Try again in 30 seconds.',
            retry_after_s: 30
          }
        };
      }
      return { ok: false, error: { code: 'UNAUTHORIZED', message: 'Invalid recovery code' } };
    }
    const newRecoveryCode = this.mintRecoveryCode();
    profile.pin = newPin;
    profile.failed_attempts = 0;
    profile.locked_until = 0;
    profile.recovery_code = newRecoveryCode;
    profile.recovery_failed_attempts = 0;
    profile.recovery_locked_until = 0;
    this.writeOperatorProfiles(profiles);
    this.invalidateOperatorSessions(operatorId);
    return { ok: true, recovery_code: newRecoveryCode };
  }

  private hostResetPin(init?: RequestInit): { ok: boolean; recovery_code?: string; error?: { code: string; message: string } } {
    const body = this.parseJsonBody<{ operator_id?: string; new_pin?: string }>(init?.body);
    const operatorId = String(body.operator_id || '').trim();
    const newPin = String(body.new_pin || '').trim();
    if (!/^\d{6}$/.test(newPin)) {
      return { ok: false, error: { code: 'VALIDATION_FAILED', message: 'new_pin must be exactly 6 digits' } };
    }
    const profiles = this.readOperatorProfiles();
    const profile = profiles.find(item => item.operator_id === operatorId);
    if (!profile) {
      return { ok: false, error: { code: 'UNAUTHORIZED', message: 'Invalid operator ID' } };
    }
    const newRecoveryCode = this.mintRecoveryCode();
    profile.pin = newPin;
    profile.failed_attempts = 0;
    profile.locked_until = 0;
    profile.recovery_code = newRecoveryCode;
    profile.recovery_failed_attempts = 0;
    profile.recovery_locked_until = 0;
    this.writeOperatorProfiles(profiles);
    this.invalidateOperatorSessions(operatorId);
    return { ok: true, recovery_code: newRecoveryCode };
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
