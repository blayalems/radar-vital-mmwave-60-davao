import { Injectable, inject, signal } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { firstValueFrom } from 'rxjs';
import {
  BleScanDevice,
  ControlStatus,
  LoginResponse,
  OperatorProfile,
  OperatorProfilesResponse,
  PreflightCheck,
  SessionNotesPayload,
  SessionRecord,
  SessionSignoff,
  SubjectProfileRecord
} from '../models/rvt.models';
import { PRODUCT_VERSION } from './app-meta';
import { StateService } from './state.service';
import { API_BASE_KEY, SERVER_URL_KEY, TOKEN_KEY, OPERATOR_TOKEN_KEY } from './rvt-storage-keys';

interface NativeHttpPlugin {
  request(options: {
    url: string;
    method: string;
    headers: Record<string, string>;
    data?: BodyInit | null;
  }): Promise<{ status?: number; data?: unknown }>;
}

interface CapacitorBridge {
  isNativePlatform?(): boolean;
  Plugins?: { CapacitorHttp?: NativeHttpPlugin; Http?: NativeHttpPlugin };
}

interface TauriBridge {
  core?: {
    invoke<T>(command: string, args?: Record<string, unknown>): Promise<T>;
  };
}

interface SandboxOperatorProfile extends OperatorProfile {
  pin: string;
  failed_attempts?: number;
  locked_until?: number;
}

const SANDBOX_OPERATOR_PROFILES_KEY = 'demo:rvt-operator-profiles';
const SANDBOX_OPERATOR_SESSIONS_KEY = 'demo:rvt-operator-sessions';
const SANDBOX_OPERATOR_SESSION_TTL_MS = 8 * 60 * 60 * 1000;

@Injectable({
  providedIn: 'root'
})
export class ApiService {
  private state = inject(StateService);
  private http = inject(HttpClient);

  private readonly API_BASE_KEY = API_BASE_KEY;
  private readonly TOKEN_KEY = TOKEN_KEY;
  private connectionAttempt = 0;
  private readonly initialization: Promise<void>;

  public readonly connectionLoading = signal(true);

  constructor() {
    this.initialization = this.initializeConnection();
  }

  whenInitialized(): Promise<void> {
    return this.initialization;
  }

  private async initializeConnection(): Promise<void> {
    this.connectionLoading.set(true);
    try {
      await this.consumePairPinFromUrl();
      await this.detectControlMode();
    } finally {
      this.connectionLoading.set(false);
    }
  }

  private withTimeout<T>(promise: Promise<T>, timeoutMs: number, message: string): Promise<T> {
    let timeoutId: ReturnType<typeof setTimeout> | undefined;
    const timeoutPromise = new Promise<never>((_, reject) => {
      timeoutId = setTimeout(() => reject(new Error(message)), timeoutMs);
    });
    return Promise.race([promise, timeoutPromise]).finally(() => {
      if (timeoutId !== undefined) clearTimeout(timeoutId);
    });
  }

  currentApiBase(): string {
    try {
      const storedBase = localStorage.getItem(this.API_BASE_KEY) || localStorage.getItem(SERVER_URL_KEY);
      const raw = String(storedBase || '').trim().replace(/\/+$/, '');
      if (!raw) return '';
      const u = new URL(raw, window.location.href);
      if (!/^https?:$/.test(u.protocol)) return '';
      return u.origin;
    } catch (_) {
      return '';
    }
  }

  setApiBase(value: string): string {
    const raw = String(value || '').trim().replace(/\/+$/, '');
    let normalized = '';
    try {
      if (raw) {
        const u = new URL(raw, window.location.href);
        if (/^https?:$/.test(u.protocol)) {
          normalized = u.origin;
        }
      }
    } catch (_) {}

    try {
      if (normalized) {
        localStorage.setItem(this.API_BASE_KEY, normalized);
        localStorage.setItem(SERVER_URL_KEY, normalized);
      } else {
        localStorage.removeItem(this.API_BASE_KEY);
        localStorage.removeItem(SERVER_URL_KEY);
      }
    } catch (_) {}
    return normalized;
  }

  pairToken(): string {
    try {
      return sessionStorage.getItem(this.TOKEN_KEY) || '';
    } catch (_) {
      return '';
    }
  }

  operatorToken(): string {
    try {
      return sessionStorage.getItem(OPERATOR_TOKEN_KEY) || '';
    } catch (_) {
      return '';
    }
  }

  authToken(): string {
    return this.operatorToken() || this.pairToken();
  }

  setPairToken(value: string): void {
    const token = value.trim();
    try {
      if (token) sessionStorage.setItem(this.TOKEN_KEY, token);
      else sessionStorage.removeItem(this.TOKEN_KEY);
    } catch (_) {}
  }

  hasPairToken(): boolean {
    return this.pairToken().length > 0;
  }

  async exchangePairPin(pin: string): Promise<void> {
    const cleanPin = String(pin || '').trim();
    if (!/^\d{6}$/.test(cleanPin)) {
      throw new Error('Enter the six-digit pairing PIN.');
    }
    const payload = await this.withTimeout(
      this.request<{ token?: string }>('/api/auth/exchange', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ pin: cleanPin })
      }, true),
      4000,
      'Pairing exchange timeout'
    );
    if (!payload.token) throw new Error('Trainer did not return a pairing token.');
    this.setPairToken(payload.token);
    await this.setTauriPairedOrigin();
  }

  private async consumePairPinFromUrl(): Promise<void> {
    const url = new URL(window.location.href);
    const pin = url.searchParams.get('pair');
    if (!pin) return;
    try {
      await this.exchangePairPin(pin);
    } catch (error: unknown) {
      this.state.pushAlert(error instanceof Error ? error.message : 'Pairing failed.', 'critical', 'pairing');
    } finally {
      url.searchParams.delete('pair');
      history.replaceState(history.state, '', `${url.pathname}${url.search}${url.hash}`);
    }
  }

  async request<T = unknown>(path: string, init?: RequestInit, bypassSandbox = false, timeoutMs = 10000): Promise<T> {
    const isSandbox = !bypassSandbox && (
      this.state.autoDemoActive()
      || this.state.demoMode()
      || (this.state.ctlOn() && this.state.ctlStatus()?.mode === 'sandbox')
    );

    if (isSandbox && path.startsWith('/api/')) {
      return this.sandboxApiJson(path, init) as T;
    }

    const base = this.currentApiBase();
    const target = /^[a-z][a-z0-9+.-]*:\/\//i.test(String(path)) ? String(path) : base + String(path);
    const method = String(init?.method || 'GET').toUpperCase();

    const cap = (window as Window & { Capacitor?: CapacitorBridge }).Capacitor;
    const nativeHttp = cap?.Plugins?.CapacitorHttp || cap?.Plugins?.Http;
    if (cap?.isNativePlatform?.() && nativeHttp?.request) {
      try {
        const headers = new Headers(init?.headers || {});
        const tok = this.authToken();
        if (tok) headers.set('X-RVT-Auth', tok);
        const headerObj: Record<string, string> = {};
        headers.forEach((v, k) => { headerObj[k] = v; });

        const resp = await this.withTimeout(nativeHttp.request({
          url: target,
          method,
          headers: headerObj,
          data: init?.body
        }), timeoutMs, 'Request timeout');

        const status = Number(resp.status || 0);
        const data = resp.data;
        if (status < 200 || status >= 300) {
          throw new Error(this.errorMessage(data, `HTTP ${status}`));
        }
        return (typeof data === 'string' ? JSON.parse(data || '{}') : data) as T;
      } catch (err) {
        console.warn('Native request failed', err);
        throw err;
      }
    }

    try {
      let httpHeaders = new HttpHeaders(init?.headers as any || {});
      const tok = this.authToken();
      if (tok && !httpHeaders.has('X-RVT-Auth')) {
        httpHeaders = httpHeaders.set('X-RVT-Auth', tok);
      }
      const body = init?.body;
      const response = this.http.request<T>(method, target, {
        body,
        headers: httpHeaders,
        responseType: 'json',
        observe: 'body'
      });
      return await this.withTimeout(firstValueFrom(response), timeoutMs, 'Request timeout');
    } catch (err: any) {
      if (err && typeof err === 'object' && 'status' in err) {
        throw new Error(this.errorMessage(err.error, `HTTP ${err.status}`));
      }
      throw new Error(err.message || 'HTTP Request failed');
    }
  }

  async download(path: string, filename: string): Promise<void> {
    const base = this.currentApiBase();
    const target = /^[a-z][a-z0-9+.-]*:\/\//i.test(path) ? path : base + path;
    const headers = new Headers();
    const token = this.authToken();
    if (token) {
      headers.set('X-RVT-Auth', token);
      headers.set('Authorization', `Bearer ${token}`);
    }
    const tauri = (window as Window & { __TAURI__?: TauriBridge }).__TAURI__?.core;
    if (tauri?.invoke && base) {
      const response = await this.withTimeout(
        tauri.invoke<{ status: number; body_base64: string; content_type: string }>('native_download', {
          request: { origin: base, path, method: 'GET', headers: Object.fromEntries(headers.entries()), body: null }
        }),
        10000,
        'Download timeout'
      );
      if (response.status < 200 || response.status >= 300) throw new Error(`Download failed: HTTP ${response.status}`);
      const binary = atob(response.body_base64);
      const bytes = Uint8Array.from(binary, char => char.charCodeAt(0));
      this.downloadBlob(new Blob([bytes], { type: response.content_type || 'application/octet-stream' }), filename);
      return;
    }
    const response = await this.withTimeout(fetch(target, { headers, cache: 'no-store' }), 10000, 'Download timeout');
    if (!response.ok) throw new Error(`Download failed: HTTP ${response.status}`);
    this.downloadBlob(await response.blob(), filename);
  }

  private downloadBlob(blob: Blob, filename: string): void {
    const href = URL.createObjectURL(blob);
    const anchor = document.createElement('a');
    anchor.href = href;
    anchor.download = filename;
    anchor.click();
    URL.revokeObjectURL(href);
  }

  async detectControlMode(): Promise<boolean> {
    const attempt = ++this.connectionAttempt;
    try {
      const r = await this.withTimeout(
        this.request<ControlStatus>('/api/status', undefined, true),
        10000,
        'Connection detection timeout'
      );
      if (attempt !== this.connectionAttempt) return false;

      this.state.ctlOn.set(true);
      this.state.autoDemoActive.set(false);
      this.state.ctlStatus.set({ ...r, mode: r.mode === 'sandbox' ? 'sandbox' : 'live' });
      const activeSession = r.active_session;
      this.state.sessionActive.set(!!activeSession);
      this.state.currentSessionId.set(activeSession?.session_id || null);
      return true;
    } catch (error: unknown) {
      if (attempt !== this.connectionAttempt) return false;
      const message = error instanceof Error ? error.message : 'Control API unavailable';
      if (
        message.includes('401') ||
        message.includes('Unauthorized') ||
        message.includes('unauthenticated') ||
        message.includes('Operator session') ||
        message.includes('session token') ||
        message.includes('LAN pair token') ||
        message.includes('pair token')
      ) {
        this.state.ctlOn.set(true);
        this.state.autoDemoActive.set(false);
        this.state.ctlStatus.set({
          ok: true,
          mode: 'live',
          reason: 'unauthenticated'
        });
        return false;
      }
      this.enableSandboxControlMode(message);
      return false;
    }
  }

  async checkConnection(timeoutMs = 10000): Promise<boolean> {
    try {
      const response = await this.withTimeout(
        this.request<{ ok?: boolean }>('/api/health', undefined, true),
        timeoutMs,
        'Server health check timeout'
      );
      return response?.ok !== false;
    } catch (_) {
      return false;
    }
  }

  enableSandboxControlMode(reason: string) {
    this.connectionAttempt++;
    this.state.ctlOn.set(true);
    this.state.sessionActive.set(false);
    this.state.ctlStopPending.set(false);
    this.state.autoDemoActive.set(true);
    this.state.ctlStatus.set({
      ok: true,
      mode: 'sandbox',
      reason
    });
    this.sandboxLoadSessions();
    this.connectionLoading.set(false);
  }

  private errorMessage(body: unknown, fallback: string): string {
    if (typeof body === 'object' && body !== null) {
      const payload = body as { error?: string | { message?: string } };
      if (typeof payload.error === 'string') return payload.error;
      if (payload.error?.message) return payload.error.message;
    }
    return fallback;
  }

  private sandboxReadSessions(): SessionRecord[] {
    return this.state.sessionItems().filter(item => item.sandbox || item.session_id.startsWith('sandbox_'));
  }

  private sandboxLoadSessions(): SessionRecord[] {
    const existing = this.sandboxReadSessions();
    if (!existing.length) {
      const iso = (mins: number) => new Date(Date.now() - mins * 60000).toISOString();
      const items = [
        { session_id: 'sandbox_20260420_091800', started_at: iso(64), duration_s: 480, subject: 'demo-A', operator: 'Demo operator', verdict: 'demo', summary: 'Completed 8 min simulated telemetry preview; no physiological conclusion.', sandbox: true },
        { session_id: 'sandbox_20260419_141803', started_at: iso(1240), duration_s: 300, subject: 'demo-B', operator: 'Demo operator', verdict: 'demo', summary: 'Calibration sandbox with stable BLE coverage and simulated gate audit.', sandbox: true }
      ] as SessionRecord[];
      this.state.sessionItems.set(items);
      return items;
    }
    return existing;
  }

  private sandboxReadOperatorProfiles(): SandboxOperatorProfile[] {
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

  private sandboxWriteOperatorProfiles(profiles: SandboxOperatorProfile[]): void {
    try {
      localStorage.setItem(SANDBOX_OPERATOR_PROFILES_KEY, JSON.stringify({
        schema_version: 'rvt-sandbox-operator-profiles-v12.0',
        profiles
      }));
    } catch (_) {}
  }

  private sandboxPublicOperatorProfiles(): OperatorProfile[] {
    return this.sandboxReadOperatorProfiles().map(({ operator_id, display_name, initials }) => ({
      operator_id,
      display_name,
      initials
    }));
  }

  private sandboxReadOperatorSessions(): Record<string, { operator_id: string; expires_at: number }> {
    try {
      const payload = JSON.parse(sessionStorage.getItem(SANDBOX_OPERATOR_SESSIONS_KEY) || '{}');
      return payload && typeof payload === 'object' ? payload : {};
    } catch (_) {
      return {};
    }
  }

  private sandboxWriteOperatorSessions(sessions: Record<string, { operator_id: string; expires_at: number }>): void {
    try {
      sessionStorage.setItem(SANDBOX_OPERATOR_SESSIONS_KEY, JSON.stringify(sessions));
    } catch (_) {}
  }

  private sandboxOperatorProfilesResponse(): OperatorProfilesResponse {
    return {
      schema_version: 'rvt-sandbox-operator-profiles-v12.0',
      profiles: this.sandboxPublicOperatorProfiles()
    };
  }

  private sandboxCreateOperatorProfile(init?: RequestInit): { ok: boolean; operator?: OperatorProfile; error?: { code: string; message: string } } {
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
    const profiles = this.sandboxReadOperatorProfiles();
    const operator: SandboxOperatorProfile = {
      operator_id: `sandbox_op_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`,
      display_name: displayName,
      initials,
      pin,
      failed_attempts: 0,
      locked_until: 0
    };
    this.sandboxWriteOperatorProfiles([...profiles, operator]);
    const { pin: _pin, failed_attempts: _failed, locked_until: _locked, ...publicOperator } = operator;
    return { ok: true, operator: publicOperator };
  }

  private sandboxLogin(init?: RequestInit): LoginResponse | { ok: false; error: { code: string; message: string; retry_after_s?: number } } {
    const body = this.parseJsonBody<{ operator_id?: string; pin?: string }>(init?.body);
    const operatorId = String(body.operator_id || '').trim();
    const pin = String(body.pin || '').trim();
    const profiles = this.sandboxReadOperatorProfiles();
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
      this.sandboxWriteOperatorProfiles(profiles);
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
    this.sandboxWriteOperatorProfiles(profiles);
    const expiresAt = now + SANDBOX_OPERATOR_SESSION_TTL_MS;
    const token = `sandbox_op_token_${operatorId}_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 10)}`;
    const sessions = this.sandboxReadOperatorSessions();
    sessions[token] = { operator_id: operatorId, expires_at: expiresAt };
    this.sandboxWriteOperatorSessions(sessions);
    const operator = { operator_id: profile.operator_id, display_name: profile.display_name, initials: profile.initials };
    return { token, expires_at: Math.floor(expiresAt / 1000), operator };
  }

  private sandboxValidateOperator(): { ok: boolean; bootstrap?: boolean; operator: OperatorProfile | null } {
    const token = this.operatorToken();
    const sessions = this.sandboxReadOperatorSessions();
    const session = token ? sessions[token] : null;
    const now = Date.now();
    if (session && Number(session.expires_at || 0) > now) {
      const operator = this.sandboxPublicOperatorProfiles().find(item => item.operator_id === session.operator_id);
      if (operator) {
        return { ok: true, operator };
      }
    }
    if (token && sessions[token]) {
      delete sessions[token];
      this.sandboxWriteOperatorSessions(sessions);
    }
    return { ok: true, bootstrap: this.sandboxPublicOperatorProfiles().length === 0, operator: null };
  }

  private tokenFromRequestInit(init?: RequestInit): string {
    try {
      return new Headers(init?.headers || {}).get('X-RVT-Auth') || this.operatorToken();
    } catch (_) {
      return this.operatorToken();
    }
  }

  private sandboxLogoutOperator(init?: RequestInit): { ok: true } {
    const token = this.tokenFromRequestInit(init);
    if (token) {
      const sessions = this.sandboxReadOperatorSessions();
      delete sessions[token];
      this.sandboxWriteOperatorSessions(sessions);
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

  private sandboxApiJson(path: string, init?: RequestInit): unknown {
    const url = new URL(path, window.location.origin);
    const method = String(init?.method || 'GET').toUpperCase();
    if (url.pathname === '/api/status') return { ok: true, mode: 'sandbox', active_session: this.state.sessionActive() ? { session_id: this.state.currentSessionId() || 'sandbox_active', sandbox: true } : null };
    if (url.pathname === '/api/health') return { ok: true, version: 'sandbox' };
    if (url.pathname === '/api/version') return { product_version: PRODUCT_VERSION, trainer: 'sandbox', dashboard: 'sandbox', firmware_expected: 'sandbox' };
    if (url.pathname === '/api/auth/validate') return this.sandboxValidateOperator();
    if (url.pathname === '/api/auth/login' && method === 'POST') {
      const login = this.sandboxLogin(init);
      if ('error' in login) throw new Error(`${login.error.message} (${login.error.code})`);
      return login;
    }
    if (url.pathname === '/api/auth/logout' && method === 'POST') return this.sandboxLogoutOperator(init);
    if (url.pathname === '/api/auth/sse-token') return { sse_token: `sandbox_sse_${Date.now().toString(36)}` };
    if (url.pathname === '/api/operator-profiles' && method === 'GET') return this.sandboxOperatorProfilesResponse();
    if (url.pathname === '/api/operator-profiles' && method === 'POST') {
      const created = this.sandboxCreateOperatorProfile(init);
      if (created.error) throw new Error(`${created.error.message} (${created.error.code})`);
      return created;
    }
    if (url.pathname === '/api/sessions') {
      const sessions = this.sandboxLoadSessions();
      return { ok: true, sessions, items: sessions };
    }
    if (url.pathname === '/api/session/current') return this.state.sessionActive() ? { session_id: this.state.currentSessionId() || 'sandbox_active', sandbox: true } : { ok: false, error: { code: 'NO_ACTIVE_SESSION' } };
    if (url.pathname === '/api/session/current/live_dashboard.json') return { meta: { sandbox: true, status: this.state.sessionActive() ? 'running' : 'waiting' } };
    if (url.pathname === '/api/session/start' && method === 'POST') {
      const id = `sandbox_${Date.now()}`;
      this.state.sessionActive.set(true);
      this.state.currentSessionId.set(id);
      return { ok: true, session_id: id, sandbox: true };
    }
    if (url.pathname === '/api/session/stop' && method === 'POST') {
      this.state.sessionActive.set(false);
      const id = this.state.currentSessionId() || `sandbox_${Date.now()}`;
      this.state.currentSessionId.set(null);
      const setup = this.state.setup();
      const session: SessionRecord = { session_id: id, started_at: new Date(Date.now() - 120000).toISOString(), duration_s: 120, subject: setup.subject_label || 'sandbox-subject', subject_label: setup.subject_label || 'sandbox-subject', operator: setup.operator_label || 'Demo operator', verdict: 'demo', summary: 'Sandbox session stopped locally; trainer was unavailable.', sandbox: true };
      this.state.sessionItems.update(items => [session, ...items.filter(item => item.session_id !== id)]);
      return { ok: true, session_id: id, session, sandbox: true };
    }
    if (url.pathname.startsWith('/api/sessions/')) {
      const parts = url.pathname.split('/');
      const sessionId = decodeURIComponent(parts[3] || '');
      const session = this.sandboxLoadSessions().find(item => item.session_id === sessionId)
        || { session_id: sessionId, sandbox: true, verdict: 'demo', summary: 'Sandbox session summary.' };
      if (url.pathname.endsWith('/summary')) {
        // Simulated quality scorecard so the Report quality card is demo-visible.
        return {
          ...session,
          sandbox: true,
          signal_quality: { pqi_lock_pct: 84.2, session_quality_score: 8.7, internal_consistency_score: 9.1, coverage_locked: 81.5, coverage_settling: 11.2 },
          hr_metrics: { rmse: 2.41, mae: 1.92, bias: -0.4, coverage_pct: 88.1 },
          rr_metrics: { rmse: 0.82, mae: 0.61, bias: 0.1, coverage_pct: 90.4 },
          gates: { primary: { passed: true, status: 'pass' }, secondary: { passed: false, status: 'deferred' } },
          verdict: {
            verdict: 'ready',
            readiness_kind: 'ready',
            categories: [
              { id: 'firmware', label: 'Firmware contract', status: 'pass', detail: 'Simulated 219-column contract intact; 207-column prefix preserved.', remediation: '' },
              { id: 'reference', label: 'Reference coverage', status: 'warn', detail: 'Simulated BLE coverage 72%.', remediation: 'Keep the oximeter within range for the full session.' }
            ]
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
    }
    if (url.pathname === '/api/defaults') return { sandbox: true, radar_port: 'COM10', ble_address: '', ble_profile: 'ailink_oximeter' };
    if (url.pathname === '/api/preflight') return { ok: true, checks: [{ name: 'Sandbox trainer', ok: true, message: 'Demo mode is available.' }] };
    if (url.pathname === '/api/ble/scan') return { ok: true, devices: [] };
    return { ok: true, sandbox: true };
  }

  async loadSessions(): Promise<SessionRecord[]> {
    const response = await this.request<{ sessions: SessionRecord[] }>('/api/sessions');
    const items = response.sessions || [];
    this.state.sessionItems.set(items);
    return items;
  }

  async startSession(): Promise<{ session_id?: string }> {
    const setup = this.state.setup();
    const payload = {
      duration_s: setup.duration_s,
      radar_port: setup.radar_port,
      ble_address: setup.ble_address,
      subject_label: setup.subject_label,
      operator_label: setup.operator_label,
      station_label: setup.station_label,
      subject_profile_id: setup.subject_profile_id,
      ble_profile: setup.ble_profile,
      skip_countdown: setup.skip_countdown,
      advanced: { notify_char: setup.notify_char }
    };
    const result = await this.request<{ session_id?: string }>('/api/session/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    this.state.sessionActive.set(true);
    this.state.currentSessionId.set(result.session_id || null);
    return result;
  }

  async stopSession(): Promise<SessionRecord> {
    const result = await this.request<SessionRecord>('/api/session/stop', { method: 'POST' });
    this.state.sessionActive.set(false);
    this.state.currentSessionId.set(null);
    await this.loadSessions().catch(() => []);
    return result;
  }

  async saveNotes(sessionId: string, notes: SessionNotesPayload): Promise<void> {
    await this.request(`/api/sessions/${encodeURIComponent(sessionId)}/notes`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(notes)
    });
  }

  async signoffSession(sessionId: string, signoff: SessionSignoff): Promise<void> {
    await this.request(`/api/sessions/${encodeURIComponent(sessionId)}/signoff`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(signoff)
    });
  }

  async scanBle(timeout_s = 4): Promise<BleScanDevice[]> {
    const res = await this.request<{ devices: BleScanDevice[] }>(`/api/ble/scan?timeout_s=${timeout_s}`);
    return res.devices || [];
  }

  async runPreflight(options: Record<string, unknown> = {}): Promise<PreflightCheck[]> {
    const params = new URLSearchParams();
    Object.entries(options).forEach(([key, value]) => {
      if (value === undefined || value === null || value === '') return;
      if (Array.isArray(value)) {
        if (value.length) params.set(key, value.join(','));
      } else {
        params.set(key, String(value));
      }
    });
    const query = params.toString();
    const res = await this.request<{ checks: PreflightCheck[] }>(`/api/preflight${query ? '?' + query : ''}`);
    return res.checks || [];
  }

  async loadSubjectProfiles(): Promise<Record<string, SubjectProfileRecord>> {
    return this.request<Record<string, SubjectProfileRecord>>('/api/subject-profiles');
  }

  async saveSubjectProfiles(profiles: Record<string, SubjectProfileRecord>): Promise<void> {
    await this.request('/api/subject-profiles', {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(profiles)
    });
  }

  async setTauriPairedOrigin(): Promise<void> {
    const base = this.currentApiBase();
    const tauri = (window as Window & { __TAURI__?: TauriBridge }).__TAURI__?.core;
    if (!tauri?.invoke || !base) return;
    try {
      await tauri.invoke('native_set_paired_origin', { origin: base });
    } catch (_) {}
  }
}
