import { Injectable, inject, signal } from '@angular/core';
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { firstValueFrom } from 'rxjs';
import {
  BleScanDevice,
  ControlStatus,
  PreflightCheck,
  SessionNotesPayload,
  SessionRecord,
  SessionSignoff,
  SubjectProfileRecord
} from '../models/rvt.models';
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

@Injectable({
  providedIn: 'root'
})
export class ApiService {
  private state = inject(StateService);
  private http = inject(HttpClient);

  private readonly API_BASE_KEY = API_BASE_KEY;
  private readonly TOKEN_KEY = TOKEN_KEY;
  private connectionAttempt = 0;

  public readonly connectionLoading = signal(true);

  constructor() {
    void this.initializeConnection();
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

    if (isSandbox && path.startsWith('/api/') && !this.isAuthPath(path)) {
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
      const httpHeaders = new HttpHeaders(init?.headers as any || {});
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

  private isAuthPath(path: string): boolean {
    const pathname = new URL(path, window.location.origin).pathname;
    return pathname === '/api/auth/validate'
      || pathname === '/api/auth/login'
      || pathname === '/api/auth/logout'
      || pathname === '/api/auth/sse-token'
      || pathname === '/api/operator-profiles';
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
    if (url.pathname === '/api/version') return { product_version: '16.1.0', trainer: 'sandbox', dashboard: 'sandbox', firmware_expected: 'sandbox' };
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
      if (url.pathname.endsWith('/summary')) return { ...session, sandbox: true };
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
    if (url.pathname === '/api/operator-profiles') return { schema_version: 'sandbox', profiles: [] };
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
      await tauri.invoke('set_paired_origin', { origin: base });
    } catch (_) {}
  }
}
