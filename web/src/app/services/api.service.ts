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
import { SourceModeService } from './source-mode.service';
import { SandboxApiService } from './sandbox-api.service';
import { API_BASE_KEY, SERVER_URL_KEY, TOKEN_KEY, OPERATOR_TOKEN_KEY } from './rvt-storage-keys';
import {
  isTrustedTrainerApiTarget,
  normalizeHttpOrigin,
  resolveTrainerRequestTarget
} from './api-target-policy';
import { clientReleaseHandshake } from './release-contract';

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
  private sourceMode = inject(SourceModeService);
  private sandboxApi = inject(SandboxApiService);
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
      return normalizeHttpOrigin(storedBase || '');
    } catch (_) {
      return '';
    }
  }

  setApiBase(value: string): string {
    const raw = String(value || '').trim().replace(/\/+$/, '');
    const normalized = normalizeHttpOrigin(raw);

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
    if (this.sourceMode.shouldUseSandboxApi(path, bypassSandbox)) {
      return this.sandboxApi.request(path, init) as T;
    }

    const base = this.currentApiBase();
    const target = resolveTrainerRequestTarget(path, base);
    const trustedTarget = isTrustedTrainerApiTarget(path, base);
    const method = String(init?.method || 'GET').toUpperCase();

    const cap = (window as Window & { Capacitor?: CapacitorBridge }).Capacitor;
    const nativeHttp = cap?.Plugins?.CapacitorHttp || cap?.Plugins?.Http;
    if (cap?.isNativePlatform?.() && nativeHttp?.request) {
      try {
        const headers = new Headers(init?.headers || {});
        const tok = this.authToken();
        if (trustedTarget && tok) {
          headers.set('X-RVT-Auth', tok);
        } else if (!trustedTarget) {
          headers.delete('X-RVT-Auth');
          headers.delete('Authorization');
        }
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
      if (trustedTarget && tok && !httpHeaders.has('X-RVT-Auth')) {
        httpHeaders = httpHeaders.set('X-RVT-Auth', tok);
      } else if (!trustedTarget) {
        httpHeaders = httpHeaders.delete('X-RVT-Auth').delete('Authorization');
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
    const target = resolveTrainerRequestTarget(path, base);
    const trustedTarget = isTrustedTrainerApiTarget(path, base);
    const headers = new Headers();
    const token = this.authToken();
    if (trustedTarget && token) {
      headers.set('X-RVT-Auth', token);
      headers.set('Authorization', `Bearer ${token}`);
    }
    const tauri = (window as Window & { __TAURI__?: TauriBridge }).__TAURI__?.core;
    if (tauri?.invoke && base && trustedTarget) {
      const parsedTarget = new URL(target);
      const response = await this.withTimeout(
        tauri.invoke<{ status: number; body_base64: string; content_type: string }>('native_download', {
          request: {
            origin: parsedTarget.origin,
            path: parsedTarget.pathname + parsedTarget.search,
            method: 'GET',
            headers: Object.fromEntries(headers.entries()),
            body: null
          }
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

      this.sourceMode.applyTrainerStatus(r);
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
        this.sourceMode.markAuthenticationRequired();
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

  enableSandboxControlMode(reason: string): boolean {
    this.connectionAttempt++;
    if (!this.sourceMode.enterAutomaticDemo(reason)) {
      this.connectionLoading.set(false);
      return false;
    }
    this.sandboxApi.ensureSessions();
    this.connectionLoading.set(false);
    return true;
  }

  private errorMessage(body: unknown, fallback: string): string {
    if (typeof body === 'object' && body !== null) {
      const payload = body as { error?: string | { message?: string } };
      if (typeof payload.error === 'string') return payload.error;
      if (payload.error?.message) return payload.error.message;
    }
    return fallback;
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
      participant_id: setup.participant_id,
      trial_id: `${setup.participant_id}-${setup.condition_id}-t${setup.trial_number}`,
      study_mode: setup.study_mode,
      study_classification: setup.study_mode,
      condition_id: setup.condition_id,
      distance_m: setup.distance_m,
      barrier_type: setup.barrier_type,
      trial_number: setup.trial_number,
      planned_duration_s: setup.duration_s,
      ble_profile: setup.ble_profile,
      skip_countdown: setup.skip_countdown,
      client_handshake: clientReleaseHandshake(),
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
