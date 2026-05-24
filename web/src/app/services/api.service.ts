import { Injectable, inject } from '@angular/core';
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

  private readonly API_BASE_KEY = 'rvt-api-base';
  private readonly TOKEN_KEY = 'rvt-pair-token';

  constructor() {
    void this.initializeConnection();
  }

  private async initializeConnection(): Promise<void> {
    await this.consumePairPinFromUrl();
    await this.detectControlMode();
  }

  // Retrieve current API base address
  currentApiBase(): string {
    try {
      const storedBase = localStorage.getItem(this.API_BASE_KEY);
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
      } else {
        localStorage.removeItem(this.API_BASE_KEY);
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
    const payload = await this.request<{ token?: string }>('/api/auth/exchange', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ pin: cleanPin })
    }, true);
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

  // Low-level HTTP requests with native Tauri/Capacitor plugins fallback
  async request<T = unknown>(path: string, init?: RequestInit, bypassSandbox = false): Promise<T> {
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

    const headers = new Headers(init?.headers || {});
    // Security design decision: We keep the pairing auth token in sessionStorage.
    // This is intentional so that it does not persist across separate browser tabs or survive
    // browser session closure, preventing authentication leaks on shared operator workstations.
    const tok = this.pairToken();
    if (tok) {
      headers.set('X-RVT-Auth', tok);
    }

    const tauri = (window as Window & { __TAURI__?: TauriBridge }).__TAURI__?.core;
    if (tauri?.invoke && base) {
      const method = String(init?.method || 'GET').toUpperCase();
      const headerObj: Record<string, string> = {};
      headers.forEach((value, key) => { headerObj[key] = value; });
      const command = path === '/api/auth/exchange' || path === '/api/server-info'
        ? 'native_pair_request'
        : 'native_http_request';
      const resp = await tauri.invoke<{ status: number; data: unknown }>(command, {
        request: {
          origin: base,
          path: String(path),
          method,
          headers: headerObj,
          body: typeof init?.body === 'string' ? init.body : null
        }
      });
      if (resp.status < 200 || resp.status >= 300) {
        throw new Error(this.errorMessage(resp.data, `HTTP ${resp.status}`));
      }
      return resp.data as T;
    }

    // Try Capacitor native Http wrapper
    const cap = (window as Window & { Capacitor?: CapacitorBridge }).Capacitor;
    const http = cap?.Plugins?.CapacitorHttp || cap?.Plugins?.Http;
    if (cap?.isNativePlatform?.() && http?.request) {
      try {
        const method = String(init?.method || 'GET').toUpperCase();
        const headerObj: Record<string, string> = {};
        headers.forEach((v, k) => { headerObj[k] = v; });

        const resp = await http.request({
          url: target,
          method,
          headers: headerObj,
          data: init?.body
        });

        const status = Number(resp.status || 0);
        const data = resp.data;
        
        if (status < 200 || status >= 300) {
          const err = new Error(this.errorMessage(data, `HTTP ${status}`));
          throw err;
        }
        return (typeof data === 'string' ? JSON.parse(data || '{}') : data) as T;
      } catch (err) {
        console.warn('Native request failed', err);
        throw err;
      }
    }

    // Standard browser fetch
    const opts = {
      cache: 'no-store' as const,
      ...init,
      headers
    };

    const res = await fetch(target, opts);
    const text = await res.text();
    let data: unknown = {};
    try {
      data = JSON.parse(text);
    } catch (_) {
      data = { text };
    }

    if (!res.ok) {
      throw new Error(this.errorMessage(data, `HTTP ${res.status}`));
    }

    return data as T;
  }

  async download(path: string, filename: string): Promise<void> {
    const base = this.currentApiBase();
    const target = /^[a-z][a-z0-9+.-]*:\/\//i.test(path) ? path : base + path;
    const headers = new Headers();
    const token = this.pairToken();
    if (token) headers.set('X-RVT-Auth', token);
    const tauri = (window as Window & { __TAURI__?: TauriBridge }).__TAURI__?.core;
    if (tauri?.invoke && base) {
      const response = await tauri.invoke<{ status: number; body_base64: string; content_type: string }>('native_download', {
        request: { origin: base, path, method: 'GET', headers: Object.fromEntries(headers.entries()), body: null }
      });
      if (response.status < 200 || response.status >= 300) throw new Error(`Download failed: HTTP ${response.status}`);
      const binary = atob(response.body_base64);
      const bytes = Uint8Array.from(binary, char => char.charCodeAt(0));
      this.downloadBlob(new Blob([bytes], { type: response.content_type || 'application/octet-stream' }), filename);
      return;
    }
    const response = await fetch(target, { headers, cache: 'no-store' });
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

  // Detect control mode / connect
  async detectControlMode(): Promise<boolean> {
    try {
      const r = await this.request<ControlStatus>('/api/status', undefined, true);
      this.state.ctlOn.set(true);
      this.state.autoDemoActive.set(false);
      this.state.ctlStatus.set({ ...r, mode: r.mode === 'sandbox' ? 'sandbox' : 'live' });
      const activeSession = r.active_session || r.session;
      this.state.currentSessionId.set(activeSession?.session_id || null);
      return true;
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Control API unavailable';
      this.enableSandboxControlMode(message);
      return false;
    }
  }

  enableSandboxControlMode(reason: string) {
    this.state.ctlOn.set(true);
    this.state.autoDemoActive.set(true);
    this.state.ctlStatus.set({
      ok: true,
      mode: 'sandbox',
      reason
    });
    this.sandboxLoadSessions();
  }

  // --- LOCAL SANDBOX / MOCK ENDPOINTS ---

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
      // Seed default sessions only on very first launch (when key doesn't exist)
      const iso = (mins: number) => new Date(Date.now() - mins * 60000).toISOString();
      const items = [
        {
          session_id: 'sandbox_20260420_091800',
          started_at: iso(64),
          duration_s: 480,
          subject: 'demo-A',
          operator: 'Demo operator',
          verdict: 'demo',
          summary: 'Completed 8 min simulated telemetry preview; no physiological conclusion.',
          sandbox: true
        },
        {
          session_id: 'sandbox_20260419_141803',
          started_at: iso(1240),
          duration_s: 300,
          subject: 'demo-B',
          operator: 'Demo operator',
          verdict: 'demo',
          summary: 'Completed 5 min simulated telemetry preview; no physiological conclusion.',
          sandbox: true
        }
      ];
      this.state.sessionItems.set(items);
      return items;
    }
    const items = existing.map(item => item.session_id.startsWith('sandbox_')
      ? { ...item, verdict: 'demo', summary: 'Simulated telemetry preview; no physiological conclusion.' }
      : item);
    this.sandboxSaveSessions(items);
    this.state.sessionItems.set(items);
    return items;
  }

  private sandboxSaveSessions(items: SessionRecord[]) {
    this.state.sessionItems.set(items);
  }

  private sandboxDefaults() {
    return {
      radar_port: this.state.setup().radar_port || 'COM10',
      ble_address: this.state.setup().ble_address || '10:22:33:9E:8F:63',
      ble_profile: this.state.setup().ble_profile || 'ailink_oximeter',
      notify_char: this.state.setup().notify_char,
      durations_s: [30, 60, 300, 480, 1200],
      sessions_root: 'local sandbox storage',
      sandbox: true
    };
  }

  private sandboxPreflight() {
    const checks: PreflightCheck[] = [
      { id: 'python_env', label: 'Python Runtime environment', status: 'good', description: 'Browser sandbox runtime ready.' },
      { id: 'firmware_file_present', label: 'Firmware contract checks', status: 'good', description: 'Firmware contract fixture loaded for UI review.' },
      { id: 'serial_port_list', label: 'Serial ports discovery', status: 'good', description: "ports=['COM10','COM11','COM12']" },
      { id: 'session_folder_writable', label: 'Storage write access', status: 'good', description: 'Session history writes to localStorage in sandbox mode.' },
      { id: 'disk_space', label: 'Free space validation', status: 'good', description: 'Browser storage is available for sample sessions.' },
      { id: 'schema_hash_consistency', label: 'Vitals Schema contract verification', status: 'good', description: 'Expected v11.0/v15.0 schema hash present.' },
      { id: 'clock_monotonic_sanity', label: 'System clock monotonic integrity', status: 'good', description: 'Local clock appears monotonic for this preview.' },
      { id: 'ble_adapter', label: 'Bluetooth Low Energy adapter status', status: 'good', description: 'Sandbox adapter active; real BLE is not touched.' },
      { id: 'serial_port_probe', label: 'Serial port status probe', status: 'good', description: 'Active port selected and simulated in sandbox.' }
    ];
    return { ok: true, checks };
  }

  private sandboxHelpSchema() {
    return {
      faq: [
        { q: 'Why am I in sandbox mode?', a: 'The trainer API did not answer /api/status, so the dashboard enabled a local functional preview. Real hardware is not touched.' },
        { q: 'How do I return to live hardware?', a: 'Set your trainer server endpoint address in Settings, or make sure the Python trainer.py script is running on the host.' }
      ]
    };
  }

  private sandboxStart(opts?: RequestInit): SessionRecord {
    let body: Record<string, unknown> = {};
    try {
      body = typeof opts?.body === 'string'
        ? JSON.parse(opts.body) as Record<string, unknown>
        : {};
    } catch (_) {}

    const duration = Number(body['duration_s']) || 30;
    const sid = 'sandbox_' + new Date().toISOString().replace(/[-:.]/g, '').slice(0, 15);
    
    this.state.currentSessionId.set(sid);
    this.state.ctlStopPending.set(false);

    const currentSession = {
      session_id: sid,
      started_at: new Date().toISOString(),
      started_ms: Date.now(),
      duration_s: duration,
      params: { ...body, duration_s: duration },
      sandbox: true
    };
    
    this.state.ctlStatus.set({
      ok: true,
      mode: 'sandbox',
      session: currentSession
    });

    return currentSession;
  }

  private sandboxCurrent(): SessionRecord | null {
    const status = this.state.ctlStatus();
    const cur = status?.session;
    if (!cur) return null;

    const elapsed = Math.max(0, (Date.now() - (cur.started_ms ?? Date.now())) / 1000);
    const dur = Number(cur.duration_s) || 30;

    return {
      ...cur,
      elapsed_s: elapsed,
      remaining_s: Math.max(0, dur - elapsed),
      duration_s: dur,
      preview: { elapsed_s: elapsed },
      status: 'running',
      sandbox: true
    };
  }

  private sandboxStop(): { ok: boolean; session?: SessionRecord; error?: string } {
    const cur = this.sandboxCurrent();
    if (!cur) return { ok: false, error: 'No active session' };

    const item = {
      session_id: cur.session_id,
      started_at: cur.started_at,
      duration_s: cur.duration_s,
      subject: String(cur.params?.['subject_label'] || 'sandbox-subject'),
      operator: String(cur.params?.['operator_label'] || 'sandbox-operator'),
      verdict: 'demo',
      summary: 'Completed simulated session ' + cur.session_id
    };

    const items = [item, ...this.sandboxReadSessions().filter(i => i.session_id !== cur.session_id)];
    this.sandboxSaveSessions(items);

    this.state.currentSessionId.set(null);
    this.state.ctlStatus.set({
      ok: true,
      mode: 'sandbox',
      session: null
    });

    return { ok: true, session: item };
  }

  private sandboxSummary(item: SessionRecord): SessionRecord {
    return {
      ...item,
      sandbox: true,
      status: 'demo',
      downloads: [],
      analysis_status: 'sandbox_only',
      message: 'Demo session summary. No physiological readiness claim is produced from simulated data.'
    };
  }

  private sandboxApiJson(path: string, opts?: RequestInit): unknown {
    if (path === '/api/status') return { ok: true, mode: 'sandbox', message: 'Local dashboard sandbox active' };
    if (path === '/api/defaults') return this.sandboxDefaults();
    if (path === '/api/serial/ports') {
      return { ok: true, ports: ['COM10', 'COM11', 'COM12'].map(device => ({ device, label: `Demo port ${device}` })), selected: this.state.setup().radar_port };
    }
    if (path === '/api/preflight') return this.sandboxPreflight();
    const preflightMatch = path.match(/^\/api\/preflight\/([^/?]+)$/);
    if (preflightMatch) {
      const checkId = decodeURIComponent(preflightMatch[1]);
      return this.sandboxPreflight().checks.find(check => check.id === checkId)
        || { id: checkId, status: 'bad', label: checkId, description: 'Demo check not implemented.' };
    }
    if (path.startsWith('/api/ble/scan')) {
      const devices: BleScanDevice[] = [
        { name: 'Demo oximeter', address: this.state.setup().ble_address, id: this.state.setup().ble_address }
      ];
      return { ok: true, sandbox: true, devices };
    }
    if (path === '/api/subject-profiles') {
      const profiles: Record<string, SubjectProfileRecord> = {
        adult_default: { label: 'Adult Default', age_group: 'adult', fitness_level: 'typical' },
        adult_athlete: { label: 'Adult Athlete', age_group: 'adult', fitness_level: 'athlete' }
      };
      return { schema_version: 'rvt-subject-profiles-v12.0', profiles };
    }
    if (path === '/api/help/schema') return this.sandboxHelpSchema();
    if (path === '/api/session/start') return this.sandboxStart(opts);
    if (path === '/api/session/current') return this.sandboxCurrent();
    if (path === '/api/session/stop') return this.sandboxStop();
    if (path === '/api/sessions') return { items: this.sandboxLoadSessions(), sandbox: true };
    const summaryMatch = path.match(/^\/api\/sessions\/([^/]+)\/summary$/);
    if (summaryMatch) {
      const sessionId = decodeURIComponent(summaryMatch[1]);
      const item = this.sandboxReadSessions().find(session => session.session_id === sessionId);
      return item ? this.sandboxSummary(item) : { error: 'Demo session not found' };
    }
    const notesMatch = path.match(/^\/api\/sessions\/([^/]+)\/notes$/);
    if (notesMatch) {
      const sessionId = decodeURIComponent(notesMatch[1]);
      if (String(opts?.method || 'GET').toUpperCase() === 'PUT') {
        const body = typeof opts?.body === 'string' ? JSON.parse(opts.body) as { review_summary?: string } : {};
        this.state.sessionNotes.update(notes => ({ ...notes, [sessionId]: String(body.review_summary || '').slice(0, 4000) }));
      }
      return { session_id: sessionId, review_summary: this.state.sessionNotes()[sessionId] || '', notes: [] } satisfies SessionNotesPayload;
    }
    const signoffMatch = path.match(/^\/api\/sessions\/([^/]+)\/signoff$/);
    if (signoffMatch) {
      const sessionId = decodeURIComponent(signoffMatch[1]);
      if (String(opts?.method || 'GET').toUpperCase() === 'PUT') {
        const body = typeof opts?.body === 'string' ? JSON.parse(opts.body) as SessionSignoff : {} as SessionSignoff;
        this.state.sessionSignoffs.update(items => ({ ...items, [sessionId]: { ...body, session_id: sessionId, signed_at: new Date().toISOString() } }));
      }
      return this.state.sessionSignoffs()[sessionId] || { session_id: sessionId, operator_name: '', initials: '', validation_comment: '', signed_at: null };
    }
    const dataMatch = path.match(/^\/api\/sessions\/([^/]+)\/data/);
    if (dataMatch) return { ok: true, sandbox: true, rows: [] };
    const compareMatch = path.match(/^\/api\/sessions\/([^/]+)\/compare$/);
    if (compareMatch) {
      const item = this.sandboxReadSessions().find(session => session.session_id === decodeURIComponent(compareMatch[1]));
      return { sandbox: true, selected: item ? this.sandboxSummary(item) : null, previous: null, best: null, sweep_delta_rows: [] };
    }
    const analyseStatusMatch = path.match(/^\/api\/sessions\/([^/]+)\/analyse\/status$/);
    if (analyseStatusMatch) return { sandbox: true, status: 'sandbox_only', progress_pct: 0 };
    return { error: 'Not found in sandbox' };
  }

  private async setTauriPairedOrigin(): Promise<void> {
    const tauri = (window as Window & { __TAURI__?: TauriBridge }).__TAURI__?.core;
    const origin = this.currentApiBase();
    if (tauri?.invoke && origin) {
      await tauri.invoke('native_set_paired_origin', { origin });
    }
  }
}
