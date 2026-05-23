import { Injectable, inject } from '@angular/core';
import { StateService } from './state.service';

@Injectable({
  providedIn: 'root'
})
export class ApiService {
  private state = inject(StateService);

  private readonly API_BASE_KEY = 'rvt-api-base';
  private readonly TOKEN_KEY = 'rvt-pair-token';

  constructor() {
    this.detectControlMode();
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

  // Low-level HTTP requests with native Tauri/Capacitor plugins fallback
  async request(path: string, init?: RequestInit): Promise<any> {
    const isSandbox = this.state.autoDemoActive() || this.state.demoMode() || (this.state.ctlOn() && this.state.ctlStatus()?.['mode'] === 'sandbox');

    if (isSandbox && path.startsWith('/api/')) {
      return this.sandboxApiJson(path, init);
    }

    const base = this.currentApiBase();
    const target = /^[a-z][a-z0-9+.-]*:\/\//i.test(String(path)) ? String(path) : base + String(path);

    const headers = new Headers(init?.headers || {});
    // Security design decision: We keep the pairing auth token in sessionStorage.
    // This is intentional so that it does not persist across separate browser tabs or survive
    // browser session closure, preventing authentication leaks on shared operator workstations.
    const tok = sessionStorage.getItem(this.TOKEN_KEY);
    if (tok) {
      headers.set('X-RVT-Auth', tok);
    }

    // Try Capacitor native Http wrapper
    const cap = (window as any).Capacitor;
    const http = cap?.Plugins?.CapacitorHttp || cap?.Plugins?.Http || (window as any).CapacitorHttp || (window as any).Http;
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
          const err: any = new Error(data?.error?.message || data?.error || `HTTP ${status}`);
          err.status = status;
          err.body = data;
          throw err;
        }
        return typeof data === 'string' ? JSON.parse(data || '{}') : data;
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
    let data: any = {};
    try {
      data = JSON.parse(text);
    } catch (_) {
      data = { text };
    }

    if (!res.ok) {
      const err: any = new Error(data?.error?.message || data?.error || `HTTP ${res.status}`);
      err.status = res.status;
      err.body = data;
      throw err;
    }

    return data;
  }

  // Detect control mode / connect
  async detectControlMode(): Promise<boolean> {
    try {
      const r = await this.request('/api/status');
      this.state.ctlOn.set(true);
      this.state.ctlStatus.set(r);
      return true;
    } catch (e: any) {
      this.enableSandboxControlMode(e.message || 'Control API unavailable');
      return false;
    }
  }

  enableSandboxControlMode(reason: string) {
    this.state.ctlOn.set(true);
    this.state.ctlStatus.set({
      ok: true,
      mode: 'sandbox',
      reason
    });
    this.sandboxLoadSessions();
  }

  // --- LOCAL SANDBOX / MOCK ENDPOINTS ---

  private sandboxStoreKey(): string {
    return 'rvt-sandbox-sessions';
  }

  private sandboxReadSessions(): any[] {
    try {
      const raw = localStorage.getItem(this.sandboxStoreKey());
      if (raw) {
        const parsed = JSON.parse(raw);
        if (Array.isArray(parsed)) return parsed;
      }
    } catch (_) {}
    return [];
  }

  private sandboxLoadSessions(): any[] {
    const raw = localStorage.getItem(this.sandboxStoreKey());
    if (raw === null) {
      // Seed default sessions only on very first launch (when key doesn't exist)
      const iso = (mins: number) => new Date(Date.now() - mins * 60000).toISOString();
      const items = [
        {
          session_id: 'sandbox_20260420_091800',
          started_at: iso(64),
          duration_s: 480,
          subject: 'demo-A',
          operator: 'codex',
          verdict: 'ready',
          summary: 'Completed 8 min sandbox telemetry review; stable waveforms recorded.'
        },
        {
          session_id: 'sandbox_20260419_141803',
          started_at: iso(1240),
          duration_s: 300,
          subject: 'demo-B',
          operator: 'codex',
          verdict: 'conditional',
          summary: 'Completed 5 min demo run; minor breathing anomalies observed.'
        }
      ];
      try {
        localStorage.setItem(this.sandboxStoreKey(), JSON.stringify(items));
      } catch (_) {}
      this.state.sessionItems.set(items);
      return items;
    }
    const items = this.sandboxReadSessions();
    this.state.sessionItems.set(items);
    return items;
  }

  private sandboxSaveSessions(items: any[]) {
    try {
      localStorage.setItem(this.sandboxStoreKey(), JSON.stringify(items));
    } catch (_) {}
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

  private sandboxPreflight(url: string) {
    const checks = [
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

  private sandboxStart(opts: any) {
    let body: any = {};
    try {
      body = typeof opts?.body === 'string' ? JSON.parse(opts.body) : (opts?.body || {});
    } catch (_) {}

    const duration = Number(body.duration_s) || 30;
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

  private sandboxCurrent() {
    const status = this.state.ctlStatus();
    const cur = status?.session;
    if (!cur) return null;

    const elapsed = Math.max(0, (Date.now() - cur.started_ms) / 1000);
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

  private sandboxStop() {
    const cur = this.sandboxCurrent();
    if (!cur) return { ok: false, error: 'No active session' };

    const item = {
      session_id: cur.session_id,
      started_at: cur.started_at,
      duration_s: cur.duration_s,
      subject: cur.params?.subject_label || 'sandbox-subject',
      operator: cur.params?.operator_label || 'sandbox-operator',
      verdict: 'ready',
      summary: 'Completed sandbox session ' + cur.session_id
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

  private sandboxApiJson(path: string, opts?: any): any {
    if (path === '/api/status') return { ok: true, mode: 'sandbox', message: 'Local dashboard sandbox active' };
    if (path === '/api/defaults') return this.sandboxDefaults();
    if (path.startsWith('/api/preflight')) return this.sandboxPreflight(path);
    if (path === '/api/help/schema') return this.sandboxHelpSchema();
    if (path === '/api/session/start') return this.sandboxStart(opts);
    if (path === '/api/session/current') return this.sandboxCurrent();
    if (path === '/api/session/stop') return this.sandboxStop();
    if (path === '/api/sessions') return { items: this.sandboxReadSessions(), sandbox: true };
    return { error: 'Not found in sandbox' };
  }
}
