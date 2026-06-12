import { DestroyRef, Injectable, computed, inject, signal } from '@angular/core';

import { ApiService } from './api.service';
import { AuthService } from './auth.service';
import { SERVER_URL_KEY } from './rvt-storage-keys';
import { StateService } from './state.service';

export type ServerLifecycleStatus = 'starting' | 'running' | 'stopped' | 'error' | 'offline';
export type ServerLifecyclePlatform = 'exe' | 'remote';

const DEFAULT_SERVER_ORIGIN = 'http://127.0.0.1:8765';
const POLL_MS = 2000;

interface TauriCore {
  invoke<T = unknown>(command: string, args?: Record<string, unknown>): Promise<T>;
}

interface TauriBridge {
  core?: TauriCore;
}

interface TauriLifecycleResponse {
  state?: 'starting' | 'running' | 'stopped' | 'error';
  message?: string;
}

export interface PairingInfo {
  pair_required?: boolean;
  origin?: string;
  active_pin?: string;
  active_pin_expires_at?: number;
  qr_png_base64?: string;
}

interface NativeTrainerStatus {
  running?: boolean;
  ready?: boolean;
  origin?: string | null;
  error?: string | null;
  bind_mode?: 'local' | 'lan';
}

@Injectable({
  providedIn: 'root'
})
export class ServerLifecycleService {
  private readonly api = inject(ApiService);
  private readonly auth = inject(AuthService);
  private readonly state = inject(StateService);
  private readonly destroyRef = inject(DestroyRef);

  readonly status = signal<ServerLifecycleStatus>('starting');
  readonly lastError = signal<string | null>(null);
  readonly logTail = signal<string[]>([]);
  readonly serverAddress = signal(this.readServerAddress());
  readonly bindMode = signal<'local' | 'lan'>('local');
  readonly pairingInfo = signal<PairingInfo | null>(null);
  private readonly nowEpochS = signal(Math.floor(Date.now() / 1000));
  readonly pairingTtlSeconds = computed<number | null>(() => {
    const expiresAt = this.pairingInfo()?.active_pin_expires_at;
    if (!expiresAt) return null;
    return Math.max(0, Math.floor(expiresAt - this.nowEpochS()));
  });
  readonly pairingUrl = computed<string>(() => {
    const info = this.pairingInfo();
    if (!info?.origin) return '';
    const origin = info.origin.replace(/\/+$/, '');
    const ttl = this.pairingTtlSeconds();
    if (info.active_pin && (ttl === null || ttl > 0)) {
      return `${origin}/?pair=${info.active_pin}`;
    }
    return `${origin}/pair`;
  });
  readonly pairingQrDataUrl = computed<string>(() => {
    const qr = this.pairingInfo()?.qr_png_base64;
    return qr ? `data:image/png;base64,${qr}` : '';
  });
  readonly platform = computed<ServerLifecyclePlatform>(() => this.hasTauriDesktop() ? 'exe' : 'remote');
  readonly blocksLive = computed(() => !this.state.demoMode() && ['offline', 'error', 'stopped'].includes(this.status()));
  readonly statusLabel = computed(() => {
    switch (this.status()) {
      case 'running': return 'Server running';
      case 'starting': return 'Starting...';
      case 'stopped': return 'Server stopped';
      case 'error': return 'Server error';
      case 'offline': return 'Server offline';
    }
  });

  private bootstrapStarted = false;
  private pollTimer: number | null = null;
  private pairingTicker: number | null = null;
  private lastSyncedOrigin = '';

  constructor() {
    this.destroyRef.onDestroy(() => {
      this.stopPolling();
      this.stopPairingTicker();
    });
    void this.bootstrap();
  }

  async bootstrap(): Promise<void> {
    if (this.bootstrapStarted) return;
    this.bootstrapStarted = true;
    if (this.hasTauriDesktop()) {
      await this.startServer();
    } else {
      await this.retryConnection();
    }
  }

  setServerAddress(value: string): string {
    const normalized = this.api.setApiBase(value || DEFAULT_SERVER_ORIGIN) || DEFAULT_SERVER_ORIGIN;
    this.serverAddress.set(normalized);
    try {
      localStorage.setItem(SERVER_URL_KEY, normalized);
    } catch (_) {}
    return normalized;
  }

  async startServer(bindMode?: 'local' | 'lan'): Promise<void> {
    if (!this.hasTauriDesktop()) {
      await this.retryConnection();
      return;
    }
    this.status.set('starting');
    this.lastError.set(null);
    try {
      const lifecycle = await this.invoke<TauriLifecycleResponse>('trainer_start', { bindMode: bindMode || 'local' });
      this.applyLifecycle(lifecycle);
      await this.refreshTauriDetails();
      this.startPolling();
    } catch (error: unknown) {
      this.setError(error instanceof Error ? error.message : 'Could not start bundled trainer.');
    }
  }

  async stopServer(): Promise<void> {
    if (!this.hasTauriDesktop()) {
      this.status.set('offline');
      return;
    }
    try {
      const lifecycle = await this.invoke<TauriLifecycleResponse>('trainer_stop');
      this.applyLifecycle(lifecycle);
      await this.refreshLogTail();
    } catch (error: unknown) {
      this.setError(error instanceof Error ? error.message : 'Could not stop bundled trainer.');
    }
  }

  async restartServer(bindMode?: 'local' | 'lan'): Promise<void> {
    if (this.hasTauriDesktop()) {
      await this.stopServer();
      await this.startServer(bindMode);
      // The sidecar restart wipes the trainer's in-memory operator sessions, so
      // the pre-restart token is dead. Lock the station immediately: the lock
      // overlay walks the operator through PIN re-login against the on-disk
      // profiles instead of leaving a half-broken authenticated UI.
      this.auth.lock();
    } else {
      await this.retryConnection();
    }
  }

  async retryConnection(): Promise<void> {
    this.status.set('starting');
    this.lastError.set(null);
    this.setServerAddress(this.serverAddress());
    const connected = await this.api.checkConnection();
    if (!connected) {
      this.status.set('offline');
      this.lastError.set(`No Python trainer answered ${this.serverAddress()}/api/health.`);
      return;
    }
    await this.api.detectControlMode();
    this.status.set('running');
  }

  openPairPage(): void {
    if (typeof window === 'undefined') return;
    window.open(this.pairUrl(), '_blank', 'noopener');
  }

  pairUrl(): string {
    return `${this.serverAddress().replace(/\/+$/, '')}/pair`;
  }

  async refreshLogTail(): Promise<void> {
    if (!this.hasTauriDesktop()) return;
    try {
      const lines = await this.invoke<string[]>('trainer_log_tail');
      this.logTail.set(Array.isArray(lines) ? lines.slice(-20) : []);
    } catch (_) {}
  }

  private async refreshTauriDetails(): Promise<void> {
    const detail = await this.invoke<NativeTrainerStatus>('native_trainer_status');
    if (detail.origin) {
      this.setServerAddress(detail.origin);
    }
    if (detail.bind_mode) {
      this.bindMode.set(detail.bind_mode);
    }
    if (detail.error) {
      this.setError(detail.error);
      this.pairingInfo.set(null);
    } else if (detail.running && detail.ready) {
      this.status.set('running');
      if (detail.origin && detail.origin !== this.lastSyncedOrigin) {
        this.lastSyncedOrigin = detail.origin;
        await this.api.detectControlMode();
      }
      if (this.bindMode() === 'lan') {
        try {
          const info = await this.api.request<Record<string, unknown>>('/api/native-pairing-info?format=qr', undefined, true);
          this.setPairingInfo(this.parsePairingInfo(info));
        } catch (_) {
          try {
            const info = await this.api.request<Record<string, unknown>>('/api/server-info', undefined, true);
            this.setPairingInfo(this.parsePairingInfo(info));
          } catch (_) {
            this.setPairingInfo(null);
          }
        }
      } else {
        this.setPairingInfo(null);
      }
    } else if (detail.running) {
      this.status.set('starting');
      this.setPairingInfo(null);
    } else {
      this.status.set('stopped');
      this.setPairingInfo(null);
    }
    await this.refreshLogTail();
  }

  private parsePairingInfo(raw: Record<string, unknown> | null | undefined): PairingInfo {
    const envelope = (raw && typeof raw === 'object' ? raw : {}) as Record<string, unknown>;
    const payload = (envelope['data'] && typeof envelope['data'] === 'object'
      ? envelope['data']
      : envelope) as Record<string, unknown>;
    const host = typeof payload['host'] === 'string' ? payload['host'] : '';
    const port = Number(payload['port']);
    const origin = typeof payload['origin'] === 'string' && payload['origin']
      ? String(payload['origin'])
      : (host && Number.isFinite(port) && port > 0 ? `http://${host}:${port}` : '');
    const pinRaw = payload['active_pin'];
    const pin = typeof pinRaw === 'string' || typeof pinRaw === 'number' ? String(pinRaw) : '';
    const expiresAt = Number(payload['active_pin_expires_at']);
    const qrRaw = payload['qr_png_base64'];
    const qr = typeof qrRaw === 'string' && /^[A-Za-z0-9+/=]+$/.test(qrRaw) ? qrRaw : '';
    return {
      pair_required: payload['pair_required'] === true,
      origin: origin || undefined,
      active_pin: /^\d{6}$/.test(pin) ? pin : undefined,
      active_pin_expires_at: Number.isFinite(expiresAt) && expiresAt > 0 ? expiresAt : undefined,
      qr_png_base64: qr || undefined
    };
  }

  private setPairingInfo(info: PairingInfo | null): void {
    this.pairingInfo.set(info);
    if (info?.active_pin_expires_at) {
      this.nowEpochS.set(Math.floor(Date.now() / 1000));
      this.startPairingTicker();
    } else {
      this.stopPairingTicker();
    }
  }

  private startPairingTicker(): void {
    if (typeof window === 'undefined' || this.pairingTicker !== null) return;
    this.pairingTicker = window.setInterval(() => {
      this.nowEpochS.set(Math.floor(Date.now() / 1000));
    }, 1000);
  }

  private stopPairingTicker(): void {
    if (typeof window === 'undefined' || this.pairingTicker === null) return;
    window.clearInterval(this.pairingTicker);
    this.pairingTicker = null;
  }

  private startPolling(): void {
    if (typeof window === 'undefined' || this.pollTimer !== null) return;
    this.pollTimer = window.setInterval(() => {
      void this.refreshTauriDetails().catch(error => {
        this.setError(error instanceof Error ? error.message : 'Trainer status polling failed.');
      });
    }, POLL_MS);
  }

  private stopPolling(): void {
    if (typeof window === 'undefined' || this.pollTimer === null) return;
    window.clearInterval(this.pollTimer);
    this.pollTimer = null;
  }

  private applyLifecycle(response: TauriLifecycleResponse): void {
    const state = response?.state;
    if (state === 'error') {
      this.setError(response.message || 'Bundled trainer failed.');
    } else if (state === 'running' || state === 'starting' || state === 'stopped') {
      this.status.set(state);
      this.lastError.set(null);
    }
  }

  private setError(message: string): void {
    this.status.set('error');
    this.lastError.set(message);
  }

  private invoke<T>(command: string, args?: Record<string, unknown>): Promise<T> {
    const core = this.tauriCore();
    if (!core) throw new Error('Tauri IPC is not available in this shell.');
    return core.invoke<T>(command, args);
  }

  private tauriCore(): TauriCore | null {
    if (typeof window === 'undefined') return null;
    return ((window as Window & { __TAURI__?: TauriBridge }).__TAURI__?.core) || null;
  }

  private hasTauriDesktop(): boolean {
    if (!this.tauriCore()) return false;
    if (typeof navigator === 'undefined') return true;
    return !/android|iphone|ipad|ipod/i.test(navigator.userAgent);
  }

  private readServerAddress(): string {
    try {
      return this.api.currentApiBase() || localStorage.getItem(SERVER_URL_KEY) || this.defaultRemoteOrigin();
    } catch (_) {
      return this.defaultRemoteOrigin();
    }
  }

  private defaultRemoteOrigin(): string {
    if (typeof window === 'undefined') return DEFAULT_SERVER_ORIGIN;
    try {
      const { protocol, hostname, origin } = window.location;
      if (!/^https?:$/.test(protocol)) return DEFAULT_SERVER_ORIGIN;
      const localOrLan = /^(localhost|127\.0\.0\.1|0\.0\.0\.0|10\.|192\.168\.|172\.(1[6-9]|2\d|3[0-1])\.)/i.test(hostname);
      return localOrLan ? origin : DEFAULT_SERVER_ORIGIN;
    } catch (_) {
      return DEFAULT_SERVER_ORIGIN;
    }
  }
}
