import { DestroyRef, Injectable, computed, inject, signal } from '@angular/core';

import { ApiService } from './api.service';
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
  active_pin_expires_at?: number;
  ttl_seconds?: number;
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
  private readonly state = inject(StateService);
  private readonly destroyRef = inject(DestroyRef);

  readonly status = signal<ServerLifecycleStatus>('starting');
  readonly lastError = signal<string | null>(null);
  readonly logTail = signal<string[]>([]);
  readonly serverAddress = signal(this.readServerAddress());
  readonly bindMode = signal<'local' | 'lan'>('local');
  readonly pairingInfo = signal<PairingInfo | null>(null);
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
  private lastSyncedOrigin = '';

  constructor() {
    this.destroyRef.onDestroy(() => this.stopPolling());
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
          const info = await this.api.request<any>('/api/server-info', undefined, true);
          const now = Math.floor(Date.now() / 1000);
          const expiresAt = Number(info.active_pin_expires_at) || 0;
          const ttl = Math.max(0, Math.floor(expiresAt - now));
          this.pairingInfo.set({
            pair_required: info.pair_required,
            active_pin_expires_at: expiresAt,
            ttl_seconds: ttl
          });
        } catch (_) {
          this.pairingInfo.set(null);
        }
      } else {
        this.pairingInfo.set(null);
      }
    } else if (detail.running) {
      this.status.set('starting');
      this.pairingInfo.set(null);
    } else {
      this.status.set('stopped');
      this.pairingInfo.set(null);
    }
    await this.refreshLogTail();
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
