import { Injectable, signal } from '@angular/core';

export type SseTransportState = 'idle' | 'connecting' | 'connected' | 'backoff';

export type SseDriverEvent =
  | { type: 'open' }
  | { type: 'live'; data: string }
  | { type: 'session_warning'; data: string }
  | { type: 'stopped'; data: string }
  | { type: 'data_update' }
  | { type: 'fallback'; reason: 'connection_failed' | 'error_threshold' };

export interface SseDriverConfig {
  canConnect: () => boolean;
  baseUrl: () => string;
  mintToken: () => Promise<string>;
  onEvent: (event: SseDriverEvent) => void;
}

@Injectable({
  providedIn: 'root'
})
export class SseDriverService {
  private config: SseDriverConfig | null = null;
  private source: EventSource | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private pollingRecoveryTimer: ReturnType<typeof setTimeout> | null = null;
  private connectGeneration = 0;
  private connecting = false;
  private reconnectAttempts = 0;
  private recentErrors: number[] = [];
  private pollingOnly = false;

  private readonly transportStateSignal = signal<SseTransportState>('idle');
  readonly transportState = this.transportStateSignal.asReadonly();
  private readonly nextRetryAtMsSignal = signal<number | null>(null);
  readonly nextRetryAtMs = this.nextRetryAtMsSignal.asReadonly();

  isConnected(): boolean {
    return this.transportStateSignal() === 'connected';
  }

  isPollingOnly(): boolean {
    return this.pollingOnly;
  }

  async connect(config?: SseDriverConfig): Promise<void> {
    if (config && (this.source || this.connecting || this.reconnectTimer)) return;
    if (config) this.config = config;
    const activeConfig = this.config;
    if (
      !activeConfig
      || this.pollingOnly
      || typeof EventSource === 'undefined'
      || !activeConfig.canConnect()
      || this.source
      || this.connecting
      || this.reconnectTimer
    ) {
      return;
    }

    const generation = ++this.connectGeneration;
    this.connecting = true;
    this.transportStateSignal.set('connecting');
    let token = '';
    try {
      token = await activeConfig.mintToken();
      if (!this.canOpen(generation, activeConfig)) return;

      const base = activeConfig.baseUrl().replace(/\/+$/, '');
      const subscribePath = '/api/events/subscribe';
      const url = token
        ? `${base}${subscribePath}?token=${encodeURIComponent(token)}`
        : `${base}${subscribePath}`;
      const source = new EventSource(url);
      if (!this.canOpen(generation, activeConfig)) {
        source.close();
        return;
      }
      this.source = source;
      this.bindSource(source, generation, activeConfig);
    } catch (error) {
      if (generation !== this.connectGeneration || !activeConfig.canConnect()) return;
      console.warn('SSE connection failed', error);
      activeConfig.onEvent({ type: 'fallback', reason: 'connection_failed' });
      this.scheduleReconnect();
    } finally {
      if (generation === this.connectGeneration) {
        this.connecting = false;
        if (!this.source && !this.reconnectTimer) {
          this.transportStateSignal.set('idle');
        }
      }
    }
  }

  renew(): void {
    const config = this.config;
    this.cancel();
    this.config = config;
    if (config) void this.connect();
  }

  reset(): void {
    this.pollingOnly = false;
    this.clearPollingRecoveryTimer();
    this.cancel();
  }

  cancel(): void {
    this.connectGeneration++;
    this.connecting = false;
    this.reconnectAttempts = 0;
    this.recentErrors = [];
    this.clearReconnectTimer();
    this.clearPollingRecoveryTimer();
    this.closeSource();
    this.nextRetryAtMsSignal.set(null);
    this.transportStateSignal.set('idle');
  }

  private canOpen(generation: number, config: SseDriverConfig): boolean {
    return generation === this.connectGeneration
      && config === this.config
      && config.canConnect()
      && !this.source;
  }

  private isCurrent(source: EventSource, generation: number): boolean {
    return generation === this.connectGeneration && this.source === source;
  }

  private bindSource(source: EventSource, generation: number, config: SseDriverConfig): void {
    source.onopen = () => {
      if (!this.isCurrent(source, generation)) return;
      this.recentErrors = [];
      this.reconnectAttempts = 0;
      this.clearReconnectTimer();
      this.nextRetryAtMsSignal.set(null);
      this.transportStateSignal.set('connected');
      config.onEvent({ type: 'open' });
    };

    source.addEventListener('live', (event: MessageEvent) => {
      if (!this.isCurrent(source, generation)) return;
      config.onEvent({ type: 'live', data: event.data || '' });
    });
    source.addEventListener('session_warning', (event: MessageEvent) => {
      if (!this.isCurrent(source, generation)) return;
      config.onEvent({ type: 'session_warning', data: event.data || '' });
    });
    source.addEventListener('stopped', (event: MessageEvent) => {
      if (!this.isCurrent(source, generation)) return;
      config.onEvent({ type: 'stopped', data: event.data || '' });
    });
    source.addEventListener('data_update', () => {
      if (!this.isCurrent(source, generation)) return;
      config.onEvent({ type: 'data_update' });
    });

    source.onerror = () => {
      if (!this.isCurrent(source, generation)) return;
      // Single-use SSE tokens cannot be replayed by browser auto-reconnect;
      // close the stale EventSource immediately so the next attempt mints a
      // fresh token.
      this.closeSource();
      const now = Date.now();
      this.recentErrors = this.recentErrors.filter(timestamp => now - timestamp < 180_000);
      this.recentErrors.push(now);
      if (this.recentErrors.length <= 3) {
        config.onEvent({ type: 'fallback', reason: 'connection_failed' });
        this.scheduleReconnect();
        return;
      }

      console.warn('SSE failure threshold reached. Falling back to polling.');
      this.pollingOnly = true;
      this.clearReconnectTimer();
      this.nextRetryAtMsSignal.set(null);
      this.transportStateSignal.set('idle');
      config.onEvent({ type: 'fallback', reason: 'error_threshold' });
      this.schedulePollingRecovery();
    };
  }

  private schedulePollingRecovery(): void {
    if (!this.pollingOnly || !this.config || !this.config.canConnect()) return;
    this.clearPollingRecoveryTimer();
    this.pollingRecoveryTimer = setTimeout(() => {
      this.pollingRecoveryTimer = null;
      if (!this.pollingOnly || !this.config?.canConnect()) return;
      this.pollingOnly = false;
      this.recentErrors = [];
      this.reconnectAttempts = 0;
      void this.connect();
    }, 60_000);
  }

  private clearPollingRecoveryTimer(): void {
    if (!this.pollingRecoveryTimer) return;
    clearTimeout(this.pollingRecoveryTimer);
    this.pollingRecoveryTimer = null;
  }

  private scheduleReconnect(): void {
    const config = this.config;
    if (this.pollingOnly || !config || !config.canConnect()) {
      this.transportStateSignal.set('idle');
      return;
    }
    this.clearReconnectTimer();
    const backoffSeconds = this.reconnectAttempts === 0
      ? 15
      : this.reconnectAttempts === 1
        ? 30
        : 60;
    const jitterMs = Math.floor(Math.random() * 2000) - 1000;
    const delayMs = Math.max(1000, backoffSeconds * 1000 + jitterMs);
    this.nextRetryAtMsSignal.set(Date.now() + delayMs);
    this.transportStateSignal.set('backoff');
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.nextRetryAtMsSignal.set(null);
      if (!this.config?.canConnect()) {
        this.transportStateSignal.set('idle');
        return;
      }
      this.reconnectAttempts++;
      void this.connect();
    }, delayMs);
  }

  private clearReconnectTimer(): void {
    if (!this.reconnectTimer) return;
    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = null;
  }

  private closeSource(): void {
    if (!this.source) return;
    try {
      this.source.close();
    } catch (_) {}
    this.source = null;
  }
}
