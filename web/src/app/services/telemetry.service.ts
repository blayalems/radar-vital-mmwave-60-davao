import { Injectable, effect, inject, untracked } from '@angular/core';
import { LivePayload } from '../models/rvt.models';
import { AudioService } from './audio.service';
import { StateService } from './state.service';
import { ApiService } from './api.service';
import { AuthService } from './auth.service';
import { OPERATOR_TOKEN_KEY } from './rvt-storage-keys';
import { SourceModeService } from './source-mode.service';
import { SseDriverEvent, SseDriverService } from './sse-driver.service';

@Injectable({
  providedIn: 'root',
})
export class TelemetryService {
  private state = inject(StateService);
  private api = inject(ApiService);
  private audio = inject(AudioService);
  private auth = inject(AuthService);
  private sourceMode = inject(SourceModeService);
  private sseDriver = inject(SseDriverService);

  private pollTimer: ReturnType<typeof setTimeout> | null = null;
  private staleTimer: ReturnType<typeof setTimeout> | null = null;
  private running = false;
  private browserOnline = typeof navigator === 'undefined' || navigator.onLine;
  private transportGeneration = 0;
  private httpPollFailures = 0;
  private demoT = 0;
  private readonly alertCooldownUntil = new Map<string, number>();
  private readonly handleBrowserOffline = () => this.onBrowserOffline();
  private readonly handleBrowserOnline = () => this.onBrowserOnline();
  /** Epoch ms of the next scheduled SSE reconnect attempt (null when connected). */
  readonly nextRetryAtMs = this.sseDriver.nextRetryAtMs;

  constructor() {
    window.addEventListener('offline', this.handleBrowserOffline);
    window.addEventListener('online', this.handleBrowserOnline);
    this.start();
    window.addEventListener('rvt-operator-authenticated', () => this.reconnect());
    effect(() => {
      const simulating = this.state.demoSourceActive();
      const isLocked = this.auth.isLocked();
      if (isLocked) {
        this.stopSse();
        this.clearPollTimer();
      } else if (simulating) {
        this.stopSse();
        this.scheduleNextPoll(0);
      } else if (this.running) {
        this.stopSse();
        this.clearPollTimer();
        untracked(() => {
          void this.api.detectControlMode().then(() => {
            this.scheduleNextPoll(0);
            this.startSse();
          });
        });
      }
    });
  }

  start() {
    this.running = true;
    if (!this.browserOnline && !this.state.demoSourceActive()) {
      this.markLiveTransportOffline();
      return;
    }
    this.scheduleNextPoll(1000);
    this.startSse();
  }

  reconnect(): void {
    if (!this.browserOnline && !this.state.demoSourceActive()) {
      this.markLiveTransportOffline();
      return;
    }
    this.sseDriver.reset();
    this.clearPollTimer();
    this.httpPollFailures = 0;
    if (!this.running) {
      this.running = true;
    }
    this.scheduleNextPoll(0);
    void this.startSse();
  }

  stop() {
    this.running = false;
    this.transportGeneration++;
    this.clearPollTimer();
    this.stopSse();
    this.clearStaleTimer();
  }

  private clearPollTimer() {
    if (this.pollTimer) {
      clearTimeout(this.pollTimer);
      this.pollTimer = null;
    }
  }

  private scheduleNextPoll(delayMs: number) {
    if (!this.running) return;
    if (!this.browserOnline && !this.state.demoSourceActive()) return;
    this.clearPollTimer();
    this.pollTimer = setTimeout(() => this.poll(), delayMs);
  }

  private async poll() {
    if (!this.running) return;
    if (!this.browserOnline && !this.state.demoSourceActive()) return;
    const transportGeneration = this.transportGeneration;

    if (this.state.paused()) {
      this.scheduleNextPoll(this.state.liveBufferSeconds() * 1000);
      return;
    }

    if (this.state.demoSourceActive()) {
      this.runSimulationStep();
      this.scheduleNextPoll(1000);
      return;
    }

    if (this.sseDriver.isConnected()) return;

    this.state.ctlStatus.set(this.state.ctlStatus() || { ok: true, mode: 'loading' });

    try {
      const path = '/api/session/current/live_dashboard.json';
      const startMs = Date.now();
      const payload = await this.api.request<Partial<LivePayload>>(`${path}?t=${Date.now()}`);
      if (!this.browserOnline || transportGeneration !== this.transportGeneration) return;
      const latency = Date.now() - startMs;
      if (!this.applyLivePayload(payload)) {
        throw new Error('live payload missing telemetry fields');
      }
      this.httpPollFailures = 0;
      this.state.ctlStatus.update((s) => ({ ...(s ?? { ok: true }), ok: true, latency }));
      this.scheduleNextPoll(500);
    } catch (error: unknown) {
      if (!this.browserOnline || transportGeneration !== this.transportGeneration) return;
      const message = error instanceof Error ? error.message : 'poll failed';
      console.warn('Telemetry poll failed', error);
      this.state.telemetryStale.set(true);
      this.state.ctlStatus.update((s) => ({ ...(s ?? { ok: false }), ok: false, error: message }));
      const noActiveSession = /NO_ACTIVE_SESSION|NO_LIVE_DASHBOARD|no active session|active live_dashboard\.json/i.test(message);
      if (noActiveSession) {
        // A status probe can report a real active session before its first
        // dashboard payload is written. Keep that authoritative witness so a
        // transient NO_LIVE_DASHBOARD response cannot let the operator switch
        // to simulated data over a live capture.
        const status = this.state.ctlStatus();
        const activeSession = status?.active_session ?? status?.session;
        const activeSessionId = String(activeSession?.session_id || '');
        if (status?.mode !== 'sandbox' && activeSessionId) {
          this.state.sessionActive.set(true);
          this.state.currentSessionId.set(activeSessionId);
          this.state.ctlStatus.update((s) => ({
            ...(s ?? { ok: true }),
            ok: true,
            error: undefined,
            last_poll_reason: 'No active telemetry payload yet'
          }));
        } else {
          this.state.sessionActive.set(false);
          this.state.currentSessionId.set(null);
          this.state.ctlStatus.update((s) => ({
            ...(s ?? { ok: true }),
            ok: true,
            error: undefined,
            last_stop_reason: 'No active telemetry session'
          }));
        }
        this.scheduleNextPoll(1000);
      } else {
        this.emitAlert(`Live connection unavailable: ${message}`, 'critical');
        if (this.state.autoDemoOnDisconnect() && this.sourceMode.setAutomaticDemoActive(true)) {
          this.scheduleNextPoll(0);
        } else {
          this.httpPollFailures++;
          const baseDelay = Math.min(60_000, 3_000 * 2 ** Math.min(this.httpPollFailures - 1, 4));
          const jitter = Math.floor(Math.random() * 1000) - 500;
          this.scheduleNextPoll(Math.max(1000, baseDelay + jitter));
        }
      }
    }
  }

  private async startSse(): Promise<void> {
    // Tauri keeps browser CSP at connect-src 'self'. EventSource bypasses the
    // HttpClient interceptor, so the native shell uses origin-pinned polling.
    if (this.isTauriNative()) return;
    if (this.state.demoSourceActive()) return;
    if (!this.running) return;
    if (!this.browserOnline) return;
    if (this.api.hasPairToken() && !sessionStorage.getItem(OPERATOR_TOKEN_KEY)) {
      this.scheduleNextPoll(0);
      return;
    }

    await this.sseDriver.connect({
      canConnect: () => this.running
        && this.browserOnline
        && !this.auth.isLocked()
        && !this.state.demoSourceActive()
        && !this.isTauriNative(),
      baseUrl: () => this.api.currentApiBase(),
      mintToken: async () => {
        if (!sessionStorage.getItem(OPERATOR_TOKEN_KEY)) return '';
        const response = await this.api.request<{ sse_token: string }>(
          '/api/auth/sse-token',
          { method: 'POST' }
        );
        return response?.sse_token || '';
      },
      onEvent: event => this.handleSseEvent(event)
    });
  }

  private renewSseConnection(): void {
    this.sseDriver.renew();
  }

  private handleSseEvent(event: SseDriverEvent): void {
    switch (event.type) {
      case 'open':
        console.log('SSE connection successfully opened.');
        this.clearPollTimer();
        return;
      case 'live':
        try {
          if (this.applyLivePayload(JSON.parse(event.data || '{}'))) {
            this.state.ctlStatus.update(status => ({
              ...(status ?? { ok: true }),
              ok: true,
              mode: 'live',
              error: undefined,
              reason: undefined
            }));
          }
        } catch (error) {
          console.warn('SSE live parse failed', error);
        }
        return;
      case 'session_warning': {
        const payload = this.parseSseJson(event.data);
        // The contractual 12 h stream deadline (AGENTS.md invariant 11) is routine:
        // tell the operator it is automatic and immediately rotate the one-use token.
        const isDeadline = payload['reason'] === 'deadline_approaching';
        const message = isDeadline
          ? 'Live stream is renewing automatically — no action needed.'
          : this.eventMessage(payload, 'Session warning from telemetry stream.');
        this.emitAlert(message, 'warn', 'sse-session-warning');
        if (isDeadline) this.renewSseConnection();
        return;
      }
      case 'stopped': {
        const payload = this.parseSseJson(event.data);
        const message = this.eventMessage(payload, 'Telemetry session stopped.');
        this.reconcileStoppedEvent(payload, message);
        return;
      }
      case 'data_update':
        this.state.ctlStatus.update((status) => ({
          ...(status ?? { ok: true }),
          ok: true,
          last_data_update_ms: Date.now()
        }));
        return;
      case 'fallback':
        this.scheduleNextPoll(0);
        return;
    }
  }

  private isTauriNative(): boolean {
    return Boolean((window as any).__TAURI__?.core?.invoke);
  }

  private parseSseJson(data: string): Record<string, unknown> {
    try {
      const parsed = JSON.parse(data || '{}');
      return parsed && typeof parsed === 'object' ? parsed as Record<string, unknown> : {};
    } catch (_) {
      return {};
    }
  }

  private eventMessage(payload: Record<string, unknown>, fallback: string): string {
    return String(payload['message'] || payload['msg'] || payload['reason'] || payload['code'] || fallback);
  }

  private reconcileStoppedEvent(payload: Record<string, unknown>, message: string): void {
    const serverSessionId = String(payload['session_id'] || payload['active_session_id'] || '');
    const currentSessionId = this.state.currentSessionId();
    const serverDeclaresDifferentSession = Boolean(serverSessionId && currentSessionId && serverSessionId !== currentSessionId);

    this.state.sessionActive.set(false);
    this.state.currentSessionId.set(null);
    this.state.telemetryStale.set(true);
    this.state.ctlStopPending.set(false);
    this.state.ctlStatus.update((s) => ({
      ...(s ?? { ok: true }),
      ok: true,
      session: null,
      active_session: null,
      last_stop_reason: message,
      last_stop_ms: Date.now(),
      replaced_session: serverDeclaresDifferentSession
    }));
    this.emitAlert(message, serverDeclaresDifferentSession ? 'critical' : 'warn', 'sse-stopped');
    this.stopSse();
    this.scheduleNextPoll(0);
  }

  private stopSse(): void {
    this.sseDriver.cancel();
  }

  private onBrowserOffline(): void {
    this.browserOnline = false;
    if (!this.running || this.state.demoSourceActive()) return;
    this.transportGeneration++;
    this.clearPollTimer();
    this.stopSse();
    this.clearStaleTimer();
    this.markLiveTransportOffline();
  }

  private onBrowserOnline(): void {
    const wasOffline = !this.browserOnline;
    this.browserOnline = true;
    if (!wasOffline || !this.running || this.auth.isLocked() || this.state.demoSourceActive()) return;
    this.transportGeneration++;
    this.state.ctlStatus.update(status => ({
      ...(status ?? { ok: false }),
      ok: false,
      mode: 'live',
      error: 'Reconnecting to trainer...',
      reason: 'browser_online'
    }));
    this.reconnect();
  }

  private markLiveTransportOffline(): void {
    this.state.telemetryStale.set(true);
    this.state.ctlStatus.update(status => ({
      ...(status ?? { ok: false }),
      ok: false,
      mode: 'live',
      error: 'Browser is offline.',
      reason: 'browser_offline'
    }));
  }

  private clearStaleTimer(): void {
    if (!this.staleTimer) return;
    clearTimeout(this.staleTimer);
    this.staleTimer = null;
  }

  private runSimulationStep() {
    this.demoT += 1;
    const t = this.demoT;
    const N = 120;
    const hrBase = 72 + 4 * Math.sin(t / 12);
    const rrBase = 15 + 1.5 * Math.sin(t / 18 + 1);
    // Simulated lock-state lifecycle: warmup -> settling -> locked, with a short
    // motion burst every 45 s so the motion chip and "holding" states are demo-visible.
    const inMotion = t % 45 >= 42 ? 1 : 0;
    const phaseName = inMotion ? 'POST_MOTION' : t < 6 ? 'WARMUP' : t < 14 ? 'SETTLING' : 'LOCKED';
    const phaseCode = ({ WARMUP: 1, SETTLING: 2, LOCKED: 3, POST_MOTION: 4 } as Record<string, number>)[phaseName];
    const ts: number[] = [];
    const rep_hr: number[] = [];
    const can_hr: number[] = [];
    const raw_hr: number[] = [];
    const raw_hr_corrected: number[] = [];
    const ble_hr: number[] = [];
    const rep_rr: number[] = [];
    const can_rr: number[] = [];
    const ble_rr: number[] = [];
    const breath: number[] = [];
    const heart: number[] = [];

    for (let i = 0; i < N; i++) {
      ts[i] = t - N + i;
      const repHrVal = hrBase + 2 * Math.sin(i / 3 + t / 5) + (Math.random() - 0.5);
      rep_hr[i] = repHrVal;
      can_hr[i] = repHrVal + (Math.random() - 0.5) * 1.2;
      const rawHrVal = repHrVal + (Math.random() - 0.5) * 3.5 + 1.2;
      raw_hr[i] = rawHrVal;
      raw_hr_corrected[i] = rawHrVal - Math.max(0, Math.min(25, 0.4 * (rawHrVal - 55)));
      ble_hr[i] = repHrVal + (Math.random() - 0.5) * 0.7 - 0.3;
      const repRrVal = rrBase + 0.4 * Math.sin(i / 4 + t / 6) + (Math.random() - 0.5) * 0.15;
      rep_rr[i] = repRrVal;
      can_rr[i] = repRrVal + (Math.random() - 0.5) * 0.3;
      ble_rr[i] = repRrVal + (Math.random() - 0.5) * 0.2;
      breath[i] = Math.sin((i + t) * 0.12) + 0.1 * Math.sin((i + t) * 0.5);
      heart[i] = Math.sin((i + t) * 0.9) * 0.6 + 0.15 * Math.sin((i + t) * 3.5);
    }

    const x = 0.35 + 0.18 * Math.sin(t / 14);
    const y = 0.80 + 0.10 * Math.cos(t / 16);
    const distCm = Math.hypot(x, y) * 100;
    const payload = {
      meta: {
        status: this.state.currentSessionId() ? 'running' : 'waiting',
        elapsed_s: t,
        remaining_s: Math.max(0, (this.state.setup().duration_s || 30) - t),
        session_dir: '/sessions/demo_2026_04_18',
        sandbox: true,
      },
      radar: {
        rows: 1450 + t,
        fps_hz: 20,
        reported_hr: rep_hr.at(-1),
        reported_rr: rep_rr.at(-1),
        candidate_hr: can_hr.at(-1),
        candidate_rr: can_rr.at(-1),
        raw_hr: raw_hr.at(-1),
        raw_hr_uncorrected: raw_hr.at(-1),
        raw_hr_corrected: raw_hr_corrected.at(-1),
        distance_cm: distCm,
        motion: 0.1 * Math.sin(t / 5) + 0.05,
        human: true,
        num_targets: 1,
        primary_x: x,
        primary_y: y,
        primary_dop_speed_cms: 1.47,
        spatial_source: 'target_info',
        spatial_age_ms: 40,
        target_info_ok: true,
        pqi_heart: 0.82 + 0.05 * Math.sin(t / 10),
        pqi_breath: 0.88,
        trusted_hr_fresh: true,
        trusted_rr_fresh: true,
        session_phase: phaseCode,
        session_phase_name: phaseName,
        hr_locked_live: phaseName === 'LOCKED' ? 1 : 0,
        hr_confidence: Math.max(0.05, Math.min(0.95, 0.78 + 0.08 * Math.sin(t / 7) - (inMotion ? 0.35 : 0))),
        hr_confidence_source_name: 'AUTO_PHASE',
        logged_hr_valid: inMotion ? 0 : 1,
        logged_rr_valid: 1,
        doppler_motion: inMotion,
        loop_dt_mean_ms: 3.8 + 0.4 * Math.sin(t / 12),
        loop_dt_max_ms: 9 + Math.round(2 * Math.sin(t / 9)),
        heap_free_kb: 182.5,
        heap_min_free_kb: 164.2,
        radar_uart_overflow_count: 0,
        radar_crc_err_count: 0,
        i2c_recover_count: Math.floor(t / 180),
        lcd_reinit_count: 0,
        wdt_near_miss_count: 0,
        cmd_rx_count: 0,
        cmd_err_count: 0,
        fw_uptime_s: t,
      },
      ble: {
        address: this.state.setup().ble_address,
        profile: this.state.setup().ble_profile,
        hr: ble_hr.at(-1),
        rr: ble_rr.at(-1),
        connected: true,
      },
      faults: [],
      events: [
        { ts: `T+${t}s`, message: 'Publish pipeline healthy' },
        { ts: `T+${Math.max(0, t - 1)}s`, message: 'Target lock maintained' },
      ],
      series: {
        ts,
        hr: rep_hr,
        rr: rep_rr,
        reported_hr: rep_hr,
        reported_rr: rep_rr,
        candidate_hr: can_hr,
        candidate_rr: can_rr,
        raw_hr,
        raw_hr_uncorrected: raw_hr,
        raw_hr_corrected,
        ble_hr,
        ble_rr,
        breath,
        heart,
        breath_phase: breath,
        heart_phase: heart,
      },
      analysis: {
        schema_hash: 'demo-v16-material',
        schema_invalid_count: 0,
        schema_warning_count: 0,
        reconnect_attempts: 0,
        funnel_survival_pct: 100,
        fw_truthfulness: { version: 'v16.5.10-demo', module_version_valid: true },
        gate_audit: { hr_eval_bins: 120, rr_eval_bins: 120 },
        hr_gate_reason_histogram: { OK: 120 },
        rr_gate_reason_histogram: { OK: 120 },
        agc_anomaly_flags: { gain_floor_pct: 0, near_field_pct: 0, skipdsp_pct: 0 },
        ble_ref_quality: { status: 'good', raw_packets: 120, parsed_rows: 120, packet_loss_pct: 0, decode_error_pct: 0 },
        ml_readiness_verdict: { verdict: 'ready', headline: 'Demo telemetry — simulated data only' },
      },
    };
    this.applyLivePayload(payload);
  }

  private applyLivePayload(payload: Partial<LivePayload>): boolean {
    if (!this.hasLivePayloadShape(payload)) return false;
    const receivedAt = Date.now();
    const normalized: LivePayload = {
      meta: payload.meta || {},
      radar: payload.radar || {},
      ble: payload.ble || {},
      thresholds: payload.thresholds || {},
      faults: Array.isArray(payload.faults) ? payload.faults : [],
      events: Array.isArray(payload.events) ? payload.events : [],
      series: this.boundSeries(payload.series),
      analysis: payload.analysis || null,
    };
    normalized.meta.received_at_ms = receivedAt;
    normalized.meta.stale = false;
    this.state.lastPayload.set(normalized);
    this.state.lastLivePayload.set(normalized);
    this.state.liveReceivedAt.set(receivedAt);
    this.state.telemetryStale.set(false);
    const payloadStatus = String(normalized.meta['status'] || '').toLowerCase();
    const payloadSessionId = String(normalized.meta['session_id'] || normalized.meta['active_session_id'] || '');
    const inactiveStatus = ['idle', 'waiting', 'stopped', 'complete', 'completed', 'analysis complete'].includes(payloadStatus)
      || payloadStatus.startsWith('stopped ');
    if (inactiveStatus || normalized.meta['active'] === false) {
      this.state.sessionActive.set(false);
      this.state.currentSessionId.set(null);
    } else if (payloadSessionId && ['running', 'active', 'recording'].includes(payloadStatus)) {
      this.state.sessionActive.set(true);
      this.state.currentSessionId.set(payloadSessionId);
    }
    this.clearStaleTimer();
    this.staleTimer = setTimeout(() => this.state.telemetryStale.set(true), 3500);

    const thresholds = this.state.kpiThresholds();
    const hr = this.validVitalNumber(normalized.radar.reported_hr);
    const rr = this.validVitalNumber(normalized.radar.reported_rr);
    if (hr !== null && (hr < thresholds.hrLow || hr > thresholds.hrHigh)) {
      this.emitAlert(`Heart rate ${Math.round(hr)} bpm outside ${thresholds.hrLow}-${thresholds.hrHigh} bpm`, 'warn', 'heart-rate');
    }
    if (rr !== null && (rr < thresholds.rrLow || rr > thresholds.rrHigh)) {
      this.emitAlert(`Respiration ${Math.round(rr)} br/min outside ${thresholds.rrLow}-${thresholds.rrHigh} br/min`, 'warn', 'respiration');
    }
    normalized.faults.forEach((fault) => {
      const record = typeof fault === 'object' ? fault : null;
      const message = typeof fault === 'string' ? fault : record?.['message'] || record?.['msg'] || record?.['code'];
      if (message) this.emitAlert(String(message), 'critical', 'fault');
    });

    this.state.spark.update((s) => {
      const hrSeries = this.appendFinite(s.hr, normalized.radar.reported_hr);
      const rrSeries = this.appendFinite(s.rr, normalized.radar.reported_rr);
      const fpsSeries = this.appendFinite(s.fps, this.resolveFps(normalized));
      const distSeries = this.appendFinite(s.dist, normalized.radar.distance_cm);
      return { hr: hrSeries, rr: rrSeries, fps: fpsSeries, dist: distSeries };
    });
    return true;
  }

  private hasLivePayloadShape(payload: Partial<LivePayload> | null | undefined): payload is Partial<LivePayload> {
    if (!payload || typeof payload !== 'object') return false;
    return Boolean(
      payload.meta ||
      payload.radar ||
      payload.ble ||
      payload.thresholds ||
      payload.series ||
      payload.analysis ||
      payload.faults ||
      payload.events
    );
  }

  private appendFinite(series: number[], value: unknown): number[] {
    const numeric = Number(value);
    return Number.isFinite(numeric) ? [...series, numeric].slice(-20) : series.slice(-20);
  }

  /**
   * Clamp every telemetry series array to the configured `maxChartPoints` so a
   * long-running session cannot grow the waveform/trend arrays without bound
   * (the setting was previously declared but never enforced). Non-array fields
   * (e.g. timestamps metadata) pass through unchanged.
   */
  private boundSeries(raw: unknown): LivePayload['series'] {
    const max = Math.max(1, Math.floor(this.state.maxChartPoints() || 3600));
    const src = raw && typeof raw === 'object' ? (raw as Record<string, unknown>) : {};
    const out: Record<string, unknown> = {};
    for (const [key, value] of Object.entries(src)) {
      out[key] = Array.isArray(value) && value.length > max ? value.slice(-max) : value;
    }
    return out as LivePayload['series'];
  }

  private resolveFps(payload: LivePayload): number | null {
    const directFps = payload.radar['fps_hz'] ?? payload.radar['fps'];
    if (directFps !== undefined && directFps !== null) {
      const numeric = Number(directFps);
      if (Number.isFinite(numeric)) return numeric;
    }
    const series = payload.series as Record<string, unknown> | undefined;
    const timestamps = (series?.['ts'] ?? series?.['t']) as number[] | undefined;
    if (!Array.isArray(timestamps) || timestamps.length < 3) return null;
    const numTimestamps = timestamps.map(t => Number(t)).filter(Number.isFinite);
    const deltas = numTimestamps.slice(1).map((val, idx) => val - numTimestamps[idx]).filter(val => val > 0);
    if (!deltas.length) return null;
    deltas.sort((a, b) => a - b);
    const median = deltas[Math.floor(deltas.length / 2)];
    return median > 0 ? 1 / median : null;
  }

  private validVitalNumber(value: unknown): number | null {
    const numeric = Number(value);
    return Number.isFinite(numeric) && numeric > 0 ? numeric : null;
  }

  private emitAlert(message: string, severity: 'warn' | 'critical', source = 'telemetry'): void {
    const now = Date.now();
    const key = `${source}:${message}`;
    const cooldownUntil = this.alertCooldownUntil.get(key) || 0;
    if (cooldownUntil > now) return;
    this.alertCooldownUntil.set(key, now + 60_000);
    const before = this.state.alertHistory().length;
    this.state.pushAlert(message, severity, source, now);
    if (this.state.alertHistory().length === before) return;
    this.audio.playAlertBeep(severity === 'critical' ? 'bad' : 'warn');
    this.audio.speakAlert(message, severity === 'critical' ? 'bad' : 'warn');
  }
}
