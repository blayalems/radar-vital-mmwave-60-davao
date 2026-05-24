import { Injectable, effect, inject } from '@angular/core';
import { LivePayload } from '../models/rvt.models';
import { AudioService } from './audio.service';
import { StateService } from './state.service';
import { ApiService } from './api.service';

@Injectable({
  providedIn: 'root'
})
export class TelemetryService {
  private state = inject(StateService);
  private api = inject(ApiService);
  private audio = inject(AudioService);

  private pollTimer: ReturnType<typeof setTimeout> | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private staleTimer: ReturnType<typeof setTimeout> | null = null;
  private sseReconnectAttempts = 0;
  private running = false;
  private sse: EventSource | null = null;
  private sseMode = false;
  private sseErrors: number[] = [];
  private httpPollFailures = 0;
  private demoT = 0;

  constructor() {
    this.start();
    effect(() => {
      const simulating = this.state.demoMode() || this.state.autoDemoActive();
      if (simulating) {
        this.stopSse();
        this.clearReconnectTimer();
        this.scheduleNextPoll(0);
      } else if (this.running && !this.sseMode) {
        void this.api.detectControlMode().then(() => {
          this.scheduleNextPoll(0);
          this.startSse();
        });
      }
    });
  }

  start() {
    this.running = true;
    this.scheduleNextPoll(1000);
    this.startSse();
  }

  stop() {
    this.running = false;
    this.clearPollTimer();
    this.stopSse();
    this.clearReconnectTimer();
    if (this.staleTimer) clearTimeout(this.staleTimer);
  }

  private clearPollTimer() {
    if (this.pollTimer) {
      clearTimeout(this.pollTimer);
      this.pollTimer = null;
    }
  }

  private clearReconnectTimer() {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  private scheduleNextPoll(delayMs: number) {
    if (!this.running) return;
    this.clearPollTimer();
    this.pollTimer = setTimeout(() => this.poll(), delayMs);
  }

  private async poll() {
    if (!this.running) return;

    if (this.state.paused()) {
      this.scheduleNextPoll(this.state.liveBufferSeconds() * 1000);
      return;
    }

    if (this.state.demoMode() || this.state.autoDemoActive()) {
      this.runSimulationStep();
      this.scheduleNextPoll(1000);
      return;
    }

    // If SSE is active, standard HTTP poll is stopped entirely (no-op treadmill eliminated)
    if (this.sseMode) {
      return;
    }

    this.state.ctlStatus.set(this.state.ctlStatus() || { ok: true, mode: 'loading' });

    try {
      const path = '/api/session/current/live_dashboard.json';
      const startMs = Date.now();
      const payload = await this.api.request<Partial<LivePayload>>(`${path}?t=${Date.now()}`);
      
      const latency = Date.now() - startMs;
      this.applyLivePayload(payload);
      
      this.httpPollFailures = 0;
      this.state.ctlStatus.update(s => ({ ...(s ?? { ok: true }), ok: true, latency }));
      this.scheduleNextPoll(1000);
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'poll failed';
      console.warn('Telemetry poll failed', error);
      this.state.telemetryStale.set(true);
      this.state.ctlStatus.update(s => ({ ...(s ?? { ok: false }), ok: false, error: message }));
      this.emitAlert(`Live connection unavailable: ${message}`, 'critical');
      
      if (this.state.autoDemoOnDisconnect()) {
        this.state.autoDemoActive.set(true);
        this.scheduleNextPoll(0);
      } else {
        this.httpPollFailures++;
        const baseDelay = Math.min(60_000, 3_000 * (2 ** Math.min(this.httpPollFailures - 1, 4)));
        const jitter = Math.floor(Math.random() * 1000) - 500;
        this.scheduleNextPoll(Math.max(1000, baseDelay + jitter));
      }
    }
  }

  private startSse() {
    if (typeof EventSource === 'undefined') return;
    if (this.state.demoMode() || this.state.autoDemoActive()) return;
    if (!this.running) return;
    // EventSource cannot attach the LAN pairing header; authenticated links use polling.
    if (this.api.hasPairToken()) {
      this.scheduleNextPoll(0);
      return;
    }

    try {
      const base = this.api.currentApiBase();
      this.sse = new EventSource(`${base}/api/events/subscribe`);
      
      this.sse.onopen = () => {
        console.log('SSE connection successfully opened.');
        this.sseMode = true;
        this.sseErrors = [];
        this.sseReconnectAttempts = 0;
        this.clearPollTimer(); // Stop the polling no-op treadmill
        this.clearReconnectTimer();
      };

      this.sse.addEventListener('live', (ev: MessageEvent) => {
        try {
          const raw = JSON.parse(ev.data || '{}');
          this.applyLivePayload(raw);
        } catch (e) {
          console.warn('SSE live parse failed', e);
        }
      });

      this.sse.onerror = () => {
        const now = Date.now();
        this.sseErrors = this.sseErrors.filter(t => now - t < 60000);
        this.sseErrors.push(now);
        
        if (this.sseErrors.length > 3) {
          console.warn('SSE failure threshold reached. Falling back to polling.');
          this.stopSse();
          this.scheduleNextPoll(0); // Fallback to HTTP polling
          this.scheduleSseReconnect();
        }
      };
    } catch (e) {
      console.warn('SSE connection failed', e);
      this.scheduleSseReconnect();
    }
  }

  private scheduleSseReconnect() {
    if (!this.running) return;
    this.clearReconnectTimer();

    // Bounded backoff: 15s, 30s, 60s max
    const backoffSeconds = this.sseReconnectAttempts === 0 ? 15
                         : this.sseReconnectAttempts === 1 ? 30
                         : 60;
    
    // Random jitter +/- 1s to avoid synchronizations
    const jitterMs = Math.floor(Math.random() * 2000) - 1000;
    const delayMs = Math.max(1000, (backoffSeconds * 1000) + jitterMs);

    console.log(`SSE reconnect scheduled in ${delayMs}ms (attempt ${this.sseReconnectAttempts + 1})`);

    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      if (!this.running) return;
      this.sseReconnectAttempts++;
      this.startSse();
    }, delayMs);
  }

  private stopSse() {
    if (this.sse) {
      try {
        this.sse.close();
      } catch (_) {}
      this.sse = null;
    }
    this.sseMode = false;
  }

  private runSimulationStep() {
    this.demoT += 1;
    const t = this.demoT;
    const N = 120;
    const ts = Array.from({ length: N }, (_, i) => t - N + i);
    const hrBase = 72 + 4 * Math.sin(t / 12);
    const rrBase = 15 + 1.5 * Math.sin(t / 18 + 1);

    const rep_hr = ts.map((_, i) => hrBase + 2 * Math.sin(i / 3 + t / 5) + (Math.random() - .5));
    const can_hr = rep_hr.map(v => v + (Math.random() - .5) * 1.2);
    const raw_hr = rep_hr.map(v => v + (Math.random() - .5) * 3.5 + 1.2);
    const raw_hr_corrected = raw_hr.map(v => v - Math.max(0, Math.min(25, 0.4 * (v - 55))));
    const ble_hr = rep_hr.map(v => v + (Math.random() - .5) * 0.7 - 0.3);

    const rep_rr = ts.map((_, i) => rrBase + 0.4 * Math.sin(i / 4 + t / 6) + (Math.random() - .5) * .15);
    const can_rr = rep_rr.map(v => v + (Math.random() - .5) * .3);
    const ble_rr = rep_rr.map(v => v + (Math.random() - .5) * .2);

    const breath = ts.map((_, i) => Math.sin((i + t) * 0.12) + 0.1 * Math.sin((i + t) * 0.5));
    const heart = ts.map((_, i) => Math.sin((i + t) * 0.9) * 0.6 + 0.15 * Math.sin((i + t) * 3.5));

    const x = 0.35 + 0.18 * Math.sin(t / 14);
    const y = 1.85 + 0.22 * Math.cos(t / 16);
    const distCm = Math.hypot(x, y) * 100;

    const payload = {
      meta: {
        status: this.state.currentSessionId() ? 'running' : 'waiting',
        elapsed_s: t,
        // M7 dynamically calculated remaining session duration based on setup.duration_s
        remaining_s: Math.max(0, (this.state.setup().duration_s || 30) - t),
        session_dir: '/sessions/demo_2026_04_18',
        sandbox: true
      },
      radar: {
        rows: 1450 + t,
        fps_hz: 20,
        reported_hr: rep_hr[rep_hr.length - 1],
        reported_rr: rep_rr[rep_rr.length - 1],
        candidate_hr: can_hr[can_hr.length - 1],
        candidate_rr: can_rr[can_rr.length - 1],
        raw_hr: raw_hr[raw_hr.length - 1],
        raw_hr_uncorrected: raw_hr[raw_hr.length - 1],
        raw_hr_corrected: raw_hr_corrected[raw_hr_corrected.length - 1],
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
        hr_arbiter_corrected: false,
        hr_rejectphase_rejected: false,
        trusted_hr_fresh: true,
        hr_agree_err_bpm: 0.4,
        hr_zc_bpm: hrBase,
        hr_zc_conf: 0.7,
        hr_spec_bpm: hrBase + 0.3,
        hr_spec_mag: 0.0032,
        hr_trusted_anchor_value: hrBase,
        hr_anchor_err_bpm: 0.4,
        hr_publish_reason_name: 'published',
        hr_pre_rejectphase: hrBase + 0.7,
        hr_post_rejectphase: hrBase + 0.4,
        hr_post_blend: hrBase + 0.2,
        hr_post_coherence: hrBase,
        hr_final_publish_candidate: rep_hr[rep_hr.length - 1],
        hr_raw_high_bias_suspect: false,
        rr_anchor_fresh: true,
        rr_raw_agree_ok: true,
        rr_fundamental_recovery_triggered: false,
        rr_gate_reason_name: 'OK',
        rr_publish_reason_name: 'published',
        rr_zc_bpm: rrBase,
        rr_zc_conf: 0.82,
        rr_spec_bpm: rrBase + 0.1,
        rr_spec_conf: 0.81,
        rr_raw_anchor_err_bpm: 0.12,
        rr_pre_acceptphase: rrBase + 0.2,
        rr_post_acceptphase: rrBase + 0.1,
        rr_post_blend: rrBase,
        rr_post_kalman: rrBase,
        rr_final_publish_candidate: rep_rr[rep_rr.length - 1],
        rr_anchor_confidence: 0.83,
        rr_fundamental_recovery_count: 0,
        rr_raw_seed_consistent_count: 5,
        rr_midsession_raw_reanchor_allowed: true,
        trusted_rr_fresh: true
      },
      ble: {
        address: this.state.setup().ble_address,
        profile: this.state.setup().ble_profile,
        hr: ble_hr[ble_hr.length - 1],
        rr: ble_rr[ble_rr.length - 1],
        connected: true
      },
      faults: [],
      events: [
        { ts: `T+${t}s`, message: 'Publish pipeline healthy' },
        { ts: `T+${Math.max(0, t - 1)}s`, message: 'Target lock maintained' }
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
        heart_phase: heart
      },
      analysis: {
        schema_hash: 'demo-v16-material',
        schema_invalid_count: 0,
        schema_warning_count: 0,
        reconnect_attempts: 0,
        funnel_survival_pct: 100,
        fw_truthfulness: { version: 'v16.0.0-demo', module_version_valid: true },
        gate_audit: { hr_eval_bins: 120, rr_eval_bins: 120 },
        hr_gate_reason_histogram: { OK: 120 },
        rr_gate_reason_histogram: { OK: 120 },
        agc_anomaly_flags: { gain_floor_pct: 0, near_field_pct: 0, skipdsp_pct: 0 },
        ble_ref_quality: { status: 'good', raw_packets: 120, parsed_rows: 120, packet_loss_pct: 0, decode_error_pct: 0 }
      }
    };

    this.applyLivePayload(payload);
  }

  private applyLivePayload(payload: Partial<LivePayload>) {
    if (!payload) return;

    const receivedAt = Date.now();
    const normalized: LivePayload = {
      meta: payload.meta || {},
      radar: payload.radar || {},
      ble: payload.ble || {},
      thresholds: payload.thresholds || {},
      faults: Array.isArray(payload.faults) ? payload.faults : [],
      events: Array.isArray(payload.events) ? payload.events : [],
      series: payload.series || {},
      analysis: payload.analysis || null
    };
    normalized.meta.received_at_ms = receivedAt;
    normalized.meta.stale = false;

    this.state.lastPayload.set(normalized);
    this.state.lastLivePayload.set(normalized);
    this.state.liveReceivedAt.set(receivedAt);
    this.state.telemetryStale.set(false);
    if (this.staleTimer) clearTimeout(this.staleTimer);
    this.staleTimer = setTimeout(() => this.state.telemetryStale.set(true), 3500);

    const thresholds = this.state.kpiThresholds();
    const hr = Number(normalized.radar.reported_hr);
    const rr = Number(normalized.radar.reported_rr);
    if (Number.isFinite(hr) && (hr < thresholds.hrLow || hr > thresholds.hrHigh)) {
      this.emitAlert(`Heart rate ${Math.round(hr)} bpm outside ${thresholds.hrLow}-${thresholds.hrHigh} bpm`, 'warn', 'heart-rate');
    }
    if (Number.isFinite(rr) && (rr < thresholds.rrLow || rr > thresholds.rrHigh)) {
      this.emitAlert(`Respiration ${Math.round(rr)} br/min outside ${thresholds.rrLow}-${thresholds.rrHigh} br/min`, 'warn', 'respiration');
    }
    normalized.faults.forEach(fault => {
      const record = typeof fault === 'object' ? fault : null;
      const message = typeof fault === 'string'
        ? fault
        : record?.['message'] || record?.['msg'] || record?.['code'];
      if (message) this.emitAlert(String(message), 'critical', 'fault');
    });

    // Update real-time spark data
    this.state.spark.update(s => {
      const hr = [...s.hr, normalized.radar.reported_hr || 0].slice(-20);
      const rr = [...s.rr, normalized.radar.reported_rr || 0].slice(-20);
      const fps = [...s.fps, normalized.radar.rows || 0].slice(-20);
      const dist = [...s.dist, normalized.radar.distance_cm || 0].slice(-20);
      return { hr, rr, fps, dist };
    });
  }

  private emitAlert(message: string, severity: 'warn' | 'critical', source = 'telemetry'): void {
    const before = this.state.alertHistory().length;
    this.state.pushAlert(message, severity, source, Date.now());
    if (this.state.alertHistory().length === before) return;
    this.audio.playAlertBeep(severity === 'critical' ? 'bad' : 'warn');
    this.audio.speakAlert(message, severity === 'critical' ? 'bad' : 'warn');
  }
}
