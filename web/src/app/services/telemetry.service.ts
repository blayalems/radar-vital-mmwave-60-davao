import { Injectable, inject } from '@angular/core';
import { StateService } from './state.service';
import { ApiService } from './api.service';

@Injectable({
  providedIn: 'root'
})
export class TelemetryService {
  private state = inject(StateService);
  private api = inject(ApiService);

  private pollTimer: any = null;
  private sse: EventSource | null = null;
  private sseMode = false;
  private sseErrors: number[] = [];
  private demoT = 0;

  constructor() {
    this.start();
  }

  start() {
    this.scheduleNextPoll(1000);
    this.startSse();
  }

  stop() {
    if (this.pollTimer) {
      clearTimeout(this.pollTimer);
      this.pollTimer = null;
    }
    this.stopSse();
  }

  private scheduleNextPoll(delayMs: number) {
    if (this.pollTimer) clearTimeout(this.pollTimer);
    this.pollTimer = setTimeout(() => this.poll(), delayMs);
  }

  private async poll() {
    if (this.state.paused()) {
      this.scheduleNextPoll(this.state.liveBufferSeconds() * 1000);
      return;
    }

    if (this.sseMode || this.state.autoDemoActive()) {
      // If we are in SSE or Auto Demo, standard HTTP poll is handled differently or bypassed
      if (this.state.autoDemoActive()) {
        this.runSimulationStep();
      }
      this.scheduleNextPoll(1000);
      return;
    }

    this.state.ctlStatus.set(this.state.ctlStatus() || { ok: true, mode: 'loading' });

    try {
      const path = '/api/session/current/live_dashboard.json';
      const startMs = Date.now();
      const payload = await this.api.request(`${path}?t=${Date.now()}`);
      
      const latency = Date.now() - startMs;
      this.applyLivePayload(payload);
      
      this.state.ctlStatus.update(s => ({ ...s, ok: true, latency }));
      this.scheduleNextPoll(1000);
    } catch (e: any) {
      console.warn('Telemetry poll failed', e);
      this.state.ctlStatus.update(s => ({ ...s, ok: false, error: e.message }));
      
      if (this.state.autoDemoOnDisconnect()) {
        this.state.autoDemoActive.set(true);
      }
      this.scheduleNextPoll(3000); // Backoff on fail
    }
  }

  private startSse() {
    if (typeof EventSource === 'undefined') return;
    if (this.state.demoMode() || this.state.autoDemoActive()) return;

    try {
      const base = this.api.currentApiBase();
      this.sse = new EventSource(`${base}/api/events/subscribe`);
      
      this.sse.onopen = () => {
        this.sseMode = true;
        this.sseErrors = [];
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
          this.stopSse();
          this.scheduleNextPoll(0);
        }
      };
    } catch (e) {
      console.warn('SSE connection failed', e);
    }
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
        remaining_s: Math.max(0, 320 - t),
        session_dir: '/sessions/demo_2026_04_18',
        sandbox: true
      },
      radar: {
        rows: 1450 + t,
        reported_hr: rep_hr[rep_hr.length - 1],
        reported_rr: rep_rr[rep_rr.length - 1],
        candidate_hr: can_hr[can_hr.length - 1],
        candidate_rr: can_rr[can_rr.length - 1],
        raw_hr: raw_hr[raw_hr.length - 1],
        raw_hr_uncorrected: raw_hr[raw_hr.length - 1],
        raw_hr_corrected: raw_hr_corrected[raw_hr_corrected.length - 1],
        distance_cm: distCm,
        motion: 0.1 * Math.sin(t / 5) + 0.05,
        human: true
      },
      ble: {
        address: this.state.setup().ble_address,
        profile: this.state.setup().ble_profile,
        hr: ble_hr[ble_hr.length - 1],
        rr: ble_rr[ble_rr.length - 1],
        connected: true
      },
      faults: [],
      events: [],
      series: {
        ts,
        hr: rep_hr,
        rr: rep_rr,
        breath,
        heart
      }
    };

    this.applyLivePayload(payload);
  }

  private applyLivePayload(payload: any) {
    if (!payload) return;

    // Normalize
    const normalized = {
      meta: payload.meta || {},
      radar: payload.radar || {},
      ble: payload.ble || {},
      thresholds: payload.thresholds || {},
      faults: Array.isArray(payload.faults) ? payload.faults : [],
      events: Array.isArray(payload.events) ? payload.events : [],
      series: payload.series || {},
      analysis: payload.analysis || null
    };

    this.state.lastPayload.set(normalized);
    this.state.lastLivePayload.set(normalized);

    // Update real-time spark data
    this.state.spark.update(s => {
      const hr = [...s.hr, normalized.radar.reported_hr || 0].slice(-20);
      const rr = [...s.rr, normalized.radar.reported_rr || 0].slice(-20);
      const fps = [...s.fps, normalized.radar.rows || 0].slice(-20);
      const dist = [...s.dist, normalized.radar.distance_cm || 0].slice(-20);
      return { hr, rr, fps, dist };
    });
  }
}
