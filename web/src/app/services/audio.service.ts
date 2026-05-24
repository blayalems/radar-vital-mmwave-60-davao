import { Injectable, inject } from '@angular/core';
import { AlertEvent } from '../models/rvt.models';
import { StateService } from './state.service';

@Injectable({
  providedIn: 'root'
})
export class AudioService {
  private state = inject(StateService);

  private audioCtx: AudioContext | null = null;
  private voiceLastKey = '';
  private voiceLastAt = 0;

  private getAudioCtx(): AudioContext | null {
    if (typeof window === 'undefined') return null;
    if (this.audioCtx) {
      if (this.audioCtx.state === 'suspended') {
        try {
          this.audioCtx.resume()?.catch(() => {});
        } catch (_) {}
      }
      return this.audioCtx;
    }
    try {
      const AudioCtxClass = window.AudioContext || (window as Window & { webkitAudioContext?: typeof AudioContext }).webkitAudioContext;
      if (!AudioCtxClass) return null;
      this.audioCtx = new AudioCtxClass();
      return this.audioCtx;
    } catch (_) {
      return null;
    }
  }

  playAlertBeep(type: 'ok' | 'warn' | 'bad' = 'warn') {
    if (!this.state.audioAlertsEnabled()) return;
    const ctx = this.getAudioCtx();
    if (!ctx) return;

    try {
      const now = ctx.currentTime;
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      const freq = type === 'bad' ? 220 : (type === 'ok' ? 660 : 440);

      osc.type = type === 'bad' ? 'sawtooth' : 'sine';
      osc.frequency.setValueAtTime(freq, now);
      gain.gain.setValueAtTime(0.0001, now);

      const volume = this.state.audioVolume();
      const targetGain = (type === 'bad' ? 0.08 : 0.055) * volume;

      if (targetGain <= 0) {
        gain.gain.setValueAtTime(0.0001, now);
      } else {
        gain.gain.exponentialRampToValueAtTime(targetGain, now + 0.018);
      }
      gain.gain.exponentialRampToValueAtTime(0.0001, now + 0.18);

      osc.connect(gain);
      gain.connect(ctx.destination);

      osc.start(now);
      osc.stop(now + 0.2);
    } catch (_) {}
  }

  speakAlert(text: string, severity: 'ok' | 'warn' | 'bad' = 'warn', force = false) {
    if (typeof window === 'undefined' || !('speechSynthesis' in window)) return;
    if (!this.state.voiceAlertsEnabled() && !force) return;

    const key = `${severity}:${text}`.slice(0, 180);
    const now = Date.now();
    if (!force && key === this.voiceLastKey && now - this.voiceLastAt < 12000) return;

    this.voiceLastKey = key;
    this.voiceLastAt = now;

    try {
      const u = new SpeechSynthesisUtterance(String(text || 'Alert').slice(0, 160));
      u.rate = 0.95;
      u.pitch = severity === 'bad' ? 0.85 : 1;
      u.volume = Math.max(0.2, Math.min(1, this.state.audioVolume()));

      window.speechSynthesis.cancel();
      window.speechSynthesis.speak(u);
    } catch (_) {}
  }

  announceAlerts(items: AlertEvent[]) {
    if (!this.state.voiceAlertsEnabled() || (typeof document !== 'undefined' && document.hidden)) return;
    const top = items[0];
    if (!top) return;
    this.speakAlert(`Alert: ${top.msg}`, top.severity === 'critical' ? 'bad' : 'warn');
  }
}
