import { Injectable, inject } from '@angular/core';
import { AlertEvent } from '../models/rvt.models';
import { StateService } from './state.service';

@Injectable({
  providedIn: 'root'
})
export class AudioService {
  private state = inject(StateService);

  private audioCtx: AudioContext | null = null;
  private lastSpokenText = '';
  private lastSpokenAt = 0;
  private lastEndAt = 0;
  private queuedUtterance: SpeechSynthesisUtterance | null = null;
  private queueTimer: any = null;
  private isDuckingActive = false;

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
      let targetGain = (type === 'bad' ? 0.08 : 0.055) * volume;

      // Apply 20% ducking if voice alerts are active
      if (this.isDuckingActive) {
        targetGain *= 0.2;
      }

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

  private duck() {
    this.isDuckingActive = true;
  }

  private restore() {
    this.isDuckingActive = false;
  }

  private emitUtterance(u: SpeechSynthesisUtterance, severity: 'ok' | 'warn' | 'bad') {
    this.queuedUtterance = null;
    if (this.queueTimer) {
      clearTimeout(this.queueTimer);
      this.queueTimer = null;
    }

    this.duck();
    window.speechSynthesis.cancel();

    const prevOnEnd = u.onend;
    const prevOnError = u.onerror;

    u.onend = (ev) => {
      this.lastEndAt = Date.now();
      this.restore();
      if (typeof prevOnEnd === 'function') {
        try { prevOnEnd.call(u, ev); } catch (_) {}
      }
    };

    u.onerror = (ev) => {
      this.lastEndAt = Date.now();
      this.restore();
      if (typeof prevOnError === 'function') {
        try { prevOnError.call(u, ev); } catch (_) {}
      }
    };

    this.lastSpokenText = u.text;
    this.lastSpokenAt = Date.now();
    window.speechSynthesis.speak(u);
  }

  speakAlert(text: string, severity: 'ok' | 'warn' | 'bad' = 'warn', force = false) {
    if (typeof window === 'undefined' || !('speechSynthesis' in window)) return;
    if (!this.state.voiceAlertsEnabled() && !force) return;

    const cleanText = (text || '').trim();
    if (!cleanText) return;

    const now = Date.now();
    const debounceMs = severity === 'bad' ? 1000 : (severity === 'warn' ? 3000 : 1000);

    if (!force && this.lastSpokenText === cleanText && now - this.lastSpokenAt < debounceMs) {
      return;
    }

    try {
      const u = new SpeechSynthesisUtterance(cleanText.slice(0, 160));
      u.rate = 0.95;
      u.pitch = severity === 'bad' ? 0.85 : 1;
      u.volume = Math.max(0.2, Math.min(1, this.state.audioVolume()));

      if (force || severity === 'bad') {
        this.emitUtterance(u, severity);
        return;
      }

      const wait = Math.max(0, 1000 - (now - this.lastEndAt));
      if (wait > 0) {
        if (this.queueTimer) clearTimeout(this.queueTimer);
        this.queuedUtterance = u;
        this.queueTimer = setTimeout(() => {
          if (this.queuedUtterance === u) {
            this.emitUtterance(u, severity);
          }
        }, wait);
        return;
      }

      this.emitUtterance(u, severity);
    } catch (_) {}
  }

  announceAlerts(items: AlertEvent[]) {
    if (!this.state.voiceAlertsEnabled() || (typeof document !== 'undefined' && document.hidden)) return;
    const top = items[0];
    if (!top) return;
    this.speakAlert(`Alert: ${top.msg}`, top.severity === 'critical' ? 'bad' : 'warn');
  }
}
