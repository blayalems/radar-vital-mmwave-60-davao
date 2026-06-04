import { Injectable, signal, computed, NgZone, inject } from '@angular/core';

export type IdleLockReason = 'idle' | 'manual';

const STORAGE_ENABLED = 'rvt-idle-lock-enabled';
const STORAGE_TIMEOUT = 'rvt-idle-lock-timeout-min';

const DEFAULT_TIMEOUT_MIN = 10;
const MIN_TIMEOUT_MIN = 1;
const MAX_TIMEOUT_MIN = 120;

/**
 * Idle auto-lock for unattended operator stations.
 *
 * v11 dimmed and locked the console after a configurable idle window so a
 * walk-away didn't leave subject telemetry exposed on a shared station. The
 * Angular v12 dashboard regained that contract here: the service watches for
 * pointer / keyboard / touch activity, locks after the configured timeout, and
 * exposes a `locked` signal the shell renders as a focus-trapping overlay.
 *
 * The lock is a privacy curtain, not an authentication boundary — it never
 * touches the paired LAN token and unlocking is a local UI gesture. When idle
 * locking is disabled (the default) no listeners run and the feature is inert,
 * with manual lock (Ctrl+L) still available on demand.
 */
@Injectable({ providedIn: 'root' })
export class IdleLockService {
  private readonly zone = inject(NgZone);

  readonly enabled = signal<boolean>(false);
  /** Idle timeout in minutes before an automatic lock fires. */
  readonly timeoutMinutes = signal<number>(DEFAULT_TIMEOUT_MIN);
  readonly locked = signal<boolean>(false);
  readonly lockReason = signal<IdleLockReason>('idle');

  readonly timeoutMs = computed(() => this.timeoutMinutes() * 60_000);

  private timer: ReturnType<typeof setTimeout> | null = null;
  private listening = false;
  private readonly activityHandler = () => this.onActivity();
  private static readonly ACTIVITY_EVENTS = [
    'mousemove',
    'mousedown',
    'keydown',
    'touchstart',
    'wheel',
    'scroll'
  ] as const;

  constructor() {
    this.hydrate();
  }

  /** Begin honouring the configured idle policy. Safe to call repeatedly. */
  start(): void {
    this.refreshListeners();
    this.resetTimer();
  }

  /** Stop all listeners and pending timers (e.g. on teardown). */
  stop(): void {
    this.clearTimer();
    this.detachListeners();
  }

  setEnabled(value: boolean): void {
    this.enabled.set(value);
    localStorage.setItem(STORAGE_ENABLED, value ? '1' : '0');
    this.refreshListeners();
    if (value) {
      this.resetTimer();
    } else {
      this.clearTimer();
    }
  }

  setTimeoutMinutes(minutes: number): void {
    const clamped = Math.min(MAX_TIMEOUT_MIN, Math.max(MIN_TIMEOUT_MIN, Math.round(minutes)));
    this.timeoutMinutes.set(clamped);
    localStorage.setItem(STORAGE_TIMEOUT, String(clamped));
    if (this.enabled() && !this.locked()) {
      this.resetTimer();
    }
  }

  /** Lock immediately. `manual` skips the idle-policy gate (Ctrl+L always works). */
  lockNow(reason: IdleLockReason = 'manual'): void {
    if (this.locked()) return;
    this.lockReason.set(reason);
    this.locked.set(true);
    this.clearTimer();
  }

  /** Clear the lock curtain and restart the idle countdown. */
  unlock(): void {
    if (!this.locked()) return;
    this.locked.set(false);
    if (this.enabled()) {
      this.resetTimer();
    }
  }

  private onActivity(): void {
    if (this.locked()) return;
    this.resetTimer();
  }

  private resetTimer(): void {
    this.clearTimer();
    if (!this.enabled() || this.locked()) return;
    // Timers should not keep change detection churning; run outside Angular.
    this.zone.runOutsideAngular(() => {
      this.timer = setTimeout(() => {
        this.zone.run(() => this.lockNow('idle'));
      }, this.timeoutMs());
    });
  }

  private clearTimer(): void {
    if (this.timer !== null) {
      clearTimeout(this.timer);
      this.timer = null;
    }
  }

  private refreshListeners(): void {
    if (this.enabled()) {
      this.attachListeners();
    } else {
      this.detachListeners();
    }
  }

  private attachListeners(): void {
    if (this.listening || typeof window === 'undefined') return;
    this.zone.runOutsideAngular(() => {
      for (const evt of IdleLockService.ACTIVITY_EVENTS) {
        window.addEventListener(evt, this.activityHandler, { passive: true });
      }
    });
    this.listening = true;
  }

  private detachListeners(): void {
    if (!this.listening || typeof window === 'undefined') return;
    for (const evt of IdleLockService.ACTIVITY_EVENTS) {
      window.removeEventListener(evt, this.activityHandler);
    }
    this.listening = false;
  }

  private hydrate(): void {
    try {
      this.enabled.set(localStorage.getItem(STORAGE_ENABLED) === '1');
      const stored = Number(localStorage.getItem(STORAGE_TIMEOUT));
      if (Number.isFinite(stored) && stored >= MIN_TIMEOUT_MIN && stored <= MAX_TIMEOUT_MIN) {
        this.timeoutMinutes.set(stored);
      }
    } catch {
      // Defaults already applied; ignore storage access failures.
    }
  }
}
