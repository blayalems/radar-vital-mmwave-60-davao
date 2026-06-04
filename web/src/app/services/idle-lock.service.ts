import { DestroyRef, Injectable, computed, effect, inject, signal } from '@angular/core';

import { IDLE_LOCK_TIMEOUT_KEY } from './rvt-storage-keys';

export type IdleLockTimeoutMinutes = 0 | 5 | 15 | 30 | 60 | 120;

const DEFAULT_TIMEOUT: IdleLockTimeoutMinutes = 30;

@Injectable({
  providedIn: 'root'
})
export class IdleLockService {
  private readonly destroyRef = inject(DestroyRef);

  readonly timeoutMinutes = signal<IdleLockTimeoutMinutes>(this.readStoredTimeout());
  readonly locked = signal(false);
  readonly lastActivityAt = signal(Date.now());
  readonly enabled = computed(() => this.timeoutMinutes() > 0);

  private timer: number | null = null;
  private lastScheduleAt = 0;
  private readonly scheduleDebounceMs = 250;

  constructor() {
    if (typeof window !== 'undefined') {
      this.destroyRef.onDestroy(this.installActivityListeners());
      this.scheduleCheck(true);
    }

    effect(() => {
      if (typeof window === 'undefined') return;
      try {
        window.localStorage.setItem(IDLE_LOCK_TIMEOUT_KEY, String(this.timeoutMinutes()));
      } catch (_) {
        // Storage can be unavailable in private mode or restricted WebViews.
      }
      this.scheduleCheck(true);
    });
  }

  lock(): void {
    this.locked.set(true);
  }

  unlock(): void {
    this.locked.set(false);
    this.markActivity();
  }

  markActivity(): void {
    if (!this.locked()) {
      this.lastActivityAt.set(Date.now());
      this.scheduleCheck(false);
    }
  }

  setTimeoutMinutes(value: number): void {
    const allowed: IdleLockTimeoutMinutes[] = [0, 5, 15, 30, 60, 120];
    this.timeoutMinutes.set(allowed.includes(value as IdleLockTimeoutMinutes) ? value as IdleLockTimeoutMinutes : DEFAULT_TIMEOUT);
  }

  private installActivityListeners(): () => void {
    const handler = () => this.markActivity();
    for (const eventName of ['pointerdown', 'keydown', 'touchstart']) {
      window.addEventListener(eventName, handler, { passive: true });
    }
    return () => {
      for (const eventName of ['pointerdown', 'keydown', 'touchstart']) {
        window.removeEventListener(eventName, handler);
      }
    };
  }

  private scheduleCheck(force: boolean): void {
    if (typeof window === 'undefined') return;
    const now = Date.now();
    if (!force && this.timer !== null && now - this.lastScheduleAt < this.scheduleDebounceMs) return;
    this.lastScheduleAt = now;
    if (this.timer !== null) {
      window.clearTimeout(this.timer);
      this.timer = null;
    }
    if (!this.enabled() || this.locked()) return;
    const timeoutMs = this.timeoutMinutes() * 60_000;
    const elapsed = Date.now() - this.lastActivityAt();
    const remaining = Math.max(500, timeoutMs - elapsed);
    this.timer = window.setTimeout(() => {
      if (this.enabled() && Date.now() - this.lastActivityAt() >= timeoutMs) {
        this.lock();
      } else {
        this.scheduleCheck(true);
      }
    }, remaining);
  }

  private readStoredTimeout(): IdleLockTimeoutMinutes {
    if (typeof window === 'undefined' || typeof window.localStorage === 'undefined') return DEFAULT_TIMEOUT;
    try {
      const value = Number(window.localStorage.getItem(IDLE_LOCK_TIMEOUT_KEY));
      return [0, 5, 15, 30, 60, 120].includes(value) ? value as IdleLockTimeoutMinutes : DEFAULT_TIMEOUT;
    } catch (_) {
      return DEFAULT_TIMEOUT;
    }
  }
}
