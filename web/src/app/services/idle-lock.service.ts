import { Injectable, computed, effect, signal } from '@angular/core';

export type IdleLockTimeoutMinutes = 0 | 5 | 15 | 30 | 60 | 120;

const STORAGE_KEY = 'rvt-idle-lock-timeout-minutes';
const DEFAULT_TIMEOUT: IdleLockTimeoutMinutes = 30;

@Injectable({
  providedIn: 'root'
})
export class IdleLockService {
  readonly timeoutMinutes = signal<IdleLockTimeoutMinutes>(this.readStoredTimeout());
  readonly locked = signal(false);
  readonly lastActivityAt = signal(Date.now());
  readonly enabled = computed(() => this.timeoutMinutes() > 0);

  private timer: number | null = null;

  constructor() {
    if (typeof window !== 'undefined') {
      this.installActivityListeners();
      this.scheduleCheck();
    }

    effect(() => {
      localStorage.setItem(STORAGE_KEY, String(this.timeoutMinutes()));
      this.scheduleCheck();
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
      this.scheduleCheck();
    }
  }

  setTimeoutMinutes(value: number): void {
    const allowed: IdleLockTimeoutMinutes[] = [0, 5, 15, 30, 60, 120];
    this.timeoutMinutes.set(allowed.includes(value as IdleLockTimeoutMinutes) ? value as IdleLockTimeoutMinutes : DEFAULT_TIMEOUT);
  }

  private installActivityListeners(): void {
    const handler = () => this.markActivity();
    for (const eventName of ['pointerdown', 'keydown', 'touchstart']) {
      window.addEventListener(eventName, handler, { passive: true });
    }
  }

  private scheduleCheck(): void {
    if (this.timer !== null) window.clearTimeout(this.timer);
    if (!this.enabled() || this.locked()) return;
    const timeoutMs = this.timeoutMinutes() * 60_000;
    const elapsed = Date.now() - this.lastActivityAt();
    const remaining = Math.max(500, timeoutMs - elapsed);
    this.timer = window.setTimeout(() => {
      if (this.enabled() && Date.now() - this.lastActivityAt() >= timeoutMs) {
        this.lock();
      } else {
        this.scheduleCheck();
      }
    }, remaining);
  }

  private readStoredTimeout(): IdleLockTimeoutMinutes {
    const value = Number(localStorage.getItem(STORAGE_KEY));
    return [0, 5, 15, 30, 60, 120].includes(value) ? value as IdleLockTimeoutMinutes : DEFAULT_TIMEOUT;
  }
}
