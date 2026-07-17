import { Injectable, OnDestroy } from '@angular/core';

interface ChartRenderJob {
  draw: () => void;
  visible: () => boolean;
  minIntervalMs: number;
}

@Injectable({
  providedIn: 'root'
})
export class ChartRenderSchedulerService implements OnDestroy {
  private readonly pending = new Map<object, ChartRenderJob>();
  private readonly lastDrawAt = new WeakMap<object, number>();
  private frameId: number | null = null;
  private wakeTimer: ReturnType<typeof setTimeout> | null = null;
  private readonly onVisibilityChange = () => {
    if (document.visibilityState !== 'hidden') this.queueFrame();
  };

  constructor() {
    document.addEventListener('visibilitychange', this.onVisibilityChange);
  }

  request(
    owner: object,
    draw: () => void,
    visible: () => boolean = () => true,
    minIntervalMs = 100
  ): void {
    this.pending.set(owner, {
      draw,
      visible,
      minIntervalMs: Math.max(0, minIntervalMs)
    });
    this.queueFrame();
  }

  cancel(owner: object): void {
    this.pending.delete(owner);
    this.lastDrawAt.delete(owner);
    if (this.pending.size === 0) this.clearWakeTimer();
  }

  canvasVisible(canvas: HTMLCanvasElement | null | undefined): boolean {
    if (!canvas || document.visibilityState === 'hidden' || !canvas.isConnected) return false;
    if (canvas.clientWidth <= 0 || canvas.clientHeight <= 0) return false;
    const style = getComputedStyle(canvas);
    return style.display !== 'none' && style.visibility !== 'hidden';
  }

  ngOnDestroy(): void {
    document.removeEventListener('visibilitychange', this.onVisibilityChange);
    if (this.frameId !== null) cancelAnimationFrame(this.frameId);
    this.frameId = null;
    this.clearWakeTimer();
    this.pending.clear();
  }

  private queueFrame(): void {
    if (
      this.frameId !== null
      || document.visibilityState === 'hidden'
      || this.pending.size === 0
    ) {
      return;
    }
    this.frameId = requestAnimationFrame(() => {
      this.frameId = null;
      this.flush();
    });
  }

  private flush(): void {
    if (document.visibilityState === 'hidden') return;
    const now = Date.now();
    let nextDelay = Number.POSITIVE_INFINITY;

    for (const [owner, job] of this.pending) {
      if (!job.visible()) continue;
      const elapsed = now - (this.lastDrawAt.get(owner) ?? Number.NEGATIVE_INFINITY);
      if (elapsed < job.minIntervalMs) {
        nextDelay = Math.min(nextDelay, job.minIntervalMs - elapsed);
        continue;
      }

      this.pending.delete(owner);
      this.lastDrawAt.set(owner, now);
      try {
        job.draw();
      } catch (error) {
        console.warn('Chart render failed', error);
      }
    }

    this.clearWakeTimer();
    if (this.pending.size > 0 && Number.isFinite(nextDelay)) {
      this.wakeTimer = setTimeout(() => {
        this.wakeTimer = null;
        this.queueFrame();
      }, Math.max(0, nextDelay));
    }
  }

  private clearWakeTimer(): void {
    if (!this.wakeTimer) return;
    clearTimeout(this.wakeTimer);
    this.wakeTimer = null;
  }
}
