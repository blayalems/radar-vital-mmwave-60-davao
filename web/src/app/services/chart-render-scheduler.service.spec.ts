import { TestBed } from '@angular/core/testing';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { ChartRenderSchedulerService } from './chart-render-scheduler.service';

describe('ChartRenderSchedulerService', () => {
  let scheduler: ChartRenderSchedulerService;

  beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(10_000);
    vi.stubGlobal('requestAnimationFrame', (callback: FrameRequestCallback) => (
      setTimeout(() => callback(Date.now()), 0) as unknown as number
    ));
    vi.stubGlobal('cancelAnimationFrame', (id: number) => clearTimeout(id));
    TestBed.configureTestingModule({ providers: [ChartRenderSchedulerService] });
    scheduler = TestBed.inject(ChartRenderSchedulerService);
  });

  afterEach(() => {
    scheduler.ngOnDestroy();
    vi.unstubAllGlobals();
    vi.useRealTimers();
  });

  it('coalesces requests by owner and keeps only the newest draw callback', () => {
    const owner = {};
    const first = vi.fn();
    const newest = vi.fn();

    scheduler.request(owner, first);
    scheduler.request(owner, newest);
    vi.runOnlyPendingTimers();

    expect(first).not.toHaveBeenCalled();
    expect(newest).toHaveBeenCalledTimes(1);
  });

  it('rate-limits a visible chart and runs its latest pending render', () => {
    const owner = {};
    const draw = vi.fn();

    scheduler.request(owner, draw, () => true, 100);
    vi.runOnlyPendingTimers();
    expect(draw).toHaveBeenCalledTimes(1);

    vi.advanceTimersByTime(25);
    scheduler.request(owner, draw, () => true, 100);
    vi.advanceTimersByTime(74);
    expect(draw).toHaveBeenCalledTimes(1);

    vi.advanceTimersByTime(1);
    vi.runOnlyPendingTimers();
    expect(draw).toHaveBeenCalledTimes(2);
  });

  it('does not paint hidden work and can cancel it before it becomes visible', () => {
    const owner = {};
    const draw = vi.fn();
    let visible = false;

    scheduler.request(owner, draw, () => visible);
    vi.runOnlyPendingTimers();
    expect(draw).not.toHaveBeenCalled();

    visible = true;
    scheduler.request(owner, draw, () => visible);
    scheduler.cancel(owner);
    vi.runOnlyPendingTimers();
    expect(draw).not.toHaveBeenCalled();
  });
});
