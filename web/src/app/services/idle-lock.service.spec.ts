import { TestBed } from '@angular/core/testing';
import { IdleLockService } from './idle-lock.service';

describe('IdleLockService', () => {
  let service: IdleLockService;

  beforeEach(() => {
    localStorage.clear();
    vi.useFakeTimers();
    TestBed.configureTestingModule({ providers: [IdleLockService] });
    service = TestBed.inject(IdleLockService);
  });

  afterEach(() => {
    service.stop();
    vi.useRealTimers();
    localStorage.clear();
  });

  it('defaults to disabled with a 10 minute timeout', () => {
    expect(service.enabled()).toBe(false);
    expect(service.timeoutMinutes()).toBe(10);
    expect(service.locked()).toBe(false);
  });

  it('locks immediately on manual request even when disabled', () => {
    service.lockNow('manual');
    expect(service.locked()).toBe(true);
    expect(service.lockReason()).toBe('manual');
  });

  it('unlock clears the curtain', () => {
    service.lockNow('manual');
    service.unlock();
    expect(service.locked()).toBe(false);
  });

  it('auto-locks after the idle timeout when enabled', () => {
    service.setTimeoutMinutes(1);
    service.setEnabled(true);
    service.start();
    expect(service.locked()).toBe(false);
    vi.advanceTimersByTime(60_000);
    expect(service.locked()).toBe(true);
    expect(service.lockReason()).toBe('idle');
  });

  it('clamps the timeout to the supported range', () => {
    service.setTimeoutMinutes(0);
    expect(service.timeoutMinutes()).toBe(1);
    service.setTimeoutMinutes(999);
    expect(service.timeoutMinutes()).toBe(120);
  });

  it('persists enabled and timeout to localStorage', () => {
    service.setEnabled(true);
    service.setTimeoutMinutes(5);
    expect(localStorage.getItem('rvt-idle-lock-enabled')).toBe('1');
    expect(localStorage.getItem('rvt-idle-lock-timeout-min')).toBe('5');
  });

  it('does not auto-lock when disabled', () => {
    service.setEnabled(false);
    service.start();
    vi.advanceTimersByTime(60 * 60_000);
    expect(service.locked()).toBe(false);
  });
});
