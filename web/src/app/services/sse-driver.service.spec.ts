import { TestBed } from '@angular/core/testing';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { SseDriverConfig, SseDriverEvent, SseDriverService } from './sse-driver.service';

let sources: MockEventSource[] = [];

class MockEventSource {
  readonly listeners: Record<string, Array<(event: MessageEvent) => void>> = {};
  onopen: (() => void) | null = null;
  onerror: (() => void) | null = null;
  closed = false;

  constructor(readonly url: string) {
    sources.push(this);
  }

  addEventListener(type: string, callback: (event: MessageEvent) => void): void {
    (this.listeners[type] ||= []).push(callback);
  }

  close(): void {
    this.closed = true;
  }

  emit(type: string, data = ''): void {
    this.listeners[type]?.forEach(callback => callback(new MessageEvent(type, { data })));
  }
}

describe('SseDriverService', () => {
  let driver: SseDriverService;
  let originalEventSource: typeof EventSource;
  let events: SseDriverEvent[];
  let canConnect: boolean;

  beforeEach(() => {
    originalEventSource = window.EventSource;
    window.EventSource = MockEventSource as unknown as typeof EventSource;
    sources = [];
    events = [];
    canConnect = true;
    TestBed.configureTestingModule({ providers: [SseDriverService] });
    driver = TestBed.inject(SseDriverService);
  });

  afterEach(() => {
    driver.cancel();
    window.EventSource = originalEventSource;
    vi.useRealTimers();
    vi.restoreAllMocks();
  });

  it('keeps token minting single-flight and forwards typed transport events', async () => {
    const mint = deferred('stream-token');
    const config = makeConfig(() => mint.promise);

    void driver.connect(config);
    void driver.connect(config);
    expect(sources).toHaveLength(0);

    mint.resolve();
    await settle();
    expect(sources).toHaveLength(1);
    expect(sources[0].url).toBe('/api/events/subscribe?token=stream-token');

    sources[0].onopen?.();
    sources[0].emit('live', '{"radar":{"reported_hr":72}}');
    expect(driver.transportState()).toBe('connected');
    expect(events).toEqual([
      { type: 'open' },
      { type: 'live', data: '{"radar":{"reported_hr":72}}' }
    ]);
  });

  it('cancels an in-flight token mint without creating an orphan EventSource', async () => {
    const mint = deferred('unused-token');
    void driver.connect(makeConfig(() => mint.promise));

    driver.cancel();
    mint.resolve();
    await settle();

    expect(sources).toHaveLength(0);
    expect(driver.transportState()).toBe('idle');
  });

  it('renews with a new one-use token while preserving the event contract', async () => {
    const mintToken = vi.fn()
      .mockResolvedValueOnce('stream-token-1')
      .mockResolvedValueOnce('stream-token-2');
    await driver.connect(makeConfig(mintToken));
    const first = sources[0];

    driver.renew();
    await settle();

    expect(first.closed).toBe(true);
    expect(sources).toHaveLength(2);
    expect(sources[1].url).toContain('stream-token-2');
  });

  it('aborts connect and schedules reconnect without opening an unauthenticated EventSource when mintToken throws', async () => {
    vi.useFakeTimers();
    const mintToken = vi.fn()
      .mockRejectedValueOnce(new Error('401 Unauthorized'))
      .mockResolvedValueOnce('fresh-token');

    await driver.connect(makeConfig(mintToken));

    expect(sources).toHaveLength(0);
    expect(driver.transportState()).toBe('backoff');
    expect(events.at(-1)).toEqual({ type: 'fallback', reason: 'connection_failed' });

    await vi.advanceTimersByTimeAsync(17_000);
    expect(sources).toHaveLength(1);
    expect(sources[0].url).toBe('/api/events/subscribe?token=fresh-token');
  });

  it('closes stale EventSource on error to mint a fresh single-use token, enters polling-only on threshold, and periodically recovers', async () => {
    vi.useFakeTimers();
    const mintToken = vi.fn()
      .mockResolvedValueOnce('token-1')
      .mockResolvedValueOnce('token-2')
      .mockResolvedValueOnce('token-3')
      .mockResolvedValueOnce('token-4')
      .mockResolvedValueOnce('token-recovered');
    await driver.connect(makeConfig(mintToken));
    expect(sources).toHaveLength(1);

    sources[0].onerror?.();
    expect(sources[0].closed).toBe(true);
    expect(driver.transportState()).toBe('backoff');
    await vi.advanceTimersByTimeAsync(17_000);
    expect(sources).toHaveLength(2);
    expect(sources[1].url).toContain('token-2');

    sources[1].onerror?.();
    expect(sources[1].closed).toBe(true);
    await vi.advanceTimersByTimeAsync(32_000);
    expect(sources).toHaveLength(3);
    expect(sources[2].url).toContain('token-3');

    sources[2].onerror?.();
    expect(sources[2].closed).toBe(true);
    await vi.advanceTimersByTimeAsync(62_000);
    expect(sources).toHaveLength(4);
    expect(sources[3].url).toContain('token-4');

    // Fourth error crosses threshold -> pollingOnly = true
    sources[3].onerror?.();
    expect(sources[3].closed).toBe(true);
    expect(driver.transportState()).toBe('idle');
    expect(driver.isPollingOnly()).toBe(true);
    expect(driver.nextRetryAtMs()).toBeNull();
    expect(events.at(-1)).toEqual({ type: 'fallback', reason: 'error_threshold' });

    await driver.connect();
    expect(sources).toHaveLength(4);

    // Periodic recovery clears pollingOnly and mints a fresh token
    await vi.advanceTimersByTimeAsync(60_500);
    expect(driver.isPollingOnly()).toBe(false);
    expect(sources).toHaveLength(5);
    expect(sources[4].url).toContain('token-recovered');
  });

  function makeConfig(mintToken: () => Promise<string>): SseDriverConfig {
    return {
      canConnect: () => canConnect,
      baseUrl: () => '',
      mintToken,
      onEvent: event => events.push(event)
    };
  }
});

function deferred(value: string): { promise: Promise<string>; resolve: () => void } {
  let resolvePromise!: (value: string) => void;
  const promise = new Promise<string>(resolve => {
    resolvePromise = resolve;
  });
  return { promise, resolve: () => resolvePromise(value) };
}

async function settle(): Promise<void> {
  await Promise.resolve();
  await Promise.resolve();
}
