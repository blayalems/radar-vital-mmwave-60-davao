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

  it('falls back after the fourth error and exposes a cancellable retry deadline', async () => {
    vi.useFakeTimers();
    vi.spyOn(Math, 'random').mockReturnValue(0.5);
    await driver.connect(makeConfig(async () => ''));
    const source = sources[0];
    source.onopen?.();

    source.onerror?.();
    source.onerror?.();
    source.onerror?.();
    source.onerror?.();

    expect(source.closed).toBe(true);
    expect(driver.transportState()).toBe('backoff');
    expect(driver.nextRetryAtMs()).toBe(Date.now() + 15_000);
    expect(events.at(-1)).toEqual({ type: 'fallback', reason: 'error_threshold' });

    driver.cancel();
    vi.advanceTimersByTime(15_000);
    expect(sources).toHaveLength(1);
    expect(driver.nextRetryAtMs()).toBeNull();
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
