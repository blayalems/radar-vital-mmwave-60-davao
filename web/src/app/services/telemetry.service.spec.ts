import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import { signal } from '@angular/core';
import { TestBed } from '@angular/core/testing';
import { TelemetryService } from './telemetry.service';
import { StateService } from './state.service';
import { ApiService } from './api.service';
import { AudioService } from './audio.service';
import { AuthService } from './auth.service';
import { PersistenceService } from './persistence.service';
import { OPERATOR_TOKEN_KEY } from './rvt-storage-keys';
import { SseDriverService } from './sse-driver.service';

let mockEventSourceInstance: MockEventSource | null = null;
let mockEventSourceInstances: MockEventSource[] = [];

class MockEventSource {
  listeners: Record<string, Function[]> = {};
  onopen: Function | null = null;
  onerror: Function | null = null;
  closed = false;

  constructor(public url: string) {
    mockEventSourceInstance = this;
    mockEventSourceInstances.push(this);
  }

  addEventListener(event: string, callback: Function) {
    if (!this.listeners[event]) {
      this.listeners[event] = [];
    }
    this.listeners[event].push(callback);
  }

  close() {
    this.closed = true;
  }

  triggerEvent(event: string, data: any) {
    if (this.listeners[event]) {
      this.listeners[event].forEach(cb => cb(data));
    }
  }
}

describe('TelemetryService', () => {
  let service: TelemetryService;
  let mockPersistence: Partial<PersistenceService>;
  let mockAudio: Partial<AudioService>;
  let mockApi: Partial<ApiService>;
  let mockAuth: Pick<AuthService, 'isLocked'>;
  let originalEventSource: any;

  beforeEach(() => {
    localStorage.clear();
    sessionStorage.clear();
    mockEventSourceInstance = null;
    mockEventSourceInstances = [];
    originalEventSource = window.EventSource;
    (window as any).EventSource = MockEventSource;

    mockPersistence = {
      quarantineLegacyLocalStorage: vi.fn().mockResolvedValue(undefined),
      get: vi.fn().mockResolvedValue(undefined),
      put: vi.fn().mockResolvedValue(undefined)
    };

    mockAudio = {
      playAlertBeep: vi.fn(),
      speakAlert: vi.fn()
    };

    mockApi = {
      currentApiBase: vi.fn().mockReturnValue(''),
      detectControlMode: vi.fn().mockResolvedValue(undefined),
      request: vi.fn().mockResolvedValue(undefined),
      hasPairToken: vi.fn().mockReturnValue(false),
      pairToken: vi.fn().mockReturnValue('')
    };
    mockAuth = {
      isLocked: signal(false)
    };

    TestBed.configureTestingModule({
      providers: [
        TelemetryService,
        StateService,
        { provide: PersistenceService, useValue: mockPersistence },
        { provide: AudioService, useValue: mockAudio },
        { provide: ApiService, useValue: mockApi },
        { provide: AuthService, useValue: mockAuth }
      ]
    });
  });

  afterEach(() => {
    service?.stop();
    localStorage.clear();
    sessionStorage.clear();
    window.EventSource = originalEventSource;
    vi.restoreAllMocks();
  });

  it('should be created', () => {
    service = TestBed.inject(TelemetryService);
    expect(service).toBeTruthy();
  });

  it('should handle invalid JSON gracefully in SSE live event', async () => {
    const consoleWarnSpy = vi.spyOn(console, 'warn').mockImplementation(() => {});

    service = TestBed.inject(TelemetryService);

    // We need to wait for the effect to call startSse
    // Because effect() is scheduled. Since we mock detectControlMode(), we can await its resolution
    await new Promise(resolve => setTimeout(resolve, 0));

    // Force startSse if not started
    (service as any).startSse();

    expect(mockEventSourceInstance).toBeTruthy();

    const badDataEvent = new MessageEvent('live', { data: '{ invalid: json }' });

    // Trigger the event
    mockEventSourceInstance!.triggerEvent('live', badDataEvent);

    expect(consoleWarnSpy).toHaveBeenCalledWith('SSE live parse failed', expect.any(Error));
  });

  it('does not alert for standby zero vitals', () => {
    service = TestBed.inject(TelemetryService);
    const state = TestBed.inject(StateService);

    (service as any).applyLivePayload({
      meta: { status: 'waiting', active: false },
      radar: { reported_hr: 0, reported_rr: 0 }
    });

    expect(state.alertHistory()).toEqual([]);
    expect(mockAudio.playAlertBeep).not.toHaveBeenCalled();
    expect(mockAudio.speakAlert).not.toHaveBeenCalled();
  });

  it('creates only one EventSource while an SSE token request is in flight', async () => {
    service = TestBed.inject(TelemetryService);
    await settle();
    service.stop();
    mockEventSourceInstances = [];
    mockEventSourceInstance = null;
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'operator-token');

    const tokenRequest = deferred<{ sse_token: string }>();
    (mockApi.request as ReturnType<typeof vi.fn>).mockImplementation(() => tokenRequest.promise);

    service.start();
    void (service as any).startSse();

    expect(mockApi.request).toHaveBeenCalledTimes(1);
    expect(mockEventSourceInstances).toHaveLength(0);

    tokenRequest.resolve({ sse_token: 'sse-token-1' });
    await settle();

    expect(mockEventSourceInstances).toHaveLength(1);
    expect(mockEventSourceInstances[0].url).toContain('sse-token-1');
  });

  it('does not create an EventSource after stop cancels token minting', async () => {
    service = TestBed.inject(TelemetryService);
    await settle();
    service.stop();
    mockEventSourceInstances = [];
    mockEventSourceInstance = null;
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'operator-token');

    const tokenRequest = deferred<{ sse_token: string }>();
    (mockApi.request as ReturnType<typeof vi.fn>).mockImplementation(() => tokenRequest.promise);

    service.start();
    service.stop();
    tokenRequest.resolve({ sse_token: 'unused-token' });
    await settle();

    expect(mockEventSourceInstances).toHaveLength(0);
  });

  it('rotates the one-use SSE token at the stream deadline without clearing session state', async () => {
    service = TestBed.inject(TelemetryService);
    const state = TestBed.inject(StateService);
    await settle();
    service.stop();
    mockEventSourceInstances = [];
    mockEventSourceInstance = null;
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'operator-token');
    (mockApi.request as ReturnType<typeof vi.fn>)
      .mockResolvedValueOnce({ sse_token: 'sse-token-1' })
      .mockResolvedValueOnce({ sse_token: 'sse-token-2' });
    state.sessionActive.set(true);
    state.currentSessionId.set('session-live-1');

    service.start();
    await settle();
    const firstSource = mockEventSourceInstances[0];

    firstSource.triggerEvent(
      'session_warning',
      new MessageEvent('session_warning', {
        data: JSON.stringify({ reason: 'deadline_approaching' })
      })
    );
    await settle();

    expect(firstSource.closed).toBe(true);
    expect(mockEventSourceInstances).toHaveLength(2);
    expect(mockEventSourceInstances[1].url).toContain('sse-token-2');
    expect(state.sessionActive()).toBe(true);
    expect(state.currentSessionId()).toBe('session-live-1');
  });

  it('stays polling-only after the SSE failure threshold until manual reconnect', async () => {
    service = TestBed.inject(TelemetryService);
    await settle();
    const failedSource = mockEventSourceInstances.at(-1)!;

    failedSource.onerror?.();
    failedSource.onerror?.();
    failedSource.onerror?.();
    failedSource.onerror?.();
    await settle();

    expect(failedSource.closed).toBe(true);
    expect(TestBed.inject(SseDriverService).isPollingOnly()).toBe(true);
    const instancesAfterFallback = mockEventSourceInstances.length;

    await (service as any).startSse();
    expect(mockEventSourceInstances).toHaveLength(instancesAfterFallback);

    service.reconnect();
    await settle();
    expect(TestBed.inject(SseDriverService).isPollingOnly()).toBe(false);
    expect(mockEventSourceInstances).toHaveLength(instancesAfterFallback + 1);
  });
});

function deferred<T>() {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>((res) => {
    resolve = res;
  });
  return { promise, resolve };
}

async function settle(): Promise<void> {
  await new Promise(resolve => setTimeout(resolve, 0));
}
