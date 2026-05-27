import { describe, it, expect, beforeEach, afterEach, vi } from "vitest";
import { TestBed } from '@angular/core/testing';
import { TelemetryService } from './telemetry.service';
import { StateService } from './state.service';
import { ApiService } from './api.service';
import { AudioService } from './audio.service';
import { PersistenceService } from './persistence.service';

let mockEventSourceInstance: any = null;

class MockEventSource {
  listeners: Record<string, Function[]> = {};
  onopen: Function | null = null;
  onerror: Function | null = null;

  constructor(public url: string) {
    mockEventSourceInstance = this;
  }

  addEventListener(event: string, callback: Function) {
    if (!this.listeners[event]) {
      this.listeners[event] = [];
    }
    this.listeners[event].push(callback);
  }

  close() {}

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
  let originalEventSource: any;

  beforeEach(() => {
    mockEventSourceInstance = null;
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

    TestBed.configureTestingModule({
      providers: [
        TelemetryService,
        StateService,
        { provide: PersistenceService, useValue: mockPersistence },
        { provide: AudioService, useValue: mockAudio },
        { provide: ApiService, useValue: mockApi }
      ]
    });
  });

  afterEach(() => {
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
    mockEventSourceInstance.triggerEvent('live', badDataEvent);

    expect(consoleWarnSpy).toHaveBeenCalledWith('SSE live parse failed', expect.any(Error));
  });
});
