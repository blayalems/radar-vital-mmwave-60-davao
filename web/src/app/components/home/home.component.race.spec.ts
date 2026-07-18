import { signal } from '@angular/core';
import { TestBed } from '@angular/core/testing';
import { Router } from '@angular/router';
import { MatSnackBar } from '@angular/material/snack-bar';

import { SetupState } from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';
import { AudioService } from '../../services/audio.service';
import { BluetoothService } from '../../services/bluetooth.service';
import { I18nService } from '../../services/i18n.service';
import { InstallPromptService } from '../../services/install-prompt.service';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';
import { StateService } from '../../services/state.service';
import { TelemetryService } from '../../services/telemetry.service';
import { HomeComponent, preflightSetupFingerprint } from './home.component';

interface Deferred<T> {
  promise: Promise<T>;
  resolve(value: T): void;
}

function deferred<T>(): Deferred<T> {
  let resolve!: (value: T) => void;
  const promise = new Promise<T>(resolvePromise => {
    resolve = resolvePromise;
  });
  return { promise, resolve };
}

const initialSetup = (): SetupState => ({
  duration_s: 30,
  customDuration: 30,
  customUnit: 's',
  radar_port: 'COM7',
  ble_address: 'AA:BB:CC:DD:EE:01',
  ble_profile: 'ailink_oximeter',
  notify_char: '0000ffe2-0000-1000-8000-00805f9b34fb',
  subject_label: 'P-001',
  operator_label: 'Operator A',
  station_label: 'Lab 1',
  subject_profile_id: 'adult_default',
  skip_countdown: false
});

const passingChecks = (port: string) => [
  { id: 'serial_port_list', label: 'Serial', status: 'good', description: port },
  { id: 'session_folder_writable', label: 'Storage', status: 'good', description: 'ready' }
];

describe('HomeComponent preflight request ownership', () => {
  let component: HomeComponent;
  let request: ReturnType<typeof vi.fn>;
  let setup: ReturnType<typeof signal<SetupState>>;
  let preflightChecks: ReturnType<typeof signal<ReturnType<typeof passingChecks>>>;
  let preflightRunning: ReturnType<typeof signal<boolean>>;
  let snackbarOpen: ReturnType<typeof vi.fn>;
  let state: Record<string, unknown>;

  beforeEach(() => {
    vi.useFakeTimers();
    request = vi.fn();
    setup = signal(initialSetup());
    preflightChecks = signal<ReturnType<typeof passingChecks>>([]);
    preflightRunning = signal(false);
    snackbarOpen = vi.fn();
    state = {
      setup,
      preflightChecks,
      preflightRunning,
      preflightUpdatedAtMs: signal<number | null>(null),
      lastPayload: signal(null),
      spark: signal({}),
      ctlStatus: signal({ ok: true, mode: 'live' }),
      theme: signal('light'),
      sessionItems: signal([]),
      sessionNotes: signal({}),
      currentSessionId: signal<string | null>(null),
      sessionActive: signal(false),
      ctlOn: signal(false),
      triggerHaptic: vi.fn()
    };

    TestBed.configureTestingModule({
      providers: [
        { provide: StateService, useValue: state },
        { provide: ApiService, useValue: { request } },
        { provide: TelemetryService, useValue: {} },
        { provide: AudioService, useValue: { speakAlert: vi.fn() } },
        { provide: BluetoothService, useValue: {} },
        { provide: InstallPromptService, useValue: {} },
        { provide: ServerLifecycleService, useValue: {} },
        { provide: I18nService, useValue: {} },
        { provide: Router, useValue: { navigate: vi.fn() } },
        { provide: MatSnackBar, useValue: { open: snackbarOpen } }
      ]
    });
    component = TestBed.runInInjectionContext(() => new HomeComponent());
    component.selectedDuration = setup().duration_s;
  });

  afterEach(() => {
    TestBed.resetTestingModule();
    vi.clearAllTimers();
    vi.useRealTimers();
    vi.restoreAllMocks();
  });

  it('applies only the newest setup response and keeps running until it finishes', async () => {
    const runAResponse = deferred<{ checks: ReturnType<typeof passingChecks> }>();
    const runBResponse = deferred<{ checks: ReturnType<typeof passingChecks> }>();
    request.mockImplementation((path: string) => {
      if (path.includes('port=COM7')) return runAResponse.promise;
      if (path.includes('port=COM10')) return runBResponse.promise;
      throw new Error(`Unexpected request: ${path}`);
    });

    const runA = component.runPreflight();
    setup.update(value => ({ ...value, radar_port: 'COM10' }));
    const runB = component.runPreflight();

    runAResponse.resolve({ checks: passingChecks('COM7') });
    await runA;
    expect(preflightChecks()).toEqual([]);
    expect(preflightRunning()).toBe(true);

    runBResponse.resolve({ checks: passingChecks('COM10') });
    await expect(runB).resolves.toBe(true);
    expect(preflightChecks()).toEqual(passingChecks('COM10'));
    expect(preflightRunning()).toBe(false);
    expect(component.canStartSession()).toBe(true);
  });

  it('cancels Start when any submitted setup value changes during preflight', async () => {
    const preflightResponse = deferred<{ checks: ReturnType<typeof passingChecks> }>();
    request.mockImplementation((path: string) => {
      if (path.startsWith('/api/preflight?')) return preflightResponse.promise;
      if (path === '/api/session/start') return Promise.resolve({ session_id: 'should-not-start' });
      throw new Error(`Unexpected request: ${path}`);
    });

    const start = component.startSession();
    setup.update(value => ({ ...value, subject_label: 'P-002' }));
    preflightResponse.resolve({ checks: passingChecks('COM7') });
    await start;

    expect(request).not.toHaveBeenCalledWith('/api/session/start', expect.anything());
    expect(snackbarOpen).toHaveBeenCalledWith(
      expect.stringContaining('setup changed while preflight was running'),
      'Dismiss',
      { duration: 7000 }
    );
    expect(component.isStartingSession).toBe(false);
  });

  it('posts the exact click-time setup after a current successful preflight', async () => {
    request.mockImplementation((path: string) => {
      if (path.startsWith('/api/preflight?')) return Promise.resolve({ checks: passingChecks('COM7') });
      if (path === '/api/session/start') return Promise.resolve({ session_id: 'session-1' });
      throw new Error(`Unexpected request: ${path}`);
    });

    await component.startSession();

    expect(request).toHaveBeenCalledWith('/api/session/start', expect.objectContaining({
      body: JSON.stringify({
        duration_s: 30,
        radar_port: 'COM7',
        ble_address: 'AA:BB:CC:DD:EE:01',
        subject_label: 'P-001',
        operator_label: 'Operator A',
        station_label: 'Lab 1',
        subject_profile_id: 'adult_default',
        ble_profile: 'ailink_oximeter',
        skip_countdown: false,
        advanced: { notify_char: '0000ffe2-0000-1000-8000-00805f9b34fb' }
      })
    }));
  });

  it('fingerprints only the hardware inputs exercised by preflight', () => {
    const first = initialSetup();
    const renamed = { ...first, subject_label: 'P-099' };
    expect(preflightSetupFingerprint(first)).toBe(preflightSetupFingerprint(renamed));
    expect(preflightSetupFingerprint({ ...first, radar_port: 'COM10' }))
      .not.toBe(preflightSetupFingerprint(first));
  });
});
