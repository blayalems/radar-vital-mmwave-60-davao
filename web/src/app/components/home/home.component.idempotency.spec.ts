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
import { clientReleaseHandshake } from '../../services/release-contract';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';
import { StateService } from '../../services/state.service';
import { TelemetryService } from '../../services/telemetry.service';
import { HomeComponent } from './home.component';

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

const setup = (): SetupState => ({
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
  participant_id: 'P-001',
  study_mode: 'exploratory',
  condition_id: 'd060_none',
  distance_m: 0.6,
  barrier_type: 'none',
  trial_number: 1,
  skip_countdown: false
});

const passingChecks = [
  { id: 'serial_port_list', label: 'Serial', status: 'good', description: 'COM7' },
  { id: 'session_folder_writable', label: 'Storage', status: 'good', description: 'ready' }
];

describe('HomeComponent Start idempotency', () => {
  let component: HomeComponent;
  let request: ReturnType<typeof vi.fn>;
  let startResponse: Deferred<{ session_id: string }>;

  beforeEach(() => {
    startResponse = deferred<{ session_id: string }>();
    request = vi.fn((path: string) => {
      if (path.startsWith('/api/preflight?')) {
        return Promise.resolve({ checks: passingChecks });
      }
      if (path === '/api/version') {
        const release = clientReleaseHandshake();
        return Promise.resolve({
          product_version: release.product_version,
          trainer: release.product_version,
          dashboard: release.dashboard_version,
          firmware_expected: `v${release.product_version}`,
          serial_protocol: release.serial_protocol,
          serial_width_expected: release.serial_width_expected,
          schema_versions: release.schema_versions
        });
      }
      if (path === '/api/session/start') return startResponse.promise;
      throw new Error(`Unexpected request: ${path}`);
    });
    const state = {
      setup: signal(setup()),
      preflightChecks: signal<typeof passingChecks>([]),
      preflightRunning: signal(false),
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
        { provide: MatSnackBar, useValue: { open: vi.fn() } }
      ]
    });
    component = TestBed.runInInjectionContext(() => new HomeComponent());
    (component as unknown as { participantRosterValid: boolean }).participantRosterValid = true;
    component.selectedDuration = 30;
  });

  afterEach(() => {
    TestBed.resetTestingModule();
    vi.restoreAllMocks();
  });

  it('coalesces a second activation while Start is awaiting the trainer response', async () => {
    const first = component.startSession();
    await vi.waitFor(() => {
      expect(request.mock.calls.filter(([path]) => path === '/api/session/start')).toHaveLength(1);
    });

    const second = component.startSession();
    await Promise.resolve();

    const startCalls = request.mock.calls.filter(([path]) => path === '/api/session/start');
    expect(startCalls).toHaveLength(1);
    const firstPayload = JSON.parse(String(startCalls[0][1]?.body || '{}'));
    expect(firstPayload.idempotency_key).toMatch(/^[A-Za-z0-9_-]{16,128}$/);

    startResponse.resolve({ session_id: 's01' });
    await Promise.all([first, second]);
    expect(request.mock.calls.filter(([path]) => path === '/api/session/start')).toHaveLength(1);
  });
});
