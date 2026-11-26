import { signal } from '@angular/core';
import { TestBed } from '@angular/core/testing';
import { MatSnackBar } from '@angular/material/snack-bar';

import { SetupState } from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';
import { StateService } from '../../services/state.service';
import {
  CONFIRMATORY_DURATION_S,
  ParticipantStudySetupComponent,
  studyConditionId,
  studySetupError,
  studyTrialId
} from './participant-study-setup.component';

const initialSetup = (): SetupState => ({
  duration_s: 30,
  customDuration: 30,
  customUnit: 's',
  radar_port: 'COM7',
  ble_address: 'AA:BB:CC:DD:EE:01',
  ble_profile: 'ailink_oximeter',
  notify_char: 'notify',
  subject_label: '',
  operator_label: 'Operator A',
  station_label: 'Lab 1',
  subject_profile_id: 'adult_default',
  participant_id: '',
  study_mode: 'confirmatory',
  condition_id: 'd060_none',
  distance_m: 0.6,
  barrier_type: 'none',
  trial_number: 1,
  skip_countdown: false
});

describe('participant study contract', () => {
  it('builds canonical condition and deterministic trial identifiers', () => {
    expect(studyConditionId('confirmatory', 0.6, 'none')).toBe('d060_none');
    expect(studyConditionId('confirmatory', 0.8, 'cardboard')).toBe('d080_cardboard');
    expect(studyConditionId('exploratory', 1, 'none')).toBe('d100_none');
    expect(studyTrialId({
      participant_id: 'P-001',
      condition_id: 'd060_none',
      trial_number: 2
    })).toBe('P-001-d060_none-t2');
  });

  it('requires the frozen confirmatory distances, duration, participant, and trial', () => {
    const valid = {
      ...initialSetup(),
      participant_id: 'P-001',
      duration_s: CONFIRMATORY_DURATION_S
    };
    expect(studySetupError(valid)).toBe('');
    expect(studySetupError({ ...valid, participant_id: '' })).toContain('participant');
    expect(studySetupError({ ...valid, distance_m: 0.7 })).toContain('0.6, 0.8, 1.0, 2.0, or 3.0');
    expect(studySetupError({ ...valid, duration_s: 120 })).toContain('150 seconds');
    expect(studySetupError({ ...valid, trial_number: 4 })).toContain('trial 1, 2, or 3');
  });

  it('allows exploratory captures only inside the 0.5 to 3.0 m development range', () => {
    const valid = {
      ...initialSetup(),
      participant_id: 'P-002',
      study_mode: 'exploratory' as const,
      distance_m: 0.55
    };
    expect(studySetupError(valid)).toBe('');
    expect(studySetupError({ ...valid, distance_m: 0.49 })).toContain('0.5 to 3.0');
    expect(studySetupError({ ...valid, distance_m: 3.01 })).toContain('0.5 to 3.0');
  });
});

describe('ParticipantStudySetupComponent', () => {
  let setup: ReturnType<typeof signal<SetupState>>;
  let request: ReturnType<typeof vi.fn>;
  let snackbarOpen: ReturnType<typeof vi.fn>;
  let component: ParticipantStudySetupComponent;

  beforeEach(() => {
    setup = signal(initialSetup());
    request = vi.fn();
    snackbarOpen = vi.fn();
    TestBed.configureTestingModule({
      providers: [
        {
          provide: StateService,
          useValue: {
            setup,
            demoSourceActive: signal(false),
            triggerHaptic: vi.fn()
          }
        },
        { provide: ApiService, useValue: { request } },
        { provide: MatSnackBar, useValue: { open: snackbarOpen } }
      ]
    });
    component = TestBed.runInInjectionContext(() => new ParticipantStudySetupComponent());
    component.readiness.set({
      ok: true,
      schema_version: 'rvt-collection-readiness-v1',
      authorized: true,
      protocol_state: 'locked',
      plan_status: 'approved',
      authorization_record_present: true,
      blockers: [],
      unresolved_withdrawal_count: 0
    });
  });

  afterEach(() => {
    TestBed.resetTestingModule();
  });

  it('accepts the items response alias and keeps profile selection pseudonymous', async () => {
    request.mockResolvedValue({
      items: [{ participant_id: 'P-003', display_code: 'P-003', status: 'active', completed_trials: 4 }]
    });

    await component.loadParticipants();
    component.selectParticipant(component.participants()[0]);

    expect(component.participants()).toHaveLength(1);
    expect(setup().participant_id).toBe('P-003');
    expect(setup().subject_label).toBe('P-003');
  });

  it('clears and invalidates a cached participant when the roster marks consent withdrawn', async () => {
    setup.update(value => ({ ...value, participant_id: 'P-003', subject_label: 'P-003' }));
    request.mockResolvedValue({
      participants: [{ participant_id: 'P-003', display_code: 'P-003', status: 'withdrawn' }]
    });
    const validity: boolean[] = [];
    component.rosterValidityChanged.subscribe(value => validity.push(value));

    await component.loadParticipants();

    expect(setup().participant_id).toBe('');
    expect(setup().subject_label).toBe('');
    expect(validity).toEqual([false, false]);
  });

  it('fails closed and clears a cached participant when the roster cannot be verified', async () => {
    setup.update(value => ({ ...value, participant_id: 'P-003', subject_label: 'P-003' }));
    request.mockRejectedValue(new Error('Roster unavailable'));
    const validity: boolean[] = [];
    component.rosterValidityChanged.subscribe(value => validity.push(value));

    await component.loadParticipants();

    expect(component.participants()).toEqual([]);
    expect(component.loadError()).toBe('Roster unavailable');
    expect(setup().participant_id).toBe('');
    expect(setup().subject_label).toBe('');
    expect(validity).toEqual([false, false]);
  });

  it('accepts the backend profile alias when creating and selects the new profile', async () => {
    request.mockResolvedValue({
      profile: { participant_id: 'P-004', display_code: 'P-004', status: 'active' }
    });

    await component.createParticipant();

    expect(request).toHaveBeenCalledWith('/api/participants', expect.objectContaining({ method: 'POST' }));
    expect(setup().participant_id).toBe('P-004');
    expect(snackbarOpen).toHaveBeenCalledWith(
      'P-004 created and selected.',
      'Dismiss',
      { duration: 3500 }
    );
  });

  it('blocks recruitment and confirmatory mode when readiness is not authorized', async () => {
    component.readiness.set({
      ok: true,
      schema_version: 'rvt-collection-readiness-v1',
      authorized: false,
      protocol_state: 'locked',
      plan_status: 'draft',
      authorization_record_present: false,
      blockers: ['statistical_analysis_plan_not_approved'],
      unresolved_withdrawal_count: 0
    });
    setup.update(value => ({ ...value, study_mode: 'exploratory' }));

    await component.createParticipant();
    component.setStudyMode('confirmatory');

    expect(request).not.toHaveBeenCalled();
    expect(setup().study_mode).toBe('exploratory');
  });

  it('locks confirmatory duration and recomputes the canonical condition', () => {
    component.setStudyMode('confirmatory');
    component.setDistance(0.8);
    component.setBarrier('cardboard');
    component.setTrial(3);

    expect(setup()).toEqual(expect.objectContaining({
      duration_s: 150,
      condition_id: 'd080_cardboard',
      distance_m: 0.8,
      barrier_type: 'cardboard',
      trial_number: 3
    }));
  });

  it('supports exploratory distances up to 3.0 m without clamping at 1.0 m', () => {
    component.setStudyMode('confirmatory');
    component.setDistance(2.0);
    component.setStudyMode('exploratory');
    expect(setup().distance_m).toBe(2.0);

    component.setDistance(2.5);
    expect(setup()).toEqual(expect.objectContaining({
      distance_m: 2.5,
      condition_id: 'd250_none'
    }));

    component.setDistance(3.5);
    expect(setup()).toEqual(expect.objectContaining({
      distance_m: 3.0,
      condition_id: 'd300_none'
    }));
  });

  it('orders the 10x3 condition matrix by study schedule, flags <15 valid windows, and prefills setup on cell click', async () => {
    request.mockImplementation((path: string) => {
      if (path.startsWith('/api/study/schedule')) {
        return Promise.resolve({
          participant_id: 'P-003',
          entries: [
            { participant_id: 'P-003', order: 1, condition_id: 'd200_cardboard', trial_numbers: [1, 2, 3] },
            { participant_id: 'P-003', order: 2, condition_id: 'd060_none', trial_numbers: [1, 2, 3] }
          ]
        });
      }
      if (path === '/api/participants') {
        return Promise.resolve({
          participants: [{ participant_id: 'P-003', display_code: 'P-003', status: 'active', completed_trials: 1 }],
          completion_matrix: {
            conditions: ['d060_none', 'd200_cardboard'],
            trials: [1, 2, 3],
            participants: {
              'P-003': {
                participant_id: 'P-003',
                completed_trials: 1,
                expected_trials: 30,
                protocol_complete: false,
                conditions_summary: {
                  d200_cardboard: { valid_trials: 0, meets_trial_floor: false },
                  d060_none: { valid_trials: 2, meets_trial_floor: true }
                },
                cells: {
                  'd200_cardboard:t2': { status: 'completed', valid_windows: 12, meets_window_floor: false },
                  'd060_none:t1': { status: 'completed', valid_windows: 22, meets_window_floor: true }
                }
              }
            },
            participant_count: 1,
            protocol_complete_participant_count: 0,
            no_subject_attempt_count: 3,
            no_subject_qualified_count: 2,
            no_subject_qualified_duration_s: 300,
            no_subject_target_duration_s: 10800,
            no_subject_unqualified_reasons: { duration_mismatch: 1 },
            no_subject_expected: 72,
            attempt_count: 4
          }
        });
      }
      return Promise.resolve({});
    });

    await component.loadParticipants();
    component.selectParticipant(component.participants()[0]);
    await Promise.resolve();
    await Promise.resolve();

    const rows = component.conditionMatrixRows();
    expect(rows.map(r => r.conditionId)).toEqual(['d200_cardboard', 'd060_none']);
    expect(rows[0].meetsTrialFloor).toBe(false);
    expect(rows[0].cells[1].validWindows).toBe(12);
    expect(rows[0].cells[1].meetsWindowFloor).toBe(false);
    expect(rows[1].meetsTrialFloor).toBe(true);
    expect(rows[1].cells[0].validWindows).toBe(22);

    component.selectMatrixCell(rows[0], 2);
    expect(setup()).toEqual(expect.objectContaining({
      condition_id: 'd200_cardboard',
      distance_m: 2.0,
      barrier_type: 'cardboard',
      trial_number: 2,
      duration_s: 150
    }));

    expect(component.noSubjectStats()).toEqual(expect.objectContaining({
      qualifiedCount: 2,
      targetCount: 72,
      qualifiedDurationS: 300,
      targetDurationS: 10800,
      reasons: { duration_mismatch: 1 }
    }));
  });

  it('runs and resets the non-blocking 300 s MLX90614 acclimatization timer without blocking study setup', () => {
    vi.useFakeTimers();
    sessionStorage.clear();
    component.setDistance(0.6);
    component.setBarrier('none');
    expect(component.isD060NoneSelected()).toBe(true);
    expect(component.acclimatizationRemainingS()).toBe(300);

    component.startAcclimatizationTimer();
    vi.advanceTimersByTime(3000);
    expect(component.acclimatizationRemainingS()).toBe(297);
    expect(component.acclimatizationDisplay()).toBe('04:57');
    expect(studySetupError({ ...setup(), participant_id: 'P-001', duration_s: 150 })).toBe('');

    component.pauseAcclimatizationTimer();
    vi.advanceTimersByTime(2000);
    expect(component.acclimatizationRemainingS()).toBe(297);

    component.resetAcclimatizationTimer();
    expect(component.acclimatizationRemainingS()).toBe(300);
    expect(component.acclimatizationDisplay()).toBe('05:00');
    vi.useRealTimers();
  });

  it('ignores stale resolved or rejected schedule responses after switching participants', async () => {
    const pendingByPid = new Map<string, {
      resolve: (value: unknown) => void;
      reject: (reason: unknown) => void;
    }>();
    request.mockImplementation((path: string) => new Promise((resolve, reject) => {
      const pid = new URL(`http://localhost${path}`).searchParams.get('participant_id') || '';
      pendingByPid.set(pid, { resolve, reject });
    }));

    const p1 = { participant_id: 'P-001', display_code: 'P-001', status: 'active' as const };
    const p2 = { participant_id: 'P-002', display_code: 'P-002', status: 'active' as const };
    component.participants.set([p1, p2]);

    component.selectParticipant(p1);
    component.selectParticipant(p2);

    pendingByPid.get('P-002')!.resolve({
      participant_id: 'P-002',
      entries: [
        { participant_id: 'P-002', order: 1, condition_id: 'd300_none', trial_numbers: [1, 2, 3] },
        { participant_id: 'P-002', order: 2, condition_id: 'd060_cardboard', trial_numbers: [1, 2, 3] }
      ]
    });
    await Promise.resolve();
    await Promise.resolve();
    expect(component.conditionMatrixRows().map(r => r.conditionId)).toEqual(['d300_none', 'd060_cardboard']);

    // Slow response for P-001 resolves after P-002 already loaded -> must not overwrite P-002.
    pendingByPid.get('P-001')!.resolve({
      participant_id: 'P-001',
      entries: [
        { participant_id: 'P-001', order: 1, condition_id: 'd100_none', trial_numbers: [1, 2, 3] }
      ]
    });
    await Promise.resolve();
    await Promise.resolve();
    expect(component.scheduleEntries().map(e => e.participant_id)).toEqual(['P-002', 'P-002']);
    expect(component.conditionMatrixRows().map(r => r.conditionId)).toEqual(['d300_none', 'd060_cardboard']);

    // Slow failure for P-001 after switching to P-003 -> must not clear P-003.
    const p3 = { participant_id: 'P-003', display_code: 'P-003', status: 'active' as const };
    component.participants.set([p1, p2, p3]);
    component.selectParticipant(p3);
    pendingByPid.get('P-003')!.resolve({
      participant_id: 'P-003',
      entries: [{ participant_id: 'P-003', order: 1, condition_id: 'd200_none', trial_numbers: [1, 2, 3] }]
    });
    await Promise.resolve();
    await Promise.resolve();

    const failPromise = component.loadSchedule('P-001');
    const p1Second = pendingByPid.get('P-001')!;
    p1Second.reject(new Error('timeout'));
    await failPromise;
    expect(component.scheduleEntries()).toEqual([]);

    // Re-run explicit race where P-002 loads then stale P-001 rejects afterwards:
    component.selectParticipant(p1);
    const staleRejectGate = pendingByPid.get('P-001')!;
    component.selectParticipant(p2);
    pendingByPid.get('P-002')!.resolve({
      participant_id: 'P-002',
      entries: [{ participant_id: 'P-002', order: 1, condition_id: 'd300_none', trial_numbers: [1, 2, 3] }]
    });
    await Promise.resolve();
    await Promise.resolve();
    staleRejectGate.reject(new Error('late P-001 failure'));
    await Promise.resolve();
    await Promise.resolve();
    expect(component.scheduleEntries().map(e => e.condition_id)).toEqual(['d300_none']);
  });
});
