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
    expect(studySetupError({ ...valid, distance_m: 0.7 })).toContain('0.6, 0.8, or 1.0');
    expect(studySetupError({ ...valid, duration_s: 120 })).toContain('150 seconds');
    expect(studySetupError({ ...valid, trial_number: 4 })).toContain('trial 1, 2, or 3');
  });

  it('allows exploratory captures only inside the 0.5 to 1.0 m development range', () => {
    const valid = {
      ...initialSetup(),
      participant_id: 'P-002',
      study_mode: 'exploratory' as const,
      distance_m: 0.55
    };
    expect(studySetupError(valid)).toBe('');
    expect(studySetupError({ ...valid, distance_m: 0.49 })).toContain('0.5 to 1.0');
    expect(studySetupError({ ...valid, distance_m: 1.01 })).toContain('0.5 to 1.0');
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
});
