import { Injectable, signal } from '@angular/core';
import { ControlStatus, PreflightCheck, SetupState, SessionRecord, SessionSignoff } from '../../models/rvt.models';

@Injectable({
  providedIn: 'root'
})
export class SessionStore {
  ctlOn = signal<boolean>(false);
  ctlStatus = signal<ControlStatus | null>(null);
  ctlStopPending = signal<boolean>(false);
  paused = signal<boolean>(false);

  activeSubjectProfileId = signal<string>('adult_default');

  setup = signal<SetupState>({
    duration_s: 30,
    customDuration: 30,
    customUnit: 's',
    radar_port: 'COM10',
    ble_address: '10:22:33:9E:8F:63',
    ble_profile: 'ailink_oximeter',
    notify_char: '0000ffe2-0000-1000-8000-00805f9b34fb',
    subject_label: '',
    operator_label: '',
    station_label: 'Lab · Station 3',
    subject_profile_id: 'adult_default',
    participant_id: '',
    study_mode: 'confirmatory',
    condition_id: 'd060_none',
    distance_m: 0.6,
    barrier_type: 'none',
    trial_number: 1,
    skip_countdown: false,
    model_family: 'none',
    model_bundle: ''
  });

  sessionNotes = signal<Record<string, string>>({});
  sessionSignoffs = signal<Record<string, SessionSignoff>>({});
  sessionItems = signal<SessionRecord[]>([]);
  currentSessionId = signal<string | null>(null);
  sessionActive = signal<boolean>(false);
  preflightChecks = signal<PreflightCheck[]>([]);
  preflightRunning = signal<boolean>(false);
  preflightUpdatedAtMs = signal<number | null>(null);
}
