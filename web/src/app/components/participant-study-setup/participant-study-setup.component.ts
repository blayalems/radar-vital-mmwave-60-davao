import { ChangeDetectionStrategy, Component, OnInit, inject, output, signal } from '@angular/core';
import { DecimalPipe, UpperCasePipe } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { MatSnackBar } from '@angular/material/snack-bar';

import {
  BarrierType,
  CompletionMatrix,
  CompletionParticipant,
  ParticipantProfile,
  ParticipantProfilesResponse,
  SetupState,
  StudyMode,
  StudyObjective
} from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';
import { StateService } from '../../services/state.service';

export const CONFIRMATORY_DISTANCES_M = [0.6, 0.8, 1.0] as const;
export const STUDY_TRIAL_NUMBERS = [1, 2, 3] as const;
export const CONFIRMATORY_DURATION_S = 150;

export function studyConditionId(_mode: StudyMode, distanceM: number, barrier: BarrierType): string {
  const centimetres = String(Math.round(distanceM * 100)).padStart(3, '0');
  return `d${centimetres}_${barrier}`;
}

export function studyTrialId(setup: Pick<SetupState, 'participant_id' | 'condition_id' | 'trial_number'>): string {
  return `${setup.participant_id}-${setup.condition_id}-t${setup.trial_number}`;
}

export function studySetupError(
  setup: Pick<SetupState, 'participant_id' | 'study_mode' | 'distance_m' | 'barrier_type' | 'trial_number' | 'duration_s'>
): string {
  if (!setup.participant_id.trim()) return 'Select a coded participant profile.';
  if (!['none', 'cardboard'].includes(setup.barrier_type)) return 'Select a barrier condition.';
  if (!STUDY_TRIAL_NUMBERS.includes(setup.trial_number as 1 | 2 | 3)) return 'Select trial 1, 2, or 3.';
  if (setup.study_mode === 'confirmatory') {
    if (!CONFIRMATORY_DISTANCES_M.includes(setup.distance_m as 0.6 | 0.8 | 1.0)) {
      return 'Confirmatory distance must be 0.6, 0.8, or 1.0 m.';
    }
    if (setup.duration_s !== CONFIRMATORY_DURATION_S) return 'Confirmatory trials must run for 150 seconds.';
    return '';
  }
  if (setup.study_mode !== 'exploratory') return 'Select a study mode.';
  if (!Number.isFinite(setup.distance_m) || setup.distance_m < 0.5 || setup.distance_m > 1.0) {
    return 'Exploratory distance must be from 0.5 to 1.0 m.';
  }
  if (!Number.isFinite(setup.duration_s) || setup.duration_s < 1) return 'Choose a recording duration.';
  return '';
}

@Component({
  selector: 'app-participant-study-setup',
  imports: [
    DecimalPipe,
    UpperCasePipe,
    FormsModule,
    MatButtonModule,
    MatCardModule,
    MatIconModule,
    MatInputModule,
    MatProgressSpinnerModule
  ],
  templateUrl: './participant-study-setup.component.html',
  styleUrl: './participant-study-setup.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class ParticipantStudySetupComponent implements OnInit {
  protected readonly state = inject(StateService);
  private readonly api = inject(ApiService);
  private readonly snackBar = inject(MatSnackBar);

  readonly durationSelected = output<number>();
  readonly rosterValidityChanged = output<boolean>();
  readonly participants = signal<ParticipantProfile[]>([]);
  readonly completionMatrix = signal<CompletionMatrix | null>(null);
  readonly objectives = signal<StudyObjective[]>([]);
  readonly loading = signal(false);
  readonly creating = signal(false);
  readonly statusUpdating = signal(false);
  readonly attemptCreating = signal(false);
  readonly loadError = signal('');
  protected readonly confirmatoryDistances = CONFIRMATORY_DISTANCES_M;
  protected readonly trials = STUDY_TRIAL_NUMBERS;

  ngOnInit(): void {
    this.updateCondition({});
    if (this.state.setup().study_mode === 'confirmatory') this.applyConfirmatoryDuration();
    void this.loadParticipants();
    void this.loadObjectives();
  }

  async loadObjectives(): Promise<void> {
    try {
      if (typeof (this.api as Partial<ApiService>).loadStudyObjectives !== 'function') return;
      const response = await this.api.loadStudyObjectives();
      this.objectives.set(response.objectives || []);
    } catch (_) {
      this.objectives.set([]);
    }
  }

  async loadParticipants(): Promise<void> {
    this.loading.set(true);
    this.loadError.set('');
    this.rosterValidityChanged.emit(false);
    try {
      const response = typeof (this.api as Partial<ApiService>).loadParticipants === 'function'
        ? await this.api.loadParticipants()
        : await this.api.request<ParticipantProfilesResponse>('/api/participants');
      const participants = (response.participants ?? response.items ?? [])
        .filter(item => item && item.participant_id && item.display_code)
        .slice(0, 41);
      this.participants.set(participants);
      if (response.completion_matrix) {
        this.completionMatrix.set(response.completion_matrix);
      } else {
        try {
          if (typeof (this.api as Partial<ApiService>).loadCompletionMatrix === 'function') {
            this.completionMatrix.set(await this.api.loadCompletionMatrix());
          }
        } catch (_) {
          this.completionMatrix.set(null);
        }
      }
      const selected = this.state.setup().participant_id;
      const selectedParticipant = participants.find(item => item.participant_id === selected);
      if (selected && (!selectedParticipant || (selectedParticipant.status !== 'active' && selectedParticipant.participant_id !== 'P-DEMO'))) {
        this.clearParticipantSelection();
      }
      const demo = participants.find(item => item.participant_id === 'P-DEMO');
      if (!this.state.setup().participant_id && demo && this.state.demoSourceActive()) {
        this.selectParticipant(demo);
      }
      this.rosterValidityChanged.emit(this.hasActiveSelection());
    } catch (error: unknown) {
      this.participants.set([]);
      this.clearParticipantSelection();
      this.loadError.set(error instanceof Error ? error.message : 'Participant profiles are unavailable.');
      this.rosterValidityChanged.emit(false);
    } finally {
      this.loading.set(false);
    }
  }

  async createParticipant(): Promise<void> {
    if (this.creating() || this.realParticipantCount() >= 40) return;
    this.creating.set(true);
    try {
      const response = typeof (this.api as Partial<ApiService>).createParticipant === 'function'
        ? await this.api.createParticipant()
        : await this.api.request<ParticipantProfile | { participant: ParticipantProfile } | { profile: ParticipantProfile }>(
          '/api/participants',
          { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({}) }
        );
      const participant = 'participant' in response
        ? response.participant
        : ('profile' in response ? response.profile : response);
      if (!participant?.participant_id || !participant.display_code) {
        throw new Error('Trainer returned an invalid participant profile.');
      }
      this.participants.update(items => [...items.filter(item => item.participant_id !== participant.participant_id), participant]);
      this.selectParticipant(participant);
      this.snackBar.open(`${participant.display_code} created and selected.`, 'Dismiss', { duration: 3500 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Participant profile could not be created.', 'Dismiss', { duration: 6000 });
    } finally {
      this.creating.set(false);
    }
  }

  async updateStatus(status: 'active' | 'completed' | 'withdrawn'): Promise<void> {
    const participant = this.selectedParticipant();
    if (!participant || this.statusUpdating()) return;
    this.statusUpdating.set(true);
    try {
      const response = await this.api.updateParticipantStatus(participant.participant_id, status, {
        actor: 'dashboard-operator',
        reason: `Study roster status changed to ${status}`
      });
      this.participants.update(items => items.map(item =>
        item.participant_id === response.profile.participant_id ? response.profile : item
      ));
      if (status === 'withdrawn') this.clearParticipantSelection();
      this.rosterValidityChanged.emit(this.hasActiveSelection());
      this.snackBar.open(`${participant.display_code} marked ${status}.`, 'Dismiss', { duration: 3500 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Participant status could not be updated.', 'Dismiss', { duration: 6000 });
    } finally {
      this.statusUpdating.set(false);
    }
  }

  async recordNoSubjectAttempt(): Promise<void> {
    if (this.attemptCreating()) return;
    const setup = this.state.setup();
    this.attemptCreating.set(true);
    try {
      await this.api.createProtocolAttempt({
        attempt_type: 'no_subject',
        condition_id: setup.condition_id,
        trial_number: setup.trial_number,
        status: 'no_output',
        duration_s: setup.duration_s,
        actor: 'dashboard-operator',
        reason: 'No-subject false-alarm control attempt'
      });
      this.completionMatrix.set(await this.api.loadCompletionMatrix());
      this.snackBar.open('No-subject attempt logged. It counts toward the attempted ledger; a session-backed 150-second evidence row is required for the qualified denominator.', 'Dismiss', { duration: 5500 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'No-subject attempt could not be recorded.', 'Dismiss', { duration: 6000 });
    } finally {
      this.attemptCreating.set(false);
    }
  }

  selectParticipant(participant: ParticipantProfile): void {
    if (participant.status !== 'active' && participant.participant_id !== 'P-DEMO') return;
    this.updateSetup({
      participant_id: participant.participant_id,
      subject_label: participant.display_code
    });
    this.rosterValidityChanged.emit(true);
    this.state.triggerHaptic('tap');
  }

  setStudyMode(mode: StudyMode): void {
    const distance = mode === 'confirmatory'
      ? (CONFIRMATORY_DISTANCES_M.includes(this.state.setup().distance_m as 0.6 | 0.8 | 1.0)
          ? this.state.setup().distance_m
          : 0.6)
      : Math.max(0.5, Math.min(1, this.state.setup().distance_m || 0.6));
    this.updateCondition({ study_mode: mode, distance_m: distance });
    if (mode === 'confirmatory') this.applyConfirmatoryDuration();
    this.state.triggerHaptic('tap');
  }

  setDistance(value: unknown): void {
    const raw = Number(value);
    const distance = this.state.setup().study_mode === 'confirmatory'
      ? raw
      : Math.round(Math.max(0.5, Math.min(1, raw || 0.5)) * 100) / 100;
    this.updateCondition({ distance_m: distance });
  }

  setBarrier(barrier: BarrierType): void {
    this.updateCondition({ barrier_type: barrier });
    this.state.triggerHaptic('tap');
  }

  setTrial(trial: number): void {
    this.updateSetup({ trial_number: trial });
    this.state.triggerHaptic('tap');
  }

  selectedParticipant(): ParticipantProfile | undefined {
    return this.participants().find(item => item.participant_id === this.state.setup().participant_id);
  }

  completionFor(participantId: string): CompletionParticipant | undefined {
    return this.completionMatrix()?.participants?.[participantId];
  }

  realParticipantCount(): number {
    return this.participants().filter(item => item.participant_id !== 'P-DEMO').length;
  }

  private applyConfirmatoryDuration(): void {
    this.updateSetup({ duration_s: CONFIRMATORY_DURATION_S, customDuration: 150, customUnit: 's' });
    this.durationSelected.emit(CONFIRMATORY_DURATION_S);
  }

  private updateCondition(change: Partial<Pick<SetupState, 'study_mode' | 'distance_m' | 'barrier_type'>>): void {
    const setup = { ...this.state.setup(), ...change };
    this.updateSetup({
      ...change,
      condition_id: studyConditionId(setup.study_mode, setup.distance_m, setup.barrier_type)
    });
  }

  private updateSetup(change: Partial<SetupState>): void {
    this.state.setup.update(setup => ({ ...setup, ...change }));
  }

  private hasActiveSelection(): boolean {
    const selected = this.state.setup().participant_id;
    return Boolean(selected && this.participants().some(
      participant => participant.participant_id === selected && (participant.status === 'active' || participant.participant_id === 'P-DEMO')
    ));
  }

  private clearParticipantSelection(): void {
    this.updateSetup({ participant_id: '', subject_label: '' });
  }
}
