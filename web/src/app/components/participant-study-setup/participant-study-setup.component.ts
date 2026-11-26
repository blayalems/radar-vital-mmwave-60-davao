import { ChangeDetectionStrategy, Component, OnDestroy, OnInit, computed, effect, inject, output, signal } from '@angular/core';
import { DecimalPipe, KeyValuePipe, UpperCasePipe } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { MatSnackBar } from '@angular/material/snack-bar';

import {
  BarrierType,
  CompletionCell,
  CompletionMatrix,
  CompletionParticipant,
  ParticipantProfile,
  ParticipantProfilesResponse,
  SetupState,
  StudyMode,
  StudyObjective,
  StudyProtocol,
  StudyProtocolResponse,
  StudyReadinessResponse,
  StudyScheduleEntry,
  StudyScheduleResponse
} from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';
import { StateService } from '../../services/state.service';

export const CONFIRMATORY_DISTANCES_M = [0.6, 0.8, 1.0, 2.0, 3.0] as const;
export const STUDY_TRIAL_NUMBERS = [1, 2, 3] as const;
export const CONFIRMATORY_DURATION_S = 150;
export const ACCLIMATIZATION_DURATION_S = 300;
export const NO_SUBJECT_TARGET_TRIALS = 72;
export const NO_SUBJECT_TARGET_DURATION_S = 10800;

export interface ConditionMatrixRow {
  order: number;
  conditionId: string;
  label: string;
  distanceM: number;
  barrierType: BarrierType;
  validTrials: number;
  meetsTrialFloor: boolean;
  cells: Array<{
    trial: number;
    status: string;
    validWindows: number | null;
    meetsWindowFloor: boolean | null;
    sessionId: string | null;
  }>;
}

interface AcclimatizationStorageState {
  remaining_s: number;
  running: boolean;
  updated_at_ms: number;
}

export function studyConditionId(_mode: StudyMode, distanceM: number, barrier: BarrierType): string {
  const centimetres = String(Math.round(distanceM * 100)).padStart(3, '0');
  return `d${centimetres}_${barrier}`;
}

export function parseStudyConditionId(conditionId: string): { distanceM: number; barrierType: BarrierType } | null {
  const match = /^d(\d{3})_(none|cardboard)$/.exec(String(conditionId || '').trim());
  if (!match) return null;
  return {
    distanceM: Number(match[1]) / 100,
    barrierType: match[2] as BarrierType
  };
}

export function formatConditionLabel(conditionId: string): string {
  const parsed = parseStudyConditionId(conditionId);
  if (!parsed) return conditionId;
  return `${parsed.distanceM.toFixed(1)} m · ${parsed.barrierType === 'cardboard' ? '3-mm cardboard' : 'None'}`;
}

export function defaultStudyConditionIds(): string[] {
  const ids: string[] = [];
  for (const dist of CONFIRMATORY_DISTANCES_M) {
    ids.push(studyConditionId('confirmatory', dist, 'none'));
    ids.push(studyConditionId('confirmatory', dist, 'cardboard'));
  }
  return ids;
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
    if (!CONFIRMATORY_DISTANCES_M.includes(setup.distance_m as 0.6 | 0.8 | 1.0 | 2.0 | 3.0)) {
      return 'Confirmatory distance must be 0.6, 0.8, 1.0, 2.0, or 3.0 m.';
    }
    if (setup.duration_s !== CONFIRMATORY_DURATION_S) return 'Confirmatory trials must run for 150 seconds.';
    return '';
  }
  if (setup.study_mode !== 'exploratory') return 'Select a study mode.';
  if (!Number.isFinite(setup.distance_m) || setup.distance_m < 0.5 || setup.distance_m > 3.0) {
    return 'Exploratory distance must be from 0.5 to 3.0 m.';
  }
  if (!Number.isFinite(setup.duration_s) || setup.duration_s < 1) return 'Choose a recording duration.';
  return '';
}

@Component({
  selector: 'app-participant-study-setup',
  imports: [
    DecimalPipe,
    KeyValuePipe,
    UpperCasePipe,
    FormsModule,
    MatButtonModule,
    MatCardModule,
    MatIconModule,
    MatInputModule,
    MatProgressBarModule,
    MatProgressSpinnerModule
  ],
  templateUrl: './participant-study-setup.component.html',
  styleUrl: './participant-study-setup.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class ParticipantStudySetupComponent implements OnInit, OnDestroy {
  protected readonly state = inject(StateService);
  private readonly api = inject(ApiService);
  private readonly snackBar = inject(MatSnackBar);

  readonly durationSelected = output<number>();
  readonly rosterValidityChanged = output<boolean>();
  readonly participants = signal<ParticipantProfile[]>([]);
  readonly completionMatrix = signal<CompletionMatrix | null>(null);
  readonly scheduleEntries = signal<StudyScheduleEntry[]>([]);
  readonly studyProtocol = signal<StudyProtocol | null>(null);
  readonly objectives = signal<StudyObjective[]>([]);
  readonly readiness = signal<StudyReadinessResponse | null>(null);
  readonly loading = signal(false);
  readonly creating = signal(false);
  readonly statusUpdating = signal(false);
  readonly attemptCreating = signal(false);
  readonly loadError = signal('');
  protected readonly confirmatoryDistances = CONFIRMATORY_DISTANCES_M;
  protected readonly trials = STUDY_TRIAL_NUMBERS;

  // 0.6 m Unobstructed MLX90614 5-min acclimatization timer state
  readonly acclimatizationRemainingS = signal(ACCLIMATIZATION_DURATION_S);
  readonly acclimatizationRunning = signal(false);
  private acclimatizationTimer: ReturnType<typeof setInterval> | null = null;
  private scheduleRequestSeq = 0;

  readonly isD060NoneSelected = computed(() => {
    const setup = this.state.setup();
    return Math.abs(Number(setup.distance_m) - 0.6) < 0.01 && setup.barrier_type === 'none';
  });

  readonly acclimatizationDisplay = computed(() => {
    const total = Math.max(0, Math.round(this.acclimatizationRemainingS()));
    const mins = Math.floor(total / 60);
    const secs = total % 60;
    return `${String(mins).padStart(2, '0')}:${String(secs).padStart(2, '0')}`;
  });

  readonly selectedParticipantCompletion = computed<CompletionParticipant | undefined>(() => {
    const pid = this.state.setup().participant_id;
    if (!pid) return undefined;
    return this.completionMatrix()?.participants?.[pid];
  });

  readonly conditionMatrixRows = computed<ConditionMatrixRow[]>(() => {
    const pid = this.state.setup().participant_id;
    if (!pid) return [];
    const participantCompletion = this.selectedParticipantCompletion();
    const cells: Record<string, CompletionCell> = participantCompletion?.cells || {};
    const condSummary = participantCompletion?.conditions_summary || {};
    const schedule = this.scheduleEntries().filter(entry => !entry.participant_id || entry.participant_id === pid);

    const orderedEntries: Array<{ order: number; conditionId: string }> = schedule.length > 0
      ? [...schedule]
          .sort((a, b) => (a.order ?? 0) - (b.order ?? 0))
          .map((entry, idx) => ({ order: entry.order || idx + 1, conditionId: entry.condition_id }))
      : defaultStudyConditionIds().map((cid, idx) => ({ order: idx + 1, conditionId: cid }));

    return orderedEntries.map(({ order, conditionId }) => {
      const parsed = parseStudyConditionId(conditionId);
      const trialCells = STUDY_TRIAL_NUMBERS.map(trial => {
        const key = `${conditionId}:t${trial}`;
        const cell = cells[key];
        const status = cell?.status || 'missing';
        const validWindows = cell?.valid_windows ?? null;
        const meetsWindowFloor = cell?.meets_window_floor ?? (validWindows !== null ? validWindows >= 15 : null);
        return {
          trial,
          status,
          validWindows,
          meetsWindowFloor,
          sessionId: cell?.session_id ?? null
        };
      });

      const summary = condSummary[conditionId];
      const computedValidTrials = summary?.valid_trials ?? trialCells.filter(c =>
        c.status === 'completed' && c.meetsWindowFloor !== false
      ).length;
      const meetsTrialFloor = summary?.meets_trial_floor ?? (computedValidTrials >= 2);

      return {
        order,
        conditionId,
        label: formatConditionLabel(conditionId),
        distanceM: parsed?.distanceM ?? 0.6,
        barrierType: parsed?.barrierType ?? 'none',
        validTrials: computedValidTrials,
        meetsTrialFloor,
        cells: trialCells
      };
    });
  });

  readonly noSubjectStats = computed(() => {
    const matrix = this.completionMatrix();
    const qualifiedCount = matrix?.no_subject_qualified_count ?? 0;
    const targetCount = matrix?.no_subject_expected ?? NO_SUBJECT_TARGET_TRIALS;
    const qualifiedDurationS = matrix?.no_subject_qualified_duration_s ?? (qualifiedCount * CONFIRMATORY_DURATION_S);
    const targetDurationS = matrix?.no_subject_target_duration_s ?? NO_SUBJECT_TARGET_DURATION_S;
    const progressPct = targetCount > 0 ? Math.min(100, Math.round((qualifiedCount / targetCount) * 100)) : 0;
    const frozenConfigLocked = this.studyProtocol()?.no_subject?.frozen_configuration != null;
    const reasons = matrix?.no_subject_unqualified_reasons || {};
    return {
      qualifiedCount,
      targetCount,
      qualifiedDurationS,
      targetDurationS,
      progressPct,
      frozenConfigLocked,
      reasons
    };
  });

  constructor() {
    effect(() => {
      const pid = this.state.setup().participant_id;
      const demo = this.state.demoSourceActive();
      this.restoreAcclimatizationTimer(pid, demo);
    });
  }

  ngOnInit(): void {
    this.updateCondition({});
    if (this.state.setup().study_mode === 'confirmatory') this.applyConfirmatoryDuration();
    void this.loadParticipants();
    void this.loadObjectives();
    void this.loadReadiness();
    void this.loadProtocol();
  }

  ngOnDestroy(): void {
    this.scheduleRequestSeq += 1;
    this.stopAcclimatizationInterval();
  }

  async loadProtocol(): Promise<void> {
    try {
      const api = this.api as Partial<ApiService>;
      const response = typeof api.loadStudyProtocol === 'function'
        ? await api.loadStudyProtocol()
        : (typeof api.request === 'function'
            ? await api.request<StudyProtocolResponse>('/api/study/protocol')
            : null);
      if (response?.protocol) {
        this.studyProtocol.set(response.protocol);
      }
    } catch (_) {
      this.studyProtocol.set(null);
    }
  }

  async loadSchedule(participantId: string): Promise<void> {
    const seq = ++this.scheduleRequestSeq;
    const targetId = String(participantId || '').trim();
    this.scheduleEntries.set([]);
    if (!targetId) {
      return;
    }
    try {
      const api = this.api as Partial<ApiService>;
      const response = typeof api.loadStudySchedule === 'function'
        ? await api.loadStudySchedule(targetId)
        : (typeof api.request === 'function'
            ? await api.request<StudyScheduleResponse>(`/api/study/schedule?participant_id=${encodeURIComponent(targetId)}`)
            : null);
      const activePid = String(this.state.setup().participant_id || '').trim();
      if (seq !== this.scheduleRequestSeq || (activePid && activePid !== targetId)) {
        return;
      }
      this.scheduleEntries.set(response?.entries || []);
    } catch (_) {
      const activePid = String(this.state.setup().participant_id || '').trim();
      if (seq !== this.scheduleRequestSeq || (activePid && activePid !== targetId)) {
        return;
      }
      this.scheduleEntries.set([]);
    }
  }

  async loadReadiness(): Promise<void> {
    try {
      if (typeof (this.api as Partial<ApiService>).loadStudyReadiness !== 'function') return;
      this.readiness.set(await this.api.loadStudyReadiness());
    } catch (_) {
      this.readiness.set({
        ok: false,
        schema_version: 'rvt-collection-readiness-v1',
        authorized: false,
        protocol_state: 'unknown',
        plan_status: 'unknown',
        authorization_record_present: false,
        blockers: ['readiness_unavailable'],
        unresolved_withdrawal_count: 0
      });
    }
  }

  collectionAuthorized(): boolean {
    return this.readiness()?.authorized === true;
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
        .filter(item => item && item.participant_id && item.display_code);
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
      } else if (this.state.setup().participant_id) {
        void this.loadSchedule(this.state.setup().participant_id);
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
    if (this.creating() || !this.collectionAuthorized()) return;
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
    if (this.attemptCreating() || !this.collectionAuthorized()) return;
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
    void this.loadSchedule(participant.participant_id);
  }

  selectMatrixCell(row: ConditionMatrixRow, trial: number): void {
    this.updateSetup({
      distance_m: row.distanceM,
      barrier_type: row.barrierType,
      trial_number: trial,
      condition_id: row.conditionId
    });
    if (this.state.setup().study_mode === 'confirmatory') {
      this.applyConfirmatoryDuration();
    }
    this.state.triggerHaptic('tap');
  }

  cellStatusIcon(status: string): string {
    if (status === 'completed') return 'check_circle';
    if (status === 'aborted' || status === 'failed_start' || status === 'no_output' || status === 'invalid') return 'warning';
    return 'radio_button_unchecked';
  }

  // Acclimatization timer controls (strictly non-blocking)
  startAcclimatizationTimer(): void {
    if (this.acclimatizationRunning()) return;
    if (this.acclimatizationRemainingS() <= 0) {
      this.acclimatizationRemainingS.set(ACCLIMATIZATION_DURATION_S);
    }
    this.acclimatizationRunning.set(true);
    this.persistAcclimatizationTimer();
    this.stopAcclimatizationInterval();
    this.acclimatizationTimer = setInterval(() => {
      const next = Math.max(0, this.acclimatizationRemainingS() - 1);
      this.acclimatizationRemainingS.set(next);
      if (next <= 0) {
        this.acclimatizationRunning.set(false);
        this.stopAcclimatizationInterval();
      }
      this.persistAcclimatizationTimer();
    }, 1000);
    this.state.triggerHaptic('tap');
  }

  pauseAcclimatizationTimer(): void {
    this.acclimatizationRunning.set(false);
    this.stopAcclimatizationInterval();
    this.persistAcclimatizationTimer();
    this.state.triggerHaptic('tap');
  }

  resetAcclimatizationTimer(): void {
    this.acclimatizationRunning.set(false);
    this.stopAcclimatizationInterval();
    this.acclimatizationRemainingS.set(ACCLIMATIZATION_DURATION_S);
    this.persistAcclimatizationTimer();
    this.state.triggerHaptic('tap');
  }

  private stopAcclimatizationInterval(): void {
    if (this.acclimatizationTimer !== null) {
      clearInterval(this.acclimatizationTimer);
      this.acclimatizationTimer = null;
    }
  }

  private acclimatizationStorageKey(participantId = this.state.setup().participant_id, demo = this.state.demoSourceActive()): string {
    const prefix = demo ? 'demo:' : '';
    const pid = participantId.trim() || 'default';
    return `${prefix}rvt-acclimatization:${pid}`;
  }

  private persistAcclimatizationTimer(): void {
    try {
      if (typeof sessionStorage === 'undefined') return;
      const payload: AcclimatizationStorageState = {
        remaining_s: this.acclimatizationRemainingS(),
        running: this.acclimatizationRunning(),
        updated_at_ms: Date.now()
      };
      sessionStorage.setItem(this.acclimatizationStorageKey(), JSON.stringify(payload));
    } catch (_) {}
  }

  private restoreAcclimatizationTimer(participantId: string, demo: boolean): void {
    this.stopAcclimatizationInterval();
    this.acclimatizationRunning.set(false);
    try {
      if (typeof sessionStorage === 'undefined') {
        this.acclimatizationRemainingS.set(ACCLIMATIZATION_DURATION_S);
        return;
      }
      const raw = sessionStorage.getItem(this.acclimatizationStorageKey(participantId, demo));
      if (!raw) {
        this.acclimatizationRemainingS.set(ACCLIMATIZATION_DURATION_S);
        return;
      }
      const parsed = JSON.parse(raw) as Partial<AcclimatizationStorageState>;
      const baseRemaining = Number(parsed.remaining_s);
      if (!Number.isFinite(baseRemaining)) {
        this.acclimatizationRemainingS.set(ACCLIMATIZATION_DURATION_S);
        return;
      }
      if (parsed.running && Number.isFinite(parsed.updated_at_ms)) {
        const elapsed = Math.max(0, Math.floor((Date.now() - Number(parsed.updated_at_ms)) / 1000));
        const adjusted = Math.max(0, baseRemaining - elapsed);
        this.acclimatizationRemainingS.set(adjusted);
        if (adjusted > 0) {
          this.startAcclimatizationTimer();
        }
      } else {
        this.acclimatizationRemainingS.set(Math.max(0, Math.min(ACCLIMATIZATION_DURATION_S, baseRemaining)));
      }
    } catch (_) {
      this.acclimatizationRemainingS.set(ACCLIMATIZATION_DURATION_S);
    }
  }

  setStudyMode(mode: StudyMode): void {
    if (mode === 'confirmatory' && !this.collectionAuthorized()) return;
    const distance = mode === 'confirmatory'
      ? (CONFIRMATORY_DISTANCES_M.includes(this.state.setup().distance_m as 0.6 | 0.8 | 1.0 | 2.0 | 3.0)
          ? this.state.setup().distance_m
          : 0.6)
      : Math.max(0.5, Math.min(3.0, this.state.setup().distance_m || 0.6));
    this.updateCondition({ study_mode: mode, distance_m: distance });
    if (mode === 'confirmatory') this.applyConfirmatoryDuration();
    this.state.triggerHaptic('tap');
  }

  setDistance(value: unknown): void {
    const raw = Number(value);
    const distance = this.state.setup().study_mode === 'confirmatory'
      ? raw
      : Math.round(Math.max(0.5, Math.min(3.0, raw || 0.5)) * 100) / 100;
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
    this.scheduleRequestSeq += 1;
    this.updateSetup({ participant_id: '', subject_label: '' });
    this.scheduleEntries.set([]);
  }
}
