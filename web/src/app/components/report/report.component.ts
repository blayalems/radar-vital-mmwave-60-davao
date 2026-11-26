import {
  AfterViewInit,
  ChangeDetectionStrategy,
  ChangeDetectorRef,
  Component,
  ElementRef,
  OnDestroy,
  OnInit,
  ViewChild,
  computed,
  effect,
  inject,
  signal
} from '@angular/core';
import { DatePipe, DecimalPipe } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { firstValueFrom } from 'rxjs';

import { MatButtonModule } from '@angular/material/button';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatCardModule } from '@angular/material/card';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatListModule } from '@angular/material/list';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatSelectModule } from '@angular/material/select';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';

import {
  DownloadRecord,
  ReferenceObservation,
  ReferenceObservationInput,
  SessionRecord,
  SessionSignoff,
  SessionTagsResponse,
  SessionTrainingStatus,
  SessionPredictionResponse,
  StudyAnalysisJob,
  StudyObjectiveExportResponse,
  StudyObjectiveReport,
  StudyReferencesResponse
} from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';
import { ChartRenderSchedulerService } from '../../services/chart-render-scheduler.service';
import { StateService } from '../../services/state.service';
import { ConfirmDialogComponent } from '../confirm-dialog/confirm-dialog.component';
import { ReportRequestCoordinator } from './report-request-coordinator.service';
import {
  ObjectiveExportResult,
  buildFallbackObjectiveExport,
  computeBreathCountRate,
  extractObjectiveEvidenceLines,
  extractSessionConditionInfo,
  isD060NoneCondition,
  summarizeRrObservers
} from './report-thesis-helpers';

@Component({
  selector: 'app-report',
  imports: [
    DatePipe,
    DecimalPipe,
    FormsModule,
    MatButtonModule,
    MatButtonToggleModule,
    MatCardModule,
    MatDialogModule,
    MatFormFieldModule,
    MatInputModule,
    MatListModule,
    MatProgressBarModule,
    MatSelectModule,
    MatSnackBarModule,
    MatIconModule
  ],
  templateUrl: './report.component.html',
  styleUrl: './report.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class ReportComponent implements OnInit, AfterViewInit, OnDestroy {
  protected readonly state = inject(StateService);
  protected readonly api = inject(ApiService);
  private readonly requests = inject(ReportRequestCoordinator);
  private readonly cdr = inject(ChangeDetectorRef);
  private readonly snackBar = inject(MatSnackBar);
  private readonly dialog = inject(MatDialog, { optional: true });
  private readonly renderScheduler = inject(ChartRenderSchedulerService, { optional: true });

  private resizeObserver: ResizeObserver | null = null;
  private viewReady = false;

  constructor() {
    effect(() => {
      this.state.theme();
      this.sessionDataRowsSignal();
      this.compareRowsSignal();
      this.requestTrendDraw();
    });

    effect(() => {
      const summary = this.rrObserverSummary();
      if (summary.unblinded && summary.suggestedMeanBpm !== null && !this.adjudicationFinalValueInput()) {
        this.adjudicationFinalValueInput.set(String(summary.suggestedMeanBpm));
      }
    });
  }

  @ViewChild('hrReportCanvas', { static: false }) hrReportCanvas?: ElementRef<HTMLCanvasElement>;
  @ViewChild('rrReportCanvas', { static: false }) rrReportCanvas?: ElementRef<HTMLCanvasElement>;

  // Signal-backed state (with property getters/setters for seamless test/ngModel compatibility)
  readonly sessionsSignal = signal<SessionRecord[]>([]);
  get sessions(): SessionRecord[] { return this.sessionsSignal(); }
  set sessions(v: SessionRecord[]) { this.sessionsSignal.set(v); }

  readonly selectedSessionIdSignal = signal('');
  get selectedSessionId(): string { return this.selectedSessionIdSignal(); }
  set selectedSessionId(v: string) { this.selectedSessionIdSignal.set(v); }

  readonly selectedSessionSignal = signal<SessionRecord | null>(null);
  get selectedSession(): SessionRecord | null { return this.selectedSessionSignal(); }
  set selectedSession(v: SessionRecord | null) { this.selectedSessionSignal.set(v); }

  readonly selectedSummarySignal = signal<SessionRecord | null>(null);
  get selectedSummary(): SessionRecord | null { return this.selectedSummarySignal(); }
  set selectedSummary(v: SessionRecord | null) { this.selectedSummarySignal.set(v); }

  readonly comparisonSignal = signal<{ selected?: SessionRecord | null; previous?: SessionRecord | null; best?: SessionRecord | null } | null>(null);
  get comparison() { return this.comparisonSignal(); }
  set comparison(v: { selected?: SessionRecord | null; previous?: SessionRecord | null; best?: SessionRecord | null } | null) { this.comparisonSignal.set(v); }

  readonly compareSessionIdSignal = signal('');
  get compareSessionId(): string { return this.compareSessionIdSignal(); }
  set compareSessionId(v: string) { this.compareSessionIdSignal.set(v); }

  readonly compareSummarySignal = signal<SessionRecord | null>(null);
  get compareSummary(): SessionRecord | null { return this.compareSummarySignal(); }
  set compareSummary(v: SessionRecord | null) { this.compareSummarySignal.set(v); }

  readonly compareRowsSignal = signal<Array<Record<string, number | string | null>>>([]);
  get compareRows() { return this.compareRowsSignal(); }
  set compareRows(v: Array<Record<string, number | string | null>>) { this.compareRowsSignal.set(v); }

  readonly compareLoadingSignal = signal(false);
  get compareLoading(): boolean { return this.compareLoadingSignal(); }
  set compareLoading(v: boolean) { this.compareLoadingSignal.set(v); }

  readonly analysisStatusSignal = signal<{ status?: string; progress_pct?: number; last_line?: string } | null>(null);
  get analysisStatus() { return this.analysisStatusSignal(); }
  set analysisStatus(v: { status?: string; progress_pct?: number; last_line?: string } | null) { this.analysisStatusSignal.set(v); }

  readonly trainingStatusSignal = signal<SessionTrainingStatus | null>(null);
  get trainingStatus() { return this.trainingStatusSignal(); }
  set trainingStatus(v: SessionTrainingStatus | null) { this.trainingStatusSignal.set(v); }

  readonly predictionSignal = signal<SessionPredictionResponse | null>(null);
  get prediction() { return this.predictionSignal(); }
  set prediction(v: SessionPredictionResponse | null) { this.predictionSignal.set(v); }

  readonly sessionTagsInputSignal = signal('');
  get sessionTagsInput(): string { return this.sessionTagsInputSignal(); }
  set sessionTagsInput(v: string) { this.sessionTagsInputSignal.set(v); }

  readonly modelEvidenceLoadingSignal = signal(false);
  get modelEvidenceLoading(): boolean { return this.modelEvidenceLoadingSignal(); }
  set modelEvidenceLoading(v: boolean) { this.modelEvidenceLoadingSignal.set(v); }

  readonly objectiveReportsSignal = signal<StudyObjectiveReport[]>([]);
  get objectiveReports(): StudyObjectiveReport[] { return this.objectiveReportsSignal(); }
  set objectiveReports(v: StudyObjectiveReport[]) { this.objectiveReportsSignal.set(v); }

  readonly objectiveReportsLoadingSignal = signal(false);
  get objectiveReportsLoading(): boolean { return this.objectiveReportsLoadingSignal(); }
  set objectiveReportsLoading(v: boolean) { this.objectiveReportsLoadingSignal.set(v); }

  readonly objectiveReportsErrorSignal = signal('');
  get objectiveReportsError(): string { return this.objectiveReportsErrorSignal(); }
  set objectiveReportsError(v: string) { this.objectiveReportsErrorSignal.set(v); }

  readonly studyAnalysisJobSignal = signal<StudyAnalysisJob | null>(null);
  get studyAnalysisJob(): StudyAnalysisJob | null { return this.studyAnalysisJobSignal(); }
  set studyAnalysisJob(v: StudyAnalysisJob | null) { this.studyAnalysisJobSignal.set(v); }

  readonly studyAnalysisLoadingSignal = signal(false);
  get studyAnalysisLoading(): boolean { return this.studyAnalysisLoadingSignal(); }
  set studyAnalysisLoading(v: boolean) { this.studyAnalysisLoadingSignal.set(v); }

  readonly studyAnalysisErrorSignal = signal('');
  get studyAnalysisError(): string { return this.studyAnalysisErrorSignal(); }
  set studyAnalysisError(v: string) { this.studyAnalysisErrorSignal.set(v); }

  readonly studyAnalysisObjectiveSignal = signal<'objective_1_rr' | 'objective_4_hr'>('objective_1_rr');
  get studyAnalysisObjective(): 'objective_1_rr' | 'objective_4_hr' { return this.studyAnalysisObjectiveSignal(); }
  set studyAnalysisObjective(v: 'objective_1_rr' | 'objective_4_hr') { this.studyAnalysisObjectiveSignal.set(v); }

  readonly studyAnalysisFamilySignal = signal<'gradient_boosting' | 'cnn_1d'>('gradient_boosting');
  get studyAnalysisFamily(): 'gradient_boosting' | 'cnn_1d' { return this.studyAnalysisFamilySignal(); }
  set studyAnalysisFamily(v: 'gradient_boosting' | 'cnn_1d') { this.studyAnalysisFamilySignal.set(v); }

  readonly sessionDataRowsSignal = signal<Array<Record<string, number | string | null>>>([]);
  get sessionDataRows() { return this.sessionDataRowsSignal(); }
  set sessionDataRows(v: Array<Record<string, number | string | null>>) { this.sessionDataRowsSignal.set(v); }

  readonly sessionsLoadingSignal = signal(false);
  get sessionsLoading(): boolean { return this.sessionsLoadingSignal(); }
  set sessionsLoading(v: boolean) { this.sessionsLoadingSignal.set(v); }

  readonly sessionsErrorSignal = signal('');
  get sessionsError(): string { return this.sessionsErrorSignal(); }
  set sessionsError(v: string) { this.sessionsErrorSignal.set(v); }

  readonly summaryLoadingSignal = signal(false);
  get summaryLoading(): boolean { return this.summaryLoadingSignal(); }
  set summaryLoading(v: boolean) { this.summaryLoadingSignal.set(v); }

  readonly summaryErrorSignal = signal('');
  get summaryError(): string { return this.summaryErrorSignal(); }
  set summaryError(v: string) { this.summaryErrorSignal.set(v); }

  readonly sessionNotesInputSignal = signal('');
  get sessionNotesInput(): string { return this.sessionNotesInputSignal(); }
  set sessionNotesInput(v: string) { this.sessionNotesInputSignal.set(v); }

  readonly signoffSignal = signal<SessionSignoff>({
    session_id: '',
    operator_name: '',
    initials: '',
    validation_comment: '',
    signed_at: null
  });
  get signoff(): SessionSignoff { return this.signoffSignal(); }
  set signoff(v: SessionSignoff) {
    this.signoffSignal.set({
      session_id: v?.session_id ?? '',
      operator_name: v?.operator_name ?? '',
      initials: v?.initials ?? '',
      validation_comment: v?.validation_comment ?? '',
      signed_at: v?.signed_at ?? null,
      updated_at: v?.updated_at ?? null,
      schema_version: v?.schema_version
    });
  }

  readonly artifactDownloading = signal(false);
  readonly analysisStatusLoading = signal(false);
  readonly objectiveExporting = signal(false);

  // Reference observations & blinded dual-observer RR adjudication state
  readonly referencesSignal = signal<ReferenceObservation[]>([]);
  readonly rrAdjudicationSignal = signal<Record<string, unknown> | null>(null);
  readonly referencesLoading = signal(false);
  readonly referencesSaving = signal(false);
  readonly referencesError = signal('');
  readonly adjudicating = signal(false);

  readonly rrObserverIdInput = signal('');
  readonly rrEntryMode = signal<'rate' | 'count'>('rate');
  readonly rrRateInput = signal('');
  readonly rrBreathCountInput = signal('');
  readonly rrTrialDurationInput = signal('150');
  readonly adjudicationFinalValueInput = signal('');
  readonly adjudicationRationaleInput = signal('');

  readonly tempValueInput = signal('');
  readonly tempDeviceIdInput = signal('MLX90614');
  readonly tempCalibrationIdInput = signal('');
  readonly tempUncertaintyInput = signal('0.5');

  readonly hrRefValueInput = signal('');
  readonly hrRefSourceInput = signal('');

  // Memoized computed views
  readonly recentSessionChips = computed(() =>
    this.sessionsSignal().slice(0, 4).map(session => ({
      session,
      label: this.sessionChipLabel(session)
    }))
  );

  readonly reportRecordSignal = computed<SessionRecord>(() =>
    this.selectedSummarySignal() || this.selectedSessionSignal() || { session_id: '' }
  );

  readonly reportVerdictSignal = computed(() => {
    const raw = this.reportRecordSignal()?.verdict;
    return String(typeof raw === 'object' && raw !== null ? raw['verdict'] || raw['readiness_kind'] || '' : raw || '').toLowerCase();
  });

  readonly outcomeTitleSignal = computed(() => {
    if (this.selectedSummarySignal()?.sandbox) return 'DEMO - Simulated data only';
    const verdict = this.reportVerdictSignal();
    if (verdict === 'ready' || verdict === 'pass') return 'READY - Analysis passed';
    if (verdict === 'conditional' || verdict === 'warn') return 'CONDITIONAL - Review required';
    if (verdict === 'unready' || verdict === 'fail' || verdict === 'not_ready') return 'NOT READY - Review required';
    return verdict ? `${verdict.toUpperCase()} - Review required` : 'PENDING - No analysis verdict';
  });

  readonly outcomeDescriptionSignal = computed(() => {
    const message = this.selectedSummarySignal()?.['message'];
    if (typeof message === 'string' && message.trim()) return message;
    const summary = this.selectedSessionSignal()?.summary;
    if (summary) return summary;
    const verdict = this.reportVerdictSignal();
    if (verdict === 'ready' || verdict === 'pass') {
      return 'All available quality gates passed for this recording.';
    }
    return 'Load the recorded analysis summary and source artifacts before interpreting this session.';
  });

  readonly outcomeIconSignal = computed(() => {
    const verdict = this.reportVerdictSignal();
    return verdict === 'ready' || verdict === 'pass' ? 'check_circle' : 'warning';
  });

  readonly outcomeBadgeSignal = computed(() =>
    (this.selectedSummarySignal()?.sandbox ? 'demo' : this.reportVerdictSignal() || 'pending').toUpperCase()
  );

  readonly signalQualitySignal = computed(() => this.summaryObject('signal_quality'));
  readonly hrMetricsSignal = computed(() => this.summaryObject('hr_metrics'));
  readonly rrMetricsSignal = computed(() => this.summaryObject('rr_metrics'));

  readonly qualityStatCardsSignal = computed(() => {
    const sq = this.signalQualitySignal();
    return [
      { label: 'PQI lock', value: this.qualityStat(sq, 'pqi_lock_pct', 1, '%'), sub: 'locked samples' },
      { label: 'Quality score', value: this.qualityStat(sq, 'session_quality_score', 1), sub: 'session rollup' },
      { label: 'Consistency', value: this.qualityStat(sq, 'internal_consistency_score', 1), sub: 'internal agreement' },
      { label: 'Coverage locked', value: this.qualityStat(sq, 'coverage_locked', 1, '%'), sub: 'usable window' },
      { label: 'Settling', value: this.qualityStat(sq, 'coverage_settling', 1, '%'), sub: 'settling window' }
    ];
  });

  readonly qualityGatesSignal = computed(() => {
    const gates = this.summaryObject('gates');
    if (!gates) return [];
    return Object.entries(gates)
      .filter(([, value]) => value && typeof value === 'object')
      .map(([id, value]) => {
        const gate = value as Record<string, unknown>;
        const passed = typeof gate['passed'] === 'boolean' ? (gate['passed'] as boolean) : null;
        const label = `${id.charAt(0).toUpperCase()}${id.slice(1)} gate`;
        return { id, label, passed, status: String(gate['status'] ?? (passed === null ? 'n/a' : passed ? 'pass' : 'fail')) };
      });
  });

  readonly verdictCategoriesSignal = computed(() => {
    const mlVerdict = this.summaryObject('ml_readiness_verdict');
    const verdict = this.reportRecordSignal()?.verdict;
    const fromVerdictObj = typeof verdict === 'object' && verdict !== null
      ? (verdict as Record<string, unknown>)['categories']
      : null;
    const categories = Array.isArray(mlVerdict?.['categories']) ? mlVerdict['categories'] : fromVerdictObj;
    if (!Array.isArray(categories)) return [];
    return categories
      .filter(entry => entry && typeof entry === 'object')
      .map(entry => {
        const cat = entry as Record<string, unknown>;
        return {
          id: String(cat['id'] ?? ''),
          label: String(cat['label'] ?? cat['id'] ?? ''),
          status: String(cat['status'] ?? '').toLowerCase(),
          detail: String(cat['detail'] ?? ''),
          remediation: String(cat['remediation'] ?? '')
        };
      });
  });

  readonly hasQualityDataSignal = computed(() =>
    Boolean(
      this.signalQualitySignal()
      || this.hrMetricsSignal()
      || this.rrMetricsSignal()
      || this.qualityGatesSignal().length
      || this.verdictCategoriesSignal().length
    )
  );

  readonly primarySessionMetadataSignal = computed(() => {
    const records = this.metadataRecords();
    const value = (...keys: string[]): unknown => {
      for (const record of records) {
        for (const key of keys) {
          const candidate = record[key];
          if (candidate !== undefined && candidate !== null && String(candidate).trim() !== '') return candidate;
        }
      }
      return null;
    };
    const text = (candidate: unknown): string => (candidate === null ? '' : String(candidate).trim());
    const rows: Array<{ label: string; value: string }> = [];
    const add = (label: string, candidate: unknown, suffix = '') => {
      const normalized = text(candidate);
      if (normalized) rows.push({ label, value: `${normalized}${suffix}` });
    };

    add('Participant', value('participant_id', 'participant_code', 'profile_code'));
    add('Trial', value('trial_id'));
    add('Condition', value('condition_id'));
    add('Distance', value('distance_m'), ' m');
    add('Barrier', value('barrier_type', 'barrier'));
    add('Study classification', value('study_classification', 'study_mode'));
    add('Model family', value('model_family'));
    add('Model bundle', value('model_bundle', 'model_bundle_id', 'model_bundle_hash'));
    add('Product release', value('product_version'));
    add('Trainer release', value('trainer_version'));
    add('Dashboard release', value('dashboard_version'));
    add('Expected firmware', value('firmware_expected'));
    add('Observed firmware', value('firmware_observed', 'firmware_version'));

    const serialProtocol = text(value('serial_protocol'));
    const serialWidth = text(value('serial_width_observed', 'serial_width_expected'));
    if (serialProtocol || serialWidth) {
      rows.push({
        label: 'Serial contract',
        value: [serialProtocol, serialWidth ? `${serialWidth} columns` : ''].filter(Boolean).join(' / ')
      });
    }
    add('Source commit', value('source_commit'));
    return rows;
  });

  readonly compareCandidatesSignal = computed(() =>
    this.sessionsSignal().filter(item => item.session_id && item.session_id !== this.selectedSessionIdSignal())
  );

  readonly comparisonSummaryCardsSignal = computed(() => {
    const comp = this.comparisonSignal();
    return [
      { label: 'Selected', value: comp?.selected ?? null },
      { label: 'Previous', value: comp?.previous ?? null },
      { label: 'Best', value: comp?.best ?? null }
    ];
  });

  readonly compareDeltaRowsSignal = computed(() => {
    const compSummary = this.compareSummarySignal();
    if (!compSummary) return [];
    const selectedRows = this.sessionDataRowsSignal();
    const compareRows = this.compareRowsSignal();
    const fmt = (value: number | null, digits = 1) => (value === null ? '--' : value.toFixed(digits));
    const rows: Array<{ label: string; selected: string; compare: string; delta: string; tone: 'better' | 'worse' | 'neutral' }> = [];

    const meanPairs: Array<{ label: string; key: string }> = [
      { label: 'Mean HR (bpm)', key: 'reported_hr' },
      { label: 'Mean RR (br/min)', key: 'reported_rr' }
    ];
    for (const pair of meanPairs) {
      const a = this.seriesMean(selectedRows, pair.key);
      const b = this.seriesMean(compareRows, pair.key);
      rows.push({
        label: pair.label,
        selected: fmt(a),
        compare: fmt(b),
        delta: a !== null && b !== null ? (a - b >= 0 ? '+' : '') + (a - b).toFixed(1) : '--',
        tone: 'neutral'
      });
    }

    const coverageOf = (record: SessionRecord | null) => {
      const metrics = record?.['hr_metrics'];
      const value = Number((metrics && typeof metrics === 'object' ? (metrics as Record<string, unknown>)['coverage_pct'] : null));
      return Number.isFinite(value) ? value : null;
    };
    const covA = coverageOf(this.selectedSummarySignal());
    const covB = coverageOf(compSummary);
    rows.push({
      label: 'HR coverage (%)',
      selected: fmt(covA),
      compare: fmt(covB),
      delta: covA !== null && covB !== null ? (covA - covB >= 0 ? '+' : '') + (covA - covB).toFixed(1) : '--',
      tone: covA !== null && covB !== null ? (covA > covB ? 'better' : covA < covB ? 'worse' : 'neutral') : 'neutral'
    });

    rows.push({
      label: 'Verdict',
      selected: this.verdictText(this.reportRecordSignal()),
      compare: this.verdictText(compSummary),
      delta: '--',
      tone: 'neutral'
    });
    return rows;
  });

  // PR #170 Reference & Adjudication computed signals
  readonly rrObserverSummary = computed(() =>
    summarizeRrObservers(this.referencesSignal(), this.rrAdjudicationSignal())
  );

  readonly computedBreathCountRate = computed(() =>
    computeBreathCountRate(Number(this.rrBreathCountInput()), Number(this.rrTrialDurationInput()))
  );

  readonly sessionConditionInfo = computed(() =>
    extractSessionConditionInfo(this.selectedSummarySignal(), this.selectedSessionSignal())
  );

  readonly isD060NoneAllowed = computed(() =>
    isD060NoneCondition(this.selectedSummarySignal(), this.selectedSessionSignal())
  );

  readonly otherReferences = computed(() =>
    this.referencesSignal().filter(ref => ref.kind !== 'rr_observer')
  );

  readonly objectiveReportCards = computed(() =>
    this.objectiveReportsSignal().map(report => ({
      report,
      lines: extractObjectiveEvidenceLines(report)
    }))
  );

  ngOnInit(): void {
    void this.loadSessions();
    void this.loadLatestStudyAnalysis();
  }

  ngAfterViewInit(): void {
    this.viewReady = true;
    if (typeof ResizeObserver !== 'undefined') {
      this.resizeObserver = new ResizeObserver(() => this.requestTrendDraw());
      if (this.hrReportCanvas) this.resizeObserver.observe(this.hrReportCanvas.nativeElement);
      if (this.rrReportCanvas) this.resizeObserver.observe(this.rrReportCanvas.nativeElement);
    }
    this.requestTrendDraw();
  }

  ngOnDestroy(): void {
    this.renderScheduler?.cancel(this);
    this.resizeObserver?.disconnect();
  }

  protected patchSignoff(patch: Partial<SessionSignoff>): void {
    this.signoffSignal.update(current => ({ ...current, ...patch }));
  }

  async loadObjectiveReports(): Promise<void> {
    if (this.objectiveReportsLoadingSignal() || typeof (this.api as Partial<ApiService>).loadObjectiveReport !== 'function') return;
    this.objectiveReportsLoadingSignal.set(true);
    this.objectiveReportsErrorSignal.set('');
    try {
      const objectives = await this.api.loadStudyObjectives();
      const reports = await Promise.all((objectives.objectives || []).map(objective => this.api.loadObjectiveReport(objective.id)));
      this.objectiveReportsSignal.set(reports);
    } catch (error) {
      this.objectiveReportsErrorSignal.set(error instanceof Error ? error.message : 'Objective reports are not available.');
    } finally {
      this.objectiveReportsLoadingSignal.set(false);
      this.cdr.markForCheck();
    }
  }

  async exportObjective(report: StudyObjectiveReport, format: 'latex' | 'csv', copyOnly = false): Promise<void> {
    if (this.objectiveExporting()) return;
    this.objectiveExporting.set(true);
    try {
      const result = await this.fetchObjectiveExport(report, format);
      if (copyOnly) {
        await this.copyTextToClipboard(result.content);
        this.snackBar.open(`Copied ${report.objective_id} IEEE LaTeX table to clipboard.`, 'Dismiss', { duration: 3500 });
      } else {
        this.downloadText(result.content, result.filename, result.content_type || (format === 'csv' ? 'text/csv' : 'application/x-tex'));
        this.snackBar.open(`Exported ${result.filename}.`, 'Dismiss', { duration: 3500 });
      }
      this.state.triggerHaptic('success');
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Objective export failed.', 'Dismiss', { duration: 5000 });
    } finally {
      this.objectiveExporting.set(false);
      this.cdr.markForCheck();
    }
  }

  private async fetchObjectiveExport(report: StudyObjectiveReport, format: 'latex' | 'csv'): Promise<ObjectiveExportResult> {
    const isSandbox = Boolean(this.selectedSummarySignal()?.sandbox);
    try {
      const res: StudyObjectiveExportResponse = await this.api.exportObjectiveSummary(report.objective_id, format);
      if (res && typeof res.content === 'string') return res;
    } catch (_) {
      // Truthful local fallback when route is unavailable offline/mock.
    }
    return buildFallbackObjectiveExport(report, format, isSandbox);
  }

  private async copyTextToClipboard(content: string): Promise<void> {
    if (typeof navigator !== 'undefined' && navigator.clipboard?.writeText) {
      await navigator.clipboard.writeText(content);
      return;
    }
    throw new Error('Clipboard API is unavailable in this environment.');
  }

  async startStudyAnalysis(): Promise<void> {
    if (this.studyAnalysisLoadingSignal() || this.studyAnalysisIsActive()) return;
    this.studyAnalysisLoadingSignal.set(true);
    this.studyAnalysisErrorSignal.set('');
    try {
      const response = await this.api.startStudyAnalysis({
        objective_id: this.studyAnalysisObjectiveSignal(),
        session_ids: [],
        model_family: this.studyAnalysisFamilySignal()
      });
      this.studyAnalysisJobSignal.set(response.job);
      this.snackBar.open(`Study analysis ${response.job.status}.`, 'Dismiss', { duration: 3500 });
    } catch (error: unknown) {
      this.studyAnalysisErrorSignal.set(error instanceof Error ? error.message : 'Study analysis could not be started.');
    } finally {
      this.studyAnalysisLoadingSignal.set(false);
      this.cdr.markForCheck();
    }
  }

  async refreshStudyAnalysis(): Promise<void> {
    const jobId = this.studyAnalysisJobSignal()?.job_id;
    if (!jobId || this.studyAnalysisLoadingSignal()) return;
    this.studyAnalysisLoadingSignal.set(true);
    try {
      const response = await this.api.loadStudyAnalysis(jobId);
      this.studyAnalysisJobSignal.set(response.job);
      this.cdr.markForCheck();
    } catch (error: unknown) {
      this.studyAnalysisErrorSignal.set(error instanceof Error ? error.message : 'Study analysis status is unavailable.');
      this.cdr.markForCheck();
    } finally {
      this.studyAnalysisLoadingSignal.set(false);
    }
  }

  async cancelStudyAnalysis(): Promise<void> {
    const jobId = this.studyAnalysisJobSignal()?.job_id;
    if (!jobId || this.studyAnalysisLoadingSignal()) return;
    this.studyAnalysisLoadingSignal.set(true);
    try {
      const response = await this.api.cancelStudyAnalysis(jobId);
      this.studyAnalysisJobSignal.set(response.job);
      this.snackBar.open('Study analysis cancellation requested.', 'Dismiss', { duration: 3500 });
      this.cdr.markForCheck();
    } catch (error: unknown) {
      this.studyAnalysisErrorSignal.set(error instanceof Error ? error.message : 'Study analysis could not be cancelled.');
      this.cdr.markForCheck();
    } finally {
      this.studyAnalysisLoadingSignal.set(false);
    }
  }

  async loadLatestStudyAnalysis(): Promise<void> {
    try {
      const response = await this.api.loadStudyAnalyses(1);
      const latest = response.jobs?.[0];
      if (latest) {
        this.studyAnalysisJobSignal.set(latest);
        if (latest.objective_id === 'objective_1_rr' || latest.objective_id === 'objective_4_hr') {
          this.studyAnalysisObjectiveSignal.set(latest.objective_id);
        }
        const family = latest.model_family;
        if (family === 'gradient_boosting' || family === 'cnn_1d') this.studyAnalysisFamilySignal.set(family);
      }
    } catch (_) {
      // The report remains usable when the durable job index is unavailable.
    } finally {
      this.cdr.markForCheck();
    }
  }

  studyAnalysisIsActive(): boolean {
    return ['queued', 'running', 'cancelling'].includes(String(this.studyAnalysisJobSignal()?.status || ''));
  }

  studyAnalysisProgress(): number {
    const progress = Number(this.studyAnalysisJobSignal()?.progress_pct);
    return Number.isFinite(progress) ? Math.max(0, Math.min(100, progress)) : 0;
  }

  selectSessionChip(sessionId: string): void {
    if (!sessionId || sessionId === this.selectedSessionIdSignal()) return;
    this.selectedSessionIdSignal.set(sessionId);
    void this.onSessionChange();
  }

  sessionChipLabel(session: SessionRecord): string {
    const started = session.started_at ? new Date(session.started_at) : null;
    const validDate = started && Number.isFinite(started.getTime()) ? started : null;
    const day = validDate ? this.relativeSessionDay(validDate) : 'Session';
    const time = validDate
      ? validDate.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      : String(session.session_id || '').slice(0, 8);
    const subject = String(session.subject_label || session.subject || (session.sandbox ? 'Sandbox' : 'Run'));
    return `${day} ${time} - ${subject}`;
  }

  outcomeTitle(): string { return this.outcomeTitleSignal(); }
  outcomeDescription(): string { return this.outcomeDescriptionSignal(); }
  outcomeIcon(): string { return this.outcomeIconSignal(); }
  outcomeBadge(): string { return this.outcomeBadgeSignal(); }
  qualityStatCards() { return this.qualityStatCardsSignal(); }

  private relativeSessionDay(date: Date): string {
    const today = new Date();
    const startOfToday = new Date(today.getFullYear(), today.getMonth(), today.getDate()).getTime();
    const startOfDate = new Date(date.getFullYear(), date.getMonth(), date.getDate()).getTime();
    const dayDelta = Math.round((startOfDate - startOfToday) / 86400000);
    if (dayDelta === 0) return 'Today';
    if (dayDelta === -1) return 'Yesterday';
    return date.toLocaleDateString([], { month: 'short', day: 'numeric' });
  }

  async loadSessions(): Promise<void> {
    this.sessionsLoadingSignal.set(true);
    this.sessionsErrorSignal.set('');
    try {
      const items = await this.api.loadSessions();
      if (Array.isArray(items)) {
        this.sessionsSignal.set(items);
        this.state.sessionItems.set(items);

        const activeId = this.state.currentSessionId();
        if (activeId && items.some(s => s.session_id === activeId)) {
          this.selectedSessionIdSignal.set(activeId);
        } else if (items.length > 0) {
          this.selectedSessionIdSignal.set(items[0].session_id);
        }
        await this.onSessionChange();
      }
    } catch (error: unknown) {
      console.warn('Could not load sessions', error);
      this.sessionsSignal.set([]);
      this.state.sessionItems.set([]);
      this.selectedSessionIdSignal.set('');
      this.selectedSessionSignal.set(null);
      this.sessionsErrorSignal.set(error instanceof Error ? error.message : 'Report history could not be loaded from the trainer.');
    } finally {
      this.sessionsLoadingSignal.set(false);
      this.cdr.markForCheck();
    }
  }

  async onSessionChange(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    this.compareLoadingSignal.set(false);
    const found = this.sessionsSignal().find(s => s.session_id === sessionId) || null;
    this.selectedSessionSignal.set(found);
    this.selectedSummarySignal.set(null);
    this.comparisonSignal.set(null);
    this.analysisStatusSignal.set(null);
    this.trainingStatusSignal.set(null);
    this.predictionSignal.set(null);
    this.sessionTagsInputSignal.set('');
    this.sessionDataRowsSignal.set([]);
    this.summaryErrorSignal.set('');
    this.compareSessionIdSignal.set('');
    this.compareSummarySignal.set(null);
    this.compareRowsSignal.set([]);
    this.summaryLoadingSignal.set(false);
    this.referencesSignal.set([]);
    this.rrAdjudicationSignal.set(null);
    this.referencesError.set('');
    this.adjudicationFinalValueInput.set('');
    this.adjudicationRationaleInput.set('');
    this.rrTrialDurationInput.set(String(found?.duration_s && found.duration_s > 0 ? found.duration_s : 150));

    if (found) {
      this.sessionNotesInputSignal.set(this.state.sessionNotes()[sessionId] || found.summary || '');
      this.summaryLoadingSignal.set(true);
      try {
        const result = await this.requests.loadSession(sessionId);
        if (!result || this.selectedSessionIdSignal() !== sessionId) return;
        this.selectedSummarySignal.set(result.summary);
        this.sessionDataRowsSignal.set(result.data.rows || []);
        this.comparisonSignal.set(result.comparison);
        this.analysisStatusSignal.set(result.analysisStatus);
        this.sessionNotesInputSignal.set(result.notes?.review_summary ?? '');
        this.signoff = {
          session_id: sessionId,
          operator_name: result.signoff?.operator_name ?? '',
          initials: result.signoff?.initials ?? '',
          validation_comment: result.signoff?.validation_comment ?? '',
          signed_at: result.signoff?.signed_at ?? null,
          updated_at: result.signoff?.updated_at ?? null
        };
        const manifestObj = (result.summary?.['manifest'] ?? result.summary?.['session_manifest']) as Record<string, unknown> | undefined;
        const rawTags = result.summary?.['tags'] ?? manifestObj?.['tags'];
        this.sessionTagsInputSignal.set(Array.isArray(rawTags) ? rawTags.map(String).join(', ') : '');
        const summaryDur = Number(result.summary?.duration_s ?? found.duration_s ?? 150);
        if (Number.isFinite(summaryDur) && summaryDur > 0) {
          this.rrTrialDurationInput.set(String(summaryDur));
        }
        void this.loadModelEvidence(sessionId);
        void this.loadSessionReferences(sessionId);
      } catch (error: unknown) {
        if (this.selectedSessionIdSignal() === sessionId) {
          this.summaryErrorSignal.set(error instanceof Error ? error.message : 'Recorded session summary is unavailable.');
        }
      } finally {
        if (this.selectedSessionIdSignal() === sessionId) {
          this.summaryLoadingSignal.set(false);
          this.cdr.markForCheck();
          this.requestTrendDraw();
        }
      }
    } else {
      this.requests.invalidateSelection();
      this.cdr.markForCheck();
    }
  }

  async loadSessionReferences(sessionId = this.selectedSessionIdSignal()): Promise<void> {
    if (!sessionId) return;
    this.referencesLoading.set(true);
    this.referencesError.set('');
    try {
      const api = this.api as Partial<ApiService>;
      const response: StudyReferencesResponse = typeof api.loadSessionReferences === 'function'
        ? await api.loadSessionReferences(sessionId)
        : await this.api.request<StudyReferencesResponse>(`/api/sessions/${encodeURIComponent(sessionId)}/references`);
      if (this.selectedSessionIdSignal() !== sessionId) return;
      this.referencesSignal.set(response.references || []);
      this.rrAdjudicationSignal.set(response.rr_adjudication ?? null);
    } catch (error: unknown) {
      if (this.selectedSessionIdSignal() === sessionId) {
        this.referencesError.set(error instanceof Error ? error.message : 'Reference observations unavailable.');
      }
    } finally {
      if (this.selectedSessionIdSignal() === sessionId) {
        this.referencesLoading.set(false);
        this.cdr.markForCheck();
      }
    }
  }

  async submitRrObserverObservation(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId || this.referencesSaving()) return;
    const observerId = this.rrObserverIdInput().trim();
    if (!observerId) {
      this.snackBar.open('Enter a distinct observer identifier.', 'Dismiss', { duration: 4000 });
      return;
    }
    const existingDuplicate = this.referencesSignal().some(
      ref => ref.kind === 'rr_observer' && String(ref.observer_id || '').trim().toLowerCase() === observerId.toLowerCase()
    );
    if (existingDuplicate) {
      this.snackBar.open(`Observer "${observerId}" already has a locked RR observation for this session.`, 'Dismiss', { duration: 5000 });
      return;
    }

    const durationS = Number(this.rrTrialDurationInput());
    let rateBpm: number | null = null;
    if (this.rrEntryMode() === 'count') {
      rateBpm = computeBreathCountRate(Number(this.rrBreathCountInput()), durationS);
    } else {
      const val = Number(this.rrRateInput());
      rateBpm = Number.isFinite(val) && val > 0 ? val : null;
    }
    if (rateBpm === null || rateBpm <= 0) {
      this.snackBar.open('Enter a valid respiration reading or breath count.', 'Dismiss', { duration: 4000 });
      return;
    }

    await this.saveReferenceObservation(sessionId, {
      kind: 'rr_observer',
      observer_id: observerId,
      value: rateBpm,
      unit: 'br/min',
      duration_s: Number.isFinite(durationS) && durationS > 0 ? durationS : undefined
    }, () => {
      this.rrObserverIdInput.set('');
      this.rrRateInput.set('');
      this.rrBreathCountInput.set('');
    });
  }

  async submitTemperatureReference(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId || this.referencesSaving() || !this.isD060NoneAllowed()) return;
    const val = Number(this.tempValueInput());
    if (!Number.isFinite(val) || val < 20 || val > 45) {
      this.snackBar.open('Enter a valid IR temperature between 20 and 45 °C.', 'Dismiss', { duration: 4000 });
      return;
    }
    const uncertainty = Number(this.tempUncertaintyInput());
    await this.saveReferenceObservation(sessionId, {
      kind: 'temperature',
      value: val,
      unit: 'degC',
      device_id: this.tempDeviceIdInput().trim() || 'MLX90614',
      calibration_id: this.tempCalibrationIdInput().trim() || undefined,
      uncertainty: Number.isFinite(uncertainty) ? uncertainty : undefined
    }, () => {
      this.tempValueInput.set('');
    });
  }

  async submitManualHrReference(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId || this.referencesSaving()) return;
    const val = Number(this.hrRefValueInput());
    if (!Number.isFinite(val) || val <= 0) {
      this.snackBar.open('Enter a valid manual heart-rate reading.', 'Dismiss', { duration: 4000 });
      return;
    }
    await this.saveReferenceObservation(sessionId, {
      kind: 'hr',
      value: val,
      unit: 'bpm',
      observer_id: this.hrRefSourceInput().trim() || undefined
    }, () => {
      this.hrRefValueInput.set('');
      this.hrRefSourceInput.set('');
    });
  }

  private async saveReferenceObservation(
    sessionId: string,
    payload: ReferenceObservationInput,
    onSuccess: () => void
  ): Promise<void> {
    this.referencesSaving.set(true);
    this.referencesError.set('');
    try {
      const api = this.api as Partial<ApiService>;
      const response: StudyReferencesResponse = typeof api.addSessionReference === 'function'
        ? await api.addSessionReference(sessionId, payload)
        : await this.api.request<StudyReferencesResponse>(
          `/api/sessions/${encodeURIComponent(sessionId)}/references`,
          { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) }
        );
      if (this.selectedSessionIdSignal() === sessionId) {
        this.referencesSignal.set(response.references || []);
        this.rrAdjudicationSignal.set(response.rr_adjudication ?? null);
        onSuccess();
      }
      this.state.triggerHaptic('success');
      this.snackBar.open('Reference observation locked.', 'Dismiss', { duration: 3500 });
    } catch (error: unknown) {
      const msg = error instanceof Error ? error.message : 'Reference observation could not be saved.';
      this.referencesError.set(msg);
      this.snackBar.open(msg, 'Dismiss', { duration: 5000 });
    } finally {
      this.referencesSaving.set(false);
      this.cdr.markForCheck();
    }
  }

  async adjudicateRr(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    const summary = this.rrObserverSummary();
    if (!sessionId || !summary.unblinded || this.adjudicating()) return;
    const finalValue = Number(this.adjudicationFinalValueInput());
    if (!Number.isFinite(finalValue) || finalValue <= 0) {
      this.snackBar.open('Enter a valid adjudicated RR value.', 'Dismiss', { duration: 4000 });
      return;
    }
    const rationale = this.adjudicationRationaleInput().trim()
      || (summary.exceedsDiscrepancyThreshold ? '' : 'Observer agreement within 1.0 br/min.');
    if (!rationale) {
      this.snackBar.open('Provide an adjudication rationale when |RR_1 - RR_2| exceeds 1.0 br/min.', 'Dismiss', { duration: 5000 });
      return;
    }

    this.adjudicating.set(true);
    this.referencesError.set('');
    try {
      const api = this.api as Partial<ApiService>;
      const payload = {
        final_value: finalValue,
        rationale,
        actor: this.signoffSignal().operator_name.trim() || 'dashboard-operator'
      };
      const response: StudyReferencesResponse = typeof api.adjudicateSessionRr === 'function'
        ? await api.adjudicateSessionRr(sessionId, payload)
        : await this.api.request<StudyReferencesResponse>(
          `/api/sessions/${encodeURIComponent(sessionId)}/references/rr-adjudication`,
          { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) }
        );
      if (this.selectedSessionIdSignal() === sessionId) {
        this.referencesSignal.set(response.references || this.referencesSignal());
        this.rrAdjudicationSignal.set(response.rr_adjudication ?? { final_value: finalValue, rationale });
      }
      this.state.triggerHaptic('success');
      this.snackBar.open('Final adjudicated RR locked.', 'Dismiss', { duration: 3500 });
    } catch (error: unknown) {
      const msg = error instanceof Error ? error.message : 'RR adjudication failed.';
      this.referencesError.set(msg);
      this.snackBar.open(msg, 'Dismiss', { duration: 5000 });
    } finally {
      this.adjudicating.set(false);
      this.cdr.markForCheck();
    }
  }

  async saveReportNotes(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    const reviewSummary = this.sessionNotesInputSignal();
    if (!sessionId) return;
    try {
      await this.requests.saveNotes(sessionId, reviewSummary);
      this.state.sessionNotes.update(notes => ({ ...notes, [sessionId]: reviewSummary }));
      this.state.triggerHaptic('success');
      this.snackBar.open('Operator review summary saved.', 'Dismiss', { duration: 4000 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Review summary could not be saved.', 'Dismiss', { duration: 6000 });
    }
  }

  async saveSignoff(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    const current = this.signoffSignal();
    const signoffInput = {
      operator_name: current.operator_name,
      initials: current.initials.toUpperCase(),
      validation_comment: current.validation_comment || ''
    };
    if (!sessionId) return;
    try {
      const response = await this.requests.saveSignoff(sessionId, signoffInput);
      const savedSignoff: SessionSignoff = {
        ...response,
        session_id: sessionId,
        operator_name: response?.operator_name ?? signoffInput.operator_name,
        initials: response?.initials ?? signoffInput.initials,
        validation_comment: response?.validation_comment ?? signoffInput.validation_comment
      };
      this.state.sessionSignoffs.update(items => ({ ...items, [sessionId]: savedSignoff }));
      if (this.selectedSessionIdSignal() === sessionId) {
        this.signoff = savedSignoff;
      }
      this.state.triggerHaptic('success');
      this.snackBar.open('Operator validation sign-off recorded.', 'Dismiss', { duration: 4000 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Sign-off could not be saved.', 'Dismiss', { duration: 6000 });
    }
  }

  reportRecord(): SessionRecord {
    return this.reportRecordSignal();
  }

  async loadModelEvidence(sessionId = this.selectedSessionIdSignal()): Promise<void> {
    if (!sessionId) return;
    if (typeof (this.api as Partial<ApiService>).loadTrainingStatus !== 'function'
      || typeof (this.api as Partial<ApiService>).loadPrediction !== 'function') return;
    this.modelEvidenceLoadingSignal.set(true);
    try {
      const [trainingStatus, prediction] = await Promise.all([
        this.api.loadTrainingStatus(sessionId),
        this.api.loadPrediction(sessionId)
      ]);
      if (this.selectedSessionIdSignal() === sessionId) {
        this.trainingStatusSignal.set(trainingStatus);
        this.predictionSignal.set(prediction);
      }
    } catch (error: unknown) {
      if (this.selectedSessionIdSignal() === sessionId) {
        this.trainingStatusSignal.set(null);
        this.predictionSignal.set(null);
      }
      console.warn('Model evidence endpoints unavailable', error);
    } finally {
      if (this.selectedSessionIdSignal() === sessionId) {
        this.modelEvidenceLoadingSignal.set(false);
        this.cdr.markForCheck();
      }
    }
  }

  async saveSessionTags(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId) return;
    const tags = this.sessionTagsInputSignal().split(',').map(tag => tag.trim()).filter(Boolean).slice(0, 32);
    try {
      const response: SessionTagsResponse = await this.api.updateSessionTags(sessionId, tags);
      this.sessionTagsInputSignal.set(response.tags.join(', '));
      this.snackBar.open('Session tags saved.', 'Dismiss', { duration: 3500 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Session tags could not be saved.', 'Dismiss', { duration: 5000 });
    }
  }

  async deleteSelectedSession(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId || this.selectedSummarySignal()?.sandbox) return;
    if (this.dialog) {
      const confirmed = await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
        data: {
          title: 'Soft-delete recorded session?',
          message: 'Move this recorded session to the trainer trash folder?',
          confirmLabel: 'Soft-delete session'
        },
        restoreFocus: true
      }).afterClosed());
      if (!confirmed) return;
    }
    try {
      await this.api.deleteSession(sessionId);
      this.snackBar.open('Session moved to trainer trash.', 'Dismiss', { duration: 4000 });
      this.selectedSessionIdSignal.set('');
      await this.loadSessions();
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Session could not be deleted.', 'Dismiss', { duration: 6000 });
    }
  }

  primarySessionMetadata() { return this.primarySessionMetadataSignal(); }

  private metadataRecords(): Array<Record<string, unknown>> {
    const records: Array<Record<string, unknown>> = [];
    const visit = (candidate: unknown): void => {
      if (!candidate || typeof candidate !== 'object' || Array.isArray(candidate)) return;
      const record = candidate as Record<string, unknown>;
      if (records.includes(record)) return;
      records.push(record);
      for (const key of ['params', 'study', 'release', 'release_provenance', 'provenance', 'metadata', 'session_manifest']) {
        visit(record[key]);
      }
    };
    visit(this.selectedSummarySignal());
    visit(this.selectedSessionSignal());
    return records;
  }

  private summaryObject(key: string): Record<string, unknown> | null {
    const value = this.selectedSummarySignal()?.[key];
    return value && typeof value === 'object' && !Array.isArray(value) ? (value as Record<string, unknown>) : null;
  }

  signalQuality() { return this.signalQualitySignal(); }
  hrMetrics() { return this.hrMetricsSignal(); }
  rrMetrics() { return this.rrMetricsSignal(); }
  qualityGates() { return this.qualityGatesSignal(); }
  verdictCategories() { return this.verdictCategoriesSignal(); }
  hasQualityData(): boolean { return this.hasQualityDataSignal(); }

  qualityStat(block: Record<string, unknown> | null, key: string, digits = 1, suffix = ''): string {
    const value = Number(block?.[key]);
    return Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : '--';
  }

  reportVerdict(): string { return this.reportVerdictSignal(); }

  downloadSummaryJson(): void {
    this.state.triggerHaptic('tap');
    const summary = this.selectedSummarySignal();
    const session = this.selectedSessionSignal();
    if (!session || !summary) return;
    this.downloadText(
      JSON.stringify(summary, null, 2),
      `session_summary_${this.selectedSessionIdSignal()}.json`,
      'application/json'
    );
  }

  async downloadSessionCsv(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId) return;

    if (!this.selectedSummarySignal()?.sandbox && this.state.ctlStatus()?.mode !== 'sandbox') {
      try {
        await this.api.download(
          `/api/sessions/${encodeURIComponent(sessionId)}/files/radar.csv`,
          `session_data_${sessionId}.csv`
        );
        this.state.triggerHaptic('success');
        return;
      } catch (error: unknown) {
        this.snackBar.open(
          error instanceof Error ? error.message : 'Raw telemetry CSV download failed.',
          'Dismiss',
          { duration: 5000 }
        );
      }
    }

    const rows = this.sessionDataRowsSignal();
    if (!rows.length) {
      this.snackBar.open('No telemetry data recorded for this session yet.', 'Dismiss', { duration: 3000 });
      return;
    }

    const fields = Array.from(new Set(rows.flatMap(row => Object.keys(row))));
    const content = [
      fields.map(field => this.csvValue(field)).join(','),
      ...rows.map(row => fields.map(field => this.csvValue(row[field])).join(','))
    ].join('\n');
    this.downloadText(content, `session_data_${sessionId}.csv`, 'text/csv');
    this.state.triggerHaptic('success');
  }

  downloadComparisonJson(): void {
    const sessionId = this.selectedSessionIdSignal();
    const comp = this.comparisonSignal();
    if (!sessionId || !comp) return;
    this.downloadText(
      JSON.stringify(comp, null, 2),
      `session_comparison_${sessionId}.json`,
      'application/json'
    );
    this.state.triggerHaptic('success');
  }

  async downloadThesisReport(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId || this.selectedSummarySignal()?.sandbox) {
      this.downloadSummaryJson();
      return;
    }
    try {
      await this.api.download(
        `/api/report/export?session=${encodeURIComponent(sessionId)}`,
        `session_report_${sessionId}.html`
      );
      this.state.triggerHaptic('success');
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Report download failed.', 'Dismiss', { duration: 6000 });
    }
  }

  async downloadArtifact(file: DownloadRecord): Promise<void> {
    const relativePath = file.relpath || file.path;
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId || !relativePath || this.artifactDownloading()) return;
    this.artifactDownloading.set(true);
    try {
      const name = relativePath.split('/').pop() || 'artifact';
      await this.api.download(
        `/api/sessions/${encodeURIComponent(sessionId)}/files/${relativePath.split('/').map(encodeURIComponent).join('/')}`,
        name
      );
      this.state.triggerHaptic('success');
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Artifact download failed.', 'Dismiss', { duration: 5000 });
    } finally {
      this.artifactDownloading.set(false);
      this.cdr.markForCheck();
    }
  }

  async rerunAnalysis(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId || this.selectedSummarySignal()?.sandbox) return;
    try {
      await this.api.request<{ ok?: boolean }>(`/api/sessions/${encodeURIComponent(sessionId)}/analyse`, { method: 'POST' });
      this.analysisStatusSignal.set(
        await this.api.request<{ status?: string; progress_pct?: number; last_line?: string }>(
          `/api/sessions/${encodeURIComponent(sessionId)}/analyse/status`
        )
      );
      this.snackBar.open('Analysis job requested. Refresh status to view completion.', 'Dismiss', { duration: 5000 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Analysis could not be started.', 'Dismiss', { duration: 6000 });
    } finally {
      this.cdr.markForCheck();
    }
  }

  async refreshAnalysisStatus(): Promise<void> {
    const sessionId = this.selectedSessionIdSignal();
    if (!sessionId || this.analysisStatusLoading()) return;
    this.analysisStatusLoading.set(true);
    try {
      const status = await this.api.request<{ status?: string; progress_pct?: number; last_line?: string }>(
        `/api/sessions/${encodeURIComponent(sessionId)}/analyse/status`
      );
      if (this.selectedSessionIdSignal() === sessionId) {
        this.analysisStatusSignal.set(status);
      }
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Analysis status refresh failed.', 'Dismiss', { duration: 5000 });
    } finally {
      this.analysisStatusLoading.set(false);
      this.cdr.markForCheck();
    }
  }

  private csvValue(value: unknown): string {
    return `"${String(value ?? '').replaceAll('"', '""')}"`;
  }

  private downloadText(content: string, filename: string, type: string): void {
    const href = URL.createObjectURL(new Blob([content], { type }));
    const anchor = document.createElement('a');
    anchor.href = href;
    anchor.download = filename;
    anchor.click();
    URL.revokeObjectURL(href);
  }

  compareCandidates() { return this.compareCandidatesSignal(); }

  async loadCompareSession(sessionId: string): Promise<void> {
    this.compareSessionIdSignal.set(sessionId);
    if (!sessionId) {
      this.requests.invalidateComparison();
      this.compareLoadingSignal.set(false);
      this.compareSummarySignal.set(null);
      this.compareRowsSignal.set([]);
      this.requestTrendDraw();
      return;
    }
    this.compareLoadingSignal.set(true);
    try {
      const result = await this.requests.loadComparison(sessionId);
      if (!result || this.compareSessionIdSignal() !== sessionId) return;
      this.compareSummarySignal.set(result.summary);
      this.compareRowsSignal.set(result.rows);
    } catch (error: unknown) {
      if (this.compareSessionIdSignal() === sessionId) {
        this.compareSummarySignal.set(null);
        this.compareRowsSignal.set([]);
        this.snackBar.open(error instanceof Error ? error.message : 'Comparison session could not be loaded.', 'Dismiss', { duration: 5000 });
      }
    } finally {
      if (this.compareSessionIdSignal() === sessionId) {
        this.compareLoadingSignal.set(false);
        this.cdr.markForCheck();
        this.requestTrendDraw();
      }
    }
  }

  private seriesMean(rows: Array<Record<string, number | string | null>>, key: string): number | null {
    const validityKey = key === 'reported_hr' ? 'logged_hr_valid' : key === 'reported_rr' ? 'logged_rr_valid' : '';
    const values = rows
      .filter(row => {
        const value = Number(row[key]);
        if (!Number.isFinite(value) || value <= 0) return false;
        if (validityKey && row[validityKey] !== undefined && row[validityKey] !== null && Number(row[validityKey]) === 0) {
          return false;
        }
        return true;
      })
      .map(row => Number(row[key]));
    if (!values.length) return null;
    return values.reduce((sum, value) => sum + value, 0) / values.length;
  }

  private verdictText(record: SessionRecord | null): string {
    const raw = record?.verdict;
    return String(typeof raw === 'object' && raw !== null ? (raw as Record<string, unknown>)['verdict'] || '' : raw || '--');
  }

  compareDeltaRows() { return this.compareDeltaRowsSignal(); }

  private requestTrendDraw(): void {
    if (!this.viewReady) return;
    if (this.renderScheduler) {
      this.renderScheduler.request(
        this,
        () => this.drawReportTrends(),
        () => [this.hrReportCanvas, this.rrReportCanvas].some(
          ref => this.renderScheduler!.canvasVisible(ref?.nativeElement)
        ),
        100
      );
    } else {
      this.drawReportTrends();
    }
  }

  private drawReportTrends(): void {
    if (!this.hrReportCanvas || !this.rrReportCanvas) return;

    const valuesFrom = (rows: Array<Record<string, number | string | null>>, keys: string[]): number[] => rows
      .map(row => {
        for (const key of keys) {
          const value = Number(row[key]);
          if (Number.isFinite(value) && value > 0) return value;
        }
        return null;
      })
      .filter((value): value is number => value !== null);

    const rows = this.sessionDataRowsSignal();
    const compare = this.compareRowsSignal();
    const hrPoints = valuesFrom(rows, ['reported_hr', 'hr']);
    const rrPoints = valuesFrom(rows, ['reported_rr', 'rr']);
    const hrReference = valuesFrom(rows, ['ble_hr', 'ref_hr', 'reference_hr']);
    const rrReference = valuesFrom(rows, ['ble_rr', 'ref_rr', 'reference_rr']);
    const hrCompare = valuesFrom(compare, ['reported_hr', 'hr']);
    const rrCompare = valuesFrom(compare, ['reported_rr', 'rr']);

    const rootStyles = typeof getComputedStyle !== 'undefined' ? getComputedStyle(document.documentElement) : null;
    const onSurfaceVariant = rootStyles?.getPropertyValue('--md-sys-color-on-surface-variant').trim() || '#64748b';
    const outlineColor = rootStyles?.getPropertyValue('--md-sys-color-outline-variant').trim() || '#f1f5f9';

    const plotReportTrend = (
      canvasRef: ElementRef<HTMLCanvasElement>,
      color: string,
      points: number[],
      reference: number[] = [],
      overlay: number[] = []
    ) => {
      const canvas = canvasRef.nativeElement;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      if (w <= 0 || h <= 0) return;
      const dpr = window.devicePixelRatio || 1;

      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      const ctx = canvas.getContext('2d');
      if (!ctx) return;
      ctx.resetTransform();
      ctx.scale(dpr, dpr);
      ctx.clearRect(0, 0, w, h);

      if ((!Array.isArray(points) || points.length < 2) && reference.length < 2) {
        ctx.fillStyle = onSurfaceVariant;
        ctx.font = '12px Inter, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('No recorded series in this summary.', w / 2, h / 2);
        return;
      }

      const pad = 12;
      const innerW = w - pad * 2;
      const innerH = h - pad * 2;
      ctx.strokeStyle = outlineColor;
      ctx.lineWidth = 1;
      for (let i = 0; i <= 3; i++) {
        const y = pad + (innerH * i) / 3;
        ctx.beginPath();
        ctx.moveTo(pad, y);
        ctx.lineTo(w - pad, y);
        ctx.stroke();
      }

      const scalePool = points.concat(reference, overlay);
      const minV = Math.min(...scalePool) - 5;
      const maxV = Math.max(...scalePool) + 5;
      const diff = Math.max(1, maxV - minV);

      const traceLine = (series: number[]) => {
        if (series.length < 2) return;
        ctx.beginPath();
        series.forEach((val, idx) => {
          const x = pad + (idx / (series.length - 1)) * innerW;
          const y = pad + innerH - ((val - minV) / diff) * innerH;
          if (idx === 0) {
            ctx.moveTo(x, y);
          } else {
            ctx.lineTo(x, y);
          }
        });
        ctx.stroke();
      };

      if (reference.length >= 2) {
        ctx.save();
        ctx.strokeStyle = onSurfaceVariant;
        ctx.lineWidth = 2;
        traceLine(reference);
        ctx.restore();
      }

      if (points.length >= 2) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.5;
        traceLine(points);
      }

      if (overlay.length >= 2) {
        ctx.save();
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.65;
        ctx.lineWidth = 1.8;
        ctx.setLineDash([6, 4]);
        traceLine(overlay);
        ctx.restore();
      }
    };

    plotReportTrend(this.hrReportCanvas, 'rgba(182, 46, 99, 0.95)', hrPoints, hrReference, hrCompare);
    plotReportTrend(this.rrReportCanvas, 'rgba(14, 124, 114, 0.95)', rrPoints, rrReference, rrCompare);
  }
}
