import { ChangeDetectionStrategy, ChangeDetectorRef, Component, inject, OnInit, ViewChild, ElementRef, AfterViewInit, effect } from '@angular/core';
import { DatePipe, UpperCasePipe } from '@angular/common';
import { FormsModule } from '@angular/forms';

// Angular Material 3 modules
import { MatCardModule } from '@angular/material/card';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatSelectModule } from '@angular/material/select';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatListModule } from '@angular/material/list';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';

import { DownloadRecord, SessionDataPayload, SessionNotesPayload, SessionRecord, SessionSignoff } from '../../models/rvt.models';
import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';

@Component({
  selector: 'app-report',
  imports: [
    DatePipe,
    UpperCasePipe,
    FormsModule,
    MatCardModule,
    MatButtonModule,
    MatIconModule,
    MatFormFieldModule,
    MatInputModule,
    MatSelectModule,
    MatProgressBarModule,
    MatListModule,
    MatSnackBarModule
  ],
  templateUrl: './report.component.html',
  styleUrl: './report.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class ReportComponent implements OnInit, AfterViewInit {
  protected readonly state = inject(StateService);
  protected readonly api = inject(ApiService);
  private readonly cdr = inject(ChangeDetectorRef);
  private readonly snackBar = inject(MatSnackBar);

  constructor() {
    effect(() => {
      this.state.theme();
      setTimeout(() => this.drawReportTrends(), 50);
    });
  }

  @ViewChild('hrReportCanvas', { static: false }) hrReportCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('rrReportCanvas', { static: false }) rrReportCanvas!: ElementRef<HTMLCanvasElement>;

  sessions: SessionRecord[] = [];
  selectedSessionId = '';
  selectedSession: SessionRecord | null = null;
  selectedSummary: SessionRecord | null = null;
  comparison: { selected?: SessionRecord | null; previous?: SessionRecord | null; best?: SessionRecord | null } | null = null;
  compareSessionId = '';
  compareSummary: SessionRecord | null = null;
  compareRows: Array<Record<string, number | string | null>> = [];
  compareLoading = false;
  analysisStatus: { status?: string; progress_pct?: number; last_line?: string } | null = null;
  sessionDataRows: Array<Record<string, number | string | null>> = [];
  sessionsLoading = false;
  sessionsError = '';
  summaryLoading = false;
  summaryError = '';
  sessionNotesInput = '';
  signoff: SessionSignoff = { session_id: '', operator_name: '', initials: '', validation_comment: '', signed_at: null };

  ngOnInit() {
    this.loadSessions();
  }

  ngAfterViewInit() {
    this.drawReportTrends();
  }

  async loadSessions() {
    this.sessionsLoading = true;
    this.sessionsError = '';
    try {
      const resp = await this.api.request<{ items?: SessionRecord[] }>('/api/sessions');
      if (resp && Array.isArray(resp.items)) {
        this.sessions = resp.items;
        this.state.sessionItems.set(this.sessions);
        
        // Auto select current session if available, else first past run
        const activeId = this.state.currentSessionId();
        if (activeId && this.sessions.some(s => s.session_id === activeId)) {
          this.selectedSessionId = activeId;
        } else if (this.sessions.length > 0) {
          this.selectedSessionId = this.sessions[0].session_id;
        }
        this.onSessionChange();
      }
    } catch (error: unknown) {
      console.warn('Could not load sessions', error);
      this.sessions = [];
      this.state.sessionItems.set([]);
      this.selectedSessionId = '';
      this.selectedSession = null;
      this.sessionsError = error instanceof Error ? error.message : 'Report history could not be loaded from the trainer.';
    } finally {
      this.sessionsLoading = false;
      this.cdr.markForCheck();
    }
  }

  async onSessionChange() {
    this.selectedSession = this.sessions.find(s => s.session_id === this.selectedSessionId) || null;
    this.selectedSummary = null;
    this.comparison = null;
    this.analysisStatus = null;
    this.sessionDataRows = [];
    this.summaryError = '';
    // A stale dashed overlay against a different selected session is misleading.
    this.compareSessionId = '';
    this.compareSummary = null;
    this.compareRows = [];
    if (this.selectedSession) {
      this.sessionNotesInput = this.state.sessionNotes()[this.selectedSessionId] || this.selectedSession.summary || '';
      this.summaryLoading = true;
      try {
        const sessionPath = `/api/sessions/${encodeURIComponent(this.selectedSessionId)}`;
        const [summary, data, comparison, analysisStatus, notes, signoff] = await Promise.all([
          this.api.request<SessionRecord>(`${sessionPath}/summary`),
          this.api.request<SessionDataPayload>(`${sessionPath}/data?points=1000`),
          this.api.request<{ selected?: SessionRecord | null; previous?: SessionRecord | null; best?: SessionRecord | null }>(`${sessionPath}/compare`),
          this.api.request<{ status?: string; progress_pct?: number; last_line?: string }>(`${sessionPath}/analyse/status`),
          this.api.request<SessionNotesPayload>(`${sessionPath}/notes`),
          this.api.request<SessionSignoff>(`${sessionPath}/signoff`)
        ]);
        this.selectedSummary = summary;
        this.sessionDataRows = data.rows || [];
        this.comparison = comparison;
        this.analysisStatus = analysisStatus;
        this.sessionNotesInput = notes.review_summary || '';
        this.signoff = { ...signoff, session_id: this.selectedSessionId };
      } catch (error: unknown) {
        this.summaryError = error instanceof Error ? error.message : 'Recorded session summary is unavailable.';
      } finally {
        this.summaryLoading = false;
        this.cdr.markForCheck();
        setTimeout(() => this.drawReportTrends(), 50);
      }
    } else {
      this.cdr.markForCheck();
    }
  }

  async saveReportNotes(): Promise<void> {
    if (this.selectedSessionId) {
      try {
        await this.api.request<SessionNotesPayload>(`/api/sessions/${encodeURIComponent(this.selectedSessionId)}/notes`, {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ review_summary: this.sessionNotesInput })
        });
        this.state.sessionNotes.update(notes => ({ ...notes, [this.selectedSessionId]: this.sessionNotesInput }));
        this.state.triggerHaptic('success');
        this.snackBar.open('Operator review summary saved.', 'Dismiss', { duration: 4000 });
      } catch (error: unknown) {
        this.snackBar.open(error instanceof Error ? error.message : 'Review summary could not be saved.', 'Dismiss', { duration: 6000 });
      }
    }
  }

  async saveSignoff(): Promise<void> {
    if (!this.selectedSessionId) return;
    try {
      this.signoff = await this.api.request<SessionSignoff>(`/api/sessions/${encodeURIComponent(this.selectedSessionId)}/signoff`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          operator_name: this.signoff.operator_name,
          initials: this.signoff.initials.toUpperCase(),
          validation_comment: this.signoff.validation_comment
        })
      });
      this.state.sessionSignoffs.update(items => ({ ...items, [this.selectedSessionId]: this.signoff }));
      this.state.triggerHaptic('success');
      this.snackBar.open('Operator validation sign-off recorded.', 'Dismiss', { duration: 4000 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Sign-off could not be saved.', 'Dismiss', { duration: 6000 });
    }
  }

  reportRecord(): SessionRecord {
    return this.selectedSummary || this.selectedSession || { session_id: '' };
  }

  // ---- Session quality scorecard (all values come from the existing
  // /api/sessions/<id>/summary payload; nothing here adds API surface) ----

  private summaryObject(key: string): Record<string, unknown> | null {
    const value = this.selectedSummary?.[key];
    return value && typeof value === 'object' && !Array.isArray(value) ? value as Record<string, unknown> : null;
  }

  signalQuality(): Record<string, unknown> | null {
    return this.summaryObject('signal_quality');
  }

  hrMetrics(): Record<string, unknown> | null {
    return this.summaryObject('hr_metrics');
  }

  rrMetrics(): Record<string, unknown> | null {
    return this.summaryObject('rr_metrics');
  }

  qualityGates(): Array<{ id: string; passed: boolean | null; status: string }> {
    const gates = this.summaryObject('gates');
    if (!gates) return [];
    return Object.entries(gates)
      .filter(([, value]) => value && typeof value === 'object')
      .map(([id, value]) => {
        const gate = value as Record<string, unknown>;
        const passed = typeof gate['passed'] === 'boolean' ? gate['passed'] as boolean : null;
        return { id, passed, status: String(gate['status'] ?? (passed === null ? 'n/a' : passed ? 'pass' : 'fail')) };
      });
  }

  verdictCategories(): Array<{ id: string; label: string; status: string; detail: string; remediation: string }> {
    // Real trainer summaries keep the short string in `verdict` and the
    // category/remediation detail under `ml_readiness_verdict`; the sandbox
    // shape nests categories directly in the verdict object. Support both.
    const mlVerdict = this.summaryObject('ml_readiness_verdict');
    const verdict = this.reportRecord()?.verdict;
    const fromVerdictObj = typeof verdict === 'object' && verdict !== null ? (verdict as Record<string, unknown>)['categories'] : null;
    const categories = Array.isArray(mlVerdict?.['categories']) ? mlVerdict['categories'] : fromVerdictObj;
    if (!Array.isArray(categories)) {
      if (mlVerdict) {
        console.warn('ml_readiness_verdict present but categories[] missing or malformed', mlVerdict);
      }
      return [];
    }
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
  }

  hasQualityData(): boolean {
    return !!(this.signalQuality() || this.hrMetrics() || this.rrMetrics()
      || this.qualityGates().length || this.verdictCategories().length);
  }

  qualityStat(block: Record<string, unknown> | null, key: string, digits = 1, suffix = ''): string {
    const value = Number(block?.[key]);
    return Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : '--';
  }

  reportVerdict(): string {
    const raw = this.reportRecord()?.verdict;
    return String(typeof raw === 'object' && raw !== null ? raw['verdict'] || raw['readiness_kind'] || '' : raw || '').toLowerCase();
  }

  downloadSummaryJson() {
    this.state.triggerHaptic('tap');
    if (!this.selectedSession || !this.selectedSummary) return;
    const blob = new Blob([JSON.stringify(this.selectedSummary, null, 2)], { type: 'application/json' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `session_summary_${this.selectedSessionId}.json`;
    a.click();
    window.URL.revokeObjectURL(url);
  }

  async downloadSessionCsv(): Promise<void> {
    if (!this.selectedSessionId) return;

    if (!this.selectedSummary?.sandbox && this.state.ctlStatus()?.mode !== 'sandbox') {
      try {
        await this.api.download(
          `/api/sessions/${encodeURIComponent(this.selectedSessionId)}/files/radar.csv`,
          `session_data_${this.selectedSessionId}.csv`
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

    if (!this.sessionDataRows.length) {
      this.snackBar.open('No telemetry data recorded for this session yet.', 'Dismiss', { duration: 3000 });
      return;
    }

    const fields = Array.from(new Set(this.sessionDataRows.flatMap(row => Object.keys(row))));
    const content = [
      fields.map(field => this.csvValue(field)).join(','),
      ...this.sessionDataRows.map(row => fields.map(field => this.csvValue(row[field])).join(','))
    ].join('\n');
    this.downloadText(content, `session_data_${this.selectedSessionId}.csv`, 'text/csv');
    this.state.triggerHaptic('success');
  }

  downloadComparisonJson(): void {
    if (!this.selectedSessionId || !this.comparison) return;
    this.downloadText(
      JSON.stringify(this.comparison, null, 2),
      `session_comparison_${this.selectedSessionId}.json`,
      'application/json'
    );
    this.state.triggerHaptic('success');
  }

  async downloadThesisReport(): Promise<void> {
    if (!this.selectedSessionId || this.selectedSummary?.sandbox) {
      this.downloadSummaryJson();
      return;
    }
    try {
      await this.api.download(
        `/api/report/export?session=${encodeURIComponent(this.selectedSessionId)}`,
        `session_report_${this.selectedSessionId}.html`
      );
      this.state.triggerHaptic('success');
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Report download failed.', 'Dismiss', { duration: 6000 });
    }
  }

  async downloadArtifact(file: DownloadRecord): Promise<void> {
    const relativePath = file.relpath || file.path;
    if (!this.selectedSessionId || !relativePath) return;
    const name = relativePath.split('/').pop() || 'artifact';
    await this.api.download(
      `/api/sessions/${encodeURIComponent(this.selectedSessionId)}/files/${relativePath.split('/').map(encodeURIComponent).join('/')}`,
      name
    );
  }

  async rerunAnalysis(): Promise<void> {
    if (!this.selectedSessionId || this.selectedSummary?.sandbox) return;
    try {
      await this.api.request<{ ok?: boolean }>(`/api/sessions/${encodeURIComponent(this.selectedSessionId)}/analyse`, { method: 'POST' });
      this.analysisStatus = await this.api.request<{ status?: string; progress_pct?: number; last_line?: string }>(`/api/sessions/${encodeURIComponent(this.selectedSessionId)}/analyse/status`);
      this.snackBar.open('Analysis job requested. Refresh status to view completion.', 'Dismiss', { duration: 5000 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Analysis could not be started.', 'Dismiss', { duration: 6000 });
    }
  }

  async refreshAnalysisStatus(): Promise<void> {
    if (!this.selectedSessionId) return;
    this.analysisStatus = await this.api.request<{ status?: string; progress_pct?: number; last_line?: string }>(`/api/sessions/${encodeURIComponent(this.selectedSessionId)}/analyse/status`);
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

  // Only render recorded or currently streamed series; never invent report data.
  // ---- Operator-selectable session comparison (read-only; reuses the
  // existing /data and /summary routes for the second session) ----

  compareCandidates(): SessionRecord[] {
    return this.sessions.filter(item => item.session_id && item.session_id !== this.selectedSessionId);
  }

  async loadCompareSession(sessionId: string): Promise<void> {
    this.compareSessionId = sessionId;
    if (!sessionId) {
      this.compareSummary = null;
      this.compareRows = [];
      setTimeout(() => this.drawReportTrends(), 0);
      return;
    }
    this.compareLoading = true;
    try {
      const sessionPath = `/api/sessions/${encodeURIComponent(sessionId)}`;
      const [summary, data] = await Promise.all([
        this.api.request<SessionRecord>(`${sessionPath}/summary`),
        this.api.request<{ rows?: Array<Record<string, number | string | null>> }>(`${sessionPath}/data?points=1000`)
      ]);
      this.compareSummary = summary;
      this.compareRows = Array.isArray(data?.rows) ? data.rows : [];
    } catch (error: unknown) {
      this.compareSummary = null;
      this.compareRows = [];
      this.snackBar.open(error instanceof Error ? error.message : 'Comparison session could not be loaded.', 'Dismiss', { duration: 5000 });
    } finally {
      this.compareLoading = false;
      this.cdr.markForCheck();
      setTimeout(() => this.drawReportTrends(), 0);
    }
  }

  private seriesMean(rows: Array<Record<string, number | string | null>>, key: string): number | null {
    // Exclude invalid publishes: the firmware emits 0 for held frames, and a
    // held frame can also carry a stale non-zero value flagged by the
    // logged_*_valid columns. Legacy rows without validity columns fall back
    // to zero-exclusion only.
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

  compareDeltaRows(): Array<{ label: string; selected: string; compare: string; delta: string; tone: 'better' | 'worse' | 'neutral' }> {
    if (!this.compareSummary) return [];
    const fmt = (value: number | null, digits = 1) => (value === null ? '--' : value.toFixed(digits));
    const rows: Array<{ label: string; selected: string; compare: string; delta: string; tone: 'better' | 'worse' | 'neutral' }> = [];

    const meanPairs: Array<{ label: string; key: string }> = [
      { label: 'Mean HR (bpm)', key: 'reported_hr' },
      { label: 'Mean RR (br/min)', key: 'reported_rr' }
    ];
    for (const pair of meanPairs) {
      const a = this.seriesMean(this.sessionDataRows, pair.key);
      const b = this.seriesMean(this.compareRows, pair.key);
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
    const covA = coverageOf(this.selectedSummary);
    const covB = coverageOf(this.compareSummary);
    rows.push({
      label: 'HR coverage (%)',
      selected: fmt(covA),
      compare: fmt(covB),
      delta: covA !== null && covB !== null ? (covA - covB >= 0 ? '+' : '') + (covA - covB).toFixed(1) : '--',
      tone: covA !== null && covB !== null ? (covA > covB ? 'better' : covA < covB ? 'worse' : 'neutral') : 'neutral'
    });

    rows.push({
      label: 'Verdict',
      selected: this.verdictText(this.reportRecord()),
      compare: this.verdictText(this.compareSummary),
      delta: '--',
      tone: 'neutral'
    });
    return rows;
  }

  private drawReportTrends() {
    if (!this.hrReportCanvas || !this.rrReportCanvas) return;

    const hrPoints = this.sessionDataRows.map(row => Number(row['reported_hr'])).filter(Number.isFinite);
    const rrPoints = this.sessionDataRows.map(row => Number(row['reported_rr'])).filter(Number.isFinite);
    const hrCompare = this.compareRows.map(row => Number(row['reported_hr'])).filter(Number.isFinite);
    const rrCompare = this.compareRows.map(row => Number(row['reported_rr'])).filter(Number.isFinite);

    const plotReportTrend = (canvasRef: ElementRef<HTMLCanvasElement>, color: string, points: number[], overlay: number[] = []) => {
      const canvas = canvasRef.nativeElement;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      const dpr = window.devicePixelRatio || 1;

      canvas.width = w * dpr;
      canvas.height = h * dpr;
      const ctx = canvas.getContext('2d');
      if (!ctx) return;
      ctx.resetTransform();
      ctx.scale(dpr, dpr);
      ctx.clearRect(0, 0, w, h);

      if (!Array.isArray(points) || points.length < 2) {
        ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--md-sys-color-on-surface-variant').trim() || '#64748b';
        ctx.font = '12px Inter, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('No recorded series in this summary.', w / 2, h / 2);
        return;
      }

      const pad = 12;
      const innerW = w - pad * 2;
      const innerH = h - pad * 2;
      const count = points.length;

      const outlineColor = getComputedStyle(document.documentElement).getPropertyValue('--md-sys-color-outline-variant').trim() || '#f1f5f9';
      ctx.strokeStyle = outlineColor;
      ctx.lineWidth = 1;
      for (let i = 0; i <= 3; i++) {
        const y = pad + (innerH * i / 3);
        ctx.beginPath();
        ctx.moveTo(pad, y);
        ctx.lineTo(w - pad, y);
        ctx.stroke();
      }

      // Shared vertical scale so the dashed comparison overlay is read against
      // the same axis as the selected session.
      const scalePool = overlay.length >= 2 ? points.concat(overlay) : points;
      const minV = Math.min(...scalePool) - 5;
      const maxV = Math.max(...scalePool) + 5;
      const diff = maxV - minV;

      const traceLine = (series: number[]) => {
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

      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      traceLine(points);

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

    plotReportTrend(this.hrReportCanvas, 'rgba(0, 164, 150, 0.95)', hrPoints, hrCompare);
    plotReportTrend(this.rrReportCanvas, 'rgba(97, 105, 198, 0.95)', rrPoints, rrCompare);
  }
}
