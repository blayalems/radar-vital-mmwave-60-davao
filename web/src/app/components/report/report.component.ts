import { ChangeDetectionStrategy, Component, inject, OnInit, ViewChild, ElementRef, AfterViewInit, effect } from '@angular/core';
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
  analysisStatus: { status?: string; progress_pct?: number; last_line?: string } | null = null;
  sessionDataRows: Array<Record<string, number | string | null>> = [];
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
    } catch (e) {
      console.warn('Could not load sessions', e);
    }
  }

  async onSessionChange() {
    this.selectedSession = this.sessions.find(s => s.session_id === this.selectedSessionId) || null;
    this.selectedSummary = null;
    this.comparison = null;
    this.analysisStatus = null;
    this.sessionDataRows = [];
    this.summaryError = '';
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
        setTimeout(() => this.drawReportTrends(), 50);
      }
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
  private drawReportTrends() {
    if (!this.hrReportCanvas || !this.rrReportCanvas) return;

    const hrPoints = this.sessionDataRows.map(row => Number(row['reported_hr'])).filter(Number.isFinite);
    const rrPoints = this.sessionDataRows.map(row => Number(row['reported_rr'])).filter(Number.isFinite);

    const plotReportTrend = (canvasRef: ElementRef<HTMLCanvasElement>, color: string, points: number[]) => {
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

      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.beginPath();

      const minV = Math.min(...points) - 5;
      const maxV = Math.max(...points) + 5;
      const diff = maxV - minV;

      points.forEach((val, idx) => {
        const x = pad + (idx / (count - 1)) * innerW;
        const y = pad + innerH - ((val - minV) / diff) * innerH;

        if (idx === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
    };

    plotReportTrend(this.hrReportCanvas, 'rgba(0, 164, 150, 0.95)', hrPoints);
    plotReportTrend(this.rrReportCanvas, 'rgba(97, 105, 198, 0.95)', rrPoints);
  }
}
