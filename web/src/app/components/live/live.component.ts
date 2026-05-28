import { ChangeDetectionStrategy, Component, inject, OnInit, OnDestroy, ElementRef, ViewChild, AfterViewInit, effect, HostListener, signal, computed } from '@angular/core';
import { DatePipe, UpperCasePipe } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Router } from '@angular/router';

// Angular Material 3 modules
import { MatCardModule } from '@angular/material/card';
import { MatTabsModule } from '@angular/material/tabs';
import { MatTabChangeEvent } from '@angular/material/tabs';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatTableModule } from '@angular/material/table';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MatMenuModule } from '@angular/material/menu';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatChipsModule } from '@angular/material/chips';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { firstValueFrom } from 'rxjs';

import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';
import { TelemetryService } from '../../services/telemetry.service';
import { AudioService } from '../../services/audio.service';
import { ConfirmDialogComponent } from '../confirm-dialog/confirm-dialog.component';
import { KpiZoomDialogComponent } from '../kpi-zoom-dialog/kpi-zoom-dialog.component';

type TrendRange = 30 | 60 | 120 | 'max';

interface BiasBucket {
  label: string;
  bias: number;
  count: number;
  width: number;
}

@Component({
  selector: 'app-live',
  imports: [
    DatePipe,
    UpperCasePipe,
    FormsModule,
    MatCardModule,
    MatTabsModule,
    MatButtonModule,
    MatIconModule,
    MatFormFieldModule,
    MatInputModule,
    MatTableModule,
    MatDialogModule,
    MatMenuModule,
    MatSnackBarModule,
    MatButtonToggleModule,
    MatChipsModule,
    MatProgressBarModule,
    MatProgressSpinnerModule
  ],
  templateUrl: './live.component.html',
  styleUrl: './live.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class LiveComponent implements OnInit, OnDestroy, AfterViewInit {
  protected readonly state = inject(StateService);
  protected readonly api = inject(ApiService);
  protected readonly telemetry = inject(TelemetryService);
  protected readonly audio = inject(AudioService);
  private readonly router = inject(Router);
  private readonly dialog = inject(MatDialog);
  private readonly snackBar = inject(MatSnackBar);

  protected readonly Math = Math;
  protected readonly NaN = Number.NaN;

  // Canvas element references for dynamic renderers
  @ViewChild('breathCanvas', { static: false }) breathCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('heartCanvas', { static: false }) heartCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('hrTrendCanvas', { static: false }) hrTrendCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('rrTrendCanvas', { static: false }) rrTrendCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('targetCanvas', { static: false }) targetCanvas!: ElementRef<HTMLCanvasElement>;

  // Mini sparkline references
  @ViewChild('overviewHrSpark', { static: false }) overviewHrSpark!: ElementRef<HTMLCanvasElement>;
  @ViewChild('overviewRrSpark', { static: false }) overviewRrSpark!: ElementRef<HTMLCanvasElement>;
  @ViewChild('overviewFpsSpark', { static: false }) overviewFpsSpark!: ElementRef<HTMLCanvasElement>;
  @ViewChild('overviewDistSpark', { static: false }) overviewDistSpark!: ElementRef<HTMLCanvasElement>;

  private animeFrameId: number | null = null;
  private resizeObserver: ResizeObserver | null = null;
  private viewReady = false;
  private readonly onVisibilityChange = () => this.requestCanvasDraw();
  
  // Local active tab selector
  activeTabIndex = 0;
  sessionNotesInput = '';
  customTagInput = '';
  protected readonly trendRange = signal<TrendRange>(120);

  constructor() {
    effect(() => {
      const t = this.state.activeTab();
      const tabs = ['tab-overview', 'tab-waves', 'tab-hr', 'tab-rr', 'tab-snaps', 'tab-audit'];
      const idx = tabs.indexOf(t);
      if (idx !== -1) {
        this.activeTabIndex = idx;
      }
      this.state.lastPayload();
      this.state.spark();
      this.state.kpiThresholds();
      this.state.theme(); // Redraw on theme change
      this.requestCanvasDraw();
    });
  }

  ngOnInit() {
    this.sessionNotesInput = this.state.sessionNotes()[this.state.currentSessionId() || ''] || '';
  }

  ngAfterViewInit() {
    this.viewReady = true;
    if (typeof ResizeObserver !== 'undefined') {
      this.resizeObserver = new ResizeObserver(() => this.requestCanvasDraw());
      [
        this.breathCanvas, this.heartCanvas, this.hrTrendCanvas, this.rrTrendCanvas,
        this.targetCanvas, this.overviewHrSpark, this.overviewRrSpark,
        this.overviewFpsSpark, this.overviewDistSpark
      ].filter(Boolean).forEach(ref => this.resizeObserver?.observe(ref.nativeElement));
    }
    document.addEventListener('visibilitychange', this.onVisibilityChange);
    this.requestCanvasDraw();
  }

  ngOnDestroy() {
    if (this.animeFrameId !== null) {
      cancelAnimationFrame(this.animeFrameId);
    }
    this.resizeObserver?.disconnect();
    document.removeEventListener('visibilitychange', this.onVisibilityChange);
  }

  onTabChange(event: MatTabChangeEvent) {
    const tabs = ['tab-overview', 'tab-waves', 'tab-hr', 'tab-rr', 'tab-snaps', 'tab-audit'];
    this.activeTabIndex = event.index;
    this.state.activeTab.set(tabs[event.index]);
    this.state.triggerHaptic('tap');
    this.requestCanvasDraw();
  }

  @HostListener('document:keydown', ['$event'])
  onKeyboardShortcut(event: KeyboardEvent): void {
    if (event.defaultPrevented || this.state.currentView() !== 'live' || event.repeat) return;
    const target = event.target instanceof Element ? event.target : null;
    if (target?.closest('input, textarea, select, [contenteditable], [role="textbox"]')) return;

    const key = event.key.toLowerCase();
    if (event.altKey && !event.ctrlKey && !event.metaKey && !event.shiftKey) {
      const ranges: Record<string, TrendRange> = { '7': 30, '8': 60, '9': 120, '0': 'max' };
      const range = ranges[key];
      if (range !== undefined) {
        event.preventDefault();
        this.setTrendRange(range);
      }
      return;
    }
    if (event.ctrlKey || event.metaKey || event.altKey || event.shiftKey) return;

    const tags: Record<string, string> = {
      m: 'Motion',
      c: 'Cough',
      s: 'Speaking',
      b: 'Baseline'
    };
    if (tags[key]) {
      event.preventDefault();
      this.addQuickTag(tags[key]);
      return;
    }
    if (key === 'p') {
      event.preventDefault();
      this.captureSnapshot();
      return;
    }
    if (key === 'i') {
      event.preventDefault();
      this.annotateLatestSnapshot();
      return;
    }
    if (key === 'o' || key === 'v') {
      event.preventDefault();
      this.openLiveTab(key === 'o' ? 'tab-overview' : 'tab-waves');
      return;
    }
    if (key === 'z') {
      event.preventDefault();
      this.resetTrendRange();
    }
  }

  togglePause() {
    this.state.paused.set(!this.state.paused());
    this.state.triggerHaptic('tap');
  }

  async stopSession() {
    this.state.triggerHaptic('confirm');
    const confirmed = await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
      data: {
        title: 'Stop monitoring session?',
        message: 'Stop telemetry capture and compile the recorded report.',
        confirmLabel: 'Stop session'
      },
      restoreFocus: true
    }).afterClosed());
    if (confirmed) {
      try {
        this.state.ctlStopPending.set(true);
        const r = await this.api.request<{ ok?: boolean }>('/api/session/stop', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ reason: 'user_request' })
        });
        if (r && r.ok) {
          this.state.currentSessionId.set(null);
          this.state.sessionActive.set(false);
          this.audio.speakAlert('Session completed successfully.', 'ok', true);
          this.router.navigate(['/report']);
        }
      } catch (error: unknown) {
        const message = error instanceof Error ? error.message : String(error);
        this.snackBar.open(`Stop session failed: ${message}`, 'Dismiss', { duration: 7000 });
      } finally {
        this.state.ctlStopPending.set(false);
      }
    }
  }

  openKpiZoomDialog(metricKey: 'hr' | 'rr' | 'fps' | 'dist'): void {
    this.state.triggerHaptic('tap');
    let title = '';
    let unit = '';
    let color = '';

    switch (metricKey) {
      case 'hr':
        title = 'Heart Rate';
        unit = 'bpm';
        color = 'rgba(0, 164, 150, 0.95)';
        break;
      case 'rr':
        title = 'Respiration';
        unit = 'br/min';
        color = 'rgba(97, 105, 198, 0.95)';
        break;
      case 'fps':
        title = 'Frame Rate';
        unit = 'Hz';
        color = 'rgba(100, 116, 139, 0.95)';
        break;
      case 'dist':
        title = 'Target Range';
        unit = 'cm';
        color = 'rgba(14, 165, 233, 0.95)';
        break;
    }

    this.dialog.open(KpiZoomDialogComponent, {
      data: { metric: metricKey, title, unit, color },
      restoreFocus: true
    });
  }

  captureSnapshot() {
    const payload = this.state.lastPayload();
    if (!payload) return;

    this.state.triggerHaptic('success');
    const snapId = 'snap_' + Date.now();
    const snap = {
      id: snapId,
      ts: Date.now(),
      reported_hr: payload.radar?.reported_hr || 0,
      reported_rr: payload.radar?.reported_rr || 0,
      distance_cm: payload.radar?.distance_cm || 0,
      ble_hr: payload.ble?.hr || 0,
      ble_rr: payload.ble?.rr || 0
    };

    this.state.snaps.update(s => [...s, snap]);
    this.state.snapNotes.update(n => ({ ...n, [snapId]: '' }));
  }

  deleteSnap(snapId: string) {
    this.state.triggerHaptic('tap');
    this.state.snaps.update(s => s.filter(x => x.id !== snapId));
    this.state.snapNotes.update(n => {
      const next = { ...n };
      delete next[snapId];
      return next;
    });
  }

  async clearAllSnaps() {
    this.state.triggerHaptic('destructiveAccept');
    const confirmed = await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
      data: {
        title: 'Clear all snapshots?',
        message: 'This removes the pinned frames and their notes from this dashboard.',
        confirmLabel: 'Clear snapshots'
      },
      restoreFocus: true
    }).afterClosed());
    if (confirmed) {
      this.state.snaps.set([]);
      this.state.snapNotes.set({});
    }
  }

  updateSnapNote(snapId: string, val: string) {
    this.state.snapNotes.update(n => ({ ...n, [snapId]: val }));
  }

  saveSessionNotes(val: string) {
    this.sessionNotesInput = val;
    const sid = this.state.currentSessionId();
    if (sid) {
      this.state.sessionNotes.update(n => ({ ...n, [sid]: val }));
    }
  }

  addQuickTag(tag: string) {
    this.state.triggerHaptic('tap');
    const time = new Date().toLocaleTimeString('en-GB', { hour12: false });
    const entry = `[${time}] ${tag}`;
    const next = this.sessionNotesInput.trim()
      ? `${this.sessionNotesInput.trimEnd()}\n${entry}`
      : entry;
    this.saveSessionNotes(next);
  }

  addCustomTag(): void {
    const tag = this.customTagInput.trim();
    if (!tag) return;
    this.addQuickTag(tag.slice(0, 80));
    this.customTagInput = '';
  }

  exportSessionNotes(): void {
    const session = this.state.currentSessionId() || 'local-session';
    const content = [
      `Session: ${session}`,
      `Exported: ${new Date().toISOString()}`,
      '',
      this.sessionNotesInput.trim() || 'No operator notes recorded.'
    ].join('\n');
    this.downloadText(content, `session-notes-${session}.txt`, 'text/plain');
  }

  private openLiveTab(tab: string): void {
    const tabs = ['tab-overview', 'tab-waves', 'tab-hr', 'tab-rr', 'tab-snaps', 'tab-audit'];
    this.state.activeTab.set(tab);
    this.activeTabIndex = Math.max(0, tabs.indexOf(tab));
    this.requestCanvasDraw();
  }

  private annotateLatestSnapshot(): void {
    if (this.state.snaps().length === 0) this.captureSnapshot();
    const latest = this.state.snaps().at(-1);
    if (!latest) return;
    this.state.activeSnapNoteId.set(latest.id);
    this.openLiveTab('tab-snaps');
    queueMicrotask(() => {
      document.querySelector<HTMLInputElement>(`.snap-card[data-snapshot-id="${latest.id}"] input`)?.focus();
    });
  }

  async recordObservation(): Promise<void> {
    const note = this.sessionNotesInput.trim();
    if (!note) return;
    try {
      await this.api.request<{ ok?: boolean }>('/api/session/annotate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ note })
      });
      this.snackBar.open('Observation recorded in the active session.', 'Dismiss', { duration: 3500 });
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'annotation failed';
      this.snackBar.open(`Observation saved locally only: ${message}`, 'Dismiss', { duration: 6000 });
    }
  }

  formatFault(fault: string | Record<string, unknown>): string {
    if (typeof fault === 'string') return fault;
    return String(fault['message'] || fault['msg'] || fault['code'] || 'Reported sensor fault');
  }

  formatEvent(event: string | Record<string, unknown>): string {
    if (typeof event === 'string') return event;
    return String(event['message'] || event['msg'] || event['event'] || event['type'] || 'Telemetry event');
  }

  eventTime(event: string | Record<string, unknown>): string {
    if (typeof event === 'string') return '';
    return String(event['created_at'] || event['ts'] || '');
  }

  downloadChart(canvas: HTMLCanvasElement, label: string) {
    const anchor = document.createElement('a');
    anchor.href = canvas.toDataURL('image/png');
    anchor.download = `${label}_${Date.now()}.png`;
    anchor.click();
    this.state.triggerHaptic('success');
  }

  setTrendRange(range: TrendRange): void {
    this.trendRange.set(range);
    this.state.triggerHaptic('tap');
    this.requestCanvasDraw();
  }

  resetTrendRange(): void {
    this.trendRange.set(120);
    this.snackBar.open('Chart window reset to 120 seconds.', 'Dismiss', { duration: 2500 });
    this.state.triggerHaptic('tap');
    this.requestCanvasDraw();
  }

  trendRangeLabel(): string {
    return this.trendRange() === 'max' ? 'Maximum history' : `${this.trendRange()} seconds`;
  }

  metricNumber(key: string): number | null {
    const value = this.telemetryValue(key);
    const numeric = Number(value);
    return Number.isFinite(numeric) ? numeric : null;
  }

  metricText(key: string, decimals = 1, suffix = ''): string {
    const value = this.metricNumber(key);
    return value === null ? '--' : `${value.toFixed(decimals)}${suffix}`;
  }

  readingLabel(label: string, key: string, unit: string): string {
    const value = this.metricNumber(key);
    return value === null ? `${label}: not available.` : `${label}: ${Math.round(value)} ${unit}.`;
  }

  frameRateLabel(): string {
    const fps = this.computedFps();
    return fps === '--' ? 'Frame rate: not available.' : `Frame rate: ${fps} hertz.`;
  }

  chartLabel(label: string, key: string, unit: string): string {
    return `${label} chart. ${this.readingLabel('Latest reading', key, unit)}`;
  }

  liveAlertAnnouncement(): string {
    const alerts: string[] = [];
    const thresholds = this.state.kpiThresholds();
    const hr = this.metricNumber('reported_hr');
    const rr = this.metricNumber('reported_rr');
    if (this.state.telemetryStale()) alerts.push('Live telemetry is stale. Do not treat displayed values as current.');
    if (hr !== null && (hr < thresholds.hrLow || hr > thresholds.hrHigh)) {
      alerts.push(`Heart rate outside threshold at ${Math.round(hr)} beats per minute.`);
    }
    if (rr !== null && (rr < thresholds.rrLow || rr > thresholds.rrHigh)) {
      alerts.push(`Respiration outside threshold at ${Math.round(rr)} breaths per minute.`);
    }
    return alerts.join(' ');
  }

  metricLabel(key: string): string {
    const value = this.telemetryValue(key);
    if (value === undefined || value === null || value === '') return '--';
    return String(value);
  }

  metricState(key: string): 'Yes' | 'No' | '--' {
    const value = this.telemetryValue(key);
    if (value === undefined || value === null || value === '') return '--';
    if (typeof value === 'string') {
      const normalized = value.toLowerCase();
      if (['true', 'yes', 'ok', 'pass', 'passed', '1'].includes(normalized)) return 'Yes';
      if (['false', 'no', 'fail', 'failed', '0'].includes(normalized)) return 'No';
    }
    return Number(value) >= 0.5 || value === true ? 'Yes' : 'No';
  }

  qualityPercent(key: string): number {
    const value = this.metricNumber(key);
    return value === null ? 0 : Math.max(0, Math.min(100, value * 100));
  }

  qualityLabel(key: string): string {
    const value = this.metricNumber(key);
    return value === null ? 'SQI waiting for signal' : `SQI ${(value * 100).toFixed(0)}%`;
  }

  protected readonly computedFps = computed(() => {
    const direct = this.metricNumber('fps_hz');
    if (direct !== null) return direct.toFixed(1);
    const timestamps = this.seriesNumbers('ts', 't');
    if (timestamps.length < 3) return '--';
    const deltas = timestamps.slice(1).map((value, index) => value - timestamps[index]).filter(value => value > 0);
    if (deltas.length === 0) return '--';
    deltas.sort((a, b) => a - b);
    const median = deltas[Math.floor(deltas.length / 2)];
    return median > 0 ? (1 / median).toFixed(1) : '--';
  });

  protected readonly hrBiasBuckets = computed(() => {
    const raw = this.seriesNumbers('raw_hr_uncorrected', 'raw_hr');
    const reference = this.seriesNumbers('ref_hr', 'ble_hr');
    const count = Math.min(raw.length, reference.length);
    if (count === 0) return [];
    const buckets = new Map<number, { sum: number; count: number }>();
    for (let index = 0; index < count; index += 1) {
      const rawValue = raw[raw.length - count + index];
      const refValue = reference[reference.length - count + index];
      if (rawValue <= 0 || refValue <= 0) continue;
      const bucket = Math.floor(refValue / 10) * 10;
      // PERFORMANCE OPTIMIZATION: Avoid unconditional object allocation and map set on every iteration.
      // Instead, fetch the existing entry. If it exists, update it in place. If it doesn't, create it once.
      // Measured impact: Reduces allocation overhead and Map.set operations, improving bucket aggregation speed.
      const entry = buckets.get(bucket);
      if (entry) {
        entry.sum += rawValue - refValue;
        entry.count += 1;
      } else {
        buckets.set(bucket, { sum: rawValue - refValue, count: 1 });
      }
    }
    const values = [...buckets.entries()].sort(([a], [b]) => a - b).slice(-6);
    const maxBias = Math.max(1, ...values.map(([, entry]) => Math.abs(entry.sum / entry.count)));
    return values.map(([bucket, entry]) => {
      const bias = entry.sum / entry.count;
      return {
        label: `${bucket}-${bucket + 9} bpm`,
        bias,
        count: entry.count,
        width: Math.max(4, Math.abs(bias) / maxBias * 100)
      };
    });
  });

  protected readonly rrWarnings = computed(() => {
    const warnings: string[] = [];
    if (this.metricState('rr_anchor_fresh') === 'No') warnings.push('RR anchor is stale; publish values may rely on aged state.');
    const age = this.metricNumber('candidate_rr_age_ms');
    if (age !== null && age > 10000) warnings.push('RR candidate is older than 10 seconds.');
    if (this.metricState('rr_midsession_raw_reanchor_blocked') === 'Yes') {
      warnings.push(`Mid-session re-anchor blocked: ${this.metricLabel('rr_midsession_raw_reanchor_reason')}.`);
    }
    if (this.state.telemetryStale()) warnings.push('Live telemetry payload is stale.');
    return warnings;
  });

  analysisMetric(key: string): string {
    const analysis = this.state.lastPayload()?.analysis;
    const value = analysis?.[key];
    if (value === undefined || value === null || value === '') return '--';
    return typeof value === 'object' ? JSON.stringify(value) : String(value);
  }

  analysisNested(recordKey: string, key: string, decimals?: number): string {
    const analysis = this.state.lastPayload()?.analysis;
    const record = analysis?.[recordKey];
    if (!record || typeof record !== 'object') return '--';
    const value = (record as Record<string, unknown>)[key];
    if (decimals !== undefined) {
      const number = Number(value);
      return Number.isFinite(number) ? number.toFixed(decimals) : '--';
    }
    return value === undefined || value === null || value === '' ? '--' : String(value);
  }

  histogramSummary(key: string): string {
    const raw = this.state.lastPayload()?.analysis?.[key];
    if (!raw || typeof raw !== 'object') return '--';
    const entries = Object.entries(raw as Record<string, unknown>)
      .map(([label, value]) => [label, Number(value)] as const)
      .filter(([, value]) => Number.isFinite(value))
      .sort((a, b) => b[1] - a[1])
      .slice(0, 3);
    return entries.length > 0 ? entries.map(([label, value]) => `${label}=${value}`).join(' | ') : '--';
  }

  exportAuditLog(format: 'json' | 'csv'): void {
    const payload = this.state.lastPayload();
    const eventRows = (payload?.events || []).map(event => ({
      timestamp: this.eventTime(event),
      kind: 'event',
      severity: '',
      message: this.formatEvent(event)
    }));
    const alertRows = this.state.alertHistory().map(alert => ({
      timestamp: new Date(alert.ts).toISOString(),
      kind: 'alert',
      severity: alert.severity,
      message: alert.msg
    }));
    const rows = [...eventRows, ...alertRows];
    const content = format === 'json'
      ? JSON.stringify({
          exported_at: new Date().toISOString(),
          connection: this.state.ctlStatus(),
          analysis: payload?.analysis || null,
          rows
        }, null, 2)
      : [
          'timestamp,kind,severity,message',
          ...rows.map(row => [row.timestamp, row.kind, row.severity, row.message]
            .map(value => `"${String(value).replaceAll('"', '""')}"`).join(','))
        ].join('\n');
    this.downloadText(content, `radar-vital-audit.${format}`, format === 'json' ? 'application/json' : 'text/csv');
  }

  private downloadText(content: string, filename: string, type: string): void {
    const href = URL.createObjectURL(new Blob([content], { type }));
    const anchor = document.createElement('a');
    anchor.href = href;
    anchor.download = filename;
    anchor.click();
    URL.revokeObjectURL(href);
    this.state.triggerHaptic('success');
  }

  private telemetryValue(key: string): unknown {
    const payload = this.state.lastPayload();
    const radarValue = payload?.radar?.[key];
    if (radarValue !== undefined && radarValue !== null && radarValue !== '') return radarValue;
    const series = payload?.series as Record<string, unknown> | undefined;
    const values = series?.[key];
    return Array.isArray(values) && values.length > 0 ? values[values.length - 1] : undefined;
  }

  private seriesNumbers(...keys: string[]): number[] {
    const series = this.state.lastPayload()?.series as Record<string, unknown> | undefined;
    for (const key of keys) {
      const values = series?.[key];
      if (Array.isArray(values) && values.length > 0) {
        return values.map(value => Number(value)).filter(value => Number.isFinite(value));
      }
    }
    return [];
  }

  private trimTrend(points: number[]): number[] {
    const range = this.trendRange();
    return range === 'max' ? points : points.slice(-range);
  }

  // --- Real-time Waveforms & Trends Canvas plots ---
  private requestCanvasDraw(): void {
    if (!this.viewReady || document.visibilityState === 'hidden' || this.animeFrameId !== null) return;
    this.animeFrameId = requestAnimationFrame(() => {
      this.animeFrameId = null;
      this.drawWaves();
      this.drawTrends();
      this.drawOverviewSparklines();
      this.drawTargetPosition();
    });
  }

  private drawWaves() {
    // Only render waveforms if active tab is Waves (index 1)
    if (this.activeTabIndex !== 1) return;

    const payload = this.state.lastPayload();
    if (!payload || !payload.series) return;

    const breathPoints = this.seriesNumbers('breath_phase', 'breath');
    const heartPoints = this.seriesNumbers('heart_phase', 'heart');

    const drawPhase = (canvasRef: ElementRef<HTMLCanvasElement>, points: number[], color: string) => {
      const canvas = canvasRef.nativeElement;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
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

      if (points.length < 2) return;

      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();

      const pad = 8;
      const innerW = w - pad * 2;
      const innerH = h - pad * 2;
      const count = points.length;

      points.forEach((val, idx) => {
        const x = pad + (idx / (count - 1)) * innerW;
        // phase signals normalized roughly between -1.5 and 1.5
        const y = pad + innerH / 2 - (val / 1.5) * (innerH / 2);
        
        if (idx === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
    };

    if (this.breathCanvas) {
      drawPhase(this.breathCanvas, breathPoints, 'rgba(97, 105, 198, 0.95)');
    }
    if (this.heartCanvas) {
      drawPhase(this.heartCanvas, heartPoints, 'rgba(0, 164, 150, 0.95)');
    }
  }

  private drawTrends() {
    // Render HR trend (index 2) or RR trend (index 3)
    const payload = this.state.lastPayload();
    if (!payload || !payload.series) return;

    const plotTrend = (canvasRef: ElementRef<HTMLCanvasElement>, data: number[], color: string, minV: number, maxV: number, type?: 'hr' | 'rr') => {
      const canvas = canvasRef.nativeElement;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
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

      const pad = 16;
      const innerW = w - pad * 2;
      const innerH = h - pad * 2;
      const count = data.length;
      const diff = Math.max(1, maxV - minV);

      // Render threshold breach warning bands
      if (type) {
        const thresholds = this.state.kpiThresholds();
        const lowLimit = type === 'hr' ? thresholds.hrLow : thresholds.rrLow;
        const highLimit = type === 'hr' ? thresholds.hrHigh : thresholds.rrHigh;

        const yLow = pad + innerH - ((lowLimit - minV) / diff) * innerH;
        const yHigh = pad + innerH - ((highLimit - minV) / diff) * innerH;

        ctx.fillStyle = 'rgba(239, 68, 68, 0.05)';
        if (yLow < pad + innerH) {
          ctx.fillRect(pad, yLow, innerW, (pad + innerH) - yLow);
        }
        if (yHigh > pad) {
          ctx.fillRect(pad, pad, innerW, yHigh - pad);
        }

        ctx.strokeStyle = 'rgba(239, 68, 68, 0.4)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);

        if (lowLimit >= minV && lowLimit <= maxV) {
          ctx.beginPath();
          ctx.moveTo(pad, yLow);
          ctx.lineTo(w - pad, yLow);
          ctx.stroke();
        }
        if (highLimit >= minV && highLimit <= maxV) {
          ctx.beginPath();
          ctx.moveTo(pad, yHigh);
          ctx.lineTo(w - pad, yHigh);
          ctx.stroke();
        }
        ctx.setLineDash([]);
      }

      // Draw gridlines
      const outlineColor = getComputedStyle(document.documentElement).getPropertyValue('--md-sys-color-outline-variant').trim() || '#e2e8f0';
      ctx.strokeStyle = outlineColor;
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i++) {
        const y = pad + (innerH * i / 4);
        ctx.beginPath();
        ctx.moveTo(pad, y);
        ctx.lineTo(w - pad, y);
        ctx.stroke();
      }

      if (data.length < 2) return;

      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.beginPath();

      data.forEach((val, idx) => {
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

    const thresholds = this.state.kpiThresholds();
    if (this.activeTabIndex === 2 && this.hrTrendCanvas) {
      plotTrend(this.hrTrendCanvas, this.trimTrend(this.seriesNumbers('reported_hr', 'hr')), 'rgba(0, 164, 150, 0.95)', Math.max(0, thresholds.hrLow - 20), thresholds.hrHigh + 20, 'hr');
    }
    if (this.activeTabIndex === 3 && this.rrTrendCanvas) {
      plotTrend(this.rrTrendCanvas, this.trimTrend(this.seriesNumbers('reported_rr', 'rr')), 'rgba(97, 105, 198, 0.95)', Math.max(0, thresholds.rrLow - 5), thresholds.rrHigh + 5, 'rr');
    }
  }

  private drawOverviewSparklines() {
    // Only render mini sparklines if Overview tab is active (index 0)
    if (this.activeTabIndex !== 0) return;

    const spark = this.state.spark();
    
    const drawMiniSpark = (canvasRef: ElementRef<HTMLCanvasElement>, data: number[], color: string) => {
      const canvas = canvasRef.nativeElement;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
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

      if (data.length < 2) return;

      const minV = Math.min(...data);
      const maxV = Math.max(...data);
      const diff = Math.max(1, maxV - minV);

      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();

      const count = data.length;
      data.forEach((val, idx) => {
        const x = (idx / (count - 1)) * w;
        const y = h - 2 - ((val - minV) / diff) * (h - 4);
        
        if (idx === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
    };

    if (this.overviewHrSpark) drawMiniSpark(this.overviewHrSpark, spark.hr, 'rgba(0, 164, 150, 0.8)');
    if (this.overviewRrSpark) drawMiniSpark(this.overviewRrSpark, spark.rr, 'rgba(97, 105, 198, 0.8)');
    if (this.overviewFpsSpark) drawMiniSpark(this.overviewFpsSpark, spark.fps, 'rgba(100, 116, 139, 0.8)');
    if (this.overviewDistSpark) drawMiniSpark(this.overviewDistSpark, spark.dist, 'rgba(14, 165, 233, 0.8)');
  }

  private drawTargetPosition(): void {
    if (this.activeTabIndex !== 0 || !this.targetCanvas) return;
    const canvas = this.targetCanvas.nativeElement;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== width * dpr || canvas.height !== height * dpr) {
      canvas.width = width * dpr;
      canvas.height = height * dpr;
    }
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.resetTransform();
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, width, height);
    const originX = width / 2;
    const originY = height - 12;
    const radius = Math.min(width / 2 - 14, height - 24);
    ctx.strokeStyle = 'rgba(97, 105, 198, 0.22)';
    ctx.lineWidth = 1;
    for (const factor of [0.33, 0.66, 1]) {
      ctx.beginPath();
      ctx.arc(originX, originY, radius * factor, Math.PI, Math.PI * 2);
      ctx.stroke();
    }
    ctx.beginPath();
    ctx.moveTo(originX, originY);
    ctx.lineTo(originX, originY - radius);
    ctx.stroke();
    const x = this.metricNumber('primary_x');
    const y = this.metricNumber('primary_y');
    if (x === null || y === null || y < 0) return;
    const scale = radius / 3;
    const pointX = originX + x * scale;
    const pointY = originY - y * scale;
    const gradient = ctx.createRadialGradient(pointX, pointY, 1, pointX, pointY, 18);
    gradient.addColorStop(0, 'rgba(0, 164, 150, 1)');
    gradient.addColorStop(1, 'rgba(0, 164, 150, 0)');
    ctx.fillStyle = gradient;
    ctx.beginPath();
    ctx.arc(pointX, pointY, 18, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = 'rgba(0, 164, 150, 1)';
    ctx.beginPath();
    ctx.arc(pointX, pointY, 4, 0, Math.PI * 2);
    ctx.fill();
  }
}
