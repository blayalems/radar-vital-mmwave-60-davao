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
import { UndoService } from '../../services/undo.service';
import { AnnotationService } from '../../services/annotation.service';
import { ChartAnnotation, SnapshotRecord } from '../../models/rvt.models';
import { ConfirmDialogComponent } from '../confirm-dialog/confirm-dialog.component';
import { ChartDataTableComponent } from '../chart-data-table/chart-data-table.component';

type BlandAltmanMetric = 'hr' | 'rr';

interface BlandAltmanPair {
  mean: number;
  diff: number;
  pqi: number;
}
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
    MatProgressSpinnerModule,
    ChartDataTableComponent
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
  protected readonly undoService = inject(UndoService);
  protected readonly annotationService = inject(AnnotationService);

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
  @ViewChild('baCanvas', { static: false }) baCanvas?: ElementRef<HTMLCanvasElement>;
  @ViewChild('breathCanvasClone', { static: false }) breathCanvasClone?: ElementRef<HTMLCanvasElement>;
  @ViewChild('heartCanvasClone', { static: false }) heartCanvasClone?: ElementRef<HTMLCanvasElement>;
  @ViewChild('hrTrendCanvasClone', { static: false }) hrTrendCanvasClone?: ElementRef<HTMLCanvasElement>;
  @ViewChild('rrTrendCanvasClone', { static: false }) rrTrendCanvasClone?: ElementRef<HTMLCanvasElement>;

  protected readonly trendRangeLimit = computed(() => { const val = this.trendRange(); return val === 'max' ? 240 : Number(val); });
  protected readonly showBreathTable = signal(false);
  protected readonly showHeartTable = signal(false);
  protected readonly showHrTable = signal(false);
  protected readonly showRrTable = signal(false);
  protected readonly compareSelection = signal<string[]>([]);
  protected readonly selectedCompareSnaps = computed(() => {
    const selected = new Set(this.compareSelection());
    return this.sortedSnaps().filter(snap => selected.has(snap.id));
  });
  protected readonly splitScreenActive = signal(false);
  protected readonly paneBTabIndex = signal<number>(2);
  protected readonly kpiOrder = signal<string[]>(['hr', 'rr', 'fps', 'dist']);
  private draggedKpi: string | null = null;
  private dragStartX = 0;
  protected readonly baMetric = signal<BlandAltmanMetric>('hr');
  private readonly baHrPairs: BlandAltmanPair[] = [];
  private readonly baRrPairs: BlandAltmanPair[] = [];
  protected readonly baStats = computed(() => {
    const pairs = this.baMetric() === 'hr' ? this.baHrPairs : this.baRrPairs;
    if (pairs.length < 2) return null;
    const diffs = pairs.map(pair => pair.diff);
    const meanDiff = diffs.reduce((sum, value) => sum + value, 0) / diffs.length;
    const variance = diffs.map(value => (value - meanDiff) ** 2).reduce((sum, value) => sum + value, 0) / diffs.length;
    const sdDiff = Math.sqrt(variance) || 0;
    const thresh = this.baMetric() === 'hr' ? 2.0 : 1.0;
    return {
      meanDiff,
      lo: meanDiff - 1.96 * sdDiff,
      hi: meanDiff + 1.96 * sdDiff,
      exceeds: Math.abs(meanDiff) > thresh,
      unit: this.baMetric() === 'hr' ? 'BPM' : 'BrPM',
      thresh
    };
  });
  protected readonly ghostSessionActive = signal(false);
  private readonly ghostHrData = signal<number[]>([]);
  private readonly ghostRrData = signal<number[]>([]);

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
      const payload = this.state.lastPayload();
      if (payload && !this.state.paused()) {
        const radar = payload.radar || {};
        const ble = payload.ble || {};
        const radarHr = Number(radar.reported_hr);
        const radarRr = Number(radar.reported_rr);
        const bleHr = Number(ble.hr);
        const bleRr = Number(ble.rr);
        const pqiHr = Number((radar['candidate_conf'] !== undefined && radar['candidate_conf'] !== null && radar['candidate_conf'] !== '') ? radar['candidate_conf'] : (this.telemetryValue('candidate_hr_conf') ?? 0.5));
        const pqiRr = Number(this.telemetryValue('candidate_rr_conf') ?? 0.5);
        if (Number.isFinite(radarHr) && Number.isFinite(bleHr) && bleHr > 0) {
          this.baHrPairs.push({ mean: (radarHr + bleHr) / 2, diff: radarHr - bleHr, pqi: pqiHr });
          if (this.baHrPairs.length > 200) this.baHrPairs.shift();
        }
        if (Number.isFinite(radarRr) && Number.isFinite(bleRr) && bleRr > 0) {
          this.baRrPairs.push({ mean: (radarRr + bleRr) / 2, diff: radarRr - bleRr, pqi: pqiRr });
          if (this.baRrPairs.length > 200) this.baRrPairs.shift();
        }
      }
      this.state.spark();
      this.state.kpiThresholds();
      this.state.theme(); // Redraw on theme change
      this.requestCanvasDraw();
    });

    effect(() => {
      const sid = this.state.currentSessionId();
      this.baHrPairs.length = 0;
      this.baRrPairs.length = 0;
      const globalNotes = this.state.sessionNotes();
      if (sid) {
        const note = globalNotes[sid] || '';
        if (this.sessionNotesInput !== note) {
          this.sessionNotesInput = note;
        }
        void this.annotationService.loadAnnotations(sid);
      }
    });
  }

  ngOnInit() {
    this.sessionNotesInput = this.state.sessionNotes()[this.state.currentSessionId() || ''] || '';
    this.initializeKpiOrder();
  }

  ngAfterViewInit() {
    this.viewReady = true;
    if (typeof ResizeObserver !== 'undefined') {
      this.resizeObserver = new ResizeObserver(() => this.requestCanvasDraw());
      [
        this.breathCanvas, this.heartCanvas, this.hrTrendCanvas, this.rrTrendCanvas,
        this.targetCanvas, this.overviewHrSpark, this.overviewRrSpark,
        this.overviewFpsSpark, this.overviewDistSpark, this.baCanvas,
        this.breathCanvasClone, this.heartCanvasClone, this.hrTrendCanvasClone, this.rrTrendCanvasClone
      ].filter((ref): ref is ElementRef<HTMLCanvasElement> => !!ref).forEach(ref => this.resizeObserver?.observe(ref.nativeElement));
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

  initializeKpiOrder() {
    try {
      const saved = localStorage.getItem('rvt-kpi-order');
      if (saved) {
        const arr = JSON.parse(saved);
        if (Array.isArray(arr) && arr.length === 4) {
          this.kpiOrder.set(arr);
        }
      }
    } catch (_) {}
  }

  onKpiDragStart(event: PointerEvent, kpi: string) {
    this.draggedKpi = kpi;
    this.dragStartX = event.clientX;
    const target = event.currentTarget as HTMLElement;
    if (target) {
      target.setPointerCapture(event.pointerId);
    }
  }

  onKpiDragMove(event: PointerEvent) {
    if (!this.draggedKpi) return;
    const deltaX = event.clientX - this.dragStartX;
    if (Math.abs(deltaX) > 80) { // threshold to swap
      const order = [...this.kpiOrder()];
      const idx = order.indexOf(this.draggedKpi);
      if (idx !== -1) {
        const targetIdx = deltaX > 0 ? idx + 1 : idx - 1;
        if (targetIdx >= 0 && targetIdx < order.length) {
          [order[idx], order[targetIdx]] = [order[targetIdx], order[idx]];
          this.kpiOrder.set(order);
          this.dragStartX = event.clientX;
          try {
            localStorage.setItem('rvt-kpi-order', JSON.stringify(order));
          } catch (_) {}
          this.state.triggerHaptic('tap');
        }
      }
    }
  }

  onKpiDragEnd(event: PointerEvent) {
    this.draggedKpi = null;
    const target = event.currentTarget as HTMLElement;
    if (target) {
      try {
        target.releasePointerCapture(event.pointerId);
      } catch (_) {}
    }
  }

  hasBleRef(): boolean {
    const p = this.state.lastPayload();
    if (!p) return false;
    const ble = p.ble || {};
    const series = p.series || {};
    return (
      (ble.hr !== undefined && ble.hr !== null && Number.isFinite(Number(ble.hr))) ||
      (ble.rr !== undefined && ble.rr !== null && Number.isFinite(Number(ble.rr))) ||
      (Array.isArray(series.ble_hr) && series.ble_hr.some(v => v !== null && Number.isFinite(Number(v)))) ||
      (Array.isArray(series.ble_rr) && series.ble_rr.some(v => v !== null && Number.isFinite(Number(v))))
    );
  }

  toggleGhostSession(): void {
    if (this.ghostSessionActive()) {
      this.ghostSessionActive.set(false);
      this.ghostHrData.set([]);
      this.ghostRrData.set([]);
      this.snackBar.open('Ghost session overlay disabled.', 'Dismiss', { duration: 2000 });
      this.requestCanvasDraw();
      return;
    }

    const sessions = this.state.sessionItems();
    if (!sessions || sessions.length === 0) {
      this.snackBar.open('No historical sessions available for ghost overlay.', 'Dismiss', { duration: 3000 });
      return;
    }

    const choice = window.prompt('Enter session ID or prefix to overlay as ghost (leave empty for most recent):');
    if (choice === null) return;

    let selectedSession = sessions[0];
    if (choice.trim()) {
      const found = sessions.find(s => String(s.session_id || s['id'] || '').includes(choice.trim()));
      if (found) {
        selectedSession = found;
      } else {
        this.snackBar.open('Session not found. Using most recent session.', 'Dismiss', { duration: 3000 });
      }
    }

    const sid = String(selectedSession.session_id || selectedSession['id'] || '');
    if (!sid) return;

    this.api.request<{ ok?: boolean; rows?: any[] }>(`/api/sessions/${sid}/data?points=120`)
      .then(resp => {
        if (resp && Array.isArray(resp.rows)) {
          const hrValues: number[] = [];
          const rrValues: number[] = [];
          resp.rows.forEach((r: any) => {
            if (r.reported_hr !== undefined && r.reported_hr !== null) hrValues.push(r.reported_hr);
            if (r.reported_rr !== undefined && r.reported_rr !== null) rrValues.push(r.reported_rr);
          });
          this.ghostHrData.set(hrValues);
          this.ghostRrData.set(rrValues);
          this.ghostSessionActive.set(true);
          this.snackBar.open(`Ghost overlay active for session: ${sid.substring(0, 12)}`, 'Dismiss', { duration: 2000 });
          this.requestCanvasDraw();
        } else {
          this.snackBar.open('No data points found in selected session.', 'Dismiss', { duration: 3000 });
        }
      })
      .catch(err => {
        console.error('Failed to load ghost session data', err);
        this.snackBar.open('Failed to load ghost session data from API.', 'Dismiss', { duration: 3000 });
      });
  }

  getAnnotationsFor(chartKey: string): ChartAnnotation[] {
    return this.annotationService.currentAnnotations().filter(a => a.chart_key === chartKey);
  }

  async deleteAnnotation(chartKey: string, ann: ChartAnnotation): Promise<void> {
    try {
      await this.annotationService.saveAnnotation(chartKey, ann, 'delete');
      this.state.triggerHaptic('tap');
      this.requestCanvasDraw();
    } catch (e) {
      this.snackBar.open('Failed to delete annotation.', 'Dismiss', { duration: 3000 });
    }
  }

  handleChartClick(event: MouseEvent, chartKey: string): void {
    const canvas = event.currentTarget as HTMLCanvasElement;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const clickX = event.clientX - rect.left;
    const w = rect.width;

    const pad = (chartKey === 'breath' || chartKey === 'heart') ? 8 : 16;
    const innerW = w - pad * 2;
    if (innerW <= 0) return;

    const xPct = Math.max(0, Math.min(1, (clickX - pad) / innerW));

    const label = window.prompt('Enter label for new annotation:');
    if (label === null || !label.trim()) return;

    const newAnn = {
      id: crypto.randomUUID(),
      label: label.trim(),
      xPct
    };

    this.annotationService.saveAnnotation(chartKey, newAnn, 'upsert')
      .then(() => {
        this.state.triggerHaptic('tap');
        this.requestCanvasDraw();
      })
      .catch(err => {
        this.snackBar.open('Failed to save annotation.', 'Dismiss', { duration: 3000 });
      });
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
    const snap: SnapshotRecord = {
      id: snapId,
      ts: Date.now(),
      reported_hr: payload.radar?.reported_hr || 0,
      reported_rr: payload.radar?.reported_rr || 0,
      distance_cm: payload.radar?.distance_cm || 0,
      ble_hr: payload.ble?.hr || 0,
      ble_rr: payload.ble?.rr || 0,
      sortOrder: this.state.snaps().length
    };

    this.state.snaps.update(s => [...s, snap]);
    this.state.snapNotes.update(n => ({ ...n, [snapId]: '' }));
    this.undoService.push({
      label: 'Capture snapshot',
      undo: () => {
        this.state.snaps.update(snaps => snaps.filter(item => item.id !== snapId));
        this.state.snapNotes.update(notes => {
          const next = { ...notes };
          delete next[snapId];
          return next;
        });
      },
      redo: () => {
        this.state.snaps.update(snaps => [...snaps, snap]);
        this.state.snapNotes.update(notes => ({ ...notes, [snapId]: notes[snapId] || '' }));
      }
    });
  }

  deleteSnap(snapId: string) {
    this.state.triggerHaptic('tap');
    const before = this.state.snaps();
    const notesBefore = this.state.snapNotes();
    this.state.snaps.update(s => s.filter(x => x.id !== snapId));
    this.state.snapNotes.update(n => {
      const next = { ...n };
      delete next[snapId];
      return next;
    });
    this.compareSelection.update(ids => ids.filter(id => id !== snapId));
    this.undoService.push({
      label: 'Delete snapshot',
      undo: () => {
        this.state.snaps.set(before);
        this.state.snapNotes.set(notesBefore);
      },
      redo: () => {
        this.state.snaps.update(snaps => snaps.filter(x => x.id !== snapId));
        this.state.snapNotes.update(notes => {
          const next = { ...notes };
          delete next[snapId];
          return next;
        });
        this.compareSelection.update(ids => ids.filter(id => id !== snapId));
      }
    });
  }

  async clearAllSnaps() {
    this.state.triggerHaptic('warning');
    const confirmed = await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
      data: {
        title: 'Delete All Snapshots',
        message: 'Are you sure you want to permanently delete all captured snapshot frames? This action cannot be undone.'
      },
      restoreFocus: true
    }).afterClosed());
    if (confirmed) {
      const before = this.state.snaps();
      const notesBefore = this.state.snapNotes();
      const compareBefore = this.compareSelection();
      this.state.snaps.set([]);
      this.state.snapNotes.set({});
      this.compareSelection.set([]);
      this.undoService.push({
        label: 'Clear snapshots',
        undo: () => {
          this.state.snaps.set(before);
          this.state.snapNotes.set(notesBefore);
          this.compareSelection.set(compareBefore);
        },
        redo: () => {
          this.state.snaps.set([]);
          this.state.snapNotes.set({});
          this.compareSelection.set([]);
        }
      });
    }
  }

  protected sortedSnaps(): SnapshotRecord[] {
    return [...this.state.snaps()].sort((a, b) => (a.sortOrder ?? 0) - (b.sortOrder ?? 0) || a.ts - b.ts);
  }

  protected moveSnap(snapId: string, direction: -1 | 1): void {
    const ordered = this.sortedSnaps();
    const index = ordered.findIndex(snap => snap.id === snapId);
    const nextIndex = index + direction;
    if (index < 0 || nextIndex < 0 || nextIndex >= ordered.length) return;
    const before = this.state.snaps();
    [ordered[index], ordered[nextIndex]] = [ordered[nextIndex], ordered[index]];
    const orderById = new Map(ordered.map((snap, order) => [snap.id, order]));
    this.state.snaps.update(snaps => snaps.map(snap => ({ ...snap, sortOrder: orderById.get(snap.id) ?? snap.sortOrder })));
    this.undoService.push({
      label: 'Reorder snapshot',
      undo: () => this.state.snaps.set(before)
    });
    this.state.triggerHaptic('tap');
  }

  protected toggleCompareSnap(snapId: string): void {
    this.compareSelection.update(ids => {
      if (ids.includes(snapId)) return ids.filter(id => id !== snapId);
      return [...ids.slice(-1), snapId];
    });
    this.state.triggerHaptic('tap');
  }

  protected snapDelta(key: 'reported_hr' | 'reported_rr' | 'distance_cm'): string {
    const snaps = this.selectedCompareSnaps();
    if (snaps.length !== 2) return '--';
    const delta = Number(snaps[1][key]) - Number(snaps[0][key]);
    return Number.isFinite(delta) ? `${delta >= 0 ? '+' : ''}${delta.toFixed(1)}` : '--';
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

  downloadChart(canvas: HTMLCanvasElement | ElementRef<HTMLCanvasElement>, label: string) {
    const target = canvas instanceof ElementRef ? canvas.nativeElement : canvas;
    const anchor = document.createElement('a');
    anchor.href = target.toDataURL('image/png');
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
  protected requestCanvasDraw(): void {
    if (!this.viewReady || document.visibilityState === 'hidden' || this.animeFrameId !== null) return;
    this.animeFrameId = requestAnimationFrame(() => {
      this.animeFrameId = null;
      this.drawWaves();
      this.drawTrends();
      this.drawOverviewSparklines();
      this.drawTargetPosition();
      this.drawBlandAltman();
    });
  }

  private drawWaves() {
    const showWaves = this.activeTabIndex === 1 || (this.splitScreenActive() && this.paneBTabIndex() === 1);
    if (!showWaves) return;

    const payload = this.state.lastPayload();
    if (!payload || !payload.series) return;

    const breathPoints = this.seriesNumbers('breath_phase', 'breath');
    const heartPoints = this.seriesNumbers('heart_phase', 'heart');

    const drawPhase = (canvasRef: ElementRef<HTMLCanvasElement>, points: number[], color: string, chartKey: string) => {
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
        const y = pad + innerH / 2 - (val / 1.5) * (innerH / 2);
        
        if (idx === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();

      // Render annotations
      const anns = this.getAnnotationsFor(chartKey);
      anns.forEach(ann => {
        const x = pad + ann.xPct * innerW;
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(x, pad);
        ctx.lineTo(x, pad + innerH);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = '#ef4444';
        ctx.font = '10px sans-serif';
        ctx.fillText(ann.label || '', x + 4, pad + 10);
      });
    };

    if (this.activeTabIndex === 1) {
      if (this.breathCanvas) {
        drawPhase(this.breathCanvas, breathPoints, 'rgba(97, 105, 198, 0.95)', 'breath');
      }
      if (this.heartCanvas) {
        drawPhase(this.heartCanvas, heartPoints, 'rgba(0, 164, 150, 0.95)', 'heart');
      }
    }

    if (this.splitScreenActive() && this.paneBTabIndex() === 1) {
      if (this.breathCanvasClone) {
        drawPhase(this.breathCanvasClone, breathPoints, 'rgba(97, 105, 198, 0.95)', 'breath');
      }
      if (this.heartCanvasClone) {
        drawPhase(this.heartCanvasClone, heartPoints, 'rgba(0, 164, 150, 0.95)', 'heart');
      }
    }
  }

  private drawTrends() {
    const showHr = this.activeTabIndex === 2 || (this.splitScreenActive() && this.paneBTabIndex() === 2);
    const showRr = this.activeTabIndex === 3 || (this.splitScreenActive() && this.paneBTabIndex() === 3);
    if (!showHr && !showRr) return;

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

      // Draw ghost session overlay if active
      if (this.ghostSessionActive() && type) {
        const ghostData = type === 'hr' ? this.ghostHrData() : this.ghostRrData();
        if (ghostData && ghostData.length > 1) {
          ctx.strokeStyle = 'rgba(148, 163, 184, 0.6)';
          ctx.lineWidth = 1.5;
          ctx.setLineDash([5, 5]);
          ctx.beginPath();
          const ghostCount = ghostData.length;
          ghostData.forEach((val, idx) => {
            const x = pad + (idx / (ghostCount - 1)) * innerW;
            const y = pad + innerH - ((val - minV) / diff) * innerH;
            if (idx === 0) {
              ctx.moveTo(x, y);
            } else {
              ctx.lineTo(x, y);
            }
          });
          ctx.stroke();
          ctx.setLineDash([]);
        }
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

      // Draw annotations
      if (type) {
        const anns = this.getAnnotationsFor(type);
        anns.forEach(ann => {
          const x = pad + ann.xPct * innerW;
          ctx.strokeStyle = '#ef4444';
          ctx.lineWidth = 1;
          ctx.setLineDash([4, 4]);
          ctx.beginPath();
          ctx.moveTo(x, pad);
          ctx.lineTo(x, pad + innerH);
          ctx.stroke();
          ctx.setLineDash([]);

          ctx.fillStyle = '#ef4444';
          ctx.font = '10px sans-serif';
          ctx.fillText(ann.label || '', x + 4, pad + 10);
        });
      }
    };

    const thresholds = this.state.kpiThresholds();
    if (this.activeTabIndex === 2 && this.hrTrendCanvas) {
      plotTrend(this.hrTrendCanvas, this.trimTrend(this.seriesNumbers('reported_hr', 'hr')), 'rgba(0, 164, 150, 0.95)', Math.max(0, thresholds.hrLow - 20), thresholds.hrHigh + 20, 'hr');
    }
    if (this.splitScreenActive() && this.paneBTabIndex() === 2 && this.hrTrendCanvasClone) {
      plotTrend(this.hrTrendCanvasClone, this.trimTrend(this.seriesNumbers('reported_hr', 'hr')), 'rgba(0, 164, 150, 0.95)', Math.max(0, thresholds.hrLow - 20), thresholds.hrHigh + 20, 'hr');
    }

    if (this.activeTabIndex === 3 && this.rrTrendCanvas) {
      plotTrend(this.rrTrendCanvas, this.trimTrend(this.seriesNumbers('reported_rr', 'rr')), 'rgba(97, 105, 198, 0.95)', Math.max(0, thresholds.rrLow - 5), thresholds.rrHigh + 5, 'rr');
    }
    if (this.splitScreenActive() && this.paneBTabIndex() === 3 && this.rrTrendCanvasClone) {
      plotTrend(this.rrTrendCanvasClone, this.trimTrend(this.seriesNumbers('reported_rr', 'rr')), 'rgba(97, 105, 198, 0.95)', Math.max(0, thresholds.rrLow - 5), thresholds.rrHigh + 5, 'rr');
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

  private drawBlandAltman(): void {
    if (this.activeTabIndex !== 0) return;
    if (!this.baCanvas) return;

    const canvas = this.baCanvas.nativeElement;
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

    const pad = 30;
    const innerW = w - pad * 2;
    const innerH = h - pad * 2;

    const pairs = this.baMetric() === 'hr' ? this.baHrPairs : this.baRrPairs;
    if (pairs.length === 0) {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '13px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('Reference oximeter data required for Bland-Altman analysis.', w / 2, h / 2);
      return;
    }

    const means = pairs.map(p => p.mean);
    const diffs = pairs.map(p => p.diff);

    const minX = Math.min(...means) - 5;
    const maxX = Math.max(...means) + 5;
    const rangeX = Math.max(1, maxX - minX);

    const maxAbsDiff = Math.max(...diffs.map(Math.abs), 5);
    const minY = -maxAbsDiff - 2;
    const maxY = maxAbsDiff + 2;
    const rangeY = Math.max(1, maxY - minY);

    const outlineColor = getComputedStyle(document.documentElement).getPropertyValue('--md-sys-color-outline-variant').trim() || '#e2e8f0';
    const textColor = getComputedStyle(document.documentElement).getPropertyValue('--md-sys-color-on-surface-variant').trim() || '#64748b';
    ctx.strokeStyle = outlineColor;
    ctx.lineWidth = 1;

    // Draw horizontal grid lines and Y-axis labels
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = textColor;
    ctx.font = '10px sans-serif';

    const yZero = pad + innerH - ((0 - minY) / rangeY) * innerH;
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.4)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(pad, yZero);
    ctx.lineTo(w - pad, yZero);
    ctx.stroke();

    const stats = this.baStats();
    if (stats) {
      const yMean = pad + innerH - ((stats.meanDiff - minY) / rangeY) * innerH;
      const yHi = pad + innerH - ((stats.hi - minY) / rangeY) * innerH;
      const yLo = pad + innerH - ((stats.lo - minY) / rangeY) * innerH;

      ctx.strokeStyle = 'rgba(239, 68, 68, 0.8)';
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 3]);
      ctx.beginPath();
      ctx.moveTo(pad, yMean);
      ctx.lineTo(w - pad, yMean);
      ctx.stroke();

      ctx.fillStyle = 'rgba(239, 68, 68, 0.9)';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'bottom';
      ctx.fillText(`Mean Bias: ${stats.meanDiff.toFixed(2)}`, pad + 4, yMean - 2);

      ctx.strokeStyle = 'rgba(251, 146, 60, 0.8)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(pad, yHi);
      ctx.lineTo(w - pad, yHi);
      ctx.stroke();
      ctx.fillText(`+1.96 SD: ${stats.hi.toFixed(2)}`, pad + 4, yHi - 2);

      ctx.beginPath();
      ctx.moveTo(pad, yLo);
      ctx.lineTo(w - pad, yLo);
      ctx.stroke();
      ctx.fillText(`-1.96 SD: ${stats.lo.toFixed(2)}`, pad + 4, yLo + 10);

      ctx.setLineDash([]);
    }

    pairs.forEach(p => {
      const cx = pad + ((p.mean - minX) / rangeX) * innerW;
      const cy = pad + innerH - ((p.diff - minY) / rangeY) * innerH;

      let color = '#94a3b8';
      if (Number.isFinite(p.pqi)) {
        if (p.pqi > 0.3) color = '#16a34a';
        else if (p.pqi >= 0.15) color = '#f59e0b';
        else color = '#dc2626';
      }

      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
      ctx.fill();
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
      ctx.lineWidth = 0.5;
      ctx.stroke();
    });
  }
}
