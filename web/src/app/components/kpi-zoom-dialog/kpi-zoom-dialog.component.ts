import { ChangeDetectionStrategy, Component, inject, ElementRef, ViewChild, AfterViewInit, OnDestroy, effect } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { StateService } from '../../services/state.service';

export interface KpiZoomDialogData {
  metric: 'hr' | 'rr' | 'fps' | 'dist';
  title: string;
  unit: string;
  color: string;
}

@Component({
  selector: 'app-kpi-zoom-dialog',
  standalone: true,
  imports: [MatButtonModule, MatDialogModule, MatIconModule],
  template: `
    <h2 mat-dialog-title class="dialog-title-row">
      <mat-icon [style.color]="data.color" class="dialog-title-icon">{{ getIcon() }}</mat-icon>
      <span>{{ data.title }} Detailed Trend</span>
    </h2>
    
    <mat-dialog-content class="zoom-dialog-content">
      <div class="zoom-metric-hero">
        <div class="hero-value-group">
          <span class="hero-value">{{ getCurrentValue() }}</span>
          <span class="hero-unit">{{ data.unit }}</span>
        </div>
        @if (data.metric === 'hr' || data.metric === 'rr') {
          <div class="hero-meta-group">
            <span class="meta-label">Oximeter Reference:</span>
            <span class="meta-value">{{ getReferenceValue() }}</span>
          </div>
        } @else {
          <div class="hero-meta-group">
            <span class="meta-label">System Source:</span>
            <span class="meta-value">{{ data.metric === 'fps' ? 'DSP Processing Loop' : 'Spatial Radar Sensor' }}</span>
          </div>
        }
      </div>

      <div class="zoom-canvas-container">
        <canvas #zoomCanvas class="zoom-chart-canvas" role="img" [attr.aria-label]="data.title + ' detailed real-time trend chart'"></canvas>
      </div>

      @if (hasThresholds()) {
        <div class="thresholds-info-box">
          <mat-icon class="info-icon">tune</mat-icon>
          <div class="info-text">
            <strong>Active Safety Threshold Limits:</strong>
            <span class="threshold-vals">
              Low: {{ getThresholdLow() }} {{ data.unit }} | High: {{ getThresholdHigh() }} {{ data.unit }}
            </span>
          </div>
        </div>
      }
    </mat-dialog-content>
    
    <mat-dialog-actions align="end">
      <button mat-flat-button type="button" (click)="dialogRef.close()" class="mat-tonal-button">Dismiss</button>
    </mat-dialog-actions>
  `,
  styles: [`
    .dialog-title-row {
      display: flex;
      align-items: center;
      gap: 12px;
      margin: 0;
      padding: 16px 24px;
      font-size: 20px;
      font-weight: 500;
    }
    .dialog-title-icon {
      font-size: 28px;
      width: 28px;
      height: 28px;
    }
    .zoom-dialog-content {
      display: flex;
      flex-direction: column;
      gap: 20px;
      min-width: 320px;
      max-width: 580px;
      width: calc(90vw - 32px);
      padding: 8px 24px 20px 24px;
    }
    .zoom-metric-hero {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      padding: 16px;
      background: var(--md-sys-color-surface-container-high, #f1f5f9);
      border-radius: 12px;
    }
    .hero-value-group {
      display: flex;
      align-items: baseline;
      gap: 4px;
    }
    .hero-value {
      font-size: 36px;
      font-weight: 700;
      color: var(--md-sys-color-on-surface, #0f172a);
    }
    .hero-unit {
      font-size: 14px;
      color: var(--md-sys-color-on-surface-variant, #64748b);
      font-weight: 500;
    }
    .hero-meta-group {
      display: flex;
      flex-direction: column;
      align-items: flex-end;
      font-size: 12px;
    }
    .meta-label {
      color: var(--md-sys-color-on-surface-variant, #64748b);
    }
    .meta-value {
      font-weight: 600;
      color: var(--md-sys-color-primary, #00a496);
    }
    .zoom-canvas-container {
      width: 100%;
      height: 180px;
      background: var(--md-sys-color-surface-container, #ffffff);
      border: 1px solid var(--md-sys-color-outline-variant, #cbd5e1);
      border-radius: 12px;
      overflow: hidden;
      box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.02);
    }
    .zoom-chart-canvas {
      width: 100%;
      height: 100%;
      display: block;
    }
    .thresholds-info-box {
      display: flex;
      align-items: center;
      gap: 12px;
      padding: 12px 16px;
      background: var(--md-sys-color-secondary-container, #e2e8f0);
      color: var(--md-sys-color-on-secondary-container, #334155);
      border-radius: 8px;
      font-size: 13px;
    }
    .info-icon {
      color: var(--md-sys-color-secondary, #6169c6);
    }
    .info-text {
      display: flex;
      flex-direction: column;
      gap: 2px;
    }
    .threshold-vals {
      opacity: 0.9;
    }
  `],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class KpiZoomDialogComponent implements AfterViewInit, OnDestroy {
  readonly data = inject<KpiZoomDialogData>(MAT_DIALOG_DATA);
  readonly dialogRef = inject(MatDialogRef<KpiZoomDialogComponent>);
  private readonly state = inject(StateService);

  @ViewChild('zoomCanvas', { static: false }) zoomCanvas!: ElementRef<HTMLCanvasElement>;
  private animeFrameId: number | null = null;
  private resizeObserver: ResizeObserver | null = null;
  private viewReady = false;

  constructor() {
    effect(() => {
      // Re-trigger draw on new payload, theme change, or threshold updates
      this.state.lastPayload();
      this.state.spark();
      this.state.theme();
      this.state.kpiThresholds();
      this.requestDraw();
    });
  }

  ngAfterViewInit() {
    this.viewReady = true;
    if (typeof ResizeObserver !== 'undefined' && this.zoomCanvas) {
      this.resizeObserver = new ResizeObserver(() => this.requestDraw());
      this.resizeObserver.observe(this.zoomCanvas.nativeElement);
    }
    this.requestDraw();
  }

  ngOnDestroy() {
    if (this.animeFrameId !== null) {
      cancelAnimationFrame(this.animeFrameId);
    }
    this.resizeObserver?.disconnect();
  }

  getIcon(): string {
    switch (this.data.metric) {
      case 'hr': return 'ecg_heart';
      case 'rr': return 'air';
      case 'fps': return 'speed';
      case 'dist': return 'radar';
    }
  }

  getCurrentValue(): string {
    const payload = this.state.lastPayload() as any;
    if (!payload) return '--';
    switch (this.data.metric) {
      case 'hr': return payload.radar?.reported_hr ? Math.round(payload.radar.reported_hr).toString() : '--';
      case 'rr': return payload.radar?.reported_rr ? Math.round(payload.radar.reported_rr).toString() : '--';
      case 'fps': {
        const direct = payload.radar?.fps_hz ?? payload.radar?.fps;
        if (direct !== undefined && direct !== null) return Number(direct).toFixed(1);
        const series = payload?.series as Record<string, unknown> | undefined;
        const timestamps = (series?.['ts'] ?? series?.['t']) as number[] | undefined;
        if (timestamps && Array.isArray(timestamps) && timestamps.length >= 3) {
          const numTimestamps = timestamps.map(t => Number(t)).filter(Number.isFinite);
          const deltas = numTimestamps.slice(1).map((val, idx) => val - numTimestamps[idx]).filter(val => val > 0);
          if (deltas.length > 0) {
            deltas.sort((a, b) => a - b);
            const median = deltas[Math.floor(deltas.length / 2)];
            if (median > 0) return (1 / median).toFixed(1);
          }
        }
        return '--';
      }
      case 'dist': return payload.radar?.distance_cm ? Math.round(payload.radar.distance_cm).toString() : '--';
    }
  }

  getReferenceValue(): string {
    const payload = this.state.lastPayload() as any;
    if (!payload) return 'no reference';
    switch (this.data.metric) {
      case 'hr': return payload.ble?.hr ? `${Math.round(payload.ble.hr)} bpm` : 'no reference';
      case 'rr': return payload.ble?.rr ? `${Math.round(payload.ble.rr)} br/m` : 'no reference';
      default: return 'no reference';
    }
  }

  hasThresholds(): boolean {
    return this.data.metric === 'hr' || this.data.metric === 'rr';
  }

  getThresholdLow(): number {
    const t = this.state.kpiThresholds();
    return this.data.metric === 'hr' ? t.hrLow : t.rrLow;
  }

  getThresholdHigh(): number {
    const t = this.state.kpiThresholds();
    return this.data.metric === 'hr' ? t.hrHigh : t.rrHigh;
  }

  private requestDraw() {
    if (!this.viewReady || this.animeFrameId !== null) return;
    this.animeFrameId = requestAnimationFrame(() => {
      this.animeFrameId = null;
      this.drawChart();
    });
  }

  private drawChart() {
    if (!this.zoomCanvas) return;
    const canvas = this.zoomCanvas.nativeElement;
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

    const spark = this.state.spark();
    let data: number[] = [];
    switch (this.data.metric) {
      case 'hr': data = spark.hr; break;
      case 'rr': data = spark.rr; break;
      case 'fps': data = spark.fps; break;
      case 'dist': data = spark.dist; break;
    }

    if (data.length < 2) return;

    const pad = 20;
    const innerW = w - pad * 2;
    const innerH = h - pad * 2;

    const thresholds = this.state.kpiThresholds();
    let minV = Math.min(...data);
    let maxV = Math.max(...data);

    if (this.data.metric === 'hr') {
      minV = Math.min(minV, thresholds.hrLow - 10);
      maxV = Math.max(maxV, thresholds.hrHigh + 10);
    } else if (this.data.metric === 'rr') {
      minV = Math.min(minV, thresholds.rrLow - 3);
      maxV = Math.max(maxV, thresholds.rrHigh + 3);
    }

    const diff = Math.max(1, maxV - minV);

    // Draw Grid Lines
    const outlineColor = getComputedStyle(document.documentElement).getPropertyValue('--md-sys-color-outline-variant').trim() || '#e2e8f0';
    ctx.strokeStyle = outlineColor;
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = pad + (innerH * i / 4);
      ctx.beginPath();
      ctx.moveTo(pad, y);
      ctx.lineTo(w - pad, y);
      ctx.stroke();

      // Draw horizontal value labels
      const gridVal = maxV - (diff * i / 4);
      ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--md-sys-color-on-surface-variant').trim() || '#64748b';
      ctx.font = '10px Roboto, sans-serif';
      ctx.fillText(Math.round(gridVal).toString(), 4, y + 3);
    }

    // Draw safety thresholds as dotted red lines
    if (this.hasThresholds()) {
      const low = this.getThresholdLow();
      const high = this.getThresholdHigh();
      ctx.strokeStyle = 'rgba(239, 68, 68, 0.45)'; // warn red
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);

      const yLow = pad + innerH - ((low - minV) / diff) * innerH;
      ctx.beginPath();
      ctx.moveTo(pad, yLow);
      ctx.lineTo(w - pad, yLow);
      ctx.stroke();

      const yHigh = pad + innerH - ((high - minV) / diff) * innerH;
      ctx.beginPath();
      ctx.moveTo(pad, yHigh);
      ctx.lineTo(w - pad, yHigh);
      ctx.stroke();

      ctx.setLineDash([]); // reset line dash
    }

    // Draw Main Trend Curve
    ctx.strokeStyle = this.data.color;
    ctx.lineWidth = 3.5;
    ctx.beginPath();

    const count = data.length;
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
  }
}
