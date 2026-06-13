import { ChangeDetectionStrategy, Component, ElementRef, ViewChild, AfterViewInit, OnDestroy, HostListener, signal, computed, effect, inject, input } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { StateService } from '../../services/state.service';
import { ChartAnnotation } from '../../models/rvt.models';

@Component({
  selector: 'trend-canvas',
  standalone: true,
  imports: [MatButtonModule, MatIconModule],
  template: `
    <div class="trend-canvas-container" style="position: relative; width: 100%; height: 100%;">
      <canvas #canvas class="large-trend-canvas" style="display: block; width: 100%; height: 100%;"
              [attr.aria-label]="ariaLabel()"
              (wheel)="onWheel($event)"
              (touchstart)="onTouchStart($event)"
              (touchmove)="onTouchMove($event)"
              (touchend)="onTouchEnd($event)">
      </canvas>
      @if (isZoomed()) {
        <div class="zoom-indicator-overlay" style="position: absolute; bottom: 8px; left: 8px; background: rgba(0,0,0,0.7); color: white; padding: 4px 8px; border-radius: 4px; font-size: 11px; pointer-events: none; z-index: 10; font-family: monospace;">
          Zoomed (showing {{ visiblePointsCount() }} / {{ totalPointsCount() }} samples)
        </div>
        <button mat-icon-button class="reset-zoom-btn" style="position: absolute; top: 8px; right: 8px; background: rgba(0,0,0,0.7); color: white; z-index: 10; width: 36px; height: 36px; line-height: 36px;" (click)="$event.stopPropagation(); resetZoom()" title="Reset zoom">
          <mat-icon style="font-size: 20px; width: 20px; height: 20px;">zoom_out_map</mat-icon>
        </button>
      }
    </div>
  `,
  styles: [':host { display: block; width: 100%; height: 100%; }'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class TrendCanvasComponent implements AfterViewInit, OnDestroy {
  protected readonly state = inject(StateService);

  // Inputs
  data = input<number[]>([]);
  color = input<string>('rgba(0,164,150,0.95)');
  minV = input<number>(0);
  maxV = input<number>(100);
  type = input<'hr' | 'rr'>();
  ariaLabel = input<string>('');
  ghostData = input<number[]>([]);
  ghostSessionLabel = input<string | null>(null);
  ghostSessionActive = input<boolean>(false);
  annotations = input<ChartAnnotation[]>([]);

  @ViewChild('canvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  get nativeCanvas(): HTMLCanvasElement {
    return this.canvasRef.nativeElement;
  }

  // Zoom range fractions
  zoomStartFraction = signal<number>(0);
  zoomEndFraction = signal<number>(1);

  isZoomed = computed(() => this.zoomStartFraction() > 0 || this.zoomEndFraction() < 1);
  totalPointsCount = computed(() => this.data().length);
  visiblePointsCount = computed(() => {
    const N = this.data().length;
    if (N < 10) return N;
    let startIdx = Math.round(this.zoomStartFraction() * (N - 1));
    let endIdx = Math.round(this.zoomEndFraction() * (N - 1));
    if (endIdx - startIdx + 1 < 10) {
      const diff = 10 - (endIdx - startIdx + 1);
      startIdx = Math.max(0, startIdx - Math.floor(diff / 2));
      endIdx = Math.min(N - 1, startIdx + 9);
      startIdx = Math.max(0, endIdx - 9);
    }
    return endIdx - startIdx + 1;
  });

  private animeFrameId: number | null = null;
  private resizeObserver: ResizeObserver | null = null;

  // Touch gesture state
  private lastTouchDistance = 0;
  private lastTouchCenterPct = 0.5;

  constructor() {
    effect(() => {
      // Bind reactivity to signals
      this.data();
      this.color();
      this.minV();
      this.maxV();
      this.type();
      this.annotations();
      this.ghostData();
      this.ghostSessionLabel();
      this.ghostSessionActive();
      this.zoomStartFraction();
      this.zoomEndFraction();
      this.state.theme();

      this.requestDraw();
    });
  }

  ngAfterViewInit() {
    if (typeof ResizeObserver !== 'undefined') {
      this.resizeObserver = new ResizeObserver(() => this.requestDraw());
      this.resizeObserver.observe(this.canvasRef.nativeElement);
    }
    this.requestDraw();
  }

  ngOnDestroy() {
    if (this.animeFrameId !== null) {
      cancelAnimationFrame(this.animeFrameId);
    }
    this.resizeObserver?.disconnect();
  }

  @HostListener('document:keydown', ['$event'])
  onKeydown(event: KeyboardEvent): void {
    const target = event.target instanceof Element ? event.target : null;
    if (target?.closest('input, textarea, select, [contenteditable], [role="textbox"]')) return;

    if (event.key.toLowerCase() === 'z') {
      this.resetZoom();
    }
  }

  resetZoom(): void {
    this.zoomStartFraction.set(0);
    this.zoomEndFraction.set(1);
    this.requestDraw();
  }

  onWheel(event: WheelEvent): void {
    const N = this.data().length;
    if (N < 10) return;

    event.preventDefault();
    const rect = this.canvasRef.nativeElement.getBoundingClientRect();
    const clickX = event.clientX - rect.left;
    const w = rect.width;
    const pad = 16;
    const innerW = w - pad * 2;
    if (innerW <= 0) return;

    const xPct = Math.max(0, Math.min(1, (clickX - pad) / innerW));
    const zoomFactor = event.deltaY < 0 ? 0.9 : 1.1;
    this.zoom(zoomFactor, xPct);
  }

  onTouchStart(event: TouchEvent): void {
    if (event.touches.length === 2) {
      event.preventDefault();
      this.lastTouchDistance = this.getTouchDistance(event.touches);
      this.lastTouchCenterPct = this.getTouchCenterPct(event.touches);
    }
  }

  onTouchMove(event: TouchEvent): void {
    if (event.touches.length === 2) {
      event.preventDefault();
      const dist = this.getTouchDistance(event.touches);
      const centerPct = this.getTouchCenterPct(event.touches);

      if (this.lastTouchDistance > 0 && Math.abs(dist - this.lastTouchDistance) > 2) {
        const zoomFactor = this.lastTouchDistance / dist;
        this.zoom(zoomFactor, centerPct);
        this.lastTouchDistance = dist;
        this.lastTouchCenterPct = centerPct;
      }
    }
  }

  onTouchEnd(event: TouchEvent): void {
    if (event.touches.length < 2) {
      this.lastTouchDistance = 0;
    }
  }

  private getTouchDistance(touches: TouchList): number {
    const dx = touches[0].clientX - touches[1].clientX;
    const dy = touches[0].clientY - touches[1].clientY;
    return Math.sqrt(dx * dx + dy * dy);
  }

  private getTouchCenterPct(touches: TouchList): number {
    const rect = this.canvasRef.nativeElement.getBoundingClientRect();
    const pad = 16;
    const innerW = rect.width - pad * 2;
    if (innerW <= 0) return 0.5;

    const midX = (touches[0].clientX + touches[1].clientX) / 2;
    const relativeX = midX - rect.left;
    return Math.max(0, Math.min(1, (relativeX - pad) / innerW));
  }

  zoom(factor: number, centerPct: number): void {
    const N = this.data().length;
    if (N < 10) return;

    const currentStart = this.zoomStartFraction();
    const currentEnd = this.zoomEndFraction();
    const currentSpan = currentEnd - currentStart;

    let newSpan = currentSpan * factor;
    const minSpan = 9 / (N - 1);
    if (newSpan < minSpan) {
      newSpan = minSpan;
    }
    if (newSpan > 1.0) {
      newSpan = 1.0;
    }

    const centerVal = currentStart + centerPct * currentSpan;
    let newStart = centerVal - centerPct * newSpan;
    let newEnd = newStart + newSpan;

    if (newStart < 0) {
      newStart = 0;
      newEnd = newSpan;
    }
    if (newEnd > 1.0) {
      newEnd = 1.0;
      newStart = 1.0 - newSpan;
    }

    if (newSpan >= 0.99) {
      this.resetZoom();
    } else {
      this.zoomStartFraction.set(newStart);
      this.zoomEndFraction.set(newEnd);
    }
    this.requestDraw();
  }

  mapZoomedXPctToFull(zoomedXPct: number): number {
    if (!this.isZoomed()) return zoomedXPct;
    const N = this.data().length;
    if (N < 10) return zoomedXPct;
    let startIdx = Math.round(this.zoomStartFraction() * (N - 1));
    let endIdx = Math.round(this.zoomEndFraction() * (N - 1));
    if (endIdx - startIdx + 1 < 10) {
      const diff = 10 - (endIdx - startIdx + 1);
      startIdx = Math.max(0, startIdx - Math.floor(diff / 2));
      endIdx = Math.min(N - 1, startIdx + 9);
      startIdx = Math.max(0, endIdx - 9);
    }
    const clickIdx = startIdx + zoomedXPct * (endIdx - startIdx);
    return clickIdx / (N - 1);
  }

  requestDraw(): void {
    if (document.visibilityState === 'hidden' || this.animeFrameId !== null) return;
    this.animeFrameId = requestAnimationFrame(() => {
      this.animeFrameId = null;
      this.draw();
    });
  }

  private draw() {
    const canvas = this.canvasRef.nativeElement;
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
    const rawData = this.data();
    const N = rawData.length;

    let startIdx = 0;
    let endIdx = Math.max(0, N - 1);
    if (N >= 10) {
      startIdx = Math.round(this.zoomStartFraction() * (N - 1));
      endIdx = Math.round(this.zoomEndFraction() * (N - 1));
      if (endIdx - startIdx + 1 < 10) {
        const diff = 10 - (endIdx - startIdx + 1);
        startIdx = Math.max(0, startIdx - Math.floor(diff / 2));
        endIdx = Math.min(N - 1, startIdx + 9);
        startIdx = Math.max(0, endIdx - 9);
      }
    }

    const visibleData = rawData.slice(startIdx, endIdx + 1);
    const count = visibleData.length;
    const minV = this.minV();
    const maxV = this.maxV();
    const diff = Math.max(1, maxV - minV);
    const type = this.type();

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
    const outlineColor = this.canvasToken(canvas, '--md-sys-color-outline-variant', '#e2e8f0');
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
      const ghostData = this.ghostData();
      const ghostCountFull = ghostData.length;
      if (ghostData && ghostCountFull > 1) {
        let gStart = 0;
        let gEnd = ghostCountFull - 1;
        if (ghostCountFull >= 10) {
          gStart = Math.round(this.zoomStartFraction() * (ghostCountFull - 1));
          gEnd = Math.round(this.zoomEndFraction() * (ghostCountFull - 1));
          if (gEnd - gStart + 1 < 10) {
            const diff = 10 - (gEnd - gStart + 1);
            gStart = Math.max(0, gStart - Math.floor(diff / 2));
            gEnd = Math.min(ghostCountFull - 1, gStart + 9);
            gStart = Math.max(0, gEnd - 9);
          }
        }
        const visibleGhost = ghostData.slice(gStart, gEnd + 1);
        const ghostCount = visibleGhost.length;

        ctx.strokeStyle = 'rgba(148, 163, 184, 0.6)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        visibleGhost.forEach((val, idx) => {
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

    if (count < 2) return;

    ctx.strokeStyle = this.color();
    ctx.lineWidth = 3;
    ctx.beginPath();

    visibleData.forEach((val, idx) => {
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
    const anns = this.annotations();
    anns.forEach(ann => {
      const annIdx = ann.xPct * (N - 1);
      if (annIdx >= startIdx && annIdx <= endIdx) {
        const denom = (endIdx === startIdx) ? 1 : (endIdx - startIdx);
        const relativePct = (annIdx - startIdx) / denom;
        const x = pad + relativePct * innerW;

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
      }
    });
  }

  private canvasToken(canvas: HTMLCanvasElement, token: string, fallback: string): string {
    if (typeof getComputedStyle === 'undefined') return fallback;
    return getComputedStyle(canvas).getPropertyValue(token).trim() || fallback;
  }
}
