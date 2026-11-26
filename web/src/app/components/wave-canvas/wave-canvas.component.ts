import { ChangeDetectionStrategy, Component, ElementRef, ViewChild, AfterViewInit, OnDestroy, computed, effect, inject, input } from '@angular/core';
import { MatIconModule } from '@angular/material/icon';
import { ChartAnnotation } from '../../models/rvt.models';
import { ChartRenderSchedulerService } from '../../services/chart-render-scheduler.service';

@Component({
  selector: 'wave-canvas',
  standalone: true,
  imports: [MatIconModule],
  template: `
    <div class="wave-canvas-container">
      <canvas #canvas class="waveform-canvas" style="display: block; width: 100%; height: 100%;"
              [attr.aria-label]="accessibleLabel()">
      </canvas>
      @if (!hasSamples()) {
        <div class="wave-empty-state" aria-hidden="true">
          <mat-icon>waves</mat-icon>
          <strong>Waiting for waveform samples</strong>
          <span>Phase waveform data will appear once telemetry samples arrive.</span>
        </div>
      }
    </div>
  `,
  styles: [`
    :host { display: block; width: 100%; height: 100%; }
    .wave-canvas-container { position: relative; width: 100%; height: 100%; }
    .wave-empty-state {
      position: absolute;
      inset: 12px;
      display: grid;
      place-content: center;
      justify-items: center;
      gap: 4px;
      padding: 12px;
      border: 1px dashed var(--md-sys-color-outline-variant, #cbd5e1);
      border-radius: 12px;
      color: var(--md-sys-color-on-surface-variant, #64748b);
      background: color-mix(in srgb, var(--md-sys-color-surface-container-lowest, #ffffff) 88%, transparent);
      text-align: center;
      pointer-events: none;
    }
    .wave-empty-state mat-icon { width: 24px; height: 24px; font-size: 24px; color: var(--md-sys-color-primary, #36618e); }
    .wave-empty-state strong { font-size: 0.88rem; color: var(--md-sys-color-on-surface, #16212e); }
    .wave-empty-state span { font-size: 0.76rem; line-height: 1.35; max-width: 32ch; }
  `],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class WaveCanvasComponent implements AfterViewInit, OnDestroy {
  private readonly renderScheduler = inject(ChartRenderSchedulerService);

  // Inputs
  data = input<number[]>([]);
  color = input<string>('rgba(97, 105, 198, 0.95)');
  chartKey = input<string>('');
  ariaLabel = input<string>('');
  annotations = input<ChartAnnotation[]>([]);
  protected readonly hasSamples = computed(() => this.data().length >= 2);
  protected readonly accessibleLabel = computed(() => {
    const base = this.ariaLabel().trim();
    if (this.hasSamples()) return base;
    const separator = base && !base.endsWith('.') ? '.' : '';
    return `${base}${separator} Waiting for waveform samples.`.trim();
  });

  @ViewChild('canvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  get nativeCanvas(): HTMLCanvasElement {
    return this.canvasRef.nativeElement;
  }

  private resizeObserver: ResizeObserver | null = null;

  constructor() {
    effect(() => {
      // Bind reactivity to inputs
      this.data();
      this.color();
      this.chartKey();
      this.annotations();

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
    this.renderScheduler.cancel(this);
    this.resizeObserver?.disconnect();
  }

  requestDraw(): void {
    this.renderScheduler.request(
      this,
      () => this.draw(),
      () => this.renderScheduler.canvasVisible(this.canvasRef?.nativeElement),
      100
    );
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

    const points = this.data();
    if (points.length < 2) return;

    ctx.strokeStyle = this.color();
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
    const anns = this.annotations();
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
}
