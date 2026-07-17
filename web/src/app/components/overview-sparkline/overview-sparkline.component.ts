import { ChangeDetectionStrategy, Component, ElementRef, ViewChild, AfterViewInit, OnDestroy, effect, inject, input } from '@angular/core';
import { ChartRenderSchedulerService } from '../../services/chart-render-scheduler.service';

@Component({
  selector: 'overview-sparkline',
  standalone: true,
  template: `
    <canvas #canvas class="sparkline-canvas" style="display: block; width: 100%; height: 100%;"
            role="img" [attr.aria-label]="ariaLabel()">
    </canvas>
  `,
  styles: [':host { display: block; width: 100%; height: 100%; }'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class OverviewSparklineComponent implements AfterViewInit, OnDestroy {
  private readonly renderScheduler = inject(ChartRenderSchedulerService);

  // Inputs
  data = input<number[]>([]);
  color = input<string>('rgba(0, 164, 150, 0.8)');
  ariaLabel = input<string>('');

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

    // v17 drawSpark: last-60 window, 2px rounded stroke in the vital accent,
    // then the same path closed to the baseline with a 12% area fill.
    const points = this.data().slice(-60);
    if (points.length < 2) return;

    let minV = Math.min(...points);
    let maxV = Math.max(...points);
    if (maxV - minV < 1) {
      maxV += 1;
      minV -= 1;
    }

    const color = this.resolveColor(this.color());
    const count = points.length;
    ctx.beginPath();
    points.forEach((val, idx) => {
      const x = (idx / (count - 1)) * w;
      const y = h - 4 - ((val - minV) / (maxV - minV)) * (h - 8);
      if (idx === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.stroke();
    ctx.lineTo(w, h);
    ctx.lineTo(0, h);
    ctx.closePath();
    ctx.globalAlpha = 0.12;
    ctx.fillStyle = color;
    ctx.fill();
    ctx.globalAlpha = 1;
  }

  /** Canvas cannot consume `var(--…)` directly — resolve it against :root. */
  private resolveColor(raw: string): string {
    const match = /^var\((--[\w-]+)(?:,\s*(.+))?\)$/.exec(raw.trim());
    if (!match) return raw;
    const resolved = getComputedStyle(document.documentElement).getPropertyValue(match[1]).trim();
    return resolved || match[2] || '#36618e';
  }
}
