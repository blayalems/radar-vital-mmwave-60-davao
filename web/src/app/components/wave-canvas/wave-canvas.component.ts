import { ChangeDetectionStrategy, Component, ElementRef, ViewChild, AfterViewInit, OnDestroy, effect, input } from '@angular/core';
import { ChartAnnotation } from '../../models/rvt.models';

@Component({
  selector: 'wave-canvas',
  standalone: true,
  template: `
    <canvas #canvas class="waveform-canvas" style="display: block; width: 100%; height: 100%;"
            [attr.aria-label]="ariaLabel()">
    </canvas>
  `,
  styles: [':host { display: block; width: 100%; height: 100%; }'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class WaveCanvasComponent implements AfterViewInit, OnDestroy {
  // Inputs
  data = input<number[]>([]);
  color = input<string>('rgba(97, 105, 198, 0.95)');
  chartKey = input<string>('');
  ariaLabel = input<string>('');
  annotations = input<ChartAnnotation[]>([]);

  @ViewChild('canvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  get nativeCanvas(): HTMLCanvasElement {
    return this.canvasRef.nativeElement;
  }

  private animeFrameId: number | null = null;
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
    if (this.animeFrameId !== null) {
      cancelAnimationFrame(this.animeFrameId);
    }
    this.resizeObserver?.disconnect();
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
