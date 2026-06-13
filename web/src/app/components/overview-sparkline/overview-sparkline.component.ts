import { ChangeDetectionStrategy, Component, ElementRef, ViewChild, AfterViewInit, OnDestroy, effect, input } from '@angular/core';

@Component({
  selector: 'overview-sparkline',
  standalone: true,
  template: `
    <canvas #canvas class="sparkline-canvas" style="display: block; width: 100%; height: 100%;"
            role="img" [attr.aria-label]="ariaLabel()">
    </canvas>
  `,
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class OverviewSparklineComponent implements AfterViewInit, OnDestroy {
  // Inputs
  data = input<number[]>([]);
  color = input<string>('rgba(0, 164, 150, 0.8)');
  ariaLabel = input<string>('');

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

    const minV = Math.min(...points);
    const maxV = Math.max(...points);
    const diff = Math.max(1, maxV - minV);

    ctx.strokeStyle = this.color();
    ctx.lineWidth = 1.5;
    ctx.beginPath();

    const count = points.length;
    points.forEach((val, idx) => {
      const x = (idx / (count - 1)) * w;
      const y = h - 2 - ((val - minV) / diff) * (h - 4);

      if (idx === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();
  }
}
