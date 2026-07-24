import { ChangeDetectionStrategy, Component, input, ViewChild } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatChipsModule } from '@angular/material/chips';
import { MatIconModule } from '@angular/material/icon';
import { ChartDataTableComponent } from '../../chart-data-table/chart-data-table.component';
import { TrendCanvasComponent } from '../../trend-canvas/trend-canvas.component';
import { LiveRrTabViewModel } from '../live-tab-view-models';

@Component({
  selector: 'live-rr-tab',
  imports: [
    ChartDataTableComponent,
    MatButtonModule,
    MatCardModule,
    MatChipsModule,
    MatIconModule,
    TrendCanvasComponent
  ],
  templateUrl: './live-rr-tab.component.html',
  styleUrl: './live-trend-tab.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class LiveRrTabComponent {
  readonly context = input.required<LiveRrTabViewModel>();
  protected readonly Math = Math;
  @ViewChild('trendCanvas') private trendCanvas?: TrendCanvasComponent;

  mapZoomedXPctToFull(xPct: number): number {
    return this.trendCanvas?.mapZoomedXPctToFull(xPct) ?? xPct;
  }

  resetZoom(): void {
    this.trendCanvas?.resetZoom();
  }

  protected downloadTrend(): void {
    const canvas = this.trendCanvas?.nativeCanvas;
    if (canvas) this.context().downloadChart(canvas, 'respiration_trend');
  }
}
