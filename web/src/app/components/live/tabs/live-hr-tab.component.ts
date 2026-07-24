import { ChangeDetectionStrategy, Component, input, ViewChild } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatCardModule } from '@angular/material/card';
import { MatChipsModule } from '@angular/material/chips';
import { MatIconModule } from '@angular/material/icon';
import { ChartDataTableComponent } from '../../chart-data-table/chart-data-table.component';
import { TrendCanvasComponent } from '../../trend-canvas/trend-canvas.component';
import { LiveHrTabViewModel } from '../live-tab-view-models';

@Component({
  selector: 'live-hr-tab',
  imports: [
    ChartDataTableComponent,
    MatButtonModule,
    MatButtonToggleModule,
    MatCardModule,
    MatChipsModule,
    MatIconModule,
    TrendCanvasComponent
  ],
  templateUrl: './live-hr-tab.component.html',
  styleUrl: './live-trend-tab.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class LiveHrTabComponent {
  readonly context = input.required<LiveHrTabViewModel>();
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
    if (canvas) this.context().downloadChart(canvas, 'heart_rate_trend');
  }
}
