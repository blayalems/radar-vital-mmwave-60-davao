import { ChangeDetectionStrategy, Component, input, ViewChild } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatChipsModule } from '@angular/material/chips';
import { MatIconModule } from '@angular/material/icon';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { ChartDataTableComponent } from '../../chart-data-table/chart-data-table.component';
import { WaveCanvasComponent } from '../../wave-canvas/wave-canvas.component';
import { LiveWavesTabViewModel } from '../live-tab-view-models';

@Component({
  selector: 'live-waves-tab',
  imports: [
    ChartDataTableComponent,
    MatButtonModule,
    MatCardModule,
    MatChipsModule,
    MatIconModule,
    MatProgressBarModule,
    WaveCanvasComponent
  ],
  templateUrl: './live-waves-tab.component.html',
  styleUrl: './live-waves-tab.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class LiveWavesTabComponent {
  readonly context = input.required<LiveWavesTabViewModel>();
  @ViewChild('breathCanvas') private breathCanvas?: WaveCanvasComponent;
  @ViewChild('heartCanvas') private heartCanvas?: WaveCanvasComponent;

  protected downloadBreath(): void {
    const canvas = this.breathCanvas?.nativeCanvas;
    if (canvas) this.context().downloadChart(canvas, 'breathing_waveform');
  }

  protected downloadHeart(): void {
    const canvas = this.heartCanvas?.nativeCanvas;
    if (canvas) this.context().downloadChart(canvas, 'heartbeat_waveform');
  }
}
