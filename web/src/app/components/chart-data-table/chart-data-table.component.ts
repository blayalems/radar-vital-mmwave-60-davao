import { ChangeDetectionStrategy, Component, computed, input } from '@angular/core';

import { TelemetrySeries } from '../../models/rvt.models';

interface ChartRow {
  index: number;
  timestamp: string;
  hr: string;
  rr: string;
  distance: string;
}

@Component({
  selector: 'rvt-chart-data-table',
  imports: [],
  templateUrl: './chart-data-table.component.html',
  styleUrl: './chart-data-table.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class ChartDataTableComponent {
  readonly series = input<TelemetrySeries | null | undefined>(null);
  readonly limit = input(120);
  readonly label = input('Recent chart samples');

  readonly rows = computed<ChartRow[]>(() => {
    const series = this.series();
    const timestamps = this.numbers(series, 'ts', 't');
    const hr = this.numbers(series, 'reported_hr', 'hr', 'raw_hr');
    const rr = this.numbers(series, 'reported_rr', 'rr');
    const distance = this.numbers(series, 'distance_cm', 'target_distance_cm');
    const count = Math.max(timestamps.length, hr.length, rr.length, distance.length);
    const start = Math.max(0, count - this.limit());
    const rows: ChartRow[] = [];
    for (let index = start; index < count; index += 1) {
      rows.push({
        index,
        timestamp: this.formatTime(timestamps[index]),
        hr: this.format(hr[index], 'bpm'),
        rr: this.format(rr[index], 'br/min'),
        distance: this.format(distance[index], 'cm')
      });
    }
    return rows.reverse();
  });

  private numbers(series: TelemetrySeries | null | undefined, ...keys: string[]): number[] {
    for (const key of keys) {
      const value = series?.[key];
      if (Array.isArray(value)) return value.map(item => Number(item)).filter(Number.isFinite);
    }
    return [];
  }

  private format(value: number | undefined, unit: string): string {
    return Number.isFinite(value) ? `${Number(value).toFixed(1)} ${unit}` : '--';
  }

  private formatTime(value: number | undefined): string {
    if (!Number.isFinite(value)) return '--';
    const n = Number(value);
    if (n > 1_000_000_000_000) return new Date(n).toLocaleTimeString();
    return `${n.toFixed(1)}s`;
  }
}
