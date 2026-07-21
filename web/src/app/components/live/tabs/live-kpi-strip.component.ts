import { ChangeDetectionStrategy, Component, input } from '@angular/core';
import { MatIconModule } from '@angular/material/icon';
import { OverviewSparklineComponent } from '../../overview-sparkline/overview-sparkline.component';
import { LiveKpiStripViewModel } from '../live-tab-view-models';

@Component({
  selector: 'live-kpi-strip',
  imports: [MatIconModule, OverviewSparklineComponent],
  templateUrl: './live-kpi-strip.component.html',
  // Keep the extracted controls on the established Live visual contract while
  // giving their styles an independent emulated-encapsulation scope.
  styleUrl: '../live.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class LiveKpiStripComponent {
  readonly context = input.required<LiveKpiStripViewModel>();

  protected readonly Math = Math;
  protected readonly NaN = Number.NaN;

  protected get state() { return this.context().state; }
  protected get kpiOrder() { return this.context().kpiOrder; }
  protected get liveStatusAnnouncement() { return this.context().liveStatusAnnouncement; }
  protected get hrConfidencePct() { return this.context().hrConfidencePct; }
  protected get hrConfidenceSource() { return this.context().hrConfidenceSource; }
  protected get rrConfidencePct() { return this.context().rrConfidencePct; }
  protected get rrConfidenceSource() { return this.context().rrConfidenceSource; }
  protected get hrHolding() { return this.context().hrHolding; }
  protected get rrHolding() { return this.context().rrHolding; }
  protected get computedFps() { return this.context().computedFps; }

  protected openKpiZoomDialog(metric: 'hr' | 'rr' | 'fps' | 'dist'): void {
    this.context().openKpiZoomDialog(metric);
  }

  protected onKpiKeydown(event: KeyboardEvent, kpi: string): void {
    this.context().onKpiKeydown(event, kpi);
  }

  protected onKpiDragStart(event: PointerEvent, kpi: string): void {
    this.context().onKpiDragStart(event, kpi);
  }

  protected onKpiDragMove(event: PointerEvent): void {
    this.context().onKpiDragMove(event);
  }

  protected onKpiDragEnd(event: PointerEvent): void {
    this.context().onKpiDragEnd(event);
  }

  protected kpiControlLabel(label: string, key: string, unit: string): string {
    return this.context().kpiControlLabel(label, key, unit);
  }

  protected kpiReorderDescription(kpi: string): string {
    return this.context().kpiReorderDescription(kpi);
  }

  protected kpiButtonTitle(kpi: string): string {
    return this.context().kpiButtonTitle(kpi);
  }

  protected hrValueDisplay(): string { return this.context().hrValueDisplay(); }
  protected rrValueDisplay(): string { return this.context().rrValueDisplay(); }

  protected chartLabel(label: string, key: string, unit: string): string {
    return this.context().chartLabel(label, key, unit);
  }

  protected frameRateControlLabel(): string {
    return this.context().frameRateControlLabel();
  }

  protected frameRateLabel(): string { return this.context().frameRateLabel(); }
  protected targetRangeControlLabel(): string { return this.context().targetRangeControlLabel(); }
}
