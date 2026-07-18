import { UpperCasePipe } from '@angular/common';
import {
  ChangeDetectionStrategy,
  Component,
  ElementRef,
  input,
  ViewChild
} from '@angular/core';
import { FormsModule } from '@angular/forms';
import { MatButtonModule } from '@angular/material/button';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatCardModule } from '@angular/material/card';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { OverviewSparklineComponent } from '../../overview-sparkline/overview-sparkline.component';
import { LiveOverviewTabViewModel } from '../live-tab-view-models';

@Component({
  selector: 'live-overview-tab',
  imports: [
    FormsModule,
    MatButtonModule,
    MatButtonToggleModule,
    MatCardModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule,
    OverviewSparklineComponent,
    UpperCasePipe
  ],
  templateUrl: './live-overview-tab.component.html',
  // Reuse the established Live visual system under this component's own
  // emulated scope. A later CSS-only pass can reduce the duplicated rules
  // without coupling the template boundary back to LiveComponent.
  styleUrl: '../live.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class LiveOverviewTabComponent {
  readonly context = input.required<LiveOverviewTabViewModel>();

  @ViewChild('targetCanvas') targetCanvas?: ElementRef<HTMLCanvasElement>;
  @ViewChild('baCanvas') baCanvas?: ElementRef<HTMLCanvasElement>;

  protected readonly Math = Math;
  protected readonly NaN = Number.NaN;

  protected get state() { return this.context().state; }
  protected get kpiOrder() { return this.context().kpiOrder; }
  protected get liveStatusAnnouncement() { return this.context().liveStatusAnnouncement; }
  protected get baMetric() { return this.context().baMetric; }
  protected get baStats() { return this.context().baStats; }
  protected get hrConfidencePct() { return this.context().hrConfidencePct; }
  protected get hrConfidenceSource() { return this.context().hrConfidenceSource; }
  protected get rrConfidencePct() { return this.context().rrConfidencePct; }
  protected get rrConfidenceSource() { return this.context().rrConfidenceSource; }
  protected get hrHolding() { return this.context().hrHolding; }
  protected get rrHolding() { return this.context().rrHolding; }
  protected get computedFps() { return this.context().computedFps; }
  protected get motionActive() { return this.context().motionActive; }

  protected get sessionNotesInput(): string {
    return this.context().sessionNotes();
  }

  protected get customTagInput(): string {
    return this.context().customTag();
  }

  protected set customTagInput(value: string) {
    this.context().setCustomTag(value);
  }

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
  protected targetRangeDisplay(): string { return this.context().targetRangeDisplay(); }
  protected targetZoneLabel(): string { return this.context().targetZoneLabel(); }
  protected targetZoneTone() { return this.context().targetZoneTone(); }
  protected signalQualityDisplay(): string { return this.context().signalQualityDisplay(); }
  protected signalQualitySub(): string { return this.context().signalQualitySub(); }
  protected signalQualityTone() { return this.context().signalQualityTone(); }
  protected motionSummary(): string { return this.context().motionSummary(); }
  protected motionSub(): string { return this.context().motionSub(); }
  protected exportSessionNotes(): void { this.context().exportSessionNotes(); }
  protected saveSessionNotes(value: string): void { this.context().saveSessionNotes(value); }
  protected addQuickTag(tag: string): void { this.context().addQuickTag(tag); }
  protected recordObservation(): Promise<void> { return this.context().recordObservation(); }
  protected addCustomTag(): void { this.context().addCustomTag(); }
  protected captureSnapshot(): void { this.context().captureSnapshot(); }
  protected metricState(key: string) { return this.context().metricState(key); }
  protected metricLabel(key: string): string { return this.context().metricLabel(key); }

  protected metricText(key: string, decimals?: number, suffix?: string): string {
    return this.context().metricText(key, decimals, suffix);
  }

  protected hasBleRef(): boolean { return this.context().hasBleRef(); }
  protected baChartAriaLabel(): string { return this.context().baChartAriaLabel(); }
  protected requestCanvasDraw(): void { this.context().requestCanvasDraw(); }

  protected formatFault(fault: string | Record<string, unknown>): string {
    return this.context().formatFault(fault);
  }
}
