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
import { LiveOverviewTabViewModel } from '../live-tab-view-models';
import { LiveKpiStripComponent } from './live-kpi-strip.component';

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
    LiveKpiStripComponent,
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

  protected get state() { return this.context().state; }
  protected get baMetric() { return this.context().baMetric; }
  protected get baStats() { return this.context().baStats; }
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
