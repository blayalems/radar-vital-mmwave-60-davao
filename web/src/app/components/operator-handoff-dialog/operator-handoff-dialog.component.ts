import { ChangeDetectionStrategy, Component, computed, inject } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';

import { StateService } from '../../services/state.service';

@Component({
  selector: 'app-operator-handoff-dialog',
  imports: [MatButtonModule, MatDialogModule, MatIconModule],
  templateUrl: './operator-handoff-dialog.component.html',
  styleUrl: './operator-handoff-dialog.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class OperatorHandoffDialogComponent {
  protected readonly state = inject(StateService);
  private readonly dialogRef = inject(MatDialogRef<OperatorHandoffDialogComponent>);

  protected readonly activeAlerts = computed(() => this.state.alertHistory().filter(alert => !alert.dismissed).slice(0, 6));
  protected readonly latestNotes = computed(() => {
    const sid = this.state.currentSessionId();
    return sid ? (this.state.sessionNotes()[sid] || '').split('\n').filter(Boolean).slice(-5) : [];
  });
  protected readonly handoffPayload = computed(() => {
    const setup = this.state.setup();
    const payload = this.state.lastPayload();
    return {
      exported_at: new Date().toISOString(),
      session_id: this.state.currentSessionId(),
      subject: setup.subject_label,
      operator: setup.operator_label,
      station: setup.station_label,
      source: this.state.ctlStatus()?.mode || 'unknown',
      vitals: {
        radar_hr: payload?.radar?.reported_hr ?? null,
        radar_rr: payload?.radar?.reported_rr ?? null,
        ble_hr: payload?.ble?.hr ?? null,
        ble_rr: payload?.ble?.rr ?? null,
        distance_cm: payload?.radar?.distance_cm ?? null
      },
      active_alerts: this.activeAlerts().map(alert => ({ severity: alert.severity, message: alert.msg, ts: alert.ts })),
      latest_notes: this.latestNotes()
    };
  });

  async copy(): Promise<void> {
    await navigator.clipboard?.writeText(JSON.stringify(this.handoffPayload(), null, 2));
    this.dialogRef.close('copied');
  }

  exportJson(): void {
    const href = URL.createObjectURL(new Blob([JSON.stringify(this.handoffPayload(), null, 2)], { type: 'application/json' }));
    const anchor = document.createElement('a');
    anchor.href = href;
    anchor.download = `operator-handoff-${this.state.currentSessionId() || 'local'}.json`;
    anchor.click();
    URL.revokeObjectURL(href);
  }

  print(): void {
    window.print();
  }
}
