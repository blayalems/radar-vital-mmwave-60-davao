import { DatePipe } from '@angular/common';
import { ChangeDetectionStrategy, Component, computed, inject, signal } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatDialogModule } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';
import { MatListModule } from '@angular/material/list';
import { MatMenuModule } from '@angular/material/menu';
import { MatDialogRef } from '@angular/material/dialog';
import { Router } from '@angular/router';

import { AlertEvent } from '../../models/rvt.models';
import { StateService } from '../../services/state.service';

@Component({
  selector: 'app-alerts-dialog',
  imports: [DatePipe, MatButtonModule, MatButtonToggleModule, MatDialogModule, MatIconModule, MatListModule, MatMenuModule],
  templateUrl: './alerts-dialog.component.html',
  styleUrl: './alerts-dialog.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class AlertsDialogComponent {
  protected readonly state = inject(StateService);
  private readonly router = inject(Router);
  private readonly dialogRef = inject(MatDialogRef<AlertsDialogComponent>);
  protected readonly filter = signal<'active' | 'all' | 'info' | 'warn' | 'critical' | 'pinned'>('active');

  protected readonly alerts = computed(() => {
    const now = Date.now();
    const selectedFilter = this.filter();
    return [...this.state.alertHistory()].filter(alert => {
      if (selectedFilter === 'all') return true;
      if (selectedFilter === 'critical') return alert.severity === 'critical' && !alert.dismissed;
      if (selectedFilter === 'warn') return alert.severity === 'warn' && !alert.dismissed;
      if (selectedFilter === 'info') return alert.severity === 'info' && !alert.dismissed;
      if (selectedFilter === 'pinned') return this.state.alertPins().includes(alert.id);
      return !alert.dismissed && (!alert.snoozedUntil || alert.snoozedUntil <= now);
    }).sort((a, b) => {
      const aPinned = this.state.alertPins().includes(a.id) ? 1 : 0;
      const bPinned = this.state.alertPins().includes(b.id) ? 1 : 0;
      return bPinned - aPinned || b.ts - a.ts;
    });
  });

  testAlert(): void {
    this.state.pushAlert('Test alert: verify notifications, tone, and operator response.', 'warn');
  }

  snooze(alert: AlertEvent, minutes: number): void {
    this.state.snoozeAlert(alert.id, minutes);
  }

  jumpToWaveform(alert: AlertEvent): void {
    this.state.waveformSeekAt.set(alert.seekTimestamp || alert.ts);
    this.state.activeTab.set('tab-waves');
    void this.router.navigate(['/live']);
    this.dialogRef.close();
  }

  dismiss(alert: AlertEvent): void {
    this.state.dismissAlert(alert.id);
  }

  acknowledgeVisible(): void {
    this.alerts().filter(alert => !alert.dismissed).forEach(alert => this.state.dismissAlert(alert.id));
  }

  exportHistory(format: 'json' | 'csv'): void {
    const alerts = this.state.alertHistory();
    const content = format === 'json'
      ? JSON.stringify(alerts, null, 2)
      : [
          'timestamp,severity,message,pinned,dismissed,snoozed_until',
          ...alerts.map(alert => [
            new Date(alert.ts).toISOString(),
            alert.severity,
            alert.msg,
            this.state.alertPins().includes(alert.id),
            !!alert.dismissed,
            alert.snoozedUntil ? new Date(alert.snoozedUntil).toISOString() : ''
          ].map(value => this.csvValue(value)).join(','))
        ].join('\n');
    this.downloadText(
      content,
      `radar-vital-alert-history.${format}`,
      format === 'json' ? 'application/json' : 'text/csv'
    );
  }

  private csvValue(value: unknown): string {
    return `"${String(value ?? '').replaceAll('"', '""')}"`;
  }

  private downloadText(content: string, filename: string, type: string): void {
    const href = URL.createObjectURL(new Blob([content], { type }));
    const anchor = document.createElement('a');
    anchor.href = href;
    anchor.download = filename;
    anchor.click();
    URL.revokeObjectURL(href);
  }
}
