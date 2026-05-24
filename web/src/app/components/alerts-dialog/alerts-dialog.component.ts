import { DatePipe } from '@angular/common';
import { ChangeDetectionStrategy, Component, computed, inject, signal } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatDialogModule } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';
import { MatListModule } from '@angular/material/list';

import { AlertEvent } from '../../models/rvt.models';
import { StateService } from '../../services/state.service';

@Component({
  selector: 'app-alerts-dialog',
  imports: [DatePipe, MatButtonModule, MatButtonToggleModule, MatDialogModule, MatIconModule, MatListModule],
  templateUrl: './alerts-dialog.component.html',
  styleUrl: './alerts-dialog.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class AlertsDialogComponent {
  protected readonly state = inject(StateService);
  protected readonly filter = signal<'active' | 'all' | 'critical' | 'pinned'>('active');

  protected readonly alerts = computed(() => {
    const now = Date.now();
    const selectedFilter = this.filter();
    return [...this.state.alertHistory()].filter(alert => {
      if (selectedFilter === 'all') return true;
      if (selectedFilter === 'critical') return alert.severity === 'critical' && !alert.dismissed;
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

  snooze(alert: AlertEvent): void {
    this.state.snoozeAlert(alert.id);
  }

  dismiss(alert: AlertEvent): void {
    this.state.dismissAlert(alert.id);
  }
}
