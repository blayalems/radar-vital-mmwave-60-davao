import { Injectable, signal } from '@angular/core';
import { AlertEvent, AlertSeverity } from '../../models/rvt.models';

export interface KpiThresholds {
  hrLow: number;
  hrHigh: number;
  rrLow: number;
  rrHigh: number;
}

export const DEFAULT_KPI_THRESHOLDS: KpiThresholds = {
  hrLow: 40,
  hrHigh: 140,
  rrLow: 6,
  rrHigh: 30
};

@Injectable({
  providedIn: 'root'
})
export class AlertStore {
  audioAlertsEnabled = signal<boolean>(false);
  audioVolume = signal<number>(0.7);
  voiceAlertsEnabled = signal<boolean>(false);

  alertHistory = signal<AlertEvent[]>([]);
  alertPins = signal<string[]>([]);
  kpiThresholds = signal<KpiThresholds>({ ...DEFAULT_KPI_THRESHOLDS });
  freezeOnStale = signal<boolean>(true);

  pushAlert(message: string, severity: AlertSeverity = 'warn', source = 'telemetry', seekTimestamp?: number) {
    const now = Date.now();
    const latest = this.alertHistory()[0];
    if (latest?.msg === message && now - latest.ts < 15_000) return;
    this.alertHistory.update(items => [
      { id: `alert_${now}`, ts: now, msg: message, severity, source, seekTimestamp },
      ...items
    ].slice(0, 80));
  }

  toggleAlertPin(alertId: string) {
    this.alertPins.update(items => items.includes(alertId)
      ? items.filter(item => item !== alertId)
      : [...items, alertId]);
  }

  dismissAlert(alertId: string) {
    this.alertHistory.update(items => items.map(item => item.id === alertId
      ? { ...item, dismissed: true }
      : item));
    this.alertPins.update(items => items.filter(item => item !== alertId));
  }

  snoozeAlert(alertId: string, minutes = 10) {
    const until = Date.now() + minutes * 60_000;
    this.alertHistory.update(items => items.map(item => item.id === alertId
      ? { ...item, snoozedUntil: until }
      : item));
  }

  clearAlerts() {
    this.alertHistory.set([]);
    this.alertPins.set([]);
  }
}
