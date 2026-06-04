import { ChangeDetectionStrategy, Component, DestroyRef, computed, inject } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatDialog, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';
import { firstValueFrom } from 'rxjs';

import { ConfirmDialogComponent } from '../confirm-dialog/confirm-dialog.component';
import { StateService } from '../../services/state.service';

const CLIPBOARD_CLEAR_MS = 30_000;

@Component({
  selector: 'app-operator-handoff-dialog',
  imports: [MatButtonModule, MatDialogModule, MatIconModule, MatSnackBarModule],
  templateUrl: './operator-handoff-dialog.component.html',
  styleUrl: './operator-handoff-dialog.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class OperatorHandoffDialogComponent {
  protected readonly state = inject(StateService);
  private readonly dialog = inject(MatDialog);
  private readonly dialogRef = inject(MatDialogRef<OperatorHandoffDialogComponent>);
  private readonly snackBar = inject(MatSnackBar);
  private readonly destroyRef = inject(DestroyRef);
  private clipboardClearTimer: number | null = null;

  constructor() {
    this.destroyRef.onDestroy(() => {
      if (this.clipboardClearTimer !== null) {
        window.clearTimeout(this.clipboardClearTimer);
        this.clipboardClearTimer = null;
      }
    });
  }

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
  private readonly clipboardPayload = computed(() => {
    const payload = this.handoffPayload();
    return {
      exported_at: payload.exported_at,
      deidentified: true,
      source: payload.source,
      vitals: payload.vitals,
      active_alert_summary: this.activeAlerts().map(alert => ({ severity: alert.severity, ts: alert.ts })),
      active_alert_count: this.activeAlerts().length,
      latest_note_count: this.latestNotes().length
    };
  });

  async copy(): Promise<void> {
    const confirmed = await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
      data: {
        title: 'Copy de-identified handoff?',
        message: 'This copies vitals and alert counts only. Session ID, subject, operator, station, alert text, and free-text notes are excluded. The clipboard clears after 30 seconds when supported.',
        confirmLabel: 'Copy de-ID brief'
      },
      restoreFocus: true
    }).afterClosed());
    if (!confirmed) return;

    try {
      const text = JSON.stringify(this.clipboardPayload(), null, 2);
      await this.writeClipboard(text);
      this.scheduleClipboardClear(text);
      this.snackBar.open('De-identified handoff copied. Clipboard clears in 30 seconds.', 'Dismiss', { duration: 5000 });
      this.dialogRef.close('copied');
    } catch (error) {
      this.snackBar.open(error instanceof Error ? error.message : 'Clipboard unavailable. Use Export JSON instead.', 'Dismiss', { duration: 6000 });
    }
  }

  exportJson(): void {
    let href: string | null = null;
    try {
      href = URL.createObjectURL(new Blob([JSON.stringify(this.handoffPayload(), null, 2)], { type: 'application/json' }));
      const anchor = document.createElement('a');
      anchor.href = href;
      anchor.download = `operator-handoff-${this.state.currentSessionId() || 'local'}.json`;
      anchor.click();
      this.snackBar.open('Operator handoff JSON exported.', 'Dismiss', { duration: 3000 });
    } catch (_) {
      this.snackBar.open('Handoff export failed. Try Print or copy a de-identified brief.', 'Dismiss', { duration: 6000 });
    } finally {
      if (href) URL.revokeObjectURL(href);
    }
  }

  print(): void {
    try {
      window.print();
    } catch (_) {
      this.snackBar.open('Print is unavailable in this browser context.', 'Dismiss', { duration: 5000 });
    }
  }

  private async writeClipboard(text: string): Promise<void> {
    if (typeof navigator === 'undefined' || !navigator.clipboard?.writeText) {
      throw new Error('Clipboard unavailable. Use Export JSON instead.');
    }
    await navigator.clipboard.writeText(text);
  }

  private scheduleClipboardClear(expectedText: string): void {
    if (typeof window === 'undefined') return;
    if (this.clipboardClearTimer !== null) window.clearTimeout(this.clipboardClearTimer);
    this.clipboardClearTimer = window.setTimeout(async () => {
      try {
        if (navigator.clipboard?.readText) {
          const currentText = await navigator.clipboard.readText();
          if (currentText !== expectedText) return;
        }
        await navigator.clipboard?.writeText('');
      } catch (_) {
        // Some browsers allow writing but not reading the clipboard; ignore clear failures.
      }
    }, CLIPBOARD_CLEAR_MS);
  }
}
