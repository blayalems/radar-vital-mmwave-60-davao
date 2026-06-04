import { ChangeDetectionStrategy, Component, computed, inject, signal } from '@angular/core';
import { DatePipe } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MatButtonModule } from '@angular/material/button';
import { MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';

import { StateService } from '../../services/state.service';

interface HandoffLine {
  label: string;
  value: string;
}

/**
 * Operator handoff modal (v11 parity).
 *
 * Generates a structured shift-change brief that captures who is on station,
 * the live source mode, the active subject/session, the latest physiological
 * readings, and any pending alerts — plus a free-text note for the incoming
 * operator. The brief can be copied to the clipboard or printed. This is a
 * superset of the existing Ctrl+Shift+C "copy operator summary" action, giving
 * operators a reviewable, annotated handoff rather than a one-line copy.
 */
@Component({
  selector: 'app-operator-handoff-dialog',
  imports: [
    DatePipe,
    FormsModule,
    MatButtonModule,
    MatDialogModule,
    MatIconModule,
    MatFormFieldModule,
    MatInputModule,
    MatSnackBarModule
  ],
  templateUrl: './operator-handoff-dialog.component.html',
  styleUrl: './operator-handoff-dialog.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class OperatorHandoffDialogComponent {
  protected readonly state = inject(StateService);
  private readonly dialogRef = inject(MatDialogRef<OperatorHandoffDialogComponent>);
  private readonly snackBar = inject(MatSnackBar);

  protected readonly generatedAt = Date.now();
  protected handoffNote = '';

  protected readonly sourceLabel = computed(() => {
    const mode = this.state.ctlStatus()?.mode;
    if (this.state.demoMode() || this.state.autoDemoActive() || mode === 'sandbox') return 'DEMO (simulated)';
    if (mode === 'live') return 'Trainer (live)';
    return 'Trainer API';
  });

  protected readonly activeAlertCount = computed(() => {
    const now = Date.now();
    return this.state.alertHistory()
      .filter(a => !a.dismissed && (!a.snoozedUntil || a.snoozedUntil <= now))
      .length;
  });

  protected readonly criticalAlertCount = computed(() => {
    const now = Date.now();
    return this.state.alertHistory()
      .filter(a => a.severity === 'critical' && !a.dismissed && (!a.snoozedUntil || a.snoozedUntil <= now))
      .length;
  });

  protected readonly briefLines = computed<HandoffLine[]>(() => {
    const setup = this.state.setup();
    const payload = this.state.lastLivePayload() || this.state.lastPayload();
    const fmt = (v: unknown, unit: string) =>
      typeof v === 'number' && Number.isFinite(v) ? `${Math.round(v)} ${unit}` : '--';
    return [
      { label: 'Operator', value: setup.operator_label || 'Operator A' },
      { label: 'Station', value: setup.station_label || 'Unassigned' },
      { label: 'Subject', value: setup.subject_label || 'Unlabelled subject' },
      { label: 'Source', value: this.sourceLabel() },
      { label: 'Session active', value: this.state.sessionActive() ? 'Yes' : 'No' },
      { label: 'Session id', value: this.state.currentSessionId() || '—' },
      { label: 'Heart rate', value: fmt(payload?.radar?.reported_hr, 'bpm') },
      { label: 'Respiration', value: fmt(payload?.radar?.reported_rr, 'br/min') },
      { label: 'Target distance', value: fmt(payload?.radar?.distance_cm, 'cm') },
      { label: 'Active alerts', value: `${this.activeAlertCount()} (${this.criticalAlertCount()} critical)` },
      { label: 'Snapshots pinned', value: String(this.state.snaps().length) }
    ];
  });

  protected readonly copied = signal(false);

  private buildBriefText(): string {
    const header = `Radar Vital — Operator Handoff Brief\nGenerated: ${new Date(this.generatedAt).toISOString()}`;
    const body = this.briefLines().map(line => `${line.label}: ${line.value}`).join('\n');
    const note = this.handoffNote.trim()
      ? `\n\nHandoff note:\n${this.handoffNote.trim()}`
      : '';
    return `${header}\n\n${body}${note}\n`;
  }

  async copyBrief(): Promise<void> {
    this.state.triggerHaptic('confirm');
    const text = this.buildBriefText();
    try {
      if (navigator.clipboard?.writeText) {
        await navigator.clipboard.writeText(text);
      } else {
        // Fallback for non-secure contexts where the async clipboard is blocked.
        const ta = document.createElement('textarea');
        ta.value = text;
        ta.style.position = 'fixed';
        ta.style.opacity = '0';
        document.body.appendChild(ta);
        ta.select();
        document.execCommand('copy');
        document.body.removeChild(ta);
      }
      this.copied.set(true);
      this.snackBar.open('Handoff brief copied to clipboard.', 'Dismiss', { duration: 2500 });
    } catch {
      this.snackBar.open('Clipboard blocked by browser policy. Select and copy manually.', 'Dismiss', { duration: 4000 });
    }
  }

  printBrief(): void {
    this.state.triggerHaptic('tap');
    const text = this.buildBriefText();
    const win = window.open('', '_blank', 'noopener,width=640,height=720');
    if (!win) {
      this.snackBar.open('Pop-up blocked. Allow pop-ups to print the brief.', 'Dismiss', { duration: 4000 });
      return;
    }
    const escaped = text.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
    win.document.write(
      `<!doctype html><html><head><title>Operator Handoff Brief</title>` +
      `<meta charset="utf-8"><style>body{font:14px/1.5 -apple-system,Segoe UI,Roboto,sans-serif;padding:32px;color:#0f172a}pre{white-space:pre-wrap;font:inherit}</style>` +
      `</head><body><pre>${escaped}</pre></body></html>`
    );
    win.document.close();
    win.focus();
    win.print();
  }

  close(): void {
    this.dialogRef.close();
  }
}
