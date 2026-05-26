import { ChangeDetectionStrategy, Component, inject } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatListModule } from '@angular/material/list';
import { MatIconModule } from '@angular/material/icon';

@Component({
  selector: 'app-keyboard-shortcuts-dialog',
  imports: [MatButtonModule, MatDialogModule, MatListModule, MatIconModule],
  template: `
    <h2 mat-dialog-title class="shortcuts-title">
      <mat-icon>keyboard</mat-icon>
      Keyboard Shortcuts
    </h2>
    <mat-dialog-content class="shortcuts-content">
      <mat-list>
        @for (shortcut of shortcuts; track shortcut.description) {
        <mat-list-item>
          <span class="shortcut-key"><kbd>{{ shortcut.keys }}</kbd></span>
          <span class="shortcut-desc">
            {{ shortcut.description }}
            @if (shortcut.context) { <small>{{ shortcut.context }}</small> }
          </span>
        </mat-list-item>
        }
      </mat-list>
    </mat-dialog-content>
    <mat-dialog-actions align="end">
      <button mat-flat-button type="button" (click)="dialogRef.close()">Dismiss</button>
    </mat-dialog-actions>
  `,
  styles: [`
    .shortcuts-title {
      display: flex;
      align-items: center;
      gap: 12px;
      margin: 0;
      padding: 16px 24px;
      font-family: 'Inter', 'Roboto', sans-serif;
    }
    .shortcuts-content {
      max-height: min(70vh, 660px);
      padding: 8px 24px 24px !important;
    }
    mat-list-item {
      height: auto !important;
      padding: 10px 0;
      border-bottom: 1px solid var(--md-sys-color-outline-variant, #cbd5e1);
    }
    mat-list-item:last-child {
      border-bottom: none;
    }
    .shortcut-key {
      display: inline-flex;
      gap: 4px;
      font-weight: bold;
      width: 140px;
      flex-shrink: 0;
    }
    .shortcut-desc {
      display: inline-flex;
      flex-direction: column;
      font-family: 'Inter', 'Roboto', sans-serif;
      color: var(--md-sys-color-on-surface-variant, #64748b);
    }
    .shortcut-desc small {
      font-size: 11px;
      opacity: .8;
    }
    kbd {
      background-color: var(--md-sys-color-surface-container-high, #f1f5f9);
      border: 1px solid var(--md-sys-color-outline-variant, #cbd5e1);
      border-radius: 4px;
      padding: 2px 6px;
      font-family: monospace;
      font-size: 13px;
      color: var(--md-sys-color-on-surface, #0f172a);
      box-shadow: 0 1px 1px rgba(0, 0, 0, 0.1);
    }
  `],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class KeyboardShortcutsDialogComponent {
  readonly dialogRef = inject(MatDialogRef<KeyboardShortcutsDialogComponent>);
  readonly shortcuts = [
    { keys: 'Ctrl+K or /', description: 'Open command palette' },
    { keys: '?', description: 'Show keyboard shortcuts' },
    { keys: 'Ctrl+Shift+C', description: 'Copy operator summary' },
    { keys: '1 - 5', description: 'Go to Home, Live, Report, Help or Settings' },
    { keys: 'H / L / R / W / S', description: 'View aliases', context: 'S opens Settings outside Live.' },
    { keys: 'Alt+1 - Alt+6', description: 'Switch Live tabs' },
    { keys: 'Alt+7 / 8 / 9 / 0', description: 'Select 30s / 60s / 120s / Max trend window', context: 'Live only.' },
    { keys: '[ / Alt+\\', description: 'Collapse or expand desktop sidebar' },
    { keys: 'T', description: 'Cycle light, dark, night and high contrast themes' },
    { keys: 'Shift+F', description: 'Toggle Simple and Advanced live presentation' },
    { keys: 'Space', description: 'Pause or resume the telemetry display', context: 'Live only.' },
    { keys: 'A', description: 'Open alerts review' },
    { keys: 'E / X', description: 'Export current payload JSON' },
    { keys: 'Q / N', description: 'Run preflight / start a session', context: 'Home only.' },
    { keys: 'M / C / S / B', description: 'Tag motion, cough, speaking or baseline', context: 'Live only; tags include timestamps.' },
    { keys: 'P / I', description: 'Pin a snapshot / annotate the latest snapshot', context: 'Live only.' },
    { keys: 'O / V / Z', description: 'Overview / Waves / reset trend window', context: 'Live only.' },
    { keys: 'B / V', description: 'Toggle audio / voice alerts', context: 'Outside Live; Live reserves these keys for observations and Waves.' },
    { keys: 'Esc', description: 'Close dialogs and overlays' }
  ];
}
