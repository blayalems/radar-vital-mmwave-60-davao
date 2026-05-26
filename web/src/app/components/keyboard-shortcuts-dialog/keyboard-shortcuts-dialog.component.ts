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
        <mat-list-item>
          <span class="shortcut-key"><kbd>Ctrl</kbd> + <kbd>K</kbd></span>
          <span class="shortcut-desc">Open Command Palette</span>
        </mat-list-item>
        <mat-list-item>
          <span class="shortcut-key"><kbd>Ctrl</kbd> + <kbd>/</kbd> or <kbd>?</kbd></span>
          <span class="shortcut-desc">Show Keyboard Shortcuts</span>
        </mat-list-item>
        <mat-list-item>
          <span class="shortcut-key"><kbd>Alt</kbd> + <kbd>1</kbd></span>
          <span class="shortcut-desc">Switch to Live Overview</span>
        </mat-list-item>
        <mat-list-item>
          <span class="shortcut-key"><kbd>Alt</kbd> + <kbd>2</kbd></span>
          <span class="shortcut-desc">Switch to Live Waves</span>
        </mat-list-item>
        <mat-list-item>
          <span class="shortcut-key"><kbd>Alt</kbd> + <kbd>3</kbd></span>
          <span class="shortcut-desc">Switch to HR Funnel</span>
        </mat-list-item>
        <mat-list-item>
          <span class="shortcut-key"><kbd>Alt</kbd> + <kbd>4</kbd></span>
          <span class="shortcut-desc">Switch to RR Funnel</span>
        </mat-list-item>
        <mat-list-item>
          <span class="shortcut-key"><kbd>Alt</kbd> + <kbd>5</kbd></span>
          <span class="shortcut-desc">Switch to Snapshots</span>
        </mat-list-item>
        <mat-list-item>
          <span class="shortcut-key"><kbd>Alt</kbd> + <kbd>6</kbd></span>
          <span class="shortcut-desc">Switch to Audit Log</span>
        </mat-list-item>
        <mat-list-item>
          <span class="shortcut-key"><kbd>Esc</kbd></span>
          <span class="shortcut-desc">Close dialogs and overlays</span>
        </mat-list-item>
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
      font-family: 'Inter', 'Roboto', sans-serif;
      color: var(--md-sys-color-on-surface-variant, #64748b);
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
}
