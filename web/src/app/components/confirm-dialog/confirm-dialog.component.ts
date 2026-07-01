import { ChangeDetectionStrategy, Component, inject, ViewEncapsulation } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MAT_DIALOG_DATA, MatDialogModule, MatDialogRef } from '@angular/material/dialog';

export interface ConfirmDialogData {
  title: string;
  message: string;
  confirmLabel: string;
}

@Component({
  selector: 'app-confirm-dialog',
  imports: [MatButtonModule, MatDialogModule],
  template: `
    <h2 mat-dialog-title class="confirm-title">{{ data.title }}</h2>
    <mat-dialog-content class="confirm-message">{{ data.message }}</mat-dialog-content>
    <mat-dialog-actions align="end">
      <button mat-button type="button" (click)="dialogRef.close(false)">Cancel</button>
      <button mat-flat-button type="button" (click)="dialogRef.close(true)">{{ data.confirmLabel }}</button>
    </mat-dialog-actions>
  `,
  styles: [`
    /* Match the prototype's dialog chrome even when opened without the
       'm3-dialog-panel' class (this dialog is launched from many call sites). */
    .mat-mdc-dialog-surface:has(app-confirm-dialog) {
      border: 1px solid var(--md-sys-color-outline-variant);
      border-radius: var(--rv-r1, 22px);
      box-shadow: 0 24px 48px -8px color-mix(in srgb, var(--md-sys-color-shadow, #000) 36%, transparent);
    }
    .confirm-title {
      font-family: var(--rvt-ui-font, 'Inter', sans-serif);
      font-weight: 800;
      color: var(--md-sys-color-on-surface);
    }
    .confirm-message {
      color: var(--md-sys-color-on-surface-variant);
      max-width: 420px;
    }
  `],
  encapsulation: ViewEncapsulation.None,
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class ConfirmDialogComponent {
  readonly data = inject<ConfirmDialogData>(MAT_DIALOG_DATA);
  readonly dialogRef = inject(MatDialogRef<ConfirmDialogComponent>);
}
