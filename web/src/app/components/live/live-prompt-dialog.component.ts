import { ChangeDetectionStrategy, Component, inject } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { MatButtonModule } from '@angular/material/button';
import { MAT_DIALOG_DATA, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';

export interface LivePromptDialogData {
  title: string;
  label: string;
  placeholder?: string;
  confirmLabel: string;
  allowEmpty?: boolean;
  initialValue?: string;
}

@Component({
  selector: 'app-live-prompt-dialog',
  imports: [FormsModule, MatButtonModule, MatDialogModule, MatFormFieldModule, MatInputModule],
  template: `
    <h2 mat-dialog-title>{{ data.title }}</h2>
    <mat-dialog-content>
      <mat-form-field appearance="outline" style="width: 100%; margin-top: 8px;">
        <mat-label>{{ data.label }}</mat-label>
        <input matInput
               [(ngModel)]="value"
               [placeholder]="data.placeholder || ''"
               (keyup.enter)="submit()" />
      </mat-form-field>
    </mat-dialog-content>
    <mat-dialog-actions align="end">
      <button mat-button type="button" (click)="dialogRef.close(null)">Cancel</button>
      <button mat-flat-button
              type="button"
              [disabled]="!data.allowEmpty && !value.trim()"
              (click)="submit()">
        {{ data.confirmLabel }}
      </button>
    </mat-dialog-actions>
  `,
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class LivePromptDialogComponent {
  readonly data = inject<LivePromptDialogData>(MAT_DIALOG_DATA);
  readonly dialogRef = inject(MatDialogRef<LivePromptDialogComponent, string | null>);
  value = this.data.initialValue ?? '';

  protected submit(): void {
    if (!this.data.allowEmpty && !this.value.trim()) return;
    this.dialogRef.close(this.value.trim());
  }
}
