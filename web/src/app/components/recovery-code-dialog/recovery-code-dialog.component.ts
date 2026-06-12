import { ChangeDetectionStrategy, Component, Inject, signal } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MAT_DIALOG_DATA, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';

export interface RecoveryCodeDialogData {
  recoveryCode: string;
}

@Component({
  selector: 'app-recovery-code-dialog',
  standalone: true,
  imports: [
    CommonModule,
    MatDialogModule,
    MatButtonModule,
    MatIconModule,
  ],
  templateUrl: './recovery-code-dialog.component.html',
  styleUrl: './recovery-code-dialog.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class RecoveryCodeDialogComponent {
  protected readonly copied = signal(false);
  protected readonly confirmed = signal(false);

  constructor(
    public readonly dialogRef: MatDialogRef<RecoveryCodeDialogComponent>,
    @Inject(MAT_DIALOG_DATA) public readonly data: RecoveryCodeDialogData,
  ) {}

  async copyCode(): Promise<void> {
    const code = this.data.recoveryCode;
    try {
      if (navigator.clipboard?.writeText) {
        await navigator.clipboard.writeText(code);
      } else {
        // Fallback: create temporary textarea
        const ta = document.createElement('textarea');
        ta.value = code;
        ta.style.position = 'fixed';
        ta.style.opacity = '0';
        document.body.appendChild(ta);
        ta.focus();
        ta.select();
        document.execCommand('copy');
        document.body.removeChild(ta);
      }
      this.copied.set(true);
      setTimeout(() => this.copied.set(false), 2000);
    } catch (_) {
      // Silently ignore copy failures
    }
  }

  confirm(): void {
    this.confirmed.set(true);
    this.dialogRef.close();
  }
}
