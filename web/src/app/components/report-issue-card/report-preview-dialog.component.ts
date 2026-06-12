import { ChangeDetectionStrategy, Component, inject } from '@angular/core';
import { JsonPipe } from '@angular/common';
import { MatButtonModule } from '@angular/material/button';
import { MatDialogModule, MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';

import { IssueReport } from '../../services/issue-report.service';

export interface ReportPreviewDialogData {
  report: IssueReport;
  diagnosticsEnabled: boolean;
}

@Component({
  selector: 'app-report-preview-dialog',
  standalone: true,
  imports: [JsonPipe, MatButtonModule, MatDialogModule, MatIconModule],
  changeDetection: ChangeDetectionStrategy.OnPush,
  template: `
    <h2 mat-dialog-title>
      <mat-icon aria-hidden="true">preview</mat-icon>
      Report preview
    </h2>
    <mat-dialog-content class="report-preview-content">
      @if (!data.diagnosticsEnabled) {
        <p class="preview-opt-out-notice">
          <mat-icon aria-hidden="true">visibility_off</mat-icon>
          Diagnostics are <strong>off</strong>. Only version and platform will be sent.
        </p>
      }
      <pre class="preview-json" aria-label="Report JSON preview">{{ data.report | json }}</pre>
    </mat-dialog-content>
    <mat-dialog-actions align="end">
      <button mat-button mat-dialog-close>Close</button>
    </mat-dialog-actions>
  `,
  styles: [`
    mat-dialog-content { max-height: 60vh; overflow-y: auto; }
    h2[mat-dialog-title] { display: flex; align-items: center; gap: 8px; }
    .preview-opt-out-notice {
      display: flex;
      align-items: center;
      gap: 6px;
      margin-bottom: 12px;
      font-size: 0.875rem;
      opacity: 0.8;
    }
    .preview-json {
      font-size: 0.75rem;
      white-space: pre-wrap;
      word-break: break-word;
      background: var(--md-sys-color-surface-variant, #f1f5f9);
      border-radius: 8px;
      padding: 12px;
    }
  `],
})
export class ReportPreviewDialogComponent {
  protected readonly data = inject<ReportPreviewDialogData>(MAT_DIALOG_DATA);
  protected readonly dialogRef = inject(MatDialogRef<ReportPreviewDialogComponent>);
}
