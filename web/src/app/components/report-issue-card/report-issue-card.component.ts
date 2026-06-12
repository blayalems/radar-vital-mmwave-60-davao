/**
 * ReportIssueCardComponent — settings-style card for one-tap GitHub issue reporting.
 *
 * This component is STANDALONE and intentionally NOT mounted in any route or
 * settings layout yet.  Wave 2 owns settings.component and will insert
 * <app-report-issue-card> there.
 *
 * Features:
 *  - Short description textarea (pre-filled into the GitHub `description` field).
 *  - "Include diagnostics in issue reports" slide-toggle (persisted under DIAGNOSTICS_OPTIN_KEY).
 *  - "Preview report" button — opens a dialog (panelClass 'm3-dialog-panel') showing
 *    exactly what will be sent.
 *  - "Open GitHub issue" button — calls IssueReportService.openReport().
 */

import {
  ChangeDetectionStrategy,
  Component,
  inject,
  signal,
} from '@angular/core';
import { FormsModule } from '@angular/forms';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { MatSlideToggleModule } from '@angular/material/slide-toggle';

import { IssueReportService, IssueReport } from '../../services/issue-report.service';
import {
  ReportPreviewDialogComponent,
  ReportPreviewDialogData,
} from './report-preview-dialog.component';

@Component({
  selector: 'app-report-issue-card',
  standalone: true,
  imports: [
    FormsModule,
    MatButtonModule,
    MatCardModule,
    MatDialogModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule,
    MatProgressSpinnerModule,
    MatSlideToggleModule,
  ],
  templateUrl: './report-issue-card.component.html',
  styleUrl: './report-issue-card.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush,
})
export class ReportIssueCardComponent {
  protected readonly issueReport = inject(IssueReportService);
  private readonly dialog = inject(MatDialog);

  protected readonly userText = signal('');
  protected readonly busy = signal(false);

  protected get diagnosticsEnabled(): boolean {
    return this.issueReport.diagnosticsEnabled();
  }

  protected onToggleChange(checked: boolean): void {
    this.issueReport.setDiagnosticsEnabled(checked);
  }

  protected async previewReport(): Promise<void> {
    this.busy.set(true);
    let report: IssueReport;
    try {
      report = await this.issueReport.buildReport();
    } finally {
      this.busy.set(false);
    }

    this.dialog.open<ReportPreviewDialogComponent, ReportPreviewDialogData>(
      ReportPreviewDialogComponent,
      {
        panelClass: 'm3-dialog-panel',
        data: {
          report,
          diagnosticsEnabled: this.issueReport.diagnosticsEnabled(),
        } satisfies ReportPreviewDialogData,
      },
    );
  }

  protected async openIssue(): Promise<void> {
    this.busy.set(true);
    try {
      const report = await this.issueReport.buildReport();
      const url = this.issueReport.buildIssueUrl(report, this.userText() || undefined);
      await this.issueReport.openReport(url);
    } finally {
      this.busy.set(false);
    }
  }
}
