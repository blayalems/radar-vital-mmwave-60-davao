import {
  ChangeDetectionStrategy,
  Component,
  inject,
  signal
} from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';
import {
  TERMS_VERSION,
  AUTHORS,
  UNIVERSITY,
  GITHUB_REPO_URL,
  copyrightLine
} from '../../services/app-meta';

@Component({
  selector: 'app-consent-dialog',
  imports: [MatButtonModule, MatDialogModule, MatIconModule],
  template: `
    @if (!declined()) {
      <!-- ── Terms + Privacy summary ─────────────────────────────────────── -->
      <header class="consent-hero">
        <div class="consent-mark" aria-hidden="true">
          <span></span>
          <span></span>
          <mat-icon>policy</mat-icon>
        </div>
        <div>
          <p class="consent-eyebrow">First-run consent</p>
          <h2 mat-dialog-title>Terms of Use &amp; Privacy Notice</h2>
          <p>Review the local data policy before configuring a trainer or demo session.</p>
        </div>
      </header>

      <mat-dialog-content class="consent-content">
        <section class="consent-section">
          <h3>About This Software</h3>
          <p>
            <strong>Radar Vital</strong> is a research-grade mmWave radar vital-sign monitoring
            tool developed at <strong>{{ university }}</strong> for operator-supervised clinical
            research sessions. It is <strong>not a certified medical device</strong> and must not
            be used as the sole basis for clinical decisions, diagnosis, or treatment.
          </p>
        </section>

        <section class="consent-section">
          <h3>Data Collected &amp; Stays Local</h3>
          <p>
            The following information is stored <strong>on this device only</strong>:
          </p>
          <ul>
            <li>Operator name, initials, and a hashed PIN (never stored in plain text)</li>
            <li>Session vital-sign readings (heart rate, respiration rate) and associated metadata</li>
            <li>Snapshots, session reports, and operator sign-off notes</li>
          </ul>
          <p>
            No telemetry, personal health information, or usage data is transmitted to external
            servers by the application. Network communication occurs only between the dashboard
            and the locally-running trainer process.
          </p>
        </section>

        <section class="consent-section">
          <h3>Purpose</h3>
          <p>
            Data collected during sessions is used solely for academic research under the
            <strong>{{ program }}</strong> program at <strong>{{ university }}</strong>.
            Results require qualified clinical validation before any clinical use.
          </p>
        </section>

        <section class="consent-section">
          <h3>Republic Act 10173 — Data Privacy (Philippines)</h3>
          <p>
            In accordance with the <strong>Data Privacy Act of 2012 (RA 10173)</strong>:
          </p>
          <ul>
            <li>You have the right to be informed about what data is collected and how it is used.</li>
            <li>You have the right to access, correct, or erase your personal data.</li>
            <li>
              Data subjects must provide informed consent before any monitoring session begins.
              As operator, you are responsible for obtaining that consent.
            </li>
          </ul>
        </section>

        <div class="consent-links">
          <a [href]="termsUrl" target="_blank" rel="noopener noreferrer">
            <mat-icon aria-hidden="true">gavel</mat-icon>
            Full Terms of Use
          </a>
          <span aria-hidden="true">&nbsp;·&nbsp;</span>
          <a [href]="privacyUrl" target="_blank" rel="noopener noreferrer">
            <mat-icon aria-hidden="true">privacy_tip</mat-icon>
            Full Privacy Policy
          </a>
        </div>

        <p class="consent-version">
          Terms version: {{ termsVersion }}&nbsp;·&nbsp;{{ copyright }}
        </p>
      </mat-dialog-content>

      <mat-dialog-actions align="end">
        <button mat-button type="button" (click)="decline()" class="decline-btn">
          Decline
        </button>
        <button mat-flat-button type="button" (click)="accept()" cdkFocusInitial>
          Accept &amp; Continue
        </button>
      </mat-dialog-actions>
    } @else {
      <!-- ── Blocking panel shown after Decline ──────────────────────────── -->
      <h2 mat-dialog-title class="blocked-title">
        <mat-icon aria-hidden="true" class="blocked-icon">lock</mat-icon>
        Consent Required
      </h2>

      <mat-dialog-content class="blocked-content">
        <p>
          Radar Vital cannot be used without accepting the Terms of Use and Privacy Notice.
          This requirement is necessary to protect research participants and ensure compliance
          with the Data Privacy Act of 2012 (RA 10173).
        </p>
        <p>
          If you have questions about the terms before accepting, you may review the full
          documents below.
        </p>
        <div class="consent-links">
          <a [href]="termsUrl" target="_blank" rel="noopener noreferrer">
            <mat-icon aria-hidden="true">gavel</mat-icon>
            Full Terms of Use
          </a>
          <span aria-hidden="true">&nbsp;·&nbsp;</span>
          <a [href]="privacyUrl" target="_blank" rel="noopener noreferrer">
            <mat-icon aria-hidden="true">privacy_tip</mat-icon>
            Full Privacy Policy
          </a>
        </div>
      </mat-dialog-content>

      <mat-dialog-actions align="end">
        <button mat-button type="button" (click)="backToTerms()" cdkFocusInitial>
          Back to Terms
        </button>
      </mat-dialog-actions>
    }
  `,
  styleUrl: './consent-dialog.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class ConsentDialogComponent {
  private readonly dialogRef = inject(MatDialogRef<ConsentDialogComponent>);

  readonly declined = signal(false);

  readonly termsVersion = TERMS_VERSION;
  readonly university = UNIVERSITY;
  readonly program = 'BS Electronics Engineering';
  readonly authors = AUTHORS;
  readonly copyright = copyrightLine();
  readonly termsUrl = `${GITHUB_REPO_URL}/blob/main/TERMS.md`;
  readonly privacyUrl = `${GITHUB_REPO_URL}/blob/main/PRIVACY.md`;

  accept(): void {
    this.dialogRef.close(true);
  }

  decline(): void {
    this.declined.set(true);
  }

  backToTerms(): void {
    this.declined.set(false);
  }
}
