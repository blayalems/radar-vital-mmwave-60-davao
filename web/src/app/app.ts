import { Component, DestroyRef, OnInit, effect, inject } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MatSnackBarModule } from '@angular/material/snack-bar';

import { SwUpdateService } from './services/sw-update.service';
import { FirstRunService } from './services/first-run.service';
import { ConsentDialogComponent } from './components/consent-dialog/consent-dialog.component';
import { OnboardingTutorialComponent } from './components/onboarding-tutorial/onboarding-tutorial.component';

@Component({
  selector: 'app-root',
  imports: [RouterOutlet, MatDialogModule, MatSnackBarModule],
  templateUrl: './app.html',
  styleUrl: './app.css'
})
export class App implements OnInit {
  private readonly swUpdate = inject(SwUpdateService);
  private readonly firstRun = inject(FirstRunService);
  private readonly dialog = inject(MatDialog);
  private readonly destroyRef = inject(DestroyRef);

  private destroyed = false;
  private consentDialogOpen = false;
  private tutorialDialogOpen = false;

  constructor() {
    this.destroyRef.onDestroy(() => {
      this.destroyed = true;
    });

    effect(() => {
      if (this.firstRun.consentRequired() && !this.consentDialogOpen && !this.destroyed) {
        queueMicrotask(() => {
          if (!this.destroyed) this.maybeOpenConsentGate();
        });
      }
    });

    // effect() must be called inside the injection context (constructor or field
    // initializer). Watch tutorialOpen signal to open tutorial on demand,
    // covering both the auto-show after auth and replayTutorial().
    effect(() => {
      const open = this.firstRun.tutorialOpen();
      if (open && !this.tutorialDialogOpen) {
        this.openTutorialDialog();
      }
    });
  }

  ngOnInit() {
    this.swUpdate.initialize();
    this.maybeOpenConsentGate();
  }

  /** Open consent dialog if terms have not been accepted or version has bumped. */
  private maybeOpenConsentGate(): void {
    if (this.destroyed || !this.firstRun.consentRequired() || this.consentDialogOpen) return;
    this.consentDialogOpen = true;

    const ref = this.dialog.open(ConsentDialogComponent, {
      disableClose: true,
      panelClass: 'm3-dialog-panel',
      maxWidth: 'min(560px, calc(100vw - 24px))',
      width: '100%'
    });

    ref.afterClosed().subscribe((accepted: boolean) => {
      this.consentDialogOpen = false;
      if (accepted) {
        this.firstRun.acceptConsent();
      } else if (!this.destroyed && this.firstRun.consentRequired()) {
        queueMicrotask(() => this.maybeOpenConsentGate());
      }
      // If not accepted (declined), the dialog component swapped to a blocking
      // panel internally. disableClose:true ensures it stays open until the user
      // returns to terms and accepts.
    });
  }

  private openTutorialDialog(): void {
    this.tutorialDialogOpen = true;

    const ref = this.dialog.open(OnboardingTutorialComponent, {
      disableClose: false,
      panelClass: 'm3-dialog-panel',
      maxWidth: 'min(520px, calc(100vw - 24px))',
      width: '100%'
    });

    ref.afterClosed().subscribe(() => {
      this.tutorialDialogOpen = false;
      // Ensure done flag is set if user closes via Escape / backdrop click
      if (this.firstRun.tutorialOpen()) {
        this.firstRun.markTutorialDone();
      }
    });
  }
}
