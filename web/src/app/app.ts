import { Component, OnInit, inject } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';

@Component({
  selector: 'app-root',
  imports: [RouterOutlet, MatSnackBarModule],
  templateUrl: './app.html',
  styleUrl: './app.css'
})
export class App implements OnInit {
  private readonly snackBar = inject(MatSnackBar);
  private applyingUpdate = false;
  private updatePromptShown = false;

  ngOnInit() {
    this.installServiceWorker();
  }

  private installServiceWorker() {
    if (typeof window === 'undefined' || !('serviceWorker' in navigator)) return;

    navigator.serviceWorker.register('./sw.js').then(reg => {
      if (reg.waiting && navigator.serviceWorker.controller) {
        this.promptForUpdate(reg.waiting);
      }
      reg.addEventListener('updatefound', () => {
        const w = reg.installing;
        if (!w) return;
        w.addEventListener('statechange', () => {
          if (w.state === 'installed' && navigator.serviceWorker.controller) {
            this.promptForUpdate(w);
          }
        });
      });
    }).catch(err => {
      console.warn('Service worker registration failed', err);
    });

    navigator.serviceWorker.addEventListener('controllerchange', () => {
      if (this.applyingUpdate) location.reload();
    });
  }

  private promptForUpdate(worker: ServiceWorker): void {
    if (this.updatePromptShown) return;
    this.updatePromptShown = true;
    const prompt = this.snackBar.open(
      'Dashboard update available. Refresh when monitoring is paused.',
      'Refresh',
      { panelClass: 'rvt-update-snackbar' }
    );
    prompt.onAction().subscribe(() => {
      this.applyingUpdate = true;
      worker.postMessage({ type: 'SKIP_WAITING' });
    });
    prompt.afterDismissed().subscribe(() => {
      if (!this.applyingUpdate) this.updatePromptShown = false;
    });
  }
}
