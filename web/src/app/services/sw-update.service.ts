import { Injectable, inject } from '@angular/core';
import { MatDialog } from '@angular/material/dialog';
import { MatSnackBar } from '@angular/material/snack-bar';
import { SwUpdate, VersionReadyEvent } from '@angular/service-worker';
import { filter, firstValueFrom } from 'rxjs';

import { ConfirmDialogComponent } from '../components/confirm-dialog/confirm-dialog.component';

@Injectable({ providedIn: 'root' })
export class SwUpdateService {
  private readonly swUpdate = inject(SwUpdate, { optional: true });
  private readonly dialog = inject(MatDialog);
  private readonly snackBar = inject(MatSnackBar);
  private initialized = false;
  private promptOpen = false;
  private applyingUpdate = false;

  initialize(): void {
    if (this.initialized) return;
    this.initialized = true;
    this.installCustomWorkerFallback();

    if (this.swUpdate?.isEnabled) {
      this.swUpdate.versionUpdates.pipe(
        filter((event): event is VersionReadyEvent => event.type === 'VERSION_READY')
      ).subscribe(() => void this.promptReload());

      window.setInterval(() => {
        void this.checkNow(false);
      }, 6 * 60 * 60 * 1000);

      void this.checkNow(false);
    }
  }

  async checkNow(forcePrompt: boolean): Promise<boolean> {
    if (!this.swUpdate?.isEnabled) return false;
    try {
      const ready = await this.swUpdate.checkForUpdate();
      if (ready || forcePrompt) {
        return await this.promptReload();
      }
    } catch (error: unknown) {
      if (forcePrompt) {
        this.snackBar.open(error instanceof Error ? error.message : 'Service worker update check failed.', 'Dismiss', { duration: 5000 });
      }
    }
    return false;
  }

  private async promptReload(): Promise<boolean> {
    if (this.promptOpen) return false;
    this.promptOpen = true;
    const confirmed = await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
      data: {
        title: 'Update Available',
        message: 'A new Radar Vital dashboard build is ready. Reload now to apply it?',
        confirmLabel: 'Reload'
      },
      restoreFocus: true,
      panelClass: 'm3-dialog-panel'
    }).afterClosed());
    this.promptOpen = false;
    if (!confirmed) return false;
    this.applyingUpdate = true;
    await this.swUpdate?.activateUpdate().catch(() => undefined);
    document.location.reload();
    return true;
  }

  private installCustomWorkerFallback(): void {
    if (typeof window === 'undefined' || !('serviceWorker' in navigator)) return;

    navigator.serviceWorker.register('./sw.js').then(registration => {
      if (registration.waiting && navigator.serviceWorker.controller) {
        this.promptWaitingWorker(registration.waiting);
      }
      registration.addEventListener('updatefound', () => {
        const worker = registration.installing;
        if (!worker) return;
        worker.addEventListener('statechange', () => {
          if (worker.state === 'installed' && navigator.serviceWorker.controller) {
            this.promptWaitingWorker(worker);
          }
        });
      });
    }).catch(error => {
      console.warn('Service worker registration failed', error);
    });

    navigator.serviceWorker.addEventListener('controllerchange', () => {
      if (this.applyingUpdate) location.reload();
    });
  }

  private promptWaitingWorker(worker: ServiceWorker): void {
    if (this.promptOpen) return;
    this.promptOpen = true;
    const snack = this.snackBar.open(
      'Dashboard update available. Refresh when monitoring is paused.',
      'Refresh',
      { panelClass: 'rvt-update-snackbar' }
    );
    snack.onAction().subscribe(() => {
      this.applyingUpdate = true;
      worker.postMessage({ type: 'SKIP_WAITING' });
    });
    snack.afterDismissed().subscribe(() => {
      if (!this.applyingUpdate) this.promptOpen = false;
    });
  }
}
