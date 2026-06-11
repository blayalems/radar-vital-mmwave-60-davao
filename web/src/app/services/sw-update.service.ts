import { Injectable, inject } from '@angular/core';
import { MatDialog } from '@angular/material/dialog';
import { MatSnackBar } from '@angular/material/snack-bar';
import { firstValueFrom } from 'rxjs';

import { ConfirmDialogComponent } from '../components/confirm-dialog/confirm-dialog.component';

@Injectable({ providedIn: 'root' })
export class SwUpdateService {
  private readonly dialog = inject(MatDialog);
  private readonly snackBar = inject(MatSnackBar);
  private initialized = false;
  private promptOpen = false;
  private noticeOpen = false;
  private applyingUpdate = false;
  private registration: ServiceWorkerRegistration | null = null;

  initialize(): void {
    if (this.initialized) return;
    this.initialized = true;
    if (!this.supportsServiceWorker()) return;

    void this.registerCustomWorker();
    window.setInterval(() => {
      void this.checkNow(false);
    }, 6 * 60 * 60 * 1000);
  }

  async checkNow(forcePrompt: boolean): Promise<boolean> {
    if (!this.supportsServiceWorker()) return false;
    try {
      const registration = await this.ensureRegistration();
      if (!registration) return false;
      await registration.update();
      const waiting = registration.waiting;
      if (waiting && navigator.serviceWorker.controller) {
        if (forcePrompt) return await this.promptReload(waiting);
        this.notifyWaitingWorker(waiting);
        return false;
      }
    } catch (error: unknown) {
      if (forcePrompt) {
        this.snackBar.open(error instanceof Error ? error.message : 'Service worker update check failed.', 'Dismiss', { duration: 5000 });
      }
    }
    return false;
  }

  private async promptReload(worker: ServiceWorker): Promise<boolean> {
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
    worker.postMessage({ type: 'SKIP_WAITING' });
    return true;
  }

  private async registerCustomWorker(): Promise<ServiceWorkerRegistration | null> {
    try {
      const registration = await navigator.serviceWorker.register('./sw.js');
      if (!registration) return null;
      this.registration = registration;
      this.watchRegistration(registration);
      if (registration.waiting && navigator.serviceWorker.controller) {
        this.notifyWaitingWorker(registration.waiting);
      }

      navigator.serviceWorker.addEventListener('controllerchange', () => {
        if (this.applyingUpdate) location.reload();
      });
      return registration;
    } catch (_) {
      return null;
    }
  }

  private watchRegistration(registration: ServiceWorkerRegistration | null): void {
    if (!registration) return;
    registration.addEventListener('updatefound', () => {
      const worker = registration.installing;
      if (!worker) return;
      worker.addEventListener('statechange', () => {
        if (worker.state === 'installed' && navigator.serviceWorker.controller) {
          this.notifyWaitingWorker(worker);
        }
      });
    });
  }

  private notifyWaitingWorker(worker: ServiceWorker): void {
    if (this.noticeOpen || this.promptOpen) return;
    this.noticeOpen = true;
    const snack = this.snackBar.open(
      'Dashboard update available. Refresh when monitoring is paused.',
      'Refresh',
      { panelClass: 'rvt-update-snackbar' }
    );
    snack.onAction().subscribe(() => {
      void this.promptReload(worker);
    });
    snack.afterDismissed().subscribe(() => {
      this.noticeOpen = false;
    });
  }

  private async ensureRegistration(): Promise<ServiceWorkerRegistration | null> {
    if (this.registration) return this.registration;
    const existing = await navigator.serviceWorker.getRegistration('./');
    if (existing) {
      this.registration = existing;
      this.watchRegistration(existing);
      return existing;
    }
    return this.registerCustomWorker();
  }

  private supportsServiceWorker(): boolean {
    return typeof window !== 'undefined' && 'serviceWorker' in navigator;
  }
}
