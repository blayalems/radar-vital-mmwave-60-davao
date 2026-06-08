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
      await registration.update();
      const waiting = registration.waiting;
      if (waiting && navigator.serviceWorker.controller) {
        return await this.promptReload(waiting);
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

  private async registerCustomWorker(): Promise<ServiceWorkerRegistration> {
    const registration = await navigator.serviceWorker.register('./sw.js');
    this.registration = registration;
    this.watchRegistration(registration);
    if (registration.waiting && navigator.serviceWorker.controller) {
      void this.promptReload(registration.waiting);
    }

    navigator.serviceWorker.addEventListener('controllerchange', () => {
      if (this.applyingUpdate) location.reload();
    });
    return registration;
  }

  private watchRegistration(registration: ServiceWorkerRegistration): void {
    registration.addEventListener('updatefound', () => {
      const worker = registration.installing;
      if (!worker) return;
      worker.addEventListener('statechange', () => {
        if (worker.state === 'installed' && navigator.serviceWorker.controller) {
          void this.promptReload(worker);
        }
      });
    });
  }

  private async ensureRegistration(): Promise<ServiceWorkerRegistration> {
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
