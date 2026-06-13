import { inject } from '@angular/core';
import { CanActivateFn, Router } from '@angular/router';
import { SERVER_URL_KEY } from '../services/rvt-storage-keys';

export const connectRouteGuard: CanActivateFn = () => {
  const router = inject(Router);
  try {
    const serverUrl = localStorage.getItem(SERVER_URL_KEY);
    // "Demo Now" stores a placeholder trainer URL; don't treat that as a real
    // connection, otherwise demo-first users are bounced to /live and can never
    // reopen the wizard to scan a QR or pair a LAN trainer.
    const demoMode = localStorage.getItem('rvt-demo-mode') === '1';
    if (serverUrl && !demoMode) {
      void router.navigate(['/live']);
      return false;
    }
  } catch (_) {}
  return true;
};
