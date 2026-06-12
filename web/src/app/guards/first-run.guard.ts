import { inject } from '@angular/core';
import { CanActivateFn, Router } from '@angular/router';
import { SERVER_URL_KEY } from '../services/rvt-storage-keys';

export const firstRunGuard: CanActivateFn = () => {
  const router = inject(Router);
  try {
    const serverUrl = localStorage.getItem(SERVER_URL_KEY);
    if (!serverUrl) {
      void router.navigate(['/connect']);
      return false;
    }
  } catch (_) {
    void router.navigate(['/connect']);
    return false;
  }
  return true;
};
