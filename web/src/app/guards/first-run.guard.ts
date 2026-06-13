import { inject } from '@angular/core';
import { CanActivateFn, Router } from '@angular/router';
import { SERVER_URL_KEY } from '../services/rvt-storage-keys';

export const firstRunGuard: CanActivateFn = () => {
  const router = inject(Router);
  try {
    // A QR deep-link (`http://<lan-ip>:8765/?pair=<PIN>`) must reach the layout
    // so ApiService.consumePairPinFromUrl() can exchange the PIN. Don't bounce it
    // to the connect wizard — doing so drops the `pair` query and breaks one-tap
    // pairing for first-run phones/PWAs that have no stored trainer URL yet.
    const hasPairParam = new URLSearchParams(window.location.search).has('pair');
    const serverUrl = localStorage.getItem(SERVER_URL_KEY);
    if (!serverUrl && !hasPairParam) {
      void router.navigate(['/connect']);
      return false;
    }
  } catch (_) {
    void router.navigate(['/connect']);
    return false;
  }
  return true;
};
