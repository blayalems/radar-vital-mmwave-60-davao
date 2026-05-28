import { inject } from '@angular/core';
import { CanActivateFn, Router } from '@angular/router';
import { ApiService } from '../services/api.service';
import { StateService } from '../services/state.service';
import { toObservable } from '@angular/core/rxjs-interop';
import { filter, firstValueFrom } from 'rxjs';

/**
 * Route guard that ensures the initial connection status has resolved
 * before allowing navigation to active session/live monitoring views.
 */
export const connectionGuard: CanActivateFn = async () => {
  const api = inject(ApiService);
  const state = inject(StateService);
  const router = inject(Router);

  // If already finished loading, evaluate immediately
  if (!api.connectionLoading()) {
    if (!state.ctlOn()) {
      void router.navigate(['/home']);
      return false;
    }
    return true;
  }

  // Wait for initial connection resolution (4s timeout or success/fallback)
  try {
    await firstValueFrom(
      toObservable(api.connectionLoading).pipe(
        filter(loading => !loading)
      )
    );
  } catch (_) {}

  if (!state.ctlOn()) {
    void router.navigate(['/home']);
    return false;
  }
  return true;
};
