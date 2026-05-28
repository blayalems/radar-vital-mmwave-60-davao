import { inject } from '@angular/core';
import { CanActivateFn, Router } from '@angular/router';
import { ApiService } from '../services/api.service';
import { StateService } from '../services/state.service';
import { toObservable } from '@angular/core/rxjs-interop';
import { filter, firstValueFrom, timeout, of, catchError } from 'rxjs';

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

  // Wait for initial connection resolution (with a fallback timeout to let layout mount and overlay show)
  try {
    await firstValueFrom(
      toObservable(api.connectionLoading).pipe(
        filter(loading => !loading),
        timeout(1500),
        catchError(() => of(false))
      )
    );
  } catch (_) {}

  // If loading is still true after the timeout, return true to let LayoutComponent mount,
  // showing the connection-loading overlay with the "Bypass to Sandbox Mode" button.
  if (api.connectionLoading()) {
    return true;
  }

  if (!state.ctlOn()) {
    void router.navigate(['/home']);
    return false;
  }
  return true;
};
