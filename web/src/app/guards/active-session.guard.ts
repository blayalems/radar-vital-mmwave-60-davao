import { inject } from '@angular/core';
import { CanDeactivateFn } from '@angular/router';
import { MatDialog } from '@angular/material/dialog';
import { firstValueFrom } from 'rxjs';
import { StateService } from '../services/state.service';
import { ConfirmDialogComponent } from '../components/confirm-dialog/confirm-dialog.component';

export const activeSessionGuard: CanDeactivateFn<any> = async () => {
  const state = inject(StateService);
  const dialog = inject(MatDialog);

  // A confirmed stop already ends operator monitoring and must be allowed to reach its report.
  if (state.ctlOn() && state.sessionActive() && !state.ctlStopPending()) {
    state.triggerHaptic('confirm');
    const confirmed = await firstValueFrom(
      dialog.open(ConfirmDialogComponent, {
        data: {
          title: 'Leave active session?',
          message: 'Telemetry monitoring is currently active. Navigating away will not stop the session, but you will lose live visibility.',
          confirmLabel: 'Leave view'
        },
        restoreFocus: true
      }).afterClosed()
    );
    return !!confirmed;
  }

  return true;
};
