import { ChangeDetectionStrategy, Component, inject } from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';

import { IdleLockService } from '../../services/idle-lock.service';
import { StateService } from '../../services/state.service';

@Component({
  selector: 'app-idle-lock-overlay',
  imports: [MatButtonModule, MatIconModule],
  templateUrl: './idle-lock-overlay.component.html',
  styleUrl: './idle-lock-overlay.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class IdleLockOverlayComponent {
  protected readonly idleLock = inject(IdleLockService);
  protected readonly state = inject(StateService);
}
