import { ChangeDetectionStrategy, Component, computed, inject } from '@angular/core';
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
  private readonly idleLock = inject(IdleLockService);
  private readonly state = inject(StateService);

  protected readonly locked = this.idleLock.locked;
  protected readonly sessionLabel = computed(() => this.state.currentSessionId() || 'Local dashboard');
  protected readonly sourceLabel = computed(() => this.state.ctlStatus()?.mode === 'sandbox' ? 'Demo' : 'Trainer');

  protected resumeDashboard(): void {
    this.idleLock.unlock();
    this.state.triggerHaptic('tap');
  }
}
