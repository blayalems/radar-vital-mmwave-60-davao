import { ChangeDetectionStrategy, Component, ViewChild, computed, inject, signal, effect } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { MatDialog } from '@angular/material/dialog';

import { IdleLockService } from '../../services/idle-lock.service';
import { StateService } from '../../services/state.service';
import { AuthService } from '../../services/auth.service';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';
import { FirstRunService } from '../../services/first-run.service';
import { A11yModule } from '@angular/cdk/a11y';
import { OperatorProfile } from '../../models/rvt.models';
import { PinKeyboardComponent } from '../pin-keyboard/pin-keyboard.component';
import { RecoveryCodeDialogComponent } from '../recovery-code-dialog/recovery-code-dialog.component';

type ResetStep = 'idle' | 'select-operator' | 'enter-code' | 'enter-new-pin' | 'host-reset-pin';

@Component({
  selector: 'app-idle-lock-overlay',
  standalone: true,
  imports: [
    CommonModule,
    MatButtonModule,
    MatIconModule,
    MatInputModule,
    MatFormFieldModule,
    MatProgressSpinnerModule,
    PinKeyboardComponent,
    A11yModule
  ],
  templateUrl: './idle-lock-overlay.component.html',
  styleUrl: './idle-lock-overlay.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class IdleLockOverlayComponent {
  protected readonly idleLock = inject(IdleLockService);
  protected readonly state = inject(StateService);
  protected readonly auth = inject(AuthService);
  protected readonly serverLifecycle = inject(ServerLifecycleService);
  protected readonly firstRun = inject(FirstRunService);
  private readonly dialog = inject(MatDialog);

  protected readonly isLocked = computed(() =>
    !this.firstRun.consentRequired() && (this.auth.isLocked() || this.idleLock.locked())
  );
  protected readonly isExeHost = computed(() => this.serverLifecycle.platform() === 'exe');

  @ViewChild('pinKeyboard') pinKeyboard?: PinKeyboardComponent;

  selectedOperator = signal<OperatorProfile | null>(null);

  displayName = '';
  initials = '';
  onboardingPin = '';

  isAddingNewOperator = signal<boolean>(false);

  // --- PIN reset / "Forgot PIN?" flow ---
  resetStep = signal<ResetStep>('idle');
  resetOperator = signal<OperatorProfile | null>(null);
  recoveryCode = '';
  resetNewPin = '';
  hostResetPin = '';
  resetError = signal<string | null>(null);

  constructor() {
    effect(() => {
      if (this.isLocked()) {
        void this.auth.loadProfiles();
      }
    });

    effect(() => {
      const current = this.auth.currentOperator();
      if (current) {
        this.selectedOperator.set(current);
      } else {
        const list = this.auth.profiles();
        if (list.length === 1) {
          this.selectedOperator.set(list[0]);
        }
      }
    });

    effect(() => {
      if (this.idleLock.locked() && !this.auth.isLocked()) {
        this.auth.lock();
      }
    });
  }

  selectOperator(op: OperatorProfile): void {
    this.selectedOperator.set(op);
    this.auth.loginError.set(null);
  }

  deselectOperator(): void {
    this.selectedOperator.set(null);
    this.auth.loginError.set(null);
  }

  showAddOperator(): void {
    this.isAddingNewOperator.set(true);
    this.displayName = '';
    this.initials = '';
    this.onboardingPin = '';
    this.auth.loginError.set(null);
  }

  hideAddOperator(): void {
    this.isAddingNewOperator.set(false);
    this.auth.loginError.set(null);
  }

  inputValue(event: Event): string {
    return event.target instanceof HTMLInputElement ? event.target.value : '';
  }

  async handlePinSubmit(pin: string): Promise<void> {
    const op = this.selectedOperator();
    if (!op) return;

    const success = await this.auth.login(op.operator_id, pin);
    if (success) {
      this.idleLock.unlock();
    } else {
      this.pinKeyboard?.reset();
    }
  }

  pressOnboardingPin(key: string): void {
    this.state.triggerHaptic('tap');
    if (key === 'clear') {
      this.onboardingPin = '';
    } else if (key === 'backspace') {
      this.onboardingPin = this.onboardingPin.slice(0, -1);
    } else if (this.onboardingPin.length < 4) {
      this.onboardingPin += key;
    }
  }

  async submitOnboarding(): Promise<void> {
    if (!this.isOnboardingValid()) return;

    const result = await this.auth.createProfile(
      this.displayName.trim(),
      this.initials.trim().toUpperCase(),
      this.onboardingPin
    );
    if (result.success) {
      if (result.recoveryCode) {
        this.openRecoveryCodeDialog(result.recoveryCode);
      }
      this.isAddingNewOperator.set(false);
      this.idleLock.unlock();
    }
  }

  isOnboardingValid(): boolean {
    return (
      this.displayName.trim().length >= 3 &&
      this.initials.trim().length === 2 &&
      this.onboardingPin.length === 4
    );
  }

  // --- "Forgot PIN?" reset flow ---

  startForgotPin(): void {
    this.resetError.set(null);
    const profiles = this.auth.profiles();
    const selected = this.selectedOperator();
    if (selected) {
      this.resetOperator.set(selected);
      this.recoveryCode = '';
      this.resetNewPin = '';
      this.resetStep.set('enter-code');
    } else if (profiles.length === 1) {
      this.resetOperator.set(profiles[0]);
      this.recoveryCode = '';
      this.resetNewPin = '';
      this.resetStep.set('enter-code');
    } else {
      this.resetStep.set('select-operator');
    }
  }

  selectResetOperator(op: OperatorProfile): void {
    this.resetOperator.set(op);
    this.recoveryCode = '';
    this.resetNewPin = '';
    this.resetError.set(null);
    this.resetStep.set('enter-code');
  }

  cancelReset(): void {
    this.resetStep.set('idle');
    this.resetOperator.set(null);
    this.recoveryCode = '';
    this.resetNewPin = '';
    this.hostResetPin = '';
    this.resetError.set(null);
  }

  pressResetNewPin(key: string): void {
    this.state.triggerHaptic('tap');
    if (key === 'clear') {
      this.resetNewPin = '';
    } else if (key === 'backspace') {
      this.resetNewPin = this.resetNewPin.slice(0, -1);
    } else if (this.resetNewPin.length < 4) {
      this.resetNewPin += key;
    }
  }

  pressHostResetPin(key: string): void {
    this.state.triggerHaptic('tap');
    if (key === 'clear') {
      this.hostResetPin = '';
    } else if (key === 'backspace') {
      this.hostResetPin = this.hostResetPin.slice(0, -1);
    } else if (this.hostResetPin.length < 4) {
      this.hostResetPin += key;
    }
  }

  goToNewPin(): void {
    const code = this.recoveryCode.trim().toUpperCase();
    if (code.length < 14) {
      this.resetError.set('Enter the full recovery code (XXXX-XXXX-XXXX).');
      return;
    }
    this.resetError.set(null);
    this.resetNewPin = '';
    this.resetStep.set('enter-new-pin');
  }

  async submitRecoveryReset(): Promise<void> {
    const op = this.resetOperator();
    if (!op || this.resetNewPin.length !== 4) return;
    const newPin = this.resetNewPin;

    const result = await this.auth.resetPin(
      op.operator_id,
      this.recoveryCode.trim().toUpperCase(),
      newPin
    );

    if (result.success) {
      if (result.recoveryCode) {
        this.openRecoveryCodeDialog(result.recoveryCode);
      }
      this.cancelReset();
      // Log in with the new PIN automatically
      const success = await this.auth.login(op.operator_id, newPin);
      if (success) {
        this.idleLock.unlock();
      }
    } else {
      this.resetError.set(result.error || this.auth.loginError() || 'Reset failed. Check the recovery code and try again.');
    }
  }

  startHostReset(): void {
    this.resetError.set(null);
    const profiles = this.auth.profiles();
    const selected = this.selectedOperator();
    if (selected) {
      this.resetOperator.set(selected);
    } else if (profiles.length === 1) {
      this.resetOperator.set(profiles[0]);
    } else {
      // Need to pick operator first
      this.resetOperator.set(null);
    }
    this.hostResetPin = '';
    this.resetStep.set('host-reset-pin');
  }

  async submitHostReset(): Promise<void> {
    const op = this.resetOperator();
    if (!op || this.hostResetPin.length !== 4) return;
    const newPin = this.hostResetPin;

    const result = await this.auth.hostReset(op.operator_id, newPin);

    if (result.success) {
      if (result.recoveryCode) {
        this.openRecoveryCodeDialog(result.recoveryCode);
      }
      this.cancelReset();
      const success = await this.auth.login(op.operator_id, newPin);
      if (success) {
        this.idleLock.unlock();
      }
    } else {
      this.resetError.set(result.error || 'Host reset failed.');
    }
  }

  private openRecoveryCodeDialog(recoveryCode: string): void {
    this.dialog.open(RecoveryCodeDialogComponent, {
      data: { recoveryCode },
      disableClose: true,
      panelClass: 'm3-dialog-panel',
      width: '440px',
    });
  }
}
