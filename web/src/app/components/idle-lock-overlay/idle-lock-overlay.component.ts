import { ChangeDetectionStrategy, Component, ViewChild, computed, inject, signal, effect } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';

import { IdleLockService } from '../../services/idle-lock.service';
import { StateService } from '../../services/state.service';
import { AuthService } from '../../services/auth.service';
import { A11yModule } from '@angular/cdk/a11y';
import { OperatorProfile } from '../../models/rvt.models';
import { PinKeyboardComponent } from '../pin-keyboard/pin-keyboard.component';

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

  protected readonly isLocked = computed(() => this.auth.isLocked() || this.idleLock.locked());

  @ViewChild('pinKeyboard') pinKeyboard?: PinKeyboardComponent;

  selectedOperator = signal<OperatorProfile | null>(null);

  displayName = '';
  initials = '';
  onboardingPin = '';

  isAddingNewOperator = signal<boolean>(false);

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

    const success = await this.auth.createProfile(
      this.displayName.trim(),
      this.initials.trim().toUpperCase(),
      this.onboardingPin
    );
    if (success) {
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
}
