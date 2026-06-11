import { ChangeDetectionStrategy, Component, inject, signal, OnInit, ViewChild } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';

import { AuthService } from '../../services/auth.service';
import { OperatorProfile } from '../../models/rvt.models';
import { PinKeyboardComponent } from '../pin-keyboard/pin-keyboard.component';

@Component({
  selector: 'app-switch-operator-dialog',
  standalone: true,
  imports: [
    CommonModule,
    MatDialogModule,
    MatButtonModule,
    MatIconModule,
    MatProgressSpinnerModule,
    MatFormFieldModule,
    MatInputModule,
    PinKeyboardComponent
  ],
  templateUrl: './switch-operator-dialog.component.html',
  styleUrl: './switch-operator-dialog.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class SwitchOperatorDialogComponent implements OnInit {
  protected readonly auth = inject(AuthService);
  private readonly dialogRef = inject(MatDialogRef<SwitchOperatorDialogComponent>);

  selectedOperator = signal<OperatorProfile | null>(null);
  addingProfile = signal(false);
  displayName = '';
  initials = '';
  pin = '';

  @ViewChild('pinKeyboard') pinKeyboard?: PinKeyboardComponent;

  ngOnInit(): void {
    this.auth.loginError.set(null);
    void this.auth.loadProfiles();
  }

  selectOperator(op: OperatorProfile): void {
    this.addingProfile.set(false);
    this.selectedOperator.set(op);
    this.auth.loginError.set(null);
    this.pinKeyboard?.reset();
  }

  goBack(): void {
    this.selectedOperator.set(null);
    this.addingProfile.set(false);
    this.auth.loginError.set(null);
  }

  startAddProfile(): void {
    this.selectedOperator.set(null);
    this.addingProfile.set(true);
    this.displayName = '';
    this.initials = '';
    this.pin = '';
    this.auth.loginError.set(null);
  }

  inputValue(event: Event): string {
    return event.target instanceof HTMLInputElement ? event.target.value : '';
  }

  canCreateProfile(): boolean {
    return this.displayName.trim().length >= 3
      && /^[A-Za-z]{2,5}$/.test(this.initials.trim())
      && /^\d{4}$/.test(this.pin.trim());
  }

  async createProfile(): Promise<void> {
    if (!this.canCreateProfile()) return;
    const success = await this.auth.createProfile(
      this.displayName.trim(),
      this.initials.trim().toUpperCase(),
      this.pin.trim()
    );
    if (success) {
      this.dialogRef.close(true);
    }
  }

  async handlePinSubmit(pin: string): Promise<void> {
    const op = this.selectedOperator();
    if (!op) return;

    const success = await this.auth.login(op.operator_id, pin);
    if (success) {
      this.dialogRef.close(true);
    } else {
      this.pinKeyboard?.reset();
    }
  }

  close(): void {
    this.dialogRef.close(false);
  }
}
