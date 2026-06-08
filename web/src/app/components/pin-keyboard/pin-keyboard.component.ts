import { Component, Input, Output, EventEmitter, HostListener, inject } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { OperatorProfile } from '../../models/rvt.models';
import { StateService } from '../../services/state.service';

@Component({
  selector: 'app-pin-keyboard',
  standalone: true,
  imports: [CommonModule, MatButtonModule, MatIconModule, MatProgressSpinnerModule],
  templateUrl: './pin-keyboard.component.html',
  styleUrl: './pin-keyboard.component.css'
})
export class PinKeyboardComponent {
  @Input() operator: OperatorProfile | null = null;
  @Input() showCancel = false;
  @Input() error: string | null = null;
  @Input() loading = false;
  @Input() lockoutSeconds = 0;

  @Output() submitPin = new EventEmitter<string>();
  @Output() cancel = new EventEmitter<void>();

  private readonly state = inject(StateService);
  pin = '';

  pressKey(key: string): void {
    if (this.loading || this.lockoutSeconds > 0) return;
    this.state.triggerHaptic('tap');

    if (key === 'clear') {
      this.pin = '';
    } else if (key === 'backspace') {
      this.pin = this.pin.slice(0, -1);
    } else if (this.pin.length < 4) {
      this.pin += key;
      if (this.pin.length === 4) {
        this.submitPin.emit(this.pin);
      }
    }
  }

  reset(): void {
    this.pin = '';
  }

  @HostListener('window:keydown', ['$event'])
  handleKeyDown(event: KeyboardEvent): void {
    if (this.loading || this.lockoutSeconds > 0) return;
    const key = event.key;
    if (key >= '0' && key <= '9') {
      this.pressKey(key);
    } else if (key === 'Backspace') {
      this.pressKey('backspace');
    } else if (key === 'Escape') {
      if (this.showCancel) {
        this.cancel.emit();
      } else {
        this.pressKey('clear');
      }
    }
  }
}
