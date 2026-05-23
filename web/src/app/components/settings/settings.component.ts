import { Component, inject } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MatCardModule } from '@angular/material/card';
import { MatSlideToggleModule } from '@angular/material/slide-toggle';
import { MatSliderModule } from '@angular/material/slider';
import { MatButtonModule } from '@angular/material/button';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatDialogRef } from '@angular/material/dialog';
import { Router } from '@angular/router';

import { StateService, DEFAULT_KPI_THRESHOLDS } from '../../services/state.service';
import { AudioService } from '../../services/audio.service';

@Component({
  selector: 'app-settings',
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    MatCardModule,
    MatSlideToggleModule,
    MatSliderModule,
    MatButtonModule,
    MatButtonToggleModule,
    MatIconModule,
    MatInputModule,
    MatFormFieldModule
  ],
  templateUrl: './settings.component.html',
  styleUrl: './settings.component.css'
})
export class SettingsComponent {
  protected readonly state = inject(StateService);
  protected readonly audio = inject(AudioService);
  private readonly router = inject(Router);
  protected readonly Math = Math;
  
  // Optional injection of MatDialogRef for close behavior when loaded inside dialog
  private readonly dialogRef = inject(MatDialogRef<SettingsComponent>, { optional: true });

  testSound() {
    // Temporarily ensure audioAlertsEnabled is true for testing if it was off, then trigger beep
    const wasEnabled = this.state.audioAlertsEnabled();
    if (!wasEnabled) {
      this.state.audioAlertsEnabled.set(true);
    }
    this.audio.playAlertBeep('warn');
    this.audio.speakAlert('Sound check passed.', 'ok', true);
    
    if (!wasEnabled) {
      setTimeout(() => this.state.audioAlertsEnabled.set(false), 500);
    }
  }

  resetThresholds() {
    this.state.kpiThresholds.set({ ...DEFAULT_KPI_THRESHOLDS });
    this.state.triggerHaptic('confirm');
  }

  resetAllDefaults() {
    if (confirm('Are you sure you want to reset all dashboard settings to defaults?')) {
      this.state.theme.set('dark');
      this.state.density.set('comfortable');
      this.state.zenMode.set(false);
      this.state.voiceAlertsEnabled.set(false);
      this.state.audioAlertsEnabled.set(false);
      this.state.audioVolume.set(0.7);
      this.state.liveBufferSeconds.set(60);
      this.state.maxChartPoints.set(3600);
      this.state.demoMode.set(false);
      this.state.autoDemoOnDisconnect.set(false);
      this.state.autoDemoActive.set(false);
      this.state.freezeOnStale.set(true);
      this.state.hxMode.set('auto');
      this.state.activeSubjectProfileId.set('adult_default');
      this.state.kpiThresholds.set({ ...DEFAULT_KPI_THRESHOLDS });
      
      this.state.triggerHaptic('destructiveAccept');
    }
  }

  close() {
    if (this.dialogRef) {
      this.dialogRef.close();
    } else {
      this.router.navigate(['/live']);
    }
    this.state.triggerHaptic('tap');
  }

  onThresholdSliderChange(key: 'hrLow' | 'hrHigh' | 'rrLow' | 'rrHigh', val: number) {
    this.state.kpiThresholds.update(t => ({
      ...t,
      [key]: val
    }));
  }

  onLiveBufferChange(val: string) {
    const num = Math.max(10, Math.min(600, Number(val) || 60));
    this.state.liveBufferSeconds.set(num);
    this.state.maxChartPoints.set(num * 60);
  }
}
