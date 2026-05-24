import { ChangeDetectionStrategy, Component, inject, signal } from '@angular/core';
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
import { MatDialog, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';
import { firstValueFrom } from 'rxjs';
import { Router } from '@angular/router';

import { StateService, DEFAULT_KPI_THRESHOLDS } from '../../services/state.service';
import { AudioService } from '../../services/audio.service';
import { ApiService } from '../../services/api.service';
import { ConfirmDialogComponent } from '../confirm-dialog/confirm-dialog.component';

@Component({
  selector: 'app-settings',
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
    MatFormFieldModule,
    MatDialogModule,
    MatSnackBarModule
  ],
  templateUrl: './settings.component.html',
  styleUrl: './settings.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class SettingsComponent {
  protected readonly state = inject(StateService);
  protected readonly audio = inject(AudioService);
  protected readonly api = inject(ApiService);
  private readonly router = inject(Router);
  protected readonly Math = Math;
  
  // Optional injection of MatDialogRef for close behavior when loaded inside dialog
  private readonly dialogRef = inject(MatDialogRef<SettingsComponent>, { optional: true });
  private readonly dialog = inject(MatDialog);
  private readonly snackBar = inject(MatSnackBar);
  protected readonly endpointInput = signal(this.api.currentApiBase());
  protected readonly tokenInput = signal('');
  protected readonly connectionBusy = signal(false);

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

  async setDemoMode(enabled: boolean): Promise<void> {
    this.state.demoMode.set(enabled);
    if (!enabled) await this.reconnectTrainer();
  }

  async reconnectTrainer(): Promise<void> {
    this.connectionBusy.set(true);
    this.api.setApiBase(this.endpointInput());
    this.api.setPairToken(this.tokenInput());
    this.state.demoMode.set(false);
    try {
      const connected = await this.api.detectControlMode();
      this.snackBar.open(
        connected ? 'Trainer connection established.' : 'Trainer unavailable; sandbox preview remains active.',
        'Dismiss',
        { duration: 5000 }
      );
    } finally {
      this.connectionBusy.set(false);
    }
  }

  exportPreferences(): void {
    const settings = {
      theme: this.state.theme(),
      density: this.state.density(),
      fontScale: this.state.fontScale(),
      thresholds: this.state.kpiThresholds(),
      liveBufferSeconds: this.state.liveBufferSeconds(),
      freezeOnStale: this.state.freezeOnStale(),
      audioAlertsEnabled: this.state.audioAlertsEnabled(),
      voiceAlertsEnabled: this.state.voiceAlertsEnabled()
    };
    const href = URL.createObjectURL(new Blob([JSON.stringify(settings, null, 2)], { type: 'application/json' }));
    const anchor = document.createElement('a');
    anchor.href = href;
    anchor.download = 'radar-vital-dashboard-preferences.json';
    anchor.click();
    URL.revokeObjectURL(href);
  }

  async importPreferences(event: Event): Promise<void> {
    const file = (event.target as HTMLInputElement).files?.[0];
    if (!file) return;
    try {
      const raw = JSON.parse(await file.text()) as Record<string, unknown>;
      if (['light', 'dark', 'night', 'hc'].includes(String(raw['theme']))) {
        this.state.theme.set(raw['theme'] as 'light' | 'dark' | 'night' | 'hc');
      }
      if (['comfortable', 'compact'].includes(String(raw['density']))) {
        this.state.density.set(raw['density'] as 'comfortable' | 'compact');
      }
      const fontScale = Number(raw['fontScale']);
      if (Number.isFinite(fontScale)) this.state.fontScale.set(Math.max(0.9, Math.min(1.25, fontScale)));
      this.snackBar.open('Dashboard preferences imported.', 'Dismiss', { duration: 4000 });
    } catch (_) {
      this.snackBar.open('Preferences file is not valid JSON.', 'Dismiss', { duration: 5000 });
    }
    (event.target as HTMLInputElement).value = '';
  }

  resetThresholds() {
    this.state.kpiThresholds.set({ ...DEFAULT_KPI_THRESHOLDS });
    this.state.triggerHaptic('confirm');
  }

  async resetAllDefaults() {
    const confirmed = await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
      data: {
        title: 'Reset dashboard defaults?',
        message: 'Theme, alert, telemetry and source-mode preferences will be restored to defaults.',
        confirmLabel: 'Reset defaults'
      },
      restoreFocus: true
    }).afterClosed());
    if (confirmed) {
      this.state.theme.set('dark');
      this.state.density.set('comfortable');
      this.state.fontScale.set(1);
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
