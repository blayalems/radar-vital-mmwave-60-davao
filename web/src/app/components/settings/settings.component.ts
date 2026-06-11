import { ChangeDetectionStrategy, Component, computed, inject, signal } from '@angular/core';
import { FormsModule } from '@angular/forms';
import { MatCardModule } from '@angular/material/card';
import { MatSlideToggleModule } from '@angular/material/slide-toggle';
import { MatSliderModule } from '@angular/material/slider';
import { MatButtonModule } from '@angular/material/button';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatIconModule } from '@angular/material/icon';
import { MatChipsModule } from '@angular/material/chips';
import { MatInputModule } from '@angular/material/input';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatDialog, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatTooltipModule } from '@angular/material/tooltip';
import { firstValueFrom } from 'rxjs';
import { Router } from '@angular/router';

import { StateService, DEFAULT_KPI_THRESHOLDS, KPI_THRESHOLD_META, KpiThresholds } from '../../services/state.service';
import { AudioService } from '../../services/audio.service';
import { ApiService } from '../../services/api.service';
import { DynamicColorService } from '../../services/dynamic-color.service';
import { IdleLockService } from '../../services/idle-lock.service';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';
import { UpdateService } from '../../services/update.service';
import { BleScanDevice } from '../../models/rvt.models';
import { ConfirmDialogComponent } from '../confirm-dialog/confirm-dialog.component';

const PRODUCT_VERSION = '16.1.0';

@Component({
  selector: 'app-settings',
  imports: [
    FormsModule,
    MatCardModule,
    MatSlideToggleModule,
    MatSliderModule,
    MatButtonModule,
    MatButtonToggleModule,
    MatIconModule,
    MatChipsModule,
    MatInputModule,
    MatFormFieldModule,
    MatDialogModule,
    MatSnackBarModule,
    MatProgressBarModule,
    MatTooltipModule
  ],
  templateUrl: './settings.component.html',
  styleUrl: './settings.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class SettingsComponent {
  protected readonly state = inject(StateService);
  protected readonly audio = inject(AudioService);
  protected readonly api = inject(ApiService);
  protected readonly dynamicColor = inject(DynamicColorService);
  protected readonly idleLock = inject(IdleLockService);
  protected readonly serverLifecycle = inject(ServerLifecycleService);
  protected readonly updateService = inject(UpdateService);
  private readonly router = inject(Router);
  protected readonly Math = Math;
  protected readonly productVersion = PRODUCT_VERSION;
  protected readonly schemaVersion = 'v12.0';
  
  // Optional injection of MatDialogRef for close behavior when loaded inside dialog
  private readonly dialogRef = inject(MatDialogRef<SettingsComponent>, { optional: true });
  private readonly dialog = inject(MatDialog);
  private readonly snackBar = inject(MatSnackBar);
  protected readonly endpointInput = signal(this.serverLifecycle.serverAddress());
  protected readonly pinInput = signal('');
  protected readonly connectionBusy = signal(false);
  protected readonly bleScanBusy = signal(false);
  protected readonly bleScanDevices = signal<BleScanDevice[]>([]);
  protected readonly bleScanMessage = signal('No scan run yet.');

  // Material You preset swatches
  protected readonly presetColors: { hex: string; name: string }[] = [
    { hex: '#0061a4', name: 'Azure' },
    { hex: '#6750a4', name: 'Purple' },
    { hex: '#006c4c', name: 'Green' },
    { hex: '#984061', name: 'Rose' },
    { hex: '#7c5800', name: 'Gold' },
    { hex: '#006874', name: 'Teal' },
    { hex: '#ba1a1a', name: 'Red' },
    { hex: '#5d5f5f', name: 'Neutral' },
  ];

  onSourceColorInput(event: Event): void {
    const hex = (event.target as HTMLInputElement).value;
    this.dynamicColor.setSourceColor(hex);
  }

  selectPresetColor(hex: string): void {
    this.dynamicColor.setSourceColor(hex);
    this.state.triggerHaptic('tap');
  }

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
    this.serverLifecycle.setServerAddress(this.endpointInput());
    this.state.demoMode.set(false);
    try {
      await this.serverLifecycle.retryConnection();
      const connected = this.serverLifecycle.status() === 'running';
      this.snackBar.open(
        connected ? 'Trainer connection established.' : 'Trainer unavailable; sandbox preview remains active.',
        'Dismiss',
        { duration: 5000 }
      );
    } finally {
      this.connectionBusy.set(false);
    }
  }

  async pairWithPin(): Promise<void> {
    this.connectionBusy.set(true);
    this.serverLifecycle.setServerAddress(this.endpointInput());
    try {
      await this.api.exchangePairPin(this.pinInput());
      this.pinInput.set('');
      await this.serverLifecycle.retryConnection();
      this.state.demoMode.set(false);
      this.snackBar.open('LAN trainer paired for this browser session.', 'Dismiss', { duration: 5000 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Pairing failed.', 'Dismiss', { duration: 6000 });
    } finally {
      this.connectionBusy.set(false);
    }
  }

  protected readonly isSharing = computed(() => this.serverLifecycle.bindMode() === 'lan');

  async onShareToggleChange(checked: boolean): Promise<void> {
    this.connectionBusy.set(true);
    try {
      const mode = checked ? 'lan' : 'local';
      await this.serverLifecycle.restartServer(mode);
      this.endpointInput.set(this.serverLifecycle.serverAddress());
    } catch (error: unknown) {
      this.snackBar.open(
        error instanceof Error ? error.message : 'Could not change sharing mode.',
        'Dismiss',
        { duration: 5000 }
      );
    } finally {
      this.connectionBusy.set(false);
    }
  }

  async restartInLanMode(): Promise<void> {
    this.connectionBusy.set(true);
    try {
      await this.serverLifecycle.restartServer('lan');
      this.endpointInput.set(this.serverLifecycle.serverAddress());
    } catch (error: unknown) {
      this.snackBar.open(
        error instanceof Error ? error.message : 'Could not generate a new PIN.',
        'Dismiss',
        { duration: 5000 }
      );
    } finally {
      this.connectionBusy.set(false);
    }
  }

  async startPythonServer(): Promise<void> {
    this.connectionBusy.set(true);
    try {
      await this.serverLifecycle.startServer(this.serverLifecycle.bindMode());
      this.endpointInput.set(this.serverLifecycle.serverAddress());
    } finally {
      this.connectionBusy.set(false);
    }
  }

  async stopPythonServer(): Promise<void> {
    this.connectionBusy.set(true);
    try {
      await this.serverLifecycle.stopServer();
    } finally {
      this.connectionBusy.set(false);
    }
  }

  async restartPythonServer(): Promise<void> {
    this.connectionBusy.set(true);
    try {
      await this.serverLifecycle.restartServer();
      this.endpointInput.set(this.serverLifecycle.serverAddress());
    } finally {
      this.connectionBusy.set(false);
    }
  }

  async retryPythonServer(): Promise<void> {
    this.connectionBusy.set(true);
    this.serverLifecycle.setServerAddress(this.endpointInput());
    try {
      await this.serverLifecycle.retryConnection();
      this.snackBar.open(
        this.serverLifecycle.status() === 'running' ? 'Python server connection is online.' : 'Python server is still offline.',
        'Dismiss',
        { duration: 5000 }
      );
    } finally {
      this.connectionBusy.set(false);
    }
  }

  openPairPage(): void {
    this.serverLifecycle.setServerAddress(this.endpointInput());
    this.serverLifecycle.openPairPage();
  }

  async scanBleDevices(): Promise<void> {
    this.bleScanBusy.set(true);
    this.bleScanMessage.set('Scanning for BLE reference devices...');
    try {
      const result = await this.api.request<{ ok?: boolean; devices?: BleScanDevice[]; error?: string }>('/api/ble/scan');
      const devices = Array.isArray(result.devices) ? result.devices : [];
      this.bleScanDevices.set(devices);
      this.bleScanMessage.set(devices.length
        ? `${devices.length} device${devices.length === 1 ? '' : 's'} found.`
        : 'No BLE reference devices found.');
      this.state.triggerHaptic(devices.length ? 'confirm' : 'warn');
    } catch (error: unknown) {
      this.bleScanDevices.set([]);
      this.bleScanMessage.set(error instanceof Error ? error.message : 'BLE scan failed.');
      this.state.triggerHaptic('warn');
    } finally {
      this.bleScanBusy.set(false);
    }
  }

  selectBleDevice(device: BleScanDevice): void {
    const address = device.address || device.id || '';
    if (!address) {
      this.snackBar.open('Selected BLE device does not include an address.', 'Dismiss', { duration: 5000 });
      return;
    }
    this.state.setup.update(current => ({ ...current, ble_address: address }));
    this.snackBar.open(`BLE reference set to ${device.name || address}.`, 'Dismiss', { duration: 4000 });
    this.state.triggerHaptic('tap');
  }

  exportPreferences(): void {
    const settings = {
      theme: this.state.theme(),
      density: this.state.density(),
      fontScale: this.state.fontScale(),
      zenMode: this.state.zenMode(),
      hxMode: this.state.hxMode(),
      thresholds: this.state.kpiThresholds(),
      liveBufferSeconds: this.state.liveBufferSeconds(),
      freezeOnStale: this.state.freezeOnStale(),
      audioAlertsEnabled: this.state.audioAlertsEnabled(),
      voiceAlertsEnabled: this.state.voiceAlertsEnabled(),
      audioVolume: this.state.audioVolume(),
      autoDemoOnDisconnect: this.state.autoDemoOnDisconnect()
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
      if (typeof raw['zenMode'] === 'boolean') this.state.zenMode.set(raw['zenMode']);
      if (['on', 'off', 'auto'].includes(String(raw['hxMode']))) {
        this.state.hxMode.set(raw['hxMode'] as 'on' | 'off' | 'auto');
      }
      const thresholds = raw['thresholds'];
      if (thresholds && typeof thresholds === 'object') {
        const source = thresholds as Partial<Record<keyof KpiThresholds, unknown>>;
        const current = this.state.kpiThresholds();
        const clampThreshold = (key: keyof KpiThresholds): number => {
          const value = Number(source[key]);
          const limits = KPI_THRESHOLD_META[key];
          return Number.isFinite(value)
            ? Math.max(limits.min, Math.min(limits.max, Math.round(value)))
            : current[key];
        };
        this.state.kpiThresholds.set({
          hrLow: clampThreshold('hrLow'),
          hrHigh: clampThreshold('hrHigh'),
          rrLow: clampThreshold('rrLow'),
          rrHigh: clampThreshold('rrHigh')
        });
      }
      const liveBufferSeconds = Number(raw['liveBufferSeconds']);
      if (Number.isFinite(liveBufferSeconds)) {
        const seconds = Math.max(10, Math.min(600, Math.round(liveBufferSeconds)));
        this.state.liveBufferSeconds.set(seconds);
        this.state.maxChartPoints.set(seconds * 60);
      }
      if (typeof raw['freezeOnStale'] === 'boolean') this.state.freezeOnStale.set(raw['freezeOnStale']);
      if (typeof raw['audioAlertsEnabled'] === 'boolean') this.state.audioAlertsEnabled.set(raw['audioAlertsEnabled']);
      if (typeof raw['voiceAlertsEnabled'] === 'boolean') this.state.voiceAlertsEnabled.set(raw['voiceAlertsEnabled']);
      const audioVolume = Number(raw['audioVolume']);
      if (Number.isFinite(audioVolume)) this.state.audioVolume.set(Math.max(0, Math.min(1, audioVolume)));
      if (typeof raw['autoDemoOnDisconnect'] === 'boolean') this.state.autoDemoOnDisconnect.set(raw['autoDemoOnDisconnect']);
      this.state.triggerHaptic('confirm');
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
      this.dynamicColor.setEnabled(false);
      this.dynamicColor.setSourceColor('#0061a4');
      
      this.state.triggerHaptic('destructiveAccept');
    }
  }

  protected readonly updateCheckBusy = computed(() => this.updateService.busy());
  protected readonly updateCheckResult = computed(() => this.updateService.result());
  protected readonly updateInstallDisabledReason = computed(() => this.updateService.installDisabledReason());
  protected readonly updateDownloadProgress = computed(() => {
    const state = this.updateService.state();
    return state.phase === 'downloading' ? state.progress : 0;
  });

  async checkForUpdates(): Promise<void> {
    await this.updateService.checkForUpdates(PRODUCT_VERSION);
  }

  async onInstallUpdate(): Promise<void> {
    await this.updateService.installAvailable();
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
