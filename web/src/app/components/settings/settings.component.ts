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
import { IssueReportService } from '../../services/issue-report.service';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';
import { UpdateService } from '../../services/update.service';
import { CONSENT_KEY } from '../../services/rvt-storage-keys';
import { GITHUB_REPO_URL, PRODUCT_VERSION, SCHEMA_VERSION_LABEL, TERMS_VERSION } from '../../services/app-meta';
import { BleScanDevice } from '../../models/rvt.models';
import { AboutCardComponent } from '../about-card/about-card.component';
import { ConfirmDialogComponent } from '../confirm-dialog/confirm-dialog.component';
import { ReportIssueCardComponent } from '../report-issue-card/report-issue-card.component';

const DEFAULT_STORAGE_QUOTA_BYTES = 820 * 1024 * 1024;
const LOCAL_DATA_DB_NAME = 'radar-vital-v12';

type SettingsGroupId =
  | 'connections'
  | 'display'
  | 'sound'
  | 'alerts'
  | 'telemetry'
  | 'privacy'
  | 'backup'
  | 'system';

const SETTINGS_GROUP_IDS: SettingsGroupId[] = [
  'connections',
  'display',
  'sound',
  'alerts',
  'telemetry',
  'privacy',
  'backup',
  'system'
];

const SETTINGS_GROUP_TERMS: Record<SettingsGroupId, string> = {
  connections: 'connections sources source mode demo auto disconnect trainer python server sidecar lan pairing pin qr ble bluetooth reference scanner freeze stale',
  display: 'display appearance palette color theme light dark night high contrast text scale font size density material you dynamic color source swatch',
  sound: 'sound haptics audio alerts voice announcements volume test vibration haptic feedback',
  alerts: 'alerts thresholds warning heart rate hr respiration rr low high bpm br min reset',
  telemetry: 'telemetry rendering live waveform window buffer chart points performance',
  privacy: 'privacy security lock idle auto lock terms consent telemetry diagnostics issue report terms privacy withdraw',
  backup: 'backup reset import export preferences portability defaults restore',
  system: 'system about data version schema update release diagnostics export cache storage legal license github report issue'
};

interface StorageUsageSummary {
  usageBytes: number;
  quotaBytes: number;
}

interface ConsentSummary {
  version: string;
  accepted_at: string;
}

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
    MatTooltipModule,
    AboutCardComponent,
    ReportIssueCardComponent
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
  protected readonly issueReport = inject(IssueReportService);
  protected readonly serverLifecycle = inject(ServerLifecycleService);
  protected readonly updateService = inject(UpdateService);
  private readonly router = inject(Router);
  protected readonly Math = Math;
  protected readonly productVersion = PRODUCT_VERSION;
  protected readonly schemaVersion = SCHEMA_VERSION_LABEL;
  protected readonly termsVersion = TERMS_VERSION;
  protected readonly termsUrl = `${GITHUB_REPO_URL}/blob/main/TERMS.md`;
  protected readonly privacyUrl = `${GITHUB_REPO_URL}/blob/main/PRIVACY.md`;
  protected readonly consentRecord = signal<ConsentSummary | null>(this.readConsentRecord());
  protected readonly consentAcceptedLabel = computed(() => {
    const record = this.consentRecord();
    if (!record?.accepted_at) return 'Not accepted in this browser';
    const date = new Date(record.accepted_at);
    return Number.isNaN(date.getTime()) ? record.accepted_at : date.toLocaleString();
  });
  
  // Optional injection of MatDialogRef for close behavior when loaded inside dialog
  private readonly dialogRef = inject(MatDialogRef<SettingsComponent>, { optional: true });
  private readonly dialog = inject(MatDialog);
  private readonly snackBar = inject(MatSnackBar);
  protected readonly settingsSearch = signal('');
  protected readonly dataActionBusy = signal(false);
  protected readonly storageUsage = signal<StorageUsageSummary>({
    usageBytes: this.measureLocalStorageBytes(),
    quotaBytes: DEFAULT_STORAGE_QUOTA_BYTES
  });
  protected readonly normalizedSettingsSearch = computed(() => this.settingsSearch().trim().toLowerCase());
  protected readonly visibleSettingsCardCount = computed(() => {
    const query = this.normalizedSettingsSearch();
    return SETTINGS_GROUP_IDS.filter(id => !query || SETTINGS_GROUP_TERMS[id].includes(query)).length;
  });
  protected readonly storageUsagePercent = computed(() => {
    const { usageBytes, quotaBytes } = this.storageUsage();
    if (!quotaBytes) return 0;
    return Math.max(0, Math.min(100, Math.round((usageBytes / quotaBytes) * 1000) / 10));
  });
  protected readonly storageUsageLabel = computed(() => {
    const { usageBytes, quotaBytes } = this.storageUsage();
    return `${this.formatBytes(usageBytes)} / ${this.formatBytes(quotaBytes)}`;
  });
  protected readonly endpointInput = signal(this.serverLifecycle.serverAddress());
  protected readonly pinInput = signal('');
  protected readonly connectionBusy = signal(false);
  protected readonly bleScanBusy = signal(false);
  protected readonly bleScanDevices = signal<BleScanDevice[]>([]);
  protected readonly bleScanMessage = signal('No scan run yet.');

  // Material 3 Expressive exploration palettes.
  protected readonly paletteOptions: {
    id: 'azure' | 'bloom' | 'mint';
    name: string;
    desc: string;
    dots: string[];
  }[] = [
    { id: 'azure', name: 'Azure Expressive', desc: 'Refined evolution of the current azure brand.', dots: ['#36618e', '#b62e63', '#0e7c72', '#e3edf8'] },
    { id: 'bloom', name: 'Pixel Bloom', desc: 'Dynamic violet, extra-round, most playful.', dots: ['#6750a4', '#c8424d', '#1d8a7a', '#e9e1f8'] },
    { id: 'mint', name: 'Clinical Mint', desc: 'Calm green, tighter radii, focused density.', dots: ['#1e6b52', '#b12f5d', '#21698c', '#dfeee5'] },
  ];

  selectPalette(id: 'azure' | 'bloom' | 'mint'): void {
    this.state.palette.set(id);
    this.state.triggerHaptic('tap');
  }

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

  constructor() {
    this.refreshStorageEstimate();
  }

  protected settingsCardMatches(id: SettingsGroupId): boolean {
    const query = this.normalizedSettingsSearch();
    return !query || SETTINGS_GROUP_TERMS[id].includes(query);
  }

  protected clearSettingsSearch(): void {
    this.settingsSearch.set('');
    this.state.triggerHaptic('tap');
  }

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
    const confirmed = await this.confirmServerRestart(
      checked ? 'Share trainer on the local network?' : 'Stop sharing on the local network?',
      checked
        ? 'The bundled trainer must restart in LAN mode. This interrupts any live session, closes the SSE stream, stops serial capture, and invalidates existing phone pairings. You will be asked to sign in again afterwards.'
        : 'The bundled trainer must restart in local mode. This interrupts any live session, closes the SSE stream, stops serial capture, and invalidates existing phone pairings. You will be asked to sign in again afterwards.',
      checked ? 'Restart and share' : 'Restart local only'
    );
    if (!confirmed) return;
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
    const confirmed = await this.confirmServerRestart(
      'Generate a new pairing PIN?',
      'The bundled trainer must restart to mint a new PIN. This interrupts any live session, closes the SSE stream, stops serial capture, and invalidates existing phone pairings. You will be asked to sign in again afterwards.',
      'Restart and mint PIN'
    );
    if (!confirmed) return;
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

  async copyPairingLink(): Promise<void> {
    const link = this.serverLifecycle.pairingUrl();
    if (!link) return;
    try {
      await navigator.clipboard.writeText(link);
      this.snackBar.open('Pairing link copied.', 'Dismiss', { duration: 3000 });
    } catch (_) {
      this.snackBar.open('Copy failed; select and copy the pairing link manually.', 'Dismiss', { duration: 5000 });
    }
  }

  private async confirmServerRestart(title: string, message: string, confirmLabel: string): Promise<boolean> {
    return this.confirmSettingsAction(title, message, confirmLabel);
  }

  private async confirmSettingsAction(title: string, message: string, confirmLabel: string): Promise<boolean> {
    return Boolean(await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
      data: { title, message, confirmLabel },
      restoreFocus: true,
      panelClass: 'm3-dialog-panel',
      backdropClass: 'rvt-modal-blur-backdrop',
      ariaLabel: title
    }).afterClosed()));
  }

  private readConsentRecord(): ConsentSummary | null {
    try {
      const raw = localStorage.getItem(CONSENT_KEY);
      if (!raw) return null;
      const parsed = JSON.parse(raw) as Partial<ConsentSummary>;
      if (typeof parsed.version !== 'string' || typeof parsed.accepted_at !== 'string') return null;
      return { version: parsed.version, accepted_at: parsed.accepted_at };
    } catch (_) {
      return null;
    }
  }

  scrollToIssueDiagnostics(): void {
    document.getElementById('report-issue-card')?.scrollIntoView({ block: 'center', behavior: 'smooth' });
    this.state.triggerHaptic('tap');
  }

  async withdrawConsent(): Promise<void> {
    const confirmed = await this.confirmSettingsAction(
      'Withdraw terms consent?',
      'Radar Vital will reload and show the consent gate again before the dashboard can be used.',
      'Withdraw consent'
    );
    if (!confirmed) return;
    try {
      localStorage.removeItem(CONSENT_KEY);
    } catch (_) {
      // Reload still forces the app-level gate to use the in-memory missing-consent state.
    }
    this.consentRecord.set(null);
    this.state.triggerHaptic('destructiveAccept');
    this.snackBar.open('Consent withdrawn. Reloading consent gate...', 'Dismiss', { duration: 1200 });
    window.setTimeout(() => window.location.reload(), 350);
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
      palette: this.state.palette(),
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

  async exportDiagnostics(): Promise<void> {
    this.dataActionBusy.set(true);
    try {
      const report = await this.issueReport.buildReport();
      this.downloadJson('radar-vital-diagnostics.json', {
        exported_at: new Date().toISOString(),
        diagnostics_enabled: this.issueReport.diagnosticsEnabled(),
        storage: this.storageUsage(),
        report
      });
      this.state.triggerHaptic('confirm');
      this.snackBar.open('Diagnostics exported.', 'Dismiss', { duration: 4000 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Diagnostics export failed.', 'Dismiss', { duration: 5000 });
      this.state.triggerHaptic('warn');
    } finally {
      this.dataActionBusy.set(false);
    }
  }

  async clearLocalCache(): Promise<void> {
    const confirmed = await this.confirmSettingsAction(
      'Clear local cache?',
      'Cached sessions, snapshots, notes, waveform buffers, and alert history stored in this browser will be cleared. Pairing, consent, and dashboard preferences stay unchanged.',
      'Clear cache'
    );
    if (!confirmed) return;

    this.dataActionBusy.set(true);
    try {
      await this.clearIndexedDbStores();
      this.clearLegacyCacheKeys();
      this.state.snaps.set([]);
      this.state.snapNotes.set({});
      this.state.alertPins.set([]);
      this.state.alertHistory.set([]);
      this.state.sessionNotes.set({});
      this.state.sessionSignoffs.set({});
      this.state.sessionItems.set([]);
      this.refreshStorageEstimate();
      this.state.triggerHaptic('destructiveAccept');
      this.snackBar.open('Local cache cleared. Preferences and pairing are unchanged.', 'Dismiss', { duration: 5000 });
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Local cache could not be cleared.', 'Dismiss', { duration: 5000 });
      this.state.triggerHaptic('warn');
    } finally {
      this.dataActionBusy.set(false);
    }
  }

  async importPreferences(event: Event): Promise<void> {
    const file = (event.target as HTMLInputElement).files?.[0];
    if (!file) return;
    try {
      const raw = JSON.parse(await file.text()) as Record<string, unknown>;
      if (['light', 'dark', 'night', 'hc'].includes(String(raw['theme']))) {
        this.state.theme.set(raw['theme'] as 'light' | 'dark' | 'night' | 'hc');
      }
      if (['azure', 'bloom', 'mint'].includes(String(raw['palette']))) {
        this.state.palette.set(raw['palette'] as 'azure' | 'bloom' | 'mint');
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
    const confirmed = await this.confirmSettingsAction(
      'Reset dashboard defaults?',
      'Theme, alert, telemetry and source-mode preferences will be restored to defaults.',
      'Reset defaults'
    );
    if (confirmed) {
      this.state.theme.set('dark');
      this.state.palette.set('azure');
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

  private refreshStorageEstimate(): void {
    const fallbackUsage = this.measureLocalStorageBytes();
    this.storageUsage.set({
      usageBytes: fallbackUsage,
      quotaBytes: DEFAULT_STORAGE_QUOTA_BYTES
    });

    if (typeof navigator === 'undefined' || !navigator.storage?.estimate) return;
    void navigator.storage.estimate().then(estimate => {
      this.storageUsage.set({
        usageBytes: Math.max(fallbackUsage, Math.round(estimate.usage ?? 0)),
        quotaBytes: Math.round(estimate.quota ?? DEFAULT_STORAGE_QUOTA_BYTES)
      });
    }).catch(() => {
      this.storageUsage.set({
        usageBytes: fallbackUsage,
        quotaBytes: DEFAULT_STORAGE_QUOTA_BYTES
      });
    });
  }

  private measureLocalStorageBytes(): number {
    try {
      let total = 0;
      for (let i = 0; i < localStorage.length; i += 1) {
        const key = localStorage.key(i) || '';
        const value = localStorage.getItem(key) || '';
        total += (key.length + value.length) * 2;
      }
      return total;
    } catch (_) {
      return 0;
    }
  }

  private formatBytes(bytes: number): string {
    if (!Number.isFinite(bytes) || bytes <= 0) return '0 B';
    const units = ['B', 'KB', 'MB', 'GB'];
    let value = bytes;
    let unitIndex = 0;
    while (value >= 1024 && unitIndex < units.length - 1) {
      value /= 1024;
      unitIndex += 1;
    }
    const precision = value >= 10 || unitIndex === 0 ? 0 : 1;
    return `${value.toFixed(precision)} ${units[unitIndex]}`;
  }

  private downloadJson(filename: string, data: unknown): void {
    const href = URL.createObjectURL(new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' }));
    const anchor = document.createElement('a');
    anchor.href = href;
    anchor.download = filename;
    anchor.click();
    URL.revokeObjectURL(href);
  }

  private clearLegacyCacheKeys(): void {
    const keys = [
      'rvt-snaps',
      'rvt-snap-notes',
      'rvt-session-notes',
      'rvt-alert-history',
      'rvt-alert-pins',
      'rvt-sandbox-sessions'
    ];
    try {
      keys.forEach(key => localStorage.removeItem(key));
    } catch (_) {
      // Best effort only; IndexedDB is the primary cache store now.
    }
  }

  private clearIndexedDbStores(): Promise<void> {
    if (typeof indexedDB === 'undefined') return Promise.resolve();
    return new Promise((resolve, reject) => {
      const request = indexedDB.open(LOCAL_DATA_DB_NAME);
      request.onerror = () => reject(request.error || new Error('Could not open local cache.'));
      request.onsuccess = () => {
        const db = request.result;
        const stores = Array.from(db.objectStoreNames);
        if (!stores.length) {
          db.close();
          resolve();
          return;
        }
        const transaction = db.transaction(stores, 'readwrite');
        stores.forEach(store => transaction.objectStore(store).clear());
        transaction.oncomplete = () => {
          db.close();
          resolve();
        };
        transaction.onerror = () => {
          db.close();
          reject(transaction.error || new Error('Could not clear local cache.'));
        };
      };
    });
  }
}
