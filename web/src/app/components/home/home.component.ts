import { ChangeDetectionStrategy, Component, inject, OnInit, OnDestroy, ElementRef, HostListener, ViewChild, AfterViewInit, effect } from '@angular/core';
import { KeyValuePipe, UpperCasePipe } from '@angular/common';
import { DurationPipe } from '../../pipes/duration.pipe';
import { FormsModule } from '@angular/forms';
import { Router, RouterModule } from '@angular/router';

// Angular Material 3 modules
import { MatCardModule } from '@angular/material/card';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatSelectModule } from '@angular/material/select';
import { MatCheckboxModule } from '@angular/material/checkbox';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { MatChipsModule } from '@angular/material/chips';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';

import { StateService } from '../../services/state.service';
import { ApiRequestError, ApiService } from '../../services/api.service';
import { TelemetryService } from '../../services/telemetry.service';
import { AudioService } from '../../services/audio.service';
import { BluetoothService } from '../../services/bluetooth.service';
import { InstallPromptService } from '../../services/install-prompt.service';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';
import { I18nService } from '../../services/i18n.service';
import { ChartRenderSchedulerService } from '../../services/chart-render-scheduler.service';
import {
  ReleaseCompatibilityService
} from '../../services/release-compatibility.service';
import type { ClientReleaseHandshake } from '../../services/release-compatibility.service';
import { TranslatePipe } from '../../i18n/translate.pipe';
import { BleScanDevice, normalizePreflightStatus, PreflightCheck, SerialPortRecord, SessionRecord, SubjectProfileRecord, SessionDataPayload, SetupState } from '../../models/rvt.models';
import {
  PreflightRequestCoordinator,
  PreflightSetup,
  preflightSetupFingerprint
} from './preflight-request-coordinator.service';
import {
  ParticipantStudySetupComponent,
  studySetupError,
  studyTrialId
} from '../participant-study-setup/participant-study-setup.component';
export { preflightSetupFingerprint } from './preflight-request-coordinator.service';

const FALLBACK_RADAR_PORT = 'COM10';
const DEFAULT_RADAR_PORT_CHOICES = ['COM7', FALLBACK_RADAR_PORT, 'COM3', 'COM4', 'COM11', 'COM12', '/dev/ttyUSB0', '/dev/ttyUSB1'];
const START_BLOCKING_PREFLIGHT_IDS = new Set([
  'serial_port_list',
  'session_folder_writable',
  'disk_space',
  'schema_hash_consistency',
  'clock_monotonic_sanity'
]);

interface SessionStartPayload {
  idempotency_key: string;
  duration_s: number;
  radar_port: string;
  ble_address: string;
  subject_label: string;
  operator_label: string;
  station_label: string;
  subject_profile_id: string;
  participant_id: string;
  trial_id: string;
  study_mode: 'confirmatory' | 'exploratory';
  study_classification: 'confirmatory' | 'exploratory';
  condition_id: string;
  distance_m: number;
  barrier_type: 'none' | 'cardboard';
  trial_number: number;
  planned_duration_s: number;
  ble_profile: string;
  skip_countdown: boolean;
  client_handshake: ClientReleaseHandshake;
  advanced: { notify_char: string };
}

interface SessionStartIntent {
  idempotencyKey: string;
  payloadFingerprint: string;
}

export function createSessionStartIdempotencyKey(): string {
  if (typeof globalThis.crypto?.randomUUID === 'function') {
    return globalThis.crypto.randomUUID();
  }
  const bytes = new Uint8Array(16);
  globalThis.crypto?.getRandomValues?.(bytes);
  if (!bytes.some(Boolean)) {
    const timestamp = Date.now();
    for (let index = 0; index < bytes.length; index++) {
      bytes[index] = (timestamp >>> ((index % 6) * 8)) & 0xff;
    }
  }
  bytes[6] = (bytes[6] & 0x0f) | 0x40;
  bytes[8] = (bytes[8] & 0x3f) | 0x80;
  const hex = [...bytes].map(value => value.toString(16).padStart(2, '0')).join('');
  return `${hex.slice(0, 8)}-${hex.slice(8, 12)}-${hex.slice(12, 16)}-${hex.slice(16, 20)}-${hex.slice(20)}`;
}

export function mergeRadarPortChoices(...groups: Array<Array<string | undefined> | undefined>): string[] {
  const seen = new Set<string>();
  const choices: string[] = [];
  for (const group of groups) {
    for (const value of group || []) {
      const port = String(value || '').trim();
      if (!port) continue;
      const key = port.toLowerCase();
      if (seen.has(key)) continue;
      seen.add(key);
      choices.push(port);
    }
  }
  return choices;
}

export function isStartBlockingPreflightCheck(check: PreflightCheck): boolean {
  return START_BLOCKING_PREFLIGHT_IDS.has(check.id) && ['bad', 'fail', 'error'].includes(normalizePreflightStatus(check));
}

export function sessionStartTimestampMs(session: Pick<SessionRecord, 'started_at' | 'started_ms'>): number | null {
  const direct = Number(session.started_ms);
  if (Number.isFinite(direct) && direct > 0) return direct;
  if (session.started_at) {
    const parsed = new Date(session.started_at).getTime();
    if (Number.isFinite(parsed) && parsed > 0) return parsed;
  }
  return null;
}

@Component({
  selector: 'app-home',
  imports: [
    DurationPipe,
    KeyValuePipe,
    UpperCasePipe,
    FormsModule,
    RouterModule,
    MatCardModule,
    MatButtonModule,
    MatIconModule,
    MatFormFieldModule,
    MatInputModule,
    MatSelectModule,
    MatCheckboxModule,
    MatProgressSpinnerModule,
    MatChipsModule,
    MatSnackBarModule,
    TranslatePipe,
    ParticipantStudySetupComponent
  ],
  templateUrl: './home.component.html',
  styleUrl: './home.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class HomeComponent implements OnInit, OnDestroy, AfterViewInit {
  protected readonly state = inject(StateService);
  protected readonly Math = Math;
  protected readonly api = inject(ApiService);
  protected readonly telemetry = inject(TelemetryService);
  protected readonly audio = inject(AudioService);
  protected readonly bluetooth = inject(BluetoothService);
  protected readonly installPrompt = inject(InstallPromptService);
  protected readonly serverLifecycle = inject(ServerLifecycleService);
  protected readonly i18n = inject(I18nService);
  protected readonly compatibility = inject(ReleaseCompatibilityService);
  protected participantRosterValid = false;
  private readonly router = inject(Router);
  private readonly snackBar = inject(MatSnackBar);
  private readonly renderScheduler = inject(ChartRenderSchedulerService);
  private readonly preflightRequests = inject(PreflightRequestCoordinator);

  @ViewChild('radarCanvas', { static: false }) radarCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('trendCanvas', { static: false }) trendCanvas!: ElementRef<HTMLCanvasElement>;

  private canvasCtx: CanvasRenderingContext2D | null = null;
  private trendCtx: CanvasRenderingContext2D | null = null;
  private resizeObserver: ResizeObserver | null = null;
  private viewReady = false;
  private readonly sparklineRenderOwner = {};
  private sparklineQueueTimer: ReturnType<typeof setTimeout> | null = null;
  private readonly onVisibilityChange = () => this.requestCanvasDraw();

  constructor() {
    effect(() => {
      this.state.lastPayload();
      this.state.spark();
      this.state.ctlStatus();
      this.state.theme(); // Redraw on theme change
      this.state.sessionItems();
      this.state.sessionNotes();
      this.requestCanvasDraw();
      this.scheduleSessionSparklines();
    });
  }

  // Local form model binding
  radarPorts: string[] = [...DEFAULT_RADAR_PORT_CHOICES];
  bleDevices: BleScanDevice[] = [];
  subjectProfiles: Record<string, SubjectProfileRecord> = {};
  preflightError = '';
  isScanningPorts = false;
  isScanningBle = false;
  bleScanAttempted = false;
  isValidatingNativeBle = false;
  nativeBleProbeStatus = '';
  isStartingSession = false;
  selectedDuration = 30;
  private startIntent: SessionStartIntent | null = null;
  private startInFlight: Promise<void> | null = null;

  sessionFilter: 'all' | 'pass' | 'warn' | 'fail' | 'tagged' = 'all';

  get preflightChecks(): PreflightCheck[] {
    return this.state.preflightChecks();
  }

  get isPreflightRunning(): boolean {
    return this.state.preflightRunning();
  }

  ngOnInit() {
    this.selectedDuration = this.state.setup().duration_s;
    void this.initializeHome();
  }

  private async initializeHome(): Promise<void> {
    await this.serverLifecycle.bootstrap();
    if (this.serverLifecycle.status() === 'offline' || this.serverLifecycle.status() === 'error') {
      // In demo/sandbox mode the Python server is intentionally absent; still
      // load the sandbox preflight, sessions and defaults so the readiness
      // surfaces populate. Only bail out when a live trainer is expected.
      if (!this.state.demoMode() && !this.state.autoDemoActive()) {
        this.snackBar.open('Python server is offline. Use Settings > Python Server to start, pair, or retry.', 'Dismiss', { duration: 7000 });
        return;
      }
    }
    await Promise.all([
      this.refreshDefaults(),
      this.loadSubjectProfiles(),
      this.loadSessions(),
      this.compatibility.refresh(this.state.ctlStatus())
    ]);
    await this.scanSerialPorts(false);
    await this.runPreflight();
  }

  protected async installApp(): Promise<void> {
    this.state.triggerHaptic('tap');
    const outcome = await this.installPrompt.promptInstall();
    if (outcome === 'accepted') {
      this.snackBar.open('Install started. Follow your browser prompt to finish.', 'Dismiss', { duration: 4000 });
    } else if (outcome === 'dismissed') {
      this.snackBar.open('Install prompt dismissed. You can still install from your browser menu.', 'Dismiss', { duration: 4000 });
    } else {
      this.snackBar.open('Install prompt is not available in this shell.', 'Dismiss', { duration: 4000 });
    }
  }

  protected dismissInstallBanner(): void {
    this.installPrompt.dismiss();
    this.state.triggerHaptic('tap');
  }

  @HostListener('document:keydown', ['$event'])
  onKeyboardShortcut(event: KeyboardEvent): void {
    if (event.defaultPrevented || this.state.currentView() !== 'home' || event.repeat) return;
    const target = event.target instanceof Element ? event.target : null;
    if (target?.closest('input, textarea, select, [contenteditable], [role="textbox"]')) return;
    if (event.ctrlKey || event.metaKey || event.altKey || event.shiftKey) return;
    const key = event.key.toLowerCase();
    if (key === 'q') {
      event.preventDefault();
      void this.runPreflight();
    } else if (key === 'n') {
      event.preventDefault();
      void this.startSession();
    }
  }

  ngAfterViewInit() {
    this.viewReady = true;
    if (this.radarCanvas) {
      this.canvasCtx = this.radarCanvas.nativeElement.getContext('2d');
    }
    if (this.trendCanvas) {
      this.trendCtx = this.trendCanvas.nativeElement.getContext('2d');
    }
    if (typeof ResizeObserver !== 'undefined') {
      this.resizeObserver = new ResizeObserver(() => this.requestCanvasDraw());
      if (this.radarCanvas) this.resizeObserver.observe(this.radarCanvas.nativeElement);
      if (this.trendCanvas) this.resizeObserver.observe(this.trendCanvas.nativeElement);
    }
    this.requestCanvasDraw();
  }

  ngOnDestroy() {
    this.renderScheduler.cancel(this);
    this.renderScheduler.cancel(this.sparklineRenderOwner);
    if (this.sparklineQueueTimer) clearTimeout(this.sparklineQueueTimer);
    this.resizeObserver?.disconnect();
  }

  async refreshDefaults() {
    try {
      const defs = await this.api.request<{ radar_port?: string; serial_ports?: string[] }>('/api/defaults');
      if (defs) {
        if (defs.radar_port) {
          this.state.setup.update(s => {
            const current = String(s.radar_port || '').trim();
            const preserveOperatorChoice = current && current.toUpperCase() !== FALLBACK_RADAR_PORT;
            return { ...s, radar_port: preserveOperatorChoice ? current : (defs.radar_port ?? current) };
          });
        }
        this.radarPorts = mergeRadarPortChoices(
          [this.state.setup().radar_port],
          Array.isArray(defs.serial_ports) ? defs.serial_ports : undefined,
          [defs.radar_port],
          DEFAULT_RADAR_PORT_CHOICES
        );
      }
    } catch (e) {
      console.warn('Could not load hardware defaults, using fallback lists', e);
    }
  }

  selectDuration(seconds: number) {
    this.selectedDuration = seconds;
    this.state.setup.update(s => ({ ...s, duration_s: seconds }));
    this.state.triggerHaptic('tap');
  }

  selectCustomDurationMinutes(value: unknown): void {
    const minutes = Math.max(1, Math.min(60, Math.round(Number(value) || 1)));
    const seconds = minutes * 60;
    this.selectedDuration = seconds;
    this.state.setup.update(s => ({
      ...s,
      duration_s: seconds,
      customDuration: minutes,
      customUnit: 'm'
    }));
    this.state.triggerHaptic('tap');
  }

  onFormChange() {
    void this.runPreflight();
  }

  updateSetup<K extends keyof ReturnType<StateService['setup']>>(key: K, value: ReturnType<StateService['setup']>[K]): void {
    this.state.setup.update(setup => ({ ...setup, [key]: value }));
  }

  async loadSubjectProfiles(): Promise<void> {
    try {
      const response = await this.api.request<{ profiles?: Record<string, SubjectProfileRecord> }>('/api/subject-profiles');
      this.subjectProfiles = response.profiles || {};
    } catch (_) {
      this.subjectProfiles = {};
    }
  }

  async loadSessions(): Promise<void> {
    try {
      const response = await this.api.request<{ items?: SessionRecord[] }>('/api/sessions');
      this.state.sessionItems.set(Array.isArray(response.items) ? response.items : []);
    } catch (_) {
      this.state.sessionItems.set([]);
    }
  }

  async scanSerialPorts(refreshPreflightAfterChange = true) {
    const setupBefore = this.currentPreflightFingerprint();
    this.isScanningPorts = true;
    this.state.triggerHaptic('tap');
    try {
      const result = await this.api.request<{ ports?: SerialPortRecord[]; selected?: string }>('/api/serial/ports');
      const ports = (result.ports || []).map(port => port.device).filter(Boolean);
      if (ports.length) {
        const current = String(this.state.setup().radar_port || '').trim();
        this.radarPorts = mergeRadarPortChoices([current], ports, [result.selected], DEFAULT_RADAR_PORT_CHOICES);
        if (current && ports.includes(current)) {
          // Keep the operator's explicit COM choice.
        } else if (ports.length === 1) {
          this.state.setup.update(setup => ({ ...setup, radar_port: ports[0] }));
        } else if (result.selected && ports.includes(result.selected)) {
          this.state.setup.update(setup => ({ ...setup, radar_port: result.selected! }));
        }
      }
      this.state.triggerHaptic('success');
    } catch (_) {
      this.state.triggerHaptic('warn');
    } finally {
      this.isScanningPorts = false;
      if (refreshPreflightAfterChange && setupBefore !== this.currentPreflightFingerprint()) {
        void this.runPreflight();
      }
    }
  }

  async scanBleDevices() {
    this.isScanningBle = true;
    this.bleScanAttempted = true;
    this.state.triggerHaptic('tap');
    try {
      if (this.serverLifecycle.platform() === 'exe' && this.bluetooth.isSupported()) {
        const dev = await this.bluetooth.requestDevice();
        this.bleDevices = [{
          id: dev.id,
          name: dev.name || 'Windows BLE device',
          address: String(dev.nativeHandle?.address || dev.id || '').trim()
        }];
      } else if (this.state.ctlStatus()?.mode !== 'sandbox') {
        const response = await this.api.request<{ devices?: BleScanDevice[] }>('/api/ble/scan?timeout_s=3');
        this.bleDevices = response.devices || [];
      } else if (this.bluetooth.isSupported() && !this.state.demoMode()) {
        const dev = await this.bluetooth.requestDevice();
        if (dev && dev.name) {
          const address = dev.id || dev.name || '';
          this.state.setup.update(s => ({ ...s, ble_address: address }));
          void this.runPreflight();
        }
      } else {
        this.bleDevices = [
          { name: 'Demo oximeter', id: this.state.setup().ble_address, address: this.state.setup().ble_address }
        ];
      }
      this.state.triggerHaptic('success');
    } catch (err) {
      console.warn(err);
      this.state.triggerHaptic('warn');
    } finally {
      this.isScanningBle = false;
    }
  }

  protected applyBleDevice(device: BleScanDevice): void {
    const address = this.bleDeviceAddress(device);
    if (!address) {
      this.snackBar.open('BLE scan result has no hardware address. Pick a device with an address.', 'Dismiss', { duration: 5000 });
      this.state.triggerHaptic('warn');
      return;
    }
    this.updateSetup('ble_address', address);
    this.onFormChange();
    this.bleDevices = [];
    this.snackBar.open(`BLE reference set to ${device.name || address}.`, 'Dismiss', { duration: 3000 });
  }

  protected bleDeviceAddress(device: BleScanDevice): string {
    return (device.address || '').trim();
  }

  protected bleSignalClass(rssi: number | undefined): string {
    if (!Number.isFinite(rssi)) return 'unknown';
    if ((rssi ?? -100) > -60) return 'strong';
    if ((rssi ?? -100) >= -80) return 'fair';
    return 'weak';
  }

  protected bleSignalLabel(rssi: number | undefined): string {
    if (!Number.isFinite(rssi)) return 'Signal unknown';
    if ((rssi ?? -100) > -60) return 'Strong signal';
    if ((rssi ?? -100) >= -80) return 'Fair signal';
    return 'Weak signal';
  }

  async validateNativeBleReference(): Promise<void> {
    this.isValidatingNativeBle = true;
    this.nativeBleProbeStatus = '';
    this.state.triggerHaptic('tap');
    try {
      const result = await this.bluetooth.validateReferenceNotification();
      const device = result.device.name || result.device.id;
      if (result.notificationBytes !== null) {
        this.nativeBleProbeStatus =
          `Native GATT verified: received ${result.notificationBytes} byte${result.notificationBytes === 1 ? '' : 's'} from ${device}. ` +
          'This probe validates local BLE only; trainer telemetry remains the session source.';
        this.snackBar.open('Native BLE reference notification received.', 'Dismiss', { duration: 5000 });
        this.state.triggerHaptic('success');
      } else {
        this.nativeBleProbeStatus =
          `Connected to ${device}, but no AiLink notification arrived within five seconds. ` +
          'Do not approve this client for live BLE validation yet.';
        this.snackBar.open('Native BLE connected without reference data.', 'Dismiss', { duration: 6000 });
        this.state.triggerHaptic('warn');
      }
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Native BLE validation failed.';
      this.nativeBleProbeStatus = `Native GATT validation failed: ${message}`;
      this.snackBar.open(this.nativeBleProbeStatus, 'Dismiss', { duration: 7000 });
      this.state.triggerHaptic('reject');
    } finally {
      this.isValidatingNativeBle = false;
    }
  }

  async runPreflight(): Promise<boolean> {
    const setup = this.capturePreflightSetup();
    this.state.preflightRunning.set(true);
    this.preflightError = '';
    let superseded = false;
    try {
      const result = await this.preflightRequests.runAll(setup, () => this.currentPreflightFingerprint());
      if (result.status === 'superseded') {
        superseded = true;
        return false;
      }
      if (result.status === 'stale') {
        this.preflightError = 'Setup changed while checks were running. Run preflight again for the current radar and BLE selection.';
        return false;
      }
      if (result.status === 'applied') {
        this.state.preflightChecks.set(result.checks);
        this.state.preflightUpdatedAtMs.set(Date.now());
        return true;
      }
      this.preflightError = result.status === 'empty'
        ? 'Preflight returned no hardware checks.'
        : result.message;
      return false;
    } finally {
      if (!superseded) this.state.preflightRunning.set(false);
    }
  }

  async runSingleCheck(checkId: string) {
    const setup = this.capturePreflightSetup();
    this.state.preflightRunning.set(true);
    this.preflightError = '';
    this.state.triggerHaptic('tap');
    let superseded = false;
    try {
      const result = await this.preflightRequests.runSingle(checkId, setup, () => this.currentPreflightFingerprint());
      if (result.status === 'superseded') {
        superseded = true;
        return;
      }
      if (result.status === 'stale') return;
      if (result.status === 'error') {
        this.preflightError = result.message;
        this.state.triggerHaptic('reject');
        return;
      }
      const check = result.check;
      if (check && check.id) {
        this.state.preflightChecks.set(this.preflightChecks.map(c => c.id === checkId ? check : c));
        this.state.preflightUpdatedAtMs.set(Date.now());
        if (check.status === 'good') {
          this.state.triggerHaptic('success');
        } else {
          this.state.triggerHaptic('warn');
        }
      }
    } finally {
      if (!superseded) this.state.preflightRunning.set(false);
    }
  }

  getChecksPassedCount(): number {
    return this.preflightChecks.filter(c => this.checkPasses(c)).length;
  }

  getReadinessPercentage(): number {
    if (!this.preflightChecks.length) return 0;
    const good = this.preflightChecks.filter(c => this.checkPasses(c)).length;
    return Math.round((good / this.preflightChecks.length) * 100);
  }

  hasBlockingPreflightFailure(): boolean {
    return this.preflightChecks.some(check => isStartBlockingPreflightCheck(check));
  }

  preflightProgressLabel(): string {
    if (this.isPreflightRunning) return this.preflightChecks.length ? 'Refreshing hardware checks...' : 'Running hardware checks...';
    if (this.preflightChecks.length && this.preflightRequests.lastValidFingerprint !== this.currentPreflightFingerprint()) {
      return 'Setup changed — run checks again';
    }
    const updatedAt = this.state.preflightUpdatedAtMs();
    return updatedAt ? `Last checked ${new Date(updatedAt).toLocaleTimeString(undefined, { hour: 'numeric', minute: '2-digit' })}` : 'Not checked yet';
  }

  protected isCheckStartBlocking(check: PreflightCheck): boolean {
    return isStartBlockingPreflightCheck(check);
  }

  protected preflightStatusIcon(check: PreflightCheck): string {
    if (this.checkPasses(check)) return 'check_circle';
    return this.isCheckStartBlocking(check) ? 'error' : 'warning';
  }

  canStartSession(): boolean {
    return this.canStartWithoutCompatibility()
      && this.compatibility.summary().state !== 'incompatible';
  }

  private canStartWithoutCompatibility(): boolean {
    return !this.isPreflightRunning
      && this.participantRosterValid
      && this.preflightRequests.lastValidFingerprint === this.currentPreflightFingerprint()
      && this.preflightChecks.length > 0
      && !this.hasBlockingPreflightFailure()
      && !this.studySetupError();
  }

  studySetupError(): string {
    return studySetupError({ ...this.state.setup(), duration_s: this.selectedDuration });
  }

  protected setParticipantRosterValidity(valid: boolean): void {
    this.participantRosterValid = valid;
    if (!valid) this.cancelStartIntent();
  }

  protected checkPasses(check: PreflightCheck): boolean {
    return ['good', 'pass', 'ready', 'ok'].includes(normalizePreflightStatus(check));
  }

  // Placement zones mirror the firmware's distance-confidence bands
  // (optimal 40–100 cm, good ≤140 cm, acceptable ≤180 cm) and Seeed's
  // ≤1.5 m chest-height guidance for the MR60BHA2.
  placementZone(): { id: string; label: string; hint: string } {
    const distance = Number(this.state.lastPayload()?.radar?.distance_cm);
    if (!Number.isFinite(distance) || distance <= 0) {
      return { id: 'searching', label: 'No target', hint: 'Seat the subject facing the radar at chest height.' };
    }
    if (distance < 40) {
      return { id: 'close', label: 'Too close', hint: 'Move back to 40–100 cm for the cleanest signal.' };
    }
    if (distance <= 100) {
      return { id: 'optimal', label: 'Sweet spot', hint: 'Hold this position for the session.' };
    }
    if (distance <= 140) {
      return { id: 'good', label: 'Good', hint: 'Slightly closer (40–100 cm) may improve heart-rate quality.' };
    }
    if (distance <= 180) {
      return { id: 'acceptable', label: 'Acceptable', hint: 'Best results within 1.5 m of the radar.' };
    }
    return { id: 'far', label: 'Out of range', hint: 'Bring the subject within 1.5 m of the radar.' };
  }

  setSessionFilter(filter: 'all' | 'pass' | 'warn' | 'fail' | 'tagged') {
    this.sessionFilter = filter;
    this.state.triggerHaptic('tap');
    this.scheduleSessionSparklines();
  }

  getFilteredSessions(): SessionRecord[] {
    const list = this.state.sessionItems();
    if (this.sessionFilter === 'all') return list;
    if (this.sessionFilter === 'tagged') {
      return list.filter(s => this.state.sessionNotes()[s.session_id]);
    }
    return list.filter(s => {
      const verd = this.sessionVerdict(s).toLowerCase();
      if (this.sessionFilter === 'pass') return verd === 'ready' || verd === 'pass';
      if (this.sessionFilter === 'warn') return verd === 'conditional' || verd === 'warn';
      if (this.sessionFilter === 'fail') return verd === 'unready' || verd === 'fail';
      return true;
    });
  }

  getGroupedSessions(): { dateLabel: string; items: SessionRecord[] }[] {
    const list = this.getFilteredSessions();
    const sorted = [...list].sort((a, b) => {
      const timeA = sessionStartTimestampMs(a) ?? -1;
      const timeB = sessionStartTimestampMs(b) ?? -1;
      return timeB - timeA;
    });

    const groups: { [key: string]: SessionRecord[] } = {};
    for (const session of sorted) {
      const startedMs = sessionStartTimestampMs(session);
      const dateLabel = startedMs
        ? new Date(startedMs).toLocaleDateString(undefined, { weekday: 'long', year: 'numeric', month: 'long', day: 'numeric' })
        : 'Undated sessions';
      if (!groups[dateLabel]) {
        groups[dateLabel] = [];
      }
      groups[dateLabel].push(session);
    }

    return Object.keys(groups).map(dateLabel => ({
      dateLabel,
      items: groups[dateLabel]
    }));
  }

  sessionStartTimeLabel(session: SessionRecord): string {
    const startedMs = sessionStartTimestampMs(session);
    return startedMs ? new Date(startedMs).toLocaleTimeString(undefined, { hour: 'numeric', minute: '2-digit' }) : '--';
  }

  sessionSubjectLabel(session: SessionRecord): string {
    return String(
      session.subject_label ||
      session.subject ||
      session['subject_profile_label'] ||
      session['subject_profile_id'] ||
      '--'
    );
  }

  sessionDuration(session: SessionRecord): string {
    const s = session.duration_s || 0;
    if (s <= 0) return '0s';
    const mins = Math.floor(s / 60);
    const secs = s % 60;
    return mins > 0 ? `${mins}m ${secs}s` : `${secs}s`;
  }

  private sparklineCache: Record<string, number[]> = {};

  private scheduleSessionSparklines(): void {
    if (this.sparklineQueueTimer) clearTimeout(this.sparklineQueueTimer);
    this.sparklineQueueTimer = setTimeout(() => {
      this.sparklineQueueTimer = null;
      this.renderScheduler.request(
        this.sparklineRenderOwner,
        () => void this.drawAllSparklines(),
        () => Array.from(document.querySelectorAll<HTMLCanvasElement>('canvas.session-micro-sparkline'))
          .some(canvas => this.renderScheduler.canvasVisible(canvas)),
        150
      );
    }, 150);
  }

  async drawAllSparklines(): Promise<void> {
    if (typeof document === 'undefined') return;
    const canvases = document.querySelectorAll('canvas.session-micro-sparkline') as NodeListOf<HTMLCanvasElement>;
    if (!canvases.length) return;

    for (const canvas of Array.from(canvases)) {
      const sessionId = canvas.getAttribute('data-session-id');
      if (!sessionId) continue;

      let points = this.sparklineCache[sessionId];
      if (!points) {
        try {
          const response = await this.api.request<SessionDataPayload>(`/api/sessions/${encodeURIComponent(sessionId)}/data?points=20`);
          const rows = response.rows || [];
          points = rows.map(r => Number(r['reported_hr'])).filter(Number.isFinite);
          this.sparklineCache[sessionId] = points;
        } catch (_) {
          points = [];
        }
      }

      this.drawMicroSparkline(canvas, points);
    }
  }

  private drawMicroSparkline(canvas: HTMLCanvasElement, points: number[]): void {
    const w = 60;
    const h = 24;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    canvas.style.width = `${w}px`;
    canvas.style.height = `${h}px`;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, w, h);

    if (points.length < 2) {
      // Draw a light dashed line signifying empty/no-data state truthfully
      ctx.strokeStyle = '#cbd5e1';
      ctx.lineWidth = 1.2;
      ctx.setLineDash([2, 2]);
      ctx.beginPath();
      ctx.moveTo(4, h / 2);
      ctx.lineTo(w - 4, h / 2);
      ctx.stroke();
      ctx.setLineDash([]);
      return;
    }

    const minV = Math.min(...points);
    const maxV = Math.max(...points);
    const diff = Math.max(1, maxV - minV);

    ctx.beginPath();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = '#00a496'; // Teal brand line

    points.forEach((val, idx) => {
      const x = (idx / (points.length - 1)) * w;
      const y = h - ((val - minV) / diff) * h;
      if (idx === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();
  }

  sessionVerdict(session: SessionRecord): string {
    if (typeof session.verdict === 'string') return session.verdict;
    if (session.verdict && typeof session.verdict === 'object') {
      return String(
        session.verdict['outcome'] ||
        session.verdict['verdict'] ||
        session.verdict['status'] ||
        '--'
      );
    }
    return '--';
  }

  startSession(): Promise<void> {
    if (this.startInFlight) return this.startInFlight;
    const pending = this.performStartSession();
    this.startInFlight = pending.finally(() => {
      this.startInFlight = null;
    });
    return this.startInFlight;
  }

  private async performStartSession(): Promise<void> {
    this.state.triggerHaptic('sessionStart');
    if (!this.participantRosterValid) {
      this.cancelStartIntent();
      this.snackBar.open(
        'Start blocked: refresh the participant roster and select an active coded participant.',
        'Dismiss',
        { duration: 7000 }
      );
      this.state.triggerHaptic('reject');
      return;
    }
    const setupError = this.studySetupError();
    if (setupError) {
      this.cancelStartIntent();
      this.snackBar.open(`Start blocked: ${setupError}`, 'Dismiss', { duration: 7000 });
      this.state.triggerHaptic('reject');
      return;
    }
    this.isStartingSession = true;
    const payloadFingerprint = this.sessionStartPayloadFingerprint();
    if (!this.startIntent || this.startIntent.payloadFingerprint !== payloadFingerprint) {
      this.startIntent = {
        idempotencyKey: createSessionStartIdempotencyKey(),
        payloadFingerprint
      };
    }
    const intent = this.startIntent;
    const payload = this.captureSessionStartPayload(intent.idempotencyKey);
    let postAttempted = false;
    try {
      const preflightReady = await this.runPreflight();
      if (payloadFingerprint !== this.sessionStartPayloadFingerprint()) {
        this.cancelStartIntent();
        this.snackBar.open('Start cancelled: setup changed while preflight was running. Review the current settings and try again.', 'Dismiss', { duration: 7000 });
        this.state.triggerHaptic('reject');
        return;
      }
      if (!preflightReady || !this.canStartWithoutCompatibility()) {
        this.cancelStartIntent();
        this.snackBar.open('Start blocked: resolve failed preflight checks first.', 'Dismiss', { duration: 7000 });
        return;
      }

      const compatibility = await this.compatibility.refresh(this.state.ctlStatus());
      if (compatibility.blocksStart) {
        this.cancelStartIntent();
        const actions = compatibility.guidance.join(' ');
        this.snackBar.open(`Start blocked: ${compatibility.message} ${actions}`, 'Dismiss', { duration: 12000 });
        this.state.triggerHaptic('reject');
        return;
      }
      if (compatibility.state === 'unverified') {
        this.snackBar.open(
          'Compatibility is unverified. Operational capture is allowed, but exclude this legacy session from confirmatory analysis.',
          'Dismiss',
          { duration: 9000 }
        );
        this.state.triggerHaptic('warn');
      }

      postAttempted = true;
      const r = await this.api.request<SessionRecord>('/api/session/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });

      if (r && r.session_id) {
        this.cancelStartIntent();
        this.state.currentSessionId.set(r.session_id);
        this.state.sessionActive.set(true);
        this.state.ctlOn.set(true);
        this.audio.speakAlert('Session started. Please sit still.', 'ok', true);
        this.router.navigate(['/live']);
      } else {
        throw new Error('Trainer response did not identify the session.');
      }
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : String(error);
      const definitiveHttpRejection = error instanceof ApiRequestError && error.status >= 400;
      if (!postAttempted || definitiveHttpRejection) this.cancelStartIntent();
      const retryGuidance = postAttempted && !definitiveHttpRejection
        ? ' Retry Start to safely resume the same request; repeated commands will use the same idempotency key.'
        : '';
      this.snackBar.open(`Could not start session: ${message}.${retryGuidance}`, 'Dismiss', { duration: 9000 });
      this.state.triggerHaptic('reject');
    } finally {
      this.isStartingSession = false;
    }
  }

  private cancelStartIntent(): void {
    this.startIntent = null;
  }

  private sessionStartPayloadFingerprint(): string {
    return JSON.stringify(this.captureSessionStartPayload(''));
  }

  private capturePreflightSetup(): PreflightSetup {
    const setup = this.state.setup();
    return {
      radar_port: setup.radar_port,
      ble_address: setup.ble_address
    };
  }

  private currentPreflightFingerprint(): string {
    return preflightSetupFingerprint(this.state.setup());
  }

  private captureSessionStartPayload(idempotencyKey: string): SessionStartPayload {
    const setup = this.state.setup();
    return {
      idempotency_key: idempotencyKey,
      duration_s: this.selectedDuration,
      radar_port: setup.radar_port,
      ble_address: setup.ble_address,
      subject_label: setup.subject_label,
      operator_label: setup.operator_label,
      station_label: setup.station_label,
      subject_profile_id: setup.subject_profile_id,
      participant_id: setup.participant_id,
      trial_id: studyTrialId(setup),
      study_mode: setup.study_mode,
      study_classification: setup.study_mode,
      condition_id: setup.condition_id,
      distance_m: setup.distance_m,
      barrier_type: setup.barrier_type,
      trial_number: setup.trial_number,
      planned_duration_s: this.selectedDuration,
      ble_profile: setup.ble_profile,
      skip_countdown: setup.skip_countdown,
      client_handshake: this.compatibility.handshake(),
      advanced: { notify_char: setup.notify_char }
    };
  }

  reviewSession(sessionId: string) {
    this.state.currentSessionId.set(sessionId);
    this.router.navigate(['/report']);
    this.state.triggerHaptic('tap');
  }

  // --- Premium Canvas Drawing Animations ---
  private requestCanvasDraw(): void {
    if (!this.viewReady) return;
    this.renderScheduler.request(
      this,
      () => {
        this.animateRadarCanvas();
        this.animateTrendCanvas();
      },
      () => [this.radarCanvas, this.trendCanvas]
        .some(ref => this.renderScheduler.canvasVisible(ref?.nativeElement)),
      100
    );
  }

  private animateRadarCanvas() {
    const ctx = this.canvasCtx;
    if (!ctx) return;

    const el = this.radarCanvas.nativeElement;
    const w = el.clientWidth;
    const h = el.clientHeight;
    
    // Support high-DPI scaling
    const dpr = window.devicePixelRatio || 1;
    if (el.width !== w * dpr || el.height !== h * dpr) {
      el.width = w * dpr;
      el.height = h * dpr;
    }
    ctx.resetTransform();
    ctx.scale(dpr, dpr);

    ctx.clearRect(0, 0, w, h);

    const cx = w / 2;
    const cy = h / 2;
    const radius = Math.min(cx, cy) - 24;

    // Fixed teal/green instrument palette — the scope container is dark in every
    // theme (see redesign mockup), so themed surface tokens would wash out.
    const ring = 'rgba(45, 212, 191, 0.16)';
    const sweepCore = 'rgba(52, 211, 153, 0.55)';
    const blipColor = '#34d399';
    const mutedText = 'rgba(148, 197, 190, 0.75)';
    const rootStyles = getComputedStyle(document.documentElement);
    const monoFont = rootStyles.getPropertyValue('--rvt-mono-font').trim() || 'monospace';
    const uiFont = rootStyles.getPropertyValue('--rvt-ui-font').trim() || 'sans-serif';

    // Concentric range rings + crosshairs.
    ctx.strokeStyle = ring;
    ctx.lineWidth = 1;
    for (const r of [radius, radius * 0.66, radius * 0.33]) {
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.stroke();
    }
    ctx.beginPath();
    ctx.moveTo(cx - radius, cy);
    ctx.lineTo(cx + radius, cy);
    ctx.moveTo(cx, cy - radius);
    ctx.lineTo(cx, cy + radius);
    ctx.stroke();

    // Rotating sweep: a trailing conic gradient sector + a bright leading edge.
    const angle = (Date.now() / 1500) % (Math.PI * 2);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(angle);
    const conic = typeof ctx.createConicGradient === 'function' ? ctx.createConicGradient(0, 0, 0) : null;
    if (conic) {
      conic.addColorStop(0, 'rgba(52, 211, 153, 0.28)');
      conic.addColorStop(0.13, 'rgba(52, 211, 153, 0)');
      conic.addColorStop(1, 'rgba(52, 211, 153, 0)');
      ctx.fillStyle = conic;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.arc(0, 0, radius, 0, Math.PI * 0.55);
      ctx.closePath();
      ctx.fill();
    }
    ctx.strokeStyle = sweepCore;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(radius, 0);
    ctx.stroke();
    ctx.restore();

    // Dynamic wave ripples + target blip from telemetry heartbeat/breathing.
    const payload = this.state.lastPayload();
    const hasData = payload && payload.radar && payload.radar.human;

    if (hasData) {
      const reportedHr = payload.radar.reported_hr || 70;
      const reportedRr = payload.radar.reported_rr || 15;
      const range = payload.radar.distance_cm || 50;

      const hrSpeed = (Date.now() / (60000 / reportedHr)) % 1;
      const hrPulseRadius = radius * 0.33 + (radius * 0.33) * Math.sin(hrSpeed * Math.PI);
      ctx.fillStyle = 'rgba(45, 212, 191, 0.12)';
      ctx.beginPath();
      ctx.arc(cx, cy, hrPulseRadius, 0, Math.PI * 2);
      ctx.fill();

      const rrSpeed = (Date.now() / (60000 / reportedRr)) % 1;
      const rrPulseRadius = radius * 0.66 + (radius * 0.25) * Math.sin(rrSpeed * Math.PI);
      ctx.strokeStyle = 'rgba(110, 231, 183, 0.40)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(cx, cy, rrPulseRadius, 0, Math.PI * 2);
      ctx.stroke();

      // Target blip positioned by range (clamped) along the sweep heading, with a glow.
      const norm = Math.max(0.12, Math.min(1, range / 210));
      const bx = cx + radius * norm * Math.cos(angle);
      const by = cy + radius * norm * Math.sin(angle);
      ctx.save();
      ctx.fillStyle = blipColor;
      ctx.shadowColor = blipColor;
      ctx.shadowBlur = 10;
      ctx.beginPath();
      ctx.arc(bx, by, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();

      ctx.fillStyle = mutedText;
      ctx.font = `600 10px ${monoFont}`;
      ctx.textAlign = 'center';
      ctx.fillText(`Target ${Math.round(range)} cm`, cx, cy + radius + 14);
    } else {
      ctx.fillStyle = mutedText;
      ctx.font = `11px ${uiFont}`;
      ctx.textAlign = 'center';
      ctx.fillText('Searching for subject…', cx, cy + radius + 14);
    }
  }

  private animateTrendCanvas() {
    const ctx = this.trendCtx;
    if (!ctx) return;

    const el = this.trendCanvas.nativeElement;
    const w = el.clientWidth;
    const h = el.clientHeight;

    const dpr = window.devicePixelRatio || 1;
    if (el.width !== w * dpr || el.height !== h * dpr) {
      el.width = w * dpr;
      el.height = h * dpr;
    }
    ctx.resetTransform();
    ctx.scale(dpr, dpr);

    ctx.clearRect(0, 0, w, h);

    const spark = this.state.spark();
    const hrs = spark.hr;
    const rrs = spark.rr;

    const styles = getComputedStyle(document.documentElement);
    const tertiaryColor = styles.getPropertyValue('--md-sys-color-tertiary').trim() || '#00a496';
    const secondaryColor = styles.getPropertyValue('--md-sys-color-secondary').trim() || '#6169c6';
    const outlineColor = styles.getPropertyValue('--md-sys-color-outline-variant').trim() || '#e2e8f0';
    const onSurface = styles.getPropertyValue('--md-sys-color-on-surface').trim() || '#94a3b8';

    const tertiaryRGB = this.parseColorToRgb(tertiaryColor, { r: 0, g: 164, b: 150 });
    const secondaryRGB = this.parseColorToRgb(secondaryColor, { r: 97, g: 105, b: 198 });

    if (!hrs.length) {
      ctx.fillStyle = onSurface;
      ctx.font = '11px Roboto, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Awaiting trend data...', w / 2, h / 2);
      return;
    }

    // Draw gridlines
    ctx.strokeStyle = outlineColor;
    ctx.lineWidth = 1;
    for (let i = 0; i <= 2; i++) {
      const y = 8 + (h - 16) * i / 2;
      ctx.beginPath();
      ctx.moveTo(8, y);
      ctx.lineTo(w - 8, y);
      ctx.stroke();
    }

    const maxItems = 20;
    const pad = 12;
    const plotWidth = w - pad * 2;
    const plotHeight = h - pad * 2;

    const drawLine = (data: number[], color: string, minVal: number, maxVal: number) => {
      const count = data.length;
      if (count < 2) return;

      const diff = Math.max(1, maxVal - minVal);

      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.beginPath();

      for (let i = 0; i < count; i++) {
        const val = data[i];
        const x = pad + (i / (maxItems - 1)) * plotWidth;
        const y = pad + plotHeight - ((val - minVal) / diff) * plotHeight;
        
        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      }
      ctx.stroke();
    };

    // Plot HR (green/teal) ranging roughly between 40 and 160
    drawLine(hrs, `rgba(${tertiaryRGB}, 0.85)`, 40, 160);
    // Plot RR using the secondary chart accent.
    drawLine(rrs, `rgba(${secondaryRGB}, 0.85)`, 5, 35);
  }

  private parseColorToRgb(color: string, fallback: { r: number; g: number; b: number }): string {
    const value = color.trim();
    if (value.startsWith('rgb')) {
      const matches = value.match(/\d+/g);
      if (matches && matches.length >= 3) {
        return `${matches[0]}, ${matches[1]}, ${matches[2]}`;
      }
    }
    let hex = value.replace('#', '');
    if (hex.length === 3) {
      hex = hex.split('').map(channel => channel + channel).join('');
    }
    const numeric = parseInt(hex, 16);
    if (Number.isNaN(numeric)) return `${fallback.r}, ${fallback.g}, ${fallback.b}`;
    return `${(numeric >> 16) & 255}, ${(numeric >> 8) & 255}, ${numeric & 255}`;
  }
}
