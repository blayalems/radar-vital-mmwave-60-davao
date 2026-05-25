import { ChangeDetectionStrategy, Component, inject, OnInit, OnDestroy, ElementRef, ViewChild, AfterViewInit } from '@angular/core';
import { CommonModule } from '@angular/common';
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
import { ApiService } from '../../services/api.service';
import { TelemetryService } from '../../services/telemetry.service';
import { AudioService } from '../../services/audio.service';
import { BluetoothService } from '../../services/bluetooth.service';
import { BleScanDevice, PreflightCheck, SerialPortRecord, SessionRecord, SubjectProfileRecord } from '../../models/rvt.models';

@Component({
  selector: 'app-home',
  imports: [
    CommonModule,
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
    MatSnackBarModule
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
  private readonly router = inject(Router);
  private readonly snackBar = inject(MatSnackBar);

  @ViewChild('radarCanvas', { static: false }) radarCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('trendCanvas', { static: false }) trendCanvas!: ElementRef<HTMLCanvasElement>;

  private animeFrameId: number | null = null;
  private canvasCtx: CanvasRenderingContext2D | null = null;
  private trendCtx: CanvasRenderingContext2D | null = null;

  // Local form model binding
  radarPorts: string[] = ['COM3', 'COM10', 'COM11', 'COM12', '/dev/ttyUSB0', '/dev/ttyUSB1'];
  bleDevices: BleScanDevice[] = [];
  subjectProfiles: Record<string, SubjectProfileRecord> = {};
  preflightChecks: PreflightCheck[] = [];
  preflightError = '';
  isScanningPorts = false;
  isScanningBle = false;
  isValidatingNativeBle = false;
  nativeBleProbeStatus = '';
  isPreflightRunning = false;
  selectedDuration = 30;

  sessionFilter: 'all' | 'pass' | 'warn' | 'fail' | 'tagged' = 'all';

  ngOnInit() {
    this.selectedDuration = this.state.setup().duration_s;
    this.refreshDefaults();
    this.loadSubjectProfiles();
    this.loadSessions();
    this.runPreflight();
  }

  ngAfterViewInit() {
    this.initCanvasDrawing();
  }

  ngOnDestroy() {
    if (this.animeFrameId) {
      cancelAnimationFrame(this.animeFrameId);
    }
  }

  async refreshDefaults() {
    try {
      const defs = await this.api.request<{ radar_port?: string; serial_ports?: string[] }>('/api/defaults');
      if (defs) {
        if (defs.radar_port) {
          this.state.setup.update(s => ({ ...s, radar_port: defs.radar_port ?? s.radar_port }));
        }
        if (defs.serial_ports && Array.isArray(defs.serial_ports)) {
          this.radarPorts = defs.serial_ports;
        }
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

  onFormChange() {
    this.runPreflight();
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

  async scanSerialPorts() {
    this.isScanningPorts = true;
    this.state.triggerHaptic('tap');
    try {
      const result = await this.api.request<{ ports?: SerialPortRecord[]; selected?: string }>('/api/serial/ports');
      const ports = (result.ports || []).map(port => port.device).filter(Boolean);
      if (ports.length) {
        this.radarPorts = ports;
        if (result.selected && ports.includes(result.selected)) {
          this.state.setup.update(setup => ({ ...setup, radar_port: result.selected! }));
        }
      }
      this.state.triggerHaptic('success');
    } catch (_) {
      this.state.triggerHaptic('warn');
    } finally {
      this.isScanningPorts = false;
    }
  }

  async scanBleDevices() {
    this.isScanningBle = true;
    this.state.triggerHaptic('tap');
    try {
      if (this.state.ctlStatus()?.mode !== 'sandbox') {
        const response = await this.api.request<{ devices?: BleScanDevice[] }>('/api/ble/scan?timeout_s=3');
        this.bleDevices = response.devices || [];
      } else if (this.bluetooth.isSupported() && !this.state.demoMode()) {
        const dev = await this.bluetooth.requestDevice();
        if (dev && dev.name) {
          this.state.setup.update(s => ({ ...s, ble_address: dev.id || dev.name || '' }));
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

  async runPreflight() {
    this.isPreflightRunning = true;
    this.preflightError = '';
    try {
      const query = new URLSearchParams({
        port: this.state.setup().radar_port,
        address: this.state.setup().ble_address
      });
      const resp = await this.api.request<{ checks?: PreflightCheck[] }>(`/api/preflight?${query.toString()}`);
      if (resp && Array.isArray(resp.checks)) {
        this.preflightChecks = resp.checks;
      }
    } catch (error: unknown) {
      this.preflightError = error instanceof Error ? error.message : 'Preflight unavailable.';
    } finally {
      this.isPreflightRunning = false;
    }
  }

  async runSingleCheck(checkId: string) {
    this.state.triggerHaptic('tap');
    try {
      const resp = await this.api.request<PreflightCheck | { check?: PreflightCheck }>(`/api/preflight/${checkId}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          port: this.state.setup().radar_port,
          address: this.state.setup().ble_address
        })
      });
      const check = 'check' in resp && resp.check ? resp.check : resp as PreflightCheck;
      if (check && check.id) {
        this.preflightChecks = this.preflightChecks.map(c => c.id === checkId ? check : c);
        if (check.status === 'good') {
          this.state.triggerHaptic('success');
        } else {
          this.state.triggerHaptic('warn');
        }
      }
    } catch (_) {
      this.state.triggerHaptic('reject');
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
    return this.preflightChecks.some(check => ['bad', 'fail', 'error'].includes(check.status.toLowerCase()));
  }

  canStartSession(): boolean {
    return !this.isPreflightRunning && this.preflightChecks.length > 0 && !this.hasBlockingPreflightFailure();
  }

  private checkPasses(check: PreflightCheck): boolean {
    return ['good', 'pass', 'ready', 'ok'].includes(check.status.toLowerCase());
  }

  setSessionFilter(filter: 'all' | 'pass' | 'warn' | 'fail' | 'tagged') {
    this.sessionFilter = filter;
    this.state.triggerHaptic('tap');
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

  async startSession() {
    this.state.triggerHaptic('sessionStart');
    try {
      await this.runPreflight();
      if (!this.canStartSession()) {
        this.snackBar.open('Start blocked: resolve failed preflight checks first.', 'Dismiss', { duration: 7000 });
        return;
      }
      const payload = {
        duration_s: this.selectedDuration,
        radar_port: this.state.setup().radar_port,
        ble_address: this.state.setup().ble_address,
        subject_label: this.state.setup().subject_label,
        operator_label: this.state.setup().operator_label,
        station_label: this.state.setup().station_label,
        subject_profile_id: this.state.setup().subject_profile_id,
        ble_profile: this.state.setup().ble_profile,
        skip_countdown: this.state.setup().skip_countdown,
        advanced: { notify_char: this.state.setup().notify_char }
      };
      
      const r = await this.api.request<SessionRecord>('/api/session/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });

      if (r && r.session_id) {
        this.state.currentSessionId.set(r.session_id);
        this.state.ctlOn.set(true);
        this.audio.speakAlert('Session started. Please sit still.', 'ok', true);
        this.router.navigate(['/live']);
      }
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : String(error);
      this.snackBar.open(`Could not start session: ${message}`, 'Dismiss', { duration: 7000 });
      this.state.triggerHaptic('reject');
    }
  }

  reviewSession(sessionId: string) {
    this.state.currentSessionId.set(sessionId);
    this.router.navigate(['/report']);
    this.state.triggerHaptic('tap');
  }

  // --- Premium Canvas Drawing Animations ---
  private initCanvasDrawing() {
    if (this.radarCanvas) {
      this.canvasCtx = this.radarCanvas.nativeElement.getContext('2d');
    }
    if (this.trendCanvas) {
      this.trendCtx = this.trendCanvas.nativeElement.getContext('2d');
    }

    const drawLoop = () => {
      this.animateRadarCanvas();
      this.animateTrendCanvas();
      this.animeFrameId = requestAnimationFrame(drawLoop);
    };
    this.animeFrameId = requestAnimationFrame(drawLoop);
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

    const themeColor = getComputedStyle(document.documentElement).getPropertyValue('--md-sys-color-primary') || '#0061a4';

    // Draw scanning circles
    ctx.strokeStyle = 'rgba(0, 97, 164, 0.15)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(cx, cy, radius * 0.66, 0, Math.PI * 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(cx, cy, radius * 0.33, 0, Math.PI * 2);
    ctx.stroke();

    // Radar scanning sweep sweep line
    const angle = (Date.now() / 1500) % (Math.PI * 2);
    ctx.strokeStyle = `rgba(0, 97, 164, 0.35)`;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + radius * Math.cos(angle), cy + radius * Math.sin(angle));
    ctx.stroke();

    // Dynamic wave ripples from telemetry heartbeat/breathing
    const payload = this.state.lastPayload();
    const hasData = payload && payload.radar && payload.radar.human;
    
    if (hasData) {
      const reportedHr = payload.radar.reported_hr || 70;
      const reportedRr = payload.radar.reported_rr || 15;
      const range = payload.radar.distance_cm || 50;

      // Pulse circle representing heart rate frequency
      const hrSpeed = (Date.now() / (60000 / reportedHr)) % 1;
      const hrPulseRadius = radius * 0.33 + (radius * 0.33) * Math.sin(hrSpeed * Math.PI);
      ctx.fillStyle = 'rgba(0, 164, 150, 0.15)';
      ctx.beginPath();
      ctx.arc(cx, cy, hrPulseRadius, 0, Math.PI * 2);
      ctx.fill();

      // Breathing ring ripple
      const rrSpeed = (Date.now() / (60000 / reportedRr)) % 1;
      const rrPulseRadius = radius * 0.66 + (radius * 0.25) * Math.sin(rrSpeed * Math.PI);
      ctx.strokeStyle = 'rgba(97, 105, 198, 0.45)';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(cx, cy, rrPulseRadius, 0, Math.PI * 2);
      ctx.stroke();

      // Text indicators
      ctx.fillStyle = '#64748b';
      ctx.font = '10px Roboto, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`Target: ${Math.round(range)} cm`, cx, cy + radius + 12);
    } else {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '11px Roboto, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Searching for subject...', cx, cy);
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

    if (!hrs.length) {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '11px Roboto, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Awaiting trend data...', w / 2, h / 2);
      return;
    }

    // Draw gridlines
    ctx.strokeStyle = '#f1f5f9';
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
    drawLine(hrs, 'rgba(0, 164, 150, 0.85)', 40, 160);
    // Plot RR using the secondary chart accent.
    drawLine(rrs, 'rgba(97, 105, 198, 0.85)', 5, 35);
  }
}
