import { Injectable, signal, effect, inject } from '@angular/core';
import {
  AlertEvent,
  AlertSeverity,
  ControlStatus,
  DensityId,
  HapticMode,
  LivePayload,
  SessionRecord,
  SetupState,
  SnapshotRecord,
  SessionSignoff,
  ThemeId
} from '../models/rvt.models';
import { PersistenceService, StorageScope } from './persistence.service';

export interface SubjectProfile {
  label: string;
  age_group: string;
  fitness_level: string;
  expected_hr_range: [number, number];
  notes: string;
}

export interface KpiThresholds {
  hrLow: number;
  hrHigh: number;
  rrLow: number;
  rrHigh: number;
}

export const DEFAULT_KPI_THRESHOLDS: KpiThresholds = {
  hrLow: 40,
  hrHigh: 140,
  rrLow: 6,
  rrHigh: 30
};

export const KPI_THRESHOLD_META = {
  hrLow: { min: 20, max: 80 },
  hrHigh: { min: 60, max: 220 },
  rrLow: { min: 2, max: 20 },
  rrHigh: { min: 10, max: 60 }
};

export const DEFAULT_SUBJECT_PROFILES: Record<string, SubjectProfile> = {
  adult_default: {
    label: 'Adult Default',
    age_group: 'adult',
    fitness_level: 'typical',
    expected_hr_range: [50, 120],
    notes: 'General adult research profile; narrows HR warnings for typical resting sessions.'
  },
  adult_athlete: {
    label: 'Adult Athlete',
    age_group: 'adult',
    fitness_level: 'athlete',
    expected_hr_range: [40, 140],
    notes: 'Allows wider resting HR variability for trained subjects; not a diagnostic profile.'
  }
};

@Injectable({
  providedIn: 'root'
})
export class StateService {
  private readonly persistence = inject(PersistenceService);
  private activeStorageScope: StorageScope | '' = '';
  private scopedHydrated = false;
  private hydrationGeneration = 0;
  // Global View & Navigation State
  currentView = signal<string>('live');
  activeTab = signal<string>('tab-overview');
  commandPaletteOpen = signal<boolean>(false);
  alertsOpen = signal<boolean>(false);

  // Connection/Control states
  ctlOn = signal<boolean>(false);
  ctlStatus = signal<ControlStatus | null>(null);
  ctlStopPending = signal<boolean>(false);
  paused = signal<boolean>(false);
  
  // Theme & Accessibility
  theme = signal<ThemeId>('dark');
  density = signal<DensityId>('comfortable');
  fontScale = signal<number>(1);
  zenMode = signal<boolean>(false);

  // Audio Alerts
  audioAlertsEnabled = signal<boolean>(false);
  audioVolume = signal<number>(0.7);
  voiceAlertsEnabled = signal<boolean>(false);

  // Buffer & Performance
  liveBufferSeconds = signal<number>(60);
  maxChartPoints = signal<number>(3600);

  // Demo & Stale Settings
  demoMode = signal<boolean>(false);
  autoDemoOnDisconnect = signal<boolean>(false);
  autoDemoActive = signal<boolean>(false);
  freezeOnStale = signal<boolean>(true);

  // KPI Thresholds & Profiles
  kpiThresholds = signal<KpiThresholds>({ ...DEFAULT_KPI_THRESHOLDS });
  activeSubjectProfileId = signal<string>('adult_default');

  // Setup Details
  setup = signal<SetupState>({
    duration_s: 30,
    customDuration: 30,
    customUnit: 's',
    radar_port: 'COM10',
    ble_address: '10:22:33:9E:8F:63',
    ble_profile: 'ailink_oximeter',
    notify_char: '0000ffe2-0000-1000-8000-00805f9b34fb',
    subject_label: '',
    operator_label: '',
    station_label: 'Lab · Station 3',
    subject_profile_id: 'adult_default',
    skip_countdown: false
  });

  // Data variables
  lastPayload = signal<LivePayload | null>(null);
  lastLivePayload = signal<LivePayload | null>(null);
  liveReceivedAt = signal<number | null>(null);
  telemetryStale = signal<boolean>(true);
  snaps = signal<SnapshotRecord[]>([]);
  snapNotes = signal<Record<string, string>>({});
  activeSnapNoteId = signal<string | null>(null);
  alertHistory = signal<AlertEvent[]>([]);
  alertPins = signal<string[]>([]);
  sessionNotes = signal<Record<string, string>>({});
  sessionSignoffs = signal<Record<string, SessionSignoff>>({});
  sessionItems = signal<SessionRecord[]>([]);
  currentSessionId = signal<string | null>(null);

  // Sparkline buffer for live KPIs
  spark = signal<{ hr: number[]; rr: number[]; fps: number[]; dist: number[] }>({
    hr: [],
    rr: [],
    fps: [],
    dist: []
  });

  // Haptic feedback mode
  hxMode = signal<HapticMode>('auto');
  waveformSeekAt = signal<number | null>(null);

  constructor() {
    this.loadFromStorage();
    void this.persistence.quarantineLegacyLocalStorage();

    effect(() => {
      const scope = this.storageScope();
      if (scope !== this.activeStorageScope) {
        this.activeStorageScope = scope;
        void this.hydrateScopedRecords(scope);
      }
    });

    // Effect to sync settings back to storage
    effect(() => {
      localStorage.setItem('rvt-theme', this.theme());
      document.documentElement.dataset['theme'] = this.theme();
    });

    effect(() => {
      localStorage.setItem('rvt-density', this.density());
      document.documentElement.dataset['density'] = this.density();
    });

    effect(() => {
      localStorage.setItem('rvt-font-scale', String(this.fontScale()));
      document.documentElement.style.setProperty('--rvt-font-scale', String(this.fontScale()));
    });

    effect(() => {
      localStorage.setItem('rvt-zen-mode', this.zenMode() ? '1' : '0');
      document.body.classList.toggle('zen-mode', this.zenMode());
    });

    effect(() => {
      localStorage.setItem('rvt-voice-alerts', this.voiceAlertsEnabled() ? '1' : '0');
    });

    effect(() => {
      localStorage.setItem('rvt-live-buffer-seconds', String(this.liveBufferSeconds()));
    });

    effect(() => {
      localStorage.setItem('rvt-max-chart-points', String(this.maxChartPoints()));
    });

    effect(() => {
      localStorage.setItem('rvt-subject-profile', this.activeSubjectProfileId());
    });

    effect(() => {
      localStorage.setItem('rvt-hx-mode', this.hxMode());
    });

    effect(() => {
      const value = this.snaps();
      if (this.scopedHydrated) void this.persistence.put('snapshots', this.activeStorageScope, value);
    });

    effect(() => {
      const value = this.snapNotes();
      if (this.scopedHydrated) void this.persistence.put('snap-notes', this.activeStorageScope, value);
    });

    effect(() => {
      const value = this.alertPins();
      if (this.scopedHydrated) void this.persistence.put('settings-backup', `${this.activeStorageScope}:alert-pins`, value);
    });

    effect(() => {
      const value = this.alertHistory();
      if (this.scopedHydrated) void this.persistence.put('alert-history', this.activeStorageScope, value);
    });

    effect(() => {
      const value = this.sessionNotes();
      if (this.scopedHydrated) void this.persistence.put('session-notes', this.activeStorageScope, value);
    });

    effect(() => {
      const value = this.sessionSignoffs();
      if (this.scopedHydrated) void this.persistence.put('session-notes', `${this.activeStorageScope}:signoffs`, value);
    });

    effect(() => {
      const value = this.sessionItems();
      if (this.scopedHydrated && this.activeStorageScope === 'demo') void this.persistence.put('sessions', 'demo', value);
    });

    effect(() => {
      localStorage.setItem('rvt-kpi-thresholds', JSON.stringify(this.kpiThresholds()));
    });

    effect(() => {
      localStorage.setItem('rvt-freeze-on-stale', this.freezeOnStale() ? '1' : '0');
    });

    effect(() => {
      localStorage.setItem('rvt-setup', JSON.stringify(this.setup()));
    });

    effect(() => {
      document.body.dataset['view'] = this.currentView();
    });

    // M1 Effect writers for user settings
    effect(() => {
      localStorage.setItem('rvt-audio-alerts', this.audioAlertsEnabled() ? '1' : '0');
    });

    effect(() => {
      localStorage.setItem('rvt-audio-volume', String(this.audioVolume()));
    });

    effect(() => {
      localStorage.setItem('rvt-demo-mode', this.demoMode() ? '1' : '0');
    });

    effect(() => {
      localStorage.setItem('rvt-auto-demo-on-disconnect', this.autoDemoOnDisconnect() ? '1' : '0');
      document.body.classList.toggle(
        'demo-mode',
        this.demoMode() || this.autoDemoActive() || this.ctlStatus()?.mode === 'sandbox'
      );
    });
  }

  private loadFromStorage() {
    try {
      // M6 Strict allowlist enum-like validation
      const themeVal = localStorage.getItem('rvt-theme');
      if (themeVal && ['light', 'dark', 'night', 'hc'].includes(themeVal)) {
        this.theme.set(themeVal as ThemeId);
      } else if (themeVal) {
        localStorage.removeItem('rvt-theme');
      }

      const densityVal = localStorage.getItem('rvt-density');
      if (densityVal && ['comfortable', 'compact'].includes(densityVal)) {
        this.density.set(densityVal as DensityId);
      } else if (densityVal) {
        localStorage.removeItem('rvt-density');
      }

      const hxModeVal = localStorage.getItem('rvt-hx-mode');
      if (hxModeVal && ['on', 'off', 'auto'].includes(hxModeVal)) {
        this.hxMode.set(hxModeVal as HapticMode);
      } else if (hxModeVal) {
        localStorage.removeItem('rvt-hx-mode');
      }

      this.zenMode.set(localStorage.getItem('rvt-zen-mode') === '1');
      this.voiceAlertsEnabled.set(localStorage.getItem('rvt-voice-alerts') === '1');

      const liveBuf = Number(localStorage.getItem('rvt-live-buffer-seconds'));
      if (liveBuf) this.liveBufferSeconds.set(liveBuf);

      const maxChart = Number(localStorage.getItem('rvt-max-chart-points'));
      if (maxChart) this.maxChartPoints.set(maxChart);

      const activeProf = localStorage.getItem('rvt-subject-profile');
      if (activeProf) this.activeSubjectProfileId.set(activeProf);

      // M1 loadFromStorage with sane clamping for volume
      this.audioAlertsEnabled.set(localStorage.getItem('rvt-audio-alerts') === '1');
      const storedVol = localStorage.getItem('rvt-audio-volume');
      if (storedVol !== null) {
        const val = Number(storedVol);
        if (Number.isFinite(val)) {
          this.audioVolume.set(Math.max(0.0, Math.min(1.0, val)));
        }
      }
      this.demoMode.set(localStorage.getItem('rvt-demo-mode') === '1');
      this.autoDemoOnDisconnect.set(localStorage.getItem('rvt-auto-demo-on-disconnect') === '1');
      const freezeVal = localStorage.getItem('rvt-freeze-on-stale');
      if (freezeVal !== null) this.freezeOnStale.set(freezeVal !== '0');

      const setupVal = localStorage.getItem('rvt-setup');
      if (setupVal) {
        const stored = JSON.parse(setupVal);
        if (stored && typeof stored === 'object') {
          this.setup.update(current => ({ ...current, ...stored }));
        }
      }

      const fontScaleVal = Number(localStorage.getItem('rvt-font-scale'));
      if (Number.isFinite(fontScaleVal) && fontScaleVal >= 0.9 && fontScaleVal <= 1.25) {
        this.fontScale.set(fontScaleVal);
      }

      const thresholdsVal = localStorage.getItem('rvt-kpi-thresholds');
      if (thresholdsVal) this.kpiThresholds.set(JSON.parse(thresholdsVal));
    } catch (e) {
      console.warn('Failed to load from localStorage, using defaults', e);
    }
  }

  storageScope(): StorageScope {
    return this.demoMode() || this.autoDemoActive() || this.ctlStatus()?.mode === 'sandbox'
      ? 'demo'
      : 'live';
  }

  private async hydrateScopedRecords(scope: StorageScope): Promise<void> {
    const generation = ++this.hydrationGeneration;
    this.scopedHydrated = false;
    const [snaps, snapNotes, alertPins, alerts, sessionNotes, signoffs, sessions] = await Promise.all([
      this.persistence.get<SnapshotRecord[]>('snapshots', scope),
      this.persistence.get<Record<string, string>>('snap-notes', scope),
      this.persistence.get<string[]>('settings-backup', `${scope}:alert-pins`),
      this.persistence.get<AlertEvent[]>('alert-history', scope),
      this.persistence.get<Record<string, string>>('session-notes', scope),
      this.persistence.get<Record<string, SessionSignoff>>('session-notes', `${scope}:signoffs`),
      scope === 'demo' ? this.persistence.get<SessionRecord[]>('sessions', 'demo') : Promise.resolve(undefined)
    ]);
    if (generation !== this.hydrationGeneration) return;
    this.snaps.set(Array.isArray(snaps) ? snaps : []);
    this.snapNotes.set(snapNotes || {});
    this.alertPins.set(Array.isArray(alertPins) ? alertPins : []);
    this.alertHistory.set(Array.isArray(alerts) ? alerts : []);
    this.sessionNotes.set(sessionNotes || {});
    this.sessionSignoffs.set(signoffs || {});
    if (scope === 'demo' && Array.isArray(sessions)) this.sessionItems.set(sessions);
    this.scopedHydrated = true;
  }

  pushAlert(message: string, severity: AlertSeverity = 'warn', source = 'telemetry', seekTimestamp?: number) {
    const now = Date.now();
    const latest = this.alertHistory()[0];
    if (latest?.msg === message && now - latest.ts < 15_000) return;
    this.alertHistory.update(items => [
      { id: `alert_${now}`, ts: now, msg: message, severity, source, seekTimestamp },
      ...items
    ].slice(0, 80));
  }

  toggleAlertPin(alertId: string) {
    this.alertPins.update(items => items.includes(alertId)
      ? items.filter(item => item !== alertId)
      : [...items, alertId]);
  }

  dismissAlert(alertId: string) {
    this.alertHistory.update(items => items.map(item => item.id === alertId
      ? { ...item, dismissed: true }
      : item));
    this.alertPins.update(items => items.filter(item => item !== alertId));
  }

  snoozeAlert(alertId: string, minutes = 10) {
    const until = Date.now() + minutes * 60_000;
    this.alertHistory.update(items => items.map(item => item.id === alertId
      ? { ...item, snoozedUntil: until }
      : item));
  }

  clearAlerts() {
    this.alertHistory.set([]);
    this.alertPins.set([]);
  }

  // Helper vibration logic
  triggerHaptic(kind: string) {
    if (this.hxMode() === 'off') return;
    if (typeof navigator === 'undefined' || !('vibrate' in navigator)) return;

    const reducedMotion = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    const sessionActive = this.ctlOn() && !this.ctlStopPending();

    // Skip haptics during session except critical ones (destructiveAccept added to whitelist)
    if (sessionActive && !['safety', 'confirm-stop', 'reject', 'destructiveAccept'].includes(kind)) return;
    if (reducedMotion && kind !== 'safety') return;

    try {
      switch (kind) {
        case 'tap':
          navigator.vibrate(10);
          break;
        case 'confirm':
          navigator.vibrate(15);
          break;
        case 'success':
          navigator.vibrate([12, 40, 12]);
          break;
        case 'warn':
          navigator.vibrate([8, 60, 8]);
          break;
        case 'safety':
        case 'reject':
          navigator.vibrate([20, 40, 20]);
          break;
        case 'destructiveAccept':
          navigator.vibrate([40, 80, 40]);
          break;
        case 'sessionStart':
          navigator.vibrate([18, 30, 18]);
          break;
        case 'sessionStop':
          navigator.vibrate(22);
          break;
        default:
          navigator.vibrate(10);
      }
    } catch (_) {}
  }
}
