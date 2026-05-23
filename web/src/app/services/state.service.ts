import { Injectable, signal, effect, WritableSignal } from '@angular/core';

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
  // Global View & Navigation State
  currentView = signal<string>('live');
  activeTab = signal<string>('tab-overview');

  // Connection/Control states
  ctlOn = signal<boolean>(false);
  ctlStatus = signal<any>(null);
  ctlStopPending = signal<boolean>(false);
  paused = signal<boolean>(false);
  
  // Theme & Accessibility
  theme = signal<'light' | 'dark' | 'night' | 'hc'>('dark');
  density = signal<'comfortable' | 'compact'>('comfortable');
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
  setup = signal({
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
  lastPayload = signal<any>(null);
  lastLivePayload = signal<any>(null);
  snaps = signal<any[]>([]);
  snapNotes = signal<Record<string, string>>({});
  activeSnapNoteId = signal<string | null>(null);
  alertHistory = signal<any[]>([]);
  alertPins = signal<string[]>([]);
  sessionNotes = signal<Record<string, string>>({});
  sessionItems = signal<any[]>([]);
  currentSessionId = signal<string | null>(null);

  // Sparkline buffer for live KPIs
  spark = signal<{ hr: number[]; rr: number[]; fps: number[]; dist: number[] }>({
    hr: [],
    rr: [],
    fps: [],
    dist: []
  });

  // Haptic feedback mode
  hxMode = signal<'on' | 'off' | 'auto'>('auto');

  constructor() {
    this.loadFromStorage();

    // Effect to sync settings back to storage
    effect(() => {
      localStorage.setItem('rvt-theme', this.theme());
    });

    effect(() => {
      localStorage.setItem('rvt-density', this.density());
    });

    effect(() => {
      localStorage.setItem('rvt-zen-mode', this.zenMode() ? '1' : '0');
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
      localStorage.setItem('rvt-snaps', JSON.stringify(this.snaps()));
    });

    effect(() => {
      localStorage.setItem('rvt-snap-notes', JSON.stringify(this.snapNotes()));
    });

    effect(() => {
      localStorage.setItem('rvt-alert-pins', JSON.stringify(this.alertPins()));
    });

    effect(() => {
      localStorage.setItem('rvt-session-notes', JSON.stringify(this.sessionNotes()));
    });

    effect(() => {
      localStorage.setItem('rvt-kpi-thresholds', JSON.stringify(this.kpiThresholds()));
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
    });
  }

  private loadFromStorage() {
    try {
      // M6 Strict allowlist enum-like validation
      const themeVal = localStorage.getItem('rvt-theme');
      if (themeVal && ['light', 'dark', 'night', 'hc'].includes(themeVal)) {
        this.theme.set(themeVal as any);
      } else if (themeVal) {
        localStorage.removeItem('rvt-theme');
      }

      const densityVal = localStorage.getItem('rvt-density');
      if (densityVal && ['comfortable', 'compact'].includes(densityVal)) {
        this.density.set(densityVal as any);
      } else if (densityVal) {
        localStorage.removeItem('rvt-density');
      }

      const hxModeVal = localStorage.getItem('rvt-hx-mode');
      if (hxModeVal && ['on', 'off', 'auto'].includes(hxModeVal)) {
        this.hxMode.set(hxModeVal as any);
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

      const snapsVal = localStorage.getItem('rvt-snaps');
      if (snapsVal) this.snaps.set(JSON.parse(snapsVal));

      const snapNotesVal = localStorage.getItem('rvt-snap-notes');
      if (snapNotesVal) this.snapNotes.set(JSON.parse(snapNotesVal));

      const alertPinsVal = localStorage.getItem('rvt-alert-pins');
      if (alertPinsVal) this.alertPins.set(JSON.parse(alertPinsVal));

      const sessionNotesVal = localStorage.getItem('rvt-session-notes');
      if (sessionNotesVal) this.sessionNotes.set(JSON.parse(sessionNotesVal));

      const thresholdsVal = localStorage.getItem('rvt-kpi-thresholds');
      if (thresholdsVal) this.kpiThresholds.set(JSON.parse(thresholdsVal));
    } catch (e) {
      console.warn('Failed to load from localStorage, using defaults', e);
    }
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
