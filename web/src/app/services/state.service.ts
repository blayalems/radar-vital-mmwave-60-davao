import { Injectable, effect, inject } from '@angular/core';
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
  ThemeId,
  StorageScope
} from '../models/rvt.models';
import { PersistenceService } from './persistence.service';
import { DynamicColorService } from './dynamic-color.service';
import { UiStore } from './stores/ui.store';
import { SessionStore } from './stores/session.store';
import { AlertStore } from './stores/alert.store';
import { TelemetryStore } from './stores/telemetry.store';

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
  private readonly dynamicColor = inject(DynamicColorService);
  private readonly uiStore = inject(UiStore);
  private readonly sessionStore = inject(SessionStore);
  private readonly alertStore = inject(AlertStore);
  private readonly telemetryStore = inject(TelemetryStore);

  private activeStorageScope: StorageScope | '' = '';
  private scopedHydrated = false;
  private hydrationGeneration = 0;

  // Global View & Navigation State
  currentView = this.uiStore.currentView;
  activeTab = this.uiStore.activeTab;
  commandPaletteOpen = this.uiStore.commandPaletteOpen;
  alertsOpen = this.uiStore.alertsOpen;

  // Connection/Control states
  ctlOn = this.sessionStore.ctlOn;
  ctlStatus = this.sessionStore.ctlStatus;
  ctlStopPending = this.sessionStore.ctlStopPending;
  paused = this.sessionStore.paused;
  
  // Theme & Accessibility
  theme = this.uiStore.theme;
  density = this.uiStore.density;
  fontScale = this.uiStore.fontScale;
  zenMode = this.uiStore.zenMode;

  // Audio Alerts
  audioAlertsEnabled = this.alertStore.audioAlertsEnabled;
  audioVolume = this.alertStore.audioVolume;
  voiceAlertsEnabled = this.alertStore.voiceAlertsEnabled;

  // Buffer & Performance
  liveBufferSeconds = this.telemetryStore.liveBufferSeconds;
  maxChartPoints = this.telemetryStore.maxChartPoints;

  // Demo & Stale Settings
  demoMode = this.uiStore.demoMode;
  autoDemoOnDisconnect = this.uiStore.autoDemoOnDisconnect;
  autoDemoActive = this.uiStore.autoDemoActive;
  freezeOnStale = this.alertStore.freezeOnStale;

  // KPI Thresholds & Profiles
  kpiThresholds = this.alertStore.kpiThresholds;
  activeSubjectProfileId = this.sessionStore.activeSubjectProfileId;

  // Setup Details
  setup = this.sessionStore.setup;

  // Data variables
  lastPayload = this.telemetryStore.lastPayload;
  lastLivePayload = this.telemetryStore.lastLivePayload;
  liveReceivedAt = this.telemetryStore.liveReceivedAt;
  telemetryStale = this.telemetryStore.telemetryStale;
  snaps = this.telemetryStore.snaps;
  snapNotes = this.telemetryStore.snapNotes;
  activeSnapNoteId = this.telemetryStore.activeSnapNoteId;
  alertHistory = this.alertStore.alertHistory;
  alertPins = this.alertStore.alertPins;
  sessionNotes = this.sessionStore.sessionNotes;
  sessionSignoffs = this.sessionStore.sessionSignoffs;
  sessionItems = this.sessionStore.sessionItems;
  currentSessionId = this.sessionStore.currentSessionId;
  sessionActive = this.sessionStore.sessionActive;

  // Sparkline buffer for live KPIs
  spark = this.telemetryStore.spark;

  // Haptic feedback mode
  hxMode = this.uiStore.hxMode;
  waveformSeekAt = this.telemetryStore.waveformSeekAt;

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
      // Re-apply dynamic color when base theme changes (light ↔ dark)
      this.dynamicColor.reapply();
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
    this.alertStore.pushAlert(message, severity, source, seekTimestamp);
  }

  toggleAlertPin(alertId: string) {
    this.alertStore.toggleAlertPin(alertId);
  }

  dismissAlert(alertId: string) {
    this.alertStore.dismissAlert(alertId);
  }

  snoozeAlert(alertId: string, minutes = 10) {
    this.alertStore.snoozeAlert(alertId, minutes);
  }

  clearAlerts() {
    this.alertStore.clearAlerts();
  }

  triggerHaptic(kind: string) {
    this.uiStore.triggerHaptic(kind, this.sessionActive(), this.ctlStopPending());
  }
}
