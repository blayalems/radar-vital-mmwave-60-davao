import { ChangeDetectionStrategy, Component, DestroyRef, computed, inject, signal } from '@angular/core';
import { Router, RouterModule } from '@angular/router';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MatButtonModule } from '@angular/material/button';
import { MatBadgeModule } from '@angular/material/badge';
import { MatIconModule } from '@angular/material/icon';
import { MatChipsModule } from '@angular/material/chips';
import { MatMenuModule } from '@angular/material/menu';
import { MatToolbarModule } from '@angular/material/toolbar';

import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';
import { TelemetryService } from '../../services/telemetry.service';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';
import { SettingsComponent } from '../settings/settings.component';
import { AlertsDialogComponent } from '../alerts-dialog/alerts-dialog.component';
import { CommandPaletteComponent } from '../command-palette/command-palette.component';
import { AuthService } from '../../services/auth.service';
import { SwitchOperatorDialogComponent } from '../switch-operator-dialog/switch-operator-dialog.component';
import { I18nService } from '../../services/i18n.service';
import { TranslatePipe } from '../../i18n/translate.pipe';

@Component({
  selector: 'app-topbar',
  imports: [
    RouterModule,
    MatDialogModule,
    MatButtonModule,
    MatBadgeModule,
    MatIconModule,
    MatChipsModule,
    MatMenuModule,
    MatToolbarModule,
    TranslatePipe
  ],
  templateUrl: './topbar.component.html',
  styleUrl: './topbar.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class TopbarComponent {
  protected readonly state = inject(StateService);
  protected readonly serverLifecycle = inject(ServerLifecycleService);
  protected readonly auth = inject(AuthService);
  protected readonly i18n = inject(I18nService);
  protected readonly mobileActionsOpen = signal(false);
  private readonly api = inject(ApiService);
  private readonly telemetry = inject(TelemetryService);
  private readonly dialog = inject(MatDialog);
  private readonly router = inject(Router);
  private readonly destroyRef = inject(DestroyRef);

  // 1 Hz tick driving the auto-retry countdown; runs only while disconnected.
  private readonly nowMs = signal(Date.now());
  private readonly countdownTimer = typeof window !== 'undefined'
    ? window.setInterval(() => this.nowMs.set(Date.now()), 1000)
    : null;
  protected readonly retryCountdownS = computed<number | null>(() => {
    if (this.state.ctlStatus()?.ok !== false) return null;
    const at = this.telemetry.nextRetryAtMs();
    if (!at) return null;
    const seconds = Math.ceil((at - this.nowMs()) / 1000);
    return seconds > 0 ? seconds : null;
  });
  protected readonly themeIcon = computed(() => {
    switch (this.state.theme()) {
      case 'light': return 'light_mode';
      case 'dark': return 'dark_mode';
      case 'night': return 'bedtime';
      case 'hc': return 'contrast';
      default: return 'dark_mode';
    }
  });
  protected readonly themeLabel = computed(() => {
    switch (this.state.theme()) {
      case 'light': return 'Theme: Light';
      case 'dark': return 'Theme: Dark';
      case 'night': return 'Theme: Night';
      case 'hc': return 'Theme: High contrast';
      default: return 'Cycle theme';
    }
  });
  protected readonly isDemoStatus = computed(() =>
    this.state.demoMode() || this.state.autoDemoActive() || this.state.ctlStatus()?.mode === 'sandbox'
  );
  protected readonly statusIcon = computed(() => {
    if (this.isDemoStatus()) return '';
    if (this.state.ctlStatus()?.ok === false) return 'cloud_off';
    if (this.state.paused()) return 'pause_circle';
    return 'sensors';
  });
  protected readonly statusLabel = computed(() => {
    if (this.isDemoStatus()) return 'Demo';
    if (this.state.ctlStatus()?.ok === false) return 'Disconnected';
    if (this.state.paused()) return 'Paused';
    return 'Monitoring';
  });
  protected readonly statusSub = computed(() => {
    if (this.isDemoStatus()) return 'simulated';
    if (this.state.ctlStatus()?.ok === false) {
      const seconds = this.retryCountdownS();
      return seconds ? `retry in ${seconds}s` : 'offline';
    }
    const latency = this.state.ctlStatus()?.latency;
    return latency ? `${latency} ms` : 'trainer';
  });
  protected readonly statusAriaLabel = computed(() => `${this.statusLabel()}, ${this.statusSub()}`);

  constructor() {
    this.destroyRef.onDestroy(() => {
      if (this.countdownTimer !== null) window.clearInterval(this.countdownTimer);
    });
  }

  toggleMobileActions(): void {
    this.mobileActionsOpen.update(open => !open);
    this.state.triggerHaptic('tap');
  }

  closeMobileActions(): void {
    this.mobileActionsOpen.set(false);
  }

  runMobileAction(action: () => void): void {
    this.closeMobileActions();
    action();
  }

  lockProfile(): void {
    this.auth.lock();
    this.state.triggerHaptic('tap');
  }

  switchOperator(): void {
    this.dialog.open(SwitchOperatorDialogComponent, {
      width: '440px',
      maxWidth: '90vw',
      panelClass: 'm3-dialog-panel'
    });
    this.state.triggerHaptic('tap');
  }

  togglePause() {
    this.state.paused.update(p => !p);
    this.state.triggerHaptic('tap');
  }

  toggleTheme() {
    const cycle: ('light' | 'dark' | 'night' | 'hc')[] = ['light', 'dark', 'night', 'hc'];
    const current = this.state.theme();
    const nextIdx = (cycle.indexOf(current) + 1) % cycle.length;
    this.state.theme.set(cycle[nextIdx]);
    this.state.triggerHaptic('tap');
  }

  openSettings() {
    // Open in dialog if window is wide, otherwise navigate
    if (window.innerWidth > 760) {
      this.dialog.open(SettingsComponent, {
        width: '640px',
        maxWidth: '90vw',
        panelClass: 'm3-dialog-panel'
      });
    } else {
      this.router.navigate(['/settings']);
    }
    this.state.triggerHaptic('tap');
  }

  openServerSettings(): void {
    if (this.serverLifecycle.status() === 'running' || this.serverLifecycle.status() === 'starting') return;
    this.openSettings();
  }

  openAlerts() {
    this.dialog.open(AlertsDialogComponent, {
      maxWidth: 'calc(100vw - 24px)',
      restoreFocus: true,
      panelClass: 'm3-dialog-panel'
    });
    this.state.triggerHaptic('tap');
  }

  openPalette() {
    this.dialog.open(CommandPaletteComponent, {
      autoFocus: 'input',
      restoreFocus: true,
      maxWidth: 'calc(100vw - 24px)',
      panelClass: 'm3-dialog-panel',
      backdropClass: 'rvt-palette-backdrop'
    });
    this.state.triggerHaptic('tap');
  }

  async manualReconnect(): Promise<void> {
    await this.api.detectControlMode();
    this.telemetry.stop();
    this.telemetry.start();
    this.state.triggerHaptic('confirm');
  }

  exportData() {
    const data = {
      state: {
        theme: this.state.theme(),
        density: this.state.density(),
        zenMode: this.state.zenMode(),
        volume: this.state.audioVolume(),
        audioAlerts: this.state.audioAlertsEnabled(),
        voiceAlerts: this.state.voiceAlertsEnabled(),
        thresholds: this.state.kpiThresholds()
      },
      lastPayload: this.state.lastLivePayload(),
      snaps: this.state.snaps()
    };
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `rvt_export_${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
    this.state.triggerHaptic('success');
  }

  getBreadcrumbTitle(): string {
    const view = this.state.currentView();
    switch (view) {
      case 'home': return 'Session readiness';
      case 'live': return 'Live telemetry';
      case 'report': return 'Session report';
      case 'help': return 'Operator playbook';
      case 'settings': return 'Settings';
      default: return 'Radar Vital Console';
    }
  }

  getBreadcrumbEyebrow(): string {
    const view = this.state.currentView();
    switch (view) {
      case 'home': return 'Workflow';
      case 'live': return 'Monitoring';
      case 'report': return 'Review';
      case 'help': return 'Reference';
      case 'settings': return 'Console';
      default: return 'Workflow';
    }
  }
}
