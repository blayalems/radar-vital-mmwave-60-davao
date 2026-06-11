import { ChangeDetectionStrategy, Component, inject, signal } from '@angular/core';
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
    MatToolbarModule
  ],
  templateUrl: './topbar.component.html',
  styleUrl: './topbar.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class TopbarComponent {
  protected readonly state = inject(StateService);
  protected readonly serverLifecycle = inject(ServerLifecycleService);
  protected readonly auth = inject(AuthService);
  protected readonly mobileActionsOpen = signal(false);
  private readonly api = inject(ApiService);
  private readonly telemetry = inject(TelemetryService);
  private readonly dialog = inject(MatDialog);
  private readonly router = inject(Router);

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

  setDensity(d: 'comfortable' | 'compact') {
    this.state.density.set(d);
    this.state.triggerHaptic('tap');
  }

  setLiveMode(m: 'simple' | 'advanced') {
    this.state.zenMode.set(m === 'simple');
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
      case 'home': return 'Start a new session';
      case 'live': return 'Live Monitoring Console';
      case 'report': return 'ML Readiness Reports';
      case 'help': return 'Playbook & Documentation';
      case 'settings': return 'Dashboard Settings';
      default: return 'Radar Vital Console';
    }
  }

  getBreadcrumbEyebrow(): string {
    const view = this.state.currentView();
    switch (view) {
      case 'home': return 'Setup';
      case 'live': return 'Vitals Console';
      case 'report': return 'Review';
      case 'help': return 'Playbook';
      case 'settings': return 'Configuration';
      default: return 'Workflow';
    }
  }
}
