import { ChangeDetectionStrategy, Component, inject } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router, RouterModule } from '@angular/router';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MatButtonModule } from '@angular/material/button';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatBadgeModule } from '@angular/material/badge';
import { MatIconModule } from '@angular/material/icon';
import { MatChipsModule } from '@angular/material/chips';
import { MatMenuModule } from '@angular/material/menu';
import { MatToolbarModule } from '@angular/material/toolbar';

import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';
import { TelemetryService } from '../../services/telemetry.service';
import { SettingsComponent } from '../settings/settings.component';
import { AlertsDialogComponent } from '../alerts-dialog/alerts-dialog.component';
import { CommandPaletteComponent } from '../command-palette/command-palette.component';

@Component({
  selector: 'app-topbar',
  imports: [
    CommonModule,
    RouterModule,
    MatDialogModule,
    MatButtonModule,
    MatButtonToggleModule,
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
  private readonly api = inject(ApiService);
  private readonly telemetry = inject(TelemetryService);
  private readonly dialog = inject(MatDialog);
  private readonly router = inject(Router);

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
