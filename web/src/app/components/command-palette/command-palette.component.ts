import { ChangeDetectionStrategy, Component, computed, inject, signal } from '@angular/core';
import { Router } from '@angular/router';
import { MatButtonModule } from '@angular/material/button';
import { MatChipsModule } from '@angular/material/chips';
import { MatDialog, MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatListModule } from '@angular/material/list';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';
import { firstValueFrom } from 'rxjs';

import { normalizePreflightStatus, PreflightCheck, SnapshotRecord, ThemeId } from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';
import { DEFAULT_KPI_THRESHOLDS, StateService } from '../../services/state.service';
import { AlertsDialogComponent } from '../alerts-dialog/alerts-dialog.component';
import { ConfirmDialogComponent } from '../confirm-dialog/confirm-dialog.component';
import { CommandPinningService } from '../../services/command-pinning.service';

type CommandGroup = 'Navigate' | 'Live Session' | 'Source' | 'Appearance' | 'Export';

interface PaletteCommand {
  id: string;
  label: string;
  description: string;
  keywords: string;
  group: CommandGroup;
  icon: string;
  shortcut?: string;
  disabledReason?: () => string | null;
  action: () => void | Promise<unknown>;
}

@Component({
  selector: 'app-command-palette',
  imports: [
    MatButtonModule,
    MatChipsModule,
    MatDialogModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule,
    MatListModule,
    MatSnackBarModule
  ],
  templateUrl: './command-palette.component.html',
  styleUrl: './command-palette.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class CommandPaletteComponent {
  private readonly state = inject(StateService);
  private readonly api = inject(ApiService);
  private readonly router = inject(Router);
  private readonly dialog = inject(MatDialog);
  private readonly dialogRef = inject(MatDialogRef<CommandPaletteComponent>);
  private readonly snackBar = inject(MatSnackBar);
  protected readonly pinning = inject(CommandPinningService);

  protected readonly query = signal('');
  protected readonly commands: PaletteCommand[] = [
    this.navCommand('setup', 'Start a new session', 'Setup, hardware discovery and preflight checks', 'play_circle', '/home', 'start record preflight hardware'),
    this.tabCommand('overview', 'Live overview', 'KPI and reference overview', 'dashboard', 'tab-overview', 'Alt+1'),
    this.tabCommand('waves', 'Live waves', 'Breath and heartbeat phase waveforms', 'show_chart', 'tab-waves', 'Alt+2'),
    this.tabCommand('hr', 'Heart rate trends', 'Open the HR trend tab', 'ecg_heart', 'tab-hr', 'Alt+3'),
    this.tabCommand('rr', 'Respiration trends', 'Open the RR trend tab', 'pulmonology', 'tab-rr', 'Alt+4'),
    this.tabCommand('snapshots', 'Pinned snapshots', 'Review pinned frames and annotations', 'bookmark', 'tab-snaps', 'Alt+5'),
    this.tabCommand('audit', 'Audit log', 'Review actual stream events and policy fields', 'fact_check', 'tab-audit', 'Alt+6'),
    this.navCommand('reports', 'Reports and comparison', 'Summary, recorded trends and analysis status', 'assignment', '/report', 'summary compare thesis analysis'),
    this.navCommand('help', 'Operator playbook', 'Search field guidance and recovery actions', 'help', '/help', 'documentation keyboard troubleshooting'),
    this.navCommand('settings', 'Settings', 'Source, appearance, thresholds and audio', 'settings', '/settings', 'endpoint pairing token audio'),
    {
      id: 'capture',
      label: 'Pin current snapshot',
      description: 'Capture HR, RR, range and BLE reference at this moment',
      keywords: 'snapshot bookmark frame capture',
      group: 'Live Session',
      icon: 'bookmark_add',
      disabledReason: () => this.state.lastPayload() ? null : 'Requires a live or demo payload.',
      action: () => this.captureSnapshot()
    },
    {
      id: 'preflight',
      label: 'Run hardware preflight',
      description: 'Check radar serial, BLE reference and session storage readiness',
      keywords: 'check verify serial bluetooth ble readiness setup',
      group: 'Live Session',
      icon: 'fact_check',
      action: () => this.runPreflight()
    },
    {
      id: 'pause',
      label: 'Pause or resume live stream',
      description: 'Freeze or resume dashboard stream updates',
      keywords: 'pause resume freeze stream',
      group: 'Live Session',
      icon: 'pause_circle',
      action: () => this.state.paused.update(paused => !paused)
    },
    {
      id: 'stop',
      label: 'Stop active session',
      description: 'Stop recording and move to report review',
      keywords: 'stop finish end record',
      group: 'Live Session',
      icon: 'stop_circle',
      disabledReason: () => this.state.sessionActive() ? null : 'No active session is running.',
      action: () => this.stopSession()
    },
    {
      id: 'alerts',
      label: 'Open alerts',
      description: 'Filter, pin, snooze and dismiss alert history',
      keywords: 'warning fault notification drawer',
      group: 'Live Session',
      icon: 'notifications',
      action: () => this.openAlerts()
    },
    {
      id: 'copy-brief',
      label: 'Copy operator brief',
      description: 'Copy current subject, source and vitals summary',
      keywords: 'handoff clipboard summary brief',
      group: 'Export',
      icon: 'content_copy',
      action: () => this.copyOperatorBrief()
    },
    {
      id: 'print',
      label: 'Print current view',
      description: 'Open browser print output for the visible report or dashboard view',
      keywords: 'pdf report hardcopy export',
      group: 'Export',
      icon: 'print',
      action: () => window.print()
    },
    {
      id: 'export-payload',
      label: 'Export current payload JSON',
      description: 'Download the most recently received source payload',
      keywords: 'download json data export telemetry',
      group: 'Export',
      icon: 'download',
      disabledReason: () => this.state.lastLivePayload() ? null : 'No source payload is available.',
      action: () => this.exportCurrentPayload()
    },
    {
      id: 'reconnect',
      label: 'Reconnect trainer',
      description: 'Retry the configured trainer endpoint and leave fallback mode when reachable',
      keywords: 'retry live server endpoint connect',
      group: 'Source',
      icon: 'sync',
      action: () => this.reconnectTrainer()
    },
    {
      id: 'demo-toggle',
      label: 'Toggle demo mode',
      description: 'Use or stop simulated telemetry; simulation is always labelled',
      keywords: 'sandbox simulation preview source',
      group: 'Source',
      icon: 'science',
      action: () => this.toggleDemo()
    },
    this.themeCommand('light', 'Light theme'),
    this.themeCommand('dark', 'Dark theme'),
    this.themeCommand('night', 'Night theme'),
    this.themeCommand('hc', 'High contrast theme'),
    {
      id: 'density-comfort',
      label: 'Comfortable density',
      description: 'Increase spacing for touch operation',
      keywords: 'display spacing layout accessible',
      group: 'Appearance',
      icon: 'density_medium',
      action: () => this.state.density.set('comfortable')
    },
    {
      id: 'density-compact',
      label: 'Compact density',
      description: 'Show more metrics on larger displays',
      keywords: 'display spacing layout dense',
      group: 'Appearance',
      icon: 'density_small',
      action: () => this.state.density.set('compact')
    },
    {
      id: 'focus',
      label: 'Toggle focused live view',
      description: 'Show either simplified or detailed live telemetry',
      keywords: 'zen simple advanced focus details',
      group: 'Appearance',
      icon: 'visibility',
      action: () => this.state.zenMode.update(enabled => !enabled)
    },
    {
      id: 'audio-toggle',
      label: 'Toggle audio alerts',
      description: 'Enable or mute threshold and connection tones',
      keywords: 'sound beep alarm notification',
      group: 'Appearance',
      icon: 'volume_up',
      action: () => this.state.audioAlertsEnabled.update(enabled => !enabled)
    },
    {
      id: 'voice-toggle',
      label: 'Toggle voice announcements',
      description: 'Enable or mute browser speech alert callouts',
      keywords: 'speech announcement accessibility eyes busy',
      group: 'Appearance',
      icon: 'record_voice_over',
      action: () => this.state.voiceAlertsEnabled.update(enabled => !enabled)
    },
    {
      id: 'threshold-reset',
      label: 'Reset KPI alert thresholds',
      description: 'Restore the default HR and RR warning limits',
      keywords: 'limits bpm respiration reset settings',
      group: 'Appearance',
      icon: 'restart_alt',
      action: () => this.state.kpiThresholds.set({ ...DEFAULT_KPI_THRESHOLDS })
    },
    this.rangeCommand(30),
    this.rangeCommand(60),
    this.rangeCommand(120),
    {
      id: 'clear-snapshots',
      label: 'Clear all snapshots',
      description: 'Remove all pinned frames and annotations',
      keywords: 'clear snaps delete snapshots sweep',
      group: 'Live Session',
      icon: 'delete_sweep',
      action: async () => {
        this.state.triggerHaptic('destructiveAccept');
        const confirmed = await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
          data: {
            title: 'Clear all snapshots?',
            message: 'This removes the pinned frames and their notes from this dashboard.',
            confirmLabel: 'Clear snapshots'
          },
          restoreFocus: true
        }).afterClosed());
        if (confirmed) {
          this.state.snaps.set([]);
          this.state.snapNotes.set({});
          this.snackBar.open('All pinned snapshots cleared.', 'Dismiss', { duration: 3000 });
        }
      }
    },
    {
      id: 'tag-motion',
      label: 'Tag observation: Motion',
      description: 'Append [HH:MM:SS] Motion to the active session notes',
      keywords: 'tag motion observation quick active',
      group: 'Live Session',
      icon: 'tag',
      disabledReason: () => this.state.sessionActive() ? null : 'No active session is running.',
      action: () => this.appendSessionTag('Motion')
    },
    {
      id: 'tag-cough',
      label: 'Tag observation: Cough',
      description: 'Append [HH:MM:SS] Cough to the active session notes',
      keywords: 'tag cough observation quick active',
      group: 'Live Session',
      icon: 'tag',
      disabledReason: () => this.state.sessionActive() ? null : 'No active session is running.',
      action: () => this.appendSessionTag('Cough')
    },
    {
      id: 'tag-speaking',
      label: 'Tag observation: Speaking',
      description: 'Append [HH:MM:SS] Speaking to the active session notes',
      keywords: 'tag speaking observation quick active',
      group: 'Live Session',
      icon: 'tag',
      disabledReason: () => this.state.sessionActive() ? null : 'No active session is running.',
      action: () => this.appendSessionTag('Speaking')
    },
    {
      id: 'tag-baseline',
      label: 'Tag observation: Baseline',
      description: 'Append [HH:MM:SS] Baseline to the active session notes',
      keywords: 'tag baseline observation quick active',
      group: 'Live Session',
      icon: 'tag',
      disabledReason: () => this.state.sessionActive() ? null : 'No active session is running.',
      action: () => this.appendSessionTag('Baseline')
    },
    {
      id: 'export-csv',
      label: 'Export session CSV',
      description: 'Download the complete 207-column telemetry CSV',
      keywords: 'download csv data export telemetry columns',
      group: 'Export',
      icon: 'table_view',
      action: () => this.downloadSessionCsvCommand()
    },
    {
      id: 'export-audit',
      label: 'Export audit JSON',
      description: 'Download the history of alerts as a JSON file',
      keywords: 'download json audit export logs telemetry',
      group: 'Export',
      icon: 'download',
      action: () => this.downloadAuditJsonCommand()
    }
  ];

  protected readonly filteredCommands = computed(() => {
    const query = this.query().trim().toLowerCase();
    if (!query) return this.commands;
    return this.commands.filter(command =>
      `${command.label} ${command.description} ${command.keywords} ${command.group}`.toLowerCase().includes(query)
    );
  });

  protected updateQuery(event: Event): void {
    this.query.set((event.target as HTMLInputElement).value);
  }

  protected disabledReason(command: PaletteCommand): string | null {
    return command.disabledReason?.() || null;
  }

  protected run(command: PaletteCommand): void {
    const reason = this.disabledReason(command);
    if (reason) {
      this.snackBar.open(reason, 'Dismiss', { duration: 4000 });
      return;
    }
    this.state.triggerHaptic('tap');
    this.dialogRef.close();
    queueMicrotask(() => void command.action());
  }

  private navCommand(id: string, label: string, description: string, icon: string, path: string, keywords: string): PaletteCommand {
    return { id, label, description, icon, keywords, group: 'Navigate', action: () => this.router.navigate([path]) };
  }

  private tabCommand(id: string, label: string, description: string, icon: string, tab: string, shortcut: string): PaletteCommand {
    return {
      id,
      label,
      description,
      icon,
      shortcut,
      keywords: `live tab ${id}`,
      group: 'Navigate',
      action: async () => {
        this.state.activeTab.set(tab);
        await this.router.navigate(['/live']);
      }
    };
  }

  private themeCommand(theme: ThemeId, label: string): PaletteCommand {
    return {
      id: `theme-${theme}`,
      label,
      description: `Apply the ${label.toLowerCase()} across the dashboard`,
      keywords: `theme palette color ${theme}`,
      group: 'Appearance',
      icon: theme === 'hc' ? 'contrast' : 'palette',
      action: () => this.state.theme.set(theme)
    };
  }

  private rangeCommand(seconds: number): PaletteCommand {
    return {
      id: `window-${seconds}`,
      label: `${seconds}-second chart window`,
      description: `Keep the latest ${seconds} seconds visible in live trend charts`,
      keywords: `range buffer history trend ${seconds}`,
      group: 'Appearance',
      icon: 'timeline',
      action: () => {
        this.state.liveBufferSeconds.set(seconds);
        this.state.maxChartPoints.set(seconds * 60);
      }
    };
  }

  private captureSnapshot(): void {
    const payload = this.state.lastPayload();
    if (!payload) return;
    const id = `snap_${Date.now()}`;
    const snapshot: SnapshotRecord = {
      id,
      ts: Date.now(),
      reported_hr: payload.radar.reported_hr ?? 0,
      reported_rr: payload.radar.reported_rr ?? 0,
      distance_cm: payload.radar.distance_cm ?? 0,
      ble_hr: payload.ble.hr ?? 0,
      ble_rr: payload.ble.rr ?? 0
    };
    this.state.snaps.update(snapshots => [...snapshots, snapshot]);
    this.state.snapNotes.update(notes => ({ ...notes, [id]: '' }));
    this.snackBar.open('Current telemetry frame pinned.', 'Dismiss', { duration: 3000 });
  }

  private async stopSession(): Promise<void> {
    const confirmed = await firstValueFrom(this.dialog.open(ConfirmDialogComponent, {
      data: {
        title: 'Stop monitoring session?',
        message: 'Stop telemetry capture and compile the recorded report.',
        confirmLabel: 'Stop session'
      },
      restoreFocus: true
    }).afterClosed());
    if (!confirmed) return;
    try {
      this.state.ctlStopPending.set(true);
      const response = await this.api.request<{ ok?: boolean }>('/api/session/stop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ reason: 'command_palette' })
      });
      if (response.ok) {
        this.state.currentSessionId.set(null);
        this.state.sessionActive.set(false);
        await this.router.navigate(['/report']);
      }
    } catch (error: unknown) {
      this.snackBar.open(error instanceof Error ? error.message : 'Session stop failed.', 'Dismiss', { duration: 6000 });
    } finally {
      this.state.ctlStopPending.set(false);
    }
  }

  private openAlerts(): void {
    this.dialog.open(AlertsDialogComponent, {
      maxWidth: 'calc(100vw - 24px)',
      restoreFocus: true,
      panelClass: 'm3-dialog-panel'
    });
  }

  private async copyOperatorBrief(): Promise<void> {
    const payload = this.state.lastPayload();
    const setup = this.state.setup();
    const brief = [
      `Subject: ${setup.subject_label || '--'}`,
      `Operator: ${setup.operator_label || '--'}`,
      `Source: ${this.state.ctlStatus()?.mode === 'sandbox' ? 'DEMO' : 'TRAINER'}`,
      `HR: ${payload?.radar.reported_hr ?? '--'} bpm`,
      `RR: ${payload?.radar.reported_rr ?? '--'} br/min`,
      `Distance: ${payload?.radar.distance_cm ?? '--'} cm`,
      `Freshness: ${this.state.telemetryStale() ? 'STALE' : 'LIVE'}`
    ].join('\n');
    try {
      await navigator.clipboard.writeText(brief);
      this.snackBar.open('Operator brief copied.', 'Dismiss', { duration: 3000 });
    } catch (_) {
      this.snackBar.open('Clipboard permission denied.', 'Dismiss', { duration: 5000 });
    }
  }

  private exportCurrentPayload(): void {
    const payload = this.state.lastLivePayload();
    if (!payload) return;
    const href = URL.createObjectURL(new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' }));
    const anchor = document.createElement('a');
    anchor.href = href;
    anchor.download = `rvt_payload_${Date.now()}.json`;
    anchor.click();
    URL.revokeObjectURL(href);
  }

  private async reconnectTrainer(): Promise<void> {
    const connected = await this.api.detectControlMode();
    this.snackBar.open(
      connected ? 'Trainer connection established.' : 'Trainer unavailable; sandbox preview remains active.',
      'Dismiss',
      { duration: 4500 }
    );
  }

  private async toggleDemo(): Promise<void> {
    const enabled = !this.state.demoMode();
    this.state.demoMode.set(enabled);
    if (!enabled) await this.reconnectTrainer();
  }

  private async runPreflight(): Promise<void> {
    const setup = this.state.setup();
    const query = new URLSearchParams({ port: setup.radar_port, address: setup.ble_address });
    await this.router.navigate(['/home']);
    try {
      const response = await this.api.request<{ checks?: PreflightCheck[] }>(`/api/preflight?${query.toString()}`);
      const checks = response.checks || [];
      const failed = checks.filter(check => ['bad', 'fail', 'error'].includes(normalizePreflightStatus(check))).length;
      this.snackBar.open(
        failed
          ? `Preflight complete: ${failed} blocking check${failed === 1 ? '' : 's'} require attention.`
          : 'Preflight complete: no blocking failures.',
        'Dismiss',
        { duration: 5500 }
      );
    } catch (error: unknown) {
      this.snackBar.open(
        error instanceof Error ? error.message : 'Preflight could not be completed.',
        'Dismiss',
        { duration: 6000 }
      );
    }
  }

  private appendSessionTag(tag: string): void {
    const sid = this.state.currentSessionId();
    if (!sid) return;
    const time = new Date().toLocaleTimeString('en-GB', { hour12: false });
    const entry = `[${time}] ${tag}`;
    const currentNotes = this.state.sessionNotes()[sid] || '';
    const next = currentNotes.trim()
      ? `${currentNotes.trimEnd()}\n${entry}`
      : entry;
    this.state.sessionNotes.update(notes => ({ ...notes, [sid]: next }));
    
    const mode = this.state.ctlStatus()?.mode;
    if (mode && mode !== 'sandbox') {
      void this.api.request(`/api/sessions/${encodeURIComponent(sid)}/notes`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ review_summary: next })
      }).catch(err => console.error('Failed to save tagged note', err));
    }
    
    this.snackBar.open(`Observation tagged: ${tag}`, 'Dismiss', { duration: 3000 });
  }

  private async downloadSessionCsvCommand(): Promise<void> {
    const sid = this.state.currentSessionId() || this.state.sessionItems()[0]?.session_id;
    if (!sid) {
      this.snackBar.open('No session is available for export.', 'Dismiss', { duration: 3000 });
      return;
    }
    const mode = this.state.ctlStatus()?.mode;
    if (mode === 'sandbox') {
      try {
        const response = await this.api.request<{ data: Record<string, unknown>[] }>(`/api/sessions/${encodeURIComponent(sid)}/data`);
        const rows = response.data || [];
        if (!rows.length) {
          this.snackBar.open('No telemetry data recorded for this session yet.', 'Dismiss', { duration: 3000 });
          return;
        }
        const fields = Array.from(new Set(rows.flatMap(row => Object.keys(row))));
        const content = [
          fields.map(field => `"${String(field).replaceAll('"', '""')}"`).join(','),
          ...rows.map(row => fields.map(field => `"${String(row[field] ?? '').replaceAll('"', '""')}"`).join(','))
        ].join('\n');
        
        const href = URL.createObjectURL(new Blob([content], { type: 'text/csv' }));
        const anchor = document.createElement('a');
        anchor.href = href;
        anchor.download = `session_data_${sid}.csv`;
        anchor.click();
        URL.revokeObjectURL(href);
        this.state.triggerHaptic('success');
      } catch (error: unknown) {
        this.snackBar.open('Failed to build client-side CSV in sandbox.', 'Dismiss', { duration: 3000 });
      }
    } else {
      try {
        await this.api.download(
          `/api/sessions/${encodeURIComponent(sid)}/files/radar.csv`,
          `session_data_${sid}.csv`
        );
        this.state.triggerHaptic('success');
      } catch (error: unknown) {
        this.snackBar.open(
          error instanceof Error ? error.message : 'Telemetry CSV download failed.',
          'Dismiss',
          { duration: 4000 }
        );
      }
    }
  }

  private async downloadAuditJsonCommand(): Promise<void> {
    const sid = this.state.currentSessionId() || this.state.sessionItems()[0]?.session_id;
    if (!sid) {
      this.snackBar.open('No session is available for export.', 'Dismiss', { duration: 3000 });
      return;
    }
    const mode = this.state.ctlStatus()?.mode;
    if (mode === 'sandbox') {
      const alertHistory = this.state.alertHistory() || [];
      const href = URL.createObjectURL(new Blob([JSON.stringify(alertHistory, null, 2)], { type: 'application/json' }));
      const anchor = document.createElement('a');
      anchor.href = href;
      anchor.download = `session_audit_${sid}.json`;
      anchor.click();
      URL.revokeObjectURL(href);
      this.state.triggerHaptic('success');
    } else {
      try {
        await this.api.download(
          `/api/sessions/${encodeURIComponent(sid)}/files/audit.json`,
          `session_audit_${sid}.json`
        );
        this.state.triggerHaptic('success');
      } catch (error: unknown) {
        const alertHistory = this.state.alertHistory() || [];
        const href = URL.createObjectURL(new Blob([JSON.stringify(alertHistory, null, 2)], { type: 'application/json' }));
        const anchor = document.createElement('a');
        anchor.href = href;
        anchor.download = `session_audit_${sid}.json`;
        anchor.click();
        URL.revokeObjectURL(href);
        this.state.triggerHaptic('success');
      }
    }
  }

  togglePin(event: Event, cmdId: string) {
    event.stopPropagation();
    this.pinning.togglePin(cmdId);
  }
}
