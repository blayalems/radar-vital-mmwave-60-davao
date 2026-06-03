import { Injectable, inject, signal, computed } from '@angular/core';
import { Router } from '@angular/router';
import { MatDialog } from '@angular/material/dialog';
import { StateService } from './state.service';
import { UndoService } from './undo.service';
import { IdleLockService } from './idle-lock.service';
import { PINNED_COMMANDS_KEY } from './rvt-storage-keys';
import { AlertsDialogComponent } from '../components/alerts-dialog/alerts-dialog.component';
import { OperatorHandoffDialogComponent } from '../components/operator-handoff-dialog/operator-handoff-dialog.component';

export interface CommandMeta {
  id: string;
  label: string;
  icon: string;
}

@Injectable({
  providedIn: 'root'
})
export class CommandPinningService {
  private readonly router = inject(Router);
  private readonly dialog = inject(MatDialog);
  private readonly state = inject(StateService);
  private readonly undoService = inject(UndoService);
  private readonly idleLock = inject(IdleLockService);

  readonly pinnedCommandIds = signal<string[]>([]);

  readonly allCommands: CommandMeta[] = [
    { id: 'setup', label: 'Setup', icon: 'play_circle' },
    { id: 'overview', label: 'Overview', icon: 'dashboard' },
    { id: 'waves', label: 'Waves', icon: 'show_chart' },
    { id: 'hr', label: 'HR Trend', icon: 'ecg_heart' },
    { id: 'rr', label: 'RR Trend', icon: 'pulmonology' },
    { id: 'snapshots', label: 'Snapshots', icon: 'bookmark' },
    { id: 'audit', label: 'Audit', icon: 'fact_check' },
    { id: 'reports', label: 'Report', icon: 'assignment' },
    { id: 'help', label: 'Help', icon: 'help' },
    { id: 'settings', label: 'Settings', icon: 'settings' },
    { id: 'undo', label: 'Undo', icon: 'undo' },
    { id: 'redo', label: 'Redo', icon: 'redo' },
    { id: 'privacy-lock', label: 'Lock', icon: 'lock' },
    { id: 'alerts', label: 'Alerts', icon: 'notifications' },
    { id: 'operator-handoff', label: 'Handoff', icon: 'transfer_within_a_station' },
    { id: 'print', label: 'Print', icon: 'print' },
  ];

  readonly pinnedCommands = computed(() => {
    return this.allCommands.filter(c => this.pinnedCommandIds().includes(c.id));
  });

  constructor() {
    this.loadPins();
  }

  private loadPins() {
    try {
      const stored = localStorage.getItem(PINNED_COMMANDS_KEY);
      if (stored) {
        const parsed = JSON.parse(stored);
        if (Array.isArray(parsed)) {
          this.pinnedCommandIds.set(parsed.slice(0, 4));
        }
      }
    } catch (_) {}
  }

  togglePin(cmdId: string): void {
    const current = this.pinnedCommandIds();
    let next: string[];
    if (current.includes(cmdId)) {
      next = current.filter(id => id !== cmdId);
    } else {
      if (current.length >= 4) {
        next = [...current.slice(1), cmdId];
      } else {
        next = [...current, cmdId];
      }
    }
    this.pinnedCommandIds.set(next);
    try {
      localStorage.setItem(PINNED_COMMANDS_KEY, JSON.stringify(next));
    } catch (_) {}
  }

  isPinned(cmdId: string): boolean {
    return this.pinnedCommandIds().includes(cmdId);
  }

  execute(id: string): void {
    this.state.triggerHaptic('tap');
    if (id === 'setup') {
      void this.router.navigate(['/home']);
    } else if (['overview', 'waves', 'hr', 'rr', 'snapshots', 'audit'].includes(id)) {
      const tabMap: Record<string, string> = {
        overview: 'tab-overview',
        waves: 'tab-waves',
        hr: 'tab-hr',
        rr: 'tab-rr',
        snapshots: 'tab-snaps',
        audit: 'tab-audit'
      };
      this.state.activeTab.set(tabMap[id]);
      void this.router.navigate(['/live']);
    } else if (id === 'reports') {
      void this.router.navigate(['/report']);
    } else if (id === 'help') {
      void this.router.navigate(['/help']);
    } else if (id === 'settings') {
      void this.router.navigate(['/settings']);
    } else if (id === 'undo') {
      if (this.undoService.canUndo()) {
        this.undoService.undo();
      }
    } else if (id === 'redo') {
      if (this.undoService.canRedo()) {
        this.undoService.redo();
      }
    } else if (id === 'privacy-lock') {
      this.idleLock.lock();
    } else if (id === 'alerts') {
      this.dialog.open(AlertsDialogComponent, {
        maxWidth: 'calc(100vw - 24px)',
        restoreFocus: true,
        panelClass: 'm3-dialog-panel'
      });
    } else if (id === 'operator-handoff') {
      this.dialog.open(OperatorHandoffDialogComponent, {
        maxWidth: 'calc(100vw - 24px)',
        restoreFocus: true,
        panelClass: 'm3-dialog-panel'
      });
    } else if (id === 'print') {
      window.print();
    }
  }
}
