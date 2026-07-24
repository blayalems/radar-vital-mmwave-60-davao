import { Injectable, computed, inject } from '@angular/core';
import { ControlStatus, StorageScope } from '../models/rvt.models';
import { SessionStore } from './stores/session.store';
import { UiStore } from './stores/ui.store';

export type SourceMode = 'live' | 'manual-demo' | 'automatic-demo' | 'trainer-sandbox';

@Injectable({
  providedIn: 'root'
})
export class SourceModeService {
  private readonly uiStore = inject(UiStore);
  private readonly sessionStore = inject(SessionStore);

  readonly manualDemoActive = this.uiStore.demoMode.asReadonly();
  readonly autoDemoOnDisconnect = this.uiStore.autoDemoOnDisconnect.asReadonly();
  readonly automaticDemoActive = this.uiStore.autoDemoActive.asReadonly();
  readonly trainerSandboxActive = computed(
    () => this.sessionStore.ctlStatus()?.mode === 'sandbox'
  );
  readonly mode = computed<SourceMode>(() => {
    if (this.manualDemoActive()) return 'manual-demo';
    if (this.automaticDemoActive()) return 'automatic-demo';
    if (this.trainerSandboxActive()) return 'trainer-sandbox';
    return 'live';
  });
  readonly simulated = computed(() => this.mode() !== 'live');
  readonly storageScope = computed<StorageScope>(() => this.simulated() ? 'demo' : 'live');
  readonly realSessionActive = computed(() => {
    const sessionId = this.sessionStore.currentSessionId() || '';
    return this.sessionStore.sessionActive()
      && this.sessionStore.ctlStatus()?.mode !== 'sandbox'
      && !sessionId.startsWith('sandbox_');
  });

  setManualDemo(enabled: boolean): boolean {
    if (enabled && this.realSessionActive()) return false;
    this.uiStore.demoMode.set(enabled);
    return true;
  }

  setAutomaticDemoActive(enabled: boolean): boolean {
    if (enabled && this.realSessionActive()) return false;
    this.uiStore.autoDemoActive.set(enabled);
    return true;
  }

  setAutoDemoOnDisconnect(enabled: boolean): void {
    this.uiStore.autoDemoOnDisconnect.set(enabled);
  }

  restorePreferences(manualDemo: boolean, autoDemoOnDisconnect: boolean): void {
    this.uiStore.demoMode.set(manualDemo);
    this.uiStore.autoDemoOnDisconnect.set(autoDemoOnDisconnect);
  }

  clearLocalSimulation(): void {
    this.uiStore.demoMode.set(false);
    this.uiStore.autoDemoActive.set(false);
  }

  resetPreferences(): void {
    this.clearLocalSimulation();
    this.uiStore.autoDemoOnDisconnect.set(false);
  }

  applyTrainerStatus(status: ControlStatus): void {
    const mode = status.mode === 'sandbox' ? 'sandbox' : 'live';
    const activeSession = status.active_session;
    this.uiStore.autoDemoActive.set(false);
    this.sessionStore.ctlOn.set(true);
    this.sessionStore.ctlStatus.set({ ...status, mode });
    this.sessionStore.sessionActive.set(!!activeSession);
    this.sessionStore.currentSessionId.set(activeSession?.session_id || null);
  }

  markAuthenticationRequired(): void {
    this.uiStore.autoDemoActive.set(false);
    this.sessionStore.ctlOn.set(true);
    this.sessionStore.ctlStatus.set({
      ok: true,
      mode: 'live',
      reason: 'unauthenticated'
    });
  }

  enterAutomaticDemo(reason: string): boolean {
    if (!this.setAutomaticDemoActive(true)) {
      this.sessionStore.ctlOn.set(true);
      this.sessionStore.ctlStatus.update(status => ({
        ...(status ?? {}),
        ok: false,
        mode: 'live',
        reason
      }));
      return false;
    }
    this.sessionStore.ctlOn.set(true);
    this.sessionStore.sessionActive.set(false);
    this.sessionStore.ctlStopPending.set(false);
    this.sessionStore.ctlStatus.set({
      ok: true,
      mode: 'sandbox',
      reason
    });
    return true;
  }

  shouldUseSandboxApi(path: string, bypassSandbox = false): boolean {
    if (bypassSandbox || !this.simulated() || !path.startsWith('/api/')) return false;
    const pathname = String(path).split('?', 1)[0];
    return !(pathname === '/api/session/stop' && this.realSessionActive());
  }
}
