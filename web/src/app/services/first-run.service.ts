import { DestroyRef, Injectable, computed, inject, signal } from '@angular/core';
import { CONSENT_KEY, TUTORIAL_DONE_KEY } from './rvt-storage-keys';
import { TERMS_VERSION } from './app-meta';

export interface ConsentRecord {
  version: string;
  accepted_at: string;
}

@Injectable({
  providedIn: 'root'
})
export class FirstRunService {
  private readonly destroyRef = inject(DestroyRef);

  /** Internal signal: null when consent is absent/stale, record when valid. */
  private readonly _consentRecord = signal<ConsentRecord | null>(this.readConsentRecord());

  /** True when the user must accept/re-accept the current terms. */
  readonly consentRequired = computed(() => {
    const record = this._consentRecord();
    return record === null || record.version !== TERMS_VERSION;
  });

  /** True while the tutorial dialog should be open. */
  readonly tutorialOpen = signal(false);

  private tutorialShownThisSession = false;

  constructor() {
    this.bindOperatorAuthEvent();
  }

  /** Persist acceptance for the current TERMS_VERSION and update signal. */
  acceptConsent(): void {
    const record: ConsentRecord = {
      version: TERMS_VERSION,
      accepted_at: new Date().toISOString()
    };
    try {
      localStorage.setItem(CONSENT_KEY, JSON.stringify(record));
    } catch (_) {
      // Storage may be unavailable (private mode, full quota). Continue in memory.
    }
    this._consentRecord.set(record);
  }

  /** True when the operator has not yet completed the tutorial this install. */
  tutorialDone(): boolean {
    try {
      return !!localStorage.getItem(TUTORIAL_DONE_KEY);
    } catch (_) {
      return false;
    }
  }

  /** @deprecated Use tutorialDone() — kept for internal compatibility. */
  tutorialPending(): boolean {
    return !this.tutorialDone();
  }

  /** Mark the tutorial as completed. Persists to localStorage. */
  markTutorialDone(): void {
    try {
      localStorage.setItem(TUTORIAL_DONE_KEY, '1');
    } catch (_) {}
    this.tutorialOpen.set(false);
  }

  /**
   * Replay the tutorial on demand (Wave 2 wires this to the command palette).
   * Does NOT clear the persistent done-flag; just re-opens the dialog.
   */
  replayTutorial(): void {
    this.tutorialShownThisSession = false;
    this.tutorialOpen.set(true);
  }

  // ---------------------------------------------------------------------------
  // Private helpers
  // ---------------------------------------------------------------------------

  private readConsentRecord(): ConsentRecord | null {
    try {
      const raw = localStorage.getItem(CONSENT_KEY);
      if (!raw) return null;
      const parsed = JSON.parse(raw) as Partial<ConsentRecord>;
      if (typeof parsed.version !== 'string' || !parsed.accepted_at) return null;
      return { version: parsed.version, accepted_at: parsed.accepted_at };
    } catch (_) {
      return null;
    }
  }

  /** Listen for 'rvt-operator-authenticated' window event; open tutorial once per session. */
  private bindOperatorAuthEvent(): void {
    if (typeof window === 'undefined') return;

    const listener = () => {
      if (!this.tutorialShownThisSession && !this.tutorialDone()) {
        this.tutorialShownThisSession = true;
        this.tutorialOpen.set(true);
      }
    };

    window.addEventListener('rvt-operator-authenticated', listener);

    this.destroyRef.onDestroy(() => {
      window.removeEventListener('rvt-operator-authenticated', listener);
    });
  }
}
