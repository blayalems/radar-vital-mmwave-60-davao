import { DestroyRef, Injectable, computed, inject, signal } from '@angular/core';

type InstallOutcome = 'accepted' | 'dismissed' | 'unavailable';

interface BeforeInstallPromptEvent extends Event {
  prompt(): Promise<void>;
  userChoice: Promise<{ outcome: 'accepted' | 'dismissed'; platform?: string }>;
}

interface CapacitorBridge {
  isNativePlatform?(): boolean;
}

interface TauriBridge {
  core?: unknown;
}

const INSTALL_PROMPT_DISMISSED_KEY = 'rvt-install-prompt-dismissed';

@Injectable({
  providedIn: 'root'
})
export class InstallPromptService {
  private readonly destroyRef = inject(DestroyRef);
  private readonly deferredPrompt = signal<BeforeInstallPromptEvent | null>(null);
  private readonly dismissed = signal(this.readDismissed());

  readonly promptInFlight = signal(false);
  readonly available = computed(() => this.deferredPrompt() !== null);
  readonly nativeShell = computed(() => this.isNativeShell());
  readonly standalone = computed(() => this.isStandaloneDisplay());
  readonly showBanner = computed(() =>
    this.available() &&
    !this.dismissed() &&
    !this.nativeShell() &&
    !this.standalone() &&
    !this.promptInFlight()
  );

  constructor() {
    if (typeof window === 'undefined') return;

    const handleBeforeInstallPrompt = (event: Event) => {
      if (this.isNativeShell() || this.isStandaloneDisplay()) return;
      event.preventDefault();
      this.deferredPrompt.set(event as BeforeInstallPromptEvent);
    };
    const handleAppInstalled = () => {
      this.deferredPrompt.set(null);
      this.setDismissed(true);
    };

    window.addEventListener('beforeinstallprompt', handleBeforeInstallPrompt);
    window.addEventListener('appinstalled', handleAppInstalled);
    this.destroyRef.onDestroy(() => {
      window.removeEventListener('beforeinstallprompt', handleBeforeInstallPrompt);
      window.removeEventListener('appinstalled', handleAppInstalled);
    });
  }

  dismiss(): void {
    this.setDismissed(true);
  }

  async promptInstall(): Promise<InstallOutcome> {
    const prompt = this.deferredPrompt();
    if (!prompt || this.nativeShell() || this.standalone()) return 'unavailable';

    this.promptInFlight.set(true);
    try {
      await prompt.prompt();
      const choice = await prompt.userChoice.catch(() => null);
      const outcome: InstallOutcome = choice?.outcome === 'accepted' ? 'accepted' : 'dismissed';
      this.deferredPrompt.set(null);
      this.setDismissed(true);
      return outcome;
    } finally {
      this.promptInFlight.set(false);
    }
  }

  private setDismissed(value: boolean): void {
    this.dismissed.set(value);
    try {
      if (value) {
        localStorage.setItem(INSTALL_PROMPT_DISMISSED_KEY, '1');
      } else {
        localStorage.removeItem(INSTALL_PROMPT_DISMISSED_KEY);
      }
    } catch (_) {}
  }

  private readDismissed(): boolean {
    try {
      return localStorage.getItem(INSTALL_PROMPT_DISMISSED_KEY) === '1';
    } catch (_) {
      return false;
    }
  }

  private isNativeShell(): boolean {
    if (typeof window === 'undefined') return false;
    const tauri = (window as Window & { __TAURI__?: TauriBridge }).__TAURI__;
    if (tauri?.core) return true;
    const cap = (window as Window & { Capacitor?: CapacitorBridge }).Capacitor;
    return cap?.isNativePlatform?.() === true;
  }

  private isStandaloneDisplay(): boolean {
    if (typeof window === 'undefined') return false;
    const navigatorWithStandalone = navigator as Navigator & { standalone?: boolean };
    const standaloneMedia = window.matchMedia?.('(display-mode: standalone)');
    return standaloneMedia?.matches === true || navigatorWithStandalone.standalone === true;
  }
}
