import { TestBed } from '@angular/core/testing';

import { InstallPromptService } from './install-prompt.service';

class FakeBeforeInstallPromptEvent extends Event {
  prompt = vi.fn().mockResolvedValue(undefined);
  userChoice: Promise<{ outcome: 'accepted' | 'dismissed'; platform: string }>;

  constructor(outcome: 'accepted' | 'dismissed' = 'accepted') {
    super('beforeinstallprompt', { cancelable: true });
    this.userChoice = Promise.resolve({ outcome, platform: 'web' });
  }
}

describe('InstallPromptService', () => {
  beforeEach(() => {
    localStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    delete (window as Window & { Capacitor?: unknown }).Capacitor;
  });

  afterEach(() => {
    TestBed.resetTestingModule();
    localStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    delete (window as Window & { Capacitor?: unknown }).Capacitor;
  });

  function configure(): InstallPromptService {
    TestBed.configureTestingModule({
      providers: [InstallPromptService]
    });
    return TestBed.inject(InstallPromptService);
  }

  it('captures beforeinstallprompt and prevents the browser mini-infobar', () => {
    const service = configure();
    const event = new FakeBeforeInstallPromptEvent();

    window.dispatchEvent(event);

    expect(event.defaultPrevented).toBe(true);
    expect(service.available()).toBe(true);
    expect(service.showBanner()).toBe(true);
  });

  it('persists dismissal and hides the banner', () => {
    const service = configure();
    window.dispatchEvent(new FakeBeforeInstallPromptEvent());

    service.dismiss();

    expect(service.showBanner()).toBe(false);
    expect(localStorage.getItem('rvt-install-prompt-dismissed')).toBe('1');
  });

  it('prompts install and clears the one-shot deferred event', async () => {
    const service = configure();
    const event = new FakeBeforeInstallPromptEvent('accepted');
    window.dispatchEvent(event);

    await expect(service.promptInstall()).resolves.toBe('accepted');

    expect(event.prompt).toHaveBeenCalledOnce();
    expect(service.available()).toBe(false);
    expect(service.showBanner()).toBe(false);
  });

  it('returns unavailable when no deferred prompt exists', async () => {
    const service = configure();

    await expect(service.promptInstall()).resolves.toBe('unavailable');
  });

  it('suppresses install prompts inside native shells', () => {
    (window as Window & { __TAURI__?: unknown }).__TAURI__ = { core: {} };
    const service = configure();
    const event = new FakeBeforeInstallPromptEvent();

    window.dispatchEvent(event);

    expect(event.defaultPrevented).toBe(false);
    expect(service.nativeShell()).toBe(true);
    expect(service.showBanner()).toBe(false);
  });

  it('suppresses install prompts inside Capacitor native runtime', () => {
    (window as Window & { Capacitor?: unknown }).Capacitor = { isNativePlatform: () => true };
    const service = configure();
    const event = new FakeBeforeInstallPromptEvent();

    window.dispatchEvent(event);

    expect(service.nativeShell()).toBe(true);
    expect(service.showBanner()).toBe(false);
  });
});
