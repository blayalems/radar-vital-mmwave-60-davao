import { TestBed } from '@angular/core/testing';
import { FirstRunService } from './first-run.service';
import { CONSENT_KEY, TUTORIAL_DONE_KEY } from './rvt-storage-keys';
import { TERMS_VERSION } from './app-meta';

describe('FirstRunService', () => {
  let service: FirstRunService;

  beforeEach(() => {
    localStorage.clear();
    TestBed.configureTestingModule({
      providers: [FirstRunService]
    });
    service = TestBed.inject(FirstRunService);
  });

  afterEach(() => {
    localStorage.clear();
    TestBed.resetTestingModule();
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  // ---- consentRequired -------------------------------------------------------

  it('should require consent when no record exists', () => {
    expect(service.consentRequired()).toBe(true);
  });

  it('should require consent when stored version does not match TERMS_VERSION', () => {
    localStorage.setItem(CONSENT_KEY, JSON.stringify({ version: '0.0.0', accepted_at: new Date().toISOString() }));
    TestBed.resetTestingModule();
    TestBed.configureTestingModule({ providers: [FirstRunService] });
    const s = TestBed.inject(FirstRunService);
    expect(s.consentRequired()).toBe(true);
  });

  it('should not require consent after acceptConsent() is called', () => {
    service.acceptConsent();
    expect(service.consentRequired()).toBe(false);
  });

  it('should persist consent record to localStorage on acceptConsent()', () => {
    service.acceptConsent();
    const raw = localStorage.getItem(CONSENT_KEY);
    expect(raw).toBeTruthy();
    const parsed = JSON.parse(raw!);
    expect(parsed.version).toBe(TERMS_VERSION);
    expect(typeof parsed.accepted_at).toBe('string');
  });

  it('should not require consent when a valid matching record is already stored', () => {
    localStorage.setItem(CONSENT_KEY, JSON.stringify({ version: TERMS_VERSION, accepted_at: new Date().toISOString() }));
    TestBed.resetTestingModule();
    TestBed.configureTestingModule({ providers: [FirstRunService] });
    const s = TestBed.inject(FirstRunService);
    expect(s.consentRequired()).toBe(false);
  });

  it('should gracefully handle malformed JSON in CONSENT_KEY', () => {
    localStorage.setItem(CONSENT_KEY, '{invalid_json');
    TestBed.resetTestingModule();
    TestBed.configureTestingModule({ providers: [FirstRunService] });
    const s = TestBed.inject(FirstRunService);
    expect(s.consentRequired()).toBe(true);
  });

  // ---- tutorialDone / tutorialPending ----------------------------------------

  it('tutorialDone() should return false when TUTORIAL_DONE_KEY is absent', () => {
    expect(service.tutorialDone()).toBe(false);
  });

  it('tutorialDone() should return true when TUTORIAL_DONE_KEY is set', () => {
    localStorage.setItem(TUTORIAL_DONE_KEY, '1');
    expect(service.tutorialDone()).toBe(true);
  });

  it('tutorialPending() mirrors !tutorialDone()', () => {
    expect(service.tutorialPending()).toBe(true);
    localStorage.setItem(TUTORIAL_DONE_KEY, '1');
    expect(service.tutorialPending()).toBe(false);
  });

  // ---- markTutorialDone ------------------------------------------------------

  it('markTutorialDone() should set TUTORIAL_DONE_KEY and close tutorialOpen', () => {
    service.tutorialOpen.set(true);
    service.markTutorialDone();
    expect(localStorage.getItem(TUTORIAL_DONE_KEY)).toBe('1');
    expect(service.tutorialOpen()).toBe(false);
    expect(service.tutorialDone()).toBe(true);
  });

  // ---- replayTutorial --------------------------------------------------------

  it('replayTutorial() should set tutorialOpen to true without clearing done flag', () => {
    service.markTutorialDone();
    expect(service.tutorialOpen()).toBe(false);
    service.replayTutorial();
    expect(service.tutorialOpen()).toBe(true);
    // done flag is NOT cleared — reload will not re-trigger auto-open
    expect(localStorage.getItem(TUTORIAL_DONE_KEY)).toBe('1');
  });

  // ---- tutorialOpen signal ---------------------------------------------------

  it('tutorialOpen should start as false', () => {
    expect(service.tutorialOpen()).toBe(false);
  });

  // ---- rvt-operator-authenticated event -------------------------------------

  it('dispatching rvt-operator-authenticated opens tutorial when tutorial not done', async () => {
    window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    await Promise.resolve();
    expect(service.tutorialOpen()).toBe(true);
  });

  it('dispatching rvt-operator-authenticated does NOT open tutorial when tutorialDone is true', async () => {
    localStorage.setItem(TUTORIAL_DONE_KEY, '1');
    window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    await Promise.resolve();
    expect(service.tutorialOpen()).toBe(false);
  });

  it('dispatching rvt-operator-authenticated twice only opens tutorial once per session', async () => {
    window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    await Promise.resolve();
    expect(service.tutorialOpen()).toBe(true);
    // Close it
    service.tutorialOpen.set(false);
    // Second dispatch — should NOT reopen (session guard)
    window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    await Promise.resolve();
    expect(service.tutorialOpen()).toBe(false);
  });

  it('replayTutorial() resets session guard so next auth event re-opens tutorial', async () => {
    // First auth opens tutorial
    window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    await Promise.resolve();
    expect(service.tutorialOpen()).toBe(true);
    service.tutorialOpen.set(false);

    // replayTutorial resets guard
    service.replayTutorial();
    service.tutorialOpen.set(false); // user closes it

    // Now a new auth event can open it again (guard was reset)
    window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    await Promise.resolve();
    expect(service.tutorialOpen()).toBe(true);
  });
});
