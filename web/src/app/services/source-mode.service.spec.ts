import { TestBed } from '@angular/core/testing';
import { SourceModeService } from './source-mode.service';
import { SessionStore } from './stores/session.store';

describe('SourceModeService', () => {
  let service: SourceModeService;
  let sessionStore: SessionStore;

  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [SourceModeService]
    });
    service = TestBed.inject(SourceModeService);
    sessionStore = TestBed.inject(SessionStore);
  });

  it('owns explicit source transitions and their storage scope', () => {
    expect(service.mode()).toBe('live');
    expect(service.storageScope()).toBe('live');

    expect(service.setManualDemo(true)).toBe(true);
    expect(service.mode()).toBe('manual-demo');
    expect(service.storageScope()).toBe('demo');

    service.clearLocalSimulation();
    service.applyTrainerStatus({ ok: true, mode: 'sandbox' });
    expect(service.mode()).toBe('trainer-sandbox');
    expect(service.storageScope()).toBe('demo');
  });

  it('refuses simulated sources while a real trainer session is active', () => {
    sessionStore.ctlStatus.set({ ok: true, mode: 'live' });
    sessionStore.currentSessionId.set('session-live-1');
    sessionStore.sessionActive.set(true);

    expect(service.setManualDemo(true)).toBe(false);
    expect(service.setAutomaticDemoActive(true)).toBe(false);
    expect(service.mode()).toBe('live');
  });

  it('does not treat integrated mock telemetry as a real capture session', () => {
    service.applyTrainerStatus({
      ok: true,
      mode: 'sandbox',
      active_session: { session_id: 'mock', mock: true }
    });

    expect(service.realSessionActive()).toBe(false);
    expect(service.setManualDemo(true)).toBe(true);
    expect(service.mode()).toBe('manual-demo');
  });

  it('recognizes the mock-session flag from older trainers without a mode field', () => {
    service.applyTrainerStatus({
      ok: true,
      active_session: { session_id: 'mock', mock: true }
    });

    expect(service.realSessionActive()).toBe(false);
    expect(service.setManualDemo(true)).toBe(true);
  });

  it('enters automatic demo only after the real-session guard passes', () => {
    expect(service.enterAutomaticDemo('trainer unavailable')).toBe(true);
    expect(service.mode()).toBe('automatic-demo');
    expect(sessionStore.ctlOn()).toBe(true);
    expect(sessionStore.ctlStatus()).toMatchObject({
      ok: true,
      mode: 'sandbox',
      reason: 'trainer unavailable'
    });
  });

  it('applies trainer status atomically and clears local automatic fallback', () => {
    expect(service.enterAutomaticDemo('trainer unavailable')).toBe(true);

    service.applyTrainerStatus({
      ok: true,
      mode: 'live',
      active_session: { session_id: 'session-live-2' }
    });

    expect(service.mode()).toBe('live');
    expect(service.automaticDemoActive()).toBe(false);
    expect(sessionStore.sessionActive()).toBe(true);
    expect(sessionStore.currentSessionId()).toBe('session-live-2');
  });

  it('keeps a real-session Stop on trainer transport despite stale demo state', () => {
    expect(service.setManualDemo(true)).toBe(true);
    sessionStore.ctlStatus.set({ ok: true, mode: 'live' });
    sessionStore.currentSessionId.set('session-live-3');
    sessionStore.sessionActive.set(true);

    expect(service.shouldUseSandboxApi('/api/defaults')).toBe(true);
    expect(service.shouldUseSandboxApi('/api/session/stop')).toBe(false);
    expect(service.shouldUseSandboxApi('/api/session/stop?reason=test')).toBe(false);
  });
});
