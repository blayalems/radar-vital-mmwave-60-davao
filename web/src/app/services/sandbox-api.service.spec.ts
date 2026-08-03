import { TestBed } from '@angular/core/testing';
import { OPERATOR_TOKEN_KEY, SANDBOX_OPERATOR_PROFILES_KEY } from './rvt-storage-keys';
import { SandboxApiService } from './sandbox-api.service';
import { SessionStore } from './stores/session.store';

describe('SandboxApiService', () => {
  let service: SandboxApiService;
  let sessionStore: SessionStore;

  beforeEach(() => {
    localStorage.clear();
    sessionStorage.clear();
    TestBed.configureTestingModule({
      providers: [SandboxApiService]
    });
    service = TestBed.inject(SandboxApiService);
    sessionStore = TestBed.inject(SessionStore);
  });

  afterEach(() => {
    localStorage.clear();
    sessionStorage.clear();
  });

  it('preserves the session-list response aliases and demo provenance', () => {
    const response = service.request('/api/sessions') as {
      ok: boolean;
      sessions: Array<{ session_id: string; sandbox?: boolean }>;
      items: Array<{ session_id: string; sandbox?: boolean }>;
    };

    expect(response.ok).toBe(true);
    expect(response.sessions).toEqual(response.items);
    expect(response.sessions).toHaveLength(2);
    expect(response.sessions.every(session => session.sandbox && session.session_id.startsWith('sandbox_'))).toBe(true);
  });

  it('starts and stops a local session with the existing response shape', () => {
    sessionStore.setup.update(setup => ({
      ...setup,
      subject_label: 'demo-subject',
      operator_label: 'Demo operator'
    }));

    const started = service.request('/api/session/start', { method: 'POST' }) as {
      ok: boolean;
      session_id: string;
      sandbox: boolean;
    };
    expect(started).toMatchObject({ ok: true, sandbox: true });
    expect(started.session_id).toMatch(/^sandbox_/);
    expect(sessionStore.sessionActive()).toBe(true);

    const stopped = service.request('/api/session/stop', { method: 'POST' }) as {
      ok: boolean;
      session_id: string;
      sandbox: boolean;
      session: { subject_label?: string; operator?: string; sandbox?: boolean };
    };
    expect(stopped).toMatchObject({
      ok: true,
      session_id: started.session_id,
      sandbox: true,
      session: {
        subject_label: 'demo-subject',
        operator: 'Demo operator',
        sandbox: true
      }
    });
    expect(sessionStore.sessionActive()).toBe(false);
    expect(sessionStore.currentSessionId()).toBeNull();
  });

  it('keeps sandbox profiles and sessions in demo-prefixed storage', () => {
    const created = service.request('/api/operator-profiles', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ display_name: 'Demo Operator', initials: 'DO', pin: '123456' })
    }) as { operator: { operator_id: string; display_name: string; initials: string } };

    const login = service.request('/api/auth/login', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ operator_id: created.operator.operator_id, pin: '123456' })
    }) as { token: string; operator: { operator_id: string } };
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, login.token);

    expect(service.request('/api/auth/validate')).toEqual({
      ok: true,
      operator: created.operator
    });
    expect(localStorage.getItem(SANDBOX_OPERATOR_PROFILES_KEY)).toContain('Demo Operator');
    expect(localStorage.getItem('live:rvt-operator-profiles')).toBeNull();
    expect(sessionStorage.getItem('demo:rvt-operator-sessions')).toContain(login.token);
    expect(sessionStorage.getItem('live:rvt-operator-sessions')).toBeNull();
  });

  it('preserves report, preflight, and fallback payload contracts', () => {
    const summary = service.request('/api/sessions/sandbox_test/summary') as {
      sandbox: boolean;
      signal_quality: { session_quality_score: number };
      gates: Record<string, { status: string }>;
    };
    const preflight = service.request('/api/preflight') as {
      ok: boolean;
      checks: Array<{ id: string; status: string }>;
    };

    expect(summary.sandbox).toBe(true);
    expect(summary.signal_quality.session_quality_score).toBe(8.7);
    expect(Object.keys(summary.gates)).toEqual(['coverage', 'agreement', 'motion', 'reference']);
    expect(preflight.ok).toBe(true);
    expect(preflight.checks).toHaveLength(6);
    expect(service.request('/api/unimplemented')).toEqual({ ok: true, sandbox: true });
  });

  it('mirrors locked protocol, persisted participant randomization, and reference gates', () => {
    const first = service.request('/api/study/schedule?participant_id=P-001') as {
      participant_id: string;
      entries: Array<{ condition_id: string }>;
    };
    const second = service.request('/api/study/schedule?participant_id=P-001') as typeof first;
    expect(first.participant_id).toBe('P-001');
    expect(first.entries.map(item => item.condition_id)).toEqual(second.entries.map(item => item.condition_id));

    service.request('/api/study/protocol', {
      method: 'PUT',
      body: JSON.stringify({ state: 'locked', actor: 'OP-001' })
    });
    expect(() => service.request('/api/study/protocol', {
      method: 'PUT',
      body: JSON.stringify({ state: 'draft' })
    })).toThrow('locked');

    expect(() => service.request('/api/sessions/sandbox_test/references/rr-adjudication', {
      method: 'POST',
      body: JSON.stringify({ final_value: 15, rationale: 'not enough observers' })
    })).toThrow('two locked observer');
    for (const observer_id of ['OBS-A', 'OBS-B']) {
      service.request('/api/sessions/sandbox_test/references', {
        method: 'POST',
        body: JSON.stringify({ kind: 'rr_observer', observer_id, value: 15, unit: 'br/min' })
      });
    }
    const adjudicated = service.request('/api/sessions/sandbox_test/references/rr-adjudication', {
      method: 'POST',
      body: JSON.stringify({ final_value: 15, rationale: 'dual observer' })
    }) as { rr_adjudication: { final_value: number } };
    expect(adjudicated.rr_adjudication.final_value).toBe(15);
  });
});
