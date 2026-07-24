import { ChangeDetectorRef, signal } from '@angular/core';
import { TestBed } from '@angular/core/testing';
import { MatSnackBar } from '@angular/material/snack-bar';

import { SessionSignoff } from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';
import { StateService } from '../../services/state.service';
import { ReportComponent } from './report.component';

interface Deferred<T> {
  promise: Promise<T>;
  resolve(value: T): void;
  reject(reason: unknown): void;
}

function deferred<T>(): Deferred<T> {
  let resolve!: (value: T) => void;
  let reject!: (reason: unknown) => void;
  const promise = new Promise<T>((resolvePromise, rejectPromise) => {
    resolve = resolvePromise;
    reject = rejectPromise;
  });
  return { promise, resolve, reject };
}

function sessionResponses(sessionId: string) {
  return {
    summary: { session_id: sessionId, summary: `${sessionId} summary` },
    data: { rows: [{ reported_hr: sessionId === 'A' ? 61 : 72 }] },
    compare: { selected: { session_id: sessionId } },
    status: { status: `${sessionId} complete` },
    notes: { session_id: sessionId, review_summary: `${sessionId} notes` },
    signoff: {
      session_id: sessionId,
      operator_name: `${sessionId} operator`,
      initials: sessionId,
      validation_comment: `${sessionId} valid`,
      signed_at: null
    }
  };
}

describe('ReportComponent request ownership', () => {
  let component: ReportComponent;
  let request: ReturnType<typeof vi.fn>;
  let state: {
    theme: ReturnType<typeof signal<string>>;
    currentSessionId: ReturnType<typeof signal<string | null>>;
    sessionItems: ReturnType<typeof signal<unknown[]>>;
    sessionNotes: ReturnType<typeof signal<Record<string, string>>>;
    sessionSignoffs: ReturnType<typeof signal<Record<string, SessionSignoff>>>;
    triggerHaptic: ReturnType<typeof vi.fn>;
  };

  beforeEach(() => {
    request = vi.fn();
    state = {
      theme: signal('light'),
      currentSessionId: signal<string | null>(null),
      sessionItems: signal<unknown[]>([]),
      sessionNotes: signal<Record<string, string>>({}),
      sessionSignoffs: signal<Record<string, SessionSignoff>>({}),
      triggerHaptic: vi.fn()
    };
    TestBed.configureTestingModule({
      providers: [
        { provide: ApiService, useValue: { request } },
        { provide: StateService, useValue: state },
        { provide: ChangeDetectorRef, useValue: { markForCheck: vi.fn() } },
        { provide: MatSnackBar, useValue: { open: vi.fn() } }
      ]
    });
    component = TestBed.runInInjectionContext(() => new ReportComponent());
    component.sessions = [{ session_id: 'A' }, { session_id: 'B' }];
  });

  afterEach(() => {
    TestBed.resetTestingModule();
    vi.restoreAllMocks();
  });

  it('keeps the newest primary session when A resolves after B', async () => {
    const pending = new Map<string, Deferred<unknown>>();
    request.mockImplementation((path: string) => {
      const gate = deferred<unknown>();
      pending.set(path, gate);
      return gate.promise;
    });

    component.selectedSessionId = 'A';
    const loadA = component.onSessionChange();
    component.selectedSessionId = 'B';
    const loadB = component.onSessionChange();

    const b = sessionResponses('B');
    pending.get('/api/sessions/B/summary')!.resolve(b.summary);
    pending.get('/api/sessions/B/data?points=1000')!.resolve(b.data);
    pending.get('/api/sessions/B/compare')!.resolve(b.compare);
    pending.get('/api/sessions/B/analyse/status')!.resolve(b.status);
    pending.get('/api/sessions/B/notes')!.resolve(b.notes);
    pending.get('/api/sessions/B/signoff')!.resolve(b.signoff);
    await loadB;

    const a = sessionResponses('A');
    pending.get('/api/sessions/A/summary')!.resolve(a.summary);
    pending.get('/api/sessions/A/data?points=1000')!.resolve(a.data);
    pending.get('/api/sessions/A/compare')!.resolve(a.compare);
    pending.get('/api/sessions/A/analyse/status')!.resolve(a.status);
    pending.get('/api/sessions/A/notes')!.resolve(a.notes);
    pending.get('/api/sessions/A/signoff')!.resolve(a.signoff);
    await loadA;

    expect(component.selectedSessionId).toBe('B');
    expect(component.selectedSummary?.session_id).toBe('B');
    expect(component.sessionDataRows).toEqual(b.data.rows);
    expect(component.sessionNotesInput).toBe('B notes');
    expect(component.signoff.session_id).toBe('B');
    expect(component.summaryLoading).toBe(false);
  });

  it('keeps the newest comparison when A resolves after B', async () => {
    const pending = new Map<string, Deferred<unknown>>();
    request.mockImplementation((path: string) => {
      const gate = deferred<unknown>();
      pending.set(path, gate);
      return gate.promise;
    });

    const loadA = component.loadCompareSession('A');
    const loadB = component.loadCompareSession('B');

    pending.get('/api/sessions/B/summary')!.resolve({ session_id: 'B' });
    pending.get('/api/sessions/B/data?points=1000')!.resolve({ rows: [{ reported_hr: 72 }] });
    await loadB;
    pending.get('/api/sessions/A/summary')!.resolve({ session_id: 'A' });
    pending.get('/api/sessions/A/data?points=1000')!.resolve({ rows: [{ reported_hr: 61 }] });
    await loadA;

    expect(component.compareSessionId).toBe('B');
    expect(component.compareSummary?.session_id).toBe('B');
    expect(component.compareRows).toEqual([{ reported_hr: 72 }]);
    expect(component.compareLoading).toBe(false);
  });

  it('saves notes against the immutable session and text captured before await', async () => {
    const save = deferred<{ session_id: string; review_summary: string }>();
    request.mockReturnValue(save.promise);
    component.selectedSessionId = 'A';
    component.sessionNotesInput = 'A review';

    const pendingSave = component.saveReportNotes();
    component.selectedSessionId = 'B';
    component.sessionNotesInput = 'B draft';
    save.resolve({ session_id: 'A', review_summary: 'A review' });
    await pendingSave;

    expect(request).toHaveBeenCalledWith('/api/sessions/A/notes', expect.objectContaining({
      body: JSON.stringify({ review_summary: 'A review' })
    }));
    expect(state.sessionNotes()).toEqual({ A: 'A review' });
  });

  it('stores sign-off under the captured session without replacing the newly selected form', async () => {
    const save = deferred<SessionSignoff>();
    request.mockReturnValue(save.promise);
    component.selectedSessionId = 'A';
    component.signoff = {
      session_id: 'A',
      operator_name: 'Alice',
      initials: 'aa',
      validation_comment: 'A approved'
    };

    const pendingSave = component.saveSignoff();
    component.selectedSessionId = 'B';
    component.signoff = {
      session_id: 'B',
      operator_name: 'Bob',
      initials: 'BB',
      validation_comment: 'B draft'
    };
    save.resolve({
      session_id: 'A',
      operator_name: 'Alice',
      initials: 'AA',
      validation_comment: 'A approved',
      signed_at: '2026-07-17T00:00:00Z'
    });
    await pendingSave;

    expect(request).toHaveBeenCalledWith('/api/sessions/A/signoff', expect.objectContaining({
      body: JSON.stringify({
        operator_name: 'Alice',
        initials: 'AA',
        validation_comment: 'A approved'
      })
    }));
    expect(state.sessionSignoffs()['A']).toMatchObject({ session_id: 'A', initials: 'AA' });
    expect(component.signoff).toMatchObject({ session_id: 'B', operator_name: 'Bob' });
  });
});
