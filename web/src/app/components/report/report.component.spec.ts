import { ChangeDetectorRef, signal } from '@angular/core';
import { TestBed } from '@angular/core/testing';
import { MatSnackBar } from '@angular/material/snack-bar';

import { SessionSignoff } from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';
import { StateService } from '../../services/state.service';
import { ReportComponent } from './report.component';
import { buildFallbackObjectiveExport, extractObjectiveEvidenceLines } from './report-thesis-helpers';

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

  it('normalizes missing validation_comment and reads manifest.tags fallback on session load', async () => {
    request.mockImplementation((path: string) => {
      if (path === '/api/sessions/A/summary') {
        return Promise.resolve({ session_id: 'A', manifest: { tags: ['cardboard', 'stable'] } });
      }
      if (path === '/api/sessions/A/data?points=1000') return Promise.resolve({ rows: [] });
      if (path === '/api/sessions/A/compare') return Promise.resolve({});
      if (path === '/api/sessions/A/analyse/status') return Promise.resolve({ status: 'completed' });
      if (path === '/api/sessions/A/notes') return Promise.resolve({ session_id: 'A', review_summary: '' });
      if (path === '/api/sessions/A/signoff') return Promise.resolve({ session_id: 'A', operator_name: 'Alice', initials: 'AA' });
      if (path === '/api/sessions/A/references') return Promise.resolve({ session_id: 'A', references: [] });
      return Promise.resolve({});
    });

    component.selectedSessionId = 'A';
    await component.onSessionChange();

    expect(component.signoff.validation_comment).toBe('');
    expect(component.sessionTagsInput).toBe('cardboard, stable');
  });

  it('keeps the first rr_observer entry blinded until a second distinct observer locks, then unblinds and adjudicates', async () => {
    component.selectedSessionId = 'A';
    component.selectedSummary = { session_id: 'A', condition_id: 'd060_cardboard', distance_m: 0.6, barrier_type: 'cardboard', duration_s: 150 };

    request.mockImplementation((path: string, init?: RequestInit) => {
      if (path === '/api/sessions/A/references' && init?.method === 'POST') {
        const body = JSON.parse(String(init.body));
        if (body.observer_id === 'Obs-A') {
          return Promise.resolve({
            session_id: 'A',
            references: [{ observation_id: 'r1', session_id: 'A', kind: 'rr_observer', observer_id: 'Obs-A', value: 16, unit: 'br/min', locked: true }]
          });
        }
        return Promise.resolve({
          session_id: 'A',
          references: [
            { observation_id: 'r1', session_id: 'A', kind: 'rr_observer', observer_id: 'Obs-A', value: 16, unit: 'br/min', locked: true },
            { observation_id: 'r2', session_id: 'A', kind: 'rr_observer', observer_id: 'Obs-B', value: 18, unit: 'br/min', locked: true }
          ]
        });
      }
      if (path === '/api/sessions/A/references/rr-adjudication') {
        const body = JSON.parse(String(init?.body));
        return Promise.resolve({
          session_id: 'A',
          references: component.referencesSignal(),
          rr_adjudication: { final_value: body.final_value, rationale: body.rationale }
        });
      }
      return Promise.resolve({});
    });

    // Observer A locks 16 br/min -> masked
    component.rrObserverIdInput.set('Obs-A');
    component.rrRateInput.set('16');
    await component.submitRrObserverObservation();
    expect(component.rrObserverSummary().lockedCount).toBe(1);
    expect(component.rrObserverSummary().unblinded).toBe(false);
    expect(component.rrObserverSummary().entries[0].displayValue).toBe('Locked (blinded)');

    // Duplicate Observer A rejected on client
    component.rrObserverIdInput.set('obs-a');
    component.rrRateInput.set('17');
    await component.submitRrObserverObservation();
    expect(component.rrObserverSummary().lockedCount).toBe(1);

    // Observer B uses full-trial breath count (45 breaths / 150 s = 18.00 br/min) -> unblinds
    component.rrObserverIdInput.set('Obs-B');
    component.rrEntryMode.set('count');
    component.rrBreathCountInput.set('45');
    component.rrTrialDurationInput.set('150');
    expect(component.computedBreathCountRate()).toBe(18);
    await component.submitRrObserverObservation();

    expect(component.rrObserverSummary().lockedCount).toBe(2);
    expect(component.rrObserverSummary().unblinded).toBe(true);
    expect(component.rrObserverSummary().discrepancyBpm).toBe(2);
    expect(component.rrObserverSummary().exceedsDiscrepancyThreshold).toBe(true);
    expect(component.rrObserverSummary().suggestedMeanBpm).toBe(17);

    // Temperature is blocked for cardboard
    expect(component.isD060NoneAllowed()).toBe(false);

    // Adjudicate final RR
    component.adjudicationFinalValueInput.set('17');
    component.adjudicationRationaleInput.set('Consensus review of thoracic trace');
    await component.adjudicateRr();
    expect(component.rrObserverSummary().adjudicatedFinalValue).toBe(17);
  });

  it('allows temperature observation only for d060_none and exports IEEE LaTeX/CSV truthfully', async () => {
    component.selectedSessionId = 'A';
    component.selectedSummary = { session_id: 'A', condition_id: 'd060_none', distance_m: 0.6, barrier_type: 'none' };
    expect(component.isD060NoneAllowed()).toBe(true);

    const exportedBlobs: string[] = [];
    vi.spyOn(URL, 'createObjectURL').mockReturnValue('blob:mock');
    vi.spyOn(URL, 'revokeObjectURL').mockImplementation(() => {});
    const origCreateElement = document.createElement.bind(document);
    vi.spyOn(document, 'createElement').mockImplementation((tag: string) => {
      const el = origCreateElement(tag);
      if (tag === 'a') {
        vi.spyOn(el as HTMLAnchorElement, 'click').mockImplementation(() => {
          exportedBlobs.push((el as HTMLAnchorElement).download);
        });
      }
      return el;
    });

    request.mockImplementation((path: string) => {
      if (path.includes('/api/study/objectives/objective_1_rr/export?format=latex')) {
        return Promise.resolve({
          ok: true,
          filename: 'objective_1_rr_ieee_table.tex',
          content_type: 'application/x-tex',
          content: '\\begin{table}[!t]\\end{table}'
        });
      }
      if (path.includes('/api/study/objectives/objective_1_rr/export?format=csv')) {
        return Promise.resolve({
          ok: true,
          filename: 'objective_1_rr_conditions.csv',
          content_type: 'text/csv',
          content: 'objective_id,condition_id,status\nobjective_1_rr,d060_none,inconclusive\n'
        });
      }
      return Promise.resolve({});
    });

    await component.exportObjective({ objective_id: 'objective_1_rr', status: 'inconclusive' }, 'latex', false);
    await component.exportObjective({ objective_id: 'objective_1_rr', status: 'inconclusive' }, 'csv', false);
    expect(exportedBlobs).toEqual(['objective_1_rr_ieee_table.tex', 'objective_1_rr_conditions.csv']);
  });

  it('extracts nested statistical_report and false-alarm evidence truthfully and fails closed when unready', () => {
    const readyStatsPayload = {
      denominators: { n_participants: 38 },
      coverage: { coverage: 0.962, n_attempted: 1140, n_with_output: 1097 },
      participant_balanced_metrics: { bias_estimate_minus_reference: 0.14, rmse: 1.05, mae: 0.82 },
      agreement: { loa_lower: -1.85, loa_upper: 2.13 },
      condition_tost: {
        primary: {
          condition_id: 'd100_none',
          status: 'tested',
          n_participants: 38,
          mean_difference: 0.14,
          ci_low: -0.32,
          ci_high: 0.60,
          decision: 'equivalent'
        },
        secondary: []
      },
      condition_metrics: {
        d100_none: {
          condition_id: 'd100_none',
          n_participants: 38,
          valid_output_rate_pct: 97.4,
          bias_estimate_minus_reference: 0.14,
          rmse: 0.98,
          mae: 0.76,
          loa_lower: -1.70,
          loa_upper: 1.98
        }
      }
    };

    const readyLines = extractObjectiveEvidenceLines({
      objective_id: 'objective_1_rr',
      status: 'ready',
      report: { statistical_report: readyStatsPayload }
    });
    expect(readyLines).toEqual([
      { label: 'Participants (n)', value: '38' },
      { label: 'Valid output', value: '96.2%' },
      { label: 'Bias / RMSE / MAE', value: '0.14 / 1.05 / 0.82' },
      { label: 'BA 95% LoA', value: '[-1.85, 2.13]' },
      { label: '90% TOST CI / Holm', value: '[-0.32, 0.60] · equivalent' }
    ]);

    const unreadyLines = extractObjectiveEvidenceLines({
      objective_id: 'objective_1_rr',
      status: 'inconclusive',
      report: { statistical_report: readyStatsPayload }
    });
    expect(unreadyLines).toEqual([
      { label: 'Participants (n)', value: '--' },
      { label: 'Valid output', value: '--' },
      { label: 'Bias / RMSE / MAE', value: '-- / -- / --' },
      { label: 'BA 95% LoA', value: '--' },
      { label: '90% TOST CI / Holm', value: 'inconclusive' }
    ]);

    const readyCsv = buildFallbackObjectiveExport({
      objective_id: 'objective_1_rr',
      status: 'ready',
      report: { statistical_report: readyStatsPayload }
    }, 'csv', true);
    expect(readyCsv.content).toContain('# SANDBOX DEMO DATA');
    expect(readyCsv.content).toContain('"d100_none","1.0","none","primary","38","97.4","0.14","0.98","0.76","-1.70","1.98","-0.32","0.60"');

    const readyTex = buildFallbackObjectiveExport({
      objective_id: 'objective_1_rr',
      status: 'ready',
      report: { statistical_report: readyStatsPayload }
    }, 'latex', false);
    expect(readyTex.content).toContain('d100\\_none & 38 & 97.4 & 0.14 & 0.98 & 0.76 & [-1.70, 1.98] & [-0.32, 0.60] & equivalent \\\\');

    const readyFaReport = {
      objective_id: 'objective_3_false_alarm',
      status: 'ready',
      report: {
        planned_trial_count: 72,
        qualified_trial_count: 72,
        false_alarm_trial_count: 1,
        observed_false_alarm_rate: 1 / 72,
        confidence_interval_95: { low: 0.00036, high: 0.07504 },
        exact_binomial_test: {
          p_value: 0.11923,
          decision: 'does_not_support_false_alarm_rate_below_threshold'
        }
      }
    };
    const faLines = extractObjectiveEvidenceLines(readyFaReport);
    expect(faLines).toEqual([
      { label: 'False alarms', value: '1 / 72 (1.39%)' },
      { label: '95% Clopper-Pearson CI', value: '[0.04%, 7.50%]' },
      { label: 'Outcome', value: 'does_not_support_false_alarm_rate_below_threshold' }
    ]);

    const faCsv = buildFallbackObjectiveExport(readyFaReport, 'csv', false);
    expect(faCsv.content).toContain('"objective_3_false_alarm","ready","72","72","1","1.39","0.04","7.50","0.1192","does_not_support_false_alarm_rate_below_threshold"');

    const faTex = buildFallbackObjectiveExport({
      objective_id: 'objective_3_false_alarm',
      status: 'inconclusive',
      report: null
    }, 'latex', false);
    expect(faTex.content).toContain('No-subject false alarm & -- & -- & -- & -- \\\\');
  });
});
