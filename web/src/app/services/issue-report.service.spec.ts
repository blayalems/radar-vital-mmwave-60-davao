import { TestBed } from '@angular/core/testing';
import { provideHttpClient } from '@angular/common/http';
import { provideHttpClientTesting } from '@angular/common/http/testing';
import { signal } from '@angular/core';

import { IssueReportService, IssueReport } from './issue-report.service';
import { ApiService } from './api.service';
import { StateService } from './state.service';
import { ServerLifecycleService } from './server-lifecycle.service';
import { PersistenceService } from './persistence.service';
import { DIAGNOSTICS_OPTIN_KEY } from './rvt-storage-keys';
import { GITHUB_REPO_URL } from './app-meta';

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------

function buildMockApi(versionPayload: Record<string, unknown> = { product_version: '16.3.0' }) {
  return {
    request: vi.fn().mockResolvedValue(versionPayload),
    currentApiBase: vi.fn(() => ''),
    setApiBase: vi.fn(),
    checkConnection: vi.fn().mockResolvedValue(true),
    detectControlMode: vi.fn().mockResolvedValue(true),
    enableSandboxControlMode: vi.fn(),
    connectionLoading: signal(false),
  };
}

function buildMockState(overrides: Partial<{
  ctlStatus: ReturnType<typeof vi.fn>;
  alertHistory: ReturnType<typeof vi.fn>;
  demoMode: ReturnType<typeof vi.fn>;
  autoDemoActive: ReturnType<typeof vi.fn>;
}> = {}) {
  return {
    ctlStatus: overrides.ctlStatus ?? vi.fn(() => ({ ok: true, mode: 'live', latency: 42, error: null })),
    alertHistory: overrides.alertHistory ?? vi.fn(() => []),
    demoMode: overrides.demoMode ?? vi.fn(() => false),
    autoDemoActive: overrides.autoDemoActive ?? vi.fn(() => false),
  };
}

function buildMockLifecycle(platform: 'exe' | 'remote' = 'remote', logLines: string[] = []) {
  return {
    platform: vi.fn(() => platform),
    logTail: vi.fn(() => logLines),
    status: signal('running' as const),
    lastError: signal(null as string | null),
  };
}

function configure(
  api = buildMockApi(),
  state = buildMockState(),
  lifecycle = buildMockLifecycle(),
) {
  const mockPersistence = {
    quarantineLegacyLocalStorage: vi.fn().mockResolvedValue(undefined),
    get: vi.fn().mockResolvedValue(undefined),
    put: vi.fn().mockResolvedValue(undefined),
  };

  TestBed.configureTestingModule({
    providers: [
      provideHttpClient(),
      provideHttpClientTesting(),
      IssueReportService,
      { provide: ApiService, useValue: api },
      { provide: StateService, useValue: state },
      { provide: ServerLifecycleService, useValue: lifecycle },
      { provide: PersistenceService, useValue: mockPersistence },
    ],
  });

  return TestBed.inject(IssueReportService);
}

// ---------------------------------------------------------------------------
// Suite
// ---------------------------------------------------------------------------

describe('IssueReportService', () => {
  beforeEach(() => {
    localStorage.clear();
    // Remove any Tauri/Capacitor window bridges so platform detection is clean.
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    delete (window as Window & { Capacitor?: unknown }).Capacitor;
  });

  afterEach(() => {
    TestBed.resetTestingModule();
    localStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    delete (window as Window & { Capacitor?: unknown }).Capacitor;
  });

  // ---- creation ------------------------------------------------------------

  it('should be created', () => {
    const service = configure();
    expect(service).toBeTruthy();
  });

  // ---- diagnostics toggle --------------------------------------------------

  it('defaults diagnosticsEnabled to TRUE when key absent', () => {
    localStorage.removeItem(DIAGNOSTICS_OPTIN_KEY);
    const service = configure();
    expect(service.diagnosticsEnabled()).toBe(true);
  });

  it('reads stored "1" as enabled', () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure();
    expect(service.diagnosticsEnabled()).toBe(true);
  });

  it('reads stored "0" as disabled', () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    expect(service.diagnosticsEnabled()).toBe(false);
  });

  it('persists setDiagnosticsEnabled(false) to localStorage', () => {
    const service = configure();
    service.setDiagnosticsEnabled(false);
    expect(localStorage.getItem(DIAGNOSTICS_OPTIN_KEY)).toBe('0');
    expect(service.diagnosticsEnabled()).toBe(false);
  });

  it('persists setDiagnosticsEnabled(true) to localStorage', () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    service.setDiagnosticsEnabled(true);
    expect(localStorage.getItem(DIAGNOSTICS_OPTIN_KEY)).toBe('1');
    expect(service.diagnosticsEnabled()).toBe(true);
  });

  // ---- buildReport() opt-out payload minimization -------------------------

  it('when opt-out, buildReport() returns only version + platform', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    const report = await service.buildReport();
    expect(report.product_version).toBe('16.3.0');
    expect(report.platform).toBe('PWA (browser)');
    // No diagnostics fields when opted out.
    expect(report.connection_mode).toBeUndefined();
    expect(report.ctl).toBeUndefined();
    expect(report.alerts).toBeUndefined();
    expect(report.log_tail).toBeUndefined();
  });

  it('when opt-in, buildReport() includes connection_mode, ctl, alerts', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure();
    const report = await service.buildReport();
    expect(report.connection_mode).toBeDefined();
    expect(report.ctl).toBeDefined();
    expect(report.alerts).toBeDefined();
  });

  it('de-identifies alerts — only counts and last5 with age_s, no message text', async () => {
    const now = Date.now();
    const state = buildMockState({
      alertHistory: vi.fn(() => [
        { severity: 'info', msg: 'Private operator message', source: 'ctl', ts: now - 5000, id: 'a1', dismissed: false },
        { severity: 'warn', msg: 'Another private note', source: 'radar', ts: now - 10000, id: 'a2', dismissed: false },
        { severity: 'critical', msg: 'Critical private data', source: 'firmware', ts: now - 15000, id: 'a3', dismissed: false },
      ]),
    });
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure(buildMockApi(), state);
    const report = await service.buildReport();

    expect(report.alerts?.counts).toEqual({ info: 1, warn: 1, critical: 1 });
    expect(report.alerts?.last5).toHaveLength(3);
    for (const entry of report.alerts!.last5) {
      // Must not contain msg.
      expect((entry as unknown as Record<string, unknown>)['msg']).toBeUndefined();
      expect(typeof entry.age_s).toBe('number');
    }
  });

  // ---- platform branching --------------------------------------------------

  it('detects Tauri desktop as "Windows EXE"', async () => {
    (window as Window & { __TAURI__?: unknown }).__TAURI__ = {
      core: { invoke: vi.fn() },
    };
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    const report = await service.buildReport();
    expect(report.platform).toBe('Windows EXE');
  });

  it('detects Capacitor native as "Android APK"', async () => {
    (window as Window & { Capacitor?: unknown }).Capacitor = {
      isNativePlatform: () => true,
    };
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    const report = await service.buildReport();
    expect(report.platform).toBe('Android APK');
  });

  it('falls back to "PWA (browser)" when neither Tauri nor Capacitor is present', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    const report = await service.buildReport();
    expect(report.platform).toBe('PWA (browser)');
  });

  // ---- log tail collection — EXE vs remote --------------------------------

  it('collects EXE log tail from serverLifecycle.logTail() and limits to last 20', async () => {
    const lines = Array.from({ length: 30 }, (_, i) => `line ${i + 1}`);
    const lifecycle = buildMockLifecycle('exe', lines);
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure(buildMockApi(), buildMockState(), lifecycle);
    const report = await service.buildReport();
    expect(report.log_tail).toHaveLength(20);
    expect(report.log_tail![0]).toBe('line 11');
    expect(report.log_tail![19]).toBe('line 30');
  });

  it('collects remote log tail from /api/trainer/log for non-exe platform', async () => {
    const remoteLines = Array.from({ length: 25 }, (_, i) => `remote ${i + 1}`);
    const api = buildMockApi({ product_version: '16.3.0' });
    api.request
      .mockResolvedValueOnce({ product_version: '16.3.0' }) // /api/version
      .mockResolvedValueOnce({ lines: remoteLines });          // /api/trainer/log
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure(api, buildMockState(), buildMockLifecycle('remote'));
    const report = await service.buildReport();
    expect(report.log_tail).toHaveLength(20);
    expect(report.log_tail![0]).toBe('remote 6');
  });

  // ---- buildIssueUrl() field-by-field -------------------------------------

  it('URL starts with GITHUB_REPO_URL/issues/new and contains template=bug_report.yml', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    const report = await service.buildReport();
    const url = service.buildIssueUrl(report);
    expect(url).toMatch(new RegExp(`^${GITHUB_REPO_URL}/issues/new`));
    const parsed = new URL(url);
    expect(parsed.searchParams.get('template')).toBe('bug_report.yml');
  });

  it('prefills product_version and platform fields', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    const report = await service.buildReport();
    const url = service.buildIssueUrl(report);
    const parsed = new URL(url);
    expect(parsed.searchParams.get('product_version')).toBe('16.3.0');
    expect(parsed.searchParams.get('platform')).toBe('PWA (browser)');
  });

  it('prefills description with userText when supplied', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    const report = await service.buildReport();
    const url = service.buildIssueUrl(report, 'The radar freezes');
    const parsed = new URL(url);
    expect(parsed.searchParams.get('description')).toBe('The radar freezes');
  });

  it('prefills connection_mode when diagnostics are enabled', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure();
    const report = await service.buildReport();
    const url = service.buildIssueUrl(report);
    const parsed = new URL(url);
    expect(parsed.searchParams.get('connection_mode')).toBeTruthy();
  });

  it('omits connection_mode when diagnostics are disabled', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    const report = await service.buildReport();
    const url = service.buildIssueUrl(report);
    const parsed = new URL(url);
    expect(parsed.searchParams.get('connection_mode')).toBeNull();
  });

  it('diagnostics field is not set when opt-out', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '0');
    const service = configure();
    const report = await service.buildReport();
    const url = service.buildIssueUrl(report);
    const parsed = new URL(url);
    expect(parsed.searchParams.get('diagnostics')).toBeNull();
  });

  it('diagnostics field is set when opt-in', async () => {
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure();
    const report = await service.buildReport();
    const url = service.buildIssueUrl(report);
    const parsed = new URL(url);
    expect(parsed.searchParams.get('diagnostics')).not.toBeNull();
  });

  // ---- 7 500-char truncation order -----------------------------------------

  it('does not exceed MAX_URL 7500 chars when log tail is huge', async () => {
    const bigLines = Array.from({ length: 20 }, () => 'x'.repeat(500));
    const lifecycle = buildMockLifecycle('exe', bigLines);
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure(buildMockApi(), buildMockState(), lifecycle);
    const report = await service.buildReport();
    const url = service.buildIssueUrl(report);
    expect(url.length).toBeLessThanOrEqual(7500);
  });

  it('drops log_tail before alerts when truncating', async () => {
    // Build a report manually with a huge log tail.
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure();

    const hugeLogTail = Array.from({ length: 20 }, () => 'a'.repeat(500));
    const hugeAlerts = {
      counts: { info: 10, warn: 5, critical: 2 },
      last5: [
        { severity: 'warn', source: 'ctl', age_s: 10 },
        { severity: 'warn', source: 'ctl', age_s: 20 },
        { severity: 'warn', source: 'ctl', age_s: 30 },
        { severity: 'warn', source: 'ctl', age_s: 40 },
        { severity: 'warn', source: 'ctl', age_s: 50 },
      ],
    };

    const report: IssueReport = {
      product_version: '16.3.0',
      platform: 'PWA (browser)',
      connection_mode: 'LAN paired (phone/PWA to trainer)',
      ctl: { ok: true, mode: 'live', latency: 42, error: undefined },
      alerts: hugeAlerts,
      log_tail: hugeLogTail,
    };

    const url = service.buildIssueUrl(report);
    expect(url.length).toBeLessThanOrEqual(7500);

    const params = new URL(url).searchParams;
    const diag = params.get('diagnostics');
    if (diag) {
      const parsed = JSON.parse(diag) as Record<string, unknown>;
      // log_tail must be gone if alerts are still present; or both gone
      expect(parsed['log_tail']).toBeUndefined();
    }
  });

  it('always keeps product_version and platform under any truncation', async () => {
    // All fields huge — drop everything but still keep version/platform.
    localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, '1');
    const service = configure();

    const report: IssueReport = {
      product_version: '16.3.0',
      platform: 'PWA (browser)',
      connection_mode: 'LAN paired (phone/PWA to trainer)',
      ctl: { ok: true, mode: 'x'.repeat(5000), latency: 42, error: 'x'.repeat(5000) },
      alerts: {
        counts: { info: 0, warn: 0, critical: 0 },
        last5: Array.from({ length: 5 }, () => ({ severity: 'warn', source: 'x'.repeat(1000), age_s: 1 })),
      },
      log_tail: Array.from({ length: 20 }, () => 'z'.repeat(500)),
    };

    const url = service.buildIssueUrl(report);
    expect(url.length).toBeLessThanOrEqual(7500);
    const params = new URL(url).searchParams;
    expect(params.get('product_version')).toBe('16.3.0');
    expect(params.get('platform')).toBe('PWA (browser)');
  });

  // ---- openReport() --------------------------------------------------------

  it('calls window.open with _blank and noopener on non-Tauri', async () => {
    const openSpy = vi.spyOn(window, 'open').mockReturnValue(null);
    const service = configure();
    await service.openReport('https://github.com/test/issues/new');
    expect(openSpy).toHaveBeenCalledWith('https://github.com/test/issues/new', '_blank', 'noopener');
    openSpy.mockRestore();
  });

  it('calls tauri invoke plugin:shell|open on Tauri context', async () => {
    const invoke = vi.fn().mockResolvedValue(undefined);
    (window as Window & { __TAURI__?: unknown }).__TAURI__ = { core: { invoke } };

    const service = configure();
    await service.openReport('https://github.com/test/issues/new');
    expect(invoke).toHaveBeenCalledWith('plugin:shell|open', { path: 'https://github.com/test/issues/new' });
  });

  it('falls back to window.open when tauri invoke throws', async () => {
    const invoke = vi.fn().mockRejectedValue(new Error('tauri error'));
    (window as Window & { __TAURI__?: unknown }).__TAURI__ = { core: { invoke } };
    const openSpy = vi.spyOn(window, 'open').mockReturnValue(null);

    const service = configure();
    await service.openReport('https://github.com/test/issues/new');
    expect(openSpy).toHaveBeenCalled();
    openSpy.mockRestore();
  });
});
