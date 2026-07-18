import { TestBed } from '@angular/core/testing';
import { provideHttpClient } from '@angular/common/http';
import { HttpTestingController, provideHttpClientTesting } from '@angular/common/http/testing';
import { ApiService } from './api.service';
import { StateService } from './state.service';
import { PersistenceService } from './persistence.service';
import { OPERATOR_TOKEN_KEY, SERVER_URL_KEY } from './rvt-storage-keys';

describe('ApiService', () => {
  let service: ApiService;
  let state: StateService;
  let mockPersistence: Partial<PersistenceService>;
  let httpMock: HttpTestingController;

  beforeEach(() => {
    localStorage.clear();
    sessionStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    delete (window as Window & { Capacitor?: unknown }).Capacitor;
    mockPersistence = {
      quarantineLegacyLocalStorage: vi.fn().mockResolvedValue(undefined),
      get: vi.fn().mockResolvedValue(undefined),
      put: vi.fn().mockResolvedValue(undefined)
    };

    TestBed.configureTestingModule({
      providers: [
        provideHttpClient(),
        provideHttpClientTesting(),
        ApiService,
        StateService,
        { provide: PersistenceService, useValue: mockPersistence }
      ]
    });

    state = TestBed.inject(StateService);
    service = TestBed.inject(ApiService);
    httpMock = TestBed.inject(HttpTestingController);
  });

  afterEach(() => {
    for (const request of httpMock.match(() => true)) {
      request.flush({ ok: true, mode: 'sandbox' });
    }
    localStorage.clear();
    sessionStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    delete (window as Window & { Capacitor?: unknown }).Capacitor;
    vi.unstubAllGlobals();
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should support sandbox fallback when trainer is unreachable', async () => {
    // Force sandbox mode
    service.enableSandboxControlMode('Connection timed out');
    expect(state.autoDemoActive()).toBe(true);
    expect(state.ctlStatus()?.mode).toBe('sandbox');
  });

  it('preserves a real active session when sandbox fallback is requested', () => {
    state.ctlStatus.set({ ok: true, mode: 'live' });
    state.sessionActive.set(true);
    state.currentSessionId.set('session-live-1');

    expect(service.enableSandboxControlMode('Connection timed out')).toBe(false);
    expect(state.autoDemoActive()).toBe(false);
    expect(state.sessionActive()).toBe(true);
    expect(state.currentSessionId()).toBe('session-live-1');
    expect(state.ctlStatus()).toMatchObject({
      ok: false,
      mode: 'live',
      reason: 'Connection timed out'
    });
  });

  it('never sandbox-routes Stop for a real active session', async () => {
    state.ctlStatus.set({ ok: true, mode: 'live' });
    // Defense in depth for stale UI code or restored preferences that bypass
    // the guarded source-mode setter.
    expect(state.trySetDemoMode(true)).toBe(true);
    state.sessionActive.set(true);
    state.currentSessionId.set('session-live-1');

    const pending = service.request<{ ok: boolean }>('/api/session/stop', { method: 'POST' });
    const request = httpMock.expectOne('/api/session/stop');
    expect(state.sessionActive()).toBe(true);
    request.flush({ ok: true });

    await expect(pending).resolves.toEqual({ ok: true });
  });

  it('should serve mock defaults in sandbox mode', async () => {
    service.enableSandboxControlMode('Sandbox mode');
    const defaults = await service.request<{ sandbox: boolean }>('/api/defaults');
    expect(defaults.sandbox).toBe(true);
  });

  it('should serve mock preflight checks in sandbox mode', async () => {
    service.enableSandboxControlMode('Sandbox mode');
    const preflight = await service.request<{ ok: boolean; checks: any[] }>('/api/preflight');
    expect(preflight.ok).toBe(true);
    expect(preflight.checks.length).toBeGreaterThan(0);
  });

  it('should manage API base and store it in localStorage', () => {
    const base = 'http://192.168.1.100:8765';
    service.setApiBase(base);
    expect(service.currentApiBase()).toBe(base);

    service.setApiBase('');
    expect(service.currentApiBase()).toBe('');
  });

  it('should store and retrieve pair tokens', () => {
    const token = 'pairing-token-abc-123';
    service.setPairToken(token);
    expect(service.pairToken()).toBe(token);
    expect(service.hasPairToken()).toBe(true);

    service.setPairToken('');
    expect(service.pairToken()).toBe('');
    expect(service.hasPairToken()).toBe(false);
  });

  it('attaches the operator token to browser API requests', async () => {
    for (const request of httpMock.match('/api/status')) {
      request.flush({ ok: true, mode: 'live' });
    }
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'operator-token-123');

    const pending = service.request<{ ok: boolean }>('/api/status', undefined, true);
    const request = httpMock.expectOne('/api/status');
    expect(request.request.headers.get('X-RVT-Auth')).toBe('operator-token-123');
    request.flush({ ok: true });

    await expect(pending).resolves.toEqual({ ok: true });
  });

  it('attaches the operator token when only the legacy server URL key is stored', async () => {
    for (const request of httpMock.match('/api/status')) {
      request.flush({ ok: true, mode: 'live' });
    }
    localStorage.setItem(SERVER_URL_KEY, 'http://127.0.0.1:8765');
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'legacy-url-token');

    const pending = service.request<{ ok: boolean }>('/api/status', undefined, true);
    const request = httpMock.expectOne('http://127.0.0.1:8765/api/status');
    expect(request.request.headers.get('X-RVT-Auth')).toBe('legacy-url-token');
    request.flush({ ok: true });

    await expect(pending).resolves.toEqual({ ok: true });
  });

  it('does not send trainer credentials to an untrusted absolute API URL', async () => {
    for (const request of httpMock.match('/api/status')) {
      request.flush({ ok: true, mode: 'live' });
    }
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'operator-token-private');

    const pending = service.request<{ ok: boolean }>('https://untrusted.example/api/status', {
      headers: {
        'X-RVT-Auth': 'caller-supplied-token',
        Authorization: 'Bearer caller-supplied-token'
      }
    }, true);
    const request = httpMock.expectOne('https://untrusted.example/api/status');
    expect(request.request.headers.has('X-RVT-Auth')).toBe(false);
    expect(request.request.headers.has('Authorization')).toBe(false);
    request.flush({ ok: true });

    await expect(pending).resolves.toEqual({ ok: true });
  });

  it('requires an exact configured origin before authenticating an absolute API URL', async () => {
    for (const request of httpMock.match('/api/status')) {
      request.flush({ ok: true, mode: 'live' });
    }
    service.setApiBase('https://trainer.example');
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'exact-origin-token');

    const exactPending = service.request<{ ok: boolean }>(
      'https://trainer.example/api/status',
      undefined,
      true
    );
    const exactRequest = httpMock.expectOne('https://trainer.example/api/status');
    expect(exactRequest.request.headers.get('X-RVT-Auth')).toBe('exact-origin-token');
    exactRequest.flush({ ok: true });
    await expect(exactPending).resolves.toEqual({ ok: true });

    const lookalikePending = service.request<{ ok: boolean }>(
      'https://trainer.example.evil.test/api/status',
      undefined,
      true
    );
    const lookalikeRequest = httpMock.expectOne('https://trainer.example.evil.test/api/status');
    expect(lookalikeRequest.request.headers.has('X-RVT-Auth')).toBe(false);
    lookalikeRequest.flush({ ok: true });
    await expect(lookalikePending).resolves.toEqual({ ok: true });
  });

  it('does not send trainer credentials through native HTTP to an untrusted absolute URL', async () => {
    for (const request of httpMock.match('/api/status')) {
      request.flush({ ok: true, mode: 'live' });
    }
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'native-private-token');
    const nativeRequest = vi.fn().mockResolvedValue({ status: 200, data: { ok: true } });
    (window as Window & { Capacitor?: unknown }).Capacitor = {
      isNativePlatform: () => true,
      Plugins: { CapacitorHttp: { request: nativeRequest } }
    };

    await expect(service.request(
      'https://untrusted.example/api/status',
      { headers: { 'X-RVT-Auth': 'caller-token', Authorization: 'Bearer caller-token' } },
      true
    )).resolves.toEqual({ ok: true });

    expect(nativeRequest).toHaveBeenCalledOnce();
    expect(nativeRequest.mock.calls[0][0]).toMatchObject({
      url: 'https://untrusted.example/api/status',
      headers: {}
    });
  });

  it('does not send trainer credentials when downloading an untrusted absolute URL', async () => {
    for (const request of httpMock.match('/api/status')) {
      request.flush({ ok: true, mode: 'live' });
    }
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'download-private-token');
    const fetchSpy = vi.fn().mockResolvedValue({ ok: false, status: 401 });
    vi.stubGlobal('fetch', fetchSpy);

    await expect(
      service.download('https://untrusted.example/api/export', 'export.csv')
    ).rejects.toThrow('Download failed: HTTP 401');

    const init = fetchSpy.mock.calls[0][1] as RequestInit;
    const headers = new Headers(init.headers);
    expect(headers.has('X-RVT-Auth')).toBe(false);
    expect(headers.has('Authorization')).toBe(false);
  });

  it('creates and validates sandbox operator sessions without a trainer API', async () => {
    for (const request of httpMock.match('/api/status')) {
      request.flush({ ok: true, mode: 'live' });
    }
    service.enableSandboxControlMode('GitHub Pages static PWA');

    await expect(service.request('/api/operator-profiles')).resolves.toEqual({
      schema_version: 'rvt-sandbox-operator-profiles-v12.0',
      profiles: []
    });

    const created = await service.request<any>('/api/operator-profiles', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ display_name: 'Lemuel Biaya', initials: 'LB', pin: '123456' })
    });
    expect(created.ok).toBe(true);
    expect(created.operator.display_name).toBe('Lemuel Biaya');

    const login = await service.request<any>('/api/auth/login', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ operator_id: created.operator.operator_id, pin: '123456' })
    });
    expect(login.token).toMatch(/^sandbox_op_token_/);
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, login.token);

    await expect(service.request('/api/auth/validate')).resolves.toEqual({
      ok: true,
      operator: created.operator
    });

    await expect(service.request('/api/auth/login', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ operator_id: created.operator.operator_id, pin: '0000' })
    })).rejects.toThrow('Invalid operator ID or PIN');

    sessionStorage.removeItem(OPERATOR_TOKEN_KEY);
    await expect(service.request('/api/auth/logout', {
      method: 'POST',
      headers: { 'X-RVT-Auth': login.token }
    })).resolves.toEqual({ ok: true });

    sessionStorage.setItem(OPERATOR_TOKEN_KEY, login.token);
    await expect(service.request('/api/auth/validate')).resolves.toEqual({
      ok: true,
      bootstrap: false,
      operator: null
    });
  });

  it('uses the native_set_paired_origin command when syncing Tauri pairing state', async () => {
    for (const request of httpMock.match('/api/status')) {
      request.flush({ ok: true, mode: 'live' });
    }
    const invoke = vi.fn().mockResolvedValue(undefined);
    (window as Window & { __TAURI__?: unknown }).__TAURI__ = { core: { invoke } };
    service.setApiBase('http://127.0.0.1:8765');

    await service.setTauriPairedOrigin();

    expect(invoke).toHaveBeenCalledWith('native_set_paired_origin', { origin: 'http://127.0.0.1:8765' });
  });
});
