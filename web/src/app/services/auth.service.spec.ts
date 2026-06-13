import { TestBed } from '@angular/core/testing';
import { AuthService } from './auth.service';
import { ApiService } from './api.service';
import { StateService } from './state.service';
import { OPERATOR_TOKEN_KEY } from './rvt-storage-keys';

describe('AuthService', () => {
  let service: AuthService;
  let mockApi: any;
  let state: StateService;

  beforeEach(() => {
    sessionStorage.removeItem(OPERATOR_TOKEN_KEY);

    mockApi = {
      request: vi.fn(),
      detectControlMode: vi.fn().mockResolvedValue(true),
      connectionLoading: vi.fn(() => false),
      whenInitialized: vi.fn().mockResolvedValue(undefined)
    };

    TestBed.configureTestingModule({
      providers: [
        AuthService,
        StateService,
        { provide: ApiService, useValue: mockApi }
      ]
    });

    state = TestBed.inject(StateService);
    service = TestBed.inject(AuthService);
  });

  it('should be created and default to locked state', () => {
    expect(service).toBeTruthy();
    expect(service.isLocked()).toBe(true);
    expect(service.currentOperator()).toBeNull();
  });

  it('should load profiles from API', async () => {
    const mockProfiles = [
      { operator_id: 'op_1', display_name: 'Operator One', initials: 'O1' }
    ];
    mockApi.request.mockResolvedValueOnce({
      schema_version: 'rvt-operator-profiles-v12.0',
      profiles: mockProfiles
    });

    await service.loadProfiles();

    expect(mockApi.request).toHaveBeenCalledWith('/api/operator-profiles');
    expect(service.profiles()).toEqual(mockProfiles);
    expect(service.bootstrapping()).toBe(false);
  });

  it('should set bootstrapping true if no profiles exist', async () => {
    mockApi.request.mockResolvedValueOnce({
      schema_version: 'rvt-operator-profiles-v12.0',
      profiles: []
    });

    await service.loadProfiles();

    expect(service.profiles().length).toBe(0);
    expect(service.bootstrapping()).toBe(true);
  });

  it('should set state and token on successful login', async () => {
    const mockResponse = {
      token: 'test-session-token',
      expires_at: 123456789,
      operator: { operator_id: 'op_1', display_name: 'Operator One', initials: 'O1' }
    };
    mockApi.request.mockResolvedValueOnce(mockResponse);

    const success = await service.login('op_1', '1234');

    expect(success).toBe(true);
    expect(sessionStorage.getItem(OPERATOR_TOKEN_KEY)).toBe('test-session-token');
    expect(service.currentOperator()).toEqual(mockResponse.operator);
    expect(service.isLocked()).toBe(false);
    expect(state.setup().operator_label).toBe('Operator One');
  });

  it('does not immediately re-lock a fresh login because of stale unauthenticated status', async () => {
    state.ctlStatus.set({ ok: true, mode: 'live', reason: 'unauthenticated' });
    const mockResponse = {
      token: 'fresh-session-token',
      expires_at: 123456789,
      operator: { operator_id: 'op_1', display_name: 'Operator One', initials: 'O1' }
    };
    mockApi.request.mockResolvedValueOnce(mockResponse);

    const success = await service.login('op_1', '1234');
    await new Promise(resolve => setTimeout(resolve, 0));

    expect(success).toBe(true);
    expect(sessionStorage.getItem(OPERATOR_TOKEN_KEY)).toBe('fresh-session-token');
    expect(service.isLocked()).toBe(false);
    expect(service.currentOperator()).toEqual(mockResponse.operator);
    expect(state.ctlStatus()?.reason).toBeUndefined();
  });

  it('should set lockout countdown on LOCKOUT_ACTIVE error', async () => {
    const errorMsg = 'Too many failed attempts. Try again in 25 seconds. (LOCKOUT_ACTIVE)';
    mockApi.request.mockRejectedValueOnce(new Error(errorMsg));

    const success = await service.login('op_1', '1234');

    expect(success).toBe(false);
    expect(service.isLocked()).toBe(true);
    expect(service.lockoutRetryAfter()).toBe(25);
    expect(service.loginError()).toContain('Try again in 25 seconds');
  });

  it('should perform logout and call API logout', async () => {
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'some-token');
    service.isLocked.set(false);
    service.currentOperator.set({ operator_id: 'op_1', display_name: 'Operator One', initials: 'O1' });

    mockApi.request.mockResolvedValueOnce({ ok: true });

    await service.logout();

    expect(sessionStorage.getItem(OPERATOR_TOKEN_KEY)).toBeNull();
    expect(service.isLocked()).toBe(true);
    expect(service.currentOperator()).toBeNull();
    expect(mockApi.request).toHaveBeenCalledWith('/api/auth/logout', {
      method: 'POST',
      headers: { 'X-RVT-Auth': 'some-token' }
    });
  });

  it('should perform lock without waiting for API logout', () => {
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'some-token');
    service.isLocked.set(false);

    mockApi.request.mockResolvedValueOnce({ ok: true });

    service.lock();

    expect(sessionStorage.getItem(OPERATOR_TOKEN_KEY)).toBeNull();
    expect(service.isLocked()).toBe(true);
    expect(mockApi.request).toHaveBeenCalledWith('/api/auth/logout', {
      method: 'POST',
      headers: { 'X-RVT-Auth': 'some-token' }
    });
  });

  it('locks the UI when control status reports an invalid operator session', async () => {
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'stale-token');
    service.isLocked.set(false);
    service.currentOperator.set({ operator_id: 'op_1', display_name: 'Operator One', initials: 'O1' });
    mockApi.request.mockResolvedValueOnce({ ok: true });

    state.ctlStatus.set({ ok: true, mode: 'live', reason: 'unauthenticated' });
    await new Promise(resolve => setTimeout(resolve, 0));

    expect(service.isLocked()).toBe(true);
    expect(service.currentOperator()).toBeNull();
    expect(sessionStorage.getItem(OPERATOR_TOKEN_KEY)).toBeNull();
    expect(service.loginError()).toContain('Operator session expired');
    expect(mockApi.request).toHaveBeenCalledWith('/api/auth/logout', {
      method: 'POST',
      headers: { 'X-RVT-Auth': 'stale-token' }
    });
  });

  it('createProfile returns recoveryCode from API response', async () => {
    // Step 1: POST /api/operator-profiles
    mockApi.request.mockResolvedValueOnce({
      ok: true,
      operator: { operator_id: 'op_new', display_name: 'New Op', initials: 'NO' },
      recovery_code: 'ABCD-EFGH-JKMN',
    });
    // Step 2: loadProfiles (GET /api/operator-profiles)
    mockApi.request.mockResolvedValueOnce({
      schema_version: 'rvt-operator-profiles-v12.0',
      profiles: [{ operator_id: 'op_new', display_name: 'New Op', initials: 'NO' }],
    });
    // Step 3: login
    mockApi.request.mockResolvedValueOnce({
      token: 'tok-new',
      expires_at: Date.now() / 1000 + 28800,
      operator: { operator_id: 'op_new', display_name: 'New Op', initials: 'NO' },
    });

    const result = await service.createProfile('New Op', 'NO', '1234');

    expect(result.success).toBe(true);
    expect(result.recoveryCode).toBe('ABCD-EFGH-JKMN');
  });

  it('createProfile returns success:false on API error', async () => {
    mockApi.request.mockRejectedValueOnce(new Error('Server error'));

    const result = await service.createProfile('New Op', 'NO', '1234');

    expect(result.success).toBe(false);
    expect(result.recoveryCode).toBeUndefined();
  });

  it('resetPin returns success and rotated recovery code', async () => {
    mockApi.request.mockResolvedValueOnce({
      ok: true,
      recovery_code: 'NEWC-ODEN-EWCO',
    });

    const result = await service.resetPin('op_1', 'ABCD-EFGH-JKMN', '5678');

    expect(result.success).toBe(true);
    expect(result.recoveryCode).toBe('NEWC-ODEN-EWCO');
    expect(mockApi.request).toHaveBeenCalledWith(
      '/api/auth/reset-pin',
      expect.objectContaining({ method: 'POST' }),
      false,
      30000,
    );
  });

  it('resetPin handles LOCKOUT_ACTIVE error with countdown', async () => {
    mockApi.request.mockRejectedValueOnce(
      new Error('Too many failed recovery attempts. Try again in 28 seconds. (LOCKOUT_ACTIVE)')
    );

    const result = await service.resetPin('op_1', 'XXXX-YYYY-ZZZZ', '1234');

    expect(result.success).toBe(false);
    expect(service.loginError()).toContain('28 seconds');
  });

  it('hostReset returns success and new recovery code', async () => {
    mockApi.request.mockResolvedValueOnce({
      ok: true,
      recovery_code: 'HOST-RESE-TNEW',
    });

    const result = await service.hostReset('op_1', '9999');

    expect(result.success).toBe(true);
    expect(result.recoveryCode).toBe('HOST-RESE-TNEW');
    expect(mockApi.request).toHaveBeenCalledWith(
      '/api/auth/host-reset',
      expect.objectContaining({ method: 'POST' }),
      false,
      30000,
    );
  });

  it('hostReset surfaces error on failure', async () => {
    mockApi.request.mockRejectedValueOnce(new Error('Host reset failed'));

    const result = await service.hostReset('op_1', '1234');

    expect(result.success).toBe(false);
    expect(result.error).toBe('Host reset failed');
    expect(service.loginError()).toBe('Host reset failed');
  });
});
