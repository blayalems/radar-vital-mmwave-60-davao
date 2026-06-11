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
      detectControlMode: vi.fn().mockResolvedValue(true)
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
});
