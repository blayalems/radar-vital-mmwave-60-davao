import { TestBed } from '@angular/core/testing';
import { ApiService } from './api.service';
import { StateService } from './state.service';
import { PersistenceService } from './persistence.service';

describe('ApiService', () => {
  let service: ApiService;
  let state: StateService;
  let mockPersistence: Partial<PersistenceService>;

  beforeEach(() => {
    mockPersistence = {
      quarantineLegacyLocalStorage: vi.fn().mockResolvedValue(undefined),
      get: vi.fn().mockResolvedValue(undefined),
      put: vi.fn().mockResolvedValue(undefined)
    };

    TestBed.configureTestingModule({
      providers: [
        ApiService,
        StateService,
        { provide: PersistenceService, useValue: mockPersistence }
      ]
    });

    state = TestBed.inject(StateService);
    service = TestBed.inject(ApiService);
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
});
