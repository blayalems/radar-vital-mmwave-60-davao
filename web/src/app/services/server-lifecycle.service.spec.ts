import { TestBed } from '@angular/core/testing';

import { ApiService } from './api.service';
import { ServerLifecycleService } from './server-lifecycle.service';
import { StateService } from './state.service';

describe('ServerLifecycleService', () => {
  let api: {
    currentApiBase: ReturnType<typeof vi.fn>;
    setApiBase: ReturnType<typeof vi.fn>;
    checkConnection: ReturnType<typeof vi.fn>;
    detectControlMode: ReturnType<typeof vi.fn>;
  };

  beforeEach(() => {
    localStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    api = {
      currentApiBase: vi.fn(() => ''),
      setApiBase: vi.fn((value: string) => value.replace(/\/+$/, '')),
      checkConnection: vi.fn().mockResolvedValue(true),
      detectControlMode: vi.fn().mockResolvedValue(true)
    };
  });

  afterEach(() => {
    TestBed.resetTestingModule();
    localStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
  });

  function configure(invoke?: ReturnType<typeof vi.fn>): ServerLifecycleService {
    if (invoke) {
      (window as Window & { __TAURI__?: unknown }).__TAURI__ = { core: { invoke } };
    }
    TestBed.configureTestingModule({
      providers: [
        ServerLifecycleService,
        { provide: ApiService, useValue: api },
        { provide: StateService, useValue: { demoMode: () => false } }
      ]
    });
    return TestBed.inject(ServerLifecycleService);
  }

  it('detects Tauri desktop context from window.__TAURI__', async () => {
    const invoke = vi.fn((command: string) => {
      if (command === 'trainer_start') return Promise.resolve({ state: 'starting' });
      if (command === 'native_trainer_status') return Promise.resolve({ running: true, ready: true, origin: 'http://127.0.0.1:8765' });
      return Promise.resolve([]);
    });
    const service = configure(invoke);
    await service.startServer();

    expect(service.platform()).toBe('exe');
    expect(invoke).toHaveBeenCalledWith('trainer_start', { bindMode: 'local' });
  });

  it('transitions to running when Tauri trainer reports ready', async () => {
    const invoke = vi.fn((command: string) => {
      if (command === 'trainer_start') return Promise.resolve({ state: 'starting' });
      if (command === 'native_trainer_status') return Promise.resolve({ running: true, ready: true, origin: 'http://127.0.0.1:8765' });
      return Promise.resolve([]);
    });
    const service = configure(invoke);
    await service.startServer();

    expect(service.status()).toBe('running');
    expect(api.setApiBase).toHaveBeenCalledWith('http://127.0.0.1:8765');
    expect(api.detectControlMode).toHaveBeenCalled();
  });

  it('transitions to offline when remote health check fails', async () => {
    api.checkConnection.mockResolvedValue(false);
    const service = configure();
    await service.retryConnection();

    expect(service.platform()).toBe('remote');
    expect(service.status()).toBe('offline');
    expect(service.lastError()).toContain('/api/health');
  });

  it('retryConnection re-runs the health check', async () => {
    const service = configure();
    await service.retryConnection();
    await service.retryConnection();

    expect(api.checkConnection.mock.calls.length).toBeGreaterThanOrEqual(2);
  });

  it('startServer invokes trainer_start in Tauri context', async () => {
    const invoke = vi.fn((command: string) => {
      if (command === 'trainer_start') return Promise.resolve({ state: 'starting' });
      if (command === 'native_trainer_status') return Promise.resolve({ running: true, ready: false, origin: 'http://127.0.0.1:8765' });
      return Promise.resolve([]);
    });
    const service = configure(invoke);
    await service.startServer();

    expect(invoke).toHaveBeenCalledWith('trainer_start', { bindMode: 'local' });
  });

  it('startServer passes bindMode: lan to Tauri command', async () => {
    const invoke = vi.fn((command: string) => {
      if (command === 'trainer_start') return Promise.resolve({ state: 'starting' });
      if (command === 'native_trainer_status') return Promise.resolve({ running: true, ready: true, origin: 'http://127.0.0.1:8765', bind_mode: 'lan' });
      return Promise.resolve([]);
    });
    const service = configure(invoke);
    await service.startServer('lan');

    expect(invoke).toHaveBeenCalledWith('trainer_start', { bindMode: 'lan' });
    expect(service.bindMode()).toBe('lan');
  });

  it('startServer retries the remote connection outside Tauri', async () => {
    const service = configure();
    await service.startServer();

    expect(api.checkConnection).toHaveBeenCalled();
    expect(service.status()).toBe('running');
  });
});
