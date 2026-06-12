import { TestBed } from '@angular/core/testing';

import { ApiService } from './api.service';
import { AuthService } from './auth.service';
import { ServerLifecycleService } from './server-lifecycle.service';
import { StateService } from './state.service';

describe('ServerLifecycleService', () => {
  let api: {
    currentApiBase: ReturnType<typeof vi.fn>;
    setApiBase: ReturnType<typeof vi.fn>;
    checkConnection: ReturnType<typeof vi.fn>;
    detectControlMode: ReturnType<typeof vi.fn>;
    request: ReturnType<typeof vi.fn>;
  };
  let auth: { lock: ReturnType<typeof vi.fn> };

  beforeEach(() => {
    localStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    api = {
      currentApiBase: vi.fn(() => ''),
      setApiBase: vi.fn((value: string) => value.replace(/\/+$/, '')),
      checkConnection: vi.fn().mockResolvedValue(true),
      detectControlMode: vi.fn().mockResolvedValue(true),
      request: vi.fn().mockResolvedValue({})
    };
    auth = { lock: vi.fn() };
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
        { provide: AuthService, useValue: auth },
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

  it('parses native pairing details while sharing on LAN', async () => {
    const now = Math.floor(Date.now() / 1000);
    api.request.mockResolvedValue({
      origin: 'http://192.168.1.50:8765',
      pair_required: true,
      active_pin: '123456',
      active_pin_expires_at: now + 120,
      qr_png_base64: 'iVBORw0KGgo='
    });
    const invoke = vi.fn((command: string) => {
      if (command === 'trainer_start') return Promise.resolve({ state: 'starting' });
      if (command === 'native_trainer_status') return Promise.resolve({ running: true, ready: true, origin: 'http://127.0.0.1:8765', bind_mode: 'lan' });
      return Promise.resolve([]);
    });
    const service = configure(invoke);
    await service.startServer('lan');

    expect(api.request).toHaveBeenCalledWith('/api/native-pairing-info?format=qr', undefined, true);
    expect(service.pairingInfo()?.active_pin).toBe('123456');
    expect(service.pairingUrl()).toBe('http://192.168.1.50:8765/?pair=123456');
    expect(service.pairingQrDataUrl()).toBe('data:image/png;base64,iVBORw0KGgo=');
    expect(service.pairingTtlSeconds()).toBeGreaterThan(0);
  });

  it('keeps the pairing card usable when the trainer omits PIN fields', async () => {
    // Path-keyed mock: order-independent against the constructor bootstrap race.
    api.request.mockImplementation((path: string) =>
      path.startsWith('/api/native-pairing-info')
        ? Promise.reject(new Error('native pairing info unavailable'))
        : Promise.resolve({ host: '192.168.1.50', port: 8765, pair_required: true })
    );
    const invoke = vi.fn((command: string) => {
      if (command === 'trainer_start') return Promise.resolve({ state: 'starting' });
      if (command === 'native_trainer_status') return Promise.resolve({ running: true, ready: true, origin: 'http://127.0.0.1:8765', bind_mode: 'lan' });
      return Promise.resolve([]);
    });
    const service = configure(invoke);
    await service.startServer('lan');

    expect(service.pairingInfo()?.active_pin).toBeUndefined();
    expect(service.pairingTtlSeconds()).toBeNull();
    expect(service.pairingUrl()).toBe('http://192.168.1.50:8765/pair');
  });

  it('does not lock the UI when a Tauri restart fails', async () => {
    const invoke = vi.fn((command: string) => {
      if (command === 'trainer_stop') return Promise.resolve({ state: 'stopped' });
      if (command === 'trainer_start') return Promise.resolve({ state: 'error', message: 'Port 8765 is already in use' });
      if (command === 'native_trainer_status') return Promise.resolve({ running: false, ready: false, error: 'Port 8765 is already in use' });
      return Promise.resolve([]);
    });
    const service = configure(invoke);

    await expect(service.restartServer('lan')).rejects.toThrow('Port 8765 is already in use');
    expect(auth.lock).not.toHaveBeenCalled();
    expect(service.status()).toBe('error');
  });

  it('locks the UI only after a successful Tauri restart reaches running', async () => {
    const invoke = vi.fn((command: string) => {
      if (command === 'trainer_stop') return Promise.resolve({ state: 'stopped' });
      if (command === 'trainer_start') return Promise.resolve({ state: 'starting' });
      if (command === 'native_trainer_status') return Promise.resolve({ running: true, ready: true, origin: 'http://127.0.0.1:49222' });
      return Promise.resolve([]);
    });
    const service = configure(invoke);

    await service.restartServer('local');

    expect(service.status()).toBe('running');
    expect(api.setApiBase).toHaveBeenCalledWith('http://127.0.0.1:49222');
    expect(auth.lock).toHaveBeenCalledTimes(1);
    const stopIndex = invoke.mock.calls.map(([command]) => command).lastIndexOf('trainer_stop');
    const startIndex = invoke.mock.calls.map(([command]) => command).lastIndexOf('trainer_start');
    const stopOrder = invoke.mock.invocationCallOrder[stopIndex];
    const startOrder = invoke.mock.invocationCallOrder[startIndex];
    expect(stopOrder).toBeLessThan(startOrder);
  });

  it('startServer retries the remote connection outside Tauri', async () => {
    const service = configure();
    await service.startServer();

    expect(api.checkConnection).toHaveBeenCalled();
    expect(service.status()).toBe('running');
  });
});
