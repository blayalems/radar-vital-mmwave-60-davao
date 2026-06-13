import { HttpClient, provideHttpClient, withInterceptors } from '@angular/common/http';
import { TestBed } from '@angular/core/testing';
import { firstValueFrom } from 'rxjs';

import { OPERATOR_TOKEN_KEY } from '../rvt-storage-keys';
import { rvtAuthInterceptor } from './auth.interceptor';
import { rvtTauriInterceptor } from './tauri.interceptor';

describe('rvtTauriInterceptor', () => {
  beforeEach(() => {
    localStorage.clear();
    sessionStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
  });

  afterEach(() => {
    localStorage.clear();
    sessionStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    TestBed.resetTestingModule();
  });

  it('pairs the bundled trainer origin before forwarding native API calls', async () => {
    const invoke = vi.fn((command: string, args?: Record<string, unknown>) => {
      if (command === 'native_trainer_status') {
        return Promise.resolve({
          running: true,
          ready: true,
          origin: 'http://127.0.0.1:49152'
        });
      }
      if (command === 'native_set_paired_origin') {
        return Promise.resolve(undefined);
      }
      if (command === 'native_http_request') {
        return Promise.resolve({ status: 200, data: { ok: true } });
      }
      return Promise.reject(new Error(`unexpected command ${command}`));
    });
    (window as Window & { __TAURI__?: unknown }).__TAURI__ = { core: { invoke } };
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'operator-token-native');

    TestBed.configureTestingModule({
      providers: [
        provideHttpClient(withInterceptors([rvtAuthInterceptor, rvtTauriInterceptor]))
      ]
    });
    const http = TestBed.inject(HttpClient);

    await expect(firstValueFrom(http.get('/api/status'))).resolves.toEqual({ ok: true });

    expect(invoke).toHaveBeenCalledWith('native_set_paired_origin', { origin: 'http://127.0.0.1:49152' });
    const nativeRequest = invoke.mock.calls.find(([command]) => command === 'native_http_request');
    const pairOrder = invoke.mock.invocationCallOrder[invoke.mock.calls.findIndex(([command]) => command === 'native_set_paired_origin')];
    const nativeOrder = invoke.mock.invocationCallOrder[invoke.mock.calls.findIndex(([command]) => command === 'native_http_request')];
    expect(pairOrder).toBeLessThan(nativeOrder);
    expect(nativeRequest?.[1]).toMatchObject({
      request: {
        origin: 'http://127.0.0.1:49152',
        path: '/api/status',
        method: 'GET',
        headers: {
          'X-RVT-Auth': 'operator-token-native'
        }
      }
    });
  });

  it('refreshes stale loopback origins and still intercepts absolute old API URLs after restart', async () => {
    const origins = ['http://127.0.0.1:49153', 'http://127.0.0.1:49222'];
    const invoke = vi.fn((command: string, args?: Record<string, unknown>) => {
      if (command === 'native_trainer_status') {
        const origin = origins.shift() || 'http://127.0.0.1:49222';
        return Promise.resolve({ running: true, ready: true, origin });
      }
      if (command === 'native_set_paired_origin') {
        return Promise.resolve(undefined);
      }
      if (command === 'native_http_request') {
        return Promise.resolve({ status: 200, data: { ok: true } });
      }
      return Promise.reject(new Error(`unexpected command ${command}`));
    });
    (window as Window & { __TAURI__?: unknown }).__TAURI__ = { core: { invoke } };
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'operator-token-after-restart');

    TestBed.configureTestingModule({
      providers: [
        provideHttpClient(withInterceptors([rvtAuthInterceptor, rvtTauriInterceptor]))
      ]
    });
    const http = TestBed.inject(HttpClient);

    await expect(firstValueFrom(http.get('/api/status'))).resolves.toEqual({ ok: true });
    await expect(firstValueFrom(http.post('http://127.0.0.1:49153/api/auth/login', {
      operator_id: 'op_1',
      pin: '1234'
    }))).resolves.toEqual({ ok: true });

    expect(invoke).toHaveBeenCalledWith('native_set_paired_origin', { origin: 'http://127.0.0.1:49153' });
    expect(invoke).toHaveBeenCalledWith('native_set_paired_origin', { origin: 'http://127.0.0.1:49222' });
    const nativeRequests = invoke.mock.calls.filter(([command]) => command === 'native_http_request');
    expect(nativeRequests.at(-1)?.[1]).toMatchObject({
      request: {
        origin: 'http://127.0.0.1:49222',
        path: '/api/auth/login',
        method: 'POST',
        headers: {
          'X-RVT-Auth': 'operator-token-after-restart'
        }
      }
    });
  });
});
