import { HttpClient, provideHttpClient, withInterceptors } from '@angular/common/http';
import { HttpTestingController, provideHttpClientTesting } from '@angular/common/http/testing';
import { TestBed } from '@angular/core/testing';
import { firstValueFrom } from 'rxjs';

import { API_BASE_KEY, OPERATOR_TOKEN_KEY } from '../rvt-storage-keys';
import { rvtAuthInterceptor } from './auth.interceptor';

describe('rvtAuthInterceptor', () => {
  let http: HttpClient;
  let httpMock: HttpTestingController;

  beforeEach(() => {
    localStorage.clear();
    sessionStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    TestBed.configureTestingModule({
      providers: [
        provideHttpClient(withInterceptors([rvtAuthInterceptor])),
        provideHttpClientTesting()
      ]
    });
    http = TestBed.inject(HttpClient);
    httpMock = TestBed.inject(HttpTestingController);
    sessionStorage.setItem(OPERATOR_TOKEN_KEY, 'operator-secret');
  });

  afterEach(() => {
    httpMock.verify();
    localStorage.clear();
    sessionStorage.clear();
    delete (window as Window & { __TAURI__?: unknown }).__TAURI__;
    TestBed.resetTestingModule();
  });

  it('authenticates a same-origin relative API request', async () => {
    const pending = firstValueFrom(http.get('/api/status'));
    const request = httpMock.expectOne('/api/status');
    expect(request.request.headers.get('X-RVT-Auth')).toBe('operator-secret');
    request.flush({ ok: true });

    await expect(pending).resolves.toEqual({ ok: true });
  });

  it('authenticates only an absolute API URL on the exact configured origin', async () => {
    localStorage.setItem(API_BASE_KEY, 'https://trainer.example');

    const pending = firstValueFrom(http.get('https://trainer.example/api/status'));
    const request = httpMock.expectOne('https://trainer.example/api/status');
    expect(request.request.headers.get('X-RVT-Auth')).toBe('operator-secret');
    request.flush({ ok: true });

    await expect(pending).resolves.toEqual({ ok: true });
  });

  it('does not authenticate an untrusted absolute URL with an API-looking path', async () => {
    const pending = firstValueFrom(http.get('https://untrusted.example/api/status'));
    const request = httpMock.expectOne('https://untrusted.example/api/status');
    expect(request.request.headers.has('X-RVT-Auth')).toBe(false);
    request.flush({ ok: true });

    await expect(pending).resolves.toEqual({ ok: true });
  });

  it('rejects configured-origin prefix lookalikes and strips a supplied trainer header', async () => {
    localStorage.setItem(API_BASE_KEY, 'https://trainer.example');

    const pending = firstValueFrom(http.get(
      'https://trainer.example.evil.test/api/status',
      { headers: { 'X-RVT-Auth': 'caller-supplied-token' } }
    ));
    const request = httpMock.expectOne('https://trainer.example.evil.test/api/status');
    expect(request.request.headers.has('X-RVT-Auth')).toBe(false);
    request.flush({ ok: true });

    await expect(pending).resolves.toEqual({ ok: true });
  });

  it('does not treat an absolute page-origin URL as a relative API request', async () => {
    const target = new URL('/api/status', window.location.href).href;
    const pending = firstValueFrom(http.get(target));
    const request = httpMock.expectOne(target);
    expect(request.request.headers.has('X-RVT-Auth')).toBe(false);
    request.flush({ ok: true });

    await expect(pending).resolves.toEqual({ ok: true });
  });
});
