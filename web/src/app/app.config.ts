import { ApplicationConfig, provideBrowserGlobalErrorListeners } from '@angular/core';
import { provideRouter, withComponentInputBinding, withInMemoryScrolling } from '@angular/router';
import { provideAnimationsAsync } from '@angular/platform-browser/animations/async';
import { provideHttpClient, withFetch, withInterceptors, HttpInterceptorFn, HttpResponse, HttpErrorResponse } from '@angular/common/http';
import { inject } from '@angular/core';
import { from } from 'rxjs';

import { routes } from './app.routes';
import { ApiService } from './services/api.service';

// Custom functional interceptor for X-RVT-Auth token injection
export const rvtAuthInterceptor: HttpInterceptorFn = (req, next) => {
  const apiService = inject(ApiService);
  const token = apiService.pairToken();
  if (token) {
    req = req.clone({
      setHeaders: {
        'X-RVT-Auth': token
      }
    });
  }
  return next(req);
};

// Custom functional interceptor for native Tauri HTTP IPC invocation routing
export const rvtTauriInterceptor: HttpInterceptorFn = (req, next) => {
  const apiService = inject(ApiService);
  const base = apiService.currentApiBase();
  const tauri = (window as any).__TAURI__?.core;

  const isApi = req.url.startsWith('/api/') || (base && req.url.startsWith(base));

  if (tauri?.invoke && base && isApi) {
    let path = req.url;
    if (path.startsWith(base)) {
      path = path.substring(base.length);
    }

    const method = req.method.toUpperCase();
    const headersObj: Record<string, string> = {};
    req.headers.keys().forEach(key => {
      headersObj[key] = req.headers.get(key) || '';
    });

    const command = path === '/api/auth/exchange' || path === '/api/server-info'
      ? 'native_pair_request'
      : 'native_http_request';

    const bodyStr = typeof req.body === 'string'
      ? req.body
      : req.body
        ? JSON.stringify(req.body)
        : null;

    const invokePromise: Promise<HttpResponse<any>> = (tauri.invoke as any)(command, {
      request: {
        origin: base,
        path,
        method,
        headers: headersObj,
        body: bodyStr
      }
    }).then((resp: any): HttpResponse<any> => {
      if (resp.status < 200 || resp.status >= 300) {
        let errMsg = `HTTP ${resp.status}`;
        if (resp.data && typeof resp.data === 'object') {
          const errPayload = resp.data as { error?: string | { message?: string } };
          if (typeof errPayload.error === 'string') errMsg = errPayload.error;
          else if (errPayload.error?.message) errMsg = errPayload.error.message;
        }
        throw new HttpErrorResponse({
          status: resp.status,
          statusText: errMsg,
          error: resp.data,
          url: req.url
        });
      }
      return new HttpResponse({
        body: resp.data,
        status: resp.status,
        statusText: 'OK',
        url: req.url
      });
    });

    return from(invokePromise);
  }

  return next(req);
};

export const appConfig: ApplicationConfig = {
  providers: [
    provideBrowserGlobalErrorListeners(),
    provideRouter(
      routes,
      withComponentInputBinding(),
      withInMemoryScrolling({ scrollPositionRestoration: 'enabled' })
    ),
    provideAnimationsAsync(),
    provideHttpClient(
      withFetch(),
      withInterceptors([rvtAuthInterceptor, rvtTauriInterceptor])
    )
  ]
};
