import { HttpInterceptorFn, HttpResponse, HttpErrorResponse } from '@angular/common/http';
import { from } from 'rxjs';

interface TauriInvokeRequest {
  origin: string;
  path: string;
  method: string;
  headers: Record<string, string>;
  body: string | null;
}

export const rvtTauriInterceptor: HttpInterceptorFn = (req, next) => {
  const tauri = (window as any).__TAURI__?.core;

  // Retrieve current API base without injecting ApiService directly
  let base = '';
  try {
    const storedBase = localStorage.getItem('rvt-api-base');
    const raw = String(storedBase || '').trim().replace(/\/+$/, '');
    if (raw) {
      const u = new URL(raw, window.location.href);
      if (/^https?:$/.test(u.protocol)) {
        base = u.origin;
      }
    }
  } catch (_) {}

  const isApi = req.url.startsWith('/api/') || (base && req.url.startsWith(base));

  if (tauri?.invoke && base && isApi) {
    const parsedUrl = new URL(req.url, window.location.href);
    const path = parsedUrl.pathname + parsedUrl.search;

    const method = req.method.toUpperCase();
    const headersObj: Record<string, string> = {};
    req.headers.keys().forEach(key => {
      headersObj[key] = req.headers.get(key) || '';
    });

    const command = path.startsWith('/api/auth/exchange') || path.startsWith('/api/server-info')
      ? 'native_pair_request'
      : 'native_http_request';

    const bodyStr = typeof req.body === 'string'
      ? req.body
      : req.body
        ? JSON.stringify(req.body)
        : null;

    const requestPayload: TauriInvokeRequest = {
      origin: base,
      path,
      method,
      headers: headersObj,
      body: bodyStr
    };

    const invokePromise = (tauri.invoke as (cmd: string, args: { request: TauriInvokeRequest }) => Promise<any>)(command, {
      request: requestPayload
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
