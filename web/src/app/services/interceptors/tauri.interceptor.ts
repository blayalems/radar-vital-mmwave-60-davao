import { HttpInterceptorFn, HttpResponse, HttpErrorResponse } from '@angular/common/http';
import { from } from 'rxjs';

const API_BASE_KEY = 'rvt-api-base';

interface TauriInvokeRequest {
  origin: string;
  path: string;
  method: string;
  headers: Record<string, string>;
  body: string | null;
}

interface NativeTrainerStatus {
  enabled?: boolean;
  running?: boolean;
  ready?: boolean;
  origin?: string | null;
  error?: string | null;
  sessions_root?: string | null;
}

function normalizeHttpOrigin(value: string): string {
  const raw = String(value || '').trim().replace(/\/+$/, '');
  if (!raw) return '';
  try {
    const u = new URL(raw, window.location.href);
    return /^https?:$/.test(u.protocol) ? u.origin : '';
  } catch (_) {
    return '';
  }
}

async function resolveTauriApiBase(tauri: any): Promise<string> {
  let stored = '';
  try {
    stored = normalizeHttpOrigin(localStorage.getItem(API_BASE_KEY) || '');
  } catch (_) {}

  const status = await (tauri.invoke as (cmd: string) => Promise<NativeTrainerStatus>)('native_trainer_status');
  const localOrigin = normalizeHttpOrigin(String(status?.origin || ''));

  // A running bundled trainer wins over stale LAN origins in packaged EXE mode.
  // External LAN trainers are still selectable after startup through Settings,
  // which calls native_set_paired_origin before protected requests.
  if (localOrigin && status?.running !== false && (stored !== localOrigin)) {
    try { localStorage.setItem(API_BASE_KEY, localOrigin); } catch (_) {}
    return localOrigin;
  }

  if (stored) return stored;
  if (localOrigin) {
    try { localStorage.setItem(API_BASE_KEY, localOrigin); } catch (_) {}
    return localOrigin;
  }
  return '';
}

export const rvtTauriInterceptor: HttpInterceptorFn = (req, next) => {
  const tauri = (window as any).__TAURI__?.core;

  if (!tauri?.invoke) {
    return next(req);
  }

  const invokePromise = (async (): Promise<HttpResponse<any>> => {
    const base = await resolveTauriApiBase(tauri);
    const isApi = req.url.startsWith('/api/') || (base && req.url.startsWith(base));
    if (!base || !isApi) {
      throw new Error('RVT_TAURI_PASS_THROUGH');
    }

    const parsedUrl = new URL(req.url, base);
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

    const resp = await (tauri.invoke as (cmd: string, args: { request: TauriInvokeRequest }) => Promise<any>)(command, {
      request: requestPayload
    });

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
  })().catch(error => {
    if (error instanceof Error && error.message === 'RVT_TAURI_PASS_THROUGH') {
      return new Promise<HttpResponse<any>>((resolve, reject) => {
        next(req).subscribe({
          next: event => {
            if (event instanceof HttpResponse) resolve(event);
          },
          error: reject
        });
      });
    }
    throw error;
  });

  return from(invokePromise);
};
