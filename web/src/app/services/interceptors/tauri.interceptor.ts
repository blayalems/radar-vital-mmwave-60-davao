import { HttpInterceptorFn, HttpResponse, HttpErrorResponse } from '@angular/common/http';
import { from } from 'rxjs';
import { API_BASE_KEY, SERVER_URL_KEY } from '../rvt-storage-keys';
import {
  isLoopbackHttpOrigin,
  isTrustedTrainerApiTarget,
  normalizeHttpOrigin
} from '../api-target-policy';

let cachedBase: string | null = null;
let cachedLocalOrigin: string | null = null;
let tauriListenersInstalled = false;
let lastPairedOrigin: string | null = null;

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

function clearLocalCache(): void {
  if (cachedBase && cachedBase === cachedLocalOrigin) {
    cachedBase = null;
  }
  cachedLocalOrigin = null;
  lastPairedOrigin = null;
}

function installTauriSidecarListeners(tauri: any): void {
  if (tauriListenersInstalled) return;
  tauriListenersInstalled = true;
  const listen = (window as any).__TAURI__?.event?.listen;
  if (typeof listen !== 'function') return;
  try {
    listen('rvt-trainer-terminated', clearLocalCache);
    listen('rvt-trainer-error', clearLocalCache);
  } catch (_) {}
}

function explicitStoredOrigin(): string {
  try {
    const stored = normalizeHttpOrigin(localStorage.getItem(API_BASE_KEY) || localStorage.getItem(SERVER_URL_KEY) || '');
    // Dynamic bundled sidecar origins must not survive process restarts. Treat
    // loopback values from older builds as stale and let Rust provide the fresh
    // per-launch sidecar origin through native_trainer_status.
    if (stored && isLoopbackHttpOrigin(stored)) {
      localStorage.removeItem(API_BASE_KEY);
      localStorage.removeItem(SERVER_URL_KEY);
      return '';
    }
    return stored;
  } catch (_) {
    return '';
  }
}

async function resolveTauriApiBase(tauri: any): Promise<string> {
  installTauriSidecarListeners(tauri);

  const stored = explicitStoredOrigin();
  if (stored) {
    cachedBase = stored;
    if (lastPairedOrigin !== stored) {
      try {
        await (tauri.invoke as (cmd: string, args?: Record<string, unknown>) => Promise<unknown>)('native_set_paired_origin', { origin: stored });
        lastPairedOrigin = stored;
      } catch (_) {}
    }
    return stored;
  }

  if (cachedBase && cachedBase !== cachedLocalOrigin) return cachedBase;

  const status = await (tauri.invoke as (cmd: string) => Promise<NativeTrainerStatus>)('native_trainer_status');
  const localOrigin = normalizeHttpOrigin(String(status?.origin || ''));
  if (localOrigin && status?.running !== false) {
    cachedBase = localOrigin;
    cachedLocalOrigin = localOrigin;
    if (lastPairedOrigin !== localOrigin) {
      try {
        await (tauri.invoke as (cmd: string, args?: Record<string, unknown>) => Promise<unknown>)('native_set_paired_origin', { origin: localOrigin });
        lastPairedOrigin = localOrigin;
      } catch (_) {}
    }
    return localOrigin;
  }
  return '';
}

export const rvtTauriInterceptor: HttpInterceptorFn = (req, next) => {
  const tauri = (window as any).__TAURI__?.core;

  if (!tauri?.invoke) {
    return next(req);
  }

  if (!isTrustedTrainerApiTarget(req.url, explicitStoredOrigin(), {
    // Old bundled-sidecar URLs can remain in an in-flight request while the
    // sidecar restarts on a new port. They are safe only here: this interceptor
    // re-resolves them and Rust pins the request to the newly paired origin.
    allowTauriLoopback: true
  })) {
    return next(req);
  }

  const invokePromise = (async (): Promise<HttpResponse<any>> => {
    const base = await resolveTauriApiBase(tauri);
    if (!base) {
      throw new HttpErrorResponse({
        status: 0,
        statusText: 'No trainer origin is available',
        error: { error: 'No trainer origin is available' },
        url: req.url
      });
    }

    const parsedUrl = new URL(req.url, base);
    const path = parsedUrl.pathname + parsedUrl.search;
    const method = req.method.toUpperCase();
    const headersObj: Record<string, string> = {};
    req.headers.keys().forEach(key => {
      headersObj[key] = req.headers.get(key) || '';
    });

    const command = path.startsWith('/api/auth/exchange')
      || path.startsWith('/api/server-info')
      || path.startsWith('/api/native-pairing-info')
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
  })();

  return from(invokePromise);
};
