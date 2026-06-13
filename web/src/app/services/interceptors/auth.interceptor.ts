import { HttpInterceptorFn } from '@angular/common/http';
import { API_BASE_KEY, SERVER_URL_KEY, TOKEN_KEY, OPERATOR_TOKEN_KEY } from '../rvt-storage-keys';

function isApiUrl(value: string): boolean {
  if (value.startsWith('/api/')) return true;
  try {
    const parsed = new URL(value, 'http://rvt.local');
    return /^https?:$/.test(parsed.protocol) && parsed.pathname.startsWith('/api/');
  } catch (_) {
    return false;
  }
}

export const rvtAuthInterceptor: HttpInterceptorFn = (req, next) => {
  let token = '';
  try {
    token = sessionStorage.getItem(OPERATOR_TOKEN_KEY) || sessionStorage.getItem(TOKEN_KEY) || '';
  } catch (_) {}

  // Retrieve current API base without injecting ApiService directly
  let base = '';
  try {
    const storedBase = localStorage.getItem(API_BASE_KEY) || localStorage.getItem(SERVER_URL_KEY);
    const raw = String(storedBase || '').trim().replace(/\/+$/, '');
    if (raw) {
      const u = new URL(raw, window.location.href);
      if (/^https?:$/.test(u.protocol)) {
        base = u.origin;
      }
    }
  } catch (_) {}

  const isApi = isApiUrl(req.url) || Boolean(base && req.url.startsWith(base));

  if (token && isApi) {
    req = req.clone({
      setHeaders: {
        'X-RVT-Auth': token
      }
    });
  }
  return next(req);
};
