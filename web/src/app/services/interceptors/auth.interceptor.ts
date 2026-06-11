import { HttpInterceptorFn } from '@angular/common/http';
import { API_BASE_KEY, TOKEN_KEY, OPERATOR_TOKEN_KEY } from '../rvt-storage-keys';

export const rvtAuthInterceptor: HttpInterceptorFn = (req, next) => {
  let token = '';
  try {
    token = sessionStorage.getItem(OPERATOR_TOKEN_KEY) || sessionStorage.getItem(TOKEN_KEY) || '';
  } catch (_) {}

  // Retrieve current API base without injecting ApiService directly
  let base = '';
  try {
    const storedBase = localStorage.getItem(API_BASE_KEY);
    const raw = String(storedBase || '').trim().replace(/\/+$/, '');
    if (raw) {
      const u = new URL(raw, window.location.href);
      if (/^https?:$/.test(u.protocol)) {
        base = u.origin;
      }
    }
  } catch (_) {}

  const isApi = req.url.startsWith('/api/') || (base && req.url.startsWith(base));

  if (token && isApi) {
    req = req.clone({
      setHeaders: {
        'X-RVT-Auth': token
      }
    });
  }
  return next(req);
};
