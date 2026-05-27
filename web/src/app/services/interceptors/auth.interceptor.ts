import { HttpInterceptorFn } from '@angular/common/http';

export const rvtAuthInterceptor: HttpInterceptorFn = (req, next) => {
  let token = '';
  try {
    token = sessionStorage.getItem('rvt-pair-token') || '';
  } catch (_) {}

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

  if (token && isApi) {
    req = req.clone({
      setHeaders: {
        'X-RVT-Auth': token
      }
    });
  }
  return next(req);
};
