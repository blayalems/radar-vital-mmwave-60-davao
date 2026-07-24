import { HttpInterceptorFn } from '@angular/common/http';
import { API_BASE_KEY, SERVER_URL_KEY, TOKEN_KEY, OPERATOR_TOKEN_KEY } from '../rvt-storage-keys';
import { isTrustedTrainerApiTarget, normalizeHttpOrigin } from '../api-target-policy';

export const rvtAuthInterceptor: HttpInterceptorFn = (req, next) => {
  let token = '';
  try {
    token = sessionStorage.getItem(OPERATOR_TOKEN_KEY) || sessionStorage.getItem(TOKEN_KEY) || '';
  } catch (_) {}

  // Retrieve current API base without injecting ApiService directly
  let base = '';
  try {
    const storedBase = localStorage.getItem(API_BASE_KEY) || localStorage.getItem(SERVER_URL_KEY);
    base = normalizeHttpOrigin(storedBase || '');
  } catch (_) {}

  const hasTauriTransport = Boolean((window as any).__TAURI__?.core?.invoke);
  const isApi = isTrustedTrainerApiTarget(req.url, base, {
    allowTauriLoopback: hasTauriTransport
  });

  if (token && isApi) {
    req = req.clone({
      setHeaders: {
        'X-RVT-Auth': token
      }
    });
  } else if (!isApi && req.headers.has('X-RVT-Auth')) {
    req = req.clone({
      headers: req.headers.delete('X-RVT-Auth')
    });
  }
  return next(req);
};
