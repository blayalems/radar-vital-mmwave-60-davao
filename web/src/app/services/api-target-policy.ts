export interface TrainerApiTrustOptions {
  allowTauriLoopback?: boolean;
  locationHref?: string;
}

function locationHref(override?: string): string {
  if (override) return override;
  try {
    return window.location.href;
  } catch (_) {
    return 'http://rvt.local/';
  }
}

function hasExplicitSchemeOrAuthority(value: string): boolean {
  return /^[a-z][a-z0-9+.-]*:/i.test(value) || value.startsWith('//');
}

function absoluteHttpUrl(value: string, href?: string): URL | null {
  const raw = String(value || '').trim();
  if (!raw || !hasExplicitSchemeOrAuthority(raw)) return null;
  try {
    const parsed = new URL(raw, locationHref(href));
    return /^https?:$/.test(parsed.protocol) ? parsed : null;
  } catch (_) {
    return null;
  }
}

function isApiPath(pathname: string): boolean {
  return pathname === '/api' || pathname.startsWith('/api/');
}

export function normalizeHttpOrigin(value: string, href?: string): string {
  const raw = String(value || '').trim().replace(/\/+$/, '');
  if (!raw) return '';
  try {
    const parsed = new URL(raw, locationHref(href));
    if (!/^https?:$/.test(parsed.protocol) || parsed.username || parsed.password) return '';
    return parsed.origin;
  } catch (_) {
    return '';
  }
}

export function isRelativeApiTarget(value: string): boolean {
  const raw = String(value || '');
  return !raw.startsWith('//') && (raw === '/api' || raw.startsWith('/api/'));
}

export function isLoopbackHttpOrigin(value: string): boolean {
  try {
    const parsed = new URL(value);
    const host = parsed.hostname.toLowerCase();
    return parsed.protocol === 'http:'
      && (host === '127.0.0.1' || host === 'localhost' || host === '::1' || host === '[::1]');
  } catch (_) {
    return false;
  }
}

export function resolveTrainerRequestTarget(value: string, configuredOrigin: string, href?: string): string {
  const raw = String(value || '');
  if (!hasExplicitSchemeOrAuthority(raw)) return configuredOrigin + raw;
  try {
    return new URL(raw, locationHref(href)).href;
  } catch (_) {
    return raw;
  }
}

export function isTrustedTrainerApiTarget(
  value: string,
  configuredOrigin: string,
  options: TrainerApiTrustOptions = {}
): boolean {
  if (isRelativeApiTarget(value)) return true;

  const parsed = absoluteHttpUrl(value, options.locationHref);
  if (!parsed || !isApiPath(parsed.pathname) || parsed.username || parsed.password) return false;

  const exactOrigin = normalizeHttpOrigin(configuredOrigin, options.locationHref);
  if (exactOrigin && parsed.origin === exactOrigin) return true;

  return Boolean(options.allowTauriLoopback && isLoopbackHttpOrigin(parsed.origin));
}
