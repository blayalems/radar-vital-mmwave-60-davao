const CACHE = 'rvt-shell-v12.0.7';
const DASHBOARD = './index.html';
const MONOLITH = './radar_vital_live_dashboard_v12_for_v16_0.html';
const NAVIGATION_CACHE_KEY = new URL(DASHBOARD, self.registration.scope).href;
const NAVIGATION_MONOLITH_KEY = new URL(MONOLITH, self.registration.scope).href;
const APP_SHELL_PATHS = new Set([
  '',
  'connect',
  'home',
  'live',
  'report',
  'help',
  'settings',
  'index.html',
  'radar_vital_live_dashboard_v12_for_v16_0.html'
]);
const PRECACHE = [
  DASHBOARD,
  MONOLITH,
  './manifest.webmanifest',
  './icons/icon-192.png',
  './icons/icon-512.png',
  './icons/icon-maskable-512.png',
  './icons/apple-touch-icon-180.png',
  './lib/chart.umd.min.js',
  './lib/chartjs-plugin-zoom.min.js',
  './lib/hammer.min.js',
  './lib/jsqr.min.js',
  './fonts/rvt-fonts.css',
  './fonts/rvt-font-definitions.css',
  './fonts/rvt-runtime-accessibility.css',
  './fonts/figtree-latin.woff2',
  './fonts/figtree-latin-ext.woff2',
  './fonts/inter-5.woff2',
  './fonts/inter-6.woff2',
  './fonts/jetbrains-mono-5.woff2',
  './fonts/material-symbols-rounded.woff2'
];

self.addEventListener('install', event => {
  event.waitUntil(caches.open(CACHE).then(cache => cache.addAll(PRECACHE)));
});

self.addEventListener('message', event => {
  if (event.data?.type === 'SKIP_WAITING') self.skipWaiting();
});

self.addEventListener('activate', event => {
  event.waitUntil((async () => {
    const keys = await caches.keys();
    await Promise.all(keys.filter(key => key !== CACHE).map(key => caches.delete(key)));
    await self.clients.claim();
    const clients = await self.clients.matchAll();
    clients.forEach(client => client.postMessage({ type: 'SW_UPDATED' }));
  })());
});

function isAppShellRequest(url) {
  const scope = new URL(self.registration.scope);
  if (url.origin !== scope.origin || !url.pathname.startsWith(scope.pathname)) return false;
  const relativePath = url.pathname
    .slice(scope.pathname.length)
    .replace(/^\/+|\/+$/g, '');
  return APP_SHELL_PATHS.has(relativePath);
}

async function networkFirst(request, timeoutMs = 2000, cacheKey = request, fallbackKeys = []) {
  const cache = await caches.open(CACHE);
  let timeoutId = 0;
  const timeout = new Promise((_, reject) => {
    timeoutId = setTimeout(() => reject(new Error('network timeout')), timeoutMs);
  });
  try {
    const response = await Promise.race([fetch(request), timeout]);
    clearTimeout(timeoutId);
    if (response && response.ok && cacheKey) cache.put(cacheKey, response.clone()).catch(() => {});
    return response;
  } catch (_) {
    clearTimeout(timeoutId);
    for (const candidate of [cacheKey, ...fallbackKeys]) {
      if (!candidate) continue;
      const cached = await cache.match(candidate);
      if (cached) return cached;
    }
    throw _;
  }
}

async function cacheFirst(request) {
  const cache = await caches.open(CACHE);
  const cached = await cache.match(request);
  if (cached) return cached;
  const response = await fetch(request);
  if (response && response.ok) cache.put(request, response.clone()).catch(() => {});
  return response;
}

self.addEventListener('fetch', event => {
  const request = event.request;
  if (request.headers.get('accept')?.includes('text/event-stream')) {
    event.respondWith(fetch(request));
    return;
  }
  if (request.method !== 'GET') return;
  const url = new URL(request.url);
  if (url.pathname.includes('/api/session/current/live_dashboard.json')) {
    return;
  }
  if (url.pathname.includes('/api/')) {
    return;
  }
  if (isAppShellRequest(url)) {
    event.respondWith(networkFirst(request, 2000, NAVIGATION_CACHE_KEY, [NAVIGATION_MONOLITH_KEY]));
    return;
  }
  if (request.mode === 'navigate') {
    // Pairing/support pages are network-only. In particular, never replace the
    // canonical app shell with a response rendered from a URL carrying a PIN.
    event.respondWith(fetch(request));
    return;
  }
  if (url.pathname.endsWith('/manifest.webmanifest')) {
    event.respondWith(networkFirst(request, 2000));
    return;
  }
  if (url.pathname.includes('/icons/') || url.pathname.includes('/lib/') || url.pathname.includes('/fonts/')) {
    event.respondWith(cacheFirst(request));
  }
});
