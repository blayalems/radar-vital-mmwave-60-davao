const CACHE = 'rvt-shell-v12.0.1';
const DASHBOARD = '/';
const PRECACHE = [
  DASHBOARD,
  '/radar_vital_live_dashboard_v12_for_v16_0.html',
  '/manifest.webmanifest',
  '/icons/icon-192.png',
  '/icons/icon-512.png',
  '/icons/icon-maskable-512.png',
  '/icons/apple-touch-icon-180.png',
  '/lib/chart.umd.min.js',
  '/lib/chartjs-plugin-zoom.min.js',
  '/lib/hammer.min.js',
  '/lib/jsqr.min.js',
  '/assets/fonts/rvt-fonts.css',
  '/assets/fonts/inter-6.woff2',
  '/assets/fonts/jetbrains-mono-5.woff2',
  '/assets/fonts/material-symbols-rounded.woff2'
];

self.addEventListener('install', event => {
  event.waitUntil(caches.open(CACHE).then(cache => cache.addAll(PRECACHE).catch(() => {})));
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

async function networkFirst(request, timeoutMs = 2000) {
  const cache = await caches.open(CACHE);
  let timeoutId = 0;
  const timeout = new Promise((_, reject) => {
    timeoutId = setTimeout(() => reject(new Error('network timeout')), timeoutMs);
  });
  try {
    const response = await Promise.race([fetch(request), timeout]);
    clearTimeout(timeoutId);
    if (response && response.ok) cache.put(request, response.clone()).catch(() => {});
    return response;
  } catch (_) {
    clearTimeout(timeoutId);
    const cached = await cache.match(request);
    if (cached) return cached;
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
  if (url.pathname === '/api/session/current/live_dashboard.json') {
    return;
  }
  if (url.pathname.startsWith('/api/')) {
    return;
  }
  if (url.pathname === '/' || url.pathname.endsWith('/radar_vital_live_dashboard_v12_for_v16_0.html')) {
    event.respondWith(networkFirst(request, 2000));
    return;
  }
  if (url.pathname === '/manifest.webmanifest') {
    event.respondWith(networkFirst(request, 2000));
    return;
  }
  if (url.pathname.startsWith('/icons/') || url.pathname.startsWith('/lib/') || url.pathname.startsWith('/assets/fonts/')) {
    event.respondWith(cacheFirst(request));
  }
});
