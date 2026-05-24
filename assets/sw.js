const CACHE = 'rvt-shell-v12.0.2';
const DASHBOARD = './index.html';
const MONOLITH = './radar_vital_live_dashboard_v12_for_v16_0.html';
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

async function networkFirst(request, timeoutMs = 2000, shellFallback = false) {
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
    const cached = await cache.match(request)
      || (shellFallback ? await cache.match(DASHBOARD) : null)
      || (shellFallback ? await cache.match(MONOLITH) : null);
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
  if (url.pathname.includes('/api/session/current/live_dashboard.json')) {
    return;
  }
  if (url.pathname.includes('/api/')) {
    return;
  }
  if (url.pathname.endsWith('/') || url.pathname.endsWith('/radar_vital_live_dashboard_v12_for_v16_0.html') || url.pathname.endsWith('/index.html')) {
    event.respondWith(networkFirst(request, 2000, true));
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
