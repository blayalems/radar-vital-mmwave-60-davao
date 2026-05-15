/* Radar Vital service worker — registered at /sw.js by both the trainer and the GitHub Pages shell.
 *
 * Strategies:
 *   /                                            network-first 2s, cache fallback   (HTML shell)
 *   /radar_vital_live_dashboard_v12_*.html       network-first 2s, cache fallback   (HTML shell)
 *   /manifest.webmanifest                        network-first 2s                   (port-aware in trainer mode)
 *   /icons/*, /lib/*, /fonts/*                   cache-first, immutable             (static assets)
 *   /api/session/events  (SSE)                   explicit passthrough               (streams cannot be cached)
 *   /api/* (other)                               network-only                        (control endpoints must be fresh)
 *   /api/session/current/live_dashboard.json     network-only                        (live data)
 *
 * Cache name bumps on every release. Activate sequence:
 *   1. Delete caches whose key != current version.
 *   2. clients.claim().
 *   3. postMessage({type:'SW_UPDATED'}) to every client so the dashboard can show its update toast.
 *
 * Note: this SW does NOT self-unregister — that behavior is intentionally limited
 * to the legacy tombstone served at /rvt-sw.js for one release.
 */

const CACHE = 'rvt-shell-v12.0.0';
const NETWORK_FIRST_TIMEOUT_MS = 2000;
const HTML_PATHS = [
  '/',
  '/index.html',
  '/radar_vital_live_dashboard_v12_for_v16_0.html',
];
const PRECACHE = [
  '/manifest.webmanifest',
  '/assets/icons/icon-192.png',
  '/assets/icons/icon-512.png',
  '/assets/icons/icon-maskable-512.png',
  '/assets/icons/apple-touch-icon-180.png',
];

self.addEventListener('install', (event) => {
  event.waitUntil((async () => {
    const cache = await caches.open(CACHE);
    await Promise.all(PRECACHE.map(async (url) => {
      try { await cache.add(url); } catch (_) { /* ignore missing assets in dev */ }
    }));
    // Pre-warm one HTML shell entry — pick the first one that resolves.
    for (const p of HTML_PATHS) {
      try { await cache.add(p); break; } catch (_) {}
    }
    self.skipWaiting();
  })());
});

self.addEventListener('activate', (event) => {
  event.waitUntil((async () => {
    const keys = await caches.keys();
    await Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)));
    await self.clients.claim();
    const clients = await self.clients.matchAll();
    clients.forEach((c) => { try { c.postMessage({ type: 'SW_UPDATED', cache: CACHE }); } catch (_) {} });
  })());
});

self.addEventListener('message', (event) => {
  if (event.data && event.data.type === 'SKIP_WAITING') self.skipWaiting();
});

function isHtmlShell(url) {
  if (HTML_PATHS.includes(url.pathname)) return true;
  return /\/radar_vital_live_dashboard_v\d+_for_v\d+_\d+\.html$/.test(url.pathname);
}

function isStaticAsset(url) {
  return (
    url.pathname.startsWith('/assets/icons/') ||
    url.pathname.startsWith('/assets/lib/') ||
    url.pathname.startsWith('/assets/fonts/') ||
    url.pathname === '/manifest.webmanifest'
  );
}

self.addEventListener('fetch', (event) => {
  const req = event.request;
  if (req.method !== 'GET') return;
  const url = new URL(req.url);

  // 1. SSE — explicit passthrough. Streams cannot be cached.
  const accept = req.headers.get('accept') || '';
  if (accept.includes('text/event-stream')) {
    event.respondWith(fetch(req));
    return;
  }

  // 2. /api/* — network-only. Trainer authoritative.
  if (url.pathname.startsWith('/api/')) {
    event.respondWith(fetch(req));
    return;
  }

  // 3. HTML shell — network-first with 2 s timeout, cache fallback.
  if (isHtmlShell(url)) {
    event.respondWith(networkFirst(req));
    return;
  }

  // 4. Manifest — network-first (port-aware in trainer mode).
  if (url.pathname === '/manifest.webmanifest') {
    event.respondWith(networkFirst(req));
    return;
  }

  // 5. Static assets — cache-first.
  if (isStaticAsset(url)) {
    event.respondWith(cacheFirst(req));
    return;
  }

  // 6. Default — try cache, fall back to network.
  event.respondWith(
    caches.match(req).then((hit) => hit || fetch(req).catch(() => caches.match('/')))
  );
});

async function networkFirst(req) {
  const cache = await caches.open(CACHE);
  try {
    const fetchPromise = fetch(req);
    const timeout = new Promise((_, rej) => setTimeout(() => rej(new Error('timeout')), NETWORK_FIRST_TIMEOUT_MS));
    const resp = await Promise.race([fetchPromise, timeout]);
    if (resp && resp.ok) cache.put(req, resp.clone()).catch(() => {});
    return resp;
  } catch (_) {
    const hit = await cache.match(req);
    if (hit) return hit;
    // Final fallback: any cached HTML shell.
    for (const p of HTML_PATHS) {
      const fb = await cache.match(p);
      if (fb) return fb;
    }
    return new Response('Offline and no cached shell available.', { status: 503, headers: { 'Content-Type': 'text/plain' } });
  }
}

async function cacheFirst(req) {
  const cache = await caches.open(CACHE);
  const hit = await cache.match(req);
  if (hit) return hit;
  try {
    const resp = await fetch(req);
    if (resp && resp.ok) cache.put(req, resp.clone()).catch(() => {});
    return resp;
  } catch (e) {
    return new Response('', { status: 504 });
  }
}
