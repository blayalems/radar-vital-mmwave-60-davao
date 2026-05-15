import { test, expect } from '@playwright/test';

test.describe('Trainer API smoke', () => {
  test('/api/health returns 200 with ok:true', async ({ request }) => {
    const resp = await request.get('/api/health');
    expect(resp.status()).toBe(200);
    const json = await resp.json();
    // The trainer wraps responses in schema envelope or returns ok:true directly.
    const payload = json.data ?? json;
    expect(payload.ok ?? json.ok).toBe(true);
  });

  test('/api/version reports trainer/dashboard/firmware', async ({ request }) => {
    const resp = await request.get('/api/version');
    expect(resp.status()).toBe(200);
    const json = await resp.json();
    const payload = json.data ?? json;
    expect(payload.trainer).toBeTruthy();
    expect(payload.dashboard).toBeTruthy();
    expect(payload.firmware_expected).toBeTruthy();
  });

  test('/api/status returns trainer + active_session shape', async ({ request }) => {
    const resp = await request.get('/api/status');
    expect(resp.status()).toBe(200);
    const json = await resp.json();
    const payload = json.data ?? json;
    expect(payload.trainer_version).toBeTruthy();
    expect(payload.dashboard_version).toBeTruthy();
  });

  test('/manifest.webmanifest is a valid PWA manifest', async ({ request }) => {
    const resp = await request.get('/manifest.webmanifest');
    expect(resp.status()).toBe(200);
    expect(resp.headers()['content-type']).toMatch(/manifest|json/);
    const json = await resp.json();
    expect(json.name).toBeTruthy();
    expect(json.start_url).toBeTruthy();
    expect(json.display).toBe('standalone');
    expect(Array.isArray(json.icons)).toBe(true);
  });

  test('/sw.js serves the PWA service worker shell', async ({ request }) => {
    const resp = await request.get('/sw.js');
    expect(resp.status()).toBe(200);
    expect(resp.headers()['content-type']).toMatch(/javascript/);
    const body = await resp.text();
    // The new SW advertises its cache namespace and ships network-first/cache-first strategies.
    expect(body).toContain('rvt-shell-v12');
    expect(body).toMatch(/networkFirst|cacheFirst/);
    // The new SW must not self-unregister — that behavior lives only in /rvt-sw.js.
    expect(body).not.toMatch(/self\.registration\.unregister\s*\(\s*\)/);
  });

  test('/rvt-sw.js still serves the unregister tombstone for migration', async ({ request }) => {
    const resp = await request.get('/rvt-sw.js');
    expect(resp.status()).toBe(200);
    const body = await resp.text();
    // The tombstone must actually call unregister — not just mention it in a comment.
    expect(body).toMatch(/self\.registration\.unregister\s*\(\s*\)/);
  });

  test('/api/server-info advertises host, port, and pair state', async ({ request }) => {
    const resp = await request.get('/api/server-info');
    expect(resp.status()).toBe(200);
    const json = await resp.json();
    const payload = json.data ?? json;
    expect(payload.host).toBeTruthy();
    expect(typeof payload.port).toBe('number');
    expect(typeof payload.pair_required).toBe('boolean');
  });

  test('/api/auth/exchange rejects an unknown PIN with 401', async ({ request }) => {
    const resp = await request.post('/api/auth/exchange', { data: { pin: '000000' } });
    // 401 (PIN unknown) or 403 (auth disabled in --mock without --pair). Both are valid;
    // a 200 with a real token would be a security regression.
    expect([401, 403]).toContain(resp.status());
  });

  test('SSE /api/session/events responds with text/event-stream and at least one event', async ({ page }) => {
    // Load a real same-origin page first so the in-page fetch is a same-origin request
    // and we can read response.status + the stream body.
    await page.goto('/radar_vital_live_dashboard_v12_for_v16_0.html');
    const result = await page.evaluate(async () => {
      const ctrl = new AbortController();
      try {
        const resp = await fetch('/api/session/events', { headers: { Accept: 'text/event-stream' }, signal: ctrl.signal });
        const ct = resp.headers.get('content-type') || '';
        if (!resp.body) {
          ctrl.abort();
          return { status: resp.status, contentType: ct, gotChunk: false };
        }
        const reader = resp.body.getReader();
        const timer = setTimeout(() => { try { ctrl.abort(); } catch (_) {} }, 4000);
        let gotChunk = false;
        let firstChunk = '';
        try {
          const { value } = await reader.read();
          if (value && value.byteLength > 0) {
            gotChunk = true;
            firstChunk = new TextDecoder().decode(value).slice(0, 80);
          }
        } catch (_) {
          // aborted or stream ended
        }
        clearTimeout(timer);
        try { ctrl.abort(); } catch (_) {}
        return { status: resp.status, contentType: ct, gotChunk, firstChunk };
      } catch (e) {
        return { status: 0, contentType: '', gotChunk: false, error: String(e) };
      }
    });
    expect(result.status, `SSE status: ${JSON.stringify(result)}`).toBe(200);
    expect(result.contentType).toMatch(/event-stream/);
    expect(result.gotChunk, `expected at least one SSE chunk: ${JSON.stringify(result)}`).toBe(true);
  });
});
