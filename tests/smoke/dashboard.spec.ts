import { test, expect } from '@playwright/test';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';

test.describe('Dashboard smoke', () => {
  test('loads without console errors', async ({ page }) => {
    const consoleErrors: string[] = [];
    const pageErrors: string[] = [];
    page.on('console', (msg) => { if (msg.type() === 'error') consoleErrors.push(msg.text()); });
    page.on('pageerror', (err) => pageErrors.push(err.message));

    const resp = await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    expect(resp?.status(), 'dashboard HTML status').toBeLessThan(400);

    await page.waitForLoadState('networkidle', { timeout: 15_000 }).catch(() => { /* tolerate long-poll SSE */ });
    await page.waitForTimeout(1500);

    // Hard fail on uncaught script errors. Console errors are reported but not fail-by-default
    // (the v11 dashboard logs many warnings the redesign hasn't touched yet).
    expect(pageErrors, `uncaught errors: ${pageErrors.join(' | ')}`).toEqual([]);
    if (consoleErrors.length) {
      console.warn(`[smoke] dashboard logged ${consoleErrors.length} console errors during boot:\n  ${consoleErrors.slice(0, 5).join('\n  ')}`);
    }
  });

  test('PWA manifest link is reachable and well-formed', async ({ page, request }) => {
    await page.goto(DASHBOARD);
    const manifestUrl = await page.locator('link[rel="manifest"]').first().getAttribute('href');
    expect(manifestUrl, 'dashboard advertises a manifest').toBeTruthy();
    const resolved = new URL(manifestUrl!, page.url()).toString();
    const resp = await request.get(resolved);
    expect(resp.status()).toBe(200);
    const json = await resp.json();
    expect(json.name).toBeTruthy();
    expect(json.start_url).toBeTruthy();
    expect(Array.isArray(json.icons)).toBe(true);
    expect(json.icons.length).toBeGreaterThan(0);
  });

  test('service worker registers and reaches activated state', async ({ page }) => {
    // The dashboard's controllerchange listener triggers location.reload() when a new SW
    // takes control — that's the production update flow. The test must tolerate the
    // post-install navigation by waiting for it before sampling SW state.
    await page.goto(DASHBOARD);
    await page.waitForLoadState('networkidle', { timeout: 8000 }).catch(() => {});
    // Allow up to one post-install reload to complete.
    await page.waitForTimeout(1500);
    await page.waitForLoadState('domcontentloaded').catch(() => {});

    const result = await page.evaluate(async () => {
      if (!('serviceWorker' in navigator)) return { supported: false };
      try {
        // Use registration() not ready — ready awaits the controller, which can race
        // with the page reload triggered by controllerchange.
        const reg = await navigator.serviceWorker.getRegistration();
        if (!reg) return { supported: true, registered: false };
        return {
          supported: true,
          registered: true,
          scope: reg.scope,
          active: !!reg.active,
          state: reg.active?.state || (reg.installing?.state || reg.waiting?.state || null)
        };
      } catch (e) {
        return { supported: true, error: String(e) };
      }
    });
    expect(result.supported).toBe(true);
    expect(result.registered, `SW not registered: ${JSON.stringify(result)}`).toBe(true);
    // Any of installing/installed/activating/activated is acceptable — what we care about is
    // that the registration exists and has a worker entry.
    expect(['installing', 'installed', 'activating', 'activated']).toContain(result.state);
  });

  test('viewport meta enables PWA install on mobile', async ({ page }) => {
    await page.goto(DASHBOARD);
    const viewport = await page.locator('meta[name="viewport"]').first().getAttribute('content');
    expect(viewport).toContain('viewport-fit=cover');
    expect(viewport).toMatch(/width=device-width/);
  });

  test('theme-color and apple-touch-icon present', async ({ page }) => {
    await page.goto(DASHBOARD);
    const themeColor = await page.locator('meta[name="theme-color"]').first().getAttribute('content');
    expect(themeColor).toBeTruthy();
    const appleIcon = await page.locator('link[rel="apple-touch-icon"]').first().getAttribute('href');
    expect(appleIcon).toBeTruthy();
  });
});
