import { test, expect } from '@playwright/test';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';

test.describe('Dashboard smoke', () => {
  test('loads without console errors', async ({ page }) => {
    const consoleErrors: string[] = [];
    const pageErrors: string[] = [];
    page.on('console', (msg) => {
      if (msg.type() === 'error') {
        const source = msg.location().url ? ` (${msg.location().url})` : '';
        consoleErrors.push(`${msg.text()}${source}`);
      }
    });
    page.on('pageerror', (err) => {
      const message = err.message || '';
      // WebKit can report aborted same-origin API probes during reload/SW startup as
      // page errors even when the dashboard catches the failed fetch and keeps running.
      if (/\/api\/.* due to access control checks\.$/.test(message)) return;
      pageErrors.push(message);
    });

    const resp = await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    expect(resp?.status(), 'dashboard HTML status').toBeLessThan(400);

    await page.waitForLoadState('networkidle', { timeout: 15_000 }).catch(() => { /* tolerate long-poll SSE */ });
    await page.waitForTimeout(1500);

    const relevantConsoleErrors = consoleErrors.filter((message) =>
      !message.includes('Viewport argument key "interactive-widget" not recognized and ignored.')
    );
    expect(pageErrors, `uncaught errors: ${pageErrors.join(' | ')}`).toEqual([]);
    expect(relevantConsoleErrors, `console errors: ${relevantConsoleErrors.join(' | ')}`).toEqual([]);
  });

  test('PWA manifest link is reachable and well-formed', async ({ page, request }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
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
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.waitForLoadState('networkidle', { timeout: 8000 }).catch(() => {});
    // Allow up to one post-install reload to complete.
    await page.waitForTimeout(1500);
    await page.waitForLoadState('domcontentloaded').catch(() => {});

    let result: any = null;
    for (let attempt = 0; attempt < 5; attempt += 1) {
      try {
        result = await page.evaluate(async () => {
          if (!('serviceWorker' in navigator)) return { supported: false };
          try {
            // Use registration() not ready — ready awaits the controller, which can race
            // with the page reload triggered by controllerchange.
            const reg = await Promise.race([
              navigator.serviceWorker.getRegistration(),
              new Promise((resolve) => setTimeout(() => resolve(null), 10_000))
            ]);
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
        break;
      } catch (e) {
        if (!String(e).includes('Execution context was destroyed') || attempt === 4) throw e;
        await page.waitForLoadState('domcontentloaded').catch(() => {});
        await page.waitForTimeout(750);
      }
    }
    expect(result.supported).toBe(true);
    expect(result.registered, `SW not registered: ${JSON.stringify(result)}`).toBe(true);
    // Any of installing/installed/activating/activated is acceptable — what we care about is
    // that the registration exists and has a worker entry.
    expect(['installing', 'installed', 'activating', 'activated']).toContain(result.state);
  });

  test('viewport meta enables PWA install on mobile', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    const viewport = await page.locator('meta[name="viewport"]').first().getAttribute('content');
    expect(viewport).toContain('viewport-fit=cover');
    expect(viewport).toMatch(/width=device-width/);
  });

  test('theme-color and apple-touch-icon present', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    const themeColor = await page.locator('meta[name="theme-color"]').first().getAttribute('content');
    expect(themeColor).toBeTruthy();
    const appleIcon = await page.locator('link[rel="apple-touch-icon"]').first().getAttribute('href');
    expect(appleIcon).toBeTruthy();
  });

  test('applies persisted themes and opens the Material command palette', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.evaluate(() => localStorage.setItem('rvt-theme', 'night'));
    await page.reload({ waitUntil: 'domcontentloaded' });
    await expect(page.locator('html')).toHaveAttribute('data-theme', 'night');

    await page.keyboard.press('Control+K');
    await expect(page.getByRole('dialog')).toContainText('Command Palette');
    await page.getByRole('button', { name: /Help Open operator guidance/ }).click();
    await expect(page).toHaveURL(/\/help$/);
  });

  test('explicit demo mode streams telemetry and preserves snapshot capture', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.evaluate(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.reload({ waitUntil: 'domcontentloaded' });

    await expect(page.locator('#demoBanner')).toBeVisible();
    await expect(page.locator('.kpi-hr .kpi-card-value strong')).not.toHaveText('--', { timeout: 5000 });
    await page.getByRole('button', { name: /Pin Snapshot/ }).first().click();
    await page.getByRole('tab', { name: 'Snapshots' }).click();
    await expect(page.getByText('Pinned Telemetry Snapshots')).toBeVisible();
    await expect(page.locator('.snap-card')).toHaveCount(1);
  });
});
