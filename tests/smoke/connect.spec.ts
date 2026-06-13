import { test, expect } from '@playwright/test';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';

test.describe('Connect Wizard first-run onboarding', () => {
  test.use({ serviceWorkers: 'block' });

  test.beforeEach(async ({ page }) => {
    await page.route('**/api/auth/validate', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          ok: true,
          operator: { operator_id: 'op_test', display_name: 'Operator A', initials: 'OA' }
        })
      });
    });
    await page.route('**/api/operator-profiles', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          schema_version: 'rvt-operator-profiles-v12.0',
          profiles: [{ operator_id: 'op_test', display_name: 'Operator A', initials: 'OA' }]
        })
      });
    });
  });

  test('redirects to /connect on first run (server URL missing)', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.waitForURL(/\/connect$/);
    await expect(page.getByText('Welcome to Radar Vital')).toBeVisible();
    await expect(page.getByRole('button', { name: /Demo Now/ })).toBeVisible();
    await expect(page.getByRole('button', { name: /Scan Pairing QR Code/ })).toBeVisible();
  });

  test('Demo Now button configures sandbox mode and redirects to /live', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.waitForURL(/\/connect$/);

    // Mock api/status and other endpoints that /live might query
    await page.route('**/api/status', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ trainer_version: '1.0.0', dashboard_version: '1.0.0', active_session: null })
      });
    });

    await page.getByRole('button', { name: /Demo Now/ }).click();
    await page.waitForURL(/\/live$/);

    const demoMode = await page.evaluate(() => localStorage.getItem('rvt-demo-mode'));
    const serverUrl = await page.evaluate(() => localStorage.getItem('rvt.server.url'));
    expect(demoMode).toBe('1');
    expect(serverUrl).toBe('http://127.0.0.1:8765');
  });

  test('allows manual entry of address and PIN', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.waitForURL(/\/connect$/);

    // Pair against the page's own origin: when the dashboard is trainer-served
    // its CSP pins connect-src to 'self', so the pairing fetch must be
    // same-origin (the supported EXE/desktop trainer-served case). The hosted
    // static PWA has no such CSP and can pair cross-origin with a LAN trainer.
    const trainerOrigin = new URL(page.url()).origin;

    await page.getByRole('button', { name: /Enter Address & PIN Manually/ }).click();
    await page.getByLabel('Trainer Address').fill(trainerOrigin);
    await page.getByLabel('6-Digit Pairing PIN').fill('123456');

    // Mock the pairing exchange. The wizard pairs via ApiService.exchangePairPin,
    // which POSTs to the trainer's /api/auth/exchange endpoint.
    let pairCalled = false;
    await page.route('**/api/auth/exchange', async (route) => {
      pairCalled = true;
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ ok: true, token: 'mock-token' })
      });
    });

    await page.route('**/api/status', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ ok: true, mode: 'live', trainer_version: '1.0.0', dashboard_version: '1.0.0', active_session: null })
      });
    });

    await page.getByRole('button', { name: /Connect & Pair/ }).click();
    await page.waitForURL(/\/live$/);

    expect(pairCalled).toBe(true);
    const serverUrl = await page.evaluate(() => localStorage.getItem('rvt.server.url'));
    expect(serverUrl).toBe(trainerOrigin);
  });
});
