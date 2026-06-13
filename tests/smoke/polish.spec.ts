import { test, expect, type Page, type Route } from '@playwright/test';
import { seedFirstRunComplete } from './helpers/first-run';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';

async function seedUnlocked(page: Page): Promise<void> {
  await seedFirstRunComplete(page);
  await page.addInitScript(() => {
    sessionStorage.setItem('rvt-operator-token', 'mock-polish-token');
    const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
    setup.operator_label = 'Operator A';
    localStorage.setItem('rvt-setup', JSON.stringify(setup));
  });
}

async function applyPolishRoutes(page: Page, options: { offline?: boolean } = {}): Promise<void> {
  await page.route('**/api/**', async (route: Route) => {
    const url = new URL(route.request().url());
    const path = url.pathname;
    if (options.offline && (path === '/api/health' || path === '/api/status')) {
      await route.abort('failed');
      return;
    }

    if (path === '/api/health') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ ok: true }) });
      return;
    }
    if (path === '/api/status') {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ ok: true, mode: 'live', active_session: null, trainer_version: '16.3.0', dashboard_version: '16.3.0' })
      });
      return;
    }
    if (path === '/api/auth/validate') {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ ok: true, operator: { operator_id: 'op_polish', display_name: 'Operator A', initials: 'OA' } })
      });
      return;
    }
    if (path === '/api/operator-profiles') {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ schema_version: 'rvt-operator-profiles-v12.0', profiles: [] })
      });
      return;
    }
    if (path === '/api/defaults') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ radar_port: 'COM10', serial_ports: ['COM10'] }) });
      return;
    }
    if (path === '/api/serial/ports') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ ports: [{ device: 'COM10' }], selected: 'COM10' }) });
      return;
    }
    if (path === '/api/subject-profiles') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ profiles: {} }) });
      return;
    }
    if (path === '/api/preflight') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ checks: [] }) });
      return;
    }
    if (path === '/api/sessions') {
      await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ items: [] }) });
      return;
    }

    await route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ ok: true }) });
  });
}

async function gotoRoute(page: Page, path: '/home' | '/live' | '/report'): Promise<void> {
  await page.goto(path === '/live' ? DASHBOARD : path, { waitUntil: 'domcontentloaded' });
  if (path !== '/live') {
    await expect(page).toHaveURL(new RegExp(`${path}$`));
  }
  await page.locator('.initial-loading-overlay').waitFor({ state: 'hidden', timeout: 15000 }).catch(() => {});
  await expect(page.locator('section.idle-lock-overlay')).toHaveCount(0, { timeout: 15000 });
}

test.use({ serviceWorkers: 'block' });

test.describe('WS2-B polish', () => {
  test('captures and dismisses the Home PWA install banner', async ({ page }) => {
    await seedUnlocked(page);
    await applyPolishRoutes(page);
    await gotoRoute(page, '/home');

    await page.evaluate(() => {
      const event = new Event('beforeinstallprompt', { cancelable: true }) as Event & {
        prompt?: () => Promise<void>;
        userChoice?: Promise<{ outcome: string; platform: string }>;
      };
      event.prompt = () => Promise.resolve();
      event.userChoice = Promise.resolve({ outcome: 'dismissed', platform: 'web' });
      window.dispatchEvent(event);
    });

    const banner = page.getByRole('region', { name: /Install Radar Vital/i });
    await expect(banner).toBeVisible();
    await banner.getByRole('button', { name: /Dismiss install prompt/i }).click();
    await expect(banner).toHaveCount(0);
    await expect.poll(() => page.evaluate(() => localStorage.getItem('rvt-install-prompt-dismissed'))).toBe('1');
  });

  test('surfaces Home and Report empty states without invented data', async ({ page }) => {
    await seedUnlocked(page);
    await applyPolishRoutes(page);

    await gotoRoute(page, '/home');
    await expect(page.getByText('No telemetry data loaded yet')).toBeVisible();
    await expect(page.getByText('No preflight results yet')).toBeVisible();
    await expect(page.getByText('No matching sessions yet')).toBeVisible();

    await gotoRoute(page, '/report');
    await expect(page.getByText('No telemetry sessions found')).toBeVisible();
    await expect(page.getByRole('button', { name: /Retry sessions/i })).toBeVisible();
  });

  test('keeps Live disconnected state explicit when demo mode is off', async ({ page }) => {
    await seedUnlocked(page);
    await applyPolishRoutes(page, { offline: true });

    await gotoRoute(page, '/live');

    await expect(page.getByRole('heading', { name: /Live telemetry is disconnected/i })).toBeVisible();
    await expect(page.getByText(/will not synthesize vitals/i)).toBeVisible();
    await expect(page.getByRole('button', { name: /Retry connection|Start server/i })).toBeVisible();
  });
});
