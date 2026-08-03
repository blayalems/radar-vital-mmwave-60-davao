import { expect, test, type Page } from '@playwright/test';
import { seedFirstRunComplete } from './helpers/first-run';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';

async function seedUnlockedLiveOperator(page: Page): Promise<void> {
  await seedFirstRunComplete(page);
  await page.addInitScript(() => {
    sessionStorage.setItem('rvt-operator-token', 'release-test-token');
    localStorage.setItem('rvt.server.url', location.origin);
    localStorage.removeItem('rvt-demo-mode');
    const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
    setup.operator_label = 'Release Tester';
    localStorage.setItem('rvt-setup', JSON.stringify(setup));
  });
  await page.route('**/api/auth/validate', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({
      ok: true,
      operator: {
        operator_id: 'op_release',
        display_name: 'Release Tester',
        initials: 'RT'
      }
    })
  }));
}

async function mockHomeDependencies(page: Page): Promise<void> {
  await page.route('**/api/status', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({
      ok: true,
      mode: 'live',
      trainer_version: '16.5.2',
      dashboard_version: '16.5.2',
      firmware_expected: 'v16.5.2',
      serial_protocol: 'v15.1',
      serial_width_expected: 219,
      schema_versions: {
        control_api: 'rvt-control-api-v12.0',
        study_session: 'rvt-study-session-v16.5.1'
      },
      active_session: null
    })
  }));
  await page.route('**/api/version', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({
      product_version: '16.5.3',
      trainer: '16.5.2',
      dashboard: '16.5.2',
      firmware_expected: 'v16.5.2',
      serial_protocol: 'v15.1',
      serial_width_expected: 219,
      schema_versions: {
        control_api: 'rvt-control-api-v12.0',
        study_session: 'rvt-study-session-v16.5.1'
      }
    })
  }));
  await page.route('**/api/defaults', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({})
  }));
  await page.route('**/api/subject-profiles', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({ items: [] })
  }));
  await page.route('**/api/participants', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({ participants: [] })
  }));
  await page.route('**/api/sessions**', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({ items: [] })
  }));
  await page.route('**/api/serial/ports', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({ ports: [] })
  }));
  await page.route('**/api/preflight**', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({ checks: [] })
  }));
}

test.describe('release mismatch start guidance', () => {
  test.use({ serviceWorkers: 'block' });

  test('shows a responsive blocking card with concrete recovery steps', async ({ page }) => {
    await seedUnlockedLiveOperator(page);
    await mockHomeDependencies(page);
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });

    const card = page.locator('.release-compatibility-card');
    await expect(card).toHaveAttribute('data-state', 'incompatible');
    await expect(card.getByText('Release compatibility', { exact: true })).toBeVisible();
    await expect(card.getByText(/Dashboard release: expected .* received 16\.5\.2/)).toBeVisible();
    await expect(card.getByText(/Reload the dashboard/)).toBeVisible();

    const blocker = page.locator('.release-start-blocker');
    await expect(blocker).toContainText('Start blocked');
    await expect(blocker).toContainText('Restart the trainer');
    await expect(blocker).toContainText('Reload the dashboard');

    const box = await card.boundingBox();
    expect(box).not.toBeNull();
    expect(box!.x).toBeGreaterThanOrEqual(0);
    expect(box!.x + box!.width).toBeLessThanOrEqual(
      await page.evaluate(() => document.documentElement.clientWidth)
    );
  });
});
