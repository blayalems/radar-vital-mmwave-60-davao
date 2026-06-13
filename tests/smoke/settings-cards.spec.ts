import { test, expect, type Page } from '@playwright/test';
import { seedFirstRunComplete } from './helpers/first-run';

async function seedUnlockedOperator(page: Page): Promise<void> {
  await seedFirstRunComplete(page);
  await page.addInitScript(() => {
    sessionStorage.setItem('rvt-operator-token', 'mock-settings-token');
    localStorage.setItem('rvt-demo-mode', '1');
    const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
    setup.operator_label = 'Settings Tester';
    localStorage.setItem('rvt-setup', JSON.stringify(setup));
  });
  await page.route('**/api/auth/validate', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({
      ok: true,
      operator: { operator_id: 'op_settings', display_name: 'Settings Tester', initials: 'ST' }
    })
  }));
  await page.route('**/api/help/schema', route => route.fulfill({
    status: 200,
    contentType: 'application/json',
    body: JSON.stringify({ version: 'test-schema', glossary: [], faq: [], tooltips: {}, dsp_steps: [], troubleshooting: [] })
  }));
}

test.use({ serviceWorkers: 'block' });

test.describe('Settings support cards and help links', () => {
  test.beforeEach(async ({ page }) => {
    await seedUnlockedOperator(page);
  });

  test('inserts privacy, about, and issue cards in semantic settings order', async ({ page }) => {
    await page.goto('/settings', { waitUntil: 'domcontentloaded' });
    await expect(page.locator('h2.settings-title')).toHaveText('Dashboard Settings');

    const titles = (await page.locator('.settings-grid mat-card-title').allTextContents()).map(title => title.trim());
    const privacyLock = titles.indexOf('Privacy Screen Lock');
    const privacyTelemetry = titles.indexOf('Privacy & Telemetry');
    const updates = titles.indexOf('Update & Version Info');
    const about = titles.indexOf('About Radar Vital');
    const reportIssue = titles.indexOf('Report an Issue');

    expect(privacyLock).toBeGreaterThanOrEqual(0);
    expect(privacyTelemetry).toBeGreaterThan(privacyLock);
    expect(updates).toBeGreaterThanOrEqual(0);
    expect(about).toBeGreaterThan(updates);
    expect(reportIssue).toBeGreaterThan(about);

    await expect(page.getByRole('button', { name: /Review diagnostics toggle/i })).toBeVisible();
    await expect(page.getByRole('link', { name: /View terms/i })).toHaveAttribute('href', /TERMS\.md$/);
    await expect(page.getByRole('button', { name: /Withdraw consent/i })).toBeVisible();
  });

  test('exposes support commands in the command palette', async ({ page }) => {
    await page.goto('/settings', { waitUntil: 'domcontentloaded' });
    await page.keyboard.press('Control+K');

    const dialog = page.getByRole('dialog');
    await expect(dialog).toContainText('Command Palette');

    await page.getByLabel('Search commands').fill('Report an issue');
    await expect(dialog.getByRole('button', { name: /Report an issue/i })).toBeVisible();

    await page.getByLabel('Search commands').fill('About legal');
    await expect(dialog.getByRole('button', { name: /About & legal/i })).toBeVisible();

    await page.getByLabel('Search commands').fill('Install app');
    await expect(dialog.getByRole('button', { name: /Install app/i })).toBeVisible();

    await page.getByLabel('Search commands').fill('Replay tutorial');
    await dialog.getByRole('button', { name: /Replay tutorial/i }).dispatchEvent('click');
    await expect(page.locator('app-onboarding-tutorial')).toBeVisible({ timeout: 8000 });
  });

  test('adds replay tutorial and legal links to Help', async ({ page }) => {
    await page.goto('/help', { waitUntil: 'domcontentloaded' });

    await expect(page.getByRole('button', { name: /Replay tutorial/i })).toBeVisible();
    await expect(page.getByRole('link', { name: /Terms of Use/i })).toHaveAttribute('href', /TERMS\.md$/);
    await expect(page.getByRole('link', { name: /Privacy Policy/i })).toHaveAttribute('href', /PRIVACY\.md$/);
    await expect(page.getByRole('link', { name: /License/i })).toHaveAttribute('href', /LICENSE$/);
  });
});
