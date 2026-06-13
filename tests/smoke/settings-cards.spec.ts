import { test, expect, type Page } from '@playwright/test';
import { seedFirstRunComplete } from './helpers/first-run';

async function seedUnlockedOperator(page: Page): Promise<void> {
  await seedFirstRunComplete(page);
  await page.addInitScript(() => {
    const token = 'mock-settings-token';
    const operator = { operator_id: 'op_settings', display_name: 'Settings Tester', initials: 'ST', pin: '1234' };
    const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');

    sessionStorage.setItem('rvt-operator-token', token);
    sessionStorage.setItem('demo:rvt-operator-sessions', JSON.stringify({
      [token]: {
        operator_id: operator.operator_id,
        expires_at: Date.now() + 8 * 60 * 60 * 1000
      }
    }));
    localStorage.setItem('rvt-demo-mode', '1');
    localStorage.setItem('demo:rvt-operator-profiles', JSON.stringify({
      schema_version: 'rvt-sandbox-operator-profiles-v12.0',
      profiles: [operator]
    }));
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

async function gotoUnlocked(page: Page, path: string): Promise<void> {
  await page.goto(path, { waitUntil: 'domcontentloaded' });
  await expect(page.locator('.idle-lock-overlay')).toHaveCount(0, { timeout: 8000 });
}

test.use({ serviceWorkers: 'block' });

test.describe('Settings support cards and help links', () => {
  test.beforeEach(async ({ page }) => {
    await seedUnlockedOperator(page);
  });

  test('inserts privacy, about, and issue cards in semantic settings order', async ({ page }) => {
    await gotoUnlocked(page, '/settings');
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
    await expect(page.getByRole('link', { name: /View privacy/i })).toHaveAttribute('href', /PRIVACY\.md$/);
    await expect(page.getByRole('button', { name: /Withdraw consent/i })).toBeVisible();
  });

  test('exposes support commands in the command palette', async ({ page }) => {
    await gotoUnlocked(page, '/settings');
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
    await gotoUnlocked(page, '/help');

    await expect(page.getByRole('button', { name: /Replay tutorial/i })).toBeVisible();
    await expect(page.getByRole('link', { name: /Terms of Use/i })).toHaveAttribute('href', /TERMS\.md$/);
    await expect(page.getByRole('link', { name: /Privacy Policy/i })).toHaveAttribute('href', /PRIVACY\.md$/);
    await expect(page.getByRole('link', { name: /License/i })).toHaveAttribute('href', /LICENSE$/);
  });
});
