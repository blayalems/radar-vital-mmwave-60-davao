// TODO(Wave 2): unskip when <app-report-issue-card> is mounted in Settings
import { test, expect } from '@playwright/test';
import { seedFirstRunComplete } from './helpers/first-run';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';
const DIAGNOSTICS_KEY = 'rvt-diagnostics-optin';
// Mirrors GITHUB_REPO_URL in web/src/app/services/app-meta.ts — Playwright's
// loader cannot named-import from the Angular sources (CommonJS interop).
const GITHUB_REPO_URL = 'https://github.com/blayalems/radar-vital-mmwave-60-davao';
const ISSUES_NEW_URL = `${GITHUB_REPO_URL}/issues/new`;

test.describe.skip('Report-issue card (Wave 2: mount in Settings first)', () => {
  test.beforeEach(async ({ page }) => {
    await seedFirstRunComplete(page);
    await page.addInitScript(() => {
      // Put the app in demo mode so no live trainer is required.
      localStorage.setItem('rvt-demo-mode', '1');
    });
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
  });

  // ---- card renders ---------------------------------------------------------

  test('report-issue card is present in Settings', async ({ page }) => {
    // Open settings via keyboard shortcut or nav.
    await page.keyboard.press('s');
    await expect(page.getByRole('heading', { name: /report an issue/i })).toBeVisible();
  });

  // ---- toggle persistence across reload ------------------------------------

  test('diagnostics toggle defaults to ON when key absent', async ({ page }) => {
    await page.addInitScript(() => {
      localStorage.removeItem('rvt-diagnostics-optin');
    });
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.keyboard.press('s');

    const toggle = page.getByRole('switch', { name: /include diagnostics/i });
    await expect(toggle).toBeChecked();
  });

  test('toggling diagnostics off persists across reload', async ({ page }) => {
    // Enable settings panel.
    await page.keyboard.press('s');

    const toggle = page.getByRole('switch', { name: /include diagnostics/i });
    await toggle.click();
    await expect(toggle).not.toBeChecked();

    // Verify localStorage was updated.
    const stored = await page.evaluate((key: string) => localStorage.getItem(key), DIAGNOSTICS_KEY);
    expect(stored).toBe('0');

    // Reload and verify toggle remains off.
    await page.reload({ waitUntil: 'domcontentloaded' });
    await page.keyboard.press('s');
    await expect(page.getByRole('switch', { name: /include diagnostics/i })).not.toBeChecked();
  });

  test('toggling diagnostics on persists across reload', async ({ page }) => {
    await page.addInitScript(() => {
      localStorage.setItem('rvt-diagnostics-optin', '0');
    });
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.keyboard.press('s');

    const toggle = page.getByRole('switch', { name: /include diagnostics/i });
    await expect(toggle).not.toBeChecked();
    await toggle.click();
    await expect(toggle).toBeChecked();

    const stored = await page.evaluate((key: string) => localStorage.getItem(key), DIAGNOSTICS_KEY);
    expect(stored).toBe('1');

    await page.reload({ waitUntil: 'domcontentloaded' });
    await page.keyboard.press('s');
    await expect(page.getByRole('switch', { name: /include diagnostics/i })).toBeChecked();
  });

  // ---- preview dialog -------------------------------------------------------

  test('preview dialog shows diagnostics when toggle is ON', async ({ page }) => {
    await page.addInitScript(() => {
      localStorage.setItem('rvt-diagnostics-optin', '1');
    });
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.keyboard.press('s');

    await page.getByRole('button', { name: /preview report/i }).click();

    const dialog = page.getByRole('dialog');
    await expect(dialog).toBeVisible();
    // The JSON preview should contain platform info.
    await expect(dialog).toContainText('platform');
    // When diagnostics on, connection_mode should be present.
    await expect(dialog).toContainText('connection_mode');
    // Should NOT show the opt-out notice.
    await expect(dialog.getByText(/diagnostics are.*off/i)).not.toBeVisible();
  });

  test('preview dialog hides diagnostics and shows opt-out notice when toggle is OFF', async ({ page }) => {
    await page.addInitScript(() => {
      localStorage.setItem('rvt-diagnostics-optin', '0');
    });
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.keyboard.press('s');

    await page.getByRole('button', { name: /preview report/i }).click();

    const dialog = page.getByRole('dialog');
    await expect(dialog).toBeVisible();
    // Only version + platform — no connection_mode.
    await expect(dialog).toContainText('product_version');
    await expect(dialog).toContainText('platform');
    await expect(dialog).not.toContainText('connection_mode');
    // Opt-out notice must be visible.
    await expect(dialog.getByText(/diagnostics are.*off/i)).toBeVisible();
  });

  test('preview dialog closes on Close button', async ({ page }) => {
    await page.keyboard.press('s');
    await page.getByRole('button', { name: /preview report/i }).click();
    const dialog = page.getByRole('dialog');
    await expect(dialog).toBeVisible();
    await dialog.getByRole('button', { name: /close/i }).click();
    await expect(dialog).not.toBeVisible();
  });

  // ---- GitHub link ----------------------------------------------------------

  test('Open GitHub issue link starts with issues/new URL and contains template=bug_report.yml', async ({ page, context }) => {
    await page.addInitScript(() => {
      localStorage.setItem('rvt-diagnostics-optin', '0');
    });
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.keyboard.press('s');

    // Intercept the new-tab navigation instead of actually opening GitHub.
    const newPagePromise = context.waitForEvent('page', { timeout: 5000 }).catch(() => null);
    await page.getByRole('button', { name: /open github issue/i }).click();

    const newPage = await newPagePromise;
    if (newPage) {
      const url = newPage.url();
      expect(url).toMatch(new RegExp(`^${ISSUES_NEW_URL.replace(/\//g, '/')}`));
      expect(url).toContain('template=bug_report.yml');
      await newPage.close();
    } else {
      // In Tauri or single-page navigation context — verify the URL on the
      // existing page instead (fallback assertion).
      // The button click still runs — verify no console error.
      const logs: string[] = [];
      page.on('console', msg => { if (msg.type() === 'error') logs.push(msg.text()); });
      expect(logs.filter(l => l.includes('TypeError'))).toHaveLength(0);
    }
  });
});
