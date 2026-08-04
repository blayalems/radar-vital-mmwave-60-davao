import { test, expect } from '@playwright/test';
import { seedFirstRunComplete } from './helpers/first-run';

test.describe('Study Model Lab browser contract', () => {
  test.use({ serviceWorkers: 'block' });

  test.beforeEach(async ({ page }) => {
    await seedFirstRunComplete(page);
    await page.addInitScript(() => {
      const token = 'model-lab-browser-token';
      localStorage.setItem('rvt-demo-mode', '1');
      localStorage.setItem('demo:rvt-operator-profiles', JSON.stringify({
        schema_version: 'rvt-sandbox-operator-profiles-v12.0',
        profiles: [{ operator_id: 'model-lab-op', display_name: 'Model Lab Operator', initials: 'ML', pin: '123456', failed_attempts: 0, locked_until: 0 }]
      }));
      sessionStorage.setItem('rvt-operator-token', token);
      sessionStorage.setItem('demo:rvt-operator-sessions', JSON.stringify({
        [token]: { operator_id: 'model-lab-op', expires_at: Date.now() + 3600000 }
      }));
    });
  });

  test('exposes both objectives and both model families without horizontal overflow', async ({ page }) => {
    await page.goto('/report', { waitUntil: 'domcontentloaded' });
    const card = page.locator('mat-card[aria-label="Study model lab"]');
    await expect(card).toBeVisible();
    await expect(card.getByRole('radio', { name: 'RR correction (confirmatory)' })).toBeVisible();
    await expect(card.getByRole('radio', { name: 'HR correction (exploratory)' })).toBeVisible();
    await expect(card.getByRole('radio', { name: 'GBR (default)' })).toBeVisible();
    await expect(card.getByRole('radio', { name: '1-D CNN (experimental)' })).toBeVisible();

    await card.getByRole('radio', { name: 'HR correction (exploratory)' }).click();
    await card.getByRole('radio', { name: '1-D CNN (experimental)' }).click();
    await expect(card.getByRole('radio', { name: 'HR correction (exploratory)' })).toHaveAttribute('aria-checked', 'true');
    await expect(card.getByRole('radio', { name: '1-D CNN (experimental)' })).toHaveAttribute('aria-checked', 'true');

    const dimensions = await page.evaluate(() => ({
      scrollWidth: document.documentElement.scrollWidth,
      clientWidth: document.documentElement.clientWidth
    }));
    expect(dimensions.scrollWidth).toBe(dimensions.clientWidth);
  });
});
