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

  test('renders 10x3 study matrix and 5-min acclimatization cue on Home without overflow', async ({ page }) => {
    await page.goto('/home', { waitUntil: 'domcontentloaded' });
    const matrix = page.getByRole('region', { name: /Participant 10-condition randomized schedule matrix/i });
    await expect(matrix).toBeVisible();
    await expect(matrix.locator('tbody tr')).toHaveCount(10);

    await matrix.locator('tbody tr').first().getByRole('button', { name: /trial 2/i }).click();
    await expect(page.locator('.trial-summary')).toContainText('trial 2');

    await page.getByRole('radio', { name: '0.6 m' }).click();
    await page.getByRole('radio', { name: 'None' }).click();
    await expect(page.getByText(/MLX90614 5-min acclimatization cue/i)).toBeVisible();
    await page.getByRole('button', { name: /Start 5m/i }).click();
    await expect(page.getByRole('button', { name: /Pause/i })).toBeVisible();

    await expect(page.getByRole('region', { name: /No-subject false-alarm progress/i })).toBeVisible();

    const dimensions = await page.evaluate(() => ({
      scrollWidth: document.documentElement.scrollWidth,
      clientWidth: document.documentElement.clientWidth
    }));
    expect(dimensions.scrollWidth).toBe(dimensions.clientWidth);
  });

  test('masks first RR observer until second locks and exports IEEE LaTeX/CSV on Report', async ({ page }) => {
    await page.goto('/report', { waitUntil: 'domcontentloaded' });
    const refCard = page.locator('mat-card[aria-label="Reference Observations and Blinded RR Adjudication"]');
    await expect(refCard).toBeVisible();

    await refCard.getByLabel('Observer ID').fill('Obs-1');
    await refCard.getByLabel('Respiration rate (br/min)').fill('16');
    await refCard.getByRole('button', { name: /Lock Observer RR/i }).click();

    await expect(refCard.getByText('Locked (blinded)')).toBeVisible();
    await expect(refCard.getByText(/1 \/ 2 locked/i)).toBeVisible();

    await refCard.getByLabel('Observer ID').fill('Obs-2');
    await refCard.getByLabel('Respiration rate (br/min)').fill('17');
    await refCard.getByRole('button', { name: /Lock Observer RR/i }).click();

    await expect(refCard.getByText(/2 \/ 2 locked · Unblinded/i)).toBeVisible();
    await refCard.getByRole('button', { name: /Adjudicate & Lock Final RR/i }).click();
    await expect(refCard.getByText(/Final locked RR:/i)).toBeVisible();

    const objCard = page.locator('mat-card[aria-label="Manuscript objective reports"]');
    await objCard.getByRole('button', { name: /Refresh objective reports/i }).click();
    await expect(objCard.getByRole('button', { name: /Export IEEE LaTeX/i }).first()).toBeVisible();

    const texDownload = page.waitForEvent('download');
    await objCard.getByRole('button', { name: /Export IEEE LaTeX/i }).first().click();
    expect((await texDownload).suggestedFilename()).toMatch(/\.tex$/);

    const csvDownload = page.waitForEvent('download');
    await objCard.getByRole('button', { name: /Export Condition CSV/i }).first().click();
    expect((await csvDownload).suggestedFilename()).toMatch(/\.csv$/);
  });
});
