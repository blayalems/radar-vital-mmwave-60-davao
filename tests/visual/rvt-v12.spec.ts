import { expect, test } from '@playwright/test';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';
const views = ['home', 'live', 'report', 'help', 'settings'];
const themes = ['light', 'dark', 'night', 'hc'];

test.describe('v12 dashboard visual baseline', () => {
  for (const theme of themes) {
    for (const view of views) {
      test(`${theme} ${view}`, async ({ page }) => {
        // Block external font loading to prevent screenshot hanging in offline/sandboxed environments
        await page.route(/fonts\.(googleapis|gstatic)\.com/, route => route.abort());
        if (view === 'home') {
          // The radar scope is a canvas animation driven by wall-clock time.
          await page.addInitScript(() => {
            Date.now = () => 1_700_000_000_000;
          });
        }

        await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
        await page.evaluate(({ theme }) => {
          localStorage.setItem('rvt-theme', theme);
          localStorage.setItem('rvt-density', theme === 'night' ? 'compact' : 'comfortable');
          localStorage.removeItem('rvt-demo-mode');
        }, { theme });
        await page.goto(`/${view}`, { waitUntil: 'domcontentloaded' });
        await expect(page.locator('html')).toHaveAttribute('data-theme', theme);
        await expect(page.locator('app-layout')).toBeVisible();
        const snapshotName = theme === 'hc' && view === 'live'
          ? 'v12-hc-live-dark-inverse-controls.png'
          : `v12-${theme}-${view}.png`;
        await expect(page).toHaveScreenshot(snapshotName, { fullPage: true, timeout: 30000 });
      });
    }
  }
});
