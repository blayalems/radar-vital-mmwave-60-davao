import { expect, test } from '@playwright/test';

const views = ['home', 'live', 'report', 'help', 'settings'];
const themes = ['light', 'dark', 'night', 'hc'];

test.describe('v12 dashboard visual baseline', () => {
  for (const theme of themes) {
    for (const view of views) {
      test(`${theme} ${view}`, async ({ page }) => {
        await page.goto('/live_dashboard.html');
        await page.evaluate(({ theme, view }) => {
          document.documentElement.dataset.theme = theme;
          document.documentElement.dataset.density = theme === 'night' ? 'compact' : 'comfortable';
          if (typeof (window as any).switchView === 'function') (window as any).switchView(view, false);
        }, { theme, view });
        await expect(page).toHaveScreenshot(`v12-${theme}-${view}.png`, { fullPage: true });
      });
    }
  }
});
