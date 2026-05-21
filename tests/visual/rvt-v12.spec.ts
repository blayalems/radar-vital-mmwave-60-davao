import { expect, test } from '@playwright/test';

const views = ['home', 'live', 'report', 'help', 'settings'];
const themes = ['light', 'dark', 'night', 'hc'];

test.describe('v12 dashboard visual baseline', () => {
  for (const theme of themes) {
    for (const view of views) {
      test(`${theme} ${view}`, async ({ page }) => {
        // Block external font loading to prevent screenshot hanging in offline/sandboxed environments
        await page.route('**/*.{woff,woff2}', route => route.abort());
        await page.route(/fonts\.(googleapis|gstatic)\.com/, route => route.abort());

        await page.goto('/live_dashboard.html', { waitUntil: 'domcontentloaded' });
        // Allow up to one post-install reload to complete.
        await page.waitForTimeout(1500);
        await page.waitForLoadState('domcontentloaded').catch(() => {});

        for (let attempt = 0; attempt < 3; attempt += 1) {
          try {
            await page.evaluate(({ theme, view }) => {
              document.documentElement.dataset.theme = theme;
              document.documentElement.dataset.density = theme === 'night' ? 'compact' : 'comfortable';
              if (typeof (window as any).switchView === 'function') (window as any).switchView(view, false);
            }, { theme, view });
            break;
          } catch (e) {
            if (!String(e).includes('Execution context was destroyed') || attempt === 2) throw e;
            await page.waitForLoadState('domcontentloaded').catch(() => {});
            await page.waitForTimeout(750);
          }
        }
        await expect(page).toHaveScreenshot(`v12-${theme}-${view}.png`, { fullPage: true, timeout: 30000 });
      });
    }
  }
});
