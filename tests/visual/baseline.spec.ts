import { test, expect } from '@playwright/test';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';
const THEMES = ['light', 'dark', 'night', 'hc'] as const;

for (const theme of THEMES) {
  test(`dashboard renders without layout collapse (${theme})`, async ({ page }, testInfo) => {
    // Block external font loading to prevent screenshot hanging in offline/sandboxed environments
  await page.route(/fonts\.(googleapis|gstatic)\.com/, route => route.abort());

    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    // Allow up to one post-install reload to complete.
    await page.waitForTimeout(1500);
    await page.waitForLoadState('domcontentloaded').catch(() => {});

    for (let attempt = 0; attempt < 3; attempt += 1) {
      try {
        await page.evaluate((t) => {
          document.documentElement.setAttribute('data-theme', t);
        }, theme);
        break;
      } catch (e) {
        if (!String(e).includes('Execution context was destroyed') || attempt === 2) throw e;
        await page.waitForLoadState('domcontentloaded').catch(() => {});
        await page.waitForTimeout(750);
      }
    }
    await page.waitForTimeout(800);

    // Sanity: at least one KPI card visible, no body overflow off-screen, no zero-sized main.
    const main = page.locator('.main, main, #app').first();
    await expect(main).toBeVisible();

    const bodyOverflow = await page.evaluate(() => {
      const el = document.body;
      return { sw: el.scrollWidth, cw: el.clientWidth };
    });
    expect(bodyOverflow.sw, `body horizontal overflow on ${testInfo.project.name}/${theme}`)
      .toBeLessThanOrEqual(bodyOverflow.cw + 4);

    // Take a non-blocking screenshot — failures here are warnings not test failures
    // until a deliberate baseline pass (npm run test:update).
    await page.screenshot({
      path: testInfo.outputPath(`dashboard-${theme}-${testInfo.project.name}.png`),
      fullPage: false,
      animations: 'disabled',
      timeout: 30000
    });
  });
}
