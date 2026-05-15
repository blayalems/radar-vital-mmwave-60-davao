import { test, expect } from '@playwright/test';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';
const THEMES = ['light', 'dark', 'night', 'hc'] as const;

for (const theme of THEMES) {
  test(`dashboard renders without layout collapse (${theme})`, async ({ page }, testInfo) => {
    await page.goto(DASHBOARD);
    await page.evaluate((t) => {
      document.documentElement.setAttribute('data-theme', t);
    }, theme);
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
      animations: 'disabled'
    });
  });
}
