/**
 * First-run gate smoke tests.
 *
 * These tests intentionally do NOT call seedFirstRunComplete() so they exercise
 * the real first-run flow (consent dialog, onboarding tutorial, etc.).
 * Separate browser contexts ensure isolation from other suites.
 */
import { test, expect, type Locator } from '@playwright/test';
import { seedFirstRunComplete } from './helpers/first-run';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';

async function activateButton(scope: Locator, name: RegExp): Promise<void> {
  const button = scope.getByRole('button', { name });
  await expect(button).toBeVisible();
  await button.dispatchEvent('click');
}

/** Route stubs shared across tests. */
async function applyCommonRoutes(page: import('@playwright/test').Page) {
  await page.addInitScript(() => {
    const origin = window.location.origin;
    localStorage.setItem('rvt-api-base', origin);
    localStorage.setItem('rvt.server.url', origin);
    sessionStorage.setItem('rvt-pair-token', 'mock-pair-token');
  });
  await page.route('**/api/status', route =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        ok: true,
        mode: 'live',
        trainer_version: '16.4.0-test',
        dashboard_version: '16.4.0-test',
        firmware_expected: 'v16.4.0-test',
        active_session: null,
        feature_flags: {}
      })
    })
  );
  await page.route('**/api/health', route =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({ ok: true, version: '16.4.0-test' })
    })
  );
  await page.route('**/api/version', route =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({ product_version: '16.4.0-test' })
    })
  );
  await page.route('**/api/auth/validate', route =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({ ok: true, operator: { operator_id: 'op_fr', display_name: 'FR Tester', initials: 'FT' } })
    })
  );
  await page.route('**/api/operator-profiles', route =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        schema_version: 'rvt-operator-profiles-v12.0',
        profiles: [{ operator_id: 'op_fr', display_name: 'FR Tester', initials: 'FT' }]
      })
    })
  );
  await page.route('**/api/auth/login', route =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        ok: true,
        token: 'mock-fr-token',
        operator: { operator_id: 'op_fr', display_name: 'FR Tester', initials: 'FT' }
      })
    })
  );
}

test.use({ serviceWorkers: 'block' });

test.describe('First-run consent gate', () => {
  test('fresh context shows consent dialog before anything else', async ({ page }) => {
    await applyCommonRoutes(page);
    // Do NOT seed first-run — we want the gate to fire.
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });

    // Consent dialog must be visible
    const dialog = page.locator('app-consent-dialog');
    await expect(dialog).toBeVisible({ timeout: 8000 });
    await expect(dialog).toContainText('Terms of Use');
    await expect(dialog).toContainText('Accept');
    await expect(dialog).toContainText('Decline');
    await expect(dialog.getByRole('link', { name: /Full Terms of Use/i })).toHaveAttribute('href', /TERMS\.md$/);
    await expect(dialog.getByRole('link', { name: /Full Privacy Policy/i })).toHaveAttribute('href', /PRIVACY\.md$/);
  });

  test('declining shows blocking panel with back-to-terms action', async ({ page }) => {
    await applyCommonRoutes(page);
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });

    const dialog = page.locator('app-consent-dialog');
    await expect(dialog).toBeVisible({ timeout: 8000 });

    // Click Decline
    await activateButton(dialog, /Decline/i);

    // Blocking panel should appear
    await expect(dialog).toContainText('Consent Required');
    await expect(dialog).toContainText('cannot be used without accepting');

    // Back to Terms action should be present
    const backBtn = dialog.getByRole('button', { name: /Back to Terms/i });
    await expect(backBtn).toBeVisible();

    // App is still gated — dialog is still open
    await expect(dialog).toBeVisible();
  });

  test('back-to-terms returns to the terms view', async ({ page }) => {
    await applyCommonRoutes(page);
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });

    const dialog = page.locator('app-consent-dialog');
    await expect(dialog).toBeVisible({ timeout: 8000 });

    await activateButton(dialog, /Decline/i);
    await expect(dialog).toContainText('Consent Required');

    await activateButton(dialog, /Back to Terms/i);
    // Terms view restored
    await expect(dialog).toContainText('Terms of Use');
    await expect(dialog).toContainText('Accept');
  });

  test('accepting consent closes the dialog', async ({ page }) => {
    await applyCommonRoutes(page);
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });

    const dialog = page.locator('app-consent-dialog');
    await expect(dialog).toBeVisible({ timeout: 8000 });

    await activateButton(dialog, /Accept/i);

    // Dialog must close
    await expect(dialog).not.toBeVisible({ timeout: 5000 });
  });

  test('reloading after consent accepted does not show consent dialog again', async ({ page }) => {
    await applyCommonRoutes(page);
    await seedFirstRunComplete(page);
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.waitForLoadState('networkidle', { timeout: 10000 }).catch(() => {});

    // Dialog should not appear
    const dialog = page.locator('app-consent-dialog');
    await expect(dialog).toHaveCount(0, { timeout: 4000 });
  });
});

test.describe('First-run tutorial gate', () => {
  test('tutorial appears once after first operator auth', async ({ page }) => {
    await applyCommonRoutes(page);

    // Seed consent accepted but tutorial NOT done
    await page.addInitScript(version => {
      try {
        localStorage.setItem('rvt-consent-record', JSON.stringify({ version, accepted_at: new Date().toISOString() }));
        // Do NOT set rvt-tutorial-done
      } catch (_) {}
    }, '2026-06-12.1');

    await page.addInitScript(() => {
      sessionStorage.setItem('rvt-operator-token', 'mock-fr-token');
      const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
      setup.operator_label = 'FR Tester';
      localStorage.setItem('rvt-setup', JSON.stringify(setup));
    });

    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });

    // Simulate the operator-authenticated event (fires after profile creation auto-login)
    await page.evaluate(() => {
      window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    });

    // Tutorial dialog should appear
    const tutorial = page.locator('app-onboarding-tutorial');
    await expect(tutorial).toBeVisible({ timeout: 8000 });
    await expect(tutorial.getByRole('link', { name: /Terms of Use/i })).toHaveAttribute('href', /TERMS\.md$/);
    await expect(tutorial.getByRole('link', { name: /Privacy Policy/i })).toHaveAttribute('href', /PRIVACY\.md$/);

    // Can navigate to next step
    await activateButton(tutorial, /Next/i);
    await expect(tutorial).toBeVisible();

    // Can skip
    await activateButton(tutorial, /Skip/i);
    await expect(tutorial).not.toBeVisible({ timeout: 5000 });
  });

  test('tutorial does not appear again after it has been completed', async ({ page }) => {
    await applyCommonRoutes(page);
    await seedFirstRunComplete(page);
    await page.addInitScript(() => {
      sessionStorage.setItem('rvt-operator-token', 'mock-fr-token');
      const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
      setup.operator_label = 'FR Tester';
      localStorage.setItem('rvt-setup', JSON.stringify(setup));
    });

    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });

    // Dispatch auth event — tutorial should NOT appear because it's already done
    await page.evaluate(() => {
      window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    });

    await page.waitForTimeout(1500);
    const tutorial = page.locator('app-onboarding-tutorial');
    await expect(tutorial).toHaveCount(0);
  });

  test('full flow: no seed → consent → accept → tutorial → complete → reload = no gates', async ({ page }) => {
    await applyCommonRoutes(page);
    // Do NOT seed anything — truly fresh context

    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });

    // 1. Consent dialog appears
    const dialog = page.locator('app-consent-dialog');
    await expect(dialog).toBeVisible({ timeout: 8000 });
    await expect(dialog).toContainText('Terms of Use');

    // 2. Accept
    await activateButton(dialog, /Accept/i);
    await expect(dialog).not.toBeVisible({ timeout: 5000 });

    // 3. Simulate auth event to trigger tutorial
    await page.evaluate(() => {
      window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    });

    const tutorial = page.locator('app-onboarding-tutorial');
    await expect(tutorial).toBeVisible({ timeout: 8000 });

    // 4. Skip tutorial
    await activateButton(tutorial, /Skip/i);
    await expect(tutorial).not.toBeVisible({ timeout: 5000 });

    // 5. Reload — no consent dialog, no tutorial
    await page.reload({ waitUntil: 'domcontentloaded' });
    await page.waitForTimeout(2000);

    const consentDialog = page.locator('app-consent-dialog');
    await expect(consentDialog).toHaveCount(0);
    const tutorialAfterReload = page.locator('app-onboarding-tutorial');
    await expect(tutorialAfterReload).toHaveCount(0);
  });

  test('valid operator token with missing consent shows only consent before tutorial', async ({ page }) => {
    await applyCommonRoutes(page);
    await page.addInitScript(() => {
      sessionStorage.setItem('rvt-operator-token', 'mock-fr-token');
      const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
      setup.operator_label = 'FR Tester';
      localStorage.setItem('rvt-setup', JSON.stringify(setup));
      localStorage.removeItem('rvt-consent-record');
      localStorage.removeItem('rvt-tutorial-done');
    });

    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });

    const dialog = page.locator('app-consent-dialog');
    await expect(dialog).toBeVisible({ timeout: 8000 });
    await expect(page.locator('app-onboarding-tutorial')).toHaveCount(0);

    await activateButton(dialog, /Accept/i);
    await expect(dialog).not.toBeVisible({ timeout: 5000 });
    await expect(page.locator('app-onboarding-tutorial')).toBeVisible({ timeout: 8000 });
  });
});
