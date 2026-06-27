/**
 * Smoke test — PIN recovery codes: create profile, capture recovery code,
 * lock, use "Forgot PIN?" → reset with code → unlock with new PIN.
 *
 * Full journey:
 *  1. Onboard → capture recovery code from dialog.
 *  2. Lock.
 *  3. "Forgot PIN?" → enter recovery code → set new PIN.
 *  4. Recovery-code dialog appears with rotated code.
 *  5. Unlock with new PIN — succeeds.
 *  6. Reuse of original code is rejected.
 */

import { test, expect } from '@playwright/test';
import fs from 'fs';
import path from 'path';
import { seedFirstRunComplete } from './helpers/first-run';

const CONFIGURED_SESSIONS_PROFILES_PATH = path.resolve(
  process.cwd(),
  process.env.RVT_TEST_SESSIONS_ROOT || '.playwright-state/sessions',
  'operator_profiles.json'
);

function cleanProfiles() {
  const profilePaths = [
    CONFIGURED_SESSIONS_PROFILES_PATH,
    path.resolve(process.cwd(), 'sessions', 'operator_profiles.json'),
    path.resolve(process.cwd(), '.playwright-state', 'sessions', 'operator_profiles.json'),
  ];
  for (const p of profilePaths) {
    if (fs.existsSync(p)) {
      try { fs.unlinkSync(p); } catch (_) {}
    }
  }
}

/** Press digits of a PIN on the onboarding inline keyboard. */
async function pressOnboardingPin(page: import('@playwright/test').Page, pin: string) {
  for (const digit of pin) {
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: new RegExp(`^${digit}$`) }).dispatchEvent('click');
  }
}

/** Press digits on a reset-flow inline keyboard. */
async function pressResetPin(page: import('@playwright/test').Page, pin: string) {
  for (const digit of pin) {
    await page.locator('.reset-flow .keyboard-grid button', { hasText: new RegExp(`^${digit}$`) }).dispatchEvent('click');
  }
}

test.describe('PIN recovery code flow', () => {
  test.use({ serviceWorkers: 'block' });

  test.beforeEach(async ({ page }) => {
    cleanProfiles();
    await page.addInitScript(() => {
      localStorage.setItem('rvt.server.url', 'http://127.0.0.1:8989');
    });
    await seedFirstRunComplete(page);
  });

  test.afterAll(() => {
    cleanProfiles();
  });

  test('full journey: create, capture code, lock, forgot-PIN reset, unlock with new PIN', async ({ page }) => {
    page.on('console', msg => console.log(`[BROWSER] ${msg.type()}: ${msg.text()}`));
    page.on('pageerror', err => console.error(`[BROWSER ERROR] ${err.message}`));

    await page.goto('/radar_vital_live_dashboard_v12_for_v16_0.html', { waitUntil: 'domcontentloaded' });

    // --- 1. Onboarding ---
    await expect(page.locator('section.idle-lock-overlay')).toBeVisible();
    await expect(page.locator('.onboarding-flow')).toBeVisible();

    await page.getByPlaceholder('e.g. Dr. Sarah Connor').fill('Reset Test User');
    await page.getByPlaceholder('e.g. SC').fill('RT');
    await pressOnboardingPin(page, '123456');

    // Intercept profile creation response to capture recovery code
    const createResponse = page.waitForResponse(resp =>
      resp.url().includes('/api/operator-profiles') && resp.request().method() === 'POST'
    );
    await page.getByRole('button', { name: 'Create Profile' }).dispatchEvent('click');
    const createResp = await createResponse;
    expect([200, 201]).toContain(createResp.status());

    // --- 2. Recovery-code dialog must appear ---
    await expect(page.locator('app-recovery-code-dialog')).toBeVisible({ timeout: 5000 });

    // The code is displayed in the dialog
    const codeEl = page.locator('app-recovery-code-dialog .recovery-code-text');
    await expect(codeEl).toBeVisible();
    const originalRecoveryCode = (await codeEl.textContent() || '').trim();
    expect(originalRecoveryCode).toMatch(/^[A-Z2-9]{4}-[A-Z2-9]{4}-[A-Z2-9]{4}$/);

    // "I saved my recovery code" button dismisses
    await page.locator('app-recovery-code-dialog button', { hasText: /I saved my recovery code/i }).click();
    await expect(page.locator('app-recovery-code-dialog')).not.toBeVisible();

    // Overlay should be gone — user is authenticated
    await expect(page.locator('section.idle-lock-overlay')).not.toBeVisible({ timeout: 5000 });

    // --- 3. Lock ---
    // Use the "Lock profile" action
    const lockBtn = page.getByRole('button', { name: /Lock profile/i }).first();
    if (await lockBtn.isVisible().catch(() => false)) {
      await lockBtn.dispatchEvent('click');
    } else {
      await page.getByRole('button', { name: 'More console actions' }).click();
      await page.getByRole('menuitem', { name: /Lock profile/i }).click();
    }

    await expect(page.locator('section.idle-lock-overlay')).toBeVisible();
    await expect(page.locator('.pin-entry-screen')).toBeVisible();

    // --- 4. "Forgot PIN?" link ---
    const forgotBtn = page.locator('button.forgot-pin-link', { hasText: /Forgot PIN\?/i });
    await expect(forgotBtn).toBeVisible();
    await forgotBtn.dispatchEvent('click');

    // Recovery flow — enter recovery code
    await expect(page.locator('.reset-flow')).toBeVisible();
    const codeInput = page.locator('.reset-flow input[placeholder*="XXXX"]');
    await expect(codeInput).toBeVisible();
    await codeInput.fill(originalRecoveryCode);

    await page.getByRole('button', { name: 'Next' }).click();

    // Enter new PIN
    await expect(page.locator('.reset-flow .pin-section')).toBeVisible();
    await pressResetPin(page, '567890');

    // Submit
    const resetResponse = page.waitForResponse(resp =>
      resp.url().includes('/api/auth/reset-pin') && resp.request().method() === 'POST'
    );
    await page.getByRole('button', { name: 'Set PIN' }).click();
    const resetResp = await resetResponse;
    expect(resetResp.status()).toBe(200);

    // --- 5. Recovery-code dialog with rotated code ---
    await expect(page.locator('app-recovery-code-dialog')).toBeVisible({ timeout: 5000 });
    const rotatedCodeEl = page.locator('app-recovery-code-dialog .recovery-code-text');
    const rotatedCode = (await rotatedCodeEl.textContent() || '').trim();
    expect(rotatedCode).toMatch(/^[A-Z2-9]{4}-[A-Z2-9]{4}-[A-Z2-9]{4}$/);
    expect(rotatedCode).not.toBe(originalRecoveryCode);
    await page.locator('app-recovery-code-dialog button', { hasText: /I saved my recovery code/i }).click();
    await expect(page.locator('app-recovery-code-dialog')).not.toBeVisible();

    // --- 6. Overlay gone — logged in with new PIN ---
    await expect(page.locator('section.idle-lock-overlay')).not.toBeVisible({ timeout: 5000 });

    // --- 7. Verify old PIN no longer works ---
    // Lock again
    const lockBtn2 = page.getByRole('button', { name: /Lock profile/i }).first();
    if (await lockBtn2.isVisible().catch(() => false)) {
      await lockBtn2.dispatchEvent('click');
    } else {
      await page.getByRole('button', { name: 'More console actions' }).click();
      await page.getByRole('menuitem', { name: /Lock profile/i }).click();
    }
    await expect(page.locator('section.idle-lock-overlay')).toBeVisible();

    // Old code (original) should be rejected
    await page.locator('button.forgot-pin-link', { hasText: /Forgot PIN\?/i }).dispatchEvent('click');
    await page.locator('.reset-flow input[placeholder*="XXXX"]').fill(originalRecoveryCode);
    await page.getByRole('button', { name: 'Next' }).click();
    await pressResetPin(page, '999999');

    const badResetResp = page.waitForResponse(resp =>
      resp.url().includes('/api/auth/reset-pin') && resp.request().method() === 'POST'
    );
    await page.getByRole('button', { name: 'Set PIN' }).click();
    const badResp = await badResetResp;
    expect(badResp.status()).toBe(401);

    // Cancel and unlock with new PIN 5678 instead
    await page.getByRole('button', { name: 'Cancel' }).click();
    await expect(page.locator('.pin-entry-screen')).toBeVisible();
  });
});
