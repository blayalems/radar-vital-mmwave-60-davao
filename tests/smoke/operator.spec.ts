import { test, expect } from '@playwright/test';
import fs from 'fs';
import path from 'path';

const PROFILES_PATH = path.resolve(process.cwd(), 'operator_profiles.json');
const SESSION_PARENT_PROFILES_PATH = path.resolve(process.cwd(), '..', 'operator_profiles.json');
const PLAYWRIGHT_PROFILES_PATH = path.resolve(process.cwd(), '.playwright-state', 'operator_profiles.json');

function cleanProfiles() {
  for (const profilePath of [PROFILES_PATH, SESSION_PARENT_PROFILES_PATH, PLAYWRIGHT_PROFILES_PATH]) {
    if (!fs.existsSync(profilePath)) {
      continue;
    }
    try {
      fs.unlinkSync(profilePath);
    } catch (_) {}
  }
}

async function pressPin(page: import('@playwright/test').Page, pin: string) {
  const keypad = page.locator('app-pin-keyboard').last();
  for (const digit of pin) {
    await keypad.locator('.keyboard-grid button', { hasText: new RegExp(`^${digit}$`) }).click();
  }
}

async function submitPinAndWait(page: import('@playwright/test').Page, pin: string) {
  const loginResponse = page.waitForResponse(response =>
    response.url().endsWith('/api/auth/login') && response.request().method() === 'POST'
  );
  await pressPin(page, pin);
  return loginResponse;
}

async function clickConsoleAction(page: import('@playwright/test').Page, name: string) {
  const directAction = page.getByRole('button', { name }).first();
  if (await directAction.isVisible().catch(() => false)) {
    await directAction.click();
    return;
  }
  await page.getByRole('button', { name: 'More console actions' }).click();
  await page.getByRole('menuitem', { name }).click();
}

test.describe('Operator profile and lock system', () => {
  test.beforeEach(() => {
    cleanProfiles();
  });

  test.afterAll(() => {
    cleanProfiles();
  });

  test('onboards, locks, unlocks, and switches operators', async ({ page }) => {
    // 1. Load the page - should show onboarding since profiles are empty
    page.on('console', msg => {
      console.log(`[BROWSER CONSOLE] ${msg.type()}: ${msg.text()}`);
    });
    page.on('pageerror', err => {
      console.error(`[BROWSER ERROR] ${err.message}`);
    });

    await page.goto('/radar_vital_live_dashboard_v12_for_v16_0.html', { waitUntil: 'domcontentloaded' });
    
    // Wait for overlay and onboarding flow
    await expect(page.locator('section.idle-lock-overlay')).toBeVisible();
    await expect(page.locator('.onboarding-flow')).toBeVisible();

    // Fill onboarding form
    await page.getByPlaceholder('e.g. Dr. Sarah Connor').fill('Dr. Sarah Connor');
    await page.getByPlaceholder('e.g. SC').fill('SC');

    // Click PIN: 1, 2, 3, 4
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^1$/ }).click();
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^2$/ }).click();
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^3$/ }).click();
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^4$/ }).click();

    // Create profile
    await page.getByRole('button', { name: 'Create Profile' }).click();

    // Verify lock screen is gone and operator is set
    await expect(page.locator('section.idle-lock-overlay')).not.toBeVisible();
    await expect(page.locator('.operator-badge .status-txt')).toHaveText('Dr. Sarah Connor');

    // 2. Add a second operator from the authenticated switcher and switch to it.
    await clickConsoleAction(page, 'Switch operator');
    await expect(page.locator('app-switch-operator-dialog')).toBeVisible();
    await page.getByRole('button', { name: 'Add operator' }).click();
    await page.getByLabel('Display name').fill('John Connor');
    await page.getByLabel('Initials').fill('JC');
    await page.getByLabel('4-digit PIN').fill('5678');
    await page.getByRole('button', { name: 'Create and switch' }).click();
    await expect(page.locator('app-switch-operator-dialog')).not.toBeVisible();
    await expect(page.locator('.operator-badge .status-txt')).toHaveText('John Connor');

    // 3. Lock profile
    await clickConsoleAction(page, 'Lock profile');

    // Lock screen overlay and placeholder should be visible
    await expect(page.locator('section.idle-lock-overlay')).toBeVisible();
    await expect(page.locator('.locked-placeholder')).toBeVisible();
    await expect(page.locator('.pin-entry-screen')).toBeVisible();

    // 4. Unlock with John's PIN.
    await submitPinAndWait(page, '5678');
    await expect(page.locator('section.idle-lock-overlay')).not.toBeVisible();

    // 5. Switch back to Sarah while unlocked.
    await clickConsoleAction(page, 'Switch operator');
    await expect(page.locator('app-switch-operator-dialog')).toBeVisible();
    await page.locator('app-switch-operator-dialog .profile-row', { hasText: 'Dr. Sarah Connor' }).click();
    await submitPinAndWait(page, '1234');
    await expect(page.locator('app-switch-operator-dialog')).not.toBeVisible();
    await expect(page.locator('.operator-badge .status-txt')).toHaveText('Dr. Sarah Connor');

    // 6. Lock again and test invalid PIN + lockout warning.
    await clickConsoleAction(page, 'Lock profile');
    await expect(page.locator('section.idle-lock-overlay')).toBeVisible();

    // Enter wrong PIN 5 times to trigger lockout
    // 1st wrong attempt
    await expect((await submitPinAndWait(page, '1111')).status()).toBe(401);
    await expect(page.locator('.error-text')).toContainText('Invalid operator ID or PIN');

    // 2nd wrong attempt
    await expect((await submitPinAndWait(page, '1111')).status()).toBe(401);
    await expect(page.locator('.error-text')).toContainText('Invalid operator ID or PIN');

    // 3rd wrong attempt
    await expect((await submitPinAndWait(page, '1111')).status()).toBe(401);
    await expect(page.locator('.error-text')).toContainText('Invalid operator ID or PIN');

    // 4th wrong attempt
    await expect((await submitPinAndWait(page, '1111')).status()).toBe(401);
    await expect(page.locator('.error-text')).toContainText('Invalid operator ID or PIN');

    // 5th wrong attempt -> lockout
    await expect((await submitPinAndWait(page, '1111')).status()).toBe(429);
    await expect(page.locator('.error-text')).toContainText('Too many failed attempts');
  });
});
