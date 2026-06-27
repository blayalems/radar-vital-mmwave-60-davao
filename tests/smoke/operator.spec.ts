import { test, expect } from '@playwright/test';
import fs from 'fs';
import path from 'path';
import { seedFirstRunComplete } from './helpers/first-run';

const PROFILES_PATH = path.resolve(process.cwd(), 'operator_profiles.json');
const SESSION_PARENT_PROFILES_PATH = path.resolve(process.cwd(), '..', 'operator_profiles.json');
const PLAYWRIGHT_PROFILES_PATH = path.resolve(process.cwd(), '.playwright-state', 'operator_profiles.json');
const SESSIONS_PROFILES_PATH = path.resolve(process.cwd(), 'sessions', 'operator_profiles.json');
const PLAYWRIGHT_SESSIONS_PROFILES_PATH = path.resolve(process.cwd(), '.playwright-state', 'sessions', 'operator_profiles.json');
const CONFIGURED_SESSIONS_PROFILES_PATH = path.resolve(
  process.cwd(),
  process.env.RVT_TEST_SESSIONS_ROOT || '.playwright-state/sessions',
  'operator_profiles.json'
);

function cleanProfiles() {
  const profilePaths = new Set([
    PROFILES_PATH,
    SESSION_PARENT_PROFILES_PATH,
    PLAYWRIGHT_PROFILES_PATH,
    SESSIONS_PROFILES_PATH,
    PLAYWRIGHT_SESSIONS_PROFILES_PATH,
    CONFIGURED_SESSIONS_PROFILES_PATH
  ]);
  for (const profilePath of profilePaths) {
    if (!fs.existsSync(profilePath)) {
      continue;
    }
    try {
      fs.unlinkSync(profilePath);
    } catch (_) {}
  }
}

async function pressPin(page: import('@playwright/test').Page, pin: string) {
  const keypad = page.locator('app-pin-keyboard:visible').last();
  await expect(keypad).toBeVisible();
  for (const digit of pin) {
    const key = keypad.locator('.keyboard-grid button', { hasText: new RegExp(`^${digit}$`) }).first();
    await expect(key).toBeVisible();
    await key.click();
  }
}

async function submitPinAndWait(page: import('@playwright/test').Page, pin: string) {
  const loginResponse = page.waitForResponse(response =>
    response.url().endsWith('/api/auth/login') && response.request().method() === 'POST'
  );
  await pressPin(page, pin);
  return loginResponse;
}

async function clearPin(page: import('@playwright/test').Page) {
  const keypad = page.locator('app-pin-keyboard:visible').last();
  await expect(keypad).toBeVisible();
  await keypad.locator('.keyboard-grid button', { hasText: /^C$/ }).first().click();
}

async function clickConsoleAction(page: import('@playwright/test').Page, name: string) {
  const railAction = page.locator('.operator-actions button', { hasText: name }).first();
  if (await railAction.isVisible().catch(() => false)) {
    await railAction.dispatchEvent('click');
    return;
  }
  const directAction = page.getByRole('button', { name }).filter({ hasText: name }).first();
  if (await directAction.isVisible().catch(() => false)) {
    await directAction.dispatchEvent('click');
    return;
  }
  const moreActions = page
    .locator('button.tb-more[aria-label="More console actions"], button[aria-label="More console actions"]')
    .first();
  await expect(moreActions).toBeVisible();
  await moreActions.click({ force: true });
  const menuItem = page.getByRole('menuitem', { name });
  await expect(menuItem).toBeVisible();
  await menuItem.click();
}

test.describe('Operator profile and lock system', () => {
  test.use({ serviceWorkers: 'block' });

  test.beforeEach(async ({ page }) => {
    cleanProfiles();
    // Seed a configured trainer URL so PR-69's firstRunGuard does not bounce to
    // /connect, and seed first-run consent so #54's RA 10173 gate does not block.
    await page.addInitScript(() => {
      localStorage.setItem('rvt.server.url', 'http://127.0.0.1:8989');
    });
    await seedFirstRunComplete(page);
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

    // Click PIN: 1, 2, 3, 4, 5, 6
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^1$/ }).dispatchEvent('click');
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^2$/ }).dispatchEvent('click');
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^3$/ }).dispatchEvent('click');
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^4$/ }).dispatchEvent('click');
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^5$/ }).dispatchEvent('click');
    await page.locator('.onboarding-flow .keyboard-grid button', { hasText: /^6$/ }).dispatchEvent('click');

    // Create profile
    await page.getByRole('button', { name: 'Create Profile' }).dispatchEvent('click');

    // Verify lock screen is gone and operator is set
    await expect(page.locator('section.idle-lock-overlay')).not.toBeVisible();
    await expect(page.locator('.operator-badge .status-txt')).toHaveText('Dr. Sarah Connor');
    await expect(page.locator('app-recovery-code-dialog')).toBeVisible();
    await page.locator('app-recovery-code-dialog button', { hasText: /I saved my recovery code/i }).click();
    await expect(page.locator('app-recovery-code-dialog')).not.toBeVisible();

    // 2. Add a second operator from the authenticated switcher and switch to it.
    await clickConsoleAction(page, 'Switch operator');
    await expect(page.locator('app-switch-operator-dialog')).toBeVisible();
    await page.getByRole('button', { name: 'Add operator' }).dispatchEvent('click');
    await page.getByLabel('Display name').fill('John Connor');
    await page.getByLabel('Initials').fill('JC');
    await page.getByLabel('6-digit PIN').fill('567890');
    await page.getByRole('button', { name: 'Create and switch' }).dispatchEvent('click');
    await expect(page.locator('app-switch-operator-dialog')).not.toBeVisible();
    await expect(page.locator('.operator-badge .status-txt')).toHaveText('John Connor');

    // 3. Lock profile
    await clickConsoleAction(page, 'Lock profile');

    // Lock screen overlay and placeholder should be visible
    await expect(page.locator('section.idle-lock-overlay')).toBeVisible();
    await expect(page.locator('.locked-placeholder')).toBeVisible();
    await expect(page.locator('.pin-entry-screen')).toBeVisible();

    // 4. Unlock with John's PIN.
    await submitPinAndWait(page, '567890');
    await expect(page.locator('section.idle-lock-overlay')).not.toBeVisible();

    // 5. Switch back to Sarah while unlocked.
    await clickConsoleAction(page, 'Switch operator');
    await expect(page.locator('app-switch-operator-dialog')).toBeVisible();
    await page.locator('app-switch-operator-dialog .profile-row', { hasText: 'Dr. Sarah Connor' }).dispatchEvent('click');
    await submitPinAndWait(page, '123456');
    await expect(page.locator('app-switch-operator-dialog')).not.toBeVisible();
    await expect(page.locator('.operator-badge .status-txt')).toHaveText('Dr. Sarah Connor');

    // 6. Lock again and test invalid PIN + lockout warning.
    await clickConsoleAction(page, 'Lock profile');
    await expect(page.locator('section.idle-lock-overlay')).toBeVisible();
    await expect(page.locator('.pin-entry-screen')).toBeVisible();
    await clearPin(page);

    // Enter wrong PIN 5 times to trigger lockout
    // 1st wrong attempt
    await expect((await submitPinAndWait(page, '111111')).status()).toBe(401);
    await expect(page.locator('.error-text')).toContainText('Invalid operator ID or PIN');

    // 2nd wrong attempt
    await expect((await submitPinAndWait(page, '111111')).status()).toBe(401);
    await expect(page.locator('.error-text')).toContainText('Invalid operator ID or PIN');

    // 3rd wrong attempt
    await expect((await submitPinAndWait(page, '111111')).status()).toBe(401);
    await expect(page.locator('.error-text')).toContainText('Invalid operator ID or PIN');

    // 4th wrong attempt
    await expect((await submitPinAndWait(page, '111111')).status()).toBe(401);
    await expect(page.locator('.error-text')).toContainText('Invalid operator ID or PIN');

    // 5th wrong attempt -> lockout
    await expect((await submitPinAndWait(page, '111111')).status()).toBe(429);
    await expect(page.locator('.error-text')).toContainText('Too many failed attempts');
  });
});
