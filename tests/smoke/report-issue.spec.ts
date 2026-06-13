import { test, expect, type APIRequestContext } from '@playwright/test';
import fs from 'fs';
import path from 'path';
import { seedFirstRunComplete, TERMS_VERSION } from './helpers/first-run';

const SETTINGS = '/settings';
const DIAGNOSTICS_KEY = 'rvt-diagnostics-optin';
const REPORT_PIN = '2468';
const REPORT_OPERATOR = {
  display_name: 'Report Issue User',
  initials: 'RI'
};
const CONFIGURED_SESSIONS_PROFILES_PATH = path.resolve(
  process.cwd(),
  process.env.RVT_TEST_SESSIONS_ROOT || '.playwright-state/sessions',
  'operator_profiles.json'
);
// Mirrors GITHUB_REPO_URL in web/src/app/services/app-meta.ts — Playwright's
// loader cannot named-import from the Angular sources (CommonJS interop).
const GITHUB_REPO_URL = 'https://github.com/blayalems/radar-vital-mmwave-60-davao';
const ISSUES_NEW_URL = `${GITHUB_REPO_URL}/issues/new`;

async function reportIssueCard(page: import('@playwright/test').Page) {
  const card = page.locator('app-report-issue-card');
  await expect(card).toContainText('Report an Issue', { timeout: 30000 });
  await card.evaluate(element => element.scrollIntoView({ block: 'center', inline: 'nearest' }));
  return card;
}

function cleanProfiles() {
  const profilePaths = [
    CONFIGURED_SESSIONS_PROFILES_PATH,
    path.resolve(process.cwd(), 'sessions', 'operator_profiles.json'),
    path.resolve(process.cwd(), '.playwright-state', 'sessions', 'operator_profiles.json')
  ];
  for (const profilePath of profilePaths) {
    if (fs.existsSync(profilePath)) {
      try { fs.unlinkSync(profilePath); } catch (_) {}
    }
  }
}

async function createAuthenticatedOperator(request: APIRequestContext) {
  const createResponse = await request.post('/api/operator-profiles', {
    data: {
      display_name: REPORT_OPERATOR.display_name,
      initials: REPORT_OPERATOR.initials,
      pin: REPORT_PIN
    }
  });
  expect([200, 201]).toContain(createResponse.status());
  const created = await createResponse.json();
  expect(created?.ok).toBe(true);
  expect(created?.operator?.operator_id).toBeTruthy();

  const loginResponse = await request.post('/api/auth/login', {
    data: {
      operator_id: created.operator.operator_id,
      pin: REPORT_PIN
    }
  });
  expect(loginResponse.status()).toBe(200);
  const login = await loginResponse.json();
  expect(login?.token).toBeTruthy();
  return {
    token: login.token as string,
    operator: login.operator ?? created.operator
  };
}

async function setDiagnosticsPreference(
  page: import('@playwright/test').Page,
  value: '0' | '1' | null,
  auth: { token: string; operator: { display_name?: string } }
) {
  const args = [DIAGNOSTICS_KEY, value, TERMS_VERSION, auth.token, auth.operator] as const;
  const seedStorage = ([key, next, termsVersion, token, operator]: typeof args) => {
    localStorage.setItem('rvt-consent-record', JSON.stringify({
      version: termsVersion,
      accepted_at: new Date().toISOString()
    }));
    localStorage.setItem('rvt-tutorial-done', '1');
    localStorage.setItem('rvt-demo-mode', '1');
    sessionStorage.setItem('rvt-operator-token', token);
    const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
    setup.operator_label = operator.display_name || 'Report Issue User';
    localStorage.setItem('rvt-setup', JSON.stringify(setup));
    if (next === null) {
      localStorage.removeItem(key);
    } else {
      localStorage.setItem(key, next);
    }
  };
  await page.evaluate(seedStorage, args);
  await page.reload({ waitUntil: 'domcontentloaded' });
}

test.describe('Report-issue card', () => {
  let authSession: Awaited<ReturnType<typeof createAuthenticatedOperator>>;

  test.use({ serviceWorkers: 'block' });

  test.beforeEach(async ({ page, request }) => {
    cleanProfiles();
    authSession = await createAuthenticatedOperator(request);
    await seedFirstRunComplete(page);
    await page.addInitScript(([token, operator]) => {
      // Put the app in demo mode so no live trainer is required.
      localStorage.setItem('rvt-demo-mode', '1');
      sessionStorage.setItem('rvt-operator-token', token);
      const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
      setup.operator_label = operator.display_name || 'Report Issue User';
      localStorage.setItem('rvt-setup', JSON.stringify(setup));
    }, [authSession.token, authSession.operator] as const);
    await page.goto(SETTINGS, { waitUntil: 'domcontentloaded' });
  });

  test.afterAll(() => {
    cleanProfiles();
  });

  // ---- card renders ---------------------------------------------------------

  test('report-issue card is present in Settings', async ({ page }) => {
    const card = await reportIssueCard(page);
    await expect(card.getByRole('button', { name: /preview what will be sent/i })).toBeVisible();
    await expect(card.getByRole('button', { name: /open a pre-filled github issue/i })).toBeVisible();
  });

  // ---- toggle persistence across reload ------------------------------------

  test('diagnostics toggle defaults to ON when key absent', async ({ page }) => {
    await setDiagnosticsPreference(page, null, authSession);
    const card = await reportIssueCard(page);

    const toggle = card.getByRole('switch', { name: /include diagnostics/i });
    await expect(toggle).toBeChecked();
  });

  test('toggling diagnostics off persists across reload', async ({ page }) => {
    const card = await reportIssueCard(page);

    const toggle = card.getByRole('switch', { name: /include diagnostics/i });
    await toggle.dispatchEvent('click');
    await expect(toggle).not.toBeChecked();

    // Verify localStorage was updated.
    const stored = await page.evaluate((key: string) => localStorage.getItem(key), DIAGNOSTICS_KEY);
    expect(stored).toBe('0');

    // Reload and verify toggle remains off.
    await page.reload({ waitUntil: 'domcontentloaded' });
    const reloadedCard = await reportIssueCard(page);
    await expect(reloadedCard.getByRole('switch', { name: /include diagnostics/i })).not.toBeChecked();
  });

  test('toggling diagnostics on persists across reload', async ({ page }) => {
    await setDiagnosticsPreference(page, '0', authSession);
    const card = await reportIssueCard(page);

    const toggle = card.getByRole('switch', { name: /include diagnostics/i });
    await expect(toggle).not.toBeChecked();
    await toggle.dispatchEvent('click');
    await expect(toggle).toBeChecked();

    const stored = await page.evaluate((key: string) => localStorage.getItem(key), DIAGNOSTICS_KEY);
    expect(stored).toBe('1');

    await page.reload({ waitUntil: 'domcontentloaded' });
    const reloadedCard = await reportIssueCard(page);
    await expect(reloadedCard.getByRole('switch', { name: /include diagnostics/i })).toBeChecked();
  });

  // ---- preview dialog -------------------------------------------------------

  test('preview dialog shows diagnostics when toggle is ON', async ({ page }) => {
    await setDiagnosticsPreference(page, '1', authSession);
    const card = await reportIssueCard(page);

    await card.getByRole('button', { name: /preview what will be sent/i }).dispatchEvent('click');

    const dialog = page.getByRole('dialog');
    await expect(dialog).toBeVisible();
    // The JSON preview should contain platform info.
    await expect(dialog).toContainText('platform');
    // When diagnostics on, connection_mode should be present.
    await expect(dialog).toContainText('connection_mode');
    // Should NOT show the opt-out notice.
    await expect(dialog.getByText(/diagnostics are.*off/i)).not.toBeVisible();
  });

  test('preview dialog hides diagnostics and shows opt-out notice when toggle is OFF', async ({ page }) => {
    await setDiagnosticsPreference(page, '0', authSession);
    const card = await reportIssueCard(page);

    await card.getByRole('button', { name: /preview what will be sent/i }).dispatchEvent('click');

    const dialog = page.getByRole('dialog');
    await expect(dialog).toBeVisible();
    // Only version + platform — no connection_mode.
    await expect(dialog).toContainText('product_version');
    await expect(dialog).toContainText('platform');
    await expect(dialog).not.toContainText('connection_mode');
    // Opt-out notice must be visible.
    await expect(dialog.getByText(/diagnostics are.*off/i)).toBeVisible();
  });

  test('preview dialog closes on Close button', async ({ page }) => {
    const card = await reportIssueCard(page);
    await card.getByRole('button', { name: /preview what will be sent/i }).dispatchEvent('click');
    const dialog = page.getByRole('dialog');
    await expect(dialog).toBeVisible();
    await dialog.getByRole('button', { name: /close/i }).dispatchEvent('click');
    await expect(dialog).not.toBeVisible();
  });

  // ---- GitHub link ----------------------------------------------------------

  test('Open GitHub issue link starts with issues/new URL and contains template=bug_report.yml', async ({ page }) => {
    await setDiagnosticsPreference(page, '0', authSession);
    const card = await reportIssueCard(page);

    await page.evaluate(() => {
      (window as typeof window & { __rvtOpenedIssueUrl?: string }).__rvtOpenedIssueUrl = '';
      window.open = (url?: string | URL) => {
        (window as typeof window & { __rvtOpenedIssueUrl?: string }).__rvtOpenedIssueUrl = String(url ?? '');
        return null;
      };
    });
    await card.getByRole('button', { name: /open a pre-filled github issue/i }).dispatchEvent('click');

    const url = await page.waitForFunction(() => {
      return (window as typeof window & { __rvtOpenedIssueUrl?: string }).__rvtOpenedIssueUrl || '';
    });
    const openedUrl = await url.jsonValue();
    expect(openedUrl.startsWith(ISSUES_NEW_URL)).toBe(true);
    expect(openedUrl).toContain('template=bug_report.yml');
  });
});
