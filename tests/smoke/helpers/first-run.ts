import type { Page } from '@playwright/test';

/**
 * Marks the first-run gates as already completed for a test page: consent
 * accepted at the current TERMS_VERSION and the onboarding tutorial done.
 *
 * Keep TERMS_VERSION in sync with web/src/app/services/app-meta.ts — the
 * repo-hygiene contract test asserts both strings match.
 */
export const TERMS_VERSION = '2026-06-12.1';

export async function seedFirstRunComplete(page: Page): Promise<void> {
  await page.addInitScript(version => {
    try {
      localStorage.setItem(
        'rvt-consent-record',
        JSON.stringify({ version, accepted_at: new Date().toISOString() })
      );
      localStorage.setItem('rvt-tutorial-done', '1');
    } catch (_) {
      /* storage unavailable in this context — consent gate will handle it */
    }
  }, TERMS_VERSION);
}
