import type { Page } from '@playwright/test';

/**
 * Marks the first-run gates as already completed for a test page: consent
 * accepted at the current TERMS_VERSION and the onboarding tutorial done.
 *
 * Keep TERMS_VERSION in sync with web/src/app/services/app-meta.ts — the
 * repo-hygiene contract test asserts both strings match.
 */
export const TERMS_VERSION = '2026-06-12.1';

// Fixed acceptance time so the Settings "Accepted" label (rendered via
// toLocaleString) is byte-stable across runs. A live `new Date()` here made
// the settings visual baseline non-deterministic — the capture and verify
// runs printed different clock times, so the snapshot never reproduced.
export const CONSENT_ACCEPTED_AT = '2026-06-12T08:00:00.000Z';

export async function seedFirstRunComplete(page: Page): Promise<void> {
  await page.addInitScript(({ version, acceptedAt }) => {
    try {
      localStorage.setItem(
        'rvt-consent-record',
        JSON.stringify({ version, accepted_at: acceptedAt })
      );
      localStorage.setItem('rvt-tutorial-done', '1');
    } catch (_) {
      /* storage unavailable in this context — consent gate will handle it */
    }
  }, { version: TERMS_VERSION, acceptedAt: CONSENT_ACCEPTED_AT });
}
