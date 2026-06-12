/**
 * Product identity, authorship, and legal metadata — single source of truth
 * for the About card, trainer /about footer, consent dialog, and issue links.
 */
export const PRODUCT_NAME = 'Radar Vital';
export const AUTHORS = ['Lemuel Blaya', 'Angelo Diaz', 'Blessie Mugat'] as const;
export const PROGRAM = 'BS Electronics Engineering';
export const UNIVERSITY = 'University of Mindanao';
export const GITHUB_REPO_URL = 'https://github.com/blayalems/radar-vital-mmwave-60-davao';

/**
 * Version of the Terms & Privacy text the operator must have accepted.
 * Bumping this re-prompts every installation for consent (legal-review or
 * policy changes). Date-stamped, monotonically comparable as a string.
 */
export const TERMS_VERSION = '2026-06-12.1';

/** © line with an auto-updating year so the copyright never goes stale. */
export function copyrightLine(): string {
  return `© ${new Date().getFullYear()} ${AUTHORS.join(', ')} — ${PROGRAM}, ${UNIVERSITY}`;
}
