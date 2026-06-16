/**
 * Minimal WCAG 2.1 contrast helpers (no external dependency).
 *
 * Used by contrast.spec.ts to lock the corrected low-emphasis text tokens at or
 * above the AA threshold, and reusable by any future automated colour checks.
 * Formulae follow https://www.w3.org/TR/WCAG21/#dfn-contrast-ratio and
 * https://www.w3.org/TR/WCAG21/#dfn-relative-luminance.
 */

/** WCAG AA minimum contrast ratio for normal-size body text. */
export const WCAG_AA_NORMAL = 4.5

/** WCAG AA minimum contrast ratio for large text (>=18.66px bold / 24px). */
export const WCAG_AA_LARGE = 3

/** Parse a #rgb or #rrggbb hex string into 0-255 channel values. */
export function parseHex(hex: string): { r: number; g: number; b: number } {
  const value = hex.trim().replace(/^#/, "")
  const full =
    value.length === 3
      ? value
          .split("")
          .map((c) => c + c)
          .join("")
      : value
  if (full.length !== 6 || /[^0-9a-fA-F]/.test(full)) {
    throw new Error(`Invalid hex color: ${hex}`)
  }
  return {
    r: parseInt(full.slice(0, 2), 16),
    g: parseInt(full.slice(2, 4), 16),
    b: parseInt(full.slice(4, 6), 16),
  }
}

/** Relative luminance of an sRGB color in the range [0, 1]. */
export function relativeLuminance(hex: string): number {
  const { r, g, b } = parseHex(hex)
  const channel = (raw: number): number => {
    const c = raw / 255
    return c <= 0.03928 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4)
  }
  return 0.2126 * channel(r) + 0.7152 * channel(g) + 0.0722 * channel(b)
}

/** Contrast ratio between two colors, in the range [1, 21]. */
export function contrastRatio(foreground: string, background: string): number {
  const l1 = relativeLuminance(foreground)
  const l2 = relativeLuminance(background)
  const lighter = Math.max(l1, l2)
  const darker = Math.min(l1, l2)
  return (lighter + 0.05) / (darker + 0.05)
}
