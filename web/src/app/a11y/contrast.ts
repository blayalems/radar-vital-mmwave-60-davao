/**
 * Typed facade for the shared WCAG implementation consumed by Node tooling and
 * the Angular/Vitest suite. Keep the formula in contrast-core.mjs only.
 */
export {
  WCAG_AA_LARGE,
  WCAG_AA_NORMAL,
  contrastRatio,
  parseHex,
  relativeLuminance,
} from "./contrast-core.mjs"
export type { RgbaColor } from "./contrast-core.mjs"
