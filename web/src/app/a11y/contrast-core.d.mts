export interface RgbaColor {
  r: number
  g: number
  b: number
  a: number
}

export const WCAG_AA_NORMAL: 4.5
export const WCAG_AA_LARGE: 3
export function parseHex(hex: string): RgbaColor
export function relativeLuminance(hex: string): number
export function contrastRatio(
  foreground: string,
  background: string,
  canvas?: string,
): number
