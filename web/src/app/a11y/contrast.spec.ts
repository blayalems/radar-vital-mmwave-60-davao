import { describe, it, expect } from "vitest"
import { WCAG_AA_NORMAL, contrastRatio } from "./contrast"

/**
 * Regression guard for the low-emphasis text contrast fix (2026-06-17).
 *
 * `--rv-dim` is the shared token behind muted/secondary/placeholder/metadata/
 * timestamp/helper-text/table-caption/nav-label/chart-label styling. Each
 * theme+palette combination is paired here with its WORST-CASE (lightest in
 * light themes, darkest in dark themes) surface the dim text can sit on; if any
 * pairing dips below the 4.5:1 AA minimum this test fails.
 *
 * Values mirror web/src/styles/rvt-redesign-tokens.css. Night/HC inherit
 * full-strength on-surface tones and are not enumerated here.
 */
const DIM_ON_WORST_SURFACE: ReadonlyArray<{
  combo: string
  dim: string
  surface: string
}> = [
  { combo: "azure / light", dim: "#556578", surface: "#dce7f4" },
  { combo: "azure / dark", dim: "#9fb0c4", surface: "#2c3b4c" },
  { combo: "bloom / light", dim: "#5d566f", surface: "#e3d9f5" },
  { combo: "bloom / dark", dim: "#ada2c8", surface: "#362c54" },
  { combo: "mint / light", dim: "#4f6258", surface: "#d4e5db" },
  { combo: "mint / dark", dim: "#9bb0a3", surface: "#293a31" },
]

describe("low-emphasis text contrast (--rv-dim)", () => {
  for (const { combo, dim, surface } of DIM_ON_WORST_SURFACE) {
    it(`${combo}: dim ${dim} on ${surface} clears WCAG AA 4.5:1`, () => {
      expect(contrastRatio(dim, surface)).toBeGreaterThanOrEqual(WCAG_AA_NORMAL)
    })
  }

  it("black on white is the maximum ~21:1", () => {
    expect(contrastRatio("#000000", "#ffffff")).toBeCloseTo(21, 0)
  })
})
