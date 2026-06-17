import { describe, expect, it } from "vitest"
import {
  WCAG_AA_LARGE,
  WCAG_AA_NORMAL,
  contrastRatio,
  parseHex,
  relativeLuminance,
} from "./contrast"

describe("contrast helpers", () => {
  it("exposes the WCAG AA thresholds", () => {
    expect(WCAG_AA_NORMAL).toBe(4.5)
    expect(WCAG_AA_LARGE).toBe(3)
  })

  it("parses long, shorthand, and alpha hex colors", () => {
    expect(parseHex("#ffffff")).toEqual({ r: 255, g: 255, b: 255, a: 1 })
    expect(parseHex("#0af")).toEqual({ r: 0, g: 170, b: 255, a: 1 })
    expect(parseHex("#0af8")).toEqual({
      r: 0,
      g: 170,
      b: 255,
      a: 136 / 255,
    })
    expect(parseHex("#11223380")).toEqual({
      r: 17,
      g: 34,
      b: 51,
      a: 128 / 255,
    })
  })

  it("rejects invalid colors", () => {
    expect(() => parseHex("#fffff")).toThrow("Invalid hex color")
    expect(() => parseHex("red")).toThrow("Invalid hex color")
  })

  it("requires an opaque color for standalone luminance", () => {
    expect(() => relativeLuminance("#0008")).toThrow(
      "Relative luminance requires an opaque color",
    )
  })

  it("calculates the luminance endpoints", () => {
    expect(relativeLuminance("#000000")).toBe(0)
    expect(relativeLuminance("#ffffff")).toBeCloseTo(1, 8)
  })

  it("calculates black on white as approximately 21:1", () => {
    expect(contrastRatio("#000000", "#ffffff")).toBeCloseTo(21, 8)
  })

  it("composites alpha-channel foreground colors over the background", () => {
    const translucent = contrastRatio("#00000080", "#ffffff")
    expect(translucent).toBeGreaterThan(3.9)
    expect(translucent).toBeLessThan(4.1)
  })

  it("is symmetric for opaque foreground/background colors", () => {
    expect(contrastRatio("#556578", "#dce7f4")).toBeCloseTo(
      contrastRatio("#dce7f4", "#556578"),
      10,
    )
  })
})
