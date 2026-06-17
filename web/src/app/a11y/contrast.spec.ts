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

  it("parses long and shorthand hex colors", () => {
    expect(parseHex("#ffffff")).toEqual({ r: 255, g: 255, b: 255 })
    expect(parseHex("#0af")).toEqual({ r: 0, g: 170, b: 255 })
  })

  it("rejects invalid colors", () => {
    expect(() => parseHex("#ffff")).toThrow("Invalid hex color")
    expect(() => parseHex("red")).toThrow("Invalid hex color")
  })

  it("calculates the luminance endpoints", () => {
    expect(relativeLuminance("#000000")).toBe(0)
    expect(relativeLuminance("#ffffff")).toBeCloseTo(1, 8)
  })

  it("calculates black on white as approximately 21:1", () => {
    expect(contrastRatio("#000000", "#ffffff")).toBeCloseTo(21, 8)
  })

  it("is symmetric regardless of foreground/background order", () => {
    expect(contrastRatio("#556578", "#dce7f4")).toBeCloseTo(
      contrastRatio("#dce7f4", "#556578"),
      10,
    )
  })
})
