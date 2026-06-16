import { readFileSync } from "node:fs"
import { resolve } from "node:path"
import { describe, expect, it } from "vitest"
import { WCAG_AA_NORMAL, contrastRatio } from "./contrast"

const css = readFileSync(
  resolve(process.cwd(), "src/styles/rvt-redesign-tokens.css"),
  "utf8",
)

function escapeRegExp(value: string): string {
  return value.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")
}

function declaration(selector: string, token: string): string {
  const block = css.match(
    new RegExp(`${escapeRegExp(selector)}\\s*\\{([\\s\\S]*?)\\}`),
  )?.[1]

  if (!block) {
    throw new Error(`Theme selector not found: ${selector}`)
  }

  const value = block.match(
    new RegExp(`${escapeRegExp(token)}\\s*:\\s*([^;]+);`),
  )?.[1]?.trim()

  if (!value) {
    throw new Error(`Token ${token} not found in ${selector}`)
  }

  return value
}

const DIM_CASES = [
  {
    name: "azure / light",
    selector: 'html[data-theme="light"][data-palette="azure"]',
  },
  {
    name: "azure / dark",
    selector: 'html[data-theme="dark"][data-palette="azure"]',
  },
  {
    name: "bloom / light",
    selector: 'html[data-theme="light"][data-palette="bloom"]',
  },
  {
    name: "bloom / dark",
    selector: 'html[data-theme="dark"][data-palette="bloom"]',
  },
  {
    name: "mint / light",
    selector: 'html[data-theme="light"][data-palette="mint"]',
  },
  {
    name: "mint / dark",
    selector: 'html[data-theme="dark"][data-palette="mint"]',
  },
] as const

describe("low-emphasis text contrast (--rv-dim)", () => {
  for (const { name, selector } of DIM_CASES) {
    it(`${name} clears WCAG AA on the highest surface container`, () => {
      const dim = declaration(selector, "--rv-dim")
      const surface = declaration(
        selector,
        "--md-sys-color-surface-container-highest",
      )

      expect(dim).toMatch(/^#[0-9a-f]{6}$/i)
      expect(surface).toMatch(/^#[0-9a-f]{6}$/i)
      expect(contrastRatio(dim, surface)).toBeGreaterThanOrEqual(WCAG_AA_NORMAL)
    })
  }

  it("night derives dim text from the accessible on-surface-variant token", () => {
    expect(declaration('html[data-theme="night"]', "--rv-dim")).toBe(
      "var(--md-sys-color-on-surface-variant, #c3cad6)",
    )
  })

  it("high contrast pins dim text to the full-strength on-surface token", () => {
    expect(declaration('html[data-theme="hc"]', "--rv-dim")).toBe(
      "var(--md-sys-color-on-surface, #ffffff)",
    )
  })

  it("black on white is approximately 21:1", () => {
    expect(contrastRatio("#000000", "#ffffff")).toBeCloseTo(21, 0)
  })
})
