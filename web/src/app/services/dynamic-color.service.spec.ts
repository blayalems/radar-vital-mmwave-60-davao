import { TestBed } from "@angular/core/testing"
import { afterEach, beforeEach, describe, expect, it } from "vitest"
import { WCAG_AA_NORMAL, contrastRatio } from "../a11y/contrast-core.mjs"
import { DynamicColorService } from "./dynamic-color.service"

const SEEDS = [
  "#0061a4",
  "#ff0000",
  "#00ff00",
  "#0000ff",
  "#00ffff",
  "#ff00ff",
]

const THEMES = ["light", "dark", "night"] as const
const BACKGROUND_TOKENS = [
  "--md-sys-color-surface",
  "--md-sys-color-surface-container-lowest",
  "--md-sys-color-surface-container-low",
  "--md-sys-color-surface-container",
  "--md-sys-color-surface-container-high",
  "--md-sys-color-surface-container-highest",
  "--md-sys-color-secondary-container",
]

describe("DynamicColorService low-emphasis contrast", () => {
  let service: DynamicColorService

  beforeEach(() => {
    TestBed.resetTestingModule()
    localStorage.clear()
    const html = document.documentElement
    html.removeAttribute("style")
    delete html.dataset["dynamicColor"]
    delete html.dataset["palette"]
    TestBed.configureTestingModule({ providers: [DynamicColorService] })
    service = TestBed.inject(DynamicColorService)
  })

  afterEach(() => {
    const html = document.documentElement
    html.removeAttribute("style")
    delete html.dataset["dynamicColor"]
    delete html.dataset["theme"]
    TestBed.resetTestingModule()
  })

  it("keeps generated on-surface-variant text above WCAG AA on every generated surface", () => {
    const html = document.documentElement

    for (const theme of THEMES) {
      html.dataset["theme"] = theme
      for (const seed of SEEDS) {
        service.setSourceColor(seed)
        service.setEnabled(true)
        service.reapply()

        expect(html.dataset["dynamicColor"]).toBe("1")
        const foreground = html.style
          .getPropertyValue("--md-sys-color-on-surface-variant")
          .trim()
        expect(foreground).toMatch(/^#[0-9a-f]{6}$/i)

        for (const token of BACKGROUND_TOKENS) {
          const background = html.style.getPropertyValue(token).trim()
          expect(background).toMatch(/^#[0-9a-f]{6}$/i)
          expect(
            contrastRatio(foreground, background),
            `${theme} ${seed}: ${foreground} on ${token} ${background}`,
          ).toBeGreaterThanOrEqual(WCAG_AA_NORMAL)
        }
      }
    }
  })
})
