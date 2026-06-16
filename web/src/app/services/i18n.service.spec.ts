import { describe, it, expect, beforeEach } from "vitest"
import { I18nService } from "./i18n.service"

describe("I18nService", () => {
  let i18n: I18nService

  beforeEach(() => {
    i18n = new I18nService()
  })

  it("resolves an English source key", () => {
    expect(i18n.translate("nav.home")).toBe("Home")
  })

  it("falls back to the raw key when unknown", () => {
    expect(i18n.translate("does.not.exist")).toBe("does.not.exist")
  })

  it("interpolates {name} placeholders", () => {
    expect(i18n.translate("home.rerunCheck", { label: "Radar link" })).toBe(
      "Re-run check: Radar link",
    )
  })

  it("leaves unmatched placeholders untouched", () => {
    expect(i18n.translate("home.rerunCheck")).toBe("Re-run check: {label}")
  })

  it("selects the one/other plural category", () => {
    expect(i18n.plural("home.checksNeedReview", 1)).toBe("1 check needs review")
    expect(i18n.plural("home.checksNeedReview", 3)).toBe("3 checks need review")
  })

  it("uses the active locale and falls back to English per key", () => {
    i18n.registerCatalog("fil", { "nav.home": "Tahanan" })
    i18n.setLocale("fil")
    // Provided by the Filipino catalog.
    expect(i18n.translate("nav.home")).toBe("Tahanan")
    // Missing from Filipino → English source fallback.
    expect(i18n.translate("nav.settings")).toBe("Settings")
  })

  it("reports registered locales", () => {
    i18n.registerCatalog("es", { "nav.home": "Inicio" })
    expect(i18n.availableLocales()).toContain("en")
    expect(i18n.availableLocales()).toContain("es")
  })
})
