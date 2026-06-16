import { beforeEach, describe, expect, it } from "vitest"
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

  it("interpolates placeholders", () => {
    expect(i18n.translate("home.rerunCheck", { label: "Radar link" })).toBe(
      "Re-run check: Radar link",
    )
  })

  it("leaves unmatched placeholders untouched", () => {
    expect(i18n.translate("home.rerunCheck")).toBe("Re-run check: {label}")
  })

  it("selects one/other plural categories", () => {
    expect(i18n.plural("home.checksNeedReview", 1)).toBe(
      "1 check needs review",
    )
    expect(i18n.plural("home.checksNeedReview", 3)).toBe(
      "3 checks need review",
    )
  })

  it("does not let caller parameters override the plural count", () => {
    expect(
      i18n.plural("home.checksNeedReview", 1, { count: 99 }),
    ).toBe("1 check needs review")
  })

  it("falls back to .other for unsupported locale-specific categories", () => {
    i18n.registerCatalog("ar", {
      "home.checksNeedReview.other": "{count} checks need review",
    })
    i18n.setLocale("ar")
    expect(i18n.plural("home.checksNeedReview", 2)).toBe(
      "2 checks need review",
    )
  })

  it("uses the active locale and falls back to English per key", () => {
    i18n.registerCatalog("fil", { "nav.home": "Tahanan" })
    i18n.setLocale("fil")
    expect(i18n.translate("nav.home")).toBe("Tahanan")
    expect(i18n.translate("nav.settings")).toBe("Settings")
  })

  it("reports registered locales", () => {
    i18n.registerCatalog("es", { "nav.home": "Inicio" })
    expect(i18n.availableLocales()).toContain("en")
    expect(i18n.availableLocales()).toContain("es")
  })

  it("rejects an empty locale code", () => {
    expect(() => i18n.registerCatalog("   ", {})).toThrow(
      "Locale code must not be empty",
    )
  })
})
