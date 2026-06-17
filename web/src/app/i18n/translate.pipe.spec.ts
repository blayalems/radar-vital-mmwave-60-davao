import { TestBed } from "@angular/core/testing"
import { beforeEach, describe, expect, it } from "vitest"
import { I18nService } from "../services/i18n.service"
import { TranslatePipe } from "./translate.pipe"

describe("TranslatePipe", () => {
  beforeEach(() => {
    TestBed.configureTestingModule({ providers: [I18nService] })
  })

  it("translates using the explicit locale cache key", () => {
    const pipe = TestBed.runInInjectionContext(() => new TranslatePipe())
    expect(pipe.transform("nav.home", "en", 0)).toBe("Home")
  })

  it("interpolates parameters without changing the pure-pipe contract", () => {
    const pipe = TestBed.runInInjectionContext(() => new TranslatePipe())
    expect(
      pipe.transform("home.rerunCheck", "en", 0, { label: "Radar link" }),
    ).toBe("Re-run check: Radar link")
  })

  it("uses the catalog revision as a pure-pipe invalidation key", () => {
    const i18n = TestBed.inject(I18nService)
    const pipe = TestBed.runInInjectionContext(() => new TranslatePipe())
    i18n.registerCatalog("fil", { "nav.home": "Tahanan" })
    i18n.setLocale("fil")
    expect(
      pipe.transform("nav.home", i18n.locale(), i18n.revision()),
    ).toBe("Tahanan")

    i18n.registerCatalog("fil", { "nav.home": "Pangunahing pahina" })
    expect(
      pipe.transform("nav.home", i18n.locale(), i18n.revision()),
    ).toBe("Pangunahing pahina")
  })
})
