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
    expect(pipe.transform("nav.home", "en")).toBe("Home")
  })

  it("interpolates parameters without changing the pure-pipe contract", () => {
    const pipe = TestBed.runInInjectionContext(() => new TranslatePipe())
    expect(
      pipe.transform("home.rerunCheck", "en", { label: "Radar link" }),
    ).toBe("Re-run check: Radar link")
  })
})
