import { Injectable, computed, signal } from "@angular/core"
import { EN_MESSAGES, type LocaleCatalog } from "./i18n.messages.en"

/** Lightweight runtime localization service for the operator console. */
@Injectable({ providedIn: "root" })
export class I18nService {
  readonly locale = signal<string>("en")
  private readonly catalogs = new Map<string, LocaleCatalog>([["en", EN_MESSAGES]])
  private readonly catalogRevision = signal(0)
  readonly revision = this.catalogRevision.asReadonly()

  readonly activeCatalog = computed<LocaleCatalog>(() => {
    this.catalogRevision()
    return this.catalogs.get(this.locale()) ?? {}
  })

  private get english(): LocaleCatalog {
    return this.catalogs.get("en") ?? {}
  }

  registerCatalog(locale: string, catalog: LocaleCatalog): void {
    const normalized = locale.trim()
    if (!normalized) throw new Error("Locale code must not be empty")
    const existing = this.catalogs.get(normalized)
    this.catalogs.set(
      normalized,
      existing ? { ...existing, ...catalog } : { ...catalog },
    )
    this.catalogRevision.update((revision) => revision + 1)
  }

  setLocale(locale: string): void {
    this.locale.set(locale.trim() || "en")
  }

  availableLocales(): string[] {
    return [...this.catalogs.keys()]
  }

  translate(key: string, params?: Record<string, string | number | null | undefined>): string {
    return this.interpolate(this.lookup(key) ?? key, params)
  }

  plural(
    key: string,
    count: number,
    params?: Record<string, string | number | null | undefined>,
  ): string {
    const category = this.pluralCategory(count)
    const categoryKey = `${key}.${category}`
    const fallbackKey = `${key}.other`
    const template =
      this.lookup(categoryKey) ?? this.lookup(fallbackKey) ?? categoryKey
    return this.interpolate(template, { ...params, count })
  }

  private pluralCategory(count: number): string {
    if (typeof Intl === "undefined" || typeof Intl.PluralRules !== "function") {
      return Math.abs(count) === 1 ? "one" : "other"
    }

    try {
      return new Intl.PluralRules(this.locale()).select(count)
    } catch {
      return new Intl.PluralRules("en").select(count)
    }
  }

  private lookup(key: string): string | undefined {
    const active = this.activeCatalog() as Readonly<
      Record<string, string | undefined>
    >
    const english = this.english as Readonly<Record<string, string | undefined>>
    return active[key] ?? english[key]
  }

  private interpolate(
    template: string,
    params?: Record<string, string | number | null | undefined>,
  ): string {
    if (!params) return template
    return template.replace(/\{(\w+)\}/g, (match, name: string) => {
      const value = params[name]
      return value === undefined || value === null ? match : String(value)
    })
  }
}
