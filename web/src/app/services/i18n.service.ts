import { Injectable, computed, signal } from "@angular/core"
import { EN_MESSAGES, type LocaleCatalog } from "./i18n.messages.en"

/** Lightweight runtime localization service for the operator console. */
@Injectable({ providedIn: "root" })
export class I18nService {
  /** Currently active BCP-47 locale code. */
  readonly locale = signal<string>("en")

  /** Registered catalogs keyed by locale code. English is always present. */
  private readonly catalogs = new Map<string, LocaleCatalog>([["en", EN_MESSAGES]])

  /** Bumped whenever a catalog is registered or merged. */
  private readonly catalogRevision = signal(0)

  /** Catalog for the active locale, or an empty catalog for unknown locales. */
  readonly activeCatalog = computed<LocaleCatalog>(() => {
    this.catalogRevision()
    return this.catalogs.get(this.locale()) ?? {}
  })

  private get english(): LocaleCatalog {
    return this.catalogs.get("en") ?? {}
  }

  /** Register or merge a partial catalog for a locale. */
  registerCatalog(locale: string, catalog: LocaleCatalog): void {
    const normalized = locale.trim()
    if (!normalized) {
      throw new Error("Locale code must not be empty")
    }

    const existing = this.catalogs.get(normalized)
    this.catalogs.set(
      normalized,
      existing ? { ...existing, ...catalog } : { ...catalog },
    )
    this.catalogRevision.update((revision) => revision + 1)
  }

  /** Switch the active locale. Unknown locales still fall back to English. */
  setLocale(locale: string): void {
    const normalized = locale.trim()
    this.locale.set(normalized || "en")
  }

  /** List registered locale codes. */
  availableLocales(): string[] {
    return [...this.catalogs.keys()]
  }

  /**
   * Resolve a key using active locale → English → raw key fallback, then
   * interpolate `{name}` placeholders.
   */
  translate(key: string, params?: Record<string, string | number>): string {
    const template = this.lookup(key) ?? key
    return this.interpolate(template, params)
  }

  /**
   * Resolve a pluralized message using the active locale's plural rules.
   * Locale-specific categories fall back to `.other` when the catalog does not
   * provide them. The numeric argument always wins over a caller-supplied
   * `params.count` value.
   */
  plural(
    key: string,
    count: number,
    params?: Record<string, string | number>,
  ): string {
    const category = new Intl.PluralRules(this.locale()).select(count)
    const categoryKey = `${key}.${category}`
    const fallbackKey = `${key}.other`
    const template =
      this.lookup(categoryKey) ?? this.lookup(fallbackKey) ?? categoryKey

    return this.interpolate(template, { ...params, count })
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
    params?: Record<string, string | number>,
  ): string {
    if (!params) return template

    return template.replace(/\{(\w+)\}/g, (match, name: string) => {
      const value = params[name]
      return value === undefined || value === null ? match : String(value)
    })
  }
}
