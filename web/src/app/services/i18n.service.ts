import { Injectable, computed, signal } from "@angular/core"
import { EN_MESSAGES, type LocaleCatalog } from "./i18n.messages.en"

/**
 * Lightweight runtime localization service for the operator console.
 *
 * Design goals (see docs/i18n.md):
 * - No new heavy dependency and no Angular/Material build migration: this is a
 *   small signal-backed registry that components and the TranslatePipe read.
 * - English is the source of truth. Any key missing from the active locale
 *   falls back to English, and any key missing from English falls back to the
 *   key itself — so a partially translated locale never renders blank UI.
 * - Interpolation and pluralization semantics are explicit and stable so
 *   externalized strings keep the same runtime behaviour they had inline.
 */
@Injectable({ providedIn: "root" })
export class I18nService {
  /** Currently active locale code (BCP-47-ish, e.g. "en", "fil", "es"). */
  readonly locale = signal<string>("en")

  /** Registered catalogs keyed by locale code. English is always present. */
  private readonly catalogs = new Map<string, LocaleCatalog>([
    ["en", EN_MESSAGES as unknown as LocaleCatalog],
  ])

  /**
   * Internal signal bumped whenever a catalog is added/merged so that
   * `activeCatalog` (and any computed/template that reads it) recomputes.
   */
  private readonly catalogRevision = signal(0)

  /** The catalog for the active locale, or an empty object if none. */
  readonly activeCatalog = computed<LocaleCatalog>(() => {
    // Touch the revision so registerCatalog invalidates this computed.
    this.catalogRevision()
    return this.catalogs.get(this.locale()) ?? {}
  })

  private get english(): LocaleCatalog {
    return this.catalogs.get("en") ?? {}
  }

  /** Register (or merge into) the catalog for a locale. */
  registerCatalog(locale: string, catalog: LocaleCatalog): void {
    const existing = this.catalogs.get(locale)
    this.catalogs.set(locale, existing ? { ...existing, ...catalog } : { ...catalog })
    this.catalogRevision.update((n) => n + 1)
  }

  /** Switch the active locale. Unknown locales still resolve via English. */
  setLocale(locale: string): void {
    this.locale.set(locale)
  }

  /** List the locales that currently have a registered catalog. */
  availableLocales(): string[] {
    return [...this.catalogs.keys()]
  }

  /**
   * Resolve a key to a string for the active locale.
   * Resolution order: active locale → English source → the raw key.
   * `params` values replace `{name}` placeholders.
   */
  translate(key: string, params?: Record<string, string | number>): string {
    const active = this.activeCatalog()
    const template = active[key] ?? this.english[key] ?? key
    return this.interpolate(template, params)
  }

  /**
   * Resolve a pluralized message. Looks up `${key}.one` / `${key}.other`
   * (English categories) and always exposes the count as `{count}` in addition
   * to any caller-supplied params.
   */
  plural(
    key: string,
    count: number,
    params?: Record<string, string | number>,
  ): string {
    const category = Math.abs(count) === 1 ? "one" : "other"
    return this.translate(`${key}.${category}`, { count, ...params })
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
