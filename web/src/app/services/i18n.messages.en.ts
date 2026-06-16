/**
 * English source catalog for the operator console.
 *
 * This is the single source of truth for user-visible UI strings that have been
 * externalized for localization. Keys are namespaced by surface area
 * (`nav.*`, `topbar.*`, `home.*`, `common.*`, `state.*`) so translators can work
 * on a coherent slice at a time and so the rollout can proceed surface by
 * surface (see docs/i18n.md).
 *
 * Conventions:
 * - Values are plain English strings. Interpolation placeholders use the
 *   `{name}` form and are passed through verbatim by the I18nService.
 * - Pluralized messages are expressed as a sibling pair of keys suffixed
 *   `.one` / `.other`; resolve them through `I18nService.plural(baseKey, count)`
 *   where the base key omits the suffix (e.g. `home.checksNeedReview`).
 * - Do NOT translate this file. Additional locales register their own catalog
 *   via `I18nService.registerCatalog(locale, catalog)`.
 */
export const EN_MESSAGES = {
  // ---- Primary navigation (rail + bottom bar) -----------------------------
  "nav.home": "Home",
  "nav.live": "Live",
  "nav.sessions": "Sessions",
  "nav.trends": "Trends",
  "nav.settings": "Settings",
  "nav.about": "About",

  // ---- Top bar ------------------------------------------------------------
  "topbar.simpleView": "Simple view",
  "topbar.advancedView": "Advanced view",
  "topbar.comfortableDensity": "Comfortable density",
  "topbar.compactDensity": "Compact density",
  "topbar.togglePolling": "Pause or resume live polling",
  "topbar.alerts": "Alerts",
  "topbar.settings": "Settings",
  "topbar.cycleTheme": "Cycle theme",

  // ---- Home / dashboard headings & controls -------------------------------
  "home.recordingDuration": "Recording duration",
  "home.radarPort": "Radar serial port",
  "home.bleAddress": "BLE address",
  "home.scanPorts": "Scan for radar serial ports",
  "home.rerunCheck": "Re-run check: {label}",
  "home.systemChecks": "System checks",

  // ---- Common actions -----------------------------------------------------
  "common.save": "Save",
  "common.cancel": "Cancel",
  "common.close": "Close",
  "common.retry": "Retry",
  "common.dismiss": "Dismiss",
  "common.confirm": "Confirm",

  // ---- State / status messages -------------------------------------------
  "state.loading": "Loading…",
  "state.connected": "Connected",
  "state.disconnected": "Disconnected",
  "state.error": "Something went wrong",
  "state.empty": "Nothing to show yet",

  // ---- Pluralized messages (resolve via I18nService.plural) ---------------
  "home.checksNeedReview.one": "{count} check needs review",
  "home.checksNeedReview.other": "{count} checks need review",
} as const

/** Union of every defined message key. */
export type MessageKey = keyof typeof EN_MESSAGES

/**
 * Shape of a locale catalog. Translations may be partial: any key a catalog
 * omits falls back to the English source at runtime, so a locale can be shipped
 * incrementally.
 */
export type LocaleCatalog = Partial<Record<MessageKey, string>> &
  Record<string, string>
