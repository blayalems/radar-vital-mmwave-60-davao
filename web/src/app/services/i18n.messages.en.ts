/**
 * English source catalog for the operator console.
 *
 * Keys are namespaced by surface area. Interpolation placeholders use
 * `{name}`. Pluralized messages use category suffixes such as `.one` and
 * `.other` and are resolved through `I18nService.plural()`.
 */
export const EN_MESSAGES = {
  // Primary navigation
  "nav.home": "Home",
  "nav.live": "Live",
  "nav.sessions": "Sessions",
  "nav.trends": "Trends",
  "nav.settings": "Settings",
  "nav.about": "About",

  // Top bar
  "topbar.simpleView": "Simple view",
  "topbar.advancedView": "Advanced view",
  "topbar.comfortableDensity": "Comfortable density",
  "topbar.compactDensity": "Compact density",
  "topbar.togglePolling": "Pause or resume live polling",
  "topbar.alerts": "Alerts",
  "topbar.settings": "Settings",
  "topbar.cycleTheme": "Cycle theme",

  // Home/dashboard
  "home.recordingDuration": "Recording duration",
  "home.radarPort": "Radar serial port",
  "home.bleAddress": "BLE address",
  "home.scanPorts": "Scan for radar serial ports",
  "home.rerunCheck": "Re-run check: {label}",
  "home.systemChecks": "System checks",

  // Common actions
  "common.save": "Save",
  "common.cancel": "Cancel",
  "common.close": "Close",
  "common.retry": "Retry",
  "common.dismiss": "Dismiss",
  "common.confirm": "Confirm",

  // State/status
  "state.loading": "Loading…",
  "state.connected": "Connected",
  "state.disconnected": "Disconnected",
  "state.error": "Something went wrong",
  "state.empty": "Nothing to show yet",

  // Pluralized messages
  "home.checksNeedReview.one": "{count} check needs review",
  "home.checksNeedReview.other": "{count} checks need review",
} as const

export type MessageKey = keyof typeof EN_MESSAGES

/**
 * Locale catalogs are intentionally partial, but keys must exist in the English
 * source catalog so misspellings are caught at compile time.
 */
export type LocaleCatalog = Partial<Record<MessageKey, string>>
