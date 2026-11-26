/** English source catalog for the operator console. */
export const EN_MESSAGES = {
  "nav.home": "Home",
  "nav.settings": "Settings",
  "topbar.pausePolling": "Pause live polling",
  "topbar.resumePolling": "Resume live polling",
  "home.recordingDuration": "Recording duration",
  "home.radarPort": "Radar port",
  "home.bleAddress": "BLE oximeter",
  "home.scanPorts": "Scan for radar serial ports",
  "home.rerunCheck": "Re-run check: {label}",
  "home.checksNeedReview.one": "{count} check needs review",
  "home.checksNeedReview.other": "{count} checks need review",
} as const

export type MessageKey = keyof typeof EN_MESSAGES
export type LocaleCatalog = Partial<Record<MessageKey, string>>
