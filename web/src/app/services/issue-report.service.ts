/**
 * IssueReportService — one-tap GitHub issue reporting with diagnostics opt-out.
 *
 * Collects {product version + build info, platform string, connection mode,
 * ctlStatus summary, de-identified alert summary, EXE log tail} and builds a
 * pre-filled GitHub issue URL.  All content is de-identified per the
 * operator-handoff-dialog precedent (counts/types only, no operator names).
 *
 * The diagnostics toggle is stored under DIAGNOSTICS_OPTIN_KEY and defaults
 * to TRUE (opt-in) when the key is absent.  When off, the report contains
 * ONLY version + platform.
 *
 * URL is hard-capped at 7 500 chars; truncation drops the log tail first,
 * then the alert summary, never version/platform.
 *
 * On Tauri the shell-open plugin opens the URL in the system browser;
 * otherwise window.open is used.
 */

import { Injectable, computed, inject, signal } from '@angular/core';

import { GITHUB_REPO_URL } from './app-meta';
import { DIAGNOSTICS_OPTIN_KEY } from './rvt-storage-keys';
import { ApiService } from './api.service';
import { ServerLifecycleService } from './server-lifecycle.service';
import { StateService } from './state.service';

// ---- type aliases ----------------------------------------------------------

export type ControlErrorCategory = 'authentication' | 'timeout' | 'tls' | 'unreachable' | 'server' | 'other';

export interface IssueReport {
  /** Always present — never dropped by truncation. */
  product_version: string;
  /** Always present — never dropped by truncation. */
  platform: 'Windows EXE' | 'Android APK' | 'PWA (browser)';
  /** Present when diagnostics are enabled. */
  connection_mode?: 'Local EXE (bundled trainer)' | 'LAN paired (phone/PWA to trainer)' | 'Demo / sandbox' | 'Unknown';
  /** De-identified ctl status (no message text). Present when diagnostics are enabled. */
  ctl?: {
    ok: boolean | undefined;
    mode: string | undefined;
    latency: number | undefined;
    error_category: ControlErrorCategory | undefined;
  };
  /** De-identified alert summary — counts and last-5 {severity, source, age_s}. Present when diagnostics are enabled. */
  alerts?: {
    counts: { info: number; warn: number; critical: number };
    last5: Array<{ severity: string; source: string | undefined; age_s: number }>;
  };
  /** EXE log tail — last 20 lines.  Dropped first during URL truncation. */
  log_tail?: string[];
}

// ---- window augments -------------------------------------------------------

interface CapacitorBridge {
  isNativePlatform?(): boolean;
}

interface TauriCore {
  invoke<T = unknown>(command: string, args?: Record<string, unknown>): Promise<T>;
}

interface TauriBridge {
  core?: TauriCore;
}

interface VersionResponse {
  product_version?: string;
}

interface TrainerLogResponse {
  lines?: string[];
}

// ---- constants -------------------------------------------------------------

const MAX_URL = 7_500;
const CONTROL_ERROR_CATEGORIES = new Set<ControlErrorCategory>([
  'authentication',
  'timeout',
  'tls',
  'unreachable',
  'server',
  'other'
]);
const CONTROL_MODES = new Set(['live', 'sandbox', 'loading']);
const ALERT_SOURCES = new Set([
  'ctl',
  'fault',
  'firmware',
  'heart-rate',
  'pairing',
  'radar',
  'respiration',
  'session',
  'sse-session-warning',
  'sse-stopped',
  'system',
  'telemetry'
]);

function escapeRegExp(value: string): string {
  return value.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}

export function sanitizeDiagnosticLine(line: string, identityHints: string[] = []): string {
  let redacted = String(line ?? '');

  redacted = redacted.replace(/([?&]pair=)\d{6}\b/gi, '$1[REDACTED_PIN]');
  redacted = redacted.replace(/\b(pair(?:ing)?[_ -]?(?:pin|code)|active_pin)\b\s*[:=]\s*["']?\d{6}\b/gi, '$1=[REDACTED_PIN]');
  redacted = redacted.replace(/\b(X-RVT-Auth|Authorization)\b\s*[:=]\s*(Bearer\s+)?[A-Za-z0-9._~+/=-]{8,}/gi, '$1: [REDACTED_TOKEN]');
  redacted = redacted.replace(/\bBearer\s+[A-Za-z0-9._~+/=-]{8,}/gi, 'Bearer [REDACTED_TOKEN]');
  redacted = redacted.replace(/\b(rvt-operator-token|rvt-pair-token|operator_token|session_token|sse_token|token)\b\s*[:=]\s*["']?[A-Za-z0-9._~+/=-]{8,}/gi, '$1=[REDACTED_TOKEN]');
  redacted = redacted.replace(/\b[A-Z2-9]{4}-[A-Z2-9]{4}-[A-Z2-9]{4}\b/g, '[REDACTED_RECOVERY_CODE]');
  redacted = redacted.replace(/\b(?:sandbox_op|op)_[A-Za-z0-9_-]+\b/g, '[REDACTED_OPERATOR_ID]');
  redacted = redacted.replace(/\b(display_name|operator_label|initials)\b\s*[:=]\s*["']?[^"',;\]\}]+/gi, '$1=[REDACTED_OPERATOR]');
  redacted = redacted.replace(/[A-Za-z]:\\Users\\[^\\\s]+/g, 'C:\\Users\\[REDACTED_USER]');
  redacted = redacted.replace(/\/Users\/[^\/\s]+/g, '/Users/[REDACTED_USER]');

  for (const hint of identityHints) {
    const clean = String(hint || '').trim();
    if (clean.length < 3) continue;
    redacted = redacted.replace(new RegExp(escapeRegExp(clean), 'gi'), '[REDACTED_OPERATOR]');
  }

  return redacted;
}

export function categorizeControlError(value: unknown): ControlErrorCategory | undefined {
  const text = String(value ?? '').trim().toLowerCase();
  if (!text) return undefined;
  if (/(?:\b401\b|\b403\b|auth|bearer|forbidden|pair(?:ing)?|pin|token|unauthori[sz]ed)/.test(text)) {
    return 'authentication';
  }
  if (/(?:timeout|timed out|deadline)/.test(text)) return 'timeout';
  if (/(?:certificate|cert\b|https|ssl|tls)/.test(text)) return 'tls';
  if (/(?:connection refused|failed to fetch|network|offline|unreachable|unavailable)/.test(text)) {
    return 'unreachable';
  }
  if (/(?:\b5\d\d\b|internal server|server error)/.test(text)) return 'server';
  return 'other';
}

function normalizeControlMode(value: unknown): string | undefined {
  const mode = String(value ?? '').trim().toLowerCase();
  if (!mode) return undefined;
  return CONTROL_MODES.has(mode) ? mode : 'unknown';
}

function normalizeAlertSource(value: unknown): string | undefined {
  const source = String(value ?? '').trim().toLowerCase();
  if (!source) return undefined;
  return ALERT_SOURCES.has(source) ? source : 'other';
}

function safeControlSummary(value: unknown): IssueReport['ctl'] {
  if (!value || typeof value !== 'object' || Array.isArray(value)) return undefined;
  const raw = value as Record<string, unknown>;
  const explicitCategory = String(raw['error_category'] ?? '').trim().toLowerCase();
  const category = CONTROL_ERROR_CATEGORIES.has(explicitCategory as ControlErrorCategory)
    ? explicitCategory as ControlErrorCategory
    : categorizeControlError([raw['error'], raw['reason'], raw['message']].filter(Boolean).join(' '));
  const latency = Number(raw['latency']);
  return {
    ok: typeof raw['ok'] === 'boolean' ? raw['ok'] : undefined,
    mode: normalizeControlMode(raw['mode']),
    latency: Number.isFinite(latency) ? Math.max(0, Math.round(latency)) : undefined,
    error_category: category
  };
}

function safeAlertSummary(value: unknown): IssueReport['alerts'] {
  if (!value || typeof value !== 'object' || Array.isArray(value)) return undefined;
  const raw = value as Record<string, unknown>;
  const rawCounts = raw['counts'] && typeof raw['counts'] === 'object' && !Array.isArray(raw['counts'])
    ? raw['counts'] as Record<string, unknown>
    : {};
  const count = (key: string) => {
    const numeric = Number(rawCounts[key]);
    return Number.isFinite(numeric) ? Math.max(0, Math.round(numeric)) : 0;
  };
  const last5 = Array.isArray(raw['last5'])
    ? raw['last5'].slice(0, 5).map(entry => {
        const item = entry && typeof entry === 'object' && !Array.isArray(entry)
          ? entry as Record<string, unknown>
          : {};
        const age = Number(item['age_s']);
        return {
          severity: ['info', 'warn', 'critical'].includes(String(item['severity']))
            ? String(item['severity'])
            : 'unknown',
          source: normalizeAlertSource(item['source']),
          age_s: Number.isFinite(age) ? Math.max(0, Math.round(age)) : 0
        };
      })
    : [];
  return {
    counts: { info: count('info'), warn: count('warn'), critical: count('critical') },
    last5
  };
}

// ---- service ---------------------------------------------------------------

@Injectable({ providedIn: 'root' })
export class IssueReportService {
  private readonly api = inject(ApiService);
  private readonly state = inject(StateService);
  private readonly serverLifecycle = inject(ServerLifecycleService);

  /** Whether the user has opted in to including diagnostics in reports. Defaults to TRUE when key absent. */
  readonly diagnosticsEnabled = signal<boolean>(this.readOptIn());

  /** Computed alias for template binding. */
  readonly isEnabled = computed(() => this.diagnosticsEnabled());

  /** Persist the preference and update the signal. */
  setDiagnosticsEnabled(value: boolean): void {
    this.diagnosticsEnabled.set(value);
    try {
      localStorage.setItem(DIAGNOSTICS_OPTIN_KEY, value ? '1' : '0');
    } catch (_) {
      /* storage unavailable */
    }
  }

  // --------------------------------------------------------------------------
  // Public API
  // --------------------------------------------------------------------------

  /**
   * Collect the full diagnostics report.  When diagnostics are disabled,
   * returns only version + platform.
   */
  async buildReport(): Promise<IssueReport> {
    const platform = this.detectPlatform();
    const productVersion = await this.fetchProductVersion();

    if (!this.diagnosticsEnabled()) {
      return { product_version: productVersion, platform };
    }

    const connectionMode = this.detectConnectionMode();
    const ctl = this.collectCtlStatus();
    const alerts = this.collectAlerts();
    const logTail = await this.collectLogTail();

    const report: IssueReport = {
      product_version: productVersion,
      platform,
      connection_mode: connectionMode,
      ctl,
      alerts,
    };

    if (logTail !== undefined) {
      report.log_tail = logTail;
    }

    return report;
  }

  /**
   * Build a pre-filled GitHub issue URL from a report.
   * Total URL is hard-capped at MAX_URL chars — truncation drops log_tail
   * first, then alert summary, never version/platform.
   *
   * @param report   Result of buildReport().
   * @param userText Optional short description pre-filled into the `description` field.
   */
  buildIssueUrl(report: IssueReport, userText?: string): string {
    const base = `${GITHUB_REPO_URL}/issues/new`;
    const params = new URLSearchParams();
    params.set('template', 'bug_report.yml');
    params.set('product_version', report.product_version);
    params.set('platform', report.platform);

    if (userText) {
      params.set('description', userText);
    }

    if (this.diagnosticsEnabled() && report.connection_mode) {
      params.set('connection_mode', report.connection_mode);
    }

    if (!this.diagnosticsEnabled()) {
      return `${base}?${params.toString()}`;
    }

    // Build full diagnostics payload and try to fit it.
    const diagFull: Partial<IssueReport> = {
      product_version: report.product_version,
      platform: report.platform,
      connection_mode: report.connection_mode,
      ctl: safeControlSummary(report.ctl),
      alerts: safeAlertSummary(report.alerts),
      log_tail: this.sanitizeLogTail(report.log_tail),
    };

    // 1. Try with log tail.
    params.set('diagnostics', JSON.stringify(diagFull, null, 2));
    if (`${base}?${params.toString()}`.length <= MAX_URL) {
      return `${base}?${params.toString()}`;
    }

    // 2. Drop log tail.
    const noLog = { ...diagFull };
    delete noLog.log_tail;
    params.set('diagnostics', JSON.stringify(noLog, null, 2));
    if (`${base}?${params.toString()}`.length <= MAX_URL) {
      return `${base}?${params.toString()}`;
    }

    // 3. Drop alert summary.
    const noLogNoAlerts = { ...noLog };
    delete noLogNoAlerts.alerts;
    params.set('diagnostics', JSON.stringify(noLogNoAlerts, null, 2));
    if (`${base}?${params.toString()}`.length <= MAX_URL) {
      return `${base}?${params.toString()}`;
    }

    // 4. Drop diagnostics entirely — keep version/platform.
    params.delete('diagnostics');
    return `${base}?${params.toString()}`;
  }

  /**
   * Open a URL in the system browser.
   * On Tauri: uses the shell-open plugin.
   * Otherwise: window.open with noopener.
   */
  async openReport(url: string): Promise<void> {
    const tauri = (window as Window & { __TAURI__?: TauriBridge }).__TAURI__;
    if (tauri?.core?.invoke) {
      try {
        await tauri.core.invoke('plugin:shell|open', { path: url });
        return;
      } catch (_) {
        /* fall through to window.open */
      }
    }
    try {
      window.open(url, '_blank', 'noopener');
    } catch (_) {
      /* no-op */
    }
  }

  // --------------------------------------------------------------------------
  // Private helpers
  // --------------------------------------------------------------------------

  private readOptIn(): boolean {
    try {
      const stored = localStorage.getItem(DIAGNOSTICS_OPTIN_KEY);
      return stored === null ? true : stored === '1';
    } catch (_) {
      return true;
    }
  }

  private async fetchProductVersion(): Promise<string> {
    try {
      const ver = await this.api.request<VersionResponse>('/api/version');
      return ver?.product_version ?? 'unknown';
    } catch (_) {
      return 'unknown';
    }
  }

  private detectPlatform(): 'Windows EXE' | 'Android APK' | 'PWA (browser)' {
    const tauri = (window as Window & { __TAURI__?: TauriBridge }).__TAURI__;
    if (tauri?.core?.invoke) {
      return 'Windows EXE';
    }
    const cap = (window as Window & { Capacitor?: CapacitorBridge }).Capacitor;
    if (cap?.isNativePlatform?.()) {
      return 'Android APK';
    }
    return 'PWA (browser)';
  }

  private detectConnectionMode(): IssueReport['connection_mode'] {
    if (this.state.demoMode() || this.state.autoDemoActive()) {
      return 'Demo / sandbox';
    }
    const ctl = this.state.ctlStatus();
    if (ctl?.mode === 'sandbox') {
      return 'Demo / sandbox';
    }
    if (this.serverLifecycle.platform() === 'exe') {
      return 'Local EXE (bundled trainer)';
    }
    if (ctl?.ok) {
      return 'LAN paired (phone/PWA to trainer)';
    }
    return 'Unknown';
  }

  private collectCtlStatus(): IssueReport['ctl'] {
    return safeControlSummary(this.state.ctlStatus());
  }

  private collectAlerts(): IssueReport['alerts'] {
    const allAlerts = this.state.alertHistory();
    const now = Date.now();
    const counts = { info: 0, warn: 0, critical: 0 };
    for (const a of allAlerts) {
      if (a.severity === 'info') counts.info++;
      else if (a.severity === 'warn') counts.warn++;
      else if (a.severity === 'critical') counts.critical++;
    }
    // Sort explicitly: AlertStore is newest-first, while imports/restores may
    // not be. Keep only category/source/age; never copy message text.
    const last5 = [...allAlerts]
      .sort((a, b) => Number(b.ts) - Number(a.ts))
      .slice(0, 5)
      .map(a => ({
        severity: a.severity as string,
        source: normalizeAlertSource(a.source),
        age_s: Math.max(0, Math.round((now - a.ts) / 1000)),
      }));
    return { counts, last5 };
  }

  private async collectLogTail(): Promise<string[] | undefined> {
    if (this.serverLifecycle.platform() === 'exe') {
      const lines = this.serverLifecycle.logTail();
      return this.sanitizeLogTail(lines);
    }
    try {
      const resp = await this.api.request<TrainerLogResponse>('/api/trainer/log');
      if (Array.isArray(resp?.lines)) {
        return this.sanitizeLogTail(resp.lines);
      }
    } catch (_) {
      /* omit log_tail on error */
    }
    return undefined;
  }

  private sanitizeLogTail(lines: string[] | undefined): string[] | undefined {
    if (!Array.isArray(lines)) return undefined;
    const hints = this.identityHints();
    return lines.slice(-20).map(line => sanitizeDiagnosticLine(line, hints));
  }

  private identityHints(): string[] {
    const hints = new Set<string>();
    try {
      const setup = this.state.setup();
      if (setup?.operator_label) hints.add(setup.operator_label);
    } catch (_) {}
    return Array.from(hints);
  }
}
