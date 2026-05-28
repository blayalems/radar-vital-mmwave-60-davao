import { Injectable, signal, effect } from '@angular/core';

/**
 * Material You dynamic color service.
 * Generates M3 tonal palettes from a user-chosen source color using
 * a self-contained implementation of the HCT (Hue-Chroma-Tone) algorithm.
 *
 * This avoids importing @material/material-color-utilities at runtime
 * to prevent ESM resolution issues in Vitest and keep the bundle lean.
 */

const STORAGE_KEY = 'rvt-dynamic-color';
const DEFAULT_SOURCE = '#0061a4';

// ── Minimal HCT color science (inlined from Material Color Utilities) ────────

/** sRGB linearization */
function linearized(rgb8: number): number {
  const v = rgb8 / 255;
  return v <= 0.04045 ? v / 12.92 : Math.pow((v + 0.055) / 1.055, 2.4);
}

/** sRGB delinearization */
function delinearized(lin: number): number {
  const v = lin <= 0.0031308 ? lin * 12.92 : 1.055 * Math.pow(lin, 1 / 2.4) - 0.055;
  return Math.round(Math.max(0, Math.min(255, v * 255)));
}

function argbFromRgb(r: number, g: number, b: number): number {
  return (0xff << 24) | (r << 16) | (g << 8) | b;
}

function argbFromHex(hex: string): number {
  const c = hex.replace('#', '');
  return argbFromRgb(
    parseInt(c.substring(0, 2), 16),
    parseInt(c.substring(2, 4), 16),
    parseInt(c.substring(4, 6), 16)
  );
}

function hexFromArgb(argb: number): string {
  const r = (argb >> 16) & 0xff;
  const g = (argb >> 8) & 0xff;
  const b = argb & 0xff;
  return '#' + [r, g, b].map(c => c.toString(16).padStart(2, '0')).join('');
}

function redFromArgb(argb: number): number { return (argb >> 16) & 0xff; }
function greenFromArgb(argb: number): number { return (argb >> 8) & 0xff; }
function blueFromArgb(argb: number): number { return argb & 0xff; }

/** CIE-L*a*b* from ARGB */
function labFromArgb(argb: number): [number, number, number] {
  const lr = linearized(redFromArgb(argb));
  const lg = linearized(greenFromArgb(argb));
  const lb = linearized(blueFromArgb(argb));

  const x = 0.4124564 * lr + 0.3575761 * lg + 0.1804375 * lb;
  const y = 0.2126729 * lr + 0.7151522 * lg + 0.0721750 * lb;
  const z = 0.0193339 * lr + 0.1191920 * lg + 0.9503041 * lb;

  const xn = 0.95047, yn = 1.0, zn = 1.08883;
  const f = (t: number) => t > 0.008856 ? Math.cbrt(t) : (903.3 * t + 16) / 116;

  const fx = f(x / xn), fy = f(y / yn), fz = f(z / zn);
  const L = 116 * fy - 16;
  const a = 500 * (fx - fy);
  const bVal = 200 * (fy - fz);
  return [L, a, bVal];
}

/** Extract hue from ARGB via Lab */
function hueFromArgb(argb: number): number {
  const [, a, b] = labFromArgb(argb);
  let hue = (Math.atan2(b, a) * 180) / Math.PI;
  if (hue < 0) hue += 360;
  return hue;
}

/** Perceived lightness (CIE L*) */
function toneFromArgb(argb: number): number {
  return labFromArgb(argb)[0];
}

/** Generate an ARGB color from hue, chroma approximation, and tone (CIE L*) */
function argbFromHct(hue: number, chroma: number, tone: number): number {
  // Convert tone to target L*
  const targetL = tone;
  // Use Lab with polar chroma/hue
  const hRad = (hue * Math.PI) / 180;
  const a = chroma * Math.cos(hRad);
  const b = chroma * Math.sin(hRad);

  // Lab to XYZ
  const fy = (targetL + 16) / 116;
  const fx = a / 500 + fy;
  const fz = fy - b / 200;

  const xn = 0.95047, yn = 1.0, zn = 1.08883;
  const invF = (t: number) => t > 0.206897 ? t * t * t : (116 * t - 16) / 903.3;

  const X = xn * invF(fx);
  const Y = yn * invF(fy);
  const Z = zn * invF(fz);

  // XYZ to linear sRGB
  const lr = 3.2404542 * X - 1.5371385 * Y - 0.4985314 * Z;
  const lg = -0.9692660 * X + 1.8760108 * Y + 0.0415560 * Z;
  const lb = 0.0556434 * X - 0.2040259 * Y + 1.0572252 * Z;

  return argbFromRgb(delinearized(lr), delinearized(lg), delinearized(lb));
}

/** Generate a tonal palette (13 tones) from a source hue and chroma. */
function tonalPalette(hue: number, chroma: number): Map<number, number> {
  const tones = [0, 4, 6, 10, 12, 17, 20, 22, 24, 25, 30, 35, 40, 50, 60, 70, 80, 87, 90, 92, 94, 95, 96, 98, 99, 100];
  const map = new Map<number, number>();
  for (const tone of tones) {
    map.set(tone, argbFromHct(hue, chroma, tone));
  }
  return map;
}

/** Extract approximate chroma from ARGB */
function chromaFromArgb(argb: number): number {
  const [, a, b] = labFromArgb(argb);
  return Math.sqrt(a * a + b * b);
}

// ── M3 Scheme Generation ─────────────────────────────────────────────────────

interface M3Scheme {
  primary: number; onPrimary: number; primaryContainer: number; onPrimaryContainer: number;
  secondary: number; onSecondary: number; secondaryContainer: number; onSecondaryContainer: number;
  tertiary: number; onTertiary: number; tertiaryContainer: number; onTertiaryContainer: number;
  error: number; onError: number; errorContainer: number; onErrorContainer: number;
  surface: number; onSurface: number; onSurfaceVariant: number;
  outline: number; outlineVariant: number;
  surfaceContainerLowest: number; surfaceContainerLow: number; surfaceContainer: number;
  surfaceContainerHigh: number; surfaceContainerHighest: number;
  inverseSurface: number; inverseOnSurface: number; inversePrimary: number;
}

function schemeFromSourceColor(sourceArgb: number, isDark: boolean): M3Scheme {
  const sourceHue = hueFromArgb(sourceArgb);
  const sourceChroma = Math.max(chromaFromArgb(sourceArgb), 48); // Ensure vivid enough

  const primary = tonalPalette(sourceHue, sourceChroma);
  const secondary = tonalPalette(sourceHue, sourceChroma / 3);
  const tertiary = tonalPalette(sourceHue + 60, sourceChroma / 2);
  const neutral = tonalPalette(sourceHue, Math.min(sourceChroma / 12, 4));
  const neutralVariant = tonalPalette(sourceHue, Math.min(sourceChroma / 6, 8));
  const error = tonalPalette(25, 84); // Red error palette

  const get = (p: Map<number, number>, t: number) => p.get(t) ?? 0;

  if (isDark) {
    return {
      primary: get(primary, 80), onPrimary: get(primary, 20),
      primaryContainer: get(primary, 30), onPrimaryContainer: get(primary, 90),
      secondary: get(secondary, 80), onSecondary: get(secondary, 20),
      secondaryContainer: get(secondary, 30), onSecondaryContainer: get(secondary, 90),
      tertiary: get(tertiary, 80), onTertiary: get(tertiary, 20),
      tertiaryContainer: get(tertiary, 30), onTertiaryContainer: get(tertiary, 90),
      error: get(error, 80), onError: get(error, 20),
      errorContainer: get(error, 30), onErrorContainer: get(error, 90),
      surface: get(neutral, 6), onSurface: get(neutral, 90),
      onSurfaceVariant: get(neutralVariant, 80),
      outline: get(neutralVariant, 60), outlineVariant: get(neutralVariant, 30),
      surfaceContainerLowest: get(neutral, 4), surfaceContainerLow: get(neutral, 10),
      surfaceContainer: get(neutral, 12), surfaceContainerHigh: get(neutral, 17),
      surfaceContainerHighest: get(neutral, 22),
      inverseSurface: get(neutral, 90), inverseOnSurface: get(neutral, 20),
      inversePrimary: get(primary, 40),
    };
  } else {
    return {
      primary: get(primary, 40), onPrimary: get(primary, 100),
      primaryContainer: get(primary, 90), onPrimaryContainer: get(primary, 10),
      secondary: get(secondary, 40), onSecondary: get(secondary, 100),
      secondaryContainer: get(secondary, 90), onSecondaryContainer: get(secondary, 10),
      tertiary: get(tertiary, 40), onTertiary: get(tertiary, 100),
      tertiaryContainer: get(tertiary, 90), onTertiaryContainer: get(tertiary, 10),
      error: get(error, 40), onError: get(error, 100),
      errorContainer: get(error, 90), onErrorContainer: get(error, 10),
      surface: get(neutral, 98), onSurface: get(neutral, 10),
      onSurfaceVariant: get(neutralVariant, 30),
      outline: get(neutralVariant, 50), outlineVariant: get(neutralVariant, 80),
      surfaceContainerLowest: get(neutral, 100), surfaceContainerLow: get(neutral, 96),
      surfaceContainer: get(neutral, 94), surfaceContainerHigh: get(neutral, 92),
      surfaceContainerHighest: get(neutral, 90),
      inverseSurface: get(neutral, 20), inverseOnSurface: get(neutral, 95),
      inversePrimary: get(primary, 80),
    };
  }
}

// ── Angular Service ──────────────────────────────────────────────────────────

@Injectable({
  providedIn: 'root'
})
export class DynamicColorService {
  /** The user-chosen source color in hex. */
  readonly sourceColor = signal<string>(this.loadSourceColor());

  /** Whether Material You dynamic theming is enabled (vs. the static theme). */
  readonly enabled = signal<boolean>(this.loadEnabled());

  constructor() {
    effect(() => {
      const color = this.sourceColor();
      const isEnabled = this.enabled();
      this.persistPreferences(color, isEnabled);
      if (isEnabled) {
        this.applyDynamicTheme(color);
      } else {
        this.clearDynamicTheme();
      }
    });
  }

  setSourceColor(hex: string): void {
    if (/^#[0-9a-fA-F]{6}$/.test(hex)) {
      this.sourceColor.set(hex);
    }
  }

  setEnabled(value: boolean): void {
    this.enabled.set(value);
  }

  private applyDynamicTheme(sourceHex: string): void {
    try {
      const htmlEl = document.documentElement;
      const currentTheme = htmlEl.dataset['theme'] || 'dark';
      if (currentTheme === 'hc') {
        this.clearDynamicTheme();
        return;
      }

      const argb = argbFromHex(sourceHex);
      const isDark = currentTheme !== 'light';

      const scheme = schemeFromSourceColor(argb, isDark);

      const tokenMap: [string, number][] = [
        ['--md-sys-color-primary', scheme.primary],
        ['--md-sys-color-on-primary', scheme.onPrimary],
        ['--md-sys-color-primary-container', scheme.primaryContainer],
        ['--md-sys-color-on-primary-container', scheme.onPrimaryContainer],
        ['--md-sys-color-secondary', scheme.secondary],
        ['--md-sys-color-on-secondary', scheme.onSecondary],
        ['--md-sys-color-secondary-container', scheme.secondaryContainer],
        ['--md-sys-color-on-secondary-container', scheme.onSecondaryContainer],
        ['--md-sys-color-tertiary', scheme.tertiary],
        ['--md-sys-color-on-tertiary', scheme.onTertiary],
        ['--md-sys-color-tertiary-container', scheme.tertiaryContainer],
        ['--md-sys-color-on-tertiary-container', scheme.onTertiaryContainer],
        ['--md-sys-color-error', scheme.error],
        ['--md-sys-color-on-error', scheme.onError],
        ['--md-sys-color-error-container', scheme.errorContainer],
        ['--md-sys-color-on-error-container', scheme.onErrorContainer],
        ['--md-sys-color-surface', scheme.surface],
        ['--md-sys-color-on-surface', scheme.onSurface],
        ['--md-sys-color-on-surface-variant', scheme.onSurfaceVariant],
        ['--md-sys-color-outline', scheme.outline],
        ['--md-sys-color-outline-variant', scheme.outlineVariant],
        ['--md-sys-color-surface-container-lowest', scheme.surfaceContainerLowest],
        ['--md-sys-color-surface-container-low', scheme.surfaceContainerLow],
        ['--md-sys-color-surface-container', scheme.surfaceContainer],
        ['--md-sys-color-surface-container-high', scheme.surfaceContainerHigh],
        ['--md-sys-color-surface-container-highest', scheme.surfaceContainerHighest],
        ['--md-sys-color-inverse-surface', scheme.inverseSurface],
        ['--md-sys-color-inverse-on-surface', scheme.inverseOnSurface],
        ['--md-sys-color-inverse-primary', scheme.inversePrimary],
      ];

      for (const [property, argbValue] of tokenMap) {
        htmlEl.style.setProperty(property, hexFromArgb(argbValue));
      }
      htmlEl.dataset['dynamicColor'] = '1';
      this.updateNativeStatusBar();
    } catch (e) {
      console.warn('[DynamicColor] Failed to apply theme:', e);
    }
  }

  private clearDynamicTheme(): void {
    const htmlEl = document.documentElement;
    const tokens = [
      '--md-sys-color-primary', '--md-sys-color-on-primary',
      '--md-sys-color-primary-container', '--md-sys-color-on-primary-container',
      '--md-sys-color-secondary', '--md-sys-color-on-secondary',
      '--md-sys-color-secondary-container', '--md-sys-color-on-secondary-container',
      '--md-sys-color-tertiary', '--md-sys-color-on-tertiary',
      '--md-sys-color-tertiary-container', '--md-sys-color-on-tertiary-container',
      '--md-sys-color-error', '--md-sys-color-on-error',
      '--md-sys-color-error-container', '--md-sys-color-on-error-container',
      '--md-sys-color-surface', '--md-sys-color-on-surface',
      '--md-sys-color-on-surface-variant',
      '--md-sys-color-outline', '--md-sys-color-outline-variant',
      '--md-sys-color-surface-container-lowest', '--md-sys-color-surface-container-low',
      '--md-sys-color-surface-container', '--md-sys-color-surface-container-high',
      '--md-sys-color-surface-container-highest',
      '--md-sys-color-inverse-surface', '--md-sys-color-inverse-on-surface',
      '--md-sys-color-inverse-primary',
    ];
    for (const token of tokens) {
      htmlEl.style.removeProperty(token);
    }
    delete htmlEl.dataset['dynamicColor'];
    this.updateNativeStatusBar();
  }

  updateNativeStatusBar(): void {
    try {
      const cap = (window as any).Capacitor;
      if (cap?.isNativePlatform?.()) {
        const statusBar = cap?.Plugins?.StatusBar;
        if (statusBar) {
          const htmlEl = document.documentElement;
          const currentTheme = htmlEl.dataset['theme'] || 'dark';
          
          let bgColor = '#f8f9ff';
          let style = 'LIGHT'; // 'LIGHT' means dark icons on light theme

          if (currentTheme === 'light') {
            style = 'LIGHT'; // Use dark icons on light themes
          } else {
            style = 'DARK'; // Use light icons on dark/night/hc themes
          }

          if (currentTheme === 'dark') {
            bgColor = '#0f1318';
          } else if (currentTheme === 'night' || currentTheme === 'hc') {
            bgColor = '#000000';
          }

          // If dynamic color is active, read the current computed property --md-sys-color-surface
          if (htmlEl.dataset['dynamicColor'] === '1') {
            const computedBg = getComputedStyle(htmlEl).getPropertyValue('--md-sys-color-surface').trim();
            if (computedBg && (computedBg.startsWith('#') || computedBg.startsWith('rgb'))) {
              bgColor = computedBg;
            }
          }

          void statusBar.setBackgroundColor?.({ color: bgColor })?.catch(() => {});
          void statusBar.setStyle?.({ style })?.catch(() => {});
        }
      }
    } catch (_) {}
  }

  reapply(): void {
    if (this.enabled()) {
      this.applyDynamicTheme(this.sourceColor());
    }
  }

  private loadSourceColor(): string {
    try {
      const stored = localStorage.getItem(STORAGE_KEY);
      if (stored) return JSON.parse(stored).sourceColor || DEFAULT_SOURCE;
    } catch { /* ignore */ }
    return DEFAULT_SOURCE;
  }

  private loadEnabled(): boolean {
    try {
      const stored = localStorage.getItem(STORAGE_KEY);
      if (stored) return JSON.parse(stored).enabled ?? false;
    } catch { /* ignore */ }
    return false;
  }

  private persistPreferences(sourceColor: string, enabled: boolean): void {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify({ sourceColor, enabled }));
    } catch { /* quota exceeded or private browsing */ }
  }
}
