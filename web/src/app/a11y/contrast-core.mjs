/** Shared WCAG contrast implementation used by browser tests and Node tooling. */
export const WCAG_AA_NORMAL = 4.5;
export const WCAG_AA_LARGE = 3;

/** Parse #rgb, #rgba, #rrggbb, or #rrggbbaa into RGBA channels. */
export function parseHex(hex) {
  const value = String(hex).trim().replace(/^#/, '');
  if (![3, 4, 6, 8].includes(value.length) || /[^0-9a-f]/i.test(value)) {
    throw new Error(`Invalid hex color: ${hex}`);
  }

  const expanded = value.length <= 4
    ? value.split('').map((channel) => channel + channel).join('')
    : value;
  const hasAlpha = expanded.length === 8;

  return {
    r: Number.parseInt(expanded.slice(0, 2), 16),
    g: Number.parseInt(expanded.slice(2, 4), 16),
    b: Number.parseInt(expanded.slice(4, 6), 16),
    a: hasAlpha ? Number.parseInt(expanded.slice(6, 8), 16) / 255 : 1,
  };
}

function composite(foreground, background) {
  const alpha = foreground.a + background.a * (1 - foreground.a);
  if (alpha === 0) return { r: 0, g: 0, b: 0, a: 0 };

  return {
    r: (foreground.r * foreground.a + background.r * background.a * (1 - foreground.a)) / alpha,
    g: (foreground.g * foreground.a + background.g * background.a * (1 - foreground.a)) / alpha,
    b: (foreground.b * foreground.a + background.b * background.a * (1 - foreground.a)) / alpha,
    a: alpha,
  };
}

function luminanceFromRgb({ r, g, b }) {
  const channel = (raw) => {
    const value = raw / 255;
    return value <= 0.03928
      ? value / 12.92
      : ((value + 0.055) / 1.055) ** 2.4;
  };

  return 0.2126 * channel(r) + 0.7152 * channel(g) + 0.0722 * channel(b);
}

/** Relative luminance requires an opaque color because transparency needs a backdrop. */
export function relativeLuminance(hex) {
  const color = parseHex(hex);
  if (color.a !== 1) {
    throw new Error(`Relative luminance requires an opaque color: ${hex}`);
  }
  return luminanceFromRgb(color);
}

/**
 * Contrast ratio with alpha-aware compositing. Transparent backgrounds are
 * composited over the optional opaque canvas (white by default).
 */
export function contrastRatio(foreground, background, canvas = '#ffffff') {
  const canvasColor = parseHex(canvas);
  if (canvasColor.a !== 1) {
    throw new Error(`Contrast canvas must be opaque: ${canvas}`);
  }

  const resolvedBackground = composite(parseHex(background), canvasColor);
  const resolvedForeground = composite(parseHex(foreground), resolvedBackground);
  const first = luminanceFromRgb(resolvedForeground);
  const second = luminanceFromRgb(resolvedBackground);
  const lighter = Math.max(first, second);
  const darker = Math.min(first, second);
  return (lighter + 0.05) / (darker + 0.05);
}
