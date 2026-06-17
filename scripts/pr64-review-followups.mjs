import { readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const root = path.resolve(__dirname, '..');

const file = (...parts) => path.join(root, ...parts);

async function replaceExact(relativePath, replacements) {
  const target = file(...relativePath.split('/'));
  let source = await readFile(target, 'utf8');

  for (const [before, after] of replacements) {
    if (!source.includes(before)) {
      if (source.includes(after)) continue;
      throw new Error(`${relativePath}: expected text not found:\n${before}`);
    }
    source = source.replace(before, after);
  }

  await writeFile(target, source, 'utf8');
}

await writeFile(file('web', 'src', 'app', 'a11y', 'contrast-core.mjs'), `/** Shared WCAG contrast implementation used by browser tests and Node tooling. */
export const WCAG_AA_NORMAL = 4.5;
export const WCAG_AA_LARGE = 3;

/** Parse #rgb, #rgba, #rrggbb, or #rrggbbaa into RGBA channels. */
export function parseHex(hex) {
  const value = String(hex).trim().replace(/^#/, '');
  if (![3, 4, 6, 8].includes(value.length) || /[^0-9a-f]/i.test(value)) {
    throw new Error(\`Invalid hex color: \${hex}\`);
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
`, 'utf8');

await writeFile(file('web', 'src', 'app', 'a11y', 'contrast-core.d.mts'), `export interface RgbaColor {
  r: number
  g: number
  b: number
  a: number
}

export const WCAG_AA_NORMAL: 4.5
export const WCAG_AA_LARGE: 3
export function parseHex(hex: string): RgbaColor
export function relativeLuminance(hex: string): number
export function contrastRatio(
  foreground: string,
  background: string,
  canvas?: string,
): number
`, 'utf8');

await writeFile(file('web', 'src', 'app', 'a11y', 'contrast.ts'), `/**
 * Typed facade for the shared WCAG implementation consumed by Node tooling and
 * the Angular/Vitest suite. Keep the formula in contrast-core.mjs only.
 */
export {
  WCAG_AA_LARGE,
  WCAG_AA_NORMAL,
  contrastRatio,
  parseHex,
  relativeLuminance,
} from "./contrast-core.mjs"
export type { RgbaColor } from "./contrast-core.mjs"
`, 'utf8');

await writeFile(file('web', 'src', 'app', 'a11y', 'contrast.spec.ts'), `import { describe, expect, it } from "vitest"
import {
  WCAG_AA_LARGE,
  WCAG_AA_NORMAL,
  contrastRatio,
  parseHex,
  relativeLuminance,
} from "./contrast"

describe("contrast helpers", () => {
  it("exposes the WCAG AA thresholds", () => {
    expect(WCAG_AA_NORMAL).toBe(4.5)
    expect(WCAG_AA_LARGE).toBe(3)
  })

  it("parses long, shorthand, and alpha hex colors", () => {
    expect(parseHex("#ffffff")).toEqual({ r: 255, g: 255, b: 255, a: 1 })
    expect(parseHex("#0af")).toEqual({ r: 0, g: 170, b: 255, a: 1 })
    expect(parseHex("#0af8")).toEqual({
      r: 0,
      g: 170,
      b: 255,
      a: 136 / 255,
    })
    expect(parseHex("#11223380")).toEqual({
      r: 17,
      g: 34,
      b: 51,
      a: 128 / 255,
    })
  })

  it("rejects invalid colors", () => {
    expect(() => parseHex("#fffff")).toThrow("Invalid hex color")
    expect(() => parseHex("red")).toThrow("Invalid hex color")
  })

  it("requires an opaque color for standalone luminance", () => {
    expect(() => relativeLuminance("#0008")).toThrow(
      "Relative luminance requires an opaque color",
    )
  })

  it("calculates the luminance endpoints", () => {
    expect(relativeLuminance("#000000")).toBe(0)
    expect(relativeLuminance("#ffffff")).toBeCloseTo(1, 8)
  })

  it("calculates black on white as approximately 21:1", () => {
    expect(contrastRatio("#000000", "#ffffff")).toBeCloseTo(21, 8)
  })

  it("composites alpha-channel foreground colors over the background", () => {
    const translucent = contrastRatio("#00000080", "#ffffff")
    expect(translucent).toBeGreaterThan(3.9)
    expect(translucent).toBeLessThan(4.1)
  })

  it("is symmetric for opaque foreground/background colors", () => {
    expect(contrastRatio("#556578", "#dce7f4")).toBeCloseTo(
      contrastRatio("#dce7f4", "#556578"),
      10,
    )
  })
})
`, 'utf8');

await writeFile(file('scripts', 'check-contrast-tokens.mjs'), `import { readFile } from 'node:fs/promises';
import path from 'node:path';
import url from 'node:url';
import {
  WCAG_AA_NORMAL,
  contrastRatio,
} from '../web/src/app/a11y/contrast-core.mjs';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const BASE_CSS_PATH = path.join(ROOT, 'web', 'src', 'styles', 'rvt-v12-mobile-pwa-css.css');
const REDESIGN_CSS_PATH = path.join(ROOT, 'web', 'src', 'styles', 'rvt-redesign-tokens.css');

function parseDeclarations(css, selector) {
  const declarations = new Map();
  const withoutComments = css.replace(/\/\*[\s\S]*?\*\//g, '');
  const rulePattern = /([^{}]+)\{([^{}]*)\}/g;
  let rule;

  while ((rule = rulePattern.exec(withoutComments)) !== null) {
    const selectors = rule[1]
      .split(',')
      .map((candidate) => candidate.trim())
      .filter(Boolean);
    if (!selectors.includes(selector)) continue;

    const declarationPattern = /(--[\w-]+)\s*:\s*([^;]+);/g;
    let declaration;
    while ((declaration = declarationPattern.exec(rule[2])) !== null) {
      declarations.set(
        declaration[1],
        declaration[2].replace(/\s*!important\s*$/, '').trim(),
      );
    }
  }

  if (declarations.size === 0) {
    throw new Error(`Theme selector not found or empty: ${selector}`);
  }
  return declarations;
}

function effectiveTokens(layers) {
  const tokens = new Map();
  for (const { css, selector } of layers) {
    for (const [name, value] of parseDeclarations(css, selector)) {
      tokens.set(name, value);
    }
  }
  return tokens;
}

function resolveToken(tokens, name, stack = []) {
  if (stack.includes(name)) {
    throw new Error(`Circular CSS variable reference: ${[...stack, name].join(' -> ')}`);
  }

  const value = tokens.get(name);
  if (!value) throw new Error(`Token not found: ${name}`);
  if (/^#[0-9a-f]{3,8}$/i.test(value)) return value;

  const variable = value.match(/^var\(\s*(--[\w-]+)\s*(?:,\s*([^)]+))?\)$/);
  if (!variable) {
    throw new Error(`Token ${name} does not resolve to a supported color: ${value}`);
  }

  const [, reference, fallback] = variable;
  if (tokens.has(reference)) {
    return resolveToken(tokens, reference, [...stack, name]);
  }
  if (fallback && /^#[0-9a-f]{3,8}$/i.test(fallback.trim())) {
    return fallback.trim();
  }
  throw new Error(`Token ${name} references missing ${reference} without a hex fallback`);
}

const baseCss = await readFile(BASE_CSS_PATH, 'utf8');
const redesignCss = await readFile(REDESIGN_CSS_PATH, 'utf8');

const commonSurfaceTokens = [
  '--md-sys-color-surface',
  '--md-sys-color-surface-container-lowest',
  '--md-sys-color-surface-container-low',
  '--md-sys-color-surface-container',
  '--md-sys-color-surface-container-high',
  '--md-sys-color-surface-container-highest',
  '--md-sys-color-secondary-container',
];

const cases = [
  ...['azure', 'bloom', 'mint'].flatMap((palette) =>
    ['light', 'dark'].map((theme) => ({
      name: `${palette} / ${theme}`,
      layers: [{
        css: redesignCss,
        selector: `html[data-theme="${theme}"][data-palette="${palette}"]`,
      }],
      backgrounds: commonSurfaceTokens,
    })),
  ),
  {
    name: 'night',
    layers: [
      { css: baseCss, selector: 'html[data-theme="night"]' },
      { css: redesignCss, selector: 'html[data-theme="night"]' },
    ],
    backgrounds: [
      '--md-sys-color-surface',
      '--md-sys-color-surface-container-low',
      '--md-sys-color-surface-container',
      '--md-sys-color-surface-container-high',
      '--md-sys-color-primary-container',
    ],
  },
  {
    name: 'high contrast',
    layers: [
      { css: baseCss, selector: 'html[data-theme="hc"]' },
      { css: redesignCss, selector: 'html[data-theme="hc"]' },
    ],
    backgrounds: [
      '--md-sys-color-surface',
      '--md-sys-color-surface-container-lowest',
      '--md-sys-color-surface-container-low',
      '--md-sys-color-surface-container',
      '--md-sys-color-surface-container-high',
      '--md-sys-color-primary-container',
    ],
  },
];

let failed = false;
for (const testCase of cases) {
  const tokens = effectiveTokens(testCase.layers);
  for (const textToken of ['--rv-dim', '--rv-muted']) {
    const foreground = resolveToken(tokens, textToken);
    const results = testCase.backgrounds.map((backgroundToken) => {
      const background = resolveToken(tokens, backgroundToken);
      return {
        backgroundToken,
        background,
        ratio: contrastRatio(foreground, background),
      };
    });
    const worst = results.reduce((current, candidate) =>
      candidate.ratio < current.ratio ? candidate : current,
    );
    const failures = results.filter(({ ratio }) => ratio < WCAG_AA_NORMAL);

    if (failures.length > 0) {
      failed = true;
      for (const failure of failures) {
        console.error(
          `FAIL ${testCase.name} ${textToken}: ${foreground} on ` +
          `${failure.backgroundToken} ${failure.background} = ${failure.ratio.toFixed(2)}:1`,
        );
      }
    } else {
      console.log(
        `PASS ${testCase.name} ${textToken}: worst ${foreground} on ` +
        `${worst.backgroundToken} ${worst.background} = ${worst.ratio.toFixed(2)}:1`,
      );
    }
  }
}

if (failed) {
  process.exitCode = 1;
} else {
  console.log('All low-emphasis text tokens clear WCAG AA on every checked surface.');
}
`, 'utf8');

await writeFile(file('scripts', 'check-source-integrity.mjs'), `import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import url from 'node:url';

const execFileAsync = promisify(execFile);
const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const TEXT_EXTENSIONS = new Set([
  '.css', '.gradle', '.html', '.java', '.js', '.json', '.md', '.mjs', '.mts',
  '.py', '.rs', '.scss', '.sh', '.toml', '.ts', '.txt', '.xml', '.yaml', '.yml',
]);

const { stdout } = await execFileAsync(
  'git',
  ['ls-files', '--cached', '--others', '--exclude-standard', '-z'],
  { cwd: ROOT, encoding: 'utf8', maxBuffer: 16 * 1024 * 1024 },
);
const trackedTextFiles = stdout
  .split('\0')
  .filter(Boolean)
  .filter((relativePath) => TEXT_EXTENSIONS.has(path.extname(relativePath).toLowerCase()));

const contents = new Map();
for (const relativePath of trackedTextFiles) {
  const content = await readFile(path.join(ROOT, relativePath), 'utf8');
  contents.set(relativePath, content);
  const placeholder = content.match(/\btoolu_[A-Za-z0-9]+\b/);
  if (placeholder) {
    throw new Error(`${relativePath} contains an accidental tool placeholder: ${placeholder[0]}`);
  }
}

const requiredTemplateBindings = {
  'web/src/app/components/home/home.component.html': [
    /\{\{\s*getReadinessPercentage\(\)\s*\}\}/,
    /\{\{\s*port\s*\}\}/,
    /\{\{\s*check\.label\s*\}\}/,
    /\{\{[\s\S]*?state\.lastPayload\(\)\?\.radar\?\.reported_hr[\s\S]*?\}\}/,
    /\{\{\s*session\.started_at\s*\|\s*date\s*:\s*['"]shortTime['"]\s*\}\}/,
  ],
  'web/src/app/components/topbar/topbar.component.html': [
    /\{\{\s*getBreadcrumbTitle\(\)\s*\}\}/,
    /\{\{\s*serverLifecycle\.statusLabel\(\)\s*\}\}/,
    /auto-retry\s+in\s+\{\{\s*seconds\s*\}\}\s*s/,
    /\{\{\s*state\.setup\(\)\.operator_label\s*\|\|\s*['"]Operator A['"]\s*\}\}/,
    /\{\{\s*state\.paused\(\)\s*\?\s*['"]play_arrow['"]\s*:\s*['"]pause['"]\s*\}\}/,
  ],
};

for (const [relativePath, patterns] of Object.entries(requiredTemplateBindings)) {
  const content = contents.get(relativePath) ?? await readFile(path.join(ROOT, relativePath), 'utf8');
  for (const pattern of patterns) {
    if (!pattern.test(content)) {
      throw new Error(`${relativePath} is missing critical Angular binding matching ${pattern}`);
    }
  }
}

console.log(
  `Source integrity checks passed across ${trackedTextFiles.length} tracked text files; ` +
  'critical Angular bindings are intact.',
);
`, 'utf8');

await writeFile(file('web', 'src', 'app', 'i18n', 'translate.pipe.ts'), `import { Pipe, type PipeTransform, inject } from "@angular/core"
import { I18nService } from "../services/i18n.service"

/**
 * Pure template helper for the runtime i18n layer.
 *
 * Pass the locale signal value as an explicit pipe input so Angular re-runs the
 * pipe only when the key, locale, or interpolation parameters change:
 *
 *   {{ 'nav.home' | translate:i18n.locale() }}
 *   {{ 'home.rerunCheck' | translate:i18n.locale():{ label: check.label } }}
 */
@Pipe({ name: "translate", standalone: true, pure: true })
export class TranslatePipe implements PipeTransform {
  private readonly i18n = inject(I18nService)

  transform(
    key: string,
    locale: string,
    params?: Record<string, string | number>,
  ): string {
    // `locale` is an explicit cache key for Angular's pure-pipe semantics. The
    // service owns the active signal and performs the actual fallback lookup.
    void locale
    return this.i18n.translate(key, params)
  }
}
`, 'utf8');

await replaceExact('web/src/app/services/i18n.messages.en.ts', [
  ['"home.recordingDuration": "Recording duration"', '"home.recordingDuration": "Recording Duration"'],
  ['"home.radarPort": "Radar serial port"', '"home.radarPort": "Radar Port"'],
  ['"home.bleAddress": "BLE address"', '"home.bleAddress": "BLE Oximeter Address"'],
]);

await replaceExact('web/src/app/components/home/home.component.ts', [
  [
    "import { ServerLifecycleService } from '../../services/server-lifecycle.service';",
    "import { ServerLifecycleService } from '../../services/server-lifecycle.service';\nimport { I18nService } from '../../services/i18n.service';\nimport { TranslatePipe } from '../../i18n/translate.pipe';",
  ],
  [
    "    MatChipsModule,\n    MatSnackBarModule\n",
    "    MatChipsModule,\n    MatSnackBarModule,\n    TranslatePipe\n",
  ],
  [
    "  protected readonly serverLifecycle = inject(ServerLifecycleService);",
    "  protected readonly serverLifecycle = inject(ServerLifecycleService);\n  protected readonly i18n = inject(I18nService);",
  ],
]);

await replaceExact('web/src/app/components/home/home.component.html', [
  [
    '<label class="field-label" id="recDurationLabel">Recording Duration <span class="label-hint">how long to record</span></label>',
    '<label class="field-label" id="recDurationLabel">{{ \'home.recordingDuration\' | translate:i18n.locale() }} <span class="label-hint">how long to record</span></label>',
  ],
  [
    '<label class="field-label" id="radarPortLabel">Radar Port</label>',
    '<label class="field-label" id="radarPortLabel">{{ \'home.radarPort\' | translate:i18n.locale() }}</label>',
  ],
  [
    '<label class="field-label" id="bleAddressLabel">BLE Oximeter Address</label>',
    '<label class="field-label" id="bleAddressLabel">{{ \'home.bleAddress\' | translate:i18n.locale() }}</label>',
  ],
  [
    'aria-label="Scan for radar serial ports"',
    '[attr.aria-label]="\'home.scanPorts\' | translate:i18n.locale()"',
  ],
  [
    '[attr.aria-label]="\'Re-run check: \' + check.label"',
    '[attr.aria-label]="\'home.rerunCheck\' | translate:i18n.locale():{ label: check.label }"',
  ],
]);

await replaceExact('web/src/app/components/topbar/topbar.component.ts', [
  [
    "import { SwitchOperatorDialogComponent } from '../switch-operator-dialog/switch-operator-dialog.component';",
    "import { SwitchOperatorDialogComponent } from '../switch-operator-dialog/switch-operator-dialog.component';\nimport { I18nService } from '../../services/i18n.service';\nimport { TranslatePipe } from '../../i18n/translate.pipe';",
  ],
  [
    "    MatMenuModule,\n    MatToolbarModule\n",
    "    MatMenuModule,\n    MatToolbarModule,\n    TranslatePipe\n",
  ],
  [
    "  protected readonly auth = inject(AuthService);",
    "  protected readonly auth = inject(AuthService);\n  protected readonly i18n = inject(I18nService);",
  ],
]);

await replaceExact('web/src/app/components/topbar/topbar.component.html', [
  ['aria-label="Simple view"', '[attr.aria-label]="\'topbar.simpleView\' | translate:i18n.locale()"'],
  ['aria-label="Advanced view"', '[attr.aria-label]="\'topbar.advancedView\' | translate:i18n.locale()"'],
  ['aria-label="Comfortable density"', '[attr.aria-label]="\'topbar.comfortableDensity\' | translate:i18n.locale()"'],
  ['aria-label="Compact density"', '[attr.aria-label]="\'topbar.compactDensity\' | translate:i18n.locale()"'],
  ['aria-label="Pause or resume live polling"', '[attr.aria-label]="\'topbar.togglePolling\' | translate:i18n.locale()"'],
  ['aria-label="Alerts"', '[attr.aria-label]="\'topbar.alerts\' | translate:i18n.locale()"'],
  ['aria-label="Settings"', '[attr.aria-label]="\'topbar.settings\' | translate:i18n.locale()"'],
  ['aria-label="Cycle theme"', '[attr.aria-label]="\'topbar.cycleTheme\' | translate:i18n.locale()"'],
]);

await replaceExact('web/src/styles/rvt-redesign-tokens.css', [
  [
    '   - `--rv-dim` is the shared low-emphasis text token consumed by muted,\n      secondary, placeholder, metadata, timestamp, helper-text, table-caption,\n      nav-label and chart-label styles. Its previous values fell below the WCAG\n      2.1 AA 4.5:1 minimum for body text, so they were corrected here at the\n      source (see web/src/app/a11y/contrast.spec.ts for the locked ratios).',
    '   - `--rv-dim` and `--rv-muted` are the shared low-emphasis text tokens\n      consumed by secondary, placeholder, metadata, timestamp, helper-text,\n      table-caption, nav-label and chart-label styles. Their contrast is checked\n      against every supported neutral surface plus secondary-container surfaces\n      in scripts/check-contrast-tokens.mjs.',
  ],
  [
    '--rv-ink: #e0e8f2; --rv-ink-2: #d3e4ff; --rv-muted: #93a4b8; --rv-dim: #9fb0c4;',
    '--rv-ink: #e0e8f2; --rv-ink-2: #d3e4ff; --rv-muted: #94a5b9; --rv-dim: #9fb0c4;',
  ],
  [
    '  --rv-muted: var(--md-sys-color-on-surface-variant, #c3cad6);\n  --rv-dim: var(--md-sys-color-on-surface-variant, #c3cad6);',
    '  /* These narrow !important declarations guarantee the fixed Night theme\n     wins even if a stale data-palette attribute is also present. */\n  --rv-muted: var(--md-sys-color-on-surface-variant, #c3cad6) !important;\n  --rv-dim: var(--md-sys-color-on-surface-variant, #c3cad6) !important;',
  ],
  [
    '  --rv-muted: var(--md-sys-color-on-surface, #ffffff);\n  --rv-dim: var(--md-sys-color-on-surface, #ffffff);',
    '  /* High Contrast must defeat any simultaneous palette selector. */\n  --rv-muted: var(--md-sys-color-on-surface, #ffffff) !important;\n  --rv-dim: var(--md-sys-color-on-surface, #ffffff) !important;',
  ],
  [
    '/* Guarantee a >=44x44 CSS-px hit area for icon-only and segmented controls\n   without changing the rendered glyph size. Disabled controls keep the same\n   geometry (only pointer/visual state differs). */',
    '/* Guarantee a >=44x44 CSS-px host hit area for icon-only and segmented\n   controls without changing glyph size. Material icon buttons normally provide\n   a larger overlay already; the explicit host minimum keeps the target intact\n   when component overrides or clipping alter that overlay. */',
  ],
]);

await replaceExact('web/src/app/navigation-bar/navigation-tab.component.ts', [
  [
    '      /* Translation-safe: wrap long localized labels (up to two lines) instead\n         of clipping with an ellipsis, which previously hid non-English text. */',
    '      /* Translation-safe: allow a second line before deliberately truncating\n         exceptionally long labels, instead of the previous single-line clip. */',
  ],
]);

await replaceExact('tests/visual/rvt-v12.spec.ts', [
  [
    "        const stabilizeHighContrastDesktop = theme === 'hc' && testInfo.project.name === 'desktop';\n",
    "        const stabilizeHighContrastDesktop = theme === 'hc' && testInfo.project.name === 'desktop';\n        const screenshotTolerance: { maxDiffPixels?: number } = {};\n        if (stabilizeTabletHome) {\n          screenshotTolerance.maxDiffPixels = 20000;\n        } else if (stabilizeHighContrastDesktop) {\n          screenshotTolerance.maxDiffPixels = 1000;\n        }\n",
  ],
  [
    "          ...(stabilizeTabletHome\n            ? { maxDiffPixels: 20000 }\n            : stabilizeHighContrastDesktop\n              ? { maxDiffPixels: 1000 }\n              : {})\n",
    "          ...screenshotTolerance\n",
  ],
]);

await replaceExact('docs/i18n.md', [
  [
    "<h2>{{ 'home.systemChecks' | translate }}</h2>",
    "<h2>{{ 'home.systemChecks' | translate:i18n.locale() }}</h2>",
  ],
  [
    "  [attr.aria-label]=\"'home.rerunCheck' | translate:{ label: check.label }\">",
    "  [attr.aria-label]=\"'home.rerunCheck' | translate:i18n.locale():{ label: check.label }\">",
  ],
  [
    'The `{{ ... }}` delimiters are required for text-node interpolation. Attribute bindings already provide an Angular expression context:',
    'The `{{ ... }}` delimiters are required for text-node interpolation. Pass `i18n.locale()` as the explicit pure-pipe cache key so locale changes remain reactive without evaluating every translation on every change-detection cycle. Attribute bindings already provide an Angular expression context:',
  ],
  [
    'private readonly i18n = inject(I18nService)',
    'protected readonly i18n = inject(I18nService)',
  ],
  [
    '- `scripts/check-contrast-tokens.mjs`, which reads the real theme CSS and checks every light/dark palette plus Night and High Contrast.',
    '- `scripts/check-contrast-tokens.mjs`, which resolves the real base + redesign CSS variables and checks both low-emphasis tokens across every supported neutral surface and colored secondary-container surface, including Night and High Contrast.',
  ],
]);

console.log('Applied PR #64 review follow-ups.');
