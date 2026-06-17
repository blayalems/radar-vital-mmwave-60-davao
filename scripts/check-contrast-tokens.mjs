import { readFile } from 'node:fs/promises';
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
const OVERRIDE_CSS_PATH = path.join(ROOT, 'web', 'src', 'styles', 'rvt-accessibility-overrides.css');

function parseDeclarations(css, selector, required = true) {
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

    // Map#set intentionally makes the last declaration win, matching CSS.
    const declarationPattern = /(--[\w-]+)\s*:\s*([^;]+);/g;
    let declaration;
    while ((declaration = declarationPattern.exec(rule[2])) !== null) {
      declarations.set(
        declaration[1],
        declaration[2].replace(/\s*!important\s*$/, '').trim(),
      );
    }
  }

  if (required && declarations.size === 0) {
    throw new Error(`Theme selector not found or empty: ${selector}`);
  }
  return declarations;
}

function effectiveTokens(layers) {
  const tokens = new Map();
  for (const { css, selector, required = true } of layers) {
    for (const [name, value] of parseDeclarations(css, selector, required)) {
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
const overrideCss = await readFile(OVERRIDE_CSS_PATH, 'utf8');

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
      layers: [
        {
          css: redesignCss,
          selector: `html[data-theme="${theme}"][data-palette="${palette}"]`,
        },
        {
          css: overrideCss,
          selector: `html[data-theme="${theme}"][data-palette="${palette}"]`,
          required: false,
        },
      ],
      backgrounds: commonSurfaceTokens,
    })),
  ),
  {
    name: 'night',
    layers: [
      { css: baseCss, selector: 'html[data-theme="night"]' },
      { css: redesignCss, selector: 'html[data-theme="night"]' },
      { css: overrideCss, selector: 'html[data-theme="night"]' },
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
      { css: overrideCss, selector: 'html[data-theme="hc"]' },
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
