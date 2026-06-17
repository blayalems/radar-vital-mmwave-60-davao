import { readFile } from 'node:fs/promises';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const CSS_PATH = path.join(ROOT, 'web', 'src', 'styles', 'rvt-redesign-tokens.css');
const WCAG_AA_NORMAL = 4.5;

function escapeRegExp(value) {
  return value.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
}

function declaration(css, selector, token) {
  const block = css.match(
    new RegExp(`${escapeRegExp(selector)}\\s*\\{([\\s\\S]*?)\\}`),
  )?.[1];

  if (!block) {
    throw new Error(`Theme selector not found: ${selector}`);
  }

  const value = block.match(
    new RegExp(`${escapeRegExp(token)}\\s*:\\s*([^;]+);`),
  )?.[1]?.trim();

  if (!value) {
    throw new Error(`Token ${token} not found in ${selector}`);
  }

  return value;
}

function parseHex(hex) {
  const value = hex.trim().replace(/^#/, '');
  const full = value.length === 3
    ? value.split('').map((channel) => channel + channel).join('')
    : value;

  if (full.length !== 6 || /[^0-9a-f]/i.test(full)) {
    throw new Error(`Expected a six-digit hex color, received: ${hex}`);
  }

  return [
    Number.parseInt(full.slice(0, 2), 16),
    Number.parseInt(full.slice(2, 4), 16),
    Number.parseInt(full.slice(4, 6), 16),
  ];
}

function relativeLuminance(hex) {
  const channels = parseHex(hex).map((raw) => {
    const channel = raw / 255;
    return channel <= 0.03928
      ? channel / 12.92
      : ((channel + 0.055) / 1.055) ** 2.4;
  });

  return 0.2126 * channels[0] + 0.7152 * channels[1] + 0.0722 * channels[2];
}

function contrastRatio(foreground, background) {
  const first = relativeLuminance(foreground);
  const second = relativeLuminance(background);
  const lighter = Math.max(first, second);
  const darker = Math.min(first, second);
  return (lighter + 0.05) / (darker + 0.05);
}

const cases = [
  ['azure / light', 'html[data-theme="light"][data-palette="azure"]'],
  ['azure / dark', 'html[data-theme="dark"][data-palette="azure"]'],
  ['bloom / light', 'html[data-theme="light"][data-palette="bloom"]'],
  ['bloom / dark', 'html[data-theme="dark"][data-palette="bloom"]'],
  ['mint / light', 'html[data-theme="light"][data-palette="mint"]'],
  ['mint / dark', 'html[data-theme="dark"][data-palette="mint"]'],
];

const css = await readFile(CSS_PATH, 'utf8');
let failed = false;

for (const [name, selector] of cases) {
  const dim = declaration(css, selector, '--rv-dim');
  const surface = declaration(css, selector, '--md-sys-color-surface-container-highest');
  const ratio = contrastRatio(dim, surface);
  const passed = ratio >= WCAG_AA_NORMAL;
  console.log(`${passed ? 'PASS' : 'FAIL'} ${name}: ${dim} on ${surface} = ${ratio.toFixed(2)}:1`);
  failed ||= !passed;
}

const night = declaration(css, 'html[data-theme="night"]', '--rv-dim');
if (night !== 'var(--md-sys-color-on-surface-variant, #c3cad6)') {
  console.error(`FAIL night --rv-dim: ${night}`);
  failed = true;
} else {
  console.log('PASS night --rv-dim uses on-surface-variant');
}

const highContrast = declaration(css, 'html[data-theme="hc"]', '--rv-dim');
if (highContrast !== 'var(--md-sys-color-on-surface, #ffffff)') {
  console.error(`FAIL high-contrast --rv-dim: ${highContrast}`);
  failed = true;
} else {
  console.log('PASS high-contrast --rv-dim uses full-strength on-surface');
}

if (failed) {
  process.exitCode = 1;
} else {
  console.log('All low-emphasis text contrast checks passed.');
}
