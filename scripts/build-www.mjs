// Build pipeline for the dashboard:
//   1. Resolve BUILD:INCLUDE markers in web/index.html
//      → emit  radar_vital_live_dashboard_v12_for_v16_0.html  (repo root,
//        still a build artifact so file:// and trainer-served flows keep
//        working).
//   2. Copy the assembled monolith and assets/ into www/ (consumed by
//      Capacitor, Tauri, and GitHub Pages). Also copy root aliases for
//      /fonts, /icons, and /lib because the dashboard and service worker
//      intentionally request those absolute paths.
//
// Source of truth is web/. Do not edit the root .html or www/* directly.

import { promises as fs } from 'node:fs';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const WEB = path.join(ROOT, 'web');
const SHELL = path.join(WEB, 'index.html');
const DASHBOARD = path.join(ROOT, 'radar_vital_live_dashboard_v12_for_v16_0.html');
const SRC_ASSETS = path.join(ROOT, 'assets');
const OUT = path.join(ROOT, 'www');

function normalizeNewlines(text) {
  return text.replace(/\r\n?/g, '\n');
}

// Marker capture grabs (1) the leading indent on the marker line so we can
// re-apply it to the closing </style>/</script> tag in the rebuilt monolith.
const INCLUDE_RE = /([ \t]*)<!--\s*BUILD:INCLUDE\s+([^>]*?)-->/g;

function parseAttrs(s) {
  const out = {};
  for (const m of s.matchAll(/(\w+)="([^"]*)"/g)) out[m[1]] = m[2];
  return out;
}

async function assemble() {
  const shell = normalizeNewlines(await fs.readFile(SHELL, 'utf8'));
  const includes = [];
  for (const m of shell.matchAll(INCLUDE_RE)) {
    includes.push({ raw: m[0], indent: m[1] || '', attrs: parseAttrs(m[2]), index: m.index });
  }

  // Resolve all includes in parallel.
  const loaded = await Promise.all(includes.map(async (inc) => {
    const src = inc.attrs.src;
    if (!src) throw new Error(`BUILD:INCLUDE missing src= : ${inc.raw}`);
    const filePath = path.join(WEB, src);
    const body = normalizeNewlines(await fs.readFile(filePath, 'utf8'));
    const trimmed = body.endsWith('\n') ? body.slice(0, -1) : body;
    if (inc.attrs.as === 'html') {
      return trimmed;
    }
    if (inc.attrs.as === 'style') {
      const idAttr = inc.attrs.id ? ` id="${inc.attrs.id}"` : '';
      return `${inc.indent}<style${idAttr}>\n${trimmed}\n${inc.indent}</style>`;
    }
    if (inc.attrs.as === 'script') {
      const idAttr = inc.attrs.id ? ` id="${inc.attrs.id}"` : '';
      return `${inc.indent}<script${idAttr}>\n${trimmed}\n${inc.indent}</script>`;
    }
    throw new Error(`Unknown as= attribute value: "${inc.attrs.as}" in BUILD:INCLUDE`);
  }));

  // Splice from end to start so indices stay valid.
  let out = shell;
  for (let i = includes.length - 1; i >= 0; i--) {
    const inc = includes[i];
    out = out.slice(0, inc.index) + loaded[i] + out.slice(inc.index + inc.raw.length);
  }
  return out;
}

async function copyDir(from, to) {
  await fs.mkdir(to, { recursive: true });
  const entries = await fs.readdir(from, { withFileTypes: true });
  for (const entry of entries) {
    const src = path.join(from, entry.name);
    const dst = path.join(to, entry.name);
    if (entry.isDirectory()) await copyDir(src, dst);
    else if (entry.isFile()) await fs.copyFile(src, dst);
  }
}

async function main() {
  const assembled = await assemble();
  await fs.writeFile(DASHBOARD, assembled);

  await fs.rm(OUT, { recursive: true, force: true });
  await fs.mkdir(OUT, { recursive: true });
  const assembledForWww = assembled.replaceAll('./assets/', './');
  await fs.writeFile(path.join(OUT, 'index.html'), assembledForWww);
  await fs.writeFile(path.join(OUT, path.basename(DASHBOARD)), assembledForWww);

  await copyDir(path.join(SRC_ASSETS, 'fonts'), path.join(OUT, 'fonts'));
  await copyDir(path.join(SRC_ASSETS, 'icons'), path.join(OUT, 'icons'));
  await copyDir(path.join(SRC_ASSETS, 'lib'), path.join(OUT, 'lib'));
  await fs.copyFile(path.join(SRC_ASSETS, 'manifest.webmanifest'), path.join(OUT, 'manifest.webmanifest'));
  await fs.copyFile(path.join(SRC_ASSETS, 'sw.js'), path.join(OUT, 'sw.js'));
  await fs.copyFile(path.join(SRC_ASSETS, 'rvt-sw.js'), path.join(OUT, 'rvt-sw.js'));

  await fs.writeFile(
    path.join(OUT, '404.html'),
    '<!DOCTYPE html><meta http-equiv="refresh" content="0; url=./index.html">'
  );

  console.log(`Built ${path.basename(DASHBOARD)} (${assembled.length.toLocaleString()} chars) and www/.`);
}

main().catch((err) => { console.error(err); process.exit(1); });
