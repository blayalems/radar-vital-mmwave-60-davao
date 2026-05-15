// One-shot extractor: monolithic radar_vital_live_dashboard_v12_for_v16_0.html
// → web/styles/<id>.css, web/modules/<id>.js, web/index.html (shell with markers).
//
// Run once when the monolith changes shape. Day-to-day, edit the split files
// under web/ and run `npm run build:web` (which calls scripts/build-www.mjs)
// to regenerate the monolith and www/.
//
// Contract: open tags `<style id="...">` and `<script id="...">` (and their
// closes) appear on their own line. Anonymous `<script>` blocks (no id) get
// names `_anon-001`, `_anon-002`, … and are extracted just the same so the
// shell stays free of inline blocks.

import { promises as fs } from 'node:fs';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const SRC = path.join(ROOT, 'radar_vital_live_dashboard_v12_for_v16_0.html');
const WEB = path.join(ROOT, 'web');
const STYLES = path.join(WEB, 'styles');
const MODULES = path.join(WEB, 'modules');
const COMPONENTS = path.join(WEB, 'components');

const OPEN_RE = /^(\s*)<(style|script)(?:\s+([^>]*))?>\s*$/;
const closeRe = (tag) => new RegExp(`^\\s*</${tag}>\\s*$`);

async function main() {
  const raw = await fs.readFile(SRC, 'utf8');
  const lines = raw.split('\n');

  await fs.rm(STYLES, { recursive: true, force: true });
  await fs.rm(MODULES, { recursive: true, force: true });
  await fs.mkdir(STYLES, { recursive: true });
  await fs.mkdir(MODULES, { recursive: true });
  await fs.mkdir(COMPONENTS, { recursive: true });

  let mode = 'normal';
  let blockId = null;
  let blockIdRaw = null;
  let blockIndent = '';
  let blockLines = [];
  let anonCounter = 0;
  const shell = [];
  const manifest = [];
  const idCount = new Map();

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    if (mode === 'normal') {
      const m = OPEN_RE.exec(line);
      if (m) {
        const indent = m[1] ?? '';
        const tag = m[2];
        const attrs = m[3] || '';
        const idMatch = /\bid="([^"]+)"/.exec(attrs);
        blockIdRaw = idMatch ? idMatch[1] : null;
        const baseName = blockIdRaw ?? `_anon-${(++anonCounter).toString().padStart(3, '0')}`;
        const seen = (idCount.get(baseName) ?? 0) + 1;
        idCount.set(baseName, seen);
        blockId = seen === 1 ? baseName : `${baseName}~${seen}`;
        blockIndent = indent;
        mode = tag;
        blockLines = [];
        continue;
      }
      shell.push(line);
    } else {
      if (closeRe(mode).test(line)) {
        const ext = mode === 'style' ? 'css' : 'js';
        const dir = mode === 'style' ? STYLES : MODULES;
        const fileName = `${blockId}.${ext}`;
        const filePath = path.join(dir, fileName);
        await fs.writeFile(filePath, blockLines.join('\n') + (blockLines.length ? '\n' : ''));
        const rel = path.relative(WEB, filePath).split(path.sep).join('/');
        const idAttr = blockIdRaw ? ` id="${blockIdRaw}"` : '';
        shell.push(`${blockIndent}<!-- BUILD:INCLUDE src="${rel}" as="${mode}"${idAttr} -->`);
        manifest.push({ rel, as: mode, id: blockIdRaw, lines: blockLines.length });
        mode = 'normal';
        blockId = null;
        blockIdRaw = null;
        blockIndent = '';
        blockLines = [];
        continue;
      }
      blockLines.push(line);
    }
  }

  if (mode !== 'normal') {
    throw new Error(`Unterminated <${mode}> block (id=${blockId})`);
  }

  await fs.writeFile(path.join(WEB, 'index.html'), shell.join('\n'));
  await fs.writeFile(
    path.join(WEB, '.manifest.json'),
    JSON.stringify({ generatedFrom: path.basename(SRC), blocks: manifest }, null, 2) + '\n'
  );

  const styleCount = manifest.filter((b) => b.as === 'style').length;
  const scriptCount = manifest.filter((b) => b.as === 'script').length;
  console.log(`Extracted ${styleCount} stylesheets + ${scriptCount} scripts (${manifest.length} blocks).`);
  console.log(`Shell: web/index.html (${shell.length} lines)`);
}

main().catch((err) => { console.error(err); process.exit(1); });
