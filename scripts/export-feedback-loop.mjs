import { mkdir, readFile, writeFile } from 'node:fs/promises';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const SOURCE = path.join(ROOT, 'docs', 'system-feedback-loop.md');
const DEFAULT_OUT = path.join(ROOT, 'build', 'manuscript-figures');
const CHECK = process.argv.includes('--check');
const outArg = process.argv.find((arg) => arg.startsWith('--out='));
const OUT = outArg ? path.resolve(ROOT, outArg.slice('--out='.length)) : DEFAULT_OUT;

const EXPECTED = [
  'hardware-software-feedback-loop',
  'gbr-cnn-reproducibility-process',
];

function extractFigures(markdown) {
  const figures = new Map();
  const pattern =
    /<!--\s*figure:([a-z0-9-]+)\s*-->\s*```mermaid\r?\n([\s\S]*?)\r?\n```\s*\r?\n\r?\nManuscript caption \(`\1`\):\s*([\s\S]*?)(?=\r?\n\r?\n(?:##|<!--)|$)/g;
  for (const match of markdown.matchAll(pattern)) {
    const caption = match[3]
      .replace(/\r?\n/g, ' ')
      .replace(/\s+/g, ' ')
      .trim()
      .replace(/^“|”$/g, '');
    figures.set(match[1], { mermaid: `${match[2].trim()}\n`, caption });
  }
  return figures;
}

function texEscape(value) {
  return value
    .replaceAll('\\', '\\textbackslash{}')
    .replaceAll('&', '\\&')
    .replaceAll('%', '\\%')
    .replaceAll('$', '\\$')
    .replaceAll('#', '\\#')
    .replaceAll('_', '\\_')
    .replaceAll('{', '\\{')
    .replaceAll('}', '\\}')
    .replaceAll('~', '\\textasciitilde{}')
    .replaceAll('^', '\\textasciicircum{}');
}

const markdown = await readFile(SOURCE, 'utf8');
const figures = extractFigures(markdown);
const missing = EXPECTED.filter((id) => !figures.has(id));
const extra = [...figures.keys()].filter((id) => !EXPECTED.includes(id));

if (missing.length || extra.length) {
  throw new Error(
    `feedback-loop figure contract mismatch; missing=${missing.join(',') || 'none'} extra=${extra.join(',') || 'none'}`,
  );
}

for (const id of EXPECTED) {
  const figure = figures.get(id);
  if (!figure.mermaid.startsWith('flowchart ')) {
    throw new Error(`${id} must remain a Mermaid flowchart`);
  }
  if (!figure.caption || figure.caption.length < 80) {
    throw new Error(`${id} requires a manuscript-ready caption`);
  }
}

if (!CHECK) {
  await mkdir(OUT, { recursive: true });
  for (const id of EXPECTED) {
    const figure = figures.get(id);
    await writeFile(path.join(OUT, `${id}.mmd`), figure.mermaid, 'utf8');
    await writeFile(
      path.join(OUT, `${id}-caption.tex`),
      `\\caption{${texEscape(figure.caption)}}\n\\label{fig:${id}}\n`,
      'utf8',
    );
  }
  console.log(`Exported ${EXPECTED.length} feedback-loop figures to ${OUT}`);
} else {
  console.log(`Feedback-loop contract valid (${EXPECTED.length} figures).`);
}
