import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');

function projectVersion(text, label) {
  const project = text.match(/\[project\]([\s\S]*?)(?=\n\[|$)/)?.[1] ?? '';
  const value = project.match(/^\s*version\s*=\s*"(\d+)\.(\d+)\.(\d+)"\s*$/m);
  if (!value) throw new Error(`${label} has no strict [project].version`);
  return value.slice(1).map(Number);
}

function format(parts) {
  return parts.join('.');
}

const baseFlagIndex = process.argv.findIndex((arg) => arg === '--base-ref');
const inlineBase = process.argv.find((arg) => arg.startsWith('--base-ref='));
const requestedBase =
  (baseFlagIndex >= 0 ? process.argv[baseFlagIndex + 1] : undefined) ??
  inlineBase?.slice('--base-ref='.length) ??
  (process.env.GITHUB_BASE_REF ? `origin/${process.env.GITHUB_BASE_REF}` : undefined);

if (!requestedBase) {
  throw new Error('Pass --base-ref <git-ref> (for example origin/main).');
}

const current = projectVersion(readFileSync(path.join(ROOT, 'pyproject.toml'), 'utf8'), 'current branch');
let baseText;
try {
  baseText = execFileSync('git', ['show', `${requestedBase}:pyproject.toml`], {
    cwd: ROOT,
    encoding: 'utf8',
    stdio: ['ignore', 'pipe', 'pipe'],
  });
} catch (error) {
  const stderr = String(error?.stderr ?? '').trim();
  throw new Error(`Cannot read pyproject.toml from ${requestedBase}${stderr ? `: ${stderr}` : ''}`);
}
const base = projectVersion(baseText, requestedBase);

const patchStep =
  current[0] === base[0] &&
  current[1] === base[1] &&
  current[2] === base[2] + 1;
const minorStep =
  current[0] === base[0] &&
  current[1] === base[1] + 1 &&
  current[2] === 0;

if (!patchStep && !minorStep) {
  throw new Error(
    `PR version must advance exactly one patch or one minor: base=${format(base)}, current=${format(current)}`,
  );
}

console.log(`PR release step valid: ${format(base)} -> ${format(current)} (${patchStep ? 'patch' : 'minor'})`);
