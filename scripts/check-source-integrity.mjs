import { readFile } from 'node:fs/promises';
import path from 'node:path';
import url from 'node:url';

const ROOT = path.resolve(path.dirname(url.fileURLToPath(import.meta.url)), '..');
const files = [
  'docs/i18n.md',
  'scripts/check-contrast-tokens.mjs',
  'web/angular.json',
  'web/src/app/a11y/contrast-core.mjs',
  'web/src/app/a11y/contrast.spec.ts',
  'web/src/app/a11y/contrast.ts',
  'web/src/app/components/home/home.component.html',
  'web/src/app/components/topbar/topbar.component.html',
  'web/src/app/i18n/translate.pipe.ts',
  'web/src/app/services/i18n.messages.en.ts',
  'web/src/app/services/i18n.service.ts',
  'web/src/styles/rvt-accessibility-overrides.css',
  'web/src/styles/rvt-redesign-tokens.css',
];

const contents = new Map();
const barePlaceholder = new RegExp('^\\s*(tool' + 'u_[A-Za-z0-9]+)\\s*$', 'm');
for (const relativePath of files) {
  const content = await readFile(path.join(ROOT, relativePath), 'utf8');
  contents.set(relativePath, content);
  const match = content.match(barePlaceholder);
  if (match) {
    throw new Error(`${relativePath} contains an accidental tool placeholder: ${match[1]}`);
  }
  if (!content.trim()) throw new Error(`${relativePath} is unexpectedly empty`);
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
  const content = contents.get(relativePath);
  for (const pattern of patterns) {
    if (!pattern.test(content)) {
      throw new Error(`${relativePath} is missing critical binding ${pattern}`);
    }
  }
}

console.log(`Source integrity checks passed across ${files.length} critical files.`);
