import { readFile } from 'node:fs/promises';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');

const files = [
  'docs/i18n.md',
  'web/src/app/a11y/contrast.spec.ts',
  'web/src/app/components/home/home.component.html',
  'web/src/app/components/topbar/topbar.component.html',
  'web/src/app/i18n/translate.pipe.ts',
  'web/src/app/services/i18n.messages.en.ts',
  'web/src/app/services/i18n.service.ts',
];

const contents = new Map();
for (const relativePath of files) {
  const content = await readFile(path.join(ROOT, relativePath), 'utf8');
  contents.set(relativePath, content);

  const placeholder = content.match(/\btoolu_[A-Za-z0-9]+\b/);
  if (placeholder) {
    throw new Error(`${relativePath} contains an accidental tool placeholder: ${placeholder[0]}`);
  }

  if (!content.trim()) {
    throw new Error(`${relativePath} is unexpectedly empty`);
  }
}

const requiredTemplateBindings = {
  'web/src/app/components/home/home.component.html': [
    "{{ getReadinessPercentage() }}",
    "{{ port }}",
    "{{ check.label }}",
    "{{ state.lastPayload()?.radar?.reported_hr",
    "{{ session.started_at | date:'shortTime' }}",
  ],
  'web/src/app/components/topbar/topbar.component.html': [
    "{{ getBreadcrumbTitle() }}",
    "{{ serverLifecycle.statusLabel() }}",
    "auto-retry in {{ seconds }}s",
    "{{ state.setup().operator_label || 'Operator A' }}",
    "{{ state.paused() ? 'play_arrow' : 'pause' }}",
  ],
};

for (const [relativePath, bindings] of Object.entries(requiredTemplateBindings)) {
  const content = contents.get(relativePath);
  for (const binding of bindings) {
    if (!content.includes(binding)) {
      throw new Error(`${relativePath} is missing required Angular binding: ${binding}`);
    }
  }
}

console.log('Source integrity checks passed: no tool placeholders and critical Angular bindings are intact.');
