import { readFile, stat } from 'node:fs/promises';

const repository = process.env.GITHUB_REPOSITORY;
const token = process.env.GITHUB_TOKEN;
const branch = process.env.HEAD_BRANCH;
if (!repository || !token || !branch) {
  throw new Error('GITHUB_REPOSITORY, GITHUB_TOKEN, and HEAD_BRANCH are required');
}

const [owner, repo] = repository.split('/');
const apiBase = `https://api.github.com/repos/${owner}/${repo}`;
const headers = {
  Accept: 'application/vnd.github+json',
  Authorization: `Bearer ${token}`,
  'X-GitHub-Api-Version': '2022-11-28',
  'Content-Type': 'application/json',
};

async function api(path, options = {}) {
  const response = await fetch(`${apiBase}${path}`, {
    ...options,
    headers: { ...headers, ...(options.headers ?? {}) },
  });
  const text = await response.text();
  if (!response.ok) {
    throw new Error(`${options.method ?? 'GET'} ${path} failed (${response.status}): ${text}`);
  }
  return text ? JSON.parse(text) : {};
}

const writePaths = [
  'assets/fonts/rvt-runtime-accessibility.css',
  'assets/sw.js',
  'docs/i18n.md',
  'radar_vital_live_dashboard_v12_for_v16_0.html',
  'scripts/check-contrast-tokens.mjs',
  'scripts/check-source-integrity.mjs',
  'web/angular.json',
  'web/src/index.html',
  'web/src/app/components/home/home.component.html',
  'web/src/app/components/home/home.component.ts',
  'web/src/app/components/topbar/topbar.component.html',
  'web/src/app/components/topbar/topbar.component.ts',
  'web/src/app/i18n/translate.pipe.spec.ts',
  'web/src/app/i18n/translate.pipe.ts',
  'web/src/app/services/dynamic-color.service.spec.ts',
  'web/src/app/services/i18n.messages.en.ts',
  'web/src/app/services/i18n.service.ts',
  'web/src/styles/rvt-redesign-tokens.css',
];

const deletePaths = [
  'scripts/pr64-finalize.py',
  'scripts/pr64-postpatch.py',
  'scripts/pr64-publish.mjs',
  '.github/pr64-finalize-trigger',
  '.github/pr64-packaging-diagnostic.txt',
  '.github/pr64-diagnostic-comment-id',
];

const ref = await api(`/git/ref/heads/${encodeURIComponent(branch)}`);
const parentSha = ref.object.sha;
const parent = await api(`/git/commits/${parentSha}`);

const tree = [];
for (const path of writePaths) {
  const info = await stat(path);
  if (!info.isFile()) throw new Error(`Expected generated file is missing: ${path}`);
  const bytes = await readFile(path);
  const blob = await api('/git/blobs', {
    method: 'POST',
    body: JSON.stringify({ content: bytes.toString('base64'), encoding: 'base64' }),
  });
  tree.push({ path, mode: '100644', type: 'blob', sha: blob.sha });
}
for (const path of deletePaths) {
  tree.push({ path, mode: '100644', type: 'blob', sha: null });
}

const createdTree = await api('/git/trees', {
  method: 'POST',
  body: JSON.stringify({ base_tree: parent.tree.sha, tree }),
});
const commit = await api('/git/commits', {
  method: 'POST',
  body: JSON.stringify({
    message: 'fix(web): close PR64 accessibility review findings',
    tree: createdTree.sha,
    parents: [parentSha],
  }),
});
await api(`/git/refs/heads/${encodeURIComponent(branch)}`, {
  method: 'PATCH',
  body: JSON.stringify({ sha: commit.sha, force: false }),
});
console.log(`Published validated PR64 commit ${commit.sha}`);
