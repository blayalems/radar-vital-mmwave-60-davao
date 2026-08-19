import fs from 'node:fs';
import path from 'node:path';
import url from 'node:url';

export const REQUIRED_WORKFLOWS = Object.freeze([
  Object.freeze({
    name: 'Playwright tests',
    path: '.github/workflows/playwright.yml',
    events: Object.freeze(['push', 'pull_request'])
  }),
  Object.freeze({
    name: 'Security Audit',
    path: '.github/workflows/security-audit.yml',
    events: Object.freeze(['push', 'pull_request'])
  }),
  Object.freeze({
    name: 'Build Android APK (Capacitor)',
    path: '.github/workflows/build-apk.yml',
    events: Object.freeze(['push', 'pull_request'])
  }),
  Object.freeze({
    name: 'Build Windows EXE (Tauri)',
    path: '.github/workflows/build-exe.yml',
    events: Object.freeze(['push', 'pull_request'])
  })
]);

const SEMVER_TAG = /^v([0-9]+\.[0-9]+\.[0-9]+)(?:[-+][0-9A-Za-z.-]+)?$/;

export function cliOptionsFromArgs(argv) {
  const verifyOnly = argv.includes('--verify-only');
  const index = argv.indexOf('--output');
  if (index >= 0 && (!argv[index + 1] || argv[index + 1].startsWith('--'))) {
    throw new Error('--output requires a path.');
  }
  const inline = argv.find(value => value.startsWith('--output='));
  const outputPath = index >= 0
    ? argv[index + 1]
    : inline?.slice('--output='.length) || 'required-check-evidence.json';
  if (!outputPath) throw new Error('--output requires a path.');
  return { outputPath, verifyOnly };
}

export function validateTagIdentity(releaseTag, productVersion) {
  const match = SEMVER_TAG.exec(String(releaseTag || ''));
  if (!match) throw new Error(`Release tag must be semantic v<version>, received: ${releaseTag || '<empty>'}`);
  if (match[1] !== productVersion) {
    throw new Error(`Release tag product version ${match[1]} does not match ${productVersion}.`);
  }
}

async function githubRequest(fetchImpl, apiUrl, token, pathname) {
  const response = await fetchImpl(`${apiUrl}${pathname}`, {
    headers: {
      Accept: 'application/vnd.github+json',
      Authorization: `Bearer ${token}`,
      'X-GitHub-Api-Version': '2022-11-28'
    }
  });
  const body = response.status === 204 ? null : await response.json().catch(() => null);
  return { status: response.status, ok: response.ok, body };
}

async function existingTagTarget(options) {
  const encodedTag = encodeURIComponent(options.releaseTag);
  const reference = await githubRequest(
    options.fetchImpl,
    options.apiUrl,
    options.token,
    `/repos/${options.repository}/git/ref/tags/${encodedTag}`
  );
  if (reference.status === 404) return null;
  if (!reference.ok) throw new Error(`GitHub tag lookup failed with HTTP ${reference.status}.`);
  let object = reference.body?.object;
  for (let depth = 0; object?.type === 'tag' && depth < 4; depth += 1) {
    const tag = await githubRequest(
      options.fetchImpl,
      options.apiUrl,
      options.token,
      `/repos/${options.repository}/git/tags/${object.sha}`
    );
    if (!tag.ok) throw new Error(`Annotated tag lookup failed with HTTP ${tag.status}.`);
    object = tag.body?.object;
  }
  if (object?.type !== 'commit' || !/^[0-9a-f]{40}$/.test(String(object.sha || ''))) {
    throw new Error('Release tag does not resolve to a commit SHA.');
  }
  return object.sha;
}

function latestRunsByWorkflow(runs, sourceSha) {
  const result = new Map();
  for (const run of runs) {
    const workflow = REQUIRED_WORKFLOWS.find(candidate =>
      candidate.name === run?.name &&
      candidate.path === run?.path &&
      candidate.events.includes(run?.event)
    );
    if (run?.head_sha !== sourceSha || !workflow) continue;
    const previous = result.get(workflow.path);
    const timestamp = Date.parse(run.updated_at || run.created_at || '') || 0;
    const previousTimestamp = Date.parse(previous?.updated_at || previous?.created_at || '') || 0;
    if (!previous || timestamp > previousTimestamp) result.set(workflow.path, run);
  }
  return result;
}

export async function validateReleaseSource(options) {
  validateTagIdentity(options.releaseTag, options.productVersion);
  if (!/^[0-9a-f]{40}$/.test(String(options.sourceSha || ''))) {
    throw new Error('GITHUB_SHA must be a full lowercase commit SHA.');
  }

  const tagTarget = await existingTagTarget(options);
  if (tagTarget && tagTarget !== options.sourceSha) {
    throw new Error(`Existing tag ${options.releaseTag} resolves to ${tagTarget}, not ${options.sourceSha}.`);
  }

  const release = await githubRequest(
    options.fetchImpl,
    options.apiUrl,
    options.token,
    `/repos/${options.repository}/releases/tags/${encodeURIComponent(options.releaseTag)}`
  );
  if (release.ok) throw new Error(`GitHub Release ${options.releaseTag} already exists and is immutable.`);
  if (release.status !== 404) throw new Error(`GitHub Release lookup failed with HTTP ${release.status}.`);

  const runsResponse = await githubRequest(
    options.fetchImpl,
    options.apiUrl,
    options.token,
    `/repos/${options.repository}/actions/runs?head_sha=${options.sourceSha}&per_page=100`
  );
  if (!runsResponse.ok) throw new Error(`Required workflow lookup failed with HTTP ${runsResponse.status}.`);
  const latest = latestRunsByWorkflow(runsResponse.body?.workflow_runs || [], options.sourceSha);
  const checks = REQUIRED_WORKFLOWS.map(workflow => {
    const run = latest.get(workflow.path);
    if (!run) {
      throw new Error(
        `Required workflow is missing for ${options.sourceSha}: ${workflow.name} (${workflow.path}).`
      );
    }
    if (run.status !== 'completed' || run.conclusion !== 'success') {
      throw new Error(`Required workflow ${workflow.name} is ${run.status || 'unknown'}/${run.conclusion || 'pending'}.`);
    }
    if (!/^\d+$/.test(String(run.workflow_id || ''))) {
      throw new Error(`Required workflow ${workflow.name} has no valid workflow_id.`);
    }
    if (!Number.isInteger(run.run_attempt) || run.run_attempt < 1) {
      throw new Error(`Required workflow ${workflow.name} has no valid run_attempt.`);
    }
    return {
      workflow_name: workflow.name,
      workflow_id: String(run.workflow_id),
      path: workflow.path,
      event: run.event,
      conclusion: 'success',
      run_id: String(run.id),
      run_attempt: run.run_attempt,
      run_url: run.html_url,
      completed_at: run.updated_at
    };
  });

  return {
    schema_version: 'rvt-required-check-evidence-v1',
    release_tag: options.releaseTag,
    source_sha: options.sourceSha,
    generated_at: options.generatedAt || new Date().toISOString(),
    tag_state: tagTarget ? 'existing_same_sha_without_release' : 'new_tag_identity',
    checks
  };
}

async function main() {
  const root = path.resolve(path.dirname(url.fileURLToPath(import.meta.url)), '..');
  const productVersion = JSON.parse(fs.readFileSync(path.join(root, 'package.json'), 'utf8')).version;
  const required = ['RELEASE_TAG', 'GITHUB_SHA', 'GITHUB_REPOSITORY', 'GH_TOKEN'];
  for (const name of required) {
    if (!process.env[name]) throw new Error(`${name} is required.`);
  }
  const evidence = await validateReleaseSource({
    releaseTag: process.env.RELEASE_TAG,
    productVersion,
    sourceSha: process.env.GITHUB_SHA,
    repository: process.env.GITHUB_REPOSITORY,
    apiUrl: process.env.GITHUB_API_URL || 'https://api.github.com',
    token: process.env.GH_TOKEN,
    fetchImpl: fetch
  });
  const cliOptions = cliOptionsFromArgs(process.argv.slice(2));
  if (!cliOptions.verifyOnly) {
    const outputPath = path.resolve(cliOptions.outputPath);
    fs.writeFileSync(outputPath, `${JSON.stringify(evidence, null, 2)}\n`, { flag: 'wx' });
  }
  console.log(`Release source is immutable and verified: ${evidence.release_tag} @ ${evidence.source_sha}`);
}

if (process.argv[1] && path.resolve(process.argv[1]) === url.fileURLToPath(import.meta.url)) {
  main().catch(error => {
    console.error(error.message || error);
    process.exitCode = 1;
  });
}
