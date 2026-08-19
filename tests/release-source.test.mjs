import assert from 'node:assert/strict';
import test from 'node:test';

import {
  REQUIRED_WORKFLOWS,
  cliOptionsFromArgs,
  validateReleaseSource,
  validateTagIdentity
} from '../scripts/check-release-source.mjs';

const SHA = 'a'.repeat(40);
const OTHER_SHA = 'b'.repeat(40);

function response(status, body = {}) {
  return {
    status,
    ok: status >= 200 && status < 300,
    async json() { return body; }
  };
}

function runs(overrides = {}) {
  return REQUIRED_WORKFLOWS.map((workflow, index) => ({
    id: index + 10,
    workflow_id: index + 100,
    name: workflow.name,
    path: workflow.path,
    event: 'push',
    run_attempt: 1,
    head_sha: SHA,
    status: 'completed',
    conclusion: 'success',
    html_url: `https://example.invalid/runs/${index + 10}`,
    updated_at: `2026-08-19T00:0${index}:00Z`,
    ...overrides[workflow.name]
  }));
}

function fixture({ tagSha = null, releaseExists = false, workflowRuns = runs() } = {}) {
  return async url => {
    if (url.includes('/git/ref/tags/')) {
      return tagSha ? response(200, { object: { type: 'commit', sha: tagSha } }) : response(404);
    }
    if (url.includes('/releases/tags/')) return releaseExists ? response(200, { id: 1 }) : response(404);
    if (url.includes('/actions/runs?')) return response(200, { workflow_runs: workflowRuns });
    throw new Error(`unexpected URL: ${url}`);
  };
}

function options(fetchImpl) {
  return {
    releaseTag: 'v16.6.3-rc.1',
    productVersion: '16.6.3',
    sourceSha: SHA,
    repository: 'owner/repo',
    apiUrl: 'https://api.example.invalid',
    token: 'fixture',
    fetchImpl,
    generatedAt: '2026-08-19T00:00:00Z'
  };
}

test('accepts a new immutable identity after all exact-source workflows pass', async () => {
  const evidence = await validateReleaseSource(options(fixture()));
  assert.equal(evidence.tag_state, 'new_tag_identity');
  assert.deepEqual(
    evidence.checks.map(item => item.workflow_name),
    REQUIRED_WORKFLOWS.map(item => item.name)
  );
  assert.deepEqual(evidence.checks[0], {
    workflow_name: 'Playwright tests',
    workflow_id: '100',
    path: '.github/workflows/playwright.yml',
    event: 'push',
    conclusion: 'success',
    run_id: '10',
    run_attempt: 1,
    run_url: 'https://example.invalid/runs/10',
    completed_at: '2026-08-19T00:00:00Z'
  });
});

test('accepts same-SHA tag retry only while no Release exists', async () => {
  const evidence = await validateReleaseSource(options(fixture({ tagSha: SHA })));
  assert.equal(evidence.tag_state, 'existing_same_sha_without_release');
});

test('rejects a tag that resolves to different source', async () => {
  await assert.rejects(validateReleaseSource(options(fixture({ tagSha: OTHER_SHA }))), /not a{40}/);
});

test('rejects any already-published Release', async () => {
  await assert.rejects(validateReleaseSource(options(fixture({ tagSha: SHA, releaseExists: true }))), /already exists/);
});

test('rejects missing, pending, failed, and skipped required workflows', async () => {
  await assert.rejects(validateReleaseSource(options(fixture({ workflowRuns: runs().slice(1) }))), /missing/);
  for (const conclusion of ['', 'failure', 'skipped', 'cancelled']) {
    const workflowRuns = runs({ 'Playwright tests': { status: conclusion ? 'completed' : 'in_progress', conclusion } });
    await assert.rejects(validateReleaseSource(options(fixture({ workflowRuns }))), /Playwright tests/);
  }
});

test('uses the newest exact-source run and rejects a later regression', async () => {
  const workflowRuns = [
    ...runs(),
    { ...runs()[0], id: 99, conclusion: 'failure', updated_at: '2026-08-20T00:00:00Z' }
  ];
  await assert.rejects(validateReleaseSource(options(fixture({ workflowRuns }))), /failure/);
});

test('binds required checks to the trusted workflow name, path, and event', async () => {
  const wrongPath = runs({ 'Playwright tests': { path: '.github/workflows/lookalike.yml' } });
  await assert.rejects(validateReleaseSource(options(fixture({ workflowRuns: wrongPath }))), /playwright\.yml/);

  const wrongName = runs({ 'Playwright tests': { name: 'Playwright lookalike' } });
  await assert.rejects(validateReleaseSource(options(fixture({ workflowRuns: wrongName }))), /Playwright tests/);

  const manual = runs({ 'Playwright tests': { event: 'workflow_dispatch' } });
  await assert.rejects(validateReleaseSource(options(fixture({ workflowRuns: manual }))), /Playwright tests/);
});

test('rejects incomplete workflow and run provenance', async () => {
  const withoutWorkflowId = runs({ 'Playwright tests': { workflow_id: null } });
  await assert.rejects(validateReleaseSource(options(fixture({ workflowRuns: withoutWorkflowId }))), /workflow_id/);

  const withoutAttempt = runs({ 'Playwright tests': { run_attempt: 0 } });
  await assert.rejects(validateReleaseSource(options(fixture({ workflowRuns: withoutAttempt }))), /run_attempt/);
});

test('verify-only CLI mode does not request a second evidence write', () => {
  assert.deepEqual(cliOptionsFromArgs(['--verify-only']), {
    outputPath: 'required-check-evidence.json',
    verifyOnly: true
  });
  assert.throws(() => cliOptionsFromArgs(['--output']), /requires a path/);
});

test('rejects malformed or product-mismatched release tags', () => {
  assert.throws(() => validateTagIdentity('release-16.6.3', '16.6.3'), /semantic/);
  assert.throws(() => validateTagIdentity('v16.6.2', '16.6.3'), /does not match/);
});
