import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import test from 'node:test';

import { runQmsContract } from '../scripts/check-qms-contract.mjs';

function write(root, relativePath, value) {
  const target = path.join(root, relativePath);
  fs.mkdirSync(path.dirname(target), { recursive: true });
  fs.writeFileSync(
    target,
    typeof value === 'string' ? value : `${JSON.stringify(value, null, 2)}\n`
  );
}

function git(root, ...args) {
  return execFileSync('git', args, { cwd: root, encoding: 'utf8' }).trim();
}

function objectSchema(required, properties) {
  return {
    $schema: 'https://json-schema.org/draft/2020-12/schema',
    type: 'object',
    required,
    properties,
    additionalProperties: true
  };
}

function fixture() {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'rvt-qms-'));
  git(root, 'init', '-q');
  git(root, 'config', 'user.email', 'qms@example.invalid');
  git(root, 'config', 'user.name', 'QMS Test');
  git(root, 'config', 'core.autocrlf', 'false');
  write(root, 'package.json', { version: '16.5.8' });
  write(root, 'HANDOFF.md', '# Handoff\n');
  write(root, 'docs/controlled.md', '# Controlled\n');
  write(root, 'src/feature.ts', 'export const enabled = true;\n');
  write(root, 'tests/feature.spec.ts', "test('records release identity', () => {});\n");

  const documentEntrySchema = objectSchema(
    [
      'document_id',
      'title',
      'path',
      'owner_role',
      'record_type',
      'status',
      'revision',
      'effective_product_version',
      'supersedes',
      'approver_roles',
      'retention',
      'confidentiality',
      'review_interval_months'
    ],
    {
      document_id: { type: 'string', minLength: 1 },
      title: { type: 'string', minLength: 1 },
      path: { type: 'string', minLength: 1 },
      revision: { type: 'string', minLength: 1 }
    }
  );
  write(
    root,
    'quality/schemas/document-register.schema.json',
    objectSchema(
      [
        '$schema',
        'schema_version',
        'register_id',
        'register_revision',
        'effective_product_version',
        'status',
        'owner_role',
        'approval_method',
        'documents'
      ],
      {
        $schema: { type: 'string' },
        schema_version: { type: 'string' },
        register_revision: { type: 'string' },
        documents: { type: 'array', minItems: 1, items: documentEntrySchema }
      }
    )
  );
  write(
    root,
    'quality/schemas/requirements.schema.json',
    objectSchema(
      [
        '$schema',
        'schema_version',
        'register_id',
        'register_revision',
        'effective_product_version',
        'status',
        'owner_role',
        'requirements'
      ],
      {
        $schema: { type: 'string' },
        schema_version: { type: 'string' },
        requirements: {
          type: 'array',
          minItems: 1,
          items: objectSchema(
            [
              'requirement_id',
              'title',
              'statement',
              'source_refs',
              'category',
              'risk_class',
              'owner_role',
              'status',
              'implementation_paths',
              'verification_refs',
              'artifact_types',
              'acceptance_criteria',
              'change_record_required'
            ],
            {
              requirement_id: { type: 'string', minLength: 1 },
              implementation_paths: { type: 'array', minItems: 1, items: { type: 'string' } },
              verification_refs: { type: 'array', minItems: 1, items: { type: 'object' } }
            }
          )
        }
      }
    )
  );
  write(root, 'quality/schemas/release-record.schema.json', {
    $schema: 'https://json-schema.org/draft/2020-12/schema',
    type: 'object'
  });
  write(root, 'quality/document-register.json', {
    $schema: './schemas/document-register.schema.json',
    schema_version: 'rvt-qms-document-register-v1',
    register_id: 'RVT-QMS-DOCUMENT-REGISTER',
    register_revision: 'R01',
    effective_product_version: '16.5.8',
    status: 'active',
    owner_role: 'Quality lead',
    approval_method: 'protected_pull_request',
    additive_metadata: 'permitted',
    documents: [
      {
        document_id: 'RVT-QMS-CONTROLLED-001',
        title: 'Controlled test document',
        path: 'docs/controlled.md',
        owner_role: 'Document owner',
        approver_roles: ['Quality lead'],
        record_type: 'guide',
        status: 'active',
        revision: 'R01',
        effective_product_version: '16.5.8',
        supersedes: null,
        retention: {
          class: 'project_lifetime_plus_5_years',
          disposition: 'archive'
        },
        confidentiality: 'internal',
        review_interval_months: 12
      }
    ]
  });
  write(root, 'quality/requirements.json', {
    $schema: './schemas/requirements.schema.json',
    schema_version: 'rvt-qms-requirements-v1',
    register_id: 'RVT-QMS-REQUIREMENTS',
    register_revision: 'R01',
    effective_product_version: '16.5.8',
    status: 'active',
    owner_role: 'Quality lead',
    requirements: [
      {
        requirement_id: 'RVT-SESSION-001',
        title: 'Release identity is recorded',
        statement: 'Every session records its release identity.',
        source_refs: ['ISO 9001 traceability'],
        category: 'session',
        risk_class: 'high',
        owner_role: 'Software lead',
        status: 'active',
        implementation_paths: ['src/feature.ts'],
        verification_refs: [
          {
            path: 'tests/feature.spec.ts',
            selector: 'records release identity',
            kind: 'automated_test'
          }
        ],
        artifact_types: ['session_manifest'],
        acceptance_criteria: ['Automated test passes'],
        change_record_required: true
      }
    ]
  });
  git(root, 'add', '.');
  git(root, 'commit', '-qm', 'baseline');
  return root;
}

function validPrBody() {
  return `## Change identity

- Change title: Tighten QMS evidence
- Related issue, review, CAPA, or decision: Review finding
- Requirement IDs from \`quality/requirements.json\`: RVT-SESSION-001
- Administrative-only \`N/A\` justification, if applicable: Not applicable
- Intended product version: 16.5.8
- Release step from base: patch

## Purpose and scope

Validate durable evidence.

## Impact assessment

- [x] Controlled documentation

Risk class: high

## Traceability

| Requirement ID | Implementation path(s) | Verification reference(s) | Affected artifact(s) |
|---|---|---|---|
| RVT-SESSION-001 | scripts/check-qms-contract.mjs | tests/qms-contract.test.mjs | CI |

## Controlled documents

None.

## Compatibility and migration

- Backward-compatible behavior: Yes

## Verification evidence

| Check or command | Result | Environment/evidence link |
|---|---|---|
| node --test | Passed | CI |

## Release and rollback

- Rollback strategy: Revert the change commit.

## Review and authorization

- Technical review role: Technical lead

## Completion checklist

- [x] The change is limited to the stated scope.
`;
}

test('accepts schema-valid registers, additive fields, IDs, paths, and selectors', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));

  const result = runQmsContract({ root });

  assert.equal(result.ok, true, result.errors.join('\n'));
  assert.equal(result.summary.documents, 1);
  assert.equal(result.summary.requirements, 1);
});

test('rejects duplicate IDs, missing references, and absent test selectors', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));
  const requirementsPath = path.join(root, 'quality/requirements.json');
  const requirements = JSON.parse(fs.readFileSync(requirementsPath, 'utf8'));
  requirements.requirements.push({
    ...requirements.requirements[0],
    implementation_paths: ['src/missing.ts'],
    verification_refs: [
      {
        path: 'tests/feature.spec.ts',
        selector: 'does not exist',
        kind: 'automated_test'
      }
    ]
  });
  write(root, 'quality/requirements.json', requirements);

  const result = runQmsContract({ root });

  assert.equal(result.ok, false);
  assert.match(result.errors.join('\n'), /duplicate requirement_id RVT-SESSION-001/);
  assert.match(result.errors.join('\n'), /referenced path does not exist/);
  assert.match(result.errors.join('\n'), /selector not found/);
});

test('requires a revision bump whenever a registered controlled document changes', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));
  const base = git(root, 'rev-parse', 'HEAD');
  write(root, 'docs/controlled.md', '# Controlled\n\nChanged without revision.\n');
  write(root, 'HANDOFF.md', '# Handoff\n\nDocument changed.\n');
  git(root, 'add', '.');
  git(root, 'commit', '-qm', 'change controlled document');

  const failed = runQmsContract({ root, baseRef: base });
  assert.equal(failed.ok, false);
  assert.match(failed.errors.join('\n'), /changed without exactly one RNN revision increment/);

  const registerPath = path.join(root, 'quality/document-register.json');
  const register = JSON.parse(fs.readFileSync(registerPath, 'utf8'));
  register.documents[0].revision = 'R02';
  register.register_revision = 'R02';
  write(root, 'quality/document-register.json', register);
  const passed = runQmsContract({ root, baseRef: base });
  assert.doesNotMatch(passed.errors.join('\n'), /changed without exactly one RNN revision increment/);
});

test('requires revisions for controlled path renames and metadata-only changes', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));
  const base = git(root, 'rev-parse', 'HEAD');
  const registerPath = path.join(root, 'quality/document-register.json');
  const register = JSON.parse(fs.readFileSync(registerPath, 'utf8'));

  fs.renameSync(path.join(root, 'docs/controlled.md'), path.join(root, 'docs/renamed.md'));
  register.documents[0].path = 'docs/renamed.md';
  register.documents[0].owner_role = 'Technical lead';
  write(root, 'quality/document-register.json', register);

  const failed = runQmsContract({ root, baseRef: base });
  assert.equal(failed.ok, false);
  assert.match(failed.errors.join('\n'), /including rename or metadata changes/);

  register.documents[0].revision = 'R02';
  write(root, 'quality/document-register.json', register);
  const passed = runQmsContract({ root, baseRef: base });
  assert.doesNotMatch(passed.errors.join('\n'), /including rename or metadata changes/);
});

test('requires automated evidence for active high and critical requirements', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));
  const requirementsPath = path.join(root, 'quality/requirements.json');
  const requirements = JSON.parse(fs.readFileSync(requirementsPath, 'utf8'));
  requirements.requirements[0].verification_refs[0].kind = 'manual_acceptance';
  write(root, 'quality/requirements.json', requirements);

  const failed = runQmsContract({ root });
  assert.equal(failed.ok, false);
  assert.match(failed.errors.join('\n'), /active high-risk requirement requires automated_test or ci_check evidence/);

  requirements.requirements[0].status = 'draft';
  write(root, 'quality/requirements.json', requirements);
  const draft = runQmsContract({ root });
  assert.doesNotMatch(draft.errors.join('\n'), /requires automated_test or ci_check evidence/);
});

test('validates PR change records from an explicit local body', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));

  const passed = runQmsContract({ root, requirePrBody: true, prBody: validPrBody() });
  assert.equal(passed.ok, true, passed.errors.join('\n'));

  const failed = runQmsContract({
    root,
    requirePrBody: true,
    prBody: validPrBody()
      .replace('Risk class: high', 'Risk class: low / medium / high / critical')
      .replace('| node --test | Passed | CI |', '| node --test |  | CI |')
  });
  assert.equal(failed.ok, false);
  assert.match(failed.errors.join('\n'), /Risk class must select exactly/);
  assert.match(failed.errors.join('\n'), /needs at least one explicit result row/);
});

test('loads PR change record body from the GitHub event payload', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));
  const eventPath = path.join(root, 'event.json');
  write(root, 'event.json', { pull_request: { body: validPrBody() } });
  const previousEventPath = process.env.GITHUB_EVENT_PATH;
  const previousEventName = process.env.GITHUB_EVENT_NAME;
  process.env.GITHUB_EVENT_PATH = eventPath;
  process.env.GITHUB_EVENT_NAME = 'pull_request';
  t.after(() => {
    if (previousEventPath === undefined) delete process.env.GITHUB_EVENT_PATH;
    else process.env.GITHUB_EVENT_PATH = previousEventPath;
    if (previousEventName === undefined) delete process.env.GITHUB_EVENT_NAME;
    else process.env.GITHUB_EVENT_NAME = previousEventName;
  });

  const result = runQmsContract({ root, requirePrBody: true });
  assert.equal(result.ok, true, result.errors.join('\n'));
});

test('does not leak the caller GitHub event into an optional fixture check', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));
  const eventPath = path.join(root, 'event.json');
  write(root, 'event.json', { pull_request: { body: 'not a fixture change record' } });
  const previousEventPath = process.env.GITHUB_EVENT_PATH;
  const previousEventName = process.env.GITHUB_EVENT_NAME;
  process.env.GITHUB_EVENT_PATH = eventPath;
  process.env.GITHUB_EVENT_NAME = 'pull_request';
  t.after(() => {
    if (previousEventPath === undefined) delete process.env.GITHUB_EVENT_PATH;
    else process.env.GITHUB_EVENT_PATH = previousEventPath;
    if (previousEventName === undefined) delete process.env.GITHUB_EVENT_NAME;
    else process.env.GITHUB_EVENT_NAME = previousEventName;
  });

  const result = runQmsContract({ root, requirePrBody: false });
  assert.equal(result.ok, true, result.errors.join('\n'));
});

test('requires HANDOFF.md in every commit in the selected base range', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));
  const base = git(root, 'rev-parse', 'HEAD');
  write(root, 'src/feature.ts', 'export const enabled = false;\n');
  git(root, 'add', '.');
  git(root, 'commit', '-qm', 'missing handoff');
  write(root, 'HANDOFF.md', '# Handoff\n\nLater update cannot repair prior commit.\n');
  git(root, 'add', '.');
  git(root, 'commit', '-qm', 'add handoff later');

  const result = runQmsContract({ root, baseRef: base });

  assert.equal(result.ok, false);
  assert.match(result.errors.join('\n'), /commit [0-9a-f]{12} does not update HANDOFF\.md/);
  assert.equal(result.summary.checked_commits, 2);
});

test('fails closed when an explicit base ref cannot be resolved', t => {
  const root = fixture();
  t.after(() => fs.rmSync(root, { recursive: true, force: true }));

  const result = runQmsContract({ root, baseRef: 'origin/not-a-real-branch' });

  assert.equal(result.ok, false);
  assert.match(result.errors.join('\n'), /base ref does not resolve to a commit/);
});
