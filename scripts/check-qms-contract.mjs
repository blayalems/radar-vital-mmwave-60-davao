#!/usr/bin/env node

import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const REPOSITORY_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');

const INSTANCE_SCHEMAS = new Map([
  ['quality/document-register.json', 'quality/schemas/document-register.schema.json'],
  ['quality/requirements.json', 'quality/schemas/requirements.schema.json']
]);
const REQUIRED_SCHEMAS = [
  ...INSTANCE_SCHEMAS.values(),
  'quality/schemas/release-record.schema.json'
];
const RESEARCH_SCHEMAS = [
  'quality/schemas/statistical-analysis-plan.schema.json',
  'quality/schemas/statistical-report.schema.json'
];
const DOCUMENT_ID_PATTERN = /^RVT-QMS-[A-Z0-9]+(?:-[A-Z0-9]+)*$/;
const REQUIREMENT_ID_PATTERN = /^RVT-[A-Z]+-[0-9]{3}$/;
const REVISION_PATTERN = /^R([0-9]{2,})$/;
const AUTOMATED_EVIDENCE_KINDS = new Set(['automated_test', 'ci_check']);
const PR_REQUIRED_SECTIONS = [
  'Change identity',
  'Purpose and scope',
  'Impact assessment',
  'Traceability',
  'Controlled documents',
  'Compatibility and migration',
  'Verification evidence',
  'Release and rollback',
  'Review and authorization',
  'Completion checklist'
];

function readJson(root, relativePath, errors) {
  const absolute = path.join(root, relativePath);
  try {
    return JSON.parse(fs.readFileSync(absolute, 'utf8'));
  } catch (error) {
    errors.push(`${relativePath}: cannot read valid JSON (${error.message})`);
    return null;
  }
}

function git(root, args, { optional = false } = {}) {
  try {
    return execFileSync('git', args, {
      cwd: root,
      encoding: 'utf8',
      stdio: ['ignore', 'pipe', optional ? 'ignore' : 'pipe']
    }).trim();
  } catch (error) {
    if (optional) return '';
    throw error;
  }
}

function schemaTypeMatches(value, expected) {
  if (expected === 'null') return value === null;
  if (expected === 'array') return Array.isArray(value);
  if (expected === 'object') return value !== null && typeof value === 'object' && !Array.isArray(value);
  if (expected === 'integer') return Number.isInteger(value);
  if (expected === 'number') return typeof value === 'number' && Number.isFinite(value);
  return typeof value === expected;
}

function schemaValueEqual(left, right) {
  if (Object.is(left, right)) return true;
  if (left === null || right === null || typeof left !== 'object' || typeof right !== 'object') return false;
  return JSON.stringify(left) === JSON.stringify(right);
}

function resolveLocalSchemaRef(rootSchema, reference) {
  if (!String(reference).startsWith('#/')) return null;
  return String(reference)
    .slice(2)
    .split('/')
    .map(part => part.replaceAll('~1', '/').replaceAll('~0', '~'))
    .reduce((current, part) => current?.[part], rootSchema);
}

function validateSchemaValue(value, schema, location, errors, rootSchema = schema) {
  if (!schema || typeof schema !== 'object') return;
  if (schema.$ref) {
    const target = resolveLocalSchemaRef(rootSchema, schema.$ref);
    if (!target) errors.push(`${location}: unresolved or external schema reference ${schema.$ref}`);
    else validateSchemaValue(value, target, location, errors, rootSchema);
    return;
  }
  for (const candidate of schema.allOf || []) {
    validateSchemaValue(value, candidate, location, errors, rootSchema);
  }
  if (schema.not) {
    const forbiddenErrors = [];
    validateSchemaValue(value, schema.not, location, forbiddenErrors, rootSchema);
    if (forbiddenErrors.length === 0) errors.push(`${location}: matches a forbidden schema`);
  }
  if (schema.oneOf) {
    const matches = schema.oneOf.filter(candidate => {
      const candidateErrors = [];
      validateSchemaValue(value, candidate, location, candidateErrors, rootSchema);
      return candidateErrors.length === 0;
    });
    if (matches.length !== 1) errors.push(`${location}: must match exactly one allowed schema`);
    return;
  }
  if (schema.anyOf) {
    const matched = schema.anyOf.some(candidate => {
      const candidateErrors = [];
      validateSchemaValue(value, candidate, location, candidateErrors, rootSchema);
      return candidateErrors.length === 0;
    });
    if (!matched) errors.push(`${location}: does not match any allowed schema`);
    return;
  }
  if (schema.const !== undefined && !schemaValueEqual(value, schema.const)) {
    errors.push(`${location}: must equal ${JSON.stringify(schema.const)}`);
  }
  if (Array.isArray(schema.enum) && !schema.enum.includes(value)) {
    errors.push(`${location}: must be one of ${schema.enum.map(String).join(', ')}`);
  }
  if (schema.type) {
    const expected = Array.isArray(schema.type) ? schema.type : [schema.type];
    if (!expected.some(type => schemaTypeMatches(value, type))) {
      errors.push(`${location}: expected ${expected.join(' or ')}`);
      return;
    }
  }
  if (typeof value === 'string') {
    if (schema.minLength !== undefined && value.length < schema.minLength) {
      errors.push(`${location}: shorter than minLength ${schema.minLength}`);
    }
    if (schema.pattern && !new RegExp(schema.pattern).test(value)) {
      errors.push(`${location}: does not match ${schema.pattern}`);
    }
    if (schema.format === 'date-time' && Number.isNaN(Date.parse(value))) {
      errors.push(`${location}: must be an ISO date-time`);
    }
  }
  if (typeof value === 'number' && Number.isFinite(value)) {
    if (schema.minimum !== undefined && value < schema.minimum) {
      errors.push(`${location}: must be at least ${schema.minimum}`);
    }
    if (schema.maximum !== undefined && value > schema.maximum) {
      errors.push(`${location}: must be at most ${schema.maximum}`);
    }
  }
  if (Array.isArray(value)) {
    if (schema.minItems !== undefined && value.length < schema.minItems) {
      errors.push(`${location}: requires at least ${schema.minItems} items`);
    }
    if (schema.uniqueItems) {
      const keys = value.map(item => JSON.stringify(item));
      if (new Set(keys).size !== keys.length) errors.push(`${location}: items must be unique`);
    }
    if (schema.items) {
      value.forEach((item, index) => validateSchemaValue(item, schema.items, `${location}[${index}]`, errors, rootSchema));
    }
  }
  if (value !== null && typeof value === 'object' && !Array.isArray(value)) {
    for (const required of schema.required || []) {
      if (!Object.prototype.hasOwnProperty.call(value, required)) {
        errors.push(`${location}: missing required property ${required}`);
      }
    }
    for (const [key, child] of Object.entries(schema.properties || {})) {
      if (Object.prototype.hasOwnProperty.call(value, key)) {
        validateSchemaValue(value[key], child, `${location}.${key}`, errors, rootSchema);
      }
    }
    if (schema.additionalProperties === false) {
      const allowed = new Set(Object.keys(schema.properties || {}));
      for (const key of Object.keys(value)) {
        if (!allowed.has(key)) errors.push(`${location}: unexpected property ${key}`);
      }
    }
  }
}

function normalizeRepoReference(root, reference, label, errors) {
  const raw = String(reference || '').trim().replaceAll('\\', '/');
  if (!raw) {
    errors.push(`${label}: reference is empty`);
    return null;
  }
  if (path.posix.isAbsolute(raw) || /^[A-Za-z]:\//.test(raw)) {
    errors.push(`${label}: absolute paths are not allowed (${raw})`);
    return null;
  }
  const normalized = path.posix.normalize(raw);
  if (normalized === '..' || normalized.startsWith('../')) {
    errors.push(`${label}: path escapes the repository (${raw})`);
    return null;
  }
  const absolute = path.resolve(root, ...normalized.split('/'));
  if (absolute !== root && !absolute.startsWith(`${root}${path.sep}`)) {
    errors.push(`${label}: path escapes the repository (${raw})`);
    return null;
  }
  return { relative: normalized, absolute };
}

function simpleGlobMatches(root, pattern) {
  const escaped = pattern
    .replace(/[.+^${}()|[\]\\]/g, '\\$&')
    .replaceAll('**', '\u0000')
    .replaceAll('*', '[^/]*')
    .replaceAll('\u0000', '.*')
    .replaceAll('?', '[^/]');
  const matcher = new RegExp(`^${escaped}$`);
  return git(root, ['ls-files'], { optional: true })
    .split(/\r?\n/)
    .filter(Boolean)
    .some(file => matcher.test(file.replaceAll('\\', '/')));
}

function requireRepoReference(root, reference, label, errors) {
  const resolved = normalizeRepoReference(root, reference, label, errors);
  if (!resolved) return;
  if (/[*?]/.test(resolved.relative)) {
    if (!simpleGlobMatches(root, resolved.relative)) {
      errors.push(`${label}: glob matches no tracked path (${resolved.relative})`);
    }
  } else if (!fs.existsSync(resolved.absolute)) {
    errors.push(`${label}: referenced path does not exist (${resolved.relative})`);
  }
}

function validateQualityReferences(root, documentRegister, requirements, errors) {
  const documents = Array.isArray(documentRegister?.documents) ? documentRegister.documents : [];
  const documentIds = new Set();
  const documentPaths = new Set();
  for (const [index, document] of documents.entries()) {
    const label = `quality/document-register.json documents[${index}]`;
    const id = String(document?.document_id || '');
    if (!DOCUMENT_ID_PATTERN.test(id)) errors.push(`${label}: invalid document_id ${id || '<empty>'}`);
    if (documentIds.has(id)) errors.push(`${label}: duplicate document_id ${id}`);
    documentIds.add(id);
    const controlledPath = String(document?.path || '').replaceAll('\\', '/');
    if (documentPaths.has(controlledPath)) errors.push(`${label}: duplicate controlled path ${controlledPath}`);
    documentPaths.add(controlledPath);
    requireRepoReference(root, document?.path, `${label}.path`, errors);
  }
  const registerDocuments = new Map(documents.map(document => [String(document?.path || '').replaceAll('\\', '/'), document]));
  const documentRegisterEntry = registerDocuments.get('quality/document-register.json');
  if (
    documentRegisterEntry &&
    String(documentRegisterEntry.revision) !== String(documentRegister?.register_revision)
  ) {
    errors.push('quality/document-register.json: register_revision must equal its controlled-document revision');
  }
  const requirementsEntry = registerDocuments.get('quality/requirements.json');
  if (
    requirementsEntry &&
    String(requirementsEntry.revision) !== String(requirements?.register_revision)
  ) {
    errors.push('quality/requirements.json: register_revision must equal its controlled-document revision');
  }
  for (const [index, document] of documents.entries()) {
    const supersedes = document?.supersedes;
    if (supersedes && !documentIds.has(String(supersedes))) {
      errors.push(`quality/document-register.json documents[${index}].supersedes: unknown document_id ${supersedes}`);
    }
  }

  const requirementRows = Array.isArray(requirements?.requirements) ? requirements.requirements : [];
  const requirementIds = new Set();
  for (const [index, requirement] of requirementRows.entries()) {
    const label = `quality/requirements.json requirements[${index}]`;
    const id = String(requirement?.requirement_id || '');
    if (!REQUIREMENT_ID_PATTERN.test(id)) errors.push(`${label}: invalid requirement_id ${id || '<empty>'}`);
    if (requirementIds.has(id)) errors.push(`${label}: duplicate requirement_id ${id}`);
    requirementIds.add(id);
    if (requirement?.status !== 'active') continue;
    if (
      (requirement?.risk_class === 'high' || requirement?.risk_class === 'critical') &&
      !(requirement?.verification_refs || []).some(ref => AUTOMATED_EVIDENCE_KINDS.has(ref?.kind))
    ) {
      errors.push(`${label}: active ${requirement.risk_class}-risk requirement requires automated_test or ci_check evidence`);
    }
    for (const [pathIndex, implementationPath] of (requirement?.implementation_paths || []).entries()) {
      requireRepoReference(root, implementationPath, `${label}.implementation_paths[${pathIndex}]`, errors);
    }
    for (const [testIndex, testRef] of (requirement?.verification_refs || []).entries()) {
      const refLabel = `${label}.verification_refs[${testIndex}]`;
      if (!testRef || typeof testRef !== 'object') {
        errors.push(`${refLabel}: verification reference must be an object`);
        continue;
      }
      const resolved = normalizeRepoReference(root, testRef.path, refLabel, errors);
      if (!resolved) continue;
      if (!fs.existsSync(resolved.absolute) || !fs.statSync(resolved.absolute).isFile()) {
        errors.push(`${refLabel}: verification file does not exist (${resolved.relative})`);
        continue;
      }
      if (testRef.kind === 'automated_test' || testRef.kind === 'ci_check') {
        const selector = String(testRef.selector || '').trim();
        const source = fs.readFileSync(resolved.absolute, 'utf8');
        if (!selector || !source.includes(selector)) {
          errors.push(`${refLabel}: selector not found in ${resolved.relative} (${selector || '<empty>'})`);
        }
      }
    }
  }
}

function parsePullRequestEvent(eventPath, errors) {
  try {
    const event = JSON.parse(fs.readFileSync(eventPath, 'utf8'));
    return typeof event?.pull_request?.body === 'string' ? event.pull_request.body : '';
  } catch (error) {
    errors.push(`pull request event: cannot read ${eventPath} (${error.message})`);
    return '';
  }
}

function resolvePullRequestBody({ root, prBody, prBodyFile, requirePrBody }, errors) {
  if (typeof prBody === 'string') return prBody;
  if (prBodyFile) {
    const resolved = normalizeRepoReference(root, prBodyFile, '--pr-body-file', errors);
    if (!resolved) return '';
    try {
      return fs.readFileSync(resolved.absolute, 'utf8');
    } catch (error) {
      errors.push(`--pr-body-file: cannot read ${resolved.relative} (${error.message})`);
      return '';
    }
  }
  if (typeof process.env.QMS_PR_BODY === 'string') return process.env.QMS_PR_BODY;
  if (process.env.GITHUB_EVENT_PATH) {
    const body = parsePullRequestEvent(process.env.GITHUB_EVENT_PATH, errors);
    if (body || process.env.GITHUB_EVENT_NAME === 'pull_request') return body;
  }
  if (requirePrBody) errors.push('pull request change record is required but no PR body was supplied');
  return '';
}

function fieldValue(body, label) {
  const escaped = label.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  return body.match(new RegExp(`^\\s*-?\\s*${escaped}:\\s*(.+?)\\s*$`, 'mi'))?.[1]?.trim() || '';
}

function sectionBody(body, heading) {
  const lines = body.split(/\r?\n/);
  const expected = `## ${heading}`.toLowerCase();
  const start = lines.findIndex(line => line.trim().toLowerCase() === expected);
  if (start < 0) return '';
  const collected = [];
  for (let index = start + 1; index < lines.length; index++) {
    if (/^##\s+/.test(lines[index])) break;
    collected.push(lines[index]);
  }
  return collected.join('\n');
}

function validatePullRequestBody(body, errors) {
  if (!body.trim()) {
    errors.push('pull request change record body is empty');
    return;
  }
  for (const section of PR_REQUIRED_SECTIONS) {
    if (!new RegExp(`^##\\s+${section.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\s*$`, 'mi').test(body)) {
      errors.push(`pull request change record: missing section "${section}"`);
    }
  }
  const requiredFields = [
    ['Change title', fieldValue(body, 'Change title')],
    ['Intended product version', fieldValue(body, 'Intended product version')],
    ['Risk class', fieldValue(body, 'Risk class')],
    ['Rollback strategy', fieldValue(body, 'Rollback strategy')]
  ];
  for (const [label, value] of requiredFields) {
    if (!value) errors.push(`pull request change record: ${label} is blank`);
  }
  const intendedVersion = fieldValue(body, 'Intended product version');
  if (intendedVersion && !/^[0-9]+\.[0-9]+\.[0-9]+(?:[-+][0-9A-Za-z.-]+)?$/.test(intendedVersion)) {
    errors.push('pull request change record: Intended product version must be a semantic version');
  }
  const riskClass = fieldValue(body, 'Risk class').toLowerCase();
  if (riskClass && !['low', 'medium', 'high', 'critical'].includes(riskClass)) {
    errors.push('pull request change record: Risk class must select exactly low, medium, high, or critical');
  }
  const requirementLine = fieldValue(body, 'Requirement IDs from `quality/requirements.json`');
  const administrativeJustification = fieldValue(body, 'Administrative-only `N/A` justification, if applicable');
  if (!requirementLine.match(/\bRVT-[A-Z]+-[0-9]{3}\b/)) {
    if (!/^N\/A\b/i.test(requirementLine) || !administrativeJustification || /^N\/A\b/i.test(administrativeJustification)) {
      errors.push('pull request change record: provide a requirement ID or a substantive administrative-only N/A justification');
    }
  }
  const verification = sectionBody(body, 'Verification evidence');
  const evidenceRows = verification
    .split(/\r?\n/)
    .filter(line => /^\s*\|/.test(line) && !/^\s*\|(?:\s*-+\s*\|)+\s*$/.test(line));
  const hasObjectiveResult = evidenceRows.some(line => {
    const cells = line.split('|').slice(1, -1).map(cell => cell.trim());
    return cells.length >= 2 && /^(?:pass(?:ed)?|fail(?:ed)?|blocked|skipped|external|unavailable|pending)\b/i.test(cells[1]);
  });
  if (!hasObjectiveResult) {
    errors.push('pull request change record: Verification evidence needs at least one explicit result row');
  }
  const completion = sectionBody(body, 'Completion checklist');
  if (!/- \[[xX]\]/.test(completion)) {
    errors.push('pull request change record: Completion checklist has no completed control');
  }
  if (/RVT-(?:…|â€¦|\.{3})/.test(body)) {
    errors.push('pull request change record: unresolved requirement placeholder remains');
  }
}

function resolveBaseRef(root, explicitBaseRef) {
  const candidates = [];
  const isRepositoryRoot = path.resolve(root) === REPOSITORY_ROOT;
  if (explicitBaseRef) candidates.push({ value: explicitBaseRef, strict: true });
  else if (isRepositoryRoot && process.env.QMS_BASE_REF) candidates.push({ value: process.env.QMS_BASE_REF, strict: true });
  else if (isRepositoryRoot && process.env.GITHUB_BASE_SHA) candidates.push({ value: process.env.GITHUB_BASE_SHA, strict: true });
  // CI base refs describe the checked-out repository.  Never leak them into
  // isolated fixture repositories used by the contract tests; a stack branch
  // cannot resolve inside a fresh temporary git repository.
  else if (isRepositoryRoot && process.env.GITHUB_BASE_REF) {
    candidates.push({ value: `origin/${process.env.GITHUB_BASE_REF}`, strict: false });
    candidates.push({ value: process.env.GITHUB_BASE_REF, strict: true });
  } else {
    candidates.push({ value: 'HEAD^', strict: false });
  }
  for (const candidate of candidates) {
    if (git(root, ['rev-parse', '--verify', '--quiet', `${candidate.value}^{commit}`], { optional: true })) {
      return candidate.value;
    }
    if (candidate.strict) throw new Error(`base ref does not resolve to a commit: ${candidate.value}`);
  }
  return null;
}

function jsonAtRevision(root, revision, relativePath) {
  const content = git(root, ['show', `${revision}:${relativePath}`], { optional: true });
  if (!content) return null;
  try {
    return JSON.parse(content);
  } catch {
    return null;
  }
}

function enforceControlledDocumentRevisions(root, baseRef, currentRegister, errors) {
  if (!baseRef || !currentRegister) return;
  const baseRegister = jsonAtRevision(root, baseRef, 'quality/document-register.json');
  if (!baseRegister || !Array.isArray(baseRegister.documents)) return;
  const currentById = new Map((currentRegister.documents || []).map(entry => [entry.document_id, entry]));
  const baseById = new Map(baseRegister.documents.map(entry => [entry.document_id, entry]));
  const changedPaths = new Set(
    git(root, ['diff', '--name-only', '--find-renames', baseRef, '--'])
      .split(/\r?\n/)
      .filter(Boolean)
      .map(item => item.replaceAll('\\', '/'))
  );
  let currentProductVersion = '';
  try {
    currentProductVersion = String(JSON.parse(fs.readFileSync(path.join(root, 'package.json'), 'utf8')).version || '');
  } catch {
    errors.push('package.json: cannot determine current product version for controlled-document changes');
  }
  for (const [documentId, oldEntry] of baseById) {
    const current = currentById.get(documentId);
    if (!current) {
      errors.push(`${documentId}: changed controlled document was removed from the register; retain it with obsolete status`);
      continue;
    }
    const oldPath = String(oldEntry.path || '').replaceAll('\\', '/');
    const currentPath = String(current.path || '').replaceAll('\\', '/');
    const oldMetadata = { ...oldEntry };
    const currentMetadata = { ...current };
    delete oldMetadata.revision;
    delete oldMetadata.effective_product_version;
    delete currentMetadata.revision;
    delete currentMetadata.effective_product_version;
    const metadataChanged = JSON.stringify(oldMetadata) !== JSON.stringify(currentMetadata);
    const controlledContentChanged = changedPaths.has(oldPath) || changedPaths.has(currentPath);
    if (!metadataChanged && !controlledContentChanged) continue;
    const oldRevision = REVISION_PATTERN.exec(String(oldEntry.revision || ''));
    const newRevision = REVISION_PATTERN.exec(String(current.revision || ''));
    if (!oldRevision || !newRevision || Number(newRevision[1]) !== Number(oldRevision[1]) + 1) {
      errors.push(`${documentId}: controlled document ${oldPath} changed without exactly one RNN revision increment (including rename or metadata changes)`);
    }
    if (currentProductVersion && current.effective_product_version !== currentProductVersion) {
      errors.push(
        `${documentId}: changed controlled document must set effective_product_version to ${currentProductVersion}`
      );
    }
  }
  for (const [documentId, current] of currentById) {
    if (baseById.has(documentId)) continue;
    if (String(current.revision) !== 'R01') {
      errors.push(`${documentId}: newly controlled document must begin at revision R01`);
    }
    if (currentProductVersion && current.effective_product_version !== currentProductVersion) {
      errors.push(`${documentId}: newly controlled document must set effective_product_version to ${currentProductVersion}`);
    }
  }
}

function enforceHandoffPerCommit(root, baseRef, errors) {
  if (!baseRef) return 0;
  const commits = git(root, ['rev-list', '--reverse', `${baseRef}..HEAD`], { optional: true })
    .split(/\r?\n/)
    .filter(Boolean);
  for (const commit of commits) {
    const parents = git(root, ['rev-list', '--parents', '-n', '1', commit]).split(/\s+/).slice(1);
    const args = parents.length
      ? ['diff', '--name-only', parents[0], commit, '--']
      : ['diff-tree', '--root', '--no-commit-id', '--name-only', '-r', commit];
    const files = new Set(git(root, args).split(/\r?\n/).filter(Boolean));
    if (!files.has('HANDOFF.md')) {
      errors.push(`commit ${commit.slice(0, 12)} does not update HANDOFF.md`);
    }
  }
  return commits.length;
}

export function runQmsContract({
  root = process.cwd(),
  baseRef,
  prBody,
  prBodyFile,
  requirePrBody = false
} = {}) {
  root = path.resolve(root);
  const errors = [];
  const warnings = [];
  const repositoryRoot = path.resolve(root) === REPOSITORY_ROOT;
  const requiredSchemas = repositoryRoot ? [...REQUIRED_SCHEMAS, ...RESEARCH_SCHEMAS] : REQUIRED_SCHEMAS;
  const instanceSchemas = repositoryRoot
    ? new Map([...INSTANCE_SCHEMAS, ['quality/statistical-analysis-plan.json', 'quality/schemas/statistical-analysis-plan.schema.json']])
    : INSTANCE_SCHEMAS;
  for (const schemaPath of requiredSchemas) {
    const schema = readJson(root, schemaPath, errors);
    if (schema && schema.type !== 'object') errors.push(`${schemaPath}: root schema type must be object`);
  }
  const instances = {};
  for (const [instancePath, schemaPath] of instanceSchemas) {
    const instance = readJson(root, instancePath, errors);
    const schema = readJson(root, schemaPath, errors);
    instances[instancePath] = instance;
    if (instance) {
      const declared = String(instance.$schema || '').replaceAll('\\', '/');
      const expected = path.posix.relative(path.posix.dirname(instancePath), schemaPath);
      if (!declared || path.posix.normalize(declared.replace(/^\.\//, '')) !== path.posix.normalize(expected)) {
        errors.push(`${instancePath}: $schema must reference ${expected}`);
      }
    }
    if (instance && schema) validateSchemaValue(instance, schema, instancePath, errors, schema);
  }
  validateQualityReferences(
    root,
    instances['quality/document-register.json'],
    instances['quality/requirements.json'],
    errors
  );
  const pullRequestBody = resolvePullRequestBody({ root, prBody, prBodyFile, requirePrBody }, errors);
  if (pullRequestBody || requirePrBody) validatePullRequestBody(pullRequestBody, errors);
  let resolvedBase = null;
  try {
    resolvedBase = resolveBaseRef(root, baseRef);
  } catch (error) {
    errors.push(error.message);
  }
  if (!resolvedBase) warnings.push('No parent/base commit is available; commit-level QMS checks were skipped.');
  enforceControlledDocumentRevisions(
    root,
    resolvedBase,
    instances['quality/document-register.json'],
    errors
  );
  const checkedCommits = enforceHandoffPerCommit(root, resolvedBase, errors);
  return {
    ok: errors.length === 0,
    errors,
    warnings,
    summary: {
      documents: instances['quality/document-register.json']?.documents?.length || 0,
      requirements: instances['quality/requirements.json']?.requirements?.length || 0,
      checked_commits: checkedCommits,
      base_ref: resolvedBase
    }
  };
}

function parseArgs(argv) {
  const args = {};
  for (let index = 0; index < argv.length; index++) {
    if (argv[index] === '--base-ref') args.baseRef = argv[++index];
    else if (argv[index] === '--pr-body-file') args.prBodyFile = argv[++index];
    else if (argv[index] === '--require-pr-body') args.requirePrBody = true;
    else throw new Error(`unknown argument: ${argv[index]}`);
  }
  return args;
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  try {
    const result = runQmsContract(parseArgs(process.argv.slice(2)));
    for (const warning of result.warnings) console.warn(`QMS warning: ${warning}`);
    if (!result.ok) {
      for (const error of result.errors) console.error(`QMS error: ${error}`);
      process.exitCode = 1;
    } else {
      const commitLabel = result.summary.checked_commits === 1 ? 'commit' : 'commits';
      console.log(
        `QMS contract OK: ${result.summary.documents} documents, ` +
        `${result.summary.requirements} requirements, ` +
        `${result.summary.checked_commits} ${commitLabel} checked.`
      );
    }
  } catch (error) {
    console.error(`QMS error: ${error.message}`);
    process.exitCode = 1;
  }
}
