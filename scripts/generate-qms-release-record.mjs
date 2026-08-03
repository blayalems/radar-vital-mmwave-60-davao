import crypto from 'node:crypto';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT_DIR = path.resolve(__dirname, '..');
const DEFAULT_REPOSITORY = 'blayalems/radar-vital-mmwave-60-davao';
const REGISTER_PATH = path.join(ROOT_DIR, 'quality', 'document-register.json');
const DOCUMENT_REGISTER_SCHEMA_PATH = path.join(
  ROOT_DIR,
  'quality',
  'schemas',
  'document-register.schema.json'
);
const RELEASE_SCHEMA_PATH = path.join(ROOT_DIR, 'quality', 'schemas', 'release-record.schema.json');
const RELEASE_FILES = [
  'radar-vital-release.apk',
  'radar-vital-debug.apk',
  'radar-vital-release.aab',
  'radar-vital-windows-installer.exe',
  'radar-vital-windows-installer.exe.sig',
  'rvt-latest.json',
  'rvt-latest-tauri.json',
  'controlled-document-revisions.json'
];

function readArg(name) {
  const index = process.argv.indexOf(name);
  if (index >= 0 && index + 1 < process.argv.length) return process.argv[index + 1];
  const prefix = `${name}=`;
  const match = process.argv.find(value => value.startsWith(prefix));
  return match ? match.slice(prefix.length) : '';
}

function sha256(buffer) {
  return crypto.createHash('sha256').update(buffer).digest('hex');
}

function fileEvidence(filePath, relativeName) {
  const bytes = fs.readFileSync(filePath);
  return { name: relativeName, size_bytes: bytes.length, sha256: sha256(bytes) };
}

function productVersion() {
  return JSON.parse(fs.readFileSync(path.join(ROOT_DIR, 'package.json'), 'utf8')).version;
}

function tagProductVersion(releaseTag) {
  return releaseTag.slice(1).split(/[+-]/, 1)[0];
}

function signingState(fileName, options) {
  if (fileName === 'radar-vital-debug.apk') return 'unsigned';
  if (fileName === 'radar-vital-release.apk' || fileName === 'radar-vital-release.aab') {
    if (options.androidSignatureState === 'signed_release') return 'signed';
    if (options.androidSignatureState === 'unsigned_debug_fallback') return 'unsigned';
    return 'unknown';
  }
  if (fileName === 'radar-vital-windows-installer.exe') {
    if (options.windowsSignatureState === 'authenticode_verified') return 'signed';
    if (options.windowsSignatureState === 'not_verified_notsigned') return 'unsigned';
    return 'unknown';
  }
  return 'not_applicable';
}

function artifactKind(fileName) {
  if (fileName.endsWith('.apk')) return 'android_apk';
  if (fileName.endsWith('.aab')) return 'android_app_bundle';
  if (fileName.endsWith('.exe')) return 'windows_installer';
  if (fileName.endsWith('.sig')) return 'tauri_updater_signature';
  if (fileName === 'controlled-document-revisions.json') return 'controlled_document_revision_snapshot';
  return 'release_metadata';
}

function buildOptions() {
  const version = productVersion();
  const releaseTag = readArg('--release-tag') || process.env.RELEASE_TAG || `v${version}`;
  return {
    version,
    releaseTag,
    releaseVersion: readArg('--release-version') || process.env.RELEASE_VERSION || releaseTag.replace(/^v/, ''),
    distDir: path.resolve(readArg('--dist') || path.join(ROOT_DIR, 'dist')),
    repository: readArg('--repo') || process.env.GITHUB_REPOSITORY || DEFAULT_REPOSITORY,
    releasedAt: readArg('--released-at') || process.env.RELEASED_AT || new Date().toISOString(),
    sourceCommit: readArg('--source-commit') || process.env.GITHUB_SHA || '',
    sourceRef: readArg('--source-ref') || process.env.GITHUB_REF || '',
    workflowName: process.env.GITHUB_WORKFLOW || 'Release APK and EXE',
    workflowRunId: readArg('--workflow-run-id') || process.env.GITHUB_RUN_ID || '',
    workflowRunAttempt: readArg('--workflow-run-attempt') || process.env.GITHUB_RUN_ATTEMPT || '',
    androidSignatureState: process.env.ANDROID_SIGNATURE_STATE || 'unknown',
    windowsSignatureState: process.env.WINDOWS_SIGNATURE_STATE || 'unknown',
    authorizationState: process.env.QMS_AUTHORIZATION_STATE || 'pending',
    authorizedBy: process.env.QMS_AUTHORIZED_BY || null,
    authorizedAt: process.env.QMS_AUTHORIZED_AT || null,
    previousRelease: process.env.PREVIOUS_RELEASE_TAG || null,
    documentRegisterPath: path.resolve(readArg('--document-register') || REGISTER_PATH),
    documentRegisterSchemaPath: path.resolve(
      readArg('--document-register-schema') || DOCUMENT_REGISTER_SCHEMA_PATH
    ),
    releaseSchemaPath: RELEASE_SCHEMA_PATH
  };
}

function localRef(schema, reference) {
  if (!reference.startsWith('#/')) return null;
  return reference.slice(2).split('/').reduce(
    (value, key) => value?.[key.replaceAll('~1', '/').replaceAll('~0', '~')],
    schema
  );
}

function typeMatches(value, type) {
  if (type === 'null') return value === null;
  if (type === 'array') return Array.isArray(value);
  if (type === 'object') return value !== null && typeof value === 'object' && !Array.isArray(value);
  if (type === 'integer') return Number.isInteger(value);
  return typeof value === type;
}

function validateAgainstSchema(value, node, rootSchema, location, errors) {
  if (node.$ref) {
    const target = localRef(rootSchema, node.$ref);
    if (!target) errors.push(`${location}: unresolved schema reference ${node.$ref}`);
    else validateAgainstSchema(value, target, rootSchema, location, errors);
    return;
  }
  for (const candidate of node.allOf || []) {
    validateAgainstSchema(value, candidate, rootSchema, location, errors);
  }
  if (node.not) {
    const forbiddenErrors = [];
    validateAgainstSchema(value, node.not, rootSchema, location, forbiddenErrors);
    if (forbiddenErrors.length === 0) errors.push(`${location}: matches a forbidden schema`);
  }
  if (node.oneOf) {
    const matches = node.oneOf.filter(candidate => {
      const nested = [];
      validateAgainstSchema(value, candidate, rootSchema, location, nested);
      return nested.length === 0;
    });
    if (matches.length !== 1) errors.push(`${location}: must match exactly one allowed schema`);
  }
  if (node.anyOf) {
    const valid = node.anyOf.some(candidate => {
      const nested = [];
      validateAgainstSchema(value, candidate, rootSchema, location, nested);
      return nested.length === 0;
    });
    if (!valid) errors.push(`${location}: does not match any allowed schema`);
  }
  if (node.const !== undefined && value !== node.const) errors.push(`${location}: const mismatch`);
  if (node.enum && !node.enum.includes(value)) errors.push(`${location}: enum mismatch`);
  if (node.type) {
    const allowed = Array.isArray(node.type) ? node.type : [node.type];
    if (!allowed.some(type => typeMatches(value, type))) {
      errors.push(`${location}: expected ${allowed.join(' or ')}`);
      return;
    }
  }
  if (typeof value === 'string') {
    if (node.minLength !== undefined && value.length < node.minLength) errors.push(`${location}: too short`);
    if (node.pattern && !new RegExp(node.pattern).test(value)) errors.push(`${location}: pattern mismatch`);
    if (node.format === 'date-time' && Number.isNaN(Date.parse(value))) errors.push(`${location}: invalid date-time`);
  }
  if (typeof value === 'number' && node.minimum !== undefined && value < node.minimum) {
    errors.push(`${location}: below minimum`);
  }
  if (typeof value === 'number' && node.maximum !== undefined && value > node.maximum) {
    errors.push(`${location}: above maximum`);
  }
  if (Array.isArray(value)) {
    if (node.minItems !== undefined && value.length < node.minItems) errors.push(`${location}: too few items`);
    if (node.uniqueItems) {
      const keys = value.map(item => JSON.stringify(item));
      if (new Set(keys).size !== keys.length) errors.push(`${location}: items must be unique`);
    }
    if (node.items) value.forEach((item, index) =>
      validateAgainstSchema(item, node.items, rootSchema, `${location}[${index}]`, errors)
    );
  } else if (value !== null && typeof value === 'object') {
    for (const required of node.required || []) {
      if (!Object.prototype.hasOwnProperty.call(value, required)) errors.push(`${location}: missing ${required}`);
    }
    for (const [key, child] of Object.entries(node.properties || {})) {
      if (Object.prototype.hasOwnProperty.call(value, key)) {
        validateAgainstSchema(value[key], child, rootSchema, `${location}.${key}`, errors);
      }
    }
    if (node.additionalProperties === false) {
      const allowed = new Set(Object.keys(node.properties || {}));
      for (const key of Object.keys(value)) if (!allowed.has(key)) errors.push(`${location}: unexpected ${key}`);
    }
  }
}

function readAndValidateDocumentRegister(options) {
  const registerBytes = fs.readFileSync(options.documentRegisterPath);
  const register = JSON.parse(registerBytes.toString('utf8'));
  const schema = JSON.parse(fs.readFileSync(options.documentRegisterSchemaPath, 'utf8'));
  const errors = [];
  validateAgainstSchema(register, schema, schema, 'quality/document-register.json', errors);
  if (errors.length) {
    throw new Error(`QMS document register failed schema validation: ${errors.join('; ')}`);
  }
  return { registerBytes, register };
}

function controlledDocumentSnapshot(options) {
  const { registerBytes, register } = readAndValidateDocumentRegister(options);
  const documents = (register.documents || []).filter(item => item.status === 'active').map(item => {
    const relativePath = String(item.path).replaceAll('\\', '/');
    const absolutePath = path.resolve(ROOT_DIR, ...relativePath.split('/'));
    if (absolutePath !== ROOT_DIR && !absolutePath.startsWith(`${ROOT_DIR}${path.sep}`)) {
      throw new Error(`Controlled document path escapes repository: ${relativePath}`);
    }
    const evidence = fileEvidence(absolutePath, relativePath);
    return {
      document_id: item.document_id,
      revision: item.revision,
      path: relativePath,
      sha256: evidence.sha256,
      size_bytes: evidence.size_bytes,
      effective_product_version: item.effective_product_version,
      status: item.status
    };
  });
  if (!documents.length) throw new Error('Document register has no active controlled documents');
  return {
    schema_version: 'rvt-controlled-document-revision-snapshot-v1',
    register: {
      schema_version: register.schema_version,
      register_id: register.register_id,
      register_revision: register.register_revision,
      effective_product_version: register.effective_product_version,
      status: register.status,
      sha256: sha256(registerBytes)
    },
    documents
  };
}

export function generateQmsReleaseRecord(options = buildOptions()) {
  if (!/^v[0-9]+\.[0-9]+\.[0-9]+(?:[-+][0-9A-Za-z.-]+)?$/.test(options.releaseTag)) {
    throw new Error(`Release tag must start with v and carry a semantic version: ${options.releaseTag}`);
  }
  if (tagProductVersion(options.releaseTag) !== options.version) {
    throw new Error(
      `Release tag product version ${tagProductVersion(options.releaseTag)} does not match package product version ${options.version}`
    );
  }
  if (!/^[0-9a-f]{40}$/.test(options.sourceCommit)) throw new Error('Source commit must be a 40-character lowercase SHA');
  if (!options.sourceRef) throw new Error('Source ref is required');
  if (!options.workflowRunId || !/^\d+$/.test(String(options.workflowRunAttempt))) {
    throw new Error('Workflow run id and numeric run attempt are required');
  }
  if (!['pending', 'authorized', 'rejected'].includes(options.authorizationState)) {
    throw new Error(`Unsupported release authorization state: ${options.authorizationState}`);
  }
  if (options.authorizationState === 'authorized' || options.authorizationState === 'rejected') {
    if (!options.authorizedBy || !options.authorizedAt || Number.isNaN(Date.parse(options.authorizedAt))) {
      throw new Error(`${options.authorizationState} release requires an authorizer and ISO authorization time`);
    }
  } else if (options.authorizedBy || options.authorizedAt) {
    throw new Error(`${options.authorizationState} release must not claim authorization identity or time`);
  }
  const { registerBytes, register } = readAndValidateDocumentRegister(options);
  const runUrl = `https://github.com/${options.repository}/actions/runs/${options.workflowRunId}`;
  const releaseUrl = `https://github.com/${options.repository}/releases/download/${options.releaseTag}`;
  const artifacts = RELEASE_FILES.filter(name => fs.existsSync(path.join(options.distDir, name))).map(name => ({
    ...fileEvidence(path.join(options.distDir, name), name),
    kind: artifactKind(name),
    signing_state: signingState(name, options)
  }));
  if (!artifacts.some(item => item.name.endsWith('.apk'))) throw new Error('QMS record requires an APK artifact');
  if (!artifacts.some(item => item.name.endsWith('.exe'))) throw new Error('QMS record requires a Windows EXE artifact');
  const documentSnapshot = controlledDocumentSnapshot(options);
  const record = {
    schema_version: 'rvt-qms-release-record-v1',
    release_id: `RVT-REL-${options.releaseVersion}`,
    product_version: options.version,
    release_tag: options.releaseTag,
    source: {
      repository: options.repository,
      commit_sha: options.sourceCommit,
      ref: options.sourceRef
    },
    build: {
      workflow: options.workflowName,
      run_id: String(options.workflowRunId),
      run_attempt: Number(options.workflowRunAttempt),
      builder: 'github-actions',
      built_at: options.releasedAt
    },
    controlled_documents: {
      register_id: register.register_id,
      register_revision: register.register_revision,
      sha256: sha256(registerBytes),
      documents: documentSnapshot.documents.map(document => ({
        document_id: document.document_id,
        path: document.path,
        revision: document.revision,
        effective_product_version: document.effective_product_version,
        sha256: document.sha256
      }))
    },
    verification: [
      { check_id: 'release-artifact-integrity', conclusion: 'passed', evidence_url: `${releaseUrl}/SHA256SUMS` },
      { check_id: 'controlled-document-revision-snapshot', conclusion: 'passed', evidence_url: `${releaseUrl}/controlled-document-revisions.json` },
      { check_id: 'github-build-provenance-attestation', conclusion: 'external_gate', evidence_url: runUrl }
    ],
    artifacts,
    authorization: {
      state: options.authorizationState,
      authorized_by: options.authorizedBy,
      authorized_at: options.authorizedAt
    },
    rollback: {
      strategy: 'Publish a superseding release and restore the last accepted release assets and manifests.',
      previous_release: options.previousRelease
    }
  };
  const schema = JSON.parse(fs.readFileSync(options.releaseSchemaPath, 'utf8'));
  const errors = [];
  validateAgainstSchema(record, schema, schema, '$', errors);
  if (errors.length) throw new Error(`QMS release record failed schema validation: ${errors.join('; ')}`);
  return record;
}

export function validateQmsReleaseRecordFile(recordPath, schemaPath = RELEASE_SCHEMA_PATH) {
  const record = JSON.parse(fs.readFileSync(path.resolve(recordPath), 'utf8'));
  const schema = JSON.parse(fs.readFileSync(path.resolve(schemaPath), 'utf8'));
  const errors = [];
  validateAgainstSchema(record, schema, schema, '$', errors);
  if (errors.length) throw new Error(`QMS release record failed schema validation: ${errors.join('; ')}`);
  if (record.authorization.state === 'authorized' || record.authorization.state === 'rejected') {
    if (!record.authorization.authorized_by || !record.authorization.authorized_at) {
      throw new Error(`${record.authorization.state} QMS release record is missing authorizer evidence`);
    }
  } else if (record.authorization.authorized_by !== null || record.authorization.authorized_at !== null) {
    throw new Error(`${record.authorization.state} QMS release record must not claim authorizer evidence`);
  }
  return record;
}

export function writeReleaseEvidence(options = buildOptions()) {
  fs.mkdirSync(options.distDir, { recursive: true });
  const snapshotPath = path.join(options.distDir, 'controlled-document-revisions.json');
  fs.writeFileSync(snapshotPath, `${JSON.stringify(controlledDocumentSnapshot(options), null, 2)}\n`);
  const record = generateQmsReleaseRecord(options);
  const recordPath = path.join(options.distDir, 'qms-release-record.json');
  fs.writeFileSync(recordPath, `${JSON.stringify(record, null, 2)}\n`);
  const checksumNames = [...record.artifacts.map(item => item.name), 'qms-release-record.json']
    .filter((value, index, values) => values.indexOf(value) === index)
    .sort();
  const sumsPath = path.join(options.distDir, 'SHA256SUMS');
  fs.writeFileSync(
    sumsPath,
    `${checksumNames.map(name => `${sha256(fs.readFileSync(path.join(options.distDir, name)))}  ${name}`).join('\n')}\n`
  );
  return { record, recordPath, snapshotPath, sumsPath };
}

function runSelfTest() {
  const tempDir = fs.mkdtempSync(path.join(os.tmpdir(), 'rvt-qms-release-'));
  const qualityDir = fs.mkdtempSync(path.join(os.tmpdir(), 'rvt-qms-register-'));
  try {
    fs.writeFileSync(path.join(tempDir, 'radar-vital-release.apk'), 'apk');
    fs.writeFileSync(path.join(tempDir, 'radar-vital-windows-installer.exe'), 'exe');
    const registerPath = path.join(qualityDir, 'document-register.json');
    fs.writeFileSync(registerPath, JSON.stringify({
      $schema: './schemas/document-register.schema.json',
      schema_version: 'rvt-qms-document-register-v1',
      register_id: 'RVT-QMS-REG-001',
      register_revision: 'R01',
      effective_product_version: productVersion(),
      status: 'active',
      owner_role: 'quality_manager',
      approval_method: 'protected_pull_request',
      documents: [{
        document_id: 'RVT-QMS-TST-001',
        title: 'QMS release generator self-test document',
        record_type: 'record',
        revision: 'R01',
        path: 'AGENTS.md',
        status: 'active',
        owner_role: 'quality_manager',
        approver_roles: ['quality_manager'],
        effective_product_version: productVersion(),
        supersedes: null,
        retention: {
          class: 'project_lifetime',
          disposition: 'review_before_disposal'
        },
        confidentiality: 'internal',
        review_interval_months: 12
      }]
    }));
    const version = productVersion();
    const result = writeReleaseEvidence({
      ...buildOptions(),
      version,
      releaseTag: `v${version}-main.99`,
      releaseVersion: `${version}-main.99`,
      distDir: tempDir,
      repository: DEFAULT_REPOSITORY,
      releasedAt: '2026-07-29T00:00:00Z',
      sourceCommit: '0123456789abcdef0123456789abcdef01234567',
      sourceRef: 'refs/tags/test',
      workflowRunId: '99',
      workflowRunAttempt: '2',
      androidSignatureState: 'signed_release',
      windowsSignatureState: 'authenticode_verified',
      documentRegisterPath: registerPath
    });
    if (!result.record.controlled_documents.sha256) throw new Error('register hash missing');
    if (!fs.readFileSync(result.sumsPath, 'utf8').includes('qms-release-record.json')) throw new Error('record checksum missing');
    console.log('QMS release record self-test passed.');
  } finally {
    fs.rmSync(tempDir, { recursive: true, force: true });
    fs.rmSync(qualityDir, { recursive: true, force: true });
  }
}

if (process.argv.includes('--verify-record')) {
  try {
    const recordPath = readArg('--verify-record');
    if (!recordPath) throw new Error('--verify-record requires a JSON path');
    validateQmsReleaseRecordFile(recordPath, readArg('--schema') || RELEASE_SCHEMA_PATH);
    console.log(`QMS release record schema valid: ${path.resolve(recordPath)}`);
  } catch (error) {
    console.error(error);
    process.exitCode = 1;
  }
} else if (process.argv.includes('--self-test')) {
  try {
    runSelfTest();
  } catch (error) {
    console.error(error);
    process.exitCode = 1;
  }
} else if (url.fileURLToPath(import.meta.url) === path.resolve(process.argv[1] || '')) {
  try {
    const result = writeReleaseEvidence();
    console.log(`Generated ${result.recordPath}`);
    console.log(`Generated ${result.snapshotPath}`);
    console.log(`Generated ${result.sumsPath}`);
  } catch (error) {
    console.error(error);
    process.exitCode = 1;
  }
}
