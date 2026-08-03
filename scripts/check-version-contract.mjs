import { existsSync, readFileSync, readdirSync } from 'node:fs';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const failures = [];

function read(relativePath) {
  return readFileSync(path.join(ROOT, relativePath), 'utf8');
}

function json(relativePath) {
  return JSON.parse(read(relativePath));
}

function check(label, condition, detail) {
  if (!condition) failures.push(`${label}: ${detail}`);
}

function equal(label, actual, expected) {
  check(label, actual === expected, `expected ${JSON.stringify(expected)}, got ${JSON.stringify(actual)}`);
}

function contains(label, relativePath, expected) {
  const value = read(relativePath);
  check(label, value.includes(expected), `${relativePath} is missing ${JSON.stringify(expected)}`);
}

const pyproject = read('pyproject.toml');
const projectBlock = pyproject.match(/\[project\]([\s\S]*?)(?=\n\[|$)/)?.[1] ?? '';
const version = projectBlock.match(/^\s*version\s*=\s*"(\d+\.\d+\.\d+)"\s*$/m)?.[1];
check('authoritative version', Boolean(version), 'pyproject.toml [project].version is not strict X.Y.Z semver');

if (version) {
  const [major, minor, patch] = version.split('.');
  const short = `${major}.${minor}`;
  const firmwareFile = `radar_vital_v${major}_${minor}_${patch}.ino`;
  const retiredFirmwareFiles = readdirSync(ROOT).filter(
    name => /^radar_vital_v\d+_\d+_\d+\.ino$/.test(name) && name !== firmwareFile,
  );

  const jsonCarriers = [
    ['root package', 'package.json', ['version']],
    ['root lock root version', 'package-lock.json', ['version']],
    ['root lock package version', 'package-lock.json', ['packages', '', 'version']],
    ['web package', 'web/package.json', ['version']],
    ['web lock root version', 'web/package-lock.json', ['version']],
    ['web lock package version', 'web/package-lock.json', ['packages', '', 'version']],
    ['Capacitor package', 'packaging/capacitor/package.json', ['version']],
    ['Capacitor lock root version', 'packaging/capacitor/package-lock.json', ['version']],
    ['Capacitor lock package version', 'packaging/capacitor/package-lock.json', ['packages', '', 'version']],
    ['Tauri config', 'src-tauri/tauri.conf.json', ['version']],
    ['packaging Tauri config', 'packaging/tauri/tauri.conf.json', ['version']],
  ];

  for (const [label, relativePath, keys] of jsonCarriers) {
    let value = json(relativePath);
    for (const key of keys) value = value?.[key];
    equal(label, value, version);
  }

  const cargoToml = read('src-tauri/Cargo.toml');
  const cargoPackage = cargoToml.match(/\[package\]([\s\S]*?)(?=\n\[|$)/)?.[1] ?? '';
  equal('Cargo package version', cargoPackage.match(/^\s*version\s*=\s*"([^"]+)"/m)?.[1], version);

  const cargoLock = read('src-tauri/Cargo.lock');
  const ownCargoLockBlock = [...cargoLock.matchAll(/\[\[package\]\]\r?\n([\s\S]*?)(?=\r?\n\[\[package\]\]|$)/g)]
    .map((match) => match[1])
    .find((block) => /^name = "radar-vital"$/m.test(block));
  equal('Cargo lock package version', ownCargoLockBlock?.match(/^version = "([^"]+)"$/m)?.[1], version);

  contains('trainer version', 'rvt_trainer/monolith.py', `VERSION = "${version}"`);
  contains('dashboard version', 'rvt_trainer/monolith.py', `DASHBOARD_VERSION = "${version}"`);
  contains('expected firmware version', 'rvt_trainer/monolith.py', `FIRMWARE_VERSION_EXPECTED = "v${version}"`);
  contains('trainer firmware path', 'rvt_trainer/monolith.py', firmwareFile);

  contains('Angular product version', 'web/src/app/services/app-meta.ts', `PRODUCT_VERSION = '${version}'`);
  contains('Angular short version', 'web/src/app/services/app-meta.ts', `PRODUCT_VERSION_SHORT = 'v${short}'`);
  contains('Angular label version', 'web/src/app/services/app-meta.ts', `PRODUCT_VERSION_LABEL = 'App v${short}'`);
  contains(
    'built dashboard product version',
    'radar_vital_live_dashboard_v12_for_v16_0.html',
    `="${version}",`,
  );
  contains('demo firmware version', 'web/src/app/services/telemetry.service.ts', `version: 'v${version}-demo'`);

  check('current firmware source', existsSync(path.join(ROOT, firmwareFile)), `${firmwareFile} does not exist`);
  check(
    'retired firmware sources',
    retiredFirmwareFiles.length === 0,
    `${retiredFirmwareFiles.join(', ')} must not coexist with the current source`,
  );
  contains('firmware source self-name', firmwareFile, firmwareFile);
  contains('firmware version macro', firmwareFile, `#define FW_VERSION "v${version}"`);
  contains('firmware major macro', firmwareFile, `#define SKETCH_VERSION_MAJOR ${major}`);
  contains('firmware minor macro', firmwareFile, `#define SKETCH_VERSION_SUB ${minor}`);
  contains('firmware patch macro', firmwareFile, `#define SKETCH_VERSION_MOD ${patch}`);

  contains('firmware audit guidance', 'rvt_trainer/audit/runner.py', firmwareFile);
  contains('operator help firmware version', 'rvt_trainer/assets/help_schema.json', `v${version} firmware`);
  contains('issue template version', '.github/ISSUE_TEMPLATE/bug_report.yml', `placeholder: "${version}"`);
  contains('README firmware path', 'README.md', firmwareFile);
  contains('agent firmware path', 'AGENTS.md', firmwareFile);

  check(
    'trainer compatibility shim',
    existsSync(path.join(ROOT, 'radar_vital_trainer_v12_for_v16_0.py')),
    'the stable v12/v16 trainer shim was renamed or removed',
  );
  check(
    'dashboard compatibility artifact',
    existsSync(path.join(ROOT, 'radar_vital_live_dashboard_v12_for_v16_0.html')),
    'the stable v12/v16 dashboard artifact was renamed or removed',
  );
}

if (failures.length) {
  console.error(`Version contract failed (${failures.length}):`);
  for (const failure of failures) console.error(`- ${failure}`);
  process.exitCode = 1;
} else {
  console.log(`Version contract valid: ${version}`);
}
