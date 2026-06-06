import fs from 'node:fs';
import path from 'node:path';
import url from 'node:url';
import crypto from 'node:crypto';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT_DIR = path.resolve(__dirname, '..');
const PACKAGE_JSON_PATH = path.join(ROOT_DIR, 'package.json');

// Read version from root package.json
const packageJson = JSON.parse(fs.readFileSync(PACKAGE_JSON_PATH, 'utf8'));
const version = packageJson.version;
const DEFAULT_REPOSITORY = 'blayalems/radar-vital-mmwave-60-davao';

function readArg(name) {
  const idx = process.argv.indexOf(name);
  if (idx >= 0 && idx + 1 < process.argv.length) {
    return process.argv[idx + 1];
  }
  const prefix = `${name}=`;
  const match = process.argv.find(arg => arg.startsWith(prefix));
  return match ? match.slice(prefix.length) : '';
}

function buildOptions(prodVersion) {
  const releaseTag = readArg('--release-tag') || process.env.RELEASE_TAG || `v${prodVersion}`;
  const repository = readArg('--repo') || process.env.GITHUB_REPOSITORY || DEFAULT_REPOSITORY;
  const releasedAt = readArg('--released-at') || process.env.RELEASED_AT || new Date().toISOString();
  const releaseVersion = readArg('--release-version') || process.env.RELEASE_VERSION || releaseTag.replace(/^v/, '');
  const androidVersionCodeRaw = readArg('--android-version-code') || process.env.ANDROID_VERSION_CODE || '';
  const androidVersionCode = /^\d+$/.test(androidVersionCodeRaw) ? Number(androidVersionCodeRaw) : null;
  const buildNumberRaw = readArg('--build-number') || process.env.BUILD_NUMBER || androidVersionCodeRaw || '';
  const buildNumber = /^\d+$/.test(buildNumberRaw) ? Number(buildNumberRaw) : null;
  return { releaseTag, repository, releasedAt, releaseVersion, androidVersionCode, buildNumber };
}

// Helper to calculate file size and SHA-256 hash
function getFileStats(filePath) {
  const stats = fs.statSync(filePath);
  const size = stats.size;
  const fileBuffer = fs.readFileSync(filePath);
  const sha256 = crypto.createHash('sha256').update(fileBuffer).digest('hex');
  return { size, sha256 };
}

function artifactMetadata({ kind, platform, fileName, filePath, compatibility, releaseUrl, versionCode = null }) {
  const stats = getFileStats(filePath);
  const artifact = {
    kind,
    platform,
    file_name: fileName,
    url: `${releaseUrl}/${fileName}`,
    size: stats.size,
    size_bytes: stats.size,
    sha256: stats.sha256,
    compatibility
  };
  if (versionCode !== null) {
    artifact.version_code = versionCode;
  }
  return artifact;
}

function readTauriSignature(exePath) {
  const inlineSignature = readArg('--tauri-signature') || process.env.TAURI_SIGNATURE || '';
  if (inlineSignature) return inlineSignature.trim();
  const sigPath = `${exePath}.sig`;
  if (fs.existsSync(sigPath)) {
    return fs.readFileSync(sigPath, 'utf8').trim();
  }
  return '';
}

// Generate the manifest object
function generateManifest(distDir, prodVersion, options = buildOptions(prodVersion)) {
  const apkReleasePath = path.join(distDir, 'radar-vital-release.apk');
  const apkDebugPath = path.join(distDir, 'radar-vital-debug.apk');
  const exePath = path.join(distDir, 'radar-vital-windows-installer.exe');

  let apkFilePath = '';
  let apkFileName = '';

  if (fs.existsSync(apkReleasePath)) {
    apkFilePath = apkReleasePath;
    apkFileName = 'radar-vital-release.apk';
  } else if (fs.existsSync(apkDebugPath)) {
    apkFilePath = apkDebugPath;
    apkFileName = 'radar-vital-debug.apk';
  } else {
    throw new Error(`Neither radar-vital-release.apk nor radar-vital-debug.apk was found in ${distDir}`);
  }

  if (!fs.existsSync(exePath)) {
    throw new Error(`radar-vital-windows-installer.exe was not found in ${distDir}`);
  }

  const releaseUrl = `https://github.com/${options.repository}/releases/download/${options.releaseTag}`;
  const apk = artifactMetadata({
    kind: 'apk',
    platform: 'android',
    fileName: apkFileName,
    filePath: apkFilePath,
    compatibility: 'Android 8.0+',
    releaseUrl,
    versionCode: options.androidVersionCode
  });
  const exe = artifactMetadata({
    kind: 'exe',
    platform: 'windows',
    fileName: 'radar-vital-windows-installer.exe',
    filePath: exePath,
    compatibility: 'Windows 10+',
    releaseUrl
  });
  const tauriSignature = readTauriSignature(exePath);

  return {
    product_version: prodVersion,
    release_version: options.releaseVersion,
    minimum_supported: '16.0.0',
    released_at: options.releasedAt,
    release_tag: options.releaseTag,
    build_number: options.buildNumber,
    release_url: `https://github.com/${options.repository}/releases/tag/${options.releaseTag}`,
    artifacts: {
      apk,
      exe
    },
    artifact_entries: [apk, exe],
    compatibility: {
      android: 'Android 8.0+',
      windows: 'Windows 10+',
      schema_lineage: 'rvt-v12.0',
      install_mode: 'automated_native_install_with_manual_fallback',
      manual_download_guidance: 'Use the listed APK/EXE links when native install automation is unavailable.'
    },
    tauri_updater: {
      manifest_url: 'https://blayalems.github.io/radar-vital-mmwave-60-davao/rvt-latest-tauri.json',
      signature_configured: Boolean(tauriSignature)
    }
  };
}

// Generate the Tauri v2 updater manifest object
function generateTauriManifest(distDir, prodVersion, options = buildOptions(prodVersion)) {
  const exePath = path.join(distDir, 'radar-vital-windows-installer.exe');
  if (!fs.existsSync(exePath)) {
    throw new Error(`radar-vital-windows-installer.exe was not found in ${distDir}`);
  }
  const signature = readTauriSignature(exePath);
  const releaseUrl = `https://github.com/${options.repository}/releases/download/${options.releaseTag}`;
  const platform = {
    signature: signature,
    url: `${releaseUrl}/radar-vital-windows-installer.exe`
  };
  return {
    version: prodVersion,
    notes: `Radar Vital version ${prodVersion} release.`,
    pub_date: options.releasedAt,
    platforms: {
      'windows-x86_64': platform,
      'windows-x86_64.nsis': platform
    }
  };
}

// Self-test mode validation
function runSelfTest() {
  console.log('Starting self-test...');
  const tempDir = path.join(ROOT_DIR, 'dist-temp');
  if (!fs.existsSync(tempDir)) {
    fs.mkdirSync(tempDir, { recursive: true });
  }

  const mockApkContent = 'mock apk content ' + Math.random();
  const mockExeContent = 'mock exe content ' + Math.random();
  const mockSigContent = 'mock-signature-content';

  const mockApkPath = path.join(tempDir, 'radar-vital-release.apk');
  const mockExePath = path.join(tempDir, 'radar-vital-windows-installer.exe');
  const mockSigPath = path.join(tempDir, 'radar-vital-windows-installer.exe.sig');

  fs.writeFileSync(mockApkPath, mockApkContent);
  fs.writeFileSync(mockExePath, mockExeContent);
  fs.writeFileSync(mockSigPath, mockSigContent);

  const expectedApkSize = Buffer.byteLength(mockApkContent);
  const expectedExeSize = Buffer.byteLength(mockExeContent);
  const expectedApkHash = crypto.createHash('sha256').update(mockApkContent).digest('hex');
  const expectedExeHash = crypto.createHash('sha256').update(mockExeContent).digest('hex');

  try {
    const testOptions = {
      releaseTag: `v${version}-main.999`,
      repository: DEFAULT_REPOSITORY,
      releasedAt: '2026-06-06T00:00:00.000Z',
      releaseVersion: `${version}-main.999`,
      androidVersionCode: 999,
      buildNumber: 999
    };
    const manifest = generateManifest(tempDir, version, testOptions);
    console.log('Generated manifest in self-test:', JSON.stringify(manifest, null, 2));

    // Validate Schema
    if (manifest.product_version !== version) {
      throw new Error(`product_version mismatch: expected ${version}, got ${manifest.product_version}`);
    }
    if (manifest.minimum_supported !== '16.0.0') {
      throw new Error(`minimum_supported mismatch: expected 16.0.0, got ${manifest.minimum_supported}`);
    }
    if (isNaN(Date.parse(manifest.released_at))) {
      throw new Error(`released_at is not a valid ISO date: ${manifest.released_at}`);
    }
    if (manifest.release_tag !== testOptions.releaseTag) {
      throw new Error(`release_tag mismatch: expected ${testOptions.releaseTag}, got ${manifest.release_tag}`);
    }
    if (manifest.release_version !== testOptions.releaseVersion) {
      throw new Error(`release_version mismatch: expected ${testOptions.releaseVersion}, got ${manifest.release_version}`);
    }
    if (manifest.build_number !== testOptions.buildNumber) {
      throw new Error(`build_number mismatch: expected ${testOptions.buildNumber}, got ${manifest.build_number}`);
    }
    if (manifest.release_url !== `https://github.com/${DEFAULT_REPOSITORY}/releases/tag/${testOptions.releaseTag}`) {
      throw new Error(`release_url mismatch: got ${manifest.release_url}`);
    }
    if (manifest.compatibility?.install_mode !== 'automated_native_install_with_manual_fallback') {
      throw new Error(`install_mode mismatch: got ${manifest.compatibility?.install_mode}`);
    }
    if (!manifest.tauri_updater?.signature_configured) {
      throw new Error('Expected tauri_updater.signature_configured in manifest self-test');
    }

    const apk = manifest.artifacts?.apk;
    const exe = manifest.artifacts?.exe;

    if (!apk || !exe) {
      throw new Error('Missing apk or exe in artifacts');
    }
    if (!Array.isArray(manifest.artifact_entries) || manifest.artifact_entries.length !== 2) {
      throw new Error('Missing artifact_entries array');
    }

    if (apk.url !== `https://github.com/${DEFAULT_REPOSITORY}/releases/download/${testOptions.releaseTag}/radar-vital-release.apk`) {
      throw new Error(`APK url mismatch: got ${apk.url}`);
    }
    if (apk.size !== expectedApkSize) {
      throw new Error(`APK size mismatch: expected ${expectedApkSize}, got ${apk.size}`);
    }
    if (apk.size_bytes !== expectedApkSize) {
      throw new Error(`APK size_bytes mismatch: expected ${expectedApkSize}, got ${apk.size_bytes}`);
    }
    if (apk.version_code !== testOptions.androidVersionCode) {
      throw new Error(`APK version_code mismatch: expected ${testOptions.androidVersionCode}, got ${apk.version_code}`);
    }
    if (apk.sha256 !== expectedApkHash) {
      throw new Error(`APK sha256 mismatch: expected ${expectedApkHash}, got ${apk.sha256}`);
    }
    if (apk.compatibility !== 'Android 8.0+') {
      throw new Error(`APK compatibility mismatch: got ${apk.compatibility}`);
    }

    if (exe.url !== `https://github.com/${DEFAULT_REPOSITORY}/releases/download/${testOptions.releaseTag}/radar-vital-windows-installer.exe`) {
      throw new Error(`EXE url mismatch: got ${exe.url}`);
    }
    if (exe.size !== expectedExeSize) {
      throw new Error(`EXE size mismatch: expected ${expectedExeSize}, got ${exe.size}`);
    }
    if (exe.size_bytes !== expectedExeSize) {
      throw new Error(`EXE size_bytes mismatch: expected ${expectedExeSize}, got ${exe.size_bytes}`);
    }
    if (exe.sha256 !== expectedExeHash) {
      throw new Error(`EXE sha256 mismatch: expected ${expectedExeHash}, got ${exe.sha256}`);
    }
    if (exe.compatibility !== 'Windows 10+') {
      throw new Error(`EXE compatibility mismatch: got ${exe.compatibility}`);
    }

    // Validate Tauri manifest
    const tauriManifest = generateTauriManifest(tempDir, version, testOptions);
    console.log('Generated Tauri manifest in self-test:', JSON.stringify(tauriManifest, null, 2));

    if (tauriManifest.version !== version) {
      throw new Error(`Tauri version mismatch: expected ${version}, got ${tauriManifest.version}`);
    }
    if (tauriManifest.pub_date !== testOptions.releasedAt) {
      throw new Error(`Tauri pub_date mismatch: expected ${testOptions.releasedAt}, got ${tauriManifest.pub_date}`);
    }
    const tauriPlatform = tauriManifest.platforms?.['windows-x86_64'];
    if (!tauriPlatform) {
      throw new Error('Missing windows-x86_64 platform in Tauri manifest');
    }
    const tauriNsisPlatform = tauriManifest.platforms?.['windows-x86_64.nsis'];
    if (!tauriNsisPlatform) {
      throw new Error('Missing windows-x86_64.nsis platform in Tauri manifest');
    }
    if (tauriPlatform.signature !== mockSigContent) {
      throw new Error(`Tauri signature mismatch: expected ${mockSigContent}, got ${tauriPlatform.signature}`);
    }
    if (tauriPlatform.url !== `https://github.com/${DEFAULT_REPOSITORY}/releases/download/${testOptions.releaseTag}/radar-vital-windows-installer.exe`) {
      throw new Error(`Tauri URL mismatch: got ${tauriPlatform.url}`);
    }
    if (tauriNsisPlatform.signature !== mockSigContent || tauriNsisPlatform.url !== tauriPlatform.url) {
      throw new Error('Tauri NSIS platform metadata mismatch');
    }

    console.log('Self-test validation passed successfully!');
  } finally {
    // Clean up temp dir
    fs.rmSync(tempDir, { recursive: true, force: true });
  }
}

// Main execution
const isSelfTest = process.argv.includes('--self-test');
if (isSelfTest) {
  try {
    runSelfTest();
    process.exit(0);
  } catch (error) {
    console.error('Self-test failed:', error);
    process.exit(1);
  }
} else {
  try {
    const distDir = path.join(ROOT_DIR, 'dist');
    if (!fs.existsSync(distDir)) {
      fs.mkdirSync(distDir, { recursive: true });
    }
    const manifest = generateManifest(distDir, version);
    const outputPath = path.join(distDir, 'rvt-latest.json');
    fs.writeFileSync(outputPath, JSON.stringify(manifest, null, 2));
    console.log(`Successfully generated manifest: ${outputPath}`);

    // Tauri updater manifest
    const tauriManifest = generateTauriManifest(distDir, version);
    const tauriOutputPath = path.join(distDir, 'rvt-latest-tauri.json');
    fs.writeFileSync(tauriOutputPath, JSON.stringify(tauriManifest, null, 2));
    console.log(`Successfully generated Tauri manifest: ${tauriOutputPath}`);
  } catch (error) {
    console.error('Failed to generate manifest:', error);
    process.exit(1);
  }
}
