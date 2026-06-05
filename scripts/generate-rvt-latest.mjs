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

// Helper to calculate file size and SHA-256 hash
function getFileStats(filePath) {
  const stats = fs.statSync(filePath);
  const size = stats.size;
  const fileBuffer = fs.readFileSync(filePath);
  const sha256 = crypto.createHash('sha256').update(fileBuffer).digest('hex');
  return { size, sha256 };
}

// Generate the manifest object
function generateManifest(distDir, prodVersion) {
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

  const apkStats = getFileStats(apkFilePath);
  const exeStats = getFileStats(exePath);

  return {
    product_version: prodVersion,
    minimum_supported: '16.0.0',
    released_at: new Date().toISOString(),
    artifacts: {
      apk: {
        url: `https://github.com/blayalems/radar-vital-mmwave-60-davao/releases/download/v${prodVersion}/${apkFileName}`,
        size: apkStats.size,
        sha256: apkStats.sha256,
        compatibility: 'Android 8.0+'
      },
      exe: {
        url: `https://github.com/blayalems/radar-vital-mmwave-60-davao/releases/download/v${prodVersion}/radar-vital-windows-installer.exe`,
        size: exeStats.size,
        sha256: exeStats.sha256,
        compatibility: 'Windows 10+'
      }
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

  const mockApkPath = path.join(tempDir, 'radar-vital-release.apk');
  const mockExePath = path.join(tempDir, 'radar-vital-windows-installer.exe');

  fs.writeFileSync(mockApkPath, mockApkContent);
  fs.writeFileSync(mockExePath, mockExeContent);

  const expectedApkSize = Buffer.byteLength(mockApkContent);
  const expectedExeSize = Buffer.byteLength(mockExeContent);
  const expectedApkHash = crypto.createHash('sha256').update(mockApkContent).digest('hex');
  const expectedExeHash = crypto.createHash('sha256').update(mockExeContent).digest('hex');

  try {
    const manifest = generateManifest(tempDir, version);
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

    const apk = manifest.artifacts?.apk;
    const exe = manifest.artifacts?.exe;

    if (!apk || !exe) {
      throw new Error('Missing apk or exe in artifacts');
    }

    if (apk.url !== `https://github.com/blayalems/radar-vital-mmwave-60-davao/releases/download/v${version}/radar-vital-release.apk`) {
      throw new Error(`APK url mismatch: got ${apk.url}`);
    }
    if (apk.size !== expectedApkSize) {
      throw new Error(`APK size mismatch: expected ${expectedApkSize}, got ${apk.size}`);
    }
    if (apk.sha256 !== expectedApkHash) {
      throw new Error(`APK sha256 mismatch: expected ${expectedApkHash}, got ${apk.sha256}`);
    }
    if (apk.compatibility !== 'Android 8.0+') {
      throw new Error(`APK compatibility mismatch: got ${apk.compatibility}`);
    }

    if (exe.url !== `https://github.com/blayalems/radar-vital-mmwave-60-davao/releases/download/v${version}/radar-vital-windows-installer.exe`) {
      throw new Error(`EXE url mismatch: got ${exe.url}`);
    }
    if (exe.size !== expectedExeSize) {
      throw new Error(`EXE size mismatch: expected ${expectedExeSize}, got ${exe.size}`);
    }
    if (exe.sha256 !== expectedExeHash) {
      throw new Error(`EXE sha256 mismatch: expected ${expectedExeHash}, got ${exe.sha256}`);
    }
    if (exe.compatibility !== 'Windows 10+') {
      throw new Error(`EXE compatibility mismatch: got ${exe.compatibility}`);
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
  } catch (error) {
    console.error('Failed to generate manifest:', error);
    process.exit(1);
  }
}
