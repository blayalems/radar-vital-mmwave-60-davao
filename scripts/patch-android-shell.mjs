// Applies native Android shell polish after `cap sync android`.
// The android/ project is generated in CI, so source-controlled patches live here.

import { promises as fs } from 'node:fs';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const stylesPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'values', 'styles.xml');
const manifestPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'AndroidManifest.xml');
const rootGradlePath = path.join(ROOT, 'android', 'build.gradle');
const appGradlePath = path.join(ROOT, 'android', 'app', 'build.gradle');
const dataExtractionRulesPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'xml', 'data_extraction_rules.xml');
const filePathsPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'xml', 'file_paths.xml');
const androidPackagePath = path.join(ROOT, 'android', 'app', 'src', 'main', 'java', 'app', 'radarvital', 'trainer');
const mainActivityPath = path.join(androidPackagePath, 'MainActivity.java');
const openFilePluginPath = path.join(androidPackagePath, 'OpenFilePlugin.kt');

// Adaptive icon resource paths
const resDrawablePath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'drawable');
const resMipmapAnydpiPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'mipmap-anydpi-v26');
const resValuesColorsPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'values', 'colors.xml');
const variablesGradlePath = path.join(ROOT, 'android', 'variables.gradle');
// Source foreground drawable (committed in repo)
const srcForegroundDrawable = path.join(ROOT, 'assets', 'icons', 'android', 'ic_launcher_foreground.xml');

const MAIN_ACTIVITY_JAVA = `package app.radarvital.trainer;

import android.os.Bundle;

import com.getcapacitor.BridgeActivity;

public class MainActivity extends BridgeActivity {
    @Override
    public void onCreate(Bundle savedInstanceState) {
        registerPlugin(OpenFilePlugin.class);
        super.onCreate(savedInstanceState);
    }
}
`;

const OPEN_FILE_PLUGIN_KT = `package app.radarvital.trainer

import android.content.Intent
import android.net.Uri
import android.os.Build
import android.provider.Settings
import androidx.core.content.FileProvider
import com.getcapacitor.JSObject
import com.getcapacitor.Plugin
import com.getcapacitor.PluginCall
import com.getcapacitor.PluginMethod
import com.getcapacitor.annotation.CapacitorPlugin
import java.io.File

@CapacitorPlugin(name = "OpenFile")
class OpenFilePlugin : Plugin() {
    @PluginMethod
    fun openFile(call: PluginCall) {
        val rawPath = call.getString("path")
        val mimeType = call.getString("mimeType", "application/vnd.android.package-archive")
        if (rawPath.isNullOrBlank()) {
            call.reject("A file path is required.")
            return
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O &&
            !context.packageManager.canRequestPackageInstalls()
        ) {
            val permissionIntent = Intent(
                Settings.ACTION_MANAGE_UNKNOWN_APP_SOURCES,
                Uri.parse("package:\${context.packageName}")
            ).addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
            context.startActivity(permissionIntent)
            call.reject("Android requires install permission for this app. Enable it and retry.")
            return
        }

        val file = when {
            rawPath.startsWith("file://") -> File(Uri.parse(rawPath).path ?: "")
            rawPath.startsWith("content://") -> {
                call.reject("Expected a cache file path, received a content URI.")
                return
            }
            else -> File(rawPath)
        }

        if (!file.exists()) {
            call.reject("Update package was not found.")
            return
        }

        val uri = FileProvider.getUriForFile(
            context,
            "\${context.packageName}.fileprovider",
            file
        )
        val installIntent = Intent(Intent.ACTION_VIEW)
            .setDataAndType(uri, mimeType)
            .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
            .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)

        context.startActivity(installIntent)
        val result = JSObject()
        result.put("started", true)
        call.resolve(result)
    }
}
`;

function ensureItem(styleBlock, name, value) {
  const re = new RegExp(`<item\\s+name="${name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}">[^<]*<\\/item>`);
  const item = `<item name="${name}">${value}</item>`;
  if (re.test(styleBlock)) return styleBlock.replace(re, item);
  return styleBlock.replace(/(\s*<\/style>)/, `\n        ${item}$1`);
}

function patchStyle(xml, styleName) {
  const re = new RegExp(`(<style\\s+name="${styleName.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}"[^>]*>)([\\s\\S]*?)(\\s*<\\/style>)`);
  return xml.replace(re, (full, open, body, close) => {
    let block = `${open}${body}${close}`;
    block = ensureItem(block, 'android:statusBarColor', '#f4f6fb');
    block = ensureItem(block, 'android:navigationBarColor', '#ffffff');
    block = ensureItem(block, 'android:windowLightStatusBar', 'true');
    block = ensureItem(block, 'android:windowLightNavigationBar', 'true');
    return block;
  });
}

function ensureApplicationAttribute(xml, name, value) {
  const escapedName = name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  // Generated Capacitor manifests keep application attributes on one line; this
  // script only patches that deterministic template shape.
  const re = new RegExp(`\\s+${escapedName}="[^"]*"`);
  if (re.test(xml)) return xml.replace(re, `\n        ${name}="${value}"`);
  return xml.replace(/(<application\b)/, `$1\n        ${name}="${value}"`);
}

function ensureUsesPermission(xml, permission, extra = '') {
  if (xml.includes(`android:name="${permission}"`)) return xml;
  const suffix = extra ? ` ${extra}` : '';
  return xml.replace(/(<application\b)/, `    <uses-permission android:name="${permission}"${suffix} />\n\n    $1`);
}

function ensureFileProvider(xml) {
  if (xml.includes('android.support.FILE_PROVIDER_PATHS')) return xml;
  const provider = `
        <provider
            android:name="androidx.core.content.FileProvider"
            android:authorities="\${applicationId}.fileprovider"
            android:exported="false"
            android:grantUriPermissions="true">
            <meta-data
                android:name="android.support.FILE_PROVIDER_PATHS"
                android:resource="@xml/file_paths"></meta-data>
        </provider>
`;
  return xml.replace(/(\s*<\/application>)/, `${provider}$1`);
}

function ensureKotlinGradlePlugin(rootGradle) {
  if (rootGradle.includes('org.jetbrains.kotlin:kotlin-gradle-plugin')) return rootGradle;
  return rootGradle.replace(
    /(\s*classpath ['"]com\.google\.gms:google-services:[^'"]+['"])/,
    `$1\n        classpath 'org.jetbrains.kotlin:kotlin-gradle-plugin:2.0.21'`
  );
}

function ensureKotlinAndroidPlugin(appGradle) {
  if (appGradle.includes("apply plugin: 'org.jetbrains.kotlin.android'")) return appGradle;
  return appGradle.replace(
    /apply plugin: 'com\.android\.application'/,
    `apply plugin: 'com.android.application'\napply plugin: 'org.jetbrains.kotlin.android'`
  );
}

// ---------------------------------------------------------------------------
// SDK version pinning — FAIL LOUDLY if expected gradle text is absent
// ---------------------------------------------------------------------------

/**
 * Pins compileSdk and targetSdk to the given sdkVersion in the given gradle
 * text.  Throws (loudly) if neither the standard `compileSdk N` pattern nor
 * the Capacitor `compileSdkVersion N` pattern is found — we must never
 * silently drift on the SDK version.
 */
function pinSdkVersions(gradleText, sdkVersion, filePath) {
  // Capacitor templates use either `compileSdk N` or `compileSdkVersion N`
  const compileSdkRe = /\bcompileSdk(?:Version)?\s+\d+/;
  const targetSdkRe = /\btargetSdk(?:Version)?\s+\d+/;

  if (!compileSdkRe.test(gradleText)) {
    console.error(
      `[patch-android-shell] FATAL: could not find compileSdk / compileSdkVersion in ${filePath}.\n` +
      `  The Capacitor template shape may have changed — update this patch script.`
    );
    process.exit(1);
  }
  if (!targetSdkRe.test(gradleText)) {
    console.error(
      `[patch-android-shell] FATAL: could not find targetSdk / targetSdkVersion in ${filePath}.\n` +
      `  The Capacitor template shape may have changed — update this patch script.`
    );
    process.exit(1);
  }

  let patched = gradleText
    .replace(compileSdkRe, `compileSdk ${sdkVersion}`)
    .replace(targetSdkRe, `targetSdk ${sdkVersion}`);

  return patched;
}

/**
 * Pins ext variables `compileSdkVersion` and `targetSdkVersion` inside
 * variables.gradle (used by some Capacitor templates as an ext block).
 * No-ops if the file does not exist (older templates may not have it).
 * Fails loudly if the file exists but the expected pattern is missing.
 */
async function pinVariablesGradle(sdkVersion) {
  let text;
  try {
    text = await fs.readFile(variablesGradlePath, 'utf8');
  } catch (e) {
    if (e.code === 'ENOENT') return; // template doesn't use variables.gradle
    throw e;
  }

  const compileSdkVarRe = /\bcompileSdkVersion\s*=\s*\d+/;
  const targetSdkVarRe = /\btargetSdkVersion\s*=\s*\d+/;

  if (!compileSdkVarRe.test(text) || !targetSdkVarRe.test(text)) {
    console.error(
      `[patch-android-shell] FATAL: variables.gradle exists but is missing ` +
      `compileSdkVersion or targetSdkVersion assignments.\n` +
      `  File: ${variablesGradlePath}\n` +
      `  Update this patch script to match the template shape.`
    );
    process.exit(1);
  }

  text = text
    .replace(compileSdkVarRe, `compileSdkVersion = ${sdkVersion}`)
    .replace(targetSdkVarRe, `targetSdkVersion = ${sdkVersion}`);

  await fs.writeFile(variablesGradlePath, text);
  console.log(`Pinned SDK ${sdkVersion} in: ${path.relative(ROOT, variablesGradlePath)}`);
}

// ---------------------------------------------------------------------------
// Adaptive launcher icon installation
// ---------------------------------------------------------------------------

const IC_LAUNCHER_XML = `<?xml version="1.0" encoding="utf-8"?>
<adaptive-icon xmlns:android="http://schemas.android.com/apk/res/android">
    <background android:drawable="@color/ic_launcher_background" />
    <foreground android:drawable="@drawable/ic_launcher_foreground" />
    <monochrome android:drawable="@drawable/ic_launcher_foreground" />
</adaptive-icon>
`;

const IC_LAUNCHER_ROUND_XML = `<?xml version="1.0" encoding="utf-8"?>
<adaptive-icon xmlns:android="http://schemas.android.com/apk/res/android">
    <background android:drawable="@color/ic_launcher_background" />
    <foreground android:drawable="@drawable/ic_launcher_foreground" />
    <monochrome android:drawable="@drawable/ic_launcher_foreground" />
</adaptive-icon>
`;

/**
 * Ensures `@color/ic_launcher_background` is defined in colors.xml.
 * Creates the file if absent; injects the color entry if missing.
 */
async function ensureLauncherBackgroundColor(colorHex) {
  let xml;
  try {
    xml = await fs.readFile(resValuesColorsPath, 'utf8');
  } catch (e) {
    if (e.code === 'ENOENT') {
      xml = `<?xml version="1.0" encoding="utf-8"?>\n<resources>\n</resources>\n`;
    } else {
      throw e;
    }
  }

  if (xml.includes('name="ic_launcher_background"')) return; // idempotent

  xml = xml.replace(
    /(\s*<\/resources>)/,
    `\n    <color name="ic_launcher_background">${colorHex}</color>$1`
  );
  await fs.writeFile(resValuesColorsPath, xml);
  console.log(`Defined ic_launcher_background color in: ${path.relative(ROOT, resValuesColorsPath)}`);
}

async function installAdaptiveIcon() {
  // 1. Copy foreground drawable from repo source
  await fs.mkdir(resDrawablePath, { recursive: true });
  const fgSrc = await fs.readFile(srcForegroundDrawable, 'utf8');
  await fs.writeFile(path.join(resDrawablePath, 'ic_launcher_foreground.xml'), fgSrc);
  console.log(`Installed ic_launcher_foreground.xml into res/drawable/`);

  // 2. Write mipmap-anydpi-v26 adaptive icon XMLs
  await fs.mkdir(resMipmapAnydpiPath, { recursive: true });
  await fs.writeFile(path.join(resMipmapAnydpiPath, 'ic_launcher.xml'), IC_LAUNCHER_XML);
  await fs.writeFile(path.join(resMipmapAnydpiPath, 'ic_launcher_round.xml'), IC_LAUNCHER_ROUND_XML);
  console.log(`Installed adaptive icon XMLs into res/mipmap-anydpi-v26/`);

  // 3. Ensure background color resource exists
  await ensureLauncherBackgroundColor('#0E5E63');
}

// ---------------------------------------------------------------------------

async function main() {
  let rootGradle = await fs.readFile(rootGradlePath, 'utf8');
  rootGradle = ensureKotlinGradlePlugin(rootGradle);
  await fs.writeFile(rootGradlePath, rootGradle);

  let appGradle = await fs.readFile(appGradlePath, 'utf8');
  appGradle = ensureKotlinAndroidPlugin(appGradle);
  // Pin compileSdk and targetSdk to 35 — FAILS LOUDLY if patterns are absent
  appGradle = pinSdkVersions(appGradle, 35, appGradlePath);
  await fs.writeFile(appGradlePath, appGradle);
  console.log(`Pinned compileSdk/targetSdk=35 in: ${path.relative(ROOT, appGradlePath)}`);

  // Pin variables.gradle if the Capacitor template uses it (no-op if absent)
  await pinVariablesGradle(35);

  let xml = await fs.readFile(stylesPath, 'utf8');
  xml = patchStyle(xml, 'AppTheme');
  xml = patchStyle(xml, 'AppTheme.NoActionBar');
  xml = patchStyle(xml, 'AppTheme.NoActionBarLaunch');
  await fs.writeFile(stylesPath, xml);
  console.log(`Patched Android shell theme: ${path.relative(ROOT, stylesPath)}`);

  let manifest = await fs.readFile(manifestPath, 'utf8');
  manifest = ensureApplicationAttribute(manifest, 'android:allowBackup', 'false');
  manifest = ensureApplicationAttribute(manifest, 'android:dataExtractionRules', '@xml/data_extraction_rules');
  manifest = ensureUsesPermission(manifest, 'android.permission.REQUEST_INSTALL_PACKAGES');
  manifest = ensureUsesPermission(manifest, 'android.permission.WRITE_EXTERNAL_STORAGE', 'android:maxSdkVersion="28"');
  manifest = ensureFileProvider(manifest);
  await fs.writeFile(manifestPath, manifest);

  await fs.mkdir(path.dirname(dataExtractionRulesPath), { recursive: true });
  await fs.writeFile(dataExtractionRulesPath, `<?xml version="1.0" encoding="utf-8"?>
<data-extraction-rules>
    <cloud-backup disableIfNoEncryptionCapabilities="true">
        <exclude domain="root" path="." />
    </cloud-backup>
    <device-transfer>
        <exclude domain="root" path="." />
    </device-transfer>
</data-extraction-rules>
`);
  await fs.writeFile(filePathsPath, `<?xml version="1.0" encoding="utf-8"?>
<paths xmlns:android="http://schemas.android.com/apk/res/android">
    <cache-path name="updates" path="updates/" />
</paths>
`);
  await fs.mkdir(androidPackagePath, { recursive: true });
  await fs.writeFile(mainActivityPath, MAIN_ACTIVITY_JAVA);
  await fs.writeFile(openFilePluginPath, OPEN_FILE_PLUGIN_KT);
  console.log(`Patched Android Kotlin plugin: ${path.relative(ROOT, appGradlePath)}`);
  console.log(`Patched Android backup policy: ${path.relative(ROOT, manifestPath)}`);
  console.log(`Patched Android update installer plugin: ${path.relative(ROOT, openFilePluginPath)}`);

  // Install adaptive launcher icon (ic_launcher_foreground, mipmap-anydpi-v26, background color)
  await installAdaptiveIcon();
}

main().catch(err => {
  console.error(err);
  process.exit(1);
});
