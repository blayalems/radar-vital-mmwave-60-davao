// Applies native Android shell polish after `cap sync android`.
// The android/ project is generated in CI, so source-controlled patches live here.

import { promises as fs } from 'node:fs';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const stylesPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'values', 'styles.xml');
const manifestPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'AndroidManifest.xml');
const dataExtractionRulesPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'xml', 'data_extraction_rules.xml');
const filePathsPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'xml', 'file_paths.xml');
const androidPackagePath = path.join(ROOT, 'android', 'app', 'src', 'main', 'java', 'app', 'radarvital', 'trainer');
const mainActivityPath = path.join(androidPackagePath, 'MainActivity.java');
const openFilePluginPath = path.join(androidPackagePath, 'OpenFilePlugin.kt');

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

async function main() {
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
  console.log(`Patched Android backup policy: ${path.relative(ROOT, manifestPath)}`);
  console.log(`Patched Android update installer plugin: ${path.relative(ROOT, openFilePluginPath)}`);
}

main().catch(err => {
  console.error(err);
  process.exit(1);
});
