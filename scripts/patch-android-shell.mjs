// Applies native Android shell polish after `cap sync android`.
// The android/ project is generated in CI, so source-controlled patches live here.

import { promises as fs } from 'node:fs';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const stylesPath = path.join(ROOT, 'android', 'app', 'src', 'main', 'res', 'values', 'styles.xml');

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

async function main() {
  let xml = await fs.readFile(stylesPath, 'utf8');
  xml = patchStyle(xml, 'AppTheme');
  xml = patchStyle(xml, 'AppTheme.NoActionBar');
  xml = patchStyle(xml, 'AppTheme.NoActionBarLaunch');
  await fs.writeFile(stylesPath, xml);
  console.log(`Patched Android shell theme: ${path.relative(ROOT, stylesPath)}`);
}

main().catch(err => {
  console.error(err);
  process.exit(1);
});
