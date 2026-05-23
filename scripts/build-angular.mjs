// Build pipeline for the Angular Dashboard:
//   1. Build the Angular production application inside web/ into www/.
//   2. Copy the manifest and service workers from assets/ into www/ root.
//   3. Inline all output JS bundles and CSS stylesheets from index.html into a self-contained monolith
//      at the repository root (radar_vital_live_dashboard_v12_for_v16_0.html).
//

import { promises as fs } from 'node:fs';
import path from 'node:path';
import url from 'node:url';
import { execSync } from 'node:child_process';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const WEB = path.join(ROOT, 'web');
const WWW = path.join(ROOT, 'www');
const DASHBOARD = path.join(ROOT, 'radar_vital_live_dashboard_v12_for_v16_0.html');
const SRC_ASSETS = path.join(ROOT, 'assets');

function normalizeNewlines(text) {
  return text.replace(/\r\n?/g, '\n');
}

async function main() {
  console.log('--- Phase 1: Compiling Standalone Angular Dashboard ---');
  try {
    execSync('npm run build', { cwd: WEB, stdio: 'inherit' });
    console.log('Angular build finished successfully.');
  } catch (err) {
    console.error('Angular compilation failed!', err);
    process.exit(1);
  }

  console.log('--- Phase 2: Copying Static SW & Manifest Assets ---');
  await fs.copyFile(path.join(SRC_ASSETS, 'manifest.webmanifest'), path.join(WWW, 'manifest.webmanifest'));
  await fs.copyFile(path.join(SRC_ASSETS, 'sw.js'), path.join(WWW, 'sw.js'));
  await fs.copyFile(path.join(SRC_ASSETS, 'rvt-sw.js'), path.join(WWW, 'rvt-sw.js'));

  console.log('--- Phase 3: Bundling Self-Contained HTML Monolith ---');
  let indexHtml = normalizeNewlines(await fs.readFile(path.join(WWW, 'index.html'), 'utf8'));

  // 1. Gather all CSS stylesheet matches first (M4) to prevent stateful global regex mutation bugs
  const linkRegex = /<link[^>]*rel="stylesheet"[^>]*href="([^"]+)"[^>]*>/g;
  const cssMatches = Array.from(indexHtml.matchAll(linkRegex));
  
  for (const match of cssMatches) {
    const fullTag = match[0];
    const cssPath = match[1];
    
    // Skip absolute external files if any
    if (cssPath.startsWith('http') || cssPath.startsWith('//')) continue;

    console.log(`Inlining stylesheet: ${cssPath}`);
    const cssContent = await fs.readFile(path.join(WWW, cssPath), 'utf8');
    indexHtml = indexHtml.replaceAll(fullTag, `<style>\n${cssContent}\n</style>`);
  }

  // 2. Gather all JS bundle matches first (M4) to prevent stateful global regex mutation bugs
  const scriptRegex = /<script[^>]*src="([^"]+)"[^>]*><\/script>/g;
  const jsMatches = Array.from(indexHtml.matchAll(scriptRegex));
  
  for (const match of jsMatches) {
    const fullTag = match[0];
    const jsPath = match[1];

    if (jsPath.startsWith('http') || jsPath.startsWith('//')) continue;

    console.log(`Inlining JS bundle: ${jsPath}`);
    const jsContent = await fs.readFile(path.join(WWW, jsPath), 'utf8');
    indexHtml = indexHtml.replaceAll(fullTag, `<script type="module">\n${jsContent}\n</script>`);
  }

  // Post-build validation assertions: Verify that no local un-inlined stylesheets or scripts remain in the monolith
  const remainingLink = indexHtml.match(/<link[^>]*rel="stylesheet"[^>]*href="([^"]+)"[^>]*>/);
  if (remainingLink && !remainingLink[1].startsWith('http') && !remainingLink[1].startsWith('//')) {
    throw new Error(`Monolith compilation assertion failed: Un-inlined local stylesheet tag remains: ${remainingLink[0]}`);
  }

  const remainingScript = indexHtml.match(/<script[^>]*src="([^"]+)"[^>]*>/);
  if (remainingScript && !remainingScript[1].startsWith('http') && !remainingScript[1].startsWith('//')) {
    throw new Error(`Monolith compilation assertion failed: Un-inlined local script tag remains: ${remainingScript[0]}`);
  }

  console.log('Verification assertions passed: All local stylesheets and script bundles successfully inlined.');

  // Save the inlined monolith to the repository root
  await fs.writeFile(DASHBOARD, indexHtml);
  console.log(`Successfully compiled monolithic dashboard: ${path.basename(DASHBOARD)} (${indexHtml.length.toLocaleString()} characters)`);

  // Create standard fallback 404 for GitHub Pages PWA
  await fs.writeFile(
    path.join(WWW, '404.html'),
    '<!DOCTYPE html><meta http-equiv="refresh" content="0; url=./index.html">'
  );
  console.log('Dashboard build round-trip clean & compiled.');
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
