// Build pipeline for the Angular Dashboard:
//   1. Build the Angular production application inside web/ into www/.
//   2. Copy the manifest, service workers, and stable root asset aliases into www/.
//   3. Inline all output JS bundles and CSS stylesheets into one self-contained
//      document used by the root artefact and static/native entry points.
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
  for (const directory of ['fonts', 'icons', 'lib']) {
    await fs.cp(path.join(SRC_ASSETS, directory), path.join(WWW, directory), { recursive: true });
  }

  console.log('--- Phase 3: Bundling Self-Contained HTML Monolith ---');
  const indexPath = path.join(WWW, 'index.html');
  let builtIndexHtml = normalizeNewlines(await fs.readFile(indexPath, 'utf8'));
  const runtimeFontLink = '  <link rel="stylesheet" href="./fonts/rvt-fonts.css" data-rvt-runtime-fonts>\n';
  if (!builtIndexHtml.includes('data-rvt-runtime-fonts')) {
    builtIndexHtml = builtIndexHtml.replace('</head>', `${runtimeFontLink}</head>`);
    await fs.writeFile(indexPath, builtIndexHtml);
  }
  let indexHtml = builtIndexHtml;

  // 1. Gather all CSS stylesheet matches first (M4) to prevent stateful global regex mutation bugs
  const linkRegex = /<link[^>]*rel="stylesheet"[^>]*href="([^"]+)"[^>]*>/g;
  const cssMatches = Array.from(indexHtml.matchAll(linkRegex));
  
  const inlinedStylesheets = new Set();
  for (const match of cssMatches) {
    const fullTag = match[0];
    const cssPath = match[1];
    
    // Skip absolute external files if any
    if (cssPath.startsWith('http') || cssPath.startsWith('//')) continue;
    // Keep font-face URLs relative to the stable /fonts asset alias for HTML and file previews.
    if (cssPath === './fonts/rvt-fonts.css') continue;

    if (inlinedStylesheets.has(cssPath)) {
      indexHtml = indexHtml.replace(fullTag, '');
      continue;
    }
    inlinedStylesheets.add(cssPath);
    console.log(`Inlining stylesheet: ${cssPath}`);
    const cssContent = await fs.readFile(path.join(WWW, cssPath), 'utf8');
    indexHtml = indexHtml.replace(fullTag, `<style>\n${cssContent}\n</style>`);
  }

  // 2. Gather all JS bundle matches first (M4) to prevent stateful global regex mutation bugs
  const scriptRegex = /<script[^>]*src="([^"]+)"[^>]*><\/script>/g;
  const jsMatches = Array.from(indexHtml.matchAll(scriptRegex));
  
  const inlinedScripts = new Set();
  for (const match of jsMatches) {
    const fullTag = match[0];
    const jsPath = match[1];

    if (jsPath.startsWith('http') || jsPath.startsWith('//')) continue;

    if (inlinedScripts.has(jsPath)) {
      indexHtml = indexHtml.replace(fullTag, '');
      continue;
    }
    inlinedScripts.add(jsPath);
    console.log(`Inlining JS bundle: ${jsPath}`);
    const jsContent = await fs.readFile(path.join(WWW, jsPath), 'utf8');
    indexHtml = indexHtml.replace(fullTag, `<script type="module">\n${jsContent}\n</script>`);
  }

  // Post-build validation assertions: Verify that no local un-inlined stylesheets or scripts remain in the monolith
  const remainingLinks = Array.from(indexHtml.matchAll(linkRegex)).filter(match => {
    const href = match[1];
    return !href.startsWith('http') && !href.startsWith('//') && href !== './fonts/rvt-fonts.css';
  });
  if (remainingLinks.length) {
    throw new Error(`Monolith compilation assertion failed: Un-inlined local stylesheet tag remains: ${remainingLinks[0][0]}`);
  }

  const remainingScript = indexHtml.match(/<script[^>]*src="([^"]+)"[^>]*>/);
  if (remainingScript && !remainingScript[1].startsWith('http') && !remainingScript[1].startsWith('//')) {
    throw new Error(`Monolith compilation assertion failed: Un-inlined local script tag remains: ${remainingScript[0]}`);
  }

  console.log('Verification assertions passed: All local stylesheets and script bundles successfully inlined.');

  // Keep every distribution entry point on the same self-contained shell. This
  // also makes service-worker precaching deterministic for offline launch.
  await fs.writeFile(DASHBOARD, indexHtml);
  await fs.writeFile(indexPath, indexHtml);
  await fs.writeFile(path.join(WWW, path.basename(DASHBOARD)), indexHtml);
  console.log(`Successfully compiled monolithic dashboard: ${path.basename(DASHBOARD)} (${indexHtml.length.toLocaleString()} characters)`);

  // GitHub Pages serves 404.html for direct Angular route loads. Use the
  // application shell itself so /report, /settings and other routes survive
  // reloads within the repository-scoped Pages URL.
  await fs.writeFile(
    path.join(WWW, '404.html'),
    indexHtml
  );
  console.log('Dashboard build round-trip clean & compiled.');
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
