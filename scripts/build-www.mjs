// Build the www/ directory consumed by both Capacitor and Tauri.
// Copies:
//   radar_vital_live_dashboard_v12_for_v16_0.html  →  www/index.html
//   assets/                                         →  www/assets/
//   assets/manifest.webmanifest                     →  www/manifest.webmanifest
//   assets/sw.js                                    →  www/sw.js
//
// Inputs are read from process.cwd(). Outputs overwrite www/ in place.

import { promises as fs } from 'node:fs';
import path from 'node:path';
import url from 'node:url';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const SRC_DASHBOARD = path.join(ROOT, 'radar_vital_live_dashboard_v12_for_v16_0.html');
const SRC_ASSETS = path.join(ROOT, 'assets');
const OUT = path.join(ROOT, 'www');

async function copyDir(from, to) {
  await fs.mkdir(to, { recursive: true });
  const entries = await fs.readdir(from, { withFileTypes: true });
  for (const entry of entries) {
    const src = path.join(from, entry.name);
    const dst = path.join(to, entry.name);
    if (entry.isDirectory()) {
      await copyDir(src, dst);
    } else if (entry.isFile()) {
      await fs.copyFile(src, dst);
    }
  }
}

async function main() {
  await fs.rm(OUT, { recursive: true, force: true });
  await fs.mkdir(OUT, { recursive: true });

  // 1. Dashboard HTML → www/index.html AND www/<original-filename>.html so deep links work.
  const dashboardName = path.basename(SRC_DASHBOARD);
  const dashboard = await fs.readFile(SRC_DASHBOARD);
  await fs.writeFile(path.join(OUT, 'index.html'), dashboard);
  await fs.writeFile(path.join(OUT, dashboardName), dashboard);

  // 2. assets/ → www/assets/
  await copyDir(SRC_ASSETS, path.join(OUT, 'assets'));

  // 3. Hoist manifest + sw to www/ root so the registration paths (/manifest.webmanifest, /sw.js) resolve.
  await fs.copyFile(path.join(SRC_ASSETS, 'manifest.webmanifest'), path.join(OUT, 'manifest.webmanifest'));
  await fs.copyFile(path.join(SRC_ASSETS, 'sw.js'), path.join(OUT, 'sw.js'));

  // 4. 404 fallback that redirects unknown deep links to the dashboard shell — useful for GitHub Pages and Tauri.
  const fallback = `<!DOCTYPE html><meta http-equiv="refresh" content="0; url=./index.html">`;
  await fs.writeFile(path.join(OUT, '404.html'), fallback);

  console.log(`Built www/ at ${OUT}`);
}

main().catch((err) => { console.error(err); process.exit(1); });
