import { copyFileSync, cpSync, mkdirSync, rmSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(here, '..', '..', '..');
const out = resolve(here, '..', 'www');

rmSync(out, { recursive: true, force: true });
mkdirSync(out, { recursive: true });
copyFileSync(resolve(root, 'radar_vital_live_dashboard_v12_for_v16_0.html'), resolve(out, 'live_dashboard.html'));
copyFileSync(resolve(root, 'radar_vital_live_dashboard_v12_for_v16_0.html'), resolve(out, 'index.html'));
cpSync(resolve(root, 'assets'), resolve(out, 'assets'), { recursive: true });
cpSync(resolve(root, 'assets', 'icons'), resolve(out, 'icons'), { recursive: true });
cpSync(resolve(root, 'assets', 'lib'), resolve(out, 'lib'), { recursive: true });
cpSync(resolve(root, 'assets', 'fonts'), resolve(out, 'fonts'), { recursive: true });
copyFileSync(resolve(root, 'assets', 'sw.js'), resolve(out, 'sw.js'));
copyFileSync(resolve(root, 'assets', 'pair.html'), resolve(out, 'pair.html'));
