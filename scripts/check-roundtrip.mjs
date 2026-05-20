// Round-trip check: confirms web/ is the source of truth.
//
//   1. Snapshot current radar_vital_live_dashboard_v12_for_v16_0.html.
//   2. Run build-www.mjs.
//   3. Verify the rebuilt monolith equals the snapshot after newline
//      normalization. This keeps Windows checkouts from failing on CRLF-only
//      drift while still catching real source/content drift.
//
// Run in CI before tests to catch drift between web/ and the assembled
// artifact (e.g. someone edited the monolith directly instead of the split
// source).

import { promises as fs } from 'node:fs';
import path from 'node:path';
import url from 'node:url';
import { spawnSync } from 'node:child_process';

const __dirname = path.dirname(url.fileURLToPath(import.meta.url));
const ROOT = path.resolve(__dirname, '..');
const DASHBOARD = path.join(ROOT, 'radar_vital_live_dashboard_v12_for_v16_0.html');

function normalizeHtmlBytes(buf) {
  return Buffer.from(buf.toString('utf8').replace(/\r\n?/g, '\n'), 'utf8');
}

const before = await fs.readFile(DASHBOARD);
const result = spawnSync('node', [path.join(__dirname, 'build-www.mjs')], {
  cwd: ROOT,
  stdio: 'inherit',
});
if (result.status !== 0) process.exit(result.status ?? 1);

const after = await fs.readFile(DASHBOARD);
if (Buffer.compare(normalizeHtmlBytes(before), normalizeHtmlBytes(after)) !== 0) {
  console.error(`ERROR: monolith drift detected.
The assembled output from web/ does not match the committed
radar_vital_live_dashboard_v12_for_v16_0.html.

Either someone edited the monolith directly (bad — edit web/ instead) or
the splits in web/ were modified but the monolith was not re-committed
(run \`npm run build:web\` and commit the result).`);
  process.exit(1);
}
console.log('OK: web/ ↔ monolith round-trip is clean.');
