import { expect, test } from '@playwright/test';

const views = ['home', 'live', 'report', 'help', 'settings'];
const themes = ['light', 'dark', 'night', 'hc'];

test.describe('v12 dashboard visual baseline', () => {
  // Service-worker/offline behavior is covered by smoke tests. Blocking it
  // here ensures deterministic API route fixtures are not bypassed.
  test.use({ serviceWorkers: 'block' });

  test.beforeEach(async ({ page }) => {
    await page.route('**/api/auth/validate', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          ok: true,
          operator: {
            operator_id: 'op_test',
            display_name: 'Operator A',
            initials: 'OA'
          }
        })
      });
    });
    await page.route('**/api/operator-profiles', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          schema_version: 'rvt-operator-profiles-v12.0',
          profiles: [{ operator_id: 'op_test', display_name: 'Operator A', initials: 'OA' }]
        })
      });
    });
    await page.addInitScript(() => {
      sessionStorage.setItem('rvt-operator-token', 'mock-test-operator-token');
      const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
      setup.operator_label = 'Operator A';
      localStorage.setItem('rvt-setup', JSON.stringify(setup));
    });
  });

  for (const theme of themes) {
    for (const view of views) {
      test(`${theme} ${view}`, async ({ page }) => {
        // Block external font loading to prevent screenshot hanging in offline/sandboxed environments
        await page.route(/fonts\.(googleapis|gstatic)\.com/, route => route.abort());
        if (view === 'home') {
          // Suppress streamed changes before taking the Home layout/theme capture.
          // Plot rendering remains asserted on the Live route.
          await page.addInitScript(() => {
            Date.now = () => 1_700_000_000_000;
            Object.defineProperty(window, 'EventSource', {
              configurable: true,
              value: class VisualBaselineEventSource {
                onopen: ((event: Event) => void) | null = null;
                onerror: ((event: Event) => void) | null = null;
                onmessage: ((event: MessageEvent) => void) | null = null;

                constructor() {
                  setTimeout(() => this.onopen?.(new Event('open')), 0);
                }

                addEventListener(): void {}
                removeEventListener(): void {}
                close(): void {}
                dispatchEvent(): boolean { return true; }
              }
            });
            // Preflight is hardware-dependent and arrives asynchronously.
            // Intercept it before Angular boots; API behavior is covered by
            // smoke and Python contract suites rather than screenshots.
            const nativeFetch = window.fetch.bind(window);
            window.fetch = async (input: RequestInfo | URL, init?: RequestInit) => {
              const requestUrl = typeof input === 'string'
                ? input
                : input instanceof URL
                  ? input.href
                  : input.url;
              const method = (init?.method || 'GET').toUpperCase();
              if (method === 'GET' && new URL(requestUrl, window.location.href).pathname === '/api/preflight') {
                return new Response(JSON.stringify({ checks: [
                { id: 'python_env', label: 'Python environment', status: 'good', description: 'Ready.' },
                { id: 'firmware_file_present', label: 'Firmware file', status: 'good', description: 'Ready.' },
                { id: 'serial_port_list', label: 'Serial ports', status: 'good', description: 'Ready.' },
                { id: 'session_folder_writable', label: 'Session folder writable', status: 'good', description: 'Ready.' },
                { id: 'disk_space', label: 'Disk space', status: 'good', description: 'Ready.' },
                { id: 'schema_hash_consistency', label: 'Schema hash consistency', status: 'good', description: 'Ready.' },
                { id: 'clock_monotonic_sanity', label: 'Clock monotonic sanity', status: 'good', description: 'Ready.' },
                { id: 'ble_adapter', label: 'BLE adapter', status: 'warn', description: 'Validate on hardware.' },
                { id: 'ble_device_probe', label: 'BLE device probe', status: 'warn', description: 'Validate on hardware.' },
                { id: 'serial_port_probe', label: 'Serial port probe', status: 'warn', description: 'Validate on hardware.' }
                ] }), { status: 200, headers: { 'Content-Type': 'application/json' } });
              }
              return nativeFetch(input, init);
            };
          });
        }

        await page.addInitScript(({ theme }) => {
          localStorage.setItem('rvt-theme', theme);
          localStorage.setItem('rvt-density', theme === 'night' ? 'compact' : 'comfortable');
          localStorage.removeItem('rvt-demo-mode');
        }, { theme });
        await page.goto(`/${view}`, { waitUntil: 'domcontentloaded' });
        await expect(page.locator('html')).toHaveAttribute('data-theme', theme);
        await expect(page.locator('app-layout')).toBeVisible();
        if (view === 'home') {
          try {
            await expect(page.locator('.preflight-row')).toHaveCount(10, { timeout: 5000 });
          } catch {
            // The first local trainer boot can leave an in-flight component
            // initialization pending. Reload once into the same fixed state.
            await page.reload({ waitUntil: 'domcontentloaded' });
            await expect(page.locator('.preflight-row')).toHaveCount(10);
          }
          // Home owns continuously redrawn preview canvases. Live-route
          // baselines still cover rendered plots; hide only these moving
          // pixels so home comparisons validate layout and theme surfaces.
          await page.addStyleTag({
            content: '.scope-canvas, .trend-canvas-graph { visibility: hidden !important; }'
          });
        }
        const snapshotName = theme === 'hc' && view === 'live'
          ? 'v12-hc-live-dark-inverse-controls.png'
          : `v12-${theme}-${view}.png`;
        await expect(page).toHaveScreenshot(snapshotName, { fullPage: true, timeout: 30000 });
      });
    }
  }
});
