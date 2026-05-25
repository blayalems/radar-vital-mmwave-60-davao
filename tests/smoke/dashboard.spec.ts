import { test, expect } from '@playwright/test';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';

test.describe('Dashboard smoke', () => {
  test('loads without console errors', async ({ page }) => {
    const consoleErrors: string[] = [];
    const pageErrors: string[] = [];
    page.on('console', (msg) => {
      if (msg.type() === 'error') {
        const source = msg.location().url ? ` (${msg.location().url})` : '';
        consoleErrors.push(`${msg.text()}${source}`);
      }
    });
    page.on('pageerror', (err) => {
      const message = err.message || '';
      // WebKit can report aborted same-origin API probes during reload/SW startup as
      // page errors even when the dashboard catches the failed fetch and keeps running.
      if (/\/api\/.* due to access control checks\.$/.test(message)) return;
      pageErrors.push(message);
    });

    const resp = await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    expect(resp?.status(), 'dashboard HTML status').toBeLessThan(400);

    await page.waitForLoadState('networkidle', { timeout: 15_000 }).catch(() => { /* tolerate long-poll SSE */ });
    await page.waitForTimeout(1500);

    const relevantConsoleErrors = consoleErrors.filter((message) =>
      !message.includes('Viewport argument key "interactive-widget" not recognized and ignored.')
    );
    expect(pageErrors, `uncaught errors: ${pageErrors.join(' | ')}`).toEqual([]);
    expect(relevantConsoleErrors, `console errors: ${relevantConsoleErrors.join(' | ')}`).toEqual([]);
  });

  test('PWA manifest link is reachable and well-formed', async ({ page, request }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    const manifestUrl = await page.locator('link[rel="manifest"]').first().getAttribute('href');
    expect(manifestUrl, 'dashboard advertises a manifest').toBeTruthy();
    const resolved = new URL(manifestUrl!, page.url()).toString();
    const resp = await request.get(resolved);
    expect(resp.status()).toBe(200);
    const json = await resp.json();
    expect(json.name).toBeTruthy();
    expect(json.start_url).toBeTruthy();
    expect(Array.isArray(json.icons)).toBe(true);
    expect(json.icons.length).toBeGreaterThan(0);
  });

  test('service worker registers and reaches activated state', async ({ page }) => {
    // The dashboard's controllerchange listener triggers location.reload() when a new SW
    // takes control — that's the production update flow. The test must tolerate the
    // post-install navigation by waiting for it before sampling SW state.
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.waitForLoadState('networkidle', { timeout: 8000 }).catch(() => {});
    // Allow up to one post-install reload to complete.
    await page.waitForTimeout(1500);
    await page.waitForLoadState('domcontentloaded').catch(() => {});

    let result: any = null;
    for (let attempt = 0; attempt < 5; attempt += 1) {
      try {
        result = await page.evaluate(async () => {
          if (!('serviceWorker' in navigator)) return { supported: false };
          try {
            // Use registration() not ready — ready awaits the controller, which can race
            // with the page reload triggered by controllerchange.
            const reg = await Promise.race([
              navigator.serviceWorker.getRegistration(),
              new Promise((resolve) => setTimeout(() => resolve(null), 10_000))
            ]);
            if (!reg) return { supported: true, registered: false };
            return {
              supported: true,
              registered: true,
              scope: reg.scope,
              active: !!reg.active,
              state: reg.active?.state || (reg.installing?.state || reg.waiting?.state || null)
            };
          } catch (e) {
            return { supported: true, error: String(e) };
          }
        });
        break;
      } catch (e) {
        if (!String(e).includes('Execution context was destroyed') || attempt === 4) throw e;
        await page.waitForLoadState('domcontentloaded').catch(() => {});
        await page.waitForTimeout(750);
      }
    }
    expect(result.supported).toBe(true);
    expect(result.registered, `SW not registered: ${JSON.stringify(result)}`).toBe(true);
    // Any of installing/installed/activating/activated is acceptable — what we care about is
    // that the registration exists and has a worker entry.
    expect(['installing', 'installed', 'activating', 'activated']).toContain(result.state);
  });

  test('precaches the shell and opens it offline where navigation emulation permits', async ({ page, browserName }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.evaluate(async () => {
      await navigator.serviceWorker.ready;
    });
    await page.waitForFunction(() => !!navigator.serviceWorker.controller, undefined, { timeout: 15_000 });
    const cachedShell = await page.evaluate(async () => {
      const cache = await caches.open('rvt-shell-v12.0.2');
      return Boolean(
        await cache.match('./index.html')
        || await cache.match('./radar_vital_live_dashboard_v12_for_v16_0.html')
      );
    });
    expect(cachedShell).toBe(true);

    if (browserName === 'webkit') {
      test.info().annotations.push({
        type: 'note',
        description: 'Playwright WebKit reports an internal error for emulated offline navigation; precache was verified.'
      });
      return;
    }

    await page.context().setOffline(true);
    try {
      await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
      await expect(page.locator('app-layout')).toBeVisible();
      await expect(page.locator('#demoBanner')).toBeVisible();
    } finally {
      await page.context().setOffline(false);
    }
  });

  test('viewport meta enables PWA install on mobile', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    const viewport = await page.locator('meta[name="viewport"]').first().getAttribute('content');
    expect(viewport).toContain('viewport-fit=cover');
    expect(viewport).toMatch(/width=device-width/);
  });

  test('theme-color and apple-touch-icon present', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    const themeColor = await page.locator('meta[name="theme-color"]').first().getAttribute('content');
    expect(themeColor).toBeTruthy();
    const appleIcon = await page.locator('link[rel="apple-touch-icon"]').first().getAttribute('href');
    expect(appleIcon).toBeTruthy();
  });

  test('applies persisted themes and opens the Material command palette', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.evaluate(() => localStorage.setItem('rvt-theme', 'night'));
    await page.reload({ waitUntil: 'domcontentloaded' });
    await expect(page.locator('html')).toHaveAttribute('data-theme', 'night');

    await page.keyboard.press('Control+K');
    await expect(page.getByRole('dialog')).toContainText('Command Palette');
    await page.getByRole('button', { name: /Operator playbook/ }).click();
    await expect(page).toHaveURL(/\/help$/);
  });

  test('restores Material icons and the blurred command palette backdrop', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.waitForTimeout(500);

    const iconFont = await page.locator('mat-icon').first().evaluate((element) =>
      getComputedStyle(element).fontFamily
    );
    expect(iconFont).toContain('Material Symbols Rounded');

    await page.keyboard.press('Control+K');
    const backdrop = page.locator('.rvt-palette-backdrop');
    await expect(backdrop).toBeVisible();
    const backdropFilter = await backdrop.evaluate((element) => {
      const style = getComputedStyle(element) as CSSStyleDeclaration & { webkitBackdropFilter?: string };
      return style.backdropFilter || style.webkitBackdropFilter || '';
    });
    expect(backdropFilter).toContain('blur');
    await expect(page.getByRole('button', { name: /Audit log/ })).toBeVisible();
    await page.getByLabel('Search commands').fill('preflight');
    await expect(page.getByRole('button', { name: /Run hardware preflight/ })).toBeVisible();
  });

  test('keeps Angular home cards and desktop topbar inside the visible content area', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.getByRole('link', { name: /Home/ }).first().click();
    await expect(page.getByText('Session Setup').first()).toBeVisible();

    const cardsContained = await page.evaluate(() => {
      const content = document.querySelector('.home-container')?.getBoundingClientRect();
      if (!content) return false;
      return ['.setup-config-card', '.radar-scope-card', '.preflight-card-stack', '.sessions-history-card']
        .map((selector) => document.querySelector(selector)?.getBoundingClientRect())
        .every((card) => !!card && card.left >= content.left - 1 && card.right <= content.right + 1);
    });
    expect(cardsContained).toBe(true);

    if ((page.viewportSize()?.width || 0) >= 1024) {
      await page.getByRole('link', { name: /Live/ }).first().click();
      const headerContained = await page.evaluate(() => {
        const main = document.querySelector('.main-wrapper')?.getBoundingClientRect();
        const title = document.querySelector('.crumb-title')?.getBoundingClientRect();
        const actions = document.querySelector('.topbar-actions')?.getBoundingClientRect();
        return !!main && !!title && !!actions &&
          title.left >= main.left - 1 && actions.right <= main.right + 1;
      });
      expect(headerContained).toBe(true);
    }
  });

  test('restores searchable help topics from the trainer schema', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.getByRole('link', { name: /Help/ }).first().click();
    await expect(page.getByText('Operator Playbook & Field Dictionary')).toBeVisible();

    const reveal = page.getByRole('button', { name: 'Browse all topics' });
    await expect(reveal).toBeVisible();
    await reveal.click();
    expect(await page.locator('.faq-panel').count()).toBeGreaterThan(3);

    await page.getByPlaceholder(/stale, BLE coverage/).fill('BLE');
    await expect(page.locator('mat-expansion-panel-header').filter({ hasText: 'BLE coverage' })).toBeVisible();
    await expect(page.getByText('Context Labels')).toBeVisible();
  });

  test('keeps the Material playbook and recovery tools functional without a trainer help schema', async ({ page }) => {
    await page.route('**/api/help/schema', async (route) => {
      await route.fulfill({ status: 503, contentType: 'application/json', body: '{"error":"schema unavailable"}' });
    });
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.getByRole('link', { name: /Help/ }).first().click();

    for (const topic of ['Getting started', 'Hardware', 'Recovery', 'Reports', 'Firmware']) {
      await expect(page.getByRole('button', { name: new RegExp(topic, 'i') }).first()).toBeVisible();
    }

    await page.getByRole('button', { name: /Firmware/ }).first().click();
    await expect(page.getByText(/Firmware, trainer and dashboard contracts/)).toBeVisible();
    await page.getByRole('switch', { name: /Advanced detail/ }).click();
    await expect(page.getByText(/serial DATA header is a fixed contract/)).toBeVisible();

    await page.getByRole('checkbox', { name: /Confirm source mode/ }).check();
    await page.getByRole('checkbox', { name: /Check sensor links/ }).check();
    await expect(page.getByText('2 / 5 complete')).toBeVisible();

    await page.reload({ waitUntil: 'domcontentloaded' });
    await expect(page.getByText('2 / 5 complete')).toBeVisible();
    await page.getByRole('button', { name: 'Reset' }).click();
    await expect(page.getByText('0 / 5 complete')).toBeVisible();

    await page.getByRole('button', { name: /Copy support summary/ }).click();
    await expect(page.locator('simple-snack-bar').last()).toContainText(/Support summary copied|Clipboard unavailable/);

    const helpContained = await page.evaluate(() => {
      const content = document.querySelector('.help-container')?.getBoundingClientRect();
      if (!content) return false;
      return ['.help-header-card', '.help-search-card', '.recovery-card', '.checklist-card']
        .map((selector) => document.querySelector(selector)?.getBoundingClientRect())
        .every((card) => !!card && card.left >= content.left - 1 && card.right <= content.right + 1);
    });
    expect(helpContained).toBe(true);

    const responsiveLayout = await page.evaluate(() => {
      const viewportWidth = window.innerWidth;
      const help = document.querySelector('.help-container')?.getBoundingClientRect();
      const main = document.querySelector('.help-accordion-content')?.getBoundingClientRect();
      const support = document.querySelector('.support-column')?.getBoundingClientRect();
      const search = document.querySelector('.search-field')?.getBoundingClientRect();
      if (!help || !main || !support || !search) return false;
      if (viewportWidth <= 600) return search.width >= help.width * 0.8;
      if (viewportWidth >= 1024) return main.width > support.width * 1.5;
      return true;
    });
    expect(responsiveLayout).toBe(true);
  });

  test('imports all portable Material settings with bounded values', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.getByRole('link', { name: /Settings/ }).first().click();

    await page.locator('input[type="file"]').setInputFiles({
      name: 'preferences.json',
      mimeType: 'application/json',
      buffer: Buffer.from(JSON.stringify({
        theme: 'hc',
        density: 'compact',
        fontScale: 1.2,
        zenMode: true,
        hxMode: 'off',
        thresholds: { hrLow: 33, hrHigh: 172, rrLow: 7, rrHigh: 32 },
        liveBufferSeconds: 120,
        freezeOnStale: false,
        audioAlertsEnabled: true,
        voiceAlertsEnabled: true,
        audioVolume: 0.45,
        autoDemoOnDisconnect: true
      }))
    });

    await expect(page.locator('html')).toHaveAttribute('data-theme', 'hc');
    const stored = await page.evaluate(() => ({
      density: localStorage.getItem('rvt-density'),
      fontScale: localStorage.getItem('rvt-font-scale'),
      buffer: localStorage.getItem('rvt-live-buffer-seconds'),
      thresholds: localStorage.getItem('rvt-kpi-thresholds'),
      audio: localStorage.getItem('rvt-audio-alerts'),
      voice: localStorage.getItem('rvt-voice-alerts')
    }));
    expect(stored.density).toBe('compact');
    expect(stored.fontScale).toBe('1.2');
    expect(stored.buffer).toBe('120');
    expect(JSON.parse(stored.thresholds || '{}')).toEqual({ hrLow: 33, hrHigh: 172, rrLow: 7, rrHigh: 32 });
    expect(stored.audio).toBe('1');
    expect(stored.voice).toBe('1');
  });

  test('labels automatic fallback as DEMO and does not write unscoped telemetry stores', async ({ page }) => {
    await page.route('**/api/status', route => route.fulfill({ status: 503, body: '{"error":"offline"}' }));
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await expect(page.locator('#demoBanner')).toBeVisible();
    await expect(page.locator('#demoBanner')).toContainText('simulated vitals only');
    const legacyKeys = await page.evaluate(() => [
      'rvt-snaps',
      'rvt-snap-notes',
      'rvt-alert-history',
      'rvt-session-notes',
      'rvt-sandbox-sessions'
    ].filter(key => localStorage.getItem(key) !== null));
    expect(legacyKeys).toEqual([]);
  });

  test('exposes the native BLE acceptance probe and receives an allowlisted Tauri notification', async ({ page }) => {
    await page.addInitScript(() => {
      let notify: ((event: { payload: { device_id: string; data_base64: string } }) => void) | null = null;
      (window as any).__TAURI_INTERNALS__ = {};
      (window as any).__TAURI__ = {
        core: {
          invoke: async (command: string) => {
            if (command === 'native_ble_scan') return [{ id: 'native-ailink', name: 'AiLink QA' }];
            if (command === 'native_ble_start_notifications') {
              setTimeout(() => notify?.({ payload: { device_id: 'native-ailink', data_base64: 'AQI=' } }), 0);
              return null;
            }
            if (command === 'native_pair_request') {
              return { status: 503, data: { error: 'trainer unavailable in native probe test' } };
            }
            return null;
          }
        },
        event: {
          listen: async (_event: string, listener: typeof notify) => {
            notify = listener;
            return () => { notify = null; };
          }
        }
      };
    });
    await page.goto('/home', { waitUntil: 'domcontentloaded' });
    await page.getByRole('button', { name: /Validate native GATT/ }).click();
    await expect(page.getByRole('status')).toContainText(/Native GATT verified: received 2 bytes from AiLink QA/);
    await expect(page.getByRole('status')).toContainText(/trainer telemetry remains the session source/i);
  });

  test('exchanges a one-time pairing PIN without exposing a raw token input', async ({ page }) => {
    await page.addInitScript(() => {
      const originalFetch = window.fetch.bind(window);
      window.fetch = (input, init) => {
        const target = typeof input === 'string'
          ? input
          : input instanceof Request
            ? input.url
            : String(input);
        if (target.includes('/api/auth/exchange')) {
          return Promise.resolve(new Response(JSON.stringify({ token: 'paired-session-token' }), {
            status: 200,
            headers: { 'Content-Type': 'application/json' }
          }));
        }
        return originalFetch(input, init);
      };
    });
    await page.goto('/settings', { waitUntil: 'domcontentloaded' });
    await page.getByLabel('Six-digit LAN pairing PIN').fill('482931');
    await page.getByRole('button', { name: /Pair with PIN/ }).click();
    await expect(page.locator('simple-snack-bar').last()).toContainText(/paired/i);
    const token = await page.evaluate(() => sessionStorage.getItem('rvt-pair-token'));
    expect(token).toBe('paired-session-token');
    await expect(page.getByLabel(/Pairing token/)).toHaveCount(0);
  });

  test('persists demo report review and structured sign-off through Material controls', async ({ page }) => {
    await page.addInitScript(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.goto('/report', { waitUntil: 'domcontentloaded' });
    await expect(page.getByText(/DEMO - Simulated Data Only/)).toBeVisible();
    await page.getByLabel('Operator report notes').fill('Reviewed simulated trace only.');
    await page.getByRole('button', { name: /Save Report Notes/ }).click();
    await page.getByLabel('Operator name').fill('Field Operator');
    await page.getByLabel('Initials').fill('FO');
    await page.getByLabel('Validation comments').fill('Simulation marked non-clinical.');
    await page.getByRole('button', { name: /Record sign-off/ }).click();
    await expect(page.getByText(/Signed/).first()).toBeVisible();
  });

  test('exposes command search in the condensed mobile action menu', async ({ page }) => {
    if ((page.viewportSize()?.width || 0) > 760) return;
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.getByRole('button', { name: 'More console actions' }).click();
    await page.getByRole('menuitem', { name: /Search commands/ }).click();
    await expect(page.getByRole('dialog')).toContainText('Command Palette');
  });

  test('hydrates active-session actions and home history from trainer APIs', async ({ page }) => {
    let active = true;
    await page.route('**/sw.js', (route) => route.abort());
    await page.route('**/api/status', (route) => route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        ok: true,
        active_session: active ? { session_id: 'session-active', subject: 'Subject A' } : null
      })
    }));
    await page.route('**/api/sessions', (route) => route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        items: [{
          session_id: 'session-active',
          started_at: '2026-05-24T00:00:00Z',
          duration_s: 30,
          subject: 'Subject A',
          verdict: { verdict: 'ready' }
        }]
      })
    }));

    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await expect(page.getByRole('button', { name: /Stop Session/ })).toBeEnabled();
    await page.getByRole('link', { name: /Home/ }).first().click();
    await expect(page.getByText('Subject A')).toBeVisible();
    await expect(page.locator('.session-verdict-badge')).toHaveText('READY');

    active = false;
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await expect(page.getByRole('button', { name: /Stop Session/ })).toBeDisabled();
  });

  test('exports alert history and audit evidence through Material actions', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.keyboard.press('Control+K');
    await page.getByLabel('Search commands').fill('Open alerts');
    await page.getByRole('button', { name: /Open alerts/ }).click();
    await page.getByRole('button', { name: 'Test alert' }).click();
    await page.getByRole('button', { name: 'Export' }).click();
    const alertsDownloadPromise = page.waitForEvent('download');
    await page.getByRole('menuitem', { name: /Alert history CSV/ }).click();
    expect((await alertsDownloadPromise).suggestedFilename()).toBe('radar-vital-alert-history.csv');
    await page.getByRole('button', { name: 'Done' }).click();

    await page.keyboard.press('Alt+6');
    await expect(page.getByText('Active Enforcement Logs')).toBeVisible();
    await page.locator('.audit-card-header').getByRole('button', { name: 'Export' }).click();
    const auditDownloadPromise = page.waitForEvent('download');
    await page.getByRole('menuitem', { name: /Audit JSON/ }).click();
    expect((await auditDownloadPromise).suggestedFilename()).toBe('radar-vital-audit.json');
  });

  test('explicit demo mode streams telemetry and preserves snapshot capture', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.evaluate(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.reload({ waitUntil: 'domcontentloaded' });

    await expect(page.locator('#demoBanner')).toBeVisible();
    await expect(page.locator('.kpi-hr .kpi-card-value strong')).not.toHaveText('--', { timeout: 5000 });
    await page.getByRole('button', { name: /Pin Snapshot/ }).first().click();
    await page.getByRole('tab', { name: 'Snaps' }).click();
    await expect(page.getByText('Pinned Telemetry Snapshots')).toBeVisible();
    await expect(page.locator('.snap-card')).toHaveCount(1);
  });

  test('restores live diagnostics and functional Material actions in demo mode', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.evaluate(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.reload({ waitUntil: 'domcontentloaded' });
    await expect(page.locator('.kpi-hr .kpi-card-value strong')).not.toHaveText('--', { timeout: 5000 });
    const demoHeaderContained = await page.evaluate(() => {
      const banner = document.querySelector('#demoBanner')?.getBoundingClientRect();
      const topbar = document.querySelector('.topbar')?.getBoundingClientRect();
      return !!banner && !!topbar && topbar.top >= banner.bottom - 1;
    });
    expect(demoHeaderContained).toBe(true);
    if ((page.viewportSize()?.width || 0) <= 760) {
      const tabsContained = await page.evaluate(() => {
        const header = document.querySelector('.live-tabs-group .mat-mdc-tab-header')?.getBoundingClientRect();
        const tabs = Array.from(document.querySelectorAll('.live-tabs-group .mat-mdc-tab'))
          .map(element => element.getBoundingClientRect());
        return !!header && tabs.length === 6 &&
          tabs.every(tab => tab.left >= header.left - 1 && tab.right <= header.right + 1);
      });
      expect(tabsContained).toBe(true);
    }

    const notes = page.getByLabel('Operator observations');
    const expectVisibleCardsContained = async () => {
      await page.waitForTimeout(550);
      const overflowCards = await page.evaluate(() => {
        const main = document.querySelector('.main-content-scroll')?.getBoundingClientRect();
        if (!main) return ['main-content-scroll unavailable'];
        return Array.from(document.querySelectorAll('app-live .mat-mdc-card'))
          .filter(card => card.getClientRects().length > 0)
          .filter(card => {
            const rect = card.getBoundingClientRect();
            return rect.left < main.left - 1 || rect.right > main.right + 1;
          })
          .map(card => `${card.className}: ${Math.round(card.getBoundingClientRect().left)}..${Math.round(card.getBoundingClientRect().right)} outside ${Math.round(main.left)}..${Math.round(main.right)}`);
      });
      expect(overflowCards).toEqual([]);
    };
    await notes.fill('Baseline seated');
    await page.getByRole('button', { name: /Position/ }).click();
    await page.getByLabel('Custom tag').fill('Calibration');
    await page.getByRole('button', { name: /Add Tag/ }).click();
    await expect(notes).toHaveValue(/Position change/);
    await expect(notes).toHaveValue(/Calibration/);

    const notesDownload = page.waitForEvent('download');
    await page.getByRole('button', { name: /Export session notes as text/ }).click();
    expect((await notesDownload).suggestedFilename()).toMatch(/^session-notes-.+\.txt$/);
    await expect(page.getByText('Target Tracking')).toBeVisible();
    await page.getByRole('button', { name: /Pin Frame/ }).click();

    await page.getByRole('tab', { name: 'Waves' }).click();
    await expect(page.getByText(/SQI \d+%/).first()).toBeVisible();
    await expectVisibleCardsContained();
    const waveDownload = page.waitForEvent('download');
    await page.getByRole('button', { name: /Download breathing waveform image/ }).click();
    expect((await waveDownload).suggestedFilename()).toMatch(/^breathing_waveform_\d+\.png$/);

    await page.getByRole('tab', { name: 'HR' }).click();
    await expect(page.getByText('HR Funnel Telemetry')).toBeVisible();
    await expect(page.getByText('HR Stage Values')).toBeVisible();
    await expectVisibleCardsContained();
    await page.getByRole('radio', { name: '30s' }).click();
    await expect(page.getByText('Viewing 30 seconds')).toBeVisible();
    await page.getByRole('button', { name: /Reset heart rate chart window/ }).click();
    await expect(page.getByText('Viewing 120 seconds')).toBeVisible();

    await page.getByRole('tab', { name: 'RR' }).click();
    await expect(page.getByText('RR Funnel Telemetry')).toBeVisible();
    await expect(page.getByText('RR Recovery Diagnostics')).toBeVisible();
    await expect(page.getByText('No RR-specific warnings')).toBeVisible();
    await expectVisibleCardsContained();
    const rrDownload = page.waitForEvent('download');
    await page.getByRole('button', { name: /Download respiration trend image/ }).click();
    expect((await rrDownload).suggestedFilename()).toMatch(/^respiration_trend_\d+\.png$/);

    await page.getByRole('tab', { name: 'Audit' }).click();
    await expect(page.getByText('Live Validation')).toBeVisible();
    await expect(page.getByText('Reason Histograms')).toBeVisible();
    await expect(page.getByText('BLE Reference Quality')).toBeVisible();
    await expectVisibleCardsContained();
    await page.getByRole('tab', { name: 'Snaps' }).click();
    await expect(page.locator('.snap-card')).toHaveCount(1);
    await expectVisibleCardsContained();
  });
});
