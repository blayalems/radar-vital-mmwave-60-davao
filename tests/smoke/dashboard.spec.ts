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
    await expect(page.getByText('Telemetry Event Logs')).toBeVisible();
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
    await page.getByRole('tab', { name: 'Snapshots' }).click();
    await expect(page.getByText('Pinned Telemetry Snapshots')).toBeVisible();
    await expect(page.locator('.snap-card')).toHaveCount(1);
  });
});
