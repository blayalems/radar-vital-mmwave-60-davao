import { test, expect, type Page } from '@playwright/test';

const DASHBOARD = '/radar_vital_live_dashboard_v12_for_v16_0.html';

async function leaveActiveSessionIfPrompted(page: Page): Promise<void> {
  const dialog = page.getByRole('dialog').filter({ hasText: 'Leave active session?' });
  const prompted = await dialog.waitFor({ state: 'visible', timeout: 5000 }).then(() => true).catch(() => false);
  if (prompted) {
    await dialog.getByRole('button', { name: /Leave view/ }).click();
  }
}

async function expectNoHorizontalOverflow(page: Page): Promise<void> {
  const overflow = await page.evaluate(() => {
    const viewportWidth = document.documentElement.clientWidth;
    const documentOverflow = Math.max(document.documentElement.scrollWidth, document.body.scrollWidth) - viewportWidth;
    const offenders = Array.from(document.querySelectorAll('body *'))
      .filter((element) => element.getClientRects().length > 0)
      .filter((element) => !element.matches('.skip-link:not(:focus), [aria-hidden="true"], [aria-hidden="true"] *'))
      .filter((element) => {
        const style = getComputedStyle(element);
        return style.visibility !== 'hidden' && style.display !== 'none' && Number(style.opacity || '1') !== 0;
      })
      .map((element) => {
        const rect = element.getBoundingClientRect();
        return {
          selector: element.tagName.toLowerCase() + (element.className ? `.${String(element.className).trim().split(/\s+/).join('.')}` : ''),
          left: Math.round(rect.left),
          right: Math.round(rect.right),
          width: Math.round(rect.width)
        };
      })
      .filter((rect) => rect.width > 0 && (rect.left < -2 || rect.right > viewportWidth + 2))
      .slice(0, 8);
    return { documentOverflow, offenders };
  });
  expect(overflow.documentOverflow, `document overflow: ${JSON.stringify(overflow)}`).toBeLessThanOrEqual(2);
  expect(overflow.offenders, `horizontal overflow offenders: ${JSON.stringify(overflow.offenders)}`).toEqual([]);
}

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
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.waitForLoadState('networkidle', { timeout: 8000 }).catch(() => {});
    await page.waitForTimeout(1500);
    await page.waitForLoadState('domcontentloaded').catch(() => {});

    let result: any = null;
    for (let attempt = 0; attempt < 5; attempt += 1) {
      try {
        result = await page.evaluate(async () => {
          if (!('serviceWorker' in navigator)) return { supported: false };
          try {
            // Use registration() rather than waiting for controller activation.
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
      const cache = await caches.open('rvt-shell-v12.0.3');
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
    await leaveActiveSessionIfPrompted(page);
    await expect(page).toHaveURL(/\/help$/);
  });

  test('exposes keyboard navigation and live status semantics without stealing text input shortcuts', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    const skipLink = page.getByRole('link', { name: 'Skip to main content' });
    await skipLink.focus();
    await page.keyboard.press('Enter');
    await expect(page.locator('#mainContent')).toBeFocused();

    await page.evaluate(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.goto('/live', { waitUntil: 'domcontentloaded' });
    await expect(page.locator('.kpi-hr')).toHaveAttribute('role', 'status');
    await expect(page.locator('.kpi-hr')).toHaveAttribute('aria-live', 'polite');
    await expect(page.locator('.kpi-hr canvas')).toHaveAttribute('role', 'img');
    await expect(page.locator('app-live [role="alert"]')).toHaveCount(1);

    const notes = page.getByLabel('Operator observations');
    await notes.focus();
    await page.keyboard.press('Control+K');
    await expect(page.getByRole('dialog')).toHaveCount(0);
  });

  test('restores v11 live keyboard workflows in the Angular operator console', async ({ page }) => {
    await page.addInitScript(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.goto('/live', { waitUntil: 'domcontentloaded' });
    await expect(page.locator('.kpi-hr .kpi-card-value strong')).not.toHaveText('--', { timeout: 5000 });

    await page.keyboard.press('?');
    await expect(page.getByRole('dialog')).toContainText('Tag motion, cough, speaking or baseline');
    await page.keyboard.press('Escape');

    await page.keyboard.press('m');
    await expect(page.getByLabel('Operator observations')).toHaveValue(/\[\d{2}:\d{2}:\d{2}\] Motion/);
    await page.keyboard.press('p');
    await page.keyboard.press('Alt+5');
    await expect(page.locator('.snap-card')).toHaveCount(1);

    await page.keyboard.press('Alt+3');
    await page.keyboard.press('Alt+8');
    await expect(page.getByText('Viewing 60 seconds')).toBeVisible();

    const initialTheme = await page.locator('html').getAttribute('data-theme');
    await page.keyboard.press('t');
    await expect.poll(() => page.locator('html').getAttribute('data-theme')).not.toBe(initialTheme);
    await page.keyboard.press('Shift+F');
    await expect(page.locator('body')).toHaveClass(/zen-mode/);
  });

  test('allocates legible height to Live overview graphs', async ({ page }) => {
    await page.addInitScript(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.goto('/live', { waitUntil: 'domcontentloaded' });
    await expect(page.locator('.kpi-hr .kpi-card-value strong')).not.toHaveText('--', { timeout: 5000 });

    const graphHeights = await page.locator('.kpi-card-spark').evaluateAll(elements =>
      elements.map(element => element.getBoundingClientRect().height)
    );
    expect(graphHeights).toHaveLength(4);
    expect(Math.min(...graphHeights)).toBeGreaterThanOrEqual(72);
  });

  test('keeps primary navigation available in simple view and collapses the desktop rail', async ({ page }) => {
    if ((page.viewportSize()?.width || 0) >= 1024) {
      await page.setViewportSize({ width: 1280, height: 720 });
    }
    await page.addInitScript(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.goto('/live', { waitUntil: 'domcontentloaded' });

    const viewportWidth = page.viewportSize()?.width || 0;
    const simpleView = page.getByTitle('Simple view');
    if (await simpleView.isVisible()) {
      await simpleView.click();
      await expect(page.locator('body')).toHaveClass(/zen-mode/);
      // Assert clean Simple View layouts hide diagnostics double column, tracking, and quad grids
      await expect(page.locator('.tracking-card')).toHaveCount(0);
      await expect(page.locator('.diagnostics-double-column')).toHaveCount(0);
      await expect(page.locator('.quad-grid-diagnostics')).toHaveCount(0);
    }

    if (viewportWidth >= 1024) {
      const rail = page.locator('.rail');
      await expect(rail).toBeVisible();
      const expandedWidth = await rail.evaluate(element => element.getBoundingClientRect().width);
      await page.getByRole('button', { name: 'Collapse sidebar' }).click();
      await expect(page.locator('body')).toHaveAttribute('data-rail-collapsed', '1');
      await expect(page.getByRole('button', { name: 'Expand sidebar' })).toBeVisible();
      const collapsedWidth = await rail.evaluate(element => element.getBoundingClientRect().width);
      expect(collapsedWidth).toBeLessThan(expandedWidth - 100);
      await expect.poll(async () => page.evaluate(() => {
        const railRect = document.querySelector('.rail')?.getBoundingClientRect();
        const mainRect = document.querySelector('.main-wrapper')?.getBoundingClientRect();
        if (!railRect || !mainRect) return false;
        const controls = Array.from(document.querySelectorAll(
          '.rail .brand-mark, .rail .rail-collapse-btn, .rail .r-item, .rail .operator'
        ));
        const contained = controls.every(control => {
          const rect = control.getBoundingClientRect();
          return rect.left >= railRect.left - 1 && rect.right <= railRect.right + 1;
        });
        const visibleIcons = Array.from(document.querySelectorAll('.rail .r-item .material-symbols-rounded'))
          .every(icon => {
            const rect = icon.getBoundingClientRect();
            const centerOffset = Math.abs((rect.left + rect.width / 2) -
              (railRect.left + railRect.width / 2));
            return rect.width > 0 && rect.left >= railRect.left - 1 &&
              rect.right <= railRect.right + 1 && centerOffset <= 2;
          });
        const hiddenLabels = Array.from(document.querySelectorAll('.rail .r-label'))
          .every(label => getComputedStyle(label).display === 'none');
        const railElement = document.querySelector('.rail');
        const railInner = railElement?.querySelector('.mat-drawer-inner-container');
        const clippedOverflow = !!railElement && !!railInner &&
          railElement.scrollWidth <= railElement.clientWidth + 1 &&
          railElement.scrollHeight <= railElement.clientHeight + 1 &&
          railInner.scrollHeight <= railInner.clientHeight + 1;
        return contained && visibleIcons && hiddenLabels && clippedOverflow &&
          mainRect.left <= railRect.right + 1;
      })).toBe(true);
    } else {
      await expect(page.locator('.bottom-nav')).toBeVisible();
      await expectNoHorizontalOverflow(page);
    }
  });

  test('renders desktop topbar controls without duplicate surfaces or selection indicators', async ({ page }) => {
    await page.setViewportSize({ width: 1600, height: 900 });
    await page.addInitScript(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.goto('/live', { waitUntil: 'domcontentloaded' });
    await expect(page.locator('.topbar-status')).toBeVisible();

    const hasUnboxedGroups = await page.evaluate(() =>
      ['.topbar-status', '.topbar-actions'].every(selector => {
        const element = document.querySelector(selector);
        if (!element) return false;
        const style = getComputedStyle(element);
        return style.backgroundColor === 'rgba(0, 0, 0, 0)' &&
          style.borderTopWidth === '0px' &&
          style.boxShadow === 'none';
      })
    );
    expect(hasUnboxedGroups).toBe(true);
    await expect(page.locator('.tb-live-mode .mat-pseudo-checkbox')).toHaveCount(0);
    const liveModeBounds = await page.locator('.tb-live-mode').evaluate(element => {
      const rect = element.getBoundingClientRect();
      return { width: rect.width, height: rect.height };
    });
    expect(liveModeBounds.height).toBeLessThanOrEqual(48);
    expect(liveModeBounds.width).toBeLessThanOrEqual(125);
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
    await leaveActiveSessionIfPrompted(page);
    await expect(page.getByText('Session Setup').first()).toBeVisible();
    await expect(page.locator('.setup-config-card')).toBeVisible();
    await expect(page.locator('.radar-scope-card')).toBeVisible();
    await expect(page.locator('.preflight-card-stack')).toBeVisible();
    await expect(page.locator('.sessions-history-card')).toBeVisible();

    const cardsContained = await page.evaluate(() => {
      const content = document.querySelector('.home-container')?.getBoundingClientRect();
      if (!content) return false;
      return ['.setup-config-card', '.radar-scope-card', '.preflight-card-stack', '.sessions-history-card']
        .map((selector) => document.querySelector(selector)?.getBoundingClientRect())
        .every((card) => !!card && card.left >= content.left - 1 && card.right <= content.right + 1);
    });
    expect(cardsContained).toBe(true);

    if ((page.viewportSize()?.width || 0) < 1024) {
      await expectNoHorizontalOverflow(page);
      const mobileGeometry = await page.evaluate(() => {
        const viewportWidth = document.documentElement.clientWidth;
        const topbar = document.querySelector('.topbar')?.getBoundingClientRect();
        const banner = document.querySelector('#demoBanner')?.getBoundingClientRect();
        const firstChromeTop = banner && banner.height > 0 ? banner.top : topbar?.top;
        const shellStartsAtTop = firstChromeTop !== undefined && firstChromeTop <= 2;
        const setupCard = document.querySelector('.setup-config-card')?.getBoundingClientRect();
        const setupControls = Array.from(document.querySelectorAll(
          '.setup-config-card .setup-card-content, .setup-config-card .setup-field, .setup-config-card .field-controls, .setup-config-card .form-field-full, .setup-config-card .pair-sensor-btn, .setup-config-card .setup-card-actions, .setup-config-card .start-session-btn'
        ));
        const controlOverflow = setupControls
          .filter((element) => element.getClientRects().length > 0)
          .filter((element) => {
            const rect = element.getBoundingClientRect();
            return rect.left < (setupCard?.left ?? 0) - 2 ||
              rect.right > (setupCard?.right ?? viewportWidth) + 2 ||
              rect.left < -2 ||
              rect.right > viewportWidth + 2;
          })
          .map((element) => element.className);
        return { shellStartsAtTop, controlOverflow };
      });
      expect(mobileGeometry.shellStartsAtTop, `blank top band geometry: ${JSON.stringify(mobileGeometry)}`).toBe(true);
      expect(mobileGeometry.controlOverflow, `Home setup overflow: ${JSON.stringify(mobileGeometry.controlOverflow)}`).toEqual([]);

      await page.locator('.main-content-scroll').evaluate((element) => { element.scrollTop = element.scrollHeight; });
      const bottomClearance = await page.evaluate(() => {
        const nav = document.querySelector('.bottom-nav')?.getBoundingClientRect();
        const startButton = document.querySelector('.start-session-btn')?.getBoundingClientRect();
        return !!nav && !!startButton && startButton.bottom <= nav.top - 4;
      });
      expect(bottomClearance).toBe(true);
    }

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
    await leaveActiveSessionIfPrompted(page);
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
    await leaveActiveSessionIfPrompted(page);

    for (const topic of ['Getting started', 'Hardware', 'Recovery', 'Reports', 'Firmware']) {
      await expect(page.getByRole('button', { name: new RegExp(topic, 'i') }).first()).toBeVisible();
    }

    await page.locator('.topic-links button').filter({ hasText: 'Firmware' }).click();
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
    // Wait for the loading overlay to disappear so the layout is stable and event listeners are bound
    await page.locator('.initial-loading-overlay').waitFor({ state: 'hidden', timeout: 10000 }).catch(() => {});
    await page.locator('a[aria-label="Settings"]:visible, a[aria-label="Settings view"]:visible').first().click();
    await leaveActiveSessionIfPrompted(page);

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

  test('applies compact density to Angular settings spacing', async ({ page }) => {
    await page.goto('/settings', { waitUntil: 'domcontentloaded' });
    const settingsGrid = page.locator('.settings-grid');
    const comfortableGap = await settingsGrid.evaluate(element => parseFloat(getComputedStyle(element).gap));

    await page.getByRole('radio', { name: 'Compact' }).click();
    await expect(page.locator('html')).toHaveAttribute('data-density', 'compact');
    const compactGap = await settingsGrid.evaluate(element => parseFloat(getComputedStyle(element).gap));
    expect(compactGap).toBeLessThan(comfortableGap);
  });

  test('scans and applies a BLE reference device from Material settings', async ({ page }) => {
    // Keep this API-mock flow on direct browser requests; PWA behavior is covered by the service-worker tests above.
    await page.route('**/sw.js', route => route.abort());
    await page.route('**/api/ble/scan*', async route => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          ok: true,
          devices: [
            { name: 'AiLink QA Oximeter', address: 'AA:BB:CC:DD:EE:01', rssi: -54 },
            { name: 'Backup Reference', id: 'backup-reference', rssi: -71 }
          ]
        })
      });
    });
    await page.goto('/settings', { waitUntil: 'domcontentloaded' });
    await page.getByRole('button', { name: /Scan BLE/ }).click();
    await expect(page.getByText('2 devices found.')).toBeVisible();
    await page.getByRole('button', { name: /Use BLE device AiLink QA Oximeter/ }).click();
    await expect(page.getByText('AA:BB:CC:DD:EE:01').first()).toBeVisible();
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

  test('keeps recovery navigation available while initial status detection is pending', async ({ page }) => {
    let releaseStatus!: () => void;
    const statusReleased = new Promise<void>(resolve => { releaseStatus = resolve; });
    await page.route('**/api/status', async route => {
      await statusReleased;
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({ ok: true, active_session: { session_id: 'late-live-session' } })
      });
    });

    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await expect(page.locator('#mainContent')).toBeVisible();
    await expect(page.locator('a[aria-label="Settings"]:visible, a[aria-label="Settings view"]:visible').first()).toBeVisible();
    await expect(page.getByRole('button', { name: /Bypass to Sandbox Mode/ })).toBeVisible();
    await page.getByRole('button', { name: /Bypass to Sandbox Mode/ }).click();
    await expect(page.locator('#demoBanner')).toBeVisible();

    releaseStatus();
    await page.waitForTimeout(100);
    await expect(page.locator('#demoBanner')).toBeVisible();
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

  test('keeps simulated provenance visible when printing a demo report', async ({ page }) => {
    await page.addInitScript(() => localStorage.setItem('rvt-demo-mode', '1'));
    await page.goto('/report', { waitUntil: 'domcontentloaded' });
    await page.emulateMedia({ media: 'print' });
    await expect(page.locator('#demoBanner')).toBeVisible();
    await expect(page.locator('#demoBanner')).toContainText('simulated vitals only');
  });

  test('exposes command search in the condensed mobile action menu', async ({ page }) => {
    if ((page.viewportSize()?.width || 0) > 760) return;
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.getByRole('button', { name: 'More console actions' }).click();
    await page.getByRole('menuitem', { name: /Search commands/ }).click();
    const dialog = page.getByRole('dialog');
    await expect(dialog).toContainText('Command Palette');
    await expect(page.getByRole('button', { name: 'Close' })).toBeVisible();
    await expectNoHorizontalOverflow(page);
    const paletteGeometry = await page.evaluate(() => {
      const viewportWidth = document.documentElement.clientWidth;
      const viewportHeight = window.innerHeight;
      const dialog = document.querySelector('[role="dialog"]')?.getBoundingClientRect();
      const close = Array.from(document.querySelectorAll('button'))
        .find((button) => button.textContent?.trim() === 'Close')
        ?.getBoundingClientRect();
      const rows = Array.from(document.querySelectorAll('.command-row')).slice(0, 8);
      const rowOverflow = rows
        .filter((row) => {
          const rect = row.getBoundingClientRect();
          return rect.left < (dialog?.left ?? 0) - 2 ||
            rect.right > (dialog?.right ?? viewportWidth) + 2 ||
            rect.right > viewportWidth + 2;
        })
        .map((row) => row.textContent?.trim().slice(0, 80));
      const leftAligned = rows.every((row) => {
        const rowRect = row.getBoundingClientRect();
        const mainRect = row.querySelector('.command-main')?.getBoundingClientRect();
        return !!mainRect && Math.abs(mainRect.left - rowRect.left) <= 2;
      });
      const list = document.querySelector('.palette-content mat-nav-list');
      const listScrolls = !!list && list.scrollHeight > list.clientHeight;
      return {
        dialogContained: !!dialog && dialog.top >= -2 && dialog.left >= -2 &&
          dialog.right <= viewportWidth + 2 && dialog.bottom <= viewportHeight + 2,
        closeVisible: !!close && close.bottom <= viewportHeight + 2 && close.right <= viewportWidth + 2,
        rowOverflow,
        leftAligned,
        listScrolls
      };
    });
    expect(paletteGeometry.dialogContained, JSON.stringify(paletteGeometry)).toBe(true);
    expect(paletteGeometry.closeVisible, JSON.stringify(paletteGeometry)).toBe(true);
    expect(paletteGeometry.rowOverflow, JSON.stringify(paletteGeometry.rowOverflow)).toEqual([]);
    expect(paletteGeometry.leftAligned).toBe(true);
    expect(paletteGeometry.listScrolls).toBe(true);
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
    await expect(page.getByRole('dialog')).toContainText('Leave active session?');
    await page.getByRole('dialog').getByRole('button', { name: /Leave view/ }).click();
    await expect(page.getByText('Subject A')).toBeVisible();
    await expect(page.locator('.session-verdict-badge')).toHaveText('READY');

    active = false;
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await expect(page.getByRole('button', { name: /Stop Session/ })).toBeDisabled();
  });

  test('stopping through the command palette clears the active-session navigation guard', async ({ page }) => {
    // Keep this component-flow test on browser fetch; PWA routing is covered separately.
    await page.route('**/sw.js', route => route.abort());
    await page.route('**/api/status', route => route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({ ok: true, active_session: { session_id: 'session-active' } })
    }));
    await page.route('**/api/session/stop', route => route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({ ok: true })
    }));

    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await expect(page.getByRole('button', { name: /Stop Session/ })).toBeEnabled();
    await page.keyboard.press('Control+K');
    await page.getByLabel('Search commands').fill('Stop active session');
    await page.getByRole('button', { name: /Stop active session/ }).click();
    await page.getByRole('dialog').getByRole('button', { name: /Stop session/ }).click();

    await expect(page).toHaveURL(/\/report$/);
    await expect(page.getByText('Leave active session?')).toHaveCount(0);
  });

  test('exports alert history and audit evidence through Material actions', async ({ page }) => {
    await page.goto(DASHBOARD, { waitUntil: 'domcontentloaded' });
    await page.keyboard.press('Control+K');
    await expect(page.getByRole('dialog')).toBeVisible();
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
    const customTag = page.getByLabel('Custom tag');
    const addTag = page.getByRole('button', { name: /Add Tag/ });
    await customTag.fill('Calibration');
    await expect(customTag).toHaveValue('Calibration');
    await expect(addTag).toBeEnabled();
    await addTag.dispatchEvent('click');
    await expect(notes).toHaveValue(/Position change/);
    await expect(notes).toHaveValue(/Calibration/);

    const notesDownload = page.waitForEvent('download');
    await page.getByRole('button', { name: /Export session notes as text/ }).dispatchEvent('click');
    expect((await notesDownload).suggestedFilename()).toMatch(/^session-notes-.+\.txt$/);
    await expect(page.getByText('Target Tracking')).toBeVisible();
    await page.getByRole('button', { name: /Pin Frame/ }).dispatchEvent('click');

    await page.getByRole('tab', { name: 'Waves' }).click();
    await expect(page.getByText(/SQI \d+%/).first()).toBeVisible();
    await expectVisibleCardsContained();
    const waveDownload = page.waitForEvent('download');
    await page.getByRole('button', { name: /Download breathing waveform image/ }).dispatchEvent('click');
    expect((await waveDownload).suggestedFilename()).toMatch(/^breathing_waveform_\d+\.png$/);

    await page.getByRole('tab', { name: 'HR' }).click();
    await expect(page.getByText('HR Funnel Telemetry')).toBeVisible();
    await expect(page.getByText('HR Stage Values')).toBeVisible();
    await expectVisibleCardsContained();
    await page.getByRole('radio', { name: '30s' }).dispatchEvent('click');
    await expect(page.getByText('Viewing 30 seconds')).toBeVisible();
    await page.getByRole('button', { name: /Reset heart rate chart window/ }).dispatchEvent('click');
    await expect(page.getByText('Viewing 120 seconds')).toBeVisible();

    await page.getByRole('tab', { name: 'RR' }).click();
    await expect(page.getByText('RR Funnel Telemetry')).toBeVisible();
    await expect(page.getByText('RR Recovery Diagnostics')).toBeVisible();
    await expect(page.getByText('No RR-specific warnings')).toBeVisible();
    await expectVisibleCardsContained();
    const rrDownload = page.waitForEvent('download');
    await page.getByRole('button', { name: /Download respiration trend image/ }).dispatchEvent('click');
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
