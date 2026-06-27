import { test, expect, type Page } from '@playwright/test';
import fs from 'fs/promises';
import http from 'http';
import path from 'path';
import { seedFirstRunComplete } from './helpers/first-run';

const WWW = path.resolve(process.cwd(), 'www');

function contentType(filePath: string): string {
  const ext = path.extname(filePath).toLowerCase();
  if (ext === '.html') return 'text/html; charset=utf-8';
  if (ext === '.js') return 'text/javascript; charset=utf-8';
  if (ext === '.css') return 'text/css; charset=utf-8';
  if (ext === '.json' || ext === '.webmanifest') return 'application/manifest+json; charset=utf-8';
  if (ext === '.png') return 'image/png';
  if (ext === '.ico') return 'image/x-icon';
  if (ext === '.woff2') return 'font/woff2';
  return 'application/octet-stream';
}

async function startStaticPagesServer(): Promise<{ origin: string; close: () => Promise<void> }> {
  const server = http.createServer(async (req, res) => {
    const url = new URL(req.url || '/', 'http://127.0.0.1');
    if (url.pathname.startsWith('/api/')) {
      res.writeHead(404, { 'Content-Type': 'application/json; charset=utf-8' });
      res.end(JSON.stringify({ ok: false, error: 'No trainer is attached to the static PWA.' }));
      return;
    }

    const decodedPath = decodeURIComponent(url.pathname);
    const shellFallback = decodedPath === '/' || !path.extname(decodedPath);
    const requested = shellFallback ? '/404.html' : decodedPath;
    const target = path.resolve(WWW, `.${requested}`);
    if (!target.startsWith(WWW)) {
      res.writeHead(403);
      res.end('Forbidden');
      return;
    }

    try {
      const body = await fs.readFile(target);
      res.writeHead(200, { 'Content-Type': contentType(target), 'Cache-Control': 'no-store' });
      res.end(body);
    } catch (_) {
      const body = await fs.readFile(path.join(WWW, '404.html'));
      res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store' });
      res.end(body);
    }
  });

  await new Promise<void>((resolve, reject) => {
    server.once('error', reject);
    server.listen(0, '127.0.0.1', () => resolve());
  });
  const address = server.address();
  if (!address || typeof address === 'string') throw new Error('Static server did not bind a TCP port.');
  return {
    origin: `http://127.0.0.1:${address.port}`,
    close: () => new Promise<void>((resolve, reject) => server.close(error => error ? reject(error) : resolve()))
  };
}

async function createSandboxOperator(page: Page): Promise<void> {
  const overlay = page.locator('section.idle-lock-overlay');
  await expect(overlay).toBeVisible({ timeout: 25_000 });
  await expect(overlay.getByRole('heading', { name: /Create Operator/ })).toBeVisible();

  await overlay.getByLabel('Display Name').fill('Lemuel Biaya');
  await overlay.getByLabel('Initials').fill('LB');
  for (const digit of ['1', '2', '3', '4', '5', '6']) {
    await overlay.locator('.onboarding-flow .keyboard-grid button', { hasText: new RegExp(`^${digit}$`) }).click();
  }
  await overlay.getByRole('button', { name: /Create Profile/ }).click();
  await expect(overlay).toHaveCount(0, { timeout: 15_000 });
}

test.describe('Static hosted PWA operator auth', () => {
  test('creates, validates, and reopens a sandbox operator from /live without a trainer', async ({ page, browserName }) => {
    const server = await startStaticPagesServer();
    try {
      await seedFirstRunComplete(page);
      await page.goto(`${server.origin}/live`, { waitUntil: 'domcontentloaded' });
      await page.locator('#rvtLoadingOverlay').waitFor({ state: 'hidden', timeout: 20_000 }).catch(() => {});
      // First-run visitors with no configured trainer are routed to the Connect
      // wizard (PR-69). Choose sandbox mode to reach /live, where the operator
      // onboarding overlay appears.
      await page.waitForURL(/\/connect$/, { timeout: 15_000 });
      await page.getByRole('button', { name: /Demo Now/ }).click();
      await page.waitForURL(/\/live$/, { timeout: 15_000 });
      await createSandboxOperator(page);
      await expect(page.locator('#demoBanner')).toBeVisible();

      const stored = await page.evaluate(() => ({
        profiles: JSON.parse(localStorage.getItem('demo:rvt-operator-profiles') || '{"profiles":[]}').profiles,
        token: sessionStorage.getItem('rvt-operator-token'),
        sessions: JSON.parse(sessionStorage.getItem('demo:rvt-operator-sessions') || '{}')
      }));
      expect(stored.profiles).toHaveLength(1);
      expect(stored.profiles[0].display_name).toBe('Lemuel Biaya');
      expect(stored.token).toMatch(/^sandbox_op_token_/);
      expect(Object.keys(stored.sessions)).toContain(stored.token);

      await page.goto(`${server.origin}/live`, { waitUntil: 'domcontentloaded' });
      await page.locator('#rvtLoadingOverlay').waitFor({ state: 'hidden', timeout: 20_000 }).catch(() => {});
      await expect(page.locator('section.idle-lock-overlay')).toHaveCount(0, { timeout: 15_000 });
      await expect(page.locator('.operator-badge .status-txt')).toHaveText('Lemuel Biaya');

      await page.evaluate(async () => {
        await navigator.serviceWorker.ready;
      });
      await page.reload({ waitUntil: 'domcontentloaded' });
      await page.waitForFunction(() => Boolean(navigator.serviceWorker.controller), undefined, { timeout: 15_000 });

      if (browserName !== 'webkit') {
        await page.context().setOffline(true);
        try {
          await page.goto(`${server.origin}/live`, { waitUntil: 'domcontentloaded' });
          await page.locator('#rvtLoadingOverlay').waitFor({ state: 'hidden', timeout: 20_000 }).catch(() => {});
          await expect(page.locator('app-layout')).toBeVisible();
          await expect(page.locator('section.idle-lock-overlay')).toHaveCount(0, { timeout: 15_000 });
        } finally {
          await page.context().setOffline(false);
        }
      }
    } finally {
      await server.close();
    }
  });
});
