import { test, expect } from '@playwright/test';

test.describe('OTA Update and Version Info Smoke Tests', () => {
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

  test('displays product version and metadata in settings', async ({ page }) => {
    await page.goto('/settings', { waitUntil: 'domcontentloaded' });
    // Verify Update & Version Info card is visible
    const card = page.locator('.update-info-card');
    await expect(card).toBeVisible();

    // Verify product version is rendered dynamically (should be 16.1.0 in mock/production)
    const productVersion = card.locator('.settings-copy', { hasText: 'Product Version' }).locator('.row-description');
    await expect(productVersion).toContainText('16.1.0');

    // Verify accessibility attributes on the button
    const checkBtn = card.getByRole('button', { name: 'Check for Updates' });
    await expect(checkBtn).toBeVisible();
    await expect(checkBtn).toHaveAttribute('aria-describedby', 'updateResultRegion');
  });

  test('checks for updates successfully when a newer version is available', async ({ page }) => {
    // Mock update manifest with a newer version
    await page.route('**/api/update/manifest', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          product_version: '16.1.1',
          release_tag: 'v16.1.1',
          release_version: '16.1.1',
          build_number: 12,
          minimum_supported: '16.0.0',
          released_at: '2026-06-06T12:00:00Z',
          artifacts: {
            apk: {
              url: 'https://github.com/blayalems/radar-vital-mmwave-60-davao/releases/download/v16.1.1/radar-vital-release.apk',
              size_bytes: 15728640,
              sha256: 'a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2',
              compatibility: 'Android 8.0+'
            },
            exe: {
              url: 'https://github.com/blayalems/radar-vital-mmwave-60-davao/releases/download/v16.1.1/radar-vital-windows-installer.exe',
              size_bytes: 26214400,
              sha256: 'f6e5d4c3b2a1f6e5d4c3b2a1f6e5d4c3b2a1f6e5d4c3b2a1f6e5d4c3b2a1f6e5',
              compatibility: 'Windows 10+'
            }
          }
        })
      });
    });

    await page.goto('/settings', { waitUntil: 'domcontentloaded' });
    const card = page.locator('.update-info-card');
    const checkBtn = card.getByRole('button', { name: 'Check for Updates' });

    // Click check for updates and observe busy state
    await checkBtn.click();

    // Result region should exist and update with status
    const resultRegion = card.locator('#updateResultRegion');
    await expect(resultRegion).toBeVisible();
    await expect(resultRegion).toHaveAttribute('role', 'status');
    await expect(resultRegion).toHaveAttribute('aria-live', 'polite');

    // Title should show Update Available
    const titleText = resultRegion.locator('.update-title-text');
    await expect(titleText).toContainText('Update Available: v16.1.1');

    // Verify download links are shown
    const downloadApk = resultRegion.getByRole('link', { name: 'Download' }).first();
    const downloadExe = resultRegion.getByRole('link', { name: 'Download' }).last();
    await expect(downloadApk).toHaveAttribute('href', /.*radar-vital-release.apk/);
    await expect(downloadExe).toHaveAttribute('href', /.*radar-vital-windows-installer.exe/);

    // Verify snackbar alert
    await expect(page.locator('simple-snack-bar').last()).toContainText('New update 16.1.1 is available.');
  });

  test('checks for updates successfully when a newer build of the same version is available', async ({ page }) => {
    // Mock update manifest with a newer build (same version, different release tag)
    await page.route('**/api/update/manifest', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          product_version: '16.1.0',
          release_tag: 'v16.1.0-build2',
          release_version: '16.1.0',
          build_number: 15,
          minimum_supported: '16.0.0',
          released_at: '2026-06-06T13:00:00Z',
          artifacts: {
            apk: {
              url: 'https://github.com/blayalems/radar-vital-mmwave-60-davao/releases/download/v16.1.0-build2/radar-vital-release.apk',
              size_bytes: 15728640,
              sha256: 'a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4e5f6a1b2',
              compatibility: 'Android 8.0+'
            }
          }
        })
      });
    });

    await page.goto('/settings', { waitUntil: 'domcontentloaded' });
    const card = page.locator('.update-info-card');
    const checkBtn = card.getByRole('button', { name: 'Check for Updates' });

    await checkBtn.click();

    const resultRegion = card.locator('#updateResultRegion');
    await expect(resultRegion).toBeVisible();

    // Title should show New Build Available
    const titleText = resultRegion.locator('.update-title-text');
    await expect(titleText).toContainText('New Build Available: v16.1.0-build2');

    // Verify snackbar alert
    await expect(page.locator('simple-snack-bar').last()).toContainText('New build of 16.1.0 is available.');
  });

  test('checks for updates and shows up-to-date state when no updates are available', async ({ page }) => {
    // Mock update manifest with the same version and release tag
    await page.route('**/api/update/manifest', async (route) => {
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        body: JSON.stringify({
          product_version: '16.1.0',
          release_tag: 'v16.1.0',
          release_version: '16.1.0',
          build_number: 10,
          minimum_supported: '16.0.0',
          released_at: '2026-06-06T10:00:00Z',
          artifacts: {}
        })
      });
    });

    await page.goto('/settings', { waitUntil: 'domcontentloaded' });
    const card = page.locator('.update-info-card');
    const checkBtn = card.getByRole('button', { name: 'Check for Updates' });

    await checkBtn.click();

    const resultRegion = card.locator('#updateResultRegion');
    await expect(resultRegion).toBeVisible();

    // Should indicate up-to-date state
    await expect(resultRegion).toContainText('Your dashboard is up to date');

    // Verify snackbar alert
    await expect(page.locator('simple-snack-bar').last()).toContainText('Your dashboard is up to date.');
  });

  test('public route invariant: /api/update/manifest is accessible without authentication', async ({ request }) => {
    // Request public manifest proxy route directly without auth headers
    const resp = await request.get('/api/update/manifest');

    // We expect either 200 (if upstream succeeds or is mocked) or 502 (if proxy fails on network),
    // but NEVER a 401 Unauthorized because it should bypass auth check.
    expect(resp.status()).not.toBe(401);
    expect(resp.status()).not.toBe(403);
  });
});
