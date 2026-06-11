import { defineConfig, devices } from '@playwright/test';

const PYTHON = process.env.PYTHON || 'python3';
const TRAINER_PORT = process.env.RVT_TEST_PORT ? parseInt(process.env.RVT_TEST_PORT, 10) : 8989;
const BASE_URL = `http://127.0.0.1:${TRAINER_PORT}`;
const TRAINER_SESSIONS_ROOT = process.env.RVT_TEST_SESSIONS_ROOT || '.playwright-state/sessions';

export default defineConfig({
  testDir: 'tests',
  fullyParallel: false,
  retries: process.env.CI ? 1 : 0,
  workers: 1,
  reporter: process.env.CI ? [['github'], ['html', { open: 'never' }]] : 'list',
  timeout: 60_000,
  expect: { timeout: 15000, toHaveScreenshot: { maxDiffPixels: 200 } },
  use: {
    baseURL: BASE_URL,
    actionTimeout: 15000,
    navigationTimeout: 15_000,
    trace: 'retain-on-failure',
    screenshot: 'only-on-failure',
    video: 'retain-on-failure'
  },
  projects: [
    { name: 'desktop',     use: { ...devices['Desktop Chrome'], viewport: { width: 1280, height: 800 } } },
    { name: 'pixel-7',     use: { ...devices['Pixel 7'] } },
    { name: 'iphone-14',   use: { ...devices['iPhone 14 Pro'] } },
    { name: 'ipad',        use: { ...devices['iPad (gen 7)'] } }
  ],
  webServer: {
    command: `${PYTHON} radar_vital_trainer_v12_for_v16_0.py serve --mock --no-browser --control-port ${TRAINER_PORT} --host 127.0.0.1 --sessions-root ${TRAINER_SESSIONS_ROOT}`,
    url: `${BASE_URL}/api/health`,
    reuseExistingServer: !process.env.CI,
    stdout: 'pipe',
    stderr: 'pipe',
    timeout: 30_000
  }
});
