import { defineConfig, devices } from '@playwright/test';

export default defineConfig({
  testDir: '.',
  timeout: 60000,
  expect: {
    toHaveScreenshot: {
      maxDiffPixelRatio: 0.01
    }
  },
  projects: [
    { name: '390x844', use: { viewport: { width: 390, height: 844 } } },
    { name: '393x852', use: { viewport: { width: 393, height: 852 }, ...devices['iPhone 14 Pro'] } },
    { name: '412x915', use: { viewport: { width: 412, height: 915 } } },
    { name: '768x1024', use: { viewport: { width: 768, height: 1024 } } },
    { name: '1280x800', use: { viewport: { width: 1280, height: 800 } } }
  ]
});
