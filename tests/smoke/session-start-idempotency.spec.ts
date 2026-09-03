import { expect, test, type Page, type Route } from '@playwright/test';
import { seedFirstRunComplete } from './helpers/first-run';

const HOME = '/home';
const PRODUCT_VERSION = '16.6.1';

type StartPayload = {
  idempotency_key?: unknown;
  participant_id?: unknown;
  trial_id?: unknown;
};

type StartHandler = (route: Route, payload: StartPayload) => Promise<void>;

const passingChecks = [
  { id: 'serial_port_list', label: 'Serial', status: 'good', description: 'COM7 detected.' },
  { id: 'session_folder_writable', label: 'Storage', status: 'good', description: 'Session root is writable.' },
  { id: 'schema_hash_consistency', label: 'Schema', status: 'good', description: 'Schema is current.' },
  { id: 'clock_monotonic_sanity', label: 'Clock', status: 'good', description: 'Clock is monotonic.' }
];

function json(route: Route, body: unknown, status = 200): Promise<void> {
  return route.fulfill({
    status,
    contentType: 'application/json',
    body: JSON.stringify(body)
  });
}

async function seedParticipantSession(page: Page): Promise<void> {
  await seedFirstRunComplete(page);
  await page.addInitScript(() => {
    sessionStorage.setItem('rvt-operator-token', 'idempotency-test-token');
    localStorage.setItem('rvt.server.url', location.origin);
    localStorage.removeItem('rvt-demo-mode');
    localStorage.setItem('rvt-setup', JSON.stringify({
      duration_s: 150,
      customDuration: 150,
      customUnit: 's',
      radar_port: 'COM7',
      ble_address: 'AA:BB:CC:DD:EE:01',
      ble_profile: 'ailink_oximeter',
      notify_char: '0000ffe2-0000-1000-8000-00805f9b34fb',
      subject_label: 'P-001',
      operator_label: 'Idempotency Tester',
      station_label: 'QMS Station 1',
      subject_profile_id: 'adult_default',
      participant_id: 'P-001',
      study_mode: 'confirmatory',
      condition_id: 'd060_none',
      distance_m: 0.6,
      barrier_type: 'none',
      trial_number: 1,
      skip_countdown: true
    }));
  });
}

async function mockControlApi(
  page: Page,
  startHandler: StartHandler,
  participantStatus: 'active' | 'withdrawn' = 'active'
): Promise<void> {
  await page.route('**/api/**', async route => {
    const url = new URL(route.request().url());
    const path = url.pathname;

    if (path === '/api/session/start') {
      const payload = route.request().postDataJSON() as StartPayload;
      await startHandler(route, payload);
      return;
    }
    if (path === '/api/health') {
      await json(route, { ok: true });
      return;
    }
    if (path === '/api/status') {
      await json(route, {
        ok: true,
        mode: 'live',
        product_version: PRODUCT_VERSION,
        trainer_version: PRODUCT_VERSION,
        dashboard_version: PRODUCT_VERSION,
        firmware_expected: `v${PRODUCT_VERSION}`,
        serial_protocol: 'v15.2',
        serial_width_expected: 222,
        schema_versions: {
          control_api: 'rvt-control-api-v12.0',
          study_session: 'rvt-study-session-v16.5.9'
        },
        active_session: null
      });
      return;
    }
    if (path === '/api/version') {
      await json(route, {
        product_version: PRODUCT_VERSION,
        trainer: PRODUCT_VERSION,
        dashboard: PRODUCT_VERSION,
        firmware_expected: `v${PRODUCT_VERSION}`,
        serial_protocol: 'v15.2',
        serial_width_expected: 222,
        schema_versions: {
          control_api: 'rvt-control-api-v12.0',
          study_session: 'rvt-study-session-v16.5.9'
        }
      });
      return;
    }
    if (path === '/api/auth/validate') {
      await json(route, {
        ok: true,
        operator: {
          operator_id: 'op_idempotency',
          display_name: 'Idempotency Tester',
          initials: 'IT'
        }
      });
      return;
    }
    if (path === '/api/operator-profiles') {
      await json(route, {
        schema_version: 'rvt-operator-profiles-v12.0',
        profiles: [{
          operator_id: 'op_idempotency',
          display_name: 'Idempotency Tester',
          initials: 'IT'
        }]
      });
      return;
    }
    if (path === '/api/defaults') {
      await json(route, { radar_port: 'COM7', serial_ports: ['COM7'] });
      return;
    }
    if (path === '/api/serial/ports') {
      await json(route, { ports: [{ device: 'COM7' }], selected: 'COM7' });
      return;
    }
    if (path === '/api/subject-profiles') {
      await json(route, { profiles: {} });
      return;
    }
    if (path === '/api/participants') {
      await json(route, {
        schema_version: 'rvt-participant-profiles-v16.5.9',
        participants: [{
          participant_id: 'P-001',
          display_code: 'P-001',
          status: participantStatus,
          completed_trials: 0
        }]
      });
      return;
    }
    if (path === '/api/preflight') {
      await json(route, { checks: passingChecks });
      return;
    }
    if (path === '/api/sessions') {
      await json(route, { items: [] });
      return;
    }
    if (path === '/api/events/token') {
      await json(route, { token: 'idempotency-sse-token' });
      return;
    }
    if (path === '/api/events/subscribe') {
      await route.fulfill({
        status: 200,
        contentType: 'text/event-stream',
        body: 'event: heartbeat\ndata: {"ok":true}\n\n'
      });
      return;
    }

    await json(route, { ok: true });
  });
}

async function openReadyHome(page: Page, startHandler: StartHandler) {
  await seedParticipantSession(page);
  await mockControlApi(page, startHandler);
  await page.goto(HOME, { waitUntil: 'domcontentloaded' });
  const start = page.locator('.start-session-btn');
  await expect(start).toBeVisible();
  await expect(start).toBeEnabled();
  return start;
}

test.describe('session Start browser idempotency', () => {
  test.use({ serviceWorkers: 'block' });

  test('coalesces duplicate pointer and keyboard activation into one delayed POST', async ({ page }) => {
    const keys: string[] = [];
    let releaseResponse!: () => void;
    const responseGate = new Promise<void>(resolve => {
      releaseResponse = resolve;
    });
    let markPosted!: () => void;
    const posted = new Promise<void>(resolve => {
      markPosted = resolve;
    });

    const start = await openReadyHome(page, async (route, payload) => {
      keys.push(String(payload.idempotency_key || ''));
      expect(payload.participant_id).toBe('P-001');
      expect(payload.trial_id).toBe('P-001-d060_none-t1');
      markPosted();
      await responseGate;
      await json(route, { ok: true, session_id: 's-idempotent-01' });
    });

    await start.evaluate(element => {
      element.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true }));
      element.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true }));
    });
    await posted;
    await page.keyboard.press('n');
    await page.waitForTimeout(150);

    expect(keys).toHaveLength(1);
    expect(new Set(keys).size).toBe(1);
    expect(keys[0]).toMatch(/^[A-Za-z0-9_-]{16,128}$/);
    await expect(start).toBeDisabled();
    await expect(page.getByRole('status').filter({ hasText: /repeated Start commands are ignored/i })).toBeVisible();

    releaseResponse();
    await expect(page).toHaveURL(/\/live$/);
  });

  test('reuses the same key when an ambiguous response loss is explicitly retried', async ({ page }) => {
    const keys: string[] = [];

    const start = await openReadyHome(page, async (route, payload) => {
      keys.push(String(payload.idempotency_key || ''));
      if (keys.length === 1) {
        await route.abort('failed');
        return;
      }
      await json(route, { ok: true, session_id: 's-idempotent-replay' });
    });

    await start.click();
    await expect(page.getByText(/Retry Start to safely resume the same request/i)).toBeVisible();
    await expect(start).toBeEnabled();
    await start.click();

    await expect(page).toHaveURL(/\/live$/);
    expect(keys).toHaveLength(2);
    expect(keys[0]).toMatch(/^[A-Za-z0-9_-]{16,128}$/);
    expect(keys[1]).toBe(keys[0]);
  });

  test('allocates a new key after a definitive HTTP conflict', async ({ page }) => {
    const keys: string[] = [];

    const start = await openReadyHome(page, async (route, payload) => {
      keys.push(String(payload.idempotency_key || ''));
      if (keys.length === 1) {
        await json(route, {
          ok: false,
          error: {
            code: 'IDEMPOTENCY_KEY_CONFLICT',
            message: 'The idempotency key was already used for a different Start request.'
          }
        }, 409);
        return;
      }
      await json(route, { ok: true, session_id: 's-idempotent-new-intent' });
    });

    await start.click();
    await expect(page.getByText(/already used for a different Start request/i)).toBeVisible();
    await expect(start).toBeEnabled();
    await start.click();

    await expect(page).toHaveURL(/\/live$/);
    expect(keys).toHaveLength(2);
    expect(keys[0]).toMatch(/^[A-Za-z0-9_-]{16,128}$/);
    expect(keys[1]).toMatch(/^[A-Za-z0-9_-]{16,128}$/);
    expect(keys[1]).not.toBe(keys[0]);
  });

  test('fails closed when the cached participant has withdrawn from the live roster', async ({ page }) => {
    let startPosts = 0;
    await seedParticipantSession(page);
    await mockControlApi(page, async route => {
      startPosts += 1;
      await json(route, { ok: true, session_id: 'must-not-start' });
    }, 'withdrawn');

    await page.goto(HOME, { waitUntil: 'domcontentloaded' });

    const withdrawnParticipant = page.getByRole('radio', { name: /P-001, status withdrawn/i });
    await expect(withdrawnParticipant).toBeDisabled();
    await expect(withdrawnParticipant).toHaveAttribute('aria-checked', 'false');
    await expect(page.locator('.start-session-btn')).toBeDisabled();
    await expect(page.getByText('Select a coded participant profile.')).toBeVisible();
    await expect.poll(() => page.evaluate(() => {
      const setup = JSON.parse(localStorage.getItem('rvt-setup') || '{}');
      return { participantId: setup.participant_id, subjectLabel: setup.subject_label };
    })).toEqual({ participantId: '', subjectLabel: '' });
    expect(startPosts).toBe(0);
  });
});
