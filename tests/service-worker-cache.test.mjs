import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import test from 'node:test';
import vm from 'node:vm';

const source = await readFile(new URL('../assets/sw.js', import.meta.url), 'utf8');
const scope = 'https://trainer.example/app/';
const canonicalShell = `${scope}index.html`;

function response(label) {
  return {
    label,
    ok: true,
    clone() {
      return response(label);
    }
  };
}

function createHarness(fetchImpl, cached = new Map()) {
  const listeners = new Map();
  const cachePuts = [];
  const cacheMatches = [];
  const fetched = [];
  const cache = {
    addAll: async () => undefined,
    delete: async () => true,
    async put(key, value) {
      cachePuts.push(String(key));
      cached.set(String(key), value);
    },
    async match(key) {
      cacheMatches.push(String(key));
      return cached.get(String(key));
    }
  };
  const context = {
    URL,
    clearTimeout,
    console,
    setTimeout,
    caches: {
      open: async () => cache,
      keys: async () => [],
      delete: async () => true
    },
    fetch: async request => {
      fetched.push(request);
      return fetchImpl(request);
    },
    self: {
      registration: { scope },
      skipWaiting: () => undefined,
      clients: {
        claim: async () => undefined,
        matchAll: async () => []
      },
      addEventListener(type, listener) {
        listeners.set(type, listener);
      }
    }
  };
  vm.runInNewContext(source, context, { filename: 'assets/sw.js' });

  async function dispatch(url, mode = 'navigate') {
    const request = {
      url,
      method: 'GET',
      mode,
      headers: { get: () => '' }
    };
    let handled;
    listeners.get('fetch')({
      request,
      respondWith(value) {
        handled = Promise.resolve(value);
      }
    });
    return handled ? handled : Promise.resolve(undefined);
  }

  return { cacheMatches, cachePuts, dispatch, fetched };
}

test('online Angular routes cache only the canonical shell key', async () => {
  const harness = createHarness(async () => response('online'));
  const sensitiveRoute = `${scope}report?pair=123456&subject=P-001`;

  await harness.dispatch(sensitiveRoute);

  assert.equal(harness.fetched[0].url, sensitiveRoute);
  assert.deepEqual(harness.cachePuts, [canonicalShell]);
  assert.equal(harness.cachePuts.some(key => key.includes('pair=') || key.includes('/report')), false);
});

test('offline Angular routes look up only canonical shell fallbacks', async () => {
  const cached = new Map([[canonicalShell, response('offline shell')]]);
  const harness = createHarness(async () => {
    throw new Error('offline');
  }, cached);
  const sensitiveRoute = `${scope}home?pair=654321`;

  const result = await harness.dispatch(sensitiveRoute);

  assert.equal(result.label, 'offline shell');
  assert.deepEqual(harness.cacheMatches, [canonicalShell]);
  assert.equal(harness.cacheMatches.some(key => key.includes('pair=') || key.includes('/home')), false);
});

test('pairing pages remain network-only and never replace the app shell', async () => {
  const harness = createHarness(async () => response('pair page'));

  const result = await harness.dispatch(`${scope}pair?pair=123456`);

  assert.equal(result.label, 'pair page');
  assert.deepEqual(harness.cachePuts, []);
  assert.deepEqual(harness.cacheMatches, []);
});
