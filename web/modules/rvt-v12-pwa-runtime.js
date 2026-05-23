(function(){
  'use strict';
  const API_BASE_KEY = 'rvt-api-base';
  const TOKEN_KEY = 'rvt-pair-token';
  const DB_NAME = 'RVT-v12';
  const DB_VERSION = 1;
  const browserFetch = window.fetch ? window.fetch.bind(window) : null;
  let replayingHistory = false;

  function qs(){ try { return new URLSearchParams(location.search || ''); } catch (_) { return new URLSearchParams(); } }
  function isTauriNative(){
    return !!window.__TAURI_INTERNALS__ || location.hostname === 'tauri.localhost';
  }
  function normalizeApiBase(value){
    const raw = String(value || '').trim().replace(/\/+$/, '');
    if (!raw) return '';
    try {
      const u = new URL(raw, location.href);
      if (!/^https?:$/.test(u.protocol)) return '';
      return u.origin;
    } catch (_) { return ''; }
  }
  function currentApiBase(){
    try {
      const stateBase = normalizeApiBase(window.S?.apiBase || '');
      const storedBase = normalizeApiBase(localStorage.getItem(API_BASE_KEY));
      const base = stateBase || storedBase;
      if (window.S) window.S.apiBase = base;
      return base;
    } catch (_) { return ''; }
  }
  function setDemoBannerVisible(on){
    document.documentElement.dataset.sandbox = on ? '1' : '';
    if (!on) delete document.documentElement.dataset.sandbox;
    const banner = document.getElementById('demoBanner');
    if (banner) banner.hidden = !on;
    document.querySelectorAll('.demo-banner-clone').forEach(el => { el.hidden = !on; });
  }
  window.setDemoBannerVisible = setDemoBannerVisible;
  window.injectDemoBanner = function(rootEl){
    const root = rootEl || document.body;
    if (!root || root.querySelector?.('.demo-banner-clone')) return;
    const clone = document.createElement('div');
    clone.className = 'demo-banner-clone';
    clone.setAttribute('role','status');
    clone.innerHTML = '<span class="material-symbols-rounded" aria-hidden="true">warning</span>DEMO MODE - simulated vitals only; do not use for a live subject.';
    clone.hidden = document.documentElement.dataset.sandbox !== '1';
    root.insertBefore(clone, root.firstChild);
  };

  async function configureNativeStatusBar(){
    try {
      const cap = window.Capacitor;
      if (!cap?.isNativePlatform?.()) return;
      const statusBar = cap?.Plugins?.StatusBar || window.StatusBar;
      if (!statusBar) return;
      await statusBar.setOverlaysWebView?.({ overlay: false });
      await statusBar.setBackgroundColor?.({ color: '#f4f6fb' });
      await statusBar.setStyle?.({ style: 'LIGHT' });
    } catch (_) {}
  }
  configureNativeStatusBar();

  function openDb(){
    if (!('indexedDB' in window)) return Promise.resolve(null);
    return new Promise((resolve) => {
      const req = indexedDB.open(DB_NAME, DB_VERSION);
      req.onupgradeneeded = () => {
        const db = req.result;
        if (!db.objectStoreNames.contains('lastKnown')) db.createObjectStore('lastKnown');
      };
      req.onsuccess = () => resolve(req.result);
      req.onerror = () => resolve(null);
    });
  }
  async function putLastKnown(kind, payload){
    const db = await openDb(); if (!db) return;
    const tx = db.transaction('lastKnown','readwrite');
    tx.objectStore('lastKnown').put({ payload, at: Date.now() }, kind);
  }
  async function getLastKnown(kind){
    const db = await openDb(); if (!db) return null;
    return new Promise(resolve => {
      const tx = db.transaction('lastKnown','readonly');
      const req = tx.objectStore('lastKnown').get(kind);
      req.onsuccess = () => resolve(req.result || null);
      req.onerror = () => resolve(null);
    });
  }

  async function nativeHttpRequest(url, init){
    const cap = window.Capacitor;
    const http = cap?.Plugins?.CapacitorHttp || cap?.Plugins?.Http || window.CapacitorHttp || window.Http;
    if (!cap?.isNativePlatform?.() || !http?.request) return null;
    if (!/^https?:\/\//i.test(String(url || ''))) return null;
    const method = String(init?.method || 'GET').toUpperCase();
    const headers = {};
    new Headers(init?.headers || {}).forEach((v,k) => { headers[k] = v; });
    const resp = await http.request({ url, method, headers, data: init?.body });
    const status = Number(resp.status || 0);
    return {
      ok: status >= 200 && status < 300,
      status,
      json: async () => typeof resp.data === 'string' ? JSON.parse(resp.data || '{}') : (resp.data || {}),
      text: async () => typeof resp.data === 'string' ? resp.data : JSON.stringify(resp.data || {})
    };
  }
  window.api = async function(path, init){
    const base = currentApiBase();
    const target = /^[a-z][a-z0-9+.-]*:\/\//i.test(String(path)) ? String(path) : base + String(path);
    const headers = new Headers(init?.headers || {});
    const tok = sessionStorage.getItem(TOKEN_KEY);
    if (tok) headers.set('X-RVT-Auth', tok);
    const opts = Object.assign({ cache: 'no-store' }, init || {}, { headers });
    const nativeResp = await nativeHttpRequest(target, opts);
    if (nativeResp) return nativeResp;
    if (!browserFetch) throw new Error('fetch unavailable');
    return browserFetch(target, opts);
  };
  window.apiES = function(path){
    const base = currentApiBase();
    // SSE is read-only in v12; do not pass the control token in a URL.
    return new EventSource(base + path);
  };
  window.setApiBase = function(value){
    const normalized = normalizeApiBase(value);
    try {
      if (normalized) localStorage.setItem(API_BASE_KEY, normalized);
      else localStorage.removeItem(API_BASE_KEY);
    } catch (_) {}
    if (window.S) window.S.apiBase = normalized;
    return normalized;
  };

  async function consumePairPin(){
    const pin = qs().get('pair');
    if (!pin) return;
    const r = await window.api('/api/auth/exchange', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ pin })
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) {
      const err = new Error(j?.error?.message || `Pairing failed (${r.status})`);
      err.status = r.status;
      throw err;
    }
    if (j.token) sessionStorage.setItem(TOKEN_KEY, j.token);
    history.replaceState(null, '', location.pathname + location.hash);
  }

  const oldEnableSandbox = window.enableSandboxControlMode;
  if (typeof oldEnableSandbox === 'function') {
    window.enableSandboxControlMode = function(){
      const result = oldEnableSandbox.apply(this, arguments);
      setDemoBannerVisible(true);
      return result;
    };
  }
  const oldApplyLivePayload = window.applyLivePayload;
  if (typeof oldApplyLivePayload === 'function') {
    window.applyLivePayload = function(raw, opts){
      const ok = oldApplyLivePayload.apply(this, arguments);
      try {
        if (ok && window.S?.lastPayload) {
          putLastKnown(window.S?.ctl?.sandbox || window.S?.demoMode || window.S?.autoDemoActive ? 'demo' : 'live', window.S.lastPayload);
          window.S.lastDataAt = Date.now();
        }
      } catch (_) {}
      return ok;
    };
  }
  if (typeof window.apiJson === 'function') {
    window.apiJson = async function(url, opts = {}){
      if (window.S?.ctl?.sandbox && String(url).startsWith('/api/') && typeof window.sandboxApiJson === 'function') {
        return window.sandboxApiJson(url, opts);
      }
      const r = await window.api(url, opts);
      const j = await r.json().catch(() => null);
      if (!r.ok) {
        const err = new Error(j?.error?.message || j?.error || `HTTP ${r.status}`);
        err.status = r.status;
        err.body = j;
        throw err;
      }
      return j;
    };
  }
  window.detectControlMode = async function(){
    try {
      await consumePairPin();
      const r = await window.api('/api/status');
      if (!r.ok) return window.enableSandboxControlMode(`Control API returned HTTP ${r.status}`);
      const j = await r.json();
      window.S.ctl = Object.assign({}, window.S.ctl, { on: true, sandbox: false, status: j });
      setDemoBannerVisible(false);
      if (typeof window.checkTrainerVersion === 'function') window.checkTrainerVersion();
      return true;
    } catch (e) {
      return window.enableSandboxControlMode(e.message || 'Control API unavailable');
    }
  };

  const baseBackoff = window.pollBackoffMs;
  window.pollBackoffMs = function(){
    const attempt = Math.max(0, Number(window.S?.disc || 0));
    const base = Math.min(30000, 1000 * Math.pow(2, attempt));
    const jitter = base * 0.2 * (Math.random() - 0.5);
    return Math.max(250, base + jitter);
  };
  if (typeof baseBackoff !== 'function') window.pollBackoffMs.__fallback = true;

  window.startSse = function(){
    const S = window.S;
    if (!S?.ctl?.on || S.ctl.sandbox || !window.EventSource || S.sse) return false;
    try {
      const es = window.apiES('/api/events/subscribe');
      S.sse = es;
      es.addEventListener('open', () => {
        S.sseMode = true;
        S.sseErrors = [];
        if (S.pollTimer) { clearTimeout(S.pollTimer); S.pollTimer = null; }
        if (typeof window.updateConnModeBadge === 'function') window.updateConnModeBadge();
        if (typeof window.preloadSessionBuffer === 'function') window.preloadSessionBuffer();
      });
      es.addEventListener('live', ev => {
        try {
          const raw = JSON.parse(ev.data || '{}');
          if (window.applyLivePayload(raw, { allowOlder: false, allowMismatch: false })) S.lastDataAt = Date.now();
        } catch (e) { console.warn('SSE live parse failed', e); }
      });
      es.onerror = () => {
        const now = Date.now();
        S.sseErrors = (S.sseErrors || []).filter(t => now - t < 60000);
        S.sseErrors.push(now);
        if (S.sseErrors.length > 3) {
          try { es.close(); } catch (_) {}
          S.sse = null;
          S.sseMode = false;
          if (typeof window.toast === 'function') window.toast('SSE unstable; using polling','warning');
          if (typeof window.scheduleNextPoll === 'function') window.scheduleNextPoll(0);
        }
      };
      return true;
    } catch (e) {
      console.warn('SSE unavailable', e);
      return false;
    }
  };

  const oldPoll = window.poll;
  if (typeof oldPoll === 'function') {
    window.poll = async function(){
      const S = window.S;
      if (document.visibilityState === 'hidden') S.refreshMs = Math.max(5000, Number(S.refreshMs || 1000));
      if (S?.sseMode || S?.paused || S?.pollBusy || S?.demoMode) return oldPoll.apply(this, arguments);
      S.pollBusy = true;
      try {
        const controller = new AbortController();
        const fetchTimeout = setTimeout(() => controller.abort(), 8000);
        const path = String(S.dataUrl || '/api/session/current/live_dashboard.json').replace(/^\.\/live_dashboard\.json$/, '/api/session/current/live_dashboard.json');
        const res = await window.api(`${path}${path.includes('?') ? '&' : '?'}t=${Date.now()}`, { signal: controller.signal });
        clearTimeout(fetchTimeout);
        if (S.ctl?.on && !S.ctl.sandbox && res.status === 404 && typeof window.connectedPreviewPayload === 'function') {
          const p = window.connectedPreviewPayload('Connected trainer preview; no active session yet');
          S.lastPayload = p; S.disc = 0; S.autoDemoActive = false; window.render(p); window.scheduleNextPoll(S.refreshMs); return;
        }
        if (!res.ok) throw new Error('HTTP ' + res.status);
        const payload = await res.json();
        if (!window.applyLivePayload(payload,{allowOlder:false,allowMismatch:false})) { window.scheduleNextPoll(S.refreshMs); return; }
        S.lastDataAt = Date.now();
        window.scheduleNextPoll(S.refreshMs);
      } catch (e) {
        S.disc += 1;
        const cached = await getLastKnown('live');
        if (cached?.payload && Date.now() - cached.at < 60000 && typeof window.makeStalePayload === 'function') {
          const p = window.makeStalePayload(cached.payload, `Last updated ${Math.round((Date.now()-cached.at)/1000)} s ago`);
          S.lastPayload = p; window.render(p); window.scheduleNextPoll(window.pollBackoffMs()); return;
        }
        return oldPoll.apply(this, arguments);
      } finally {
        S.pollBusy = false;
        if (typeof window.exposeDebugState === 'function') window.exposeDebugState();
      }
    };
  }

  function installHistoryContract(){
    const priorView = window.switchView;
    const priorTab = window.switchTab;
    if (typeof priorView === 'function' && !priorView.__rvtV12History) {
      window.switchView = function(id, push = true){
        const view = ['home','live','report','help','settings'].includes(id) ? id : 'home';
        const result = priorView.call(this, view, false);
        if (push && !replayingHistory) history.pushState({ type:'view', view }, '', '#' + view);
        return result;
      };
      window.switchView.__rvtV12History = true;
    }
    if (typeof priorTab === 'function' && !priorTab.__rvtV12History) {
      window.switchTab = function(id){
        const result = priorTab.apply(this, arguments);
        const tab = String(id || '').replace(/^tab-/, '');
        if (!replayingHistory && (document.body.dataset.view || 'live') === 'live') {
          history.replaceState({ type:'tab', view:'live', tab }, '', '#live/' + tab);
        }
        return result;
      };
      window.switchTab.__rvtV12History = true;
    }
    window.addEventListener('popstate', ev => {
      replayingHistory = true;
      try {
        const st = ev.state || {};
        if (st.type === 'tab' && st.tab) window.switchTab('tab-' + st.tab);
        else if (st.type === 'view' && st.view) window.switchView(st.view, false);
        else if (location.hash) window.routeHash?.();
      } finally {
        replayingHistory = false;
      }
    });
  }

  function installChartLifecycle(){
    const tabCharts = {
      'tab-hr': ['hr','bucket'],
      'tab-rr': ['rr','rrDiag'],
      'tab-waves': ['breath','heart'],
      'tab-overview': ['kHr','kRr','kFps','kDist']
    };
    const priorTab = window.switchTab;
    if (typeof priorTab === 'function' && !priorTab.__rvtV12Charts) {
      window.switchTab = function(id){
        Object.values(window.S?.charts || {}).forEach(c => { try { c?.stop?.(); } catch (_) {} });
        const result = priorTab.apply(this, arguments);
        (tabCharts[id] || []).forEach(k => {
          const c = window.S?.charts?.[k];
          try { c?.resize?.(); c?.update?.('none'); } catch (_) {}
        });
        return result;
      };
      window.switchTab.__rvtV12Charts = true;
    }
    const priorView = window.switchView;
    if (typeof priorView === 'function' && !priorView.__rvtV12Charts) {
      window.switchView = function(id){
        const leavingLive = (document.body.dataset.view || '') === 'live' && id !== 'live';
        const result = priorView.apply(this, arguments);
        if (leavingLive) {
          ['hr','rr','rrDiag','breath','heart','bucket'].forEach(k => {
            const c = window.S?.charts?.[k];
            try { c?._ro?.disconnect?.(); c?.destroy?.(); delete window.S.charts[k]; } catch (_) {}
          });
        } else if (id === 'live' && typeof window.buildCharts === 'function' && !window.S?.charts?.hr) {
          try { window.buildCharts(); } catch (_) {}
        }
        return result;
      };
      window.switchView.__rvtV12Charts = true;
    }
  }

  function installVisibilityThrottle(){
    document.addEventListener('visibilitychange', () => {
      const S = window.S || {};
      if (document.visibilityState === 'hidden') {
        S._visibleRefreshMs = S.refreshMs || 1000;
        S.refreshMs = 5000;
        try { S.sse?.close?.(); } catch (_) {}
        S.sse = null; S.sseMode = false;
      } else {
        S.refreshMs = S._visibleRefreshMs || 1000;
        if (typeof window.startSse === 'function') window.startSse();
        if (typeof window.scheduleNextPoll === 'function') window.scheduleNextPoll(0);
      }
    });
  }

  function installServiceWorker(){
    if (document.documentElement.dataset.rvtArchive === '1') return;
    if (!('serviceWorker' in navigator)) return;
    if (isTauriNative()) {
      navigator.serviceWorker.getRegistrations?.().then(regs => regs.forEach(reg => reg.unregister())).catch(() => {});
      if (window.caches?.keys) {
        caches.keys().then(keys => keys.forEach(key => caches.delete(key))).catch(() => {});
      }
      return;
    }
    navigator.serviceWorker.register('/sw.js').then(reg => {
      reg.addEventListener('updatefound', () => {
        const w = reg.installing;
        if (!w) return;
        w.addEventListener('statechange', () => {
          if (w.state === 'installed' && navigator.serviceWorker.controller) {
            const action = { label: 'Reload', run: () => w.postMessage({ type: 'SKIP_WAITING' }) };
            if (typeof window.toast === 'function') window.toast('Update available','system_update',action);
          }
        });
      });
    }).catch(() => {});
    navigator.serviceWorker.addEventListener('controllerchange', () => location.reload());
  }

  function labelResponsiveTables(){
    document.querySelectorAll('table').forEach(table => {
      const labels = Array.from(table.querySelectorAll('thead th')).map(th => th.textContent.trim());
      if (!labels.length) return;
      table.querySelectorAll('tbody tr').forEach(tr => {
        Array.from(tr.children).forEach((td, i) => {
          if (!td.getAttribute('data-label') && labels[i]) td.setAttribute('data-label', labels[i]);
        });
      });
    });
  }

  function installApiFetchBridge(){
    if (!browserFetch || window.fetch?.__rvtV12ApiBridge) return;
    const bridged = function(input, init){
      const raw = typeof input === 'string' ? input : String(input?.url || '');
      try {
        const url = new URL(raw, location.href);
        const base = currentApiBase();
        const baseOrigin = base ? new URL(base).origin : location.origin;
        if (url.pathname.startsWith('/api/') && (url.origin === location.origin || url.origin === baseOrigin)) {
          return window.api(url.pathname + url.search, init || {});
        }
      } catch (_) {}
      return browserFetch(input, init);
    };
    bridged.__rvtV12ApiBridge = true;
    window.fetch = bridged;
  }

  class BluetoothBridge {
    async requestDevice(){ throw new Error('Bluetooth bridge not implemented for this platform'); }
    async connect(){ throw new Error('Bluetooth bridge not implemented for this platform'); }
    async disconnect(){}
    async startNotifications(){ throw new Error('Bluetooth bridge not implemented for this platform'); }
    async stopNotifications(){}
  }
  class WebBluetoothBridge extends BluetoothBridge {
    async requestDevice(filters){ return navigator.bluetooth.requestDevice(filters); }
    async connect(handle){ return handle.gatt.connect(); }
    async disconnect(handle){ return handle?.gatt?.disconnect?.(); }
    async startNotifications(handle, svcUUID, charUUID, cb){
      const server = handle.gatt?.connected ? handle.gatt : await handle.gatt.connect();
      const svc = await server.getPrimaryService(svcUUID);
      const ch = await svc.getCharacteristic(charUUID);
      ch.addEventListener('characteristicvaluechanged', ev => cb(ev.target.value));
      await ch.startNotifications();
      return ch;
    }
    async stopNotifications(handle, svcUUID, charUUID){
      const svc = await handle.gatt.getPrimaryService(svcUUID);
      const ch = await svc.getCharacteristic(charUUID);
      return ch.stopNotifications();
    }
  }
  class CapacitorBleBridge extends BluetoothBridge {
    constructor(){ super(); this.ble = window.Capacitor?.Plugins?.BluetoothLe || window.BluetoothLe; }
  }
  class TauriBlecBridge extends BluetoothBridge {}
  window.BluetoothBridge = BluetoothBridge;
  window.WebBluetoothBridge = WebBluetoothBridge;
  window.CapacitorBleBridge = CapacitorBleBridge;
  window.TauriBlecBridge = TauriBlecBridge;
  window.ble = window.Capacitor?.isNativePlatform?.() ? new CapacitorBleBridge()
    : window.__TAURI__ ? new TauriBlecBridge()
    : ('bluetooth' in navigator && window.isSecureContext) ? new WebBluetoothBridge()
    : null;

  function bootV12(){
    installApiFetchBridge();
    installHistoryContract();
    installChartLifecycle();
    installVisibilityThrottle();
    installServiceWorker();
    labelResponsiveTables();
    setDemoBannerVisible(!!window.S?.ctl?.sandbox || !!window.S?.demoMode || document.documentElement.dataset.sandbox === '1');
    try {
      new MutationObserver(() => {
        document.querySelectorAll('dialog[open], .sheet.open').forEach(window.injectDemoBanner);
        labelResponsiveTables();
      }).observe(document.body, { childList: true, subtree: true });
    } catch (_) {}
    if (window.visualViewport) {
      const syncKeyboard = () => {
        const kbd = Math.max(0, window.innerHeight - visualViewport.height - visualViewport.offsetTop);
        document.documentElement.style.setProperty('--keyboard-h', kbd + 'px');
      };
      visualViewport.addEventListener('resize', syncKeyboard);
      syncKeyboard();
    }
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', bootV12, { once:true });
  else bootV12();
})();
