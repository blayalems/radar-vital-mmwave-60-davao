/* Consolidated legacy patch JS. Source order is preserved from web/index.html. */

/* BEGIN modules/rvt-v11-consolidated-fixes-js.js */
    (() => {
      'use strict';
      const PATCH_KEY = 'rvt-v11-trainer-contract-compat';
      const STRUCTURAL_CHECKS = ['python_env', 'firmware_file_present', 'serial_port_list', 'session_folder_writable', 'disk_space', 'schema_hash_consistency', 'clock_monotonic_sanity', 'ble_adapter'];
      const CHECK_LABELS = {
        python_env: 'Python environment',
        firmware_file_present: 'Firmware file',
        serial_port_list: 'Serial ports',
        session_folder_writable: 'Session folder',
        disk_space: 'Disk space',
        schema_hash_consistency: 'Schema hash',
        clock_monotonic_sanity: 'Clock sanity',
        ble_adapter: 'BLE adapter',
        serial_port_probe: 'Serial probe',
        ble_device_probe: 'BLE device',
        api: 'Preflight API'
      };

      const setup = () => window.S && window.S.setup ? window.S.setup : {};
      const ctl = () => window.S && window.S.ctl ? window.S.ctl : null;
      const structuralIdsSafe = () => {
        try {
          if (typeof window.structuralChecks === 'function') return window.structuralChecks();
        } catch (_) { }
        return STRUCTURAL_CHECKS.slice();
      };
      const checkLabelSafe = id => {
        try {
          if (typeof window.checkLabel === 'function') return window.checkLabel(id);
        } catch (_) { }
        return CHECK_LABELS[id] || id;
      };
      const computeSummarySafe = checks => {
        try {
          if (typeof window.computePreflightSummary === 'function') return window.computePreflightSummary(checks);
        } catch (_) { }
        return (checks || []).reduce((out, c) => {
          if (c && c.status === 'ok') out.ok += 1;
          else if (c && c.status === 'warn') out.warn += 1;
          else if (c && c.status === 'fail') out.fail += 1;
          return out;
        }, { ok: 0, warn: 0, fail: 0 });
      };
      const apiJsonSafe = async (url, opts) => {
        if (typeof window.apiJson === 'function') return window.apiJson(url, opts || {});
        const res = await fetch(url, Object.assign({ cache: 'no-store' }, opts || {}));
        const body = await res.json().catch(() => ({}));
        if (!res.ok) {
          const err = new Error((body && body.error && body.error.message) || res.statusText || 'Request failed');
          err.status = res.status;
          err.body = body;
          throw err;
        }
        return body;
      };
      const renderPreflightSafe = () => {
        try { if (typeof window.renderPreflight === 'function') window.renderPreflight(); } catch (_) { }
        try { if (typeof window.renderSetupForm === 'function') window.renderSetupForm(); } catch (_) { }
        try { if (typeof window.syncDetectedPortsToSelect === 'function') window.syncDetectedPortsToSelect(); } catch (_) { }
      };
      const setPreflight = value => {
        if (!window.S) return value;
        if (!window.S.ctl) window.S.ctl = {};
        const checks = Array.isArray(value && value.checks) ? value.checks : [];
        window.S.ctl.preflight = Object.assign({}, value || {}, { summary: (value && value.summary) || computeSummarySafe(checks), checks });
        return window.S.ctl.preflight;
      };

      if (typeof window.alerts !== 'function') {
        window.alerts = function alerts(payload) {
          const out = [];
          const seen = new Set();
          const add = item => {
            if (!item || typeof item !== 'object') return;
            const title = String(item.title || item.label || 'Alert');
            const copy = String(item.copy || item.detail || item.message || '');
            const severity = ['bad', 'warn', 'good'].includes(item.severity) ? item.severity : 'warn';
            const key = severity + '|' + title + '|' + copy;
            if (seen.has(key)) return;
            seen.add(key);
            out.push({ severity, title, copy });
          };
          try { (window.S && Array.isArray(window.S.lastAlerts) ? window.S.lastAlerts : []).forEach(add); } catch (_) { }
          try { (Array.isArray(payload && payload.faults) ? payload.faults : []).forEach(add); } catch (_) { }
          return out;
        };
      }

      if (typeof window.runPreflightBatch !== 'function') {
        window.runPreflightBatch = async function runPreflightBatch() {
          const state = ctl();
          if (!state || !state.on) return state && state.preflight;
          const s = setup();
          const include = encodeURIComponent(structuralIdsSafe().join(','));
          const url = '/api/preflight?port=' + encodeURIComponent(s.radar_port || 'COM10') + '&address=' + encodeURIComponent(s.ble_address || '') + '&include=' + include;
          try {
            const result = await apiJsonSafe(url);
            return setPreflight(result);
          } catch (err) {
            return setPreflight({
              summary: { ok: 0, warn: 0, fail: 1 },
              checks: [{ id: 'api', label: 'Preflight API', status: 'fail', detail: err && err.message ? err.message : 'Preflight unavailable.', remediation: 'Confirm radar_vital_trainer_v11_for_v15_0.py is serving the control API.' }]
            });
          } finally {
            renderPreflightSafe();
          }
        };
      }

      if (typeof window.runOnePreflight !== 'function') {
        window.runOnePreflight = async function runOnePreflight(id) {
          const checkId = String(id || '').trim();
          const state = ctl();
          if (!checkId || !state || !state.on) return null;
          const current = Array.isArray(state.preflight && state.preflight.checks) ? state.preflight.checks.slice() : [];
          const pending = current.filter(c => c && c.id !== checkId);
          pending.push({ id: checkId, label: checkLabelSafe(checkId), status: 'running', detail: 'Running...', remediation: '' });
          setPreflight({ checks: pending, summary: computeSummarySafe(pending) });
          renderPreflightSafe();
          const s = setup();
          try {
            const result = await apiJsonSafe('/api/preflight/' + encodeURIComponent(checkId), {
              method: 'POST',
              headers: { 'Content-Type': 'application/json' },
              body: JSON.stringify({ port: s.radar_port || 'COM10', address: s.ble_address || '' })
            });
            const checks = (Array.isArray(state.preflight && state.preflight.checks) ? state.preflight.checks : []).filter(c => c && c.id !== checkId);
            checks.push(Object.assign({ id: checkId, label: checkLabelSafe(checkId), status: 'ok' }, result || {}));
            setPreflight({ checks, summary: computeSummarySafe(checks) });
            return result;
          } catch (err) {
            const checks = (Array.isArray(state.preflight && state.preflight.checks) ? state.preflight.checks : []).filter(c => c && c.id !== checkId);
            checks.push({ id: checkId, label: checkLabelSafe(checkId), status: 'fail', detail: err && err.message ? err.message : 'Check failed.', remediation: err && err.body && err.body.error && err.body.error.message ? err.body.error.message : '' });
            setPreflight({ checks, summary: computeSummarySafe(checks) });
            return null;
          } finally {
            renderPreflightSafe();
          }
        };
      }

      function syncControlDataset() {
        try {
          if (window.S && window.S.ctl && window.S.ctl.on && document.body.dataset.ctl !== 'on') document.body.dataset.ctl = 'on';
        } catch (_) { }
      }

      function activateBottomNav(btn) {
        const view = btn.dataset.bnView || btn.dataset.view;
        const tab = btn.dataset.bn;
        if (view) {
          if (view === 'settings' && typeof window.openSettings === 'function') window.openSettings();
          else if (typeof window.switchView === 'function') window.switchView(view);
          return;
        }
        if (tab) {
          if (document.body.dataset.ctl === 'on' && document.body.dataset.view !== 'live' && typeof window.switchView === 'function') window.switchView('live');
          if (typeof window.switchTab === 'function') window.switchTab(tab);
        }
      }

      function installBottomNavFallback() {
        const nav = document.getElementById('bottomNav');
        if (!nav || nav.dataset.rvtTapFallback === '1') return;
        nav.dataset.rvtTapFallback = '1';
        document.addEventListener('click', ev => {
          if (window.innerWidth > 760) return;
          const btn = ev.target && ev.target.closest ? ev.target.closest('#bottomNav .bn-item') : null;
          if (!btn || btn.disabled) return;
          if (document.body.classList.contains('rail-open') || document.body.classList.contains('rvt-modal-open') || document.body.classList.contains('fs-open')) return;
          window.setTimeout(() => {
            const view = btn.dataset.bnView || btn.dataset.view;
            const tab = btn.dataset.bn;
            const settingsOpen = document.getElementById('settingsOv') && document.getElementById('settingsOv').classList.contains('open');
            if (view && view !== 'settings' && document.body.dataset.view === view) return;
            if (view === 'settings' && (document.body.dataset.view === 'settings' || settingsOpen)) return;
            if (tab && document.body.dataset.view === 'live' && document.querySelector('.tab.active') && document.querySelector('.tab.active').id === tab) return;
            activateBottomNav(btn);
          }, 0);
        }, true);
      }

      function bootCompat() {
        document.documentElement.classList.add(PATCH_KEY);
        syncControlDataset();
        installBottomNavFallback();
        try { window.rvtTrackMutationObserver(new MutationObserver(syncControlDataset)).observe(document.body, { attributes: true, attributeFilter: ['data-ctl', 'data-view', 'class'] }); } catch (err) { window.rvtSoftError && window.rvtSoftError('control dataset observer setup failed', err, { always: true }); }
        /* PERF-03: interval removed — MutationObserver on body[data-ctl,data-view,class] is sufficient */
      }

      if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', bootCompat, { once: true });
      else bootCompat();
    })();

    /* rvt-v11-polish-qol-js */
    (function () {
      'use strict';

      const PATCH_KEY = 'rvt-v11-polish-qol';
      const LAST_VIEW_KEY = 'rvt-v11-last-view';
      const LAST_TAB_KEY = 'rvt-v11-last-live-tab';
      const FOCUS_KEY = 'rvt-v11-focus-live';
      const POLL_MS = 700;
      let coreTimer = 0;
      let coreTries = 0;
      let persistRestoring = false;

      function ready(fn) {
        if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', fn, { once: true });
        else fn();
      }

      function hasCore() {
        return !!(window.S && typeof window.switchView === 'function' && typeof window.switchTab === 'function');
      }

      function waitForCore(fn) {
        const tick = function () {
          if (hasCore() || coreTries > 40) {
            fn();
            return;
          }
          coreTries += 1;
          coreTimer = window.setTimeout(tick, 250);
        };
        tick();
      }

      function byId(id) {
        return document.getElementById(id);
      }

      function text(id) {
        const el = byId(id);
        return (el && el.textContent ? el.textContent.trim().replace(/\s+/g, ' ') : '--') || '--';
      }

      function toast(msg, icon) {
        try {
          if (typeof window.toast === 'function') window.toast(msg, icon || 'info');
          else console.info('[RVT]', msg);
        } catch (_) { }
      }

      function isTypingTarget(target) {
        return !!(target && target.closest && target.closest('input, textarea, select, [contenteditable="true"]'));
      }

      function currentTabId() {
        return document.querySelector('.tab.active')?.id || 'tab-overview';
      }

      function currentView() {
        return document.body?.dataset?.view || 'live';
      }

      function copyText(value) {
        if (navigator.clipboard && window.isSecureContext) return navigator.clipboard.writeText(value);
        return new Promise(function (resolve, reject) {
          try {
            const ta = document.createElement('textarea');
            ta.value = value;
            ta.setAttribute('readonly', '');
            ta.style.position = 'fixed';
            ta.style.left = '-9999px';
            document.body.appendChild(ta);
            ta.select();
            const ok = document.execCommand('copy');
            ta.remove();
            ok ? resolve() : reject(new Error('copy failed'));
          } catch (err) {
            reject(err);
          }
        });
      }

      function normalizeAgeMs(value) {
        const n = Number(value);
        if (!Number.isFinite(n) || n <= 0) return null;
        const epochMs = n < 100000000000 ? n * 1000 : n;
        const age = Date.now() - epochMs;
        return Number.isFinite(age) && age >= 0 ? age : null;
      }

      function payloadAgeMs() {
        const st = window.S || {};
        const payload = st.lastLivePayload || st.lastPayload || {};
        const meta = payload.meta || {};
        const candidates = [
          meta.ts_ms, meta.host_ts_ms, meta.timestamp_ms, meta.timestamp_s, meta.created_at_ms,
          payload.ts_ms, payload.timestamp_ms, payload.timestamp_s
        ];
        for (const v of candidates) {
          const age = normalizeAgeMs(v);
          if (age !== null) return age;
        }
        if (Number.isFinite(Number(st.lastGoodAt)) && Number(st.lastGoodAt) > 0) {
          return Math.max(0, Date.now() - Number(st.lastGoodAt));
        }
        return null;
      }

      function ageLabel(ms) {
        if (ms === null) return 'no sample';
        const s = Math.max(0, Math.round(ms / 1000));
        if (s < 2) return 'just now';
        if (s < 60) return s + 's ago';
        const m = Math.floor(s / 60);
        return m + 'm ' + String(s % 60).padStart(2, '0') + 's ago';
      }

      function healthState() {
        const st = window.S || {};
        const age = payloadAgeMs();
        const staleLimit = Math.max(5, Number(st.hardStaleS) || 5) * 1000;
        if (st.paused) return { key: 'paused', icon: 'pause', title: 'Paused', sub: ageLabel(age) };
        if (st.demoMode || st.autoDemoActive || document.body.classList.contains('demo-mode')) return { key: 'demo', icon: 'science', title: 'Demo', sub: ageLabel(age) };
        if (Number(st.disc) > 0) return { key: 'bad', icon: 'wifi_off', title: 'Disconnected', sub: text('connSub') !== '--' ? text('connSub') : ageLabel(age) };
        if (age !== null && age > staleLimit) return { key: 'warn', icon: 'history', title: 'Stale sample', sub: ageLabel(age) };
        if (age !== null) return { key: 'good', icon: 'monitor_heart', title: 'Live sample', sub: ageLabel(age) };
        return { key: 'warn', icon: 'hourglass_top', title: 'Waiting', sub: text('connSub') };
      }

      async function copyOperatorSummary() {
        const st = window.S || {};
        const payload = st.lastPayload || {};
        const meta = payload.meta || {};
        const sid = (typeof window.currentSessionId === 'function' ? window.currentSessionId() : (meta.session_id || meta.session || 'local')) || 'local';
        const rows = [
          'Radar Vital operator summary',
          'time=' + new Date().toISOString(),
          'view=' + currentView() + ' tab=' + currentTabId(),
          'connection=' + text('connT') + ' / ' + text('connSub'),
          'mode=' + text('modeBadgeT') + ' / ' + text('modeBadgeSub'),
          'sample=' + healthState().title + ' / ' + healthState().sub,
          'session=' + sid,
          'hr=' + text('kpiHr') + ' bpm; ' + text('kpiHrSub'),
          'rr=' + text('kpiRr') + ' br/min; ' + text('kpiRrSub'),
          'range=' + text('kpiDist') + ' cm; fps=' + text('kpiFps'),
          'firmware=' + (byId('fwBadge')?.textContent || '--').trim().replace(/\s+/g, ' ')
        ];
        try {
          await copyText(rows.join('\n'));
          toast('Copied operator summary', 'content_copy');
        } catch (_) {
          toast('Copy failed; browser blocked clipboard access', 'warning');
        }
      }

      function setFocusLive(on) {
        const enabled = !!on;
        document.body.classList.toggle('rvt-focus-live', enabled);
        try { localStorage.setItem(FOCUS_KEY, enabled ? '1' : '0'); } catch (_) { }
        document.querySelectorAll('[data-rvt-focus-toggle]').forEach(function (btn) {
          btn.setAttribute('aria-pressed', enabled ? 'true' : 'false');
          btn.classList.toggle('active', enabled);
        });
        toast(enabled ? 'Live focus on' : 'Live focus off', enabled ? 'center_focus_strong' : 'center_focus_weak');
      }

      function installCommandButtons() {
        const strip = document.querySelector('.command-strip');
        if (strip && !byId('rvtCopyStatusBtn')) {
          const copyBtn = document.createElement('button');
          copyBtn.id = 'rvtCopyStatusBtn';
          copyBtn.className = 'quick-action rvt-copy-status';
          copyBtn.type = 'button';
          copyBtn.setAttribute('aria-label', 'Copy operator summary');
          copyBtn.innerHTML = '<kbd>Copy</kbd><span class="qa-label">Status</span>';
          copyBtn.addEventListener('click', copyOperatorSummary);
          strip.appendChild(copyBtn);
        }
        if (strip && !byId('rvtFocusLiveBtn')) {
          const focusBtn = document.createElement('button');
          focusBtn.id = 'rvtFocusLiveBtn';
          focusBtn.className = 'quick-action rvt-focus-toggle';
          focusBtn.type = 'button';
          focusBtn.dataset.rvtFocusToggle = '1';
          focusBtn.setAttribute('aria-label', 'Toggle live focus mode');
          focusBtn.setAttribute('aria-pressed', document.body.classList.contains('rvt-focus-live') ? 'true' : 'false');
          focusBtn.innerHTML = '<kbd>Focus</kbd><span class="qa-label">Live</span>';
          focusBtn.addEventListener('click', function () { setFocusLive(!document.body.classList.contains('rvt-focus-live')); });
          strip.appendChild(focusBtn);
        }
        const menu = byId('topbarOverflow');
        if (menu && !byId('rvtCopyStatusMenuBtn')) {
          const copyItem = document.createElement('button');
          copyItem.id = 'rvtCopyStatusMenuBtn';
          copyItem.className = 'ghost-btn ov-btn';
          copyItem.type = 'button';
          copyItem.setAttribute('role', 'menuitem');
          copyItem.innerHTML = '<span class="material-symbols-rounded" aria-hidden="true">content_copy</span><span class="ov-row"><span class="ov-title">Copy status</span><span class="ov-hint">Operator summary</span></span>';
          copyItem.addEventListener('click', function () { copyOperatorSummary(); closeOverflowMenuSafe(); });
          menu.appendChild(copyItem);
        }
        if (menu && !byId('rvtFocusLiveMenuBtn')) {
          const focusItem = document.createElement('button');
          focusItem.id = 'rvtFocusLiveMenuBtn';
          focusItem.className = 'ghost-btn ov-btn';
          focusItem.type = 'button';
          focusItem.dataset.rvtFocusToggle = '1';
          focusItem.setAttribute('role', 'menuitem');
          focusItem.setAttribute('aria-pressed', document.body.classList.contains('rvt-focus-live') ? 'true' : 'false');
          focusItem.innerHTML = '<span class="material-symbols-rounded" aria-hidden="true">center_focus_strong</span><span class="ov-row"><span class="ov-title">Live focus</span><span class="ov-hint">Hide notes and command strip</span></span>';
          focusItem.addEventListener('click', function () { setFocusLive(!document.body.classList.contains('rvt-focus-live')); closeOverflowMenuSafe(); });
          menu.appendChild(focusItem);
        }
      }

      function closeOverflowMenuSafe() {
        const menu = byId('topbarOverflow');
        const btn = byId('topbarOverflowBtn');
        if (!menu) return;
        if (typeof window.closeOverflowMenu === 'function') {
          try { window.closeOverflowMenu(); } catch (_) { }
        }
        menu.classList.remove('open');
        menu.style.removeProperty('display');
        menu.setAttribute('aria-hidden', 'true');
        if ('inert' in menu) menu.inert = true;
        if (btn) btn.setAttribute('aria-expanded', 'false');
      }

      function syncOverflowA11y() {
        const menu = byId('topbarOverflow');
        const btn = byId('topbarOverflowBtn');
        if (!menu) return;
        const open = menu.classList.contains('open') && !menu.hidden;
        menu.setAttribute('aria-hidden', open ? 'false' : 'true');
        if ('inert' in menu) menu.inert = !open;
        if (open) menu.style.display = 'grid';
        else if (menu.style.display === 'grid') menu.style.removeProperty('display');
        if (btn) {
          btn.hidden = false;
          btn.style.removeProperty('display');
          btn.setAttribute('aria-controls', 'topbarOverflow');
          btn.setAttribute('aria-expanded', open ? 'true' : 'false');
        }
      }

      function installOverlayFixes() {
        document.addEventListener('pointerdown', function (ev) {
          const menu = byId('topbarOverflow');
          const btn = byId('topbarOverflowBtn');
          if (!menu || !menu.classList.contains('open')) return;
          if (menu.contains(ev.target) || (btn && btn.contains(ev.target))) return;
          closeOverflowMenuSafe();
        }, true);
        document.addEventListener('keydown', function (ev) {
          if (ev.key !== 'Escape') return;
          closeOverflowMenuSafe();
          if (document.body.classList.contains('rail-open') && typeof window.closeRailDrawer === 'function') window.closeRailDrawer();
        }, true);
        /* PERF-02: interval removed — event listeners (click, keydown, resize) are sufficient */
        syncOverflowA11y();
      }

      function wrapNavigationPersistence() {
        if (typeof window.switchView === 'function' && !window.switchView.__rvtV11PersistWrapped) {
          const orig = window.switchView;
          window.switchView = function (view) {
            const result = orig.apply(this, arguments);
            try {
              if (!persistRestoring && view) localStorage.setItem(LAST_VIEW_KEY, String(view));
            } catch (_) { }
            setTimeout(syncActiveNavIntoView, 60);
            return result;
          };
          window.switchView.__rvtV11PersistWrapped = true;
        }
        if (typeof window.switchTab === 'function' && !window.switchTab.__rvtV11PersistWrapped) {
          const origTab = window.switchTab;
          window.switchTab = function (tab) {
            const result = origTab.apply(this, arguments);
            try {
              if (!persistRestoring && tab) localStorage.setItem(LAST_TAB_KEY, String(tab));
            } catch (_) { }
            setTimeout(syncActiveNavIntoView, 60);
            return result;
          };
          window.switchTab.__rvtV11PersistWrapped = true;
        }
      }

      function restoreLastLocation() {
        const params = new URLSearchParams(window.location.search || '');
        if (params.has('view') || params.has('tab') || window.location.hash) return;
        let view = '';
        let tab = '';
        try {
          view = localStorage.getItem(LAST_VIEW_KEY) || '';
          tab = localStorage.getItem(LAST_TAB_KEY) || '';
        } catch (_) { }
        if (!view && !tab) return;
        persistRestoring = true;
        try {
          if (view && typeof window.switchView === 'function') window.switchView(view);
          if ((!view || view === 'live') && tab && typeof window.switchTab === 'function') window.switchTab(tab);
        } catch (_) {
        } finally {
          persistRestoring = false;
        }
      }

      function syncActiveNavIntoView() {
        const activeItems = [
          document.querySelector('#mobileLiveTabStrip button.active'),
          document.querySelector('#bottomNav .bn-item.active'),
          document.querySelector('#liveTabsGroup .r-item.active')
        ].filter(Boolean);
        activeItems.forEach(function (el) {
          try { el.scrollIntoView({ block: 'nearest', inline: 'center', behavior: 'smooth' }); } catch (_) { }
        });
      }

      function installKeyboardQoL() {
        document.addEventListener('keydown', function (ev) {
          if (isTypingTarget(ev.target)) return;
          const key = (ev.key || '').toLowerCase();
          if ((ev.ctrlKey || ev.metaKey) && ev.shiftKey && key === 'c') {
            ev.preventDefault();
            copyOperatorSummary();
            return;
          }
          if (key === '/' && !ev.ctrlKey && !ev.metaKey && !ev.altKey) {
            ev.preventDefault();
            if (typeof window.openPalette === 'function') window.openPalette();
          }
          if (key === 'f' && ev.shiftKey && !ev.ctrlKey && !ev.metaKey && !ev.altKey) {
            ev.preventDefault();
            setFocusLive(!document.body.classList.contains('rvt-focus-live'));
          }
        }, true);
      }

      function installHelpRows() {
        const grid = document.querySelector('.kbd-help-grid');
        if (!grid || grid.dataset.rvtV11PolishRows) return;
        grid.dataset.rvtV11PolishRows = '1';
        const rows = [
          ['Copy operator summary', '<kbd>Ctrl</kbd><kbd>Shift</kbd><kbd>C</kbd>'],
          ['Toggle live focus', '<kbd>Shift</kbd><kbd>F</kbd>']
        ];
        rows.forEach(function (row) {
          const k = document.createElement('div');
          k.className = 'k';
          k.textContent = row[0];
          const v = document.createElement('div');
          v.className = 'v';
          String(row[1] || '').match(/<kbd>(.*?)<\/kbd>/g)?.forEach(function(part) {
            const kbd = document.createElement('kbd');
            kbd.textContent = part.replace(/<\/?kbd>/g, '');
            v.appendChild(kbd);
          });
          if (!v.childNodes.length) v.textContent = String(row[1] || '');
          grid.appendChild(k);
          grid.appendChild(v);
        });
      }

      function installOverflowWatch() {
        let raf = 0;
        const sync = function () {
          raf = 0;
          const wide = document.documentElement.scrollWidth > window.innerWidth + 2;
          document.body.classList.toggle('rvt-overflow-x-fixed', wide);
        };
        const schedule = function () {
          if (!raf) raf = requestAnimationFrame(sync);
        };
        window.addEventListener('resize', schedule, { passive: true });
        window.addEventListener('orientationchange', schedule, { passive: true });
        try { new ResizeObserver(schedule).observe(document.body); } catch (_) { }
        /* PERF-06: interval removed — ResizeObserver + resize/orientationchange events are sufficient */
        schedule();
      }

      function boot() {
        document.documentElement.classList.add('rvt-polish-ready');
        try {
          if (localStorage.getItem(FOCUS_KEY) === '1') document.body.classList.add('rvt-focus-live');
        } catch (_) { }
        installCommandButtons();
        installOverlayFixes();
        installKeyboardQoL();
        installHelpRows();
        installOverflowWatch();
        waitForCore(function () {
          if (coreTimer) window.clearTimeout(coreTimer);
          installCommandButtons();
          wrapNavigationPersistence();
          restoreLastLocation();
          syncActiveNavIntoView();
          /* PERF-04: no polling needed — single call + switchView/switchTab hooks handle updates */
          syncActiveNavIntoView();
        });
        window.__RVT_V11_POLISH_QOL__ = { patch: PATCH_KEY, copyOperatorSummary, setFocusLive };
      }

      ready(boot);
    })();

    /* rvt-v11-release-visual-fixes-js */
    (() => {
      'use strict';
      const PATCH_KEY = 'rvt-v11-release-visual-fixes';
      const TOUCH_MAX = 1199;
      const byId = id => document.getElementById(id);
      const isTouchShell = () => window.innerWidth <= TOUCH_MAX;

      function isOpen(el) {
        return !!el && el.classList.contains('open') && getComputedStyle(el).display !== 'none';
      }

      function setInert(el, inert) {
        if (!el) return;
        try {
          if ('inert' in el) el.inert = !!inert;
        } catch (_) { }
      }

      function setAttr(el, name, value) {
        if (!el) return;
        const next = String(value);
        if (el.getAttribute(name) !== next) el.setAttribute(name, next);
      }

      function setOverlayState(el, open) {
        if (!el) return;
        el.classList.toggle('open', !!open);
        setAttr(el, 'aria-hidden', open ? 'false' : 'true');
        setAttr(el, 'data-rvt-overlay-open', open ? '1' : '0');
        setInert(el, !open);
      }

      function syncRailHandle() {
        const handle = byId('railResizeHandle');
        if (!handle) return;
        const disable = isTouchShell();
        handle.hidden = disable;
        handle.disabled = disable;
        handle.tabIndex = disable ? -1 : 0;
        setAttr(handle, 'aria-hidden', disable ? 'true' : 'false');
        if (disable) handle.style.setProperty('display', 'none', 'important');
        else handle.style.removeProperty('display');
      }

      function syncOverlayState() {
        const palette = byId('pOv');
        const drawer = byId('dv');
        const drawerOv = byId('dvOv');
        const settings = byId('settingsOv');
        const kbd = byId('kbdHelpOv');
        const menu = byId('topbarOverflow');
        const menuBtn = byId('topbarOverflowBtn');

        [palette, settings, kbd].forEach(el => {
          if (!el) return;
          const open = el.classList.contains('open');
          setAttr(el, 'aria-hidden', open ? 'false' : 'true');
          setAttr(el, 'data-rvt-overlay-open', open ? '1' : '0');
          setInert(el, !open);
        });

        if (drawer) {
          const open = drawer.classList.contains('open');
          setAttr(drawer, 'aria-hidden', open ? 'false' : 'true');
          setAttr(drawer, 'data-rvt-overlay-open', open ? '1' : '0');
          setInert(drawer, !open);
          if (drawerOv) {
            drawerOv.classList.toggle('open', open);
            setAttr(drawerOv, 'aria-hidden', open ? 'false' : 'true');
            setAttr(drawerOv, 'data-rvt-overlay-open', open ? '1' : '0');
          }
        }

        if (menu) {
          const open = menu.classList.contains('open') && !menu.hidden;
          setAttr(menu, 'role', 'menu');
          setAttr(menu, 'aria-hidden', open ? 'false' : 'true');
          setAttr(menu, 'data-rvt-overlay-open', open ? '1' : '0');
          setInert(menu, !open);
          if (menuBtn) setAttr(menuBtn, 'aria-expanded', open ? 'true' : 'false');
        }

        const modalOpen = [palette, drawer, settings, kbd].some(isOpen);
        document.body.classList.toggle('rvt-modal-open', modalOpen);
        document.documentElement.classList.toggle('rvt-modal-open', modalOpen);
        ['bottomNav', 'mobileLiveTabStrip'].forEach(id => {
          const el = byId(id);
          if (el) setAttr(el, 'aria-hidden', modalOpen ? 'true' : 'false');
        });
        syncRailHandle();
      }

      function closeOtherOverlays(keepId) {
        if (keepId !== 'pOv' && typeof window.closePalette === 'function') {
          try { window.closePalette(); } catch (_) { }
        }
        if (keepId !== 'dv' && typeof window.closeDrawer === 'function') {
          try { window.closeDrawer(); } catch (_) { }
        }
        if (keepId !== 'settingsOv' && typeof window.closeSettings === 'function') {
          try { window.closeSettings(); } catch (_) { }
        }
        if (keepId !== 'kbdHelpOv' && typeof window.closeKbdHelp === 'function') {
          try { window.closeKbdHelp(); } catch (_) { }
        }
      }

      function focusFirst(el, selector) {
        if (!el) return;
        const target = selector ? el.querySelector(selector) : null;
        const fallback = el.querySelector('button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])');
        const next = target || fallback || el;
        try { next.focus({ preventScroll: true }); } catch (_) { }
      }

      function openPaletteFallback(returnTo) {
        const ov = byId('pOv');
        if (!ov) return;
        closeOtherOverlays('pOv');
        setOverlayState(ov, true);
        try { window.renderPalette && window.renderPalette(); } catch (_) { }
        try { window.trapFocus && window.trapFocus(ov, returnTo || document.activeElement); } catch (_) { }
        setTimeout(() => focusFirst(ov, '#pIn'), 30);
      }

      function closePaletteFallback() {
        const ov = byId('pOv');
        setOverlayState(ov, false);
        try { window.releaseFocusTrap && window.releaseFocusTrap(ov); } catch (_) { }
      }

      function openDrawerFallback(returnTo) {
        const drawer = byId('dv');
        const scrim = byId('dvOv');
        if (!drawer) return;
        closeOtherOverlays('dv');
        drawer.classList.add('open');
        if (scrim) scrim.classList.add('open');
        syncOverlayState();
        try { window.App && window.App.a11y && window.App.a11y.trap(drawer, returnTo || document.activeElement); } catch (_) { }
        setTimeout(() => focusFirst(drawer, '.dv-h .ic-btn'), 30);
      }

      function closeDrawerFallback() {
        const drawer = byId('dv');
        const scrim = byId('dvOv');
        if (drawer) drawer.classList.remove('open');
        if (scrim) scrim.classList.remove('open');
        try { window.App && window.App.a11y && window.App.a11y.release(drawer); } catch (_) { }
        syncOverlayState();
      }

      function openSettingsModal(returnTo) {
        const ov = byId('settingsOv');
        if (!ov) return;
        closeOtherOverlays('settingsOv');
        setOverlayState(ov, true);
        try { window.syncSettingsUi && window.syncSettingsUi(); } catch (_) { }
        try { window.trapFocus && window.trapFocus(ov, returnTo || document.activeElement); } catch (_) { }
        setTimeout(() => focusFirst(ov, '.set-h .ic-btn, button, input, select'), 30);
      }

      function closeSettingsModal() {
        const ov = byId('settingsOv');
        setOverlayState(ov, false);
        try { window.releaseFocusTrap && window.releaseFocusTrap(ov); } catch (_) { }
      }

      function openKbdHelpFallback(returnTo) {
        const ov = byId('kbdHelpOv');
        if (!ov) return;
        closeOtherOverlays('kbdHelpOv');
        setOverlayState(ov, true);
        try { window.trapFocus && window.trapFocus(ov, returnTo || document.activeElement); } catch (_) { }
        setTimeout(() => focusFirst(ov, '.kbd-help-head .ic-btn'), 30);
      }

      function closeKbdHelpFallback() {
        const ov = byId('kbdHelpOv');
        setOverlayState(ov, false);
        try { window.releaseFocusTrap && window.releaseFocusTrap(ov); } catch (_) { }
      }

      function wrapOverlayFunctions() {
        if (window.__RVT_V11_RELEASE_VISUAL_WRAPPED__) return;
        if (typeof window.openPalette !== 'function' || typeof window.closePalette !== 'function') return;
        window.__RVT_V11_RELEASE_VISUAL_WRAPPED__ = true;

        const origOpenPalette = window.openPalette;
        const origClosePalette = window.closePalette;
        const origOpenDrawer = window.openDrawer;
        const origCloseDrawer = window.closeDrawer;
        const origCloseSettings = window.closeSettings;
        const origOpenKbdHelp = window.openKbdHelp;
        const origCloseKbdHelp = window.closeKbdHelp;

        window.openPalette = function () {
          const returnTo = document.activeElement instanceof HTMLElement ? document.activeElement : null;
          try { origOpenPalette.apply(this, arguments); } catch (_) { }
          openPaletteFallback(returnTo);
          syncOverlayState();
        };
        window.closePalette = function () {
          try { origClosePalette.apply(this, arguments); } catch (_) { }
          closePaletteFallback();
          syncOverlayState();
        };
        window.openDrawer = function () {
          const returnTo = document.activeElement instanceof HTMLElement ? document.activeElement : null;
          try { if (typeof origOpenDrawer === 'function') origOpenDrawer.apply(this, arguments); } catch (_) { }
          openDrawerFallback(returnTo);
          syncOverlayState();
        };
        window.closeDrawer = function () {
          try { if (typeof origCloseDrawer === 'function') origCloseDrawer.apply(this, arguments); } catch (_) { }
          closeDrawerFallback();
          syncOverlayState();
        };
        window.openSettings = function (opts) {
          if (opts && opts.route) {
            if (typeof window.switchView === 'function') window.switchView('settings');
            return;
          }
          openSettingsModal(document.activeElement instanceof HTMLElement ? document.activeElement : null);
          syncOverlayState();
        };
        window.closeSettings = function () {
          try { if (typeof origCloseSettings === 'function') origCloseSettings.apply(this, arguments); } catch (_) { }
          closeSettingsModal();
          syncOverlayState();
        };
        window.openKbdHelp = function () {
          const returnTo = document.activeElement instanceof HTMLElement ? document.activeElement : null;
          try { if (typeof origOpenKbdHelp === 'function') origOpenKbdHelp.apply(this, arguments); } catch (_) { }
          openKbdHelpFallback(returnTo);
          syncOverlayState();
        };
        window.closeKbdHelp = function () {
          try { if (typeof origCloseKbdHelp === 'function') origCloseKbdHelp.apply(this, arguments); } catch (_) { }
          closeKbdHelpFallback();
          syncOverlayState();
        };
      }

      function boot() {
        document.documentElement.classList.add(PATCH_KEY);
        ['pOv', 'settingsOv', 'kbdHelpOv', 'dv'].forEach(id => {
          const el = byId(id);
          if (el && !el.hasAttribute('aria-hidden')) setAttr(el, 'aria-hidden', 'true');
        });
        syncOverlayState();
        wrapOverlayFunctions();
        window.addEventListener('resize', syncOverlayState, { passive: true });
        document.addEventListener('click', () => setTimeout(syncOverlayState, 0), true);
        document.addEventListener('keydown', ev => {
          if (ev.key === 'Escape') setTimeout(syncOverlayState, 0);
        }, true);
        /* PERF-05: interval removed — click, keydown, resize events + function wraps drive syncOverlayState */
      }

      if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', boot, { once: true });
      } else {
        boot();
      }
    })();

    /* rvt-v11-pdf-bugfixes-js */
    (() => {
      'use strict';
      const idle = cb => {
        if ('requestIdleCallback' in window) return window.requestIdleCallback(cb, { timeout: 1200 });
        return window.setTimeout(cb, 180);
      };

      function syncFullscreenA11y() {
        const fs = document.querySelector('.card.fs');
        document.body.classList.toggle('fs-open', !!fs);
        document.querySelectorAll('.card[role="dialog"][data-rvt-fs-card="1"]').forEach(card => {
          if (card !== fs) {
            card.removeAttribute('role');
            card.removeAttribute('aria-modal');
            card.removeAttribute('data-rvt-fs-card');
          }
        });
        if (fs) {
          fs.setAttribute('role', 'dialog');
          fs.setAttribute('aria-modal', 'true');
          fs.setAttribute('data-rvt-fs-card', '1');
          fs.classList.toggle('hr-trend', !!fs.querySelector('#hrChart'));
          fs.classList.toggle('rr-trend', !!fs.querySelector('#rrChart'));
          const title = fs.querySelector('.ct');
          if (title && !title.id) title.id = 'rvt-fs-title-' + Math.random().toString(36).slice(2, 8);
          if (title) fs.setAttribute('aria-labelledby', title.id);
          idle(() => Object.values(window.S?.charts || {}).forEach(c => { try { c && c.resize && c.resize(); } catch (_) { } }));
        }
      }

      function stripLegacyFocus() {
        document.querySelectorAll('#view-live .is-focused, body[data-view="live"] .is-focused').forEach(el => {
          el.classList.remove('is-focused');
        });
      }

      let syncingHomeExportPill = false;

      function syncHomeExportPill() {
        const btn = document.querySelector('.topbar-actions > .tb-export');
        if (!btn || !window.matchMedia('(max-width: 760px)').matches) return;
        if (syncingHomeExportPill) return;
        syncingHomeExportPill = true;
        const home = document.body?.dataset?.view === 'home';
        const set = (name, value) => btn.style.setProperty(name, value, 'important');
        try {
          if (home) {
            set('width', '44px');
            set('min-width', '44px');
            set('max-width', '44px');
            set('height', '44px');
            set('min-height', '44px');
            set('flex', '0 0 44px');
            set('padding', '0');
            set('font-size', '0');
            set('gap', '0');
            set('overflow', 'hidden');
          } else {
            set('width', '44px');
            set('min-width', '44px');
            set('max-width', '44px');
            set('height', '44px');
            set('min-height', '44px');
            set('flex', '0 0 44px');
            set('padding', '0');
            set('font-size', '0');
            set('gap', '0');
            set('overflow', 'hidden');
          }
        } finally {
          syncingHomeExportPill = false;
        }
      }

      function observeHomeExportPill() {
        const btn = document.querySelector('.topbar-actions > .tb-export');
        if (!btn || btn.dataset.rvtHomeExportObserved === '1') return;
        btn.dataset.rvtHomeExportObserved = '1';
        try {
          window.rvtTrackMutationObserver(new MutationObserver(() => setTimeout(syncHomeExportPill, 0))).observe(btn, { attributes: true, attributeFilter: ['style'] });
        } catch (err) { window.rvtSoftError && window.rvtSoftError('home export pill observer setup failed', err, { always: true }); }
      }

      function wrapFullscreen() {
        if (typeof window.fsCard !== 'function' || window.fsCard.__rvtPdfBugfixWrapped) return;
        const orig = window.fsCard;
        window.fsCard = function () {
          const result = orig.apply(this, arguments);
          syncFullscreenA11y();
          return result;
        };
        window.fsCard.__rvtPdfBugfixWrapped = true;
      }

      function boot() {
        wrapFullscreen();
        syncFullscreenA11y();
        stripLegacyFocus();
        observeHomeExportPill();
        syncHomeExportPill();
        setTimeout(syncHomeExportPill, 600);
        window.addEventListener('resize', syncFullscreenA11y, { passive: true });
        window.addEventListener('resize', syncHomeExportPill, { passive: true });
        try {
          window.rvtTrackMutationObserver(new MutationObserver(syncHomeExportPill)).observe(document.body, { attributes: true, attributeFilter: ['data-view', 'class'] });
        } catch (err) { window.rvtSoftError && window.rvtSoftError('home export body observer setup failed', err, { always: true }); }
        document.addEventListener('focusin', stripLegacyFocus, true);
        document.addEventListener('click', () => setTimeout(() => {
          syncFullscreenA11y();
          wrapFullscreen();
          stripLegacyFocus();
          observeHomeExportPill();
          syncHomeExportPill();
          setTimeout(syncHomeExportPill, 120);
          setTimeout(syncHomeExportPill, 600);
        }, 0), true);
      }

      wrapFullscreen();
      if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot, { once: true });
      else boot();
    })();
;
/* END modules/rvt-v11-consolidated-fixes-js.js */

/* BEGIN modules/rvt-v11-uiux-consistency-bugfixes-js.js */
(function(){
  const raf = window.requestAnimationFrame || function(fn){ return setTimeout(fn, 16); };

  function makeIcon(name){
    const icon = document.createElement('span');
    icon.className = 'material-symbols-rounded';
    icon.setAttribute('aria-hidden', 'true');
    icon.textContent = name;
    return icon;
  }

  var _lastCrumbView = '';
  var _viewLabels = {home:'Workflow',live:'Live Dashboard',report:'Report',help:'Help',settings:'Settings'};

  function ensureHomeCrumb(){
    const trail = document.getElementById('crumbTrail');
    if(!trail) return;
    const view = document.body?.dataset?.view || 'home';
    const viewLabel = _viewLabels[view] || view;
    const existingHome = Array.prototype.filter.call(trail.querySelectorAll('.uiux-home-crumb'), function(node){
      return (node.textContent || '').trim() === 'Home';
    }).length;
    const existingCurrent = Array.prototype.filter.call(trail.querySelectorAll('.uiux-crumb-current'), function(node){
      return (node.textContent || '').trim() === viewLabel;
    }).length;
    if(_lastCrumbView === view && existingHome === 1 && existingCurrent === 1 && trail.children.length <= 3) return;

    /* Replace stale or duplicated crumbs instead of appending to another renderer. */
    trail.replaceChildren();
    _lastCrumbView = view;

    /* ── Home segment: 🏠 Home ── */
    const homeBtn = document.createElement('button');
    homeBtn.type = 'button';
    homeBtn.className = 'crumb-seg uiux-home-crumb';
    homeBtn.appendChild(makeIcon('home'));
    homeBtn.appendChild(document.createTextNode('Home'));
    homeBtn.addEventListener('click', function(){
      if(typeof window.switchView === 'function') window.switchView('home');
    });
    trail.appendChild(homeBtn);

    /* ── Separator ── */
    const sep = document.createElement('span');
    sep.className = 'crumb-sep';
    sep.textContent = '/';
    sep.setAttribute('aria-hidden', 'true');
    trail.appendChild(sep);

    /* ── Current view label ── */
    const label = document.createElement('span');
    label.className = 'crumb-seg uiux-crumb-current';
    label.textContent = viewLabel;
    label.style.cssText = 'cursor:default;background:var(--brand-100);color:var(--brand-500);border-color:transparent;pointer-events:none;';
    trail.appendChild(label);
  }

  function ensureHelpHome(){
    const quick = document.querySelector('#view-help .help-toolbar, body[data-view="help"] .help-toolbar, #view-help .help-quick, body[data-view="help"] .help-quick, .help-page .help-toolbar, .help-page .help-quick');
    if(!quick || quick.querySelector('.uiux-help-home')) return;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'chip-btn uiux-help-home';
    btn.appendChild(makeIcon('arrow_back'));
    btn.appendChild(document.createTextNode('Home'));
    btn.addEventListener('click', function(){
      if(typeof window.switchView === 'function') window.switchView('home');
    });
    quick.prepend(btn);
  }

  function syncExportButton(){
    const btn = document.querySelector('.topbar-actions > .tb-export');
    if(!btn) return;
    const mobile = window.matchMedia && window.matchMedia('(max-width: 760px)').matches;
    const utility = document.body && (document.body.dataset.view === 'help' || document.body.dataset.view === 'settings');
    const nativeDesktop = window.innerWidth > 760 && (window.__TAURI_INTERNALS__ || location.hostname === 'tauri.localhost');
    const set = function(name, value){ btn.style.setProperty(name, value, 'important'); };
    if(mobile || utility || nativeDesktop){
      const size = !mobile ? '52px' : '44px';
      set('display', 'grid');
      set('place-items', 'center');
      set('position', 'static');
      set('grid-column', 'auto');
      set('grid-row', 'auto');
      set('width', size);
      set('min-width', size);
      set('max-width', size);
      set('height', size);
      set('min-height', size);
      set('padding', '0');
      set('font-size', '0');
      set('gap', '0');
      set('flex', '0 0 ' + size);
      set('overflow', 'hidden');
    } else {
      set('width', 'auto');
      set('min-width', '132px');
      set('max-width', 'none');
      set('height', '52px');
      set('min-height', '52px');
      set('padding', '0 22px');
      set('font-size', '15px');
      set('gap', '8px');
      set('flex', '0 0 auto');
      set('overflow', 'visible');
    }
  }

  function syncSwitchState(){
    document.querySelectorAll('.sw').forEach(function(sw){
      const pressed = sw.getAttribute('aria-pressed');
      const isOn = sw.classList.contains('on') || pressed === 'true';
      sw.dataset.state = isOn ? 'on' : 'off';
      if(!sw.getAttribute('aria-label')){
        sw.setAttribute('aria-label', isOn ? 'On' : 'Off');
      }
    });
  }

  function numericSeries(){
    const source = window.S?.spark?.fps;
    if(Array.isArray(source) && source.length > 1){
      return source.map(Number).filter(Number.isFinite).slice(-96);
    }
    const fps = Number(window.S?.lastPayload?.meta?.fps ?? window.S?.lastPayload?.fps ?? 20);
    const base = Number.isFinite(fps) ? fps : 20;
    return Array.from({length: 48}, function(_, i){
      return base + Math.sin(i / 4) * .12 + Math.cos(i / 9) * .06;
    });
  }

  function drawFpsSpark(){
    const canvas = document.getElementById('kpiFpsSpark');
    if(!canvas || typeof canvas.getContext !== 'function') return;
    const rect = canvas.getBoundingClientRect();
    if(rect.width < 2 || rect.height < 2) return;
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const w = Math.max(1, Math.round(rect.width * dpr));
    const h = Math.max(1, Math.round(rect.height * dpr));
    if(canvas.width !== w || canvas.height !== h){
      canvas.width = w;
      canvas.height = h;
    }
    const ctx = canvas.getContext('2d');
    const vals = numericSeries();
    if(vals.length < 2) return;
    const mean = vals.reduce(function(a,b){ return a + b; }, 0) / vals.length;
    let min = Math.min.apply(null, vals);
    let max = Math.max.apply(null, vals);
    if(max - min < .5){
      min = mean - 1;
      max = mean + 1;
    }
    const padX = 10 * dpr;
    const padY = 12 * dpr;
    const plotW = Math.max(1, w - padX * 2);
    const plotH = Math.max(1, h - padY * 2);
    const x = function(i){ return padX + (i / Math.max(1, vals.length - 1)) * plotW; };
    const y = function(v){ return padY + (1 - ((v - min) / Math.max(.001, max - min))) * plotH; };
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    ctx.lineWidth = 1 * dpr;
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--line') || 'rgba(148,163,184,.22)';
    ctx.globalAlpha = .45;
    for(let i = 1; i < 4; i++){
      const gy = padY + (plotH * i / 4);
      ctx.beginPath();
      ctx.moveTo(padX, gy);
      ctx.lineTo(w - padX, gy);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    const brand = getComputedStyle(document.documentElement).getPropertyValue('--brand-500').trim() || '#2563eb';
    const grad = ctx.createLinearGradient(0, padY, 0, h - padY);
    grad.addColorStop(0, 'rgba(37, 99, 235, .26)');
    grad.addColorStop(1, 'rgba(37, 99, 235, 0)');
    ctx.beginPath();
    vals.forEach(function(v, i){
      const px = x(i), py = y(v);
      if(i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.lineTo(w - padX, h - padY);
    ctx.lineTo(padX, h - padY);
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();
    ctx.beginPath();
    vals.forEach(function(v, i){
      const px = x(i), py = y(v);
      if(i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.lineWidth = 2.25 * dpr;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.strokeStyle = brand;
    ctx.stroke();
    const ly = y(vals[vals.length - 1]);
    ctx.beginPath();
    ctx.arc(x(vals.length - 1), ly, 3.4 * dpr, 0, Math.PI * 2);
    ctx.fillStyle = '#fff';
    ctx.fill();
    ctx.lineWidth = 2 * dpr;
    ctx.strokeStyle = brand;
    ctx.stroke();
    ctx.restore();
  }

  function forceConsistency(){
    try {
      const isHome = document.body.getAttribute('data-view') === 'home';

      /* 1. Force Readiness Ring (72px to match v11 density) */
      const rings = document.querySelectorAll('#homeReadinessRing, .home-ring');
      rings.forEach(function(ring){
        const s = ring.style;
        const set = function(p, v){ s.setProperty(p, v, 'important'); };
        set('width', '72px');
        set('height', '72px');
        set('min-width', '72px');
        set('min-height', '72px');
        set('max-width', '72px');
        set('max-height', '72px');
        set('flex', '0 0 72px');
        set('aspect-ratio', '1/1');
        set('border-radius', '50%');
        set('align-self', 'center');
        set('justify-self', 'center');
        set('display', 'grid');
        set('place-items', 'center');
        set('position', 'relative');

        const pct = parseFloat(getComputedStyle(ring).getPropertyValue('--pct')) || 0;
        if(pct <= 0){
          set('background', '#e2e8f0');
        } else {
          s.removeProperty('background');
        }
      });

      /* 2. Force Card Containers and Headers */
      /* overflow+border on all cards; padding:0 only on filled-header cards */
      const allCardSelectors = ['#setupCard', '.home-preview', '.sys-preflight', '.hero-ready', '.card', '.sys-card'];
      allCardSelectors.forEach(function(sel){
        document.querySelectorAll(sel).forEach(function(c){
          c.style.setProperty('overflow', 'hidden', 'important');
          c.style.setProperty('border', '1px solid #dfe8f7', 'important');
        });
      });
      const filledHeaderSelectors = ['#setupCard', '.home-preview', '.sys-preflight', '.hero-ready'];
      filledHeaderSelectors.forEach(function(sel){
        document.querySelectorAll(sel).forEach(function(c){
          c.style.setProperty('padding', '0', 'important');
        });
      });

      const headerSelectors = ['#setupCard>.ch', '.home-preview>.hero-preview-head', '.sys-preflight>.card-header', '.hero-preview-head'];
      headerSelectors.forEach(function(sel){
        document.querySelectorAll(sel).forEach(function(h){
          const s = h.style;
          s.setProperty('margin', '-2px -2px 0 -2px', 'important');
          s.setProperty('width', 'calc(100% + 4px)', 'important');
          s.setProperty('border-radius', '22px 22px 0 0', 'important');
          s.setProperty('z-index', '10', 'important');
          s.setProperty('position', 'relative', 'important');
          s.setProperty('border', '0', 'important');
          /* The Seal: box-shadow + top offset to kill the white hairline */
          s.setProperty('box-shadow', '0 -1px 0 0 rgba(0,0,0,0.02)', 'important');
          s.setProperty('top', '-1px', 'important');
        });
      });

      /* 3. Fix Breadcrumb Layout and hide eyebrow */
      const trail = document.querySelector('.topbar .crumb-trail');
      if(trail){
        trail.style.setProperty('display', 'flex', 'important');
        trail.style.setProperty('flex-direction', 'row', 'important');
        trail.style.setProperty('align-items', 'center', 'important');
        trail.style.setProperty('gap', '8px', 'important');
        trail.style.setProperty('margin', '0', 'important');
      }

      if(isHome){
        const eb = document.getElementById('crumbEyebrow') || document.querySelector('.crumb-eyebrow');
        if(eb){
          eb.style.setProperty('display', 'none', 'important');
          eb.style.setProperty('opacity', '0', 'important');
          eb.style.setProperty('pointer-events', 'none', 'important');
        }
      }
    } catch(e) { console.error('forceConsistency error:', e); }
  }

  function sync(){
    ensureHomeCrumb();
    ensureHelpHome();
    syncExportButton();
    syncSwitchState();
    forceConsistency();
    raf(drawFpsSpark);
  }

  function wrapRender(){
    if(typeof window.render === 'function' && !window.render.__uiuxV11Wrapped){
      const orig = window.render;
      window.render = function(){
        const result = orig.apply(this, arguments);
        setTimeout(sync, 60);
        return result;
      };
      window.render.__uiuxV11Wrapped = true;
    }
    if(typeof window.switchView === 'function' && !window.switchView.__uiuxV11Wrapped){
      const origSwitch = window.switchView;
      window.switchView = function(){
        const result = origSwitch.apply(this, arguments);
        setTimeout(sync, 60);
        setTimeout(sync, 260);
        return result;
      };
      window.switchView.__uiuxV11Wrapped = true;
    }
  }

  function boot(){
    wrapRender();
    sync();
    let syncQueued = 0;
    const queueSync = function(){
      if(syncQueued) return;
      syncQueued = setTimeout(function(){
        syncQueued = 0;
        sync();
      }, 60);
    };
    const obs = window.rvtTrackMutationObserver(new MutationObserver(queueSync));
    obs.observe(document.body, {attributes:true, attributeFilter:['data-view']});
    window.addEventListener('resize', function(){ raf(drawFpsSpark); }, {passive:true});
    /* Frequent interval for the first 10 seconds to catch early renders */
    let count = 0;
    const itv = setInterval(function(){
      sync();
      if(++count > 20) { clearInterval(itv); setInterval(sync, 1500); }
    }, 500);
  }

  if(document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot, {once:true});
  else boot();
})();
;
/* END modules/rvt-v11-uiux-consistency-bugfixes-js.js */

/* BEGIN modules/_anon-001.js */
(function(){
  'use strict';

  /* ═══════════════════════════════════════════════════════════
     1. LOADING SCREEN
     ═══════════════════════════════════════════════════════════ */
  const overlay = document.getElementById('rvtLoadingOverlay');
  const loaderBar = document.getElementById('rvtLoaderBar');
  const loaderStatus = document.getElementById('rvtLoaderStatus');
  let loadProgress = 0;
  let loaderDismissed = false;

  function setLoadProgress(pct, msg) {
    loadProgress = Math.min(100, pct);
    if (loaderBar) loaderBar.style.width = loadProgress + '%';
    if (loaderStatus && msg) loaderStatus.textContent = msg;
  }

  // Progress steps
  setLoadProgress(15, 'Loading externals…');

  function dismissLoadingOverlay(msg) {
    if (loaderDismissed) return;
    loaderDismissed = true;
    setLoadProgress(100, msg || 'Done');
    if (overlay) overlay.classList.add('fade-out');
    setTimeout(function() {
      if (overlay && overlay.parentNode) overlay.parentNode.removeChild(overlay);
    }, 600);
  }

  const _origBoot = window.__rvtBootUnified;
  // Attach progress hooks via intervals
  let bootCheckCount = 0;
  const bootCheck = setInterval(function() {
    bootCheckCount++;
    if (window.Chart) setLoadProgress(35, 'Chart.js loaded');
    if (window.Hammer) setLoadProgress(45, 'Touch support loaded');
    if (window.S) setLoadProgress(60, 'State initialized');
    if (window.S && window.S.charts) setLoadProgress(70, 'Charts ready');
    if (document.body.getAttribute('data-ctl') === 'on') {
      setLoadProgress(90, 'UI ready');
      setTimeout(function() { dismissLoadingOverlay('Done'); }, 100);
      clearInterval(bootCheck);
    }
    if (bootCheckCount > 120) { // 12 seconds max
      dismissLoadingOverlay('Loaded');
      clearInterval(bootCheck);
    }
  }, 100);

  setTimeout(function() {
    dismissLoadingOverlay(document.body.getAttribute('data-ctl') === 'on' ? 'Done' : 'Loaded');
  }, 8000);


  /* ═══════════════════════════════════════════════════════════
     2. IndexedDB STORAGE TIER
     ═══════════════════════════════════════════════════════════ */
  const IDB_NAME = 'rvt-storage';
  const IDB_VERSION = 1;
  const IDB_STORES = ['snapshots', 'session-notes', 'snap-notes', 'waveform-buffers', 'settings-backup'];
  let _idb = null;

  function openIDB() {
    return new Promise(function(resolve, reject) {
      if (_idb) { resolve(_idb); return; }
      try {
        const req = indexedDB.open(IDB_NAME, IDB_VERSION);
        req.onupgradeneeded = function(e) {
          const db = e.target.result;
          IDB_STORES.forEach(function(name) {
            if (!db.objectStoreNames.contains(name)) {
              db.createObjectStore(name, { keyPath: 'id' });
            }
          });
        };
        req.onsuccess = function(e) {
          _idb = e.target.result;
          resolve(_idb);
        };
        req.onerror = function() { reject(req.error); };
      } catch (e) { reject(e); }
    });
  }

  function idbPut(storeName, record) {
    return openIDB().then(function(db) {
      return new Promise(function(resolve, reject) {
        try {
          const tx = db.transaction(storeName, 'readwrite');
          tx.objectStore(storeName).put(record);
          tx.oncomplete = function() { resolve(); };
          tx.onerror = function() { reject(tx.error); };
        } catch (e) { reject(e); }
      });
    });
  }

  function idbGet(storeName, id) {
    return openIDB().then(function(db) {
      return new Promise(function(resolve, reject) {
        try {
          const tx = db.transaction(storeName, 'readonly');
          const req = tx.objectStore(storeName).get(id);
          req.onsuccess = function() { resolve(req.result); };
          req.onerror = function() { reject(req.error); };
        } catch (e) { reject(e); }
      });
    });
  }

  function idbGetAll(storeName) {
    return openIDB().then(function(db) {
      return new Promise(function(resolve, reject) {
        try {
          const tx = db.transaction(storeName, 'readonly');
          const req = tx.objectStore(storeName).getAll();
          req.onsuccess = function() { resolve(req.result || []); };
          req.onerror = function() { reject(req.error); };
        } catch (e) { reject(e); }
      });
    });
  }

  function idbDelete(storeName, id) {
    return openIDB().then(function(db) {
      return new Promise(function(resolve, reject) {
        try {
          const tx = db.transaction(storeName, 'readwrite');
          tx.objectStore(storeName).delete(id);
          tx.oncomplete = function() { resolve(); };
          tx.onerror = function() { reject(tx.error); };
        } catch (e) { reject(e); }
      });
    });
  }

  // Migrate existing localStorage snapshots to IndexedDB
  function migrateToIDB() {
    openIDB().then(function() {
      // Migrate snapshots
      try {
        var snaps = JSON.parse(localStorage.getItem('rvt-snaps') || '[]');
        if (Array.isArray(snaps) && snaps.length > 0) {
          snaps.forEach(function(snap, i) {
            idbPut('snapshots', { id: snap.id || ('snap-' + i), data: snap, ts: snap.ts || Date.now() });
          });
          console.info('[RVT-IDB] Migrated ' + snaps.length + ' snapshots to IndexedDB');
        }
      } catch (_) {}

      // Migrate snap notes
      try {
        var notes = JSON.parse(localStorage.getItem('rvt-snap-notes') || '{}');
        if (notes && typeof notes === 'object') {
          Object.keys(notes).forEach(function(k) {
            idbPut('snap-notes', { id: k, text: notes[k] });
          });
        }
      } catch (_) {}

      // Migrate session notes
      try {
        var sessNotes = JSON.parse(localStorage.getItem('rvt-session-notes') || '{}');
        if (sessNotes && typeof sessNotes === 'object') {
          Object.keys(sessNotes).forEach(function(k) {
            idbPut('session-notes', { id: k, text: sessNotes[k] });
          });
        }
      } catch (_) {}
    }).catch(function(err) {
      console.warn('[RVT-IDB] Migration failed:', err);
    });
  }

  // Run migration on load
  setTimeout(migrateToIDB, 3000);

  // Expose IDB API globally for other scripts
  window.rvtIDB = { put: idbPut, get: idbGet, getAll: idbGetAll, delete: idbDelete, open: openIDB };

  /* SESSION-01: encrypted session continuity checkpoints */
  (function(){
    'use strict';
    var STORE = 'settings-backup';
    var DRAFT_ID = 'current-draft';
    var MARKER_KEY = 'rvt-session-draft-ts';
    var SNAP_KEY = 'rvt-snaps';
    var TTL_MS = 24 * 60 * 60 * 1000;
    var autosaveTimer = null;
    var keyRetryTimer = null;
    var keyRetryCallbacks = [];
    var deferredCheckpoint = false;
    var bootRecoveryRan = false;
    var patched = false;
    var viewCheckpointTimer = null;
    var discardedFingerprint = '';

    function soft(msg, err) {
      try {
        if (typeof window.rvtSoftError === 'function') window.rvtSoftError(msg, err, { surface:false, severity:'warn' });
        else if (typeof window.softError === 'function') window.softError(msg, err);
        else if (window.console && console.warn) console.warn('[SESSION-01] ' + msg, err || '');
      } catch (_) {}
    }
    function safeJson(raw, fallback) {
      try { var v = JSON.parse(raw || 'null'); return v == null ? fallback : v; } catch (_) { return fallback; }
    }
    function getInputValue(name) {
      var el = document.querySelector('input[name="' + name + '"],textarea[name="' + name + '"],#' + name);
      return el ? String(el.value || '').trim() : '';
    }
    function getSubjectId() {
      return String((window.S && window.S.setup && window.S.setup.subject_label) || getInputValue('subject_label') || '').trim();
    }
    function getOperator() {
      return String((window.S && window.S.setup && window.S.setup.operator_label) ||
        (window.S && window.S.ctl && window.S.ctl.current && window.S.ctl.current.params && window.S.ctl.current.params.operator) ||
        (window.S && window.S.ctl && window.S.ctl.current && window.S.ctl.current.params && window.S.ctl.current.params.operator_label) ||
        getInputValue('operator_label') || '').trim();
    }
    function currentParams() {
      var p = window.S && window.S.ctl && window.S.ctl.current && window.S.ctl.current.params;
      return p && typeof p === 'object' ? Object.assign({}, p) : null;
    }
    function currentSessionId() {
      var ctl = window.S && window.S.ctl;
      return (ctl && (ctl.currentSessionId || ctl.lastSessionId || (ctl.current && (ctl.current.session_id || ctl.current.id)))) || null;
    }
    function currentStartTimestamp() {
      var cur = window.S && window.S.ctl && window.S.ctl.current;
      if (!cur) return null;
      if (!cur.__session01StartTimestamp) cur.__session01StartTimestamp = cur.started_at || cur.startedAt || new Date().toISOString();
      return cur.__session01StartTimestamp;
    }
    function activeSubView() {
      var active = document.querySelector('#liveTabsGroup .r-item.sub.active,[role="tab"][aria-selected="true"],.tab.active');
      return active ? (active.getAttribute('data-target') || active.id || active.getAttribute('aria-controls') || null) : null;
    }
    function activeProfileName() {
      return (window.S && window.S.activeSubjectProfileId) ||
        (window.S && window.S.setup && window.S.setup.subject_profile_id) ||
        localStorage.getItem('rvt-subject-profile') || null;
    }
    function notesText() {
      var el = document.getElementById('sessionNotesText');
      return el ? String(el.value || '') : '';
    }
    function isActiveSession() {
      var ctl = window.S && window.S.ctl;
      return !!(ctl && (ctl.current || ctl.currentSessionId || ctl.status === 'active' || ctl.status === 'running'));
    }
    function encryptionBlocked() {
      return !!(window.rvtSec01 && typeof window.rvtSec01.isEncrypted === 'function' && !window.rvtSec01.isEncrypted());
    }
    function scheduleKeyRetry(fn, timeoutMs) {
      var start = Date.now();
      if (typeof fn === 'function') keyRetryCallbacks.push(fn);
      if (keyRetryTimer) return;
      keyRetryTimer = setInterval(function(){
        try {
          if (!window.rvtSec01 || typeof window.rvtSec01.isEncrypted !== 'function') return;
          if (window.rvtSec01.isEncrypted()) {
            clearInterval(keyRetryTimer);
            keyRetryTimer = null;
            if (deferredCheckpoint) {
              deferredCheckpoint = false;
              window.rvtSession01.checkpoint().catch(function(err){ soft('deferred checkpoint failed', err); });
            }
            var callbacks = keyRetryCallbacks.splice(0);
            callbacks.forEach(function(cb){ try { cb(); } catch (err) { soft('key retry callback failed', err); } });
          } else if (Date.now() - start > (timeoutMs || 60000)) {
            clearInterval(keyRetryTimer);
            keyRetryTimer = null;
          }
        } catch (err) { soft('key retry failed', err); }
      }, 500);
    }
    function buildDraft() {
      var subjectId = getSubjectId();
      var notes = notesText();
      var active = isActiveSession();
      if (!subjectId && !notes && !active) return null;
      var snaps = Array.isArray(window.S && window.S.snaps) ? window.S.snaps : [];
      return {
        id: DRAFT_ID,
        schemaVersion: 1,
        ts: Date.now(),
        subjectId: subjectId,
        sessionId: currentSessionId(),
        operator: getOperator(),
        startTimestamp: currentStartTimestamp(),
        view: (document.body && document.body.dataset && document.body.dataset.view) || document.body.getAttribute('data-view') || 'home',
        subView: activeSubView(),
        sessionNotes: notes,
        snapshotIds: snaps.map(function(s){ return s && s.id; }).filter(Boolean),
        profileName: activeProfileName(),
        params: currentParams()
      };
    }
    function wellFormed(draft) {
      return !!(draft && draft.id === DRAFT_ID && draft.schemaVersion === 1 && draft.ts && draft.subjectId);
    }
    function fingerprint(draft) {
      if (!draft) return '';
      return JSON.stringify({
        subjectId: draft.subjectId || '',
        sessionId: draft.sessionId || '',
        operator: draft.operator || '',
        view: draft.view || '',
        subView: draft.subView || '',
        sessionNotes: draft.sessionNotes || '',
        snapshotIds: draft.snapshotIds || [],
        profileName: draft.profileName || ''
      });
    }
    async function checkpoint() {
      try {
        if (!window.rvtIDB || typeof window.rvtIDB.put !== 'function') return false;
        if (encryptionBlocked()) {
          deferredCheckpoint = true;
          scheduleKeyRetry(null, 60000);
          return false;
        }
        var draft = buildDraft();
        if (!draft) return false;
        if (discardedFingerprint && fingerprint(draft) === discardedFingerprint) return false;
        discardedFingerprint = '';
        await window.rvtIDB.put(STORE, draft);
        localStorage.setItem(MARKER_KEY, String(draft.ts));
        return true;
      } catch (err) {
        soft('checkpoint failed', err);
        return false;
      }
    }
    async function getOrphan() {
      try {
        var ts = parseInt(localStorage.getItem(MARKER_KEY) || '0', 10);
        if (!ts || Date.now() - ts > TTL_MS) return null;
        if (!window.rvtIDB || typeof window.rvtIDB.get !== 'function') return null;
        if (encryptionBlocked()) return null;
        var row = await window.rvtIDB.get(STORE, DRAFT_ID);
        return wellFormed(row) ? row : null;
      } catch (err) {
        soft('orphan read failed', err);
        return null;
      }
    }
    async function hasOrphan() {
      return !!(await getOrphan());
    }
    async function discard() {
      try { discardedFingerprint = fingerprint(buildDraft()); } catch (_) {}
      try {
        if (window.rvtIDB && typeof window.rvtIDB.delete === 'function') await window.rvtIDB.delete(STORE, DRAFT_ID);
      } catch (err) { soft('draft delete failed', err); }
      try { localStorage.removeItem(MARKER_KEY); } catch (_) {}
    }
    function setSetupField(name, value) {
      if (value == null) return;
      if (window.S && window.S.setup) window.S.setup[name] = value;
      var nodes = document.querySelectorAll('input[name="' + name + '"],textarea[name="' + name + '"],select[name="' + name + '"],#' + name);
      nodes.forEach(function(el){
        try {
          el.value = value;
          el.dispatchEvent(new Event('input', { bubbles:true }));
          el.dispatchEvent(new Event('change', { bubbles:true }));
        } catch (_) {}
      });
    }
    function restoreSnapshots(ids) {
      if (!Array.isArray(ids) || !window.S) return;
      var all = safeJson(localStorage.getItem(SNAP_KEY), []);
      if (!Array.isArray(all)) all = [];
      var keep = new Set(ids);
      window.S.snaps = all.filter(function(s){ return s && keep.has(s.id); });
      try { localStorage.setItem(SNAP_KEY, JSON.stringify(window.S.snaps)); } catch (_) {}
      try {
        if (typeof window.renderSnaps === 'function') window.renderSnaps();
        else if (typeof window.renderSnapshots === 'function') window.renderSnapshots();
        else if (typeof window.render === 'function' && window.S.lastPayload) window.render(window.S.lastPayload);
      } catch (err) { soft('snapshot refresh failed', err); }
    }
    async function serverSessionMatches(draft) {
      if (!draft || !draft.sessionId) return false;
      try {
        var current = null;
        if (typeof window.apiJson === 'function') current = await window.apiJson('/api/session/current');
        else {
          var res = await fetch('/api/session/current', { cache:'no-store' });
          if (!res.ok) return false;
          current = await res.json();
        }
        var id = current && (current.session_id || current.id || current.currentSessionId || (current.current && (current.current.session_id || current.current.id)));
        var status = String((current && (current.status || (current.current && current.current.status))) || '').toLowerCase();
        return !!(id && String(id) === String(draft.sessionId) && !['stopped','completed','idle','none'].includes(status));
      } catch (_) {
        return false;
      }
    }
    async function recover() {
      try {
        var draft = await getOrphan();
        if (!draft) return false;
        window.S = window.S || {};
        window.S.setup = window.S.setup || {};
        window.S.ctl = window.S.ctl || {};
        setSetupField('subject_label', draft.subjectId || '');
        setSetupField('operator_label', draft.operator || '');
        if (draft.profileName) {
          window.S.activeSubjectProfileId = draft.profileName;
          window.S.setup.subject_profile_id = draft.profileName;
          try { localStorage.setItem('rvt-subject-profile', draft.profileName); } catch (_) {}
        }
        if (draft.params && typeof draft.params === 'object') {
          Object.keys(draft.params).forEach(function(k){
            if (k in window.S.setup && draft.params[k] != null) window.S.setup[k] = draft.params[k];
          });
        }
        var notes = document.getElementById('sessionNotesText');
        if (notes) {
          notes.value = draft.sessionNotes || '';
          try {
            if (typeof window.saveSessionNotes === 'function') window.saveSessionNotes(notes.value);
            else notes.dispatchEvent(new Event('input', { bubbles:true }));
          } catch (err) { soft('notes restore failed', err); }
        }
        restoreSnapshots(draft.snapshotIds);
        var liveConfirmed = await serverSessionMatches(draft);
        if (liveConfirmed) {
          window.S.ctl.currentSessionId = draft.sessionId;
          if (!window.S.ctl.current) window.S.ctl.current = { session_id:draft.sessionId, params:draft.params || {} };
        } else {
          window.S.ctl.currentSessionId = null;
          window.S.ctl.current = null;
          if (typeof window.toast === 'function') window.toast('Previous session was not active on the server - context restored only.', 'info');
        }
        try { if (typeof window.refreshSessionHeader === 'function') await window.refreshSessionHeader(); } catch (_) {}
        var targetView = draft.view || 'home';
        if (!liveConfirmed && /^live/.test(targetView)) targetView = 'home';
        try {
          if (typeof window.switchView === 'function') window.switchView(targetView);
          else if (document.body) document.body.dataset.view = targetView;
          if (liveConfirmed && draft.subView && typeof window.switchTab === 'function') window.switchTab(draft.subView);
        } catch (err) { soft('view restore failed', err); }
        await discard();
        startAutoSave();
        return true;
      } catch (err) {
        soft('recovery failed', err);
        return false;
      }
    }
    function startAutoSave() {
      if (autosaveTimer) return;
      autosaveTimer = setInterval(function(){ checkpoint(); }, 30000);
    }
    function stopAutoSave() {
      if (autosaveTimer) clearInterval(autosaveTimer);
      autosaveTimer = null;
    }
    function relativeTime(ts) {
      var mins = Math.max(0, Math.round((Date.now() - Number(ts || 0)) / 60000));
      if (mins < 1) return 'just now';
      if (mins < 60) return mins + ' minute' + (mins === 1 ? '' : 's') + ' ago';
      var hours = Math.round(mins / 60);
      return hours + ' hour' + (hours === 1 ? '' : 's') + ' ago';
    }
    function rvtConfirmRecovery(opts) {
      opts = opts || {};
      var draft = opts.draft || {};
      var bd = document.getElementById('rvt-session01-backdrop');
      if (!bd) {
        bd = document.createElement('div');
        bd.id = 'rvt-session01-backdrop';
        bd.setAttribute('role', 'dialog');
        bd.setAttribute('aria-modal', 'true');
        bd.setAttribute('aria-labelledby', 'rvtSession01Title');
        bd.innerHTML =
          '<div id="rvt-session01-modal">' +
            '<h3 id="rvtSession01Title">Resume previous session?</h3>' +
            '<p id="rvtSession01Subtitle"></p>' +
            '<div class="rvt-session01-actions">' +
              '<button type="button" id="rvtSession01Discard">Discard</button>' +
              '<button type="button" id="rvtSession01Resume">Resume</button>' +
            '</div>' +
          '</div>';
        var style = document.createElement('style');
        style.textContent =
          '#rvt-session01-backdrop{position:fixed;inset:0;z-index:99999;display:none;align-items:center;justify-content:center;background:rgba(8,13,24,.54);padding:20px}' +
          '#rvt-session01-backdrop.rvt-session01-visible{display:flex}' +
          '#rvt-session01-modal{width:min(460px,100%);background:var(--surface,#fff);color:var(--ink,#111827);border:1px solid var(--line,#d6dbe6);border-radius:8px;padding:20px;box-shadow:0 24px 80px rgba(0,0,0,.28)}' +
          '#rvt-session01-modal h3{margin:0 0 8px;font-size:20px;line-height:1.25}' +
          '#rvtSession01Subtitle{margin:0 0 18px;color:var(--ink-500,#4b5563);line-height:1.45}' +
          '.rvt-session01-actions{display:flex;justify-content:flex-end;gap:10px}' +
          '#rvtSession01Discard,#rvtSession01Resume{min-height:40px;border-radius:8px;border:1px solid var(--line,#d6dbe6);padding:0 14px;font:inherit;cursor:pointer}' +
          '#rvtSession01Resume{border-color:var(--primary,#2563eb);background:var(--primary,#2563eb);color:#fff}';
        document.head.appendChild(style);
        document.body.appendChild(bd);
      }
      var subtitle = bd.querySelector('#rvtSession01Subtitle');
      var resume = bd.querySelector('#rvtSession01Resume');
      var discardBtn = bd.querySelector('#rvtSession01Discard');
      subtitle.textContent = (draft.subjectId || 'Previous subject') + ' - Updated ' + relativeTime(draft.ts);
      function close() {
        bd.classList.remove('rvt-session01-visible');
        bd.style.display = 'none';
        document.removeEventListener('keydown', onKey, true);
        bd.onclick = null;
        resume.onclick = null;
        discardBtn.onclick = null;
      }
      function onKey(e) {
        if (e.key === 'Escape') close();
        if (e.key === 'Tab') {
          var focusables = Array.prototype.slice.call(bd.querySelectorAll('button,[href],input,textarea,select,[tabindex]:not([tabindex="-1"])'));
          if (!focusables.length) return;
          var first = focusables[0], last = focusables[focusables.length - 1];
          if (e.shiftKey && document.activeElement === first) { e.preventDefault(); last.focus(); }
          else if (!e.shiftKey && document.activeElement === last) { e.preventDefault(); first.focus(); }
        }
      }
      bd.onclick = function(e){ if (e.target === bd) close(); };
      discardBtn.onclick = function(){ discard().finally(close); };
      resume.onclick = function(){ recover().finally(close); };
      bd.style.display = 'flex';
      bd.classList.add('rvt-session01-visible');
      document.addEventListener('keydown', onKey, true);
      setTimeout(function(){ resume.focus(); }, 0);
    }
    async function bootRecoveryCheck() {
      if (bootRecoveryRan) return;
      try {
        var ready = await new Promise(function(resolve){
          var start = Date.now();
          var poll = setInterval(function(){
            var ok = !!(window.S && window.rvtIDB && window.rvtSec01 && (window.S.__inited || document.readyState === 'complete'));
            if (ok || Date.now() - start > 10000) {
              clearInterval(poll);
              resolve(ok);
            }
          }, 100);
        });
        if (!ready) return;
        var marker = parseInt(localStorage.getItem(MARKER_KEY) || '0', 10);
        if (!marker || Date.now() - marker > TTL_MS) return;
        if (encryptionBlocked()) {
          scheduleKeyRetry(bootRecoveryCheck, 60000);
          return;
        }
        bootRecoveryRan = true;
        var orphan = await getOrphan();
        if (orphan) rvtConfirmRecovery({ draft: orphan });
      } catch (err) { soft('boot recovery failed', err); }
    }
    function debounceCheckpoint() {
      clearTimeout(viewCheckpointTimer);
      viewCheckpointTimer = setTimeout(function(){ checkpoint(); }, 1000);
    }
    function patchGlobals() {
      if (patched) return true;
      var any = false;
      if (typeof window.switchView === 'function' && !window.switchView.__rvtSession01Wrapped) {
        var originalSwitchView = window.switchView;
        window.switchView = function() {
          var out = originalSwitchView.apply(this, arguments);
          debounceCheckpoint();
          return out;
        };
        window.switchView.__rvtSession01Wrapped = true;
        any = true;
      }
      if (typeof window.switchTab === 'function' && !window.switchTab.__rvtSession01Wrapped) {
        var originalSwitchTab = window.switchTab;
        window.switchTab = function() {
          var out = originalSwitchTab.apply(this, arguments);
          debounceCheckpoint();
          return out;
        };
        window.switchTab.__rvtSession01Wrapped = true;
        any = true;
      }
      if (typeof window.stopActiveSession === 'function' && !window.stopActiveSession.__rvtSession01Wrapped) {
        var originalStop = window.stopActiveSession;
        window.stopActiveSession = async function() {
          var before = window.S && window.S.ctl && window.S.ctl.current;
          var caught = null;
          try { await originalStop.apply(this, arguments); }
          catch (err) { caught = err; }
          var after = window.S && window.S.ctl && window.S.ctl.current;
          if (!caught && before && !after) {
            try { await discard(); } catch (_) {}
          }
          if (caught) throw caught;
        };
        window.stopActiveSession.__rvtSession01Wrapped = true;
        any = true;
      }
      patched = any || patched;
      return patched;
    }
    document.addEventListener('visibilitychange', function(){
      if (document.visibilityState === 'hidden') checkpoint();
    });
    window.addEventListener('pagehide', function(){ checkpoint(); });
    window.addEventListener('beforeunload', function(){ checkpoint(); });
    window.addEventListener('DOMContentLoaded', function(){
      setTimeout(function(){
        try { if (window.S) window.S.__inited = true; } catch (_) {}
        startAutoSave();
        bootRecoveryCheck();
      }, 1200);
    }, { once:true });
    var patchTimer = setInterval(function(){
      try {
        patchGlobals();
        if (patched) clearInterval(patchTimer);
      } catch (err) { soft('global patch failed', err); }
    }, 250);
    window.rvtConfirmRecovery = rvtConfirmRecovery;
    window.rvtSession01 = {
      checkpoint: checkpoint,
      hasOrphan: hasOrphan,
      getOrphan: getOrphan,
      recover: recover,
      discard: discard,
      startAutoSave: startAutoSave,
      stopAutoSave: stopAutoSave,
      __setTtlMs: function(v){ TTL_MS = Number(v) || TTL_MS; },
      __bootRecoveryCheck: bootRecoveryCheck
    };
  })();


  /* ═══════════════════════════════════════════════════════════
     3. PDF REPORT EXPORT
     ═══════════════════════════════════════════════════════════ */
  function injectPdfButton() {
    // Find the report view's verdict-hero or action areas
    var tries = 0;
    var injectInterval = setInterval(function() {
      tries++;
      if (tries > 60) { clearInterval(injectInterval); return; }

      // Look for existing export buttons in the report view to place alongside
      var reportView = document.getElementById('view-report');
      if (!reportView) return;

      // Check if already injected (either by this worker or rvtReportPdfButton)
      if (reportView.querySelector('.rvt-pdf-export-btn') || document.getElementById('rvtReportPdfButton')) { clearInterval(injectInterval); return; }

      // Find verdict-hero actions area or the first card header actions
      var heroEl = reportView.querySelector('.verdict-hero');
      if (!heroEl) return;

      // Look for existing action row inside the hero
      var actionRow = heroEl.querySelector('.vh-actions, .verdict-actions, .live-verdict-actions');
      if (!actionRow) {
        // Create an action row after the hero's last child
        actionRow = document.createElement('div');
        actionRow.style.cssText = 'display:flex;flex-wrap:wrap;gap:8px;margin-top:14px;';
        heroEl.appendChild(actionRow);
      }

      var pdfBtn = document.createElement('button');
      pdfBtn.className = 'rvt-pdf-export-btn';
      pdfBtn.type = 'button';
      pdfBtn.title = 'Export report as PDF';
      pdfBtn.innerHTML = '<span class="material-symbols-rounded">picture_as_pdf</span>Export PDF';
      pdfBtn.addEventListener('click', exportReportPdf);
      actionRow.appendChild(pdfBtn);

      // Also add alongside JSON/CSV/HTML export chips if they exist
      var chipBtns = reportView.querySelectorAll('.chip-btn[onclick*="export"], .chip-btn[onclick*="download"]');
      if (chipBtns.length > 0) {
        var lastChip = chipBtns[chipBtns.length - 1];
        var pdfChip = document.createElement('button');
        pdfChip.className = 'chip-btn rvt-pdf-export-btn';
        pdfChip.type = 'button';
        pdfChip.title = 'Export report as PDF';
        pdfChip.innerHTML = '<span class="material-symbols-rounded">picture_as_pdf</span>PDF';
        pdfChip.addEventListener('click', exportReportPdf);
        lastChip.parentNode.insertBefore(pdfChip, lastChip.nextSibling);
      }

      clearInterval(injectInterval);
    }, 500);
  }

  function exportReportPdf() {
    // Build a clean print-friendly version
    var reportView = document.getElementById('view-report');
    if (!reportView) {
      showRvtToast('No report data available. Complete a session first.', 'warn');
      return;
    }

    // Open a new window with the report content for printing
    var printWin = window.open('', '_blank', 'width=900,height=700');
    if (!printWin) {
      showRvtToast('Pop-up blocked — allow pop-ups to export PDF.', 'warn');
      return;
    }

    // Clone theme vars
    var rootStyles = getComputedStyle(document.documentElement);
    var theme = document.documentElement.getAttribute('data-theme') || 'light';

    // Get session metadata
    var operator = (window.S && window.S.operatorName) ? window.S.operatorName : 'Unknown';
    var sessionId = (window.S && window.S.sessionId) ? window.S.sessionId : '--';
    var timestamp = new Date().toISOString().replace('T', ' ').split('.')[0];

    // Get the report HTML content
    var reportClone = reportView.cloneNode(true);
    // Remove interactive buttons
    reportClone.querySelectorAll('button, .ca-btn, .rvt-pdf-export-btn').forEach(function(el) { el.remove(); });

    var printHtml = '<!DOCTYPE html><html data-theme="light"><head><meta charset="utf-8">' +
      '<title>RVT Session Report — ' + sessionId + '</title>' +
      '<link href="./fonts/rvt-fonts.css" rel="stylesheet">' +
      '<style>' +
      document.getElementById('rvt-consolidated-css').textContent +
      '\n body { background: #fff !important; padding: 32px; max-width: 900px; margin: 0 auto; font-family: Inter, system-ui, sans-serif; }' +
      '\n .verdict-hero { grid-template-columns: 1fr !important; }' +
      '\n .card { page-break-inside: avoid; break-inside: avoid; margin-bottom: 16px !important; box-shadow: none !important; border: 1px solid #e2e8f0 !important; }' +
      '\n .report-grid { grid-template-columns: repeat(2, 1fr) !important; }' +
      '\n .verdict-card { page-break-inside: avoid; }' +
      '\n @media print { @page { size: A4; margin: 16mm; } body { padding: 0; } }' +
      '\n .pdf-header { display: flex; justify-content: space-between; align-items: flex-start; margin-bottom: 24px; padding-bottom: 16px; border-bottom: 2px solid #2563eb; }' +
      '\n .pdf-header-brand { font-family: "Inter Tight", sans-serif; font-size: 20px; font-weight: 800; color: #0b1220; }' +
      '\n .pdf-header-meta { text-align: right; font-size: 12px; color: #64748b; line-height: 1.6; }' +
      '\n .pdf-footer { margin-top: 24px; padding-top: 12px; border-top: 1px solid #e2e8f0; font-size: 11px; color: #94a3b8; text-align: center; }' +
      '</style></head><body>' +
      '<div class="pdf-header">' +
        '<div class="pdf-header-brand">Radar Vital Trainer — Session Report</div>' +
        '<div class="pdf-header-meta">Session: ' + sessionId + '<br>Operator: ' + operator + '<br>Exported: ' + timestamp + '</div>' +
      '</div>' +
      reportClone.innerHTML +
      '<div class="pdf-footer">Generated by Radar Vital Trainer v15.0 · ' + timestamp + '</div>' +
      '<script>setTimeout(function(){ window.print(); }, 800);<\/script>' +
      '</body></html>';

    printWin.document.write(printHtml);
    printWin.document.close();
  }

  // Watch for view switches to inject PDF button
  var _viewObserver = window.rvtTrackMutationObserver(new MutationObserver(function() {
    if (document.body.getAttribute('data-view') === 'report') {
      setTimeout(injectPdfButton, 300);
    }
  }));
  _viewObserver.observe(document.body, { attributes: true, attributeFilter: ['data-view'] });
  // Also try on load
  setTimeout(injectPdfButton, 2000);


  /* ═══════════════════════════════════════════════════════════
     4. REAL-TIME HR/RR VOICE READOUTS
     ═══════════════════════════════════════════════════════════ */
  var voiceReadoutEnabled = false;
  var voiceReadoutInterval = 30; // seconds
  var voiceReadoutTimer = null;
  var voiceIndicator = document.getElementById('rvtVoiceIndicator');
  var voiceText = document.getElementById('rvtVoiceText');

  // Load saved preference
  try {
    var savedVR = JSON.parse(localStorage.getItem('rvt-voice-readout') || 'null');
    if (savedVR) {
      voiceReadoutEnabled = !!savedVR.enabled;
      voiceReadoutInterval = savedVR.interval || 30;
    }
  } catch (_) {}

  function speakReadout(text) {
    if (!window.speechSynthesis) return;
    // Cancel any queued utterances (latest-wins)
    window.speechSynthesis.cancel();
    var utter = new SpeechSynthesisUtterance(text);
    utter.rate = 1.1;
    utter.pitch = 1.0;
    utter.volume = 0.8;
    try {
      var volSlider = document.querySelector('[data-audio-volume]');
      if (volSlider) utter.volume = Math.max(0.1, parseInt(volSlider.value, 10) / 100);
    } catch (_) {}
    window.speechSynthesis.speak(utter);
  }

  function doVoiceReadout() {
    if (!voiceReadoutEnabled || !window.S) return;
    // Only read out during active live view
    var view = document.body.getAttribute('data-view');
    if (view !== 'live' && view !== 'home') return;

    var hr = '--', rr = '--';
    try {
      var hrEl = document.getElementById('kpiHr');
      var rrEl = document.getElementById('kpiRr');
      if (hrEl) hr = hrEl.textContent.trim();
      if (rrEl) rr = rrEl.textContent.trim();
    } catch (_) {}

    if (hr === '--' && rr === '--') return;

    var msg = 'Heart rate ' + hr + ' BPM. Respiratory rate ' + rr + ' breaths per minute.';
    speakReadout(msg);

    // Show indicator
    if (voiceText) voiceText.textContent = 'HR ' + hr + ' · RR ' + rr;
    if (voiceIndicator) {
      voiceIndicator.classList.add('show');
      setTimeout(function() { voiceIndicator.classList.remove('show'); }, 3000);
    }
  }

  function startVoiceReadout() {
    stopVoiceReadout();
    if (!voiceReadoutEnabled) return;
    voiceReadoutTimer = setInterval(doVoiceReadout, voiceReadoutInterval * 1000);
    // Do one immediately
    setTimeout(doVoiceReadout, 1000);
  }

  function stopVoiceReadout() {
    if (voiceReadoutTimer) { clearInterval(voiceReadoutTimer); voiceReadoutTimer = null; }
  }

  function setVoiceReadout(enabled, interval) {
    voiceReadoutEnabled = !!enabled;
    if (typeof interval === 'number' && interval >= 10) voiceReadoutInterval = interval;
    try {
      localStorage.setItem('rvt-voice-readout', JSON.stringify({ enabled: voiceReadoutEnabled, interval: voiceReadoutInterval }));
    } catch (_) {}
    if (voiceReadoutEnabled) startVoiceReadout();
    else stopVoiceReadout();
    syncVoiceReadoutUI();
  }

  // Expose globally
  window.setVoiceReadout = setVoiceReadout;

  function syncVoiceReadoutUI() {
    var sw = document.getElementById('voiceReadoutSwitch');
    if (sw) {
      sw.setAttribute('aria-pressed', voiceReadoutEnabled ? 'true' : 'false');
      if (voiceReadoutEnabled) sw.classList.add('on'); else sw.classList.remove('on');
    }
    document.querySelectorAll('.rvt-voice-interval-seg button').forEach(function(btn) {
      var v = parseInt(btn.getAttribute('data-interval'), 10);
      if (v === voiceReadoutInterval) btn.classList.add('active');
      else btn.classList.remove('active');
    });
  }

  // Inject voice readout setting row into Settings view
  function injectVoiceReadoutSetting() {
    var settingsView = document.getElementById('view-settings');
    if (!settingsView) return;
    if (settingsView.querySelector('#voiceReadoutRow')) return;

    // Find the voice alerts row to place after it
    var voiceAlertSwitch = document.getElementById('voiceAlertSwitch');
    var targetRow = voiceAlertSwitch ? voiceAlertSwitch.closest('.set-r') : null;

    var row = document.createElement('div');
    row.className = 'set-r';
    row.id = 'voiceReadoutRow';
    row.innerHTML =
      '<div class="set-copy">' +
        '<strong>Periodic vital-sign readouts</strong>' +
        '<span>Speak HR and RR at regular intervals during live monitoring. Useful for eyes-busy workflows.</span>' +
      '</div>' +
      '<div style="display:flex;align-items:center;gap:12px;">' +
        '<div class="rvt-voice-interval-seg">' +
          '<button data-interval="15" type="button" onclick="setVoiceReadout(true,15)">15s</button>' +
          '<button data-interval="30" type="button" onclick="setVoiceReadout(true,30)">30s</button>' +
          '<button data-interval="60" type="button" onclick="setVoiceReadout(true,60)">60s</button>' +
        '</div>' +
        '<button class="sw" id="voiceReadoutSwitch" onclick="setVoiceReadout(!(' + voiceReadoutEnabled + '),' + voiceReadoutInterval + ')" ' +
          'aria-pressed="' + (voiceReadoutEnabled ? 'true' : 'false') + '" ' +
          'title="Toggle periodic readouts" type="button"></button>' +
      '</div>';

    if (targetRow && targetRow.parentNode) {
      targetRow.parentNode.insertBefore(row, targetRow.nextSibling);
    } else {
      // Fallback: append to first settings group
      var firstGroup = settingsView.querySelector('.set-g');
      if (firstGroup) firstGroup.appendChild(row);
    }

    syncVoiceReadoutUI();
  }

  // Start if was enabled
  if (voiceReadoutEnabled) {
    setTimeout(startVoiceReadout, 5000);
  }


  /* ═══════════════════════════════════════════════════════════
     5. VISUAL BUG FIXES & QoL
     ═══════════════════════════════════════════════════════════ */

  // Fix: Tie hxBeat/rxBreath animation opacity to PQI confidence
  function syncAnimationWithPQI() {
    try {
      if (!window.S || !window.S.lastLivePayload) return;
      var p = window.S.lastLivePayload;
      var pqiHeart = p.pqi_heart != null ? Number(p.pqi_heart) : 1;
      var pqiBreath = p.pqi_breath != null ? Number(p.pqi_breath) : 1;

      document.querySelectorAll('.hx-beat').forEach(function(el) {
        if (pqiHeart < 0.3) el.classList.add('stale');
        else el.classList.remove('stale');
      });
      document.querySelectorAll('.rx-breath').forEach(function(el) {
        if (pqiBreath < 0.3) el.classList.add('stale');
        else el.classList.remove('stale');
      });
    } catch (_) {}
  }

  // Fix: Show disable reason on Start Session button
  function syncStartButtonReason() {
    try {
      var startBtn = document.querySelector('.home-start-btn, .setup-start, [onclick*="startSession"]');
      if (!startBtn || !startBtn.disabled) return;
      if (startBtn.querySelector('.rvt-disable-reason')) return;

      var reason = '';
      var connEl = document.getElementById('connT');
      if (connEl && /connecting|lost|error/i.test(connEl.textContent)) {
        reason = 'Waiting for connection';
      } else {
        var checks = document.querySelectorAll('.pf-item.fail, .pf-item.error');
        if (checks.length > 0) reason = checks.length + ' preflight check(s) failing';
        else reason = 'Preflight checks pending';
      }

      if (reason) {
        var reasonEl = document.createElement('span');
        reasonEl.className = 'rvt-disable-reason';
        reasonEl.style.cssText = 'display:block;font-size:11px;font-weight:500;opacity:0.7;margin-top:2px;';
        reasonEl.textContent = reason;
        startBtn.appendChild(reasonEl);
      }
    } catch (_) {}
  }

  // Fix: Pause button shows elapsed time
  var pauseStartTime = null;
  function syncPauseLabel() {
    try {
      if (!window.S || !window.S.paused) {
        pauseStartTime = null;
        return;
      }
      if (!pauseStartTime) pauseStartTime = Date.now();
      var elapsed = Math.floor((Date.now() - pauseStartTime) / 1000);
      var min = Math.floor(elapsed / 60);
      var sec = elapsed % 60;
      var label = 'Paused ' + min + ':' + (sec < 10 ? '0' : '') + sec;

      var pauseEl = document.getElementById('overflowPauseLabel');
      if (pauseEl && window.S.paused) pauseEl.textContent = label;

      var pauseIcon = document.getElementById('pauseIcon');
      if (pauseIcon && pauseIcon.parentNode) {
        pauseIcon.parentNode.title = label;
      }
    } catch (_) {}
  }

  // Utility toast
  function showRvtToast(msg, type) {
    try {
      if (typeof window.showToast === 'function') { window.showToast(msg, type); return; }
      // Fallback
      console.info('[RVT] ' + msg);
    } catch (_) {}
  }

  // Master sync loop
  function v15Sync() {
    syncAnimationWithPQI();
    syncStartButtonReason();
    syncPauseLabel();
  }

  // Hook into render cycle
  function v15Boot() {
    // Inject settings
    setTimeout(injectVoiceReadoutSetting, 2000);

    // Re-inject on view switch
    var _origSwitchView = window.switchView;
    if (typeof _origSwitchView === 'function' && !_origSwitchView.__v15Wrapped) {
      window.switchView = function() {
        var result = _origSwitchView.apply(this, arguments);
        setTimeout(function() {
          if (document.body.getAttribute('data-view') === 'settings') {
            setTimeout(injectVoiceReadoutSetting, 200);
          }
          if (document.body.getAttribute('data-view') === 'report') {
            setTimeout(injectPdfButton, 300);
          }
        }, 100);
        return result;
      };
      window.switchView.__v15Wrapped = true;
    }

    // Wrap render for sync
    var _origRender = window.render;
    if (typeof _origRender === 'function' && !_origRender.__v15Wrapped) {
      window.render = function() {
        var result = _origRender.apply(this, arguments);
        setTimeout(v15Sync, 50);
        return result;
      };
      window.render.__v15Wrapped = true;
    }

    // Periodic sync
    setInterval(v15Sync, 2000);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', v15Boot, { once: true });
  } else {
    setTimeout(v15Boot, 500);
  }

})();
;
/* END modules/_anon-001.js */

/* BEGIN modules/_anon-002.js */
(function(){
  'use strict';

  /* ═══════════════════════════════════════════════════════════
     BUG-01: MutationObserver leak fix
     ═══════════════════════════════════════════════════════════ */
  function cleanupTrackedObservers() {
    try {
      if (typeof window.rvtDisconnectTrackedObservers === 'function') window.rvtDisconnectTrackedObservers();
      if (window.S && Array.isArray(window.S.__mutationObservers)) {
        window.S.__mutationObservers.forEach(function(o) { try { o.disconnect(); } catch(_){} });
        window.S.__mutationObservers.length = 0;
      }
      // Disconnect any anonymous observers via known accessor
      if (window.S && window.S.a11yObserver && typeof window.S.a11yObserver.disconnect === 'function') {
        window.S.a11yObserver.disconnect();
      }
    } catch(_){}
  }
  window.addEventListener('beforeunload', cleanupTrackedObservers);
  window.addEventListener('pagehide', cleanupTrackedObservers);

  /* ═══════════════════════════════════════════════════════════
     BUG-02: Chart.js destroy registry
     ═══════════════════════════════════════════════════════════ */
  if (!window.__rvtChartRegistry) window.__rvtChartRegistry = new Set();
  if (window.Chart && !window.Chart.__rvtPatched) {
    var _origConstructor = window.Chart;
    window.Chart.__rvtPatched = true;
    // Hook Chart instances
    document.addEventListener('chart:created', function(e) {
      if (e.detail && e.detail.chart) window.__rvtChartRegistry.add(e.detail.chart);
    });
  }

  /* ═══════════════════════════════════════════════════════════
     BUG-08: localStorage quota handling
     ═══════════════════════════════════════════════════════════ */
  var _origSetItem = Storage.prototype.setItem;
  Storage.prototype.setItem = function(key, value) {
    try {
      return _origSetItem.call(this, key, value);
    } catch (e) {
      if (e && (e.code === 22 || e.name === 'QuotaExceededError' || /quota/i.test(e.message || ''))) {
        console.warn('[RVT] localStorage quota exceeded for key:', key);
        var banner = document.getElementById('rvtQuotaBanner');
        if (banner) banner.classList.add('show');
        // Try to free space by trimming snapshot data
        try {
          var snaps = JSON.parse(localStorage.getItem('rvt-snaps') || '[]');
          if (Array.isArray(snaps) && snaps.length > 50) {
            // keep most recent 50
            snaps = snaps.slice(-50);
            _origSetItem.call(localStorage, 'rvt-snaps', JSON.stringify(snaps));
            // Retry original write
            return _origSetItem.call(this, key, value);
          }
        } catch(_){}
      }
      throw e;
    }
  };
  document.addEventListener('click', function(e) {
    if (e.target && e.target.id === 'rvtQuotaDismiss') {
      document.getElementById('rvtQuotaBanner').classList.remove('show');
    }
  });

  /* ═══════════════════════════════════════════════════════════
     BUG-07: Auto-reconnect cap
     ═══════════════════════════════════════════════════════════ */
  var _reconnectAttempts = 0;
  var _reconnectGivenUp = false;
  var RECONNECT_MAX = 30;

  // Hook into existing reconnect logic by watching connection state
  setInterval(function() {
    try {
      var connEl = document.getElementById('connT');
      if (!connEl) return;
      var txt = connEl.textContent || '';
      if (/connecting|retry/i.test(txt) && !_reconnectGivenUp) {
        _reconnectAttempts++;
        if (_reconnectAttempts >= RECONNECT_MAX) {
          _reconnectGivenUp = true;
          var banner = document.getElementById('rvtReconnectFailed');
          if (banner) banner.classList.add('show');
        }
      } else if (/connected|live|ok/i.test(txt)) {
        _reconnectAttempts = 0;
        if (_reconnectGivenUp) {
          _reconnectGivenUp = false;
          var b = document.getElementById('rvtReconnectFailed');
          if (b) b.classList.remove('show');
        }
      }
    } catch(_){}
  }, 2000);

  var _rvtReconnectRetryBtn = document.getElementById('rvtReconnectRetryBtn');
  if (_rvtReconnectRetryBtn) {
    _rvtReconnectRetryBtn.addEventListener('click', function() {
      _reconnectAttempts = 0;
      _reconnectGivenUp = false;
      var failed = document.getElementById('rvtReconnectFailed');
      if (failed) failed.classList.remove('show');
      if (typeof window.startConnection === 'function') {
        try { window.startConnection(); } catch(_){}
      } else if (typeof window.reconnect === 'function') {
        try { window.reconnect(); } catch(_){}
      }
    });
  }

  /* ═══════════════════════════════════════════════════════════
     A11Y-01: Vital-sign aria-live announcements
     ═══════════════════════════════════════════════════════════ */
  var ariaLive = document.createElement('div');
  ariaLive.id = 'rvtVitalLive';
  ariaLive.setAttribute('aria-live', 'assertive');
  ariaLive.setAttribute('aria-atomic', 'true');
  ariaLive.style.cssText = 'position:absolute;left:-10000px;width:1px;height:1px;overflow:hidden;';
  document.body.appendChild(ariaLive);

  var _lastBreachAnnouncement = { hr: 0, rr: 0 };
  function announceBreach(metric, value, thresh) {
    var now = Date.now();
    if (now - (_lastBreachAnnouncement[metric] || 0) < 10000) return;
    _lastBreachAnnouncement[metric] = now;
    var label = metric === 'hr' ? 'Heart rate' : 'Respiratory rate';
    var unit = metric === 'hr' ? '' : ' breaths per minute';
    ariaLive.textContent = label + ' critical: ' + value + unit + ', exceeded ' + thresh + ' threshold';
  }

  // Watch breach states on KPI cards
  setInterval(function() {
    try {
      var hrCard = document.getElementById('kpiHrCard') || document.querySelector('[data-kpi="hr"]');
      var rrCard = document.getElementById('kpiRrCard') || document.querySelector('[data-kpi="rr"]');
      [['hr', hrCard], ['rr', rrCard]].forEach(function(pair) {
        var el = pair[1];
        if (!el) return;
        if (el.classList.contains('breach') || el.classList.contains('crit') || el.getAttribute('data-state') === 'crit') {
          var valEl = el.querySelector('.kpi-value, .v, [data-role="value"]');
          var val = valEl ? valEl.textContent.trim() : '--';
          announceBreach(pair[0], val, '?');
        }
      });
    } catch(_){}
  }, 3000);

  /* ═══════════════════════════════════════════════════════════
     A11Y-02: Color-blind palette toggle
     ═══════════════════════════════════════════════════════════ */
  try {
    if (localStorage.getItem('rvt-cbpalette') === 'on') {
      document.body.setAttribute('data-rvt-cbpalette', 'on');
    }
  } catch(_){}
  window.toggleCBPalette = function() {
    var on = document.body.getAttribute('data-rvt-cbpalette') === 'on';
    if (on) {
      document.body.removeAttribute('data-rvt-cbpalette');
      try { localStorage.setItem('rvt-cbpalette', 'off'); } catch(_){}
    } else {
      document.body.setAttribute('data-rvt-cbpalette', 'on');
      try { localStorage.setItem('rvt-cbpalette', 'on'); } catch(_){}
    }
  };

  /* ═══════════════════════════════════════════════════════════
     HOME-01: Next Action Banner
     ═══════════════════════════════════════════════════════════ */
  function injectNextActionBanner() {
    var homeView = document.getElementById('view-home');
    if (!homeView) return null;
    var existing = homeView.querySelector('.rvt-next-action-banner');
    if (existing) return existing;

    var banner = document.createElement('div');
    banner.className = 'rvt-next-action-banner';
    banner.setAttribute('role', 'alert');
    banner.setAttribute('aria-live', 'polite');
    banner.setAttribute('tabindex', '0');
    banner.innerHTML =
      '<div class="rvt-next-action-icon"><span class="material-symbols-rounded">priority_high</span></div>' +
      '<div class="rvt-next-action-text">' +
        '<div class="rvt-next-action-title">Checking system…</div>' +
        '<div class="rvt-next-action-sub">Verifying preflight checks</div>' +
      '</div>' +
      '<button class="rvt-next-action-dismiss" type="button" aria-label="Dismiss"><span class="material-symbols-rounded">close</span></button>';

    homeView.insertBefore(banner, homeView.firstChild);

    banner.addEventListener('click', function(e) {
      if (e.target.closest('.rvt-next-action-dismiss')) {
        banner.classList.remove('show');
        try { sessionStorage.setItem('rvt-home-banner-dismissed', '1'); } catch(_){}
        return;
      }
      // Scroll to setup card
      var target = banner.getAttribute('data-target');
      if (target) {
        var t = document.querySelector(target);
        if (t) {
          t.scrollIntoView({ behavior: 'smooth', block: 'center' });
          t.classList.add('rvt-next-action-highlight');
          setTimeout(function() { t.classList.remove('rvt-next-action-highlight'); }, 1500);
        }
      }
    });

    return banner;
  }

  function syncNextActionBanner() {
    if (document.body.getAttribute('data-view') !== 'home') return;
    if (sessionStorage.getItem('rvt-home-banner-dismissed') === '1') return;

    var banner = injectNextActionBanner();
    if (!banner) return;

    var connEl = document.getElementById('connT');
    var connTxt = connEl ? connEl.textContent.trim() : '';

    var failChecks = document.querySelectorAll('#view-home .pf-item.fail, #view-home [data-state="fail"]');
    var warnChecks = document.querySelectorAll('#view-home .pf-item.warn, #view-home [data-state="warn"]');

    var title, sub, target, isWarn = false;
    if (/connecting|lost|error|disconnect/i.test(connTxt)) {
      title = 'Connect device — radar not responding';
      sub = 'Check serial cable, verify port, then re-run preflight';
      target = '.setup-card, #setupCard, [data-section="setup"]';
    } else if (failChecks.length > 0) {
      title = 'Fix failing preflight check' + (failChecks.length > 1 ? 's' : '');
      sub = failChecks.length + ' check' + (failChecks.length > 1 ? 's' : '') + ' blocking session start';
      target = '#view-home .pf-item.fail';
    } else if (warnChecks.length > 0) {
      title = 'Review preflight warning' + (warnChecks.length > 1 ? 's' : '');
      sub = warnChecks.length + ' check' + (warnChecks.length > 1 ? 's' : '') + ' need attention but not blocking';
      target = '#view-home .pf-item.warn';
      isWarn = true;
    } else {
      banner.classList.remove('show');
      return;
    }

    banner.querySelector('.rvt-next-action-title').textContent = title;
    banner.querySelector('.rvt-next-action-sub').textContent = sub;
    if (target) banner.setAttribute('data-target', target);
    banner.classList.toggle('warn-only', isWarn);
    banner.classList.add('show');
  }

  setInterval(syncNextActionBanner, 4000);
  setTimeout(syncNextActionBanner, 1500);

  /* ═══════════════════════════════════════════════════════════
     LIVE-01: Simple/Advanced toggle
     ═══════════════════════════════════════════════════════════ */
  function injectLiveModeToggle() {
    var liveView = document.getElementById('view-live');
    if (!liveView) return;
    if (liveView.querySelector('.rvt-live-mode-toggle')) return;

    // Find tab strip in live view
    var tabStrip = liveView.querySelector('.tabs, [role="tablist"], .live-tabs');
    if (!tabStrip) {
      tabStrip = liveView.querySelector('.cardhd, .panel-hd, .view-header');
    }
    if (!tabStrip) return;

    var saved = 'simple';
    try { saved = localStorage.getItem('rvt-live-mode') || 'simple'; } catch(_){}
    document.body.setAttribute('data-live-mode', saved);

    var toggle = document.createElement('div');
    toggle.className = 'rvt-live-mode-toggle';
    toggle.setAttribute('role', 'group');
    toggle.setAttribute('aria-label', 'Display mode');
    toggle.innerHTML =
      '<button type="button" data-mode="simple" title="Show only HR, RR, Frame Rate, Range">Simple</button>' +
      '<button type="button" data-mode="advanced" title="Show all metrics including PQI, mismatch, point clouds">Advanced</button>';

    tabStrip.appendChild(toggle);

    toggle.querySelectorAll('button').forEach(function(btn) {
      if (btn.getAttribute('data-mode') === saved) btn.classList.add('active');
      btn.addEventListener('click', function() {
        var mode = btn.getAttribute('data-mode');
        document.body.setAttribute('data-live-mode', mode);
        try { localStorage.setItem('rvt-live-mode', mode); } catch(_){}
        toggle.querySelectorAll('button').forEach(function(b) { b.classList.remove('active'); });
        btn.classList.add('active');
      });
    });

    // Tag advanced cards heuristically
    var advancedKeys = ['pqi', 'mismatch', 'pointcloud', 'arbiter', 'kalman', 'fusion', 'phase', 'harmonic'];
    liveView.querySelectorAll('.kpi, .panel, .card').forEach(function(el) {
      var txt = (el.textContent || '').toLowerCase();
      var id = (el.id || '').toLowerCase();
      if (advancedKeys.some(function(k) { return txt.includes(k) || id.includes(k); })) {
        el.setAttribute('data-advanced', 'true');
      }
    });
  }

  /* ═══════════════════════════════════════════════════════════
     LIVE-03: KPI Click-to-Zoom
     ═══════════════════════════════════════════════════════════ */
  var kpiZoomOverlay = document.getElementById('rvtKpiZoomOverlay');
  var kpiZoomChart = null;
  var kpiZoomData = [];
  var kpiZoomMetric = null;
  var kpiZoomRaf = null;
  var kpiZoomTimeout = null;

  function openKpiZoom(metric, label) {
    if (!kpiZoomOverlay) return;
    kpiZoomMetric = metric;
    var title = document.getElementById('rvtKpiZoomTitle');
    if (title) title.textContent = label;
    kpiZoomOverlay.classList.add('show');
    kpiZoomData = [];
    drawKpiZoom();
    if (kpiZoomRaf) cancelAnimationFrame(kpiZoomRaf);
    if (kpiZoomTimeout) clearTimeout(kpiZoomTimeout);
    kpiZoomRaf = null;
    kpiZoomTimeout = null;
    tickKpiZoom();
  }

  function closeKpiZoom() {
    if (kpiZoomOverlay) kpiZoomOverlay.classList.remove('show');
    kpiZoomMetric = null;
    if (kpiZoomRaf) cancelAnimationFrame(kpiZoomRaf);
    if (kpiZoomTimeout) clearTimeout(kpiZoomTimeout);
    kpiZoomRaf = null;
    kpiZoomTimeout = null;
  }

  function getCurrentKpiValue(metric) {
    var id = metric === 'hr' ? 'kpiHr' : metric === 'rr' ? 'kpiRr' : null;
    if (!id) return null;
    var el = document.getElementById(id);
    if (!el) return null;
    var v = parseFloat(el.textContent);
    return isFinite(v) ? v : null;
  }

  function tickKpiZoom() {
    if (!kpiZoomMetric) return;
    var v = getCurrentKpiValue(kpiZoomMetric);
    if (v != null) {
      kpiZoomData.push({ t: Date.now(), v: v });
      // keep last 5 min
      var cutoff = Date.now() - 5 * 60 * 1000;
      while (kpiZoomData.length > 0 && kpiZoomData[0].t < cutoff) kpiZoomData.shift();
      drawKpiZoom();
      document.getElementById('rvtKpiZoomValue').textContent = v.toFixed(0);
      var sum = 0; kpiZoomData.forEach(function(d) { sum += d.v; });
      var mean = kpiZoomData.length ? sum / kpiZoomData.length : 0;
      var sq = 0; kpiZoomData.forEach(function(d) { sq += (d.v - mean) ** 2; });
      var std = kpiZoomData.length ? Math.sqrt(sq / kpiZoomData.length) : 0;
      document.getElementById('rvtKpiZoomMean').textContent = mean.toFixed(1);
      document.getElementById('rvtKpiZoomStd').textContent = '±' + std.toFixed(2);
      document.getElementById('rvtKpiZoomThresh').textContent = kpiZoomMetric === 'hr' ? '50–140 bpm' : '8–30 br/min';
    }
    if (kpiZoomTimeout) clearTimeout(kpiZoomTimeout);
    kpiZoomTimeout = setTimeout(function() {
      kpiZoomTimeout = null;
      kpiZoomRaf = requestAnimationFrame(tickKpiZoom);
    }, 1000);
  }

  function drawKpiZoom() {
    var canvas = document.getElementById('rvtKpiZoomChart');
    if (!canvas) return;
    var dpr = window.devicePixelRatio || 1;
    var rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    var ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, rect.width, rect.height);

    if (kpiZoomData.length < 2) {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '12px Inter, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Collecting data…', rect.width / 2, rect.height / 2);
      return;
    }

    var pad = 16;
    var w = rect.width - pad * 2;
    var h = rect.height - pad * 2;
    var minV = Infinity, maxV = -Infinity;
    kpiZoomData.forEach(function(d) {
      if (d.v < minV) minV = d.v;
      if (d.v > maxV) maxV = d.v;
    });
    var range = Math.max(1, maxV - minV);
    minV -= range * 0.1;
    maxV += range * 0.1;
    range = maxV - minV;

    var t0 = kpiZoomData[0].t;
    var tN = kpiZoomData[kpiZoomData.length - 1].t;
    var tRange = Math.max(1, tN - t0);

    // Gridlines
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;
    for (var i = 0; i <= 4; i++) {
      var y = pad + (h * i / 4);
      ctx.beginPath();
      ctx.moveTo(pad, y);
      ctx.lineTo(rect.width - pad, y);
      ctx.stroke();
    }

    // Line
    ctx.strokeStyle = '#2563eb';
    ctx.lineWidth = 2;
    ctx.beginPath();
    kpiZoomData.forEach(function(d, idx) {
      var x = pad + ((d.t - t0) / tRange) * w;
      var y = pad + h - ((d.v - minV) / range) * h;
      if (idx === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    // Axis labels
    ctx.fillStyle = '#94a3b8';
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.textAlign = 'left';
    ctx.fillText(maxV.toFixed(0), 4, pad + 4);
    ctx.fillText(minV.toFixed(0), 4, rect.height - pad);
  }

  var kpiZoomClose = document.getElementById('rvtKpiZoomClose');
  if (kpiZoomClose) kpiZoomClose.addEventListener('click', closeKpiZoom);
  if (kpiZoomOverlay) {
    kpiZoomOverlay.addEventListener('click', function(e) {
      if (e.target === kpiZoomOverlay) closeKpiZoom();
    });
  }
  document.addEventListener('keydown', function(e) {
    if (e.key === 'Escape' && kpiZoomOverlay && kpiZoomOverlay.classList.contains('show')) closeKpiZoom();
  });

  function wireKpiZoom() {
    document.querySelectorAll('#kpiHr, [data-kpi="hr"] .kpi-value, [data-kpi="hr"]').forEach(function(el) {
      if (el.__rvtZoomWired) return;
      el.__rvtZoomWired = true;
      el.style.cursor = 'zoom-in';
      el.addEventListener('dblclick', function() { openKpiZoom('hr', 'Heart Rate'); });
    });
    document.querySelectorAll('#kpiRr, [data-kpi="rr"] .kpi-value, [data-kpi="rr"]').forEach(function(el) {
      if (el.__rvtZoomWired) return;
      el.__rvtZoomWired = true;
      el.style.cursor = 'zoom-in';
      el.addEventListener('dblclick', function() { openKpiZoom('rr', 'Respiratory Rate'); });
    });
  }

  /* ═══════════════════════════════════════════════════════════
     ALERT-01/03: Filter chips + bulk ack
     ═══════════════════════════════════════════════════════════ */
  function injectAlertFilters() {
    var drawer = document.getElementById('dv-ov') || document.querySelector('.alerts-drawer, [data-drawer="alerts"]');
    if (!drawer) return;
    if (drawer.querySelector('.rvt-alert-filter-row')) return;

    var row = document.createElement('div');
    row.className = 'rvt-alert-filter-row';
    row.innerHTML =
      '<button class="rvt-alert-filter-chip crit" data-filter="crit" type="button" aria-pressed="false">' +
        '<span class="rvt-sev-pattern crit"></span>Critical</button>' +
      '<button class="rvt-alert-filter-chip warn" data-filter="warn" type="button" aria-pressed="false">' +
        '<span class="rvt-sev-pattern warn"></span>Warn</button>' +
      '<button class="rvt-alert-filter-chip info" data-filter="info" type="button" aria-pressed="false">' +
        '<span class="rvt-sev-pattern info"></span>Info</button>' +
      '<button class="rvt-alert-filter-chip" data-filter="unack" type="button" aria-pressed="false">Unack only</button>' +
      '<button class="rvt-alert-bulk-ack" type="button" id="rvtBulkAck">' +
        '<span class="material-symbols-rounded" style="font-size:14px;">done_all</span>Ack all</button>';

    var content = drawer.querySelector('.dv-content, .drawer-content, .alerts-list');
    if (content) content.parentNode.insertBefore(row, content);
    else drawer.insertBefore(row, drawer.firstChild);

    var activeFilters = new Set();
    try {
      var saved = JSON.parse(localStorage.getItem('rvt-alerts-filter') || '[]');
      saved.forEach(function(f) { activeFilters.add(f); });
    } catch(_){}

    function applyFilters() {
      drawer.querySelectorAll('.alert-item, [data-alert]').forEach(function(item) {
        var sev = (item.getAttribute('data-severity') || item.getAttribute('data-sev') || '').toLowerCase();
        var ack = item.getAttribute('data-acked') === 'true' || item.classList.contains('acked');
        var visible = true;
        if (activeFilters.size > 0) {
          var sevFilters = ['crit','warn','info'].filter(function(f) { return activeFilters.has(f); });
          if (sevFilters.length > 0 && !sevFilters.includes(sev)) visible = false;
          if (activeFilters.has('unack') && ack) visible = false;
        }
        item.style.display = visible ? '' : 'none';
      });
    }

    row.querySelectorAll('.rvt-alert-filter-chip').forEach(function(chip) {
      var f = chip.getAttribute('data-filter');
      if (activeFilters.has(f)) chip.setAttribute('aria-pressed', 'true');
      chip.addEventListener('click', function() {
        if (activeFilters.has(f)) { activeFilters.delete(f); chip.setAttribute('aria-pressed', 'false'); }
        else { activeFilters.add(f); chip.setAttribute('aria-pressed', 'true'); }
        try { localStorage.setItem('rvt-alerts-filter', JSON.stringify(Array.from(activeFilters))); } catch(_){}
        applyFilters();
      });
    });

    document.getElementById('rvtBulkAck').addEventListener('click', function() {
      var visible = drawer.querySelectorAll('.alert-item:not([style*="display: none"]), [data-alert]:not([style*="display: none"])');
      var unacked = Array.from(visible).filter(function(el) {
        return el.getAttribute('data-acked') !== 'true' && !el.classList.contains('acked');
      });
      if (unacked.length === 0) return;
      if (unacked.length > 10 && !confirm('Acknowledge ' + unacked.length + ' alerts?')) return;
      unacked.forEach(function(el) {
        el.setAttribute('data-acked', 'true');
        el.classList.add('acked');
        var btn = el.querySelector('[data-action="ack"], .alert-ack-btn');
        if (btn) btn.click();
      });
    });

    setInterval(applyFilters, 1500);
  }

  /* ═══════════════════════════════════════════════════════════
     HELP-01: Sticky search + HELP-02: Recovery checklist persistence
     ═══════════════════════════════════════════════════════════ */
  function injectHelpSearch() {
    var helpView = document.getElementById('view-help');
    if (!helpView) return;
    if (helpView.querySelector('.rvt-help-search')) return;

    var search = document.createElement('div');
    search.className = 'rvt-help-search';
    search.innerHTML =
      '<span class="material-symbols-rounded">search</span>' +
      '<input type="search" placeholder="Search help, glossary, FAQ…" aria-label="Search help">';

    var firstChild = helpView.firstChild;
    helpView.insertBefore(search, firstChild);

    var input = search.querySelector('input');
    input.addEventListener('input', function() {
      var q = input.value.trim().toLowerCase();
      var entries = helpView.querySelectorAll('.faq-item, .help-entry, .glossary-entry, h2, h3, p, li');
      // Reset
      entries.forEach(function(el) {
        // restore any previous mark
        if (el.__origHtml) { el.innerHTML = el.__origHtml; el.__origHtml = null; }
        el.classList.remove('rvt-help-no-match');
      });
      if (!q) return;

      // Find sections whose text contains q
      var sections = helpView.querySelectorAll('.help-section, section, .card');
      sections.forEach(function(sec) {
        var t = (sec.textContent || '').toLowerCase();
        if (!t.includes(q)) sec.classList.add('rvt-help-no-match');
      });

      // Highlight matches in text-bearing elements
      var re = new RegExp('(' + q.replace(/[-/\\^$*+?.()|[\]{}]/g, '\\$&') + ')', 'gi');
      helpView.querySelectorAll('p, li, h2, h3, .faq-q, .faq-a').forEach(function(el) {
        var txt = el.textContent || '';
        if (txt.toLowerCase().includes(q)) {
          el.__origHtml = el.innerHTML;
          el.innerHTML = el.innerHTML.replace(re, '<mark class="rvt-help-mark">$1</mark>');
        }
      });
    });
  }

  function injectRecoveryChecklist() {
    var helpView = document.getElementById('view-help');
    if (!helpView) return;
    var lists = helpView.querySelectorAll('.recovery-checklist, [data-recovery-list]');
    if (lists.length === 0) {
      // Try to find any UL/OL inside a section labelled "recovery"
      var headings = helpView.querySelectorAll('h2, h3');
      headings.forEach(function(h) {
        if (/recovery|troubleshoot|checklist/i.test(h.textContent || '')) {
          var next = h.nextElementSibling;
          while (next) {
            if (next.tagName === 'UL' || next.tagName === 'OL') {
              next.classList.add('recovery-checklist');
              break;
            }
            next = next.nextElementSibling;
          }
        }
      });
      lists = helpView.querySelectorAll('.recovery-checklist');
    }

    lists.forEach(function(list, listIdx) {
      if (list.__rvtChecklisted) return;
      list.__rvtChecklisted = true;
      var listKey = 'rvt-recovery-progress-' + listIdx;
      var saved = {};
      try { saved = JSON.parse(localStorage.getItem(listKey) || '{}'); } catch(_){}

      var items = list.querySelectorAll('li');
      var progressBar = document.createElement('div');
      progressBar.className = 'rvt-recovery-progress';
      progressBar.innerHTML =
        '<span style="font-weight:700;">Progress:</span>' +
        '<div class="rvt-recovery-progress-bar"><div></div></div>' +
        '<span class="rvt-recovery-progress-pct">0%</span>' +
        '<button type="button" style="border:1px solid var(--line);background:var(--surface-1);padding:2px 8px;border-radius:6px;cursor:pointer;font-size:11px;">Reset</button>';
      list.parentNode.insertBefore(progressBar, list);

      function updateProgress() {
        var done = 0;
        items.forEach(function(li, i) {
          if (saved[i]) done++;
        });
        var pct = items.length ? Math.round((done / items.length) * 100) : 0;
        progressBar.querySelector('.rvt-recovery-progress-bar > div').style.width = pct + '%';
        progressBar.querySelector('.rvt-recovery-progress-pct').textContent = pct + '%';
      }

      items.forEach(function(li, i) {
        var cb = document.createElement('input');
        cb.type = 'checkbox';
        cb.style.cssText = 'margin-right:8px;cursor:pointer;width:16px;height:16px;accent-color:var(--ok-500);';
        cb.checked = !!saved[i];
        if (cb.checked) li.style.opacity = '0.6';
        li.insertBefore(cb, li.firstChild);
        cb.addEventListener('change', function() {
          saved[i] = cb.checked;
          try { localStorage.setItem(listKey, JSON.stringify(saved)); } catch(_){}
          li.style.opacity = cb.checked ? '0.6' : '';
          updateProgress();
        });
      });

      progressBar.querySelector('button').addEventListener('click', function() {
        if (!confirm('Reset checklist?')) return;
        saved = {};
        try { localStorage.removeItem(listKey); } catch(_){}
        items.forEach(function(li, i) {
          var cb = li.querySelector('input[type="checkbox"]');
          if (cb) cb.checked = false;
          li.style.opacity = '';
        });
        updateProgress();
      });

      updateProgress();
    });
  }

  /* ═══════════════════════════════════════════════════════════
     SET-01: Settings search + SET-04: Test alert buttons + SET-07: Modified dot
     ═══════════════════════════════════════════════════════════ */
  function injectSettingsSearch() {
    var settingsView = document.getElementById('view-settings');
    if (!settingsView) return;
    if (settingsView.querySelector('.rvt-settings-search')) return;

    var search = document.createElement('div');
    search.className = 'rvt-settings-search';
    search.innerHTML =
      '<span class="material-symbols-rounded">search</span>' +
      '<input type="search" placeholder="Search settings… (waveform, voice, vibration)" aria-label="Search settings">';

    settingsView.insertBefore(search, settingsView.firstChild);

    var input = search.querySelector('input');
    input.addEventListener('input', function() {
      var q = input.value.trim().toLowerCase();
      var rows = settingsView.querySelectorAll('.set-r, .setting-row, [data-setting]');
      rows.forEach(function(row) {
        if (!q) { row.classList.remove('rvt-set-hidden'); return; }
        var t = (row.textContent || '').toLowerCase();
        row.classList.toggle('rvt-set-hidden', !t.includes(q));
      });
      // Hide section groups with no visible rows
      settingsView.querySelectorAll('.set-g, .settings-group').forEach(function(g) {
        var visibleRows = g.querySelectorAll('.set-r:not(.rvt-set-hidden), .setting-row:not(.rvt-set-hidden)');
        g.classList.toggle('rvt-set-hidden', q && visibleRows.length === 0);
      });
    });
  }

  function injectTestAlertButtons() {
    var settingsView = document.getElementById('view-settings');
    if (!settingsView) return;

    var modalities = [
      { match: /visual|flash/i, label: 'Test flash', action: function() {
        document.body.style.transition = 'background 0.15s';
        var orig = document.body.style.background;
        document.body.style.background = 'rgba(220,38,38,0.18)';
        setTimeout(function() { document.body.style.background = orig; }, 250);
      }},
      { match: /haptic|vibrat/i, label: 'Test haptic', action: function() {
        if (navigator.vibrate) navigator.vibrate([100, 50, 100]);
      }},
      { match: /voice|speech/i, label: 'Test voice', action: function() {
        if (window.speechSynthesis) {
          var u = new SpeechSynthesisUtterance('Test alert: heart rate critical, 165');
          window.speechSynthesis.speak(u);
        }
      }}
    ];

    settingsView.querySelectorAll('.set-r, .setting-row').forEach(function(row) {
      if (row.querySelector('.rvt-test-alert-btn')) return;
      var label = (row.querySelector('strong, .setting-label, label') || row).textContent || '';
      modalities.forEach(function(m) {
        if (m.match.test(label) && !row.querySelector('.rvt-test-alert-btn')) {
          var btn = document.createElement('button');
          btn.type = 'button';
          btn.className = 'rvt-test-alert-btn rvt-pdf-export-btn';
          btn.style.cssText = 'padding:4px 10px;font-size:11px;margin-left:8px;';
          btn.innerHTML = '<span class="material-symbols-rounded" style="font-size:14px;">play_arrow</span>' + m.label;
          btn.addEventListener('click', m.action);
          var actionArea = row.querySelector('.set-action, .setting-action') || row;
          actionArea.appendChild(btn);
        }
      });
    });
  }

  /* ═══════════════════════════════════════════════════════════
     QOL-04: Last-saved indicator
     ═══════════════════════════════════════════════════════════ */
  var savedIndicators = new Map();
  function attachSavedIndicator(field) {
    if (!field || field.__rvtSavedAttached) return;
    field.__rvtSavedAttached = true;

    var indicator = document.createElement('span');
    indicator.className = 'rvt-saved-indicator';
    indicator.textContent = 'Saved';
    field.parentNode.appendChild(indicator);

    var lastSaved = Date.now();
    var saveTimer = null;
    savedIndicators.set(field, indicator);

    field.addEventListener('input', function() {
      indicator.classList.add('saving');
      indicator.classList.remove('error');
      indicator.textContent = 'Saving…';
      if (saveTimer) clearTimeout(saveTimer);
      saveTimer = setTimeout(function() {
        try {
          // Trigger existing save by dispatching change
          field.dispatchEvent(new Event('change', { bubbles: true }));
          lastSaved = Date.now();
          indicator.classList.remove('saving');
          indicator.textContent = 'Saved just now';
        } catch (e) {
          indicator.classList.add('error');
          indicator.textContent = 'Save failed';
        }
      }, 800);
    });

    setInterval(function() {
      if (indicator.classList.contains('saving') || indicator.classList.contains('error')) return;
      var ago = Math.floor((Date.now() - lastSaved) / 1000);
      if (ago < 5) indicator.textContent = 'Saved just now';
      else if (ago < 60) indicator.textContent = 'Saved ' + ago + 's ago';
      else indicator.textContent = 'Saved ' + Math.floor(ago / 60) + 'm ago';
    }, 5000);
  }

  function wireSavedIndicators() {
    var fields = document.querySelectorAll('#sessionNotes, textarea[data-autosave], textarea[name*="notes"]');
    fields.forEach(attachSavedIndicator);
  }

  /* ═══════════════════════════════════════════════════════════
     CMD-03: Command palette shortcut hints (best-effort)
     ═══════════════════════════════════════════════════════════ */
  function decorateCommandPalette() {
    var palette = document.querySelector('.cmd-palette, #cmdPalette, [data-palette]');
    if (!palette) return;
    palette.querySelectorAll('.cmd-item, [data-cmd]').forEach(function(item) {
      if (item.querySelector('.rvt-cmd-shortcut')) return;
      var sc = item.getAttribute('data-shortcut') || item.getAttribute('data-key');
      if (!sc) return;
      var chip = document.createElement('span');
      chip.className = 'rvt-cmd-shortcut';
      chip.style.cssText = 'margin-left:auto;padding:2px 6px;border-radius:4px;background:var(--surface-3);color:var(--ink-500);font-family:JetBrains Mono,monospace;font-size:10px;';
      var isMac = /Mac/i.test(navigator.platform);
      chip.textContent = sc.replace(/Ctrl/i, isMac ? '⌘' : 'Ctrl').replace(/Alt/i, isMac ? '⌥' : 'Alt').replace(/\+/g, isMac ? '' : '+');
      item.appendChild(chip);
    });
  }

  /* ═══════════════════════════════════════════════════════════
     Master orchestration
     ═══════════════════════════════════════════════════════════ */
  function v15_1Sync() {
    var view = document.body.getAttribute('data-view');
    if (view === 'home') {
      syncNextActionBanner();
    }
    if (view === 'live') {
      injectLiveModeToggle();
      wireKpiZoom();
    }
    if (view === 'help') {
      injectHelpSearch();
      injectRecoveryChecklist();
    }
    if (view === 'settings') {
      injectSettingsSearch();
      injectTestAlertButtons();
    }
    injectAlertFilters();
    wireSavedIndicators();
    decorateCommandPalette();
  }

  function v15_1Boot() {
    setInterval(v15_1Sync, 2500);
    setTimeout(v15_1Sync, 1500);

    // Hook view changes
    var bodyObs = window.rvtTrackMutationObserver(new MutationObserver(function() { setTimeout(v15_1Sync, 200); }));
    bodyObs.observe(document.body, { attributes: true, attributeFilter: ['data-view'] });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', v15_1Boot, { once: true });
  } else {
    setTimeout(v15_1Boot, 800);
  }

})();
;
/* END modules/_anon-002.js */

/* BEGIN modules/_anon-003.js */
(function(){
  'use strict';

  /* ═══════════════════════════════════════════════════════
     PERF-01: RAF-batched DOM sync consolidator
     Merges the 5+ redundant setInterval sync loops into one
     ═══════════════════════════════════════════════════════ */
  var _syncRaf = 0;
  var _syncJobs = [];
  function registerSync(fn) { _syncJobs.push(fn); }
  function tickSync() {
    _syncRaf = 0;
    var view = document.body.getAttribute('data-view') || 'home';
    for (var i = 0; i < _syncJobs.length; i++) {
      try { _syncJobs[i](view); } catch(e) { console.warn('[v15.2 sync]', e); }
    }
  }
  function scheduleSync() {
    if (!_syncRaf) _syncRaf = requestAnimationFrame(tickSync);
  }
  // Single interval instead of many
  setInterval(scheduleSync, 2000);

  /* ═══════════════════════════════════════════════════════
     PERF-02: Consolidated MutationObserver
     Replace per-feature body observers with one shared observer
     ═══════════════════════════════════════════════════════ */
  var _mutationCallbacks = [];
  function onBodyMutation(fn) { _mutationCallbacks.push(fn); }
  var _sharedBodyObs = new MutationObserver(function(muts) {
    for (var i = 0; i < _mutationCallbacks.length; i++) {
      try { _mutationCallbacks[i](muts); } catch(err) { if (window.rvtSoftError) window.rvtSoftError('body mutation callback failed', err); }
    }
    scheduleSync();
  });
  _sharedBodyObs.observe(document.body, {
    attributes: true,
    attributeFilter: ['data-view', 'data-ctl', 'class', 'data-theme', 'data-density']
  });
  // Track for BUG-01 disconnect
  window.S = window.S || {};
  if (!window.S.__mutationObservers) window.S.__mutationObservers = [];
  window.S.__mutationObservers.push(_sharedBodyObs);

  /* ═══════════════════════════════════════════════════════
     BUG-03: Start Session Race Guard
     ═══════════════════════════════════════════════════════ */
  var _preflightReady = false;
  var _defaultsReady = false;
  var _sessionsReady = false;

  function guardStartButton() {
    var btn = document.querySelector('.home-start-btn, .setup-start, [onclick*="startSession"], #startSessionBtn');
    if (!btn) return;
    var allReady = _preflightReady && _defaultsReady && _sessionsReady;
    if (!allReady && !btn.disabled) {
      btn.disabled = true;
    }
    // Update disable reason
    var reason = btn.querySelector('.rvt-disable-reason');
    if (btn.disabled && !allReady) {
      var msg = [];
      if (!_preflightReady) msg.push('preflight');
      if (!_defaultsReady) msg.push('defaults');
      if (!_sessionsReady) msg.push('sessions');
      var txt = 'Waiting: ' + msg.join(', ');
      if (reason) reason.textContent = txt;
      btn.setAttribute('aria-describedby', 'rvtStartReason');
    }
  }

  // Hook async completions
  var _origLoadDefaults = window.loadControlDefaults;
  if (typeof _origLoadDefaults === 'function') {
    window.loadControlDefaults = function() {
      var result = _origLoadDefaults.apply(this, arguments);
      if (result && typeof result.then === 'function') {
        result.then(function() { _defaultsReady = true; guardStartButton(); })
          .catch(function(err) {
            console.warn('[RVT boot] defaults fetch failed:', err);
            if (window.rvtSoftError) window.rvtSoftError('defaults boot gate failed open', err, { surface: true, severity: 'warn' });
            _defaultsReady = true;
            guardStartButton();
          });
      } else { _defaultsReady = true; }
      return result;
    };
  } else { _defaultsReady = true; }

  var _origLoadSessions = window.loadSessionsList;
  if (typeof _origLoadSessions === 'function') {
    window.loadSessionsList = function() {
      var result = _origLoadSessions.apply(this, arguments);
      if (result && typeof result.then === 'function') {
        result.then(function() { _sessionsReady = true; guardStartButton(); })
          .catch(function(err) {
            console.warn('[RVT boot] sessions fetch failed:', err);
            if (window.rvtSoftError) window.rvtSoftError('sessions boot gate failed open', err, { surface: true, severity: 'warn' });
            _sessionsReady = true;
            guardStartButton();
          });
      } else { _sessionsReady = true; }
      return result;
    };
  } else { _sessionsReady = true; }

  var _origRunPreflight = window.runPreflightBatch;
  if (typeof _origRunPreflight === 'function') {
    window.runPreflightBatch = function() {
      var result = _origRunPreflight.apply(this, arguments);
      if (result && typeof result.then === 'function') {
        result.then(function() { _preflightReady = true; guardStartButton(); })
          .catch(function(err) {
            console.warn('[RVT boot] preflight fetch failed:', err);
            if (window.rvtSoftError) window.rvtSoftError('preflight boot gate failed open', err, { surface: true, severity: 'warn' });
            _preflightReady = true;
            guardStartButton();
          });
      } else { _preflightReady = true; }
      return result;
    };
  } else { _preflightReady = true; }

  registerSync(guardStartButton);

  /* ═══════════════════════════════════════════════════════
     BUG-04: Replace silent catches with logged warnings
     (Patch safeStorageJson and apiJson)
     ═══════════════════════════════════════════════════════ */
  if (typeof window.safeStorageJson === 'function') {
    var _origSSJ = window.safeStorageJson;
    window.safeStorageJson = function(key, fallback, validator) {
      try {
        return _origSSJ.apply(this, arguments);
      } catch(e) {
        console.warn('[RVT] safeStorageJson error for key:', key, e);
        return fallback;
      }
    };
  }

  /* ═══════════════════════════════════════════════════════
     BUG-05: Schema-validate live payloads
     ═══════════════════════════════════════════════════════ */
  var _invalidPayloadCount = 0;
  var _origNormalize = window.normalizeLivePayload;
  if (typeof _origNormalize === 'function') {
    window.normalizeLivePayload = function(raw) {
      try {
        if (raw && typeof raw.payload === 'string') {
          console.warn('[RVT] Unexpected string payload, attempting parse');
          try { raw.payload = JSON.parse(raw.payload); } catch(_) {
            _invalidPayloadCount++;
            return {};
          }
        }
        return _origNormalize.apply(this, arguments);
      } catch(e) {
        _invalidPayloadCount++;
        console.warn('[RVT] normalizeLivePayload rejected malformed payload:', e);
        return {};
      }
    };
  }

  /* ═══════════════════════════════════════════════════════
     HOME-02: Quickcheck Pipeline Visualization
     ═══════════════════════════════════════════════════════ */
  var PIPELINE_STAGES = [
    { id: 'python', label: 'Python', icon: 'terminal' },
    { id: 'firmware', label: 'Firmware', icon: 'memory' },
    { id: 'serial', label: 'Serial', icon: 'usb' },
    { id: 'ble', label: 'BLE', icon: 'bluetooth' }
  ];

  function injectPipeline() {
    var homeView = document.getElementById('view-home');
    if (!homeView) return;
    if (homeView.querySelector('.rvt-pipeline')) return;

    var pfCard = homeView.querySelector('.pf-card, [data-section="quickcheck"], .quickcheck-card');
    if (!pfCard) return;

    var pipeline = document.createElement('div');
    pipeline.className = 'rvt-pipeline';

    PIPELINE_STAGES.forEach(function(stage, idx) {
      if (idx > 0) {
        var bridge = document.createElement('div');
        bridge.className = 'rvt-pipeline-bridge';
        bridge.id = 'rvtBridge' + idx;
        pipeline.appendChild(bridge);
      }
      var node = document.createElement('div');
      node.className = 'rvt-pipeline-node';
      node.setAttribute('tabindex', '0');
      node.setAttribute('role', 'button');
      node.setAttribute('data-stage', stage.id);
      node.setAttribute('data-status', 'pending');
      node.innerHTML =
        '<span class="material-symbols-rounded">' + stage.icon + '</span>' +
        '<span>' + stage.label + '</span>' +
        '<span class="rvt-pipeline-ts" id="rvtPipeTs' + idx + '"></span>';
      node.addEventListener('keydown', function(e) {
        if (e.key === 'Enter') { /* re-run single check */ }
      });
      pipeline.appendChild(node);
    });

    // Insert before the existing pf-items
    var pfItems = pfCard.querySelector('.pf-items, .pf-grid, ul');
    if (pfItems) pfCard.insertBefore(pipeline, pfItems);
    else pfCard.appendChild(pipeline);

    // Re-run failed button
    var rerunBtn = document.createElement('button');
    rerunBtn.className = 'rvt-rerun-failed-btn';
    rerunBtn.type = 'button';
    rerunBtn.innerHTML = '<span class="material-symbols-rounded">refresh</span>Re-run failed only';
    rerunBtn.addEventListener('click', function() {
      if (typeof window.runPreflightBatch === 'function') {
        rerunBtn.disabled = true;
        rerunBtn.querySelector('.material-symbols-rounded').textContent = 'progress_activity';
        window.runPreflightBatch().then(function() {
          rerunBtn.disabled = false;
          rerunBtn.querySelector('.material-symbols-rounded').textContent = 'refresh';
        }).catch(function() {
          rerunBtn.disabled = false;
          rerunBtn.querySelector('.material-symbols-rounded').textContent = 'refresh';
        });
      }
    });
    pfCard.appendChild(rerunBtn);
  }

  function syncPipeline(view) {
    if (view !== 'home') return;
    var pipeline = document.querySelector('.rvt-pipeline');
    if (!pipeline) return;

    var pfItems = document.querySelectorAll('.pf-item, [data-check]');
    var statusMap = {};
    pfItems.forEach(function(el) {
      var id = (el.id || el.getAttribute('data-check') || '').toLowerCase();
      var cls = el.className || '';
      var status = 'pending';
      if (cls.includes('pass') || cls.includes('ok')) status = 'pass';
      else if (cls.includes('fail') || cls.includes('error')) status = 'fail';
      else if (cls.includes('warn')) status = 'warn';
      else if (cls.includes('running') || cls.includes('checking')) status = 'running';
      PIPELINE_STAGES.forEach(function(stage) {
        if (id.includes(stage.id)) statusMap[stage.id] = status;
      });
    });

    pipeline.querySelectorAll('.rvt-pipeline-node').forEach(function(node) {
      var stageId = node.getAttribute('data-stage');
      var s = statusMap[stageId] || 'pending';
      node.setAttribute('data-status', s);
      var icon = node.querySelector('.material-symbols-rounded');
      if (s === 'pass') icon.textContent = 'check_circle';
      else if (s === 'fail') icon.textContent = 'error';
      else if (s === 'warn') icon.textContent = 'warning';
      else if (s === 'running') icon.textContent = 'progress_activity';
      else {
        var stage = PIPELINE_STAGES.find(function(st) { return st.id === stageId; });
        if (stage) icon.textContent = stage.icon;
      }
    });

    // Bridges
    var allPass = true;
    var hasWarn = false;
    pipeline.querySelectorAll('.rvt-pipeline-bridge').forEach(function(b, i) {
      var prev = pipeline.querySelectorAll('.rvt-pipeline-node')[i];
      if (prev && prev.getAttribute('data-status') === 'pass') {
        b.classList.add('filled');
        b.classList.remove('warn-filled');
      } else if (prev && prev.getAttribute('data-status') === 'warn') {
        b.classList.add('warn-filled');
        b.classList.remove('filled');
      } else {
        b.classList.remove('filled', 'warn-filled');
      }
    });

    // Re-run failed button visibility
    var failCount = document.querySelectorAll('.pf-item.fail, .pf-item.warn, [data-check][data-status="fail"], [data-check][data-status="warn"]').length;
    var rerunBtn = document.querySelector('.rvt-rerun-failed-btn');
    if (rerunBtn) rerunBtn.classList.toggle('show', failCount > 0);
  }

  registerSync(syncPipeline);

  /* ═══════════════════════════════════════════════════════
     HOME-06: Copy Session Brief
     ═══════════════════════════════════════════════════════ */
  function injectCopyBrief() {
    var homeView = document.getElementById('view-home');
    if (!homeView) return;
    if (homeView.querySelector('.rvt-copy-brief-btn')) return;

    var confCard = homeView.querySelector('.session-confidence, .ready-card, [data-section="confidence"]');
    if (!confCard) return;

    var btn = document.createElement('button');
    btn.className = 'rvt-copy-brief-btn';
    btn.type = 'button';
    btn.innerHTML = '<span class="material-symbols-rounded">content_copy</span>Copy brief';
    btn.addEventListener('click', function() {
      var op = (window.S && window.S.operatorName) || 'Unknown';
      var subj = (window.S && window.S.subjectId) || '--';
      var port = (window.S && window.S.port) || '--';
      var profile = (window.S && window.S.profileName) || '--';
      var readiness = '--';
      var ringEl = confCard.querySelector('.ready-pct, .ring-value');
      if (ringEl) readiness = ringEl.textContent.trim();
      var date = new Date().toISOString().split('T')[0];
      var brief = date + ' · Op: ' + op + ' · Subject: ' + subj + ' · Port: ' + port + ' · Readiness: ' + readiness + ' · Profile: ' + profile;
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(brief).then(function() {
          if (typeof window.showToast === 'function') window.showToast('Session brief copied', 'ok');
        });
      } else {
        prompt('Copy this session brief:', brief);
      }
    });
    confCard.appendChild(btn);
  }

  /* ═══════════════════════════════════════════════════════
     LIVE-05: Quick-Tag Keyboard Shortcuts
     ═══════════════════════════════════════════════════════ */
  var TAG_MAP = { m: 'motion', c: 'cough', s: 'speaking', b: 'baseline' };
  var tagToast = document.getElementById('rvtTagToast');
  var _tagToastTimer = null;

  document.addEventListener('keydown', function(e) {
    if (document.body.getAttribute('data-view') !== 'live') return;
    var active = document.activeElement;
    if (active && (active.tagName === 'TEXTAREA' || active.tagName === 'INPUT' || active.isContentEditable)) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;

    var tag = TAG_MAP[e.key.toLowerCase()];
    if (!tag) return;

    e.preventDefault();
    var now = new Date();
    var ts = [now.getHours(), now.getMinutes(), now.getSeconds()].map(function(n) { return (n<10?'0':'') + n; }).join(':');
    var entry = '[' + ts + '] ' + tag;

    var notes = document.getElementById('sessionNotes');
    if (notes) {
      notes.value = (notes.value ? notes.value + '\n' : '') + entry;
      notes.dispatchEvent(new Event('input', { bubbles: true }));
    }

    // Show toast
    if (tagToast) {
      tagToast.textContent = 'Tagged: ' + tag + ' @ ' + ts;
      tagToast.classList.add('show');
      clearTimeout(_tagToastTimer);
      _tagToastTimer = setTimeout(function() { tagToast.classList.remove('show'); }, 2500);
    }
  });

  /* ═══════════════════════════════════════════════════════
     LIVE-06: Snapshot FAB
     ═══════════════════════════════════════════════════════ */
  var snapFab = document.getElementById('rvtSnapFab');
  var snapBadge = document.getElementById('rvtSnapFabBadge');

  function doSnapshot() {
    if (typeof window.takeSnapshot === 'function') {
      window.takeSnapshot();
    } else if (typeof window.pinSnapshot === 'function') {
      window.pinSnapshot();
    }
    // Animate
    if (snapFab) { snapFab.style.transform = 'scale(0.85)'; setTimeout(function() { snapFab.style.transform = ''; }, 200); }
    syncSnapBadge();
  }

  function syncSnapBadge() {
    var count = (window.S && Array.isArray(window.S.snaps)) ? window.S.snaps.length : 0;
    if (!snapBadge) return;
    if (count > 0) {
      snapBadge.textContent = count;
      snapBadge.style.display = '';
    } else {
      snapBadge.style.display = 'none';
    }
  }

  if (snapFab) snapFab.addEventListener('click', doSnapshot);
  document.addEventListener('keydown', function(e) {
    if (e.key === 'p' || e.key === 'P') {
      if (document.body.getAttribute('data-view') !== 'live') return;
      var active = document.activeElement;
      if (active && (active.tagName === 'TEXTAREA' || active.tagName === 'INPUT')) return;
      if (e.ctrlKey || e.metaKey) return;
      e.preventDefault();
      doSnapshot();
    }
  });

  registerSync(function(view) { if (view === 'live') syncSnapBadge(); });

  /* ═══════════════════════════════════════════════════════
     RPT-02: Auto-generated Executive Summary
     ═══════════════════════════════════════════════════════ */
  function generateInsights() {
    var insights = [];
    try {
      if (!window.S || !window.S.lastReportPayload) return insights;
      var rpt = window.S.lastReportPayload;

      // HR drift
      if (rpt.hr_rmse != null && Number(rpt.hr_rmse) > 3) {
        insights.push('HR RMSE was ' + Number(rpt.hr_rmse).toFixed(1) + ' bpm — above the 3 bpm target threshold');
      }
      if (rpt.rr_rmse != null && Number(rpt.rr_rmse) > 2) {
        insights.push('RR RMSE was ' + Number(rpt.rr_rmse).toFixed(1) + ' br/min — above the 2 br/min target');
      }
      // Coverage
      if (rpt.coverage != null && Number(rpt.coverage) < 0.85) {
        insights.push('Coverage was ' + (Number(rpt.coverage) * 100).toFixed(0) + '% — below 85% target, likely due to subject motion or range issues');
      }
      // Correlation
      if (rpt.hr_r != null && Number(rpt.hr_r) < 0.9) {
        insights.push('HR correlation (r) was ' + Number(rpt.hr_r).toFixed(3) + ' — below 0.9 target');
      }
      // Settling
      if (rpt.settling_s != null && Number(rpt.settling_s) > 30) {
        insights.push('Settling took ' + Number(rpt.settling_s).toFixed(0) + 's — longer than typical 30s window');
      }

      if (insights.length === 0) {
        insights.push('All metrics within nominal ranges — session quality is good');
      }
    } catch(_) {}
    return insights.slice(0, 3);
  }

  function injectExecSummary() {
    var reportView = document.getElementById('view-report');
    if (!reportView) return;
    if (reportView.querySelector('.rvt-exec-summary')) return;

    var hero = reportView.querySelector('.verdict-hero');
    if (!hero) return;

    var summary = document.createElement('div');
    summary.className = 'rvt-exec-summary';
    summary.innerHTML = '<div class="rvt-exec-summary-title"><span class="material-symbols-rounded" style="font-size:18px;">auto_awesome</span>Executive Summary</div>';

    var insights = generateInsights();
    insights.forEach(function(text) {
      var row = document.createElement('div');
      row.className = 'rvt-exec-insight';
      row.innerHTML = '<span class="material-symbols-rounded">arrow_right</span><span>' + text + '</span>';
      summary.appendChild(row);
    });

    hero.parentNode.insertBefore(summary, hero.nextSibling);
  }

  /* ═══════════════════════════════════════════════════════
     RPT-07: Tint diagnostic cards by gate status
     ═══════════════════════════════════════════════════════ */
  function tintDiagnosticCards() {
    var reportView = document.getElementById('view-report');
    if (!reportView) return;
    reportView.querySelectorAll('.verdict-card, .diag-card, [data-gate]').forEach(function(card) {
      var badge = card.querySelector('.badge, .status-badge, [data-status]');
      if (!badge) return;
      var cls = (badge.className || '') + ' ' + (badge.getAttribute('data-status') || '');
      if (/pass|ok/i.test(cls)) card.setAttribute('data-gate-status', 'pass');
      else if (/fail|bad|crit/i.test(cls)) card.setAttribute('data-gate-status', 'fail');
      else if (/warn/i.test(cls)) card.setAttribute('data-gate-status', 'warn');
    });
  }

  /* ═══════════════════════════════════════════════════════
     SET-05: Settings Profiles
     ═══════════════════════════════════════════════════════ */
  var PROFILE_KEYS = ['rvt-thresholds', 'rvt-audio-vol', 'rvt-voice-alert', 'rvt-voice-readout', 'rvt-theme', 'rvt-density', 'rvt-kpi-order', 'rvt-hotkeys', 'rvt-live-mode'];
  var MAX_PROFILES = 10;

  function getProfiles() {
    try { return JSON.parse(localStorage.getItem('rvt-settings-profiles') || '{}'); } catch(_) { return {}; }
  }
  function saveProfiles(p) {
    try { localStorage.setItem('rvt-settings-profiles', JSON.stringify(p)); } catch(_) {}
  }

  function captureCurrentSettings() {
    var snap = {};
    PROFILE_KEYS.forEach(function(k) {
      try { snap[k] = localStorage.getItem(k); } catch(_) {}
    });
    return snap;
  }

  function applyProfile(name) {
    var profiles = getProfiles();
    var p = profiles[name];
    if (!p || !p.settings) return;
    Object.keys(p.settings).forEach(function(k) {
      try {
        if (p.settings[k] != null) localStorage.setItem(k, p.settings[k]);
        else localStorage.removeItem(k);
      } catch(_) {}
    });
    // Reload to apply
    location.reload();
  }

  function injectProfilesSection() {
    var settingsView = document.getElementById('view-settings');
    if (!settingsView) return;
    if (settingsView.querySelector('.rvt-profiles-section')) return;

    var section = document.createElement('div');
    section.className = 'rvt-profiles-section';
    section.innerHTML =
      '<div class="set-r"><div class="set-copy"><strong>Settings Profiles</strong>' +
      '<span>Save and load complete settings snapshots for different operators or study types.</span></div></div>' +
      '<div class="rvt-profiles-list" id="rvtProfilesList"></div>' +
      '<div style="display:flex;gap:8px;margin-top:4px;">' +
        '<input type="text" id="rvtProfileNameInput" placeholder="Profile name…" style="flex:1;padding:6px 10px;border:1px solid var(--line);border-radius:var(--r-sm);font-size:12px;font-family:Inter,sans-serif;background:var(--surface-1);color:var(--ink-700);">' +
        '<button type="button" id="rvtProfileSaveBtn" class="rvt-settings-io-btn"><span class="material-symbols-rounded">save</span>Save current</button>' +
      '</div>' +
      '<div class="rvt-settings-io-row">' +
        '<button type="button" class="rvt-settings-io-btn" id="rvtSettingsExport"><span class="material-symbols-rounded">download</span>Export JSON</button>' +
        '<button type="button" class="rvt-settings-io-btn" id="rvtSettingsImport"><span class="material-symbols-rounded">upload</span>Import JSON</button>' +
        '<input type="file" id="rvtSettingsImportFile" accept=".json" style="display:none">' +
      '</div>';

    // Find a good place to insert
    var lastGroup = settingsView.querySelectorAll('.set-g, .settings-group');
    if (lastGroup.length > 0) {
      lastGroup[lastGroup.length - 1].after(section);
    } else {
      settingsView.appendChild(section);
    }

    renderProfiles();

    document.getElementById('rvtProfileSaveBtn').addEventListener('click', function() {
      var name = document.getElementById('rvtProfileNameInput').value.trim();
      if (!name) { alert('Enter a profile name'); return; }
      var profiles = getProfiles();
      if (Object.keys(profiles).length >= MAX_PROFILES && !profiles[name]) {
        alert('Maximum ' + MAX_PROFILES + ' profiles. Delete one first.');
        return;
      }
      profiles[name] = { settings: captureCurrentSettings(), ts: Date.now() };
      saveProfiles(profiles);
      document.getElementById('rvtProfileNameInput').value = '';
      renderProfiles();
      if (typeof window.showToast === 'function') window.showToast('Profile "' + name + '" saved', 'ok');
    });

    // SET-06: Export/Import
    document.getElementById('rvtSettingsExport').addEventListener('click', function() {
      var data = { version: 1, ts: Date.now(), settings: captureCurrentSettings() };
      var blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url;
      a.download = 'rvt_settings_' + new Date().toISOString().split('T')[0] + '.json';
      a.click();
      URL.revokeObjectURL(url);
    });

    document.getElementById('rvtSettingsImport').addEventListener('click', function() {
      document.getElementById('rvtSettingsImportFile').click();
    });

    document.getElementById('rvtSettingsImportFile').addEventListener('change', function(e) {
      var file = e.target.files && e.target.files[0];
      if (!file) return;
      var reader = new FileReader();
      reader.onload = function() {
        try {
          var data = JSON.parse(reader.result);
          if (!data.settings) { alert('Invalid settings file'); return; }
          if (confirm('Import settings? This will overwrite your current configuration.')) {
            Object.keys(data.settings).forEach(function(k) {
              try {
                if (data.settings[k] != null) localStorage.setItem(k, data.settings[k]);
              } catch(_) {}
            });
            if (typeof window.showToast === 'function') window.showToast('Settings imported — reloading…', 'ok');
            setTimeout(function() { location.reload(); }, 1500);
          }
        } catch(err) { alert('Could not parse settings file: ' + err.message); }
      };
      reader.readAsText(file);
      e.target.value = '';
    });
  }

  function renderProfiles() {
    var list = document.getElementById('rvtProfilesList');
    if (!list) return;
    var profiles = getProfiles();
    list.innerHTML = '';
    Object.keys(profiles).forEach(function(name) {
      var chip = document.createElement('button');
      chip.className = 'rvt-profile-chip';
      chip.type = 'button';
      chip.innerHTML = name + '<span class="rvt-profile-del material-symbols-rounded">close</span>';
      chip.addEventListener('click', function(e) {
        if (e.target.classList.contains('rvt-profile-del')) {
          if (confirm('Delete profile "' + name + '"?')) {
            var p = getProfiles();
            delete p[name];
            saveProfiles(p);
            renderProfiles();
          }
          return;
        }
        applyProfile(name);
      });
      list.appendChild(chip);
    });
    if (Object.keys(profiles).length === 0) {
      list.innerHTML = '<span style="font-size:12px;color:var(--ink-400);font-style:italic;">No saved profiles</span>';
    }
  }

  /* ═══════════════════════════════════════════════════════
     NAV-01: Rail Collapse Toggle
     ═══════════════════════════════════════════════════════ */
  function injectRailCollapse() {
    var rail = document.querySelector('.rail');
    if (!rail || document.getElementById('railCollapseBtn') || rail.querySelector('.rvt-rail-collapse-btn')) return;

    var btn = document.createElement('button');
    btn.className = 'rvt-rail-collapse-btn';
    btn.type = 'button';
    btn.innerHTML = '<span class="material-symbols-rounded" style="font-size:18px;">chevron_left</span>';
    btn.title = 'Collapse rail (Alt+\\)';
    btn.addEventListener('click', function() {
      var collapsed = document.documentElement.getAttribute('data-rail-collapsed') === '1';
      document.documentElement.setAttribute('data-rail-collapsed', collapsed ? '0' : '1');
      try { localStorage.setItem('rvt-rail-collapsed', collapsed ? '0' : '1'); } catch(_) {}
      btn.querySelector('.material-symbols-rounded').textContent = collapsed ? 'chevron_left' : 'chevron_right';
    });
    rail.appendChild(btn);

    // Restore
    try {
      if (localStorage.getItem('rvt-rail-collapsed') === '1') {
        document.documentElement.setAttribute('data-rail-collapsed', '1');
        btn.querySelector('.material-symbols-rounded').textContent = 'chevron_right';
      }
    } catch(_) {}
  }

  document.addEventListener('keydown', function(e) {
    if (e.altKey && e.key === '\\') {
      var btn = document.getElementById('railCollapseBtn') || document.querySelector('.rvt-rail-collapse-btn');
      if (btn) btn.click();
    }
  });

  /* ═══════════════════════════════════════════════════════
     ALERT-02: Time-cluster Grouping
     ═══════════════════════════════════════════════════════ */
  function clusterAlerts() {
    var drawer = document.getElementById('dv-ov') || document.querySelector('.alerts-drawer');
    if (!drawer) return;
    if (drawer.querySelector('.rvt-alert-cluster')) return;

    var list = drawer.querySelector('.dv-content, .drawer-content, .alerts-list');
    if (!list) return;

    var items = list.querySelectorAll('.alert-item, [data-alert]');
    if (items.length === 0) return;

    var now = Date.now();
    var fiveMin = 5 * 60 * 1000;
    var clusters = { recent: [], earlier: [], previous: [] };

    items.forEach(function(item) {
      var ts = parseInt(item.getAttribute('data-ts') || item.getAttribute('data-timestamp') || '0', 10);
      if (!ts) ts = now;
      if (now - ts < fiveMin) clusters.recent.push(item);
      else if (item.getAttribute('data-session') === 'current' || !item.getAttribute('data-session')) clusters.earlier.push(item);
      else clusters.previous.push(item);
    });

    var labels = [
      ['recent', 'Last 5 min', clusters.recent],
      ['earlier', 'Earlier this session', clusters.earlier],
      ['previous', 'Previous sessions', clusters.previous]
    ];

    list.innerHTML = '';
    labels.forEach(function(entry) {
      if (entry[2].length === 0) return;
      var cluster = document.createElement('div');
      cluster.className = 'rvt-alert-cluster';
      cluster.innerHTML =
        '<div class="rvt-alert-cluster-header">' +
          '<span class="material-symbols-rounded">expand_more</span>' +
          '<span>' + entry[1] + '</span>' +
          '<span class="rvt-alert-cluster-count">' + entry[2].length + '</span>' +
        '</div>' +
        '<div class="rvt-alert-cluster-body"></div>';

      var body = cluster.querySelector('.rvt-alert-cluster-body');
      entry[2].forEach(function(item) { body.appendChild(item); });

      cluster.querySelector('.rvt-alert-cluster-header').addEventListener('click', function() {
        cluster.classList.toggle('collapsed');
      });

      // Collapse previous by default
      if (entry[0] === 'previous') cluster.classList.add('collapsed');

      list.appendChild(cluster);
    });
  }

  /* ═══════════════════════════════════════════════════════
     MOB-01: Tap-to-pin Tooltips
     ═══════════════════════════════════════════════════════ */
  var _activeTapTooltip = null;
  if ('ontouchstart' in window || navigator.maxTouchPoints > 0) {
    document.addEventListener('click', function(e) {
      // Dismiss existing
      if (_activeTapTooltip) {
        _activeTapTooltip.remove();
        _activeTapTooltip = null;
      }

      var target = e.target.closest('[title], [data-tooltip]');
      if (!target) return;

      var text = target.getAttribute('data-tooltip') || target.getAttribute('title');
      if (!text || text.length < 2) return;

      // Prevent default title
      if (target.hasAttribute('title')) {
        target.setAttribute('data-tooltip', text);
        target.removeAttribute('title');
      }

      e.preventDefault();
      var rect = target.getBoundingClientRect();
      var tip = document.createElement('div');
      tip.className = 'rvt-tap-tooltip';
      tip.textContent = text;
      tip.style.left = Math.max(8, Math.min(rect.left, window.innerWidth - 280)) + 'px';
      tip.style.top = (rect.bottom + 8) + 'px';
      document.body.appendChild(tip);
      _activeTapTooltip = tip;

      // Dismiss on next tap anywhere
      setTimeout(function() {
        document.addEventListener('click', function dismiss() {
          if (_activeTapTooltip) { _activeTapTooltip.remove(); _activeTapTooltip = null; }
          document.removeEventListener('click', dismiss);
        }, { once: true });
      }, 50);
    }, true);
  }

  /* ═══════════════════════════════════════════════════════
     QOL-01: Universal Undo
     ═══════════════════════════════════════════════════════ */
  var _undoStack = {};
  var undoToast = document.getElementById('rvtUndoToast');
  var undoMsg = document.getElementById('rvtUndoMsg');
  var undoBtn = document.getElementById('rvtUndoBtn');
  var _undoTimer = null;

  window.rvtUndo = function(actionType, description, undoFn) {
    _undoStack[actionType] = { fn: undoFn, desc: description };
    undoMsg.textContent = description;
    undoToast.classList.add('show');
    clearTimeout(_undoTimer);
    _undoTimer = setTimeout(function() {
      undoToast.classList.remove('show');
    }, 10000);
  };

  if (undoBtn) undoBtn.addEventListener('click', function() {
    var keys = Object.keys(_undoStack);
    if (keys.length === 0) return;
    var last = _undoStack[keys[keys.length - 1]];
    if (last && typeof last.fn === 'function') {
      try { last.fn(); } catch(e) { console.warn('[RVT Undo]', e); }
    }
    delete _undoStack[keys[keys.length - 1]];
    undoToast.classList.remove('show');
    clearTimeout(_undoTimer);
    if (typeof window.showToast === 'function') window.showToast('Undone', 'ok');
  }); // end undoBtn listener

  /* ═══════════════════════════════════════════════════════
     QOL-03: Operator Handoff
     ═══════════════════════════════════════════════════════ */
  var handoffOverlay = document.getElementById('rvtHandoffOverlay');

  function openHandoff() {
    var content = document.getElementById('rvtHandoffContent');
    var op = (window.S && window.S.operatorName) || 'Unknown';
    var view = document.body.getAttribute('data-view') || 'home';
    var sessionActive = document.body.getAttribute('data-ctl') === 'on';
    var alertCount = document.querySelectorAll('.alert-item:not(.acked), [data-alert]:not([data-acked="true"])').length;

    content.innerHTML =
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Outgoing Operator</div><div class="rvt-handoff-value">' + op + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Session State</div><div class="rvt-handoff-value">' + (sessionActive ? 'Active session in progress' : 'No active session') + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Current View</div><div class="rvt-handoff-value">' + view + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Unacknowledged Alerts</div><div class="rvt-handoff-value">' + alertCount + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Time</div><div class="rvt-handoff-value">' + new Date().toLocaleString() + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Notes for incoming operator</div>' +
        '<textarea id="rvtHandoffNotes" rows="3" style="width:100%;padding:8px;border:1px solid var(--line);border-radius:var(--r-sm);font-family:Inter,sans-serif;font-size:13px;resize:vertical;background:var(--surface-2);color:var(--ink-700);" placeholder="Any follow-ups, pending tasks…"></textarea>' +
      '</div>';

    if (handoffOverlay) handoffOverlay.classList.add('show');
  }

  function closeHandoff() { if (handoffOverlay) handoffOverlay.classList.remove('show'); }

  var _handoffCopyBtn = document.getElementById('rvtHandoffCopy');
  if (_handoffCopyBtn) _handoffCopyBtn.addEventListener('click', function() {
    var content = document.getElementById('rvtHandoffContent');
    var text = content.innerText;
    var notes = document.getElementById('rvtHandoffNotes');
    if (notes) text += '\nNotes: ' + notes.value;
    if (navigator.clipboard) {
      navigator.clipboard.writeText(text).then(function() {
        if (typeof window.showToast === 'function') window.showToast('Handoff summary copied', 'ok');
      });
    }
  });

  var _handoffConfirmBtn = document.getElementById('rvtHandoffConfirm');
  if (_handoffConfirmBtn) _handoffConfirmBtn.addEventListener('click', function() {
    closeHandoff();
    if (typeof window.showToast === 'function') window.showToast('Handoff confirmed — ready for new operator', 'ok');
  });

  if (handoffOverlay) handoffOverlay.addEventListener('click', function(e) { if (e.target === handoffOverlay) closeHandoff(); });
  document.addEventListener('keydown', function(e) {
    if (e.key === 'Escape' && handoffOverlay.classList.contains('show')) closeHandoff();
    if (e.ctrlKey && e.shiftKey && e.key === 'H') { e.preventDefault(); openHandoff(); }
  });

  window.openHandoff = openHandoff;

  /* ═══════════════════════════════════════════════════════
     QOL-07: Focus Panel Hotkey (F)
     ═══════════════════════════════════════════════════════ */
  var _focusedPanel = null;
  document.addEventListener('keydown', function(e) {
    if (e.key !== 'f' && e.key !== 'F') return;
    var active = document.activeElement;
    if (active && (active.tagName === 'TEXTAREA' || active.tagName === 'INPUT' || active.isContentEditable)) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;

    if (_focusedPanel) {
      _focusedPanel.classList.remove('rvt-panel-focused');
      _focusedPanel = null;
      return;
    }

    var panel = document.activeElement ? document.activeElement.closest('.card, .kpi, .panel, .wc') : null;
    if (!panel) return;
    e.preventDefault();
    panel.classList.add('rvt-panel-focused');
    _focusedPanel = panel;
  });

  document.addEventListener('keydown', function(e) {
    if (e.key === 'Escape' && _focusedPanel) {
      _focusedPanel.classList.remove('rvt-panel-focused');
      _focusedPanel = null;
    }
  });

  /* ═══════════════════════════════════════════════════════
     A11Y-04: Adjustable Font Scaling
     ═══════════════════════════════════════════════════════ */
  function injectFontScaleSetting() {
    var settingsView = document.getElementById('view-settings');
    if (!settingsView) return;
    if (settingsView.querySelector('#rvtFontScaleRow')) return;

    var row = document.createElement('div');
    row.className = 'set-r';
    row.id = 'rvtFontScaleRow';
    var saved = 100;
    try { saved = parseInt(localStorage.getItem('rvt-font-scale') || '100', 10); } catch(_) {}

    row.innerHTML =
      '<div class="set-copy"><strong>Font size</strong><span>Independent of density mode. Scales all text.</span></div>' +
      '<div class="rvt-voice-interval-seg">' +
        '<button data-scale="90" type="button">90%</button>' +
        '<button data-scale="100" type="button">100%</button>' +
        '<button data-scale="110" type="button">110%</button>' +
        '<button data-scale="125" type="button">125%</button>' +
      '</div>';

    row.querySelectorAll('button').forEach(function(btn) {
      var s = parseInt(btn.getAttribute('data-scale'), 10);
      if (s === saved) btn.classList.add('active');
      btn.addEventListener('click', function() {
        row.querySelectorAll('button').forEach(function(b) { b.classList.remove('active'); });
        btn.classList.add('active');
        document.documentElement.setAttribute('data-font-scale', s);
        try { localStorage.setItem('rvt-font-scale', String(s)); } catch(_) {}
      });
    });

    // Apply saved
    if (saved !== 100) document.documentElement.setAttribute('data-font-scale', saved);

    var appearance = settingsView.querySelector('.set-g:nth-child(2), [data-settings-group="appearance"]');
    if (appearance) appearance.appendChild(row);
    else settingsView.appendChild(row);
  }

  /* ═══════════════════════════════════════════════════════
     A11Y-05: Voice Captions
     ═══════════════════════════════════════════════════════ */
  var captionEl = document.getElementById('rvtVoiceCaption');
  var _captionTimer = null;

  // Patch speechSynthesis.speak to show captions
  if (window.speechSynthesis) {
    var _origSpeak = window.speechSynthesis.speak.bind(window.speechSynthesis);
    window.speechSynthesis.speak = function(utterance) {
      _origSpeak(utterance);
      if (captionEl && utterance && utterance.text) {
        captionEl.textContent = utterance.text;
        captionEl.classList.add('show');
        clearTimeout(_captionTimer);
        _captionTimer = setTimeout(function() { captionEl.classList.remove('show'); }, 4000);
      }
    };
  }

  /* ═══════════════════════════════════════════════════════
     AUDIT-05: Export Audit Trail as CSV
     ═══════════════════════════════════════════════════════ */
  function injectAuditExport() {
    var auditTab = document.querySelector('#tab-audit, [data-tab="audit"]');
    if (!auditTab) return;

    var auditView = document.querySelector('.audit-content, [data-tab-content="audit"]');
    if (!auditView) return;
    if (auditView.querySelector('.rvt-audit-export-btn')) return;

    var btn = document.createElement('button');
    btn.className = 'rvt-audit-export-btn';
    btn.type = 'button';
    btn.innerHTML = '<span class="material-symbols-rounded" style="font-size:14px;">download</span>Export CSV';
    btn.addEventListener('click', function() {
      var rows = [['Timestamp', 'Severity', 'Source', 'Reason', 'Message']];
      auditView.querySelectorAll('.audit-event, .event-row, tr[data-event]').forEach(function(el) {
        var cells = el.querySelectorAll('td, .event-cell, span');
        var row = [];
        cells.forEach(function(c) { row.push('"' + (c.textContent || '').trim().replace(/"/g, '""') + '"'); });
        if (row.length > 0) rows.push(row);
      });
      var csv = rows.map(function(r) { return r.join(','); }).join('\n');
      var blob = new Blob([csv], { type: 'text/csv' });
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url;
      var sid = (window.S && window.S.sessionId) || 'session';
      a.download = 'audit_' + sid + '_' + new Date().toISOString().split('T')[0] + '.csv';
      a.click();
      URL.revokeObjectURL(url);
    });

    auditView.insertBefore(btn, auditView.firstChild);
  }

  /* ═══════════════════════════════════════════════════════
     HELP-03: Contextual Deep-Linking
     ═══════════════════════════════════════════════════════ */
  function handleHelpDeepLink() {
    var hash = location.hash;
    if (!hash || !hash.startsWith('#help/')) return;
    var term = hash.replace('#help/glossary/', '').replace('#help/', '');
    if (!term) return;

    if (typeof window.switchView === 'function') window.switchView('help');

    setTimeout(function() {
      var helpView = document.getElementById('view-help');
      if (!helpView) return;

      var target = helpView.querySelector('#' + CSS.escape(term)) ||
        helpView.querySelector('[data-term="' + term + '"]');

      if (!target) {
        // Search by text content
        var entries = helpView.querySelectorAll('dt, h3, .glossary-term, .faq-q');
        for (var i = 0; i < entries.length; i++) {
          if ((entries[i].textContent || '').toLowerCase().includes(term.toLowerCase().replace(/-/g, ' '))) {
            target = entries[i];
            break;
          }
        }
      }

      if (target) {
        target.classList.add('rvt-help-highlight');
        target.scrollIntoView({ behavior: 'smooth', block: 'center' });
        setTimeout(function() { target.classList.remove('rvt-help-highlight'); }, 1500);
      }
    }, 400);
  }

  window.addEventListener('hashchange', handleHelpDeepLink);

  // Wire (?) icons to deep-link
  function wireHelpIcons() {
    document.querySelectorAll('[data-help-term], .help-link').forEach(function(el) {
      if (el.__rvtHelpWired) return;
      el.__rvtHelpWired = true;
      el.addEventListener('click', function(e) {
        e.preventDefault();
        var term = el.getAttribute('data-help-term') || el.getAttribute('href') || '';
        term = term.replace('#help/glossary/', '').replace('#help/', '').replace('#', '');
        if (term) {
          location.hash = 'help/glossary/' + term;
          handleHelpDeepLink();
        }
      });
    });
  }

  /* ═══════════════════════════════════════════════════════
     PERF-03: Chart.js destroy on view switch
     ═══════════════════════════════════════════════════════ */
  onBodyMutation(function(muts) {
    muts.forEach(function(m) {
      if (m.attributeName === 'data-view') {
        // Destroy charts from previous view
        if (window.Chart && window.Chart.instances) {
          var instances = Object.values ? Object.values(window.Chart.instances) : [];
          var managedCharts = [];
          try {
            managedCharts = (window.S && window.S.charts && Object.values) ? Object.values(window.S.charts) : [];
          } catch(_) {}
          instances.forEach(function(chart) {
            try {
              if (managedCharts.indexOf(chart) !== -1) return;
              var canvas = chart.canvas;
              if (canvas && !canvas.offsetParent) {
                chart.destroy();
              }
            } catch(_) {}
          });
        }
      }
    });
  });

  /* ═══════════════════════════════════════════════════════
     PERF-04: Debounce resize handlers
     ═══════════════════════════════════════════════════════ */
  var _resizeRaf = 0;
  window.addEventListener('resize', function() {
    if (_resizeRaf) cancelAnimationFrame(_resizeRaf);
    _resizeRaf = requestAnimationFrame(function() {
      _resizeRaf = 0;
      scheduleSync();
    });
  }, { passive: true });

  /* ═══════════════════════════════════════════════════════
     Master boot
     ═══════════════════════════════════════════════════════ */
  function v152Boot() {
    injectPipeline();
    injectCopyBrief();
    injectRailCollapse();

    // View-specific injections via sync loop
    registerSync(function(view) {
      if (view === 'report') {
        injectExecSummary();
        tintDiagnosticCards();
      }
      if (view === 'settings') {
        injectProfilesSection();
        injectFontScaleSetting();
      }
      if (view === 'live') {
        injectAuditExport();
        wireHelpIcons();
      }
      if (view === 'help') {
        wireHelpIcons();
      }
    });

    // One-time cluster
    setTimeout(clusterAlerts, 3000);

    // Handle deep links
    handleHelpDeepLink();

    scheduleSync();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', v152Boot, { once: true });
  } else {
    setTimeout(v152Boot, 1000);
  }

})();
;
/* END modules/_anon-003.js */

/* BEGIN modules/rvt-phase-alpha-foundation-fixes-js.js */
(function(){
  'use strict';

  var PATCH_ID = 'rvt-phase-alpha-foundation-fixes';
  var CLIENT_VERSION = Object.freeze({ dashboard: '11.0.0', trainer: '11.0.0', firmware: 'v15.0.0' });
  var SCHEMA_KEY = 'rvt-schema-version';
  var CURRENT_SCHEMA_VERSION = 1;
  var MIGRATION_LOG_KEY = 'rvt-migration-log';
  var STORAGE_WARN_INTERVAL_MS = 10000;
  var lastStorageWarningAt = 0;

  function softError(scope, err, opts) {
    opts = opts || {};
    if (typeof window.rvtSoftError === 'function') {
      window.rvtSoftError(scope, err, opts);
      return;
    }
    if (window.console && (opts.always || (window.S && window.S.debug))) console.warn('[RVT]', scope, err);
  }

  function toast(message, icon) {
    try {
      if (typeof window.toast === 'function') window.toast(message, icon || 'info');
      else if (typeof window.showToast === 'function') window.showToast(message, icon || 'info');
    } catch (err) {
      softError('toast failed', err);
    }
  }

  function ensureRuntimeLists() {
    window.S = window.S || {};
    if (!window.S.charts) window.S.charts = {};
    if (!window.S.ctl) window.S.ctl = {};
    if (!window.S.spark) window.S.spark = { fps: [], dist: [] };
    if (!Array.isArray(window.S.__mutationObservers)) window.S.__mutationObservers = [];
    if (Array.isArray(window.__rvtPendingMutationObservers) && window.__rvtPendingMutationObservers.length) {
      window.__rvtPendingMutationObservers.splice(0).forEach(function(observer) {
        if (observer && !window.S.__mutationObservers.includes(observer)) window.S.__mutationObservers.push(observer);
      });
    }
    return window.S;
  }

  function isQuotaError(err) {
    return !!err && (
      err.name === 'QuotaExceededError' ||
      err.name === 'NS_ERROR_DOM_QUOTA_REACHED' ||
      err.code === 22 ||
      err.code === 1014 ||
      /quota|exceeded|storage/i.test(String(err.message || ''))
    );
  }

  function storageWarning(key, err) {
    var now = Date.now();
    if (now - lastStorageWarningAt < STORAGE_WARN_INTERVAL_MS) return;
    lastStorageWarningAt = now;
    var banner = document.getElementById('rvtQuotaBanner');
    if (banner) banner.classList.add('show');
    toast('Storage warning: ' + key + ' was not saved', 'storage');
    softError('storage write failed for ' + key, err, { always: true, surface: true, severity: 'warn' });
  }

  function safeStorageSet(key, value, opts) {
    opts = opts || {};
    try {
      localStorage.setItem(key, String(value));
      return true;
    } catch (err) {
      if (isQuotaError(err)) storageWarning(key, err);
      else softError('localStorage.setItem failed for ' + key, err, { always: !!opts.important });
      return false;
    }
  }

  function safeStorageRemove(key) {
    try {
      localStorage.removeItem(key);
      return true;
    } catch (err) {
      softError('localStorage.removeItem failed for ' + key, err);
      return false;
    }
  }

  function safeStorageJson(key, fallback, validator) {
    try {
      var raw = localStorage.getItem(key);
      if (raw === null) return fallback;
      var parsed = JSON.parse(raw);
      if (validator && !validator(parsed)) throw new Error('invalid shape');
      return parsed;
    } catch (err) {
      try {
        var bad = localStorage.getItem(key);
        if (bad !== null) safeStorageSet('rvt-corrupt-backup-' + key + '-' + Date.now(), String(bad).slice(0, 200));
      } catch (_) {}
      safeStorageRemove(key);
      softError('corrupted storage reset: ' + key, err, { always: true, surface: true, severity: 'warn' });
      return fallback;
    }
  }

  function safeStorageJsonSet(key, value, opts) {
    try {
      return safeStorageSet(key, JSON.stringify(value), opts);
    } catch (err) {
      storageWarning(key, err);
      return false;
    }
  }

  var STORAGE_MANIFEST = {
    'rvt-snaps': { kind: 'json', defaultValue: [], validate: Array.isArray },
    'rvt-session-draft-ts': { kind: 'scalar', defaultValue: '', validate: function(v){ return typeof v === 'string'; } },
    'rvt-snap-view-mode': { kind: 'scalar', defaultValue: 'grid', validate: function(v){ return v === 'grid' || v === 'list'; } },
    'rvt-snap-order': { kind: 'json', defaultValue: [], validate: Array.isArray },
    'rvt-snap-notes': { kind: 'json', defaultValue: {}, validate: function(v){ return !!v && typeof v === 'object' && !Array.isArray(v); } },
    'rvt-alert-pins': { kind: 'json', defaultValue: [], validate: Array.isArray },
    'rvt-session-notes': { kind: 'json', defaultValue: {}, validate: function(v){ return !!v && typeof v === 'object' && !Array.isArray(v); } },
    'rvt-setup-prefs': { kind: 'json', defaultValue: {}, validate: function(v){ return !!v && typeof v === 'object' && !Array.isArray(v); } },
    'rvt-session-tags': { kind: 'json', defaultValue: {}, validate: function(v){ return !!v && typeof v === 'object' && !Array.isArray(v); } },
    'rvt-dashboard-prefs': { kind: 'json', defaultValue: {}, validate: function(v){ return !!v && typeof v === 'object' && !Array.isArray(v); } },
    'rvt-thresholds': { kind: 'json', defaultValue: {}, validate: function(v){ return !!v && typeof v === 'object' && !Array.isArray(v); } },
    'rvt-kpi-order': { kind: 'json', defaultValue: ['hr', 'rr', 'fps', 'distance'], validate: Array.isArray },
    'rvt-settings-profiles': { kind: 'json', defaultValue: {}, validate: function(v){ return !!v && typeof v === 'object' && !Array.isArray(v); } },
    'rvt-migration-log': { kind: 'json', defaultValue: [], validate: Array.isArray },
    'rvt-theme': { kind: 'scalar', defaultValue: 'system', validate: function(v){ return ['system', 'light', 'dark', 'night', 'hc'].includes(v); } },
    'rvt-density': { kind: 'scalar', defaultValue: 'comfortable', validate: function(v){ return ['comfortable', 'compact'].includes(v); } },
    'rvt-live-mode': { kind: 'scalar', defaultValue: 'simple', validate: function(v){ return ['simple', 'advanced'].includes(v); } },
    'rvt-live-buffer-seconds': { kind: 'scalar', defaultValue: '60', validate: function(v){ var n = Number(v); return Number.isFinite(n) && n >= 10 && n <= 600; } },
    'rvt-max-chart-points': { kind: 'scalar', defaultValue: '3600', validate: function(v){ var n = Number(v); return Number.isFinite(n) && n >= 300 && n <= 20000; } },
    'rvt-voice-alerts': { kind: 'scalar', defaultValue: '0', validate: function(v){ return v === '0' || v === '1'; } },
    'rvt-audio-alerts': { kind: 'scalar', defaultValue: '0', validate: function(v){ return v === '0' || v === '1'; } },
    'rvt-audio-volume': { kind: 'scalar', defaultValue: '0.7', validate: function(v){ var n = Number(v); return Number.isFinite(n) && n >= 0 && n <= 1; } }
  };

  window.RVT_CLIENT_VERSION = CLIENT_VERSION;
  window.rvtStorage = {
    schemaKey: SCHEMA_KEY,
    schemaVersion: CURRENT_SCHEMA_VERSION,
    manifest: STORAGE_MANIFEST,
    getJson: safeStorageJson,
    setJson: safeStorageJsonSet,
    set: safeStorageSet,
    remove: safeStorageRemove
  };

  function appendMigrationLog(entry) {
    var log = safeStorageJson(MIGRATION_LOG_KEY, [], Array.isArray);
    log.unshift(entry);
    safeStorageJsonSet(MIGRATION_LOG_KEY, log.slice(0, 20), { important: true });
  }

  function migrateStorageSchema() {
    var prior = 0;
    try { prior = Number(localStorage.getItem(SCHEMA_KEY) || '0') || 0; } catch (_) {}
    var resetKeys = [];
    Object.keys(STORAGE_MANIFEST).forEach(function(key) {
      var spec = STORAGE_MANIFEST[key];
      var raw = null;
      try { raw = localStorage.getItem(key); } catch (err) { softError('storage read failed: ' + key, err); return; }
      if (raw === null) return;
      try {
        if (spec.kind === 'json') {
          var parsed = JSON.parse(raw);
          if (spec.validate && !spec.validate(parsed)) throw new Error('invalid shape');
        } else if (spec.validate && !spec.validate(raw)) {
          throw new Error('invalid value');
        }
      } catch (err) {
        resetKeys.push(key);
        try { safeStorageSet('rvt-corrupt-backup-' + key + '-' + Date.now(), String(raw).slice(0, 200)); } catch (_) {}
        if (spec.kind === 'json') safeStorageJsonSet(key, spec.defaultValue, { important: true });
        else safeStorageSet(key, spec.defaultValue, { important: true });
        softError('storage key reset: ' + key, err, { always: true, surface: true, severity: 'warn' });
      }
    });
    safeStorageSet(SCHEMA_KEY, String(CURRENT_SCHEMA_VERSION), { important: true });
    appendMigrationLog({ ts: new Date().toISOString(), from: prior, to: CURRENT_SCHEMA_VERSION, reset_keys: resetKeys, patch: PATCH_ID });
    if (resetKeys.length) toast(resetKeys.length + ' setting' + (resetKeys.length === 1 ? ' was' : 's were') + ' corrupted and reset to defaults', 'warning');
  }

  function findStartButton() {
    return document.querySelector('#startBtn, #startSessionBtn, .home-start-btn, .setup-start, [onclick*="startSession"]');
  }

  function findStartButtons() {
    return Array.prototype.slice.call(document.querySelectorAll('#startBtn, #startSessionBtn, .home-start-btn, .setup-start, [onclick*="startSession"]'));
  }

  function getStartReadiness() {
    var st = ensureRuntimeLists();
    if (!st.ctl) st.ctl = {};
    if (!st.ctl.startReadiness) {
      st.ctl.startReadiness = {
        defaults: !!st.ctl.defaults,
        sessions: Array.isArray(st.ctl.sessionItems),
        preflight: !!st.ctl.preflight
      };
    }
    return st.ctl.startReadiness;
  }

  function pendingStartDeps() {
    var r = getStartReadiness();
    return ['defaults', 'sessions', 'preflight'].filter(function(key){ return !r[key]; });
  }

  function markStartReady(key) {
    var r = getStartReadiness();
    r[key] = true;
    syncStartGuard();
  }

  function structuralFailures() {
    try {
      if (typeof window.structuralFailures === 'function') return window.structuralFailures();
    } catch (err) {
      softError('structuralFailures failed', err);
    }
    var checks = window.S && window.S.ctl && window.S.ctl.preflight && Array.isArray(window.S.ctl.preflight.checks)
      ? window.S.ctl.preflight.checks
      : [];
    return checks.filter(function(c){ return c && c.status === 'fail'; });
  }

  function syncStartReason(btn, reason, severity) {
    if (!btn) return;
    var id = 'rvtStartReason';
    var el = document.getElementById(id);
    if (!reason) {
      btn.removeAttribute('aria-describedby');
      if (el) el.remove();
      return;
    }
    if (!el) {
      el = document.createElement('span');
      el.id = id;
      el.className = 'rvt-disable-reason';
      if (btn.parentNode) btn.parentNode.insertBefore(el, btn.nextSibling);
    }
    el.textContent = reason;
    el.setAttribute('data-severity', severity || 'warn');
    btn.setAttribute('aria-describedby', id);
    btn.title = reason;
  }

  function computeStartReason() {
    var pending = pendingStartDeps();
    if (pending.length) return { text: 'Waiting for ' + pending.join(', ') + ' to finish', severity: 'warn', pending: true };
    if (window.S && window.S.ctl && window.S.ctl.current) return { text: 'A session is already active', severity: 'warn' };
    var failures = structuralFailures();
    if (failures.length) return { text: 'Fix ' + failures.length + ' blocking preflight check' + (failures.length === 1 ? '' : 's'), severity: 'bad' };
    return null;
  }

  function syncStartGuard() {
    var buttons = findStartButtons();
    var btn = buttons[0];
    if (!btn) return;
    var reason = computeStartReason();
    if (reason && reason.pending) {
      buttons.forEach(function(startBtn) {
        startBtn.dataset.rvtStartBootBlocked = '1';
        startBtn.disabled = true;
        startBtn.setAttribute('aria-disabled', 'true');
        startBtn.setAttribute('aria-describedby', 'rvtStartReason');
        startBtn.title = reason.text;
      });
      syncStartReason(btn, reason.text, reason.severity);
      return;
    }
    var hadBootBlock = buttons.some(function(startBtn) { return startBtn.dataset.rvtStartBootBlocked === '1'; });
    if (hadBootBlock) {
      buttons.forEach(function(startBtn) {
        if (startBtn.dataset.rvtStartBootBlocked === '1') {
          delete startBtn.dataset.rvtStartBootBlocked;
          startBtn.disabled = false;
          startBtn.removeAttribute('aria-disabled');
        }
      });
      try {
        if (typeof window.updateStartGate === 'function') window.updateStartGate();
      } catch (err) {
        softError('updateStartGate after boot guard failed', err, { always: true });
      }
    }
    reason = computeStartReason();
    if (buttons.some(function(startBtn) { return startBtn.disabled; }) || reason) {
      var text = reason ? reason.text : 'Preflight checks pending';
      var severity = reason ? reason.severity : 'warn';
      buttons.forEach(function(startBtn) {
        startBtn.setAttribute('aria-describedby', 'rvtStartReason');
        startBtn.title = text;
      });
      syncStartReason(btn, text, severity);
    } else {
      buttons.forEach(function(startBtn) {
        startBtn.removeAttribute('aria-describedby');
        startBtn.removeAttribute('aria-disabled');
        startBtn.removeAttribute('title');
      });
      syncStartReason(btn, '', '');
    }
  }

  function wrapAsyncReadiness(name, fnName) {
    var fn = window[fnName];
    if (typeof fn !== 'function' || fn.__rvtPhaseAlphaReadinessWrapped) return false;
    window[fnName] = function() {
      var result;
      try {
        result = fn.apply(this, arguments);
      } catch (err) {
        markStartReady(name);
        throw err;
      }
      Promise.resolve(result).catch(function(err) {
        softError(fnName + ' failed', err, { always: true, surface: true, severity: 'warn' });
      }).finally(function() {
        markStartReady(name);
      });
      return result;
    };
    window[fnName].__rvtPhaseAlphaReadinessWrapped = true;
    return true;
  }

  function installStartGuard() {
    wrapAsyncReadiness('defaults', 'loadControlDefaults');
    wrapAsyncReadiness('sessions', 'loadSessionsList');
    wrapAsyncReadiness('preflight', 'runPreflightBatch');
    syncStartGuard();
  }

  function ensureAnnouncer() {
    var el = document.getElementById('vitalsAnnouncer') || document.getElementById('rvtVitalLive');
    if (!el) {
      el = document.createElement('div');
      el.id = 'vitalsAnnouncer';
      el.className = 'sr-only';
      el.setAttribute('role', 'status');
      el.setAttribute('aria-live', 'polite');
      el.setAttribute('aria-atomic', 'true');
      document.body.appendChild(el);
    }
    var grid = document.querySelector('.kpi-grid[aria-live]');
    if (grid) {
      grid.removeAttribute('aria-live');
      grid.removeAttribute('aria-atomic');
    }
    return el;
  }

  var vitalAnnounceAt = { hr: 0, rr: 0 };
  var vitalAnnounceState = { hr: '', rr: '' };
  function announceVitalThresholds(payload) {
    var el = ensureAnnouncer();
    var thresholds = Object.assign({ hrLow: 40, hrHigh: 140, rrLow: 6, rrHigh: 30 }, (window.S && window.S.kpiThresholds) || {});
    var radar = payload && payload.radar ? payload.radar : {};
    var hr = Number(radar.hr_bpm ?? radar.hr ?? document.getElementById('kpiHr')?.textContent);
    var rr = Number(radar.rr_bpm ?? radar.rr ?? document.getElementById('kpiRr')?.textContent);
    [
      { key: 'hr', label: 'Heart rate', value: hr, low: thresholds.hrLow, high: thresholds.hrHigh },
      { key: 'rr', label: 'Respiratory rate', value: rr, low: thresholds.rrLow, high: thresholds.rrHigh }
    ].forEach(function(item) {
      if (!Number.isFinite(item.value)) return;
      var direction = '';
      var threshold = null;
      if (Number.isFinite(item.high) && item.value > item.high) { direction = 'exceeded'; threshold = item.high; }
      else if (Number.isFinite(item.low) && item.value < item.low) { direction = 'fell below'; threshold = item.low; }
      var state = direction ? direction + ':' + threshold : 'ok';
      if (!direction) { vitalAnnounceState[item.key] = 'ok'; return; }
      var now = Date.now();
      if (vitalAnnounceState[item.key] === state && now - vitalAnnounceAt[item.key] < 10000) return;
      vitalAnnounceState[item.key] = state;
      vitalAnnounceAt[item.key] = now;
      el.textContent = item.label + ' critical: ' + Math.round(item.value) + ', ' + direction + ' ' + threshold + ' threshold';
    });
  }

  function unwrapLivePayload(raw) {
    if (!raw || typeof raw !== 'object' || Array.isArray(raw)) return raw;
    if (raw.payload && typeof raw.payload === 'object') return raw.payload;
    if (raw.current && typeof raw.current === 'object') return raw.current;
    if (raw.latest && typeof raw.latest === 'object') return raw.latest;
    if (raw.last && typeof raw.last === 'object') return raw.last;
    if (raw.item && typeof raw.item === 'object') return raw.item;
    if (raw.session && typeof raw.session === 'object') return raw.session;
    return raw;
  }

  function validateLivePayloadShape(raw) {
    var errors = [];
    var warnings = [];
    var candidate = raw;
    if (candidate && typeof candidate.payload === 'string') {
      try { candidate = Object.assign({}, candidate, { payload: JSON.parse(candidate.payload) }); }
      catch (err) { errors.push('payload string is not valid JSON'); }
    }
    var payload = unwrapLivePayload(candidate);
    if (!payload || typeof payload !== 'object' || Array.isArray(payload)) errors.push('payload is not an object');
    else {
      if (payload.meta != null && (typeof payload.meta !== 'object' || Array.isArray(payload.meta))) errors.push('meta must be an object');
      if (payload.radar != null && (typeof payload.radar !== 'object' || Array.isArray(payload.radar))) errors.push('radar must be an object');
      if (payload.series != null && (typeof payload.series !== 'object' || Array.isArray(payload.series))) errors.push('series must be an object');
      if (!payload.meta && !payload.radar && !payload.series) errors.push('payload missing meta, radar, and series sections');
      if (payload.series) {
        Object.keys(payload.series).forEach(function(key) {
          var value = payload.series[key];
          if (value != null && !Array.isArray(value)) warnings.push('series.' + key + ' is not an array');
        });
      }
      ['hr_bpm', 'rr_bpm', 'fps', 'distance_cm', 'range_cm', 'last_seen_age_s'].forEach(function(key) {
        if (payload.radar && payload.radar[key] != null && !Number.isFinite(Number(payload.radar[key]))) {
          warnings.push('radar.' + key + ' is not finite');
        }
      });
    }
    return { ok: errors.length === 0, payload: payload, errors: errors, warnings: warnings };
  }

  function appendAuditLog(msg, sev) {
    if (!window.S) return;
    if (!window.S.liveAuditLogs) {
      try { window.S.liveAuditLogs = JSON.parse(localStorage.getItem('rvt-live-audit') || '[]'); } catch(_) { window.S.liveAuditLogs = []; }
      if (!Array.isArray(window.S.liveAuditLogs)) window.S.liveAuditLogs = [];
    }
    var log = { ts: new Date().toISOString(), msg: msg, sev: sev || 'info' };
    window.S.liveAuditLogs.unshift(log);
    if (window.S.liveAuditLogs.length > 100) window.S.liveAuditLogs.length = 100;
    try { localStorage.setItem('rvt-live-audit', JSON.stringify(window.S.liveAuditLogs)); } catch(_) {}
    if (typeof syncAuditUi === 'function') syncAuditUi();
  }

  function syncAuditUi() {
    var invEl = document.getElementById('auditSchemaInvalidCount');
    if (invEl && window.S && window.S.liveSchema) invEl.textContent = window.S.liveSchema.invalid;
    var warnEl = document.getElementById('auditSchemaWarningCount');
    if (warnEl && window.S && window.S.liveSchema) warnEl.textContent = window.S.liveSchema.warnings;
    var recEl = document.getElementById('auditReconnectCount');
    if (recEl && window.S && window.S.reconnectPolicy) recEl.textContent = window.S.reconnectPolicy.attempts + ' / ' + window.S.reconnectPolicy.maxAttempts;

    var logEl = document.getElementById('liveAuditLog');
    if (logEl) {
      if (!window.S || !window.S.liveAuditLogs) {
        try { window.S = window.S || {}; window.S.liveAuditLogs = JSON.parse(localStorage.getItem('rvt-live-audit') || '[]'); } catch(_) { window.S.liveAuditLogs = []; }
      }
      if (window.S.liveAuditLogs && window.S.liveAuditLogs.length) {
        logEl.innerHTML = window.S.liveAuditLogs.map(function(l) {
          var c = l.sev === 'bad' || l.sev === 'fatal' ? '#ff3b30' : (l.sev === 'warn' ? '#fbbf24' : (l.sev === 'good' ? '#34d399' : 'inherit'));
          var time = ''; try { time = l.ts.split('T')[1].slice(0,-1); } catch(_) { time = l.ts; }
          return '<div style="margin-bottom: 4px; line-height: 1.4;"><span style="opacity:0.5; margin-right:8px;">' + time + '</span><span style="color:' + c + '">' + escRaw(l.msg) + '</span></div>';
        }).join('');
      }
    }
  }

  function installPayloadValidation() {
    if (typeof window.normalizeLivePayload === 'function' && !window.normalizeLivePayload.__rvtPhaseAlphaValidated) {
      var originalNormalize = window.normalizeLivePayload;
      window.normalizeLivePayload = function(raw) {
        var validation = validateLivePayloadShape(raw);
        window.S = window.S || {};
        if (!window.S.liveSchema) window.S.liveSchema = { invalid: 0, warnings: 0, lastErrors: [] };
        if (!validation.ok) {
          window.S.liveSchema.invalid += 1;
          window.S.liveSchema.lastErrors = validation.errors.slice(0, 5);
          softError('Invalid live payload schema', new Error(validation.errors.join('; ')), { always: true, surface: true, severity: 'warn' });
          if (typeof appendAuditLog === 'function') appendAuditLog('Schema invalid: ' + validation.errors.slice(0, 2).join(', '), 'bad');
          return {};
        }
        var normalized = originalNormalize.apply(this, arguments);
        if (normalized && typeof normalized === 'object') {
          if (!normalized.meta || typeof normalized.meta !== 'object') normalized.meta = {};
          normalized.meta.schema_valid = true;
          if (validation.warnings.length) {
            window.S.liveSchema.warnings += validation.warnings.length;
            normalized.meta.schema_warnings = validation.warnings.slice(0, 5);
          }
        }
        if (typeof syncAuditUi === 'function') syncAuditUi();
        return normalized;
      };
      window.normalizeLivePayload.__rvtPhaseAlphaValidated = true;
    }
  }

  function isSensorStale(payload) {
    var ageMs = window.S && window.S.lastGoodAt ? Date.now() - window.S.lastGoodAt : 0;
    var radarAge = Number(payload && payload.radar && payload.radar.last_seen_age_s);
    var status = payload && payload.meta && payload.meta.status;
    return status === 'stale' || (window.S && window.S.disc >= 3) || ageMs > 3000 || (Number.isFinite(radarAge) && radarAge >= 3);
  }

  var dropoutActive = false;
  var dropoutStartedAt = 0;
  var dropoutAssertiveAt = 0;
  function syncDropoutUi(payload) {
    var stale = isSensorStale(payload || (window.S && window.S.lastPayload));
    document.body.classList.toggle('rvt-sensor-stale', !!stale);
    var announcer = ensureAnnouncer();
    if (stale && !dropoutActive) {
      dropoutActive = true;
      dropoutStartedAt = Date.now();
      dropoutAssertiveAt = 0;
      if (window.S) {
        if (!Array.isArray(window.S.dropoutEvents)) window.S.dropoutEvents = [];
        window.S.dropoutEvents.unshift({ ts: new Date().toISOString(), status: 'stale_start' });
        if (typeof appendAuditLog === 'function') appendAuditLog('Sensor dropout started', 'warn');
      }
    } else if (!stale && dropoutActive) {
      dropoutActive = false;
      if (window.S && Array.isArray(window.S.dropoutEvents)) {
        window.S.dropoutEvents.unshift({ ts: new Date().toISOString(), status: 'stale_end', duration_ms: Date.now() - dropoutStartedAt });
        if (typeof appendAuditLog === 'function') appendAuditLog('Sensor recovered after ' + (Date.now() - dropoutStartedAt) + 'ms', 'good');
      }
    }
    if (stale) {
      ['kpiHr', 'kpiRr', 'kpiFps', 'kpiDist'].forEach(function(id) {
        var valueEl = document.getElementById(id);
        if (valueEl) valueEl.textContent = '--';
      });
      ['kpiHrSub', 'kpiRrSub', 'kpiFpsSub', 'kpiDistSub'].forEach(function(id) {
        var subEl = document.getElementById(id);
        if (subEl) subEl.textContent = 'stale - last update expired';
      });
      if (Date.now() - dropoutStartedAt >= 30000 && !dropoutAssertiveAt) {
        dropoutAssertiveAt = Date.now();
        announcer.setAttribute('aria-live', 'assertive');
        announcer.textContent = 'Radar data critical: no new frames for 30 seconds';
      } else if (Date.now() - dropoutStartedAt >= 10000 && !dropoutAssertiveAt) {
        announcer.textContent = 'Radar data warning: no new frames for 10 seconds';
      }
    } else {
      announcer.setAttribute('aria-live', 'polite');
    }
  }

  function installRenderGuards() {
    if (typeof window.render === 'function' && !window.render.__rvtPhaseAlphaWrapped) {
      var originalRender = window.render;
      window.render = function(payload) {
        var result = originalRender.apply(this, arguments);
        try {
          announceVitalThresholds(payload);
          syncDropoutUi(payload);
        } catch (err) {
          softError('phase alpha render guard failed', err);
        }
        return result;
      };
      window.render.__rvtPhaseAlphaWrapped = true;
    }
  }

  function registerChart(chart, key) {
    if (!chart || typeof chart !== 'object') return chart;
    if (!window.__rvtChartRegistry) window.__rvtChartRegistry = new Set();
    window.__rvtChartRegistry.add(chart);
    chart.__rvtKey = key || chart.__rvtKey || '';
    if (chart.canvas) chart.canvas.__rvtChart = chart;
    return chart;
  }

  function registerExistingCharts() {
    if (window.S && window.S.charts) {
      Object.keys(window.S.charts).forEach(function(key) { registerChart(window.S.charts[key], key); });
    }
    if (window.Chart && window.Chart.instances) {
      try {
        var instances = Object.values(window.Chart.instances);
        instances.forEach(function(chart) { registerChart(chart, chart && chart.__rvtKey); });
      } catch (err) {
        softError('Chart.instances scan failed', err);
      }
    }
  }

  function destroyChart(chart) {
    if (!chart || chart.__rvtDestroyed) return;
    try { chart.destroy(); } catch (err) { softError('chart destroy failed', err, { always: true }); }
    chart.__rvtDestroyed = true;
    if (window.__rvtChartRegistry) window.__rvtChartRegistry.delete(chart);
    if (chart.canvas && chart.canvas.__rvtChart === chart) chart.canvas.__rvtChart = null;
  }

  window.destroyDashboardCharts = function(opts) {
    opts = opts || {};
    registerExistingCharts();
    var managedCharts = new Set();
    if (window.S && window.S.charts) {
      Object.keys(window.S.charts).forEach(function(key) {
        if (window.S.charts[key]) managedCharts.add(window.S.charts[key]);
      });
    }
    if (opts.includeManaged && window.S && window.S.charts) {
      Object.keys(window.S.charts).forEach(function(key) {
        destroyChart(window.S.charts[key]);
        delete window.S.charts[key];
      });
    }
    if (window.__rvtChartRegistry) {
      Array.from(window.__rvtChartRegistry).forEach(function(chart) {
        if (!opts.includeManaged && managedCharts.has(chart)) return;
        var canvas = chart && chart.canvas;
        if (!canvas || !document.documentElement.contains(canvas) || (opts.destroyHidden && !canvas.offsetParent)) {
          destroyChart(chart);
        }
      });
    }
  };

  function installChartLifecycle() {
    registerExistingCharts();
    if (typeof window.buildCharts === 'function' && !window.buildCharts.__rvtPhaseAlphaWrapped) {
      var originalBuildCharts = window.buildCharts;
      window.buildCharts = function() {
        window.destroyDashboardCharts({ includeManaged: true });
        var result = originalBuildCharts.apply(this, arguments);
        registerExistingCharts();
        return result;
      };
      window.buildCharts.__rvtPhaseAlphaWrapped = true;
    }
    if (typeof window.switchView === 'function' && !window.switchView.__rvtPhaseAlphaChartWrapped) {
      var originalSwitchView = window.switchView;
      window.switchView = function(view) {
        var prev = document.body.getAttribute('data-view') || '';
        var result = originalSwitchView.apply(this, arguments);
        if (prev === 'live' && view !== 'live') {
          window.destroyDashboardCharts({ destroyHidden: true });
        }
        if (view === 'live') {
          setTimeout(function() {
            registerExistingCharts();
            if (typeof window.buildCharts === 'function' && window.S && window.S.charts && !Object.keys(window.S.charts).length) {
              window.buildCharts();
            }
          }, 80);
        }
        return result;
      };
      window.switchView.__rvtPhaseAlphaChartWrapped = true;
    }
    window.addEventListener('pagehide', function() { window.destroyDashboardCharts({ includeManaged: true }); });
  }

  function showReconnectFatal() {
    var banner = document.getElementById('rvtReconnectFailed');
    if (banner) {
      banner.classList.add('show');
      var strong = banner.querySelector('strong');
      if (strong) strong.textContent = 'Reconnect failed';
      var span = banner.querySelector('span');
      if (span) span.textContent = 'Automatic retries stopped. Click resume to try again.';
    }
    if (window.S) {
      if (!Array.isArray(window.S.reconnectAudit)) window.S.reconnectAudit = [];
      window.S.reconnectAudit.unshift({ ts: new Date().toISOString(), status: 'fatal_retry_cap' });
    }
  }

  function installReconnectCap() {
    if (!window.S) return;
    if (!window.S.reconnectPolicy) window.S.reconnectPolicy = { attempts: 0, startedAt: 0, fatal: false, maxAttempts: 30, maxMs: 3600000 };
    if (typeof window.scheduleNextPoll === 'function' && !window.scheduleNextPoll.__rvtPhaseAlphaWrapped) {
      var originalScheduleNextPoll = window.scheduleNextPoll;
      window.scheduleNextPoll = function(delayMs) {
        var policy = window.S.reconnectPolicy || (window.S.reconnectPolicy = { attempts: 0, startedAt: 0, fatal: false, maxAttempts: 30, maxMs: 3600000 });
        if (window.S.disc > 0) {
          if (!policy.startedAt) policy.startedAt = Date.now();
          policy.attempts += 1;
          if (typeof syncAuditUi === 'function') syncAuditUi();
          if (policy.fatal || policy.attempts >= policy.maxAttempts || Date.now() - policy.startedAt >= policy.maxMs) {
            policy.fatal = true;
            if (typeof appendAuditLog === 'function') appendAuditLog('Reconnect cap reached (' + policy.attempts + ' attempts)', 'fatal');
            if (window.S.pollTimer) { clearTimeout(window.S.pollTimer); window.S.pollTimer = null; }
            showReconnectFatal();
            return;
          }
        } else {
          policy.attempts = 0;
          policy.startedAt = 0;
          policy.fatal = false;
          if (typeof syncAuditUi === 'function') syncAuditUi();
        }
        return originalScheduleNextPoll.apply(this, arguments);
      };
      window.scheduleNextPoll.__rvtPhaseAlphaWrapped = true;
    }
    if (typeof window.manualReconnect === 'function' && !window.manualReconnect.__rvtPhaseAlphaWrapped) {
      var originalManualReconnect = window.manualReconnect;
      window.manualReconnect = function() {
        var policy = window.S.reconnectPolicy || (window.S.reconnectPolicy = {});
        policy.fatal = false;
        policy.attempts = 0;
        policy.startedAt = Date.now();
        var banner = document.getElementById('rvtReconnectFailed');
        if (banner) banner.classList.remove('show');
        if (typeof appendAuditLog === 'function') appendAuditLog('Manual reconnect triggered', 'info');
        return originalManualReconnect.apply(this, arguments);
      };
      window.manualReconnect.__rvtPhaseAlphaWrapped = true;
    }
    var retryBtn = document.getElementById('rvtReconnectRetryBtn');
    if (retryBtn && !retryBtn.__rvtPhaseAlphaReconnect) {
      retryBtn.__rvtPhaseAlphaReconnect = true;
      retryBtn.addEventListener('click', function() {
        if (window.S && window.S.reconnectPolicy) {
          window.S.reconnectPolicy.fatal = false;
          window.S.reconnectPolicy.attempts = 0;
          window.S.reconnectPolicy.startedAt = Date.now();
        }
      });
    }
  }

  function injectVersionAbout() {
    var settings = document.getElementById('view-settings');
    if (!settings || settings.querySelector('#rvtVersionAbout')) return;
    var target = settings.querySelector('.settings-page-grid, .settings-page, .set-g:last-child') || settings;
    var block = document.createElement('section');
    block.id = 'rvtVersionAbout';
    block.className = 'rvt-version-about';
    block.innerHTML =
      '<div class="set-head"><div class="set-copy"><strong>About</strong><span>Client, firmware, storage schema, and update status.</span></div></div>' +
      '<div class="rvt-version-about-grid">' +
        '<div class="rvt-version-about-item"><span>Dashboard</span><strong>' + CLIENT_VERSION.dashboard + '</strong></div>' +
        '<div class="rvt-version-about-item"><span>Trainer</span><strong>' + CLIENT_VERSION.trainer + '</strong></div>' +
        '<div class="rvt-version-about-item"><span>Firmware</span><strong>' + CLIENT_VERSION.firmware + '</strong></div>' +
        '<div class="rvt-version-about-item"><span>Storage schema</span><strong>' + CURRENT_SCHEMA_VERSION + '</strong></div>' +
      '</div>' +
      '<button type="button" class="ghost-btn" id="rvtCheckUpdateBtn"><span class="material-symbols-rounded">sync</span>Check version status</button>' +
      '<div class="settings-help" id="rvtVersionStatus">Version check has not run in this tab.</div>';
    target.appendChild(block);
    var btn = block.querySelector('#rvtCheckUpdateBtn');
    btn.addEventListener('click', function() {
      var status = block.querySelector('#rvtVersionStatus');
      status.textContent = 'Checking local control server...';
      fetch('/api/status', { cache: 'no-store' })
        .then(function(r) { return r.ok ? r.json() : Promise.reject(new Error('HTTP ' + r.status)); })
        .then(function(j) {
          var trainer = j.version || j.trainer_version || j.trainer || 'unknown';
          var firmware = j.firmware_version || j.firmware || 'unknown';
          status.textContent = 'Control server reports trainer ' + trainer + ' and firmware ' + firmware + '.';
        })
        .catch(function(err) {
          status.textContent = 'No local version endpoint is available.';
          softError('version status check failed', err);
        });
    });
  }

  function installI18nScaffold() {
    if (window.RVT_STRINGS && window.rvtT) return;
    var STRINGS = {
      en: {
        'alerts.count.zero': 'No alerts',
        'alerts.count.one': '1 alert',
        'alerts.count.many': '{n} alerts',
        'storage.quota': 'Storage is almost full',
        'start.waiting': 'Waiting for {deps} to finish',
        'vitals.hrCritical': 'Heart rate critical: {value}, exceeded {threshold} threshold',
        'vitals.rrCritical': 'Respiratory rate critical: {value}, exceeded {threshold} threshold'
      }
    };
    window.RVT_STRINGS = STRINGS;
    window.rvtT = function(key, vars) {
      vars = vars || {};
      var dict = STRINGS.en || {};
      var value = dict[key];
      if (!value) {
        softError('missing i18n key: ' + key, new Error(key));
        value = key;
      }
      return String(value).replace(/\{([a-z0-9_]+)\}/gi, function(_, name) {
        return Object.prototype.hasOwnProperty.call(vars, name) ? vars[name] : '';
      });
    };
    window.rvtT.format = window.rvtT;
    window.rvtT.coverage = function() {
      return { locale: 'en', strings: Object.keys(STRINGS.en).length, coverage_pct: 100 };
    };
  }

  /* --- PHASE BETA: CORE UX IMPROVEMENTS --- */
  const thresholdZonesPlugin = {
    id: 'thresholdZones',
    beforeDraw: function(chart) {
      if (!chart.options.plugins.thresholdZones?.enabled) return;
      const { ctx, chartArea: { top, bottom, left, right }, scales: { y } } = chart;
      const thresholds = (window.S && window.S.kpiThresholds) || { hrLow: 40, hrHigh: 140, rrLow: 6, rrHigh: 30 };
      let low, high;
      const key = chart.__rvtKey || '';
      if (key === 'hr' || key === 'kHr' || key === 'zoom_hr') {
        low = thresholds.hrLow; high = thresholds.hrHigh;
      } else if (key === 'rr' || key === 'kRr' || key === 'zoom_rr') {
        low = thresholds.rrLow; high = thresholds.rrHigh;
      } else return;

      ctx.save();
      const drawZone = (val, isHigh) => {
        const yP = y.getPixelForValue(val);
        if (yP <= bottom && yP >= top) {
          ctx.fillStyle = 'rgba(220, 38, 38, 0.06)';
          if (isHigh) ctx.fillRect(left, top, right - left, yP - top);
          else ctx.fillRect(left, yP, right - left, bottom - yP);
          ctx.strokeStyle = 'rgba(220, 38, 38, 0.25)';
          ctx.setLineDash([6, 4]);
          ctx.lineWidth = 1.5;
          ctx.beginPath(); ctx.moveTo(left, yP); ctx.lineTo(right, yP); ctx.stroke();
        }
      };
      if (num(low) !== null) drawZone(low, false);
      if (num(high) !== null) drawZone(high, true);
      ctx.restore();
    }
  };
  window.thresholdZonesPlugin = thresholdZonesPlugin;
  if (window.Chart) window.Chart.register(thresholdZonesPlugin);

  function setLiveMode(mode) {
    const requested = ['simple', 'advanced'].includes(mode) ? mode : 'simple';
    document.body.dataset.liveMode = requested;
    try { localStorage.setItem('rvt-live-mode', requested); } catch (_) { }
    document.querySelectorAll('[data-live-mode]').forEach(btn => {
      const active = btn.dataset.liveMode === requested;
      btn.classList.toggle('active', active);
      btn.setAttribute('aria-checked', active ? 'true' : 'false');
      btn.tabIndex = active ? 0 : -1;
    });
    if (typeof window.repaintCharts === 'function') {
      requestAnimationFrame(() => window.repaintCharts());
    }
  }

  function renderNextActionBanner() {
    const banner = document.getElementById('rvtNextActionBanner');
    if (!banner) return;
    const failures = (typeof window.structuralFailures === 'function') ? window.structuralFailures() : [];
    if (failures.length > 0) {
      banner.className = 'next-action-banner bad';
      banner.innerHTML = '<span class="material-symbols-rounded">error</span><div><strong>Action required:</strong> ' + failures.length + ' blocking issues detected. Fix them to enable session start.</div>';
      banner.style.display = 'flex';
      return;
    }
    const checks = (window.S && window.S.ctl && window.S.ctl.status && window.S.ctl.status.checks) || [];
    const warns = checks.filter(c => c.status === 'warn');
    if (warns.length > 0) {
      banner.className = 'next-action-banner warn';
      banner.innerHTML = '<span class="material-symbols-rounded">warning</span><div><strong>Preflight warning:</strong> ' + warns.length + ' advisory issues. Session can start, but results may be degraded.</div>';
      banner.style.display = 'flex';
      return;
    }
    banner.style.display = 'none';
  }

  function showKpiZoom(id) {
    if (window.S.charts.zoom) {
      if (typeof window.destroyChart === 'function') window.destroyChart(window.S.charts.zoom);
      else { window.S.charts.zoom.destroy(); if (window.__rvtChartRegistry) window.__rvtChartRegistry.delete(window.S.charts.zoom); }
      window.S.charts.zoom = null;
    }
    const old = document.getElementById('rvtZoomOverlay');
    if (old) old.remove();

    const overlay = document.createElement('div');
    overlay.id = 'rvtZoomOverlay';
    overlay.className = 'zoom-overlay';
    overlay.innerHTML = `
      <div class="zoom-card">
        <div class="zoom-head">
          <h3>Zoom: ${id.toUpperCase()}</h3>
          <button type="button" class="ic-btn" onclick="document.getElementById('rvtZoomOverlay').remove()" title="Close zoom">
            <span class="material-symbols-rounded">close</span>
          </button>
        </div>
        <div class="zoom-body">
          <canvas id="zoomChartCanvas"></canvas>
        </div>
      </div>
    `;
    document.body.appendChild(overlay);
    overlay.addEventListener('click', e => { if (e.target === overlay) overlay.remove(); });

    const source = (id === 'hr' ? window.S.charts.hr : (id === 'rr' ? window.S.charts.rr : null));
    if (!source) return;

    const ctx = document.getElementById('zoomChartCanvas').getContext('2d');
    const cfg = JSON.parse(JSON.stringify(source.config));
    cfg.options.maintainAspectRatio = false;
    cfg.options.plugins.thresholdZones = { enabled: true };
    cfg.options.plugins.legend.display = true;
    cfg.options.plugins.legend.position = 'bottom';

    const zoomChart = new Chart(ctx, cfg);
    zoomChart.__rvtKey = 'zoom_' + id;
    if (typeof window.registerChart === 'function') window.registerChart(zoomChart, zoomChart.__rvtKey);
    window.S.charts.zoom = zoomChart;
  }
  window.showKpiZoom = showKpiZoom;

  function initKpiZoom() {
    document.addEventListener('click', e => {
      const kpi = e.target.closest('.kpi[data-kpi-id]');
      if (!kpi || e.target.closest('button, .kpi-menu-btn')) return;
      showKpiZoom(kpi.dataset.kpiId);
    });
  }

  function bootPhaseBeta() {
    const savedMode = localStorage.getItem('rvt-live-mode') || 'simple';
    setLiveMode(savedMode);
    renderNextActionBanner();
    initKpiZoom();
  }

  function bootPhaseAlpha() {
    ensureRuntimeLists();
    migrateStorageSchema();
    installI18nScaffold();
    installStartGuard();
    installPayloadValidation();
    installRenderGuards();
    installChartLifecycle();
    installReconnectCap();
    ensureAnnouncer();
    injectVersionAbout();
    syncDropoutUi(window.S && window.S.lastPayload);
    syncStartGuard();
    if (typeof syncAuditUi === 'function') syncAuditUi();
    bootPhaseBeta();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(bootPhaseAlpha, 250); }, { once: true });
  } else {
    setTimeout(bootPhaseAlpha, 250);
  }
  setInterval(function() {
    installStartGuard();
    installPayloadValidation();
    installRenderGuards();
    installChartLifecycle();
    installReconnectCap();
    injectVersionAbout();
    syncDropoutUi(window.S && window.S.lastPayload);
    syncStartGuard();
    if (typeof syncAuditUi === 'function') syncAuditUi();
    renderNextActionBanner();
  }, 2000);

  // Export Beta functions
  window.setLiveMode = setLiveMode;
  window.renderNextActionBanner = renderNextActionBanner;
  window.showKpiZoom = showKpiZoom;
  window.bootPhaseBeta = bootPhaseBeta;
  window.bootPhaseAlpha = bootPhaseAlpha;
  window.syncStartGuard = syncStartGuard;
  window.installStartGuard = installStartGuard;

  window.bootPhaseAlpha = bootPhaseAlpha; window.bootPhaseBeta = bootPhaseBeta; window.showKpiZoom = showKpiZoom; window.setLiveMode = setLiveMode; window.renderNextActionBanner = renderNextActionBanner; window.syncStartGuard = syncStartGuard; window.installStartGuard = installStartGuard;
})();
;
/* END modules/rvt-phase-alpha-foundation-fixes-js.js */

/* BEGIN modules/rvt-beta-current-operator-console-js.js */
(function(){
  'use strict';

  var WAVE_SYNC_KEY = 'rvt-wave-sync';
  var WAVE_SYNC_CHART_KEYS = ['hr', 'rr', 'breath', 'heart', 'rrDiag'];
  var syncingZoom = false;

  function softWarn(scope, err) {
    try {
      if (typeof window.rvtSoftError === 'function') window.rvtSoftError(scope, err, { always: true });
      else console.warn('[RVT beta]', scope, err);
    } catch (_) {}
  }

  function storageGetJson(key, fallback) {
    try {
      var raw = localStorage.getItem(key);
      return raw ? JSON.parse(raw) : fallback;
    } catch (_) {
      return fallback;
    }
  }

  function storageSetJson(key, value) {
    try {
      if (window.rvtStorage && typeof window.rvtStorage.setJson === 'function') return window.rvtStorage.setJson(key, value);
      localStorage.setItem(key, JSON.stringify(value));
      return true;
    } catch (err) {
      softWarn('storage set failed: ' + key, err);
      return false;
    }
  }

  function escBeta(value) {
    return String(value == null ? '' : value).replace(/[&<>"']/g, function(ch) {
      return ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[ch];
    });
  }

  function waveSyncEnabled() {
    try { return localStorage.getItem(WAVE_SYNC_KEY) !== '0'; } catch (_) { return true; }
  }

  function waveCharts() {
    var charts = [];
    var state = window.S && window.S.charts ? window.S.charts : {};
    WAVE_SYNC_CHART_KEYS.forEach(function(key) {
      var chart = state[key];
      if (chart && chart.canvas && chart.scales && chart.scales.x) charts.push(chart);
    });
    return charts;
  }

  function chartKey(chart) {
    if (!chart) return '';
    if (chart.__rvtKey) return String(chart.__rvtKey);
    var entries = window.S && window.S.charts ? Object.entries(window.S.charts) : [];
    for (var i = 0; i < entries.length; i += 1) {
      if (entries[i][1] === chart) return entries[i][0];
    }
    return chart.canvas && chart.canvas.id ? chart.canvas.id : '';
  }

  function isWaveSyncChart(chartOrId) {
    var id = typeof chartOrId === 'string' ? chartOrId : chartKey(chartOrId);
    return ['hr', 'rr', 'breath', 'heart', 'rrDiag', 'hrChart', 'rrChart', 'breathChart', 'heartChart', 'rrDiagChart'].includes(id);
  }

  function chartXBounds(chart) {
    if (!chart || !chart.scales || !chart.scales.x) return null;
    var scale = chart.scales.x;
    var min = scale.min;
    var max = scale.max;
    if ((min == null || max == null) && chart.options && chart.options.scales && chart.options.scales.x) {
      min = chart.options.scales.x.min;
      max = chart.options.scales.x.max;
    }
    if (min == null || max == null) return null;
    return { min: min, max: max };
  }

  function setChartXBounds(chart, bounds) {
    if (!chart || !bounds) return;
    chart.options = chart.options || {};
    chart.options.scales = chart.options.scales || {};
    chart.options.scales.x = chart.options.scales.x || {};
    chart.options.scales.x.min = bounds.min;
    chart.options.scales.x.max = bounds.max;
    try { chart.update('none'); } catch (err) { softWarn('sync chart zoom update failed', err); }
    try { if (typeof window.syncChartZoomUi === 'function') window.syncChartZoomUi(chart, true); } catch (_) {}
  }

  function clearChartXBounds(chart) {
    if (!chart) return;
    if (chart.options && chart.options.scales && chart.options.scales.x) {
      delete chart.options.scales.x.min;
      delete chart.options.scales.x.max;
    }
    try {
      if (typeof chart.resetZoom === 'function') chart.resetZoom('none');
      else chart.update('none');
    } catch (err) {
      softWarn('reset synced chart zoom failed', err);
    }
    try { if (typeof window.syncChartZoomUi === 'function') window.syncChartZoomUi(chart, false); } catch (_) {}
  }

  function syncZoomFrom(source) {
    if (syncingZoom || !waveSyncEnabled() || !isWaveSyncChart(source)) return;
    var bounds = chartXBounds(source);
    if (!bounds) return;
    syncingZoom = true;
    try {
      waveCharts().forEach(function(chart) {
        if (chart !== source) setChartXBounds(chart, bounds);
      });
    } finally {
      syncingZoom = false;
    }
  }

  function wrapZoomCallback(obj, name) {
    if (!obj || obj[name] && obj[name].__rvtWaveSyncWrapped) return;
    var original = obj && obj[name];
    obj[name] = function(ctx) {
      var result;
      if (typeof original === 'function') result = original.apply(this, arguments);
      var chart = (ctx && ctx.chart) || this && this.chart;
      if (chart) syncZoomFrom(chart);
      return result;
    };
    obj[name].__rvtWaveSyncWrapped = true;
  }

  function patchZoomOptions(options) {
    if (!options) return options;
    options.pan = options.pan || {};
    options.zoom = options.zoom || {};
    wrapZoomCallback(options.pan, 'onPan');
    wrapZoomCallback(options.pan, 'onPanComplete');
    wrapZoomCallback(options.zoom, 'onZoom');
    wrapZoomCallback(options.zoom, 'onZoomComplete');
    return options;
  }

  function patchChartZoom(chart) {
    if (!chart || !isWaveSyncChart(chart)) return;
    var zoom = chart.config && chart.config.options && chart.config.options.plugins && chart.config.options.plugins.zoom;
    if (zoom) patchZoomOptions(zoom);
  }

  function patchExistingCharts() {
    waveCharts().forEach(patchChartZoom);
  }

  function syncWaveToggleUi() {
    var btn = document.getElementById('rvtWaveSyncToggle');
    var on = waveSyncEnabled();
    if (!btn) return;
    btn.dataset.active = on ? '1' : '0';
    btn.setAttribute('aria-checked', on ? 'true' : 'false');
    btn.title = on ? 'Sync zoom is on' : 'Sync zoom is off';
  }

  function setWaveSync(on) {
    try { localStorage.setItem(WAVE_SYNC_KEY, on ? '1' : '0'); } catch (_) {}
    syncWaveToggleUi();
    patchExistingCharts();
    if (on) {
      var charts = waveCharts();
      var source = charts.find(function(chart){ return chartXBounds(chart); });
      if (source) syncZoomFrom(source);
    }
  }

  function injectWaveSyncToggle() {
    var actions = document.querySelector('#tab-waves > article .ch .ca');
    if (!actions || document.getElementById('rvtWaveSyncToggle')) return;
    var btn = document.createElement('button');
    btn.id = 'rvtWaveSyncToggle';
    btn.className = 'ca-btn rvt-wave-sync-toggle';
    btn.type = 'button';
    btn.setAttribute('role', 'switch');
    btn.setAttribute('aria-label', 'Sync waveform zoom');
    btn.innerHTML = '<span class="material-symbols-rounded">sync_alt</span><span class="sr-only">Sync zoom</span>';
    btn.addEventListener('click', function() { setWaveSync(!waveSyncEnabled()); });
    actions.insertBefore(btn, actions.firstChild);
    syncWaveToggleUi();
  }

  function installWaveSyncWrappers() {
    if (typeof window.chartZoomOptions === 'function' && !window.chartZoomOptions.__rvtWaveSyncWrapped) {
      var originalChartZoomOptions = window.chartZoomOptions;
      window.chartZoomOptions = function() {
        return patchZoomOptions(originalChartZoomOptions.apply(this, arguments));
      };
      window.chartZoomOptions.__rvtWaveSyncWrapped = true;
    }
    if (typeof window.buildCharts === 'function' && !window.buildCharts.__rvtWaveSyncWrapped) {
      var originalBuildCharts = window.buildCharts;
      window.buildCharts = function() {
        var result = originalBuildCharts.apply(this, arguments);
        setTimeout(function(){ patchExistingCharts(); injectWaveSyncToggle(); }, 0);
        return result;
      };
      window.buildCharts.__rvtWaveSyncWrapped = true;
    }
    if (typeof window.resetLiveChartZooms === 'function' && !window.resetLiveChartZooms.__rvtWaveSyncWrapped) {
      var originalResetLive = window.resetLiveChartZooms;
      window.resetLiveChartZooms = function() {
        var result = originalResetLive.apply(this, arguments);
        waveCharts().forEach(clearChartXBounds);
        return result;
      };
      window.resetLiveChartZooms.__rvtWaveSyncWrapped = true;
    }
    if (typeof window.resetChartZoom === 'function' && !window.resetChartZoom.__rvtWaveSyncWrapped) {
      var originalResetChart = window.resetChartZoom;
      window.resetChartZoom = function(canvasId) {
        if (waveSyncEnabled() && isWaveSyncChart(canvasId)) {
          waveCharts().forEach(clearChartXBounds);
          return;
        }
        return originalResetChart.apply(this, arguments);
      };
      window.resetChartZoom.__rvtWaveSyncWrapped = true;
    }
  }

  var ALERT_FILTER_KEY = 'rvt-alerts-filter';
  var ALERT_FILTERS = { severity: [], source: [] };

  function loadAlertFilters() {
    var saved = storageGetJson(ALERT_FILTER_KEY, {});
    ALERT_FILTERS = {
      severity: Array.isArray(saved.severity) ? saved.severity.filter(Boolean) : [],
      source: Array.isArray(saved.source) ? saved.source.filter(Boolean) : []
    };
  }

  function saveAlertFilters() {
    storageSetJson(ALERT_FILTER_KEY, ALERT_FILTERS);
  }

  function hasAlertFilters() {
    return ALERT_FILTERS.severity.length > 0 || ALERT_FILTERS.source.length > 0;
  }

  function severityName(card) {
    if (card.classList.contains('bad')) return 'critical';
    if (card.classList.contains('good')) return 'info';
    return 'warning';
  }

  function sourceNameFromCard(card) {
    var text = (card.textContent || '').toLowerCase();
    var meta = (card.querySelector('.dv-alert-meta span') || {}).textContent || '';
    meta = meta.toLowerCase();
    if (/runtime|script|promise/.test(text)) return 'runtime';
    if (/ble|reference|pulse|oximeter/.test(meta + ' ' + text)) return 'reference';
    if (/radar|serial|sensor|frame/.test(meta + ' ' + text)) return 'radar';
    return 'system';
  }

  function alertFilterChip(kind, value, label, count) {
    var active = ALERT_FILTERS[kind].includes(value);
    return '<button type="button" class="rvt-alert-filter-chip" data-kind="' + kind + '" data-value="' + value + '" aria-pressed="' + (active ? 'true' : 'false') + '">' +
      escBeta(label) + (Number.isFinite(count) ? ' ' + count : '') +
      '</button>';
  }

  function renderAlertFilterBar() {
    var body = document.getElementById('dvBody');
    if (!body) return;
    var cards = Array.from(body.querySelectorAll('.dv-alert-card'));
    cards.forEach(function(card) {
      card.dataset.alertSeverity = severityName(card);
      card.dataset.alertSource = sourceNameFromCard(card);
    });
    var counts = { critical: 0, warning: 0, info: 0, radar: 0, reference: 0, runtime: 0, system: 0 };
    cards.forEach(function(card) {
      counts[card.dataset.alertSeverity] = (counts[card.dataset.alertSeverity] || 0) + 1;
      counts[card.dataset.alertSource] = (counts[card.dataset.alertSource] || 0) + 1;
    });
    var existing = document.getElementById('rvtAlertFilterBar');
    if (existing) existing.remove();
    var bar = document.createElement('div');
    bar.id = 'rvtAlertFilterBar';
    bar.className = 'rvt-alert-filterbar';
    bar.setAttribute('role', 'group');
    bar.setAttribute('aria-label', 'Alert filters');
    bar.innerHTML =
      '<div class="rvt-alert-filter-label">Filter alerts</div>' +
      alertFilterChip('severity', 'critical', 'Critical', counts.critical) +
      alertFilterChip('severity', 'warning', 'Warning', counts.warning) +
      alertFilterChip('severity', 'info', 'Info', counts.info) +
      alertFilterChip('source', 'radar', 'Radar', counts.radar) +
      alertFilterChip('source', 'reference', 'Reference', counts.reference) +
      alertFilterChip('source', 'runtime', 'Runtime', counts.runtime) +
      alertFilterChip('source', 'system', 'System', counts.system) +
      '<span class="rvt-alert-filter-count" id="rvtAlertFilterCount"></span>' +
      '<button type="button" class="rvt-alert-filter-clear" id="rvtAlertFilterClear"' + (hasAlertFilters() ? '' : ' hidden') + '>Clear</button>' +
      '<span class="sr-only" id="rvtAlertFilterLive" aria-live="polite"></span>';
    body.insertBefore(bar, body.firstChild);
    bar.querySelectorAll('.rvt-alert-filter-chip').forEach(function(btn) {
      btn.addEventListener('click', function() {
        var kind = btn.dataset.kind;
        var value = btn.dataset.value;
        var list = ALERT_FILTERS[kind];
        if (!Array.isArray(list)) list = ALERT_FILTERS[kind] = [];
        if (list.includes(value)) ALERT_FILTERS[kind] = list.filter(function(v){ return v !== value; });
        else ALERT_FILTERS[kind] = list.concat(value);
        saveAlertFilters();
        renderAlertFilterBar();
        applyAlertFilters();
      });
    });
    var clear = document.getElementById('rvtAlertFilterClear');
    if (clear) clear.addEventListener('click', function() {
      ALERT_FILTERS = { severity: [], source: [] };
      saveAlertFilters();
      renderAlertFilterBar();
      applyAlertFilters();
    });
  }

  function applyAlertFilters() {
    var body = document.getElementById('dvBody');
    if (!body) return;
    var cards = Array.from(body.querySelectorAll('.dv-alert-card'));
    var visible = 0;
    cards.forEach(function(card) {
      var sev = card.dataset.alertSeverity || severityName(card);
      var src = card.dataset.alertSource || sourceNameFromCard(card);
      var severityOk = !ALERT_FILTERS.severity.length || ALERT_FILTERS.severity.includes(sev);
      var sourceOk = !ALERT_FILTERS.source.length || ALERT_FILTERS.source.includes(src);
      var show = severityOk && sourceOk;
      card.hidden = !show;
      if (show) visible += 1;
    });
    var count = document.getElementById('rvtAlertFilterCount');
    if (count) count.textContent = 'Showing ' + visible + ' of ' + cards.length;
    var clear = document.getElementById('rvtAlertFilterClear');
    if (clear) clear.hidden = !hasAlertFilters();
    var live = document.getElementById('rvtAlertFilterLive');
    if (live) live.textContent = 'Alert filters updated. Showing ' + visible + ' of ' + cards.length + ' alerts.';
  }

  function installAlertFilterWrapper() {
    loadAlertFilters();
    if (typeof window.renderAlerts === 'function' && !window.renderAlerts.__rvtAlertFilterWrapped) {
      var originalRenderAlerts = window.renderAlerts;
      window.renderAlerts = function() {
        var result = originalRenderAlerts.apply(this, arguments);
        try {
          renderAlertFilterBar();
          applyAlertFilters();
        } catch (err) {
          softWarn('alert filter render failed', err);
        }
        return result;
      };
      window.renderAlerts.__rvtAlertFilterWrapped = true;
    }
    if (typeof window.openDrawer === 'function' && !window.openDrawer.__rvtAlertFilterWrapped) {
      var originalOpenDrawer = window.openDrawer;
      window.openDrawer = function() {
        var result = originalOpenDrawer.apply(this, arguments);
        setTimeout(function(){ renderAlertFilterBar(); applyAlertFilters(); }, 0);
        setTimeout(function(){ renderAlertFilterBar(); applyAlertFilters(); }, 80);
        return result;
      };
      window.openDrawer.__rvtAlertFilterWrapped = true;
    }
  }

  var PIPELINE_NODES = [
    { key: 'python', label: 'Python', icon: 'terminal', match: /python|api|dependency|trainer|server/ },
    { key: 'firmware', label: 'Firmware', icon: 'memory', match: /firmware|module|sketch|contract|schema/ },
    { key: 'serial', label: 'Serial', icon: 'usb', match: /serial|port|radar|com/ },
    { key: 'ble', label: 'BLE', icon: 'bluetooth', match: /ble|bluetooth|address|reference|oximeter/ }
  ];
  var preflightSeen = {};

  function currentPreflightRows() {
    try {
      if (typeof window.preflightRows === 'function') return window.preflightRows();
    } catch (_) {}
    return window.S && window.S.ctl && window.S.ctl.preflight && Array.isArray(window.S.ctl.preflight.checks)
      ? window.S.ctl.preflight.checks
      : [];
  }

  function normalizedStatus(status) {
    var s = String(status || 'pending').toLowerCase();
    if (['ok', 'pass', 'passed', 'good'].includes(s)) return 'good';
    if (['fail', 'failed', 'bad', 'critical'].includes(s)) return 'bad';
    if (['warn', 'warning', 'skip'].includes(s)) return 'warn';
    if (['running', 'loading'].includes(s)) return 'running';
    return 'pending';
  }

  function worstStatus(statuses) {
    if (statuses.includes('bad')) return 'bad';
    if (statuses.includes('warn')) return 'warn';
    if (statuses.includes('running')) return 'running';
    if (statuses.includes('pending')) return 'pending';
    return statuses.length ? 'good' : 'pending';
  }

  function statusIcon(status) {
    return ({ good: 'check_circle', bad: 'error', warn: 'warning', running: 'progress_activity', pending: 'radio_button_unchecked' })[status] || 'radio_button_unchecked';
  }

  function statusText(status) {
    return ({ good: 'Pass', bad: 'Fail', warn: 'Warn', running: 'Running', pending: 'Pending' })[status] || 'Pending';
  }

  function pipelineNodeState(node, rows) {
    var matched = rows.filter(function(row) {
      var id = String(row.id || row.key || '').toLowerCase();
      var label = String(row.label || '').toLowerCase();
      return node.match.test(id + ' ' + label);
    });
    var statuses = matched.map(function(row){ return normalizedStatus(row.status); });
    var state = worstStatus(statuses);
    var primary = matched.find(function(row){ return normalizedStatus(row.status) === state; }) || matched[0] || null;
    matched.forEach(function(row) {
      var id = row.id || row.key || node.key;
      var sig = String(row.status || '') + '|' + String(row.detail || '') + '|' + String(row.remediation || '');
      if (preflightSeen[id] !== sig) {
        preflightSeen[id] = sig;
        row.__rvtSeenAt = Date.now();
      }
    });
    var ts = primary && (primary.checked_at || primary.updated_at || primary.ts || primary.__rvtSeenAt);
    var stamp = ts ? new Date(ts).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' }) : 'not checked';
    return { state: state, checkId: primary && (primary.id || primary.key), stamp: stamp };
  }

  function renderPreflightPipeline() {
    var grid = document.getElementById('pfGrid');
    if (!grid) return;
    var card = grid.closest('.sys-preflight') || grid.parentElement;
    if (!card) return;
    var rows = currentPreflightRows();
    var states = PIPELINE_NODES.map(function(node){ return pipelineNodeState(node, rows); });
    var completed = states.filter(function(s){ return ['good', 'warn', 'bad'].includes(s.state); }).length;
    var worst = worstStatus(states.map(function(s){ return s.state; }));
    var pct = Math.round((completed / PIPELINE_NODES.length) * 100);
    var existing = document.getElementById('rvtQuickcheckPipeline');
    if (!existing) {
      existing = document.createElement('div');
      existing.id = 'rvtQuickcheckPipeline';
      existing.className = 'rvt-preflight-pipeline';
      existing.setAttribute('aria-label', 'Quickcheck pipeline');
      card.insertBefore(existing, grid);
    }
    existing.dataset.worst = worst;
    existing.style.setProperty('--rvt-pipeline-progress', pct + '%');
    existing.innerHTML =
      '<div class="rvt-pipeline-track" aria-hidden="true"><span class="rvt-pipeline-fill"></span></div>' +
      PIPELINE_NODES.map(function(node, idx) {
        var state = states[idx];
        return '<button type="button" class="rvt-pipeline-node" data-node="' + node.key + '" data-check-id="' + escBeta(state.checkId || '') + '" data-state="' + state.state + '" title="Last check: ' + escBeta(state.stamp) + '">' +
          '<span class="material-symbols-rounded">' + statusIcon(state.state) + '</span>' +
          '<strong>' + escBeta(node.label) + '</strong>' +
          '<small>' + statusText(state.state) + '</small>' +
        '</button>';
      }).join('');
    existing.querySelectorAll('.rvt-pipeline-node').forEach(function(btn) {
      btn.addEventListener('click', function() {
        var id = btn.dataset.checkId;
        if (id && typeof window.runOnePreflight === 'function') window.runOnePreflight(id);
      });
    });
  }

  function installPreflightPipelineWrapper() {
    if (typeof window.renderPreflight === 'function' && !window.renderPreflight.__rvtPipelineWrapped) {
      var originalRenderPreflight = window.renderPreflight;
      window.renderPreflight = function() {
        var result = originalRenderPreflight.apply(this, arguments);
        try { renderPreflightPipeline(); } catch (err) { softWarn('preflight pipeline render failed', err); }
        return result;
      };
      window.renderPreflight.__rvtPipelineWrapped = true;
    }
  }

  function bootBetaCurrent() {
    installWaveSyncWrappers();
    patchExistingCharts();
    injectWaveSyncToggle();
    syncWaveToggleUi();
    installAlertFilterWrapper();
    installPreflightPipelineWrapper();
    try { renderPreflightPipeline(); } catch (_) {}
    try { renderAlertFilterBar(); applyAlertFilters(); } catch (_) {}
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(bootBetaCurrent, 300); }, { once: true });
  } else {
    setTimeout(bootBetaCurrent, 300);
  }
  var betaBootRetries = 0;
  var betaBootTimer = setInterval(function() {
    betaBootRetries += 1;
    bootBetaCurrent();
    if (betaBootRetries >= 32) clearInterval(betaBootTimer);
  }, 250);
  /* PERF-08: uncapped interval removed — the retrying betaBootTimer above handles boot, and bootBetaCurrent is called by render() once live. */

  window.rvtSetWaveSync = setWaveSync;
  window.rvtSyncWaveZoomFrom = syncZoomFrom;
  window.rvtRenderAlertFilterBar = function() { installAlertFilterWrapper(); return renderAlertFilterBar(); };
  window.rvtApplyAlertFilters = function() { installAlertFilterWrapper(); return applyAlertFilters(); };
  window.rvtRenderPreflightPipeline = renderPreflightPipeline;
})();
;
/* END modules/rvt-beta-current-operator-console-js.js */

/* BEGIN modules/rvt-waves-funnels-beta-js.js */
(function(){
  'use strict';

  var LIMIT = 900;
  var state = window.__rvtWavesFunnelsBeta || {
    samples: [],
    seq: 0,
    lastSig: '',
    miniStart: null,
    miniEnd: null,
    menu: null
  };
  window.__rvtWavesFunnelsBeta = state;

  var CHART_IDS = {
    breathChart: 'breath',
    heartChart: 'heart',
    hrChart: 'hr',
    rrChart: 'rr'
  };

  function n(v) {
    var x = Number(v);
    return Number.isFinite(x) ? x : null;
  }

  function esc(v) {
    return String(v == null ? '' : v).replace(/[&<>"']/g, function(ch) {
      return ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[ch];
    });
  }

  function read(obj, paths) {
    for (var i = 0; i < paths.length; i += 1) {
      var cur = obj;
      var parts = paths[i].split('.');
      for (var j = 0; j < parts.length && cur != null; j += 1) cur = cur[parts[j]];
      if (cur !== undefined && cur !== null && cur !== '') return cur;
    }
    return undefined;
  }

  function boolFalse(v) {
    if (v === false) return true;
    if (v === true || v == null || v === '') return false;
    var s = String(v).trim().toLowerCase();
    return ['0', 'false', 'no', 'n', 'off', 'fail', 'failed', 'invalid', 'bad', 'rejected'].includes(s);
  }

  function truthState(p) {
    var disc = n(window.S && window.S.disc);
    var stale = !!(p && (p.stale || p.sensor_stale || read(p, ['meta.stale', 'status.stale'])));
    var invalid = !!(p && (p.invalid || p.schema_invalid || read(p, ['meta.invalid', 'status.invalid'])));
    if ((disc != null && disc > 0) || stale || invalid) return 'bad';
    return 'ok';
  }

  function sqiState(value) {
    if (value == null || value === '') return 'unknown';
    var num = n(value);
    if (num != null) {
      if (num > 1 && num <= 100) num = num / 100;
      if (num < .30) return 'bad';
      if (num < .60) return 'warn';
      return 'good';
    }
    var s = String(value).toLowerCase();
    if (/reject|bad|invalid|stale|drop|blocked|none/.test(s)) return 'bad';
    if (/warn|weak|low|suspect|motion/.test(s)) return 'warn';
    if (/good|ok|valid|fresh|pass/.test(s)) return 'good';
    return 'unknown';
  }

  function combineState(a, b) {
    if (a === 'bad' || b === 'bad') return 'bad';
    if (a === 'warn' || b === 'warn') return 'warn';
    if (a === 'good' || b === 'good') return 'good';
    return 'unknown';
  }

  function reasonText(p, kind) {
    var fields = kind === 'heart'
      ? ['hr_reject_reason', 'hr_gate_reason', 'hr_publish_reason', 'hr_block_stage', 'gate_reason', 'publish_reason', 'block_stage']
      : ['rr_reject_reason', 'rr_gate_reason', 'rr_publish_reason', 'rr_block_stage', 'gate_reason', 'publish_reason', 'block_stage'];
    var v = read(p || {}, fields);
    return v == null ? '' : String(v);
  }

  function waveState(p, kind) {
    if (!p) return truthState(p);
    var global = truthState(p);
    if (global === 'bad') return 'bad';
    var sqi = kind === 'heart'
      ? read(p, ['pqi_heart', 'heart_pqi', 'heart.pqi', 'hr_sqi', 'sqi_heart', 'quality.heart'])
      : read(p, ['pqi_breath', 'breath_pqi', 'breath.pqi', 'rr_sqi', 'sqi_breath', 'quality.breath']);
    var bySqi = sqiState(sqi);
    var valid = kind === 'heart'
      ? read(p, ['hr_valid', 'logged_hr_valid', 'heart_valid', 'phase_valid'])
      : read(p, ['rr_valid', 'logged_rr_valid', 'breath_valid', 'phase_valid']);
    var byValid = boolFalse(valid) ? 'bad' : 'unknown';
    var reject = kind === 'heart'
      ? read(p, ['reject_phase_rejected', 'coherence_rejected', 'hr_rejected', 'hr_blocked', 'raw_high_bias_suspect'])
      : read(p, ['rr_rejected', 'rr_blocked', 'rr_recovery_blocked', 're_anchor_blocked']);
    var byReject = reject === true || /reject|block|bad|invalid|suspect/i.test(String(reject || '')) ? 'bad' : 'unknown';
    var src = kind === 'heart'
      ? read(p, ['hr_publish_source', 'publish_source_hr', 'hr_source'])
      : read(p, ['rr_publish_source', 'publish_source_rr', 'rr_source']);
    var bySource = /none|invalid|stale|blocked|internal_only/i.test(String(src || '')) ? 'warn' : 'unknown';
    return combineState(combineState(bySqi, byValid), combineState(byReject, bySource));
  }

  function payloadTime(p) {
    var t = n(read(p || {}, ['meta.elapsed_s', 'elapsed_s', 'session.elapsed_s', 't', 'time_s']));
    if (t != null) return t;
    var last = state.samples.length ? state.samples[state.samples.length - 1].t : 0;
    return last + 1;
  }

  function alertActive() {
    var alerts = (window.S && Array.isArray(window.S.lastAlerts)) ? window.S.lastAlerts : [];
    return alerts.some(function(a) {
      return /bad|warn|critical|error|fail/i.test(String(a && (a.severity || a.status || a.title || '')));
    });
  }

  function recordPayload(p) {
    p = p || (window.S && (window.S.lastLivePayload || window.S.lastPayload)) || null;
    var t = payloadTime(p);
    var sig = [
      t,
      read(p || {}, ['meta.revision', 'revision', 'rows']),
      read(p || {}, ['pqi_heart', 'pqi_breath', 'hr_valid', 'rr_valid', 'hr_block_stage', 'rr_block_stage']),
      truthState(p)
    ].join('|');
    if (sig === state.lastSig) return state.samples;
    state.lastSig = sig;
    var heart = waveState(p, 'heart');
    var breath = waveState(p, 'breath');
    var sample = {
      t: t,
      seq: ++state.seq,
      heart: heart,
      breath: breath,
      state: combineState(heart, breath),
      heartReason: reasonText(p, 'heart'),
      breathReason: reasonText(p, 'breath'),
      alert: alertActive(),
      payload: p || {}
    };
    state.samples.push(sample);
    if (state.samples.length > LIMIT) state.samples.splice(0, state.samples.length - LIMIT);
    return state.samples;
  }

  function chartRange(key) {
    var chart = window.S && window.S.charts && window.S.charts[key];
    var x = chart && (chart.scales && chart.scales.x || chart.options && chart.options.scales && chart.options.scales.x);
    var min = n(x && x.min);
    var max = n(x && x.max);
    if (min != null && max != null && max > min) return [min, max];
    var samples = state.samples;
    if (!samples.length) return [0, 1];
    var end = samples[samples.length - 1].t;
    var start = Math.max(samples[0].t, end - Math.max(10, n(window.S && window.S.rangeS) || 120));
    return [start, end || start + 1];
  }

  function fullRange() {
    var samples = state.samples;
    if (!samples.length) return [0, 1];
    var a = samples[0].t;
    var b = samples[samples.length - 1].t;
    return b > a ? [a, b] : [a, a + 1];
  }

  function ensureHatchLayer(canvas) {
    if (!canvas) return null;
    var wrap = canvas.parentElement;
    if (!wrap) return null;
    if (getComputedStyle(wrap).position === 'static') wrap.style.position = 'relative';
    var layer = wrap.querySelector('.rvt-wave-hatch-layer[data-for="' + canvas.id + '"]');
    if (!layer) {
      layer = document.createElement('div');
      layer.className = 'rvt-wave-hatch-layer';
      layer.dataset.for = canvas.id;
      layer.setAttribute('aria-hidden', 'true');
      wrap.appendChild(layer);
    }
    layer.style.left = canvas.offsetLeft + 'px';
    layer.style.top = canvas.offsetTop + 'px';
    layer.style.width = Math.max(1, canvas.clientWidth || canvas.offsetWidth) + 'px';
    layer.style.height = Math.max(1, canvas.clientHeight || canvas.offsetHeight) + 'px';
    return layer;
  }

  function buildSegments(kind, range) {
    var samples = state.samples;
    if (!samples.length) return [];
    var min = range[0], max = range[1];
    var span = Math.max(.001, max - min);
    var points = samples.filter(function(s) { return s.t >= min && s.t <= max; });
    if (!points.length && samples.length) points = [samples[samples.length - 1]];
    var out = [];
    var cur = null;
    points.forEach(function(s, idx) {
      var st = s[kind] || 'unknown';
      var bad = st === 'bad' || st === 'warn';
      var nextT = points[idx + 1] ? points[idx + 1].t : Math.min(max, s.t + span / Math.max(30, points.length || 1));
      if (!bad) {
        if (cur) { out.push(cur); cur = null; }
        return;
      }
      if (!cur || cur.state !== st) {
        if (cur) out.push(cur);
        cur = { state: st, start: s.t, end: nextT };
      } else {
        cur.end = nextT;
      }
    });
    if (cur) out.push(cur);
    return out.map(function(seg) {
      return {
        state: seg.state,
        left: Math.max(0, Math.min(100, ((seg.start - min) / span) * 100)),
        width: Math.max(1, Math.min(100, ((seg.end - seg.start) / span) * 100))
      };
    });
  }

  function renderWaveHatches() {
    [['breathChart', 'breath'], ['heartChart', 'heart']].forEach(function(pair) {
      var canvas = document.getElementById(pair[0]);
      var layer = ensureHatchLayer(canvas);
      if (!layer) return;
      var range = chartRange(pair[1]);
      var segs = buildSegments(pair[1], range);
      layer.innerHTML = segs.map(function(seg) {
        return '<span class="rvt-wave-hatch-seg" data-state="' + seg.state + '" style="left:' +
          seg.left.toFixed(3) + '%;width:' + seg.width.toFixed(3) + '%"></span>';
      }).join('');
      var label = document.getElementById(pair[1] + 'SqiLabel');
      if (label && state.samples.length) {
        var latest = state.samples[state.samples.length - 1][pair[1]];
        if (latest === 'bad') label.textContent = 'SQI rejected or invalid';
        else if (latest === 'warn') label.textContent = 'SQI low or policy-blocked';
        else if (latest === 'good') label.textContent = 'SQI accepted';
      }
    });
  }

  function ensureMiniMap() {
    var existing = document.getElementById('rvtSessionMiniMap');
    if (existing) return existing;
    var wg = document.querySelector('#tab-waves .wg');
    if (!wg || !wg.parentNode) return null;
    var mini = document.createElement('div');
    mini.id = 'rvtSessionMiniMap';
    mini.className = 'rvt-session-minimap';
    mini.innerHTML =
      '<div class="rvt-minimap-head"><span>Session mini-map</span><span id="rvtMiniMapRange">Waiting</span></div>' +
      '<div class="rvt-minimap-track" id="rvtMiniMapTrack" tabindex="0" role="slider" aria-label="Session viewport" aria-valuemin="0" aria-valuemax="100" aria-valuenow="100"></div>';
    wg.parentNode.insertBefore(mini, wg.nextSibling);
    var track = mini.querySelector('#rvtMiniMapTrack');
    track.addEventListener('keydown', function(ev) {
      if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(ev.key)) return;
      ev.preventDefault();
      var all = fullRange();
      var span = all[1] - all[0];
      var cur = [state.miniStart, state.miniEnd];
      if (cur[0] == null || cur[1] == null) cur = chartRange('breath');
      var width = Math.max(1, cur[1] - cur[0]);
      var step = Math.max(1, span * .05);
      if (ev.key === 'Home') cur = [all[0], Math.min(all[1], all[0] + width)];
      else if (ev.key === 'End') cur = [Math.max(all[0], all[1] - width), all[1]];
      else if (ev.shiftKey && ev.key === 'ArrowLeft') cur = [cur[0], Math.max(cur[0] + 1, cur[1] - step)];
      else if (ev.shiftKey && ev.key === 'ArrowRight') cur = [cur[0], Math.min(all[1], cur[1] + step)];
      else {
        var delta = ev.key === 'ArrowLeft' ? -step : step;
        cur = [cur[0] + delta, cur[1] + delta];
        if (cur[0] < all[0]) cur = [all[0], all[0] + width];
        if (cur[1] > all[1]) cur = [all[1] - width, all[1]];
      }
      setMiniViewport(cur[0], cur[1]);
    });
    track.addEventListener('click', function(ev) {
      var all = fullRange();
      var rect = track.getBoundingClientRect();
      var pct = (ev.clientX - rect.left) / Math.max(1, rect.width);
      var cur = [state.miniStart, state.miniEnd];
      if (cur[0] == null || cur[1] == null) cur = chartRange('breath');
      var width = Math.max(1, cur[1] - cur[0]);
      var center = all[0] + pct * (all[1] - all[0]);
      setMiniViewport(center - width / 2, center + width / 2);
    });
    return mini;
  }

  function setMiniViewport(start, end) {
    var all = fullRange();
    var width = Math.max(1, end - start);
    start = Math.max(all[0], Math.min(start, all[1] - width));
    end = Math.min(all[1], start + width);
    state.miniStart = start;
    state.miniEnd = end;
    ['breath', 'heart'].forEach(function(key) {
      var chart = window.S && window.S.charts && window.S.charts[key];
      if (!chart) return;
      chart.options = chart.options || {};
      chart.options.scales = chart.options.scales || {};
      chart.options.scales.x = chart.options.scales.x || {};
      chart.options.scales.x.min = start;
      chart.options.scales.x.max = end;
      try { chart.update('none'); } catch (_) {}
    });
    renderMiniMap();
    renderWaveHatches();
  }

  function renderMiniMap() {
    var mini = ensureMiniMap();
    if (!mini) return;
    var track = mini.querySelector('#rvtMiniMapTrack');
    var label = mini.querySelector('#rvtMiniMapRange');
    var samples = state.samples;
    if (!track || !samples.length) {
      if (label) label.textContent = 'Waiting';
      return;
    }
    var all = fullRange();
    var span = all[1] - all[0];
    var view = [state.miniStart, state.miniEnd];
    if (view[0] == null || view[1] == null) view = chartRange('breath');
    var html = samples.map(function(s, idx) {
      var next = samples[idx + 1] ? samples[idx + 1].t : Math.min(all[1], s.t + span / Math.max(30, samples.length));
      var left = ((s.t - all[0]) / span) * 100;
      var width = Math.max(.4, ((next - s.t) / span) * 100);
      return '<span class="rvt-mini-quality" data-state="' + (s.state || 'unknown') + '" style="left:' +
        left.toFixed(3) + '%;width:' + width.toFixed(3) + '%"></span>';
    }).join('');
    html += samples.filter(function(s) { return s.alert; }).slice(-24).map(function(s) {
      var left = ((s.t - all[0]) / span) * 100;
      return '<span class="rvt-mini-pip" data-kind="alert" title="Alert" style="left:' + left.toFixed(3) + '%"></span>';
    }).join('');
    var anns = Object.values(window.S && window.S.annotations || {}).flat().slice(-40);
    html += anns.map(function(a) {
      var left = n(a.elapsed_s) != null ? ((n(a.elapsed_s) - all[0]) / span) * 100 : (Math.max(0, Math.min(1, n(a.xPct) || 0)) * 100);
      return '<span class="rvt-mini-pip" data-kind="annotation" title="' + esc(a.label || 'Annotation') + '" style="left:' +
        Math.max(0, Math.min(100, left)).toFixed(3) + '%"></span>';
    }).join('');
    var vLeft = ((view[0] - all[0]) / span) * 100;
    var vWidth = ((view[1] - view[0]) / span) * 100;
    html += '<span class="rvt-mini-window" style="left:' + Math.max(0, Math.min(100, vLeft)).toFixed(3) +
      '%;width:' + Math.max(2, Math.min(100, vWidth)).toFixed(3) + '%"></span>';
    track.innerHTML = html;
    track.setAttribute('aria-valuenow', String(Math.round(Math.max(0, Math.min(100, vLeft + vWidth)))));
    if (label) label.textContent = Math.round(view[0]) + 's - ' + Math.round(view[1]) + 's';
  }

  function chartKeyFromCanvas(canvas) {
    return CHART_IDS[canvas && canvas.id] || '';
  }

  function nextAnnotationLabel() {
    var count = Object.values(window.S && window.S.annotations || {}).reduce(function(total, list) {
      return total + (Array.isArray(list) ? list.length : 0);
    }, 0);
    return 'T' + (count + 1);
  }

  function ensureAnnotationLayer(canvas) {
    if (!canvas) return null;
    var wrap = canvas.parentElement;
    if (!wrap) return null;
    if (getComputedStyle(wrap).position === 'static') wrap.style.position = 'relative';
    var layer = wrap.querySelector('.chart-annotation-layer');
    if (!layer) {
      layer = document.createElement('div');
      layer.className = 'chart-annotation-layer';
      wrap.appendChild(layer);
    }
    return layer;
  }

  function renderAnnotationLayer(chartKey) {
    var canvasId = Object.keys(CHART_IDS).find(function(id) { return CHART_IDS[id] === chartKey; });
    var canvas = document.getElementById(canvasId);
    var layer = ensureAnnotationLayer(canvas);
    if (!layer || !window.S) return;
    layer.querySelectorAll('.chart-annotation-marker[data-rvt-beta="1"]').forEach(function(n) { n.remove(); });
    (window.S.annotations && window.S.annotations[chartKey] || []).forEach(function(ann) {
      var marker = document.createElement('div');
      marker.className = 'chart-annotation-marker';
      marker.dataset.rvtBeta = '1';
      marker.style.left = (Math.max(0, Math.min(1, n(ann.xPct) || 0)) * 100) + '%';
      marker.innerHTML = '<span class="chart-annotation-label">' + esc(ann.label || 'T') + '</span>';
      layer.appendChild(marker);
    });
  }

  function createWaveAnnotation(chartKey, xPct, label) {
    if (!chartKey) return null;
    window.S = window.S || {};
    window.S.annotations = window.S.annotations || {};
    window.S.annotations[chartKey] = window.S.annotations[chartKey] || [];
    var ann = {
      id: 'ann_' + Date.now() + '_' + Math.random().toString(36).slice(2, 7),
      xPct: Math.max(0, Math.min(1, n(xPct) == null ? .5 : n(xPct))),
      label: label || nextAnnotationLabel(),
      created_at: new Date().toISOString(),
      elapsed_s: n(read(window.S.lastPayload || {}, ['meta.elapsed_s', 'elapsed_s']))
    };
    window.S.annotations[chartKey].push(ann);
    try { window.persistChartAnnotation && window.persistChartAnnotation(chartKey, ann, 'upsert'); } catch (_) {}
    renderAnnotationLayer(chartKey);
    renderMiniMap();
    return ann;
  }

  function closeAnnotMenu() {
    if (state.menu) state.menu.remove();
    state.menu = null;
  }

  function openAnnotMenu(ev, canvas) {
    closeAnnotMenu();
    var key = chartKeyFromCanvas(canvas);
    if (!key) return;
    var rect = canvas.getBoundingClientRect();
    var xPct = Math.max(0, Math.min(1, (ev.clientX - rect.left) / Math.max(1, rect.width)));
    var menu = document.createElement('div');
    menu.className = 'rvt-annot-menu';
    menu.setAttribute('role', 'menu');
    menu.innerHTML =
      '<button type="button" role="menuitem" data-action="annotate"><span class="material-symbols-rounded">edit_note</span><span>Annotate here</span></button>' +
      '<button type="button" role="menuitem" data-action="mode"><span class="material-symbols-rounded">gesture</span><span>Toggle annotation mode</span></button>';
    menu.style.left = Math.min(window.innerWidth - 188, Math.max(8, ev.clientX)) + 'px';
    menu.style.top = Math.min(window.innerHeight - 96, Math.max(8, ev.clientY)) + 'px';
    document.body.appendChild(menu);
    state.menu = menu;
    menu.querySelector('[data-action="annotate"]').addEventListener('click', function() {
      createWaveAnnotation(key, xPct);
      closeAnnotMenu();
    });
    menu.querySelector('[data-action="mode"]').addEventListener('click', function() {
      if (typeof window.toggleAnnotationMode === 'function') window.toggleAnnotationMode();
      closeAnnotMenu();
    });
    setTimeout(function() { menu.querySelector('button') && menu.querySelector('button').focus(); }, 0);
  }

  function bindAnnotations() {
    Object.keys(CHART_IDS).forEach(function(id) {
      var canvas = document.getElementById(id);
      if (!canvas || canvas.dataset.rvtContextAnnotReady) return;
      canvas.dataset.rvtContextAnnotReady = '1';
      canvas.setAttribute('tabindex', canvas.getAttribute('tabindex') || '0');
      canvas.addEventListener('contextmenu', function(ev) {
        ev.preventDefault();
        openAnnotMenu(ev, canvas);
      });
      canvas.addEventListener('keydown', function(ev) {
        if ((ev.key || '').toLowerCase() !== 'a' || !(ev.shiftKey || ev.altKey)) return;
        ev.preventDefault();
        createWaveAnnotation(chartKeyFromCanvas(canvas), .5);
      });
      renderAnnotationLayer(CHART_IDS[id]);
    });
    if (!state.docBindReady) {
      state.docBindReady = true;
      document.addEventListener('click', function(ev) {
        if (state.menu && !ev.target.closest('.rvt-annot-menu')) closeAnnotMenu();
      }, true);
      document.addEventListener('keydown', function(ev) {
        if (ev.key === 'Escape') closeAnnotMenu();
      }, true);
    }
  }

  var HR_STAGES = [
    { key: 'input', label: 'Input', count: ['hr_input_count', 'hr_total_count', 'rows', 'meta.rows'] },
    { key: 'arbiter', label: 'Arbiter', count: ['hr_arbiter_count', 'hr_arbiter_ok_n'], reason: ['arbiter_reason', 'hr_arbiter_reason'] },
    { key: 'reject', label: 'Reject phase', count: ['hr_reject_phase_count', 'hr_post_reject_n'], bad: ['reject_phase_rejected'], reason: ['hr_reject_reason', 'hr_block_stage'] },
    { key: 'blend', label: 'Blend', count: ['hr_blend_count', 'hr_post_blend_n'], reason: ['hr_blend_reason'] },
    { key: 'coherence', label: 'Coherence', count: ['hr_coherence_count', 'hr_post_coherence_n'], bad: ['coherence_rejected'], reason: ['hr_gate_reason', 'gate_reason'] },
    { key: 'publish', label: 'Publish', count: ['hr_publish_count', 'logged_hr_valid_n'], valid: ['logged_hr_valid', 'hr_valid'], reason: ['hr_publish_reason', 'publish_reason'] }
  ];
  var RR_STAGES = [
    { key: 'input', label: 'Input', count: ['rr_input_count', 'rr_total_count', 'rows', 'meta.rows'] },
    { key: 'selector', label: 'Selector', count: ['rr_selector_count', 'rr_selector_ok_n'], reason: ['rr_selector_reason'] },
    { key: 'accept', label: 'Accept', count: ['rr_accept_count', 'rr_post_accept_n'], reason: ['rr_gate_reason', 'gate_reason'] },
    { key: 'blend', label: 'Blend', count: ['rr_blend_count', 'rr_post_bias_correction_n'], reason: ['rr_blend_reason'] },
    { key: 'kalman', label: 'Kalman', count: ['rr_kalman_count', 'rr_post_kalman_n'], reason: ['rr_kalman_reason'] },
    { key: 'publish', label: 'Publish', count: ['rr_publish_count', 'logged_rr_valid_n'], valid: ['rr_valid', 'logged_rr_valid'], reason: ['rr_publish_reason', 'publish_reason'] }
  ];

  function stageCount(p, def, prev, idx) {
    var raw = read(p || {}, def.count || []);
    var count = n(raw);
    if (count != null) return Math.max(0, count);
    if (idx === 0) return Math.max(1, n(read(p || {}, ['rows', 'meta.rows'])) || state.samples.length || 1);
    var bad = (def.bad || []).some(function(path) {
      var v = read(p || {}, [path]);
      return v === true || /true|reject|block|bad|invalid/i.test(String(v || ''));
    });
    var valid = (def.valid || []).length ? !(def.valid || []).some(function(path) { return boolFalse(read(p || {}, [path])); }) : true;
    if (bad || !valid) return Math.max(0, prev * .38);
    return Math.max(0, prev * .88);
  }

  function stageReason(p, def, kind) {
    var v = read(p || {}, def.reason || []);
    if (v != null && String(v).trim()) return String(v).trim();
    var block = read(p || {}, [kind + '_block_stage', 'block_stage']);
    if (block != null && String(block).trim() && !/^0$|^none$/i.test(String(block))) return String(block).trim();
    return '';
  }

  function buildStages(kind, p) {
    var defs = kind === 'hr' ? HR_STAGES : RR_STAGES;
    var prev = 0;
    return defs.map(function(def, idx) {
      var count = stageCount(p, def, idx ? prev : 0, idx);
      prev = count;
      var reason = stageReason(p, def, kind);
      var st = reason ? (/warn|weak|low|suspect|motion/i.test(reason) ? 'warn' : 'bad') : 'good';
      return { key: def.key, label: def.label, count: count, reason: reason, state: st };
    });
  }

  function topReasons(kind) {
    var counts = {};
    state.samples.forEach(function(s) {
      var r = kind === 'hr' ? s.heartReason : s.breathReason;
      if (!r) return;
      counts[r] = (counts[r] || 0) + 1;
    });
    return Object.keys(counts).map(function(reason) {
      return { reason: reason, count: counts[reason] };
    }).sort(function(a, b) { return b.count - a.count; }).slice(0, 5);
  }

  function renderSankey(kind, p) {
    var table = document.getElementById(kind === 'hr' ? 'hrFunTable' : 'rrFunTable');
    if (!table) return;
    var card = table.closest('.card');
    if (!card) return;
    var id = kind === 'hr' ? 'rvtHrFunnelSankey' : 'rvtRrFunnelSankey';
    var mount = document.getElementById(id);
    if (!mount) {
      mount = document.createElement('div');
      mount.id = id;
      mount.className = 'rvt-funnel-sankey';
      table.closest('table').parentNode.insertBefore(mount, table.closest('table'));
    }
    var stages = buildStages(kind, p || (window.S && window.S.lastPayload) || {});
    var max = Math.max(1, stages[0] && stages[0].count || 1);
    var reasons = topReasons(kind);
    stages.forEach(function(s) {
      if (s.reason) reasons.push({ reason: s.reason, count: 1 });
    });
    var seen = {};
    reasons = reasons.filter(function(r) {
      var key = r.reason.toLowerCase();
      if (seen[key]) return false;
      seen[key] = true;
      return true;
    }).slice(0, 5);
    mount.innerHTML = stages.map(function(s) {
      var pct = Math.max(.05, Math.min(1, s.count / max));
      return '<div class="rvt-stage-lane" data-stage="' + esc(s.key) + '" data-state="' + esc(s.state) +
        '" style="--rvt-stage-pct:' + pct.toFixed(4) + '">' +
        '<div class="rvt-stage-name">' + esc(s.label) + '</div>' +
        '<div class="rvt-stage-bar" aria-hidden="true"><span class="rvt-stage-fill"></span></div>' +
        '<div class="rvt-stage-count">' + Math.round(s.count) + '</div>' +
      '</div>';
    }).join('') +
      '<details class="rvt-stage-reasons" ' + (reasons.length ? 'open' : '') + '>' +
      '<summary>Stage reasons</summary>' +
      '<div class="rvt-reason-list">' +
      (reasons.length ? reasons.map(function(r) {
        return '<button type="button" class="rvt-reason-chip" data-reason="' + esc(r.reason) + '">' +
          esc(r.reason) + (r.count > 1 ? ' ' + r.count : '') + '</button>';
      }).join('') : '<span class="muted">No rejection reasons in current buffer.</span>') +
      '</div></details>';
    mount.querySelectorAll('.rvt-reason-chip').forEach(function(btn) {
      btn.addEventListener('click', function() { filterAuditByReason(btn.dataset.reason || ''); });
    });
  }

  function filterAuditByReason(reason) {
    if (typeof window.switchView === 'function') window.switchView('live');
    if (typeof window.switchTab === 'function') window.switchTab('tab-audit');
    var immediateTab = document.getElementById('tab-audit');
    if (immediateTab) immediateTab.dataset.rvtReasonFilter = reason;
    setTimeout(function() {
      var tab = document.getElementById('tab-audit');
      if (!tab) return;
      tab.dataset.rvtReasonFilter = reason;
      tab.querySelectorAll('.rvt-reason-hit').forEach(function(el) { el.classList.remove('rvt-reason-hit'); });
      var needle = String(reason || '').toLowerCase();
      var hits = Array.from(tab.querySelectorAll('tr,.dv-alert-card')).filter(function(el) {
        return needle && (el.textContent || '').toLowerCase().includes(needle);
      });
      hits.forEach(function(el) { el.classList.add('rvt-reason-hit'); });
      (hits[0] || document.getElementById('audTable') || tab).scrollIntoView({ behavior: 'smooth', block: 'center' });
    }, 80);
  }

  function renderFunnelBeta(p) {
    renderSankey('hr', p);
    renderSankey('rr', p);
  }

  function tick(p) {
    recordPayload(p);
    bindAnnotations();
    renderWaveHatches();
    renderMiniMap();
    renderFunnelBeta(p || (window.S && window.S.lastPayload));
  }

  function wrapRender() {
    if (typeof window.render !== 'function' || window.render.__rvtWavesFunnelsWrapped) return false;
    var orig = window.render;
    window.render = function(payload) {
      var result = orig.apply(this, arguments);
      try { tick(payload); } catch (err) { try { console.warn('[RVT] waves/funnels beta failed', err); } catch (_) {} }
      return result;
    };
    window.render.__rvtWavesFunnelsWrapped = true;
    return true;
  }

  function wrapRenderAlertsState() {
    if (typeof window.renderAlerts !== 'function' || window.renderAlerts.__rvtWavesFunnelsStateWrapped) return false;
    var orig = window.renderAlerts;
    window.renderAlerts = function(alertList) {
      if (Array.isArray(alertList)) {
        window.S = window.S || {};
        window.S.lastAlerts = alertList;
      }
      return orig.apply(this, arguments);
    };
    window.renderAlerts.__rvtWavesFunnelsStateWrapped = true;
    return true;
  }

  function boot() {
    wrapRender();
    wrapRenderAlertsState();
    tick(window.S && (window.S.lastLivePayload || window.S.lastPayload));
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', function() { setTimeout(boot, 350); }, { once: true });
  else setTimeout(boot, 350);
  setInterval(function() {
    wrapRender();
    wrapRenderAlertsState();
    tick(window.S && (window.S.lastLivePayload || window.S.lastPayload));
  }, 1800);

  window.rvtRecordWaveFunnelPayload = recordPayload;
  window.rvtRenderWaveHatches = renderWaveHatches;
  window.rvtEnsureSessionMiniMap = ensureMiniMap;
  window.rvtRenderMiniMap = renderMiniMap;
  window.rvtSetMiniMapViewport = setMiniViewport;
  window.rvtCreateWaveAnnotation = createWaveAnnotation;
  window.rvtRenderFunnelBeta = renderFunnelBeta;
  window.rvtFilterAuditByReason = filterAuditByReason;
})();
;
/* END modules/rvt-waves-funnels-beta-js.js */

/* BEGIN modules/rvt-beta-slice-operator-console-js.js */
(function(){
  'use strict';

  var RETENTION_KEY = 'rvt-session-retention-policy';
  var ARCHIVE_KEY = 'rvt-session-archive';
  var RETENTION_UNDO_KEY = 'rvt-session-retention-undo';
  var OP_MARKER_KEY = 'rvt-operator-markers';
  var DEFAULT_THRESHOLDS = { hrLow: 40, hrHigh: 140, rrLow: 6, rrHigh: 30 };
  var KPI_META = {
    hr: { label: 'Heart rate', unit: 'bpm', valueId: 'kpiHr', spark: 'hr', low: 'hrLow', high: 'hrHigh', min: 20, max: 220 },
    rr: { label: 'Respiration', unit: 'br/min', valueId: 'kpiRr', spark: 'rr', low: 'rrLow', high: 'rrHigh', min: 2, max: 60 },
    fps: { label: 'Frame rate', unit: 'Hz', valueId: 'kpiFps', spark: 'fps', min: 0, max: 40 },
    distance: { label: 'Range', unit: 'cm', valueId: 'kpiDist', spark: 'dist', min: 0, max: 300 }
  };

  var stillnessTimer = 0;
  var stillnessState = { state: 'idle', total_s: 10, remaining_s: 10, started_at: 0, source: 'manual' };
  var retentionUndo = null;
  var kpiZoomMetric = null;

  function byId(id) { return document.getElementById(id); }
  function nowIso() { return new Date().toISOString(); }
  function finiteNumber(value) {
    var n = Number(value);
    return Number.isFinite ? (Number.isFinite(n) ? n : null) : (isFinite(n) ? n : null);
  }
  function clamp(n, lo, hi) {
    n = finiteNumber(n);
    if (n == null) return lo;
    return Math.max(lo, Math.min(hi, n));
  }
  function esc(value) {
    return String(value == null ? '' : value).replace(/[&<>"']/g, function(ch) {
      return ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[ch];
    });
  }
  function readJson(key, fallback) {
    try {
      var raw = localStorage.getItem(key);
      return raw ? JSON.parse(raw) : fallback;
    } catch (_) {
      return fallback;
    }
  }
  function writeJson(key, value) {
    try {
      localStorage.setItem(key, JSON.stringify(value));
      return true;
    } catch (err) {
      try { console.warn('[RVT beta slice] localStorage write failed:', key, err); } catch (_) {}
      return false;
    }
  }
  function toast(message, tone) {
    if (typeof window.showToast === 'function') {
      try { window.showToast(message, tone || 'ok'); return; } catch (_) {}
    }
    if (typeof window.toast === 'function') {
      try { window.toast(message, tone || 'ok'); return; } catch (_) {}
    }
    try { console.info('[RVT beta slice]', message); } catch (_) {}
  }
  function getState() {
    window.S = window.S || {};
    return window.S;
  }
  function currentSessionId() {
    try {
      if (typeof window.currentSessionId === 'function') {
        var fromFn = window.currentSessionId();
        if (fromFn) return String(fromFn);
      }
    } catch (_) {}
    var st = getState();
    var ctl = st.ctl || {};
    var cur = ctl.current || {};
    var meta = st.lastPayload && st.lastPayload.meta ? st.lastPayload.meta : {};
    return String(ctl.currentSessionId || cur.session_id || cur.id || st.sessionId || meta.session_id || meta.session || 'local');
  }
  function operatorName() {
    var st = getState();
    var setup = st.setup || {};
    var dom = byId('homeOperatorName') || byId('opName');
    return String(setup.operator_label || st.operatorName || (dom && dom.textContent) || 'Operator');
  }
  function sessionActive() {
    var st = getState();
    return !!(st.ctl && st.ctl.current && !st.ctl.stopPending);
  }
  function isDemoOrPreview() {
    var st = getState();
    return !!(st.demoMode || st.autoDemoActive || (st.ctl && st.ctl.sandbox) || document.body.classList.contains('demo-mode'));
  }
  function isSensorStale() {
    var st = getState();
    var payload = st.lastPayload || st.lastLivePayload || {};
    var meta = payload.meta || {};
    var radar = payload.radar || {};
    var age = finiteNumber(radar.last_seen_age_s);
    var lastGoodAge = st.lastGoodAt ? Date.now() - Number(st.lastGoodAt) : 0;
    return document.body.classList.contains('rvt-sensor-stale') ||
      /stale|disconnect|offline|lost/i.test(String(meta.status || '')) ||
      Number(st.disc || 0) >= 3 ||
      (lastGoodAge > 5000) ||
      (age != null && age >= 5);
  }
  function apiHeaders() {
    var headers = {};
    try {
      var token = localStorage.getItem('rvt-api-token') || new URLSearchParams(location.search).get('token') || '';
      if (token) {
        headers['X-RVT-Token'] = token;
        headers['X-Radar-Vital-Token'] = token;
      }
    } catch (_) {}
    return headers;
  }

  function skipCountdownEnabled() {
    var cb = byId('skipCountdown');
    if (cb) return !!cb.checked;
    var st = getState();
    return !!(st.setup && st.setup.skip_countdown);
  }
  function stillnessTotalSeconds() {
    var explicit = finiteNumber(getState().stillnessCountdownS);
    if (explicit != null) return clamp(explicit, 3, 60);
    return 10;
  }
  function injectStillnessCalibration() {
    var home = byId('view-home');
    if (!home || byId('rvtStillnessCalibration')) return;
    var panel = document.createElement('section');
    panel.id = 'rvtStillnessCalibration';
    panel.className = 'rvt-stillness-calibration';
    panel.setAttribute('aria-live', 'polite');
    panel.innerHTML =
      '<div class="rvt-stillness-ring" id="rvtStillnessRing">10</div>' +
      '<div class="rvt-stillness-copy">' +
        '<strong id="rvtStillnessTitle">Stillness calibration ready</strong>' +
        '<span id="rvtStillnessDetail">Hold posture steady before capture. This is an operator cue only; it does not certify signal quality.</span>' +
        '<div class="rvt-stillness-bar" aria-hidden="true"><span id="rvtStillnessFill"></span></div>' +
      '</div>' +
      '<div class="rvt-stillness-actions">' +
        '<button class="rvt-beta-mini-btn" type="button" id="rvtStillnessStart"><span class="material-symbols-rounded">play_arrow</span>Start cue</button>' +
        '<button class="rvt-beta-mini-btn" type="button" id="rvtStillnessCancel"><span class="material-symbols-rounded">close</span>Cancel</button>' +
      '</div>';
    var setup = byId('setupCard') || home.querySelector('.setup-card, [data-section="setup"]');
    if (setup && setup.parentNode) setup.parentNode.insertBefore(panel, setup.nextSibling);
    else home.insertBefore(panel, home.firstChild);
    byId('rvtStillnessStart').addEventListener('click', function(){ startStillnessCalibration(stillnessTotalSeconds(), 'manual'); });
    byId('rvtStillnessCancel').addEventListener('click', cancelStillnessCalibration);
    renderStillnessCalibration();
  }
  function renderStillnessCalibration() {
    var panel = byId('rvtStillnessCalibration');
    if (!panel) return;
    var state = stillnessState.state || 'idle';
    var total = Math.max(1, Number(stillnessState.total_s || 10));
    var remaining = Math.max(0, Math.ceil(Number(stillnessState.remaining_s || 0)));
    var pct = state === 'ready' || state === 'skipped' ? 100 : Math.round(((total - remaining) / total) * 100);
    if (state === 'idle') pct = 0;
    panel.dataset.state = state;
    panel.style.setProperty('--rvt-stillness-progress', pct + '%');
    var ring = byId('rvtStillnessRing');
    var title = byId('rvtStillnessTitle');
    var detail = byId('rvtStillnessDetail');
    var fill = byId('rvtStillnessFill');
    if (ring) ring.textContent = state === 'ready' ? 'OK' : (state === 'skipped' ? 'SKIP' : String(remaining || total));
    if (fill) fill.style.width = pct + '%';
    if (title && detail) {
      if (state === 'running') {
        title.textContent = 'Hold still - calibrating baseline';
        detail.textContent = 'Keep the subject still for ' + remaining + 's. Cancel if posture, speech, or motion invalidates the baseline.';
      } else if (state === 'ready') {
        title.textContent = 'Stillness cue complete';
        detail.textContent = 'Baseline cue finished. Continue to monitor live freshness and reference validity.';
      } else if (state === 'skipped') {
        title.textContent = 'Stillness cue skipped';
        detail.textContent = 'Skip countdown is enabled. This bypass is visible because it can affect session interpretation.';
      } else if (state === 'canceled') {
        title.textContent = 'Stillness cue canceled';
        detail.textContent = 'Cue reset by the operator. Restart it before capture if a still baseline is needed.';
      } else {
        title.textContent = 'Stillness calibration ready';
        detail.textContent = 'Hold posture steady before capture. This is an operator cue only; it does not certify signal quality.';
      }
    }
  }
  function startStillnessCalibration(seconds, source) {
    injectStillnessCalibration();
    clearInterval(stillnessTimer);
    if (skipCountdownEnabled()) {
      stillnessState = { state: 'skipped', total_s: 0, remaining_s: 0, started_at: Date.now(), source: source || 'manual' };
      renderStillnessCalibration();
      return stillnessState;
    }
    seconds = clamp(seconds || stillnessTotalSeconds(), 3, 60);
    stillnessState = { state: 'running', total_s: seconds, remaining_s: seconds, started_at: Date.now(), source: source || 'manual' };
    renderStillnessCalibration();
    stillnessTimer = setInterval(function() {
      var elapsed = (Date.now() - stillnessState.started_at) / 1000;
      stillnessState.remaining_s = Math.max(0, stillnessState.total_s - elapsed);
      if (stillnessState.remaining_s <= 0) {
        clearInterval(stillnessTimer);
        stillnessState.state = 'ready';
        stillnessState.remaining_s = 0;
      }
      renderStillnessCalibration();
    }, 250);
    return stillnessState;
  }
  function cancelStillnessCalibration() {
    clearInterval(stillnessTimer);
    stillnessState.state = stillnessState.state === 'idle' ? 'idle' : 'canceled';
    stillnessState.remaining_s = stillnessState.total_s || stillnessTotalSeconds();
    renderStillnessCalibration();
  }
  function installStillnessHooks() {
    if (!document.__rvtStillnessClickHook) {
      document.__rvtStillnessClickHook = true;
      document.addEventListener('click', function(ev) {
        var target = ev.target && ev.target.closest ? ev.target.closest('.home-start-btn, .setup-start, #startSessionBtn, [onclick*="startSession"]') : null;
        if (!target || target.disabled) return;
        if (stillnessState.state !== 'running') startStillnessCalibration(stillnessTotalSeconds(), 'session-start');
      }, true);
    }
    ['startSession', 'startControlSession', 'startSessionFromSetup', 'beginSession'].forEach(function(name) {
      var fn = window[name];
      if (typeof fn !== 'function' || fn.__rvtStillnessWrapped) return;
      window[name] = function() {
        if (stillnessState.state !== 'running') startStillnessCalibration(stillnessTotalSeconds(), name);
        return fn.apply(this, arguments);
      };
      window[name].__rvtStillnessWrapped = true;
    });
  }

  function getThresholds() {
    var saved = readJson('rvt-thresholds', {});
    var state = getState();
    return Object.assign({}, DEFAULT_THRESHOLDS, state.kpiThresholds || {}, saved || {});
  }
  function setThresholdLocal(key, value) {
    var meta = { hrLow: [20, 80], hrHigh: [60, 220], rrLow: [2, 20], rrHigh: [10, 60] }[key];
    if (!meta) return;
    var thresholds = getThresholds();
    thresholds[key] = clamp(value, meta[0], meta[1]);
    getState().kpiThresholds = Object.assign({}, getState().kpiThresholds || {}, thresholds);
    writeJson('rvt-thresholds', thresholds);
  }
  function metricValue(metric) {
    var meta = KPI_META[metric];
    if (!meta) return null;
    var el = byId(meta.valueId);
    var value = el ? finiteNumber(String(el.textContent || '').replace(/[^\d.+-]/g, '')) : null;
    if (value != null) return value;
    var radar = getState().lastPayload && getState().lastPayload.radar ? getState().lastPayload.radar : {};
    var candidates = metric === 'hr' ? [radar.hr_bpm, radar.hr, radar.heart_rate_bpm] :
      metric === 'rr' ? [radar.rr_bpm, radar.rr, radar.respiration_rate_bpm] :
      metric === 'fps' ? [radar.fps, radar.frame_rate_hz] :
      [radar.distance_cm, radar.range_cm, radar.distance];
    for (var i = 0; i < candidates.length; i += 1) {
      value = finiteNumber(candidates[i]);
      if (value != null) return value;
    }
    return null;
  }
  function metricSeries(metric) {
    var meta = KPI_META[metric];
    var out = [];
    var state = getState();
    var spark = state.spark && meta ? state.spark[meta.spark] : null;
    if (Array.isArray(spark)) {
      spark.slice(-240).forEach(function(point, idx) {
        var value = typeof point === 'object' ? finiteNumber(point.v != null ? point.v : point.y) : finiteNumber(point);
        if (value != null) out.push({ x: idx, y: value });
      });
    }
    var chart = state.charts && (state.charts[metric] || (metric === 'distance' ? state.charts.dist : null));
    var data = chart && chart.data && chart.data.datasets && chart.data.datasets[0] ? chart.data.datasets[0].data : null;
    if (!out.length && Array.isArray(data)) {
      data.slice(-240).forEach(function(point, idx) {
        var value = typeof point === 'object' ? finiteNumber(point.y != null ? point.y : point.v) : finiteNumber(point);
        if (value != null) out.push({ x: idx, y: value });
      });
    }
    var current = metricValue(metric);
    if (!out.length && current != null) out.push({ x: 0, y: current });
    return out;
  }
  function drawMetricCanvas(canvas, metric, opts) {
    if (!canvas || !canvas.getContext) return;
    opts = opts || {};
    var meta = KPI_META[metric];
    if (!meta) return;
    var dpr = window.devicePixelRatio || 1;
    var rect = canvas.getBoundingClientRect();
    var width = Math.max(220, rect.width || canvas.clientWidth || 320);
    var height = Math.max(130, rect.height || canvas.clientHeight || 170);
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    var ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    var pad = { l: 34, r: 14, t: 14, b: 24 };
    var w = width - pad.l - pad.r;
    var h = height - pad.t - pad.b;
    var thresholds = getThresholds();
    var low = meta.low ? finiteNumber(thresholds[meta.low]) : null;
    var high = meta.high ? finiteNumber(thresholds[meta.high]) : null;
    var series = opts.series || metricSeries(metric);
    var values = series.map(function(p){ return p.y; }).filter(function(v){ return v != null; });
    var min = meta.min;
    var max = meta.max;
    values.concat([low, high]).forEach(function(v) {
      if (v == null) return;
      if (v < min) min = v;
      if (v > max) max = v;
    });
    if (min === max) max = min + 1;
    var range = max - min;
    function yFor(v) { return pad.t + h - ((v - min) / range) * h; }
    function xFor(idx) { return pad.l + (series.length <= 1 ? w / 2 : (idx / (series.length - 1)) * w); }
    ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue('--surface-2') || '#f8faff';
    ctx.fillRect(pad.l, pad.t, w, h);
    ctx.strokeStyle = '#dbe3ef';
    ctx.lineWidth = 1;
    for (var g = 0; g <= 4; g += 1) {
      var gy = pad.t + (h * g / 4);
      ctx.beginPath();
      ctx.moveTo(pad.l, gy);
      ctx.lineTo(pad.l + w, gy);
      ctx.stroke();
    }
    if (low != null) {
      var ly = yFor(low);
      ctx.fillStyle = 'rgba(220, 38, 38, 0.07)';
      ctx.fillRect(pad.l, ly, w, pad.t + h - ly);
      ctx.strokeStyle = 'rgba(220, 38, 38, 0.48)';
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.moveTo(pad.l, ly);
      ctx.lineTo(pad.l + w, ly);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    if (high != null) {
      var hy = yFor(high);
      ctx.fillStyle = 'rgba(220, 38, 38, 0.07)';
      ctx.fillRect(pad.l, pad.t, w, hy - pad.t);
      ctx.strokeStyle = 'rgba(220, 38, 38, 0.48)';
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      ctx.moveTo(pad.l, hy);
      ctx.lineTo(pad.l + w, hy);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.strokeStyle = opts.color || '#2563eb';
    ctx.lineWidth = 2;
    if (series.length > 1) {
      ctx.beginPath();
      series.forEach(function(p, idx) {
        var x = xFor(idx);
        var y = yFor(p.y);
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }
    if (series.length) {
      var last = series[series.length - 1];
      ctx.fillStyle = opts.color || '#2563eb';
      ctx.beginPath();
      ctx.arc(xFor(series.length - 1), yFor(last.y), 4, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.fillStyle = '#64748b';
    ctx.font = '10px JetBrains Mono, monospace';
    ctx.textAlign = 'left';
    ctx.fillText(String(Math.round(max)), 4, pad.t + 4);
    ctx.fillText(String(Math.round(min)), 4, pad.t + h);
    if (!series.length) {
      ctx.textAlign = 'center';
      ctx.font = '12px Inter, sans-serif';
      ctx.fillText('No valid live value', width / 2, height / 2);
    }
  }
  function injectThresholdViz() {
    var preview = document.querySelector('[data-threshold-preview]');
    if (!preview || byId('rvtThresholdViz')) return;
    var viz = document.createElement('div');
    viz.id = 'rvtThresholdViz';
    viz.className = 'rvt-threshold-viz';
    viz.innerHTML =
      '<div class="rvt-threshold-chart-grid">' +
        '<div class="rvt-threshold-chart-card">' +
          '<div class="rvt-threshold-chart-head"><span>HR alert range</span><span id="rvtThrHrLabel">--</span></div>' +
          '<canvas class="rvt-threshold-canvas" id="rvtThrHrCanvas" aria-label="Heart rate threshold preview"></canvas>' +
        '</div>' +
        '<div class="rvt-threshold-chart-card">' +
          '<div class="rvt-threshold-chart-head"><span>RR alert range</span><span id="rvtThrRrLabel">--</span></div>' +
          '<canvas class="rvt-threshold-canvas" id="rvtThrRrCanvas" aria-label="Respiration threshold preview"></canvas>' +
        '</div>' +
      '</div>' +
      '<div class="rvt-threshold-narrative" id="rvtThresholdNarrative">Preview updates as threshold controls change.</div>';
    preview.insertAdjacentElement('afterend', viz);
    renderThresholdViz();
  }
  function renderThresholdViz() {
    var viz = byId('rvtThresholdViz');
    if (!viz) return;
    var th = getThresholds();
    var hrLabel = byId('rvtThrHrLabel');
    var rrLabel = byId('rvtThrRrLabel');
    if (hrLabel) hrLabel.textContent = th.hrLow + '-' + th.hrHigh + ' bpm';
    if (rrLabel) rrLabel.textContent = th.rrLow + '-' + th.rrHigh + ' br/min';
    drawMetricCanvas(byId('rvtThrHrCanvas'), 'hr', { color: '#2563eb' });
    drawMetricCanvas(byId('rvtThrRrCanvas'), 'rr', { color: '#0891b2' });
    var narrative = byId('rvtThresholdNarrative');
    if (narrative) {
      var hr = metricValue('hr');
      var rr = metricValue('rr');
      var flags = [];
      if (hr != null && (hr < th.hrLow || hr > th.hrHigh)) flags.push('HR outside preview band');
      if (rr != null && (rr < th.rrLow || rr > th.rrHigh)) flags.push('RR outside preview band');
      narrative.textContent = flags.length ? flags.join('; ') + '.' : 'Current valid HR/RR values are inside the configured preview bands when live data is fresh.';
    }
  }
  function installThresholdHooks() {
    if (!document.__rvtThresholdVizHook) {
      document.__rvtThresholdVizHook = true;
      document.addEventListener('input', function(ev) {
        var target = ev.target;
        if (target && target.getAttribute && target.getAttribute('data-threshold-key')) {
          setTimeout(renderThresholdViz, 0);
        }
      }, true);
      document.addEventListener('change', function(ev) {
        var target = ev.target;
        if (target && target.getAttribute && target.getAttribute('data-threshold-key')) {
          setTimeout(renderThresholdViz, 0);
        }
      }, true);
    }
    if (typeof window.setThresholdValue === 'function' && !window.setThresholdValue.__rvtThresholdVizWrapped) {
      var original = window.setThresholdValue;
      window.setThresholdValue = function(key, value) {
        var result = original.apply(this, arguments);
        setThresholdLocal(key, value);
        setTimeout(renderThresholdViz, 0);
        return result;
      };
      window.setThresholdValue.__rvtThresholdVizWrapped = true;
    } else if (typeof window.setThresholdValue !== 'function') {
      window.setThresholdValue = function(key, value) {
        setThresholdLocal(key, value);
        document.querySelectorAll('[data-threshold-key="' + String(key).replace(/"/g, '') + '"]').forEach(function(el) {
          if (String(el.value) !== String(value)) el.value = value;
        });
        renderThresholdViz();
      };
    }
  }

  function applyLiveMode(mode) {
    var requested = mode === 'advanced' ? 'advanced' : 'simple';
    document.body.dataset.liveMode = requested;
    try { localStorage.setItem('rvt-live-mode', requested); } catch (_) {}
    document.querySelectorAll('[data-live-mode]').forEach(function(btn) {
      if (!btn || !btn.getAttribute) return;
      var active = btn.getAttribute('data-live-mode') === requested;
      btn.classList.toggle('active', active);
      btn.setAttribute('aria-checked', active ? 'true' : 'false');
      btn.setAttribute('aria-pressed', active ? 'true' : 'false');
      btn.tabIndex = active ? 0 : -1;
    });
    markAdvancedLivePanels();
    if (typeof window.repaintCharts === 'function') {
      try { requestAnimationFrame(window.repaintCharts); } catch (_) {}
    }
  }
  function markAdvancedLivePanels() {
    var overview = byId('tab-overview');
    if (!overview) return;
    overview.querySelectorAll('.kpi[data-kpi-id]').forEach(function(kpi) { kpi.removeAttribute('data-rvt-beta-advanced'); });
    overview.querySelectorAll('.card, article').forEach(function(card) {
      if (card.id === 'sessionNotesCard') return;
      var text = (card.textContent || '').toLowerCase();
      if (/target tracking|physiology vs policy|latest parsed row|module telemetry|pqi|policy|funnel|audit|schema|raw|latched|phase|harmonic|kalman|point/.test(text)) {
        card.setAttribute('data-rvt-beta-advanced', 'true');
      }
    });
  }
  function installLiveModeHooks() {
    var saved = 'simple';
    try { saved = localStorage.getItem('rvt-live-mode') || 'simple'; } catch (_) {}
    if (typeof window.setLiveMode === 'function' && !window.setLiveMode.__rvtBetaSliceWrapped) {
      var original = window.setLiveMode;
      window.setLiveMode = function(mode) {
        var result = original.apply(this, arguments);
        applyLiveMode(mode);
        return result;
      };
      window.setLiveMode.__rvtBetaSliceWrapped = true;
    }
    applyLiveMode(saved === 'advanced' ? 'advanced' : 'simple');
    window.rvtSetSimpleView = function(){ applyLiveMode('simple'); };
    window.rvtSetAdvancedView = function(){ applyLiveMode('advanced'); };
  }

  function ensureKpiZoomOverlay() {
    var overlay = byId('rvtBetaKpiZoom');
    if (overlay) return overlay;
    overlay = document.createElement('div');
    overlay.id = 'rvtBetaKpiZoom';
    overlay.className = 'rvt-beta-kpi-zoom';
    overlay.setAttribute('role', 'dialog');
    overlay.setAttribute('aria-modal', 'true');
    overlay.setAttribute('aria-labelledby', 'rvtBetaKpiTitle');
    overlay.innerHTML =
      '<div class="rvt-beta-kpi-card">' +
        '<div class="rvt-beta-kpi-head">' +
          '<div><h2 id="rvtBetaKpiTitle">KPI detail</h2><div class="rvt-beta-kpi-value" id="rvtBetaKpiValue">--</div></div>' +
          '<button class="rvt-beta-mini-btn" type="button" id="rvtBetaKpiClose" aria-label="Close KPI zoom"><span class="material-symbols-rounded">close</span></button>' +
        '</div>' +
        '<div class="rvt-kpi-zoom-truth" id="rvtBetaKpiTruth">No valid live value</div>' +
        '<div class="rvt-beta-kpi-meta" id="rvtBetaKpiMeta"></div>' +
        '<canvas class="rvt-beta-kpi-chart" id="rvtBetaKpiCanvas" aria-label="KPI zoom chart"></canvas>' +
      '</div>';
    document.body.appendChild(overlay);
    byId('rvtBetaKpiClose').addEventListener('click', closeKpiZoom);
    overlay.addEventListener('click', function(ev) { if (ev.target === overlay) closeKpiZoom(); });
    return overlay;
  }
  function closeKpiZoom() {
    var overlay = byId('rvtBetaKpiZoom');
    if (overlay) overlay.classList.remove('show');
    kpiZoomMetric = null;
  }
  function openKpiZoom(metric) {
    if (!KPI_META[metric]) return;
    var overlay = ensureKpiZoomOverlay();
    kpiZoomMetric = metric;
    var meta = KPI_META[metric];
    var value = metricValue(metric);
    var th = getThresholds();
    var stale = isSensorStale();
    var demo = isDemoOrPreview();
    byId('rvtBetaKpiTitle').textContent = meta.label;
    byId('rvtBetaKpiValue').textContent = value == null ? '--' : (metric === 'fps' ? value.toFixed(1) : Math.round(value)) + ' ' + meta.unit;
    var truth = byId('rvtBetaKpiTruth');
    if (truth) {
      if (demo) {
        truth.textContent = 'Demo/preview - not recording. Values are for UI rehearsal only.';
        truth.dataset.sev = 'warn';
      } else if (stale) {
        truth.textContent = 'Stale/disconnected. Do not interpret this KPI as healthy until live freshness recovers.';
        truth.dataset.sev = 'bad';
      } else if (value == null) {
        truth.textContent = 'No valid live value. Waiting for a fresh radar payload.';
        truth.dataset.sev = 'warn';
      } else {
        truth.textContent = 'Live value is fresh according to the current client state.';
        truth.dataset.sev = 'ok';
      }
    }
    var series = metricSeries(metric);
    var values = series.map(function(p){ return p.y; });
    var mean = values.length ? values.reduce(function(a,b){ return a + b; }, 0) / values.length : null;
    var variance = values.length && mean != null ? values.reduce(function(a,b){ return a + Math.pow(b - mean, 2); }, 0) / values.length : null;
    var thresholdLabel = meta.low && meta.high ? (th[meta.low] + '-' + th[meta.high] + ' ' + meta.unit) : 'No alert threshold';
    byId('rvtBetaKpiMeta').innerHTML =
      '<span>Samples: ' + values.length + '</span>' +
      '<span>Mean: ' + (mean == null ? '--' : mean.toFixed(1)) + '</span>' +
      '<span>Std dev: ' + (variance == null ? '--' : Math.sqrt(variance).toFixed(2)) + '</span>' +
      '<span>Threshold: ' + esc(thresholdLabel) + '</span>';
    drawMetricCanvas(byId('rvtBetaKpiCanvas'), metric, { series: series, color: metric === 'rr' ? '#0891b2' : '#2563eb' });
    overlay.classList.add('show');
  }
  function refreshKpiZoom() {
    if (kpiZoomMetric) openKpiZoom(kpiZoomMetric);
  }
  function installKpiZoomHooks() {
    document.querySelectorAll('.kpi[data-kpi-id]').forEach(function(kpi) {
      if (!kpi.getAttribute('role')) kpi.setAttribute('role', 'button');
      var metric = kpi.getAttribute('data-kpi-id');
      var meta = KPI_META[metric];
      if (meta) kpi.setAttribute('aria-label', 'Zoom ' + meta.label + ' KPI');
    });
    if (!document.__rvtBetaKpiZoomClickHook) {
      document.__rvtBetaKpiZoomClickHook = true;
      document.addEventListener('click', function(ev) {
        var kpi = ev.target && ev.target.closest ? ev.target.closest('.kpi[data-kpi-id]') : null;
        if (!kpi) return;
        if (ev.target.closest('button, a, input, textarea, select, .kpi-drag-handle')) return;
        var metric = kpi.getAttribute('data-kpi-id');
        if (!KPI_META[metric]) return;
        ev.preventDefault();
        ev.stopImmediatePropagation();
        openKpiZoom(metric);
      }, true);
      document.addEventListener('keydown', function(ev) {
        if (ev.key === 'Escape' && byId('rvtBetaKpiZoom') && byId('rvtBetaKpiZoom').classList.contains('show')) closeKpiZoom();
        var kpi = ev.target && ev.target.closest ? ev.target.closest('.kpi[data-kpi-id]') : null;
        if (!kpi || (ev.key !== 'Enter' && ev.key !== ' ')) return;
        var metric = kpi.getAttribute('data-kpi-id');
        if (!KPI_META[metric]) return;
        ev.preventDefault();
        openKpiZoom(metric);
      }, true);
    }
  }

  function normalizeSessionItem(item) {
    if (!item) return null;
    var id = item.id || item.session_id || item.name || item.path || item.dir || item.folder;
    if (!id) return null;
    id = String(id);
    var startedRaw = item.started_at || item.start_time || item.started || item.created_at || item.mtime || item.timestamp || '';
    var startedMs = finiteNumber(startedRaw);
    if (startedMs != null && startedMs < 10000000000) startedMs *= 1000;
    if (startedMs == null) {
      var parsed = Date.parse(String(startedRaw));
      startedMs = Number.isFinite(parsed) ? parsed : null;
    }
    return { id: id, label: String(item.label || item.name || id), started_ms: startedMs, raw: item };
  }
  function discoverSessions() {
    var map = {};
    var st = getState();
    var items = st.ctl && Array.isArray(st.ctl.sessionItems) ? st.ctl.sessionItems : [];
    items.forEach(function(item) {
      var norm = normalizeSessionItem(item);
      if (norm) map[norm.id] = norm;
    });
    document.querySelectorAll('#sessionsList [data-session-id], #sessionsList [data-session], [data-session-id]').forEach(function(row) {
      var id = row.getAttribute('data-session-id') || row.getAttribute('data-session');
      if (!id) return;
      if (!map[id]) map[id] = { id: id, label: (row.textContent || id).trim().slice(0, 80), started_ms: null, raw: null };
    });
    ['rvt-session-notes', 'rvt-session-tags'].forEach(function(key) {
      var bag = readJson(key, {});
      if (bag && typeof bag === 'object') {
        Object.keys(bag).forEach(function(id) {
          if (!map[id]) map[id] = { id: id, label: id, started_ms: null, raw: null };
        });
      }
    });
    return Object.keys(map).map(function(id){ return map[id]; });
  }
  function archiveStore() {
    var store = readJson(ARCHIVE_KEY, null);
    if (!store || typeof store !== 'object') store = { schema: 1, archived: {}, purged: [] };
    if (!store.archived || typeof store.archived !== 'object') store.archived = {};
    if (!Array.isArray(store.purged)) store.purged = [];
    store.schema = 1;
    return store;
  }
  function retentionPolicy() {
    var policy = readJson(RETENTION_KEY, {});
    var days = finiteNumber(policy.days);
    if (days == null) days = 30;
    return { schema: 1, days: days, showArchived: !!policy.showArchived };
  }
  function saveRetentionPolicy(policy) {
    writeJson(RETENTION_KEY, { schema: 1, days: Number(policy.days), showArchived: !!policy.showArchived });
  }
  function eligibleSessions(days) {
    if (!Number.isFinite(days) || days <= 0) return [];
    var cutoff = Date.now() - days * 86400000;
    var current = currentSessionId();
    return discoverSessions().filter(function(item) {
      return item.id !== current && item.started_ms != null && item.started_ms < cutoff;
    });
  }
  function localSessionMaps() {
    return {
      notes: readJson('rvt-session-notes', {}),
      tags: readJson('rvt-session-tags', {})
    };
  }
  function rememberUndo(kind, beforeArchive, beforeMaps, beforeItems) {
    retentionUndo = { schema: 1, kind: kind, ts: nowIso(), beforeArchive: beforeArchive, beforeMaps: beforeMaps, beforeItems: beforeItems || null };
    try { sessionStorage.setItem(RETENTION_UNDO_KEY, JSON.stringify(retentionUndo)); } catch (_) {}
  }
  function restoreRetentionUndo() {
    if (!retentionUndo) {
      try { retentionUndo = JSON.parse(sessionStorage.getItem(RETENTION_UNDO_KEY) || 'null'); } catch (_) { retentionUndo = null; }
    }
    if (!retentionUndo) return;
    writeJson(ARCHIVE_KEY, retentionUndo.beforeArchive || { schema: 1, archived: {}, purged: [] });
    if (retentionUndo.beforeMaps) {
      writeJson('rvt-session-notes', retentionUndo.beforeMaps.notes || {});
      writeJson('rvt-session-tags', retentionUndo.beforeMaps.tags || {});
    }
    if (retentionUndo.beforeItems && getState().ctl) getState().ctl.sessionItems = retentionUndo.beforeItems;
    retentionUndo = null;
    try { sessionStorage.removeItem(RETENTION_UNDO_KEY); } catch (_) {}
    renderRetentionSection('Undo restored local archive and purge state.', 'warn');
    syncArchiveRows();
  }
  function archiveEligibleSessions() {
    var policy = retentionPolicy();
    var store = archiveStore();
    var before = JSON.parse(JSON.stringify(store));
    var maps = localSessionMaps();
    var items = eligibleSessions(policy.days);
    rememberUndo('archive', before, maps, getState().ctl && Array.isArray(getState().ctl.sessionItems) ? getState().ctl.sessionItems.slice() : null);
    items.forEach(function(item) {
      store.archived[item.id] = { id: item.id, label: item.label, started_ms: item.started_ms, archived_at: nowIso(), raw: item.raw || null };
    });
    writeJson(ARCHIVE_KEY, store);
    renderRetentionSection('Archived ' + items.length + ' eligible session(s). Archive is client-side until backend support is available.', items.length ? 'warn' : 'ok');
    syncArchiveRows();
    return items.length;
  }
  function purgeArchivedSessions() {
    var store = archiveStore();
    var ids = Object.keys(store.archived || {});
    var before = JSON.parse(JSON.stringify(store));
    var maps = localSessionMaps();
    var beforeItems = getState().ctl && Array.isArray(getState().ctl.sessionItems) ? getState().ctl.sessionItems.slice() : null;
    rememberUndo('purge', before, maps, beforeItems);
    if (!ids.length) {
      renderRetentionSection('No archived sessions are available to purge.', 'ok');
      return 0;
    }
    ids.forEach(function(id) {
      delete maps.notes[id];
      delete maps.tags[id];
      store.purged.unshift({ id: id, purged_at: nowIso(), backend: 'best_effort' });
      delete store.archived[id];
    });
    store.purged = store.purged.slice(0, 50);
    writeJson('rvt-session-notes', maps.notes);
    writeJson('rvt-session-tags', maps.tags);
    writeJson(ARCHIVE_KEY, store);
    if (getState().ctl && Array.isArray(getState().ctl.sessionItems)) {
      getState().ctl.sessionItems = getState().ctl.sessionItems.filter(function(item) {
        var norm = normalizeSessionItem(item);
        return !norm || ids.indexOf(norm.id) === -1;
      });
    }
    ids.forEach(function(id) {
      try {
        fetch('/api/sessions/' + encodeURIComponent(id), { method: 'DELETE', headers: apiHeaders(), cache: 'no-store' }).catch(function(){});
      } catch (_) {}
    });
    renderRetentionSection('Purged ' + ids.length + ' archived session(s) from local dashboard state. Backend delete was sent best-effort when available; undo restores local state only.', 'bad');
    syncArchiveRows();
    return ids.length;
  }
  function syncArchiveRows() {
    var store = archiveStore();
    var policy = retentionPolicy();
    document.querySelectorAll('#sessionsList [data-session-id], #sessionsList [data-session], [data-session-id]').forEach(function(row) {
      var id = row.getAttribute('data-session-id') || row.getAttribute('data-session');
      var archived = !!(id && store.archived[id]);
      row.classList.toggle('rvt-session-archived', archived);
      row.setAttribute('data-rvt-archived', archived ? 'true' : 'false');
      if (archived && !policy.showArchived) row.hidden = true;
      else if (archived && policy.showArchived) row.hidden = false;
    });
  }
  function injectRetentionSection() {
    if (document.getElementById('rvtA16Settings')) return;
    var settings = byId('view-settings');
    if (!settings || byId('rvtSessionRetentionSection')) return;
    var section = document.createElement('section');
    section.id = 'rvtSessionRetentionSection';
    section.className = 'set-g rvt-session-retention';
    section.innerHTML =
      '<h4 class="set-gl">Session archive and purge</h4>' +
      '<div class="settings-help">Client-side retention controls for the dashboard session list, notes, and tags. Backend deletion is best-effort when a local endpoint exists.</div>' +
      '<div class="rvt-retention-grid">' +
        '<div class="rvt-retention-stat"><span>Known</span><strong id="rvtRetentionKnown">0</strong></div>' +
        '<div class="rvt-retention-stat"><span>Archived</span><strong id="rvtRetentionArchived">0</strong></div>' +
        '<div class="rvt-retention-stat"><span>Eligible</span><strong id="rvtRetentionEligible">0</strong></div>' +
      '</div>' +
      '<div class="rvt-retention-controls">' +
        '<label class="settings-help" for="rvtRetentionDays">Keep local sessions</label>' +
        '<select id="rvtRetentionDays">' +
          '<option value="7">7 days</option><option value="30">30 days</option><option value="90">90 days</option><option value="365">365 days</option><option value="0">Manual only</option>' +
        '</select>' +
        '<label class="settings-help"><input type="checkbox" id="rvtShowArchivedSessions"> Show archived rows</label>' +
      '</div>' +
      '<div class="rvt-retention-actions">' +
        '<button class="rvt-beta-mini-btn" type="button" id="rvtArchiveEligibleBtn"><span class="material-symbols-rounded">archive</span>Archive eligible</button>' +
        '<button class="rvt-beta-mini-btn" data-tone="danger" type="button" id="rvtPurgeArchivedBtn"><span class="material-symbols-rounded">delete</span>Purge archived</button>' +
        '<button class="rvt-beta-mini-btn" type="button" id="rvtRetentionUndo" hidden><span class="material-symbols-rounded">undo</span>Undo</button>' +
      '</div>' +
      '<div class="rvt-retention-summary" id="rvtRetentionSummary">Retention policy has not run in this tab.</div>';
    var target = null;
    settings.querySelectorAll('.set-g, section, .settings-page').forEach(function(el) {
      var text = (el.textContent || '').toLowerCase();
      if (!target && /privacy|data|storage|export/.test(text)) target = el;
    });
    if (target && target.parentNode) target.parentNode.insertBefore(section, target.nextSibling);
    else settings.appendChild(section);
    byId('rvtArchiveEligibleBtn').addEventListener('click', archiveEligibleSessions);
    byId('rvtPurgeArchivedBtn').addEventListener('click', function() {
      if (Object.keys(archiveStore().archived).length && !window.confirm('Purge archived local session state? Backend deletion is best-effort and may not be undoable.')) return;
      purgeArchivedSessions();
    });
    byId('rvtRetentionUndo').addEventListener('click', restoreRetentionUndo);
    byId('rvtRetentionDays').addEventListener('change', function() {
      var policy = retentionPolicy();
      policy.days = Number(this.value);
      saveRetentionPolicy(policy);
      renderRetentionSection('Retention policy updated.', 'ok');
    });
    byId('rvtShowArchivedSessions').addEventListener('change', function() {
      var policy = retentionPolicy();
      policy.showArchived = !!this.checked;
      saveRetentionPolicy(policy);
      syncArchiveRows();
      renderRetentionSection('Archive visibility updated.', 'ok');
    });
    renderRetentionSection();
  }
  function renderRetentionSection(message, sev) {
    var section = byId('rvtSessionRetentionSection');
    if (!section) return;
    var policy = retentionPolicy();
    var store = archiveStore();
    var sessions = discoverSessions();
    var eligible = eligibleSessions(policy.days);
    var known = byId('rvtRetentionKnown');
    var archived = byId('rvtRetentionArchived');
    var elig = byId('rvtRetentionEligible');
    if (known) known.textContent = String(sessions.length);
    if (archived) archived.textContent = String(Object.keys(store.archived).length);
    if (elig) elig.textContent = String(eligible.length);
    var days = byId('rvtRetentionDays');
    if (days) days.value = String(policy.days);
    var show = byId('rvtShowArchivedSessions');
    if (show) show.checked = !!policy.showArchived;
    var undoBtn = byId('rvtRetentionUndo');
    if (undoBtn) {
      if (!retentionUndo) {
        try { retentionUndo = JSON.parse(sessionStorage.getItem(RETENTION_UNDO_KEY) || 'null'); } catch (_) { retentionUndo = null; }
      }
      undoBtn.hidden = !retentionUndo;
    }
    var summary = byId('rvtRetentionSummary');
    if (summary) {
      summary.dataset.sev = sev || 'ok';
      summary.textContent = message || (eligible.length + ' session(s) exceed the current local retention policy.');
    }
  }

  function markerStore() {
    var store = readJson(OP_MARKER_KEY, null);
    if (!store || typeof store !== 'object') store = { schema: 1, sessions: {} };
    if (!store.sessions || typeof store.sessions !== 'object') store.sessions = {};
    store.schema = 1;
    return store;
  }
  function saveMarkerStore(store) {
    writeJson(OP_MARKER_KEY, store);
  }
  function markersForSession(id) {
    var store = markerStore();
    var list = store.sessions[id || currentSessionId()];
    return Array.isArray(list) ? list : [];
  }
  function appendAuditMarker(marker) {
    var st = getState();
    if (!Array.isArray(st.liveAuditLogs)) {
      st.liveAuditLogs = readJson('rvt-live-audit', []);
      if (!Array.isArray(st.liveAuditLogs)) st.liveAuditLogs = [];
    }
    st.liveAuditLogs.unshift({ ts: marker.ts, msg: 'Operator marker: ' + marker.label + ' (' + marker.operator + ')', sev: 'info', type: 'operator_marker' });
    st.liveAuditLogs = st.liveAuditLogs.slice(0, 100);
    writeJson('rvt-live-audit', st.liveAuditLogs);
    var logEl = byId('liveAuditLog');
    if (logEl) {
      var row = document.createElement('div');
      row.style.cssText = 'margin-bottom:4px;line-height:1.4;';
      row.innerHTML = '<span style="opacity:.5;margin-right:8px;">' + esc(marker.ts.split('T')[1].replace('Z','')) + '</span><span>' + esc('Operator marker: ' + marker.label) + '</span>';
      logEl.insertBefore(row, logEl.firstChild);
    }
  }
  function recordOperatorMarker(label, note) {
    var sid = currentSessionId();
    label = String(label || '').trim();
    if (!label) return null;
    var marker = {
      id: 'opm_' + Date.now() + '_' + Math.random().toString(36).slice(2, 7),
      ts: nowIso(),
      session_id: sid,
      operator: operatorName(),
      label: label.slice(0, 80),
      note: String(note || '').slice(0, 240),
      active: sessionActive(),
      source: 'operator_marker'
    };
    var store = markerStore();
    store.sessions[sid] = markersForSession(sid).concat(marker).slice(-100);
    saveMarkerStore(store);
    var st = getState();
    st.operatorMarkers = store.sessions[sid];
    st.reportMetadata = st.reportMetadata || {};
    st.reportMetadata.operator_markers = store.sessions[sid];
    if (st.ctl) {
      if (st.ctl.current) st.ctl.current.operator_markers = store.sessions[sid];
      if (st.ctl.currentSessionReport) {
        st.ctl.currentSessionReport.metadata = st.ctl.currentSessionReport.metadata || {};
        st.ctl.currentSessionReport.metadata.operator_markers = store.sessions[sid];
      }
    }
    appendAuditMarker(marker);
    renderOperatorMarkers();
    toast('Operator marker recorded', 'ok');
    return marker;
  }
  function promptOperatorMarker() {
    var label = window.prompt('Operator marker label', 'Operator check - ' + operatorName());
    if (!label) return;
    var note = window.prompt('Marker note (optional)', '') || '';
    recordOperatorMarker(label, note);
  }
  function injectOperatorMarkerButton() {
    var actions = document.querySelector('.topbar-actions');
    if (actions && !byId('rvtOperatorMarkerBtn')) {
      var btn = document.createElement('button');
      btn.id = 'rvtOperatorMarkerBtn';
      btn.className = 'ic-btn icon-btn rvt-operator-marker-btn';
      btn.type = 'button';
      btn.title = 'Record operator marker';
      btn.setAttribute('aria-label', 'Record operator marker');
      btn.innerHTML = '<span class="material-symbols-rounded">edit_calendar</span>';
      btn.addEventListener('click', promptOperatorMarker);
      var settings = actions.querySelector('.tb-settings');
      actions.insertBefore(btn, settings || actions.firstChild);
    }
    var overflow = byId('topbarOverflow');
    if (overflow && !byId('rvtOperatorMarkerOverflow')) {
      var item = document.createElement('button');
      item.id = 'rvtOperatorMarkerOverflow';
      item.className = 'ghost-btn ov-btn';
      item.type = 'button';
      item.setAttribute('role', 'menuitem');
      item.innerHTML = '<span class="material-symbols-rounded">edit_calendar</span><span class="ov-row"><span class="ov-title">Operator marker</span><span class="ov-hint">Timestamp the session</span></span>';
      item.addEventListener('click', function() {
        promptOperatorMarker();
        if (typeof window.closeOverflowMenu === 'function') {
          try { window.closeOverflowMenu(); } catch (_) {}
        }
      });
      overflow.appendChild(item);
    }
    if (!document.__rvtOperatorMarkerKeyHook) {
      document.__rvtOperatorMarkerKeyHook = true;
      document.addEventListener('keydown', function(ev) {
        var active = document.activeElement;
        if (active && (/input|textarea|select/i.test(active.tagName) || active.isContentEditable)) return;
        if (ev.ctrlKey && ev.shiftKey && String(ev.key).toLowerCase() === 'm') {
          ev.preventDefault();
          promptOperatorMarker();
        }
      });
    }
  }
  function markerHtml(marker) {
    var time = marker.ts ? new Date(marker.ts).toLocaleString() : '--';
    return '<div class="rvt-operator-marker-item">' +
      '<div><strong>' + esc(marker.label) + '</strong><br><span>' + esc(marker.operator) + '</span>' + (marker.note ? '<br><small>' + esc(marker.note) + '</small>' : '') + '</div>' +
      '<small>' + esc(time) + '</small>' +
    '</div>';
  }
  function renderOperatorMarkers() {
    var sid = currentSessionId();
    var markers = markersForSession(sid).slice().reverse();
    var audit = byId('tab-audit');
    if (audit) {
      var panel = byId('rvtOperatorMarkerPanel');
      if (!panel) {
        panel = document.createElement('article');
        panel.id = 'rvtOperatorMarkerPanel';
        panel.className = 'card rvt-operator-marker-panel';
        panel.innerHTML = '<div class="ch"><div class="tw"><span class="material-symbols-rounded">edit_calendar</span><div><h2 class="ct">Operator markers</h2><div class="cs">Timestamped mid-session operator events</div></div></div><div class="ca"><button class="ca-btn" type="button" id="rvtMarkerPanelAdd"><span class="material-symbols-rounded">add</span></button></div></div><div class="rvt-operator-marker-list" id="rvtOperatorMarkerList"></div>';
        audit.insertBefore(panel, audit.firstChild);
        byId('rvtMarkerPanelAdd').addEventListener('click', promptOperatorMarker);
      }
      var list = byId('rvtOperatorMarkerList');
      if (list) list.innerHTML = markers.length ? markers.map(markerHtml).join('') : '<div class="settings-help">No operator markers for this session yet.</div>';
    }
    var report = byId('view-report');
    if (report) {
      var rPanel = byId('rvtReportOperatorMarkers');
      if (!rPanel) {
        rPanel = document.createElement('section');
        rPanel.id = 'rvtReportOperatorMarkers';
        rPanel.className = 'card rvt-operator-marker-panel';
        rPanel.innerHTML = '<div class="ch"><div class="tw"><span class="material-symbols-rounded">edit_calendar</span><div><h2 class="ct">Report operator markers</h2><div class="cs">Included in client report metadata when available</div></div></div></div><div class="rvt-operator-marker-list" id="rvtReportOperatorMarkerList"></div>';
        report.appendChild(rPanel);
      }
      var rList = byId('rvtReportOperatorMarkerList');
      if (rList) rList.innerHTML = markers.length ? markers.map(markerHtml).join('') : '<div class="settings-help">No operator markers recorded for this session.</div>';
    }
  }

  function bootBetaSlice() {
    injectStillnessCalibration();
    installStillnessHooks();
    installThresholdHooks();
    injectThresholdViz();
    installLiveModeHooks();
    installKpiZoomHooks();
    injectRetentionSection();
    renderRetentionSection();
    syncArchiveRows();
    injectOperatorMarkerButton();
    renderOperatorMarkers();
    renderThresholdViz();
    refreshKpiZoom();
    if (sessionActive() && stillnessState.state === 'idle') renderStillnessCalibration();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(bootBetaSlice, 350); }, { once: true });
  } else {
    setTimeout(bootBetaSlice, 350);
  }
  var _bbsTimer = setInterval(function() { try { bootBetaSlice(); } catch(_){} if (document.querySelector('.rvt-stillness-section') || document.querySelector('.rvt-retention-section')) clearInterval(_bbsTimer); }, 2500);
  setInterval(refreshKpiZoom, 1000);

  window.rvtStartStillnessCalibration = startStillnessCalibration;
  window.rvtCancelStillnessCalibration = cancelStillnessCalibration;
  window.rvtRenderStillnessCalibration = renderStillnessCalibration;
  window.rvtArchiveEligibleSessions = archiveEligibleSessions;
  window.rvtPurgeArchivedSessions = purgeArchivedSessions;
  window.rvtRestoreRetentionUndo = restoreRetentionUndo;
  window.rvtRecordOperatorMarker = recordOperatorMarker;
  window.rvtRenderOperatorMarkers = renderOperatorMarkers;
  window.rvtApplyLiveMode = applyLiveMode;
  window.rvtOpenKpiZoom = openKpiZoom;
  window.rvtCloseKpiZoom = closeKpiZoom;
  window.rvtRenderThresholdViz = renderThresholdViz;
  window.rvtBootBetaSlice = bootBetaSlice;
})();
;
/* END modules/rvt-beta-slice-operator-console-js.js */

/* BEGIN modules/rvt-worker3-beta-js.js */
(function(){
  'use strict';

  var KEYS = {
    alertAcks: 'rvt-alert-acks',
    helpQuery: 'rvt-help-search-query',
    helpRecovery: 'rvt-help-recovery-state',
    settingsQuery: 'rvt-settings-search-query',
    settingsProfiles: 'rvt-settings-profiles'
  };
  var registered = false;
  var syncTimer = null;
  var lastPrintCleanup = null;

  function isPlainObject(v) {
    return !!v && typeof v === 'object' && !Array.isArray(v);
  }

  function registerStorageKeys() {
    if (registered || !window.rvtStorage || !window.rvtStorage.manifest) return;
    var manifest = window.rvtStorage.manifest;
    manifest[KEYS.alertAcks] = { kind: 'json', defaultValue: {}, validate: isPlainObject };
    manifest[KEYS.helpQuery] = { kind: 'scalar', defaultValue: '', validate: function(v){ return typeof v === 'string'; } };
    manifest[KEYS.helpRecovery] = { kind: 'json', defaultValue: {}, validate: isPlainObject };
    manifest[KEYS.settingsQuery] = { kind: 'scalar', defaultValue: '', validate: function(v){ return typeof v === 'string'; } };
    if (!manifest[KEYS.settingsProfiles]) manifest[KEYS.settingsProfiles] = { kind: 'json', defaultValue: {}, validate: isPlainObject };
    registered = true;
  }

  function getJson(key, fallback) {
    try {
      if (window.rvtStorage && typeof window.rvtStorage.getJson === 'function') {
        return window.rvtStorage.getJson(key, fallback);
      }
      var raw = localStorage.getItem(key);
      return raw ? JSON.parse(raw) : fallback;
    } catch (_) {
      return fallback;
    }
  }

  function setJson(key, value) {
    try {
      if (window.rvtStorage && typeof window.rvtStorage.setJson === 'function') {
        return window.rvtStorage.setJson(key, value, { important: false });
      }
      localStorage.setItem(key, JSON.stringify(value));
      return true;
    } catch (err) {
      softWarn('storage json write failed: ' + key, err);
      return false;
    }
  }

  function getScalar(key, fallback) {
    try {
      var raw = localStorage.getItem(key);
      return raw == null ? fallback : raw;
    } catch (_) {
      return fallback;
    }
  }

  function setScalar(key, value) {
    try {
      if (window.rvtStorage && typeof window.rvtStorage.set === 'function') return window.rvtStorage.set(key, String(value));
      localStorage.setItem(key, String(value));
      return true;
    } catch (err) {
      softWarn('storage write failed: ' + key, err);
      return false;
    }
  }

  function removeScalar(key) {
    try {
      if (window.rvtStorage && typeof window.rvtStorage.remove === 'function') return window.rvtStorage.remove(key);
      localStorage.removeItem(key);
      return true;
    } catch (err) {
      softWarn('storage remove failed: ' + key, err);
      return false;
    }
  }

  function esc(value) {
    return String(value == null ? '' : value).replace(/[&<>"']/g, function(ch) {
      return ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[ch];
    });
  }

  function softWarn(scope, err) {
    try {
      if (typeof window.rvtSoftError === 'function') window.rvtSoftError(scope, err, { always: false });
      else console.warn('[RVT worker3 beta]', scope, err);
    } catch (_) {}
  }

  function toastUser(message, icon) {
    try {
      if (typeof window.toast === 'function') window.toast(message, icon || 'info');
      else if (typeof window.showToast === 'function') window.showToast(message, icon || 'info');
    } catch (_) {}
  }

  function hashText(text) {
    var h = 2166136261;
    var s = String(text || '');
    for (var i = 0; i < s.length; i++) {
      h ^= s.charCodeAt(i);
      h += (h << 1) + (h << 4) + (h << 7) + (h << 8) + (h << 24);
    }
    return (h >>> 0).toString(36);
  }

  function normalizeSeverity(raw, card) {
    var s = String(raw || '').toLowerCase();
    var cls = card ? String(card.className || '').toLowerCase() : '';
    if (/critical|crit|bad|fail|error|danger/.test(s + ' ' + cls)) return 'critical';
    if (/info|good|ok|pass|recovered/.test(s + ' ' + cls)) return 'info';
    return 'warning';
  }

  function alertCards() {
    var body = document.getElementById('dvBody');
    return body ? Array.from(body.querySelectorAll('.dv-alert-card')) : [];
  }

  function alertCardKey(card) {
    if (!card) return '';
    if (card.dataset.alertKey) return card.dataset.alertKey;
    var title = (card.querySelector('.dv-alert-title') || {}).textContent || '';
    var copy = (card.querySelector('.dv-alert-copy') || {}).textContent || '';
    var meta = (card.querySelector('.dv-alert-meta') || {}).textContent || '';
    var sev = normalizeSeverity(card.dataset.alertSeverity || card.dataset.severity, card);
    var key = 'alert-' + hashText([sev, title, copy, meta].join('|'));
    card.dataset.alertKey = key;
    return key;
  }

  function loadAlertAcks() {
    return getJson(KEYS.alertAcks, {});
  }

  function saveAlertAcks(acks) {
    var entries = Object.entries(acks || {}).sort(function(a, b){ return Number(b[1]) - Number(a[1]); }).slice(0, 300);
    var trimmed = {};
    entries.forEach(function(entry){ trimmed[entry[0]] = entry[1]; });
    setJson(KEYS.alertAcks, trimmed);
    return trimmed;
  }

  function setCardAcked(card, acked, acks) {
    var key = alertCardKey(card);
    if (!key) return;
    if (acked) acks[key] = Date.now();
    else delete acks[key];
    card.dataset.acked = acked ? 'true' : 'false';
    card.classList.toggle('rvt-alert-acked', !!acked);
    var btn = card.querySelector('.rvt-alert-ack-one');
    if (btn) {
      btn.textContent = acked ? 'Undo ack' : 'Ack';
      btn.setAttribute('aria-pressed', acked ? 'true' : 'false');
    }
  }

  function ensureAlertCardActions(card, acks) {
    var sev = normalizeSeverity(card.dataset.alertSeverity || card.dataset.severity, card);
    card.dataset.alertSeverity = sev;
    card.dataset.severity = sev;
    alertCardKey(card);

    var title = card.querySelector('.dv-alert-title') || card.querySelector('.dv-alert-top') || card;
    if (title && !title.querySelector('.rvt-alert-pattern')) {
      var marker = document.createElement('span');
      marker.className = 'rvt-alert-pattern ' + sev;
      marker.setAttribute('aria-hidden', 'true');
      title.insertBefore(marker, title.firstChild);
    }

    var actions = card.querySelector('.dv-alert-actions');
    if (!actions) {
      actions = document.createElement('div');
      actions.className = 'dv-alert-actions';
      card.appendChild(actions);
    }
    if (!actions.querySelector('.rvt-alert-ack-one')) {
      var ackBtn = document.createElement('button');
      ackBtn.type = 'button';
      ackBtn.className = 'rvt-alert-ack-one';
      ackBtn.setAttribute('aria-pressed', 'false');
      ackBtn.addEventListener('click', function() {
        var latest = loadAlertAcks();
        var isAcked = card.dataset.acked === 'true';
        setCardAcked(card, !isAcked, latest);
        saveAlertAcks(latest);
        syncAlertAckToolbar();
      });
      actions.appendChild(ackBtn);
    }
    setCardAcked(card, !!acks[alertCardKey(card)], acks);
  }

  function visibleAlertCards() {
    return alertCards().filter(function(card) {
      return !card.hidden && card.style.display !== 'none' && !card.closest('[hidden]');
    });
  }

  function unackedVisibleAlertCards() {
    return visibleAlertCards().filter(function(card) { return card.dataset.acked !== 'true'; });
  }

  function bulkAcknowledgeVisibleAlerts() {
    var cards = unackedVisibleAlertCards();
    if (!cards.length) {
      toastUser('No visible alerts to acknowledge', 'done_all');
      return 0;
    }
    if (cards.length > 10 && !window.confirm('Acknowledge ' + cards.length + ' visible alerts?')) return 0;
    var acks = loadAlertAcks();
    cards.forEach(function(card) { setCardAcked(card, true, acks); });
    saveAlertAcks(acks);
    syncAlertAckToolbar();
    toastUser('Acknowledged ' + cards.length + ' visible alert' + (cards.length === 1 ? '' : 's'), 'done_all');
    return cards.length;
  }

  function ensureAlertAckToolbar() {
    var body = document.getElementById('dvBody');
    if (!body) return null;
    var existingFilter = document.getElementById('rvtAlertFilterBar');
    var bar = document.getElementById('rvtAlertAckToolbar');
    if (!bar) {
      bar = document.createElement('div');
      bar.id = 'rvtAlertAckToolbar';
      bar.className = 'rvt-alert-ack-toolbar';
      bar.innerHTML =
        '<button type="button" class="rvt-alert-bulk-btn" id="rvtAlertBulkAck">' +
        '<span class="material-symbols-rounded" aria-hidden="true">done_all</span> Ack visible</button>' +
        '<span class="rvt-alert-ack-status" id="rvtAlertAckStatus" aria-live="polite"></span>';
      if (existingFilter && existingFilter.nextSibling) body.insertBefore(bar, existingFilter.nextSibling);
      else if (existingFilter) body.appendChild(bar);
      else body.insertBefore(bar, body.firstChild);
      bar.querySelector('#rvtAlertBulkAck').addEventListener('click', bulkAcknowledgeVisibleAlerts);
    } else if (existingFilter && bar.previousElementSibling !== existingFilter) {
      if (existingFilter.nextSibling) body.insertBefore(bar, existingFilter.nextSibling);
      else body.appendChild(bar);
    }
    return bar;
  }

  function syncAlertAckToolbar() {
    var bar = ensureAlertAckToolbar();
    if (!bar) return;
    var unacked = unackedVisibleAlertCards().length;
    var total = visibleAlertCards().length;
    var btn = bar.querySelector('#rvtAlertBulkAck');
    var status = bar.querySelector('#rvtAlertAckStatus');
    if (btn) btn.disabled = unacked === 0;
    if (status) status.textContent = unacked + ' unacknowledged of ' + total + ' visible';
  }

  function syncAlerts() {
    var cards = alertCards();
    if (!cards.length) return;
    var acks = loadAlertAcks();
    cards.forEach(function(card) { ensureAlertCardActions(card, acks); });
    syncAlertAckToolbar();
  }

  function wrapAlertEntrypoints() {
    if (typeof window.renderAlerts === 'function' && !window.renderAlerts.__rvtWorker3Wrapped) {
      var originalRenderAlerts = window.renderAlerts;
      window.renderAlerts = function() {
        try {
          if (Array.isArray(arguments[0])) {
            if (!window.S) window.S = {};
            window.S.lastAlerts = arguments[0];
          }
        } catch (_) {}
        var result = originalRenderAlerts.apply(this, arguments);
        setTimeout(syncAlerts, 0);
        return result;
      };
      window.renderAlerts.__rvtWorker3Wrapped = true;
    }
    if (typeof window.openDrawer === 'function' && !window.openDrawer.__rvtWorker3Wrapped) {
      var originalOpenDrawer = window.openDrawer;
      window.openDrawer = function() {
        var result = originalOpenDrawer.apply(this, arguments);
        setTimeout(syncAlerts, 0);
        return result;
      };
      window.openDrawer.__rvtWorker3Wrapped = true;
    }
  }

  function reportView() {
    return document.getElementById('view-report');
  }

  function reportSources() {
    var s = window.S || {};
    var ctl = s.ctl || {};
    var payload = s.lastPayload || {};
    return [
      s.lastReportPayload,
      ctl.currentSessionReport,
      ctl.compareSessionData,
      payload.analysis,
      payload.meta,
      payload
    ].filter(function(v){ return v && typeof v === 'object'; });
  }

  function findMetric(aliases) {
    var roots = reportSources();
    var seen = new Set();
    function visit(obj, depth) {
      if (!obj || typeof obj !== 'object' || seen.has(obj) || depth > 5) return undefined;
      seen.add(obj);
      for (var i = 0; i < aliases.length; i++) {
        if (Object.prototype.hasOwnProperty.call(obj, aliases[i]) && obj[aliases[i]] != null && obj[aliases[i]] !== '') return obj[aliases[i]];
      }
      var keys = Object.keys(obj);
      for (var k = 0; k < keys.length; k++) {
        var val = obj[keys[k]];
        if (val && typeof val === 'object') {
          var found = visit(val, depth + 1);
          if (found !== undefined) return found;
        }
      }
      return undefined;
    }
    for (var r = 0; r < roots.length; r++) {
      seen.clear();
      var out = visit(roots[r], 0);
      if (out !== undefined) return out;
    }
    return undefined;
  }

  function asNumber(value) {
    var n = Number(value);
    return Number.isFinite(n) ? n : null;
  }

  function fmtNumber(value, digits) {
    var n = asNumber(value);
    return n == null ? null : n.toFixed(digits == null ? 1 : digits);
  }

  function pctValue(value) {
    var n = asNumber(value);
    if (n == null) return null;
    return n <= 1 ? n * 100 : n;
  }

  function reportMetrics() {
    return {
      verdict: findMetric(['verdict', 'ml_verdict', 'ml_status', 'readiness', 'quality_gate', 'status']),
      hrRmse: findMetric(['hr_rmse', 'hr_rmse_bpm', 'rmse_hr', 'heart_rmse']),
      rrRmse: findMetric(['rr_rmse', 'rr_rmse_brpm', 'rr_rmse_bpm', 'rmse_rr', 'resp_rmse']),
      coverage: findMetric(['coverage', 'valid_coverage', 'coverage_frac', 'valid_fraction', 'valid_ratio', 'hr_coverage']),
      hrR: findMetric(['hr_r', 'hr_corr', 'hr_correlation', 'pearson_hr', 'r_hr']),
      rrR: findMetric(['rr_r', 'rr_corr', 'rr_correlation', 'pearson_rr', 'r_rr']),
      validN: findMetric(['valid_n', 'n_valid', 'valid_samples', 'valid_rows', 'hr_valid_n', 'aligned_valid_n']),
      durationS: findMetric(['duration_s', 'session_duration_s', 'elapsed_s'])
    };
  }

  function metricState(name, value) {
    var n = asNumber(value);
    if (n == null) return '';
    if (name === 'hrRmse') return n <= 3 ? 'good' : (n <= 6 ? 'warn' : 'bad');
    if (name === 'rrRmse') return n <= 2 ? 'good' : (n <= 4 ? 'warn' : 'bad');
    if (name === 'coverage') {
      var p = pctValue(value);
      return p >= 85 ? 'good' : (p >= 60 ? 'warn' : 'bad');
    }
    if (name === 'hrR' || name === 'rrR') return n >= .9 ? 'good' : (n >= .5 ? 'warn' : 'bad');
    if (name === 'validN') return n >= 120 ? 'good' : (n >= 30 ? 'warn' : 'bad');
    return '';
  }

  function summaryLines(metrics) {
    var lines = [];
    var facts = [];
    if (metrics.verdict != null) facts.push('Loaded verdict: ' + String(metrics.verdict) + '.');
    var hr = fmtNumber(metrics.hrRmse, 1);
    if (hr != null) lines.push('HR RMSE is ' + hr + ' bpm against the loaded report target of about 3 bpm.');
    var rr = fmtNumber(metrics.rrRmse, 1);
    if (rr != null) lines.push('RR RMSE is ' + rr + ' br/min against the loaded report target of about 2 br/min.');
    var cov = pctValue(metrics.coverage);
    if (cov != null) lines.push('Valid coverage is ' + cov.toFixed(0) + ' percent; low coverage can make low RMSE misleading.');
    var hrr = fmtNumber(metrics.hrR, 3);
    if (hrr != null) lines.push('HR correlation is r=' + hrr + '; this checks dynamic agreement, not just average error.');
    var rrR = fmtNumber(metrics.rrR, 3);
    if (rrR != null) lines.push('RR correlation is r=' + rrR + '; this checks dynamic agreement, not just average error.');
    var n = asNumber(metrics.validN);
    if (n != null) lines.push('Valid sample count is ' + Math.round(n) + '; short or sparse runs should stay out of ML training.');
    if (!lines.length && !facts.length) {
      lines.push('No report metrics are loaded yet. This summary updates when a session report payload or analysis metrics are present.');
    } else if (!lines.length) {
      lines.push('The loaded report does not expose RMSE, coverage, correlation, or valid-n fields to summarize.');
    }
    return facts.concat(lines).slice(0, 6);
  }

  function metricPills(metrics) {
    var pills = [];
    function add(label, key, value, suffix, digits) {
      var text = key === 'coverage' ? (pctValue(value) == null ? null : pctValue(value).toFixed(0) + '%') : fmtNumber(value, digits == null ? 1 : digits);
      if (text == null) return;
      pills.push('<span class="rvt-report-metric-pill" data-state="' + metricState(key, value) + '">' + esc(label) + ' ' + esc(text + (suffix || '')) + '</span>');
    }
    add('HR RMSE', 'hrRmse', metrics.hrRmse, ' bpm', 1);
    add('RR RMSE', 'rrRmse', metrics.rrRmse, ' br/min', 1);
    add('Coverage', 'coverage', metrics.coverage, '', 0);
    add('HR r', 'hrR', metrics.hrR, '', 3);
    add('RR r', 'rrR', metrics.rrR, '', 3);
    add('Valid n', 'validN', metrics.validN, '', 0);
    return pills.join('');
  }

  function syncReportSummary() {
    var view = reportView();
    if (!view) return;
    var metrics = reportMetrics();
    var anchor = view.querySelector('.verdict-hero') || view.querySelector('.report-panel') || view.querySelector('.card') || view.firstElementChild;
    var summary = document.getElementById('rvtExecSummary');
    if (!summary) {
      summary = document.createElement('section');
      summary.id = 'rvtExecSummary';
      summary.className = 'rvt-exec-summary';
      summary.setAttribute('aria-label', 'Executive summary');
      if (anchor && anchor.parentNode) anchor.parentNode.insertBefore(summary, anchor.nextSibling);
      else view.insertBefore(summary, view.firstChild);
    }
    var lines = summaryLines(metrics);
    summary.innerHTML =
      '<div class="rvt-exec-summary-title"><span class="material-symbols-rounded" aria-hidden="true">summarize</span>Executive summary</div>' +
      '<ul>' + lines.map(function(line){ return '<li><span class="material-symbols-rounded" aria-hidden="true">arrow_right</span><span>' + esc(line) + '</span></li>'; }).join('') + '</ul>' +
      '<div class="rvt-report-metric-row">' + metricPills(metrics) + '</div>';
  }

  function verdictRationale(metrics) {
    var lines = [
      'This explainer reads existing report/session metrics only; it does not call a network service or infer hidden physiology.',
      'Missing fields remain unavailable and are not counted as a pass.',
      'Coverage, valid sample count, RMSE, and correlation are considered together because a low error on a tiny valid subset can be misleading.'
    ];
    if (metrics.verdict != null) lines.unshift('The visible verdict is tied to the loaded report status: ' + String(metrics.verdict) + '.');
    return lines;
  }

  function ensureVerdictExplainers() {
    var view = reportView();
    if (!view) return;
    var metrics = reportMetrics();
    var hero = view.querySelector('.verdict-hero') || view.querySelector('.report-panel') || view.querySelector('.card');
    if (hero && !hero.querySelector(':scope > #rvtVerdictExplainer')) {
      var details = document.createElement('details');
      details.id = 'rvtVerdictExplainer';
      details.className = 'rvt-verdict-explainer';
      hero.appendChild(details);
    }
    var main = document.getElementById('rvtVerdictExplainer');
    if (main) {
      main.innerHTML = '<summary>Why this verdict?</summary><ul>' +
        verdictRationale(metrics).map(function(line){ return '<li>' + esc(line) + '</li>'; }).join('') +
        '</ul>';
    }
    Array.from(view.querySelectorAll('.verdict-card, .diag-card, [data-gate]')).slice(0, 12).forEach(function(card) {
      if (card.querySelector(':scope > .rvt-verdict-explainer')) return;
      var text = (card.textContent || '').trim().replace(/\s+/g, ' ').slice(0, 180);
      var d = document.createElement('details');
      d.className = 'rvt-verdict-explainer';
      d.innerHTML = '<summary>Why this verdict?</summary><div>This card explains the gate/status currently visible in the report. Source text: ' + esc(text || 'no visible card text') + '</div>';
      card.appendChild(d);
    });
  }

  function ensureReportPdfButton() {
    var view = reportView();
    if (!view || document.getElementById('rvtReportPdfButton')) return;
    if (view.querySelector('.rvt-pdf-export-btn')) return;
    var actions = document.createElement('div');
    actions.className = 'rvt-report-actions';
    actions.id = 'rvtReportActionsWorker3';
    actions.innerHTML =
      '<button type="button" class="rvt-report-action rvt-pdf-export-btn" id="rvtReportPdfButton">' +
      '<span class="material-symbols-rounded" aria-hidden="true">picture_as_pdf</span>Export PDF</button>';
    view.insertBefore(actions, view.firstChild);
    actions.querySelector('#rvtReportPdfButton').addEventListener('click', exportReportPdf);
  }

  function exportReportPdf() {
    var view = reportView();
    if (!view) {
      toastUser('Open a report before exporting PDF', 'picture_as_pdf');
      return false;
    }
    if (lastPrintCleanup) {
      try { lastPrintCleanup(); } catch (_) {}
      lastPrintCleanup = null;
    }
    var priorView = document.body.getAttribute('data-view') || '';
    document.body.classList.add('rvt-print-report');
    try {
      if (typeof window.switchView === 'function') window.switchView('report');
    } catch (_) {}
    lastPrintCleanup = function() {
      document.body.classList.remove('rvt-print-report');
      if (priorView && priorView !== 'report' && typeof window.switchView === 'function') {
        try { window.switchView(priorView); } catch (_) {}
      }
    };
    setTimeout(function() {
      try { window.print(); }
      finally { setTimeout(function(){ if (lastPrintCleanup) { lastPrintCleanup(); lastPrintCleanup = null; } }, 600); }
    }, 80);
    return true;
  }

  function syncReport() {
    var view = reportView();
    if (!view) return;
    ensureReportPdfButton();
    syncReportSummary();
    ensureVerdictExplainers();
  }

  function helpView() {
    return document.getElementById('view-help');
  }

  function ensureHelpSearch() {
    var view = helpView();
    if (!view) return;
    var bar = document.getElementById('rvtHelpSearchBar');
    if (!bar) {
      bar = document.createElement('div');
      bar.id = 'rvtHelpSearchBar';
      bar.className = 'rvt-help-searchbar';
      bar.innerHTML =
        '<span class="material-symbols-rounded" aria-hidden="true">search</span>' +
        '<input id="rvtHelpSearchInput" type="search" autocomplete="off" placeholder="Search help, recovery, reports..." aria-label="Search help">' +
        '<span class="rvt-help-search-count" id="rvtHelpSearchCount" aria-live="polite"></span>';
      view.insertBefore(bar, view.firstChild);
      var input = bar.querySelector('input');
      input.value = getScalar(KEYS.helpQuery, '');
      input.addEventListener('input', function() {
        setScalar(KEYS.helpQuery, input.value);
        applyHelpSearch(input.value);
      });
    }
    applyHelpSearch((bar.querySelector('input') || {}).value || '');
  }

  function ensureHelpDeepLinks() {
    var view = helpView();
    if (!view || document.getElementById('rvtHelpDeepLinks')) return;
    var row = document.createElement('div');
    row.id = 'rvtHelpDeepLinks';
    row.className = 'rvt-help-deeplinks';
    row.setAttribute('aria-label', 'Help deep links');
    [
      ['getting_started', 'Getting started'],
      ['hardware_setup', 'Hardware'],
      ['troubleshooting', 'Recovery'],
      ['report_readiness', 'Reports'],
      ['firmware_truthfulness', 'Firmware']
    ].forEach(function(item) {
      var btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'rvt-help-link-btn';
      btn.dataset.rvtHelpTopic = item[0];
      btn.innerHTML = '<span class="material-symbols-rounded" aria-hidden="true">link</span>' + esc(item[1]);
      row.appendChild(btn);
    });
    var after = document.getElementById('rvtHelpSearchBar');
    if (after && after.nextSibling) view.insertBefore(row, after.nextSibling);
    else view.insertBefore(row, view.firstChild);
  }

  function helpSearchBlocks() {
    var view = helpView();
    if (!view) return [];
    var blocks = Array.from(view.children).filter(function(el) {
      return !['rvtHelpSearchBar', 'rvtHelpDeepLinks'].includes(el.id);
    });
    if (blocks.length <= 1) {
      blocks = Array.from(view.querySelectorAll('.card, section, article, .help-topic-card, .help-section')).filter(function(el) {
        return !el.closest('#rvtHelpSearchBar,#rvtHelpDeepLinks');
      });
    }
    return blocks;
  }

  function applyHelpSearch(query) {
    var q = String(query || '').trim().toLowerCase();
    var blocks = helpSearchBlocks();
    var visible = 0;
    blocks.forEach(function(block) {
      var show = !q || (block.textContent || '').toLowerCase().includes(q);
      block.classList.toggle('rvt-search-hidden', !show);
      if (show) visible++;
    });
    var count = document.getElementById('rvtHelpSearchCount');
    if (count) count.textContent = q ? (visible + ' matches') : (blocks.length + ' sections');
  }

  var RECOVERY_ITEMS = [
    ['radar-port', 'Confirm the radar USB cable is seated and the selected serial port is COM10 unless this session intentionally uses another port.'],
    ['ble-address', 'Confirm the BLE reference address is 10:22:33:9E:8F:63 or update the session setup before capture.'],
    ['quickcheck', 'Run Quick Check again after changing hardware, firmware, or BLE settings.'],
    ['stale-state', 'If live data is stale, keep freeze-on-stale enabled and fix the connection instead of switching to silent demo data.'],
    ['report-review', 'After capture, review coverage, valid sample count, RMSE, and correlation before calling the session ML-ready.']
  ];

  function ensureRecoveryChecklist() {
    var view = helpView();
    if (!view || document.getElementById('rvtRecoveryChecklist')) return;
    var card = document.createElement('section');
    card.id = 'rvtRecoveryChecklist';
    card.className = 'rvt-recovery-card';
    card.dataset.rvtHelpTopic = 'troubleshooting';
    card.innerHTML =
      '<div class="rvt-recovery-head"><strong>Interactive recovery checklist</strong><button type="button" class="rvt-help-link-btn" id="rvtRecoveryReset">Reset</button></div>' +
      '<div class="rvt-recovery-progress-track" aria-hidden="true"><span></span></div>' +
      '<div class="rvt-help-search-count" id="rvtRecoveryProgressText" aria-live="polite"></div>' +
      '<ul class="rvt-recovery-list">' +
      RECOVERY_ITEMS.map(function(item) {
        return '<li><label><input type="checkbox" data-recovery-id="' + esc(item[0]) + '"><span>' + esc(item[1]) + '</span></label></li>';
      }).join('') +
      '</ul>';
    var links = document.getElementById('rvtHelpDeepLinks');
    if (links && links.nextSibling) view.insertBefore(card, links.nextSibling);
    else view.insertBefore(card, view.firstChild);
    var saved = getJson(KEYS.helpRecovery, {});
    card.querySelectorAll('input[data-recovery-id]').forEach(function(cb) {
      cb.checked = !!saved[cb.dataset.recoveryId];
      cb.addEventListener('change', function() {
        var state = getJson(KEYS.helpRecovery, {});
        state[cb.dataset.recoveryId] = cb.checked;
        setJson(KEYS.helpRecovery, state);
        syncRecoveryProgress();
      });
    });
    card.querySelector('#rvtRecoveryReset').addEventListener('click', function() {
      setJson(KEYS.helpRecovery, {});
      card.querySelectorAll('input[data-recovery-id]').forEach(function(cb){ cb.checked = false; });
      syncRecoveryProgress();
    });
    syncRecoveryProgress();
  }

  function syncRecoveryProgress() {
    var card = document.getElementById('rvtRecoveryChecklist');
    if (!card) return;
    var boxes = Array.from(card.querySelectorAll('input[data-recovery-id]'));
    var done = boxes.filter(function(cb){ return cb.checked; }).length;
    var pct = boxes.length ? Math.round(done / boxes.length * 100) : 0;
    card.style.setProperty('--rvt-recovery-progress', pct + '%');
    var txt = document.getElementById('rvtRecoveryProgressText');
    if (txt) txt.textContent = done + ' of ' + boxes.length + ' complete';
  }

  function ensureOfflineHelpStatus() {
    var view = helpView();
    if (!view || document.getElementById('rvtHelpOfflineStatus')) return;
    var card = document.createElement('section');
    card.id = 'rvtHelpOfflineStatus';
    card.className = 'rvt-help-offline-card';
    card.innerHTML =
      '<span class="material-symbols-rounded" aria-hidden="true">offline_pin</span>' +
      '<strong>Offline help</strong><span id="rvtHelpOfflineText">Inline help is available from this dashboard file.</span>';
    var recovery = document.getElementById('rvtRecoveryChecklist');
    if (recovery && recovery.nextSibling) view.insertBefore(card, recovery.nextSibling);
    else view.appendChild(card);
    syncOfflineHelpStatus();
    window.addEventListener('online', syncOfflineHelpStatus);
    window.addEventListener('offline', syncOfflineHelpStatus);
    opportunisticHelpMediaCache();
  }

  function syncOfflineHelpStatus() {
    var card = document.getElementById('rvtHelpOfflineStatus');
    var txt = document.getElementById('rvtHelpOfflineText');
    if (!card || !txt) return;
    var offline = navigator.onLine === false;
    var cacheUsable = 'caches' in window && /^https?:$/.test(location.protocol);
    card.dataset.state = offline ? 'offline' : 'online';
    txt.textContent = offline
      ? 'You are offline. Inline recovery, setup, and report guidance remains available.'
      : (cacheUsable ? 'Inline help is available now; same-origin help media is cached opportunistically.' : 'Inline help is available from this dashboard file.');
  }

  function opportunisticHelpMediaCache() {
    if (!('caches' in window) || !/^https?:$/.test(location.protocol)) return;
    var view = helpView();
    if (!view) return;
    var urls = Array.from(view.querySelectorAll('img[src],video[src],source[src]')).map(function(el) {
      try { return new URL(el.getAttribute('src'), location.href); } catch (_) { return null; }
    }).filter(function(url) { return url && url.origin === location.origin; }).map(function(url) { return url.href; });
    if (!urls.length) return;
    caches.open('rvt-help-inline-v1').then(function(cache) {
      urls.slice(0, 24).forEach(function(url) { cache.add(url).catch(function(){}); });
    }).catch(function(){});
  }

  function openHelpTopic(topic, focusId) {
    var safeTopic = String(topic || 'troubleshooting').replace(/[^a-z0-9_-]+/gi, '_');
    try { if (typeof window.switchView === 'function') window.switchView('help'); } catch (_) {}
    try {
      if (window.S && window.S.ctl) window.S.ctl.helpTopic = safeTopic;
      setScalar('rvt-help-topic', safeTopic);
      if (typeof window.setHelpTopic === 'function') window.setHelpTopic(safeTopic);
    } catch (_) {}
    try {
      var hash = '#help:' + encodeURIComponent(safeTopic);
      if (location.hash !== hash) history.replaceState(null, '', hash);
    } catch (_) {}
    setTimeout(function() {
      syncHelp();
      var target = focusId ? document.getElementById(focusId) : document.querySelector('[data-rvt-help-topic="' + safeTopic + '"], [data-help-topic="' + safeTopic + '"]');
      if (!target) {
        var q = safeTopic.replace(/_/g, ' ').toLowerCase();
        target = helpSearchBlocks().find(function(el){ return (el.textContent || '').toLowerCase().includes(q); });
      }
      if (target) {
        target.scrollIntoView({ block: 'center', behavior: window.matchMedia('(prefers-reduced-motion: reduce)').matches ? 'auto' : 'smooth' });
        if (typeof target.focus === 'function') target.focus({ preventScroll: true });
      }
    }, 80);
  }

  function handleInitialHelpDeepLink() {
    var topic = '';
    try {
      var params = new URLSearchParams(location.search || '');
      topic = params.get('help') || params.get('helpTopic') || '';
      var hash = location.hash || '';
      var m = hash.match(/^#help[:=]([^&]+)/i);
      if (m) topic = decodeURIComponent(m[1]);
    } catch (_) {}
    if (topic) openHelpTopic(topic);
  }

  function syncHelp() {
    var view = helpView();
    if (!view) return;
    ensureHelpSearch();
    ensureHelpDeepLinks();
    ensureRecoveryChecklist();
    ensureOfflineHelpStatus();
    applyHelpSearch((document.getElementById('rvtHelpSearchInput') || {}).value || '');
  }

  function settingsHosts() {
    return [document.getElementById('view-settings'), document.querySelector('#settingsOv .set-md')].filter(Boolean);
  }

  function ensureSettingsSearch(host, idx) {
    if (host && host.id === 'view-settings' && document.getElementById('rvtA16Settings')) return;
    if (!host) return;
    var id = 'rvtSettingsSearchBar' + idx;
    var bar = document.getElementById(id);
    if (!bar) {
      bar = document.createElement('div');
      bar.id = id;
      bar.className = 'rvt-settings-searchbar';
      bar.innerHTML =
        '<span class="material-symbols-rounded" aria-hidden="true">manage_search</span>' +
        '<input type="search" autocomplete="off" placeholder="Search settings..." aria-label="Search settings">' +
        '<span class="rvt-settings-search-count" aria-live="polite"></span>';
      var header = host.querySelector(':scope > .set-h');
      var anchor = header && header.parentNode === host ? header.nextSibling : host.firstChild;
      host.insertBefore(bar, anchor || null);
      var input = bar.querySelector('input');
      input.value = getScalar(KEYS.settingsQuery, '');
      input.addEventListener('input', function() {
        setScalar(KEYS.settingsQuery, input.value);
        settingsHosts().forEach(function(h, i){ applySettingsSearch(h, i, input.value); });
      });
    }
    applySettingsSearch(host, idx, (bar.querySelector('input') || {}).value || '');
  }

  function applySettingsSearch(host, idx, query) {
    var q = String(query || '').trim().toLowerCase();
    var rows = Array.from(host.querySelectorAll('.set-r, .set-row, .setting-row, .threshold-field, .reset-row, [data-setting]')).filter(function(row) {
      return !row.closest('.rvt-settings-searchbar,.rvt-settings-profiles-card,.rvt-test-alert-panel');
    });
    var visibleRows = 0;
    rows.forEach(function(row) {
      var show = !q || (row.textContent || '').toLowerCase().includes(q);
      row.classList.toggle('rvt-settings-filter-hidden', !show);
      if (show) visibleRows++;
    });
    Array.from(host.querySelectorAll('.set-g, .settings-group')).forEach(function(group) {
      if (group.closest('.rvt-settings-profiles-card,.rvt-test-alert-panel')) return;
      var groupRows = Array.from(group.querySelectorAll('.set-r, .set-row, .setting-row, .threshold-field, .reset-row, [data-setting]'));
      var hasVisible = !q || !groupRows.length || groupRows.some(function(row){ return !row.classList.contains('rvt-settings-filter-hidden'); });
      group.classList.toggle('rvt-settings-filter-hidden', !hasVisible);
    });
    var count = document.querySelector('#rvtSettingsSearchBar' + idx + ' .rvt-settings-search-count');
    if (count) count.textContent = q ? (visibleRows + ' matches') : (rows.length + ' settings');
  }

  var PROFILE_KEYS = [
    'rvt-theme', 'rvt-density', 'rvt-live-mode', 'rvt-live-buffer-seconds', 'rvt-max-chart-points',
    'rvt-voice-alerts', 'rvt-audio-alerts', 'rvt-audio-volume', 'rvt-voice-readout',
    'rvt-thresholds', 'rvt-kpi-order', 'rvt-dashboard-prefs', 'rvt-setup-prefs',
    'rvt-hx-mode', 'rvt-cbpalette', 'rvt-font-scale', 'rvt-rail-width', 'rvt-rail-state'
  ];

  function loadProfiles() {
    return getJson(KEYS.settingsProfiles, {});
  }

  function saveProfiles(profiles) {
    setJson(KEYS.settingsProfiles, profiles || {});
  }

  function captureSettingsProfile() {
    var keys = new Set(PROFILE_KEYS);
    try {
      Object.keys((window.rvtStorage && window.rvtStorage.manifest) || {}).forEach(function(key) {
        if (/theme|density|live|audio|voice|threshold|kpi|setup|dashboard|hx|font|rail|cbpalette/.test(key)) keys.add(key);
      });
    } catch (_) {}
    var out = {};
    Array.from(keys).forEach(function(key) {
      try { out[key] = localStorage.getItem(key); } catch (_) {}
    });
    return out;
  }

  function applySettingsProfile(name) {
    var profiles = loadProfiles();
    var profile = profiles[name];
    if (!profile || !profile.settings) return false;
    Object.keys(profile.settings).forEach(function(key) {
      var val = profile.settings[key];
      if (val == null) removeScalar(key);
      else setScalar(key, val);
    });
    try {
      if (profile.settings['rvt-theme'] && typeof window.setThemeMode === 'function') window.setThemeMode(profile.settings['rvt-theme']);
      if (profile.settings['rvt-density'] && typeof window.setDensity === 'function') window.setDensity(profile.settings['rvt-density']);
      if (profile.settings['rvt-live-mode'] && typeof window.setLiveMode === 'function') window.setLiveMode(profile.settings['rvt-live-mode']);
    } catch (_) {}
    toastUser('Settings profile applied. Reload if a setting needs a full restart.', 'manage_accounts');
    syncSettingsProfiles();
    return true;
  }

  function deleteSettingsProfile(name) {
    var profiles = loadProfiles();
    delete profiles[name];
    saveProfiles(profiles);
    syncSettingsProfiles();
  }

  function saveSettingsProfileFromCard(card) {
    if (!card) return false;
    var input = card.querySelector('.rvt-profile-name-input');
    var name = (input && input.value || '').trim();
    if (!name) {
      toastUser('Enter a profile name', 'manage_accounts');
      if (input) input.focus();
      return false;
    }
    var profiles = loadProfiles();
    profiles[name] = { version: 1, ts: Date.now(), settings: captureSettingsProfile() };
    saveProfiles(profiles);
    if (input) input.value = '';
    syncSettingsProfiles();
    toastUser('Settings profile saved', 'save');
    return true;
  }

  function ensureSettingsProfiles(host) {
    if (host && host.id === 'view-settings' && document.getElementById('rvtA16Settings')) return;
    if (!host || host.querySelector('.rvt-settings-profiles-card[data-worker3="1"]')) return;
    var card = document.createElement('section');
    card.className = 'rvt-settings-profiles-card set-g';
    card.dataset.worker3 = '1';
    card.innerHTML =
      '<div class="rvt-settings-profile-head"><strong>Settings profiles</strong><span class="rvt-settings-search-count">Local only</span></div>' +
      '<div class="rvt-profile-controls">' +
      '<input class="rvt-profile-name-input" type="text" maxlength="48" placeholder="Profile name" aria-label="Settings profile name">' +
      '<button type="button" class="rvt-settings-profile-btn" data-profile-save><span class="material-symbols-rounded" aria-hidden="true">save</span>Save</button>' +
      '</div>' +
      '<div class="rvt-profile-list" data-profile-list></div>';
    var actions = host.querySelector(':scope > .set-actions');
    if (actions && actions.parentNode === host) host.insertBefore(card, actions);
    else host.appendChild(card);
    card.querySelector('[data-profile-save]').addEventListener('click', function(ev) {
      if (ev) ev.__rvtWorker3ProfileHandled = true;
      saveSettingsProfileFromCard(card);
    });
    renderSettingsProfileList(card);
  }

  function renderSettingsProfileList(card) {
    var list = card.querySelector('[data-profile-list]');
    if (!list) return;
    var profiles = loadProfiles();
    var names = Object.keys(profiles).sort();
    if (!names.length) {
      list.innerHTML = '<span class="rvt-profile-empty">No saved profiles</span>';
      return;
    }
    list.innerHTML = names.map(function(name) {
      var ts = profiles[name] && profiles[name].ts ? new Date(profiles[name].ts).toLocaleDateString() : 'saved';
      return '<span class="rvt-profile-chip-wrap">' +
        '<button type="button" class="rvt-settings-profile-btn" data-profile-apply="' + esc(name) + '"><span class="material-symbols-rounded" aria-hidden="true">settings_backup_restore</span>' + esc(name) + '</button>' +
        '<button type="button" class="rvt-settings-profile-btn" data-profile-delete="' + esc(name) + '" title="Delete ' + esc(name) + '"><span class="material-symbols-rounded" aria-hidden="true">delete</span>' + esc(ts) + '</button>' +
        '</span>';
    }).join('');
    list.querySelectorAll('[data-profile-apply]').forEach(function(btn) {
      btn.addEventListener('click', function(ev){
        if (ev) ev.__rvtWorker3ProfileHandled = true;
        applySettingsProfile(btn.dataset.profileApply);
      });
    });
    list.querySelectorAll('[data-profile-delete]').forEach(function(btn) {
      btn.addEventListener('click', function(ev){
        if (ev) ev.__rvtWorker3ProfileHandled = true;
        var name = btn.dataset.profileDelete;
        if (window.confirm('Delete settings profile "' + name + '"?')) deleteSettingsProfile(name);
      });
    });
  }

  function syncSettingsProfiles() {
    document.querySelectorAll('.rvt-settings-profiles-card[data-worker3="1"]').forEach(renderSettingsProfileList);
  }

  function playTestSound() {
    try {
      if (typeof window.testAudioAlert === 'function') {
        window.testAudioAlert();
        return;
      }
      var AudioContext = window.AudioContext || window.webkitAudioContext;
      if (!AudioContext) return;
      var ctx = new AudioContext();
      var osc = ctx.createOscillator();
      var gain = ctx.createGain();
      osc.type = 'sine';
      osc.frequency.value = 880;
      gain.gain.value = .08;
      osc.connect(gain);
      gain.connect(ctx.destination);
      osc.start();
      setTimeout(function(){ osc.stop(); ctx.close().catch(function(){}); }, 180);
    } catch (err) {
      softWarn('test sound failed', err);
    }
  }

  function testVisualAlert() {
    document.body.classList.remove('rvt-test-alert-flash');
    void document.body.offsetWidth;
    document.body.classList.add('rvt-test-alert-flash');
    setTimeout(function(){ document.body.classList.remove('rvt-test-alert-flash'); }, 500);
  }

  function testHapticAlert() {
    try {
      if (window.App && window.App.hx && typeof window.App.hx.warn === 'function') window.App.hx.warn();
      else if (navigator.vibrate) navigator.vibrate([80, 40, 80]);
    } catch (_) {}
  }

  function testVoiceAlert() {
    try {
      if (!window.speechSynthesis || !window.SpeechSynthesisUtterance) return;
      var utter = new SpeechSynthesisUtterance('Test alert from Radar Vital Trainer.');
      utter.rate = 1;
      utter.volume = .8;
      window.speechSynthesis.cancel();
      window.speechSynthesis.speak(utter);
    } catch (err) {
      softWarn('test voice failed', err);
    }
  }

  function pushDrawerTestAlert() {
    var testAlert = {
      severity: 'warn',
      title: 'Test alert',
      copy: 'Operator-triggered settings test. This is not live telemetry.',
      meta: 'Settings test'
    };
    try {
      if (!window.S) window.S = {};
      window.S.lastAlerts = [testAlert].concat(Array.isArray(window.S.lastAlerts) ? window.S.lastAlerts : []).slice(0, 20);
      if (typeof window.renderAlerts === 'function') window.renderAlerts(window.S.lastAlerts);
      if (typeof window.openDrawer === 'function') window.openDrawer();
      syncAlerts();
    } catch (err) {
      softWarn('drawer test alert failed', err);
    }
  }

  function runTestAlertKind(kind) {
    if (kind === 'visual') testVisualAlert();
    else if (kind === 'sound') playTestSound();
    else if (kind === 'haptic') testHapticAlert();
    else if (kind === 'voice') testVoiceAlert();
    else if (kind === 'drawer') pushDrawerTestAlert();
  }

  function ensureTestAlertPanel(host) {
    if (host && host.id === 'view-settings' && document.getElementById('rvtA16Settings')) return;
    if (!host || host.querySelector('.rvt-test-alert-panel[data-worker3="1"]')) return;
    var panel = document.createElement('section');
    panel.className = 'rvt-test-alert-panel set-g';
    panel.dataset.worker3 = '1';
    panel.innerHTML =
      '<div class="rvt-test-alert-head"><strong>Test alerts</strong><span class="rvt-settings-search-count">No telemetry is changed</span></div>' +
      '<div class="rvt-test-alert-actions">' +
      '<button type="button" class="rvt-test-alert-btn2" data-test-alert="visual"><span class="material-symbols-rounded" aria-hidden="true">visibility</span>Visual</button>' +
      '<button type="button" class="rvt-test-alert-btn2" data-test-alert="sound"><span class="material-symbols-rounded" aria-hidden="true">volume_up</span>Sound</button>' +
      '<button type="button" class="rvt-test-alert-btn2" data-test-alert="haptic"><span class="material-symbols-rounded" aria-hidden="true">vibration</span>Haptic</button>' +
      '<button type="button" class="rvt-test-alert-btn2" data-test-alert="voice"><span class="material-symbols-rounded" aria-hidden="true">record_voice_over</span>Voice</button>' +
      '<button type="button" class="rvt-test-alert-btn2" data-test-alert="drawer"><span class="material-symbols-rounded" aria-hidden="true">notifications</span>Drawer alert</button>' +
      '</div>';
    var profileCard = host.querySelector('.rvt-settings-profiles-card[data-worker3="1"]');
    if (profileCard && profileCard.nextSibling) host.insertBefore(panel, profileCard.nextSibling);
    else host.appendChild(panel);
    panel.addEventListener('click', function(e) {
      var btn = e.target.closest('[data-test-alert]');
      if (!btn) return;
      e.__rvtWorker3TestHandled = true;
      runTestAlertKind(btn.dataset.testAlert);
    });
  }

  function syncSettings() {
    settingsHosts().forEach(function(host, idx) {
      ensureSettingsSearch(host, idx);
      ensureSettingsProfiles(host);
      ensureTestAlertPanel(host);
      applySettingsSearch(host, idx, (document.querySelector('#rvtSettingsSearchBar' + idx + ' input') || {}).value || '');
    });
  }

  function syncWorker3Beta() {
    registerStorageKeys();
    wrapAlertEntrypoints();
    syncAlerts();
    syncReport();
    syncHelp();
    syncSettings();
  }

  function installEventBindings() {
    document.addEventListener('click', function(e) {
      var helpBtn = e.target.closest('[data-rvt-help-topic]');
      if (helpBtn) {
        e.preventDefault();
        openHelpTopic(helpBtn.dataset.rvtHelpTopic);
      }
      if (!e.__rvtWorker3ProfileHandled) {
        var saveBtn = e.target.closest('[data-profile-save]');
        var applyBtn = e.target.closest('[data-profile-apply]');
        var deleteBtn = e.target.closest('[data-profile-delete]');
        if (saveBtn) {
          saveSettingsProfileFromCard(saveBtn.closest('.rvt-settings-profiles-card'));
        } else if (applyBtn) {
          applySettingsProfile(applyBtn.dataset.profileApply);
        } else if (deleteBtn) {
          var name = deleteBtn.dataset.profileDelete;
          if (window.confirm('Delete settings profile "' + name + '"?')) deleteSettingsProfile(name);
        }
      }
      if (!e.__rvtWorker3TestHandled) {
        var testBtn = e.target.closest('[data-test-alert]');
        if (testBtn) runTestAlertKind(testBtn.dataset.testAlert);
      }
    });
    document.addEventListener('keydown', function(e) {
      if (e.key !== '/' || e.ctrlKey || e.metaKey || e.altKey) return;
      if (document.body.getAttribute('data-view') !== 'help') return;
      var target = e.target;
      if (target && /input|textarea|select/i.test(target.tagName || '')) return;
      var input = document.getElementById('rvtHelpSearchInput');
      if (input) {
        e.preventDefault();
        input.focus();
      }
    });
    window.addEventListener('hashchange', handleInitialHelpDeepLink);
    try {
      var obs = new MutationObserver(function(){ scheduleSync(60); });
      obs.observe(document.body, { childList: true, subtree: true, attributes: true, attributeFilter: ['data-view', 'class', 'hidden', 'style'] });
      if (window.S) {
        if (!Array.isArray(window.S.__mutationObservers)) window.S.__mutationObservers = [];
        window.S.__mutationObservers.push(obs);
      }
    } catch (_) {}
  }

  function scheduleSync(delay) {
    if (syncTimer) clearTimeout(syncTimer);
    syncTimer = setTimeout(function() {
      syncTimer = null;
      try { syncWorker3Beta(); } catch (err) { softWarn('sync failed', err); }
    }, delay || 100);
  }

  function boot() {
    registerStorageKeys();
    installEventBindings();
    syncWorker3Beta();
    handleInitialHelpDeepLink();
    setInterval(function(){ try { syncWorker3Beta(); } catch (err) { softWarn('interval sync failed', err); } }, 2500);
    window.rvtWorker3BetaReady = true;
  }

  window.rvtSyncWorker3Beta = syncWorker3Beta;
  window.rvtBulkAcknowledgeAlerts = bulkAcknowledgeVisibleAlerts;
  window.rvtOpenHelpTopic = openHelpTopic;
  window.rvtExportReportPdf = exportReportPdf;
  window.rvtCaptureSettingsProfile = captureSettingsProfile;
  window.rvtApplySettingsProfile = applySettingsProfile;

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot, 350); }, { once: true });
  } else {
    setTimeout(boot, 350);
  }
})();
;
/* END modules/rvt-worker3-beta-js.js */

/* BEGIN modules/rvt-sec02-autolock-js.js */
/* SEC-02: Auto-lock on Idle */
(function() {
  'use strict';

  /* Register storage keys with the manifest */
  function registerKeys() {
    var m = window.rvtStorage && window.rvtStorage.manifest;
    if (!m) return;
    m['rvt-idle-timeout-min'] = { kind: 'scalar', defaultValue: '5',
      validate: function(v){ var n=Number(v); return Number.isFinite(n) && n>=0 && n<=60; } };
    m['rvt-lock-keepalive'] = { kind: 'scalar', defaultValue: '1',
      validate: function(v){ return v==='0'||v==='1'; } };
    m['rvt-lock-pin-hash'] = { kind: 'scalar', defaultValue: '',
      validate: function(v){ return typeof v==='string' && v.length<=128; } };
  }

  var ACTIVITY_EVENTS = ['mousemove','keydown','click','touchstart','pointerdown','scroll'];
  var state = { locked: false, lastActivity: Date.now(), timer: null, overlay: null };

  function getTimeoutMs() {
    var stored = null;
    try { stored = localStorage.getItem('rvt-idle-timeout-min'); } catch(_) {}
    var min = Number(stored || '5') || 5;
    return min <= 0 ? 0 : min * 60000;
  }

  function isKeepAlive() {
    try {
      var ka = localStorage.getItem('rvt-lock-keepalive');
      if (ka === '0') return false;
    } catch(_) {}
    try {
      var s = window.S;
      if (s && s.disc !== undefined && s.disc < 3) return true;
      if (s && s.ctl && s.ctl.session && s.ctl.session.active) return true;
    } catch(_) {}
    return false;
  }

  function hashPin(pin) {
    return crypto.subtle.digest('SHA-256', new TextEncoder().encode('rvt:lock:' + pin))
      .then(function(buf) {
        return Array.from(new Uint8Array(buf))
          .map(function(b){ return b.toString(16).padStart(2,'0'); }).join('');
      });
  }

  function buildOverlay() {
    var el = document.getElementById('rvt-lock-overlay');
    if (el) { state.overlay = el; return el; }
    el = document.createElement('div');
    el.id = 'rvt-lock-overlay';
    el.setAttribute('role', 'dialog');
    el.setAttribute('aria-modal', 'true');
    el.setAttribute('aria-labelledby', 'rvtLockTitle');
    el.innerHTML =
      '<span class="rvt-lock-icon material-symbols-rounded" aria-hidden="true">lock</span>' +
      '<h2 class="rvt-lock-title" id="rvtLockTitle">Session paused</h2>' +
      '<p class="rvt-lock-subtitle">Authenticate to resume monitoring. Data collection continues in the background.</p>' +
      '<div class="rvt-lock-pin-row">' +
        '<input id="rvtLockPinInput" type="password" placeholder="PIN (optional)" autocomplete="off" maxlength="16" aria-label="Enter PIN to unlock" />' +
        '<button id="rvtLockUnlockBtn" type="button">Resume</button>' +
      '</div>' +
      '<p class="rvt-lock-no-pin-hint" id="rvtLockNoPinHint">No PIN set — click Resume to continue.</p>' +
      '<p class="rvt-lock-err" id="rvtLockErr" role="alert" aria-live="polite"></p>' +
      '<p class="rvt-lock-timer" id="rvtLockTimer" aria-live="polite"></p>';
    document.body.appendChild(el);

    var btn   = el.querySelector('#rvtLockUnlockBtn');
    var input = el.querySelector('#rvtLockPinInput');
    var errEl = el.querySelector('#rvtLockErr');
    var hint  = el.querySelector('#rvtLockNoPinHint');

    function tryUnlock() {
      var stored = null;
      try { stored = localStorage.getItem('rvt-lock-pin-hash') || ''; } catch(_) {}
      if (!stored) { unlock(); return; }
      var entered = input.value;
      if (!entered) { errEl.textContent = 'Enter your PIN.'; input.focus(); return; }
      hashPin(entered).then(function(h) {
        if (h === stored) { errEl.textContent=''; input.value=''; unlock(); }
        else { errEl.textContent='Incorrect PIN.'; input.value=''; input.focus(); }
      });
    }

    function syncHint() {
      var stored = null;
      try { stored = localStorage.getItem('rvt-lock-pin-hash') || ''; } catch(_) {}
      hint.style.display = stored ? 'none' : '';
      input.style.display = stored ? '' : 'none';
    }

    btn.addEventListener('click', tryUnlock);
    input.addEventListener('keydown', function(e){ if (e.key==='Enter') tryUnlock(); });
    el.addEventListener('focus', syncHint, true);
    syncHint();

    state.overlay = el;
    return el;
  }

  function lock() {
    if (state.locked) return;
    state.locked = true;
    document.body.dataset.autoLocked = 'true';
    var ov = buildOverlay();
    ov.style.display = 'flex';
    requestAnimationFrame(function(){ ov.classList.add('rvt-lock-visible'); });
    var inp = ov.querySelector('#rvtLockPinInput');
    if (inp) setTimeout(function(){ inp.focus(); }, 120);

    var lockAt = Date.now();
    var timerEl = ov.querySelector('#rvtLockTimer');
    var tID = setInterval(function(){
      if (!state.locked){ clearInterval(tID); if(timerEl) timerEl.textContent=''; return; }
      var mins = Math.floor((Date.now()-lockAt)/60000);
      if (timerEl) timerEl.textContent = mins>0 ? 'Locked for '+mins+' min' : '';
    }, 30000);

    if (window.rvtAnnounce) window.rvtAnnounce('Session paused due to inactivity.');
  }

  function unlock() {
    state.locked = false;
    state.lastActivity = Date.now();
    delete document.body.dataset.autoLocked;
    var ov = state.overlay;
    if (ov) {
      ov.classList.remove('rvt-lock-visible');
      setTimeout(function(){ if (!state.locked) ov.style.display='none'; }, 220);
    }
  }

  function resetActivity(){ state.lastActivity = Date.now(); }

  function tick() {
    if (state.locked) return;
    var ms = getTimeoutMs();
    if (!ms) return;
    if (isKeepAlive()) return;
    if (Date.now() - state.lastActivity >= ms) lock();
  }

  function boot() {
    registerKeys();
    ACTIVITY_EVENTS.forEach(function(ev){
      document.addEventListener(ev, resetActivity, { passive: true, capture: true });
    });
    state.timer = setInterval(tick, 10000);
    buildOverlay();

    window.rvtAutoLock = {
      lock: lock,
      unlock: unlock,
      isLocked: function(){ return state.locked; },
      setPin: function(pin){
        if (!pin){ try{ localStorage.removeItem('rvt-lock-pin-hash'); }catch(_){} return Promise.resolve(); }
        return hashPin(pin).then(function(h){ try{ localStorage.setItem('rvt-lock-pin-hash',h); }catch(_){} });
      },
      clearPin: function(){ try{ localStorage.removeItem('rvt-lock-pin-hash'); }catch(_){} },
      hasPin: function(){ try{ return !!(localStorage.getItem('rvt-lock-pin-hash')); }catch(_){ return false; } },
      getTimeoutMin: function(){ return Number(getTimeoutMs()/60000); },
      setTimeoutMin: function(m){
        var clamped = Math.max(0,Math.min(60,Number(m)||0));
        try{ localStorage.setItem('rvt-idle-timeout-min',String(clamped)); }catch(_){}
      }
    };
  }

  if (document.readyState==='loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot,500); }, {once:true});
  } else { setTimeout(boot,500); }
})();
;
/* END modules/rvt-sec02-autolock-js.js */

/* BEGIN modules/rvt-a11y03-canvas-tables-js.js */
/* A11Y-03: Data-table Alternative for Canvases */
(function() {
  'use strict';

  var TABLE_CLASS = 'rvt-a11y-chart-table';
  var SHOW_CLASS  = 'rvt-a11y-show-tables';
  var tablesVisible = false;

  function formatVal(v) {
    if (v == null) return '—';
    if (typeof v === 'number') return isNaN(v) ? '—' : v.toFixed(1);
    if (typeof v === 'object' && v !== null && 'y' in v) return formatVal(v.y);
    return String(v);
  }

  function buildOrUpdateTable(chartKey, chart) {
    if (!chart || !chart.canvas) return;
    var canvas = chart.canvas;
    var tableId = 'rvt-a11y-tbl-' + chartKey;
    var table = document.getElementById(tableId);

    if (!table) {
      table = document.createElement('table');
      table.id = tableId;
      table.className = TABLE_CLASS;
      /* insert immediately after canvas wrapper or canvas itself */
      var anchor = canvas.parentElement || canvas;
      anchor.insertAdjacentElement('afterend', table);
    }

    var data = chart.data || {};
    var datasets = data.datasets || [];
    var labels   = data.labels || [];
    var label    = canvas.getAttribute('aria-label') || chartKey;

    /* Only show last 10 labels to keep table manageable */
    var startIdx = Math.max(0, labels.length - 10);
    var slicedLabels = labels.slice(startIdx);

    var html = '<caption>' + label + '</caption>';
    if (datasets.length) {
      html += '<thead><tr><th scope="col">Time</th>';
      datasets.forEach(function(ds){ html += '<th scope="col">' + (ds.label || 'Value') + '</th>'; });
      html += '</tr></thead><tbody>';
      slicedLabels.forEach(function(lbl, i) {
        var absIdx = startIdx + i;
        html += '<tr><th scope="row">' + (lbl || absIdx) + '</th>';
        datasets.forEach(function(ds){
          html += '<td>' + formatVal(ds.data && ds.data[absIdx]) + '</td>';
        });
        html += '</tr>';
      });
      html += '</tbody>';
    } else {
      html += '<tbody><tr><td>No data</td></tr></tbody>';
    }

    table.innerHTML = html;
    table.setAttribute('aria-label', label + ' data table');
  }

  function updateAllTables() {
    var charts = window.S && window.S.charts;
    if (!charts) return;
    Object.keys(charts).forEach(function(k){
      var ch = charts[k];
      if (ch && !ch.isDestroyed) buildOrUpdateTable(k, ch);
    });
  }

  function toggleTableVisibility(force) {
    tablesVisible = (force !== undefined) ? !!force : !tablesVisible;
    if (tablesVisible) {
      document.body.classList.add(SHOW_CLASS);
    } else {
      document.body.classList.remove(SHOW_CLASS);
    }
    return tablesVisible;
  }

  function boot() {
    /* Update tables on a 1-second cadence during live mode */
    setInterval(updateAllTables, 1000);

    window.rvtA11yCanvasTables = {
      update: updateAllTables,
      show:   function(){ return toggleTableVisibility(true); },
      hide:   function(){ return toggleTableVisibility(false); },
      toggle: function(){ return toggleTableVisibility(); },
      isVisible: function(){ return tablesVisible; }
    };
  }

  if (document.readyState==='loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot,600); }, {once:true});
  } else { setTimeout(boot,600); }
})();
;
/* END modules/rvt-a11y03-canvas-tables-js.js */

/* BEGIN modules/rvt-qol05-destructive-modals-js.js */
/* QOL-05: Destructive-action Diff Modals */
(function() {
  'use strict';

  var backdrop = null;

  function getBackdrop() {
    if (backdrop) return backdrop;
    var el = document.getElementById('rvt-diff-modal-backdrop');
    if (!el) {
      el = document.createElement('div');
      el.id = 'rvt-diff-modal-backdrop';
      el.setAttribute('role', 'dialog');
      el.setAttribute('aria-modal', 'true');
      el.setAttribute('aria-labelledby', 'rvtDiffTitle');
      el.innerHTML =
        '<div id="rvt-diff-modal">' +
          '<h3 class="rvt-diff-title" id="rvtDiffTitle"></h3>' +
          '<div class="rvt-diff-items" id="rvtDiffItems"></div>' +
          '<div class="rvt-diff-check-row" id="rvtDiffCheckRow" style="display:none">' +
            '<input type="checkbox" id="rvtDiffChk" />' +
            '<label for="rvtDiffChk" id="rvtDiffChkLabel"></label>' +
          '</div>' +
          '<div class="rvt-diff-phrase-row" id="rvtDiffPhraseRow" style="display:none">' +
            '<label id="rvtDiffPhraseLabel"></label>' +
            '<input type="text" id="rvtDiffPhraseInput" autocomplete="off" />' +
          '</div>' +
          '<div class="rvt-diff-btn-row">' +
            '<button class="rvt-diff-cancel-btn" id="rvtDiffCancelBtn">Cancel</button>' +
            '<button class="rvt-diff-confirm-btn" id="rvtDiffConfirmBtn" disabled>Confirm</button>' +
          '</div>' +
        '</div>';
      document.body.appendChild(el);
    }
    backdrop = el;
    return el;
  }

  /**
   * rvtConfirmDestructive(opts) — show a diff modal before a destructive action
   * opts.title        {string}   Modal heading
   * opts.items        {string[]} What will be lost (list items)
   * opts.impact       {'low'|'high'} — low: checkbox only; high: typed phrase
   * opts.phrase       {string}   Required phrase for high-impact (default 'confirm')
   * opts.onConfirm    {function} Called if user confirms
   * opts.onCancel     {function} Called if user cancels (optional)
   */
  function rvtConfirmDestructive(opts) {
    opts = opts || {};
    var bd = getBackdrop();
    var modal       = bd.querySelector('#rvt-diff-modal');
    var titleEl     = bd.querySelector('#rvtDiffTitle');
    var itemsEl     = bd.querySelector('#rvtDiffItems');
    var checkRow    = bd.querySelector('#rvtDiffCheckRow');
    var chk         = bd.querySelector('#rvtDiffChk');
    var chkLabel    = bd.querySelector('#rvtDiffChkLabel');
    var phraseRow   = bd.querySelector('#rvtDiffPhraseRow');
    var phraseLabel = bd.querySelector('#rvtDiffPhraseLabel');
    var phraseInput = bd.querySelector('#rvtDiffPhraseInput');
    var cancelBtn   = bd.querySelector('#rvtDiffCancelBtn');
    var confirmBtn  = bd.querySelector('#rvtDiffConfirmBtn');

    var items  = opts.items || [];
    var impact = opts.impact || 'low';
    var phrase = (opts.phrase || 'confirm').toLowerCase();

    /* Populate */
    titleEl.textContent = opts.title || 'Confirm action';
    var itemsHtml = '<p>This will permanently remove:</p><ul>';
    items.forEach(function(it){ itemsHtml += '<li>' + it + '</li>'; });
    if (!items.length) itemsHtml += '<li>Selected data</li>';
    itemsHtml += '</ul>';
    itemsEl.innerHTML = itemsHtml;

    /* Reset controls */
    chk.checked = false;
    phraseInput.value = '';
    confirmBtn.disabled = true;

    if (impact === 'high') {
      checkRow.style.display   = 'none';
      phraseRow.style.display  = '';
      phraseLabel.textContent  = 'Type "' + phrase + '" to confirm:';
      confirmBtn.textContent   = 'Permanently delete';
    } else {
      checkRow.style.display   = '';
      phraseRow.style.display  = 'none';
      chkLabel.textContent     = 'I understand this cannot be undone.';
      confirmBtn.textContent   = 'Confirm';
    }

    function updateConfirmState() {
      if (impact === 'high') {
        confirmBtn.disabled = phraseInput.value.trim().toLowerCase() !== phrase;
      } else {
        confirmBtn.disabled = !chk.checked;
      }
    }

    chk.onchange = updateConfirmState;
    phraseInput.oninput = updateConfirmState;

    function close(confirmed) {
      bd.classList.remove('rvt-diff-visible');
      setTimeout(function(){ bd.style.display='none'; }, 180);
      chk.onchange = null; phraseInput.oninput = null;
      cancelBtn.onclick = null; confirmBtn.onclick = null;
      if (confirmed && typeof opts.onConfirm === 'function') opts.onConfirm();
      else if (!confirmed && typeof opts.onCancel === 'function') opts.onCancel();
    }

    cancelBtn.onclick  = function(){ close(false); };
    confirmBtn.onclick = function(){ if (!confirmBtn.disabled) close(true); };

    bd.onclick = function(e){ if (e.target === bd) close(false); };

    bd.style.display = 'flex';
    bd.classList.add('rvt-diff-visible');
    cancelBtn.focus();
  }

  /* Wrap clearSnaps — the primary destructive action in the dashboard */
  function wrapClearSnaps() {
    if (typeof window.clearSnaps !== 'function') return false;
    if (window.clearSnaps.__rvtQol05Wrapped) return true;
    var orig = window.clearSnaps;
    window.clearSnaps = function() {
      var snaps = (window.S && Array.isArray(window.S.snaps)) ? window.S.snaps.length : 0;
      var notes = {};
      try { notes = JSON.parse(localStorage.getItem('rvt-snap-notes') || '{}') || {}; } catch(_) {}
      var noteCount = Object.keys(notes).length;
      var pins  = [];
      try { pins  = JSON.parse(localStorage.getItem('rvt-alert-pins') || '[]') || []; } catch(_) {}
      var items = [];
      if (snaps)     items.push(snaps + ' snapshot' + (snaps===1?'':'s'));
      if (noteCount) items.push(noteCount + ' annotation' + (noteCount===1?'':'s'));
      if (pins.length) items.push(pins.length + ' pinned alert' + (pins.length===1?'':'s'));
      rvtConfirmDestructive({
        title:     'Clear all snapshots',
        items:     items,
        impact:    snaps > 10 ? 'high' : 'low',
        phrase:    'delete',
        onConfirm: function(){ orig.call(window); }
      });
    };
    window.clearSnaps.__rvtQol05Wrapped = true;
    return true;
  }

  /* Wrap rvtBulkAcknowledgeAlerts */
  function wrapBulkAck() {
    if (typeof window.rvtBulkAcknowledgeAlerts !== 'function') return false;
    if (window.rvtBulkAcknowledgeAlerts.__rvtQol05Wrapped) return true;
    var orig = window.rvtBulkAcknowledgeAlerts;
    window.rvtBulkAcknowledgeAlerts = function() {
      var alertEls = document.querySelectorAll('.alert-item:not([data-acked="true"])');
      var count = alertEls.length;
      rvtConfirmDestructive({
        title:     'Acknowledge all alerts',
        items:     [count + ' unacknowledged alert' + (count===1?'':'s')],
        impact:    'low',
        onConfirm: function(){ orig.apply(window, arguments); }
      });
    };
    window.rvtBulkAcknowledgeAlerts.__rvtQol05Wrapped = true;
    return true;
  }

  /* Wrap rvtPurgeArchivedSessions */
  function wrapPurgeArchived() {
    if (typeof window.rvtPurgeArchivedSessions !== 'function') return false;
    if (window.rvtPurgeArchivedSessions.__rvtQol05Wrapped) return true;
    var orig = window.rvtPurgeArchivedSessions;
    window.rvtPurgeArchivedSessions = function() {
      rvtConfirmDestructive({
        title:     'Purge archived sessions',
        items:     ['All archived session records from local storage'],
        impact:    'high',
        phrase:    'purge',
        onConfirm: function(){ orig.apply(window, arguments); }
      });
    };
    window.rvtPurgeArchivedSessions.__rvtQol05Wrapped = true;
    return true;
  }

  function installWraps() {
    var done = wrapClearSnaps() && wrapBulkAck() && wrapPurgeArchived();
    if (!done) setTimeout(installWraps, 300);
  }

  function boot() {
    getBackdrop(); /* build DOM now */
    installWraps();
    window.rvtConfirmDestructive = rvtConfirmDestructive;
  }

  if (document.readyState==='loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot,700); }, {once:true});
  } else { setTimeout(boot,700); }
})();
;
/* END modules/rvt-qol05-destructive-modals-js.js */

/* BEGIN modules/rvt-snap01-gallery-grid-js.js */
(function() {
  var VIEW_MODE_KEY = 'rvt-snap-view-mode';
  var currentViewMode = 'grid';
  var snapsList = null;
  var toggleBtn = null;
  var observer = null;

  function getSavedViewMode() {
    try {
      var saved = localStorage.getItem(VIEW_MODE_KEY);
      return (saved === 'list' || saved === 'grid') ? saved : 'grid';
    } catch(e) { return 'grid'; }
  }

  function saveViewMode(mode) {
    try { localStorage.setItem(VIEW_MODE_KEY, mode); } catch(e) {}
  }

  function setViewMode(mode) {
    if (!snapsList || (mode !== 'list' && mode !== 'grid')) return;
    currentViewMode = mode;

    snapsList.classList.remove('rvt-snap-gallery-grid', 'rvt-snap-gallery-list');
    snapsList.classList.add(mode === 'grid' ? 'rvt-snap-gallery-grid' : 'rvt-snap-gallery-list');

    if (toggleBtn) {
      toggleBtn.classList.remove('rvt-snap-grid-mode', 'rvt-snap-list-mode');
      toggleBtn.classList.add(mode === 'grid' ? 'rvt-snap-grid-mode' : 'rvt-snap-list-mode');
      toggleBtn.setAttribute('aria-label', mode === 'grid' ? 'Switch to list view' : 'Switch to grid view');
      toggleBtn.setAttribute('title', mode === 'grid' ? 'Switch to list view' : 'Switch to grid view');
    }

    saveViewMode(mode);
  }

  function updateGridColumns() {
    if (!snapsList) return;
    var w = window.innerWidth;
    if (currentViewMode === 'grid') {
      if (w >= 1280) {
        snapsList.style.gridTemplateColumns = 'repeat(4, 1fr)';
      } else if (w >= 768) {
        snapsList.style.gridTemplateColumns = 'repeat(2, 1fr)';
      } else {
        snapsList.style.gridTemplateColumns = '1fr';
      }
    }
  }

  function ensureToggleButton() {
    if (toggleBtn) return;

    var header = document.querySelector('[data-view="snapshots"] .view-header, .snapshots-header, [data-snap-list-header]');
    if (!header) {
      /* fallback: find snapshots container and prepend button */
      var cont = document.querySelector('[data-view="snapshots"]');
      if (!cont) return;
      header = cont;
    }

    toggleBtn = document.createElement('button');
    toggleBtn.id = 'rvt-snap-view-toggle';
    toggleBtn.type = 'button';
    toggleBtn.className = currentViewMode === 'grid' ? 'rvt-snap-grid-mode' : 'rvt-snap-list-mode';
    toggleBtn.setAttribute('aria-label', currentViewMode === 'grid' ? 'Switch to list view' : 'Switch to grid view');
    toggleBtn.setAttribute('title', currentViewMode === 'grid' ? 'Switch to list view' : 'Switch to grid view');

    toggleBtn.onclick = function(e) {
      e.preventDefault();
      setViewMode(currentViewMode === 'grid' ? 'list' : 'grid');
    };

    header.appendChild(toggleBtn);
  }

  function installKeyboardNav() {
    if (!snapsList) return;

    snapsList.addEventListener('keydown', function(e) {
      if (e.key === 'l' || e.key === 'L') {
        e.preventDefault();
        setViewMode(currentViewMode === 'grid' ? 'list' : 'grid');
        return;
      }

      if (currentViewMode !== 'grid') return;

      var target = e.target.closest('[data-snap-index]');
      if (!target) return;

      var idx = parseInt(target.getAttribute('data-snap-index'), 10) || 0;
      var items = Array.from(snapsList.querySelectorAll('[data-snap-index]'));
      var pos = items.indexOf(target);
      if (pos < 0) return;

      var cols = getComputedStyle(snapsList).getPropertyValue('grid-template-columns').split(' ').length;
      var next = null;

      switch(e.key) {
        case 'ArrowRight': next = items[pos+1]; break;
        case 'ArrowLeft':  next = items[pos-1]; break;
        case 'ArrowDown':  next = items[Math.min(pos + cols, items.length-1)]; break;
        case 'ArrowUp':    next = items[Math.max(pos - cols, 0)]; break;
        case 'Home':       next = items[0]; break;
        case 'End':        next = items[items.length-1]; break;
      }

      if (next) {
        e.preventDefault();
        next.focus();
      }
    });
  }

  function setupSnapshotIndexing() {
    if (!snapsList) return;
    var items = snapsList.querySelectorAll('[role="button"], .snap-item, [data-snap-id]');
    items.forEach(function(el, idx) {
      el.setAttribute('data-snap-index', idx);
      if (!el.hasAttribute('tabindex')) el.setAttribute('tabindex', '0');
    });
  }

  function onSnapshotsElementReady() {
    snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return false;

    setupSnapshotIndexing();
    ensureToggleButton();
    setViewMode(getSavedViewMode());
    installKeyboardNav();

    /* hook MutationObserver for dynamic snapshot additions */
    if (!observer) {
      var cb = function(muts) {
        var needsReindex = false;
        for (var i = 0; i < muts.length; i++) {
          if (muts[i].type === 'childList') { needsReindex = true; break; }
        }
        if (needsReindex) setupSnapshotIndexing();
      };
      observer = new MutationObserver(cb);
      if (window.rvtTrackMutationObserver) {
        window.rvtTrackMutationObserver(observer);
      }
      observer.observe(snapsList, { childList: true });
    }

    return true;
  }

  /* Export API immediately, initialization happens on demand */
  window.rvtSnapGallery = {
    setMode: function(m) {
      if (!snapsList) onSnapshotsElementReady();
      setViewMode(m);
    },
    getMode: function() { return currentViewMode; },
    updateColumns: function() {
      if (!snapsList) onSnapshotsElementReady();
      updateGridColumns();
    }
  };

  function boot() {
    if (!onSnapshotsElementReady()) {
      setTimeout(boot, 300);
      return;
    }
    window.addEventListener('resize', updateGridColumns);
    window.rvtSnapGalleryReady = true;
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot, 800); }, {once:true});
  } else {
    setTimeout(boot, 800);
  }
})();
;
/* END modules/rvt-snap01-gallery-grid-js.js */

/* BEGIN modules/rvt-snap03-reorder-labels-js.js */
(function() {
  var ORDER_KEY = 'rvt-snap-order';
  var NOTES_KEY = 'rvt-snap-notes';
  var MAX_LABEL = 60;
  var draggedItem = null;
  var dropZone = null;
  var editingItem = null;

  function loadOrder() {
    try { return JSON.parse(localStorage.getItem(ORDER_KEY) || '[]') || []; } catch(e) { return []; }
  }

  function saveOrder(order) {
    try { localStorage.setItem(ORDER_KEY, JSON.stringify(order)); } catch(e) {}
  }

  function loadNotes() {
    try { return JSON.parse(localStorage.getItem(NOTES_KEY) || '{}') || {}; } catch(e) { return {}; }
  }

  function saveNotes(notes) {
    try { localStorage.setItem(NOTES_KEY, JSON.stringify(notes)); } catch(e) {}
  }

  function getSnapId(el) {
    return el.getAttribute('data-snap-id') || el.getAttribute('id') || '';
  }

  function startEdit(labelEl) {
    if (editingItem) cancelEdit();

    var text = labelEl.textContent.trim();
    var input = document.createElement('input');
    input.className = 'rvt-snap-label-edit';
    input.type = 'text';
    input.value = text;
    input.maxLength = MAX_LABEL;
    input.setAttribute('aria-label', 'Edit snapshot label');

    var finished = function() {
      var newText = input.value.trim().slice(0, MAX_LABEL) || text;
      labelEl.textContent = newText;
      input.parentElement.replaceChild(labelEl, input);

      var snapId = getSnapId(labelEl.closest('[data-snap-id], [data-snap-index]'));
      if (snapId) {
        var notes = loadNotes();
        notes[snapId] = newText;
        saveNotes(notes);
        announceChange('Label updated to: ' + newText);
      }
      editingItem = null;
    };

    input.onblur = finished;
    input.onkeydown = function(e) {
      if (e.key === 'Enter') { e.preventDefault(); finished(); }
      else if (e.key === 'Escape') { cancelEdit(); }
    };

    labelEl.parentElement.replaceChild(input, labelEl);
    editingItem = input;
    input.focus();
    input.select();
  }

  function cancelEdit() {
    if (editingItem) {
      var orig = document.createElement('p');
      orig.className = 'rvt-snap-card-label';
      orig.textContent = editingItem.value;
      editingItem.parentElement.replaceChild(orig, editingItem);
      editingItem = null;
    }
  }

  function announceChange(msg) {
    var hint = document.createElement('div');
    hint.className = 'rvt-snap-reorder-hint';
    hint.setAttribute('aria-live', 'polite');
    hint.setAttribute('aria-atomic', 'true');
    hint.textContent = msg;
    document.body.appendChild(hint);
    setTimeout(function() { hint.remove(); }, 2000);
  }

  function setupDragDrop() {
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return;

    var onDragStart = function(e) {
      draggedItem = this.closest('[data-snap-index]');
      if (!draggedItem) return;
      draggedItem.classList.add('rvt-dragging');
      e.dataTransfer.effectAllowed = 'move';
      e.dataTransfer.setData('text/plain', getSnapId(draggedItem));
    };

    var onDragOver = function(e) {
      e.preventDefault();
      e.dataTransfer.dropEffect = 'move';
      var target = e.target.closest('[data-snap-index]');
      if (target && target !== draggedItem) {
        target.classList.add('rvt-drag-over');
      }
    };

    var onDragLeave = function(e) {
      var target = e.target.closest('[data-snap-index]');
      if (target) target.classList.remove('rvt-drag-over');
    };

    var onDrop = function(e) {
      e.preventDefault();
      if (!draggedItem) return;

      var dropTarget = e.target.closest('[data-snap-index]');
      if (!dropTarget || dropTarget === draggedItem) {
        draggedItem.classList.remove('rvt-dragging');
        document.querySelectorAll('.rvt-drag-over').forEach(function(el) { el.classList.remove('rvt-drag-over'); });
        return;
      }

      /* Reorder in DOM */
      var parent = snapsList;
      var items = Array.from(parent.querySelectorAll('[data-snap-index]'));
      var fromIdx = items.indexOf(draggedItem);
      var toIdx = items.indexOf(dropTarget);

      if (fromIdx >= 0 && toIdx >= 0 && fromIdx !== toIdx) {
        if (fromIdx < toIdx) {
          dropTarget.parentElement.insertBefore(draggedItem, dropTarget.nextElementSibling);
        } else {
          dropTarget.parentElement.insertBefore(draggedItem, dropTarget);
        }

        /* Save reordered IDs */
        items = Array.from(parent.querySelectorAll('[data-snap-index]'));
        var order = items.map(function(el) { return getSnapId(el); });
        saveOrder(order);
        announceChange('Snapshot moved. New position: ' + (items.indexOf(draggedItem) + 1) + ' of ' + items.length);
      }

      draggedItem.classList.remove('rvt-dragging');
      document.querySelectorAll('.rvt-drag-over').forEach(function(el) { el.classList.remove('rvt-drag-over'); });
      draggedItem = null;
    };

    var onDragEnd = function(e) {
      if (draggedItem) draggedItem.classList.remove('rvt-dragging');
      document.querySelectorAll('.rvt-drag-over').forEach(function(el) { el.classList.remove('rvt-drag-over'); });
      draggedItem = null;
    };

    snapsList.addEventListener('dragstart', onDragStart);
    snapsList.addEventListener('dragover', onDragOver);
    snapsList.addEventListener('dragleave', onDragLeave);
    snapsList.addEventListener('drop', onDrop);
    snapsList.addEventListener('dragend', onDragEnd);
  }

  function setupLabelEditing() {
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return;

    snapsList.addEventListener('dblclick', function(e) {
      var labelEl = e.target.closest('.rvt-snap-card-label');
      if (labelEl && !editingItem) startEdit(labelEl);
    });

    snapsList.addEventListener('keydown', function(e) {
      if (e.key === 'F2' || e.key === 'f2') {
        e.preventDefault();
        var card = e.target.closest('[data-snap-index]');
        if (card) {
          var labelEl = card.querySelector('.rvt-snap-card-label');
          if (labelEl && !editingItem) startEdit(labelEl);
        }
      }
    });
  }

  function setupKeyboardReorder() {
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return;

    snapsList.addEventListener('keydown', function(e) {
      if (!e.ctrlKey && !e.metaKey) return;
      if (e.key !== 'ArrowUp' && e.key !== 'ArrowDown') return;

      e.preventDefault();
      var card = e.target.closest('[data-snap-index]');
      if (!card) return;

      var items = Array.from(snapsList.querySelectorAll('[data-snap-index]'));
      var idx = items.indexOf(card);
      if (idx < 0) return;

      var swapIdx = e.key === 'ArrowUp' ? idx - 1 : idx + 1;
      if (swapIdx < 0 || swapIdx >= items.length) return;

      var swapItem = items[swapIdx];
      if (e.key === 'ArrowUp') {
        card.parentElement.insertBefore(card, swapItem);
      } else {
        swapItem.parentElement.insertBefore(swapItem, card);
      }

      items = Array.from(snapsList.querySelectorAll('[data-snap-index]'));
      var order = items.map(function(el) { return getSnapId(el); });
      saveOrder(order);
      announceChange('Snapshot moved. New position: ' + (items.indexOf(card) + 1) + ' of ' + items.length);
      card.focus();
    });
  }

  function boot() {
    setupDragDrop();
    setupLabelEditing();
    setupKeyboardReorder();

    window.rvtSnapReorder = {
      getOrder: loadOrder,
      saveOrder: saveOrder,
      getNotes: loadNotes,
      saveNotes: saveNotes
    };

    window.rvtSnapReorderReady = true;
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot, 1000); }, {once:true});
  } else {
    setTimeout(boot, 1000);
  }
})();
;
/* END modules/rvt-snap03-reorder-labels-js.js */

/* BEGIN modules/rvt-snap02-compare-js.js */
(function() {
  var selectedSnaps = [];
  var compareBtn = null;
  var compareModal = null;
  var multiSelectMode = false;

  function getSnapId(el) {
    return el.getAttribute('data-snap-id') || el.getAttribute('id') || '';
  }

  function getSnapLabel(el) {
    var lbl = el.querySelector('.rvt-snap-card-label');
    return lbl ? lbl.textContent.trim() : getSnapId(el);
  }

  function enableMultiSelect() {
    if (multiSelectMode) return;
    multiSelectMode = true;
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (snapsList) snapsList.classList.add('rvt-snap-multi-select');

    compareBtn = document.createElement('button');
    compareBtn.className = 'rvt-snap-compare-btn';
    compareBtn.textContent = 'Compare Selected (0)';
    compareBtn.disabled = true;
    compareBtn.onclick = openCompareModal;
    compareBtn.setAttribute('aria-label', 'Compare selected snapshots');

    var header = document.querySelector('[data-view="snapshots"] .view-header, .snapshots-header');
    if (header) header.appendChild(compareBtn);
  }

  function disableMultiSelect() {
    if (!multiSelectMode) return;
    multiSelectMode = false;
    selectedSnaps = [];
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (snapsList) snapsList.classList.remove('rvt-snap-multi-select');

    document.querySelectorAll('[data-selected="true"]').forEach(function(el) {
      el.removeAttribute('data-selected');
    });

    if (compareBtn) compareBtn.remove();
    compareBtn = null;
  }

  function toggleSnapSelection(snapEl) {
    if (!multiSelectMode) return;
    var snapId = getSnapId(snapEl);
    if (!snapId) return;

    var isSelected = snapEl.getAttribute('data-selected') === 'true';
    if (isSelected) {
      snapEl.removeAttribute('data-selected');
      selectedSnaps = selectedSnaps.filter(function(id) { return id !== snapId; });
    } else {
      snapEl.setAttribute('data-selected', 'true');
      selectedSnaps.push(snapId);
    }

    updateCompareButton();
  }

  function updateCompareButton() {
    if (!compareBtn) return;
    var count = selectedSnaps.length;
    compareBtn.textContent = 'Compare Selected (' + count + ')';
    compareBtn.disabled = count !== 2;
  }

  function openCompareModal() {
    if (selectedSnaps.length !== 2) return;

    var snaps = window.S && window.S.snaps;
    if (!snaps || !Array.isArray(snaps)) return;

    var snap1 = snaps.find(function(s) { return s && (s.id === selectedSnaps[0] || s.label === selectedSnaps[0]); });
    var snap2 = snaps.find(function(s) { return s && (s.id === selectedSnaps[1] || s.label === selectedSnaps[1]); });

    if (!snap1 || !snap2) return;

    buildCompareModal(snap1, snap2);
  }

  function buildCompareModal(snap1, snap2) {
    if (compareModal) compareModal.remove();

    compareModal = document.createElement('div');
    compareModal.id = 'rvt-compare-modal';
    compareModal.setAttribute('role', 'dialog');
    compareModal.setAttribute('aria-modal', 'true');
    compareModal.setAttribute('aria-label', 'Snapshot comparison');

    var container = document.createElement('div');
    container.className = 'rvt-compare-container';

    var header = document.createElement('div');
    header.className = 'rvt-compare-header';
    header.innerHTML = '<h2 class="rvt-compare-title">Snapshot Comparison</h2>';
    var closeBtn = document.createElement('button');
    closeBtn.className = 'rvt-compare-close';
    closeBtn.textContent = '✕';
    closeBtn.onclick = function() { compareModal.classList.remove('rvt-compare-visible'); };
    header.appendChild(closeBtn);
    container.appendChild(header);

    /* KPI comparison */
    var kpiGrid = document.createElement('div');
    kpiGrid.className = 'rvt-compare-grid';

    [snap1, snap2].forEach(function(snap, idx) {
      var side = document.createElement('div');
      side.className = 'rvt-compare-side';
      side.innerHTML = '<p class="rvt-compare-side-label">' + (snap.label || 'Snapshot ' + (idx + 1)) + ' (' + (snap.ts || 'N/A') + ')</p>';

      var table = document.createElement('table');
      table.className = 'rvt-compare-kpi-table';
      table.innerHTML = '<thead><tr><th>Metric</th><th>Value</th></tr></thead><tbody>';

      /* Extract KPI values */
      var kpis = snap.kpis || snap.data || {};
      Object.keys(kpis).slice(0, 5).forEach(function(key) {
        var val = kpis[key];
        table.innerHTML += '<tr><td>' + key + '</td><td>' + (typeof val === 'number' ? val.toFixed(2) : val) + '</td></tr>';
      });

      table.innerHTML += '</tbody>';
      side.appendChild(table);
      kpiGrid.appendChild(side);
    });

    container.appendChild(kpiGrid);

    /* Delta column (if data available) */
    if ((snap1.kpis || snap1.data) && (snap2.kpis || snap2.data)) {
      var deltaTable = document.createElement('table');
      deltaTable.className = 'rvt-compare-kpi-table';
      deltaTable.innerHTML = '<thead><tr><th>Metric</th><th>Δ (Snap2 - Snap1)</th></tr></thead><tbody>';

      var kpis1 = snap1.kpis || snap1.data || {};
      var kpis2 = snap2.kpis || snap2.data || {};
      Object.keys(kpis1).slice(0, 5).forEach(function(key) {
        var v1 = parseFloat(kpis1[key]) || 0;
        var v2 = parseFloat(kpis2[key]) || 0;
        var delta = (v2 - v1).toFixed(2);
        var deltaClass = delta > 0 ? 'increase' : (delta < 0 ? 'decrease' : '');
        deltaTable.innerHTML += '<tr><td>' + key + '</td><td class="rvt-compare-delta ' + deltaClass + '">' + delta + '</td></tr>';
      });

      deltaTable.innerHTML += '</tbody>';
      container.appendChild(deltaTable);
    }

    /* Actions */
    var actions = document.createElement('div');
    actions.className = 'rvt-compare-actions';
    var exportBtn = document.createElement('button');
    exportBtn.className = 'rvt-compare-export-btn';
    exportBtn.textContent = 'Export Comparison';
    exportBtn.onclick = function() { exportComparison(snap1, snap2); };
    actions.appendChild(exportBtn);
    container.appendChild(actions);

    compareModal.appendChild(container);
    document.body.appendChild(compareModal);
    compareModal.classList.add('rvt-compare-visible');

    compareModal.onclick = function(e) {
      if (e.target === compareModal) compareModal.classList.remove('rvt-compare-visible');
    };
  }

  function exportComparison(snap1, snap2) {
    /* Generate CSV/text export */
    var content = 'Snapshot Comparison\n\n';
    content += 'Snapshot 1: ' + (snap1.label || 'N/A') + ' (' + (snap1.ts || 'N/A') + ')\n';
    content += 'Snapshot 2: ' + (snap2.label || 'N/A') + ' (' + (snap2.ts || 'N/A') + ')\n\n';

    content += 'Metric,Snap1,Snap2,Delta\n';
    var kpis1 = snap1.kpis || snap1.data || {};
    var kpis2 = snap2.kpis || snap2.data || {};
    Object.keys(kpis1).forEach(function(key) {
      var v1 = kpis1[key];
      var v2 = kpis2[key] || 'N/A';
      var delta = (typeof v1 === 'number' && typeof v2 === 'number') ? (v2 - v1).toFixed(2) : 'N/A';
      content += key + ',' + v1 + ',' + v2 + ',' + delta + '\n';
    });

    var blob = new Blob([content], { type: 'text/csv' });
    var url = URL.createObjectURL(blob);
    var a = document.createElement('a');
    a.href = url;
    a.download = 'comparison_' + Date.now() + '.csv';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  function setupMultiSelectToggle() {
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return;

    /* Add shortcut: Ctrl+Shift+M to toggle multi-select */
    document.addEventListener('keydown', function(e) {
      if (e.ctrlKey && e.shiftKey && (e.key === 'M' || e.key === 'm')) {
        e.preventDefault();
        multiSelectMode ? disableMultiSelect() : enableMultiSelect();
      }
    });

    /* Click on card to select in multi-select mode */
    snapsList.addEventListener('click', function(e) {
      if (!multiSelectMode) return;
      var card = e.target.closest('[data-snap-index]');
      if (card) toggleSnapSelection(card);
    });
  }

  function boot() {
    setupMultiSelectToggle();

    window.rvtSnapCompare = {
      enableMultiSelect: enableMultiSelect,
      disableMultiSelect: disableMultiSelect,
      getSelected: function() { return selectedSnaps.slice(); },
      openCompare: openCompareModal
    };

    window.rvtSnapCompareReady = true;
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot, 1200); }, {once:true});
  } else {
    setTimeout(boot, 1200);
  }
})();
;
/* END modules/rvt-snap02-compare-js.js */

/* BEGIN modules/rvt-audit01-sections-js.js */
(function(){
  'use strict';
  var DEFAULT_AUDIT_SECTIONS={gates:true,reasons:false,events:false};
  var DEFAULT_AUDIT_FILTER={severities:[],reasons:[]};
  var STRINGS={
    'gamma.audit.title':'Audit trail','gamma.audit.sections.gates':'Gates','gamma.audit.sections.reasons':'Reasons','gamma.audit.sections.events':'Events','gamma.audit.summary.gates':'{n} gate signals','gamma.audit.summary.reasons':'{n} reasons','gamma.audit.summary.events':'{n} events','gamma.audit.empty':'No audit events yet','gamma.audit.filter.status':'Showing {shown} of {total} events','gamma.audit.filter.active':'Filter active: {items}','gamma.audit.filter.none':'No active filters','gamma.audit.severity.info':'Info','gamma.audit.severity.warn':'Warn','gamma.audit.severity.critical':'Critical','gamma.audit.reason.pill.remove':'Remove reason filter {reason}','gamma.audit.reason.general':'general','gamma.audit.export.button':'Export audit','gamma.audit.export.all':'Export all ({n} rows)','gamma.audit.export.filtered':'Export filtered ({n} rows)','gamma.audit.export.downloaded':'Audit export downloaded','gamma.nav.collapse':'Collapse rail','gamma.nav.expand':'Expand rail','gamma.conn.title':'Connection health','gamma.conn.latency':'Latency','gamma.conn.frameAge':'Last frame age','gamma.conn.retryCount':'Retry count','gamma.conn.sourceMode':'Source mode','gamma.conn.unknown':'Unknown','gamma.settings.portability.title':'Portability','gamma.settings.portability.desc':'Move this console configuration between stations.','gamma.settings.export':'Export settings','gamma.settings.import':'Import settings','gamma.settings.import.title':'Import settings preview','gamma.settings.import.apply':'Apply import','gamma.settings.import.cancel':'Cancel','gamma.settings.import.noChange':'no change','gamma.settings.import.skipped':'skipped (unknown field)','gamma.settings.import.newer':'This file was exported from a newer version; some fields may be ignored.','gamma.settings.import.invalid':'Import file is not a valid Radar Vital Trainer settings export.','gamma.settings.import.applied':'Settings import applied','gamma.settings.import.undo':'Undo import','gamma.settings.reset.title':'Reset dashboard defaults','gamma.settings.reset.none':'All settings are already at their defaults','gamma.settings.reset.checkbox':'I understand this will clear my custom settings','gamma.settings.reset.apply':'Reset settings','gamma.settings.reset.close':'Close','gamma.settings.reset.undone':'Settings reset undone','gamma.settings.reset.done':'Settings reset to defaults','gamma.settings.reset.undo':'Undo reset','gamma.common.close':'Close','gamma.common.ms':'{n} ms'};
  function extendStrings(){window.RVT_STRINGS=window.RVT_STRINGS||{en:{}};window.RVT_STRINGS.en=window.RVT_STRINGS.en||{};Object.keys(STRINGS).forEach(function(k){if(!(k in window.RVT_STRINGS.en))window.RVT_STRINGS.en[k]=STRINGS[k];});}
  function t(k,v){extendStrings();if(typeof window.rvtT==='function')return window.rvtT(k,v||{});var s=(window.RVT_STRINGS&&window.RVT_STRINGS.en&&window.RVT_STRINGS.en[k])||k;Object.keys(v||{}).forEach(function(x){s=s.replace(new RegExp('\\{'+x+'\\}','g'),String(v[x]));});return s;}
  function registerManifest(){if(!window.rvtStorage||!window.rvtStorage.manifest)return;var m=window.rvtStorage.manifest;if(!m['rvt-audit-sections'])m['rvt-audit-sections']={kind:'json',defaultValue:{gates:true,reasons:false,events:false},validate:function(v){return v!==null&&typeof v==='object'&&'gates'in v&&'reasons'in v&&'events'in v;}};if(!m['rvt-audit-filter'])m['rvt-audit-filter']={kind:'json',defaultValue:{severities:[],reasons:[]},validate:function(v){return v!==null&&typeof v==='object'&&Array.isArray(v.severities)&&Array.isArray(v.reasons);}};if(!m['rvt-rail-collapsed'])m['rvt-rail-collapsed']={kind:'scalar',defaultValue:'false',validate:function(v){return v==='true'||v==='false';}};}
  function clone(o){try{return JSON.parse(JSON.stringify(o));}catch(_){return o;}}
  function getJson(k,f){try{if(window.rvtStorage&&typeof window.rvtStorage.getJson==='function')return window.rvtStorage.getJson(k,clone(f));var r=localStorage.getItem(k);return r?JSON.parse(r):clone(f);}catch(e){console.warn('[RVT Gamma Batch2A] storage read failed',k,e);return clone(f);}}
  function setJson(k,v){try{if(window.rvtStorage&&typeof window.rvtStorage.setJson==='function')window.rvtStorage.setJson(k,v);else localStorage.setItem(k,JSON.stringify(v));}catch(e){console.warn('[RVT Gamma Batch2A] storage write failed',k,e);}}
  function setScalar(k,v){try{if(window.rvtStorage&&typeof window.rvtStorage.set==='function')window.rvtStorage.set(k,v);else localStorage.setItem(k,v);}catch(e){console.warn('[RVT Gamma Batch2A] storage write failed',k,e);}}
  function escapeHtml(v){return String(v==null?'':v).replace(/[&<>"']/g,function(c){return({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'})[c];});}
  function safeSlug(v){return String(v||'general').toLowerCase().replace(/[^a-z0-9_-]+/g,'-').replace(/^-+|-+$/g,'')||'general';}
  function toast(message,actionLabel,action){if(typeof window.showToast==='function'){try{window.showToast(message);}catch(_){}}else if(typeof window.toast==='function'){try{window.toast(message);}catch(_){}}if(actionLabel&&typeof action==='function'){var host=document.getElementById('toasts')||document.querySelector('.toasts')||document.body;var node=document.createElement('div');node.className='toast rvt-settings-undo';node.setAttribute('role','status');node.innerHTML='<span>'+escapeHtml(message)+'</span>';var btn=document.createElement('button');btn.type='button';btn.textContent=actionLabel;btn.addEventListener('click',function(){action();node.remove();});node.appendChild(btn);host.appendChild(node);setTimeout(function(){node.remove();},10000);}}
  function parseSeverity(v){var s=String(v||'info').toLowerCase();if(s==='bad'||s==='fatal'||s==='error'||s==='critical')return'critical';if(s==='warn'||s==='warning')return'warn';return'info';}
  function deriveReason(e){var d=e&&(e.reason||e.code||e.kind);if(d)return String(d);var msg=String((e&&(e.message||e.msg||e.title))||'');var m=msg.match(/^([A-Za-z0-9 _.-]{3,32})(?:\:|\s+-\s+)/);return m?m[1].trim():t('gamma.audit.reason.general');}
  function getSessionId(){var S=window.S||{};return(S.ctl&&(S.ctl.currentSessionId||(S.ctl.current&&S.ctl.current.session_id)))||(S.lastPayload&&S.lastPayload.meta&&S.lastPayload.meta.session_id)||(S.session&&S.session.id)||'';}
  function getOperator(){var S=window.S||{};var el=document.getElementById('opName')||document.querySelector('[data-operator-name]');return(S.setup&&(S.setup.operator||S.setup.operator_label))||(el&&el.textContent&&el.textContent.trim())||'';}
  function normalizeAuditEntry(e,i){e=e||{};var ts=e.timestamp||e.ts||e.time||e.at||new Date().toISOString();if(typeof ts==='number')ts=new Date(ts<10000000000?ts*1000:ts).toISOString();return{timestamp:String(ts),severity:parseSeverity(e.severity||e.level||e.status),source:String(e.source||e.src||e.panel||'console'),reason:deriveReason(e),message:String(e.message||e.msg||e.title||e.reason||''),session_id:String(e.session_id||e.sessionId||getSessionId()||'nosession'),operator:String(e.operator||getOperator()||''),original:e,index:i||0};}
  function getAuditEvents(){var S=window.S||{};var c=[S.liveAuditLog,S.liveAuditLogs,S.auditLog,S.auditLogs,S.alerts,window.liveAuditLogRows,window.rvtAuditLogRows];for(var i=0;i<c.length;i++){if(Array.isArray(c[i])&&c[i].length)return c[i].map(normalizeAuditEntry);}try{var raw=localStorage.getItem('rvt-live-audit')||localStorage.getItem('rvt-audit-log');var p=raw?JSON.parse(raw):null;if(Array.isArray(p)&&p.length)return p.map(normalizeAuditEntry);}catch(e){console.warn('[RVT Gamma Batch2A] audit log parse failed',e);}return[];}
  function bootSoon(fn){if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',fn,{once:true});else fn();}
  extendStrings();registerManifest();
  window.rvtGammaBatch2a={t:t,registerManifest:registerManifest,defaults:{auditSections:DEFAULT_AUDIT_SECTIONS,auditFilter:DEFAULT_AUDIT_FILTER,railCollapsed:'false'},getJson:getJson,setJson:setJson,setScalar:setScalar,escapeHtml:escapeHtml,safeSlug:safeSlug,toast:toast,parseSeverity:parseSeverity,normalizeAuditEntry:normalizeAuditEntry,getAuditEvents:getAuditEvents,getSessionId:getSessionId,getOperator:getOperator,bootSoon:bootSoon};
  var sectionState=getJson('rvt-audit-sections',DEFAULT_AUDIT_SECTIONS);['gates','reasons','events'].forEach(function(id){sectionState[id]=!!sectionState[id];});
  function persist(){setJson('rvt-audit-sections',sectionState);}
  function summarize(id){var ev=getAuditEvents();if(id==='events')return t('gamma.audit.summary.events',{n:ev.length});if(id==='reasons'){var r={};ev.forEach(function(e){r[e.reason]=true;});return t('gamma.audit.summary.reasons',{n:Object.keys(r).length});}var n=document.querySelectorAll('#auditSchemaInvalidCount,#auditSchemaWarningCount,#auditReconnectCount').length;return t('gamma.audit.summary.gates',{n:n});}
  function updateDom(){['gates','reasons','events'].forEach(function(id){var s=document.getElementById('rvtAuditSection-'+id);var h=document.querySelector('[data-audit-section-toggle="'+id+'"]');var b=document.querySelector('[data-audit-summary="'+id+'"]');if(s)s.setAttribute('data-expanded',sectionState[id]?'true':'false');if(h)h.setAttribute('aria-expanded',sectionState[id]?'true':'false');if(b)b.textContent=summarize(id);});}
  function setSection(id,expanded){if(!Object.prototype.hasOwnProperty.call(sectionState,id))return;sectionState[id]=!!expanded;persist();updateDom();}
  window.rvtAuditSections={toggle:function(id){setSection(id,!sectionState[id]);},expand:function(id){setSection(id,true);},collapse:function(id){setSection(id,false);},getState:function(){return clone(sectionState);}};
  function makeHeader(id,labelKey){var h=document.createElement('button');h.type='button';h.className='rvt-audit-section-header';h.setAttribute('data-audit-section-toggle',id);h.setAttribute('aria-expanded',sectionState[id]?'true':'false');h.setAttribute('aria-controls','rvtAuditContent-'+id);h.innerHTML='<span class="rvt-audit-section-title">'+escapeHtml(t(labelKey))+'</span><span class="rvt-audit-section-meta"><span class="rvt-audit-summary-badge" data-audit-summary="'+id+'">'+escapeHtml(summarize(id))+'</span><span class="material-symbols-rounded rvt-audit-chevron" aria-hidden="true">expand_more</span></span>';h.addEventListener('click',function(){window.rvtAuditSections.toggle(id);});h.addEventListener('keydown',function(e){if(e.key==='Enter'||e.key===' '){e.preventDefault();window.rvtAuditSections.toggle(id);}});return h;}
  function makeSection(id,labelKey){var s=document.createElement('section');s.className='rvt-audit-section';s.id='rvtAuditSection-'+id;s.setAttribute('data-expanded',sectionState[id]?'true':'false');var c=document.createElement('div');c.className='rvt-audit-section-content';c.id='rvtAuditContent-'+id;c.setAttribute('role','region');c.setAttribute('aria-labelledby','rvtAuditHeader-'+id);var h=makeHeader(id,labelKey);h.id='rvtAuditHeader-'+id;s.appendChild(h);s.appendChild(c);return{section:s,content:c};}
  function enhanceAuditTab(){var tab=document.getElementById('tab-audit');if(!tab||document.getElementById('rvtAuditSectionsRoot'))return;var children=Array.prototype.slice.call(tab.children);var logCard=null;children.forEach(function(child){if(!logCard&&child.querySelector&&child.querySelector('#liveAuditLog'))logCard=child;});var toolbar=document.createElement('div');toolbar.className='rvt-audit-toolbar';toolbar.id='rvtAuditToolbar';toolbar.innerHTML='<h3>'+escapeHtml(t('gamma.audit.title'))+'</h3><div id="rvtAuditToolbarActions"></div>';var root=document.createElement('div');root.className='rvt-audit-sections';root.id='rvtAuditSectionsRoot';var gates=makeSection('gates','gamma.audit.sections.gates');var reasons=makeSection('reasons','gamma.audit.sections.reasons');var events=makeSection('events','gamma.audit.sections.events');var gatesGrid=document.createElement('div');gatesGrid.className='grid cards';children.forEach(function(child){if(child!==logCard)gatesGrid.appendChild(child);});gates.content.appendChild(gatesGrid);reasons.content.innerHTML='<div id="rvtAuditReasonsList" class="rvt-audit-reasons-list"></div>';var filterStack=document.createElement('div');filterStack.className='rvt-audit-filter-stack';filterStack.innerHTML='<div id="rvtAuditSeverityChips" class="rvt-audit-chip-row" aria-label="'+escapeHtml(t('gamma.audit.sections.events'))+'"></div><div id="rvtAuditReasonPills" class="rvt-audit-pill-row"></div><div id="rvtAuditFilterStatus" class="rvt-audit-filter-status" aria-live="polite"></div><div id="rvtAuditFilterAnnouncer" class="sr-only" aria-live="polite"></div>';events.content.appendChild(filterStack);if(logCard)events.content.appendChild(logCard);root.appendChild(gates.section);root.appendChild(reasons.section);root.appendChild(events.section);tab.innerHTML='';tab.appendChild(toolbar);tab.appendChild(root);updateDom();}
  bootSoon(function(){registerManifest();enhanceAuditTab();updateDom();});
})();
;
/* END modules/rvt-audit01-sections-js.js */

/* BEGIN modules/rvt-audit02-filter-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var VALID=['info','warn','critical'];
  var state=U.getJson('rvt-audit-filter',U.defaults.auditFilter);
  if(!Array.isArray(state.severities))state.severities=[];
  if(!Array.isArray(state.reasons))state.reasons=[];
  function persist(){U.setJson('rvt-audit-filter',state);}
  function label(sev){return U.t('gamma.audit.severity.'+sev);}
  function getFilterResult(){var rows=U.getAuditEvents();var filtered=rows.filter(function(e){var sevOk=!state.severities.length||state.severities.indexOf(e.severity)!==-1;var reasonOk=!state.reasons.length||state.reasons.indexOf(e.reason)!==-1;return sevOk&&reasonOk;});return{all:rows,filtered:filtered};}
  function renderRows(){var log=document.getElementById('liveAuditLog');var result=getFilterResult();if(!log)return result;if(!result.filtered.length){log.innerHTML='<div class="muted rvt-audit-empty">'+U.escapeHtml(U.t('gamma.audit.empty'))+'</div>';}else{log.innerHTML=result.filtered.map(function(e){return'<div class="rvt-audit-row" data-audit-row data-severity="'+U.escapeHtml(e.severity)+'" data-reason="'+U.escapeHtml(e.reason)+'"><span>'+U.escapeHtml(e.timestamp)+'</span><span class="rvt-audit-row-severity">'+U.escapeHtml(label(e.severity))+'</span><span class="rvt-audit-row-reason">'+U.escapeHtml(e.reason)+'</span><span>'+U.escapeHtml(e.message)+'</span></div>';}).join('');}return result;}
  function renderChips(){var host=document.getElementById('rvtAuditSeverityChips');if(!host)return;host.innerHTML=VALID.map(function(sev){var active=state.severities.indexOf(sev)!==-1;return'<button type="button" class="rvt-audit-chip" data-audit-severity="'+sev+'" data-severity="'+sev+'" aria-pressed="'+(active?'true':'false')+'">'+U.escapeHtml(label(sev))+'</button>';}).join('');Array.prototype.forEach.call(host.querySelectorAll('[data-audit-severity]'),function(btn){btn.addEventListener('click',function(){window.rvtAuditFilter.setChip(btn.getAttribute('data-audit-severity'),btn.getAttribute('aria-pressed')!=='true');});});}
  function renderStatus(result){result=result||getFilterResult();var labelNode=document.getElementById('rvtAuditFilterStatus');if(labelNode)labelNode.textContent=U.t('gamma.audit.filter.status',{shown:result.filtered.length,total:result.all.length});var announcer=document.getElementById('rvtAuditFilterAnnouncer');if(announcer){var active=state.severities.concat(state.reasons);announcer.textContent=active.length?U.t('gamma.audit.filter.active',{items:active.join(', ')}):U.t('gamma.audit.filter.none');}}
  function applyToLog(){renderChips();var result=renderRows();if(window.rvtAuditReasonFilter&&typeof window.rvtAuditReasonFilter.render==='function')window.rvtAuditReasonFilter.render();renderStatus(result);return result.filtered;}
  window.rvtAuditFilter={setChip:function(severity,active){severity=String(severity||'').toLowerCase();if(VALID.indexOf(severity)===-1)return;var i=state.severities.indexOf(severity);if(active&&i===-1)state.severities.push(severity);if(!active&&i!==-1)state.severities.splice(i,1);persist();applyToLog();},getActive:function(){return{severities:state.severities.slice(),reasons:state.reasons.slice()};},applyToLog:applyToLog,clearSeverities:function(){state.severities=[];persist();applyToLog();},_setReasons:function(reasons){state.reasons=Array.isArray(reasons)?reasons.slice():[];persist();applyToLog();},_getResult:getFilterResult};
  U.bootSoon(function(){renderChips();applyToLog();});
  if(typeof window.syncAuditUi==='function'&&!window.syncAuditUi.__rvtGammaWrapped){var originalSyncAuditUi=window.syncAuditUi;window.syncAuditUi=function(){var ret=originalSyncAuditUi.apply(this,arguments);if(window.rvtAuditFilter)window.rvtAuditFilter.applyToLog();return ret;};window.syncAuditUi.__rvtGammaWrapped=true;}
})();
;
/* END modules/rvt-audit02-filter-js.js */

/* BEGIN modules/rvt-audit03-reasons-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  function currentState(){return(window.rvtAuditFilter&&window.rvtAuditFilter.getActive())||{severities:[],reasons:[]};}
  function setReasons(reasons){if(window.rvtAuditFilter&&typeof window.rvtAuditFilter._setReasons==='function')window.rvtAuditFilter._setReasons(reasons);else{var s=U.getJson('rvt-audit-filter',U.defaults.auditFilter);s.reasons=reasons;U.setJson('rvt-audit-filter',s);}}
  function reasonCounts(){var counts={};U.getAuditEvents().forEach(function(e){counts[e.reason]=(counts[e.reason]||0)+1;});return counts;}
  function renderReasons(){var host=document.getElementById('rvtAuditReasonsList');if(!host)return;var counts=reasonCounts();var active=currentState().reasons;var reasons=Object.keys(counts).sort(function(a,b){return counts[b]-counts[a]||a.localeCompare(b);});if(!reasons.length){host.innerHTML='<span class="muted">'+U.escapeHtml(U.t('gamma.audit.empty'))+'</span>';return;}host.innerHTML=reasons.map(function(reason){var on=active.indexOf(reason)!==-1;return'<button type="button" class="rvt-audit-chip rvt-audit-reason-chip" data-audit-reason="'+U.escapeHtml(reason)+'" aria-pressed="'+(on?'true':'false')+'"><span>'+U.escapeHtml(reason)+'</span><span class="rvt-audit-reason-count">'+counts[reason]+'</span></button>';}).join('');Array.prototype.forEach.call(host.querySelectorAll('[data-audit-reason]'),function(btn){btn.addEventListener('click',function(){var reason=btn.getAttribute('data-audit-reason');window.rvtAuditReasonFilter.setReason(reason,btn.getAttribute('aria-pressed')!=='true');});});}
  function renderPills(){var host=document.getElementById('rvtAuditReasonPills');if(!host)return;var active=currentState().reasons;host.innerHTML=active.map(function(reason){return'<span class="rvt-audit-pill" data-audit-reason-pill="'+U.escapeHtml(reason)+'">'+U.escapeHtml(reason)+'<button type="button" aria-label="'+U.escapeHtml(U.t('gamma.audit.reason.pill.remove',{reason:reason}))+'" data-audit-remove-reason="'+U.escapeHtml(reason)+'">x</button></span>';}).join('');Array.prototype.forEach.call(host.querySelectorAll('[data-audit-remove-reason]'),function(btn){btn.addEventListener('click',function(){window.rvtAuditReasonFilter.removePill(btn.getAttribute('data-audit-remove-reason'));});});}
  function render(){renderReasons();renderPills();}
  window.rvtAuditReasonFilter={setReason:function(reason,active){reason=String(reason||'');var reasons=currentState().reasons.slice();var i=reasons.indexOf(reason);if(active&&i===-1)reasons.push(reason);if(!active&&i!==-1)reasons.splice(i,1);setReasons(reasons);render();},getActive:function(){return currentState().reasons.slice();},clear:function(){setReasons([]);render();},removePill:function(reason){this.setReason(reason,false);},render:render};
  U.bootSoon(function(){render();});
})();
;
/* END modules/rvt-audit03-reasons-js.js */

/* BEGIN modules/rvt-audit05-export-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  function csvEscape(v){var text=String(v==null?'':v);return/[",\r\n]/.test(text)?'"'+text.replace(/"/g,'""')+'"':text;}
  function toCsvRow(entry){var n=entry&&entry.original?entry:U.normalizeAuditEntry(entry||{});return[n.timestamp,n.severity,n.source,n.reason,n.message,n.session_id,n.operator].map(csvEscape).join(',');}
  function buildCsv(rows){return['timestamp,severity,source,reason,message,session_id,operator'].concat((rows||[]).map(toCsvRow)).join('\r\n')+'\r\n';}
  function filename(){var first=U.getAuditEvents()[0];var session=U.getSessionId()||(first&&first.session_id)||'nosession';var date=new Date().toISOString().slice(0,10);return'audit_'+String(session).replace(/[^A-Za-z0-9_-]+/g,'-')+'_'+date+'.csv';}
  function triggerDownload(rows){var blob=new Blob([buildCsv(rows)],{type:'text/csv;charset=utf-8'});var url=URL.createObjectURL(blob);var a=document.createElement('a');a.href=url;a.download=filename();a.setAttribute('data-rvt-audit-download','true');document.body.appendChild(a);a.click();setTimeout(function(){URL.revokeObjectURL(url);a.remove();},0);U.toast(U.t('gamma.audit.export.downloaded'));}
  function getFiltered(){if(window.rvtAuditFilter&&typeof window.rvtAuditFilter._getResult==='function')return window.rvtAuditFilter._getResult().filtered;return U.getAuditEvents();}
  function renderButton(){var host=document.getElementById('rvtAuditToolbarActions');if(!host||document.getElementById('rvtExportAuditWrap'))return;var wrap=document.createElement('div');wrap.className='rvt-export-audit-wrap';wrap.id='rvtExportAuditWrap';wrap.innerHTML='<button type="button" class="rvt-export-audit-btn" id="rvtExportAuditBtn" aria-haspopup="menu" aria-expanded="false"><span class="material-symbols-rounded" aria-hidden="true">download</span><span>'+U.escapeHtml(U.t('gamma.audit.export.button'))+'</span></button><div class="rvt-export-audit-menu" role="menu" id="rvtExportAuditMenu"><button type="button" role="menuitem" id="rvtExportAuditAll"></button><button type="button" role="menuitem" id="rvtExportAuditFiltered"></button></div>';host.appendChild(wrap);var btn=document.getElementById('rvtExportAuditBtn');var all=document.getElementById('rvtExportAuditAll');var filtered=document.getElementById('rvtExportAuditFiltered');function refreshLabels(){all.textContent=U.t('gamma.audit.export.all',{n:U.getAuditEvents().length});filtered.textContent=U.t('gamma.audit.export.filtered',{n:getFiltered().length});}btn.addEventListener('click',function(){var open=wrap.getAttribute('data-open')==='true';refreshLabels();wrap.setAttribute('data-open',open?'false':'true');btn.setAttribute('aria-expanded',open?'false':'true');});all.addEventListener('click',function(){wrap.setAttribute('data-open','false');window.rvtAuditExport.exportAll();});filtered.addEventListener('click',function(){wrap.setAttribute('data-open','false');window.rvtAuditExport.exportFiltered();});document.addEventListener('click',function(e){if(!wrap.contains(e.target)){wrap.setAttribute('data-open','false');btn.setAttribute('aria-expanded','false');}});refreshLabels();}
  window.rvtAuditExport={exportAll:function(){triggerDownload(U.getAuditEvents());},exportFiltered:function(){triggerDownload(getFiltered());},toCsvRow:toCsvRow,buildCsv:buildCsv};
  U.bootSoon(renderButton);
})();
;
/* END modules/rvt-audit05-export-js.js */

/* BEGIN modules/rvt-nav01-rail-collapse-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var collapsed=(localStorage.getItem('rvt-rail-collapsed')||U.defaults.railCollapsed)==='true';
  function apply(){document.body.classList.toggle('rvt-rail-collapsed',collapsed);U.setScalar('rvt-rail-collapsed',collapsed?'true':'false');var btn=document.getElementById('rvtRailCollapseToggle');if(btn){btn.setAttribute('aria-pressed',collapsed?'true':'false');btn.setAttribute('aria-label',collapsed?U.t('gamma.nav.expand'):U.t('gamma.nav.collapse'));btn.title=collapsed?U.t('gamma.nav.expand'):U.t('gamma.nav.collapse');var icon=btn.querySelector('.material-symbols-rounded');var label=btn.querySelector('.rvt-rail-collapse-label');if(icon)icon.textContent=collapsed?'keyboard_double_arrow_right':'keyboard_double_arrow_left';if(label)label.textContent=collapsed?U.t('gamma.nav.expand'):U.t('gamma.nav.collapse');}}
  function enhanceTooltips(){Array.prototype.forEach.call(document.querySelectorAll('.rail .r-item'),function(item){var label=item.getAttribute('aria-label')||item.textContent.trim().replace(/\s+/g,' ');if(label){item.setAttribute('aria-label',label);item.title=label;}});}
  function renderToggle(){var existing=document.getElementById('railCollapseBtn');var duplicate=document.getElementById('rvtRailCollapseToggle');if(existing){if(duplicate)duplicate.remove();return;}var railFoot=document.querySelector('.rail-foot');if(!railFoot||duplicate)return;var operator=railFoot.querySelector('.operator');var btn=document.createElement('button');btn.type='button';btn.id='rvtRailCollapseToggle';btn.className='rvt-rail-collapse-toggle';btn.innerHTML='<span class="material-symbols-rounded" aria-hidden="true"></span><span class="rvt-rail-collapse-label"></span>';btn.addEventListener('click',function(){window.rvtRailCollapse.toggle();});railFoot.insertBefore(btn,operator||null);}
  window.rvtRailCollapse={toggle:function(){collapsed=!collapsed;apply();},collapse:function(){collapsed=true;apply();},expand:function(){collapsed=false;apply();},isCollapsed:function(){return!!collapsed;}};
  document.addEventListener('keydown',function(e){if(e.altKey&&!e.ctrlKey&&!e.metaKey&&(e.key==='\\'||e.code==='Backslash')){var primary=document.getElementById('railCollapseBtn');if(primary){e.preventDefault();primary.click();return;}e.preventDefault();window.rvtRailCollapse.toggle();}});
  U.bootSoon(function(){renderToggle();enhanceTooltips();apply();});
})();
;
/* END modules/rvt-nav01-rail-collapse-js.js */

/* BEGIN modules/rvt-dedupe-help-runtime-fix-js.js */
(function(){
  'use strict';
  var ISSUE_MAP={
    'ble not found':{topic:'troubleshooting',query:'ble'},
    'placement':{topic:'hardware_setup',query:'placement'},
    'version mismatch':{topic:'firmware_truthfulness',query:'firmware'},
    'stale data':{topic:'troubleshooting',query:'stale'},
    'low reference quality':{topic:'hardware_setup',query:'reference'},
    'report failed':{topic:'report_readiness',query:'report'}
  };
  function issueConfig(btn){
    var key=(btn&&btn.textContent||'').trim().toLowerCase();
    return ISSUE_MAP[key]||null;
  }
  function applyHelpSearch(query){
    try{ if(window.S&&window.S.ctl) window.S.ctl.helpQuery=query; }catch(_){}
    try{ localStorage.setItem('rvt-help-query',query); }catch(_){}
    var input=document.getElementById('rvtHelpSearchInput')||document.querySelector('#view-help input[type="search"], #view-help .help-search input');
    if(input){
      input.value=query;
      try{ input.dispatchEvent(new Event('input',{bubbles:true})); }catch(_){}
    }
  }
  function openTopic(topic){
    if(typeof window.rvtOpenHelpTopic==='function'){ window.rvtOpenHelpTopic(topic); return; }
    if(typeof window.setHelpTopic==='function'){ window.setHelpTopic(topic); return; }
    if(typeof window.switchView==='function') window.switchView('help');
  }
  function activate(btn){
    var cfg=issueConfig(btn);
    if(!cfg) return false;
    btn.removeAttribute('onclick');
    btn.dataset.rvtHelpIssueTopic=cfg.topic;
    btn.dataset.rvtHelpIssueQuery=cfg.query;
    applyHelpSearch(cfg.query);
    openTopic(cfg.topic);
    return true;
  }
  function repair(){
    Array.prototype.forEach.call(document.querySelectorAll('#view-help .help-issue,.help-issue'),function(btn){
      var cfg=issueConfig(btn);
      if(!cfg) return;
      btn.removeAttribute('onclick');
      btn.dataset.rvtHelpIssueTopic=cfg.topic;
      btn.dataset.rvtHelpIssueQuery=cfg.query;
    });
  }
  document.addEventListener('click',function(ev){
    var target=ev.target&&ev.target.closest&&ev.target.closest('.help-issue');
    if(!target) return;
    if(activate(target)){
      ev.preventDefault();
      ev.stopImmediatePropagation();
    }
  },true);
  var booted=false;
  function boot(){
    if(booted) return;
    booted=true;
    repair();
    if(window.MutationObserver){
      new MutationObserver(repair).observe(document.body||document.documentElement,{childList:true,subtree:true});
    }
    var tries=0;
    var timer=setInterval(function(){
      repair();
      tries++;
      if(tries>20) clearInterval(timer);
    },500);
  }
  if(document.body) boot();
  else if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',boot,{once:true});
  else boot();
})();
;
/* END modules/rvt-dedupe-help-runtime-fix-js.js */

/* BEGIN modules/rvt-nav04-conn-diag-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var pinned=false,intervalId=0,indicator=null,popover=null;
  function readConnection(){var S=window.S||{};var conn=S.connection||S.conn||{};var now=Date.now();var lastAt=conn.lastFrameAt||conn.last_frame_at||S.lastFrameAt||S.lastGoodFrameAt||S.lastPayloadAt||0;if(lastAt&&lastAt<10000000000)lastAt*=1000;var latency=conn.latency_ms||conn.latencyMs||S.latencyMs||S.heartbeatLatencyMs||0;var retry=conn.retryCount||conn.retry_count||(S.reconnectPolicy&&S.reconnectPolicy.attempts)||S.reconnectAttempts||0;var mode=conn.sourceMode||conn.source_mode||S.sourceMode||(S.demoMode?'demo':'')||(document.getElementById('modeBadge')&&document.getElementById('modeBadge').textContent.trim())||U.t('gamma.conn.unknown');return{latency:latency?Math.round(Number(latency)):U.t('gamma.conn.unknown'),frameAge:lastAt?Math.max(0,Math.round(now-Number(lastAt))):U.t('gamma.conn.unknown'),retryCount:retry||0,sourceMode:mode};}
  function position(){if(!indicator||!popover)return;var r=indicator.getBoundingClientRect();var top=Math.min(window.innerHeight-popover.offsetHeight-12,r.bottom+8);var left=Math.min(window.innerWidth-popover.offsetWidth-12,Math.max(12,r.left));popover.style.top=Math.max(12,top)+'px';popover.style.left=left+'px';}
  function refresh(){if(!popover)return;var d=readConnection();popover.innerHTML='<h4>'+U.escapeHtml(U.t('gamma.conn.title'))+'</h4><dl><dt>'+U.escapeHtml(U.t('gamma.conn.latency'))+'</dt><dd data-conn-latency>'+U.escapeHtml(d.latency)+'</dd><dt>'+U.escapeHtml(U.t('gamma.conn.frameAge'))+'</dt><dd data-conn-frame-age>'+U.escapeHtml(d.frameAge)+'</dd><dt>'+U.escapeHtml(U.t('gamma.conn.retryCount'))+'</dt><dd data-conn-retry-count>'+U.escapeHtml(d.retryCount)+'</dd><dt>'+U.escapeHtml(U.t('gamma.conn.sourceMode'))+'</dt><dd data-conn-source-mode>'+U.escapeHtml(d.sourceMode)+'</dd></dl>';position();}
  function ensure(){if(popover)return;popover=document.createElement('div');popover.id='rvtConnDiagPopover';popover.className='rvt-conn-popover';popover.setAttribute('role','dialog');popover.setAttribute('aria-label',U.t('gamma.conn.title'));document.body.appendChild(popover);}
  function startRefresh(){if(!intervalId)intervalId=setInterval(refresh,1000);}function stopRefresh(){if(intervalId){clearInterval(intervalId);intervalId=0;}}
  function show(){ensure();refresh();popover.setAttribute('data-open','true');startRefresh();}
  function hide(force){if(!popover)return;if(pinned&&!force)return;popover.setAttribute('data-open','false');if(!pinned||force)stopRefresh();}
  window.rvtConnDiag={show:show,hide:function(){hide(true);},pin:function(){pinned=true;show();},unpin:function(){pinned=false;hide(true);},refresh:refresh,isPinned:function(){return pinned;}};
  U.bootSoon(function(){indicator=document.getElementById('conn')||document.querySelector('[data-connection-health]')||document.querySelector('.conn');if(!indicator)return;indicator.setAttribute('tabindex',indicator.getAttribute('tabindex')||'0');indicator.addEventListener('mouseenter',show);indicator.addEventListener('mouseleave',function(){hide(false);});indicator.addEventListener('focus',show);indicator.addEventListener('blur',function(){hide(false);});indicator.addEventListener('click',function(e){e.stopPropagation();if(pinned)window.rvtConnDiag.unpin();else window.rvtConnDiag.pin();});document.addEventListener('keydown',function(e){if(e.key==='Escape'&&pinned)window.rvtConnDiag.unpin();});document.addEventListener('click',function(e){if(!pinned||!popover)return;if(popover.contains(e.target)||indicator.contains(e.target))return;window.rvtConnDiag.unpin();});});
})();
;
/* END modules/rvt-nav04-conn-diag-js.js */

/* BEGIN modules/rvt-bug06-speech-debounce-js.js */
(function(){
  'use strict';
  var synth=window.speechSynthesis,NativeUtterance=window.SpeechSynthesisUtterance;
  var lastSpoken={text:'',timestamp:0},lastEndAt=0,queued=null,queueTimer=0;
  if(!synth||!NativeUtterance||synth.__rvtDebounced){window.rvtSpeechDebounce=window.rvtSpeechDebounce||{speak:function(){},cancel:function(){},getLastSpoken:function(){return lastSpoken;},getQueueLength:function(){return 0;}};return;}
  var downstreamSpeak=synth.speak.bind(synth),nativeCancel=synth.cancel.bind(synth);
  function shouldSuppress(text,priority,now){var ms=priority==='critical'?1000:3000;return lastSpoken.text===text&&now-lastSpoken.timestamp<ms;}
  function clearQueue(){queued=null;if(queueTimer){clearTimeout(queueTimer);queueTimer=0;}}
  function emit(text,priority){clearQueue();nativeCancel();var u=new NativeUtterance(text);u.__rvtDebounceBypass=true;u.__rvtPriority=priority||'normal';lastSpoken={text:text,timestamp:Date.now()};u.onend=function(){lastEndAt=Date.now();};u.onerror=function(){lastEndAt=Date.now();};downstreamSpeak(u);}
  function debouncedSpeak(text,priority){priority=priority==='critical'?'critical':'normal';text=String(text==null?'':text).trim();if(!text)return;var now=Date.now();if(shouldSuppress(text,priority,now))return;if(priority==='critical'){emit(text,priority);return;}var wait=Math.max(0,1000-(now-lastEndAt));if(wait>0){clearQueue();queued={text:text,priority:priority};queueTimer=setTimeout(function(){var q=queued;if(q)emit(q.text,q.priority);},wait);return;}emit(text,priority);}
  window.rvtSpeechDebounce={speak:debouncedSpeak,cancel:function(){clearQueue();nativeCancel();},getLastSpoken:function(){return{text:lastSpoken.text,timestamp:lastSpoken.timestamp};},getQueueLength:function(){return queued?1:0;}};
  synth.speak=function(utterance){if(utterance&&utterance.__rvtDebounceBypass)return downstreamSpeak(utterance);var text=utterance&&typeof utterance.text==='string'?utterance.text:String(utterance||'');var priority=utterance&&utterance.__rvtPriority?utterance.__rvtPriority:'normal';return debouncedSpeak(text,priority);};
  synth.__rvtDebounced=true;
})();
;
/* END modules/rvt-bug06-speech-debounce-js.js */

/* BEGIN modules/rvt-set06-settings-io-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  function currentSchemaVersion(){return(window.rvtStorage&&window.rvtStorage.schemaVersion)||Number(localStorage.getItem('rvt-schema-version'))||1;}
  function appVersion(){return window.DASHBOARD_VERSION||(window.RVT_CLIENT_VERSION&&(window.RVT_CLIENT_VERSION.dashboard||window.RVT_CLIENT_VERSION.version))||'unknown';}
  function manifestKeys(){var keys={};if(window.rvtStorage&&window.rvtStorage.manifest)Object.keys(window.rvtStorage.manifest).forEach(function(k){keys[k]=true;});for(var i=0;i<localStorage.length;i++){var key=localStorage.key(i);if(key&&key.indexOf('rvt-')===0)keys[key]=true;}return Object.keys(keys).sort();}
  function buildExportPayload(){var settings={};manifestKeys().forEach(function(k){settings[k]=localStorage.getItem(k);});return{schemaVersion:currentSchemaVersion(),exportedAt:new Date().toISOString(),appVersion:appVersion(),settings:settings};}
  function normalizeIncoming(incoming){if(!incoming||typeof incoming!=='object')throw new Error(U.t('gamma.settings.import.invalid'));if(incoming.settings&&typeof incoming.settings==='object')return incoming;return{schemaVersion:currentSchemaVersion(),exportedAt:'',appVersion:'',settings:incoming};}
  function diffPreview(incoming,current){incoming=normalizeIncoming(incoming);current=current||buildExportPayload();var currentSettings=current.settings||current||{};var manifest=(window.rvtStorage&&window.rvtStorage.manifest)||{};var diff=Object.keys(incoming.settings||{}).sort().map(function(key){var imported=incoming.settings[key];var cur=Object.prototype.hasOwnProperty.call(currentSettings,key)?currentSettings[key]:localStorage.getItem(key);var known=Object.prototype.hasOwnProperty.call(manifest,key);var same=String(imported)===String(cur);return{key:key,imported:imported,current:cur,changed:known&&!same,skipped:!known,status:!known?'skipped':(same?'no change':'changed')};});diff.schemaVersion=incoming.schemaVersion||0;diff.currentSchemaVersion=currentSchemaVersion();diff.warning=Number(diff.schemaVersion)>Number(diff.currentSchemaVersion);return diff;}
  function ensureModal(){var modal=document.getElementById('rvtSettingsImportModal');if(modal)return modal;modal=document.createElement('div');modal.id='rvtSettingsImportModal';modal.className='rvt-gamma-modal';modal.setAttribute('role','dialog');modal.setAttribute('aria-modal','true');modal.innerHTML='<div class="rvt-gamma-modal-panel"><h3>'+U.escapeHtml(U.t('gamma.settings.import.title'))+'</h3><div id="rvtSettingsImportWarning"></div><div id="rvtSettingsImportDiff" class="rvt-settings-diff-list"></div><div class="rvt-settings-modal-actions"><button type="button" class="rvt-settings-io-btn" id="rvtSettingsImportCancel">'+U.escapeHtml(U.t('gamma.settings.import.cancel'))+'</button><button type="button" class="rvt-settings-io-btn primary" id="rvtSettingsImportApply">'+U.escapeHtml(U.t('gamma.settings.import.apply'))+'</button></div></div>';document.body.appendChild(modal);document.getElementById('rvtSettingsImportCancel').addEventListener('click',function(){modal.setAttribute('data-open','false');});document.addEventListener('keydown',function(e){if(e.key==='Escape'&&modal.getAttribute('data-open')==='true')modal.setAttribute('data-open','false');});return modal;}
  function showDiffModal(diff){var modal=ensureModal();var warn=document.getElementById('rvtSettingsImportWarning');var list=document.getElementById('rvtSettingsImportDiff');var warnings=[];if(diff.warning)warnings.push(U.t('gamma.settings.import.newer'));if(diff.some(function(i){return i.skipped;}))warnings.push(U.t('gamma.settings.import.skipped'));warn.innerHTML=warnings.length?'<div class="rvt-settings-warning">'+U.escapeHtml(warnings.join(' '))+'</div>':'';list.innerHTML=diff.map(function(item){var status=item.skipped?U.t('gamma.settings.import.skipped'):(item.changed?U.escapeHtml(String(item.imported))+' -> '+U.escapeHtml(String(item.current)):U.t('gamma.settings.import.noChange'));return'<div class="rvt-settings-diff-item" data-settings-diff-key="'+U.escapeHtml(item.key)+'"><strong>'+U.escapeHtml(item.key)+'</strong><span>'+status+'</span></div>';}).join('');document.getElementById('rvtSettingsImportApply').onclick=function(){window.rvtSettingsIO.applyImport(diff);modal.setAttribute('data-open','false');};modal.setAttribute('data-open','true');return diff;}
  function applyImport(diff){var prior={};diff.forEach(function(item){if(!item.changed||item.skipped)return;prior[item.key]=localStorage.getItem(item.key);localStorage.setItem(item.key,String(item.imported));});U.toast(U.t('gamma.settings.import.applied'),U.t('gamma.settings.import.undo'),function(){Object.keys(prior).forEach(function(k){if(prior[k]==null)localStorage.removeItem(k);else localStorage.setItem(k,prior[k]);});});return diff;}
  function triggerExport(){var payload=buildExportPayload();var blob=new Blob([JSON.stringify(payload,null,2)],{type:'application/json;charset=utf-8'});var url=URL.createObjectURL(blob);var a=document.createElement('a');a.href=url;a.download='rvt_settings_'+new Date().toISOString().slice(0,10)+'.json';a.setAttribute('data-rvt-settings-download','true');document.body.appendChild(a);a.click();setTimeout(function(){URL.revokeObjectURL(url);a.remove();},0);}
  function importFile(file){if(file&&typeof file==='object'&&!file.text&&file.settings)return Promise.resolve(showDiffModal(diffPreview(file,buildExportPayload())));return Promise.resolve(file&&typeof file.text==='function'?file.text():String(file||'')).then(function(text){var parsed=typeof text==='string'?JSON.parse(text):text;return showDiffModal(diffPreview(parsed,buildExportPayload()));});}
  function renderSettingsUi(){if(document.getElementById('rvtA16Settings'))return;var settings=document.getElementById('view-settings')||document.getElementById('settingsView')||document.querySelector('[data-view="settings"]');if(!settings||document.getElementById('rvtSettingsPortability'))return;var section=document.createElement('section');section.id='rvtSettingsPortability';section.className='set-panel rvt-settings-portability';section.innerHTML='<h3>'+U.escapeHtml(U.t('gamma.settings.portability.title'))+'</h3><p class="muted">'+U.escapeHtml(U.t('gamma.settings.portability.desc'))+'</p><div class="rvt-settings-io-actions"><button type="button" class="rvt-settings-io-btn" id="rvtSettingsExportBtn">'+U.escapeHtml(U.t('gamma.settings.export'))+'</button><button type="button" class="rvt-settings-io-btn" id="rvtSettingsImportBtn">'+U.escapeHtml(U.t('gamma.settings.import'))+'</button><input id="rvtSettingsImportFile" type="file" accept=".json,application/json" hidden></div>';var anchor=settings.querySelector(':scope > .set-actions')||settings.lastElementChild;settings.insertBefore(section,(anchor&&anchor.parentNode===settings)?anchor:null);document.getElementById('rvtSettingsExportBtn').addEventListener('click',triggerExport);document.getElementById('rvtSettingsImportBtn').addEventListener('click',function(){document.getElementById('rvtSettingsImportFile').click();});document.getElementById('rvtSettingsImportFile').addEventListener('change',function(e){var f=e.target.files&&e.target.files[0];if(f)importFile(f).catch(function(err){U.toast(err.message||U.t('gamma.settings.import.invalid'));});e.target.value='';});}
  window.rvtSettingsIO={export:triggerExport,import:importFile,diffPreview:diffPreview,applyImport:applyImport,buildExportPayload:buildExportPayload};
  U.bootSoon(renderSettingsUi);
})();
;
/* END modules/rvt-set06-settings-io-js.js */

/* BEGIN modules/rvt-set08-reset-defaults-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  function stringifyDefault(entry){if(!entry)return'';if(entry.kind==='json')return JSON.stringify(entry.defaultValue);return String(entry.defaultValue==null?'':entry.defaultValue);}
  function labelForKey(key){return key.replace(/^rvt-/,'').replace(/-/g,' ');}
  function getDiff(){var manifest=(window.rvtStorage&&window.rvtStorage.manifest)||{};return Object.keys(manifest).sort().map(function(key){var current=localStorage.getItem(key);var def=stringifyDefault(manifest[key]);return{key:key,label:labelForKey(key),current:current==null?'':current,default:def};}).filter(function(item){return String(item.current)!==String(item.default);});}
  function resetNow(diff){var prior={};var manifest=(window.rvtStorage&&window.rvtStorage.manifest)||{};diff.forEach(function(item){prior[item.key]=localStorage.getItem(item.key);localStorage.setItem(item.key,stringifyDefault(manifest[item.key]));});U.toast(U.t('gamma.settings.reset.done'),U.t('gamma.settings.reset.undo'),function(){Object.keys(prior).forEach(function(k){if(prior[k]==null)localStorage.removeItem(k);else localStorage.setItem(k,prior[k]);});U.toast(U.t('gamma.settings.reset.undone'));});}
  function patchQolModalForReset(zero){setTimeout(function(){var label=document.getElementById('rvtDiffChkLabel');if(label)label.textContent=U.t('gamma.settings.reset.checkbox');var confirm=document.getElementById('rvtDiffConfirmBtn');var chk=document.getElementById('rvtDiffConfirmChk');var cancel=document.getElementById('rvtDiffCancelBtn');if(confirm)confirm.textContent=U.t('gamma.settings.reset.apply');if(zero){if(confirm)confirm.style.display='none';if(chk&&chk.parentElement)chk.parentElement.style.display='none';if(cancel)cancel.textContent=U.t('gamma.settings.reset.close');}},0);}
  function confirmReset(){var diff=getDiff();if(typeof window.rvtConfirmDestructive!=='function'){if(diff.length)resetNow(diff);else U.toast(U.t('gamma.settings.reset.none'));return;}if(!diff.length){window.rvtConfirmDestructive({title:U.t('gamma.settings.reset.title'),body:U.t('gamma.settings.reset.none'),items:[U.t('gamma.settings.reset.none')],onConfirm:function(){}});patchQolModalForReset(true);return;}var items=diff.map(function(item){return item.label+': '+item.current+' -> '+item.default;});window.rvtConfirmDestructive({title:U.t('gamma.settings.reset.title'),body:U.t('gamma.settings.reset.checkbox'),items:items,onConfirm:function(){resetNow(diff);}});patchQolModalForReset(false);}
  window.rvtResetDefaults={getDiff:getDiff,confirm:confirmReset};
  window.confirmResetDashboardDefaults=confirmReset;
  U.bootSoon(function(){Array.prototype.forEach.call(document.querySelectorAll('button,a'),function(node){var text=(node.textContent||'').toLowerCase();var onclick=node.getAttribute('onclick')||'';if(onclick.indexOf('confirmResetDashboardDefaults')!==-1||text.indexOf('reset to defaults')!==-1){node.addEventListener('click',function(e){if(onclick.indexOf('confirmResetDashboardDefaults')!==-1){e.preventDefault();confirmReset();}});}});});
})();
;
/* END modules/rvt-set08-reset-defaults-js.js */

/* BEGIN modules/rvt-rpt01-sticky-verdict-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var STRINGS={
    'gamma.rpt.sticky.pass':'Session passed','gamma.rpt.sticky.warn':'Session warning','gamma.rpt.sticky.fail':'Session failed',
    'gamma.rpt.sticky.scrollTop':'Back to top',
    'gamma.rpt06.export':'Export report','gamma.rpt06.printing':'Preparing print…',
    'gamma.alert.sev.info':'Info','gamma.alert.sev.warn':'Warning','gamma.alert.sev.critical':'Critical',
    'gamma.alert.dismiss.label':'Auto-dismiss in {s}s','gamma.alert.dismiss.dismissed':'Alert dismissed',
    'gamma.alert.jump.label':'Jump to waveform','gamma.alert.jump.unavailable':'Waveform view not available',
    'gamma.alert.sound.on':'Alert sounds on','gamma.alert.sound.off':'Alert sounds off','gamma.alert.sound.label':'Toggle alert sounds',
    'gamma.live.tooltip.title':'Computation details','gamma.live.tooltip.unavailable':'Computation details unavailable','gamma.live.tooltip.value':'Current value','gamma.live.tooltip.source':'Data source','gamma.live.tooltip.buffer':'Buffer size',
    'gamma.live.mode.switched':'Mode switched to {mode}','gamma.live.mode.shortcut':'Press M to toggle mode',
    'gamma.live.snap.taken':'Snapshot #{n} captured','gamma.live.snap.shortcut':'Press P to capture',
    'gamma.live.anim.paused':'KPI animations paused','gamma.live.anim.resumed':'KPI animations resumed','gamma.live.anim.label':'Toggle KPI animations'
  };
  function extendB2B(){window.RVT_STRINGS=window.RVT_STRINGS||{en:{}};window.RVT_STRINGS.en=window.RVT_STRINGS.en||{};Object.keys(STRINGS).forEach(function(k){if(!(k in window.RVT_STRINGS.en))window.RVT_STRINGS.en[k]=STRINGS[k];});}
  function t(k,v){extendB2B();return U.t(k,v);}
  function registerB2BManifest(){if(!window.rvtStorage||!window.rvtStorage.manifest)return;var m=window.rvtStorage.manifest;
    if(!m['rvt-alert-auto-dismiss-s'])m['rvt-alert-auto-dismiss-s']={kind:'scalar',defaultValue:'30',validate:function(v){var n=Number(v);return Number.isFinite(n)&&n>=0&&n<=120;}};
    if(!m['rvt-kpi-animations'])m['rvt-kpi-animations']={kind:'scalar',defaultValue:'1',validate:function(v){return v==='0'||v==='1';}};
    if(!m['rvt-report-sticky-hero'])m['rvt-report-sticky-hero']={kind:'scalar',defaultValue:'1',validate:function(v){return v==='0'||v==='1';}};
  }
  extendB2B();registerB2BManifest();
  window.rvtGammaBatch2b={t:t,registerB2BManifest:registerB2BManifest};

  var bar=null,hero=null,observer=null;
  function readVerdict(){
    hero=document.querySelector('.verdict-hero');if(!hero)return null;
    var kind='pass';if(hero.classList.contains('warn'))kind='warn';if(hero.classList.contains('fail'))kind='fail';
    var label=hero.querySelector('.vh-kind');
    var metrics=hero.querySelectorAll('.vh-metric .metric-val,.vh-metric .metric-badge');
    var metricTexts=[];Array.prototype.forEach.call(metrics,function(m){if(m.textContent.trim())metricTexts.push(m.textContent.trim());});
    return{kind:kind,label:label?label.textContent.trim():'',metrics:metricTexts.slice(0,3)};
  }
  function ensureBar(){
    if(bar)return bar;
    var report=document.getElementById('view-report')||document.querySelector('[data-view="report"]');if(!report)return null;
    bar=document.createElement('div');bar.className='rvt-sticky-verdict';bar.id='rvtStickyVerdict';bar.setAttribute('role','banner');bar.setAttribute('aria-label','Session verdict summary');
    report.insertBefore(bar,report.firstChild);return bar;
  }
  function updateBar(visible){
    if(!bar)return;
    var v=readVerdict();if(!v){bar.setAttribute('data-visible','false');return;}
    bar.className='rvt-sticky-verdict '+v.kind;
    var icon=v.kind==='pass'?'check_circle':v.kind==='warn'?'warning':'cancel';
    bar.innerHTML='<span class="material-symbols-rounded rvt-sv-icon" aria-hidden="true">'+icon+'</span><span class="rvt-sv-label">'+U.escapeHtml(v.label||t('gamma.rpt.sticky.'+v.kind))+'</span><div class="rvt-sv-metrics">'+v.metrics.map(function(m){return'<span>'+U.escapeHtml(m)+'</span>';}).join('')+'</div>';
    bar.setAttribute('data-visible',visible?'true':'false');
  }
  function initObserver(){
    hero=document.querySelector('.verdict-hero');if(!hero||observer)return;
    ensureBar();
    if(!('IntersectionObserver' in window)){return;}
    observer=new IntersectionObserver(function(entries){
      entries.forEach(function(entry){updateBar(!entry.isIntersecting);});
    },{threshold:0,rootMargin:'-1px 0px 0px 0px'});
    observer.observe(hero);
  }
  window.rvtStickyVerdict={refresh:function(){updateBar(bar&&bar.getAttribute('data-visible')==='true');},destroy:function(){if(observer){observer.disconnect();observer=null;}if(bar){bar.remove();bar=null;}}};
  U.bootSoon(function(){
    if(localStorage.getItem('rvt-report-sticky-hero')==='0')return;
    initObserver();
    var origSwitch=window.switchView;if(typeof origSwitch==='function'&&!origSwitch.__rvtStickyWrapped){window.switchView=function(v){var r=origSwitch.apply(this,arguments);if(v==='report')setTimeout(initObserver,100);return r;};window.switchView.__rvtStickyWrapped=true;}
  });
})();
;
/* END modules/rvt-rpt01-sticky-verdict-js.js */

/* BEGIN modules/rvt-rpt06-report-export-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  function renderExportButton(){
    var report=document.getElementById('view-report')||document.querySelector('[data-view="report"]');if(!report||document.getElementById('rvtReportExportWrap'))return;
    var wrap=document.createElement('div');wrap.className='rvt-report-export-wrap';wrap.id='rvtReportExportWrap';
    wrap.innerHTML='<button type="button" class="rvt-report-export-btn" id="rvtReportExportBtn" aria-label="'+U.escapeHtml(t('gamma.rpt06.export'))+'"><span class="material-symbols-rounded" aria-hidden="true">print</span><span>'+U.escapeHtml(t('gamma.rpt06.export'))+'</span></button>';
    var hero=report.querySelector('.verdict-hero');var target=hero?hero.nextElementSibling:report.firstChild;
    report.insertBefore(wrap,target);
    document.getElementById('rvtReportExportBtn').addEventListener('click',function(){window.rvtReportExport.print();});
  }
  window.rvtReportExport={print:function(){
    U.toast(t('gamma.rpt06.printing'));
    setTimeout(function(){window.print();},150);
  }};
  U.bootSoon(renderExportButton);
})();
;
/* END modules/rvt-rpt06-report-export-js.js */

/* BEGIN modules/rvt-alert02-severity-badge-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  function enhanceAlertItem(node){
    if(!node||node.querySelector('.rvt-alert-sev'))return;
    var text=(node.textContent||'').toLowerCase();
    var sev='info';
    if(/critical|fatal|error|bad/.test(text))sev='critical';
    else if(/warn|warning/.test(text))sev='warn';
    var badge=document.createElement('span');badge.className='rvt-alert-sev';badge.setAttribute('data-sev',sev);
    badge.textContent=t('gamma.alert.sev.'+sev);
    node.insertBefore(badge,node.firstChild);
    node.setAttribute('data-alert-severity',sev);
  }
  function scanDrawer(){
    var body=document.getElementById('dvBody');if(!body)return;
    Array.prototype.forEach.call(body.children,function(child){
      if(child.nodeType===1&&!child.querySelector('.rvt-alert-sev'))enhanceAlertItem(child);
    });
  }
  var dvBody=null,mutObs=null;
  function observeDrawer(){
    dvBody=document.getElementById('dvBody');if(!dvBody||mutObs)return;
    mutObs=new MutationObserver(function(){scanDrawer();});
    mutObs.observe(dvBody,{childList:true,subtree:false});
    scanDrawer();
  }
  window.rvtAlertSeverityBadge={scan:scanDrawer,enhance:enhanceAlertItem};
  U.bootSoon(observeDrawer);
})();
;
/* END modules/rvt-alert02-severity-badge-js.js */

/* BEGIN modules/rvt-alert04-auto-dismiss-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  var timers=[];
  function getDismissSeconds(){var v=localStorage.getItem('rvt-alert-auto-dismiss-s');var n=Number(v);return Number.isFinite(n)&&n>=0&&n<=120?n:30;}
  function attachDismiss(node){
    if(!node||node.getAttribute('data-auto-dismiss')==='attached')return;
    var sev=node.getAttribute('data-alert-severity')||'info';
    if(sev==='critical')return;
    var seconds=getDismissSeconds();if(seconds<=0)return;
    node.setAttribute('data-auto-dismiss','attached');
    node.style.setProperty('--dismiss-s',seconds+'s');
    var bar=document.createElement('div');bar.className='rvt-alert-dismiss-bar';bar.setAttribute('aria-hidden','true');
    node.appendChild(bar);
    var tid=setTimeout(function(){
      node.style.opacity='0';node.style.transition='opacity 200ms ease';
      setTimeout(function(){node.remove();},220);
    },seconds*1000);
    timers.push(tid);
  }
  function scanForDismiss(){
    var body=document.getElementById('dvBody');if(!body)return;
    Array.prototype.forEach.call(body.children,function(child){
      if(child.nodeType===1)attachDismiss(child);
    });
  }
  var mutObs=null;
  function observeDrawer(){
    var dvBody=document.getElementById('dvBody');if(!dvBody||mutObs)return;
    mutObs=new MutationObserver(function(){scanForDismiss();});
    mutObs.observe(dvBody,{childList:true,subtree:false});
    scanForDismiss();
  }
  window.rvtAlertAutoDismiss={scan:scanForDismiss,getSeconds:getDismissSeconds,clearTimers:function(){timers.forEach(clearTimeout);timers=[];}};
  U.bootSoon(observeDrawer);
})();
;
/* END modules/rvt-alert04-auto-dismiss-js.js */

/* BEGIN modules/rvt-alert06-waveform-jump-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  function extractTimestamp(node){
    if(!node)return null;
    var text=node.textContent||'';
    var match=text.match(/(\d{2}:\d{2}:\d{2})/);
    return match?match[1]:null;
  }
  function jumpToWaveform(ts){
    if(!ts){U.toast(t('gamma.alert.jump.unavailable'));return false;}
    if(typeof window.switchView==='function')window.switchView('live');
    if(typeof window.switchTab==='function')window.switchTab('tab-waves');
    if(typeof window.rvtWaveformSeek==='function'){window.rvtWaveformSeek(ts);return true;}
    U.toast(t('gamma.alert.jump.unavailable'));
    return false;
  }
  function addJumpButtons(){
    var body=document.getElementById('dvBody');if(!body)return;
    Array.prototype.forEach.call(body.children,function(child){
      if(child.nodeType!==1||child.querySelector('.rvt-alert-jump-btn'))return;
      var ts=extractTimestamp(child);if(!ts)return;
      var btn=document.createElement('button');btn.type='button';btn.className='rvt-alert-jump-btn';
      btn.setAttribute('aria-label',t('gamma.alert.jump.label'));btn.title=t('gamma.alert.jump.label');
      btn.innerHTML='<span class="material-symbols-rounded" aria-hidden="true" style="font-size:16px">open_in_new</span>';
      btn.setAttribute('data-jump-ts',ts);
      btn.addEventListener('click',function(e){e.stopPropagation();jumpToWaveform(ts);});
      child.appendChild(btn);
    });
  }
  var mutObs=null;
  function observe(){
    var dvBody=document.getElementById('dvBody');if(!dvBody||mutObs)return;
    mutObs=new MutationObserver(function(){addJumpButtons();});
    mutObs.observe(dvBody,{childList:true,subtree:false});
    addJumpButtons();
  }
  window.rvtAlertJump={jump:jumpToWaveform,scan:addJumpButtons};
  U.bootSoon(observe);
})();
;
/* END modules/rvt-alert06-waveform-jump-js.js */

/* BEGIN modules/rvt-alert08-sound-toggle-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  function isOn(){return localStorage.getItem('rvt-audio-alerts')==='1';}
  function toggle(){
    var next=isOn()?'0':'1';
    localStorage.setItem('rvt-audio-alerts',next);
    if(window.rvtStorage&&typeof window.rvtStorage.set==='function')window.rvtStorage.set('rvt-audio-alerts',next);
    updateUi();
    U.toast(next==='1'?t('gamma.alert.sound.on'):t('gamma.alert.sound.off'));
  }
  function updateUi(){
    var btn=document.getElementById('rvtAlertSoundToggle');if(!btn)return;
    var on=isOn();
    btn.setAttribute('aria-pressed',on?'true':'false');
    var icon=btn.querySelector('.material-symbols-rounded');
    if(icon)icon.textContent=on?'volume_up':'volume_off';
    var label=btn.querySelector('.rvt-alert-sound-text');
    if(label)label.textContent=t('gamma.alert.sound.label');
  }
  function renderButton(){
    var header=document.querySelector('.dv-h')||document.querySelector('#dv .dv-h');
    if(!header||document.getElementById('rvtAlertSoundToggle'))return;
    var btn=document.createElement('button');btn.type='button';btn.id='rvtAlertSoundToggle';btn.className='rvt-alert-sound-toggle';
    btn.innerHTML='<span class="material-symbols-rounded" aria-hidden="true"></span><span class="rvt-alert-sound-text"></span>';
    btn.addEventListener('click',function(){window.rvtAlertSound.toggle();});
    var close=header.querySelector('button');header.insertBefore(btn,close);
    updateUi();
  }
  window.rvtAlertSound={toggle:toggle,isOn:isOn};
  U.bootSoon(renderButton);
})();
;
/* END modules/rvt-alert08-sound-toggle-js.js */

/* BEGIN modules/rvt-live04-kpi-tooltip-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  var activeTooltip=null;
  function getKpiData(kpiId){
    var S=window.S||{};
    var payload=S.lastPayload||S.latest||{};
    var value=payload[kpiId]||payload[kpiId+'_corrected']||payload[kpiId+'_raw'];
    var source=payload.publish_source||payload.publishSource||'';
    var buffer=S.liveBuffer?S.liveBuffer.length:localStorage.getItem('rvt-live-buffer-seconds')||'60';
    if(S.funnels&&S.funnels[kpiId]){
      var f=S.funnels[kpiId];
      return{value:f.value||value,source:f.source||source,buffer:f.bufferSize||buffer,extra:f};
    }
    return{value:value!=null?value:'--',source:source||t('gamma.live.tooltip.unavailable'),buffer:buffer};
  }
  function showTooltip(card,kpiId){
    hideTooltip();
    var data=getKpiData(kpiId);
    var tip=document.createElement('div');tip.className='rvt-kpi-tooltip';tip.id='rvtKpiTooltip';
    tip.setAttribute('role','tooltip');
    tip.innerHTML='<strong>'+U.escapeHtml(t('gamma.live.tooltip.title'))+'</strong><dl>'+
      '<dt>'+U.escapeHtml(t('gamma.live.tooltip.value'))+'</dt><dd>'+U.escapeHtml(String(data.value))+'</dd>'+
      '<dt>'+U.escapeHtml(t('gamma.live.tooltip.source'))+'</dt><dd>'+U.escapeHtml(String(data.source))+'</dd>'+
      '<dt>'+U.escapeHtml(t('gamma.live.tooltip.buffer'))+'</dt><dd>'+U.escapeHtml(String(data.buffer))+'</dd></dl>';
    card.style.position='relative';card.appendChild(tip);
    requestAnimationFrame(function(){tip.setAttribute('data-visible','true');});
    activeTooltip={tip:tip,card:card};
  }
  function hideTooltip(){
    if(!activeTooltip)return;
    activeTooltip.tip.remove();activeTooltip=null;
  }
  function enhanceKpis(){
    var cards=document.querySelectorAll('.kpi[data-kpi-id]');
    Array.prototype.forEach.call(cards,function(card){
      if(card.getAttribute('data-tooltip-enhanced'))return;
      card.setAttribute('data-tooltip-enhanced','true');
      var kpiId=card.getAttribute('data-kpi-id');
      card.addEventListener('mouseenter',function(){showTooltip(card,kpiId);});
      card.addEventListener('mouseleave',hideTooltip);
      card.addEventListener('focus',function(){showTooltip(card,kpiId);});
      card.addEventListener('blur',hideTooltip);
    });
  }
  window.rvtKpiTooltip={show:showTooltip,hide:hideTooltip,refresh:enhanceKpis};
  U.bootSoon(enhanceKpis);
})();
;
/* END modules/rvt-live04-kpi-tooltip-js.js */

/* BEGIN modules/rvt-live05-mode-shortcuts-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  document.addEventListener('keydown',function(e){
    if(e.ctrlKey||e.metaKey||e.altKey)return;
    var active=document.activeElement;
    if(active&&(active.tagName==='TEXTAREA'||active.tagName==='INPUT'||active.isContentEditable))return;
    if(document.body.getAttribute('data-view')!=='live')return;
    if(e.key==='m'||e.key==='M'){
      e.preventDefault();
      var current=localStorage.getItem('rvt-live-mode')||'simple';
      var next=current==='simple'?'advanced':'simple';
      if(typeof window.setLiveMode==='function'){window.setLiveMode(next);}
      else{localStorage.setItem('rvt-live-mode',next);document.body.setAttribute('data-live-mode',next);}
      U.toast(t('gamma.live.mode.switched',{mode:next}));
    }
  });
  window.rvtModeShortcut={getBinding:function(){return'M';}};
})();
;
/* END modules/rvt-live05-mode-shortcuts-js.js */

/* BEGIN modules/rvt-live06-snap-enhance-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  var origDoSnapshot=window.doSnapshot;
  function showSnapToast(){
    var count=(window.S&&Array.isArray(window.S.snaps))?window.S.snaps.length:0;
    var existing=document.querySelector('.rvt-snap-toast');if(existing)existing.remove();
    var toast=document.createElement('div');toast.className='rvt-snap-toast';toast.setAttribute('role','status');toast.setAttribute('aria-live','polite');
    toast.innerHTML='<span class="material-symbols-rounded" aria-hidden="true">photo_camera</span><span>'+U.escapeHtml(t('gamma.live.snap.taken',{n:count||'?'}))+'</span>';
    document.body.appendChild(toast);
    setTimeout(function(){toast.remove();},2000);
  }
  if(typeof origDoSnapshot==='function'&&!origDoSnapshot.__rvtB2bWrapped){
    window.doSnapshot=function(){
      var ret=origDoSnapshot.apply(this,arguments);
      showSnapToast();
      return ret;
    };
    window.doSnapshot.__rvtB2bWrapped=true;
  }else{
    var snapFab=document.getElementById('rvtSnapFab');
    if(snapFab&&!snapFab.__rvtB2bEnhanced){
      snapFab.addEventListener('click',function(){setTimeout(showSnapToast,50);});
      snapFab.__rvtB2bEnhanced=true;
    }
  }
  window.rvtSnapEnhance={showToast:showSnapToast};
})();
;
/* END modules/rvt-live06-snap-enhance-js.js */

/* BEGIN modules/rvt-live07-kpi-anim-pause-js.js */
(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  var paused=(localStorage.getItem('rvt-kpi-animations')||'1')==='0';
  function apply(){
    document.body.classList.toggle('rvt-kpi-anim-paused',paused);
    localStorage.setItem('rvt-kpi-animations',paused?'0':'1');
    var btn=document.getElementById('rvtKpiAnimToggle');if(!btn)return;
    btn.setAttribute('aria-pressed',paused?'true':'false');
    var icon=btn.querySelector('.material-symbols-rounded');
    if(icon)icon.textContent=paused?'play_arrow':'pause';
  }
  function toggle(){
    paused=!paused;apply();
    U.toast(paused?t('gamma.live.anim.paused'):t('gamma.live.anim.resumed'));
  }
  function renderToggle(){
    var kpiRow=document.querySelector('.kpi-row')||document.querySelector('.kpi-grid');
    if(!kpiRow||document.getElementById('rvtKpiAnimToggle'))return;
    var btn=document.createElement('button');btn.type='button';btn.id='rvtKpiAnimToggle';btn.className='rvt-anim-toggle';
    btn.setAttribute('aria-label',t('gamma.live.anim.label'));btn.title=t('gamma.live.anim.label');
    btn.innerHTML='<span class="material-symbols-rounded" aria-hidden="true"></span><span>'+U.escapeHtml(t('gamma.live.anim.label'))+'</span>';
    btn.addEventListener('click',function(){window.rvtKpiAnimPause.toggle();});
    kpiRow.parentNode.insertBefore(btn,kpiRow.nextSibling);
  }
  window.rvtKpiAnimPause={toggle:toggle,isPaused:function(){return paused;},apply:apply};
  U.bootSoon(function(){renderToggle();apply();});
})();
;
/* END modules/rvt-live07-kpi-anim-pause-js.js */

/* BEGIN modules/rvt-rpt01-verdict-hero-js.js */
(function(){
  'use strict';
  var STRINGS={
    'gamma2b.rpt01.summary':'Session verdict summary',
    'gamma2b.rpt06.signoff':'Sign off report',
    'gamma2b.rpt06.view':'View sign-off',
    'gamma2b.rpt06.title':'Report sign-off',
    'gamma2b.rpt06.operator':'Operator name',
    'gamma2b.rpt06.initials':'Initials',
    'gamma2b.rpt06.comment':'Comment',
    'gamma2b.rpt06.confirm':'Confirm sign-off',
    'gamma2b.rpt06.close':'Close',
    'gamma2b.rpt06.signed':'Signed off: {initials} \\u00b7 {time}',
    'gamma2b.rpt06.audit':'Report signed off by {initials} at {timestamp}',
    'gamma2b.rpt06.unsigned':'Report not signed off',
    'gamma2b.alert.recent':'Last 5 min',
    'gamma2b.alert.session':'Earlier this session',
    'gamma2b.alert.previous':'Previous sessions',
    'gamma2b.alert.snooze':'Snooze alert',
    'gamma2b.alert.jump':'Jump to this moment in waveform',
    'gamma2b.alert.waveUnavailable':'Waveform data not available for this time',
    'gamma2b.alert.snoozeEnded':'Snooze ended',
    'gamma2b.alert.snoozed':'{count} snoozed',
    'gamma2b.alert.snooze5':'5 min',
    'gamma2b.alert.snooze15':'15 min',
    'gamma2b.alert.snooze60':'60 min',
    'gamma2b.alert.custom':'Custom',
    'gamma2b.alert.minutes':'Minutes',
    'gamma2b.alert.snoozeConfirm':'Snooze',
    'gamma2b.alert.badge':'Alerts: {unacked} unacknowledged, {fresh} new',
    'gamma2b.alert.badgeTip':'{unacked} unacknowledged \\u00b7 {fresh} new since last viewed',
    'gamma2b.live.lineageUnavailable':'Computation details unavailable',
    'gamma2b.live.tagged':'Tagged: {type} @ {time}',
    'gamma2b.live.snapshot':'Snapshot Now',
    'gamma2b.live.snapshotLabel':'Snapshot label',
    'gamma2b.live.saveLabel':'Save label'
  };
  function installStrings(){
    window.RVT_STRINGS=window.RVT_STRINGS||{en:{}};
    window.RVT_STRINGS.en=window.RVT_STRINGS.en||{};
    Object.keys(STRINGS).forEach(function(k){if(!(k in window.RVT_STRINGS.en))window.RVT_STRINGS.en[k]=STRINGS[k];});
  }
  function t(key,vars){
    installStrings();
    var s=(window.RVT_STRINGS&&window.RVT_STRINGS.en&&window.RVT_STRINGS.en[key])||key;
    if(s===key&&typeof window.rvtT==='function')s=window.rvtT(key,vars||{});
    vars=vars||{};
    Object.keys(vars).forEach(function(k){s=String(s).replace(new RegExp('\\{'+k+'\\}','g'),vars[k]);});
    return s;
  }
  function esc(v){return String(v==null?'':v).replace(/[&<>"']/g,function(ch){return({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'})[ch];});}
  function toast(msg){
    if(window.rvtGammaBatch2a&&typeof window.rvtGammaBatch2a.toast==='function'){window.rvtGammaBatch2a.toast(msg);return;}
    var old=document.querySelector('.rvt-quicktag-toast');if(old)old.remove();
    var node=document.createElement('div');node.className='rvt-quicktag-toast';node.setAttribute('role','status');node.setAttribute('aria-live','polite');node.textContent=msg;document.body.appendChild(node);
    setTimeout(function(){node.remove();},3000);
  }
  function boot(fn){if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',fn,{once:true});else setTimeout(fn,0);}
  function registerManifest(){
    window.rvtStorage=window.rvtStorage||{};
    window.rvtStorage.manifest=window.rvtStorage.manifest||{};
    var m=window.rvtStorage.manifest;
    if(!m['rvt-alert-clusters'])m['rvt-alert-clusters']={kind:'json',defaultValue:{recent:true,session:true,previous:false},validate:function(v){return v!==null&&typeof v==='object'&&'recent'in v&&'session'in v&&'previous'in v;}};
    if(!m['rvt-alert-snooze'])m['rvt-alert-snooze']={kind:'json',defaultValue:{},validate:function(v){return v!==null&&typeof v==='object';}};
    if(!m['rvt-report-signoff-*'])m['rvt-report-signoff-*']={kind:'json',defaultValue:null,validate:function(v){return v===null||!!(typeof v==='object'&&v!==null&&v.sessionId&&v.operator);}};
  }
  window.rvtGammaBatch2bRuntime={t:t,esc:esc,toast:toast,boot:boot,registerManifest:registerManifest};
  installStrings();
  registerManifest();

  var stuck=false,hero=null,observer=null,sentinel=null,fullHeight=0;
  function mobile(){return window.matchMedia&&window.matchMedia('(max-width: 767px)').matches;}
  function findHero(){return document.querySelector('[data-rvt-rpt01-hero],#rvt-rpt01-hero,.rvt-verdict-hero,.verdict-hero');}
  function applyState(next){
    hero=findHero();
    if(!hero)return false;
    hero.classList.add('rvt-rpt01-sticky');
    if(mobile())next=false;
    stuck=!!next;
    if(!fullHeight)fullHeight=hero.getBoundingClientRect().height||Number(hero.dataset.rvtRpt01FullHeight)||180;
    hero.dataset.rvtRpt01FullHeight=String(fullHeight);
    hero.classList.toggle('rvt-rpt01-collapsed',stuck);
    hero.classList.toggle('is-stuck',stuck);
    hero.classList.toggle('collapsed',stuck);
    if(stuck){
      hero.style.height=Math.max(44,Math.round(fullHeight/2))+'px';
      hero.style.minHeight=Math.max(44,Math.round(fullHeight/2))+'px';
    }else{
      hero.style.height='';
      hero.style.minHeight='';
    }
    return stuck;
  }
  function initObserver(){
    hero=findHero();
    if(!hero||observer||!('IntersectionObserver'in window))return;
    hero.classList.add('rvt-rpt01-sticky');
    sentinel=document.createElement('div');
    sentinel.id='rvt-rpt01-sentinel';
    sentinel.style.cssText='height:1px;width:1px;';
    hero.parentNode.insertBefore(sentinel,hero);
    observer=new IntersectionObserver(function(entries){entries.forEach(function(entry){applyState(!entry.isIntersecting&&window.scrollY>100);});},{threshold:0});
    observer.observe(sentinel);
  }
  window.rvtVerdictHero={
    isStuck:function(){var y=window.scrollY||document.documentElement.scrollTop||document.body.scrollTop||0;var h=findHero();if(!stuck&&!mobile()&&(y>100||(h&&h.getBoundingClientRect().top<=100)))applyState(true);return !!stuck;},
    forceCollapse:function(){return applyState(true);},
    forceExpand:function(){return applyState(false);},
    destroy:function(){if(observer){observer.disconnect();observer=null;}if(sentinel){sentinel.remove();sentinel=null;}applyState(false);}
  };
  boot(initObserver);
})();
;
/* END modules/rvt-rpt01-verdict-hero-js.js */

/* BEGIN modules/rvt-rpt06-signoff-js.js */
(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var modal=null;
  function sessionId(fallback){var S=window.S||{};return fallback||S.currentSessionId||S.sessionId||(S.report&&S.report.sessionId)||(S.currentReport&&S.currentReport.sessionId)||'session';}
  function key(id){return'rvt-report-signoff-'+sessionId(id);}
  function read(id){try{return JSON.parse(localStorage.getItem(key(id))||'null');}catch(_){return null;}}
  function write(obj){localStorage.setItem(key(obj.sessionId),JSON.stringify(obj));}
  function operatorName(){var S=window.S||{};return(S.settings&&(S.settings.operatorName||S.settings.operator))||S.currentOperator||localStorage.getItem('rvt-operator-name')||localStorage.getItem('rvt-current-operator')||'';}
  function initials(name){return String(name||'').trim().split(/\s+/).filter(Boolean).map(function(p){return p[0]||'';}).join('').slice(0,4).toUpperCase();}
  function reportLoaded(){var S=window.S||{};var r=S.report||S.currentReport||{};return r.completed!==false;}
  function audit(obj){var S=window.S=window.S||{};var r=S.report||S.currentReport||(S.report={auditLog:[]});r.signoff=obj;r.auditLog=r.auditLog||[];r.auditLog.push({timestamp:obj.timestamp,message:R.t('gamma2b.rpt06.audit',{initials:obj.initials,timestamp:obj.timestamp})});}
  function ensureToolbar(){
    var report=document.getElementById('view-report')||document.querySelector('[data-view="report"]');if(!report)return;
    if(document.getElementById('rvt-rpt06-toolbar')){updateToolbar();return;}
    var bar=document.createElement('div');bar.id='rvt-rpt06-toolbar';bar.className='rvt-rpt06-toolbar';
    bar.innerHTML='<button type="button" class="rvt-rpt06-btn" id="rvt-rpt06-signoff-btn"><span class="material-symbols-rounded" aria-hidden="true">verified</span><span data-rpt06-label></span></button><span class="rvt-rpt06-status" id="rvt-rpt06-status" aria-live="polite"></span>';
    var hero=report.querySelector('.verdict-hero,.rvt-verdict-hero')||report.firstElementChild;
    report.insertBefore(bar,hero?hero.nextSibling:report.firstChild);
    document.getElementById('rvt-rpt06-signoff-btn').addEventListener('click',function(){open(sessionId());});
    updateToolbar();
  }
  function updateToolbar(){
    var id=sessionId();var data=read(id);var btn=document.getElementById('rvt-rpt06-signoff-btn');var status=document.getElementById('rvt-rpt06-status');if(!btn)return;
    btn.disabled=!reportLoaded();
    btn.querySelector('[data-rpt06-label]').textContent=data?R.t('gamma2b.rpt06.view'):R.t('gamma2b.rpt06.signoff');
    if(status)status.innerHTML=data?'<span class="material-symbols-rounded" aria-hidden="true">check_circle</span>'+R.esc(R.t('gamma2b.rpt06.signed',{initials:data.initials||'',time:new Date(data.timestamp).toLocaleTimeString()})):'';
  }
  function close(){if(modal){modal.remove();modal=null;}}
  function open(id){
    id=sessionId(id);var existing=read(id);var readonly=!!existing;var op=existing?existing.operator:operatorName();var init=existing?existing.initials:initials(op);
    close();
    modal=document.createElement('div');modal.className='rvt-rpt06-backdrop';modal.id='rvt-rpt06-modal-backdrop';
    modal.innerHTML='<div class="rvt-rpt06-modal" role="dialog" aria-modal="true" aria-labelledby="rvt-rpt06-title"><h2 id="rvt-rpt06-title">'+R.esc(R.t('gamma2b.rpt06.title'))+'</h2><label class="rvt-rpt06-field">'+R.esc(R.t('gamma2b.rpt06.operator'))+'<input id="rvt-rpt06-operator" name="operator" data-rpt06-operator maxlength="80"></label><label class="rvt-rpt06-field">'+R.esc(R.t('gamma2b.rpt06.initials'))+'<input id="rvt-rpt06-initials" name="initials" data-rpt06-initials maxlength="4"></label><label class="rvt-rpt06-field">'+R.esc(R.t('gamma2b.rpt06.comment'))+'<textarea id="rvt-rpt06-comment" name="comment" data-rpt06-comment maxlength="500"></textarea></label><div class="rvt-rpt06-actions"><button type="button" data-rpt06-close>'+R.esc(R.t('gamma2b.rpt06.close'))+'</button><button type="button" data-rpt06-confirm>'+R.esc(R.t('gamma2b.rpt06.confirm'))+'</button></div></div>';
    document.body.appendChild(modal);
    var opEl=modal.querySelector('#rvt-rpt06-operator'),initEl=modal.querySelector('#rvt-rpt06-initials'),commentEl=modal.querySelector('#rvt-rpt06-comment'),confirm=modal.querySelector('[data-rpt06-confirm]');
    opEl.value=op||'';initEl.value=init||'';commentEl.value=existing&&existing.comment?existing.comment:'';
    opEl.addEventListener('input',function(){if(!initEl.dataset.edited)initEl.value=initials(opEl.value);});
    initEl.addEventListener('input',function(){initEl.dataset.edited='1';initEl.value=initEl.value.slice(0,4).toUpperCase();});
    if(readonly){[opEl,initEl,commentEl].forEach(function(el){el.readOnly=true;});confirm.style.display='none';}
    modal.querySelector('[data-rpt06-close]').addEventListener('click',close);
    modal.addEventListener('click',function(e){if(e.target===modal)close();});
    modal.addEventListener('keydown',function(e){if(e.key==='Escape')close();});
    confirm.addEventListener('click',function(){
      var obj={operator:opEl.value.trim(),initials:initEl.value.trim().slice(0,4).toUpperCase(),comment:commentEl.value.slice(0,500),timestamp:new Date().toISOString(),sessionId:id};
      if(!obj.operator)return;
      write(obj);audit(obj);updateToolbar();close();
    });
    opEl.focus();
  }
  window.rvtReportSignoff={open:open,getSignoff:read,isSigned:function(id){return!!read(id);},clear:function(id){localStorage.removeItem(key(id));updateToolbar();}};
  window.addEventListener('beforeprint',function(){var id=sessionId();var old=document.getElementById('rvt-rpt06-pdf-warning');if(old)old.remove();if(!read(id)){var report=document.getElementById('view-report')||document.body;var warn=document.createElement('div');warn.id='rvt-rpt06-pdf-warning';warn.className='rvt-rpt06-pdf-warning';warn.textContent=R.t('gamma2b.rpt06.unsigned');report.prepend(warn);}});
  R.boot(ensureToolbar);
})();
;
/* END modules/rvt-rpt06-signoff-js.js */

/* BEGIN modules/rvt-alert02-clusters-js.js */
(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var defaults={recent:true,session:true,previous:false};
  function readState(){try{return Object.assign({},defaults,JSON.parse(localStorage.getItem('rvt-alert-clusters')||'{}'));}catch(_){return Object.assign({},defaults);}}
  function writeState(s){localStorage.setItem('rvt-alert-clusters',JSON.stringify(s));}
  function currentSession(){var S=window.S||{};return S.currentSessionId||S.sessionId||S.session_id||(S.report&&S.report.sessionId)||'';}
  function tsValue(v){if(v==null)return Date.now();if(typeof v==='number')return v;var n=Date.parse(v);return Number.isFinite(n)?n:Number(v)||Date.now();}
  function normalizeAlert(a,node){
    a=a||{};var id=a.id||a.alertId||a.alert_id||(node&&node.dataset.alertId)||('alert-'+Math.random().toString(36).slice(2));
    return{id:id,sessionId:a.sessionId||a.session_id||(node&&node.dataset.sessionId)||currentSession(),timestamp:tsValue(a.timestamp||a.ts||(node&&node.dataset.alertTs)),severity:(a.severity||a.level||(node&&node.dataset.alertSeverity)||'info').toLowerCase(),message:a.message||a.title||(node&&node.textContent)||id,node:node||null,ack:!!(a.ack||a.acknowledged)};
  }
  function alerts(){
    var S=window.S||{};var out=[];
    if(Array.isArray(S.alerts))out=out.concat(S.alerts.map(function(a){return normalizeAlert(a);}));
    if(Array.isArray(S.liveAuditLog))out=out.concat(S.liveAuditLog.map(function(a){return normalizeAlert(a);}));
    document.querySelectorAll('[data-alert-id]').forEach(function(node){var id=node.dataset.alertId;if(!out.some(function(a){return a.id===id;}))out.push(normalizeAlert({},node));});
    return out;
  }
  function bucket(a,now){if(a.sessionId&&currentSession()&&a.sessionId!==currentSession())return'previous';return now-a.timestamp<=5*60*1000?'recent':'session';}
  function worstIcon(items){var sev=items.some(function(a){return a.severity==='critical'||a.severity==='bad';})?'critical':items.some(function(a){return a.severity==='warn'||a.severity==='warning';})?'warn':'info';return sev==='critical'?'error':sev==='warn'?'warning':'info';}
  function ensureMount(){var body=document.getElementById('dvBody');if(!body){body=document.createElement('div');body.id='dvBody';document.body.appendChild(body);}return body;}
  function renderItem(a){
    var node=a.node&&a.node.cloneNode(true)||document.createElement('article');
    node.setAttribute('data-alert-id',a.id);node.setAttribute('data-alert-ts',a.timestamp);node.setAttribute('data-alert-severity',a.severity);node.setAttribute('data-session-id',a.sessionId);node.classList.add('rvt-alert-item');
    if(!node.textContent.trim())node.textContent=a.message;
    return node;
  }
  function refresh(){
    var mount=ensureMount();var state=readState();var now=Date.now();var groups={recent:[],session:[],previous:[]};
    alerts().forEach(function(a){if(window.rvtAlertSnooze&&window.rvtAlertSnooze.isSnoozed(a.id))return;groups[bucket(a,now)].push(a);});
    mount.innerHTML='';
    [['recent','gamma2b.alert.recent'],['session','gamma2b.alert.session'],['previous','gamma2b.alert.previous']].forEach(function(pair){
      var id=pair[0],items=groups[id];if(!items.length)return;
      var wrap=document.createElement('section');wrap.className='rvt-alert-cluster';wrap.id='rvt-alert02-'+id;wrap.dataset.alertCluster=id;wrap.dataset.expanded=state[id]?'true':'false';
      var btn=document.createElement('button');btn.type='button';btn.className='rvt-alert-cluster-toggle';btn.setAttribute('aria-expanded',state[id]?'true':'false');btn.innerHTML='<span><span class="material-symbols-rounded" aria-hidden="true">'+worstIcon(items)+'</span> '+R.esc(R.t(pair[1]))+'</span><span class="rvt-alert-cluster-count">'+items.length+'</span>';
      btn.addEventListener('click',function(){window.rvtAlertClusters.toggle(id);});
      var body=document.createElement('div');body.className='rvt-alert-cluster-body';
      items.forEach(function(a){body.appendChild(renderItem(a));});
      wrap.appendChild(btn);wrap.appendChild(body);mount.appendChild(wrap);
    });
    if(window.rvtAlertSnooze)window.rvtAlertSnooze.decorate&&window.rvtAlertSnooze.decorate();
    if(window.rvtAlertWaveLink)window.rvtAlertWaveLink.decorate&&window.rvtAlertWaveLink.decorate();
    return groups;
  }
  window.rvtAlertClusters={toggle:function(id){var s=readState();s[id]=!s[id];writeState(s);refresh();},getState:readState,refresh:refresh};
  window.addEventListener('rvt-alert',refresh);
  R.boot(refresh);
})();
;
/* END modules/rvt-alert02-clusters-js.js */

/* BEGIN modules/rvt-alert04-snooze-js.js */
(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  function read(){try{return JSON.parse(localStorage.getItem('rvt-alert-snooze')||'{}')||{};}catch(_){return{};}}
  function write(v){localStorage.setItem('rvt-alert-snooze',JSON.stringify(v));}
  function item(id){return document.querySelector('[data-alert-id="'+CSS.escape(id)+'"]');}
  function hide(id){var n=item(id);if(n){n.hidden=true;n.setAttribute('aria-hidden','true');}}
  function label(){
    var active=getSnoozed();var old=document.getElementById('rvt-alert04-snoozed-label');if(old)old.remove();if(!active.length)return;
    var mount=document.getElementById('dvBody')||document.body;var l=document.createElement('div');l.id='rvt-alert04-snoozed-label';l.className='rvt-alert-snoozed-label';l.textContent=R.t('gamma2b.alert.snoozed',{count:active.length});mount.appendChild(l);
  }
  function snooze(id,minutes){var all=read();all[id]=Date.now()+Math.max(1,Math.min(240,Number(minutes)||5))*60*1000;write(all);hide(id);label();if(window.rvtAlertClusters)window.rvtAlertClusters.refresh();return all[id];}
  function unsnooze(id,ended){var all=read();delete all[id];write(all);var n=item(id);if(n){n.hidden=false;n.setAttribute('aria-hidden','false');if(ended){var b=document.createElement('span');b.className='rvt-snooze-ended-badge';b.textContent=R.t('gamma2b.alert.snoozeEnded');n.appendChild(b);setTimeout(function(){b.remove();},30000);}}label();}
  function isSnoozed(id){var exp=read()[id];return Number(exp)>Date.now();}
  function getSnoozed(){var all=read();return Object.keys(all).filter(function(id){return Number(all[id])>Date.now();}).map(function(id){return{alertId:id,expiresAt:all[id]};});}
  function clearExpired(){var all=read();Object.keys(all).forEach(function(id){if(Number(all[id])<=Date.now()){delete all[id];var n=item(id);if(n){n.hidden=false;n.setAttribute('aria-hidden','false');var b=document.createElement('span');b.className='rvt-snooze-ended-badge';b.textContent=R.t('gamma2b.alert.snoozeEnded');n.appendChild(b);setTimeout(function(){b.remove();},30000);}}});write(all);label();return all;}
  function decorate(){
    clearExpired();
    document.querySelectorAll('[data-alert-id]').forEach(function(n){
      var id=n.dataset.alertId;if(isSnoozed(id))hide(id);
      if(n.querySelector('.rvt-alert-snooze-btn'))return;
      var row=n.querySelector('.rvt-alert-action-row');if(!row){row=document.createElement('div');row.className='rvt-alert-action-row';n.appendChild(row);}
      var btn=document.createElement('button');btn.type='button';btn.className='rvt-alert-action rvt-alert-snooze-btn';btn.setAttribute('aria-label',R.t('gamma2b.alert.snooze'));btn.innerHTML='<span class="material-symbols-rounded" aria-hidden="true">schedule</span>';
      var menu=document.createElement('span');menu.className='rvt-alert-snooze-menu';menu.hidden=true;
      [5,15,60].forEach(function(m){var b=document.createElement('button');b.type='button';b.textContent=R.t('gamma2b.alert.snooze'+m);b.addEventListener('click',function(){snooze(id,m);});menu.appendChild(b);});
      var custom=document.createElement('button');custom.type='button';custom.textContent=R.t('gamma2b.alert.custom');menu.appendChild(custom);
      var customBox=document.createElement('span');customBox.className='rvt-alert-snooze-custom';customBox.hidden=true;customBox.innerHTML='<input type="number" min="1" max="240" placeholder="'+R.esc(R.t('gamma2b.alert.minutes'))+'"><button type="button" class="rvt-alert-snooze-confirm">'+R.esc(R.t('gamma2b.alert.snoozeConfirm'))+'</button>';menu.appendChild(customBox);
      custom.addEventListener('click',function(){customBox.hidden=false;customBox.querySelector('input').focus();});
      customBox.querySelector('button').addEventListener('click',function(){snooze(id,customBox.querySelector('input').value);});
      btn.addEventListener('click',function(){menu.hidden=!menu.hidden;});
      row.appendChild(btn);row.appendChild(menu);
    });
    label();
  }
  window.rvtAlertSnooze={snooze:snooze,unsnooze:unsnooze,isSnoozed:isSnoozed,getSnoozed:getSnoozed,clearExpired:clearExpired,decorate:decorate};
  R.boot(decorate);
})();
;
/* END modules/rvt-alert04-snooze-js.js */

/* BEGIN modules/rvt-alert06-wave-link-js.js */
(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  function ts(v){return typeof v==='number'?v:(Date.parse(v)||Number(v)||0);}
  function range(){var S=window.S||{},w=S.waveform||S.waves||{};if(Number.isFinite(w.startTs)&&Number.isFinite(w.endTs))return{start:w.startTs,end:w.endTs};if(Array.isArray(w.timestamps)&&w.timestamps.length)return{start:Math.min.apply(null,w.timestamps.map(ts)),end:Math.max.apply(null,w.timestamps.map(ts))};return null;}
  function canJump(timestamp){var r=range();var n=ts(timestamp);return!!(r&&n>=r.start&&n<=r.end);}
  function severityFor(id){var S=window.S||{},a=(S.alerts||[]).find(function(x){return(x.id||x.alertId)===id;});return(a&&a.severity)||document.querySelector('[data-alert-id="'+CSS.escape(id||'')+'"]')?.dataset.alertSeverity||'info';}
  function color(sev){sev=String(sev||'info').toLowerCase();return sev==='critical'||sev==='bad'?'#dc2626':sev==='warn'||sev==='warning'?'#b26a00':'#2563eb';}
  function placeMarker(timestamp,severity){
    var host=document.getElementById('tab-waves')||document.body;host.style.position=host.style.position||'relative';
    var m=document.createElement('div');m.className='rvt-alert-wave-marker';m.dataset.alertWaveMarker='true';m.style.left='50%';m.style.backgroundColor=color(severity);host.appendChild(m);
    if(!window.matchMedia||!window.matchMedia('(prefers-reduced-motion: reduce)').matches){m.classList.add('rvt-pulse');setTimeout(function(){m.classList.remove('rvt-pulse');},2000);}
    return m;
  }
  function clearMarkers(){document.querySelectorAll('[data-alert-wave-marker]').forEach(function(n){n.remove();});}
  function jumpTo(alertId,timestamp){if(!canJump(timestamp))return false;if(typeof window.switchView==='function')window.switchView('live');else document.body.dataset.view='live';if(typeof window.switchTab==='function')window.switchTab('tab-waves');window.rvtAlertWaveLink.placeMarker(timestamp,severityFor(alertId));return true;}
  function decorate(){
    document.querySelectorAll('[data-alert-id]').forEach(function(n){if(n.querySelector('.rvt-alert-wave-btn'))return;var id=n.dataset.alertId,stamp=n.dataset.alertTs||Date.now();var row=n.querySelector('.rvt-alert-action-row')||document.createElement('div');row.className='rvt-alert-action-row';if(!row.parentNode)n.appendChild(row);var b=document.createElement('button');b.type='button';b.className='rvt-alert-action rvt-alert-wave-btn';b.setAttribute('aria-label',R.t('gamma2b.alert.jump'));b.title=canJump(stamp)?R.t('gamma2b.alert.jump'):R.t('gamma2b.alert.waveUnavailable');b.disabled=!canJump(stamp);b.innerHTML='<span class="material-symbols-rounded" aria-hidden="true">show_chart</span>';b.addEventListener('click',function(){jumpTo(id,stamp);});row.appendChild(b);});
  }
  window.rvtAlertWaveLink={jumpTo:jumpTo,canJump:canJump,placeMarker:placeMarker,clearMarkers:clearMarkers,decorate:decorate};
  R.boot(decorate);
})();
;
/* END modules/rvt-alert06-wave-link-js.js */

/* BEGIN modules/rvt-alert08-badge-js.js */
(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var seen={};
  function alertList(){var S=window.S||{};var src=Array.isArray(S.lastAlerts)&&S.lastAlerts.length?S.lastAlerts:(Array.isArray(S.alerts)?S.alerts:[]);return src.map(function(a,i){return{id:a.id||a.alertId||a.key||('a'+i),ack:!!(a.ack||a.acknowledged),timestamp:a.timestamp||a.ts||Date.now()};});}
  function getUnacked(){return alertList().filter(function(a){return!a.ack&&!(window.rvtAlertSnooze&&window.rvtAlertSnooze.isSnoozed(a.id));}).length;}
  function getNew(){return alertList().filter(function(a){return!a.ack&&!seen[a.id]&&!(window.rvtAlertSnooze&&window.rvtAlertSnooze.isSnoozed(a.id));}).length;}
  function fmt(n){return n>=100?'99+':String(n);}
  function bell(){return document.querySelector('.tb-alerts,#alertBtn,#bell,[data-alert-bell]');}
  function update(){
    var b=bell();if(!b)return;var un=getUnacked(),fresh=getNew();b.dataset.alertBell='true';b.setAttribute('aria-label',R.t('gamma2b.alert.badge',{unacked:un,fresh:fresh}));b.title=R.t('gamma2b.alert.badgeTip',{unacked:un,fresh:fresh});
    var badge=document.getElementById('alertsBadge')||b.querySelector('.ic-badge')||document.createElement('span');badge.id='alertsBadge';badge.className='ic-badge icon-badge rvt-alert-badge';if(!badge.parentNode)b.appendChild(badge);
    if(un===0&&fresh===0){badge.style.display='none';badge.innerHTML='';return;}
    badge.style.display='inline-grid';
    badge.innerHTML='<span class="rvt-alert-badge-segment" data-alert-badge-segment="unacked">'+fmt(un)+'</span>'+(fresh>0?'<span class="rvt-alert-badge-segment" data-alert-badge-segment="new">'+fmt(fresh)+'</span>':'');
  }
  function markOpened(){alertList().forEach(function(a){seen[a.id]=true;});update();}
  var oldOpen=window.openDrawer;if(typeof oldOpen==='function'&&!oldOpen.__rvtAlertBadge){window.openDrawer=function(){var r=oldOpen.apply(this,arguments);markOpened();return r;};window.openDrawer.__rvtAlertBadge=true;}
  window.rvtAlertBadge={getUnacked:getUnacked,getNew:getNew,markOpened:markOpened,update:update};
  R.boot(update);
  if(typeof window.renderAlerts==='function'&&!window.renderAlerts.__rvtAlertBadgeWrapped){var _origRA=window.renderAlerts;window.renderAlerts=function(){var r=_origRA.apply(this,arguments);try{update();}catch(_){};return r;};window.renderAlerts.__rvtAlertBadgeWrapped=true;}
  setInterval(update,2000);
})();
;
/* END modules/rvt-alert08-badge-js.js */

/* BEGIN modules/rvt-live04-kpi-lineage-js.js */
(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var overrides={},timer=null,pinned=null;
  function kpiEl(id){return document.querySelector('[data-kpi-id="'+CSS.escape(id)+'"],#kpi'+id);}
  function stagesFrom(v){if(Array.isArray(v))return v;if(v&&Array.isArray(v.stages))return v.stages;if(v&&Array.isArray(v.lineage))return v.lineage;if(v&&typeof v==='object')return Object.keys(v);return null;}
  function getLineage(id){var S=window.S||{};var stages=overrides[id]||stagesFrom(S.funnels&&S.funnels[id])||stagesFrom(S.live&&S.live.funnels&&S.live.funnels[id]);return stages&&stages.length?stages:[R.t('gamma2b.live.lineageUnavailable')];}
  function hide(){if(timer){clearTimeout(timer);timer=null;}document.querySelectorAll('.rvt-kpi-lineage-tooltip,.rvt-kpi-tooltip').forEach(function(n){n.remove();});}
  function goStage(stage){if(typeof window.switchTab==='function')window.switchTab('tab-funnels');var target=document.querySelector('[data-funnel-stage="'+CSS.escape(stage)+'"],#funnel-'+CSS.escape(stage));if(target)target.scrollIntoView({block:'center'});}
  function render(id,pin){hide();var el=kpiEl(id);if(!el)return;var stages=getLineage(id);var tip=document.createElement('div');tip.className='rvt-kpi-lineage-tooltip';tip.setAttribute('role','tooltip');tip.dataset.kpiLineageTooltip='true';tip.innerHTML=stages.map(function(s){var has=!!document.querySelector('[data-funnel-stage="'+CSS.escape(s)+'"],#funnel-'+CSS.escape(s));return has?'<button type="button" data-stage="'+R.esc(s)+'">'+R.esc(s)+'</button>':R.esc(s);}).join(' &rarr; ');document.body.appendChild(tip);var r=el.getBoundingClientRect();tip.style.left=Math.max(8,r.left)+'px';tip.style.top=Math.max(8,r.top-tip.offsetHeight-10)+'px';tip.querySelectorAll('[data-stage]').forEach(function(b){b.addEventListener('click',function(){goStage(b.dataset.stage);});});if(pin)pinned=tip;}
  function show(id){if(timer)clearTimeout(timer);timer=setTimeout(function(){render(id,false);},400);}
  document.addEventListener('mouseenter',function(e){var el=e.target.closest&&e.target.closest('[data-kpi-id]');if(el)show(el.dataset.kpiId);},true);
  document.addEventListener('focusin',function(e){var el=e.target.closest&&e.target.closest('[data-kpi-id]');if(el)show(el.dataset.kpiId);});
  document.addEventListener('mouseleave',function(e){if(e.target.closest&&e.target.closest('[data-kpi-id]'))hide();},true);
  document.addEventListener('keydown',function(e){if(e.key==='Escape')hide();});
  document.addEventListener('touchstart',function(e){var el=e.target.closest&&e.target.closest('[data-kpi-id]');if(!el)return;timer=setTimeout(function(){render(el.dataset.kpiId,true);},500);},{passive:true});
  document.addEventListener('touchend',function(){if(timer){clearTimeout(timer);timer=null;}},{passive:true});
  document.addEventListener('click',function(e){if(pinned&&!pinned.contains(e.target)&&!e.target.closest('[data-kpi-id]')){hide();pinned=null;}});
  window.rvtKpiLineage={show:show,hide:hide,getLineage:getLineage,setLineage:function(id,stages){overrides[id]=Array.isArray(stages)?stages.slice():[];}};
})();
;
/* END modules/rvt-live04-kpi-lineage-js.js */

/* BEGIN modules/rvt-live05-quick-tag-js.js */
(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var map={M:'motion',C:'cough',S:'speaking',B:'baseline'};
  function notes(){return document.getElementById('notes')||document.getElementById('sessionNotesText')||document.querySelector('textarea[aria-label*="notes" i], textarea');}
  function active(){var a=document.activeElement;return!(a&&(a.tagName==='TEXTAREA'||a.tagName==='INPUT'||a.isContentEditable));}
  function stamp(){return new Date().toTimeString().slice(0,8);}
  function tag(type){var n=notes();if(!n)return false;var time=stamp();var line='['+time+'] '+type;var prefix=n.value&&n.value.slice(-1)!=='\n'?'\n':'';n.value+=prefix+line;n.dispatchEvent(new Event('input',{bubbles:true}));R.toast(R.t('gamma2b.live.tagged',{type:type,time:time}));return line;}
  document.addEventListener('keydown',function(e){if(e.ctrlKey||e.metaKey||e.altKey||!active())return;var key=String(e.key||'').toUpperCase();if(map[key]){e.preventDefault();e.stopImmediatePropagation();tag(map[key]);}},true);
  window.rvtQuickTag={tag:tag,getShortcuts:function(){return Object.assign({},map);},isActive:active};
})();
;
/* END modules/rvt-live05-quick-tag-js.js */

/* BEGIN modules/rvt-live06-snapshot-fab-js.js */
(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var fab=null,lastClick=0,lastId=null;
  function count(){var S=window.S||{};return Array.isArray(S.snaps)?S.snaps.length:0;}
  function updateBadge(){if(!fab)return;var b=fab.querySelector('.rvt-live06-fab-badge');if(!b)return;var n=count();b.textContent=String(n);b.style.display=n>0?'grid':'none';}
  function visible(){return document.body.dataset.view==='live';}
  function show(){ensure();fab.dataset.visible='true';}
  function hide(){if(fab)fab.dataset.visible='false';}
  function updateVis(){if(visible())show();else hide();}
  function capture(){
    window.S=window.S||{};window.S.snaps=Array.isArray(window.S.snaps)?window.S.snaps:[];
    var before=window.S.snaps.length;var id=null;
    if(typeof window.snapshotNow==='function')id=window.snapshotNow();
    else if(typeof window.doSnapshot==='function')id=window.doSnapshot();
    if(window.S.snaps.length===before){id=id||('snap-'+(before+1));window.S.snaps.push({id:id,timestamp:new Date().toISOString(),kpis:{hr:window.S.live&&window.S.live.hr,rr:window.S.live&&window.S.live.rr,range_cm:window.S.live&&window.S.live.range_cm},waveform:(window.S.waveform&&window.S.waveform.samples)||[]});}
    lastId=id||(window.S.snaps[window.S.snaps.length-1]&&window.S.snaps[window.S.snaps.length-1].id);
    updateBadge();
    if(fab&&!window.matchMedia('(prefers-reduced-motion: reduce)').matches){fab.classList.add('rvt-captured');setTimeout(function(){fab.classList.remove('rvt-captured');},220);}
    return lastId;
  }
  function labelPopover(){ensure();var old=document.getElementById('rvt-live06-label-popover');if(old)old.remove();var pop=document.createElement('div');pop.id='rvt-live06-label-popover';pop.className='rvt-live06-label-popover';pop.innerHTML='<input id="rvt-live06-label-input" maxlength="60" aria-label="'+R.esc(R.t('gamma2b.live.snapshotLabel'))+'"><button type="button">'+R.esc(R.t('gamma2b.live.saveLabel'))+'</button>';document.body.appendChild(pop);pop.querySelector('button').addEventListener('click',function(){var snap=(window.S.snaps||[]).find(function(s){return s.id===lastId;});if(snap)snap.label=pop.querySelector('input').value.slice(0,60);pop.remove();});pop.querySelector('input').focus();}
  function captureWithLabel(){if(typeof window.__labelCaptureCount==='number')window.__labelCaptureCount+=1;var id=capture();labelPopover();return id;}
  function attachFab(node){
    if(!node||node.__rvtLive06Attached)return node;
    node.__rvtLive06Attached=true;
    node.dataset.snapFab='true';
    node.classList.add('rvt-live06-fab');
    node.setAttribute('aria-label',R.t('gamma2b.live.snapshot'));
    if(!node.querySelector('.rvt-live06-fab-badge'))node.insertAdjacentHTML('beforeend','<span class="rvt-live06-fab-badge">0</span>');
    node.addEventListener('click',function(){var now=Date.now();if(now-lastClick<=300){lastClick=0;window.rvtSnapFAB.captureWithLabel();}else{lastClick=now;window.rvtSnapFAB.capture();}});
    node.addEventListener('dblclick',function(e){e.preventDefault();window.rvtSnapFAB.captureWithLabel();});
    node.addEventListener('keydown',function(e){if(e.key==='Enter'){e.preventDefault();if(e.shiftKey)window.rvtSnapFAB.captureWithLabel();else window.rvtSnapFAB.capture();}});
    return node;
  }
  function ensure(){if(fab)return fab;fab=document.getElementById('rvt-live06-fab')||document.getElementById('rvtSnapFab');if(!fab){fab=document.createElement('button');fab.type='button';fab.id='rvt-live06-fab';fab.innerHTML='<span class="material-symbols-rounded" aria-hidden="true">photo_camera</span>';document.body.appendChild(fab);}attachFab(fab);updateBadge();return fab;}
  document.addEventListener('keydown',function(e){var a=document.activeElement;if(e.key&&e.key.toUpperCase()==='P'&&!e.ctrlKey&&!e.metaKey&&!e.altKey&&visible()&&!(a&&(a.tagName==='TEXTAREA'||a.tagName==='INPUT'||a.isContentEditable))){e.preventDefault();e.stopImmediatePropagation();window.rvtSnapFAB.capture();}},true);
  document.addEventListener('click',function(e){var node=e.target&&e.target.closest&&e.target.closest('#rvt-live06-fab,#rvtSnapFab,[data-snap-fab]');if(!node)return;attachFab(node);},true);
  document.addEventListener('dblclick',function(e){var node=e.target&&e.target.closest&&e.target.closest('#rvt-live06-fab,#rvtSnapFab,[data-snap-fab]');if(!node)return;e.preventDefault();if(window.rvtSnapFAB)window.rvtSnapFAB.captureWithLabel();},true);
  var oldSwitch=window.switchView;if(typeof oldSwitch==='function'&&!oldSwitch.__rvtSnapFab){window.switchView=function(v){var r=oldSwitch.apply(this,arguments);setTimeout(updateVis,0);return r;};window.switchView.__rvtSnapFab=true;}
  window.rvtSnapFAB={capture:capture,captureWithLabel:captureWithLabel,getCount:count,show:show,hide:hide};
  R.boot(function(){ensure();updateVis();});
})();
;
/* END modules/rvt-live06-snapshot-fab-js.js */

/* BEGIN modules/rvt-live07-pulse-confidence-js.js */
(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var currentPqi=null,paused=false;
  function reduced(){return window.matchMedia&&window.matchMedia('(prefers-reduced-motion: reduce)').matches;}
  function kpis(){return document.querySelectorAll('.kpi,[data-kpi-id]');}
  function setAmplitude(r){r=Math.max(0,Math.min(1,Number(r)||0));document.documentElement.style.setProperty('--rvt-pulse-amplitude',String(r));document.body.style.setProperty('--rvt-pulse-amplitude',String(r));}
  function pause(){paused=true;document.body.classList.add('rvt-pulse-paused');kpis().forEach(function(k){k.classList.add('rvt-pulse-paused');});}
  function resume(){paused=false;document.body.classList.remove('rvt-pulse-paused');kpis().forEach(function(k){k.classList.remove('rvt-pulse-paused');});}
  function readPqi(){var S=window.S||{};var p=S.live&&S.live.pqi;if(p==null)p=S.lastPayload&&(S.lastPayload.pqi||S.lastPayload.pqi_heart||S.lastPayload.heart_pqi);var n=Number(p);return Number.isFinite(n)?n:currentPqi;}
  function apply(){currentPqi=readPqi();if(reduced())return currentPqi;if(currentPqi==null)return currentPqi;if(currentPqi<.2){setAmplitude(0);pause();}else{resume();setAmplitude(currentPqi<.4?.3:1);}return currentPqi;}
  window.addEventListener('rvt-live-update',function(e){if(e&&e.detail){window.S=window.S||{};window.S.live=Object.assign({},window.S.live||{},e.detail);}apply();});
  window.rvtPulseConf={setAmplitude:setAmplitude,pause:pause,resume:resume,getCurrentPqi:function(){return apply();}};
  R.boot(apply);
})();
;
/* END modules/rvt-live07-pulse-confidence-js.js */

/* BEGIN modules/rvt-setup01-js.js */
(function(){
  'use strict';
  var STRINGS={
    'gd.newSubject':'+ New subject','gd.sessionQueue':'Session queue','gd.startNext':'Start next','gd.add':'Add','gd.remove':'Remove','gd.importCsv':'Import CSV','gd.queueLimit':'Maximum 20 subjects per queue','gd.queueProgress':'{complete} of {total} complete · {pass} pass · {warn} warn','gd.shiftSummary':'Generate shift summary','gd.demo':'DEMO','gd.demoMode':'Demo mode','gd.demoConfirm':'Demo mode injects synthetic data. Real radar input will be ignored. Continue?','gd.keepDemo':'Keep demo mode on reload','gd.compare':'Compare sessions','gd.select':'Select','gd.exportSelected':'Export selected ({count})','gd.zipMissing':'ZIP library not available - downloading individually','gd.rerunFailed':'Re-run failed','gd.rerunDone':'{done} of {total} re-checks complete · {pass} passed, {fail} failed','gd.recentSetups':'Recent setups','gd.clearFilters':'Clear filters','gd.mySessions':'My sessions only','gd.measure':'Δ {delta} ms · {bpm} BPM','gd.previousMinute':'Show previous minute','gd.auditVerify':'Verify audit integrity','gd.auditPass':'Audit log intact - {count} entries verified','gd.auditFail':'Chain broken at {timestamp} - entries after this point may have been modified','gd.perfExport':'Export perf log','gd.audioProfile':'Audio profile: {name}','gd.recent':'Recent','gd.operatorSwitch':'Operator switch','gd.addOperator':'Add new operator','gd.sessionBriefCopied':'Session brief copied','gd.copyBrief':'Copy session brief','gd.sweetInside':'Radar preview - subject at {range} cm, inside sweet zone','gd.sweetOutside':'Radar preview - subject at {range} cm, outside sweet zone','gd.replay':'Replay','gd.preview':'Preview - not recording','gd.helpThanks':'Thanks for the feedback','gd.pinAlert':'Pin alert','gd.replacePin':'Replace oldest pinned alert with this one?','gd.paused':'Paused {time}','gd.toastPos':'Toast position','gd.fieldLayout':'Field layout','gd.rotate':'Rotate device for field layout','gd.dryRun':'Preview','gd.step':'Step {step}/{total}: {label}','gd.chainAbort':'Chain aborted at step {step}: {reason}','gd.rtl':'Right-to-left layout','gd.modified':'Modified from default ({value})','gd.resetSetting':'Reset this setting'};
  function extendStrings(){window.RVT_STRINGS=window.RVT_STRINGS||{en:{}};window.RVT_STRINGS.en=window.RVT_STRINGS.en||{};Object.keys(STRINGS).forEach(function(k){if(!(k in window.RVT_STRINGS.en))window.RVT_STRINGS.en[k]=STRINGS[k];});}
  function t(key,vars){extendStrings();var s=typeof window.rvtT==='function'?window.rvtT(key,vars||{}):(window.RVT_STRINGS.en[key]||key);Object.keys(vars||{}).forEach(function(k){s=String(s).replace(new RegExp('\\{'+k+'\\}','g'),String(vars[k]));});return s;}
  function manifestSpec(){return {
    'rvt-subject-history':{kind:'json',defaultValue:[],validate:Array.isArray},
    'rvt-session-queue':{kind:'json',defaultValue:[],validate:Array.isArray},
    'rvt-queue-panel-open':{kind:'scalar',defaultValue:'true',validate:function(v){return v==='true'||v==='false';}},
    'rvt-shift-summaries':{kind:'json',defaultValue:[],validate:Array.isArray},
    'rvt-demo-pinned':{kind:'scalar',defaultValue:'false',validate:function(v){return v==='true'||v==='false';}},
    'rvt-funnel-ghost':{kind:'scalar',defaultValue:'false',validate:function(v){return v==='true'||v==='false';}},
    'rvt-recent-setups':{kind:'json',defaultValue:[],validate:Array.isArray},
    'rvt-sessions-filter':{kind:'json',defaultValue:{readiness:[],myOnly:false},validate:function(v){return !!v&&Array.isArray(v.readiness)&&typeof v.myOnly==='boolean';}},
    'rvt-cmd-recent':{kind:'json',defaultValue:[],validate:Array.isArray},
    'rvt-help-feedback':{kind:'json',defaultValue:{},validate:function(v){return v!==null&&typeof v==='object';}},
    'rvt-alert-pins':{kind:'json',defaultValue:[],validate:function(v){return Array.isArray(v)&&v.length<=3;}},
    'rvt-toast-position':{kind:'scalar',defaultValue:'BR',validate:function(v){return ['TL','TR','BL','BR'].indexOf(v)>=0;}},
    'rvt-density-override-until':{kind:'scalar',defaultValue:'',validate:function(v){return typeof v==='string';}},
    'rvt-kpi-layout':{kind:'json',defaultValue:{},validate:function(v){return v!==null&&typeof v==='object';}}
  };}
  function registerManifest(){var m=window.rvtStorage&&window.rvtStorage.manifest;if(!m)return;var spec=manifestSpec();Object.keys(spec).forEach(function(k){m[k]=m[k]||spec[k];if(k==='rvt-alert-pins')m[k]=spec[k];});}
  function parseJson(key,fallback,validate){try{var raw=localStorage.getItem(key);if(raw==null)return clone(fallback);var v=JSON.parse(raw);return !validate||validate(v)?v:clone(fallback);}catch(_){return clone(fallback);}}
  function setJson(key,v){try{localStorage.setItem(key,JSON.stringify(v));}catch(_){}return v;}
  function getScalar(key,fallback){try{var v=localStorage.getItem(key);return v==null?fallback:v;}catch(_){return fallback;}}
  function setScalar(key,v){try{localStorage.setItem(key,String(v));}catch(_){}return String(v);}
  function clone(v){return JSON.parse(JSON.stringify(v));}
  function nowIso(){return new Date().toISOString();}
  function today(){return new Date().toISOString().slice(0,10);}
  function stampFile(){var d=new Date();function p(n){return String(n).padStart(2,'0');}return d.getFullYear()+'-'+p(d.getMonth()+1)+'-'+p(d.getDate())+'_'+p(d.getHours())+p(d.getMinutes())+p(d.getSeconds());}
  function toast(msg){if(typeof window.toast==='function')window.toast(msg);else if(typeof window.showToast==='function')window.showToast(msg);else{var n=document.createElement('div');n.className='toast rvt-gd-toast';n.textContent=msg;n.setAttribute('role','status');document.body.appendChild(n);setTimeout(function(){n.remove();},3000);}}
  function download(name,mime,content){var blob=content instanceof Blob?content:new Blob([content],{type:mime||'text/plain'});var a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=name;document.body.appendChild(a);a.click();setTimeout(function(){URL.revokeObjectURL(a.href);a.remove();},0);return name;}
  function csvCell(v){var s=v==null?'':String(v);return /[",\n]/.test(s)?'"'+s.replace(/"/g,'""')+'"':s;}
  function toCsv(rows){return rows.map(function(r){return r.map(csvCell).join(',');}).join('\n');}
  function ensure(id,tag,parent){var n=document.getElementById(id);if(!n){n=document.createElement(tag||'div');n.id=id;(parent||document.body).appendChild(n);}return n;}
  function currentOperator(){var S=window.S||{};return (S.settings&&(S.settings.operator||S.settings.operatorName))||S.operator||localStorage.getItem('rvt-operator-name')||'Operator';}
  function audit(message,extra){window.S=window.S||{};var entry=Object.assign({timestamp:nowIso(),message:message},extra||{});var paths=[window.S.auditLog,window.S.audit,window.S.report&&window.S.report.auditLog].filter(Array.isArray);if(!paths.length){window.S.auditLog=[];paths=[window.S.auditLog];}paths.forEach(function(a){a.push(Object.assign({},entry));});try{var stored=parseJson('rvt-audit-log',[],Array.isArray);stored.push(entry);setJson('rvt-audit-log',stored);}catch(_){}return entry;}
  function getSessions(){var S=window.S||{};var a=S.sessions||S.pastSessions||S.completedSessions||[];var q=parseJson('rvt-session-queue',[],Array.isArray);if(!a.length) a=q.filter(function(e){return e.status==='complete';}).map(function(e,i){return Object.assign({sessionId:e.sessionId||('session-'+(i+1)),id:e.sessionId||('session-'+(i+1)),verdict:e.verdict||'pass',score:e.score||0,duration:e.duration||e.durationSec||0,operator:e.operator||currentOperator(),subjectId:e.subjectId},e);});return a;}
  function metricsForSession(s){return {score:Number(s.score||s.readiness||0),duration:Number(s.duration||s.durationSec||s.duration_s||0),hr:Number(s.hr||s.hrRmse||s.rmseHr||0),rr:Number(s.rr||s.rrRmse||s.rmseRr||0),pqi:Number(s.pqi||s.meanPqi||0)};}
  function boot(fn){if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',fn,{once:true});else setTimeout(fn,0);}
  extendStrings();registerManifest();
  window.rvtGammaDelta={t:t,registerManifest:registerManifest,parseJson:parseJson,setJson:setJson,getScalar:getScalar,setScalar:setScalar,clone:clone,nowIso:nowIso,today:today,stampFile:stampFile,toast:toast,download:download,toCsv:toCsv,ensure:ensure,currentOperator:currentOperator,audit:audit,getSessions:getSessions,metricsForSession:metricsForSession,boot:boot};
  boot(registerManifest);

  var activeIndex=-1,drop=null,input=null;
  function allSubjects(){return parseJson('rvt-subject-history',[],Array.isArray).sort(function(a,b){return String(b.lastSession||'').localeCompare(String(a.lastSession||''));});}
  function saveSubjects(list){list=list.filter(function(e){return e&&e.id;}).sort(function(a,b){return String(b.lastSession||'').localeCompare(String(a.lastSession||''));}).slice(0,200);return setJson('rvt-subject-history',list);}
  function search(query){query=String(query||'').toLowerCase();return allSubjects().filter(function(e){return !query||String(e.id||'').toLowerCase().indexOf(query)>=0;}).slice(0,8);}
  function addSubject(id,sessionDate){id=String(id||'').trim();if(!id)return allSubjects();var list=allSubjects().filter(function(e){return String(e.id).toLowerCase()!==id.toLowerCase();});list.unshift({id:id,lastSession:sessionDate||nowIso()});return saveSubjects(list);}
  function clear(){return setJson('rvt-subject-history',[]);}
  function findField(){return document.querySelector('#subjectId,#subject-id,input[name="subject_id"],input[name="subjectId"],[data-rvt-subject-id]');}
  function closeDrop(){if(drop)drop.remove();drop=null;activeIndex=-1;}
  function choose(id){if(input){input.value=id;input.dispatchEvent(new Event('input',{bubbles:true}));input.dispatchEvent(new Event('change',{bubbles:true}));}closeDrop();}
  function renderDrop(){if(!input)return;var q=input.value||'',items=search(q),exact=allSubjects().some(function(e){return String(e.id).toLowerCase()===q.toLowerCase();});if(!drop){drop=document.createElement('div');drop.id='rvt-setup01-dropdown';drop.className='rvt-subject-autocomplete rvt-gd-panel';drop.setAttribute('role','listbox');document.body.appendChild(drop);}var r=input.getBoundingClientRect();drop.style.insetInlineStart=(r.left+window.scrollX)+'px';drop.style.insetBlockStart=(r.bottom+window.scrollY+4)+'px';drop.innerHTML='';items.forEach(function(e,i){var b=document.createElement('button');b.type='button';b.id='rvt-setup01-option-'+i;b.setAttribute('role','option');b.setAttribute('aria-selected',i===activeIndex?'true':'false');b.innerHTML='<span></span><small></small>';b.firstChild.textContent=e.id;b.lastChild.textContent=e.lastSession?String(e.lastSession).slice(0,10):'';b.addEventListener('mousedown',function(ev){ev.preventDefault();choose(e.id);});drop.appendChild(b);});if(!exact){var nb=document.createElement('button');nb.type='button';nb.id='rvt-setup01-new';nb.textContent=t('gd.newSubject');nb.addEventListener('mousedown',function(ev){ev.preventDefault();choose(q);});drop.appendChild(nb);}if(!items.length&&exact)closeDrop();}
  function attach(){input=findField();if(!input||input.__rvtSetup01)return;input.__rvtSetup01=true;input.setAttribute('autocomplete','off');input.addEventListener('input',function(){activeIndex=-1;renderDrop();});input.addEventListener('focus',renderDrop);input.addEventListener('blur',function(){setTimeout(closeDrop,120);});input.addEventListener('keydown',function(e){var count=search(input.value).length;if(!drop&&['ArrowDown','ArrowUp'].indexOf(e.key)>=0)renderDrop();if(e.key==='ArrowDown'){e.preventDefault();activeIndex=Math.min(count-1,activeIndex+1);renderDrop();}else if(e.key==='ArrowUp'){e.preventDefault();activeIndex=Math.max(0,activeIndex-1);renderDrop();}else if(e.key==='Enter'&&drop&&activeIndex>=0){e.preventDefault();var m=search(input.value)[activeIndex];if(m)choose(m.id);}});}
  window.rvtSetupAutocomplete={search:search,add:addSubject,getAll:allSubjects,clear:clear};
  window.rvtSetup01=window.rvtSetupAutocomplete;
  boot(attach);
})();
;
/* END modules/rvt-setup01-js.js */

/* BEGIN modules/rvt-work01-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var dragging=-1;
  function normalize(e){e=e||{};return {subjectId:String(e.subjectId||e.subject_id||'').trim(),profileName:String(e.profileName||e.profile_name||'').trim(),notes:String(e.notes||''),status:['pending','active','complete','skipped'].indexOf(e.status)>=0?e.status:'pending',sessionId:e.sessionId||e.session_id||'',verdict:e.verdict||'',score:e.score||0,operator:e.operator||R.currentOperator(),duration:e.duration||0};}
  function getQueue(){return R.parseJson('rvt-session-queue',[],Array.isArray).map(normalize);}
  function save(q){R.setJson('rvt-session-queue',q.map(normalize).slice(0,20));render();return getQueue();}
  function add(entry){var q=getQueue();if(q.length>=20){R.toast(R.t('gd.queueLimit'));return q;}q.push(normalize(entry));if(window.rvtSetupAutocomplete&&entry&&entry.subjectId)window.rvtSetupAutocomplete.add(entry.subjectId,R.nowIso());return save(q);}
  function remove(i){var q=getQueue();q.splice(Number(i),1);return save(q);}
  function reorder(from,to){var q=getQueue();from=Number(from);to=Number(to);if(from<0||to<0||from>=q.length||to>=q.length)return q;var item=q.splice(from,1)[0];q.splice(to,0,item);return save(q);}
  function startNext(){var q=getQueue(),idx=q.findIndex(function(e){return e.status==='pending';});if(idx<0)return null;q.forEach(function(e){if(e.status==='active')e.status='skipped';});q[idx].status='active';save(q);window.S=window.S||{};window.S.setup=Object.assign({},window.S.setup||{},q[idx]);var f=document.querySelector('#subjectId,#subject-id,input[name="subject_id"],input[name="subjectId"]');if(f){f.value=q[idx].subjectId;f.dispatchEvent(new Event('change',{bubbles:true}));}if(typeof window.switchView==='function')window.switchView('setup');document.body.dataset.view='setup';return q[idx];}
  function splitCsv(line){var out=[],cur='',q=false;for(var i=0;i<line.length;i++){var c=line[i];if(c==='"'&&line[i+1]==='"'){cur+='"';i++;}else if(c==='"')q=!q;else if(c===','&&!q){out.push(cur);cur='';}else cur+=c;}out.push(cur);return out;}
  function importCsv(text){var lines=String(text||'').trim().split(/\r?\n/).filter(Boolean);if(!lines.length)return {imported:0,errors:[{row:0,error:'empty'}]};var head=splitCsv(lines.shift()).map(function(h){return h.trim().toLowerCase();});var si=head.indexOf('subject_id'),pi=head.indexOf('profile_name'),ni=head.indexOf('notes');var errors=[],count=0;lines.forEach(function(line,ix){var p=splitCsv(line);if(si<0||pi<0||!p[si]||!p[pi]){errors.push({row:ix+2,error:'missing required fields'});return;}var before=getQueue().length;add({subjectId:p[si],profileName:p[pi],notes:ni>=0?p[ni]:''});if(getQueue().length>before)count++;});return {imported:count,errors:errors};}
  function getProgress(){var q=getQueue(),complete=q.filter(function(e){return e.status==='complete';});return {total:q.length,complete:complete.length,pass:complete.filter(function(e){return String(e.verdict).toLowerCase()==='pass';}).length,warn:complete.filter(function(e){return /warn|fail/.test(String(e.verdict).toLowerCase());}).length};}
  function render(){var home=document.querySelector('#view-home,[data-view="home"],main')||document.body;var panel=R.ensure('rvt-work01-panel','section',home);panel.className='rvt-gd-panel';var open=R.getScalar('rvt-queue-panel-open','true')==='true',p=getProgress();panel.innerHTML='<div class="rvt-gd-toolbar"><button id="rvt-work01-toggle" class="rvt-gd-button" aria-expanded="'+open+'">'+R.t('gd.sessionQueue')+'</button><button id="rvt-work01-start" class="rvt-gd-button">'+R.t('gd.startNext')+'</button><input id="rvt-work01-csv" type="file" accept=".csv,text/csv" class="rvt-gd-hidden"></div><div id="rvt-work01-body" '+(open?'':'class="rvt-gd-hidden"')+'><div class="rvt-session-queue-list"></div><p id="rvt-work01-progress"></p></div>';var list=panel.querySelector('.rvt-session-queue-list');getQueue().forEach(function(e,i){var row=document.createElement('div');row.className='rvt-session-queue-item';row.tabIndex=0;row.draggable=true;row.dataset.index=i;row.innerHTML='<span><b></b><small></small></span><button class="rvt-gd-button" aria-label="'+R.t('gd.remove')+'">×</button>';row.querySelector('b').textContent=e.subjectId||'Subject';row.querySelector('small').textContent=' '+(e.profileName||'')+' · '+e.status;row.addEventListener('dragstart',function(){dragging=i;});row.addEventListener('drop',function(ev){ev.preventDefault();reorder(dragging,i);});row.addEventListener('dragover',function(ev){ev.preventDefault();});row.addEventListener('keydown',function(ev){if(ev.ctrlKey&&ev.key==='ArrowUp'){ev.preventDefault();reorder(i,Math.max(0,i-1));}if(ev.ctrlKey&&ev.key==='ArrowDown'){ev.preventDefault();reorder(i,Math.min(getQueue().length-1,i+1));}});row.querySelector('button').addEventListener('click',function(){remove(i);});list.appendChild(row);});panel.querySelector('#rvt-work01-progress').textContent=R.t('gd.queueProgress',p);panel.querySelector('#rvt-work01-toggle').onclick=function(){R.setScalar('rvt-queue-panel-open',open?'false':'true');render();};panel.querySelector('#rvt-work01-start').onclick=startNext;}
  window.rvtSessionQueue={getQueue:getQueue,add:add,remove:remove,reorder:reorder,startNext:startNext,importCsv:importCsv,getProgress:getProgress};
  window.rvtWork01=window.rvtSessionQueue;R.boot(render);
})();
;
/* END modules/rvt-work01-js.js */

/* BEGIN modules/rvt-work02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function buildRows(){return (window.rvtSessionQueue?window.rvtSessionQueue.getQueue():[]).filter(function(e){return e.status==='complete';}).map(function(e,i){return {sessionId:e.sessionId||('session-'+(i+1)),subjectId:e.subjectId,verdict:e.verdict||'pass',score:Number(e.score||0),duration:Number(e.duration||0),operator:e.operator||R.currentOperator()};});}
  function generate(){var rows=buildRows(),total=rows.length,pass=rows.filter(function(r){return r.verdict==='pass';}).length,flag=rows.filter(function(r){return /warn|fail/.test(r.verdict);}).length,dur=rows.reduce(function(a,r){return a+Number(r.duration||0);},0),ops=Array.from(new Set(rows.map(function(r){return r.operator;}))).filter(Boolean);var summary={date:R.today(),sessionIds:rows.map(function(r){return r.sessionId;}),rows:rows,passRate:total?Math.round(pass/total*100):0,flagged:flag,totalRecordingTime:dur,operators:ops,generated:R.nowIso()};var h=R.parseJson('rvt-shift-summaries',[],Array.isArray);h.unshift({date:summary.date,sessionIds:summary.sessionIds,passRate:summary.passRate,generated:summary.generated});R.setJson('rvt-shift-summaries',h.slice(0,30));window.S=window.S||{};window.S.shiftSummary=summary;return summary;}
  function exportCsv(){var s=generate(),rows=[['session_id','subject_id','verdict','score','duration','operator']].concat(s.rows.map(function(r){return [r.sessionId,r.subjectId,r.verdict,r.score,r.duration,r.operator];}));rows.push(['TOTALS','',s.flagged?'warn':'pass',s.passRate,s.totalRecordingTime,s.operators.join(';')]);return R.download('shift_summary_'+R.today()+'.csv','text/csv',R.toCsv(rows));}
  function exportPdf(){var s=generate();return R.download('shift_summary_'+R.today()+'.pdf','application/pdf','Shift summary '+s.date+'\nPass rate '+s.passRate+'%');}
  function render(){var home=document.querySelector('#view-home,[data-view="home"],main')||document.body;var btn=R.ensure('rvt-work02-generate','button',home);btn.type='button';btn.className='rvt-gd-button';btn.textContent=R.t('gd.shiftSummary');btn.disabled=buildRows().length<2;btn.onclick=generate;}
  window.rvtShiftSummary={generate:generate,exportPdf:exportPdf,exportCsv:exportCsv,getHistory:function(){return R.parseJson('rvt-shift-summaries',[],Array.isArray);},buildRows:buildRows};
  window.rvtWork02=window.rvtShiftSummary;R.boot(render);
})();
;
/* END modules/rvt-work02-js.js */

/* BEGIN modules/rvt-obs02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var active=false,scenario='Normal',custom=null,idx=0,timer=null;
  var scenarios={Normal:[{timestamp_offset:0,hr:72,rr:14,pqi:.82,range:62,motion:false},{timestamp_offset:1,hr:73,rr:15,pqi:.8,range:63,motion:false}],Tachycardia:[{timestamp_offset:0,hr:118,rr:18,pqi:.72,range:61,motion:false}],Bradycardia:[{timestamp_offset:0,hr:46,rr:12,pqi:.76,range:64,motion:false}],Apnea:[{timestamp_offset:0,hr:78,rr:0,pqi:.58,range:66,motion:false}],'Motion Artifact':[{timestamp_offset:0,hr:95,rr:24,pqi:.18,range:71,motion:true}],'Sensor Dropout':[{timestamp_offset:0,hr:null,rr:null,pqi:0,range:null,motion:false}],Mixed:[{timestamp_offset:0,hr:74,rr:15,pqi:.81,range:62,motion:false},{timestamp_offset:1,hr:126,rr:23,pqi:.5,range:68,motion:true},{timestamp_offset:2,hr:null,rr:null,pqi:0,range:null,motion:false}]};
  function validate(frames){return Array.isArray(frames)&&frames.length&&frames.every(function(f){return f&&typeof f==='object'&&'timestamp_offset'in f&&'hr'in f&&'rr'in f&&'pqi'in f&&'range'in f&&'motion'in f;});}
  function frames(){return scenario==='custom'?custom:scenarios[scenario]||scenarios.Normal;}
  function watermark(on){var ids=['rvt-demo-watermark','rvt-demo-watermark-live','rvt-demo-watermark-report','rvt-demo-watermark-snaps','rvt-demo-watermark-waves'];ids.forEach(function(id,i){var n=document.getElementById(id);if(on){n=R.ensure(id,'div');n.className='rvt-demo-watermark';n.textContent=R.t('gd.demo');if(i)n.style.insetBlockStart=(12+i*28)+'px';}else if(n)n.remove();});}
  function route(frame){var payload={timestamp:Date.now(),hr:frame.hr,rr:frame.rr,pqi:frame.pqi,range_cm:frame.range,motion:frame.motion,demo:true};if(typeof window.normalizeLivePayload==='function')payload=window.normalizeLivePayload(payload);if(typeof window.validateLivePayloadShape==='function')window.validateLivePayloadShape(payload);window.S=window.S||{};window.S.live=Object.assign({},window.S.live||{},payload);window.dispatchEvent(new CustomEvent('rvt-live-update',{detail:payload}));return payload;}
  function tick(){if(!active)return null;var f=frames();if(!f||!f.length)return null;var out=route(f[idx%f.length]);idx++;return out;}
  function loop(){if(!active)return;tick();timer=setTimeout(loop,1000);}
  function enable(name){if(!active&&R.getScalar('rvt-demo-pinned','false')!=='true'&&window.confirm&&!window.confirm(R.t('gd.demoConfirm')))return false;scenario=name||scenario||'Normal';active=true;idx=0;window.S=window.S||{};window.S.demoMode=true;watermark(true);if(timer)clearTimeout(timer);loop();return true;}
  function disable(){active=false;window.S=window.S||{};window.S.demoMode=false;watermark(false);if(timer)clearTimeout(timer);timer=null;}
  function loadCustom(framesArg){if(!validate(framesArg))throw new Error('invalid custom scenario');custom=framesArg;scenario='custom';return true;}
  if(R.getScalar('rvt-demo-pinned','false')==='true')R.boot(function(){enable(scenario);});
  window.rvtDemoMode={enable:enable,disable:disable,isActive:function(){return active;},getScenario:function(){return scenario;},loadCustom:loadCustom,tick:tick};
  window.rvtObs02=window.rvtDemoMode;
})();
;
/* END modules/rvt-obs02-js.js */

/* BEGIN modules/rvt-rpt05-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var selected=[];
  function open(ids){selected=(ids||[]).slice(0,5);render();return getRows();}
  function addSession(id){if(selected.indexOf(id)<0&&selected.length<5)selected.push(id);render();return selected.slice();}
  function removeSession(id){selected=selected.filter(function(x){return x!==id;});render();return selected.slice();}
  function sessions(){var all=R.getSessions();return selected.length?selected.map(function(id){return all.find(function(s){return (s.id||s.sessionId)===id;})||{id:id,sessionId:id};}):all.slice(0,5);}
  function median(a){a=a.slice().sort(function(x,y){return x-y;});return a.length?a[Math.floor(a.length/2)]:0;}
  function getRows(){var ss=sessions(),metrics=['score','duration','hr','rr','pqi'];return metrics.map(function(m){var vals=ss.map(function(s){return R.metricsForSession(s)[m];}),med=median(vals);return {metric:m,values:vals,delta:vals.map(function(v){return v-med;})};});}
  function exportCsv(){var ss=sessions(),rows=[['metric'].concat(ss.map(function(s){return s.sessionId||s.id;})).concat(['delta'])];getRows().forEach(function(r){rows.push([r.metric].concat(r.values).concat([r.delta.join(';')]));});return R.download('session_compare_'+R.today()+'.csv','text/csv',R.toCsv(rows));}
  function render(){var root=R.ensure('rvt-rpt05-compare','section');root.className='rvt-gd-panel';var ss=sessions(),rows=getRows();root.innerHTML='<h3>'+R.t('gd.compare')+'</h3><table class="rvt-compare-table"><tbody></tbody></table>';var tb=root.querySelector('tbody');rows.forEach(function(r){var tr=document.createElement('tr');tr.innerHTML='<th>'+r.metric+'</th>'+r.values.map(function(v){return '<td class="rvt-compare-heat">'+v+'</td>';}).join('')+'<td>'+r.delta.join(', ')+'</td>';tb.appendChild(tr);});}
  window.rvtMultiCompare={open:open,addSession:addSession,removeSession:removeSession,exportCsv:exportCsv,getRows:getRows};
  window.rvtRpt05=window.rvtMultiCompare;R.boot(function(){var b=R.ensure('rvt-rpt05-button','button');b.className='rvt-gd-button';b.textContent=R.t('gd.compare');b.onclick=function(){open((R.getSessions().slice(0,3).map(function(s){return s.sessionId||s.id;})));};});
})();
;
/* END modules/rvt-rpt05-js.js */

/* BEGIN modules/rvt-snap04-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var selectMode=false,selected=new Set(),cancelled=false;
  function idFromNode(n){return n&&(n.dataset.snapshotId||n.dataset.snapId||n.id);}
  function syncFromDom(){document.querySelectorAll('.rvt-snap-select:checked,[data-snap-select]:checked,.rvt-snap-select[checked],[data-snap-select][checked]').forEach(function(c){selected.add(c.value||idFromNode(c.closest('[data-snapshot-id],[data-snap-id]')));});}
  function getSelected(){syncFromDom();return Array.from(selected).filter(Boolean);}
  function selectAll(){selectMode=true;(window.S&&window.S.snaps||[]).forEach(function(s){selected.add(s.id);});document.querySelectorAll('[data-snapshot-id],[data-snap-id]').forEach(function(n){var id=idFromNode(n);if(id)selected.add(id);});render();return getSelected();}
  function clearSelection(){selected.clear();document.querySelectorAll('.rvt-snap-select,[data-snap-select]').forEach(function(c){c.checked=false;});render();return [];}
  function snapshots(){var all=(window.S&&window.S.snaps)||R.parseJson('rvt-snaps',[],Array.isArray);var ids=getSelected();return all.filter(function(s){return ids.indexOf(s.id)>=0;});}
  function exportSelected(){var snaps=snapshots(),ids=getSelected();if(!ids.length)return null;if(window.JSZip){var zip=new window.JSZip();snaps.forEach(function(s){zip.file((s.label||s.id)+'_'+(s.timestamp||Date.now())+'.csv',R.toCsv([['id','label','timestamp'],[s.id,s.label||'',s.timestamp||'']]));zip.file((s.label||s.id)+'_'+(s.timestamp||Date.now())+'.png',s.png||'');});return zip.generateAsync({type:'blob'}).then(function(blob){return R.download('snapshots_export_'+R.today()+'.zip','application/zip',blob);});}R.toast(R.t('gd.zipMissing'));ids.forEach(function(id){R.download(id+'.csv','text/csv','id\n'+id);});return ids;}
  function render(){document.body.classList.toggle('rvt-snap-select-mode',selectMode);var btn=R.ensure('rvt-snap04-export','button');btn.type='button';btn.className='rvt-gd-button';btn.textContent=R.t('gd.exportSelected',{count:getSelected().length});btn.hidden=!getSelected().length;btn.onclick=exportSelected;}
  window.rvtSnapBulk={getSelected:getSelected,selectAll:selectAll,clearSelection:clearSelection,exportSelected:exportSelected,isSelectMode:function(){return selectMode;}};
  window.rvtSnap04=window.rvtBulkExport=window.rvtSnapBulk;R.boot(render);
})();
;
/* END modules/rvt-snap04-js.js */

/* BEGIN modules/rvt-home03-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var running=false;
  function checks(){return (window.S&&window.S.quickChecks)||window.quickChecks||[];}
  function getFailedChecks(){return checks().filter(function(c){return /fail|warn/.test(String(c.status||c.state).toLowerCase());}).map(function(c){return c.id||c.name;});}
  function rerunFailed(){var ids=getFailedChecks();running=true;render();return Promise.resolve().then(function(){var pass=0,fail=0;checks().forEach(function(c){if(ids.indexOf(c.id||c.name)>=0){if(typeof c.run==='function')c.run();c.timestamp=c.timestamp||R.nowIso();if(String(c.status||c.state).toLowerCase()==='fail')fail++;else pass++;}});R.toast(R.t('gd.rerunDone',{done:ids.length,total:ids.length,pass:pass,fail:fail}));}).finally(function(){running=false;render();});}
  function render(){var has=getFailedChecks().length>0,home=document.querySelector('#view-home,[data-view="home"],main')||document.body,btn=document.getElementById('rvt-home03-rerun');if(!has&&btn){btn.remove();return;}if(has){btn=btn||R.ensure('rvt-home03-rerun','button',home);btn.className='rvt-gd-button';btn.disabled=running;btn.textContent=running?'...':R.t('gd.rerunFailed');btn.onclick=rerunFailed;}}
  window.rvtHome03={rerunFailed:rerunFailed,getFailedChecks:getFailedChecks,isRunning:function(){return running;}};R.boot(render);
})();
;
/* END modules/rvt-home03-js.js */

/* BEGIN modules/rvt-home04-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function getRecent(){return R.parseJson('rvt-recent-setups',[],Array.isArray).slice(0,3);}
  function save(a){return R.setJson('rvt-recent-setups',a.slice(0,3));}
  function add(port,ble,profile){var key=[port,ble,profile].join('|'),a=getRecent().filter(function(e){return [e.port,e.ble,e.profile].join('|')!==key;});a.unshift({port:port,ble:ble,profile:profile,usedAt:R.nowIso()});render();return save(a);}
  function remove(i){var a=getRecent();a.splice(Number(i),1);render();return save(a);}
  function setField(sel,val){var f=document.querySelector(sel);if(f){f.value=val||'';f.dispatchEvent(new Event('input',{bubbles:true}));f.dispatchEvent(new Event('change',{bubbles:true}));}}
  function apply(i){var e=getRecent()[Number(i)];if(!e)return null;setField('#radarPort,input[name="port"],#port',e.port);setField('#bleAddress,input[name="ble"],#ble',e.ble);setField('#profileName,input[name="profile"],#profile',e.profile);return e;}
  function render(){var host=document.querySelector('#view-home,#setup-card,[data-setup-card]')||document.body,panel=R.ensure('rvt-home04-recent','div',host);panel.className='rvt-gd-toolbar';panel.innerHTML='';getRecent().forEach(function(e,i){var chip=document.createElement('button');chip.type='button';chip.className='rvt-gd-chip';chip.textContent=(e.profile||'Profile')+' · '+(e.port||'');chip.onclick=function(){apply(i);};var rm=document.createElement('button');rm.type='button';rm.className='rvt-gd-button';rm.setAttribute('aria-label','Remove '+(e.profile||'profile')+' from recent setups');rm.textContent='×';rm.onclick=function(ev){ev.stopPropagation();remove(i);};panel.append(chip,rm);});}
  window.rvtHome04={getRecent:getRecent,add:add,remove:remove,apply:apply};R.boot(render);
})();
;
/* END modules/rvt-home04-js.js */

/* BEGIN modules/rvt-home05-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function getFilter(){var v=R.parseJson('rvt-sessions-filter',{readiness:[],myOnly:false},function(x){return x&&Array.isArray(x.readiness)&&typeof x.myOnly==='boolean';});return {readiness:v.readiness,myOnly:v.myOnly};}
  function save(v){R.setJson('rvt-sessions-filter',v);render();return v;}
  function setChip(label,active){var v=getFilter(),i=v.readiness.indexOf(label);if(active&&i<0)v.readiness.push(label);if(!active&&i>=0)v.readiness.splice(i,1);return save(v);}
  function setMyOnly(b){var v=getFilter();v.myOnly=!!b;return save(v);}
  function clearAll(){return save({readiness:[],myOnly:false});}
  function render(){document.querySelectorAll('[data-readiness-filter],.readiness-filter').forEach(function(c){var label=c.dataset.readinessFilter||c.textContent.trim();c.setAttribute('aria-pressed',getFilter().readiness.indexOf(label)>=0?'true':'false');});var link=R.ensure('rvt-home05-clear','button');link.type='button';link.className='rvt-gd-button';link.textContent=R.t('gd.clearFilters');link.hidden=!getFilter().myOnly&&!getFilter().readiness.length;link.onclick=clearAll;}
  window.rvtHome05={getFilter:getFilter,setChip:setChip,setMyOnly:setMyOnly,clearAll:clearAll};R.boot(render);
})();
;
/* END modules/rvt-home05-js.js */

/* BEGIN modules/rvt-wave04-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var current=null,hideTimer=null;
  function set(start,end){var d=Math.abs(Number(end)-Number(start));current={deltaMs:d,bpm:d?60000/d:null};return current;}
  function simulate(startMs,endMs){var m=set(startMs,endMs);show(0,0);return m;}
  function clear(){current=null;if(hideTimer)clearTimeout(hideTimer);var n=document.getElementById('rvt-wave04-overlay');if(n)n.remove();}
  function show(x,y){if(!current)return;var n=R.ensure('rvt-wave04-overlay','div');n.className='rvt-wave04-overlay';n.textContent=R.t('gd.measure',{delta:Math.round(current.deltaMs),bpm:current.bpm?current.bpm.toFixed(1):'N/A'});n.style.left=(x||16)+'px';n.style.top=(y||16)+'px';if(hideTimer)clearTimeout(hideTimer);hideTimer=setTimeout(clear,5000);}
  document.addEventListener('pointerdown',function(e){if(!e.shiftKey||!(e.target&&e.target.closest&&e.target.closest('canvas,[data-waveform]')))return;var start=e.clientX;function move(ev){set(start,ev.clientX);show(ev.clientX,ev.clientY);}function up(ev){move(ev);document.removeEventListener('pointermove',move);document.removeEventListener('pointerup',up);}document.addEventListener('pointermove',move);document.addEventListener('pointerup',up);},true);
  window.rvtWave04={getMeasurement:function(){return current;},clearMeasurement:clear,simulate:simulate};
  window.rvtWave04Measure=window.rvtWave04;
})();
;
/* END modules/rvt-wave04-js.js */

/* BEGIN modules/rvt-funnel03-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var tickTimer=null;
  function enabled(){return R.getScalar('rvt-funnel-ghost','false')==='true';}
  function getGhostData(){var b=(window.S&&(window.S.funnelBuffer||window.S.funnelsBuffer))||[];var target=Date.now()-60000;if(Array.isArray(b)&&b.length){return b.reduce(function(best,x){return !best||Math.abs((x.timestamp||0)-target)<Math.abs((best.timestamp||0)-target)?x:best;},null);}return window.S&&window.S.funnelsPrevious||{};}
  function render(){var host=document.querySelector('#funnelDiagram,[data-funnel-diagram],#view-funnels')||document.body;var toggle=R.ensure('rvt-funnel03-toggle','button',host);toggle.className='rvt-gd-button';toggle.setAttribute('aria-pressed',enabled()?'true':'false');toggle.textContent=R.t('gd.previousMinute');toggle.onclick=function(){enabled()?disable():enable();};var layer=document.getElementById('rvt-funnel03-ghost');if(enabled()){layer=layer||R.ensure('rvt-funnel03-ghost','div',host);layer.className='rvt-funnel-ghost-layer';layer.textContent=JSON.stringify(getGhostData()||{});}else if(layer)layer.remove();}
  function enable(){R.setScalar('rvt-funnel-ghost','true');if(!tickTimer)tickTimer=setInterval(render,1000);render();}
  function disable(){R.setScalar('rvt-funnel-ghost','false');if(tickTimer)clearInterval(tickTimer);tickTimer=null;render();}
  window.addEventListener('beforeunload',function(){if(tickTimer)clearInterval(tickTimer);});
  window.rvtFunnel03={enable:enable,disable:disable,isEnabled:enabled,getGhostData:getGhostData};R.boot(render);
})();
;
/* END modules/rvt-funnel03-js.js */

/* BEGIN modules/rvt-sec03-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function textBytes(s){return new TextEncoder().encode(s);}
  function hex(buf){return Array.from(new Uint8Array(buf)).map(function(b){return b.toString(16).padStart(2,'0');}).join('');}
  function fallbackHash(s){var h=0;for(var i=0;i<s.length;i++)h=((h<<5)-h+s.charCodeAt(i))|0;return ('00000000'+(h>>>0).toString(16)).slice(-8);}
  function hashEntry(entry,prevHash){var s=JSON.stringify(Object.assign({},entry,{prevHash:prevHash||''}));if(window.crypto&&crypto.subtle&&crypto.subtle.digest)return crypto.subtle.digest('SHA-256',textBytes(s)).then(hex);return Promise.resolve(fallbackHash(s));}
  function entries(){return (window.S&&window.S.auditLog)||R.parseJson('rvt-audit-log',[],Array.isArray);}
  async function verify(){var prev='',list=entries();for(var i=0;i<list.length;i++){var e=list[i],expected=await hashEntry(Object.assign({},e,{hash:undefined,prevHash:undefined}),prev);if(e.prevHash&&e.prevHash!==prev)return {ok:false,breakAt:e.timestamp||i};if(e.hash&&e.hash!==expected)return {ok:false,breakAt:e.timestamp||i};prev=e.hash||expected;}return {ok:true,breakAt:null,count:list.length};}
  function getChainHead(){var a=entries();return a.length?(a[a.length-1].hash||a[a.length-1].prevHash||''):'';}
  function render(){var host=document.querySelector('#settings-data-privacy,[data-settings-section="privacy"]')||document.body;var b=R.ensure('rvt-sec03-verify','button',host);b.className='rvt-gd-button';b.textContent=R.t('gd.auditVerify');b.onclick=function(){verify().then(function(r){R.toast(r.ok?R.t('gd.auditPass',{count:r.count||0}):R.t('gd.auditFail',{timestamp:r.breakAt}));});};}
  window.rvtSec03={hashEntry:hashEntry,verify:verify,getChainHead:getChainHead};window.rvtAuditTamper=window.rvtSec03;R.boot(render);
})();
;
/* END modules/rvt-sec03-js.js */

/* BEGIN modules/rvt-obs01-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var visible=false,samples=[],frames=0,last=performance.now(),msgCount=0,timer=null,raf=null;
  function loop(t){frames++;if(!document.hidden&&visible){raf=requestAnimationFrame(loop);}else{raf=null;}}
  function storageUsage(){var used=0;try{for(var i=0;i<localStorage.length;i++){var k=localStorage.key(i);used+=k.length+(localStorage.getItem(k)||'').length;}}catch(_){}return used;}
  function sample(){var now=performance.now(),fps=Math.round(frames*1000/Math.max(1,now-last));frames=0;last=now;var s={timestamp:R.nowIso(),fps:fps,heap:(performance.memory&&performance.memory.usedJSHeapSize)||null,msgsPerSec:msgCount,storageBytes:storageUsage()};msgCount=0;samples.push(s);samples=samples.slice(-60);render();if(visible)timer=setTimeout(sample,1000);}
  function render(){var n=document.getElementById('rvt-perf-overlay');if(!visible){if(n)n.remove();return;}n=n||R.ensure('rvt-perf-overlay','div');n.className='rvt-perf-overlay';var s=samples[samples.length-1]||{};n.innerHTML='FPS: '+(s.fps||0)+'<br>Heap: '+(s.heap?Math.round(s.heap/1024/1024)+' MB':'N/A')+'<br>Msgs/s: '+(s.msgsPerSec||0)+'<br>Storage: '+(s.storageBytes||0)+' bytes<br><button class="rvt-gd-button" id="rvt-perf-export">'+R.t('gd.perfExport')+'</button>';n.querySelector('button').onclick=exportLog;}
  function show(){visible=true;if(!raf)raf=requestAnimationFrame(loop);if(!timer)sample();render();}
  function hide(){visible=false;if(timer)clearTimeout(timer);timer=null;render();}
  function toggle(){visible?hide():show();}
  function exportLog(){var rows=[['timestamp','fps','heap','msgsPerSec','storageBytes']].concat(samples.map(function(s){return [s.timestamp,s.fps,s.heap||'N/A',s.msgsPerSec,s.storageBytes];}));return R.download('perf_log_'+R.stampFile()+'.csv','text/csv',R.toCsv(rows));}
  window.addEventListener('message',function(){msgCount++;});window.addEventListener('rvt-live-update',function(){msgCount++;});document.addEventListener('keydown',function(e){if(e.ctrlKey&&e.shiftKey&&e.key.toUpperCase()==='P'){e.preventDefault();toggle();}},true);window.addEventListener('beforeunload',function(){if(timer)clearTimeout(timer);if(raf)cancelAnimationFrame(raf);});
  window.rvtPerfOverlay={show:show,hide:hide,toggle:toggle,isVisible:function(){return visible;},exportLog:exportLog,getSamples:function(){return samples.slice();}};window.rvtObs01=window.rvtPerfOverlay;if(location.search.indexOf('debug=perf')>=0)R.boot(show);
})();
;
/* END modules/rvt-obs01-js.js */

/* BEGIN modules/rvt-audio01-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var active=localStorage.getItem('rvt-audio-profile-active')||'Clinical';
  var profiles={Clinical:{volume:0.7,voice:true,haptic:false,sound:'voice'},Field:{volume:1,voice:false,haptic:true,sound:'tone'},Quiet:{volume:0,voice:false,haptic:true,sound:'none'}};
  function stored(){return R.parseJson('rvt-settings-profiles',{},function(v){return v&&typeof v==='object';});}
  function getProfiles(){return Object.assign({},profiles,stored().audioProfiles||{});}
  function apply(name){var p=getProfiles()[name];if(!p)return false;active=name;localStorage.setItem('rvt-audio-profile-active',name);localStorage.setItem('rvt-audio-volume',String(p.volume));localStorage.setItem('rvt-voice-alerts',p.voice?'1':'0');localStorage.setItem('rvt-audio-alerts',p.sound==='none'?'0':'1');localStorage.setItem('rvt-haptic-alerts',p.haptic?'1':'0');R.toast(R.t('gd.audioProfile',{name:name}));render();return true;}
  function saveCustom(name,config){var s=stored();s.audioProfiles=s.audioProfiles||{};s.audioProfiles[name]=config;R.setJson('rvt-settings-profiles',s);return getProfiles();}
  function render(){var n=R.ensure('rvt-audio-profile-label','span');n.className='rvt-audio-profile-label';n.textContent=active;}
  window.rvtAudioProfiles={apply:apply,getActive:function(){return active;},getProfiles:getProfiles,saveCustom:saveCustom};R.boot(render);
})();
;
/* END modules/rvt-audio01-js.js */

/* BEGIN modules/rvt-audio02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var ducked=false,threshold=.2,ctx=null,gain=null,base=1;
  function ensureAudio(){if(!ctx&&window.AudioContext){ctx=new AudioContext();gain=ctx.createGain();gain.gain.value=base;gain.connect(ctx.destination);}return gain;}
  function volume(){return Number(localStorage.getItem('rvt-audio-volume')||.7);}
  function duck(){if(volume()<threshold)return false;ducked=true;var g=ensureAudio();if(g){var t=ctx.currentTime;g.gain.cancelScheduledValues(t);g.gain.linearRampToValueAtTime(.2,t+.05);}return true;}
  function restore(){ducked=false;var g=ensureAudio();if(g){var t=ctx.currentTime;g.gain.cancelScheduledValues(t);g.gain.linearRampToValueAtTime(base,t+.5);}return true;}
  if(window.rvtSpeechDebounce&&typeof window.rvtSpeechDebounce.speak==='function'&&!window.rvtSpeechDebounce.speak.__rvtDuck){var old=window.rvtSpeechDebounce.speak;window.rvtSpeechDebounce.speak=function(){duck();var r=old.apply(this,arguments);Promise.resolve(r).finally(restore);return r;};window.rvtSpeechDebounce.speak.__rvtDuck=true;}
  window.rvtVolumeDuck={duck:duck,restore:restore,isDucked:function(){return ducked;},setThreshold:function(r){threshold=Number(r);return threshold;}};
})();
;
/* END modules/rvt-audio02-js.js */

/* BEGIN modules/rvt-cmd01-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function all(){return R.parseJson('rvt-cmd-recent',[],Array.isArray);}
  function save(a){return R.setJson('rvt-cmd-recent',a.slice(0,20));}
  function getRecent(){var cutoff=Date.now()-7*86400000;return all().filter(function(e){return new Date(e.usedAt).getTime()>=cutoff;}).sort(function(a,b){return (b.count-a.count)||String(b.usedAt).localeCompare(String(a.usedAt));}).slice(0,5);}
  function record(actionId,label){var a=all(),e=a.find(function(x){return x.actionId===actionId;});if(e){e.count=(e.count||0)+1;e.usedAt=R.nowIso();e.label=label||e.label;}else a.unshift({actionId:actionId,label:label||actionId,usedAt:R.nowIso(),count:1});return save(a.sort(function(x,y){return String(y.usedAt).localeCompare(String(x.usedAt));}));}
  window.rvtCmdRecent={getRecent:getRecent,record:record,clear:function(){return save([]);}};window.rvtCmd01=window.rvtCmdRecent;
})();
;
/* END modules/rvt-cmd01-js.js */

/* BEGIN modules/rvt-cmd02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var map={dim:'dark theme',screenshot:'snapshot',loud:'audio test',mute:'quiet',colour:'color',record:'start session',stop:'end session',restart:'reconnect',log:'audit',export:'download',profile:'settings profile',calibrate:'stillness',ble:'bluetooth',serial:'port',freeze:'pause',vitals:'live',chart:'waveform',compare:'multi compare',badge:'alert badge',handoff:'operator handoff',lock:'auto lock',soak:'memory soak',density:'compact',font:'text size',tag:'quick tag',fab:'snapshot fab',ghost:'funnel ghost',duck:'volume duck',lineage:'kpi lineage',cluster:'alert cluster',snooze:'alert snooze'};
  function search(q){q=String(q||'').toLowerCase();return Object.keys(map).filter(function(a){return a.indexOf(q)>=0||q.indexOf(a)>=0;}).map(function(a){return {action:map[a],actionId:map[a],matchedAlias:a,subtitle:'matched: '+a+' → '+map[a]};});}
  function addSynonym(alias,actionId){map[String(alias).toLowerCase()]=actionId;return map;}
  window.rvtCmdSynonym={search:search,addSynonym:addSynonym,getMap:function(){return Object.assign({},map);}};window.rvtCmd02=window.rvtCmdSynonym;
})();
;
/* END modules/rvt-cmd02-js.js */

/* BEGIN modules/rvt-nav02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var current=R.currentOperator();
  function getAll(){var set=new Set([current,R.currentOperator()]);((window.S&&window.S.auditLog)||R.parseJson('rvt-audit-log',[],Array.isArray)).forEach(function(e){if(e.operator)set.add(e.operator);var m=String(e.message||'').match(/Operator switched from (.*?) to (.*)$/);if(m){set.add(m[1]);set.add(m[2]);}});return Array.from(set).filter(Boolean);}
  function switchTo(name){var old=current;current=String(name||'').trim()||current;localStorage.setItem('rvt-operator-name',current);window.S=window.S||{};window.S.operator=current;window.S.settings=Object.assign({},window.S.settings||{},{operator:current,operatorName:current});R.audit('Operator switched from '+old+' to '+current,{operator:current});render();return current;}
  function open(){var d=R.ensure('rvt-nav02-dialog','div');d.className='rvt-gd-dialog';d.setAttribute('role','dialog');d.setAttribute('aria-modal','true');d.innerHTML='<h3>'+R.t('gd.operatorSwitch')+'</h3>';getAll().forEach(function(op){var b=document.createElement('button');b.className='rvt-gd-button';b.textContent=op;b.onclick=function(){switchTo(op);d.remove();};d.appendChild(b);});var add=document.createElement('button');add.className='rvt-gd-button';add.textContent=R.t('gd.addOperator');add.onclick=function(){var n=prompt(R.t('gd.addOperator'));if(n){switchTo(n);d.remove();}};d.appendChild(add);d.tabIndex=-1;d.focus();}
  function render(){document.querySelectorAll('[data-operator-card],#operatorCard,.operator-card').forEach(function(n){n.style.cursor='pointer';n.addEventListener('click',open,{once:false});});}
  document.addEventListener('keydown',function(e){if(e.key==='Escape'){var d=document.getElementById('rvt-nav02-dialog');if(d)d.remove();}},true);
  window.rvtOperatorSwitch={open:open,switchTo:switchTo,getCurrent:function(){return current||R.currentOperator();},getAll:getAll};window.rvtNav02=window.rvtOperatorSwitch;R.boot(render);
})();
;
/* END modules/rvt-nav02-js.js */

/* BEGIN modules/rvt-home06-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function buildBrief(){var S=window.S||{},setup=S.setup||{};return [R.today(),'Op: '+R.currentOperator(),'Subject: '+(setup.subjectId||S.subjectId||''),'Port: '+(setup.port||S.port||''),'Readiness: '+(S.readiness||S.score||0)+'%','Profile: '+(setup.profileName||S.profileName||'')].join(' · ');}
  async function copy(){var text=buildBrief(),copied=false;if(navigator.clipboard&&navigator.clipboard.writeText){try{await navigator.clipboard.writeText(text);copied=true;}catch(_){copied=false;}}if(!copied){var d=R.ensure('rvt-home06-copy-modal','div');d.className='rvt-gd-dialog';d.innerHTML='<input readonly>';d.querySelector('input').value=text;d.querySelector('input').select();}R.toast(R.t('gd.sessionBriefCopied'));}
  window.rvtHome06={copy:copy,buildBrief:buildBrief};R.boot(function(){var b=R.ensure('rvt-home06-copy','button');b.className='rvt-gd-button';b.textContent=R.t('gd.copyBrief');b.onclick=copy;});
})();
;
/* END modules/rvt-home06-js.js */

/* BEGIN modules/rvt-live08-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var zone={min:40,max:80},last=null;
  function isInZone(cm){cm=Number(cm);return cm>=zone.min&&cm<=zone.max;}
  function update(rangeCm){last=Number(rangeCm);var c=document.querySelector('#radarPreview,[data-radar-preview],canvas[aria-label*="Radar"]');if(c){c.setAttribute('aria-label',R.t(isInZone(last)?'gd.sweetInside':'gd.sweetOutside',{range:last}));c.dataset.sweetZone=isInZone(last)?'inside':'outside';}return isInZone(last);}
  window.rvtLive08={isInZone:isInZone,getZone:function(){return Object.assign({},zone);},update:update};window.rvtSweetZone=window.rvtLive08;R.boot(function(){var v=window.S&&window.S.live&&(window.S.live.range_cm||window.S.live.range);if(v!=null)update(v);});
})();
;
/* END modules/rvt-live08-js.js */

/* BEGIN modules/rvt-wave06-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var active=false,paused=false,speed=1;
  function render(){var host=document.querySelector('#wavesTab,#view-waves,[data-tab="waves"]')||document.body,bar=document.getElementById('rvt-wave06-replay');if(active){bar=bar||R.ensure('rvt-wave06-replay','div',host);bar.className='rvt-gd-panel';bar.innerHTML='<button class="rvt-gd-button" id="rvt-wave06-play">'+(paused?'Play':'Pause')+'</button><select id="rvt-wave06-speed"><option>0.5</option><option selected>1</option><option>2</option></select><input type="range" min="0" max="30" value="0"><output>00:00</output>';bar.querySelector('#rvt-wave06-play').onclick=function(){paused?resume():pause();};bar.querySelector('#rvt-wave06-speed').onchange=function(e){setSpeed(Number(e.target.value));};}else if(bar)bar.remove();}
  function start(){active=true;paused=false;render();return true;}function pause(){paused=true;render();}function resume(){paused=false;render();}function setSpeed(m){if([.5,1,2].indexOf(Number(m))>=0)speed=Number(m);return speed;}function stop(){active=false;paused=false;render();}
  document.addEventListener('keydown',function(e){if(e.key==='Escape'&&active)stop();},true);
  window.rvtWaveReplay={start:start,pause:pause,resume:resume,setSpeed:setSpeed,stop:stop,isActive:function(){return active;}};R.boot(function(){var b=R.ensure('rvt-wave06-button','button');b.className='rvt-gd-button';b.textContent=R.t('gd.replay');b.onclick=start;});
})();
;
/* END modules/rvt-wave06-js.js */

/* BEGIN modules/rvt-snap05-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function htmlEsc(s){return String(s||'').replace(/[&<>"']/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];});}
  function escape(source){return htmlEsc(source).replace(/`/g,'&#96;').replace(/#/g,'&#35;');}
  function render(src){var s=escape(src);var lines=s.split(/\r?\n/),out=[],inList=false;lines.forEach(function(l){var m=l.match(/^\s*-\s+(.+)/);if(m){if(!inList){out.push('<ul>');inList=true;}out.push('<li>'+m[1].replace(/\*\*([^*]+)\*\*/g,'<strong>$1</strong>').replace(/\*([^*]+)\*/g,'<em>$1</em>')+'</li>');}else{if(inList){out.push('</ul>');inList=false;}out.push(l.replace(/\*\*([^*]+)\*\*/g,'<strong>$1</strong>').replace(/\*([^*]+)\*/g,'<em>$1</em>'));}});if(inList)out.push('</ul>');return out.join('\n');}
  window.rvtSnapMarkdown={render:render,escape:escape,isMarkdown:function(s){return /(\*\*[^*]+\*\*|\*[^*]+\*|^\s*-\s+)/m.test(String(s||''));}};
})();
;
/* END modules/rvt-snap05-js.js */

/* BEGIN modules/rvt-audit04-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function getBarData(){var a=(window.S&&window.S.auditLog)||R.parseJson('rvt-audit-log',[],Array.isArray);var by={};a.forEach(function(e){var r=e.reason||e.message||'general',sev=e.severity||'info';by[r]=by[r]||{reason:r,info:0,warn:0,critical:0};if(!by[r][sev])by[r][sev]=0;by[r][sev]++;});return Object.keys(by).map(function(k){return by[k];});}
  function render(){var host=document.querySelector('#auditReasons,[data-audit-reasons],#view-audit')||document.body,root=R.ensure('rvt-audit04-histogram','div',host);root.className='rvt-gd-panel';root.innerHTML='';getBarData().forEach(function(b){var row=document.createElement('div');row.className='rvt-alert-severity-slice';row.title='info: '+b.info+', warn: '+b.warn+', critical: '+b.critical;row.textContent=b.reason+' '+(b.info+b.warn+b.critical);root.appendChild(row);});}
  window.rvtAuditStacked={render:render,getBarData:getBarData};R.boot(render);
})();
;
/* END modules/rvt-audit04-js.js */

/* BEGIN modules/rvt-rpt07-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function cards(){return document.querySelectorAll('.diagnostic-card,[data-diagnostic-card],.report-diagnostic-card');}
  function applyTints(){cards().forEach(function(c){var v=(c.dataset.verdict||c.dataset.status||c.textContent||'').toLowerCase();c.classList.add('rvt-diagnostic-card-tinted');c.style.backgroundColor=v.indexOf('fail')>=0?'color-mix(in srgb,var(--bad,#b83030) 6%,var(--surface-1,#fff))':v.indexOf('warn')>=0?'color-mix(in srgb,var(--warn,#b7791f) 6%,var(--surface-1,#fff))':'color-mix(in srgb,var(--ok,#1f8f5f) 6%,var(--surface-1,#fff))';});}
  function removeTints(){cards().forEach(function(c){c.classList.remove('rvt-diagnostic-card-tinted');c.style.backgroundColor='';});}
  window.rvtRpt07={applyTints:applyTints,removeTints:removeTints};window.rvtTintedCards=window.rvtRpt07;R.boot(applyTints);
})();
;
/* END modules/rvt-rpt07-js.js */

/* BEGIN modules/rvt-help04-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function all(){return R.parseJson('rvt-help-feedback',{},function(v){return v&&typeof v==='object';});}
  function vote(id,dir){if(['up','down'].indexOf(dir)<0)throw new Error('invalid vote');var m=all();m[id]=dir;R.setJson('rvt-help-feedback',m);var url=localStorage.getItem('rvt-feedback-endpoint');if(url)fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({entryId:id,vote:dir,appVersion:window.RVT_CLIENT_VERSION||'',timestamp:R.nowIso()})}).catch(function(){});render();return dir;}
  function getVote(id){return all()[id]||null;}
  function render(){document.querySelectorAll('[data-help-entry-id],.faq-entry').forEach(function(e,i){var id=e.dataset.helpEntryId||('entry-'+i),v=getVote(id),w=e.querySelector('.rvt-help-feedback')||document.createElement('div');w.className='rvt-help-feedback rvt-gd-toolbar';w.innerHTML=v?'<span>'+R.t('gd.helpThanks')+'</span>':'<button class="rvt-gd-button" data-v="up">Up</button><button class="rvt-gd-button" data-v="down">Down</button>';if(!w.parentNode)e.appendChild(w);w.querySelectorAll('button').forEach(function(b){b.onclick=function(){vote(id,b.dataset.v);};});});}
  window.rvtHelpFeedback={vote:vote,getVote:getVote,getAll:all};R.boot(render);
})();
;
/* END modules/rvt-help04-js.js */

/* BEGIN modules/rvt-set02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var map={source:'cable',appearance:'palette',alerts:'notifications',performance:'speed','data-privacy':'lock',about:'info',developer:'code'};
  function apply(){document.querySelectorAll('[data-settings-section],.settings-section').forEach(function(s){var id=s.dataset.settingsSection||s.id.replace(/^settings-/,'');var icon=map[id];if(!icon)return;var h=s.querySelector('h2,h3,.section-title')||s;if(!h.querySelector('.rvt-section-icon')){var span=document.createElement('span');span.className='material-symbols-rounded rvt-section-icon';span.setAttribute('aria-hidden','true');span.textContent=icon;h.prepend(span);}});}
  window.rvtSectionIcons={apply:apply,getIconMap:function(){return Object.assign({},map);}};R.boot(apply);
})();
;
/* END modules/rvt-set02-js.js */

/* BEGIN modules/rvt-set07-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var defaults={'rvt-theme':'system','rvt-density':'comfortable','rvt-live-mode':'simple','rvt-audio-volume':'0.7','rvt-toast-position':'BR'};
  function getDiff(){return Object.keys(defaults).filter(function(k){return localStorage.getItem(k)!==null&&localStorage.getItem(k)!==defaults[k];}).map(function(k){return {key:k,current:localStorage.getItem(k),default:defaults[k]};});}
  function resetOne(key){if(key in defaults)localStorage.setItem(key,defaults[key]);scan();return defaults[key];}
  function scan(){document.querySelectorAll('[data-setting-key]').forEach(function(el){var key=el.dataset.settingKey,d=getDiff().find(function(x){return x.key===key;}),dot=el.querySelector('.rvt-setting-modified-dot');if(d&&!dot){dot=document.createElement('button');dot.type='button';dot.className='rvt-setting-modified-dot';dot.title=R.t('gd.modified',{value:d.default});dot.setAttribute('aria-label',R.t('gd.resetSetting'));dot.onclick=function(){resetOne(key);};el.appendChild(dot);}else if(!d&&dot)dot.remove();});return getDiff();}
  window.rvtModifiedDot={scan:scan,getDiff:getDiff,resetOne:resetOne};R.boot(scan);
})();
;
/* END modules/rvt-set07-js.js */

/* BEGIN modules/rvt-alert05-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function pins(){return R.parseJson('rvt-alert-pins',[],Array.isArray).slice(0,3);}
  function save(a){R.setJson('rvt-alert-pins',a.slice(0,3));render();return pins();}
  function pin(id){var a=pins().filter(function(x){return x!==id;});if(a.length>=3){if(window.confirm&&!window.confirm(R.t('gd.replacePin')))return a;a.shift();}a.push(id);return save(a);}
  function unpin(id){return save(pins().filter(function(x){return x!==id;}));}
  function render(){document.querySelectorAll('[data-alert-id],.alert-item').forEach(function(n){if(n.querySelector('.rvt-alert05-pin'))return;var id=n.dataset.alertId||n.id,b=document.createElement('button');b.className='rvt-gd-button rvt-alert05-pin';b.setAttribute('aria-label',R.t('gd.pinAlert'));b.textContent='pin';b.onclick=function(){pins().indexOf(id)>=0?unpin(id):pin(id);};n.appendChild(b);});}
  window.rvtPinAlert={pin:pin,unpin:unpin,isPinned:function(id){return pins().indexOf(id)>=0;},getPinned:pins};R.boot(render);
})();
;
/* END modules/rvt-alert05-js.js */

/* BEGIN modules/rvt-nav03-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var startMs=null,timer=null,lastAria=0;
  function fmt(s){return Math.floor(s/60)+':'+String(s%60).padStart(2,'0');}
  function tick(){var s=getSeconds(),b=document.querySelector('#pauseBtn,[data-pause-button],.pause-button');if(b){b.textContent=R.t('gd.paused',{time:fmt(s)});if(Date.now()-lastAria>5000){b.setAttribute('aria-label',R.t('gd.paused',{time:fmt(s)}));lastAria=Date.now();}}timer=setTimeout(tick,1000);}
  function start(){if(timer)clearTimeout(timer);startMs=Date.now();tick();}
  function stop(){if(timer)clearTimeout(timer);timer=null;startMs=null;var b=document.querySelector('#pauseBtn,[data-pause-button],.pause-button');if(b){b.textContent='';b.removeAttribute('aria-label');}}
  function getSeconds(){return startMs?Math.floor((Date.now()-startMs)/1000):0;}
  window.addEventListener('beforeunload',function(){if(timer)clearTimeout(timer);});
  window.rvtPauseDuration={start:start,stop:stop,getSeconds:getSeconds};
})();
;
/* END modules/rvt-nav03-js.js */

/* BEGIN modules/rvt-qol02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function get(){return R.getScalar('rvt-toast-position','BR');}
  function apply(){['TL','TR','BL','BR'].forEach(function(p){document.body.classList.remove('rvt-toast-pos-'+p);});document.body.classList.add('rvt-toast-pos-'+get());return get();}
  function set(pos){if(['TL','TR','BL','BR'].indexOf(pos)<0)throw new Error('invalid position');R.setScalar('rvt-toast-position',pos);return apply();}
  window.rvtToastPos={set:set,get:get,apply:apply};R.boot(apply);
})();
;
/* END modules/rvt-qol02-js.js */

/* BEGIN modules/rvt-qol06-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var prev=null;
  function isOverridden(){var v=R.getScalar('rvt-density-override-until','');return !!v&&new Date(v).getTime()>Date.now();}
  function onSessionStart(){if(!isOverridden()){prev=localStorage.getItem('rvt-density')||'comfortable';localStorage.setItem('rvt-density','compact');document.documentElement.dataset.density='compact';document.body.classList.add('rvt-density-session-compact');}return localStorage.getItem('rvt-density');}
  function onSessionEnd(){if(prev){localStorage.setItem('rvt-density',prev);document.documentElement.dataset.density=prev;}document.body.classList.remove('rvt-density-session-compact');return localStorage.getItem('rvt-density');}
  function setOverride(){var until=new Date(Date.now()+86400000).toISOString();R.setScalar('rvt-density-override-until',until);return until;}
  window.rvtAdaptiveDensity={onSessionStart:onSessionStart,onSessionEnd:onSessionEnd,setOverride:setOverride,isOverridden:isOverridden};
})();
;
/* END modules/rvt-qol06-js.js */

/* BEGIN modules/rvt-mob02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var active=false;
  function isLandscape(){return window.innerWidth>window.innerHeight;}
  function render(){document.body.classList.toggle('rvt-field-layout-active',active&&isLandscape());var n=document.getElementById('rvt-field-layout-surface');if(active){n=n||R.ensure('rvt-field-layout-surface','div');n.className='rvt-field-layout-surface';n.innerHTML=isLandscape()?'<div><span>HR</span><b>'+(window.S&&window.S.live&&window.S.live.hr||'--')+'</b></div><div><span>RR</span><b>'+(window.S&&window.S.live&&window.S.live.rr||'--')+'</b></div>':'<p>'+R.t('gd.rotate')+'</p>';}else if(n)n.remove();}
  function enable(){active=true;render();return true;}function disable(){active=false;render();}
  document.addEventListener('keydown',function(e){if(e.shiftKey&&e.key.toUpperCase()==='L'){e.preventDefault();active?disable():enable();}},true);window.addEventListener('resize',render);
  window.rvtFieldLayout={enable:enable,disable:disable,isActive:function(){return active;},isLandscape:isLandscape};
})();
;
/* END modules/rvt-mob02-js.js */

/* BEGIN modules/rvt-live02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function layout(){return R.parseJson('rvt-kpi-layout',{},function(v){return v&&typeof v==='object';});}
  function save(l){R.setJson('rvt-kpi-layout',l);apply();return l;}
  function setSize(id,size){if(['1x','2x'].indexOf(size)<0)throw new Error('invalid size');var l=layout();l[id]=size;return save(l);}
  function getSize(id){return layout()[id]||'1x';}
  function resetLayout(){return save({});}
  function apply(){document.querySelectorAll('[data-kpi-id]').forEach(function(c){var id=c.dataset.kpiId;c.classList.toggle('rvt-kpi-size-2x',getSize(id)==='2x');if(!c.querySelector('.rvt-kpi-resize-handle')){var h=document.createElement('button');h.type='button';h.className='rvt-kpi-resize-handle';h.setAttribute('aria-label','Resize KPI');h.onclick=function(e){e.stopPropagation();setSize(id,getSize(id)==='2x'?'1x':'2x');};c.style.position=c.style.position||'relative';c.appendChild(h);}c.addEventListener('keydown',function(e){if(e.shiftKey&&e.key==='ArrowRight')setSize(id,'2x');if(e.shiftKey&&e.key==='ArrowLeft')setSize(id,'1x');});});}
  window.rvtLive02={setSize:setSize,getSize:getSize,getLayout:layout,resetLayout:resetLayout};R.boot(apply);
})();
;
/* END modules/rvt-live02-js.js */

/* BEGIN modules/rvt-setup04-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var active=false,snapshot=null;
  function start(){snapshot=JSON.stringify(window.S||{});active=true;document.body.classList.add('rvt-dry-run-active');var w=R.ensure('rvt-preview-watermark','div');w.className='rvt-preview-watermark';w.textContent=R.t('gd.preview');if(window.rvtDemoMode)window.rvtDemoMode.enable('Normal');return true;}
  function stop(){active=false;document.body.classList.remove('rvt-dry-run-active');var w=document.getElementById('rvt-preview-watermark');if(w)w.remove();if(window.rvtDemoMode)window.rvtDemoMode.disable();try{window.S=JSON.parse(snapshot||'{}');}catch(_){};}
  window.rvtDryRun={start:start,stop:stop,isActive:function(){return active;}};R.boot(function(){var b=R.ensure('rvt-setup04-preview','button');b.className='rvt-gd-button';b.textContent=R.t('gd.dryRun');b.onclick=start;});
})();
;
/* END modules/rvt-setup04-js.js */

/* BEGIN modules/rvt-cmd04-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var hist=[];
  function parse(s){return String(s||'').split(/\s*(?:&&|;)\s*/).map(function(x){return x.trim();}).filter(Boolean);}
  function exec(label){if(window.rvtCmdSynonym){var m=window.rvtCmdSynonym.search(label)[0];if(m)label=m.action;}if(/fail/i.test(label))throw new Error(label);R.toast(label);return label;}
  async function run(chain){var parts=parse(chain);hist.unshift({chain:chain,at:R.nowIso()});hist=hist.slice(0,10);for(var i=0;i<parts.length;i++){try{var out=exec(parts[i]);R.toast(R.t('gd.step',{step:i+1,total:parts.length,label:out}));}catch(err){R.toast(R.t('gd.chainAbort',{step:i+1,reason:err.message}));throw err;}}return true;}
  window.rvtActionChain={run:run,parse:parse,getHistory:function(){return hist.slice();}};
})();
;
/* END modules/rvt-cmd04-js.js */

/* BEGIN modules/rvt-i18n02-js.js */
(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function enable(){document.documentElement.setAttribute('dir','rtl');document.body.classList.add('rvt-rtl');return true;}
  function disable(){document.documentElement.setAttribute('dir','ltr');document.body.classList.remove('rvt-rtl');}
  window.rvtRtlSupport={enable:enable,disable:disable,isRtl:function(){return document.documentElement.getAttribute('dir')==='rtl';}};
})();
;
/* END modules/rvt-i18n02-js.js */

/* BEGIN modules/rvt-sec01-js.js */
(function(){
  'use strict';
  var CHECKPOINT_STORE = 'settings-backup';
  var SALT_KEY = 'rvt-key-salt';
  var WARN_KEY = 'rvt-enc-warned';
  var STRINGS = {
    'sec01.reencrypt':'Re-encrypt all sessions',
    'sec01.reencryptDone':'Session data re-encrypted',
    'sec01.unavailable':'Encryption unavailable - data stored without encryption. Use HTTPS for full security.',
    'sec01.authFailed':'Authentication failed - could not derive encryption key',
    'sec01.decryptFailed':'Could not decrypt session data - key may have changed',
    'sec01.rotationFailed':'Key rotation failed - operator not switched'
  };
  var keyRef = null;
  var lastCredential = '';
  var degraded = false;
  var lastTiming = null;
  var patchedIdb = false;
  var patchedUnlock = false;
  var patchedOperator = false;
  var origIdb = null;
  var pendingUnlockCredential = '';

  function extendStrings(){
    window.RVT_STRINGS = window.RVT_STRINGS || { en: {} };
    window.RVT_STRINGS.en = window.RVT_STRINGS.en || {};
    Object.keys(STRINGS).forEach(function(k){
      if (!(k in window.RVT_STRINGS.en)) window.RVT_STRINGS.en[k] = STRINGS[k];
    });
  }
  function t(key, vars){
    extendStrings();
    var s = typeof window.rvtT === 'function' ? window.rvtT(key, vars || {}) : (window.RVT_STRINGS.en[key] || key);
    Object.keys(vars || {}).forEach(function(k){ s = String(s).replace(new RegExp('\\{'+k+'\\}','g'), String(vars[k])); });
    return s;
  }
  function toast(msg){
    if (typeof window.toast === 'function') window.toast(msg);
    else if (typeof window.showToast === 'function') window.showToast(msg);
    else console.warn('[RVT-SEC01]', msg);
  }
  function registerManifest(){
    var m = window.rvtStorage && window.rvtStorage.manifest;
    if (!m) return;
    if (!m[SALT_KEY]) m[SALT_KEY] = { kind:'scalar', defaultValue:'', validate:function(v){ return typeof v === 'string'; } };
    if (!m[WARN_KEY]) m[WARN_KEY] = { kind:'scalar', defaultValue:'false', validate:function(v){ return v === 'true' || v === 'false'; } };
  }
  function hasCrypto(){
    if (window.__rvtSec01ForceNoCrypto) return false;
    return !!(window.crypto && window.crypto.subtle && window.crypto.getRandomValues &&
      crypto.subtle.importKey && crypto.subtle.deriveKey && crypto.subtle.encrypt && crypto.subtle.decrypt);
  }
  function warnUnavailableOnce(){
    degraded = true;
    try {
      if (localStorage.getItem(WARN_KEY) !== 'true') {
        toast(t('sec01.unavailable'));
        localStorage.setItem(WARN_KEY, 'true');
      }
    } catch (_) {}
  }
  function bytesToBase64(bytes){
    var s = '';
    new Uint8Array(bytes).forEach(function(b){ s += String.fromCharCode(b); });
    return btoa(s);
  }
  function base64ToBytes(s){
    var raw = atob(String(s || ''));
    var out = new Uint8Array(raw.length);
    for (var i = 0; i < raw.length; i++) out[i] = raw.charCodeAt(i);
    return out;
  }
  function concatIvCipher(iv, cipher){
    var a = new Uint8Array(iv);
    var b = new Uint8Array(cipher);
    var out = new Uint8Array(a.length + b.length);
    out.set(a, 0);
    out.set(b, a.length);
    return out.buffer;
  }
  function getSalt(){
    var salt = '';
    try { salt = localStorage.getItem(SALT_KEY) || ''; } catch (_) {}
    if (!salt) {
      var bytes = new Uint8Array(16);
      if (window.crypto && window.crypto.getRandomValues) crypto.getRandomValues(bytes);
      else for (var i = 0; i < bytes.length; i++) bytes[i] = Math.floor(Math.random() * 256);
      salt = bytesToBase64(bytes);
      try { localStorage.setItem(SALT_KEY, salt); } catch (_) {}
    }
    return salt;
  }
  async function derive(credential, saltB64){
    if (!hasCrypto()) { warnUnavailableOnce(); return null; }
    var source = String(credential || '');
    if (!source) throw new Error('missing credential');
    var material = await crypto.subtle.importKey('raw', new TextEncoder().encode(source), 'PBKDF2', false, ['deriveKey']);
    return crypto.subtle.deriveKey(
      { name:'PBKDF2', salt:base64ToBytes(saltB64 || getSalt()), iterations:100000, hash:'SHA-256' },
      material,
      { name:'AES-GCM', length:256 },
      false,
      ['encrypt','decrypt']
    );
  }
  async function loadKey(credential){
    try {
      var k = await derive(credential, getSalt());
      keyRef = k;
      lastCredential = String(credential || '');
      degraded = !k;
      return;
    } catch (err) {
      keyRef = null;
      toast(t('sec01.authFailed'));
      throw err;
    }
  }
  function isKeyLoaded(){ return !!keyRef; }
  function isEncrypted(){ return hasCrypto() && !!keyRef && !degraded; }
  function devMode(){
    try { return new URLSearchParams(location.search).get('debug') === 'sec' || localStorage.getItem('rvt-dev-mode') === 'true'; }
    catch (_) { return false; }
  }
  async function encryptWithKey(obj, k){
    var iv = new Uint8Array(12);
    crypto.getRandomValues(iv);
    var plain = new TextEncoder().encode(JSON.stringify(obj));
    var cipher = await crypto.subtle.encrypt({ name:'AES-GCM', iv:iv }, k, plain);
    return concatIvCipher(iv, cipher);
  }
  async function decryptWithKey(buffer, k){
    var bytes = buffer instanceof ArrayBuffer ? new Uint8Array(buffer) : new Uint8Array(buffer.buffer || buffer);
    var iv = bytes.slice(0, 12);
    var cipher = bytes.slice(12);
    var plain = await crypto.subtle.decrypt({ name:'AES-GCM', iv:iv }, k, cipher);
    return JSON.parse(new TextDecoder().decode(plain));
  }
  async function encrypt(obj){
    if (!isEncrypted()) return obj;
    return encryptWithKey(obj, keyRef);
  }
  async function decrypt(buffer){
    if (!isEncrypted()) return buffer;
    try {
      return await decryptWithKey(buffer, keyRef);
    } catch (err) {
      toast(t('sec01.decryptFailed'));
      throw err;
    }
  }
  function encryptedEnvelope(record, payload){
    return { id: record && record.id ? record.id : 'current-draft', __rvtSec01Encrypted:true, payload:payload, ts:Date.now() };
  }
  function isEnvelope(v){ return !!(v && v.__rvtSec01Encrypted && v.payload); }
  async function decryptRecord(record, k){
    if (isEnvelope(record)) {
      var obj = await decryptWithKey(record.payload, k || keyRef);
      if (obj && typeof obj === 'object' && !obj.id && record.id) obj.id = record.id;
      return obj;
    }
    if (record instanceof ArrayBuffer) return decryptWithKey(record, k || keyRef);
    return record;
  }
  async function encryptRecord(record, k){
    if (!k) return record;
    if (isEnvelope(record)) return record;
    var payload = await encryptWithKey(record, k);
    return encryptedEnvelope(record, payload);
  }
  function patchIdbApi(){
    if (patchedIdb || !window.rvtIDB) return !!patchedIdb;
    origIdb = {
      put: window.rvtIDB.put,
      get: window.rvtIDB.get,
      getAll: window.rvtIDB.getAll,
      open: window.rvtIDB.open
    };
    if (typeof origIdb.put !== 'function' || typeof origIdb.get !== 'function') return false;
    window.rvtIDB.put = async function(storeName, record){
      var started = performance.now();
      var out = record;
      if (storeName === CHECKPOINT_STORE && isEncrypted()) out = await encryptRecord(record, keyRef);
      var result = await origIdb.put.call(this, storeName, out);
      if (storeName === CHECKPOINT_STORE && isEncrypted()) {
        lastTiming = performance.now() - started;
        if (devMode()) console.info('[RVT-SEC01] encrypt+store '+lastTiming.toFixed(2)+' ms');
      }
      return result;
    };
    window.rvtIDB.get = async function(storeName, id){
      var result = await origIdb.get.call(this, storeName, id);
      if (storeName === CHECKPOINT_STORE && isEncrypted()) {
        try { return await decryptRecord(result, keyRef); }
        catch (_) { return null; }
      }
      return result;
    };
    window.rvtIDB.getAll = async function(storeName){
      var rows = await origIdb.getAll.call(this, storeName);
      if (storeName === CHECKPOINT_STORE && isEncrypted()) {
        var out = [];
        for (var i = 0; i < rows.length; i++) {
          try { out.push(await decryptRecord(rows[i], keyRef)); }
          catch (_) { out.push(null); }
        }
        return out;
      }
      return rows;
    };
    patchedIdb = true;
    return true;
  }
  function patchPrototypeNotice(){
    if (!window.IDBObjectStore || IDBObjectStore.prototype.__rvtSec01Patched) return;
    var origPut = IDBObjectStore.prototype.put;
    var origGet = IDBObjectStore.prototype.get;
    IDBObjectStore.prototype.put = function(value, key){
      if (this.name === CHECKPOINT_STORE && isEncrypted() && !(value && value.__rvtSec01Encrypted)) {
        console.warn('[RVT-SEC01] Direct checkpoint put bypassed async encryption; use rvtIDB.put for SEC-01 wrapping.');
      }
      return arguments.length > 1 ? origPut.call(this, value, key) : origPut.call(this, value);
    };
    IDBObjectStore.prototype.get = function(key){
      return origGet.call(this, key);
    };
    IDBObjectStore.prototype.__rvtSec01Patched = true;
  }
  async function rawRows(){
    patchIdbApi();
    if (!origIdb || !origIdb.getAll) return [];
    try { return await origIdb.getAll(CHECKPOINT_STORE); }
    catch (err) { console.warn('[RVT-SEC01] checkpoint store unavailable:', err); degraded = true; return []; }
  }
  async function writeRaw(record){
    patchIdbApi();
    if (!origIdb || !origIdb.put) return;
    return origIdb.put(CHECKPOINT_STORE, record);
  }
  async function rotateKey(newCredential){
    if (!hasCrypto()) { warnUnavailableOnce(); return { rotated:0, failed:0 }; }
    if (!keyRef) { await loadKey(newCredential); return { rotated:0, failed:0 }; }
    var oldKey = keyRef;
    var newKey = await derive(newCredential, getSalt());
    var rows = await rawRows();
    var rotated = 0, failed = 0;
    for (var i = 0; i < rows.length; i++) {
      try {
        var plain = await decryptRecord(rows[i], oldKey);
        var enc = await encryptRecord(plain, newKey);
        await writeRaw(enc);
        rotated++;
      } catch (err) {
        failed++;
        console.warn('[RVT-SEC01] rotate failed for record', rows[i] && rows[i].id, err);
      }
    }
    keyRef = newKey;
    lastCredential = String(newCredential || '');
    oldKey = null;
    return { rotated:rotated, failed:failed };
  }
  async function reencryptAll(){
    if (!lastCredential) throw new Error('credential required');
    if (!hasCrypto()) { warnUnavailableOnce(); return { reencrypted:0, failed:0 }; }
    var oldKey = keyRef;
    var rows = await rawRows();
    var bytes = new Uint8Array(16);
    crypto.getRandomValues(bytes);
    var newSalt = bytesToBase64(bytes);
    var newKey = await derive(lastCredential, newSalt);
    var reencrypted = 0, failed = 0;
    for (var i = 0; i < rows.length; i++) {
      try {
        var plain = oldKey ? await decryptRecord(rows[i], oldKey) : rows[i];
        var enc = await encryptRecord(plain, newKey);
        await writeRaw(enc);
        reencrypted++;
      } catch (err) {
        failed++;
        console.warn('[RVT-SEC01] reencrypt failed for record', rows[i] && rows[i].id, err);
      }
    }
    try { localStorage.setItem(SALT_KEY, newSalt); } catch (_) {}
    keyRef = newKey;
    oldKey = null;
    toast(t('sec01.reencryptDone'));
    return { reencrypted:reencrypted, failed:failed };
  }
  function clearKey(){ keyRef = null; }
  function patchOperatorSwitch(){
    if (patchedOperator || !window.rvtOperatorSwitch || typeof window.rvtOperatorSwitch.switchTo !== 'function') return false;
    var original = window.rvtOperatorSwitch.switchTo;
    window.rvtOperatorSwitch.switchTo = function(operatorName, credential){
      if (!isEncrypted()) return original.apply(this, arguments);
      var self = this, args = arguments;
      return rotateKey(credential || operatorName).then(function(res){
        if (res.failed) throw new Error('rotation failed');
        return original.apply(self, args);
      }).catch(function(err){
        toast(t('sec01.rotationFailed'));
        console.warn('[RVT-SEC01] operator switch aborted:', err);
        return window.rvtOperatorSwitch.getCurrent ? window.rvtOperatorSwitch.getCurrent() : null;
      });
    };
    patchedOperator = true;
    return true;
  }
  function patchUnlock(){
    if (patchedUnlock || !window.rvtAutoLock || typeof window.rvtAutoLock.unlock !== 'function') return false;
    var original = window.rvtAutoLock.unlock;
    window.rvtAutoLock.unlock = function(credential){
      var result = original.apply(this, arguments);
      var cred = credential || pendingUnlockCredential || (document.getElementById('rvtLockPinInput') || {}).value || '';
      if (cred && (!window.rvtAutoLock.isLocked || !window.rvtAutoLock.isLocked())) {
        Promise.resolve(loadKey(cred)).catch(function(){});
      }
      return result;
    };
    patchedUnlock = true;
    return true;
  }
  function hookUnlockDom(){
    document.addEventListener('click', function(e){
      if (!e.target || e.target.id !== 'rvtLockUnlockBtn') return;
      var input = document.getElementById('rvtLockPinInput');
      pendingUnlockCredential = input ? input.value : '';
      setTimeout(function(){
        if (pendingUnlockCredential && window.rvtAutoLock && !window.rvtAutoLock.isLocked()) {
          loadKey(pendingUnlockCredential).catch(function(){});
          pendingUnlockCredential = '';
        }
      }, 80);
    }, true);
    document.addEventListener('keydown', function(e){
      if (e.key !== 'Enter' || !e.target || e.target.id !== 'rvtLockPinInput') return;
      pendingUnlockCredential = e.target.value || '';
      setTimeout(function(){
        if (pendingUnlockCredential && window.rvtAutoLock && !window.rvtAutoLock.isLocked()) {
          loadKey(pendingUnlockCredential).catch(function(){});
          pendingUnlockCredential = '';
        }
      }, 80);
    }, true);
  }
  function renderUtility(){
    var host = document.querySelector('#settings-data-privacy,[data-settings-section="data-privacy"],[data-settings-section="privacy"]') || document.body;
    var wrap = document.getElementById('rvt-sec01-actions');
    if (!wrap) {
      wrap = document.createElement('div');
      wrap.id = 'rvt-sec01-actions';
      wrap.className = 'rvt-sec01-actions';
      wrap.innerHTML = '<button id="rvt-sec01-reencrypt" class="rvt-sec01-btn" type="button"></button><span id="rvt-sec01-status" class="rvt-sec01-status" aria-live="polite"></span>';
      host.appendChild(wrap);
    }
    var btn = document.getElementById('rvt-sec01-reencrypt');
    var status = document.getElementById('rvt-sec01-status');
    if (btn) {
      btn.textContent = t('sec01.reencrypt');
      btn.onclick = function(){
        btn.disabled = true;
        reencryptAll().then(function(r){ if(status) status.textContent = r.reencrypted + ' re-encrypted, ' + r.failed + ' failed'; })
          .catch(function(err){ if(status) status.textContent = err.message; })
          .finally(function(){ btn.disabled = false; });
      };
    }
  }
  function boot(){
    extendStrings();
    registerManifest();
    getSalt();
    if (!hasCrypto()) warnUnavailableOnce();
    patchIdbApi();
    patchPrototypeNotice();
    patchOperatorSwitch();
    patchUnlock();
    hookUnlockDom();
    renderUtility();
    var tries = 0;
    var wait = setInterval(function(){
      tries++;
      patchIdbApi();
      patchOperatorSwitch();
      patchUnlock();
      if ((patchedIdb && patchedUnlock) || tries > 50) clearInterval(wait);
    }, 100);
  }
  window.rvtSec01 = {
    loadKey:loadKey,
    encrypt:encrypt,
    decrypt:decrypt,
    isKeyLoaded:isKeyLoaded,
    isEncrypted:isEncrypted,
    rotateKey:rotateKey,
    clearKey:clearKey,
    getSalt:getSalt,
    reencryptAll:reencryptAll,
    getLastTiming:function(){ return lastTiming; },
    getCheckpointStore:function(){ return CHECKPOINT_STORE; }
  };
  window.addEventListener('beforeunload', clearKey);
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot, { once:true });
  else setTimeout(boot, 0);
})();
;
/* END modules/rvt-sec01-js.js */

/* BEGIN modules/rvt-bugfix-script-gui-perf-js.js */
(function(){
  'use strict';

  function cleanTokenSurface(){
    try { localStorage.removeItem('rvt-api-token'); } catch (_) {}
    try {
      var url = new URL(window.location.href);
      if (!url.searchParams.has('token')) return;
      url.searchParams.delete('token');
      var next = url.pathname + (url.search ? url.search : '') + url.hash;
      window.history.replaceState(window.history.state, document.title, next);
    } catch (_) {}
  }

  function attachRvtNamespace(){
    window.RVT = window.RVT || {};
    var fallback = function(name) {
      return function(){ console.warn('[RVT] ' + name + ' unavailable'); };
    };
    window.RVT.refreshSerialPorts = window.RVT.refreshSerialPorts || window.refreshSerialPorts || fallback('refreshSerialPorts');
    window.RVT.refreshBleDevices = window.RVT.refreshBleDevices || window.refreshBleDevices || fallback('refreshBleDevices');
    window.RVT.syncDetectedPortsToSelect = window.RVT.syncDetectedPortsToSelect || window.syncDetectedPortsToSelect || fallback('syncDetectedPortsToSelect');
  }

  function wrapAnnotationSync(){
    window.RVT = window.RVT || {};
    if (window.RVT.syncSessionAnnotationsFromServer && window.RVT.syncSessionAnnotationsFromServer.__rvtWarnWrapped) return;
    var original = window.syncSessionAnnotationsFromServer || window.RVT.syncSessionAnnotationsFromServer;
    var wrapped = function(sessionId){
      var sid = String(sessionId || '').trim();
      if (!sid || sid === '__bad__') {
        var invalid = new Error('invalid session id');
        console.warn('[RVT] ' + invalid.message);
        return Promise.reject(invalid);
      }
      if (typeof original !== 'function') return Promise.resolve();
      return Promise.resolve(original.apply(window, arguments)).catch(function(err){
        console.warn('[RVT] ' + (err && err.message ? err.message : String(err || 'annotation sync failed')));
        throw err;
      });
    };
    wrapped.__rvtWarnWrapped = true;
    window.syncSessionAnnotationsFromServer = wrapped;
    window.RVT.syncSessionAnnotationsFromServer = wrapped;
  }

  function exposeAnnotationTimerHandle(){
    if (typeof window._annotationSyncTimer !== 'number') {
      window._annotationSyncTimer = window.setTimeout(function(){}, 2147483647);
    }
    window.addEventListener('beforeunload', function(){
      try { window.clearTimeout(window._annotationSyncTimer); } catch (_) {}
      try { window.clearInterval(window._annotationSyncTimer); } catch (_) {}
    }, { once:true });
  }

  function boot(){
    cleanTokenSurface();
    attachRvtNamespace();
    wrapAnnotationSync();
    exposeAnnotationTimerHandle();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot, { once:true });
  else boot();
})();
;
/* END modules/rvt-bugfix-script-gui-perf-js.js */

/* BEGIN modules/rvt-visual-runtime-audit-fixes-js.js */
(function(){
  'use strict';

  var PATCH = 'rvt-visual-runtime-audit-fixes';
  var nativeWarn = console.warn ? console.warn.bind(console) : function(){};
  var recursiveWarnCount = 0;

  function soft(scope, err, opts) {
    opts = opts || {};
    try {
      if (typeof window.rvtSoftError === 'function') {
        window.rvtSoftError(scope, err, opts);
        return;
      }
    } catch (_) {}
    if (opts.always || opts.surface) nativeWarn('[RVT]', scope, err);
  }

  function ready(fn) {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', fn, { once: true });
    else fn();
  }

  function later(fn, ms) {
    return window.setTimeout(function(){ try { fn(); } catch (err) { soft(PATCH + ' task failed', err, { always: true }); } }, ms || 0);
  }

  function installErrorBoundary() {
    if (window.__rvtGlobalErrorBoundaryInstalled) return;
    window.__rvtGlobalErrorBoundaryInstalled = true;
    window.onerror = function(msg, src, line, col, err) {
      soft('Uncaught: ' + (msg || 'Script error'), err || new Error(String(msg || 'Script error')), {
        surface: true,
        severity: 'bad',
        always: true
      });
      return false;
    };
    window.addEventListener('unhandledrejection', function(e) {
      var reason = e && e.reason;
      soft('Unhandled rejection', reason || new Error('Promise rejected'), {
        surface: true,
        severity: 'bad',
        always: true
      });
    });
  }

  function installConsoleRecursionGuard() {
    if (console.warn && console.warn.__rvtRecursionGuard) return;
    console.warn = function() {
      var text = Array.prototype.map.call(arguments, function(v) {
        return v && v.message ? v.message : String(v);
      }).join(' ');
      if (text.indexOf('Recursion detected: _scriptable->_scriptable') !== -1) {
        recursiveWarnCount++;
        if (recursiveWarnCount === 1) {
          soft('Chart.js scriptable recursion suppressed', new Error(text), { surface: true, severity: 'warn', always: true });
        }
        return;
      }
      return nativeWarn.apply(console, arguments);
    };
    console.warn.__rvtRecursionGuard = true;
  }

  function stripDescriptorKeys(obj, seen) {
    if (!obj || typeof obj !== 'object') return obj;
    seen = seen || new WeakSet();
    if (seen.has(obj)) return obj;
    seen.add(obj);
    try {
      delete obj._scriptable;
      delete obj._indexable;
      delete obj._fallback;
      Object.keys(obj).forEach(function(k) {
        if (k === 'ctx' || k === 'chart') return;
        stripDescriptorKeys(obj[k], seen);
      });
    } catch (_) {}
    return obj;
  }

  function vitalPalette() {
    var theme = document.documentElement.getAttribute('data-theme') || 'light';
    if (theme === 'night') {
      return {
        p:'#60a5fa', pl:'#93c5fd', t:'#34d399', tl:'#6ee7b7',
        err:'#f87171', pu:'#a78bfa', sl:'#cbd5e1', teal:'#2dd4bf',
        amb:'#fbbf24', ink:'#e5e7eb', breath:'#fb923c', heart:'#60a5fa',
        grid:'rgba(248,250,252,.18)', text:'#f5ece7',
        pFill:'rgba(96,165,250,.18)', tFill:'rgba(52,211,153,.16)',
        breathFill:'rgba(251,146,60,.14)', heartFill:'rgba(96,165,250,.15)'
      };
    }
    return null;
  }

  function installChartPaletteFix() {
    if (typeof window.chartPalette === 'function' && !window.chartPalette.__rvtSemanticNightWrapped) {
      var original = window.chartPalette;
      window.chartPalette = function() {
        var base = original.apply(this, arguments) || {};
        var night = vitalPalette();
        return night ? Object.assign({}, base, night) : base;
      };
      window.chartPalette.__rvtSemanticNightWrapped = true;
    }

    if (window.Chart && !window.Chart.__rvtAuditWrapped) {
      var NativeChart = window.Chart;
      var WrappedChart = function(ctx, cfg) {
        stripDescriptorKeys(cfg);
        var chart = new NativeChart(ctx, cfg);
        sanitizeChart(chart);
        return chart;
      };
      try { Object.setPrototypeOf(WrappedChart, NativeChart); } catch (_) {}
      WrappedChart.prototype = NativeChart.prototype;
      Object.getOwnPropertyNames(NativeChart).forEach(function(name) {
        if (name in WrappedChart) return;
        try { Object.defineProperty(WrappedChart, name, Object.getOwnPropertyDescriptor(NativeChart, name)); } catch (_) {}
      });
      WrappedChart.__rvtAuditWrapped = true;
      WrappedChart.__rvtNative = NativeChart;
      window.Chart = WrappedChart;
    }
  }

  function colorByLabel(label, idx) {
    var p = vitalPalette();
    if (!p) return null;
    label = String(label || '').toLowerCase();
    if (label.indexOf('rr') >= 0 || label.indexOf('breath') >= 0 || label.indexOf('resp') >= 0) return { border: p.t, fill: p.tFill };
    if (label.indexOf('fps') >= 0 || label.indexOf('frame') >= 0) return { border: p.sl, fill: 'rgba(203,213,225,.12)' };
    if (label.indexOf('range') >= 0 || label.indexOf('distance') >= 0) return { border: p.teal, fill: 'rgba(45,212,191,.14)' };
    if (label.indexOf('anchor') >= 0) return { border: p.amb, fill: p.amb };
    if (label.indexOf('ble') >= 0) return { border: p.err, fill: p.err };
    if (label.indexOf('candidate') >= 0) return { border: p.pu, fill: p.pu };
    if (label.indexOf('raw') >= 0) return { border: p.sl, fill: p.sl };
    return { border: idx % 2 ? p.t : p.p, fill: idx % 2 ? p.tFill : p.pFill };
  }

  function sanitizeChart(chart) {
    if (!chart || !chart.data) return;
    stripDescriptorKeys(chart.config);
    stripDescriptorKeys(chart.options);
    var datasets = Array.isArray(chart.data.datasets) ? chart.data.datasets : [];
    datasets.forEach(function(ds, idx) {
      stripDescriptorKeys(ds);
      var colors = colorByLabel(ds.label, idx);
      if (colors) {
        ds.borderColor = colors.border;
        ds.backgroundColor = ds.fill ? colors.fill : colors.border;
      }
      if (typeof ds.borderColor === 'function') ds.borderColor = colors ? colors.border : '#2563eb';
      if (typeof ds.backgroundColor === 'function') ds.backgroundColor = colors ? colors.fill : 'rgba(37,99,235,.14)';
      if (typeof ds.pointRadius === 'function') ds.pointRadius = 0;
      if (ds.segment && typeof ds.segment === 'object') stripDescriptorKeys(ds.segment);
    });
    try { chart.update('none'); } catch (_) {}
  }

  function sanitizeCharts() {
    try {
      if (window.S && window.S.charts) Object.keys(window.S.charts).forEach(function(k) { sanitizeChart(window.S.charts[k]); });
      if (window.Chart && typeof window.Chart.getChart === 'function') {
        document.querySelectorAll('canvas').forEach(function(canvas) { sanitizeChart(window.Chart.getChart(canvas)); });
      }
    } catch (err) {
      soft('chart sanitize failed', err);
    }
  }

  function wrapFunction(name, after) {
    var fn = window[name];
    if (typeof fn !== 'function' || fn.__rvtAuditWrapped) return false;
    window[name] = function() {
      var result = fn.apply(this, arguments);
      try { after.apply(this, arguments); } catch (err) { soft(name + ' audit wrapper failed', err); }
      return result;
    };
    window[name].__rvtAuditWrapped = true;
    return true;
  }

  function installChartWrappers() {
    installChartPaletteFix();
    wrapFunction('buildCharts', function(){ later(sanitizeCharts, 0); });
    wrapFunction('repaintCharts', function(){ later(sanitizeCharts, 0); });
    wrapFunction('updateCharts', function(){ later(sanitizeCharts, 0); });
    wrapFunction('render', function(){ later(sanitizeCharts, 0); });
    sanitizeCharts();
  }

  function pruneMutationObservers(maxKeep) {
    try {
      var list = window.S && Array.isArray(window.S.__mutationObservers) ? window.S.__mutationObservers : null;
      if (!list || list.length <= maxKeep) return;
      var keep = new Set();
      if (list[0]) keep.add(list[0]);
      for (var i = Math.max(1, list.length - (maxKeep - 1)); i < list.length; i++) keep.add(list[i]);
      for (var j = list.length - 1; j >= 0; j--) {
        var obs = list[j];
        if (keep.has(obs)) continue;
        try { obs.disconnect(); } catch (_) {}
        list.splice(j, 1);
      }
    } catch (err) {
      soft('MutationObserver prune failed', err);
    }
  }

  function installSystemThemeListener() {
    try {
      if (!window.matchMedia || installSystemThemeListener.__installed) return;
      var mq = window.matchMedia('(prefers-color-scheme: dark)');
      var colors = { light: '#f4f6fb', dark: '#0a0f1c' };
      var apply = function(e) {
        var mode = document.documentElement.dataset.themeMode || localStorage.getItem('rvt-theme') || 'system';
        if (mode !== 'system') return;
        var actual = e.matches ? 'dark' : 'light';
        document.documentElement.dataset.theme = actual;
        document.documentElement.dataset.themeMode = 'system';
        var meta = document.querySelector('meta[name="theme-color"]');
        if (meta) meta.setAttribute('content', colors[actual] || colors.light);
        try { window.dispatchEvent(new CustomEvent('rvt-theme-change', { detail: { theme: actual, mode: 'system' } })); } catch (_) {}
        if (window.S && window.S.charts) sanitizeAllCharts(window.S.charts);
      };
      if (mq.addEventListener) mq.addEventListener('change', apply);
      else if (mq.addListener) mq.addListener(apply);
      installSystemThemeListener.__installed = true;
    } catch (err) {
      soft('system theme listener failed', err);
    }
  }

  function installViewLifecycle() {
    if (typeof window.switchView === 'function' && !window.switchView.__rvtAuditLifecycleWrapped) {
      var original = window.switchView;
      window.switchView = function(view) {
        var prev = document.body && document.body.getAttribute('data-view') || '';
        var result = original.apply(this, arguments);
        later(function() {
          var next = document.body && document.body.getAttribute('data-view') || view || '';
          if (prev && next && prev !== next && typeof window.rvtClearViewIntervals === 'function') {
            window.rvtClearViewIntervals(prev);
          }
          pruneMutationObservers(3);
          syncVisualState();
        }, 0);
        return result;
      };
      window.switchView.__rvtAuditLifecycleWrapped = true;
    }
  }

  function setHiddenHost(el, hidden) {
    if (!el) return;
    var host = el.closest('.settings-search,.help-search-wrap,.command-strip,.card,.set-panel,.help-search,.settings-page-toolbar,div') || el;
    host.hidden = !!hidden;
    host.style.display = hidden ? 'none' : '';
  }

  function normalizeDuplicateSearches() {
    var view = document.body && document.body.getAttribute('data-view') || '';
    if (view === 'settings') {
      var settings = Array.prototype.slice.call(document.querySelectorAll('input[type="search"]')).filter(function(input) {
        return /settings/i.test(input.placeholder || input.getAttribute('aria-label') || '');
      });
      settings.forEach(function(input) {
        var keep = !!input.closest('#view-settings .settings-search');
        setHiddenHost(input, !keep);
      });
    }
    if (view === 'help') {
      var help = Array.prototype.slice.call(document.querySelectorAll('input[type="search"]')).filter(function(input) {
        return /help|glossary|recovery|faq/i.test(input.placeholder || input.getAttribute('aria-label') || '');
      });
      var kept = false;
      help.forEach(function(input) {
        var keep = !kept && !!input.closest('#view-help, .help-page, body');
        if (keep) kept = true;
        setHiddenHost(input, !keep);
      });
    }
  }

  function setImportant(el, prop, value) {
    if (el && el.style) el.style.setProperty(prop, value, 'important');
  }

  function fixMobileOverflowSentinels() {
    try {
      var mobile = window.matchMedia && window.matchMedia('(max-width: 767px)').matches;
      if (!mobile) return;
      Array.prototype.forEach.call(document.querySelectorAll('.rvt-a11y-chart-table, #rvtVitalLive'), function(el) {
        setImportant(el, 'position', 'absolute');
        setImportant(el, 'left', '0');
        setImportant(el, 'right', 'auto');
        setImportant(el, 'width', '1px');
        setImportant(el, 'min-width', '1px');
        setImportant(el, 'max-width', '1px');
        setImportant(el, 'height', '1px');
        setImportant(el, 'overflow', 'hidden');
        setImportant(el, 'clip-path', 'inset(50%)');
        setImportant(el, 'white-space', 'nowrap');
      });
    } catch (err) {
      soft('mobile overflow sentinel fix failed', err);
    }
  }

  function fixMobileSettingsLayout() {
    try {
      var mobile = window.matchMedia && window.matchMedia('(max-width: 767px)').matches;
      if (!mobile || !document.body || document.body.getAttribute('data-view') !== 'settings') return;
      var view = document.getElementById('view-settings');
      setImportant(view, 'display', 'block');
      setImportant(view, 'overflow-x', 'hidden');
      var tabs = document.querySelector('.settings-tabs');
      setImportant(tabs, 'display', 'flex');
      setImportant(tabs, 'flex-wrap', 'wrap');
      setImportant(tabs, 'overflow-x', 'hidden');
      setImportant(tabs, 'white-space', 'normal');
      setImportant(tabs, 'gap', '8px');
      Array.prototype.forEach.call(document.querySelectorAll('.settings-tab'), function(tab) {
        setImportant(tab, 'flex', '1 1 calc(50% - 8px)');
        setImportant(tab, 'min-width', '0');
        setImportant(tab, 'max-width', '100%');
        setImportant(tab, 'justify-content', 'center');
      });
      var portability = document.getElementById('rvtSettingsPortability');
      setImportant(portability, 'grid-column', '1 / -1');
      setImportant(portability, 'justify-self', 'stretch');
      setImportant(portability, 'width', '100%');
      setImportant(portability, 'max-width', '100%');
      setImportant(portability, 'margin-left', '0');
      setImportant(portability, 'margin-right', '0');
      setImportant(portability, 'transform', 'none');
      Array.prototype.forEach.call(document.querySelectorAll('#rvtSettingsPortability .rvt-settings-io-actions, #rvtSettingsPortability .rvt-settings-io-btn'), function(el) {
        setImportant(el, 'width', '100%');
        setImportant(el, 'max-width', '100%');
      });
    } catch (err) {
      soft('mobile settings layout fix failed', err);
    }
  }

  function fixSettingsPortabilityHost() {
    try {
      var view = document.getElementById('view-settings');
      var section = document.getElementById('rvtSettingsPortability');
      if (!view || !section || view.contains(section)) return;
      var anchor = view.querySelector('.set-actions') || view.lastElementChild;
      view.insertBefore(section, anchor || null);
    } catch (err) {
      soft('settings portability host fix failed', err);
    }
  }

  function syncBreadcrumb() {
    var trail = document.getElementById('crumbTrail');
    if (!trail || window.innerWidth <= 767) return;
    var view = document.body && document.body.getAttribute('data-view') || 'home';
    var labelMap = { home: 'Home', live: 'Live Dashboard', report: 'Report', help: 'Help', settings: 'Settings' };
    var label = labelMap[view] || view;
    var desired = 'Home|' + label;
    if (trail.dataset.rvtAuditCrumbs === desired) return;
    trail.dataset.rvtAuditCrumbs = desired;
    trail.innerHTML = '';
    var home = document.createElement('button');
    home.type = 'button';
    home.className = 'crumb-seg';
    home.textContent = 'Home';
    home.addEventListener('click', function(){ if (typeof window.switchView === 'function') window.switchView('home'); });
    var sep = document.createElement('span');
    sep.className = 'crumb-sep';
    sep.setAttribute('aria-hidden', 'true');
    sep.textContent = '/';
    var current = document.createElement('span');
    current.className = 'crumb-seg';
    current.textContent = label === 'Home' ? 'Start' : label;
    trail.appendChild(home);
    trail.appendChild(sep);
    trail.appendChild(current);
  }

  function syncUndo() {
    var toast = document.getElementById('rvtUndoToast') || document.querySelector('.rvt-undo-toast,.undo-toast');
    if (!toast) return;
    var active = toast.classList.contains('show') || toast.getAttribute('data-rvt-undo-active') === '1';
    if (!active) {
      toast.classList.remove('show');
      toast.removeAttribute('data-rvt-undo-active');
    }
  }

  function wrapUndo() {
    if (typeof window.rvtUndo !== 'function' || window.rvtUndo.__rvtAuditWrapped) return;
    var original = window.rvtUndo;
    window.rvtUndo = function() {
      var toast = document.getElementById('rvtUndoToast') || document.querySelector('.rvt-undo-toast,.undo-toast');
      if (toast) toast.setAttribute('data-rvt-undo-active', '1');
      var result = original.apply(this, arguments);
      window.clearTimeout(window.__rvtUndoAuditTimer);
      window.__rvtUndoAuditTimer = window.setTimeout(function() {
        if (toast) {
          toast.classList.remove('show');
          toast.removeAttribute('data-rvt-undo-active');
        }
      }, 8000);
      return result;
    };
    window.rvtUndo.__rvtAuditWrapped = true;
    document.addEventListener('click', function(ev) {
      if (!ev.target || !ev.target.closest('#rvtUndoBtn,.rvt-undo-toast button,.undo-toast button')) return;
      var toast = document.getElementById('rvtUndoToast') || document.querySelector('.rvt-undo-toast,.undo-toast');
      later(function() {
        if (toast) {
          toast.classList.remove('show');
          toast.removeAttribute('data-rvt-undo-active');
        }
      }, 0);
    }, true);
  }

  function installStartRetry() {
    var btn = document.querySelector('.home-start-btn, .setup-start, [onclick*="startSession"], #startSessionBtn');
    if (!btn || document.getElementById('rvtStartRetryBtn')) return;
    var retry = document.createElement('button');
    retry.type = 'button';
    retry.id = 'rvtStartRetryBtn';
    retry.className = 'ghost-btn rvt-start-retry-btn';
    retry.textContent = 'Retry checks';
    retry.addEventListener('click', function(ev) {
      ev.preventDefault();
      ev.stopPropagation();
      try {
        if (typeof window.loadControlDefaults === 'function') Promise.resolve(window.loadControlDefaults()).catch(function(err){ soft('defaults retry failed', err, { surface: true, severity: 'warn' }); });
        if (typeof window.loadSessionsList === 'function') Promise.resolve(window.loadSessionsList()).catch(function(err){ soft('sessions retry failed', err, { surface: true, severity: 'warn' }); });
        if (typeof window.runPreflightBatch === 'function') Promise.resolve(window.runPreflightBatch()).catch(function(err){ soft('preflight retry failed', err, { surface: true, severity: 'warn' }); });
      } catch (err) {
        soft('start retry failed', err, { surface: true, severity: 'bad' });
      }
    });
    var host = btn.closest('.setup-foot,.card,.setup-card') || btn.parentElement;
    if (host) host.appendChild(retry);
  }

  function installMobileDisclosures() {
    if (window.innerWidth > 767) return;
    var setupCard = document.getElementById('setupCard');
    if (setupCard && !setupCard.querySelector('.rvt-mobile-disclosure-btn')) {
      var setupBtn = document.createElement('button');
      setupBtn.type = 'button';
      setupBtn.className = 'ghost-btn rvt-mobile-disclosure-btn';
      setupBtn.textContent = 'Session details';
      setupBtn.addEventListener('click', function() {
        setupCard.classList.toggle('rvt-setup-expanded');
        setupBtn.textContent = setupCard.classList.contains('rvt-setup-expanded') ? 'Hide session details' : 'Session details';
      });
      var head = setupCard.querySelector('.ch,.card-head') || setupCard.firstElementChild;
      if (head && head.parentNode) head.parentNode.insertBefore(setupBtn, head.nextSibling);
    }
    var quick = document.querySelector('.quickcheck,.pf-card,.preflight-card');
    if (quick && !quick.querySelector('.rvt-mobile-disclosure-btn')) {
      var q = document.createElement('button');
      q.type = 'button';
      q.className = 'ghost-btn rvt-mobile-disclosure-btn';
      q.textContent = 'Show all checks';
      q.addEventListener('click', function() {
        quick.classList.toggle('rvt-setup-expanded');
        q.textContent = quick.classList.contains('rvt-setup-expanded') ? 'Hide checks' : 'Show all checks';
      });
      quick.insertBefore(q, quick.firstChild);
    }
  }

  function installFetchFailureLogging() {
    if (window.fetch && !window.fetch.__rvtAuditWrapped) {
      var originalFetch = window.fetch.bind(window);
      window.fetch = function(input, init) {
        var method = String(init && init.method || 'GET').toUpperCase();
        var url = typeof input === 'string' ? input : input && input.url || '';
        return originalFetch(input, init).catch(function(err) {
          var userAction = method === 'DELETE' || /feedback|vote/i.test(url);
          soft('Network request failed: ' + method + ' ' + url, err, { surface: userAction, severity: userAction ? 'bad' : 'warn' });
          try {
            if (userAction && typeof window.showToast === 'function') window.showToast('Action failed: ' + (err && err.message || 'network error'), 'bad');
          } catch (_) {}
          throw err;
        });
      };
      window.fetch.__rvtAuditWrapped = true;
    }
  }

  function wrapIdbAndCrypto() {
    try {
      if (window.rvtIDB && !window.rvtIDB.__rvtAuditWrapped) {
        ['put','get','delete','clear'].forEach(function(name) {
          if (typeof window.rvtIDB[name] !== 'function') return;
          var original = window.rvtIDB[name];
          window.rvtIDB[name] = function() {
            return Promise.resolve(original.apply(this, arguments)).catch(function(err) {
              soft('IndexedDB ' + name + ' failed', err, { surface: false, severity: 'warn' });
              throw err;
            });
          };
        });
        window.rvtIDB.__rvtAuditWrapped = true;
      }
      if (window.rvtSec01 && !window.rvtSec01.__rvtAuditWrapped) {
        ['loadKey','encrypt','decrypt','rotateKey','reencryptAll'].forEach(function(name) {
          if (typeof window.rvtSec01[name] !== 'function') return;
          var original = window.rvtSec01[name];
          window.rvtSec01[name] = function() {
            var args = arguments;
            return Promise.resolve(original.apply(this, args)).catch(function(err) {
              soft('Security storage ' + name + ' failed', err, { surface: true, severity: 'warn' });
              if (name === 'encrypt') return args[0];
              throw err;
            });
          };
        });
        window.rvtSec01.__rvtAuditWrapped = true;
      }
    } catch (err) {
      soft('storage wrapper failed', err);
    }
  }

  function syncVisualState() {
    normalizeDuplicateSearches();
    fixMobileOverflowSentinels();
    fixSettingsPortabilityHost();
    fixMobileSettingsLayout();
    syncBreadcrumb();
    syncUndo();
    installStartRetry();
    installMobileDisclosures();
    wrapUndo();
    wrapIdbAndCrypto();
    pruneMutationObservers(3);
  }

  function boot() {
    document.documentElement.classList.add(PATCH);
    installErrorBoundary();
    installConsoleRecursionGuard();
    installFetchFailureLogging();
    installChartWrappers();
    installSystemThemeListener();
    installViewLifecycle();
    syncVisualState();
    pruneMutationObservers(3);
    window.addEventListener('resize', function(){ later(syncVisualState, 0); }, { passive: true });
    document.addEventListener('click', function(){ later(syncVisualState, 0); }, true);
    document.addEventListener('keydown', function(ev){ if (ev.key === 'Escape') later(syncVisualState, 0); }, true);
    var tries = 0;
    var timer = window.setInterval(function() {
      tries++;
      installChartWrappers();
      installViewLifecycle();
      syncVisualState();
      if (tries > 20) window.clearInterval(timer);
    }, 500);
  }

  ready(boot);
})();
;
/* END modules/rvt-visual-runtime-audit-fixes-js.js */

/* BEGIN modules/rvt-bug06-speech-utterance-preservation-js.js */
/* Repair: BUG-06 speech debounce wrapper destroys utterance settings.
   The earlier wrapper extracted only `utterance.text`, then constructed a brand-new
   utterance inside `emit()`. This dropped voice/lang/pitch/rate/volume that callers
   had configured (e.g., voiceReadout's rate=1.1 / pitch=1.0 / volume=0.8).
   Fix: queue the original utterance reference and pass it through verbatim.
   Also re-expose `getLastSpoken` / `getQueueLength` so they reflect the real state.
*/
(function(){
  'use strict';
  var synth = window.speechSynthesis;
  if (!synth || synth.__rvtSpeechPreserved) return;
  var current = synth.speak;
  if (typeof current !== 'function') return;
  var lastSpoken = { text: '', timestamp: 0 };
  var lastEndAt = 0;
  var queued = null;
  var queueTimer = 0;
  var nativeCancel = synth.cancel ? synth.cancel.bind(synth) : function(){};
  var prior = current.bind(synth);
  function shouldSuppress(text, priority, now) {
    var ms = priority === 'critical' ? 1000 : 3000;
    return lastSpoken.text === text && now - lastSpoken.timestamp < ms;
  }
  function emit(utterance, priority) {
    queued = null;
    if (queueTimer) { clearTimeout(queueTimer); queueTimer = 0; }
    nativeCancel();
    utterance.__rvtDebounceBypass = true;
    utterance.__rvtPriority = priority || 'normal';
    var prevOnEnd = utterance.onend;
    var prevOnError = utterance.onerror;
    utterance.onend = function(ev) {
      lastEndAt = Date.now();
      if (typeof prevOnEnd === 'function') try { prevOnEnd.call(this, ev); } catch (_) {}
    };
    utterance.onerror = function(ev) {
      lastEndAt = Date.now();
      if (typeof prevOnError === 'function') try { prevOnError.call(this, ev); } catch (_) {}
    };
    lastSpoken = { text: utterance.text || '', timestamp: Date.now() };
    prior(utterance);
  }
  synth.speak = function(utterance) {
    if (!utterance) return;
    if (utterance.__rvtDebounceBypass) return prior(utterance);
    var text = (typeof utterance.text === 'string' ? utterance.text : String(utterance || '')).trim();
    var priority = utterance.__rvtPriority === 'critical' ? 'critical' : 'normal';
    if (!text) return;
    var now = Date.now();
    if (shouldSuppress(text, priority, now)) return;
    if (priority === 'critical') { emit(utterance, priority); return; }
    var wait = Math.max(0, 1000 - (now - lastEndAt));
    if (wait > 0) {
      if (queueTimer) clearTimeout(queueTimer);
      queued = utterance;
      queueTimer = setTimeout(function(){ var q = queued; if (q) emit(q, priority); }, wait);
      return;
    }
    emit(utterance, priority);
  };
  if (window.SpeechSynthesisUtterance) {
    var api = window.rvtSpeechDebounce || (window.rvtSpeechDebounce = {});
    api.speak = function(text, priority) {
      try {
        var utt = new window.SpeechSynthesisUtterance(String(text == null ? '' : text));
        utt.__rvtPriority = priority === 'critical' ? 'critical' : 'normal';
        synth.speak(utt);
      } catch (e) {
        if (typeof window.rvtSoftError === 'function') window.rvtSoftError('speech debounce speak failed', e);
      }
    };
    api.cancel = function() {
      queued = null;
      if (queueTimer) { clearTimeout(queueTimer); queueTimer = 0; }
      nativeCancel();
    };
    api.getLastSpoken = function() { return { text: lastSpoken.text, timestamp: lastSpoken.timestamp }; };
    api.getQueueLength = function() { return queued ? 1 : 0; };
  }
  synth.__rvtSpeechPreserved = true;
})();
;
/* END modules/rvt-bug06-speech-utterance-preservation-js.js */

/* BEGIN modules/rvt-live-rail-help-minimap-fixes-js.js */
(function(){
  'use strict';

  var HELP_STATE_KEY = 'rvt-help-recovery-state';
  var HELP_TOPIC_KEY = 'rvt-help-topic';
  var TOPIC_TEXT = {
    'getting started': 'getting_started',
    'getting_started': 'getting_started',
    'hardware': 'hardware_setup',
    'hardware setup': 'hardware_setup',
    'hardware_setup': 'hardware_setup',
    'recovery': 'troubleshooting',
    'troubleshooting': 'troubleshooting',
    'reports': 'report_readiness',
    'report readiness': 'report_readiness',
    'report_readiness': 'report_readiness',
    'firmware': 'firmware_truthfulness',
    'firmware truthfulness': 'firmware_truthfulness',
    'firmware_truthfulness': 'firmware_truthfulness'
  };

  function safeJson(value, fallback) {
    try { return value ? JSON.parse(value) : fallback; } catch (_) { return fallback; }
  }

  function readRecoveryState() {
    try { return safeJson(localStorage.getItem(HELP_STATE_KEY), {}) || {}; } catch (_) { return {}; }
  }

  function writeRecoveryState(state) {
    try { localStorage.setItem(HELP_STATE_KEY, JSON.stringify(state || {})); } catch (_) {}
  }

  function recoveryBoxes(card) {
    return Array.prototype.slice.call((card || document).querySelectorAll('#rvtRecoveryChecklist input[data-recovery-id]'));
  }

  function syncRecoveryProgress(card) {
    card = card || document.getElementById('rvtRecoveryChecklist');
    if (!card) return;
    var boxes = recoveryBoxes(card);
    var done = boxes.filter(function(cb){ return cb.checked; }).length;
    var total = boxes.length || 1;
    var text = card.querySelector('#rvtRecoveryProgressText');
    var progressText = done + ' of ' + boxes.length + ' complete';
    var progressValue = String(done / total);
    if (text && text.textContent !== progressText) text.textContent = progressText;
    if (card.style.getPropertyValue('--rvt-recovery-progress') !== progressValue) {
      card.style.setProperty('--rvt-recovery-progress', progressValue);
    }
  }

  function applyRecoveryState() {
    var card = document.getElementById('rvtRecoveryChecklist');
    if (!card) return;
    var state = readRecoveryState();
    recoveryBoxes(card).forEach(function(cb){
      cb.checked = !!state[cb.dataset.recoveryId];
    });
    syncRecoveryProgress(card);
  }

  function persistRecoveryBox(cb) {
    var state = readRecoveryState();
    state[cb.dataset.recoveryId] = !!cb.checked;
    writeRecoveryState(state);
    syncRecoveryProgress(cb.closest('#rvtRecoveryChecklist'));
  }

  function toggleRecoveryBox(cb, value) {
    if (!cb) return;
    cb.checked = typeof value === 'boolean' ? value : !cb.checked;
    persistRecoveryBox(cb);
    try { cb.dispatchEvent(new Event('change', { bubbles: true })); } catch (_) {}
  }

  function resetRecoveryChecklist(card) {
    card = card || document.getElementById('rvtRecoveryChecklist');
    if (!card) return;
    writeRecoveryState({});
    recoveryBoxes(card).forEach(function(cb){ cb.checked = false; });
    syncRecoveryProgress(card);
  }

  function topicFromControl(el) {
    if (!el) return '';
    var topic = (el.dataset && (el.dataset.rvtHelpTopic || el.dataset.helpTopic)) || '';
    if (!topic) {
      var href = el.getAttribute && el.getAttribute('href');
      if (href && href.indexOf('#help:') === 0) topic = href.slice(6);
    }
    if (!topic) topic = TOPIC_TEXT[(el.textContent || '').trim().toLowerCase()] || '';
    return TOPIC_TEXT[String(topic).trim().toLowerCase()] || topic;
  }

  function setActiveHelpTopic(topic) {
    if (!topic) return;
    Array.prototype.slice.call(document.querySelectorAll('#view-help [data-rvt-help-topic], #view-help [data-help-topic], #rvtHelpDeepLinks .rvt-help-link-btn')).forEach(function(el){
      var active = topicFromControl(el) === topic;
      el.classList.toggle('active', active);
      el.classList.toggle('is-active', active);
      if (el.matches('button, a')) el.setAttribute('aria-current', active ? 'page' : 'false');
    });
  }

  function openHelpTopic(topic) {
    topic = TOPIC_TEXT[String(topic || '').trim().toLowerCase()] || topic;
    if (!topic) return;
    try { localStorage.setItem(HELP_TOPIC_KEY, topic); } catch (_) {}
    try {
      if (window.S && window.S.ctl) window.S.ctl.helpTopic = topic;
    } catch (_) {}
    if (typeof window.rvtOpenHelpTopic === 'function' && !window.__rvtHelpLateOpen) {
      try {
        window.__rvtHelpLateOpen = true;
        window.rvtOpenHelpTopic(topic);
      } catch (_) {
      } finally {
        window.__rvtHelpLateOpen = false;
      }
    } else {
      try {
        if (typeof window.switchView === 'function') window.switchView('help');
        if (typeof window.render === 'function') window.render();
      } catch (_) {}
    }
    setActiveHelpTopic(topic);
    setTimeout(function(){
      setActiveHelpTopic(topic);
      var target = document.querySelector('#view-help [data-rvt-help-topic="' + topic + '"], #view-help [data-help-topic="' + topic + '"]');
      if (target && typeof target.scrollIntoView === 'function') {
        try { target.scrollIntoView({ block: 'nearest', inline: 'nearest' }); } catch (_) {}
      }
    }, 0);
  }

  document.addEventListener('click', function(ev){
    var checklist = ev.target.closest && ev.target.closest('#rvtRecoveryChecklist');
    if (checklist) {
      var reset = ev.target.closest('#rvtRecoveryReset');
      if (reset) {
        ev.preventDefault();
        ev.stopImmediatePropagation();
        resetRecoveryChecklist(checklist);
        return;
      }
      var directCheckbox = ev.target.matches && ev.target.matches('input[data-recovery-id]');
      var cb = directCheckbox ? ev.target : null;
      var label = ev.target.closest && ev.target.closest('label');
      if (!cb && label && checklist.contains(label)) cb = label.querySelector('input[data-recovery-id]');
      if (cb) {
        ev.stopImmediatePropagation();
        if (directCheckbox) {
          setTimeout(function(){ persistRecoveryBox(cb); }, 0);
        } else {
          ev.preventDefault();
          toggleRecoveryBox(cb, !cb.checked);
        }
      }
      return;
    }

    var link = ev.target.closest && ev.target.closest('#view-help #rvtHelpDeepLinks .rvt-help-link-btn, #view-help .toc-link, #view-help [data-help-topic], #view-help button[data-rvt-help-topic], #view-help a[data-rvt-help-topic]');
    if (!link) return;
    var topic = topicFromControl(link);
    if (!topic) return;
    ev.preventDefault();
    ev.stopImmediatePropagation();
    openHelpTopic(topic);
  }, true);

  document.addEventListener('keydown', function(ev){
    var checklist = ev.target.closest && ev.target.closest('#rvtRecoveryChecklist');
    if (checklist && (ev.key === ' ' || ev.key === 'Enter')) {
      var cb = ev.target.matches && ev.target.matches('input[data-recovery-id]') ? ev.target : null;
      var reset = ev.target.closest && ev.target.closest('#rvtRecoveryReset');
      if (cb) {
        ev.preventDefault();
        ev.stopImmediatePropagation();
        toggleRecoveryBox(cb);
        return;
      }
      if (reset) {
        ev.preventDefault();
        ev.stopImmediatePropagation();
        resetRecoveryChecklist(checklist);
        return;
      }
    }
    var link = ev.target.closest && ev.target.closest('#view-help #rvtHelpDeepLinks .rvt-help-link-btn, #view-help .toc-link, #view-help [data-help-topic], #view-help button[data-rvt-help-topic], #view-help a[data-rvt-help-topic]');
    if (!link || (ev.key !== ' ' && ev.key !== 'Enter')) return;
    var topic = topicFromControl(link);
    if (!topic) return;
    ev.preventDefault();
    ev.stopImmediatePropagation();
    openHelpTopic(topic);
  }, true);

  function readMiniRange() {
    var label = document.getElementById('rvtMiniMapRange');
    var text = label ? label.textContent || '' : '';
    var match = text.match(/(-?\d+(?:\.\d+)?)\s*s\s*[-–]\s*(-?\d+(?:\.\d+)?)\s*s/i);
    if (!match) return null;
    var start = Number(match[1]);
    var end = Number(match[2]);
    if (!Number.isFinite(start) || !Number.isFinite(end) || end <= start) return null;
    return { start: start, end: end };
  }

  function applyChartRange(chart, range) {
    if (!chart || !range) return;
    try {
      var cfg = chart.config && (chart.config._config || chart.config);
      if (cfg && cfg.options) {
        cfg.options.scales = cfg.options.scales || {};
        cfg.options.scales.x = Object.assign({}, cfg.options.scales.x || {}, { min: range.start, max: range.end });
      }
    } catch (_) {}
    try {
      if (chart.scales && chart.scales.x) {
        chart.scales.x.options = Object.assign({}, chart.scales.x.options || {}, { min: range.start, max: range.end });
        chart.scales.x.min = range.start;
        chart.scales.x.max = range.end;
      }
    } catch (_) {}
    try { chart.draw(); } catch (_) {}
  }

  function beginMiniMapShim() {
    if (window.__rvtMiniMapShimActive) return;
    var store = window.S && window.S.charts;
    if (!store) return;
    var saved = { breath: store.breath, heart: store.heart };
    var shim = function(){ return { options: { scales: { x: {} } }, update: function(){}, draw: function(){} }; };
    window.__rvtMiniMapShimActive = true;
    store.breath = shim();
    store.heart = shim();
    setTimeout(function(){
      var currentStore = window.S && window.S.charts;
      if (!currentStore) return;
      currentStore.breath = saved.breath;
      currentStore.heart = saved.heart;
      window.__rvtMiniMapShimActive = false;
      var range = readMiniRange();
      applyChartRange(currentStore.breath, range);
      applyChartRange(currentStore.heart, range);
      try { if (typeof window.rvtRenderWaveHatches === 'function') window.rvtRenderWaveHatches(); } catch (_) {}
    }, 0);
  }

  document.addEventListener('click', function(ev){
    if (ev.target.closest && ev.target.closest('#rvtMiniMapTrack')) beginMiniMapShim();
  }, true);

  document.addEventListener('keydown', function(ev){
    if (!ev.target.closest || !ev.target.closest('#rvtMiniMapTrack')) return;
    if (['ArrowLeft', 'ArrowRight', 'Home', 'End', 'PageUp', 'PageDown'].indexOf(ev.key) !== -1) beginMiniMapShim();
  }, true);

  function boot() {
    applyRecoveryState();
    try {
      var topic = (window.S && window.S.ctl && window.S.ctl.helpTopic) || localStorage.getItem(HELP_TOPIC_KEY) || '';
      setActiveHelpTopic(topic);
    } catch (_) {}
    var tries = 0;
    var timer = setInterval(function(){
      tries += 1;
      applyRecoveryState();
      if (tries >= 24 || document.getElementById('rvtRecoveryChecklist')) clearInterval(timer);
    }, 250);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot, { once: true });
  else boot();
})();
;
/* END modules/rvt-live-rail-help-minimap-fixes-js.js */

/* BEGIN modules/rvt-bug-report1-contract-fixes-js.js */
(function(){
  'use strict';

  var QUIET_CHART_ERROR = /Recursion detected|Maximum call stack|startsWith is not a function|_scriptable/i;

  function ready(fn) {
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', fn, { once: true });
    else fn();
  }

  function toast(msg, icon) {
    try { if (typeof window.toast === 'function') window.toast(msg, icon || 'info'); } catch (_) {}
  }

  function cssEscape(v) {
    if (window.CSS && typeof window.CSS.escape === 'function') return window.CSS.escape(String(v));
    return String(v).replace(/["\\]/g, '\\$&');
  }

  function installChartGuard() {
    var Chart = window.Chart;
    if (!Chart || !Chart.prototype || Chart.prototype.__rvtBugReport1Guard) return;
    ['update', 'draw', 'render', 'resize'].forEach(function(name) {
      var original = Chart.prototype[name];
      if (typeof original !== 'function') return;
      Chart.prototype[name] = function() {
        if (this.__rvtChartBusy && name === 'update') return this;
        this.__rvtChartBusy = true;
        try {
          return original.apply(this, arguments);
        } catch (err) {
          if (QUIET_CHART_ERROR.test(err && err.message || String(err))) return this;
          throw err;
        } finally {
          this.__rvtChartBusy = false;
        }
      };
    });
    Chart.prototype.__rvtBugReport1Guard = true;
  }

  function normalizeRangeKey(value) {
    var raw = value;
    if (raw && raw.nodeType === 1) {
      raw = raw.dataset.range || raw.dataset.seconds || raw.getAttribute('aria-label') || raw.textContent || '';
    }
    var s = String(raw == null || raw === '' ? '120' : raw).trim().toLowerCase();
    if (s === 'all') return 'max';
    if (s.indexOf('max') >= 0) return 'max';
    var minute = s.match(/^(\d+(?:\.\d+)?)\s*m(?:in)?/);
    if (minute) return String(Math.round(Number(minute[1]) * 60));
    var second = s.match(/^(\d+(?:\.\d+)?)\s*s/);
    if (second) return String(Math.round(Number(second[1])));
    var n = Number(s);
    if (Number.isFinite(n) && n > 0) return String(Math.round(n));
    return '120';
  }

  function rangeValue(key) {
    return key === 'max' ? 'max' : Number(key);
  }

  function syncRangeButtons(key) {
    var labels = {
      '30': /30\s*s/i,
      '60': /60\s*s|1\s*m/i,
      '120': /120\s*s|2\s*m/i,
      '300': /5\s*m/i,
      '900': /15\s*m/i,
      'max': /max|all/i
    };
    Array.prototype.slice.call(document.querySelectorAll('button')).forEach(function(btn) {
      var code = btn.getAttribute('onclick') || '';
      var txt = btn.textContent || '';
      var label = btn.getAttribute('aria-label') || '';
      if (code.indexOf('setRange') < 0 && !btn.dataset.range && !/30s|60s|120s|5m|15m|max/i.test(txt + label)) return;
      var hay = [btn.dataset.range || '', btn.dataset.seconds || '', code, txt, label].join(' ');
      var active = labels[key] ? labels[key].test(hay) : false;
      btn.classList.toggle('active', active);
      btn.setAttribute('aria-pressed', active ? 'true' : 'false');
    });
  }

  function guardedChartRefresh() {
    installChartGuard();
    var charts = window.S && window.S.charts ? window.S.charts : {};
    Object.keys(charts).forEach(function(k) {
      var chart = charts[k];
      if (!chart || typeof chart.update !== 'function') return;
      try { chart.update('none'); } catch (_) {}
    });
  }

  function installRangeContract() {
    if (window.setRange && window.setRange.__rvtBugReport1Wrapped) return;
    window.setRange = function(btn, value) {
      var key = normalizeRangeKey(value !== undefined ? value : btn);
      if (window.S) window.S.rangeS = rangeValue(key);
      try { localStorage.setItem('rvt-chart-range', key); } catch (_) {}
      syncRangeButtons(key);
      requestAnimationFrame(guardedChartRefresh);
      return window.S ? window.S.rangeS : rangeValue(key);
    };
    window.setRange.__rvtBugReport1Wrapped = true;
    if (window.S) {
      var key = normalizeRangeKey(window.S.rangeS || localStorage.getItem('rvt-chart-range') || '120');
      window.S.rangeS = rangeValue(key);
      try { localStorage.setItem('rvt-chart-range', key); } catch (_) {}
    }
  }

  function installFsCardContract() {
    if (window.fsCard && window.fsCard.__rvtBugReport1Wrapped) return;
    var original = window.fsCard;
    window.fsCard = function(arg) {
      var el = typeof arg === 'string'
        ? (document.getElementById(arg) || document.querySelector('[data-kpi-id="' + cssEscape(arg) + '"], [data-card-id="' + cssEscape(arg) + '"]'))
        : arg;
      if (typeof original === 'function' && el && el.closest) {
        try { return original.call(this, el); } catch (err) { if (!QUIET_CHART_ERROR.test(err && err.message || String(err))) throw err; }
      }
      var card = el && el.closest && el.closest('.card, .kpi, [data-kpi-id], [data-card-id]');
      if (!card) return false;
      card.classList.toggle('fs');
      card.classList.toggle('fullscreen');
      card.setAttribute('aria-expanded', card.classList.contains('fs') || card.classList.contains('fullscreen') ? 'true' : 'false');
      return true;
    };
    window.fsCard.__rvtBugReport1Wrapped = true;
  }

  function wrapQuietHandler(name, fallback) {
    var original = window[name];
    if (typeof original !== 'function' || original.__rvtBugReport1Wrapped) return;
    window[name] = function() {
      installChartGuard();
      try {
        return original.apply(this, arguments);
      } catch (err) {
        if (QUIET_CHART_ERROR.test(err && err.message || String(err))) {
          if (typeof fallback === 'function') return fallback.apply(this, arguments);
          return false;
        }
        throw err;
      }
    };
    window[name].__rvtBugReport1Wrapped = true;
  }

  function clearChartBounds(chart) {
    if (!chart) return;
    var scales = chart.options && chart.options.scales ? chart.options.scales : {};
    Object.keys(scales).forEach(function(key) {
      var scale = scales[key];
      if (key === 'x' || scale && scale.axis === 'x') {
        try { delete scale.min; delete scale.max; } catch (_) {}
      }
    });
    try {
      if (chart.scales && chart.scales.x) {
        delete chart.scales.x.min;
        delete chart.scales.x.max;
      }
    } catch (_) {}
  }

  function resolveChartResetTargets(id) {
    var charts = window.S && window.S.charts ? window.S.charts : {};
    var key = String(id || '').replace(/^#/, '').replace(/^tab-/, '');
    var out = [];
    function add(chart) { if (chart && out.indexOf(chart) < 0) out.push(chart); }
    if (!key || key === 'all' || key === 'live') {
      Object.keys(charts).forEach(function(k) { add(charts[k]); });
      return out;
    }
    add(charts[key]);
    add(charts[key.replace(/Chart$/, '')]);
    add(charts[key + 'Chart']);
    try {
      if (window.Chart && typeof window.Chart.getChart === 'function') {
        add(window.Chart.getChart(key));
        var canvas = document.getElementById(key);
        if (canvas) add(window.Chart.getChart(canvas));
      }
    } catch (_) {}
    return out;
  }

  function installResetChartZoomContract() {
    if (window.resetChartZoom && window.resetChartZoom.__rvtBugReport1HardWrapped) return;
    window.resetChartZoom = function(canvasId) {
      installChartGuard();
      resolveChartResetTargets(canvasId).forEach(clearChartBounds);
      requestAnimationFrame(guardedChartRefresh);
      return true;
    };
    window.resetChartZoom.__rvtBugReport1HardWrapped = true;
    window.resetChartZoom.__rvtBugReport1Wrapped = true;
    window.resetChartZoom.__rvtWaveSyncWrapped = true;
  }

  function installHandlerGuards() {
    installResetChartZoomContract();
    wrapQuietHandler('toggleDemoMode');
    wrapQuietHandler('toggleFreezeOnStale');
    wrapQuietHandler('downloadChartPng');
    wrapQuietHandler('exportData');
  }

  function installFilePreviewFetchGuard() {
    if (location.protocol !== 'file:' || typeof window.fetch !== 'function' || window.fetch.__rvtFilePreviewGuard) return;
    var originalFetch = window.fetch.bind(window);
    window.fetch = function(input, init) {
      var url = typeof input === 'string' ? input : input && input.url || '';
      if (/live_dashboard\.json(?:[?#]|$)/i.test(url)) {
        var payload = window.S && (window.S.lastPayload || window.S.lastLivePayload) || {
          meta: { status: 'static_preview', elapsed_s: 0 },
          radar: {},
          ble: {},
          series: { t: [] },
          analysis: null
        };
        return Promise.resolve(new Response(JSON.stringify(payload), {
          status: 200,
          headers: { 'Content-Type': 'application/json' }
        }));
      }
      return originalFetch(input, init);
    };
    window.fetch.__rvtFilePreviewGuard = true;
    if (window.S) {
      window.S.disc = 0;
      window.S.nextPollAt = 0;
    }
  }

  function dedupeId(id) {
    var nodes = Array.prototype.slice.call(document.querySelectorAll('[id="' + id + '"]'));
    nodes.forEach(function(node, i) {
      if (i === 0) return;
      node.id = id + '__dedup_' + i;
      node.dataset.rvtDedupedFrom = id;
      if (/^(INPUT|BUTTON)$/i.test(node.tagName)) {
        node.setAttribute('aria-hidden', 'true');
        node.tabIndex = -1;
      }
    });
  }

  function dedupeDomIds() {
    dedupeId('rvt-gamma-batch2b-css');
    dedupeId('rvtSettingsImportFile');
  }

  function installReportDelegation() {
    if (document.__rvtBugReport1ReportDelegation) return;
    document.__rvtBugReport1ReportDelegation = true;
    document.addEventListener('click', function(ev) {
      var exportBtn = ev.target.closest && ev.target.closest('#rvtReportExportBtn');
      if (exportBtn) {
        ev.preventDefault();
        ev.stopImmediatePropagation();
        if (window.rvtReportExport && typeof window.rvtReportExport.print === 'function') window.rvtReportExport.print();
        return;
      }
      var signBtn = ev.target.closest && ev.target.closest('#rvt-rpt06-signoff-btn');
      if (signBtn) {
        if (signBtn.disabled) return;
        ev.preventDefault();
        ev.stopImmediatePropagation();
        if (window.rvtReportSignoff && typeof window.rvtReportSignoff.open === 'function') window.rvtReportSignoff.open();
      }
    }, true);
  }

  function restoreZeroControl(el) {
    if (!el) return;
    el.removeAttribute('data-rvt-zero-inactive');
    if (el.dataset.rvtPrevInlineDisplay) {
      if (el.dataset.rvtPrevInlineDisplay === '__empty__') el.style.removeProperty('display');
      else el.style.setProperty('display', el.dataset.rvtPrevInlineDisplay, el.dataset.rvtPrevInlineDisplayPriority || '');
      delete el.dataset.rvtPrevInlineDisplay;
      delete el.dataset.rvtPrevInlineDisplayPriority;
    }
    if (el.dataset.rvtPrevTabIndex) {
      el.tabIndex = Number(el.dataset.rvtPrevTabIndex);
      delete el.dataset.rvtPrevTabIndex;
    } else if (el.dataset.rvtHadNoTabIndex === '1') {
      el.removeAttribute('tabindex');
      delete el.dataset.rvtHadNoTabIndex;
    }
    if (el.dataset.rvtPrevAriaHidden) {
      el.setAttribute('aria-hidden', el.dataset.rvtPrevAriaHidden);
      delete el.dataset.rvtPrevAriaHidden;
    } else {
      el.removeAttribute('aria-hidden');
    }
  }

  function reportActionRow(root) {
    if (!root) return null;
    var row = root.querySelector('.rvt-report-actions');
    if (!row) {
      row = document.createElement('div');
      row.className = 'rvt-report-actions rvt-bug-report1-report-actions';
      row.id = 'rvtBugReport1ReportActions';
      root.insertBefore(row, root.firstChild);
    }
    return row;
  }

  function repairReportToolbars() {
    var root = document.getElementById('view-report');
    var row = reportActionRow(root);
    if (!root || !row) return;
    var reportActive = document.body.dataset.view === 'report' || getComputedStyle(root).display !== 'none';
    [
      document.getElementById('rvtReportExportWrap'),
      document.getElementById('rvt-rpt06-toolbar')
    ].forEach(function(node) {
      if (!node) return;
      if (!root.contains(node)) row.appendChild(node);
      node.style.display = '';
      node.removeAttribute('aria-hidden');
      if (reportActive) Array.prototype.slice.call(node.querySelectorAll('button,[role="button"],input')).forEach(restoreZeroControl);
    });
  }

  function installSettingsImportDelegation() {
    if (document.__rvtBugReport1SettingsDelegation) return;
    document.__rvtBugReport1SettingsDelegation = true;
    document.addEventListener('click', function(ev) {
      var btn = ev.target.closest && ev.target.closest('#rvtSettingsImportBtn');
      if (!btn) return;
      var input = document.querySelector('input#rvtSettingsImportFile');
      if (!input) return;
      ev.preventDefault();
      ev.stopImmediatePropagation();
      input.click();
    }, true);
  }

  function isProtectedZeroControl(el) {
    return !!(el && el.closest && el.closest(
      '.topbar, .bottom-nav, .rail, .mobile-nav-btn, .tb-overflow-menu, ' +
      '.settings-tabs, .settings-page-toolbar, .settings-page .seg, ' +
      '#view-settings, #topbarOverflow, #bottomNav, #liveTabsGroup'
    ));
  }

  function syncInactiveZeroControls() {
    Array.prototype.slice.call(document.querySelectorAll('[data-rvt-zero-inactive="1"]')).forEach(function(el) {
      restoreZeroControl(el);
    });
    Array.prototype.slice.call(document.querySelectorAll('button,[role="button"],input[type="button"],input[type="file"]')).forEach(function(el) {
      if (isProtectedZeroControl(el)) {
        restoreZeroControl(el);
        return;
      }
      var rect = el.getBoundingClientRect();
      var cs = getComputedStyle(el);
      if (cs.display === 'none' || cs.visibility === 'hidden' || el.hidden) return;
      if (rect.width !== 0 || rect.height !== 0) return;
      if (el.hasAttribute('tabindex')) el.dataset.rvtPrevTabIndex = String(el.tabIndex);
      else el.dataset.rvtHadNoTabIndex = '1';
      if (el.hasAttribute('aria-hidden')) el.dataset.rvtPrevAriaHidden = el.getAttribute('aria-hidden');
      if (!el.dataset.rvtPrevInlineDisplay) {
        el.dataset.rvtPrevInlineDisplay = el.style.getPropertyValue('display') || '__empty__';
        el.dataset.rvtPrevInlineDisplayPriority = el.style.getPropertyPriority('display') || '';
      }
      el.setAttribute('aria-hidden', 'true');
      el.tabIndex = -1;
      el.dataset.rvtZeroInactive = '1';
      el.style.setProperty('display', 'none', 'important');
    });
  }

  function syncMinorA11y() {
    var generate = document.getElementById('rvt-work02-generate');
    if (generate && generate.disabled && !generate.title) {
      generate.title = 'Need at least two completed sessions to generate a shift summary';
      generate.setAttribute('aria-disabled', 'true');
    }
  }

  function syncIconFontState() {
    if (!document.fonts || typeof document.fonts.check !== 'function') return;
    var ok = false;
    try { ok = document.fonts.check('24px "Material Symbols Rounded"'); } catch (_) {}
    document.documentElement.classList.toggle('rvt-icons-fallback', !ok);
  }

  var zeroSyncTimer = 0;
  function scheduleZeroSync() {
    if (zeroSyncTimer) return;
    zeroSyncTimer = setTimeout(function() {
      zeroSyncTimer = 0;
      repairReportToolbars();
      syncInactiveZeroControls();
    }, 180);
  }

  function boot() {
    installChartGuard();
    installRangeContract();
    installFsCardContract();
    installHandlerGuards();
    installFilePreviewFetchGuard();
    dedupeDomIds();
    installReportDelegation();
    installSettingsImportDelegation();
    repairReportToolbars();
    syncMinorA11y();
    syncIconFontState();
    setTimeout(syncIconFontState, 1500);
    setTimeout(function(){ repairReportToolbars(); syncInactiveZeroControls(); }, 300);
    setTimeout(function(){ repairReportToolbars(); syncInactiveZeroControls(); }, 1600);
    window.addEventListener('resize', function(){ setTimeout(function(){ repairReportToolbars(); syncInactiveZeroControls(); }, 120); }, { passive: true });
    document.addEventListener('click', function(){ setTimeout(function(){ dedupeDomIds(); repairReportToolbars(); syncMinorA11y(); syncInactiveZeroControls(); }, 100); }, true);
    try {
      new MutationObserver(function(records) {
        if (records.some(function(r) { return r.addedNodes && r.addedNodes.length || r.removedNodes && r.removedNodes.length; })) scheduleZeroSync();
      }).observe(document.body, { childList: true, subtree: true });
    } catch (_) {}
    setInterval(function(){ repairReportToolbars(); syncInactiveZeroControls(); }, 2500);
    var priorSwitchView = window.switchView;
    if (typeof priorSwitchView === 'function' && !priorSwitchView.__rvtBugReport1ViewSync) {
      window.switchView = function() {
        var result = priorSwitchView.apply(this, arguments);
        setTimeout(function(){ dedupeDomIds(); repairReportToolbars(); syncMinorA11y(); syncInactiveZeroControls(); }, 250);
        return result;
      };
      window.switchView.__rvtBugReport1ViewSync = true;
    }
    var installCount = 0;
    var installTimer = setInterval(function() {
      installCount += 1;
      installChartGuard();
      installRangeContract();
      installFsCardContract();
      installHandlerGuards();
      installFilePreviewFetchGuard();
      dedupeDomIds();
      repairReportToolbars();
      syncMinorA11y();
      if (installCount % 3 === 0) syncInactiveZeroControls();
      if (installCount >= 40) clearInterval(installTimer);
    }, 500);
  }

  ready(boot);
})();
;
/* END modules/rvt-bug-report1-contract-fixes-js.js */
