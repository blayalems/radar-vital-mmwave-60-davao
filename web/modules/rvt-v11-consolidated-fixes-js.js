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
