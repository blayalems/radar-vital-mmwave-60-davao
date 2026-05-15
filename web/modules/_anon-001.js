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
      '<link href="/fonts/rvt-fonts.css" rel="stylesheet">' +
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
