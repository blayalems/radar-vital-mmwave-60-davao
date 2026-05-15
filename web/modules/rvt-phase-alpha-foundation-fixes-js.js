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
