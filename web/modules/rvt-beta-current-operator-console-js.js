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
