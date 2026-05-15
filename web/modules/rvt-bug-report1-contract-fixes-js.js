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
