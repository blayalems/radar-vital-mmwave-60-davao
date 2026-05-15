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
