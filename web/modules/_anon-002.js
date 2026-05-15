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

  document.getElementById('rvtReconnectRetryBtn').addEventListener('click', function() {
    _reconnectAttempts = 0;
    _reconnectGivenUp = false;
    document.getElementById('rvtReconnectFailed').classList.remove('show');
    if (typeof window.startConnection === 'function') {
      try { window.startConnection(); } catch(_){}
    } else if (typeof window.reconnect === 'function') {
      try { window.reconnect(); } catch(_){}
    }
  });

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
    kpiZoomMetric = metric;
    document.getElementById('rvtKpiZoomTitle').textContent = label;
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
    kpiZoomOverlay.classList.remove('show');
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

  document.getElementById('rvtKpiZoomClose').addEventListener('click', closeKpiZoom);
  kpiZoomOverlay.addEventListener('click', function(e) {
    if (e.target === kpiZoomOverlay) closeKpiZoom();
  });
  document.addEventListener('keydown', function(e) {
    if (e.key === 'Escape' && kpiZoomOverlay.classList.contains('show')) closeKpiZoom();
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
