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
      var header = host.querySelector('.set-h');
      if (header && header.nextSibling) host.insertBefore(bar, header.nextSibling);
      else host.insertBefore(bar, host.firstChild);
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
    var actions = host.querySelector('.set-actions');
    if (actions) host.insertBefore(card, actions);
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
