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
