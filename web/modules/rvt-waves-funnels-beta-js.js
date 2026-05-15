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
