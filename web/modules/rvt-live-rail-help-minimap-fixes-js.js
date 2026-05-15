(function(){
  'use strict';

  var HELP_STATE_KEY = 'rvt-help-recovery-state';
  var HELP_TOPIC_KEY = 'rvt-help-topic';
  var TOPIC_TEXT = {
    'getting started': 'getting_started',
    'getting_started': 'getting_started',
    'hardware': 'hardware_setup',
    'hardware setup': 'hardware_setup',
    'hardware_setup': 'hardware_setup',
    'recovery': 'troubleshooting',
    'troubleshooting': 'troubleshooting',
    'reports': 'report_readiness',
    'report readiness': 'report_readiness',
    'report_readiness': 'report_readiness',
    'firmware': 'firmware_truthfulness',
    'firmware truthfulness': 'firmware_truthfulness',
    'firmware_truthfulness': 'firmware_truthfulness'
  };

  function safeJson(value, fallback) {
    try { return value ? JSON.parse(value) : fallback; } catch (_) { return fallback; }
  }

  function readRecoveryState() {
    try { return safeJson(localStorage.getItem(HELP_STATE_KEY), {}) || {}; } catch (_) { return {}; }
  }

  function writeRecoveryState(state) {
    try { localStorage.setItem(HELP_STATE_KEY, JSON.stringify(state || {})); } catch (_) {}
  }

  function recoveryBoxes(card) {
    return Array.prototype.slice.call((card || document).querySelectorAll('#rvtRecoveryChecklist input[data-recovery-id]'));
  }

  function syncRecoveryProgress(card) {
    card = card || document.getElementById('rvtRecoveryChecklist');
    if (!card) return;
    var boxes = recoveryBoxes(card);
    var done = boxes.filter(function(cb){ return cb.checked; }).length;
    var total = boxes.length || 1;
    var text = card.querySelector('#rvtRecoveryProgressText');
    var progressText = done + ' of ' + boxes.length + ' complete';
    var progressValue = String(done / total);
    if (text && text.textContent !== progressText) text.textContent = progressText;
    if (card.style.getPropertyValue('--rvt-recovery-progress') !== progressValue) {
      card.style.setProperty('--rvt-recovery-progress', progressValue);
    }
  }

  function applyRecoveryState() {
    var card = document.getElementById('rvtRecoveryChecklist');
    if (!card) return;
    var state = readRecoveryState();
    recoveryBoxes(card).forEach(function(cb){
      cb.checked = !!state[cb.dataset.recoveryId];
    });
    syncRecoveryProgress(card);
  }

  function persistRecoveryBox(cb) {
    var state = readRecoveryState();
    state[cb.dataset.recoveryId] = !!cb.checked;
    writeRecoveryState(state);
    syncRecoveryProgress(cb.closest('#rvtRecoveryChecklist'));
  }

  function toggleRecoveryBox(cb, value) {
    if (!cb) return;
    cb.checked = typeof value === 'boolean' ? value : !cb.checked;
    persistRecoveryBox(cb);
    try { cb.dispatchEvent(new Event('change', { bubbles: true })); } catch (_) {}
  }

  function resetRecoveryChecklist(card) {
    card = card || document.getElementById('rvtRecoveryChecklist');
    if (!card) return;
    writeRecoveryState({});
    recoveryBoxes(card).forEach(function(cb){ cb.checked = false; });
    syncRecoveryProgress(card);
  }

  function topicFromControl(el) {
    if (!el) return '';
    var topic = (el.dataset && (el.dataset.rvtHelpTopic || el.dataset.helpTopic)) || '';
    if (!topic) {
      var href = el.getAttribute && el.getAttribute('href');
      if (href && href.indexOf('#help:') === 0) topic = href.slice(6);
    }
    if (!topic) topic = TOPIC_TEXT[(el.textContent || '').trim().toLowerCase()] || '';
    return TOPIC_TEXT[String(topic).trim().toLowerCase()] || topic;
  }

  function setActiveHelpTopic(topic) {
    if (!topic) return;
    Array.prototype.slice.call(document.querySelectorAll('#view-help [data-rvt-help-topic], #view-help [data-help-topic], #rvtHelpDeepLinks .rvt-help-link-btn')).forEach(function(el){
      var active = topicFromControl(el) === topic;
      el.classList.toggle('active', active);
      el.classList.toggle('is-active', active);
      if (el.matches('button, a')) el.setAttribute('aria-current', active ? 'page' : 'false');
    });
  }

  function openHelpTopic(topic) {
    topic = TOPIC_TEXT[String(topic || '').trim().toLowerCase()] || topic;
    if (!topic) return;
    try { localStorage.setItem(HELP_TOPIC_KEY, topic); } catch (_) {}
    try {
      if (window.S && window.S.ctl) window.S.ctl.helpTopic = topic;
    } catch (_) {}
    if (typeof window.rvtOpenHelpTopic === 'function' && !window.__rvtHelpLateOpen) {
      try {
        window.__rvtHelpLateOpen = true;
        window.rvtOpenHelpTopic(topic);
      } catch (_) {
      } finally {
        window.__rvtHelpLateOpen = false;
      }
    } else {
      try {
        if (typeof window.switchView === 'function') window.switchView('help');
        if (typeof window.render === 'function') window.render();
      } catch (_) {}
    }
    setActiveHelpTopic(topic);
    setTimeout(function(){
      setActiveHelpTopic(topic);
      var target = document.querySelector('#view-help [data-rvt-help-topic="' + topic + '"], #view-help [data-help-topic="' + topic + '"]');
      if (target && typeof target.scrollIntoView === 'function') {
        try { target.scrollIntoView({ block: 'nearest', inline: 'nearest' }); } catch (_) {}
      }
    }, 0);
  }

  document.addEventListener('click', function(ev){
    var checklist = ev.target.closest && ev.target.closest('#rvtRecoveryChecklist');
    if (checklist) {
      var reset = ev.target.closest('#rvtRecoveryReset');
      if (reset) {
        ev.preventDefault();
        ev.stopImmediatePropagation();
        resetRecoveryChecklist(checklist);
        return;
      }
      var directCheckbox = ev.target.matches && ev.target.matches('input[data-recovery-id]');
      var cb = directCheckbox ? ev.target : null;
      var label = ev.target.closest && ev.target.closest('label');
      if (!cb && label && checklist.contains(label)) cb = label.querySelector('input[data-recovery-id]');
      if (cb) {
        ev.stopImmediatePropagation();
        if (directCheckbox) {
          setTimeout(function(){ persistRecoveryBox(cb); }, 0);
        } else {
          ev.preventDefault();
          toggleRecoveryBox(cb, !cb.checked);
        }
      }
      return;
    }

    var link = ev.target.closest && ev.target.closest('#view-help #rvtHelpDeepLinks .rvt-help-link-btn, #view-help .toc-link, #view-help [data-help-topic], #view-help button[data-rvt-help-topic], #view-help a[data-rvt-help-topic]');
    if (!link) return;
    var topic = topicFromControl(link);
    if (!topic) return;
    ev.preventDefault();
    ev.stopImmediatePropagation();
    openHelpTopic(topic);
  }, true);

  document.addEventListener('keydown', function(ev){
    var checklist = ev.target.closest && ev.target.closest('#rvtRecoveryChecklist');
    if (checklist && (ev.key === ' ' || ev.key === 'Enter')) {
      var cb = ev.target.matches && ev.target.matches('input[data-recovery-id]') ? ev.target : null;
      var reset = ev.target.closest && ev.target.closest('#rvtRecoveryReset');
      if (cb) {
        ev.preventDefault();
        ev.stopImmediatePropagation();
        toggleRecoveryBox(cb);
        return;
      }
      if (reset) {
        ev.preventDefault();
        ev.stopImmediatePropagation();
        resetRecoveryChecklist(checklist);
        return;
      }
    }
    var link = ev.target.closest && ev.target.closest('#view-help #rvtHelpDeepLinks .rvt-help-link-btn, #view-help .toc-link, #view-help [data-help-topic], #view-help button[data-rvt-help-topic], #view-help a[data-rvt-help-topic]');
    if (!link || (ev.key !== ' ' && ev.key !== 'Enter')) return;
    var topic = topicFromControl(link);
    if (!topic) return;
    ev.preventDefault();
    ev.stopImmediatePropagation();
    openHelpTopic(topic);
  }, true);

  function readMiniRange() {
    var label = document.getElementById('rvtMiniMapRange');
    var text = label ? label.textContent || '' : '';
    var match = text.match(/(-?\d+(?:\.\d+)?)\s*s\s*[-–]\s*(-?\d+(?:\.\d+)?)\s*s/i);
    if (!match) return null;
    var start = Number(match[1]);
    var end = Number(match[2]);
    if (!Number.isFinite(start) || !Number.isFinite(end) || end <= start) return null;
    return { start: start, end: end };
  }

  function applyChartRange(chart, range) {
    if (!chart || !range) return;
    try {
      var cfg = chart.config && (chart.config._config || chart.config);
      if (cfg && cfg.options) {
        cfg.options.scales = cfg.options.scales || {};
        cfg.options.scales.x = Object.assign({}, cfg.options.scales.x || {}, { min: range.start, max: range.end });
      }
    } catch (_) {}
    try {
      if (chart.scales && chart.scales.x) {
        chart.scales.x.options = Object.assign({}, chart.scales.x.options || {}, { min: range.start, max: range.end });
        chart.scales.x.min = range.start;
        chart.scales.x.max = range.end;
      }
    } catch (_) {}
    try { chart.draw(); } catch (_) {}
  }

  function beginMiniMapShim() {
    if (window.__rvtMiniMapShimActive) return;
    var store = window.S && window.S.charts;
    if (!store) return;
    var saved = { breath: store.breath, heart: store.heart };
    var shim = function(){ return { options: { scales: { x: {} } }, update: function(){}, draw: function(){} }; };
    window.__rvtMiniMapShimActive = true;
    store.breath = shim();
    store.heart = shim();
    setTimeout(function(){
      var currentStore = window.S && window.S.charts;
      if (!currentStore) return;
      currentStore.breath = saved.breath;
      currentStore.heart = saved.heart;
      window.__rvtMiniMapShimActive = false;
      var range = readMiniRange();
      applyChartRange(currentStore.breath, range);
      applyChartRange(currentStore.heart, range);
      try { if (typeof window.rvtRenderWaveHatches === 'function') window.rvtRenderWaveHatches(); } catch (_) {}
    }, 0);
  }

  document.addEventListener('click', function(ev){
    if (ev.target.closest && ev.target.closest('#rvtMiniMapTrack')) beginMiniMapShim();
  }, true);

  document.addEventListener('keydown', function(ev){
    if (!ev.target.closest || !ev.target.closest('#rvtMiniMapTrack')) return;
    if (['ArrowLeft', 'ArrowRight', 'Home', 'End', 'PageUp', 'PageDown'].indexOf(ev.key) !== -1) beginMiniMapShim();
  }, true);

  function boot() {
    applyRecoveryState();
    try {
      var topic = (window.S && window.S.ctl && window.S.ctl.helpTopic) || localStorage.getItem(HELP_TOPIC_KEY) || '';
      setActiveHelpTopic(topic);
    } catch (_) {}
    var tries = 0;
    var timer = setInterval(function(){
      tries += 1;
      applyRecoveryState();
      if (tries >= 24 || document.getElementById('rvtRecoveryChecklist')) clearInterval(timer);
    }, 250);
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot, { once: true });
  else boot();
})();
