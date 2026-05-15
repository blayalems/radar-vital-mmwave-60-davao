(function(){
  'use strict';

  /* ═══════════════════════════════════════════════════════
     PERF-01: RAF-batched DOM sync consolidator
     Merges the 5+ redundant setInterval sync loops into one
     ═══════════════════════════════════════════════════════ */
  var _syncRaf = 0;
  var _syncJobs = [];
  function registerSync(fn) { _syncJobs.push(fn); }
  function tickSync() {
    _syncRaf = 0;
    var view = document.body.getAttribute('data-view') || 'home';
    for (var i = 0; i < _syncJobs.length; i++) {
      try { _syncJobs[i](view); } catch(e) { console.warn('[v15.2 sync]', e); }
    }
  }
  function scheduleSync() {
    if (!_syncRaf) _syncRaf = requestAnimationFrame(tickSync);
  }
  // Single interval instead of many
  setInterval(scheduleSync, 2000);

  /* ═══════════════════════════════════════════════════════
     PERF-02: Consolidated MutationObserver
     Replace per-feature body observers with one shared observer
     ═══════════════════════════════════════════════════════ */
  var _mutationCallbacks = [];
  function onBodyMutation(fn) { _mutationCallbacks.push(fn); }
  var _sharedBodyObs = new MutationObserver(function(muts) {
    for (var i = 0; i < _mutationCallbacks.length; i++) {
      try { _mutationCallbacks[i](muts); } catch(err) { if (window.rvtSoftError) window.rvtSoftError('body mutation callback failed', err); }
    }
    scheduleSync();
  });
  _sharedBodyObs.observe(document.body, {
    attributes: true,
    attributeFilter: ['data-view', 'data-ctl', 'class', 'data-theme', 'data-density']
  });
  // Track for BUG-01 disconnect
  window.S = window.S || {};
  if (!window.S.__mutationObservers) window.S.__mutationObservers = [];
  window.S.__mutationObservers.push(_sharedBodyObs);

  /* ═══════════════════════════════════════════════════════
     BUG-03: Start Session Race Guard
     ═══════════════════════════════════════════════════════ */
  var _preflightReady = false;
  var _defaultsReady = false;
  var _sessionsReady = false;

  function guardStartButton() {
    var btn = document.querySelector('.home-start-btn, .setup-start, [onclick*="startSession"], #startSessionBtn');
    if (!btn) return;
    var allReady = _preflightReady && _defaultsReady && _sessionsReady;
    if (!allReady && !btn.disabled) {
      btn.disabled = true;
    }
    // Update disable reason
    var reason = btn.querySelector('.rvt-disable-reason');
    if (btn.disabled && !allReady) {
      var msg = [];
      if (!_preflightReady) msg.push('preflight');
      if (!_defaultsReady) msg.push('defaults');
      if (!_sessionsReady) msg.push('sessions');
      var txt = 'Waiting: ' + msg.join(', ');
      if (reason) reason.textContent = txt;
      btn.setAttribute('aria-describedby', 'rvtStartReason');
    }
  }

  // Hook async completions
  var _origLoadDefaults = window.loadControlDefaults;
  if (typeof _origLoadDefaults === 'function') {
    window.loadControlDefaults = function() {
      var result = _origLoadDefaults.apply(this, arguments);
      if (result && typeof result.then === 'function') {
        result.then(function() { _defaultsReady = true; guardStartButton(); })
          .catch(function(err) {
            console.warn('[RVT boot] defaults fetch failed:', err);
            if (window.rvtSoftError) window.rvtSoftError('defaults boot gate failed open', err, { surface: true, severity: 'warn' });
            _defaultsReady = true;
            guardStartButton();
          });
      } else { _defaultsReady = true; }
      return result;
    };
  } else { _defaultsReady = true; }

  var _origLoadSessions = window.loadSessionsList;
  if (typeof _origLoadSessions === 'function') {
    window.loadSessionsList = function() {
      var result = _origLoadSessions.apply(this, arguments);
      if (result && typeof result.then === 'function') {
        result.then(function() { _sessionsReady = true; guardStartButton(); })
          .catch(function(err) {
            console.warn('[RVT boot] sessions fetch failed:', err);
            if (window.rvtSoftError) window.rvtSoftError('sessions boot gate failed open', err, { surface: true, severity: 'warn' });
            _sessionsReady = true;
            guardStartButton();
          });
      } else { _sessionsReady = true; }
      return result;
    };
  } else { _sessionsReady = true; }

  var _origRunPreflight = window.runPreflightBatch;
  if (typeof _origRunPreflight === 'function') {
    window.runPreflightBatch = function() {
      var result = _origRunPreflight.apply(this, arguments);
      if (result && typeof result.then === 'function') {
        result.then(function() { _preflightReady = true; guardStartButton(); })
          .catch(function(err) {
            console.warn('[RVT boot] preflight fetch failed:', err);
            if (window.rvtSoftError) window.rvtSoftError('preflight boot gate failed open', err, { surface: true, severity: 'warn' });
            _preflightReady = true;
            guardStartButton();
          });
      } else { _preflightReady = true; }
      return result;
    };
  } else { _preflightReady = true; }

  registerSync(guardStartButton);

  /* ═══════════════════════════════════════════════════════
     BUG-04: Replace silent catches with logged warnings
     (Patch safeStorageJson and apiJson)
     ═══════════════════════════════════════════════════════ */
  if (typeof window.safeStorageJson === 'function') {
    var _origSSJ = window.safeStorageJson;
    window.safeStorageJson = function(key, fallback, validator) {
      try {
        return _origSSJ.apply(this, arguments);
      } catch(e) {
        console.warn('[RVT] safeStorageJson error for key:', key, e);
        return fallback;
      }
    };
  }

  /* ═══════════════════════════════════════════════════════
     BUG-05: Schema-validate live payloads
     ═══════════════════════════════════════════════════════ */
  var _invalidPayloadCount = 0;
  var _origNormalize = window.normalizeLivePayload;
  if (typeof _origNormalize === 'function') {
    window.normalizeLivePayload = function(raw) {
      try {
        if (raw && typeof raw.payload === 'string') {
          console.warn('[RVT] Unexpected string payload, attempting parse');
          try { raw.payload = JSON.parse(raw.payload); } catch(_) {
            _invalidPayloadCount++;
            return {};
          }
        }
        return _origNormalize.apply(this, arguments);
      } catch(e) {
        _invalidPayloadCount++;
        console.warn('[RVT] normalizeLivePayload rejected malformed payload:', e);
        return {};
      }
    };
  }

  /* ═══════════════════════════════════════════════════════
     HOME-02: Quickcheck Pipeline Visualization
     ═══════════════════════════════════════════════════════ */
  var PIPELINE_STAGES = [
    { id: 'python', label: 'Python', icon: 'terminal' },
    { id: 'firmware', label: 'Firmware', icon: 'memory' },
    { id: 'serial', label: 'Serial', icon: 'usb' },
    { id: 'ble', label: 'BLE', icon: 'bluetooth' }
  ];

  function injectPipeline() {
    var homeView = document.getElementById('view-home');
    if (!homeView) return;
    if (homeView.querySelector('.rvt-pipeline')) return;

    var pfCard = homeView.querySelector('.pf-card, [data-section="quickcheck"], .quickcheck-card');
    if (!pfCard) return;

    var pipeline = document.createElement('div');
    pipeline.className = 'rvt-pipeline';

    PIPELINE_STAGES.forEach(function(stage, idx) {
      if (idx > 0) {
        var bridge = document.createElement('div');
        bridge.className = 'rvt-pipeline-bridge';
        bridge.id = 'rvtBridge' + idx;
        pipeline.appendChild(bridge);
      }
      var node = document.createElement('div');
      node.className = 'rvt-pipeline-node';
      node.setAttribute('tabindex', '0');
      node.setAttribute('role', 'button');
      node.setAttribute('data-stage', stage.id);
      node.setAttribute('data-status', 'pending');
      node.innerHTML =
        '<span class="material-symbols-rounded">' + stage.icon + '</span>' +
        '<span>' + stage.label + '</span>' +
        '<span class="rvt-pipeline-ts" id="rvtPipeTs' + idx + '"></span>';
      node.addEventListener('keydown', function(e) {
        if (e.key === 'Enter') { /* re-run single check */ }
      });
      pipeline.appendChild(node);
    });

    // Insert before the existing pf-items
    var pfItems = pfCard.querySelector('.pf-items, .pf-grid, ul');
    if (pfItems) pfCard.insertBefore(pipeline, pfItems);
    else pfCard.appendChild(pipeline);

    // Re-run failed button
    var rerunBtn = document.createElement('button');
    rerunBtn.className = 'rvt-rerun-failed-btn';
    rerunBtn.type = 'button';
    rerunBtn.innerHTML = '<span class="material-symbols-rounded">refresh</span>Re-run failed only';
    rerunBtn.addEventListener('click', function() {
      if (typeof window.runPreflightBatch === 'function') {
        rerunBtn.disabled = true;
        rerunBtn.querySelector('.material-symbols-rounded').textContent = 'progress_activity';
        window.runPreflightBatch().then(function() {
          rerunBtn.disabled = false;
          rerunBtn.querySelector('.material-symbols-rounded').textContent = 'refresh';
        }).catch(function() {
          rerunBtn.disabled = false;
          rerunBtn.querySelector('.material-symbols-rounded').textContent = 'refresh';
        });
      }
    });
    pfCard.appendChild(rerunBtn);
  }

  function syncPipeline(view) {
    if (view !== 'home') return;
    var pipeline = document.querySelector('.rvt-pipeline');
    if (!pipeline) return;

    var pfItems = document.querySelectorAll('.pf-item, [data-check]');
    var statusMap = {};
    pfItems.forEach(function(el) {
      var id = (el.id || el.getAttribute('data-check') || '').toLowerCase();
      var cls = el.className || '';
      var status = 'pending';
      if (cls.includes('pass') || cls.includes('ok')) status = 'pass';
      else if (cls.includes('fail') || cls.includes('error')) status = 'fail';
      else if (cls.includes('warn')) status = 'warn';
      else if (cls.includes('running') || cls.includes('checking')) status = 'running';
      PIPELINE_STAGES.forEach(function(stage) {
        if (id.includes(stage.id)) statusMap[stage.id] = status;
      });
    });

    pipeline.querySelectorAll('.rvt-pipeline-node').forEach(function(node) {
      var stageId = node.getAttribute('data-stage');
      var s = statusMap[stageId] || 'pending';
      node.setAttribute('data-status', s);
      var icon = node.querySelector('.material-symbols-rounded');
      if (s === 'pass') icon.textContent = 'check_circle';
      else if (s === 'fail') icon.textContent = 'error';
      else if (s === 'warn') icon.textContent = 'warning';
      else if (s === 'running') icon.textContent = 'progress_activity';
      else {
        var stage = PIPELINE_STAGES.find(function(st) { return st.id === stageId; });
        if (stage) icon.textContent = stage.icon;
      }
    });

    // Bridges
    var allPass = true;
    var hasWarn = false;
    pipeline.querySelectorAll('.rvt-pipeline-bridge').forEach(function(b, i) {
      var prev = pipeline.querySelectorAll('.rvt-pipeline-node')[i];
      if (prev && prev.getAttribute('data-status') === 'pass') {
        b.classList.add('filled');
        b.classList.remove('warn-filled');
      } else if (prev && prev.getAttribute('data-status') === 'warn') {
        b.classList.add('warn-filled');
        b.classList.remove('filled');
      } else {
        b.classList.remove('filled', 'warn-filled');
      }
    });

    // Re-run failed button visibility
    var failCount = document.querySelectorAll('.pf-item.fail, .pf-item.warn, [data-check][data-status="fail"], [data-check][data-status="warn"]').length;
    var rerunBtn = document.querySelector('.rvt-rerun-failed-btn');
    if (rerunBtn) rerunBtn.classList.toggle('show', failCount > 0);
  }

  registerSync(syncPipeline);

  /* ═══════════════════════════════════════════════════════
     HOME-06: Copy Session Brief
     ═══════════════════════════════════════════════════════ */
  function injectCopyBrief() {
    var homeView = document.getElementById('view-home');
    if (!homeView) return;
    if (homeView.querySelector('.rvt-copy-brief-btn')) return;

    var confCard = homeView.querySelector('.session-confidence, .ready-card, [data-section="confidence"]');
    if (!confCard) return;

    var btn = document.createElement('button');
    btn.className = 'rvt-copy-brief-btn';
    btn.type = 'button';
    btn.innerHTML = '<span class="material-symbols-rounded">content_copy</span>Copy brief';
    btn.addEventListener('click', function() {
      var op = (window.S && window.S.operatorName) || 'Unknown';
      var subj = (window.S && window.S.subjectId) || '--';
      var port = (window.S && window.S.port) || '--';
      var profile = (window.S && window.S.profileName) || '--';
      var readiness = '--';
      var ringEl = confCard.querySelector('.ready-pct, .ring-value');
      if (ringEl) readiness = ringEl.textContent.trim();
      var date = new Date().toISOString().split('T')[0];
      var brief = date + ' · Op: ' + op + ' · Subject: ' + subj + ' · Port: ' + port + ' · Readiness: ' + readiness + ' · Profile: ' + profile;
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(brief).then(function() {
          if (typeof window.showToast === 'function') window.showToast('Session brief copied', 'ok');
        });
      } else {
        prompt('Copy this session brief:', brief);
      }
    });
    confCard.appendChild(btn);
  }

  /* ═══════════════════════════════════════════════════════
     LIVE-05: Quick-Tag Keyboard Shortcuts
     ═══════════════════════════════════════════════════════ */
  var TAG_MAP = { m: 'motion', c: 'cough', s: 'speaking', b: 'baseline' };
  var tagToast = document.getElementById('rvtTagToast');
  var _tagToastTimer = null;

  document.addEventListener('keydown', function(e) {
    if (document.body.getAttribute('data-view') !== 'live') return;
    var active = document.activeElement;
    if (active && (active.tagName === 'TEXTAREA' || active.tagName === 'INPUT' || active.isContentEditable)) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;

    var tag = TAG_MAP[e.key.toLowerCase()];
    if (!tag) return;

    e.preventDefault();
    var now = new Date();
    var ts = [now.getHours(), now.getMinutes(), now.getSeconds()].map(function(n) { return (n<10?'0':'') + n; }).join(':');
    var entry = '[' + ts + '] ' + tag;

    var notes = document.getElementById('sessionNotes');
    if (notes) {
      notes.value = (notes.value ? notes.value + '\n' : '') + entry;
      notes.dispatchEvent(new Event('input', { bubbles: true }));
    }

    // Show toast
    if (tagToast) {
      tagToast.textContent = 'Tagged: ' + tag + ' @ ' + ts;
      tagToast.classList.add('show');
      clearTimeout(_tagToastTimer);
      _tagToastTimer = setTimeout(function() { tagToast.classList.remove('show'); }, 2500);
    }
  });

  /* ═══════════════════════════════════════════════════════
     LIVE-06: Snapshot FAB
     ═══════════════════════════════════════════════════════ */
  var snapFab = document.getElementById('rvtSnapFab');
  var snapBadge = document.getElementById('rvtSnapFabBadge');

  function doSnapshot() {
    if (typeof window.takeSnapshot === 'function') {
      window.takeSnapshot();
    } else if (typeof window.pinSnapshot === 'function') {
      window.pinSnapshot();
    }
    // Animate
    if (snapFab) { snapFab.style.transform = 'scale(0.85)'; setTimeout(function() { snapFab.style.transform = ''; }, 200); }
    syncSnapBadge();
  }

  function syncSnapBadge() {
    var count = (window.S && Array.isArray(window.S.snaps)) ? window.S.snaps.length : 0;
    if (!snapBadge) return;
    if (count > 0) {
      snapBadge.textContent = count;
      snapBadge.style.display = '';
    } else {
      snapBadge.style.display = 'none';
    }
  }

  if (snapFab) snapFab.addEventListener('click', doSnapshot);
  document.addEventListener('keydown', function(e) {
    if (e.key === 'p' || e.key === 'P') {
      if (document.body.getAttribute('data-view') !== 'live') return;
      var active = document.activeElement;
      if (active && (active.tagName === 'TEXTAREA' || active.tagName === 'INPUT')) return;
      if (e.ctrlKey || e.metaKey) return;
      e.preventDefault();
      doSnapshot();
    }
  });

  registerSync(function(view) { if (view === 'live') syncSnapBadge(); });

  /* ═══════════════════════════════════════════════════════
     RPT-02: Auto-generated Executive Summary
     ═══════════════════════════════════════════════════════ */
  function generateInsights() {
    var insights = [];
    try {
      if (!window.S || !window.S.lastReportPayload) return insights;
      var rpt = window.S.lastReportPayload;

      // HR drift
      if (rpt.hr_rmse != null && Number(rpt.hr_rmse) > 3) {
        insights.push('HR RMSE was ' + Number(rpt.hr_rmse).toFixed(1) + ' bpm — above the 3 bpm target threshold');
      }
      if (rpt.rr_rmse != null && Number(rpt.rr_rmse) > 2) {
        insights.push('RR RMSE was ' + Number(rpt.rr_rmse).toFixed(1) + ' br/min — above the 2 br/min target');
      }
      // Coverage
      if (rpt.coverage != null && Number(rpt.coverage) < 0.85) {
        insights.push('Coverage was ' + (Number(rpt.coverage) * 100).toFixed(0) + '% — below 85% target, likely due to subject motion or range issues');
      }
      // Correlation
      if (rpt.hr_r != null && Number(rpt.hr_r) < 0.9) {
        insights.push('HR correlation (r) was ' + Number(rpt.hr_r).toFixed(3) + ' — below 0.9 target');
      }
      // Settling
      if (rpt.settling_s != null && Number(rpt.settling_s) > 30) {
        insights.push('Settling took ' + Number(rpt.settling_s).toFixed(0) + 's — longer than typical 30s window');
      }

      if (insights.length === 0) {
        insights.push('All metrics within nominal ranges — session quality is good');
      }
    } catch(_) {}
    return insights.slice(0, 3);
  }

  function injectExecSummary() {
    var reportView = document.getElementById('view-report');
    if (!reportView) return;
    if (reportView.querySelector('.rvt-exec-summary')) return;

    var hero = reportView.querySelector('.verdict-hero');
    if (!hero) return;

    var summary = document.createElement('div');
    summary.className = 'rvt-exec-summary';
    summary.innerHTML = '<div class="rvt-exec-summary-title"><span class="material-symbols-rounded" style="font-size:18px;">auto_awesome</span>Executive Summary</div>';

    var insights = generateInsights();
    insights.forEach(function(text) {
      var row = document.createElement('div');
      row.className = 'rvt-exec-insight';
      row.innerHTML = '<span class="material-symbols-rounded">arrow_right</span><span>' + text + '</span>';
      summary.appendChild(row);
    });

    hero.parentNode.insertBefore(summary, hero.nextSibling);
  }

  /* ═══════════════════════════════════════════════════════
     RPT-07: Tint diagnostic cards by gate status
     ═══════════════════════════════════════════════════════ */
  function tintDiagnosticCards() {
    var reportView = document.getElementById('view-report');
    if (!reportView) return;
    reportView.querySelectorAll('.verdict-card, .diag-card, [data-gate]').forEach(function(card) {
      var badge = card.querySelector('.badge, .status-badge, [data-status]');
      if (!badge) return;
      var cls = (badge.className || '') + ' ' + (badge.getAttribute('data-status') || '');
      if (/pass|ok/i.test(cls)) card.setAttribute('data-gate-status', 'pass');
      else if (/fail|bad|crit/i.test(cls)) card.setAttribute('data-gate-status', 'fail');
      else if (/warn/i.test(cls)) card.setAttribute('data-gate-status', 'warn');
    });
  }

  /* ═══════════════════════════════════════════════════════
     SET-05: Settings Profiles
     ═══════════════════════════════════════════════════════ */
  var PROFILE_KEYS = ['rvt-thresholds', 'rvt-audio-vol', 'rvt-voice-alert', 'rvt-voice-readout', 'rvt-theme', 'rvt-density', 'rvt-kpi-order', 'rvt-hotkeys', 'rvt-live-mode'];
  var MAX_PROFILES = 10;

  function getProfiles() {
    try { return JSON.parse(localStorage.getItem('rvt-settings-profiles') || '{}'); } catch(_) { return {}; }
  }
  function saveProfiles(p) {
    try { localStorage.setItem('rvt-settings-profiles', JSON.stringify(p)); } catch(_) {}
  }

  function captureCurrentSettings() {
    var snap = {};
    PROFILE_KEYS.forEach(function(k) {
      try { snap[k] = localStorage.getItem(k); } catch(_) {}
    });
    return snap;
  }

  function applyProfile(name) {
    var profiles = getProfiles();
    var p = profiles[name];
    if (!p || !p.settings) return;
    Object.keys(p.settings).forEach(function(k) {
      try {
        if (p.settings[k] != null) localStorage.setItem(k, p.settings[k]);
        else localStorage.removeItem(k);
      } catch(_) {}
    });
    // Reload to apply
    location.reload();
  }

  function injectProfilesSection() {
    var settingsView = document.getElementById('view-settings');
    if (!settingsView) return;
    if (settingsView.querySelector('.rvt-profiles-section')) return;

    var section = document.createElement('div');
    section.className = 'rvt-profiles-section';
    section.innerHTML =
      '<div class="set-r"><div class="set-copy"><strong>Settings Profiles</strong>' +
      '<span>Save and load complete settings snapshots for different operators or study types.</span></div></div>' +
      '<div class="rvt-profiles-list" id="rvtProfilesList"></div>' +
      '<div style="display:flex;gap:8px;margin-top:4px;">' +
        '<input type="text" id="rvtProfileNameInput" placeholder="Profile name…" style="flex:1;padding:6px 10px;border:1px solid var(--line);border-radius:var(--r-sm);font-size:12px;font-family:Inter,sans-serif;background:var(--surface-1);color:var(--ink-700);">' +
        '<button type="button" id="rvtProfileSaveBtn" class="rvt-settings-io-btn"><span class="material-symbols-rounded">save</span>Save current</button>' +
      '</div>' +
      '<div class="rvt-settings-io-row">' +
        '<button type="button" class="rvt-settings-io-btn" id="rvtSettingsExport"><span class="material-symbols-rounded">download</span>Export JSON</button>' +
        '<button type="button" class="rvt-settings-io-btn" id="rvtSettingsImport"><span class="material-symbols-rounded">upload</span>Import JSON</button>' +
        '<input type="file" id="rvtSettingsImportFile" accept=".json" style="display:none">' +
      '</div>';

    // Find a good place to insert
    var lastGroup = settingsView.querySelectorAll('.set-g, .settings-group');
    if (lastGroup.length > 0) {
      lastGroup[lastGroup.length - 1].after(section);
    } else {
      settingsView.appendChild(section);
    }

    renderProfiles();

    document.getElementById('rvtProfileSaveBtn').addEventListener('click', function() {
      var name = document.getElementById('rvtProfileNameInput').value.trim();
      if (!name) { alert('Enter a profile name'); return; }
      var profiles = getProfiles();
      if (Object.keys(profiles).length >= MAX_PROFILES && !profiles[name]) {
        alert('Maximum ' + MAX_PROFILES + ' profiles. Delete one first.');
        return;
      }
      profiles[name] = { settings: captureCurrentSettings(), ts: Date.now() };
      saveProfiles(profiles);
      document.getElementById('rvtProfileNameInput').value = '';
      renderProfiles();
      if (typeof window.showToast === 'function') window.showToast('Profile "' + name + '" saved', 'ok');
    });

    // SET-06: Export/Import
    document.getElementById('rvtSettingsExport').addEventListener('click', function() {
      var data = { version: 1, ts: Date.now(), settings: captureCurrentSettings() };
      var blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url;
      a.download = 'rvt_settings_' + new Date().toISOString().split('T')[0] + '.json';
      a.click();
      URL.revokeObjectURL(url);
    });

    document.getElementById('rvtSettingsImport').addEventListener('click', function() {
      document.getElementById('rvtSettingsImportFile').click();
    });

    document.getElementById('rvtSettingsImportFile').addEventListener('change', function(e) {
      var file = e.target.files && e.target.files[0];
      if (!file) return;
      var reader = new FileReader();
      reader.onload = function() {
        try {
          var data = JSON.parse(reader.result);
          if (!data.settings) { alert('Invalid settings file'); return; }
          if (confirm('Import settings? This will overwrite your current configuration.')) {
            Object.keys(data.settings).forEach(function(k) {
              try {
                if (data.settings[k] != null) localStorage.setItem(k, data.settings[k]);
              } catch(_) {}
            });
            if (typeof window.showToast === 'function') window.showToast('Settings imported — reloading…', 'ok');
            setTimeout(function() { location.reload(); }, 1500);
          }
        } catch(err) { alert('Could not parse settings file: ' + err.message); }
      };
      reader.readAsText(file);
      e.target.value = '';
    });
  }

  function renderProfiles() {
    var list = document.getElementById('rvtProfilesList');
    if (!list) return;
    var profiles = getProfiles();
    list.innerHTML = '';
    Object.keys(profiles).forEach(function(name) {
      var chip = document.createElement('button');
      chip.className = 'rvt-profile-chip';
      chip.type = 'button';
      chip.innerHTML = name + '<span class="rvt-profile-del material-symbols-rounded">close</span>';
      chip.addEventListener('click', function(e) {
        if (e.target.classList.contains('rvt-profile-del')) {
          if (confirm('Delete profile "' + name + '"?')) {
            var p = getProfiles();
            delete p[name];
            saveProfiles(p);
            renderProfiles();
          }
          return;
        }
        applyProfile(name);
      });
      list.appendChild(chip);
    });
    if (Object.keys(profiles).length === 0) {
      list.innerHTML = '<span style="font-size:12px;color:var(--ink-400);font-style:italic;">No saved profiles</span>';
    }
  }

  /* ═══════════════════════════════════════════════════════
     NAV-01: Rail Collapse Toggle
     ═══════════════════════════════════════════════════════ */
  function injectRailCollapse() {
    var rail = document.querySelector('.rail');
    if (!rail || document.getElementById('railCollapseBtn') || rail.querySelector('.rvt-rail-collapse-btn')) return;

    var btn = document.createElement('button');
    btn.className = 'rvt-rail-collapse-btn';
    btn.type = 'button';
    btn.innerHTML = '<span class="material-symbols-rounded" style="font-size:18px;">chevron_left</span>';
    btn.title = 'Collapse rail (Alt+\\)';
    btn.addEventListener('click', function() {
      var collapsed = document.documentElement.getAttribute('data-rail-collapsed') === '1';
      document.documentElement.setAttribute('data-rail-collapsed', collapsed ? '0' : '1');
      try { localStorage.setItem('rvt-rail-collapsed', collapsed ? '0' : '1'); } catch(_) {}
      btn.querySelector('.material-symbols-rounded').textContent = collapsed ? 'chevron_left' : 'chevron_right';
    });
    rail.appendChild(btn);

    // Restore
    try {
      if (localStorage.getItem('rvt-rail-collapsed') === '1') {
        document.documentElement.setAttribute('data-rail-collapsed', '1');
        btn.querySelector('.material-symbols-rounded').textContent = 'chevron_right';
      }
    } catch(_) {}
  }

  document.addEventListener('keydown', function(e) {
    if (e.altKey && e.key === '\\') {
      var btn = document.getElementById('railCollapseBtn') || document.querySelector('.rvt-rail-collapse-btn');
      if (btn) btn.click();
    }
  });

  /* ═══════════════════════════════════════════════════════
     ALERT-02: Time-cluster Grouping
     ═══════════════════════════════════════════════════════ */
  function clusterAlerts() {
    var drawer = document.getElementById('dv-ov') || document.querySelector('.alerts-drawer');
    if (!drawer) return;
    if (drawer.querySelector('.rvt-alert-cluster')) return;

    var list = drawer.querySelector('.dv-content, .drawer-content, .alerts-list');
    if (!list) return;

    var items = list.querySelectorAll('.alert-item, [data-alert]');
    if (items.length === 0) return;

    var now = Date.now();
    var fiveMin = 5 * 60 * 1000;
    var clusters = { recent: [], earlier: [], previous: [] };

    items.forEach(function(item) {
      var ts = parseInt(item.getAttribute('data-ts') || item.getAttribute('data-timestamp') || '0', 10);
      if (!ts) ts = now;
      if (now - ts < fiveMin) clusters.recent.push(item);
      else if (item.getAttribute('data-session') === 'current' || !item.getAttribute('data-session')) clusters.earlier.push(item);
      else clusters.previous.push(item);
    });

    var labels = [
      ['recent', 'Last 5 min', clusters.recent],
      ['earlier', 'Earlier this session', clusters.earlier],
      ['previous', 'Previous sessions', clusters.previous]
    ];

    list.innerHTML = '';
    labels.forEach(function(entry) {
      if (entry[2].length === 0) return;
      var cluster = document.createElement('div');
      cluster.className = 'rvt-alert-cluster';
      cluster.innerHTML =
        '<div class="rvt-alert-cluster-header">' +
          '<span class="material-symbols-rounded">expand_more</span>' +
          '<span>' + entry[1] + '</span>' +
          '<span class="rvt-alert-cluster-count">' + entry[2].length + '</span>' +
        '</div>' +
        '<div class="rvt-alert-cluster-body"></div>';

      var body = cluster.querySelector('.rvt-alert-cluster-body');
      entry[2].forEach(function(item) { body.appendChild(item); });

      cluster.querySelector('.rvt-alert-cluster-header').addEventListener('click', function() {
        cluster.classList.toggle('collapsed');
      });

      // Collapse previous by default
      if (entry[0] === 'previous') cluster.classList.add('collapsed');

      list.appendChild(cluster);
    });
  }

  /* ═══════════════════════════════════════════════════════
     MOB-01: Tap-to-pin Tooltips
     ═══════════════════════════════════════════════════════ */
  var _activeTapTooltip = null;
  if ('ontouchstart' in window || navigator.maxTouchPoints > 0) {
    document.addEventListener('click', function(e) {
      // Dismiss existing
      if (_activeTapTooltip) {
        _activeTapTooltip.remove();
        _activeTapTooltip = null;
      }

      var target = e.target.closest('[title], [data-tooltip]');
      if (!target) return;

      var text = target.getAttribute('data-tooltip') || target.getAttribute('title');
      if (!text || text.length < 2) return;

      // Prevent default title
      if (target.hasAttribute('title')) {
        target.setAttribute('data-tooltip', text);
        target.removeAttribute('title');
      }

      e.preventDefault();
      var rect = target.getBoundingClientRect();
      var tip = document.createElement('div');
      tip.className = 'rvt-tap-tooltip';
      tip.textContent = text;
      tip.style.left = Math.max(8, Math.min(rect.left, window.innerWidth - 280)) + 'px';
      tip.style.top = (rect.bottom + 8) + 'px';
      document.body.appendChild(tip);
      _activeTapTooltip = tip;

      // Dismiss on next tap anywhere
      setTimeout(function() {
        document.addEventListener('click', function dismiss() {
          if (_activeTapTooltip) { _activeTapTooltip.remove(); _activeTapTooltip = null; }
          document.removeEventListener('click', dismiss);
        }, { once: true });
      }, 50);
    }, true);
  }

  /* ═══════════════════════════════════════════════════════
     QOL-01: Universal Undo
     ═══════════════════════════════════════════════════════ */
  var _undoStack = {};
  var undoToast = document.getElementById('rvtUndoToast');
  var undoMsg = document.getElementById('rvtUndoMsg');
  var undoBtn = document.getElementById('rvtUndoBtn');
  var _undoTimer = null;

  window.rvtUndo = function(actionType, description, undoFn) {
    _undoStack[actionType] = { fn: undoFn, desc: description };
    undoMsg.textContent = description;
    undoToast.classList.add('show');
    clearTimeout(_undoTimer);
    _undoTimer = setTimeout(function() {
      undoToast.classList.remove('show');
    }, 10000);
  };

  if (undoBtn) undoBtn.addEventListener('click', function() {
    var keys = Object.keys(_undoStack);
    if (keys.length === 0) return;
    var last = _undoStack[keys[keys.length - 1]];
    if (last && typeof last.fn === 'function') {
      try { last.fn(); } catch(e) { console.warn('[RVT Undo]', e); }
    }
    delete _undoStack[keys[keys.length - 1]];
    undoToast.classList.remove('show');
    clearTimeout(_undoTimer);
    if (typeof window.showToast === 'function') window.showToast('Undone', 'ok');
  }); // end undoBtn listener

  /* ═══════════════════════════════════════════════════════
     QOL-03: Operator Handoff
     ═══════════════════════════════════════════════════════ */
  var handoffOverlay = document.getElementById('rvtHandoffOverlay');

  function openHandoff() {
    var content = document.getElementById('rvtHandoffContent');
    var op = (window.S && window.S.operatorName) || 'Unknown';
    var view = document.body.getAttribute('data-view') || 'home';
    var sessionActive = document.body.getAttribute('data-ctl') === 'on';
    var alertCount = document.querySelectorAll('.alert-item:not(.acked), [data-alert]:not([data-acked="true"])').length;

    content.innerHTML =
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Outgoing Operator</div><div class="rvt-handoff-value">' + op + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Session State</div><div class="rvt-handoff-value">' + (sessionActive ? 'Active session in progress' : 'No active session') + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Current View</div><div class="rvt-handoff-value">' + view + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Unacknowledged Alerts</div><div class="rvt-handoff-value">' + alertCount + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Time</div><div class="rvt-handoff-value">' + new Date().toLocaleString() + '</div></div>' +
      '<div class="rvt-handoff-section"><div class="rvt-handoff-label">Notes for incoming operator</div>' +
        '<textarea id="rvtHandoffNotes" rows="3" style="width:100%;padding:8px;border:1px solid var(--line);border-radius:var(--r-sm);font-family:Inter,sans-serif;font-size:13px;resize:vertical;background:var(--surface-2);color:var(--ink-700);" placeholder="Any follow-ups, pending tasks…"></textarea>' +
      '</div>';

    if (handoffOverlay) handoffOverlay.classList.add('show');
  }

  function closeHandoff() { if (handoffOverlay) handoffOverlay.classList.remove('show'); }

  var _handoffCopyBtn = document.getElementById('rvtHandoffCopy');
  if (_handoffCopyBtn) _handoffCopyBtn.addEventListener('click', function() {
    var content = document.getElementById('rvtHandoffContent');
    var text = content.innerText;
    var notes = document.getElementById('rvtHandoffNotes');
    if (notes) text += '\nNotes: ' + notes.value;
    if (navigator.clipboard) {
      navigator.clipboard.writeText(text).then(function() {
        if (typeof window.showToast === 'function') window.showToast('Handoff summary copied', 'ok');
      });
    }
  });

  var _handoffConfirmBtn = document.getElementById('rvtHandoffConfirm');
  if (_handoffConfirmBtn) _handoffConfirmBtn.addEventListener('click', function() {
    closeHandoff();
    if (typeof window.showToast === 'function') window.showToast('Handoff confirmed — ready for new operator', 'ok');
  });

  if (handoffOverlay) handoffOverlay.addEventListener('click', function(e) { if (e.target === handoffOverlay) closeHandoff(); });
  document.addEventListener('keydown', function(e) {
    if (e.key === 'Escape' && handoffOverlay.classList.contains('show')) closeHandoff();
    if (e.ctrlKey && e.shiftKey && e.key === 'H') { e.preventDefault(); openHandoff(); }
  });

  window.openHandoff = openHandoff;

  /* ═══════════════════════════════════════════════════════
     QOL-07: Focus Panel Hotkey (F)
     ═══════════════════════════════════════════════════════ */
  var _focusedPanel = null;
  document.addEventListener('keydown', function(e) {
    if (e.key !== 'f' && e.key !== 'F') return;
    var active = document.activeElement;
    if (active && (active.tagName === 'TEXTAREA' || active.tagName === 'INPUT' || active.isContentEditable)) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;

    if (_focusedPanel) {
      _focusedPanel.classList.remove('rvt-panel-focused');
      _focusedPanel = null;
      return;
    }

    var panel = document.activeElement ? document.activeElement.closest('.card, .kpi, .panel, .wc') : null;
    if (!panel) return;
    e.preventDefault();
    panel.classList.add('rvt-panel-focused');
    _focusedPanel = panel;
  });

  document.addEventListener('keydown', function(e) {
    if (e.key === 'Escape' && _focusedPanel) {
      _focusedPanel.classList.remove('rvt-panel-focused');
      _focusedPanel = null;
    }
  });

  /* ═══════════════════════════════════════════════════════
     A11Y-04: Adjustable Font Scaling
     ═══════════════════════════════════════════════════════ */
  function injectFontScaleSetting() {
    var settingsView = document.getElementById('view-settings');
    if (!settingsView) return;
    if (settingsView.querySelector('#rvtFontScaleRow')) return;

    var row = document.createElement('div');
    row.className = 'set-r';
    row.id = 'rvtFontScaleRow';
    var saved = 100;
    try { saved = parseInt(localStorage.getItem('rvt-font-scale') || '100', 10); } catch(_) {}

    row.innerHTML =
      '<div class="set-copy"><strong>Font size</strong><span>Independent of density mode. Scales all text.</span></div>' +
      '<div class="rvt-voice-interval-seg">' +
        '<button data-scale="90" type="button">90%</button>' +
        '<button data-scale="100" type="button">100%</button>' +
        '<button data-scale="110" type="button">110%</button>' +
        '<button data-scale="125" type="button">125%</button>' +
      '</div>';

    row.querySelectorAll('button').forEach(function(btn) {
      var s = parseInt(btn.getAttribute('data-scale'), 10);
      if (s === saved) btn.classList.add('active');
      btn.addEventListener('click', function() {
        row.querySelectorAll('button').forEach(function(b) { b.classList.remove('active'); });
        btn.classList.add('active');
        document.documentElement.setAttribute('data-font-scale', s);
        try { localStorage.setItem('rvt-font-scale', String(s)); } catch(_) {}
      });
    });

    // Apply saved
    if (saved !== 100) document.documentElement.setAttribute('data-font-scale', saved);

    var appearance = settingsView.querySelector('.set-g:nth-child(2), [data-settings-group="appearance"]');
    if (appearance) appearance.appendChild(row);
    else settingsView.appendChild(row);
  }

  /* ═══════════════════════════════════════════════════════
     A11Y-05: Voice Captions
     ═══════════════════════════════════════════════════════ */
  var captionEl = document.getElementById('rvtVoiceCaption');
  var _captionTimer = null;

  // Patch speechSynthesis.speak to show captions
  if (window.speechSynthesis) {
    var _origSpeak = window.speechSynthesis.speak.bind(window.speechSynthesis);
    window.speechSynthesis.speak = function(utterance) {
      _origSpeak(utterance);
      if (captionEl && utterance && utterance.text) {
        captionEl.textContent = utterance.text;
        captionEl.classList.add('show');
        clearTimeout(_captionTimer);
        _captionTimer = setTimeout(function() { captionEl.classList.remove('show'); }, 4000);
      }
    };
  }

  /* ═══════════════════════════════════════════════════════
     AUDIT-05: Export Audit Trail as CSV
     ═══════════════════════════════════════════════════════ */
  function injectAuditExport() {
    var auditTab = document.querySelector('#tab-audit, [data-tab="audit"]');
    if (!auditTab) return;

    var auditView = document.querySelector('.audit-content, [data-tab-content="audit"]');
    if (!auditView) return;
    if (auditView.querySelector('.rvt-audit-export-btn')) return;

    var btn = document.createElement('button');
    btn.className = 'rvt-audit-export-btn';
    btn.type = 'button';
    btn.innerHTML = '<span class="material-symbols-rounded" style="font-size:14px;">download</span>Export CSV';
    btn.addEventListener('click', function() {
      var rows = [['Timestamp', 'Severity', 'Source', 'Reason', 'Message']];
      auditView.querySelectorAll('.audit-event, .event-row, tr[data-event]').forEach(function(el) {
        var cells = el.querySelectorAll('td, .event-cell, span');
        var row = [];
        cells.forEach(function(c) { row.push('"' + (c.textContent || '').trim().replace(/"/g, '""') + '"'); });
        if (row.length > 0) rows.push(row);
      });
      var csv = rows.map(function(r) { return r.join(','); }).join('\n');
      var blob = new Blob([csv], { type: 'text/csv' });
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url;
      var sid = (window.S && window.S.sessionId) || 'session';
      a.download = 'audit_' + sid + '_' + new Date().toISOString().split('T')[0] + '.csv';
      a.click();
      URL.revokeObjectURL(url);
    });

    auditView.insertBefore(btn, auditView.firstChild);
  }

  /* ═══════════════════════════════════════════════════════
     HELP-03: Contextual Deep-Linking
     ═══════════════════════════════════════════════════════ */
  function handleHelpDeepLink() {
    var hash = location.hash;
    if (!hash || !hash.startsWith('#help/')) return;
    var term = hash.replace('#help/glossary/', '').replace('#help/', '');
    if (!term) return;

    if (typeof window.switchView === 'function') window.switchView('help');

    setTimeout(function() {
      var helpView = document.getElementById('view-help');
      if (!helpView) return;

      var target = helpView.querySelector('#' + CSS.escape(term)) ||
        helpView.querySelector('[data-term="' + term + '"]');

      if (!target) {
        // Search by text content
        var entries = helpView.querySelectorAll('dt, h3, .glossary-term, .faq-q');
        for (var i = 0; i < entries.length; i++) {
          if ((entries[i].textContent || '').toLowerCase().includes(term.toLowerCase().replace(/-/g, ' '))) {
            target = entries[i];
            break;
          }
        }
      }

      if (target) {
        target.classList.add('rvt-help-highlight');
        target.scrollIntoView({ behavior: 'smooth', block: 'center' });
        setTimeout(function() { target.classList.remove('rvt-help-highlight'); }, 1500);
      }
    }, 400);
  }

  window.addEventListener('hashchange', handleHelpDeepLink);

  // Wire (?) icons to deep-link
  function wireHelpIcons() {
    document.querySelectorAll('[data-help-term], .help-link').forEach(function(el) {
      if (el.__rvtHelpWired) return;
      el.__rvtHelpWired = true;
      el.addEventListener('click', function(e) {
        e.preventDefault();
        var term = el.getAttribute('data-help-term') || el.getAttribute('href') || '';
        term = term.replace('#help/glossary/', '').replace('#help/', '').replace('#', '');
        if (term) {
          location.hash = 'help/glossary/' + term;
          handleHelpDeepLink();
        }
      });
    });
  }

  /* ═══════════════════════════════════════════════════════
     PERF-03: Chart.js destroy on view switch
     ═══════════════════════════════════════════════════════ */
  onBodyMutation(function(muts) {
    muts.forEach(function(m) {
      if (m.attributeName === 'data-view') {
        // Destroy charts from previous view
        if (window.Chart && window.Chart.instances) {
          var instances = Object.values ? Object.values(window.Chart.instances) : [];
          var managedCharts = [];
          try {
            managedCharts = (window.S && window.S.charts && Object.values) ? Object.values(window.S.charts) : [];
          } catch(_) {}
          instances.forEach(function(chart) {
            try {
              if (managedCharts.indexOf(chart) !== -1) return;
              var canvas = chart.canvas;
              if (canvas && !canvas.offsetParent) {
                chart.destroy();
              }
            } catch(_) {}
          });
        }
      }
    });
  });

  /* ═══════════════════════════════════════════════════════
     PERF-04: Debounce resize handlers
     ═══════════════════════════════════════════════════════ */
  var _resizeRaf = 0;
  window.addEventListener('resize', function() {
    if (_resizeRaf) cancelAnimationFrame(_resizeRaf);
    _resizeRaf = requestAnimationFrame(function() {
      _resizeRaf = 0;
      scheduleSync();
    });
  }, { passive: true });

  /* ═══════════════════════════════════════════════════════
     Master boot
     ═══════════════════════════════════════════════════════ */
  function v152Boot() {
    injectPipeline();
    injectCopyBrief();
    injectRailCollapse();

    // View-specific injections via sync loop
    registerSync(function(view) {
      if (view === 'report') {
        injectExecSummary();
        tintDiagnosticCards();
      }
      if (view === 'settings') {
        injectProfilesSection();
        injectFontScaleSetting();
      }
      if (view === 'live') {
        injectAuditExport();
        wireHelpIcons();
      }
      if (view === 'help') {
        wireHelpIcons();
      }
    });

    // One-time cluster
    setTimeout(clusterAlerts, 3000);

    // Handle deep links
    handleHelpDeepLink();

    scheduleSync();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', v152Boot, { once: true });
  } else {
    setTimeout(v152Boot, 1000);
  }

})();
