/* SEC-02: Auto-lock on Idle */
(function() {
  'use strict';

  /* Register storage keys with the manifest */
  function registerKeys() {
    var m = window.rvtStorage && window.rvtStorage.manifest;
    if (!m) return;
    m['rvt-idle-timeout-min'] = { kind: 'scalar', defaultValue: '5',
      validate: function(v){ var n=Number(v); return Number.isFinite(n) && n>=0 && n<=60; } };
    m['rvt-lock-keepalive'] = { kind: 'scalar', defaultValue: '1',
      validate: function(v){ return v==='0'||v==='1'; } };
    m['rvt-lock-pin-hash'] = { kind: 'scalar', defaultValue: '',
      validate: function(v){ return typeof v==='string' && v.length<=128; } };
  }

  var ACTIVITY_EVENTS = ['mousemove','keydown','click','touchstart','pointerdown','scroll'];
  var state = { locked: false, lastActivity: Date.now(), timer: null, overlay: null };

  function getTimeoutMs() {
    var stored = null;
    try { stored = localStorage.getItem('rvt-idle-timeout-min'); } catch(_) {}
    var min = Number(stored || '5') || 5;
    return min <= 0 ? 0 : min * 60000;
  }

  function isKeepAlive() {
    try {
      var ka = localStorage.getItem('rvt-lock-keepalive');
      if (ka === '0') return false;
    } catch(_) {}
    try {
      var s = window.S;
      if (s && s.disc !== undefined && s.disc < 3) return true;
      if (s && s.ctl && s.ctl.session && s.ctl.session.active) return true;
    } catch(_) {}
    return false;
  }

  function hashPin(pin) {
    return crypto.subtle.digest('SHA-256', new TextEncoder().encode('rvt:lock:' + pin))
      .then(function(buf) {
        return Array.from(new Uint8Array(buf))
          .map(function(b){ return b.toString(16).padStart(2,'0'); }).join('');
      });
  }

  function buildOverlay() {
    var el = document.getElementById('rvt-lock-overlay');
    if (el) { state.overlay = el; return el; }
    el = document.createElement('div');
    el.id = 'rvt-lock-overlay';
    el.setAttribute('role', 'dialog');
    el.setAttribute('aria-modal', 'true');
    el.setAttribute('aria-labelledby', 'rvtLockTitle');
    el.innerHTML =
      '<span class="rvt-lock-icon material-symbols-rounded" aria-hidden="true">lock</span>' +
      '<h2 class="rvt-lock-title" id="rvtLockTitle">Session paused</h2>' +
      '<p class="rvt-lock-subtitle">Authenticate to resume monitoring. Data collection continues in the background.</p>' +
      '<div class="rvt-lock-pin-row">' +
        '<input id="rvtLockPinInput" type="password" placeholder="PIN (optional)" autocomplete="off" maxlength="16" aria-label="Enter PIN to unlock" />' +
        '<button id="rvtLockUnlockBtn" type="button">Resume</button>' +
      '</div>' +
      '<p class="rvt-lock-no-pin-hint" id="rvtLockNoPinHint">No PIN set — click Resume to continue.</p>' +
      '<p class="rvt-lock-err" id="rvtLockErr" role="alert" aria-live="polite"></p>' +
      '<p class="rvt-lock-timer" id="rvtLockTimer" aria-live="polite"></p>';
    document.body.appendChild(el);

    var btn   = el.querySelector('#rvtLockUnlockBtn');
    var input = el.querySelector('#rvtLockPinInput');
    var errEl = el.querySelector('#rvtLockErr');
    var hint  = el.querySelector('#rvtLockNoPinHint');

    function tryUnlock() {
      var stored = null;
      try { stored = localStorage.getItem('rvt-lock-pin-hash') || ''; } catch(_) {}
      if (!stored) { unlock(); return; }
      var entered = input.value;
      if (!entered) { errEl.textContent = 'Enter your PIN.'; input.focus(); return; }
      hashPin(entered).then(function(h) {
        if (h === stored) { errEl.textContent=''; input.value=''; unlock(); }
        else { errEl.textContent='Incorrect PIN.'; input.value=''; input.focus(); }
      });
    }

    function syncHint() {
      var stored = null;
      try { stored = localStorage.getItem('rvt-lock-pin-hash') || ''; } catch(_) {}
      hint.style.display = stored ? 'none' : '';
      input.style.display = stored ? '' : 'none';
    }

    btn.addEventListener('click', tryUnlock);
    input.addEventListener('keydown', function(e){ if (e.key==='Enter') tryUnlock(); });
    el.addEventListener('focus', syncHint, true);
    syncHint();

    state.overlay = el;
    return el;
  }

  function lock() {
    if (state.locked) return;
    state.locked = true;
    document.body.dataset.autoLocked = 'true';
    var ov = buildOverlay();
    ov.style.display = 'flex';
    requestAnimationFrame(function(){ ov.classList.add('rvt-lock-visible'); });
    var inp = ov.querySelector('#rvtLockPinInput');
    if (inp) setTimeout(function(){ inp.focus(); }, 120);

    var lockAt = Date.now();
    var timerEl = ov.querySelector('#rvtLockTimer');
    var tID = setInterval(function(){
      if (!state.locked){ clearInterval(tID); if(timerEl) timerEl.textContent=''; return; }
      var mins = Math.floor((Date.now()-lockAt)/60000);
      if (timerEl) timerEl.textContent = mins>0 ? 'Locked for '+mins+' min' : '';
    }, 30000);

    if (window.rvtAnnounce) window.rvtAnnounce('Session paused due to inactivity.');
  }

  function unlock() {
    state.locked = false;
    state.lastActivity = Date.now();
    delete document.body.dataset.autoLocked;
    var ov = state.overlay;
    if (ov) {
      ov.classList.remove('rvt-lock-visible');
      setTimeout(function(){ if (!state.locked) ov.style.display='none'; }, 220);
    }
  }

  function resetActivity(){ state.lastActivity = Date.now(); }

  function tick() {
    if (state.locked) return;
    var ms = getTimeoutMs();
    if (!ms) return;
    if (isKeepAlive()) return;
    if (Date.now() - state.lastActivity >= ms) lock();
  }

  function boot() {
    registerKeys();
    ACTIVITY_EVENTS.forEach(function(ev){
      document.addEventListener(ev, resetActivity, { passive: true, capture: true });
    });
    state.timer = setInterval(tick, 10000);
    buildOverlay();

    window.rvtAutoLock = {
      lock: lock,
      unlock: unlock,
      isLocked: function(){ return state.locked; },
      setPin: function(pin){
        if (!pin){ try{ localStorage.removeItem('rvt-lock-pin-hash'); }catch(_){} return Promise.resolve(); }
        return hashPin(pin).then(function(h){ try{ localStorage.setItem('rvt-lock-pin-hash',h); }catch(_){} });
      },
      clearPin: function(){ try{ localStorage.removeItem('rvt-lock-pin-hash'); }catch(_){} },
      hasPin: function(){ try{ return !!(localStorage.getItem('rvt-lock-pin-hash')); }catch(_){ return false; } },
      getTimeoutMin: function(){ return Number(getTimeoutMs()/60000); },
      setTimeoutMin: function(m){
        var clamped = Math.max(0,Math.min(60,Number(m)||0));
        try{ localStorage.setItem('rvt-idle-timeout-min',String(clamped)); }catch(_){}
      }
    };
  }

  if (document.readyState==='loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot,500); }, {once:true});
  } else { setTimeout(boot,500); }
})();
