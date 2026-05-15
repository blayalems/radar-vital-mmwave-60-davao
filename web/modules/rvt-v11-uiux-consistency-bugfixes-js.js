(function(){
  const raf = window.requestAnimationFrame || function(fn){ return setTimeout(fn, 16); };

  function makeIcon(name){
    const icon = document.createElement('span');
    icon.className = 'material-symbols-rounded';
    icon.setAttribute('aria-hidden', 'true');
    icon.textContent = name;
    return icon;
  }

  var _lastCrumbView = '';
  var _viewLabels = {home:'Workflow',live:'Live Dashboard',report:'Report',help:'Help',settings:'Settings'};

  function ensureHomeCrumb(){
    const trail = document.getElementById('crumbTrail');
    if(!trail) return;
    const view = document.body?.dataset?.view || 'home';
    const viewLabel = _viewLabels[view] || view;
    const existingHome = Array.prototype.filter.call(trail.querySelectorAll('.uiux-home-crumb'), function(node){
      return (node.textContent || '').trim() === 'Home';
    }).length;
    const existingCurrent = Array.prototype.filter.call(trail.querySelectorAll('.uiux-crumb-current'), function(node){
      return (node.textContent || '').trim() === viewLabel;
    }).length;
    if(_lastCrumbView === view && existingHome === 1 && existingCurrent === 1 && trail.children.length <= 3) return;

    /* Replace stale or duplicated crumbs instead of appending to another renderer. */
    trail.replaceChildren();
    _lastCrumbView = view;

    /* ── Home segment: 🏠 Home ── */
    const homeBtn = document.createElement('button');
    homeBtn.type = 'button';
    homeBtn.className = 'crumb-seg uiux-home-crumb';
    homeBtn.appendChild(makeIcon('home'));
    homeBtn.appendChild(document.createTextNode('Home'));
    homeBtn.addEventListener('click', function(){
      if(typeof window.switchView === 'function') window.switchView('home');
    });
    trail.appendChild(homeBtn);

    /* ── Separator ── */
    const sep = document.createElement('span');
    sep.className = 'crumb-sep';
    sep.textContent = '/';
    sep.setAttribute('aria-hidden', 'true');
    trail.appendChild(sep);

    /* ── Current view label ── */
    const label = document.createElement('span');
    label.className = 'crumb-seg uiux-crumb-current';
    label.textContent = viewLabel;
    label.style.cssText = 'cursor:default;background:var(--brand-100);color:var(--brand-500);border-color:transparent;pointer-events:none;';
    trail.appendChild(label);
  }

  function ensureHelpHome(){
    const quick = document.querySelector('#view-help .help-toolbar, body[data-view="help"] .help-toolbar, #view-help .help-quick, body[data-view="help"] .help-quick, .help-page .help-toolbar, .help-page .help-quick');
    if(!quick || quick.querySelector('.uiux-help-home')) return;
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'chip-btn uiux-help-home';
    btn.appendChild(makeIcon('arrow_back'));
    btn.appendChild(document.createTextNode('Home'));
    btn.addEventListener('click', function(){
      if(typeof window.switchView === 'function') window.switchView('home');
    });
    quick.prepend(btn);
  }

  function syncExportButton(){
    const btn = document.querySelector('.topbar-actions > .tb-export');
    if(!btn) return;
    const mobile = window.matchMedia && window.matchMedia('(max-width: 760px)').matches;
    const set = function(name, value){ btn.style.setProperty(name, value, 'important'); };
    if(mobile){
      set('width', '44px');
      set('min-width', '44px');
      set('max-width', '44px');
      set('height', '44px');
      set('min-height', '44px');
      set('padding', '0');
      set('font-size', '0');
      set('gap', '0');
      set('flex', '0 0 44px');
      set('overflow', 'hidden');
    } else {
      set('width', 'auto');
      set('min-width', '132px');
      set('max-width', 'none');
      set('height', '52px');
      set('min-height', '52px');
      set('padding', '0 22px');
      set('font-size', '15px');
      set('gap', '8px');
      set('flex', '0 0 auto');
      set('overflow', 'visible');
    }
  }

  function syncSwitchState(){
    document.querySelectorAll('.sw').forEach(function(sw){
      const pressed = sw.getAttribute('aria-pressed');
      const isOn = sw.classList.contains('on') || pressed === 'true';
      sw.dataset.state = isOn ? 'on' : 'off';
      if(!sw.getAttribute('aria-label')){
        sw.setAttribute('aria-label', isOn ? 'On' : 'Off');
      }
    });
  }

  function numericSeries(){
    const source = window.S?.spark?.fps;
    if(Array.isArray(source) && source.length > 1){
      return source.map(Number).filter(Number.isFinite).slice(-96);
    }
    const fps = Number(window.S?.lastPayload?.meta?.fps ?? window.S?.lastPayload?.fps ?? 20);
    const base = Number.isFinite(fps) ? fps : 20;
    return Array.from({length: 48}, function(_, i){
      return base + Math.sin(i / 4) * .12 + Math.cos(i / 9) * .06;
    });
  }

  function drawFpsSpark(){
    const canvas = document.getElementById('kpiFpsSpark');
    if(!canvas || typeof canvas.getContext !== 'function') return;
    const rect = canvas.getBoundingClientRect();
    if(rect.width < 2 || rect.height < 2) return;
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const w = Math.max(1, Math.round(rect.width * dpr));
    const h = Math.max(1, Math.round(rect.height * dpr));
    if(canvas.width !== w || canvas.height !== h){
      canvas.width = w;
      canvas.height = h;
    }
    const ctx = canvas.getContext('2d');
    const vals = numericSeries();
    if(vals.length < 2) return;
    const mean = vals.reduce(function(a,b){ return a + b; }, 0) / vals.length;
    let min = Math.min.apply(null, vals);
    let max = Math.max.apply(null, vals);
    if(max - min < .5){
      min = mean - 1;
      max = mean + 1;
    }
    const padX = 10 * dpr;
    const padY = 12 * dpr;
    const plotW = Math.max(1, w - padX * 2);
    const plotH = Math.max(1, h - padY * 2);
    const x = function(i){ return padX + (i / Math.max(1, vals.length - 1)) * plotW; };
    const y = function(v){ return padY + (1 - ((v - min) / Math.max(.001, max - min))) * plotH; };
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    ctx.lineWidth = 1 * dpr;
    ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--line') || 'rgba(148,163,184,.22)';
    ctx.globalAlpha = .45;
    for(let i = 1; i < 4; i++){
      const gy = padY + (plotH * i / 4);
      ctx.beginPath();
      ctx.moveTo(padX, gy);
      ctx.lineTo(w - padX, gy);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    const brand = getComputedStyle(document.documentElement).getPropertyValue('--brand-500').trim() || '#2563eb';
    const grad = ctx.createLinearGradient(0, padY, 0, h - padY);
    grad.addColorStop(0, 'rgba(37, 99, 235, .26)');
    grad.addColorStop(1, 'rgba(37, 99, 235, 0)');
    ctx.beginPath();
    vals.forEach(function(v, i){
      const px = x(i), py = y(v);
      if(i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.lineTo(w - padX, h - padY);
    ctx.lineTo(padX, h - padY);
    ctx.closePath();
    ctx.fillStyle = grad;
    ctx.fill();
    ctx.beginPath();
    vals.forEach(function(v, i){
      const px = x(i), py = y(v);
      if(i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.lineWidth = 2.25 * dpr;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.strokeStyle = brand;
    ctx.stroke();
    const ly = y(vals[vals.length - 1]);
    ctx.beginPath();
    ctx.arc(x(vals.length - 1), ly, 3.4 * dpr, 0, Math.PI * 2);
    ctx.fillStyle = '#fff';
    ctx.fill();
    ctx.lineWidth = 2 * dpr;
    ctx.strokeStyle = brand;
    ctx.stroke();
    ctx.restore();
  }

  function forceConsistency(){
    try {
      const isHome = document.body.getAttribute('data-view') === 'home';

      /* 1. Force Readiness Ring (72px to match v11 density) */
      const rings = document.querySelectorAll('#homeReadinessRing, .home-ring');
      rings.forEach(function(ring){
        const s = ring.style;
        const set = function(p, v){ s.setProperty(p, v, 'important'); };
        set('width', '72px');
        set('height', '72px');
        set('min-width', '72px');
        set('min-height', '72px');
        set('max-width', '72px');
        set('max-height', '72px');
        set('flex', '0 0 72px');
        set('aspect-ratio', '1/1');
        set('border-radius', '50%');
        set('align-self', 'center');
        set('justify-self', 'center');
        set('display', 'grid');
        set('place-items', 'center');
        set('position', 'relative');

        const pct = parseFloat(getComputedStyle(ring).getPropertyValue('--pct')) || 0;
        if(pct <= 0){
          set('background', '#e2e8f0');
        } else {
          s.removeProperty('background');
        }
      });

      /* 2. Force Card Containers and Headers */
      /* overflow+border on all cards; padding:0 only on filled-header cards */
      const allCardSelectors = ['#setupCard', '.home-preview', '.sys-preflight', '.hero-ready', '.card', '.sys-card'];
      allCardSelectors.forEach(function(sel){
        document.querySelectorAll(sel).forEach(function(c){
          c.style.setProperty('overflow', 'hidden', 'important');
          c.style.setProperty('border', '1px solid #dfe8f7', 'important');
        });
      });
      const filledHeaderSelectors = ['#setupCard', '.home-preview', '.sys-preflight', '.hero-ready'];
      filledHeaderSelectors.forEach(function(sel){
        document.querySelectorAll(sel).forEach(function(c){
          c.style.setProperty('padding', '0', 'important');
        });
      });

      const headerSelectors = ['#setupCard>.ch', '.home-preview>.hero-preview-head', '.sys-preflight>.card-header', '.hero-preview-head'];
      headerSelectors.forEach(function(sel){
        document.querySelectorAll(sel).forEach(function(h){
          const s = h.style;
          s.setProperty('margin', '-2px -2px 0 -2px', 'important');
          s.setProperty('width', 'calc(100% + 4px)', 'important');
          s.setProperty('border-radius', '22px 22px 0 0', 'important');
          s.setProperty('z-index', '10', 'important');
          s.setProperty('position', 'relative', 'important');
          s.setProperty('border', '0', 'important');
          /* The Seal: box-shadow + top offset to kill the white hairline */
          s.setProperty('box-shadow', '0 -1px 0 0 rgba(0,0,0,0.02)', 'important');
          s.setProperty('top', '-1px', 'important');
        });
      });

      /* 3. Fix Breadcrumb Layout and hide eyebrow */
      const trail = document.querySelector('.topbar .crumb-trail');
      if(trail){
        trail.style.setProperty('display', 'flex', 'important');
        trail.style.setProperty('flex-direction', 'row', 'important');
        trail.style.setProperty('align-items', 'center', 'important');
        trail.style.setProperty('gap', '8px', 'important');
        trail.style.setProperty('margin', '0', 'important');
      }

      if(isHome){
        const eb = document.getElementById('crumbEyebrow') || document.querySelector('.crumb-eyebrow');
        if(eb){
          eb.style.setProperty('display', 'none', 'important');
          eb.style.setProperty('opacity', '0', 'important');
          eb.style.setProperty('pointer-events', 'none', 'important');
        }
      }
    } catch(e) { console.error('forceConsistency error:', e); }
  }

  function sync(){
    ensureHomeCrumb();
    ensureHelpHome();
    syncExportButton();
    syncSwitchState();
    forceConsistency();
    raf(drawFpsSpark);
  }

  function wrapRender(){
    if(typeof window.render === 'function' && !window.render.__uiuxV11Wrapped){
      const orig = window.render;
      window.render = function(){
        const result = orig.apply(this, arguments);
        setTimeout(sync, 60);
        return result;
      };
      window.render.__uiuxV11Wrapped = true;
    }
    if(typeof window.switchView === 'function' && !window.switchView.__uiuxV11Wrapped){
      const origSwitch = window.switchView;
      window.switchView = function(){
        const result = origSwitch.apply(this, arguments);
        setTimeout(sync, 60);
        setTimeout(sync, 260);
        return result;
      };
      window.switchView.__uiuxV11Wrapped = true;
    }
  }

  function boot(){
    wrapRender();
    sync();
    let syncQueued = 0;
    const queueSync = function(){
      if(syncQueued) return;
      syncQueued = setTimeout(function(){
        syncQueued = 0;
        sync();
      }, 60);
    };
    const obs = window.rvtTrackMutationObserver(new MutationObserver(queueSync));
    obs.observe(document.body, {attributes:true, attributeFilter:['data-view']});
    window.addEventListener('resize', function(){ raf(drawFpsSpark); }, {passive:true});
    /* Frequent interval for the first 10 seconds to catch early renders */
    let count = 0;
    const itv = setInterval(function(){
      sync();
      if(++count > 20) { clearInterval(itv); setInterval(sync, 1500); }
    }, 500);
  }

  if(document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot, {once:true});
  else boot();
})();
