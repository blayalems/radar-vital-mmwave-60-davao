(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var STRINGS={
    'gamma.rpt.sticky.pass':'Session passed','gamma.rpt.sticky.warn':'Session warning','gamma.rpt.sticky.fail':'Session failed',
    'gamma.rpt.sticky.scrollTop':'Back to top',
    'gamma.rpt06.export':'Export report','gamma.rpt06.printing':'Preparing print…',
    'gamma.alert.sev.info':'Info','gamma.alert.sev.warn':'Warning','gamma.alert.sev.critical':'Critical',
    'gamma.alert.dismiss.label':'Auto-dismiss in {s}s','gamma.alert.dismiss.dismissed':'Alert dismissed',
    'gamma.alert.jump.label':'Jump to waveform','gamma.alert.jump.unavailable':'Waveform view not available',
    'gamma.alert.sound.on':'Alert sounds on','gamma.alert.sound.off':'Alert sounds off','gamma.alert.sound.label':'Toggle alert sounds',
    'gamma.live.tooltip.title':'Computation details','gamma.live.tooltip.unavailable':'Computation details unavailable','gamma.live.tooltip.value':'Current value','gamma.live.tooltip.source':'Data source','gamma.live.tooltip.buffer':'Buffer size',
    'gamma.live.mode.switched':'Mode switched to {mode}','gamma.live.mode.shortcut':'Press M to toggle mode',
    'gamma.live.snap.taken':'Snapshot #{n} captured','gamma.live.snap.shortcut':'Press P to capture',
    'gamma.live.anim.paused':'KPI animations paused','gamma.live.anim.resumed':'KPI animations resumed','gamma.live.anim.label':'Toggle KPI animations'
  };
  function extendB2B(){window.RVT_STRINGS=window.RVT_STRINGS||{en:{}};window.RVT_STRINGS.en=window.RVT_STRINGS.en||{};Object.keys(STRINGS).forEach(function(k){if(!(k in window.RVT_STRINGS.en))window.RVT_STRINGS.en[k]=STRINGS[k];});}
  function t(k,v){extendB2B();return U.t(k,v);}
  function registerB2BManifest(){if(!window.rvtStorage||!window.rvtStorage.manifest)return;var m=window.rvtStorage.manifest;
    if(!m['rvt-alert-auto-dismiss-s'])m['rvt-alert-auto-dismiss-s']={kind:'scalar',defaultValue:'30',validate:function(v){var n=Number(v);return Number.isFinite(n)&&n>=0&&n<=120;}};
    if(!m['rvt-kpi-animations'])m['rvt-kpi-animations']={kind:'scalar',defaultValue:'1',validate:function(v){return v==='0'||v==='1';}};
    if(!m['rvt-report-sticky-hero'])m['rvt-report-sticky-hero']={kind:'scalar',defaultValue:'1',validate:function(v){return v==='0'||v==='1';}};
  }
  extendB2B();registerB2BManifest();
  window.rvtGammaBatch2b={t:t,registerB2BManifest:registerB2BManifest};

  var bar=null,hero=null,observer=null;
  function readVerdict(){
    hero=document.querySelector('.verdict-hero');if(!hero)return null;
    var kind='pass';if(hero.classList.contains('warn'))kind='warn';if(hero.classList.contains('fail'))kind='fail';
    var label=hero.querySelector('.vh-kind');
    var metrics=hero.querySelectorAll('.vh-metric .metric-val,.vh-metric .metric-badge');
    var metricTexts=[];Array.prototype.forEach.call(metrics,function(m){if(m.textContent.trim())metricTexts.push(m.textContent.trim());});
    return{kind:kind,label:label?label.textContent.trim():'',metrics:metricTexts.slice(0,3)};
  }
  function ensureBar(){
    if(bar)return bar;
    var report=document.getElementById('view-report')||document.querySelector('[data-view="report"]');if(!report)return null;
    bar=document.createElement('div');bar.className='rvt-sticky-verdict';bar.id='rvtStickyVerdict';bar.setAttribute('role','banner');bar.setAttribute('aria-label','Session verdict summary');
    report.insertBefore(bar,report.firstChild);return bar;
  }
  function updateBar(visible){
    if(!bar)return;
    var v=readVerdict();if(!v){bar.setAttribute('data-visible','false');return;}
    bar.className='rvt-sticky-verdict '+v.kind;
    var icon=v.kind==='pass'?'check_circle':v.kind==='warn'?'warning':'cancel';
    bar.innerHTML='<span class="material-symbols-rounded rvt-sv-icon" aria-hidden="true">'+icon+'</span><span class="rvt-sv-label">'+U.escapeHtml(v.label||t('gamma.rpt.sticky.'+v.kind))+'</span><div class="rvt-sv-metrics">'+v.metrics.map(function(m){return'<span>'+U.escapeHtml(m)+'</span>';}).join('')+'</div>';
    bar.setAttribute('data-visible',visible?'true':'false');
  }
  function initObserver(){
    hero=document.querySelector('.verdict-hero');if(!hero||observer)return;
    ensureBar();
    if(!('IntersectionObserver' in window)){return;}
    observer=new IntersectionObserver(function(entries){
      entries.forEach(function(entry){updateBar(!entry.isIntersecting);});
    },{threshold:0,rootMargin:'-1px 0px 0px 0px'});
    observer.observe(hero);
  }
  window.rvtStickyVerdict={refresh:function(){updateBar(bar&&bar.getAttribute('data-visible')==='true');},destroy:function(){if(observer){observer.disconnect();observer=null;}if(bar){bar.remove();bar=null;}}};
  U.bootSoon(function(){
    if(localStorage.getItem('rvt-report-sticky-hero')==='0')return;
    initObserver();
    var origSwitch=window.switchView;if(typeof origSwitch==='function'&&!origSwitch.__rvtStickyWrapped){window.switchView=function(v){var r=origSwitch.apply(this,arguments);if(v==='report')setTimeout(initObserver,100);return r;};window.switchView.__rvtStickyWrapped=true;}
  });
})();
