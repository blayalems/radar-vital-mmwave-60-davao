(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  var paused=(localStorage.getItem('rvt-kpi-animations')||'1')==='0';
  function apply(){
    document.body.classList.toggle('rvt-kpi-anim-paused',paused);
    localStorage.setItem('rvt-kpi-animations',paused?'0':'1');
    var btn=document.getElementById('rvtKpiAnimToggle');if(!btn)return;
    btn.setAttribute('aria-pressed',paused?'true':'false');
    var icon=btn.querySelector('.material-symbols-rounded');
    if(icon)icon.textContent=paused?'play_arrow':'pause';
  }
  function toggle(){
    paused=!paused;apply();
    U.toast(paused?t('gamma.live.anim.paused'):t('gamma.live.anim.resumed'));
  }
  function renderToggle(){
    var kpiRow=document.querySelector('.kpi-row')||document.querySelector('.kpi-grid');
    if(!kpiRow||document.getElementById('rvtKpiAnimToggle'))return;
    var btn=document.createElement('button');btn.type='button';btn.id='rvtKpiAnimToggle';btn.className='rvt-anim-toggle';
    btn.setAttribute('aria-label',t('gamma.live.anim.label'));btn.title=t('gamma.live.anim.label');
    btn.innerHTML='<span class="material-symbols-rounded" aria-hidden="true"></span><span>'+U.escapeHtml(t('gamma.live.anim.label'))+'</span>';
    btn.addEventListener('click',function(){window.rvtKpiAnimPause.toggle();});
    kpiRow.parentNode.insertBefore(btn,kpiRow.nextSibling);
  }
  window.rvtKpiAnimPause={toggle:toggle,isPaused:function(){return paused;},apply:apply};
  U.bootSoon(function(){renderToggle();apply();});
})();
