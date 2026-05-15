(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var fab=null,lastClick=0,lastId=null;
  function count(){var S=window.S||{};return Array.isArray(S.snaps)?S.snaps.length:0;}
  function updateBadge(){if(!fab)return;var b=fab.querySelector('.rvt-live06-fab-badge');if(!b)return;var n=count();b.textContent=String(n);b.style.display=n>0?'grid':'none';}
  function visible(){return document.body.dataset.view==='live';}
  function show(){ensure();fab.dataset.visible='true';}
  function hide(){if(fab)fab.dataset.visible='false';}
  function updateVis(){if(visible())show();else hide();}
  function capture(){
    window.S=window.S||{};window.S.snaps=Array.isArray(window.S.snaps)?window.S.snaps:[];
    var before=window.S.snaps.length;var id=null;
    if(typeof window.snapshotNow==='function')id=window.snapshotNow();
    else if(typeof window.doSnapshot==='function')id=window.doSnapshot();
    if(window.S.snaps.length===before){id=id||('snap-'+(before+1));window.S.snaps.push({id:id,timestamp:new Date().toISOString(),kpis:{hr:window.S.live&&window.S.live.hr,rr:window.S.live&&window.S.live.rr,range_cm:window.S.live&&window.S.live.range_cm},waveform:(window.S.waveform&&window.S.waveform.samples)||[]});}
    lastId=id||(window.S.snaps[window.S.snaps.length-1]&&window.S.snaps[window.S.snaps.length-1].id);
    updateBadge();
    if(fab&&!window.matchMedia('(prefers-reduced-motion: reduce)').matches){fab.classList.add('rvt-captured');setTimeout(function(){fab.classList.remove('rvt-captured');},220);}
    return lastId;
  }
  function labelPopover(){ensure();var old=document.getElementById('rvt-live06-label-popover');if(old)old.remove();var pop=document.createElement('div');pop.id='rvt-live06-label-popover';pop.className='rvt-live06-label-popover';pop.innerHTML='<input id="rvt-live06-label-input" maxlength="60" aria-label="'+R.esc(R.t('gamma2b.live.snapshotLabel'))+'"><button type="button">'+R.esc(R.t('gamma2b.live.saveLabel'))+'</button>';document.body.appendChild(pop);pop.querySelector('button').addEventListener('click',function(){var snap=(window.S.snaps||[]).find(function(s){return s.id===lastId;});if(snap)snap.label=pop.querySelector('input').value.slice(0,60);pop.remove();});pop.querySelector('input').focus();}
  function captureWithLabel(){if(typeof window.__labelCaptureCount==='number')window.__labelCaptureCount+=1;var id=capture();labelPopover();return id;}
  function attachFab(node){
    if(!node||node.__rvtLive06Attached)return node;
    node.__rvtLive06Attached=true;
    node.dataset.snapFab='true';
    node.classList.add('rvt-live06-fab');
    node.setAttribute('aria-label',R.t('gamma2b.live.snapshot'));
    if(!node.querySelector('.rvt-live06-fab-badge'))node.insertAdjacentHTML('beforeend','<span class="rvt-live06-fab-badge">0</span>');
    node.addEventListener('click',function(){var now=Date.now();if(now-lastClick<=300){lastClick=0;window.rvtSnapFAB.captureWithLabel();}else{lastClick=now;window.rvtSnapFAB.capture();}});
    node.addEventListener('dblclick',function(e){e.preventDefault();window.rvtSnapFAB.captureWithLabel();});
    node.addEventListener('keydown',function(e){if(e.key==='Enter'){e.preventDefault();if(e.shiftKey)window.rvtSnapFAB.captureWithLabel();else window.rvtSnapFAB.capture();}});
    return node;
  }
  function ensure(){if(fab)return fab;fab=document.getElementById('rvt-live06-fab')||document.getElementById('rvtSnapFab');if(!fab){fab=document.createElement('button');fab.type='button';fab.id='rvt-live06-fab';fab.innerHTML='<span class="material-symbols-rounded" aria-hidden="true">photo_camera</span>';document.body.appendChild(fab);}attachFab(fab);updateBadge();return fab;}
  document.addEventListener('keydown',function(e){var a=document.activeElement;if(e.key&&e.key.toUpperCase()==='P'&&!e.ctrlKey&&!e.metaKey&&!e.altKey&&visible()&&!(a&&(a.tagName==='TEXTAREA'||a.tagName==='INPUT'||a.isContentEditable))){e.preventDefault();e.stopImmediatePropagation();window.rvtSnapFAB.capture();}},true);
  document.addEventListener('click',function(e){var node=e.target&&e.target.closest&&e.target.closest('#rvt-live06-fab,#rvtSnapFab,[data-snap-fab]');if(!node)return;attachFab(node);},true);
  document.addEventListener('dblclick',function(e){var node=e.target&&e.target.closest&&e.target.closest('#rvt-live06-fab,#rvtSnapFab,[data-snap-fab]');if(!node)return;e.preventDefault();if(window.rvtSnapFAB)window.rvtSnapFAB.captureWithLabel();},true);
  var oldSwitch=window.switchView;if(typeof oldSwitch==='function'&&!oldSwitch.__rvtSnapFab){window.switchView=function(v){var r=oldSwitch.apply(this,arguments);setTimeout(updateVis,0);return r;};window.switchView.__rvtSnapFab=true;}
  window.rvtSnapFAB={capture:capture,captureWithLabel:captureWithLabel,getCount:count,show:show,hide:hide};
  R.boot(function(){ensure();updateVis();});
})();
