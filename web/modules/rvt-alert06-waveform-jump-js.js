(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  function extractTimestamp(node){
    if(!node)return null;
    var text=node.textContent||'';
    var match=text.match(/(\d{2}:\d{2}:\d{2})/);
    return match?match[1]:null;
  }
  function jumpToWaveform(ts){
    if(!ts){U.toast(t('gamma.alert.jump.unavailable'));return false;}
    if(typeof window.switchView==='function')window.switchView('live');
    if(typeof window.switchTab==='function')window.switchTab('tab-waves');
    if(typeof window.rvtWaveformSeek==='function'){window.rvtWaveformSeek(ts);return true;}
    U.toast(t('gamma.alert.jump.unavailable'));
    return false;
  }
  function addJumpButtons(){
    var body=document.getElementById('dvBody');if(!body)return;
    Array.prototype.forEach.call(body.children,function(child){
      if(child.nodeType!==1||child.querySelector('.rvt-alert-jump-btn'))return;
      var ts=extractTimestamp(child);if(!ts)return;
      var btn=document.createElement('button');btn.type='button';btn.className='rvt-alert-jump-btn';
      btn.setAttribute('aria-label',t('gamma.alert.jump.label'));btn.title=t('gamma.alert.jump.label');
      btn.innerHTML='<span class="material-symbols-rounded" aria-hidden="true" style="font-size:16px">open_in_new</span>';
      btn.setAttribute('data-jump-ts',ts);
      btn.addEventListener('click',function(e){e.stopPropagation();jumpToWaveform(ts);});
      child.appendChild(btn);
    });
  }
  var mutObs=null;
  function observe(){
    var dvBody=document.getElementById('dvBody');if(!dvBody||mutObs)return;
    mutObs=new MutationObserver(function(){addJumpButtons();});
    mutObs.observe(dvBody,{childList:true,subtree:false});
    addJumpButtons();
  }
  window.rvtAlertJump={jump:jumpToWaveform,scan:addJumpButtons};
  U.bootSoon(observe);
})();
