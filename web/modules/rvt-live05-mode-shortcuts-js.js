(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  document.addEventListener('keydown',function(e){
    if(e.ctrlKey||e.metaKey||e.altKey)return;
    var active=document.activeElement;
    if(active&&(active.tagName==='TEXTAREA'||active.tagName==='INPUT'||active.isContentEditable))return;
    if(document.body.getAttribute('data-view')!=='live')return;
    if(e.key==='m'||e.key==='M'){
      e.preventDefault();
      var current=localStorage.getItem('rvt-live-mode')||'simple';
      var next=current==='simple'?'advanced':'simple';
      if(typeof window.setLiveMode==='function'){window.setLiveMode(next);}
      else{localStorage.setItem('rvt-live-mode',next);document.body.setAttribute('data-live-mode',next);}
      U.toast(t('gamma.live.mode.switched',{mode:next}));
    }
  });
  window.rvtModeShortcut={getBinding:function(){return'M';}};
})();
