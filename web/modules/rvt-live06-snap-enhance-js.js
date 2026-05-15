(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  var origDoSnapshot=window.doSnapshot;
  function showSnapToast(){
    var count=(window.S&&Array.isArray(window.S.snaps))?window.S.snaps.length:0;
    var existing=document.querySelector('.rvt-snap-toast');if(existing)existing.remove();
    var toast=document.createElement('div');toast.className='rvt-snap-toast';toast.setAttribute('role','status');toast.setAttribute('aria-live','polite');
    toast.innerHTML='<span class="material-symbols-rounded" aria-hidden="true">photo_camera</span><span>'+U.escapeHtml(t('gamma.live.snap.taken',{n:count||'?'}))+'</span>';
    document.body.appendChild(toast);
    setTimeout(function(){toast.remove();},2000);
  }
  if(typeof origDoSnapshot==='function'&&!origDoSnapshot.__rvtB2bWrapped){
    window.doSnapshot=function(){
      var ret=origDoSnapshot.apply(this,arguments);
      showSnapToast();
      return ret;
    };
    window.doSnapshot.__rvtB2bWrapped=true;
  }else{
    var snapFab=document.getElementById('rvtSnapFab');
    if(snapFab&&!snapFab.__rvtB2bEnhanced){
      snapFab.addEventListener('click',function(){setTimeout(showSnapToast,50);});
      snapFab.__rvtB2bEnhanced=true;
    }
  }
  window.rvtSnapEnhance={showToast:showSnapToast};
})();
