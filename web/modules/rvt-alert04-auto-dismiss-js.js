(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  var timers=[];
  function getDismissSeconds(){var v=localStorage.getItem('rvt-alert-auto-dismiss-s');var n=Number(v);return Number.isFinite(n)&&n>=0&&n<=120?n:30;}
  function attachDismiss(node){
    if(!node||node.getAttribute('data-auto-dismiss')==='attached')return;
    var sev=node.getAttribute('data-alert-severity')||'info';
    if(sev==='critical')return;
    var seconds=getDismissSeconds();if(seconds<=0)return;
    node.setAttribute('data-auto-dismiss','attached');
    node.style.setProperty('--dismiss-s',seconds+'s');
    var bar=document.createElement('div');bar.className='rvt-alert-dismiss-bar';bar.setAttribute('aria-hidden','true');
    node.appendChild(bar);
    var tid=setTimeout(function(){
      node.style.opacity='0';node.style.transition='opacity 200ms ease';
      setTimeout(function(){node.remove();},220);
    },seconds*1000);
    timers.push(tid);
  }
  function scanForDismiss(){
    var body=document.getElementById('dvBody');if(!body)return;
    Array.prototype.forEach.call(body.children,function(child){
      if(child.nodeType===1)attachDismiss(child);
    });
  }
  var mutObs=null;
  function observeDrawer(){
    var dvBody=document.getElementById('dvBody');if(!dvBody||mutObs)return;
    mutObs=new MutationObserver(function(){scanForDismiss();});
    mutObs.observe(dvBody,{childList:true,subtree:false});
    scanForDismiss();
  }
  window.rvtAlertAutoDismiss={scan:scanForDismiss,getSeconds:getDismissSeconds,clearTimers:function(){timers.forEach(clearTimeout);timers=[];}};
  U.bootSoon(observeDrawer);
})();
