(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  function enhanceAlertItem(node){
    if(!node||node.querySelector('.rvt-alert-sev'))return;
    var text=(node.textContent||'').toLowerCase();
    var sev='info';
    if(/critical|fatal|error|bad/.test(text))sev='critical';
    else if(/warn|warning/.test(text))sev='warn';
    var badge=document.createElement('span');badge.className='rvt-alert-sev';badge.setAttribute('data-sev',sev);
    badge.textContent=t('gamma.alert.sev.'+sev);
    node.insertBefore(badge,node.firstChild);
    node.setAttribute('data-alert-severity',sev);
  }
  function scanDrawer(){
    var body=document.getElementById('dvBody');if(!body)return;
    Array.prototype.forEach.call(body.children,function(child){
      if(child.nodeType===1&&!child.querySelector('.rvt-alert-sev'))enhanceAlertItem(child);
    });
  }
  var dvBody=null,mutObs=null;
  function observeDrawer(){
    dvBody=document.getElementById('dvBody');if(!dvBody||mutObs)return;
    mutObs=new MutationObserver(function(){scanDrawer();});
    mutObs.observe(dvBody,{childList:true,subtree:false});
    scanDrawer();
  }
  window.rvtAlertSeverityBadge={scan:scanDrawer,enhance:enhanceAlertItem};
  U.bootSoon(observeDrawer);
})();
