(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var seen={};
  function alertList(){var S=window.S||{};var src=Array.isArray(S.lastAlerts)&&S.lastAlerts.length?S.lastAlerts:(Array.isArray(S.alerts)?S.alerts:[]);return src.map(function(a,i){return{id:a.id||a.alertId||a.key||('a'+i),ack:!!(a.ack||a.acknowledged),timestamp:a.timestamp||a.ts||Date.now()};});}
  function getUnacked(){return alertList().filter(function(a){return!a.ack&&!(window.rvtAlertSnooze&&window.rvtAlertSnooze.isSnoozed(a.id));}).length;}
  function getNew(){return alertList().filter(function(a){return!a.ack&&!seen[a.id]&&!(window.rvtAlertSnooze&&window.rvtAlertSnooze.isSnoozed(a.id));}).length;}
  function fmt(n){return n>=100?'99+':String(n);}
  function bell(){return document.querySelector('.tb-alerts,#alertBtn,#bell,[data-alert-bell]');}
  function update(){
    var b=bell();if(!b)return;var un=getUnacked(),fresh=getNew();b.dataset.alertBell='true';b.setAttribute('aria-label',R.t('gamma2b.alert.badge',{unacked:un,fresh:fresh}));b.title=R.t('gamma2b.alert.badgeTip',{unacked:un,fresh:fresh});
    var badge=document.getElementById('alertsBadge')||b.querySelector('.ic-badge')||document.createElement('span');badge.id='alertsBadge';badge.className='ic-badge icon-badge rvt-alert-badge';if(!badge.parentNode)b.appendChild(badge);
    if(un===0&&fresh===0){badge.style.display='none';badge.innerHTML='';return;}
    badge.style.display='inline-grid';
    badge.innerHTML='<span class="rvt-alert-badge-segment" data-alert-badge-segment="unacked">'+fmt(un)+'</span>'+(fresh>0?'<span class="rvt-alert-badge-segment" data-alert-badge-segment="new">'+fmt(fresh)+'</span>':'');
  }
  function markOpened(){alertList().forEach(function(a){seen[a.id]=true;});update();}
  var oldOpen=window.openDrawer;if(typeof oldOpen==='function'&&!oldOpen.__rvtAlertBadge){window.openDrawer=function(){var r=oldOpen.apply(this,arguments);markOpened();return r;};window.openDrawer.__rvtAlertBadge=true;}
  window.rvtAlertBadge={getUnacked:getUnacked,getNew:getNew,markOpened:markOpened,update:update};
  R.boot(update);
  if(typeof window.renderAlerts==='function'&&!window.renderAlerts.__rvtAlertBadgeWrapped){var _origRA=window.renderAlerts;window.renderAlerts=function(){var r=_origRA.apply(this,arguments);try{update();}catch(_){};return r;};window.renderAlerts.__rvtAlertBadgeWrapped=true;}
  setInterval(update,2000);
})();
