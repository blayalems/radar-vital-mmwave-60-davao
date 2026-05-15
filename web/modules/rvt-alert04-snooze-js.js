(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  function read(){try{return JSON.parse(localStorage.getItem('rvt-alert-snooze')||'{}')||{};}catch(_){return{};}}
  function write(v){localStorage.setItem('rvt-alert-snooze',JSON.stringify(v));}
  function item(id){return document.querySelector('[data-alert-id="'+CSS.escape(id)+'"]');}
  function hide(id){var n=item(id);if(n){n.hidden=true;n.setAttribute('aria-hidden','true');}}
  function label(){
    var active=getSnoozed();var old=document.getElementById('rvt-alert04-snoozed-label');if(old)old.remove();if(!active.length)return;
    var mount=document.getElementById('dvBody')||document.body;var l=document.createElement('div');l.id='rvt-alert04-snoozed-label';l.className='rvt-alert-snoozed-label';l.textContent=R.t('gamma2b.alert.snoozed',{count:active.length});mount.appendChild(l);
  }
  function snooze(id,minutes){var all=read();all[id]=Date.now()+Math.max(1,Math.min(240,Number(minutes)||5))*60*1000;write(all);hide(id);label();if(window.rvtAlertClusters)window.rvtAlertClusters.refresh();return all[id];}
  function unsnooze(id,ended){var all=read();delete all[id];write(all);var n=item(id);if(n){n.hidden=false;n.setAttribute('aria-hidden','false');if(ended){var b=document.createElement('span');b.className='rvt-snooze-ended-badge';b.textContent=R.t('gamma2b.alert.snoozeEnded');n.appendChild(b);setTimeout(function(){b.remove();},30000);}}label();}
  function isSnoozed(id){var exp=read()[id];return Number(exp)>Date.now();}
  function getSnoozed(){var all=read();return Object.keys(all).filter(function(id){return Number(all[id])>Date.now();}).map(function(id){return{alertId:id,expiresAt:all[id]};});}
  function clearExpired(){var all=read();Object.keys(all).forEach(function(id){if(Number(all[id])<=Date.now()){delete all[id];var n=item(id);if(n){n.hidden=false;n.setAttribute('aria-hidden','false');var b=document.createElement('span');b.className='rvt-snooze-ended-badge';b.textContent=R.t('gamma2b.alert.snoozeEnded');n.appendChild(b);setTimeout(function(){b.remove();},30000);}}});write(all);label();return all;}
  function decorate(){
    clearExpired();
    document.querySelectorAll('[data-alert-id]').forEach(function(n){
      var id=n.dataset.alertId;if(isSnoozed(id))hide(id);
      if(n.querySelector('.rvt-alert-snooze-btn'))return;
      var row=n.querySelector('.rvt-alert-action-row');if(!row){row=document.createElement('div');row.className='rvt-alert-action-row';n.appendChild(row);}
      var btn=document.createElement('button');btn.type='button';btn.className='rvt-alert-action rvt-alert-snooze-btn';btn.setAttribute('aria-label',R.t('gamma2b.alert.snooze'));btn.innerHTML='<span class="material-symbols-rounded" aria-hidden="true">schedule</span>';
      var menu=document.createElement('span');menu.className='rvt-alert-snooze-menu';menu.hidden=true;
      [5,15,60].forEach(function(m){var b=document.createElement('button');b.type='button';b.textContent=R.t('gamma2b.alert.snooze'+m);b.addEventListener('click',function(){snooze(id,m);});menu.appendChild(b);});
      var custom=document.createElement('button');custom.type='button';custom.textContent=R.t('gamma2b.alert.custom');menu.appendChild(custom);
      var customBox=document.createElement('span');customBox.className='rvt-alert-snooze-custom';customBox.hidden=true;customBox.innerHTML='<input type="number" min="1" max="240" placeholder="'+R.esc(R.t('gamma2b.alert.minutes'))+'"><button type="button" class="rvt-alert-snooze-confirm">'+R.esc(R.t('gamma2b.alert.snoozeConfirm'))+'</button>';menu.appendChild(customBox);
      custom.addEventListener('click',function(){customBox.hidden=false;customBox.querySelector('input').focus();});
      customBox.querySelector('button').addEventListener('click',function(){snooze(id,customBox.querySelector('input').value);});
      btn.addEventListener('click',function(){menu.hidden=!menu.hidden;});
      row.appendChild(btn);row.appendChild(menu);
    });
    label();
  }
  window.rvtAlertSnooze={snooze:snooze,unsnooze:unsnooze,isSnoozed:isSnoozed,getSnoozed:getSnoozed,clearExpired:clearExpired,decorate:decorate};
  R.boot(decorate);
})();
