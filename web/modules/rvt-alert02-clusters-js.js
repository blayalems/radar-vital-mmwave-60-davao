(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var defaults={recent:true,session:true,previous:false};
  function readState(){try{return Object.assign({},defaults,JSON.parse(localStorage.getItem('rvt-alert-clusters')||'{}'));}catch(_){return Object.assign({},defaults);}}
  function writeState(s){localStorage.setItem('rvt-alert-clusters',JSON.stringify(s));}
  function currentSession(){var S=window.S||{};return S.currentSessionId||S.sessionId||S.session_id||(S.report&&S.report.sessionId)||'';}
  function tsValue(v){if(v==null)return Date.now();if(typeof v==='number')return v;var n=Date.parse(v);return Number.isFinite(n)?n:Number(v)||Date.now();}
  function normalizeAlert(a,node){
    a=a||{};var id=a.id||a.alertId||a.alert_id||(node&&node.dataset.alertId)||('alert-'+Math.random().toString(36).slice(2));
    return{id:id,sessionId:a.sessionId||a.session_id||(node&&node.dataset.sessionId)||currentSession(),timestamp:tsValue(a.timestamp||a.ts||(node&&node.dataset.alertTs)),severity:(a.severity||a.level||(node&&node.dataset.alertSeverity)||'info').toLowerCase(),message:a.message||a.title||(node&&node.textContent)||id,node:node||null,ack:!!(a.ack||a.acknowledged)};
  }
  function alerts(){
    var S=window.S||{};var out=[];
    if(Array.isArray(S.alerts))out=out.concat(S.alerts.map(function(a){return normalizeAlert(a);}));
    if(Array.isArray(S.liveAuditLog))out=out.concat(S.liveAuditLog.map(function(a){return normalizeAlert(a);}));
    document.querySelectorAll('[data-alert-id]').forEach(function(node){var id=node.dataset.alertId;if(!out.some(function(a){return a.id===id;}))out.push(normalizeAlert({},node));});
    return out;
  }
  function bucket(a,now){if(a.sessionId&&currentSession()&&a.sessionId!==currentSession())return'previous';return now-a.timestamp<=5*60*1000?'recent':'session';}
  function worstIcon(items){var sev=items.some(function(a){return a.severity==='critical'||a.severity==='bad';})?'critical':items.some(function(a){return a.severity==='warn'||a.severity==='warning';})?'warn':'info';return sev==='critical'?'error':sev==='warn'?'warning':'info';}
  function ensureMount(){var body=document.getElementById('dvBody');if(!body){body=document.createElement('div');body.id='dvBody';document.body.appendChild(body);}return body;}
  function renderItem(a){
    var node=a.node&&a.node.cloneNode(true)||document.createElement('article');
    node.setAttribute('data-alert-id',a.id);node.setAttribute('data-alert-ts',a.timestamp);node.setAttribute('data-alert-severity',a.severity);node.setAttribute('data-session-id',a.sessionId);node.classList.add('rvt-alert-item');
    if(!node.textContent.trim())node.textContent=a.message;
    return node;
  }
  function refresh(){
    var mount=ensureMount();var state=readState();var now=Date.now();var groups={recent:[],session:[],previous:[]};
    alerts().forEach(function(a){if(window.rvtAlertSnooze&&window.rvtAlertSnooze.isSnoozed(a.id))return;groups[bucket(a,now)].push(a);});
    mount.innerHTML='';
    [['recent','gamma2b.alert.recent'],['session','gamma2b.alert.session'],['previous','gamma2b.alert.previous']].forEach(function(pair){
      var id=pair[0],items=groups[id];if(!items.length)return;
      var wrap=document.createElement('section');wrap.className='rvt-alert-cluster';wrap.id='rvt-alert02-'+id;wrap.dataset.alertCluster=id;wrap.dataset.expanded=state[id]?'true':'false';
      var btn=document.createElement('button');btn.type='button';btn.className='rvt-alert-cluster-toggle';btn.setAttribute('aria-expanded',state[id]?'true':'false');btn.innerHTML='<span><span class="material-symbols-rounded" aria-hidden="true">'+worstIcon(items)+'</span> '+R.esc(R.t(pair[1]))+'</span><span class="rvt-alert-cluster-count">'+items.length+'</span>';
      btn.addEventListener('click',function(){window.rvtAlertClusters.toggle(id);});
      var body=document.createElement('div');body.className='rvt-alert-cluster-body';
      items.forEach(function(a){body.appendChild(renderItem(a));});
      wrap.appendChild(btn);wrap.appendChild(body);mount.appendChild(wrap);
    });
    if(window.rvtAlertSnooze)window.rvtAlertSnooze.decorate&&window.rvtAlertSnooze.decorate();
    if(window.rvtAlertWaveLink)window.rvtAlertWaveLink.decorate&&window.rvtAlertWaveLink.decorate();
    return groups;
  }
  window.rvtAlertClusters={toggle:function(id){var s=readState();s[id]=!s[id];writeState(s);refresh();},getState:readState,refresh:refresh};
  window.addEventListener('rvt-alert',refresh);
  R.boot(refresh);
})();
