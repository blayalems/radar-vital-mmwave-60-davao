(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  function ts(v){return typeof v==='number'?v:(Date.parse(v)||Number(v)||0);}
  function range(){var S=window.S||{},w=S.waveform||S.waves||{};if(Number.isFinite(w.startTs)&&Number.isFinite(w.endTs))return{start:w.startTs,end:w.endTs};if(Array.isArray(w.timestamps)&&w.timestamps.length)return{start:Math.min.apply(null,w.timestamps.map(ts)),end:Math.max.apply(null,w.timestamps.map(ts))};return null;}
  function canJump(timestamp){var r=range();var n=ts(timestamp);return!!(r&&n>=r.start&&n<=r.end);}
  function severityFor(id){var S=window.S||{},a=(S.alerts||[]).find(function(x){return(x.id||x.alertId)===id;});return(a&&a.severity)||document.querySelector('[data-alert-id="'+CSS.escape(id||'')+'"]')?.dataset.alertSeverity||'info';}
  function color(sev){sev=String(sev||'info').toLowerCase();return sev==='critical'||sev==='bad'?'#dc2626':sev==='warn'||sev==='warning'?'#b26a00':'#2563eb';}
  function placeMarker(timestamp,severity){
    var host=document.getElementById('tab-waves')||document.body;host.style.position=host.style.position||'relative';
    var m=document.createElement('div');m.className='rvt-alert-wave-marker';m.dataset.alertWaveMarker='true';m.style.left='50%';m.style.backgroundColor=color(severity);host.appendChild(m);
    if(!window.matchMedia||!window.matchMedia('(prefers-reduced-motion: reduce)').matches){m.classList.add('rvt-pulse');setTimeout(function(){m.classList.remove('rvt-pulse');},2000);}
    return m;
  }
  function clearMarkers(){document.querySelectorAll('[data-alert-wave-marker]').forEach(function(n){n.remove();});}
  function jumpTo(alertId,timestamp){if(!canJump(timestamp))return false;if(typeof window.switchView==='function')window.switchView('live');else document.body.dataset.view='live';if(typeof window.switchTab==='function')window.switchTab('tab-waves');window.rvtAlertWaveLink.placeMarker(timestamp,severityFor(alertId));return true;}
  function decorate(){
    document.querySelectorAll('[data-alert-id]').forEach(function(n){if(n.querySelector('.rvt-alert-wave-btn'))return;var id=n.dataset.alertId,stamp=n.dataset.alertTs||Date.now();var row=n.querySelector('.rvt-alert-action-row')||document.createElement('div');row.className='rvt-alert-action-row';if(!row.parentNode)n.appendChild(row);var b=document.createElement('button');b.type='button';b.className='rvt-alert-action rvt-alert-wave-btn';b.setAttribute('aria-label',R.t('gamma2b.alert.jump'));b.title=canJump(stamp)?R.t('gamma2b.alert.jump'):R.t('gamma2b.alert.waveUnavailable');b.disabled=!canJump(stamp);b.innerHTML='<span class="material-symbols-rounded" aria-hidden="true">show_chart</span>';b.addEventListener('click',function(){jumpTo(id,stamp);});row.appendChild(b);});
  }
  window.rvtAlertWaveLink={jumpTo:jumpTo,canJump:canJump,placeMarker:placeMarker,clearMarkers:clearMarkers,decorate:decorate};
  R.boot(decorate);
})();
