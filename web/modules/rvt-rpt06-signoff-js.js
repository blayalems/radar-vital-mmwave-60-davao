(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var modal=null;
  function sessionId(fallback){var S=window.S||{};return fallback||S.currentSessionId||S.sessionId||(S.report&&S.report.sessionId)||(S.currentReport&&S.currentReport.sessionId)||'session';}
  function key(id){return'rvt-report-signoff-'+sessionId(id);}
  function read(id){try{return JSON.parse(localStorage.getItem(key(id))||'null');}catch(_){return null;}}
  function write(obj){localStorage.setItem(key(obj.sessionId),JSON.stringify(obj));}
  function operatorName(){var S=window.S||{};return(S.settings&&(S.settings.operatorName||S.settings.operator))||S.currentOperator||localStorage.getItem('rvt-operator-name')||localStorage.getItem('rvt-current-operator')||'';}
  function initials(name){return String(name||'').trim().split(/\s+/).filter(Boolean).map(function(p){return p[0]||'';}).join('').slice(0,4).toUpperCase();}
  function reportLoaded(){var S=window.S||{};var r=S.report||S.currentReport||{};return r.completed!==false;}
  function audit(obj){var S=window.S=window.S||{};var r=S.report||S.currentReport||(S.report={auditLog:[]});r.signoff=obj;r.auditLog=r.auditLog||[];r.auditLog.push({timestamp:obj.timestamp,message:R.t('gamma2b.rpt06.audit',{initials:obj.initials,timestamp:obj.timestamp})});}
  function ensureToolbar(){
    var report=document.getElementById('view-report')||document.querySelector('[data-view="report"]');if(!report)return;
    if(document.getElementById('rvt-rpt06-toolbar')){updateToolbar();return;}
    var bar=document.createElement('div');bar.id='rvt-rpt06-toolbar';bar.className='rvt-rpt06-toolbar';
    bar.innerHTML='<button type="button" class="rvt-rpt06-btn" id="rvt-rpt06-signoff-btn"><span class="material-symbols-rounded" aria-hidden="true">verified</span><span data-rpt06-label></span></button><span class="rvt-rpt06-status" id="rvt-rpt06-status" aria-live="polite"></span>';
    var hero=report.querySelector('.verdict-hero,.rvt-verdict-hero')||report.firstElementChild;
    report.insertBefore(bar,hero?hero.nextSibling:report.firstChild);
    document.getElementById('rvt-rpt06-signoff-btn').addEventListener('click',function(){open(sessionId());});
    updateToolbar();
  }
  function updateToolbar(){
    var id=sessionId();var data=read(id);var btn=document.getElementById('rvt-rpt06-signoff-btn');var status=document.getElementById('rvt-rpt06-status');if(!btn)return;
    btn.disabled=!reportLoaded();
    btn.querySelector('[data-rpt06-label]').textContent=data?R.t('gamma2b.rpt06.view'):R.t('gamma2b.rpt06.signoff');
    if(status)status.innerHTML=data?'<span class="material-symbols-rounded" aria-hidden="true">check_circle</span>'+R.esc(R.t('gamma2b.rpt06.signed',{initials:data.initials||'',time:new Date(data.timestamp).toLocaleTimeString()})):'';
  }
  function close(){if(modal){modal.remove();modal=null;}}
  function open(id){
    id=sessionId(id);var existing=read(id);var readonly=!!existing;var op=existing?existing.operator:operatorName();var init=existing?existing.initials:initials(op);
    close();
    modal=document.createElement('div');modal.className='rvt-rpt06-backdrop';modal.id='rvt-rpt06-modal-backdrop';
    modal.innerHTML='<div class="rvt-rpt06-modal" role="dialog" aria-modal="true" aria-labelledby="rvt-rpt06-title"><h2 id="rvt-rpt06-title">'+R.esc(R.t('gamma2b.rpt06.title'))+'</h2><label class="rvt-rpt06-field">'+R.esc(R.t('gamma2b.rpt06.operator'))+'<input id="rvt-rpt06-operator" name="operator" data-rpt06-operator maxlength="80"></label><label class="rvt-rpt06-field">'+R.esc(R.t('gamma2b.rpt06.initials'))+'<input id="rvt-rpt06-initials" name="initials" data-rpt06-initials maxlength="4"></label><label class="rvt-rpt06-field">'+R.esc(R.t('gamma2b.rpt06.comment'))+'<textarea id="rvt-rpt06-comment" name="comment" data-rpt06-comment maxlength="500"></textarea></label><div class="rvt-rpt06-actions"><button type="button" data-rpt06-close>'+R.esc(R.t('gamma2b.rpt06.close'))+'</button><button type="button" data-rpt06-confirm>'+R.esc(R.t('gamma2b.rpt06.confirm'))+'</button></div></div>';
    document.body.appendChild(modal);
    var opEl=modal.querySelector('#rvt-rpt06-operator'),initEl=modal.querySelector('#rvt-rpt06-initials'),commentEl=modal.querySelector('#rvt-rpt06-comment'),confirm=modal.querySelector('[data-rpt06-confirm]');
    opEl.value=op||'';initEl.value=init||'';commentEl.value=existing&&existing.comment?existing.comment:'';
    opEl.addEventListener('input',function(){if(!initEl.dataset.edited)initEl.value=initials(opEl.value);});
    initEl.addEventListener('input',function(){initEl.dataset.edited='1';initEl.value=initEl.value.slice(0,4).toUpperCase();});
    if(readonly){[opEl,initEl,commentEl].forEach(function(el){el.readOnly=true;});confirm.style.display='none';}
    modal.querySelector('[data-rpt06-close]').addEventListener('click',close);
    modal.addEventListener('click',function(e){if(e.target===modal)close();});
    modal.addEventListener('keydown',function(e){if(e.key==='Escape')close();});
    confirm.addEventListener('click',function(){
      var obj={operator:opEl.value.trim(),initials:initEl.value.trim().slice(0,4).toUpperCase(),comment:commentEl.value.slice(0,500),timestamp:new Date().toISOString(),sessionId:id};
      if(!obj.operator)return;
      write(obj);audit(obj);updateToolbar();close();
    });
    opEl.focus();
  }
  window.rvtReportSignoff={open:open,getSignoff:read,isSigned:function(id){return!!read(id);},clear:function(id){localStorage.removeItem(key(id));updateToolbar();}};
  window.addEventListener('beforeprint',function(){var id=sessionId();var old=document.getElementById('rvt-rpt06-pdf-warning');if(old)old.remove();if(!read(id)){var report=document.getElementById('view-report')||document.body;var warn=document.createElement('div');warn.id='rvt-rpt06-pdf-warning';warn.className='rvt-rpt06-pdf-warning';warn.textContent=R.t('gamma2b.rpt06.unsigned');report.prepend(warn);}});
  R.boot(ensureToolbar);
})();
