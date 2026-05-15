(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function getBarData(){var a=(window.S&&window.S.auditLog)||R.parseJson('rvt-audit-log',[],Array.isArray);var by={};a.forEach(function(e){var r=e.reason||e.message||'general',sev=e.severity||'info';by[r]=by[r]||{reason:r,info:0,warn:0,critical:0};if(!by[r][sev])by[r][sev]=0;by[r][sev]++;});return Object.keys(by).map(function(k){return by[k];});}
  function render(){var host=document.querySelector('#auditReasons,[data-audit-reasons],#view-audit')||document.body,root=R.ensure('rvt-audit04-histogram','div',host);root.className='rvt-gd-panel';root.innerHTML='';getBarData().forEach(function(b){var row=document.createElement('div');row.className='rvt-alert-severity-slice';row.title='info: '+b.info+', warn: '+b.warn+', critical: '+b.critical;row.textContent=b.reason+' '+(b.info+b.warn+b.critical);root.appendChild(row);});}
  window.rvtAuditStacked={render:render,getBarData:getBarData};R.boot(render);
})();
