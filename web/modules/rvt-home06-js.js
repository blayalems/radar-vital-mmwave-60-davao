(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function buildBrief(){var S=window.S||{},setup=S.setup||{};return [R.today(),'Op: '+R.currentOperator(),'Subject: '+(setup.subjectId||S.subjectId||''),'Port: '+(setup.port||S.port||''),'Readiness: '+(S.readiness||S.score||0)+'%','Profile: '+(setup.profileName||S.profileName||'')].join(' · ');}
  async function copy(){var text=buildBrief(),copied=false;if(navigator.clipboard&&navigator.clipboard.writeText){try{await navigator.clipboard.writeText(text);copied=true;}catch(_){copied=false;}}if(!copied){var d=R.ensure('rvt-home06-copy-modal','div');d.className='rvt-gd-dialog';d.innerHTML='<input readonly>';d.querySelector('input').value=text;d.querySelector('input').select();}R.toast(R.t('gd.sessionBriefCopied'));}
  window.rvtHome06={copy:copy,buildBrief:buildBrief};R.boot(function(){var b=R.ensure('rvt-home06-copy','button');b.className='rvt-gd-button';b.textContent=R.t('gd.copyBrief');b.onclick=copy;});
})();
