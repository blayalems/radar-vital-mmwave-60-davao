(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var running=false;
  function checks(){return (window.S&&window.S.quickChecks)||window.quickChecks||[];}
  function getFailedChecks(){return checks().filter(function(c){return /fail|warn/.test(String(c.status||c.state).toLowerCase());}).map(function(c){return c.id||c.name;});}
  function rerunFailed(){var ids=getFailedChecks();running=true;render();return Promise.resolve().then(function(){var pass=0,fail=0;checks().forEach(function(c){if(ids.indexOf(c.id||c.name)>=0){if(typeof c.run==='function')c.run();c.timestamp=c.timestamp||R.nowIso();if(String(c.status||c.state).toLowerCase()==='fail')fail++;else pass++;}});R.toast(R.t('gd.rerunDone',{done:ids.length,total:ids.length,pass:pass,fail:fail}));}).finally(function(){running=false;render();});}
  function render(){var has=getFailedChecks().length>0,home=document.querySelector('#view-home,[data-view="home"],main')||document.body,btn=document.getElementById('rvt-home03-rerun');if(!has&&btn){btn.remove();return;}if(has){btn=btn||R.ensure('rvt-home03-rerun','button',home);btn.className='rvt-gd-button';btn.disabled=running;btn.textContent=running?'...':R.t('gd.rerunFailed');btn.onclick=rerunFailed;}}
  window.rvtHome03={rerunFailed:rerunFailed,getFailedChecks:getFailedChecks,isRunning:function(){return running;}};R.boot(render);
})();
