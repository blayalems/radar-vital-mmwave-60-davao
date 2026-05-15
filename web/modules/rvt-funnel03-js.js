(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var tickTimer=null;
  function enabled(){return R.getScalar('rvt-funnel-ghost','false')==='true';}
  function getGhostData(){var b=(window.S&&(window.S.funnelBuffer||window.S.funnelsBuffer))||[];var target=Date.now()-60000;if(Array.isArray(b)&&b.length){return b.reduce(function(best,x){return !best||Math.abs((x.timestamp||0)-target)<Math.abs((best.timestamp||0)-target)?x:best;},null);}return window.S&&window.S.funnelsPrevious||{};}
  function render(){var host=document.querySelector('#funnelDiagram,[data-funnel-diagram],#view-funnels')||document.body;var toggle=R.ensure('rvt-funnel03-toggle','button',host);toggle.className='rvt-gd-button';toggle.setAttribute('aria-pressed',enabled()?'true':'false');toggle.textContent=R.t('gd.previousMinute');toggle.onclick=function(){enabled()?disable():enable();};var layer=document.getElementById('rvt-funnel03-ghost');if(enabled()){layer=layer||R.ensure('rvt-funnel03-ghost','div',host);layer.className='rvt-funnel-ghost-layer';layer.textContent=JSON.stringify(getGhostData()||{});}else if(layer)layer.remove();}
  function enable(){R.setScalar('rvt-funnel-ghost','true');if(!tickTimer)tickTimer=setInterval(render,1000);render();}
  function disable(){R.setScalar('rvt-funnel-ghost','false');if(tickTimer)clearInterval(tickTimer);tickTimer=null;render();}
  window.addEventListener('beforeunload',function(){if(tickTimer)clearInterval(tickTimer);});
  window.rvtFunnel03={enable:enable,disable:disable,isEnabled:enabled,getGhostData:getGhostData};R.boot(render);
})();
