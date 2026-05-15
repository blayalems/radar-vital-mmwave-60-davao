(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var currentPqi=null,paused=false;
  function reduced(){return window.matchMedia&&window.matchMedia('(prefers-reduced-motion: reduce)').matches;}
  function kpis(){return document.querySelectorAll('.kpi,[data-kpi-id]');}
  function setAmplitude(r){r=Math.max(0,Math.min(1,Number(r)||0));document.documentElement.style.setProperty('--rvt-pulse-amplitude',String(r));document.body.style.setProperty('--rvt-pulse-amplitude',String(r));}
  function pause(){paused=true;document.body.classList.add('rvt-pulse-paused');kpis().forEach(function(k){k.classList.add('rvt-pulse-paused');});}
  function resume(){paused=false;document.body.classList.remove('rvt-pulse-paused');kpis().forEach(function(k){k.classList.remove('rvt-pulse-paused');});}
  function readPqi(){var S=window.S||{};var p=S.live&&S.live.pqi;if(p==null)p=S.lastPayload&&(S.lastPayload.pqi||S.lastPayload.pqi_heart||S.lastPayload.heart_pqi);var n=Number(p);return Number.isFinite(n)?n:currentPqi;}
  function apply(){currentPqi=readPqi();if(reduced())return currentPqi;if(currentPqi==null)return currentPqi;if(currentPqi<.2){setAmplitude(0);pause();}else{resume();setAmplitude(currentPqi<.4?.3:1);}return currentPqi;}
  window.addEventListener('rvt-live-update',function(e){if(e&&e.detail){window.S=window.S||{};window.S.live=Object.assign({},window.S.live||{},e.detail);}apply();});
  window.rvtPulseConf={setAmplitude:setAmplitude,pause:pause,resume:resume,getCurrentPqi:function(){return apply();}};
  R.boot(apply);
})();
