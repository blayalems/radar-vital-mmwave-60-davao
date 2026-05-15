(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var prev=null;
  function isOverridden(){var v=R.getScalar('rvt-density-override-until','');return !!v&&new Date(v).getTime()>Date.now();}
  function onSessionStart(){if(!isOverridden()){prev=localStorage.getItem('rvt-density')||'comfortable';localStorage.setItem('rvt-density','compact');document.documentElement.dataset.density='compact';document.body.classList.add('rvt-density-session-compact');}return localStorage.getItem('rvt-density');}
  function onSessionEnd(){if(prev){localStorage.setItem('rvt-density',prev);document.documentElement.dataset.density=prev;}document.body.classList.remove('rvt-density-session-compact');return localStorage.getItem('rvt-density');}
  function setOverride(){var until=new Date(Date.now()+86400000).toISOString();R.setScalar('rvt-density-override-until',until);return until;}
  window.rvtAdaptiveDensity={onSessionStart:onSessionStart,onSessionEnd:onSessionEnd,setOverride:setOverride,isOverridden:isOverridden};
})();
