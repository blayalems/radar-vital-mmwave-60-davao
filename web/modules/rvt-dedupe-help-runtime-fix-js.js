(function(){
  'use strict';
  var ISSUE_MAP={
    'ble not found':{topic:'troubleshooting',query:'ble'},
    'placement':{topic:'hardware_setup',query:'placement'},
    'version mismatch':{topic:'firmware_truthfulness',query:'firmware'},
    'stale data':{topic:'troubleshooting',query:'stale'},
    'low reference quality':{topic:'hardware_setup',query:'reference'},
    'report failed':{topic:'report_readiness',query:'report'}
  };
  function issueConfig(btn){
    var key=(btn&&btn.textContent||'').trim().toLowerCase();
    return ISSUE_MAP[key]||null;
  }
  function applyHelpSearch(query){
    try{ if(window.S&&window.S.ctl) window.S.ctl.helpQuery=query; }catch(_){}
    try{ localStorage.setItem('rvt-help-query',query); }catch(_){}
    var input=document.getElementById('rvtHelpSearchInput')||document.querySelector('#view-help input[type="search"], #view-help .help-search input');
    if(input){
      input.value=query;
      try{ input.dispatchEvent(new Event('input',{bubbles:true})); }catch(_){}
    }
  }
  function openTopic(topic){
    if(typeof window.rvtOpenHelpTopic==='function'){ window.rvtOpenHelpTopic(topic); return; }
    if(typeof window.setHelpTopic==='function'){ window.setHelpTopic(topic); return; }
    if(typeof window.switchView==='function') window.switchView('help');
  }
  function activate(btn){
    var cfg=issueConfig(btn);
    if(!cfg) return false;
    btn.removeAttribute('onclick');
    btn.dataset.rvtHelpIssueTopic=cfg.topic;
    btn.dataset.rvtHelpIssueQuery=cfg.query;
    applyHelpSearch(cfg.query);
    openTopic(cfg.topic);
    return true;
  }
  function repair(){
    Array.prototype.forEach.call(document.querySelectorAll('#view-help .help-issue,.help-issue'),function(btn){
      var cfg=issueConfig(btn);
      if(!cfg) return;
      btn.removeAttribute('onclick');
      btn.dataset.rvtHelpIssueTopic=cfg.topic;
      btn.dataset.rvtHelpIssueQuery=cfg.query;
    });
  }
  document.addEventListener('click',function(ev){
    var target=ev.target&&ev.target.closest&&ev.target.closest('.help-issue');
    if(!target) return;
    if(activate(target)){
      ev.preventDefault();
      ev.stopImmediatePropagation();
    }
  },true);
  var booted=false;
  function boot(){
    if(booted) return;
    booted=true;
    repair();
    if(window.MutationObserver){
      new MutationObserver(repair).observe(document.body||document.documentElement,{childList:true,subtree:true});
    }
    var tries=0;
    var timer=setInterval(function(){
      repair();
      tries++;
      if(tries>20) clearInterval(timer);
    },500);
  }
  if(document.body) boot();
  else if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',boot,{once:true});
  else boot();
})();
