(function(){
  'use strict';
  var STRINGS={
    'gamma2b.rpt01.summary':'Session verdict summary',
    'gamma2b.rpt06.signoff':'Sign off report',
    'gamma2b.rpt06.view':'View sign-off',
    'gamma2b.rpt06.title':'Report sign-off',
    'gamma2b.rpt06.operator':'Operator name',
    'gamma2b.rpt06.initials':'Initials',
    'gamma2b.rpt06.comment':'Comment',
    'gamma2b.rpt06.confirm':'Confirm sign-off',
    'gamma2b.rpt06.close':'Close',
    'gamma2b.rpt06.signed':'Signed off: {initials} \\u00b7 {time}',
    'gamma2b.rpt06.audit':'Report signed off by {initials} at {timestamp}',
    'gamma2b.rpt06.unsigned':'Report not signed off',
    'gamma2b.alert.recent':'Last 5 min',
    'gamma2b.alert.session':'Earlier this session',
    'gamma2b.alert.previous':'Previous sessions',
    'gamma2b.alert.snooze':'Snooze alert',
    'gamma2b.alert.jump':'Jump to this moment in waveform',
    'gamma2b.alert.waveUnavailable':'Waveform data not available for this time',
    'gamma2b.alert.snoozeEnded':'Snooze ended',
    'gamma2b.alert.snoozed':'{count} snoozed',
    'gamma2b.alert.snooze5':'5 min',
    'gamma2b.alert.snooze15':'15 min',
    'gamma2b.alert.snooze60':'60 min',
    'gamma2b.alert.custom':'Custom',
    'gamma2b.alert.minutes':'Minutes',
    'gamma2b.alert.snoozeConfirm':'Snooze',
    'gamma2b.alert.badge':'Alerts: {unacked} unacknowledged, {fresh} new',
    'gamma2b.alert.badgeTip':'{unacked} unacknowledged \\u00b7 {fresh} new since last viewed',
    'gamma2b.live.lineageUnavailable':'Computation details unavailable',
    'gamma2b.live.tagged':'Tagged: {type} @ {time}',
    'gamma2b.live.snapshot':'Snapshot Now',
    'gamma2b.live.snapshotLabel':'Snapshot label',
    'gamma2b.live.saveLabel':'Save label'
  };
  function installStrings(){
    window.RVT_STRINGS=window.RVT_STRINGS||{en:{}};
    window.RVT_STRINGS.en=window.RVT_STRINGS.en||{};
    Object.keys(STRINGS).forEach(function(k){if(!(k in window.RVT_STRINGS.en))window.RVT_STRINGS.en[k]=STRINGS[k];});
  }
  function t(key,vars){
    installStrings();
    var s=(window.RVT_STRINGS&&window.RVT_STRINGS.en&&window.RVT_STRINGS.en[key])||key;
    if(s===key&&typeof window.rvtT==='function')s=window.rvtT(key,vars||{});
    vars=vars||{};
    Object.keys(vars).forEach(function(k){s=String(s).replace(new RegExp('\\{'+k+'\\}','g'),vars[k]);});
    return s;
  }
  function esc(v){return String(v==null?'':v).replace(/[&<>"']/g,function(ch){return({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'})[ch];});}
  function toast(msg){
    if(window.rvtGammaBatch2a&&typeof window.rvtGammaBatch2a.toast==='function'){window.rvtGammaBatch2a.toast(msg);return;}
    var old=document.querySelector('.rvt-quicktag-toast');if(old)old.remove();
    var node=document.createElement('div');node.className='rvt-quicktag-toast';node.setAttribute('role','status');node.setAttribute('aria-live','polite');node.textContent=msg;document.body.appendChild(node);
    setTimeout(function(){node.remove();},3000);
  }
  function boot(fn){if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',fn,{once:true});else setTimeout(fn,0);}
  function registerManifest(){
    window.rvtStorage=window.rvtStorage||{};
    window.rvtStorage.manifest=window.rvtStorage.manifest||{};
    var m=window.rvtStorage.manifest;
    if(!m['rvt-alert-clusters'])m['rvt-alert-clusters']={kind:'json',defaultValue:{recent:true,session:true,previous:false},validate:function(v){return v!==null&&typeof v==='object'&&'recent'in v&&'session'in v&&'previous'in v;}};
    if(!m['rvt-alert-snooze'])m['rvt-alert-snooze']={kind:'json',defaultValue:{},validate:function(v){return v!==null&&typeof v==='object';}};
    if(!m['rvt-report-signoff-*'])m['rvt-report-signoff-*']={kind:'json',defaultValue:null,validate:function(v){return v===null||!!(typeof v==='object'&&v!==null&&v.sessionId&&v.operator);}};
  }
  window.rvtGammaBatch2bRuntime={t:t,esc:esc,toast:toast,boot:boot,registerManifest:registerManifest};
  installStrings();
  registerManifest();

  var stuck=false,hero=null,observer=null,sentinel=null,fullHeight=0;
  function mobile(){return window.matchMedia&&window.matchMedia('(max-width: 767px)').matches;}
  function findHero(){return document.querySelector('[data-rvt-rpt01-hero],#rvt-rpt01-hero,.rvt-verdict-hero,.verdict-hero');}
  function applyState(next){
    hero=findHero();
    if(!hero)return false;
    hero.classList.add('rvt-rpt01-sticky');
    if(mobile())next=false;
    stuck=!!next;
    if(!fullHeight)fullHeight=hero.getBoundingClientRect().height||Number(hero.dataset.rvtRpt01FullHeight)||180;
    hero.dataset.rvtRpt01FullHeight=String(fullHeight);
    hero.classList.toggle('rvt-rpt01-collapsed',stuck);
    hero.classList.toggle('is-stuck',stuck);
    hero.classList.toggle('collapsed',stuck);
    if(stuck){
      hero.style.height=Math.max(44,Math.round(fullHeight/2))+'px';
      hero.style.minHeight=Math.max(44,Math.round(fullHeight/2))+'px';
    }else{
      hero.style.height='';
      hero.style.minHeight='';
    }
    return stuck;
  }
  function initObserver(){
    hero=findHero();
    if(!hero||observer||!('IntersectionObserver'in window))return;
    hero.classList.add('rvt-rpt01-sticky');
    sentinel=document.createElement('div');
    sentinel.id='rvt-rpt01-sentinel';
    sentinel.style.cssText='height:1px;width:1px;';
    hero.parentNode.insertBefore(sentinel,hero);
    observer=new IntersectionObserver(function(entries){entries.forEach(function(entry){applyState(!entry.isIntersecting&&window.scrollY>100);});},{threshold:0});
    observer.observe(sentinel);
  }
  window.rvtVerdictHero={
    isStuck:function(){var y=window.scrollY||document.documentElement.scrollTop||document.body.scrollTop||0;var h=findHero();if(!stuck&&!mobile()&&(y>100||(h&&h.getBoundingClientRect().top<=100)))applyState(true);return !!stuck;},
    forceCollapse:function(){return applyState(true);},
    forceExpand:function(){return applyState(false);},
    destroy:function(){if(observer){observer.disconnect();observer=null;}if(sentinel){sentinel.remove();sentinel=null;}applyState(false);}
  };
  boot(initObserver);
})();
