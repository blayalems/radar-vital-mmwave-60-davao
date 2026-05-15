(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var overrides={},timer=null,pinned=null;
  function kpiEl(id){return document.querySelector('[data-kpi-id="'+CSS.escape(id)+'"],#kpi'+id);}
  function stagesFrom(v){if(Array.isArray(v))return v;if(v&&Array.isArray(v.stages))return v.stages;if(v&&Array.isArray(v.lineage))return v.lineage;if(v&&typeof v==='object')return Object.keys(v);return null;}
  function getLineage(id){var S=window.S||{};var stages=overrides[id]||stagesFrom(S.funnels&&S.funnels[id])||stagesFrom(S.live&&S.live.funnels&&S.live.funnels[id]);return stages&&stages.length?stages:[R.t('gamma2b.live.lineageUnavailable')];}
  function hide(){if(timer){clearTimeout(timer);timer=null;}document.querySelectorAll('.rvt-kpi-lineage-tooltip,.rvt-kpi-tooltip').forEach(function(n){n.remove();});}
  function goStage(stage){if(typeof window.switchTab==='function')window.switchTab('tab-funnels');var target=document.querySelector('[data-funnel-stage="'+CSS.escape(stage)+'"],#funnel-'+CSS.escape(stage));if(target)target.scrollIntoView({block:'center'});}
  function render(id,pin){hide();var el=kpiEl(id);if(!el)return;var stages=getLineage(id);var tip=document.createElement('div');tip.className='rvt-kpi-lineage-tooltip';tip.setAttribute('role','tooltip');tip.dataset.kpiLineageTooltip='true';tip.innerHTML=stages.map(function(s){var has=!!document.querySelector('[data-funnel-stage="'+CSS.escape(s)+'"],#funnel-'+CSS.escape(s));return has?'<button type="button" data-stage="'+R.esc(s)+'">'+R.esc(s)+'</button>':R.esc(s);}).join(' &rarr; ');document.body.appendChild(tip);var r=el.getBoundingClientRect();tip.style.left=Math.max(8,r.left)+'px';tip.style.top=Math.max(8,r.top-tip.offsetHeight-10)+'px';tip.querySelectorAll('[data-stage]').forEach(function(b){b.addEventListener('click',function(){goStage(b.dataset.stage);});});if(pin)pinned=tip;}
  function show(id){if(timer)clearTimeout(timer);timer=setTimeout(function(){render(id,false);},400);}
  document.addEventListener('mouseenter',function(e){var el=e.target.closest&&e.target.closest('[data-kpi-id]');if(el)show(el.dataset.kpiId);},true);
  document.addEventListener('focusin',function(e){var el=e.target.closest&&e.target.closest('[data-kpi-id]');if(el)show(el.dataset.kpiId);});
  document.addEventListener('mouseleave',function(e){if(e.target.closest&&e.target.closest('[data-kpi-id]'))hide();},true);
  document.addEventListener('keydown',function(e){if(e.key==='Escape')hide();});
  document.addEventListener('touchstart',function(e){var el=e.target.closest&&e.target.closest('[data-kpi-id]');if(!el)return;timer=setTimeout(function(){render(el.dataset.kpiId,true);},500);},{passive:true});
  document.addEventListener('touchend',function(){if(timer){clearTimeout(timer);timer=null;}},{passive:true});
  document.addEventListener('click',function(e){if(pinned&&!pinned.contains(e.target)&&!e.target.closest('[data-kpi-id]')){hide();pinned=null;}});
  window.rvtKpiLineage={show:show,hide:hide,getLineage:getLineage,setLineage:function(id,stages){overrides[id]=Array.isArray(stages)?stages.slice():[];}};
})();
