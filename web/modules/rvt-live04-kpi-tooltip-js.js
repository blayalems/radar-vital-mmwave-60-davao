(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  var activeTooltip=null;
  function getKpiData(kpiId){
    var S=window.S||{};
    var payload=S.lastPayload||S.latest||{};
    var value=payload[kpiId]||payload[kpiId+'_corrected']||payload[kpiId+'_raw'];
    var source=payload.publish_source||payload.publishSource||'';
    var buffer=S.liveBuffer?S.liveBuffer.length:localStorage.getItem('rvt-live-buffer-seconds')||'60';
    if(S.funnels&&S.funnels[kpiId]){
      var f=S.funnels[kpiId];
      return{value:f.value||value,source:f.source||source,buffer:f.bufferSize||buffer,extra:f};
    }
    return{value:value!=null?value:'--',source:source||t('gamma.live.tooltip.unavailable'),buffer:buffer};
  }
  function showTooltip(card,kpiId){
    hideTooltip();
    var data=getKpiData(kpiId);
    var tip=document.createElement('div');tip.className='rvt-kpi-tooltip';tip.id='rvtKpiTooltip';
    tip.setAttribute('role','tooltip');
    tip.innerHTML='<strong>'+U.escapeHtml(t('gamma.live.tooltip.title'))+'</strong><dl>'+
      '<dt>'+U.escapeHtml(t('gamma.live.tooltip.value'))+'</dt><dd>'+U.escapeHtml(String(data.value))+'</dd>'+
      '<dt>'+U.escapeHtml(t('gamma.live.tooltip.source'))+'</dt><dd>'+U.escapeHtml(String(data.source))+'</dd>'+
      '<dt>'+U.escapeHtml(t('gamma.live.tooltip.buffer'))+'</dt><dd>'+U.escapeHtml(String(data.buffer))+'</dd></dl>';
    card.style.position='relative';card.appendChild(tip);
    requestAnimationFrame(function(){tip.setAttribute('data-visible','true');});
    activeTooltip={tip:tip,card:card};
  }
  function hideTooltip(){
    if(!activeTooltip)return;
    activeTooltip.tip.remove();activeTooltip=null;
  }
  function enhanceKpis(){
    var cards=document.querySelectorAll('.kpi[data-kpi-id]');
    Array.prototype.forEach.call(cards,function(card){
      if(card.getAttribute('data-tooltip-enhanced'))return;
      card.setAttribute('data-tooltip-enhanced','true');
      var kpiId=card.getAttribute('data-kpi-id');
      card.addEventListener('mouseenter',function(){showTooltip(card,kpiId);});
      card.addEventListener('mouseleave',hideTooltip);
      card.addEventListener('focus',function(){showTooltip(card,kpiId);});
      card.addEventListener('blur',hideTooltip);
    });
  }
  window.rvtKpiTooltip={show:showTooltip,hide:hideTooltip,refresh:enhanceKpis};
  U.bootSoon(enhanceKpis);
})();
