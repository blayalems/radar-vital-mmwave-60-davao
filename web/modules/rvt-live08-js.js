(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var zone={min:40,max:80},last=null;
  function isInZone(cm){cm=Number(cm);return cm>=zone.min&&cm<=zone.max;}
  function update(rangeCm){last=Number(rangeCm);var c=document.querySelector('#radarPreview,[data-radar-preview],canvas[aria-label*="Radar"]');if(c){c.setAttribute('aria-label',R.t(isInZone(last)?'gd.sweetInside':'gd.sweetOutside',{range:last}));c.dataset.sweetZone=isInZone(last)?'inside':'outside';}return isInZone(last);}
  window.rvtLive08={isInZone:isInZone,getZone:function(){return Object.assign({},zone);},update:update};window.rvtSweetZone=window.rvtLive08;R.boot(function(){var v=window.S&&window.S.live&&(window.S.live.range_cm||window.S.live.range);if(v!=null)update(v);});
})();
