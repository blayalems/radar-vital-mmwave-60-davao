(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function pins(){return R.parseJson('rvt-alert-pins',[],Array.isArray).slice(0,3);}
  function save(a){R.setJson('rvt-alert-pins',a.slice(0,3));render();return pins();}
  function pin(id){var a=pins().filter(function(x){return x!==id;});if(a.length>=3){if(window.confirm&&!window.confirm(R.t('gd.replacePin')))return a;a.shift();}a.push(id);return save(a);}
  function unpin(id){return save(pins().filter(function(x){return x!==id;}));}
  function render(){document.querySelectorAll('[data-alert-id],.alert-item').forEach(function(n){if(n.querySelector('.rvt-alert05-pin'))return;var id=n.dataset.alertId||n.id,b=document.createElement('button');b.className='rvt-gd-button rvt-alert05-pin';b.setAttribute('aria-label',R.t('gd.pinAlert'));b.textContent='pin';b.onclick=function(){pins().indexOf(id)>=0?unpin(id):pin(id);};n.appendChild(b);});}
  window.rvtPinAlert={pin:pin,unpin:unpin,isPinned:function(id){return pins().indexOf(id)>=0;},getPinned:pins};R.boot(render);
})();
