(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var defaults={'rvt-theme':'system','rvt-density':'comfortable','rvt-live-mode':'simple','rvt-audio-volume':'0.7','rvt-toast-position':'BR'};
  function getDiff(){return Object.keys(defaults).filter(function(k){return localStorage.getItem(k)!==null&&localStorage.getItem(k)!==defaults[k];}).map(function(k){return {key:k,current:localStorage.getItem(k),default:defaults[k]};});}
  function resetOne(key){if(key in defaults)localStorage.setItem(key,defaults[key]);scan();return defaults[key];}
  function scan(){document.querySelectorAll('[data-setting-key]').forEach(function(el){var key=el.dataset.settingKey,d=getDiff().find(function(x){return x.key===key;}),dot=el.querySelector('.rvt-setting-modified-dot');if(d&&!dot){dot=document.createElement('button');dot.type='button';dot.className='rvt-setting-modified-dot';dot.title=R.t('gd.modified',{value:d.default});dot.setAttribute('aria-label',R.t('gd.resetSetting'));dot.onclick=function(){resetOne(key);};el.appendChild(dot);}else if(!d&&dot)dot.remove();});return getDiff();}
  window.rvtModifiedDot={scan:scan,getDiff:getDiff,resetOne:resetOne};R.boot(scan);
})();
