(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function getFilter(){var v=R.parseJson('rvt-sessions-filter',{readiness:[],myOnly:false},function(x){return x&&Array.isArray(x.readiness)&&typeof x.myOnly==='boolean';});return {readiness:v.readiness,myOnly:v.myOnly};}
  function save(v){R.setJson('rvt-sessions-filter',v);render();return v;}
  function setChip(label,active){var v=getFilter(),i=v.readiness.indexOf(label);if(active&&i<0)v.readiness.push(label);if(!active&&i>=0)v.readiness.splice(i,1);return save(v);}
  function setMyOnly(b){var v=getFilter();v.myOnly=!!b;return save(v);}
  function clearAll(){return save({readiness:[],myOnly:false});}
  function render(){document.querySelectorAll('[data-readiness-filter],.readiness-filter').forEach(function(c){var label=c.dataset.readinessFilter||c.textContent.trim();c.setAttribute('aria-pressed',getFilter().readiness.indexOf(label)>=0?'true':'false');});var link=R.ensure('rvt-home05-clear','button');link.type='button';link.className='rvt-gd-button';link.textContent=R.t('gd.clearFilters');link.hidden=!getFilter().myOnly&&!getFilter().readiness.length;link.onclick=clearAll;}
  window.rvtHome05={getFilter:getFilter,setChip:setChip,setMyOnly:setMyOnly,clearAll:clearAll};R.boot(render);
})();
