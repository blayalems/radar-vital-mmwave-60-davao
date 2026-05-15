(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function layout(){return R.parseJson('rvt-kpi-layout',{},function(v){return v&&typeof v==='object';});}
  function save(l){R.setJson('rvt-kpi-layout',l);apply();return l;}
  function setSize(id,size){if(['1x','2x'].indexOf(size)<0)throw new Error('invalid size');var l=layout();l[id]=size;return save(l);}
  function getSize(id){return layout()[id]||'1x';}
  function resetLayout(){return save({});}
  function apply(){document.querySelectorAll('[data-kpi-id]').forEach(function(c){var id=c.dataset.kpiId;c.classList.toggle('rvt-kpi-size-2x',getSize(id)==='2x');if(!c.querySelector('.rvt-kpi-resize-handle')){var h=document.createElement('button');h.type='button';h.className='rvt-kpi-resize-handle';h.setAttribute('aria-label','Resize KPI');h.onclick=function(e){e.stopPropagation();setSize(id,getSize(id)==='2x'?'1x':'2x');};c.style.position=c.style.position||'relative';c.appendChild(h);}c.addEventListener('keydown',function(e){if(e.shiftKey&&e.key==='ArrowRight')setSize(id,'2x');if(e.shiftKey&&e.key==='ArrowLeft')setSize(id,'1x');});});}
  window.rvtLive02={setSize:setSize,getSize:getSize,getLayout:layout,resetLayout:resetLayout};R.boot(apply);
})();
