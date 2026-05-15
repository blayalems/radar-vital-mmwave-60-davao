(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function all(){return R.parseJson('rvt-help-feedback',{},function(v){return v&&typeof v==='object';});}
  function vote(id,dir){if(['up','down'].indexOf(dir)<0)throw new Error('invalid vote');var m=all();m[id]=dir;R.setJson('rvt-help-feedback',m);var url=localStorage.getItem('rvt-feedback-endpoint');if(url)fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({entryId:id,vote:dir,appVersion:window.RVT_CLIENT_VERSION||'',timestamp:R.nowIso()})}).catch(function(){});render();return dir;}
  function getVote(id){return all()[id]||null;}
  function render(){document.querySelectorAll('[data-help-entry-id],.faq-entry').forEach(function(e,i){var id=e.dataset.helpEntryId||('entry-'+i),v=getVote(id),w=e.querySelector('.rvt-help-feedback')||document.createElement('div');w.className='rvt-help-feedback rvt-gd-toolbar';w.innerHTML=v?'<span>'+R.t('gd.helpThanks')+'</span>':'<button class="rvt-gd-button" data-v="up">Up</button><button class="rvt-gd-button" data-v="down">Down</button>';if(!w.parentNode)e.appendChild(w);w.querySelectorAll('button').forEach(function(b){b.onclick=function(){vote(id,b.dataset.v);};});});}
  window.rvtHelpFeedback={vote:vote,getVote:getVote,getAll:all};R.boot(render);
})();
