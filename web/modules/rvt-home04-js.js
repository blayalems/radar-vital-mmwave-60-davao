(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function getRecent(){return R.parseJson('rvt-recent-setups',[],Array.isArray).slice(0,3);}
  function save(a){return R.setJson('rvt-recent-setups',a.slice(0,3));}
  function add(port,ble,profile){var key=[port,ble,profile].join('|'),a=getRecent().filter(function(e){return [e.port,e.ble,e.profile].join('|')!==key;});a.unshift({port:port,ble:ble,profile:profile,usedAt:R.nowIso()});render();return save(a);}
  function remove(i){var a=getRecent();a.splice(Number(i),1);render();return save(a);}
  function setField(sel,val){var f=document.querySelector(sel);if(f){f.value=val||'';f.dispatchEvent(new Event('input',{bubbles:true}));f.dispatchEvent(new Event('change',{bubbles:true}));}}
  function apply(i){var e=getRecent()[Number(i)];if(!e)return null;setField('#radarPort,input[name="port"],#port',e.port);setField('#bleAddress,input[name="ble"],#ble',e.ble);setField('#profileName,input[name="profile"],#profile',e.profile);return e;}
  function render(){var host=document.querySelector('#view-home,#setup-card,[data-setup-card]')||document.body,panel=R.ensure('rvt-home04-recent','div',host);panel.className='rvt-gd-toolbar';panel.innerHTML='';getRecent().forEach(function(e,i){var chip=document.createElement('button');chip.type='button';chip.className='rvt-gd-chip';chip.textContent=(e.profile||'Profile')+' · '+(e.port||'');chip.onclick=function(){apply(i);};var rm=document.createElement('button');rm.type='button';rm.className='rvt-gd-button';rm.setAttribute('aria-label','Remove '+(e.profile||'profile')+' from recent setups');rm.textContent='×';rm.onclick=function(ev){ev.stopPropagation();remove(i);};panel.append(chip,rm);});}
  window.rvtHome04={getRecent:getRecent,add:add,remove:remove,apply:apply};R.boot(render);
})();
