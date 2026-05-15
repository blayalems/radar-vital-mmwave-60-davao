(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  function isOn(){return localStorage.getItem('rvt-audio-alerts')==='1';}
  function toggle(){
    var next=isOn()?'0':'1';
    localStorage.setItem('rvt-audio-alerts',next);
    if(window.rvtStorage&&typeof window.rvtStorage.set==='function')window.rvtStorage.set('rvt-audio-alerts',next);
    updateUi();
    U.toast(next==='1'?t('gamma.alert.sound.on'):t('gamma.alert.sound.off'));
  }
  function updateUi(){
    var btn=document.getElementById('rvtAlertSoundToggle');if(!btn)return;
    var on=isOn();
    btn.setAttribute('aria-pressed',on?'true':'false');
    var icon=btn.querySelector('.material-symbols-rounded');
    if(icon)icon.textContent=on?'volume_up':'volume_off';
    var label=btn.querySelector('.rvt-alert-sound-text');
    if(label)label.textContent=t('gamma.alert.sound.label');
  }
  function renderButton(){
    var header=document.querySelector('.dv-h')||document.querySelector('#dv .dv-h');
    if(!header||document.getElementById('rvtAlertSoundToggle'))return;
    var btn=document.createElement('button');btn.type='button';btn.id='rvtAlertSoundToggle';btn.className='rvt-alert-sound-toggle';
    btn.innerHTML='<span class="material-symbols-rounded" aria-hidden="true"></span><span class="rvt-alert-sound-text"></span>';
    btn.addEventListener('click',function(){window.rvtAlertSound.toggle();});
    var close=header.querySelector('button');header.insertBefore(btn,close);
    updateUi();
  }
  window.rvtAlertSound={toggle:toggle,isOn:isOn};
  U.bootSoon(renderButton);
})();
