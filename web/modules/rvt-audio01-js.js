(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var active=localStorage.getItem('rvt-audio-profile-active')||'Clinical';
  var profiles={Clinical:{volume:0.7,voice:true,haptic:false,sound:'voice'},Field:{volume:1,voice:false,haptic:true,sound:'tone'},Quiet:{volume:0,voice:false,haptic:true,sound:'none'}};
  function stored(){return R.parseJson('rvt-settings-profiles',{},function(v){return v&&typeof v==='object';});}
  function getProfiles(){return Object.assign({},profiles,stored().audioProfiles||{});}
  function apply(name){var p=getProfiles()[name];if(!p)return false;active=name;localStorage.setItem('rvt-audio-profile-active',name);localStorage.setItem('rvt-audio-volume',String(p.volume));localStorage.setItem('rvt-voice-alerts',p.voice?'1':'0');localStorage.setItem('rvt-audio-alerts',p.sound==='none'?'0':'1');localStorage.setItem('rvt-haptic-alerts',p.haptic?'1':'0');R.toast(R.t('gd.audioProfile',{name:name}));render();return true;}
  function saveCustom(name,config){var s=stored();s.audioProfiles=s.audioProfiles||{};s.audioProfiles[name]=config;R.setJson('rvt-settings-profiles',s);return getProfiles();}
  function render(){var n=R.ensure('rvt-audio-profile-label','span');n.className='rvt-audio-profile-label';n.textContent=active;}
  window.rvtAudioProfiles={apply:apply,getActive:function(){return active;},getProfiles:getProfiles,saveCustom:saveCustom};R.boot(render);
})();
