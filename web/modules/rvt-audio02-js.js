(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var ducked=false,threshold=.2,ctx=null,gain=null,base=1;
  function ensureAudio(){if(!ctx&&window.AudioContext){ctx=new AudioContext();gain=ctx.createGain();gain.gain.value=base;gain.connect(ctx.destination);}return gain;}
  function volume(){return Number(localStorage.getItem('rvt-audio-volume')||.7);}
  function duck(){if(volume()<threshold)return false;ducked=true;var g=ensureAudio();if(g){var t=ctx.currentTime;g.gain.cancelScheduledValues(t);g.gain.linearRampToValueAtTime(.2,t+.05);}return true;}
  function restore(){ducked=false;var g=ensureAudio();if(g){var t=ctx.currentTime;g.gain.cancelScheduledValues(t);g.gain.linearRampToValueAtTime(base,t+.5);}return true;}
  if(window.rvtSpeechDebounce&&typeof window.rvtSpeechDebounce.speak==='function'&&!window.rvtSpeechDebounce.speak.__rvtDuck){var old=window.rvtSpeechDebounce.speak;window.rvtSpeechDebounce.speak=function(){duck();var r=old.apply(this,arguments);Promise.resolve(r).finally(restore);return r;};window.rvtSpeechDebounce.speak.__rvtDuck=true;}
  window.rvtVolumeDuck={duck:duck,restore:restore,isDucked:function(){return ducked;},setThreshold:function(r){threshold=Number(r);return threshold;}};
})();
