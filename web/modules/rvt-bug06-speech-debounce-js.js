(function(){
  'use strict';
  var synth=window.speechSynthesis,NativeUtterance=window.SpeechSynthesisUtterance;
  var lastSpoken={text:'',timestamp:0},lastEndAt=0,queued=null,queueTimer=0;
  if(!synth||!NativeUtterance||synth.__rvtDebounced){window.rvtSpeechDebounce=window.rvtSpeechDebounce||{speak:function(){},cancel:function(){},getLastSpoken:function(){return lastSpoken;},getQueueLength:function(){return 0;}};return;}
  var downstreamSpeak=synth.speak.bind(synth),nativeCancel=synth.cancel.bind(synth);
  function shouldSuppress(text,priority,now){var ms=priority==='critical'?1000:3000;return lastSpoken.text===text&&now-lastSpoken.timestamp<ms;}
  function clearQueue(){queued=null;if(queueTimer){clearTimeout(queueTimer);queueTimer=0;}}
  function emit(text,priority){clearQueue();nativeCancel();var u=new NativeUtterance(text);u.__rvtDebounceBypass=true;u.__rvtPriority=priority||'normal';lastSpoken={text:text,timestamp:Date.now()};u.onend=function(){lastEndAt=Date.now();};u.onerror=function(){lastEndAt=Date.now();};downstreamSpeak(u);}
  function debouncedSpeak(text,priority){priority=priority==='critical'?'critical':'normal';text=String(text==null?'':text).trim();if(!text)return;var now=Date.now();if(shouldSuppress(text,priority,now))return;if(priority==='critical'){emit(text,priority);return;}var wait=Math.max(0,1000-(now-lastEndAt));if(wait>0){clearQueue();queued={text:text,priority:priority};queueTimer=setTimeout(function(){var q=queued;if(q)emit(q.text,q.priority);},wait);return;}emit(text,priority);}
  window.rvtSpeechDebounce={speak:debouncedSpeak,cancel:function(){clearQueue();nativeCancel();},getLastSpoken:function(){return{text:lastSpoken.text,timestamp:lastSpoken.timestamp};},getQueueLength:function(){return queued?1:0;}};
  synth.speak=function(utterance){if(utterance&&utterance.__rvtDebounceBypass)return downstreamSpeak(utterance);var text=utterance&&typeof utterance.text==='string'?utterance.text:String(utterance||'');var priority=utterance&&utterance.__rvtPriority?utterance.__rvtPriority:'normal';return debouncedSpeak(text,priority);};
  synth.__rvtDebounced=true;
})();
