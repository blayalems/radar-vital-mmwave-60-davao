(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var startMs=null,timer=null,lastAria=0;
  function fmt(s){return Math.floor(s/60)+':'+String(s%60).padStart(2,'0');}
  function tick(){var s=getSeconds(),b=document.querySelector('#pauseBtn,[data-pause-button],.pause-button');if(b){b.textContent=R.t('gd.paused',{time:fmt(s)});if(Date.now()-lastAria>5000){b.setAttribute('aria-label',R.t('gd.paused',{time:fmt(s)}));lastAria=Date.now();}}timer=setTimeout(tick,1000);}
  function start(){if(timer)clearTimeout(timer);startMs=Date.now();tick();}
  function stop(){if(timer)clearTimeout(timer);timer=null;startMs=null;var b=document.querySelector('#pauseBtn,[data-pause-button],.pause-button');if(b){b.textContent='';b.removeAttribute('aria-label');}}
  function getSeconds(){return startMs?Math.floor((Date.now()-startMs)/1000):0;}
  window.addEventListener('beforeunload',function(){if(timer)clearTimeout(timer);});
  window.rvtPauseDuration={start:start,stop:stop,getSeconds:getSeconds};
})();
