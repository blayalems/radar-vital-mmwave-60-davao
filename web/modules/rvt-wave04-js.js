(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var current=null,hideTimer=null;
  function set(start,end){var d=Math.abs(Number(end)-Number(start));current={deltaMs:d,bpm:d?60000/d:null};return current;}
  function simulate(startMs,endMs){var m=set(startMs,endMs);show(0,0);return m;}
  function clear(){current=null;if(hideTimer)clearTimeout(hideTimer);var n=document.getElementById('rvt-wave04-overlay');if(n)n.remove();}
  function show(x,y){if(!current)return;var n=R.ensure('rvt-wave04-overlay','div');n.className='rvt-wave04-overlay';n.textContent=R.t('gd.measure',{delta:Math.round(current.deltaMs),bpm:current.bpm?current.bpm.toFixed(1):'N/A'});n.style.left=(x||16)+'px';n.style.top=(y||16)+'px';if(hideTimer)clearTimeout(hideTimer);hideTimer=setTimeout(clear,5000);}
  document.addEventListener('pointerdown',function(e){if(!e.shiftKey||!(e.target&&e.target.closest&&e.target.closest('canvas,[data-waveform]')))return;var start=e.clientX;function move(ev){set(start,ev.clientX);show(ev.clientX,ev.clientY);}function up(ev){move(ev);document.removeEventListener('pointermove',move);document.removeEventListener('pointerup',up);}document.addEventListener('pointermove',move);document.addEventListener('pointerup',up);},true);
  window.rvtWave04={getMeasurement:function(){return current;},clearMeasurement:clear,simulate:simulate};
  window.rvtWave04Measure=window.rvtWave04;
})();
