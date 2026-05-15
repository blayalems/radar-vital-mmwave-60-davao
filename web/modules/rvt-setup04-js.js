(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var active=false,snapshot=null;
  function start(){snapshot=JSON.stringify(window.S||{});active=true;document.body.classList.add('rvt-dry-run-active');var w=R.ensure('rvt-preview-watermark','div');w.className='rvt-preview-watermark';w.textContent=R.t('gd.preview');if(window.rvtDemoMode)window.rvtDemoMode.enable('Normal');return true;}
  function stop(){active=false;document.body.classList.remove('rvt-dry-run-active');var w=document.getElementById('rvt-preview-watermark');if(w)w.remove();if(window.rvtDemoMode)window.rvtDemoMode.disable();try{window.S=JSON.parse(snapshot||'{}');}catch(_){};}
  window.rvtDryRun={start:start,stop:stop,isActive:function(){return active;}};R.boot(function(){var b=R.ensure('rvt-setup04-preview','button');b.className='rvt-gd-button';b.textContent=R.t('gd.dryRun');b.onclick=start;});
})();
