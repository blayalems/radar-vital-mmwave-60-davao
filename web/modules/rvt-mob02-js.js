(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var active=false;
  function isLandscape(){return window.innerWidth>window.innerHeight;}
  function render(){document.body.classList.toggle('rvt-field-layout-active',active&&isLandscape());var n=document.getElementById('rvt-field-layout-surface');if(active){n=n||R.ensure('rvt-field-layout-surface','div');n.className='rvt-field-layout-surface';n.innerHTML=isLandscape()?'<div><span>HR</span><b>'+(window.S&&window.S.live&&window.S.live.hr||'--')+'</b></div><div><span>RR</span><b>'+(window.S&&window.S.live&&window.S.live.rr||'--')+'</b></div>':'<p>'+R.t('gd.rotate')+'</p>';}else if(n)n.remove();}
  function enable(){active=true;render();return true;}function disable(){active=false;render();}
  document.addEventListener('keydown',function(e){if(e.shiftKey&&e.key.toUpperCase()==='L'){e.preventDefault();active?disable():enable();}},true);window.addEventListener('resize',render);
  window.rvtFieldLayout={enable:enable,disable:disable,isActive:function(){return active;},isLandscape:isLandscape};
})();
