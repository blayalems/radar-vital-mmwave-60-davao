(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function enable(){document.documentElement.setAttribute('dir','rtl');document.body.classList.add('rvt-rtl');return true;}
  function disable(){document.documentElement.setAttribute('dir','ltr');document.body.classList.remove('rvt-rtl');}
  window.rvtRtlSupport={enable:enable,disable:disable,isRtl:function(){return document.documentElement.getAttribute('dir')==='rtl';}};
})();
