(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function get(){return R.getScalar('rvt-toast-position','BR');}
  function apply(){['TL','TR','BL','BR'].forEach(function(p){document.body.classList.remove('rvt-toast-pos-'+p);});document.body.classList.add('rvt-toast-pos-'+get());return get();}
  function set(pos){if(['TL','TR','BL','BR'].indexOf(pos)<0)throw new Error('invalid position');R.setScalar('rvt-toast-position',pos);return apply();}
  window.rvtToastPos={set:set,get:get,apply:apply};R.boot(apply);
})();
