(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var hist=[];
  function parse(s){return String(s||'').split(/\s*(?:&&|;)\s*/).map(function(x){return x.trim();}).filter(Boolean);}
  function exec(label){if(window.rvtCmdSynonym){var m=window.rvtCmdSynonym.search(label)[0];if(m)label=m.action;}if(/fail/i.test(label))throw new Error(label);R.toast(label);return label;}
  async function run(chain){var parts=parse(chain);hist.unshift({chain:chain,at:R.nowIso()});hist=hist.slice(0,10);for(var i=0;i<parts.length;i++){try{var out=exec(parts[i]);R.toast(R.t('gd.step',{step:i+1,total:parts.length,label:out}));}catch(err){R.toast(R.t('gd.chainAbort',{step:i+1,reason:err.message}));throw err;}}return true;}
  window.rvtActionChain={run:run,parse:parse,getHistory:function(){return hist.slice();}};
})();
