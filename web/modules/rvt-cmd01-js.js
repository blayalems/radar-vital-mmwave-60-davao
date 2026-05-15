(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function all(){return R.parseJson('rvt-cmd-recent',[],Array.isArray);}
  function save(a){return R.setJson('rvt-cmd-recent',a.slice(0,20));}
  function getRecent(){var cutoff=Date.now()-7*86400000;return all().filter(function(e){return new Date(e.usedAt).getTime()>=cutoff;}).sort(function(a,b){return (b.count-a.count)||String(b.usedAt).localeCompare(String(a.usedAt));}).slice(0,5);}
  function record(actionId,label){var a=all(),e=a.find(function(x){return x.actionId===actionId;});if(e){e.count=(e.count||0)+1;e.usedAt=R.nowIso();e.label=label||e.label;}else a.unshift({actionId:actionId,label:label||actionId,usedAt:R.nowIso(),count:1});return save(a.sort(function(x,y){return String(y.usedAt).localeCompare(String(x.usedAt));}));}
  window.rvtCmdRecent={getRecent:getRecent,record:record,clear:function(){return save([]);}};window.rvtCmd01=window.rvtCmdRecent;
})();
