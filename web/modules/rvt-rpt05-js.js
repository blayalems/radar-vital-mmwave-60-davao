(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var selected=[];
  function open(ids){selected=(ids||[]).slice(0,5);render();return getRows();}
  function addSession(id){if(selected.indexOf(id)<0&&selected.length<5)selected.push(id);render();return selected.slice();}
  function removeSession(id){selected=selected.filter(function(x){return x!==id;});render();return selected.slice();}
  function sessions(){var all=R.getSessions();return selected.length?selected.map(function(id){return all.find(function(s){return (s.id||s.sessionId)===id;})||{id:id,sessionId:id};}):all.slice(0,5);}
  function median(a){a=a.slice().sort(function(x,y){return x-y;});return a.length?a[Math.floor(a.length/2)]:0;}
  function getRows(){var ss=sessions(),metrics=['score','duration','hr','rr','pqi'];return metrics.map(function(m){var vals=ss.map(function(s){return R.metricsForSession(s)[m];}),med=median(vals);return {metric:m,values:vals,delta:vals.map(function(v){return v-med;})};});}
  function exportCsv(){var ss=sessions(),rows=[['metric'].concat(ss.map(function(s){return s.sessionId||s.id;})).concat(['delta'])];getRows().forEach(function(r){rows.push([r.metric].concat(r.values).concat([r.delta.join(';')]));});return R.download('session_compare_'+R.today()+'.csv','text/csv',R.toCsv(rows));}
  function render(){var root=R.ensure('rvt-rpt05-compare','section');root.className='rvt-gd-panel';var ss=sessions(),rows=getRows();root.innerHTML='<h3>'+R.t('gd.compare')+'</h3><table class="rvt-compare-table"><tbody></tbody></table>';var tb=root.querySelector('tbody');rows.forEach(function(r){var tr=document.createElement('tr');tr.innerHTML='<th>'+r.metric+'</th>'+r.values.map(function(v){return '<td class="rvt-compare-heat">'+v+'</td>';}).join('')+'<td>'+r.delta.join(', ')+'</td>';tb.appendChild(tr);});}
  window.rvtMultiCompare={open:open,addSession:addSession,removeSession:removeSession,exportCsv:exportCsv,getRows:getRows};
  window.rvtRpt05=window.rvtMultiCompare;R.boot(function(){var b=R.ensure('rvt-rpt05-button','button');b.className='rvt-gd-button';b.textContent=R.t('gd.compare');b.onclick=function(){open((R.getSessions().slice(0,3).map(function(s){return s.sessionId||s.id;})));};});
})();
