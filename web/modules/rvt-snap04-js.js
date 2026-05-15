(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;var selectMode=false,selected=new Set(),cancelled=false;
  function idFromNode(n){return n&&(n.dataset.snapshotId||n.dataset.snapId||n.id);}
  function syncFromDom(){document.querySelectorAll('.rvt-snap-select:checked,[data-snap-select]:checked,.rvt-snap-select[checked],[data-snap-select][checked]').forEach(function(c){selected.add(c.value||idFromNode(c.closest('[data-snapshot-id],[data-snap-id]')));});}
  function getSelected(){syncFromDom();return Array.from(selected).filter(Boolean);}
  function selectAll(){selectMode=true;(window.S&&window.S.snaps||[]).forEach(function(s){selected.add(s.id);});document.querySelectorAll('[data-snapshot-id],[data-snap-id]').forEach(function(n){var id=idFromNode(n);if(id)selected.add(id);});render();return getSelected();}
  function clearSelection(){selected.clear();document.querySelectorAll('.rvt-snap-select,[data-snap-select]').forEach(function(c){c.checked=false;});render();return [];}
  function snapshots(){var all=(window.S&&window.S.snaps)||R.parseJson('rvt-snaps',[],Array.isArray);var ids=getSelected();return all.filter(function(s){return ids.indexOf(s.id)>=0;});}
  function exportSelected(){var snaps=snapshots(),ids=getSelected();if(!ids.length)return null;if(window.JSZip){var zip=new window.JSZip();snaps.forEach(function(s){zip.file((s.label||s.id)+'_'+(s.timestamp||Date.now())+'.csv',R.toCsv([['id','label','timestamp'],[s.id,s.label||'',s.timestamp||'']]));zip.file((s.label||s.id)+'_'+(s.timestamp||Date.now())+'.png',s.png||'');});return zip.generateAsync({type:'blob'}).then(function(blob){return R.download('snapshots_export_'+R.today()+'.zip','application/zip',blob);});}R.toast(R.t('gd.zipMissing'));ids.forEach(function(id){R.download(id+'.csv','text/csv','id\n'+id);});return ids;}
  function render(){document.body.classList.toggle('rvt-snap-select-mode',selectMode);var btn=R.ensure('rvt-snap04-export','button');btn.type='button';btn.className='rvt-gd-button';btn.textContent=R.t('gd.exportSelected',{count:getSelected().length});btn.hidden=!getSelected().length;btn.onclick=exportSelected;}
  window.rvtSnapBulk={getSelected:getSelected,selectAll:selectAll,clearSelection:clearSelection,exportSelected:exportSelected,isSelectMode:function(){return selectMode;}};
  window.rvtSnap04=window.rvtBulkExport=window.rvtSnapBulk;R.boot(render);
})();
